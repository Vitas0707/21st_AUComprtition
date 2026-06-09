# AU Competition — 麦轮小车控制系统

## 硬件清单

| 器件 | 型号 |
|------|------|
| 麦轮 + 底盘 | 48mm 麦轮，联轴器，四轮底盘 |
| 电机 | mg310 减速编码器电机（13 PPR，减速比 1:20） |
| 主控 | STM32F407VET6 @ 168MHz |
| 电机驱动 | L298N |
| OLED | 0.96寸 I2C |
| 蓝牙 | HC-05（透传） |
| 摄像头 | 串口通信，帧头+数据+帧尾协议 |

## 系统架构

```
main.c（应用层，状态机驱动）
  └─ Car_RotateAngle / Car_DriveDistance / Car_StrafeDistance（位置环）
       └─ Motor_ControlWheel（速度环）
            └─ Motor_PIDIncrementalUpdate（增量式 PID）
                 └─ Motor_StartWheel（PWM + 方向 GPIO）
                      └─ Motor_UpdateEncoder（编码器反馈）
```

## 理论知识笔记

### 1. PID 控制

PID 控制器由三个环节组成，输出 = P + I + D：

- **P（比例）**：输出与当前误差成正比。`output = Kp × error`
  - 增大 Kp 能加快响应，但过大会振荡
  - 纯 P 控制有**静差**（稳态误差），因为有摩擦力等阻力时，需要一定的输出来抵消阻力，而输出 = Kp × error 意味着必须有误差才有输出
- **I（积分）**：输出与误差累积量成正比。`integral = Σ(error × dt)`，`output += Ki × integral`
  - **消除静差**：只要误差不为零，积分就一直累积直到输出足够大、推动系统消除误差
  - **积分饱和（windup）**：误差大且持续时间长时，积分累积过大，即使误差已归零，积分"惯性"仍推动系统冲过目标，引发超调和振荡
  - **抗饱和方法**：限制积分上限（clamping）、误差大时清零积分（conditional integration）、接近目标时复位积分
- **D（微分）**：输出与误差变化率成正比。`output = Kd × (error - prev_error) / dt`
  - **提供阻尼/制动**：误差在缩小时（接近目标），导数为负，输出减小，起到"刹车"效果
  - 对噪声敏感，通常配合低通滤波使用

### 2. 增量式 PID

位置式 PID 直接计算最终输出，增量式 PID 计算输出的**变化量**：

```
delta_output = Kp × error + Ki × integral + Kd × derivative
output = prev_output + delta_output
```

优点：
- 天然防突变：输出是叠加的，不会因参数突变产生阶跃
- 方便限幅：限幅 `output` 而非 `delta_output`，不丢积分信息
- 适合嵌入式：上次输出存于结构体，无状态丢失

注意事项：
- `delta_output = Kp × error + Ki × integral + Kd × derivative` 公式中的 `error` 是当前误差（含 LPF 后），非误差增量
- 即使 Ki = 0，`output += delta_output` 仍有累积效应（P 项误差每次不同）
- 停机时必须清零 `prev_output` 和 `integral`

### 3. 一阶低通滤波（Low-Pass Filter, LPF）

```
filtered = α × raw + (1 − α) × prev_filtered
```

- **α** ∈ [0, 1] 是滤波系数
  - α = 1：不过滤（输出 = 输入），响应最快但噪声最多
  - α = 0.1：强过滤（90% 旧值），曲线平滑但相位滞后大
  - α = 0.5：折中
- **相位滞后**是 LPF 的主要代价。滞后 = 旧值占比越大，响应越慢
  - 在 PID 中，过强的 LPF 会降低 D 项的制动效率，导致欠阻尼
- 本项目用 LPF 过滤位置误差和速度误差，避免编码器量化噪声放大到 PWM

### 4. 串级控制（Cascade Control）

本项目使用**位置环（外环）+ 速度环（内环）**的串级结构：

- **外环（位置环）**：10ms 周期，输入为位置误差（编码器计数），输出为目标速度。当前为 PD+I 结构
- **内环（速度环）**：10ms 周期，输入为速度误差（m/s），输出为 PWM 占空比。当前为增量式 PID，但在位置模式下 Ki 被临时清零

串级控制的优势：
- 外环只关心"还剩多少距离"，不关心轮子怎么转
- 内环只关心"轮子怎么转到指定速度"，不关心全局目标
- 分离关注点，调参互不耦合

**双积分陷阱**：如果内环和外环都有积分器，两个积分器会互相推拉形成慢速极限环振荡（本项目的 #003 号 bug）。解决方案是位置模式下禁用速度积分，仅由位置积分消除静差。

### 5. 麦轮运动学

麦轮（Mecanum Wheel）的 45° 辊子使得：
- **直行**：四个轮子同向转
- **平移**：对角线轮子反向转（左前+右后 vs 右前+左后），轮子行程 = 车体位移 × √2
- **原地旋转**：左侧轮子一方向、右侧轮子反方向。轮位移 = 角度 × 轨距/2

转角 → 编码器计数公式（以本项目参数为例）：
```
90° × 0.165m/2 = 0.1295m 轮位移
0.1295m / (π × 0.048m) = 0.859 圈
0.859 × 1040 (CPR) ≈ 894 counts
```

### 6. PWM 死区

接近目标时，PID 输出很小，但电机有**启动死区**——低于某一占空比时，电机无法克服静摩擦力启动。本项目 `MIN_PWM_THRESHOLD = 50`：
- 好处：保证轮子能动
- 坏处：接近目标时"刹车"能力受限于 50，可能导致低振幅振荡（#004 已通过增强 D 增益 50 倍解决）

## 当前 PID 架构（Issue #001~#004 最终态）

### 位置环（Car_RotateAngle / Car_DriveDistance）

```
pos_error = target_counts - d_avg         // 位置误差
pos_error_lpf = α × pos_raw + (1-α) × prev   // α = 0.5
pos_integral += pos_error × dt              // 钳位 ±50
pos_out = Kp×pos_error + Ki×pos_integral + Kd×derivative/10ms
```

| 参数 | 值 | 说明 |
|------|-----|------|
| POS_KP | 1.0 | P 项，响应速度 |
| POS_KI | 0.005 | I 项，消除静差（可调到 0.05 减小剩余偏差） |
| POS_KD | **5.0** | D 项（#004 从 0.1 提升，增强制动） |
| POS_INTEGRAL_LIMIT | 30.0 | 积分上限 |
| LPF α | **0.5** |（#004 从 0.1 提升，减少相位滞后） |
| 积分复位阈值 | **400 counts** | 接近目标时清零积分和速度 PID 历史 |

### 速度环（Motor_ControlWheel → Motor_PIDIncrementalUpdate）

```
raw_error = target_speed - actual_speed    // 速度误差 (m/s)
error = α × raw_error + (1-α) × prev       // α = 0.5
integral += error × dt                      // 钳位 ±200
delta_out = Kp×error + Ki×integral + Kd×derivative
out = prev_out + delta_out                  // 钳位 ±100
PWM = min(|out| + 50, 60)                   // 范围 [50, 60]
```

| 参数 | 值 | 说明 |
|------|-----|------|
| Kp | 4.27 | P 项 |
| Ki | 0（位置模式下） | **#003 临时清零**，防止双积分耦合 |
| Kd | 9.43 | D 项 |
| 积分上限 | ±200 | #001 从 ±500 降低 |
| PWM 范围 | [50, 60] | 含 50 前馈 |

## 模块说明

### 蓝牙通信
透传模式，手机发送指令控制任务切换。`M` = 移动指令，`B` = 回退指令。

### 摄像头通信
串口通信，帧头+数据+帧尾协议，状态机解码。中断接收后通过标志位驱动主循环中小车运动。

### 电机控制（核心）

封装层级：

1. **硬件抽象层**：`Motor_SetPWM_Internal` / `Motor_SetDirection_Internal`
2. **编码器更新**：`Motor_UpdateEncoder` — 读取定时器计数，计算 delta、rpm、速度
3. **PID 核心**：`Motor_PIDIncrementalUpdate` — 增量式 PID 更新
4. **速度控制**：`Motor_ControlWheel` — 固定周期调用 PID + 编码器
5. **位置控制**：`Car_DriveDistance` / `Car_RotateAngle` — 位置环 + 到达判断
6. **平移控制**：`Car_StrafeDistance` — 开环速度控制

## PID 调参经验

1. 使用串口动态打印数据，配合上位机实时查看曲线
2. 先调 Kp：让实际速度在目标速度附近小幅度震荡
3. 增加 Kd：抑制震荡，一直加大直到不再抖动
4. Kp、Kd 乘以 0.7~0.8，增加 Ki：消除静差，让曲线贴合目标
5. **四个轮子单独调试时上升曲线要尽量一致**，否则启动时偏航
6. 停止轮子时必须清零 PID 状态，否则下次启动不准
7. 位置环调参：先保证速度环稳定，再逐级向上调

## Issue 记录

| Issue | 问题 | 根因 | 修复 |
|-------|------|------|------|
| #001 | 负载转弯剧烈振荡（冲过→反冲→再冲） | 积分饱和（上限 ±500） | 上限降至 ±200 + 接近目标时复位积分 |
| #002 | 空载正常/负载 80° 振荡 | 积分复位阈值 20 太小（仅 2% 行程） | 阈值扩至 200 + 条件积分 + 位置积分 |
| #003 | ±2.5° 低速极限环振荡 | 位置 I + 速度 I 双积分耦合 | 位置模式下 Ki 临时清零 |
| #004 | 到 90° 附近停不下来 | D 增益太弱（有效 0.01）+ 到达条件用四轮分离判断 | POS_KD 0.1→5.0 + d_avg 统一判断 + LPF 减半 |

## 引脚分配

| 电机 | PWM 通道 | 方向引脚 | 编码器定时器 |
|------|----------|----------|-------------|
| FL（左前） | TIM4 CH3 | PB0/PB1 | TIM5 |
| FR（右前） | TIM4 CH4 | PC0/PC1 | TIM3 |
| RL（左后） | TIM4 CH1 | PD14/PD15 | TIM1 |
| RR（右后） | TIM4 CH2 | PD10/PD11 | TIM8 |

## 关键参数速查

| 参数 | 值 | 说明 |
|------|-----|------|
| ENCODER_CPR | 1040 | = 13 PPR × 20 减速比 × 4 倍频 |
| WHEEL_DIAMETER_M | 0.048 | 轮径 48mm |
| TRACK_WIDTH_M | 0.165 | 轨距 165mm |
| PID_CONTROL_PERIOD_MS | 10 | PID 周期 10ms |
| MIN_PWM_THRESHOLD | 50 | 最小 PWM |
| MAX_PWM_THRESHOLD | 60 | 最大 PWM |
| 90° 对应 counts | ~894 | |
| 125° 对应 counts | ~1241 | |