# AU Competition — 麦轮小车控制系统

## 项目概述

基于 STM32F407VET6 的四轮麦克纳姆轮竞赛小车控制系统。通过摄像头识别路径、蓝牙接收指令，实现自动行走、转弯、平移、转圈等运动，完成竞赛任务。

## 硬件清单

| 器件 | 型号 |
|------|------|
| 主控 | STM32F407VET6 @ 168MHz |
| 麦轮 + 底盘 | 48mm 麦轮，联轴器，四轮底盘 |
| 电机 | mg310 减速编码器电机（13 PPR，减速比 1:20） |
| 电机驱动 | L298N × 4 |
| OLED | 0.96 寸 I2C（SSD1315） |
| 蓝牙 | HC-05（UART 透传） |
| 摄像头 | 串口通信（UART4），自定义帧协议 |

## 引脚分配

### 电机控制

| 电机 | PWM 通道 | 方向引脚 | 编码器定时器 |
|------|----------|----------|-------------|
| FL（左前） | TIM4 CH3 (PB8) | PB0 / PB1 | TIM5 |
| FR（右前） | TIM4 CH4 (PB9) | PC0 / PC1 | TIM3 |
| RL（左后） | TIM4 CH1 (PB6) | PD14 / PD15 | TIM1 |
| RR（右后） | TIM4 CH2 (PB7) | PD10 / PD11 | TIM8 |

### 通信接口

| 外设 | UART | 说明 |
|------|------|------|
| 蓝牙 HC-05 | USART2 | 透传模式，接收指令 |
| 摄像头 | UART4 | 自定义帧协议，中断接收 |
| 上位机调参 | USART1 | 实时 PID 参数下发 |

## 系统架构

```
main.c（应用层，权限状态机驱动）
  ├── Move_permission    → 路径执行（前进/转弯/平移/到达）
  ├── Show_permission    → OLED 显示摄像头识别结果
  ├── Revolve_permission → 原地转圈 × N
  └── Back_permission    → 路径回退

Motor.c（运动控制层）
  ├── Car_DriveDistance()    → 直行位置环（四轮平均反馈）
  ├── Car_RotateAngle()      → 旋转位置环（逐轮独立反馈）
  ├── Car_StrafeDistance()   → 平移位置环（同向对配对平均反馈）
  └── Motor_ControlWheel()   → 速度环（增量式 PID）
       ├── Motor_UpdateEncoder()    → 编码器读取 / 转速计算
       ├── Motor_PIDIncrementalUpdate() → 增量 PID 更新
       └── Motor_StartWheel()       → PWM + 方向 GPIO

外设模块
  ├── Camera.c    → 摄像头协议解析（<M>/<G>/<R> 指令）
  ├── Bluetooth.c → 蓝牙 UART 回调（M/B/S 指令）
  ├── Host.c      → 上位机通信（PID 参数实时调优）
  └── OLED/       → OLED 显示驱动
```

## 控制策略详解

### 三种运动原语的控制策略

| 运动 | 位置环反馈 | 输出方式 | 终止条件 |
|------|-----------|---------|---------|
| **直行** (`Car_DriveDistance`) | 四轮平均 `d_avg = (d0+d1+d2+d3)/4` | 单一 `pos_out` → 四轮同速 | `d_avg >= target_counts` |
| **旋转** (`Car_RotateAngle`) | 逐轮独立编码器 delta | 每轮独立 `pos_out` | 四轮全部 `>= target` (AND) |
| **平移** (`Car_StrafeDistance`) | 同向对配对平均：对 A (FL+RR)、对 B (FR+RL) | 每对共享 `pos_out` | 两对各自平均 `>= target` (AND) |

### 控制层级（循环时序）

```
每个 1ms 周期：
  位置环（Car_* 函数内部）
    → 编码器 delta → LPF 滤波 → PD 计算 → 目标速度
                    ↓
  速度环（Motor_ControlWheel）
    → 编码器转速 → 增量式 PID → PWM + 方向
                    ↓
  轮子（硬件反馈）
    → 编码器脉冲 → total_counts / speed_m_s
```

### 旋转运动学

Mecanum 旋转时 FL/RL 同向、FR/RR 反向。位移公式：
```
轮位移 = 角度(rad) × 轨距/2
编码器计数 = 轮位移 / (π × 轮径) × CPR
```
例：90° × (0.20m / 2) = 0.157m → 约 1085 counts（轨距 0.20m 时）。

### 平移运动学

平移时对角线轮子反向转动，轮子行程 = 车体位移 × √2。同向对：FL ↔ RR、FR ↔ RL。

## PID 参数（当前代码值）

### 位置环参数

| 参数 | 值 | 说明 |
|------|-----|------|
| `POS_KP` | 5.5 | 位置环比例增益 |
| `POS_KD` | 5.5 | 位置环微分增益 |
| `POS_INTEGRAL_RESET_THRESHOLD` | 400 counts | 接近目标时复位积分阈值 |
| LPF α | 0.3 | 位置误差低通滤波系数 |

### 速度环参数

| 参数 | 值 | 说明 |
|------|-----|------|
| 默认 Kp | 5.0 | 速度环比例增益 |
| 默认 Ki | 3.0 | 速度环积分增益 |
| 默认 Kd | 10.0 | 速度环微分增益 |
| 积分上限 | ±200 | 抗积分饱和 |
| `PID_OUTPUT_LIMIT` | ±100 | PID 输出限幅 |
| LPF α | 0.3 | 速度误差低通滤波系数 |
| `SPEED_INTEGRAL_WINDOW` | 0.15 m/s | 条件积分窗口 |
| PWM 范围 | [50, 60] | 含 50 启动死区前馈 |

### 初始化 PID（`main.c` 中实际设置）

四轮初始值均为 `Kp=4.71, Ki=2.94, Kd=2.67`，上限 100。注意与头文件默认值不同——代码中通过 `Motor_PIDInit` 覆盖了默认值。

## 关键参数速查

| 参数 | 值 | 说明 |
|------|-----|------|
| `ENCODER_CPR` | 1040 | = 13 PPR × 20 减速比 × 4 倍频 |
| `WHEEL_DIAMETER_M` | 0.048 | 轮径 48mm |
| `TRACK_WIDTH_M` | 0.20 | 轨距 200mm |
| `PID_CONTROL_PERIOD_MS` | 1 | PID 控制周期 1ms |
| `MIN_PWM_THRESHOLD` | 50 | 最小 PWM 占空比 |
| `MAX_PWM_THRESHOLD` | 60 | 最大 PWM 占空比 |
| `ONE_GRID_M` | 0.30 | 每格距离 |
| `QUARTER_DEGREE` | 125 | 四分之一圈角度 |
| `ONE_CIRCLE` | 500 | 完整一圈的编码器计数 |

## 模块说明

### 蓝牙通信（`Core/peripherals/Bluetooth/`）

HC-05 透传模式，USART2 中断接收。指令：
- `M` — 触发摄像头拍照 + 路径执行（前进移动）
- `B` — 触发路径回退
- `S` — 发送停止指令给摄像头

### 摄像头通信（`Core/peripherals/Camera/`）

UART4 中断接收，状态机解析自定义帧协议：
- `<M>` 帧 — 路径数据（方向 + 步数），最多 100 步
- `<G>` 帧 — 数值数据（转圈次数）
- `<R>` 帧 — 字符串索引（用于 OLED 显示类别）

解析完成后通过 `Show_permission` / `Move_permission` / `Revolve_permission` 标志驱动主循环状态机。

### 上位机通信（`Core/peripherals/Host/`）

USART1 接收 PID 参数实时调优指令（`Kp`/`Ki`/`Kd`），支持运行时动态调整。

### OLED 显示（`Core/peripherals/OLED/`）

I2C 接口，0.96 寸 128×64。显示摄像头识别结果（动物/人类/水果）。

### 电机控制（`Core/peripherals/Motor/`）——核心模块

封装层级：
1. **硬件抽象层** — `Motor_SetPWM_Internal` / `Motor_SetDirection_Internal`
2. **编码器更新** — `Motor_UpdateEncoder`：读定时器计数，计算 delta、rpm、线速度
3. **PID 核心** — `Motor_PIDIncrementalUpdate`：增量式 PID，含 LPF + 条件积分 + 输出限幅
4. **速度控制** — `Motor_ControlWheel`：固定周期调用编码器 + PID，输出 PWM
5. **位置控制** — `Car_DriveDistance` / `Car_RotateAngle` / `Car_StrafeDistance`：位置环 PD + 终止判断

## 理论知识笔记

### 1. PID 控制

- **P（比例）**：输出与当前误差成正比。纯 P 控制存在静差。
- **I（积分）**：消除静差，但容易饱和（windup），需限制上限或在接近目标时清零。
- **D（微分）**：提供阻尼/制动，抑制震荡，但对噪声敏感需配合 LPF。

### 2. 增量式 PID

计算输出的变化量而非绝对值，天然防突变、方便限幅、适合嵌入式。

### 3. 一阶低通滤波（LPF）

`filtered = α × raw + (1−α) × prev_filtered`。α 越小滤波越强但相位滞后越大，PID 中过强 LPF 会降低 D 项制动效率。

### 4. 串级控制

位置环（外环）+ 速度环（内环）的串级结构。外环输出作为内环目标，分离关注点。注意**双积分陷阱**：内外环同时有积分器会形成慢速极限环振荡。

### 5. 麦轮运动学

- **直行**：四轮同向
- **平移**：对角线反向，轮子行程 = 车体位移 × √2
- **旋转**：左侧同向、右侧反向，轮位移 = 角度 × 轨距/2

### 6. PWM 死区

电机有启动死区——低于某一占空比无法克服静摩擦。本项目 PWM 范围 [50, 60]，低阈值保证启动，高阈值限制最大速度。

## Issue 修复记录

| Issue | 问题 | 根因 | 修复 |
|-------|------|------|------|
| #001 | 负载转弯剧烈振荡 | 积分饱和（上限 ±500） | 上限降至 ±200 + 接近目标时复位积分 |
| #002 | 空载正常/负载 80° 振荡 | 积分复位阈值 20 太小（仅 2% 行程） | 阈值扩至 400 + 条件积分窗口 0.15 m/s |
| #003 | ±2.5° 低速极限环振荡 | 位置 I + 速度 I 双积分耦合 | 位置模式下速度环 Ki 临时清零 |
| #004 | 到 90° 附近停不下来 | D 增益太弱 + 到达条件用四轮分离判断 | POS_KD 0.1→5.5 + d_avg 统一判断 + LPF 调整 |
| #005 | 多段直行累积右偏 | 到达条件 OR 逻辑（任意轮到就停） | 改为四轮平均 `d_avg >= target` |
| #006 | 旋转向外侧漂移 | 四轮平均反馈掩盖慢轮差异 + 积分复位干扰 | 逐轮独立位置环 + 四轮 AND 终止 + 删除积分复位 |
| #007 | 平移旋转偏移 + 末尾振荡 | 逐轮独立与 sync PI 双闭环竞争 | 同向对配对平均位置环（删除 sync 耦合） |

## 构建与烧录

### 前置条件

- CMake ≥ 3.16
- ARM GCC 工具链 (`arm-none-eabi-gcc`)
- STM32CubeMX 生成的 HAL 驱动（已包含在 `Drivers/`）

### 构建

```bash
cmake --preset default
cmake --build build
```

### 烧录

使用 ST-Link / J-Link 或串口 ISP 烧录 `build/*.elf` 或 `build/*.bin` 到 STM32F407VET6。

## 项目结构

```
AU/
├── Core/
│   ├── Inc/               # HAL 配置头文件
│   ├── Src/               # HAL 配置源文件 + main.c
│   └── peripherals/       # 外设驱动模块
│       ├── Bluetooth/     # 蓝牙通信
│       ├── Camera/        # 摄像头协议
│       ├── Host/          # 上位机调参
│       ├── Motor/         # 电机控制（核心）
│       └── OLED/          # OLED 显示
├── Drivers/               # STM32 HAL 库
├── docs/issues/           # Issue 详细记录
├── CMakeLists.txt         # CMake 构建
├── CMakePresets.json      # CMake 预设
└── AU.ioc                 # STM32CubeMX 项目配置
```

## PID 调参经验

1. 使用串口打印数据，配合上位机实时查看曲线（`Host` 模块支持在线调参）
2. 先调 Kp：让实际速度在目标速度附近小幅度震荡
3. 增加 Kd：抑制震荡，一直加大直到不再抖动
4. Kp、Kd 乘以 0.7~0.8，增加 Ki：消除静差
5. 四个轮子单独调试时上升曲线要尽量一致，否则启动时偏航
6. 停止轮子时必须清零 PID 状态，否则下次启动不准
7. 位置环调参：先保证速度环稳定，再逐级向上调
8. ARR 值不宜过小，否则控制精度太低