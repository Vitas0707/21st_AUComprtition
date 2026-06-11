# AU Competition — 麦轮小车控制系统

## 项目概述

基于 STM32F407VET6 的四轮麦克纳姆轮竞赛小车控制系统。CanMV K230 视觉计算机通过 YOLO11 实时识别路径（箭头+数字）并通过 UART 发送指令，STM32 主控接收后驱动麦克纳姆轮完成自动行走、转弯、平移、转圈等运动，完成竞赛任务。蓝牙用于远程启停控制，上位机支持 PID 参数在线调优。

## 硬件清单

| 器件 | 型号 |
|------|------|
| 主控 | STM32F407VET6 @ 168MHz |
| 视觉计算机 | 01Studio CanMV K230（RISC-V 双核），YOLO11 + 箭头分类 + 灰度巡线 |
| 麦轮 + 底盘 | 48mm 麦轮，联轴器，四轮底盘 |
| 电机 | mg310 减速编码器电机（13 PPR，减速比 1:20） |
| 电机驱动 | L298N × 2 |
| OLED | 0.96 寸 I2C（SSD1315） |
| 蓝牙 | HC-05（UART 透传） |
| PCB | 立创EDA 设计（`Board/` 目录） |

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
| CanMV K230（视觉计算机） | UART4 | 自定义帧协议 `<[type][data...]>`，YOLO 识别 + 巡线结果 |
| 上位机调参 | USART1 | 实时 PID 参数下发 |

## 系统架构

```
main.c（应用层，显式状态机 CarState_t 驱动）
  ├── CAR_IDLE            → 等待蓝牙指令启动任务
  ├── CAR_WAITING_CAMERA  → 等待 CanMV K230 识别结果（含超时保护）
  ├── CAR_MOVING          → 路径执行（前进/转弯/平移/回退/到达）
  ├── CAR_RECOVERY        → 超时恢复（前后微移重试）
  └── CAR_STA             → OLED 显示 / 原地转圈（非移动任务）

Motor.c（运动控制层）
  ├── Car_DriveDistance()    → 直行位置环（四轮平均反馈）
  ├── Car_RotateAngle()      → 旋转位置环（逐轮独立反馈）
  ├── Car_StrafeDistance()   → 平移位置环（同向对配对平均反馈）
  └── Motor_ControlWheel()   → 速度环（增量式 PID）
       ├── Motor_UpdateEncoder()    → 编码器读取 / 转速计算
       ├── Motor_PIDIncrementalUpdate() → 增量 PID 更新
       └── Motor_StartWheel()       → PWM + 方向 GPIO

外设模块
  ├── Camera.c    → UART 协议解析（<M>/<G>/<R> 指令），接收 CanMV K230 数据
  ├── Bluetooth.c → 蓝牙 UART 回调（M/D/B/S 指令）
  ├── Host.c      → 上位机通信（PID 参数实时调优）
  └── OLED/       → OLED 显示驱动

Vision/（CanMV K230 MicroPython，RISC-V）
  ├── main.py         → 主程序：YOLO11 目标检测 + 箭头级联分类 + 串口指令发送
  ├── vision.py       → 灰度巡线：左右 ROI 黑线 blob 检测 → 偏移量 → 导航状态
  ├── camera.py       → 巡线纯算法模块（find_lane_blobs / calc_offset / draw_center_lines）
  ├── arrow.py        → 箭头自学习识别（SelfLearningApp, 余弦相似度对比特征向量）
  ├── ArrowClassifier.py → 箭头 CNN 分类器（kmodel 推理）
  ├── photo.py        → 拍照模块
  ├── QR Code.py      → 二维码检测
  ├── location.py     → 定位模块
  ├── grib.py         → 网格模块
  └── organizize--arrow&number.py → 箭头+数字整理
```

## 主循环状态机

### 状态定义（`CarState_t`）

| 状态 | 说明 |
|------|------|
| `CAR_IDLE` | 上电初始状态，等待蓝牙指令。不做任何动作，防止上电误触发。 |
| `CAR_WAITING_CAMERA` | 已向 CanMV K230 发送 `'1'` 请求，等待 `<M>` 帧回复。含超时保护。 |
| `CAR_MOVING` | 正在执行路径移动（直行/转弯/回退/到达终点）。 |
| `CAR_RECOVERY` | 超时恢复：前后微移帮助 K230 重新捕获目标。 |
| `CAR_STA` | 非移动任务：OLED 显示识别结果或原地转圈。 |

### 状态切换规则

```
   ┌─────────┐ 蓝牙'M'  ┌──────────────────┐  Move_permission  ┌──────────┐
   │  IDLE   │─────────▶│ WAITING_CAMERA   │─────────────────▶│  MOVING  │
   └─────────┘          └──────────────────┘                   └──────────┘
                              │         ▲                          │
                              │ 5s超时   │ 微移完成                  │ 移动完成
                              ▼         │                          │ + 发'1'
                         ┌──────────┐   │                          │
                         │ RECOVERY │───┘                          │
                         └──────────┘                              │
                                                                   │
                         ┌──────────┐                              │
                         │ CAR_STA  │◀── 蓝牙'D'                   │
                         └──────────┘                              │
                              │                                    │
                              │ Show/Revolve完成                   │
                              ▼                                    │
                         ┌─────────┐                               │
                         │  IDLE   │                               │
                         └─────────┘                               │
                                          发'1'切 WAITING ◀─────────┘
```

### 超时保护与恢复机制

- **超时时间**：5 秒（5000ms），使用 `HAL_GetTick()` 计时
- **超时动作**：进入 `CAR_RECOVERY` 状态，执行前进 0.10m → 后退 0.10m 微移
- **恢复后**：重新发送 `'1'` 指令给 CanMV K230 → 切回 `CAR_WAITING_CAMERA` 等待
- **微移期间**：不检查 `Move_permission`；微移完成后若 K230 已回复则直接进入 `CAR_MOVING`，否则重新计时等待
- **无限重试**：K230 持续无回复时小车会反复超时恢复，不卡死

## 蓝牙指令

| 指令 | 触发动作 | 状态切换 |
|------|---------|---------|
| `M` | 启动路径执行任务，向 K230 发送 `'1'`，回复 `Received` | `IDLE` → `WAITING_CAMERA` |
| `D` | 触发 K230 拍照 + OLED 显示/转圈（不移动小车） | → `CAR_STA` |
| `B` | 触发路径回退，回复 `Received` | 置 `Back_permission = true` |
| `S` | 向 K230 发送停止指令 `'0'`，回复 `Received` | 不变 |

## CanMV K230 通信协议

UART4（115200bps）连接 STM32 与 CanMV K230，自定义帧协议 `<[type][data...]>`：

| 帧类型 | 示例 | 含义 | 触发标志 |
|--------|------|------|---------|
| `<M>` | `<MF2>` | 前进 2 格 | `Move_permission = true` |
| `<M>` | `<MR1>` | 右转（前进 1 格后右转） | `Move_permission = true` |
| `<M>` | `<ML1>` | 左转（前进 1 格后左转） | `Move_permission = true` |
| `<M>` | `<MB1>` | 后退 1 格 | `Move_permission = true` |
| `<M>` | `<MO>` | 到达终点 | `Move_permission = true` |
| `<G>` | `<G3>` | 原地转 3 圈 | `Revolve_permission = true` |
| `<R>` | `<R0>` | 显示类别（0=动物, 1=人类, 2=水果） | `Show_permission = true` |

协议支持方向和数字分开发送（例如先发 `<MF` 再发 `<2>` 也能正确拼接）。

STM32 向 K230 发送的控制指令：`'1'` = 启动识别，`'0'` = 停止/待机。

### 方向编码（`Steps[].direction`）

| 值 | 含义 |
|----|------|
| 0 | 无效（触发微移重试） |
| 1 | 直行前进 |
| 2 | 右转 |
| 3 | 左转 |
| 4 | 后退 |
| 5 | 到达终点 |

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
| `ONE_GRID_M` | 0.31 | 每格距离 |
| `QUARTER_DEGREE` | 133.5 | 四分之一圈角度（编码器计数） |
| `ONE_CIRCLE` | 525 | 完整一圈的编码器计数 |
| `CAMERA_TIMEOUT_MS` | 5000 | K230 等待超时 |
| `RECOVERY_DISTANCE_M` | 0.10 | 超时恢复微移距离 |

## 模块说明

### 蓝牙通信（`Core/peripherals/Bluetooth/`）

HC-05 透传模式，USART2 中断接收。指令：

- **`M`** — 启动路径执行：置 `is_running = true`，切换到 `CAR_WAITING_CAMERA` 状态，向 K230 发出 `'1'` 请求拍照
- **`D`** — 显示模式：切换到 `CAR_STA` 状态，触发 K230 拍照但小车不移动，仅 OLED 显示识别结果或原地转圈
- **`B`** — 置 `Back_permission = true`，由主循环在 `CAR_IDLE` 或 `CAR_WAITING_CAMERA` 中响应并进入 `CAR_MOVING` 回退
- **`S`** — 向 K230 发送 `'0'` 停止指令

收到指令后蓝牙回复 `"Received"` 确认。

### CanMV K230 协议解析（`Core/peripherals/Camera/`）

STM32 端 UART4 中断接收，三级状态机解析 K230 发来的自定义帧协议：

1. **状态 0**：等待帧头 `<`
2. **状态 1**：记录数据类型（M/G/R）
3. **状态 2**：接收 payload 直到帧尾 `>`，然后根据类型解析并置位对应的 permission 标志

- `<M>` 帧 — 路径数据（方向 + 步数），最多 100 步，支持方向/数字合并或分开发送
- `<G>` 帧 — 数值数据（转圈次数）
- `<R>` 帧 — 字符串索引（用于 OLED 显示类别：0=动物, 1=人类, 2=水果）

解析完成后通过蓝牙回传确认（如 `M12` 表示前进 2 格），并置位 `Move_permission` / `Show_permission` / `Revolve_permission`。

### 视觉计算机（`Vision/`）

基于 **01Studio CanMV K230**（RISC-V 双核处理器）运行 MicroPython：

**YOLO11 目标检测**（`main.py`）：
- 加载 `yolo11n_det_320.kmodel`，实时检测数字（0~6）、方向箭头（L/R/F）、目标类别（ANIMAL/PERSON/FRUIT）
- 置信度 ≥ 0.6，NMS 0.45，最大检测框 50
- 检测到 L/R 箭头后触发 `arrow.kmodel` 级联分类提高准确率
- **连续确认机制**：连续 5 帧相同结果才通过 UART 发送 `<M...>` 指令，防误识别
- **状态切换**：收到 `'1'` → `STATE_RUNNING` 全速推理；收到 `'0'` → `STATE_STANDBY` 待机（清屏）

**灰度巡线**（`vision.py` + `camera.py`）：
- 左 ROI `(0,600,300,480)` / 右 ROI `(1620,600,300,480)` 检测黑线 blob
- 计算道路中心偏移量 → 5 帧平滑 → 导航状态（左转 / 右转 / 直行）
- 基于 1920×1080 FHD 灰度图像，阈值 (0, 60)
- 纯算法模块 `camera.py`（无硬件耦合，可被其他模块调用）

**箭头自学习识别**（`arrow.py`）：
- 通过余弦相似度对比特征向量数据库识别箭头方向
- 与 `RecognitionApp` 不同的另一种箭头识别方案
- 协议 `0xFE J <l/r/u/d> 0xEF`

**辅助模块**：
- `photo.py` — 拍照存档
- `QR Code.py` — 二维码检测（场地标定/任务识别）
- `location.py` — 定位模块
- `grib.py` — 网格分割

视觉计算机与 STM32 通过 UART4（115200bps）连接，使用 `<[type][data...]>` 自定义帧协议通信。

### 上位机通信（`Core/peripherals/Host/`）

USART1 接收 PID 参数实时调优指令（`Kp`/`Ki`/`Kd`），支持运行时动态调整。

### OLED 显示（`Core/peripherals/OLED/`）

I2C 接口，0.96 寸 128×64。在 `CAR_STA` 状态下根据 `Show_permission` 显示 K230 识别结果（动物/人类/水果）。

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
| #008 | 摄像头无回复导致小车卡死 | 无超时机制，主循环空转死等 | 5s 超时保护 + RECOVERY 微移恢复 + 显式状态机重构 |

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

### CanMV K230 部署

将 `Vision/` 目录下 `.py` 文件和 `.kmodel` 模型文件拷贝到 K230 的 MicroSD 卡 `/sdcard/` 目录，上电自动运行 `main.py`。

## 项目结构

```
AU/
├── Board/                     # 立创EDA PCB 工程文件
│   ├── AUCompetition_2026-04-21.eprj2   # PCB 项目
│   └── ProPrj_AUCompetition.epro        # 原理图项目
├── Core/
│   ├── Inc/                   # HAL 配置头文件
│   ├── Src/                   # HAL 配置源文件 + main.c
│   └── peripherals/           # 外设驱动模块
│       ├── Bluetooth/         # 蓝牙通信
│       ├── Camera/            # K230 协议解析 + 状态机全局变量
│       ├── Host/              # 上位机调参
│       ├── Motor/             # 电机控制（核心）
│       └── OLED/              # OLED 显示
├── Vision/                    # CanMV K230 MicroPython 视觉代码
│   ├── main.py                # 主程序（YOLO11 检测 + 箭头分类 + 串口输出）
│   ├── vision.py              # 灰度巡线（左右 ROI blob 检测 + 偏移量）
│   ├── camera.py              # 巡线纯算法模块（无硬件耦合）
│   ├── arrow.py               # 箭头自学习识别（余弦相似度）
│   ├── ArrowClassifier.py     # 箭头 CNN 分类器（kmodel）
│   ├── photo.py               # 拍照模块
│   ├── QR Code.py             # 二维码检测
│   ├── location.py            # 定位模块
│   ├── grib.py                # 网格模块
│   └── organizize--arrow&number.py  # 箭头+数字整理
├── Drivers/                   # STM32 HAL 库
├── docs/issues/               # Issue 详细记录
├── CMakeLists.txt             # CMake 构建
├── CMakePresets.json          # CMake 预设
└── AU.ioc                     # STM32CubeMX 项目配置
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
