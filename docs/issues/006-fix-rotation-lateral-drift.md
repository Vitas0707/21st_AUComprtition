# Issue #006: 转弯时小车向外侧漂移 —— 逐轮独立位置环

**Status:** `ready-for-agent`
**Created:** 2026-06-07
**Labels:** bug, motor-control, position-loop, rotation

---

## Problem Statement

Mecanum 四轮小车在调用 `Car_RotateAngle` 原地旋转时，车体向外侧漂移（CCW 左转时向右漂移）。当前用 `QUARTER_DEGREE=130°` 硬编码补偿角度偏差，未从根本上解决漂移问题。串口数据确认：旋转时右侧轮（FR/RR）编码器增量明显慢于左侧，达到某个点后被快轮的平均值强制停止，慢轮未到位。

## Root Cause (from grill-me diagnostics)

三个叠加根因：

1. **位置环反馈用四轮平均 `d_avg`**：`Car_RotateAngle` 将 FL/FR/RL/RR 四个轮的编码器位移取平均得到一个 `pos_error`，然后用单一的 `pos_out` 驱动四轮。即使 RL 比 FL 快，位置环也感知不到差异。

2. **终止条件 `d_avg >= target_counts`**：快轮多跑 + 慢轮少跑 = 平均值刚好达标 → 慢轮没转够角度 → 旋转结束时有残余偏角 → 用 130° 补。

3. **接近目标时 `Motor_ResetPIDIntegral` 清零速度环积分**：`Car_RotateAngle` 中当位置误差小于 `POS_INTEGRAL_RESET_THRESHOLD` 时调用 `Motor_ResetPIDIntegral` 清零四个轮的速度环积分，导致 RR/FR 之间被积分补偿压住的 0.08 m/s 速度差异重新出现。

实车数据验证：RR 在旋转时持续比 FR 快约 0.08 m/s（向后方向），右侧两个后轮推力不平衡 → 产生横向平移分力 → "左转右漂"。

## Solution

将 `Car_RotateAngle` 的位置环从"四轮平均反馈 + 单一输出"改为**逐轮独立位置环**：

- 每轮用自己的编码器 delta 独立计算位置误差
- 每轮独立维护位置环的 prev_error / integral
- 每轮用自己的 pos_out 作为速度命令
- 终止条件改为四轮全部达到 target

同时删除旋转函数中对 `Motor_ResetPIDIntegral` 的调用，保持速度环积分正常工作。

## User Stories

1. 作为小车操作者，我希望 CCW 原地旋转 90° 后车体几何中心不产生平移偏移，这样旋转后能正对下一个路口的中心线。
2. 作为小车操作者，我希望 CW 原地旋转 90° 后同样不产生平移偏移，左右转行为对称。
3. 作为调试者，我希望旋转过程中能通过串口看到四轮各自的编码器增量和位置环输出，以便验证逐轮控制是否生效。
4. 作为调试者，我希望旋转完成时四轮编码器增量均在目标值附近（误差 < 5 个计数），而非仅平均值达标。
5. 作为维护者，我希望 `Car_RotateAngle` 的位置环逻辑与 `Car_DriveDistance` 保持结构一致（逐轮独立），降低后续维护成本。

## Implementation Decisions

### 决策 1：位置环状态从 Car_t 中移除，改为函数内局部变量

`Car_t` 目前只有一组 `pos_prev_error` / `pos_integral` / `lpf_alpha`。逐轮独立需要 4 组。这些状态不需要跨函数持久化（每次 `Car_RotateAngle` 调用都是全新的一次旋转），在函数内用 4 组局部变量维护即可，不改动 `Car_t` 结构体。

### 决策 2：每轮独立 LPF（低通滤波）

每轮维护独立的 `pos_prev_error_raw` 用于 LPF：
```
pos_error_fl = lpf_alpha * (target - dl0) + (1 - lpf_alpha) * pos_error_fl_prev
```

lpf_alpha 仍使用 `car->lpf_alpha`（默认 0.5）。

### 决策 3：每轮独立积分和抗饱和

每轮维护独立的 `pos_integral_xx`，使用相同的 `POS_INTEGRAL_LIMIT`（50.0）进行钳位。

### 决策 4：删除旋转中的 Motor_ResetPIDIntegral 调用

旋转函数中不再调用 `Motor_ResetPIDIntegral`。速度环积分在 10ms 周期自行收敛，不应被位置环外部清零。

### 决策 5：终止条件改为四轮 AND

```c
if (dl0 >= target && dr1 >= target && dl2 >= target && dr3 >= target) {
    // 旋转完成
}
```

### 决策 6：左右速度方向保持现有逻辑

Mecanum 旋转时 FL/RL 同向、FR/RR 反向，运动学上是正确的，不做修改。仅改位置环反馈和输出层面。

### 决策 7：超时逻辑不变

超时仍然返回 -2，保持现有超时时间计算逻辑。

## Testing Decisions

### 测试方法

1. **串口日志验证**：在旋转循环中 printf 四轮编码器增量（dl0, dr1, dl2, dr3）和各自的 pos_out（已有 printf 框架，扩展即可）。
2. **完成时验证**：旋转结束后 printf 四轮最终编码器增量，确认四轮均 >= target。
3. **实车地面测试**：将 `QUARTER_DEGREE` 恢复为 90，测试 CCW 和 CW 各 5 次，观察是否还有向外漂移，以及停位后是否正对路口。

### 好测试的标准

- 只测试外部可观察行为（旋转完成后的车体位置/朝向），不测试内部 PID 中间值
- 相同条件下多次测试结果一致（非随机漂移）
- CCW 和 CW 行为对称

### 参考现有测试

- `Car_DriveDistance` 已经有 printf 输出四轮速度的注释代码（第 343 行、第 528 行），旋转函数可复用同样的格式。

## Out of Scope

- 不修改 `Car_DriveDistance` 和 `Car_StrafeDistance` 的位置环（它们的问题已有独立 issue）
- 不调整四轮 PID 参数（Kp/Ki/Kd 的机械差异补偿在直行场景已验证有效）
- 不修改 Mecanum 运动学模型（旋转时四轮同速反向是正确的）

## System Context (Zoom-Out)

本轮 fix 涉及以下模块及它们之间的关系：

### 控制层级（顶层到底层）

```
Main Loop (main.c)
├── 路径规划 / 权限管理
│   ├── Move_permission     → 触发 Car_DriveDistance / Car_RotateAngle
│   ├── Show_permission     → OLED 显示 Camera_Data
│   ├── Revolve_permission  → Car_RotateAngle × N 圈
│   └── Back_permission     → 回退（反向 Car_DriveDistance + Car_RotateAngle）
│
├── 高层运动原语 (Motor.c)
│   ├── Car_DriveDistance()   ← 直线行驶位置环（逐轮独立的 #005 已修复）
│   ├── Car_StrafeDistance()  ← 横向移动（开环速度，无位置环）
│   └── Car_RotateAngle()     ← ★ 本次修改目标：原地旋转位置环
│
├── 中层速度控制 (Motor.c)
│   ├── Motor_ControlWheel()  ← 10ms 周期速度环入口
│   │   ├── Motor_UpdateEncoder()  ← 读编码器 → 算 speed_m_s
│   │   └── Motor_PIDIncrementalUpdate() ← 增量 PID → 输出 PWM
│   └── Motor_ResetPIDIntegral()   ← ★ 旋转中被错误调用的清零函数
│
└── 底层硬件抽象 (Motor.c)
    ├── Motor_StartWheel()  ← 设置 GPIO 方向 + PWM 占空比
    ├── Motor_StopWheel()   ← 清零 PWM + 刹车
    └── Motor_SetDirectionInverted() ← 方向反转补偿
```

### 控制环关系（时序）

```
每个 10ms 周期：
  ┌─────────────────────────────────────────────────┐
  │ 位置环 (Car_RotateAngle 内部)                    │
  │   pos_error = target_counts - encoder_delta       │
  │   pos_out = POS_KP*e + POS_KI*∫e + POS_KD*de     │
  │   → 输出 left_speed / right_speed                 │
  └────────────────────┬────────────────────────────┘
                       ▼
  ┌─────────────────────────────────────────────────┐
  │ 速度环 (Motor_ControlWheel 内部)                  │
  │   speed_error = target_speed - speed_m_s          │
  │   pwm_delta = KP*e + KI*∫e + KD*de                │
  │   → 输出 PWM + 方向                               │
  └────────────────────┬────────────────────────────┘
                       ▼
  ┌─────────────────────────────────────────────────┐
  │ 轮子 (编码器反馈)                                │
  │   rpm → speed_m_s → total_counts                  │
  │   → 反馈给速度环和位置环                          │
  └─────────────────────────────────────────────────┘
```

### 当前问题：反馈路径断裂

```
本轮修复前：
  FL ████████████████████ 编码器 → ┐
  FR ██████████████░░░░░░ 编码器 → ┤
  RL ████████████████████ 编码器 → ┼→ d_avg → 1 个 pos_out → 4 轮共享
  RR ████████████████░░░░ 编码器 → ┘
                                    ↑ 慢轮被快轮掩盖
  
  终止条件: d_avg >= target → 快轮到位就停
  → 慢轮没转够 → 角度不足 → 130° 补丁

本轮修复后：
  FL ████████████████████ 编码器 → pos_out_fl → Motor_ControlWheel(FL)
  FR ████████████████████ 编码器 → pos_out_fr → Motor_ControlWheel(FR)
  RL ████████████████████ 编码器 → pos_out_rl → Motor_ControlWheel(RL)
  RR ████████████████████ 编码器 → pos_out_rr → Motor_ControlWheel(RR)
                                    ↑ 四轮独立闭环
  
  终止条件: 四轮都 >= target → 每轮都到位才停
```

### 相关数据结构

| 结构体 | 位置 | 职责 |
|--------|------|------|
| `Motor_t` | Motor.h | 单轮：PWM/Gpio/编码器/速度/PID 状态 |
| `Car_t` | Motor.h | 四轮聚合：fl/fr/rl/rr 指针 + 公共参数 + 一组共享位置环状态（本次不改） |
| `CameraData_t` | Camera.h | 摄像头识别结果（类别 + 数值） |
| `step_t` | Camera.h | 路径步骤（方向 + 步数） |

### 文件依赖

```
main.c
  ├── Motor.h      ← 所有运动原语
  ├── Camera.h     ← <M> / <R> / <G> 协议解析
  ├── Bluetooth.h  ← 蓝牙 UART 回调
  ├── oled.h       ← OLED 显示
  └── host.h       ← 主机通信

Motor.h / Motor.c
  ├── stm32f4xx_hal_gpio.h
  ├── stm32f4xx_hal_tim.h
  └── main.h       ← HAL_GetTick(), printf
```

### 本轮 fix 只改

- **Motor.c** — `Car_RotateAngle()` 函数（约 90 行，第 446-538 行）
- **main.c** — 恢复 `QUARTER_DEGREE` 为 90（在 #006.2 测试阶段）

### 不涉及的模块

- 摄像头协议解析（Camera.c）—— 只管发指令、不管运动执行
- OLED 显示 —— 不受影响
- 蓝牙通信 —— 不受影响
- 主机通信（host.c）—— 不受影响
- `Car_DriveDistance` / `Car_StrafeDistance` —— 独立模块，各自有 pending issues
- `Car_t` 结构体 —— 不改动，位置环状态用局部变量

## Further Notes

- 该修复是 #003（旋转中禁用速度环 Ki）和 #004（位置环阻尼和到达条件）的延续。之前的修改已经撤销了旋转中清零 Ki 的代码，但未解决 d_avg 反馈和复位积分的问题。
- 预期修复后 `QUARTER_DEGREE` 可以恢复为 90（或接近 90 的值）。