# Issue #003: 转弯极限环振荡 —— 位置模式下禁用速度积分

**Status:** `ready-for-agent`
**Created:** 2026-06-07
**Labels:** bug, motor-control, pid
**Parent:** #002

---

## Problem Statement

Issue #002 修复（条件积分 + 位置积分 + 阈值 200）在空载下正常工作，但带负载转弯时在 ~80° 位置出现 ±2.5° 的缓慢来回摆动（极限环振荡）。根因是位置环积分和速度环积分在 cascade 结构中同时工作：速度环积分让轮速跟踪滞后于位置环指令，两个积分器互相推拉形成慢速极限环。空载时惯性小、跟得上指令所以不抖；负载大时惯性增大、相位延迟增加，极限环出现。

## Solution

在 `Car_RotateAngle` 和 `Car_DriveDistance` 的位置环循环期间，临时将四个轮子的速度环 `ki` 设为零，循环结束后恢复。位置环由 `pos_integral` 消除静差，速度环仅用 P+D 跟踪位置环输出的速度指令，避免双积分耦合。

同时回退 Issue #002 中引入的条件积分逻辑（`SPEED_INTEGRAL_WINDOW`），恢复为无条件积分。因为 ki=0 时积分不会启动，条件判断成为死代码。

## User Stories

1. As a 比赛小车操作员，I want 带负载转弯时不再出现 ±2.5° 的缓慢摇摆，so that 路径准确不浪费时间
2. As a 比赛小车操作员，I want 直线行驶在负载下也不出现极限环，so that 直行稳定
3. As a 调试工程师，I want 速度环积分在位置模式下被禁用而在独立使用时正常，so that 两种使用场景不互相干扰
4. As a 测试者，I want 空载行为不受影响，so that 回归测试通过
5. As a 开发者，I want 位置模式的 ki 禁用逻辑集中在位置环循环入口/出口，so that 容易理解和维护

## Implementation Decisions

1. **位置模式下禁用速度积分**：在 `Car_RotateAngle` 和 `Car_DriveDistance` 的循环前保存四个轮子的 `pid.ki`，设为零；循环结束后恢复原值
2. **回退条件积分**：将 `Motor_PIDIncrementalUpdate` 中的条件积分逻辑恢复为无条件 `integral += error * dt_s`（即回退 #002 中新增的 `SPEED_INTEGRAL_WINDOW` 判断）
3. **保留位置积分**：`pos_integral`、`POS_KI`、`POS_INTEGRAL_LIMIT` 保持不变（#002 引入），它们是消除静差的唯一积分源
4. **保留积分复位阈值 200**：`POS_INTEGRAL_RESET_THRESHOLD` 保持 200（#002 引入），接近目标时清零位置积分和速度 PID 历史状态
5. **不新增结构体字段**：不用 `position_mode` 标志位，直接临时改 ki（调用栈清晰，`Motor_ControlWheel` 在 main.c 中被注释掉）
6. **不修改 `Motor_ControlWheel`**：函数行为不变
7. **不修改 `Motor_ResetPIDIntegral`**：函数行为不变
8. **不修改 `Car_StrafeDistance`**：其为开环控制，无位置环

## Testing Decisions

- **良好测试的标准**：带负载执行 90°/180° 转弯，目测无 ±2.5° 摇摆，设定 125° 到达角度 ≥ 90°
- **测试方式**：硬件在环 —— 实际带负载小板车运行标准路线
- **回归测试**：空载转弯行为与 #002 一致；`Motor_ControlWheel` 独立驱动速度 PID 正常（ki 不被意外清零）

## Out of Scope

- 速度环 Kp/Kd 重新整定
- 位置环 PD 参数调整
- PWM 前馈值调整
- `Car_StrafeDistance` 改造
- 新增位置模式标志位到 `Motor_t`

## Further Notes

- 如果 `pos_integral` 消除静差效果不够（转弯仍差几度），可逐步增大 `POS_KI` 从 0.005→0.01→0.02
- 如果速度环纯 P+D 导致负载响应太慢，可考虑给 P 项加一个负载前馈（`FEED_FORWARD_PWM` 已经存在）
- 最终目标是 `Car_RotateAngle(&car, CAR_DIR_CW, 90)` 就能转到 90°，不再需要设 125°
- 备选方案（position_mode 标志位）更干净但改动范围更大，当前方案最小化改动、风险最低