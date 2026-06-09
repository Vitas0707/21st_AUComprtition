# Issue #001: 转弯振荡 —— 积分饱和修复

**Status:** `ready-for-agent`  
**Created:** 2026-06-06  
**Labels:** bug, motor-control, pid

---

## Problem Statement

加了负载后，小车转弯时出现反复振荡：先转过目标角度 → 再反向转回来 → 又转回去。这是因为速度环增量式 PID 的积分项在负载增加后持续累积（上限 ±500），接近目标时位置环输出缩小的目标速度被积分"淹没"，导致车轮冲过目标、产生反向误差、积分反向累积、形成反复振荡。

## Solution

在 `Car_RotateAngle` 和 `Car_DriveDistance` 的位置环中，当位置误差缩小到接近目标的范围内时，主动清零四个轮子速度 PID 的积分项，防止积分拖尾引发超调振荡。同时在 `Motor_PIDIncrementalUpdate` 中增加更紧的积分钳位，减少大误差下的积分累积速度。

## User Stories

1. As a 比赛小车操作员, I want 转弯在加了负载后也能一次到位不振荡, so that 路径准确、不浪费时间
2. As a 调试工程师, I want PID 积分项在接近目标时自动清零, so that 我不需要手动反复调参来压制振荡
3. As a 系统设计者, I want 直线行驶也能受益于积分抗饱和, so that 直行过冲也能被抑制
4. As a 开发者, I want 积分清零逻辑和位置环耦合, so that 不影响速度环独立使用时的行为
5. As a 测试者, I want 无负载时行为不受影响, so that 改动不引入回归

## Implementation Decisions

1. **积分清零时机**：在位置环循环中，当 `pos_error < 阈值`（建议初始值 20 counts）时，对四个轮子调用清零
2. **积分清零接口**：新增 `Motor_ResetPIDIntegral(Motor_t* m)` 函数，只清零 `integral`、`prev_error`、`prev_prev_error`，不改变 output
3. **积分钳位收紧**：将 `Motor_PIDIncrementalUpdate` 中的积分上限从 ±500 降低到 ±200
4. **不修改 `Motor_ControlWheel` 的行为**：清零逻辑由上层位置环控制
5. **不修改 `Car_StrafeDistance`**：它为开环速度控制，不受积分饱和影响

## Affected Modules

- `Core/peripherals/Motor/Motor.h` — 新增 `Motor_ResetPIDIntegral` 声明
- `Core/peripherals/Motor/Motor.c` — 新增 `Motor_ResetPIDIntegral` 实现 + 收紧积分上限
- `Core/Src/main.c` — `Car_RotateAngle` / `Car_DriveDistance` 中集成清零调用

## Testing Decisions

- **良好测试的标准**：只验证外部可观测行为（车轮不振荡、转角在容忍范围内），不验证内部 PID 状态
- **测试方式**：硬件在环测试 —— 带负载执行 90°/180°/370° 转弯，通过编码器日志确认无反向转动
- **回归测试**：无负载时转弯行为应与改动前一致

## Out of Scope

- 重新整定速度环 PID 参数（Kp/Ki/Kd）
- 修改位置环 POS_KP/POS_KD
- 调整 PWM 死区或前馈值
- `Car_StrafeDistance` 的改造

## Further Notes

- 如果积分清零阈值太小，可能导致接近目标时"刹不住"；如果太大，清零太早又可能达不到目标。建议从 20 counts 开始迭代
- 另一个备选方案是反计算积分（back-calculation anti-windup），但增量式 PID 中实现较复杂，清零法更直接