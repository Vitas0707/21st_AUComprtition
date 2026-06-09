# Issue #004: 位置环阻尼不足 + 到达条件导致无法停止

**Status:** `ready-for-agent`
**Created:** 2026-06-07
**Labels:** bug, motor-control, pid
**Parent:** #003

---

## Problem Statement

Issue #003 修复（位置模式下禁用速度积分）后，转弯能到 90° 附近，但无法停在目标。串口日志显示：

- `target_counts = 1241`（125° 目标）
- 第一次超调到 1430（+189 counts ≈ 19°），然后回调到 1130（-111 counts）
- 在 1130 ~ 1430 之间反复摆动（~300 counts ≈ 30° 的峰峰振幅）
- 阻尼极慢，末尾仍在 1130~1320 振荡
- 从未触发 `reached` 条件，无法自然停止

三个并发根因：

1. **位置环 D 增益太小**：`POS_KD=0.1` 除以 `PID_CONTROL_PERIOD_MS=10` → 有效 D = 0.01，只有 P 的 1/100。位置环是事实上的 PI，没有制动项。
2. **到达判断用四轮分离的绝对值**：`dl0 >= target_counts-5 && dl2 >= target_counts-5` 判定左侧到达。转弯时两侧编码器不同步，一侧先到、另一侧还没到 → 全程继续 → 过冲。
3. **lpf_alpha=0.1 滤波太狠**：`pos_error = 0.1*raw + 0.9*prev`，相当于加了一层低通，引入额外相位滞后。

## Solution

四管齐下：

1. `POS_KD` 从 0.1 提升到 5.0（有效 D = 0.5，制动增强 50 倍）
2. 到达判断改为用 `d_avg >= target_counts`（四轮平均，统一标准）
3. 默认 `lpf_alpha` 从 0.1 提高到 0.5（减半滤波延迟）
4. `POS_INTEGRAL_RESET_THRESHOLD` 从 200 提升到 400（振荡时积分不被清零，帮助拉住偏差）

## User Stories

1. As a 比赛小车操作员，I want 转弯能在接近目标后自动停下，so that 不需要手动发停止指令
2. As a 比赛小车操作员，I want 转弯不反复振荡，so that 路径准确不浪费时间
3. As a 调试工程师，I want 位置环有足够的 D 增益来制动，so that 系统不欠阻尼
4. As a 测试者，I want 空载下到达判断仍然正确，so that 不引入回归
5. As a 开发者，I want 到达条件可理解且稳定，so that 调试时不用猜测

## Implementation Decisions

1. **POS_KD 提升**：从 0.1f 提升到 5.0f（在 Motor.h 中）
2. **默认 lpf_alpha 提升**：从 0.1f 提升到 0.5f（在 Motor.h 中）。注意 Car_Init 中赋值 `DEFAULT_ERROR_LPF_ALPHA` 会自动生效
3. **到达判断改为 d_avg**：`Car_RotateAngle` 中移除 `left_reached`/`right_reached` 分离判断，改为 `d_avg >= target_counts` 单一条件
4. **POS_INTEGRAL_RESET_THRESHOLD 提升**：从 200 提升到 400（在 Motor.h 中）
5. **不修改 Car_DriveDistance 的到达条件**：直行时四轮平均判断合理，且直行不存在方向不一致问题
6. **不修改 PID 结构**：保持 ki=0 + pos_integral 的架构
7. **不修改 MIN_PWM_THRESHOLD**：待进一步测试是否需要

## Testing Decisions

- **反馈循环**：`printf("%d,%d\r\n", (int)d_avg, (int)target_counts)` 在 Car_RotateAngle 中输出日志
- **良好测试的标准**：d_avg 第一次超过 target_counts 后立即触发 reached，无反向振荡
- **测试方式**：硬件在环，带负载执行 125° 转弯，对比修复前后日志
- **回归测试**：空载转弯 + 直行均正常

## Out of Scope

- 速度环 Kp/Kd 重新整定
- MIN_PWM_THRESHOLD 调整
- Car_DriveDistance 到达条件修改
- 位置环结构改为完整 PID（保留当前 PD+I 架构）

## Further Notes

- 如果 POS_KD=5.0 仍然制动不足（首次过冲 > 50 counts），可继续增大到 8.0 或 10.0
- 如果 d_avg 到目标后还触发 reached 太慢，可减小阈值从 5 到 3（`d_avg >= target_counts - 3`）
- 日志确认修复后，删除 printf
- 备选方案：在到达条件触发前降低最大 PWM（vel limit），但改动更大