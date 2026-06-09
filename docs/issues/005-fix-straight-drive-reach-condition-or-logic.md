# Issue #005: 直线行驶到达条件 OR 逻辑导致逐段累积右偏

**Status:** `ready-for-agent`
**Created:** 2026-06-07
**Labels:** bug, motor-control, position-loop

---

## Problem Statement

直线行驶（`Car_DriveDistance`）每段 0.3m，走到第 3-4 段时小车出现明显右偏。第 1 段还可接受，后面越来越偏。

## Root Cause

`Car_DriveDistance` 中**到达条件使用 OR 逻辑**（任意一个轮子编码器计数 >= target 就判定到达），但**位置环控制的是四轮平均距离**：

```c
// 位置环：pos_out 基于四轮平均
int64_t current_counts = (d0 + d1 + d2 + d3) / 4;
float pos_raw_error = (float)(target_counts - current_counts);

// 到达判断：任意一个轮子到就停
if (d0 >= target_counts-5) reached = true;  // FL
if (d1 >= target_counts-5) reached = true;  // FR
if (d2 >= target_counts-5) reached = true;  // RL
if (d3 >= target_counts-5) reached = true;  // RR
```

当右侧轮因机械差异（摩擦力小/电机效率高）略快于左侧时，右侧轮先到达目标计数，触发全车刹车。左侧轮累积少走距离。每段 0.3m 重复一次，第 3-4 段偏差距累积到肉眼可见。

Issue #004 已修复 `Car_RotateAngle` 的同样问题（改为 `d_avg >= target_counts`），但当时决策明确排除了 `Car_DriveDistance`（见 #004 Out of Scope 第 4 条："不修改 Car_DriveDistance 的到达条件：直行时四轮平均判断合理，且直行不存在方向不一致问题"）。**现在实际测试证明：直行同样存在方向不一致（左右机械差异），到达条件 OR 逻辑同样导致问题。**

## Solution

将 `Car_DriveDistance` 的到达条件从四轮 OR 逻辑改为四轮平均单一条件。

### 具体改动

**文件：** 电机控制模块  
**位置：** `Car_DriveDistance` 函数内到达判断段  

**当前代码（四轮 OR）：**
```c
bool reached = false;
d0 = car->fl->total_counts - start_counts[0]; if (d0 < 0) d0 = -d0; if (d0 >= target_counts-5) reached = true;
d1 = car->fr->total_counts - start_counts[1]; if (d1 < 0) d1 = -d1; if (d1 >= target_counts-5) reached = true;
d2 = car->rl->total_counts - start_counts[2]; if (d2 < 0) d2 = -d2; if (d2 >= target_counts-5) reached = true;
d3 = car->rr->total_counts - start_counts[3]; if (d3 < 0) d3 = -d3; if (d3 >= target_counts-5) reached = true;

if (reached) { ... }
```

**改为（四轮平均）：**
```c
// 复用 loop 前面已经算过的 current_counts = (d0+d1+d2+d3)/4
// 或重新计算 d_avg
int64_t d_avg = (d0 + d1 + d2 + d3) / 4;
if (d_avg >= target_counts - 5) { ... }
```

## User Stories

1. As a 比赛小车操作员，I want 连走多段直线不出现累积偏航，so that 路径准确不偏离赛道
2. As a 调试工程师，I want 直行到达条件与转弯一致（均为四轮平均），so that 行为可预期
3. As a 测试者，I want 修复后单段直行距离仍然准确，so that 不引入回归

## Implementation Decisions

1. **到达条件改为 d_avg**：与 #004 对 `Car_RotateAngle` 的修复保持一致
2. **不修改位置环结构**：pos_out 仍基于四轮平均计算四轮相同 wheel_speed，无需引入差速控制
3. **阈值维持 target_counts - 5**：与现有逻辑一致
4. **不修改速度环 PID 参数**：左右轮 PID 相同是另一个独立问题（假设 #3），本次不处理

## Testing Decisions

- **反馈循环**：临时启用主循环中被注释的 `printf("para:%.2f,%.2f,%.2f,%.2f\r\n", FL.speed_m_s, FR.speed_m_s, RL.speed_m_s, RR.speed_m_s)`，验证四轮速度是否对称
- **良好测试的标准**：连续行走 5 段 0.3m 后无肉眼可见偏航
- **测试方式**：硬件在环，实际带负载板车运行标准直行路线
- **回归测试**：单段 0.3m 直行距离准确（编码器计数验证）

## Out of Scope

- FR 方向反转统一（假设 #2）
- 左右轮独立 PID 标定（假设 #3）
- 速度环 Kp/Ki/Kd 重新整定
- `Car_StrafeDistance` 到达条件修改（横向移动暂未发现问题）

## Further Notes

- 此修复是 #004 的自然延续——#004 修复了转弯的同一问题，当时排除了直行，现在实际数据证明直行也需要
- 备选方案：如果 d_avg 到达条件后仍偏航，可能需要引入独立左右 PID 或增加差速校正项
- 与 #004 联动：`DEFAULT_ERROR_LPF_ALPHA=0.5f` 和 `POS_KD=5.0f` 已在 #004 中调整，本修复受益于这些改进