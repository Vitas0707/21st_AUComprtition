# Car_DriveDistance 到达条件改为四轮平均

## Parent

#005: 直线行驶到达条件 OR 逻辑导致逐段累积右偏

## What to build

将 `Car_DriveDistance` 的到达判断从四轮分别判断（OR 逻辑）改为四轮平均计数（`d_avg >= target_counts - 5`），与 #004 对 `Car_RotateAngle` 的修复保持一致。

当前代码中，loop 前半段已计算 `current_counts = (d0+d1+d2+d3)/4`，loop 后半段到达判断处重算 d0~d3 后使用 OR 逻辑。修复方案：到达判断处复用已计算的 `current_counts` 替代 OR 逻辑。

## Acceptance criteria

- [ ] `Car_DriveDistance` 到达条件使用四轮平均 `current_counts >= target_counts - 5`
- [ ] 编译通过，无 warning
- [ ] 连续行走 5 段 0.3m 无肉眼可见偏航
- [ ] 单段 0.3m 直行距离准确（不引入回归）

## Blocked by

None - can start immediately