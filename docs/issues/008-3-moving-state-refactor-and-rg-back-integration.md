# Issue #008.3: MOVING 状态重构 + R/G 穿插 + Back 整合

**Status:** `ready-for-agent`
**Created:** 2026-06-09
**Labels:** feature, state-machine, refactor, main-loop

---

## Parent

[#008 摄像头识别超时保护 & 主循环状态机重构](./008-camera-timeout-protection-and-main-loop-state-machine.md)

## What to build

将现有 `Steps[Step_Index]` 的 `switch(direction)` 逻辑从旧的 `if/else if` 链迁移到 `MOVING` 状态内。移动完成后先穿插执行 `Show_permission`（OLED 显示）和 `Revolve_permission`（转圈），再发 `'1'` 切 `WAITING_CAMERA`。`Back_permission` 作为 MOVING 子类型处理。删除旧的 `if (Move_permission) ... else if (Show_permission) ... else if (Revolve_permission) ... else if (Back_permission)` 链。

## Acceptance Criteria

- [ ] `MOVING` 状态内实现 `switch(Steps[Step_Index].direction)`：case 1 直行、case 2 右转、case 3 左转、case 4 终点、case 0 无效微移
- [ ] 每个 case 内移动完成后：
  - 若 `Show_permission` → 执行 OLED 显示 → `Show_permission = false`
  - 若 `Revolve_permission` → 执行转圈 → `Revolve_permission = false`
  - `Move_permission = false`
  - `HAL_UART_Transmit(..., '1', ...)` 发下一帧请求
  - `wait_start_tick = HAL_GetTick()`
  - `car_state = WAITING_CAMERA`
- [ ] `Back_permission` 作为 MOVING 状态的子分支：反向执行 Steps 移动序列
- [ ] 删除旧的 `if/else if` 链（`Move_permission` / `Show_permission` / `Revolve_permission` / `Back_permission` 的顶层判断）
- [ ] 完整流程测试：正常移动 → 显示 → 转圈 → 回退，各个路径不受影响

## Blocked by

[#008.2 超时检测 + 恢复微移](./008-2-timeout-detection-and-recovery-micro-move.md)