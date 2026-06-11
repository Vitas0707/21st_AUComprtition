# Issue #008.2: 超时检测 + 恢复微移

**Status:** `ready-for-agent`
**Created:** 2026-06-09
**Labels:** feature, timeout, recovery, main-loop

---

## Parent

[#008 摄像头识别超时保护 & 主循环状态机重构](./008-camera-timeout-protection-and-main-loop-state-machine.md)

## What to build

实现 WAITING_CAMERA 和 RECOVERY 两个状态。WAITING_CAMERA 状态下轮询 `Move_permission`，若为 true 则切 MOVING。每轮检查 `HAL_GetTick() - wait_start_tick > 1000`，超时则进入 RECOVERY。RECOVERY 状态执行恢复序列：重发 `'1'` → 后退 0.05m → 前进 0.05m → 清零所有 permission → 记录新 `wait_start_tick` → 切回 WAITING_CAMERA。

## Acceptance Criteria

- [ ] `WAITING_CAMERA` 状态下：
  - `Move_permission == true` → `car_state = MOVING`
  - `HAL_GetTick() - wait_start_tick > 1000` → `car_state = RECOVERY`
  - 不检查 `Show_permission` / `Revolve_permission`（忽略 R/G 数据）
- [ ] `RECOVERY` 状态下：
  - `HAL_UART_Transmit(&CAMERA_UART_HANDLE, "1", 1, HAL_MAX_DELAY)` 重发指令
  - `Car_DriveDistance(&car, CAR_DIR_BACKWARD, 0.05f)`
  - `Car_DriveDistance(&car, CAR_DIR_FORWARD, 0.05f)`
  - `Move_permission = Show_permission = Revolve_permission = false`
  - `wait_start_tick = HAL_GetTick()`
  - `car_state = WAITING_CAMERA`
- [ ] 遮挡摄像头测试：1 秒后小车前后微移 → 继续等待

## Blocked by

[#008.1 IDLE 状态守卫 + 蓝牙启动](./008-1-idle-state-guard-and-bluetooth-start.md)