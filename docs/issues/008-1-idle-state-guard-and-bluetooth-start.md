# Issue #008.1: IDLE 状态守卫 + 蓝牙启动

**Status:** `ready-for-agent`
**Created:** 2026-06-09
**Labels:** feature, state-machine, main-loop

---

## Parent

[#008 摄像头识别超时保护 & 主循环状态机重构](./008-camera-timeout-protection-and-main-loop-state-machine.md)

## What to build

新增主循环状态机的基础设施：定义 `CarState_t` 枚举（IDLE / WAITING_CAMERA / MOVING / RECOVERY）、`is_running` 标志、`wait_start_tick` 时间戳。实现 IDLE 状态（上电后主循环不做任何事），仅当蓝牙收到 `'M'` 指令时切到 WAITING_CAMERA 并发 `'1'` 给摄像头。

## Acceptance Criteria

- [ ] `Camera.h` 新增 `CarState_t` 枚举、`extern CarState_t car_state`、`extern bool is_running`、`extern uint32_t wait_start_tick`
- [ ] `Camera.c` 定义全局变量，`is_running = false`, `car_state = IDLE`
- [ ] `main.c` 主循环用 `switch(car_state)`（此时只处理 `IDLE`，其他状态为 break）
- [ ] 蓝牙 UART 回调中 `'M'` 分支：`is_running = true; car_state = WAITING_CAMERA; wait_start_tick = HAL_GetTick(); HAL_UART_Transmit(..., '1', ...)`
- [ ] 上电后无论摄像头提前发什么数据，不触发移动、不触发超时
- [ ] 蓝牙 `'M'` 后发 `'1'` 给摄像头，进入等待

## Blocked by

None — 可立即开始