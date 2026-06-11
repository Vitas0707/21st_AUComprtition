# Issue #008: 摄像头识别超时保护 & 主循环状态机重构

**Status:** `ready-for-agent`
**Created:** 2026-06-09
**Labels:** feature, camera, state-machine, timeout, main-loop

---

## Problem Statement

小车每移动完一格后，通过串口向摄像头发送 `'1'` 指令请求下一帧识别结果。摄像头在 UART 中断回调中逐帧解析，解析到 M 类型数据后置 `Move_permission = true`。主循环轮询该标志位驱动小车移动。

摄像头在某些光照/角度条件下可能识别不到下一格目标，导致 `Move_permission` 永远不置 `true`，主循环空转死等，小车卡在原地不动。

目前 `main.c` 主循环使用 `if/else if` 链轮询 `Move_permission` / `Show_permission` / `Revolve_permission` / `Back_permission` 四个布尔标志，但没有任何超时机制。

## Solution

引入 **1 秒超时保护 + 自动恢复机制**：主循环在等待摄像头回复期间启动计时，超时后自动重发 `'1'` 指令并执行前后微移（后退 0.05m + 前进 0.05m），通过微小位置调整帮助摄像头重新捕获目标。恢复后重新进入等待状态，无限重试直到成功。

同时，将主循环从 `if/else if` 链重构为显式状态机 `CarState_t`，以清晰区分"任务未启动"（IDLE）、"正在等摄像头"（WAITING_CAMERA）、"正在移动"（MOVING）、"恢复中"（RECOVERY）四种状态。

## User Stories

1. 作为小车操作者，我希望移动完成后如果摄像头未识别到目标，小车能在 1 秒后自动微调位置并重试，而不是无限期卡在原地。
2. 作为调试者，我希望知道小车当前处于哪个状态（IDLE / WAITING_CAMERA / MOVING / RECOVERY），以便通过串口日志排查问题。
3. 作为维护者，我希望主循环状态机逻辑清晰，不要把"等待摄像头"和"初始空闲"混为一谈。
4. 作为小车操作者，我希望上电后小车不会因为没有收到摄像头数据就触发误微移。
5. 作为调试者，我希望超时恢复期间如果摄像头发来数据，微移不会被中断（避免半途而废导致位置不准）。
6. 作为小车操作者，我希望等待摄像头期间如有 R（显示）或 G（转圈）类型数据不干扰等待逻辑，不会因穿插执行而延误超时判定。

## Implementation Decisions

### 决策 1：引入主循环状态机

用枚举 `CarState_t { IDLE, WAITING_CAMERA, MOVING, RECOVERY }` 替代当前 `if/else if` 链。

状态切换规则：
- `IDLE` → 蓝牙收到 `'M'` → 切 `WAITING_CAMERA`（并发 `'1'` 给摄像头）
- `WAITING_CAMERA` → `Move_permission == true` → 切 `MOVING`
- `WAITING_CAMERA` → `HAL_GetTick() - wait_start > 1000` → 切 `RECOVERY`
- `MOVING` → 移动完成、发 `'1'` → 切 `WAITING_CAMERA`
- `RECOVERY` → 微移完成 → 切 `WAITING_CAMERA`

`Steps[Step_Index].direction` 的枚举（1=直行, 2=右转, 3=左转, 4=终点, 0=无效）保持不变，在 MOVING 状态下内部 switch。

### 决策 2：1 秒超时阈值

使用 `HAL_GetTick()` 时间戳差值，阈值 `1000`（ms）。在 `WAITING_CAMERA` 状态进入时记录 `wait_start_tick`，每次循环检查差值。不使用硬件定时器，利用 Systick 的 1ms 精度已足够。

### 决策 3：恢复动作为重发指令 + 前后微移

超时后执行序列：
1. 重发 `'1'` 给摄像头（`HAL_UART_Transmit(&CAMERA_UART_HANDLE, ...)`）
2. `Car_DriveDistance(car, CAR_DIR_BACKWARD, 0.05f)`
3. `Car_DriveDistance(car, CAR_DIR_FORWARD, 0.05f)`
4. 切回 `WAITING_CAMERA`

与现有 `direction == 0`（无效方向）的微移行为一致，复用同样的距离参数。

### 决策 4：微移期间忽略中断数据

`RECOVERY` 状态下不检查 `Move_permission` / `Show_permission` / `Revolve_permission`。微移完成后手动将所有 permission 清零再切回 `WAITING_CAMERA`，防止微移过程中摄像头恰好回数据但小车位置未到位导致的误执行。

### 决策 5：`is_running` 标志防止上电误触发

`is_running` 初始为 `false`，仅当蓝牙 UART 收到 `'M'` 指令时（`HAL_UART_RxCpltCallback` 中 `Bluetooth_Rx_Buffer[0] == 'M'`）才置 `true` 并切到 `WAITING_CAMERA`。

`IDLE` 状态下即使摄像头提前发来数据导致 `Move_permission == true`，也不会触发移动或超时。`IDLE` 状态不做任何动作。

### 决策 6：R/G 类型数据仅穿插执行

`Show_permission`（R 类型，OLED 显示）和 `Revolve_permission`（G 类型，转圈）在 `MOVING` 状态结束后、切回 `WAITING_CAMERA` 之前穿插执行。

`WAITING_CAMERA` 期间到来的 R/G 被忽略（等待状态只检查 `Move_permission`），因为它们不是移动指令，不会推进 `Steps`。

### 决策 7：MOVING 完成后发 '1'

保持现有逻辑：每个 `Steps[Step_Index]` 处理完后，`HAL_UART_Transmit(&CAMERA_UART_HANDLE, (uint8_t*)&cmd, sizeof(cmd), HAL_MAX_DELAY)` 发送 `'1'`，然后切 `WAITING_CAMERA`。

### 决策 8：Back_permission 融入状态机

`Back_permission`（回退流程）作为 `MOVING` 状态的一个子类型处理——回退也是一系列移动动作，执行方式与普通 MOVING 类似。

### 状态机示意

```
  ┌─────────┐  蓝牙'M'   ┌──────────────────┐  Move_permission  ┌──────────┐
  │  IDLE   │──────────▶│  WAITING_CAMERA  │─────────────────▶│  MOVING  │
  └─────────┘           └──────────────────┘                   └──────────┘
                              │          ▲                          │
                              │ 1s超时    │ 微移完成                  │ 移动完成+发'1'
                              ▼          │                          │
                         ┌──────────┐    │                          │
                         │ RECOVERY │────┘                          │
                         └──────────┘                               │
                                                                    │
                                             发'1'切WAITING ◀────────┘
```

## Testing Decisions

### 测试方法

1. **正常流程测试**：蓝牙 `'M'` 启动 → 小车正常移动 N 格 → 每格都按时收到摄像头数据 → 无超时触发
2. **超时触发测试**：移动完成后物理遮挡摄像头 → 确认 1 秒后触发微移恢复 → 恢复后继续等待
3. **连续超时测试**：持续遮挡摄像头 → 确认无限循环恢复（不崩溃、不退出）
4. **上电误触发测试**：上电后不发送蓝牙 `'M'`，等待 > 1 秒 → 确认不触发微移
5. **微移中断测试**：微移过程中模拟摄像头发来数据 → 确认微移完整执行后再处理
6. **R/G 忽略测试**：WAITING 期间摄像头发来 R 类型数据 → 确认不打断等待、不置 WAITING 计时器

### 好测试的标准

- 正常流程不受超时保护影响（不引入额外延迟）
- 超时后自动恢复，不卡死
- 上电空闲状态不误触发
- 微移动作完整执行，不被数据中断
- 串口日志可看到状态切换轨迹

### 依赖的测试基础设施

- 蓝牙串口工具发送 `'M'` / `'B'` / `'S'` 指令
- 物理遮挡或断开摄像头模拟识别失败
- 串口日志监控状态机切换

## Out of Scope

- 不修改 `Car_DriveDistance` / `Car_RotateAngle` / `Car_StrafeDistance` 内部逻辑（内部 timeout = -2 的处理不在本次范围）
- 不修改摄像头通信协议（`<M...>` / `<R...>` / `<G...>` 帧格式不变）
- 不修改蓝牙通信协议
- 不修改 `Camera.c` 中 `CAMERA_UART_Receive_Process` 状态机解析逻辑
- 不修改 Motor PID 参数
- 不做 HITL（硬件在环测试），仅实车地面测试

## System Context (Zoom-Out)

### 当前主循环架构（重构前）

```
main loop:
  if (Move_permission)       → 执行 Steps[Step_Index] 移动, 发 '1', Move_permission = false
  else if (Show_permission)  → OLED 显示, Show_permission = false
  else if (Revolve_permission) → 转圈, Revolve_permission = false
  else if (Back_permission)  → 回退路径, Step_Index--
  HAL_Delay(10)              → 10ms 主循环周期
```

**问题**：所有 permission 为 false 时，loop 以 10ms 周期空转，无限等待。

### 重构后主循环架构

```
main loop:
  switch (state):
    IDLE           → 不做任何事
    WAITING_CAMERA → 等 Move_permission 或超时
    MOVING         → 执行 Steps[Step_Index], 完成后插入 R/G 处理
    RECOVERY       → 前后微移, 清零所有 permission, 切 WAITING_CAMERA
  HAL_Delay(10)
```

### 涉及文件

| 文件 | 改动 |
|------|------|
| `Core/peripherals/Camera/Camera.h` | 新增 `CarState_t` 枚举、`is_running` 标志、`wait_start_tick` 变量 |
| `Core/peripherals/Camera/Camera.c` | 新增全局变量定义 |
| `Core/Src/main.c` | `while(1)` 主循环用 `switch(state)` 重构；蓝牙 `'M'` 回调中设置 `is_running = true` 并切状态 |

### 与现有运动原语的关系

```
┌─────────────────────────────────────────────────────────────┐
│ Car_DriveDistance（直行）                                    │
│   由 MOVING 状态的 case 1/2/3/4 调用                         │
│   内部已有 timeout_ms 保护（返回 -2），但 main.c 不检查      │
│   → 本次不修改，但未来可考虑处理返回值                       │
├─────────────────────────────────────────────────────────────┤
│ Car_RotateAngle（旋转）                                      │
│   由 MOVING 状态的 case 2/3 调用                             │
│   内部已有 timeout_ms 保护                                   │
├─────────────────────────────────────────────────────────────┤
│ Car_StrafeDistance（平移）                                   │
│   当前未使用，保留                                           │
└─────────────────────────────────────────────────────────────┘
```

## Further Notes

- `Back_permission`（回退流程）在重构后作为 `MOVING` 子类型处理，回退本质上是反向移动 `Steps`，同样需要移动完成后发 `'1'` 等待下一帧
- `case 4`（到达终点）`Step_Index--` 的行为需要确认：到达终点后 `Step_Index` 减一，是否意味着回退一步？这一步是否也需要超时保护？
- `case 0`（direction 无效）的微移逻辑与超时恢复的微移可复用，但当前保留在 MOVING 内部作为兜底处理
- 超时无限重试意味着即使摄像头彻底故障，小车也会持续微移——是否需要一个最大重试上限（如 10 次）后停住等待人为介入？