# Issue #006.1: 旋转模式逐轮独立位置环 —— 代码修改

**Status:** `ready-for-agent`
**Created:** 2026-06-07
**Labels:** bug, motor-control, position-loop, rotation

---

## Parent

[#006: 转弯时小车向外侧漂移 —— 逐轮独立位置环](./006-fix-rotation-lateral-drift.md)

## What to build

修改 `Car_RotateAngle` 函数，将位置环从"四轮平均反馈 + 单一输出"改为**逐轮独立位置环**。具体改动：

### 改动 1：四轮独立位置误差计算

删除 `d_avg` 对四个轮编码器增量求平均的逻辑。每轮用自己的编码器 delta 独立计算位置误差、LPF 滤波、积分和微分，产生独立的 `pos_out_fl` / `pos_out_fr` / `pos_out_rl` / `pos_out_rr`。

位置环状态（prev_error、integral）改用函数内局部变量，每轮各维护一组，不改动 `Car_t` 结构体。

### 改动 2：每轮独立的速度命令

`Motor_ControlWheel` 调用时，每轮传入自己独立的 pos_out 转换后的速度，而非同侧两轮共享同一个 `left_speed` / `right_speed`。

方向逻辑保持现有 Mecanum 旋转模型：FL/RL 同向、FR/RR 反向。

### 改动 3：终止条件改为四轮 AND

将 `if (d_avg >= target_counts)` 改为四轮各自达到 target 才完成：
```c
if (dl0 >= target && dr1 >= target && dl2 >= target && dr3 >= target)
```

### 改动 4：删除旋转中的 Motor_ResetPIDIntegral 调用

移除第 492-498 行（`fabsf(pos_error) < POS_INTEGRAL_RESET_THRESHOLD` 分支中清零速度环积分的代码块）。速度环积分在 10ms 周期自行收敛，不应被位置环外部清零。

### 改动 5：扩展 printf 调试输出

在现有的 `printf("counts:...")` 中增加四轮各自的 pos_out 值，方便串口验证逐轮控制是否生效。

## Acceptance criteria

- [ ] `Car_RotateAngle` 不再使用 `d_avg` 计算位置环反馈
- [ ] 每轮独立计算位置误差、积分、微分和 pos_out
- [ ] 每轮独立调用 `Motor_ControlWheel`，传入自己的速度命令
- [ ] 终止条件为四轮编码器增量**全部** >= target
- [ ] 旋转函数中不再调用 `Motor_ResetPIDIntegral`
- [ ] 串口输出四轮各自的 pos_out，可区分 FL/FR/RL/RR
- [ ] 编译通过，无 warning

## Blocked by

None - can start immediately.