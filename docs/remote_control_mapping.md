# 遥控器数值与动作对照表

本文档按当前 [mode_task.c](../app/task/mode_task/mode_task.c) 的实际实现整理，目标是把：

- 遥控器原始数值
- 拨杆/拨轮位置
- 写入共享池后的模式与动作
- 组合起来代表的中文意思

放到一张清晰的对照表里。

## 先校正一个容易混淆的点

当前代码里，`sw1` 和 `sw2` 的数值定义都是：

| 数值 | 位置 |
| --- | --- |
| `1` | `UP` |
| `2` | `DN` |
| `3` | `MI` |

所以：

- `sw2 = 1` 才是 `up`
- `sw2 = 2` 是 `down`
- `sw2 = 3` 是 `mid`

如果上位机里看到 `sw2 = 2`，它对应的是 `down`，不是 `up`。

## `iw` 的边沿定义

`iw` 不是按绝对值直接判动作，而是按边沿事件判动作：

| 事件名 | 代码条件 | 中文意思 |
| --- | --- | --- |
| `iw 上边沿` | 上一拍 `iw > 694`，当前拍 `iw <= 694` | 拨轮从较大值跨到上阈值以内 |
| `iw 下边沿` | 上一拍 `iw < 1354`，当前拍 `iw >= 1354` | 拨轮从较小值跨到下阈值以上 |

## `sw2` 直接决定的全局模式

| `sw2` 数值 | `sw2` 位置 | 全局模式 | 中文意思 |
| --- | --- | --- | --- |
| `1` | `UP` | `MODE_GLOBAL_MANUAL_CTRL` | 手动控制模式，主要用于底盘手动与部分机构动作 |
| `2` | `DN` | `MODE_GLOBAL_RELEASE_CTRL` | 释放/安全模式，下游任务应进入清输出或安全态 |
| `3` | `MI` | `MODE_GLOBAL_ENGINEER_CTRL` | 工程动作模式，用于取矿、兑换、一矿等动作链 |

## 当 `sw2 = 1` (`UP`) 时的组合含义

这时全局模式是 `MODE_GLOBAL_MANUAL_CTRL`。

### 模式切换表

| `sw2` | `sw1` | `iw` 事件 | 结果底盘模式 | 中文意思 |
| --- | --- | --- | --- | --- |
| `UP` | `UP` | `iw 上边沿` | `MODE_CHASSIS_PITCH3_TORQUE_COLLECTION` | 进入 `pitch3` 力矩采集相关模式 |
| `UP` | `MI` | `iw 上边沿` | `MODE_CHASSIS_SECONDARY_ORE` | 进入二矿动作模式 |
| `UP` | `DN` | `iw 上边沿` | `MODE_CHASSIS_CHECK` | 进入检查模式 |
| `UP` | 任意 | `iw 下边沿`，且当前在 `PITCH3_TORQUE_COLLECTION / URGENT_MEASURE / SECONDARY_ORE` | `MODE_CHASSIS_NORMAL` | 从保持型特殊模式退回普通底盘模式 |
| `UP` | 任意 | 无上述触发 | `MODE_CHASSIS_NORMAL` | 默认普通底盘模式 |

### 中文理解

当 `sw2 = up` 时，可以把 `sw1` 理解成“手动模式里的功能分组选择”，`iw` 理解成“触发一次进入/退出”的拨轮事件。

## 当 `sw2 = 3` (`MI`) 时的组合含义

这时全局模式是 `MODE_GLOBAL_ENGINEER_CTRL`。

### 模式切换表

| `sw2` | `sw1` | `iw` 事件 | 结果底盘模式 | 中文意思 |
| --- | --- | --- | --- | --- |
| `MI` | `UP` | `iw 上边沿` | `MODE_CHASSIS_GET_ENERGY_UNIT` | 进入取矿动作模式 0 |
| `MI` | `UP` | `iw 下边沿` | `MODE_CHASSIS_GET_ENERGY_UNIT1` | 进入取矿动作模式 1 |
| `MI` | `MI` | `iw 上边沿` | `MODE_CHASSIS_EXCHANGE` | 进入兑换动作模式 |
| `MI` | `MI` | `iw 下边沿` | `MODE_CHASSIS_GET_ENERGY_UNIT2` | 进入取矿动作模式 2 |
| `MI` | `DN` | `iw 上边沿` | `MODE_CHASSIS_PRIMARY` | 进入一矿动作模式 |

### 中文理解

当 `sw2 = mid` 时，可以把 `sw1` 理解成“工程模式里的动作大类选择”，`iw` 理解成“按边沿触发切到某个具体动作模式”。

## 当 `sw2 = 2` (`DN`) 时的含义

| `sw2` | 全局模式 | 底盘模式 | 中文意思 |
| --- | --- | --- | --- |
| `DN` | `MODE_GLOBAL_RELEASE_CTRL` | `MODE_CHASSIS_RELEASE` | 进入释放/安全态 |

这里不再通过 `sw1 + iw` 进入动作模式。

## 动作步进规则

进入某些模式后，`sw1` 不再只是选模式，而会继续被用来推进动作步骤。

### `clamp_action` 步进

`clamp_action` 只在下面这些模式里有效：

- `MODE_CHASSIS_GET_ENERGY_UNIT`
- `MODE_CHASSIS_GET_ENERGY_UNIT1`
- `MODE_CHASSIS_GET_ENERGY_UNIT2`
- `MODE_CHASSIS_PRIMARY`
- `MODE_CHASSIS_SECONDARY_ORE`

步进规则：

| 前提 | `sw1` 事件 | 结果 | 中文意思 |
| --- | --- | --- | --- |
| 先把 `sw1` 回到 `MI`，解锁动作步进 | `MI -> DN` | `clamp_action + 1`，最大到 `MODE_CLAMP_ACTION_TWO` | 动作前进一步 |
| 先把 `sw1` 回到 `MI`，解锁动作步进 | `MI -> UP` | `clamp_action - 1`，最小到 `MODE_CLAMP_UN_CMD` | 动作后退一步 |

注意：

- 当前代码里 `clamp_action` 最多推进到 `ACTION_TWO`
- 虽然某些 `arm_task` 分支里还有 `ACTION_THREE`，但 `mode_task` 现在不会把它推进到那里

### `exchange_action` 步进

`exchange_action` 只在 `MODE_CHASSIS_EXCHANGE` 中有效。

| 前提 | `sw1` 事件 | 结果 | 中文意思 |
| --- | --- | --- | --- |
| 先把 `sw1` 回到 `MI`，解锁兑换动作 | `MI -> DN` | `MODE_EXCHANGE_PICK_ACTION1` | 选择兑换动作 1 |
| 先把 `sw1` 回到 `MI`，解锁兑换动作 | `MI -> UP` | `MODE_EXCHANGE_PICK_ACTION2` | 选择兑换动作 2 |

### `primary_turn_ore_flag` 触发

`primary_turn_ore_flag` 只在 `MODE_CHASSIS_PRIMARY` 中有效。

| 前提 | `iw` 事件 | 结果 | 中文意思 |
| --- | --- | --- | --- |
| 当前底盘模式是 `MODE_CHASSIS_PRIMARY` | `iw 下边沿` | `primary_turn_ore_flag = 1` | 触发一次“转矿”标志 |
| 离开 `MODE_CHASSIS_PRIMARY` | 无需额外操作 | `primary_turn_ore_flag = 0` | 自动清零 |

## 最常用的中文速查

| 组合 | 结果 | 中文意思 |
| --- | --- | --- |
| `sw2 = 1 (UP)` | `MANUAL_CTRL` | 手动模式 |
| `sw2 = 2 (DN)` | `RELEASE_CTRL` | 释放/安全模式 |
| `sw2 = 3 (MI)` | `ENGINEER_CTRL` | 工程动作模式 |
| `sw2 = 1, sw1 = 1, iw 上边沿` | `PITCH3_TORQUE_COLLECTION` | 进入 `pitch3` 力矩采集 |
| `sw2 = 1, sw1 = 3, iw 上边沿` | `SECONDARY_ORE` | 进入二矿模式 |
| `sw2 = 1, sw1 = 2, iw 上边沿` | `CHECK` | 进入检查模式 |
| `sw2 = 3, sw1 = 1, iw 上边沿` | `GET_ENERGY_UNIT` | 进入取矿模式 0 |
| `sw2 = 3, sw1 = 1, iw 下边沿` | `GET_ENERGY_UNIT1` | 进入取矿模式 1 |
| `sw2 = 3, sw1 = 3, iw 上边沿` | `EXCHANGE` | 进入兑换模式 |
| `sw2 = 3, sw1 = 3, iw 下边沿` | `GET_ENERGY_UNIT2` | 进入取矿模式 2 |
| `sw2 = 3, sw1 = 2, iw 上边沿` | `PRIMARY` | 进入一矿模式 |

## 说明

1. 本文档描述的是 `mode_task` 写入共享池后的模式/动作语义，不是底层电机立刻执行的电流或位置命令。
2. 真正的底盘控制与机械臂控制，分别由 `chassis_task` 与 `arm_task` 消费这些共享状态去执行。
3. 如果后续修改了 `mode_task.c` 里的边沿条件、步进上限或模式名，这份表需要一起更新。
