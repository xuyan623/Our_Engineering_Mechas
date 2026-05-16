# MATLAB 仿真参考

## 1. 目的

本文档用于记录 `new_robot_code` 当前正式链路中，与后续 MATLAB / Simulink 仿真直接相关的必要数据、控制语义、模式定义、执行器映射和已知缺口。

约束：

- 本文档只记录当前仓库里**已经存在且可验证**的信息。
- 不补写仓库中不存在的实物参数，不做猜测。
- 适用对象是当前 `new_robot_code` 正式链，不包含已经删除的临时隔离测试链。

---

## 2. 源码事实来源

后续 MATLAB 建模时，以下文件是当前正式链的事实源：

- `app/main.c`
- `app/config/app_config.h`
- `app/task/mode_task/mode_task.h`
- `app/task/mode_task/mode_task.c`
- `app/task/chassis_task/chassis_task.c`
- `app/task/arm_task/arm_task.c`
- `app/task/motor_communications_task/mct.c`
- `app/algorithm/kinematics/kinematics.c`
- `app/algorithm/gravity_comp/gravity_comp.c`
- `app/task/vofa_task/vofa_task.c`

---

## 3. 正式软件链路总览

当前正式启动顺序：

1. `bsp_register_all()`
2. `event_bus_init(&g_event_bus)`
3. `mpu_device_init(9.8f)`
4. `imu_task_start()`
5. `input_task_start(devices)`
6. `mode_task_start()`
7. `mct_start(devices)`
8. `chassis_task_start()`
9. `arm_task_start()`
10. `vofa_task_start(devices)`

对仿真最重要的 owner 边界：

- `mode_task`：只产出模式与动作状态。
- `chassis_task`：只计算底盘与后腿目标，不直接碰物理总线。
- `arm_task`：只计算机械臂目标与前馈，不直接碰物理总线。
- `motor_communications_task`：是唯一正式电机通信 owner，唯一调用：
  - `motor_transmit_all()`
  - `motor_receive_all()`

这意味着后续 MATLAB 仿真如果要对齐当前软件结构，建议至少分成三层：

- 上层模式层：`mode_task`
- 控制层：`chassis_task` / `arm_task`
- 执行器与通信层：`motor_communications_task + motor abstraction`

---

## 4. 任务周期与实时节拍

当前与仿真相关的正式周期：

- `chassis_task`：`6 ms`
- `arm_task`：`6 ms`
- `motor_communications_task` 主循环：`5 ms`
- `P1010B active_query` 周期：`10 ms`
- `vofa_task`：`10 ms`

这意味着：

- 底盘与机械臂控制计算是 `~166.7 Hz`
- 电机统一通信刷新是 `200 Hz`
- 腿部 `P1010B` 的 query 模式反馈是 `100 Hz`

如果 MATLAB 先做离散控制级仿真，推荐基础步长先从 `5 ms` 或 `1 ms` 开始。

---

## 5. 执行器清单与正式命名

### 5.1 电机总表

| 名称 | 作用 | Vendor / 型号 | 正式控制语义 | 备注 |
|---|---|---|---|---|
| `chassis_fr` | 底盘右前轮 | DJI C620 | `CURRENT` | 已安装 |
| `chassis_fl` | 底盘左前轮 | DJI C620 | `CURRENT` | 已安装 |
| `chassis_bl` | 底盘左后轮 | DJI C620 | `CURRENT` | 已安装 |
| `chassis_br` | 底盘右后轮 | DJI C620 | `CURRENT` | 已安装 |
| `roll3` | 机械臂滚转轴 3 | DJI GM6020 | `CURRENT`，本地双环 PID | 已安装 |
| `joint_leg_r` | 右后腿 | P1010B | `CURRENT`，query-mode 反馈 | 已安装 |
| `joint_leg_l` | 左后腿 | P1010B | `CURRENT`，query-mode 反馈 | 已安装 |
| `big_yaw` | 机械臂大 yaw | Damiao DM4340 | `ANGLE` | 已安装 |
| `pitch1` | 机械臂第一 pitch | Damiao DM10010L | `ANGLE` | 已安装 |
| `roll1` | 预留轴 | Damiao DM4340 | 已注册但 `installed = false` | 当前未装 |
| `roll2` | 机械臂滚转轴 2 | Damiao DM4310 | `ANGLE` | 已安装 |
| `grip` | 夹爪 | Damiao DM4310 | `ANGLE` | 已安装 |
| `pitch3` | 机械臂第三 pitch | Damiao DM4310 | `ANGLE` | 已安装 |
| `pitch2` | 机械臂第二 pitch | GO8010 | `ANGLE` | 已安装 |

### 5.2 正式 VOFA 通道顺序

当前 `vofa_task` 只上传 14 路电机角度反馈，顺序与 `motor` 注册顺序一致：

| 通道 | 电机 |
|---|---|
| `I0` | `chassis_fr` |
| `I1` | `chassis_fl` |
| `I2` | `chassis_bl` |
| `I3` | `chassis_br` |
| `I4` | `roll3` |
| `I5` | `joint_leg_r` |
| `I6` | `joint_leg_l` |
| `I7` | `big_yaw` |
| `I8` | `pitch1` |
| `I9` | `roll1` |
| `I10` | `roll2` |
| `I11` | `grip` |
| `I12` | `pitch3` |
| `I13` | `pitch2` |

---

## 6. 模式系统

### 6.1 全局模式

`sw2` 直接决定全局模式：

- `UP` -> `MODE_GLOBAL_MANUAL_CTRL`
- `MI` -> `MODE_GLOBAL_ENGINEER_CTRL`
- `DN` -> `MODE_GLOBAL_RELEASE_CTRL`

### 6.2 底盘模式枚举

当前 `ChassisMode`：

- `MODE_CHASSIS_RELEASE`
- `MODE_CHASSIS_NORMAL`
- `MODE_CHASSIS_RESCUE`
- `MODE_CHASSIS_SUPPLY`
- `MODE_CHASSIS_PITCH3_TORQUE_COLLECTION`
- `MODE_CHASSIS_URGENT_MEASURE`
- `MODE_CHASSIS_EXCHANGE`
- `MODE_CHASSIS_PRIMARY`
- `MODE_CHASSIS_GET_ENERGY_UNIT`
- `MODE_CHASSIS_GET_ENERGY_UNIT1`
- `MODE_CHASSIS_GET_ENERGY_UNIT2`
- `MODE_CHASSIS_CLAMP_CATCH`
- `MODE_CHASSIS_SECONDARY_ORE`
- `MODE_CHASSIS_STOP`
- `MODE_CHASSIS_DEFEND`
- `MODE_CHASSIS_CHECK`

### 6.3 工程模式下的机械臂入口

`sw2 = MI` 时：

- `sw1 = UP` + `iw` 上边沿 -> `MODE_CHASSIS_GET_ENERGY_UNIT`
- `sw1 = UP` + `iw` 下边沿 -> `MODE_CHASSIS_GET_ENERGY_UNIT1`
- `sw1 = MI` + `iw` 上边沿 -> `MODE_CHASSIS_EXCHANGE`
- `sw1 = MI` + `iw` 下边沿 -> `MODE_CHASSIS_GET_ENERGY_UNIT2`
- `sw1 = DN` + `iw` 上边沿 -> `MODE_CHASSIS_PRIMARY`

### 6.4 夹取动作状态

枚举定义：

- `MODE_CLAMP_UN_CMD`
- `MODE_CLAMP_ACTION_ONE`
- `MODE_CLAMP_ACTION_TWO`
- `MODE_CLAMP_ACTION_THREE`

当前代码中的一个重要事实：

- `mode_task` 现在只会把 `clamp_action` 推进到 `ACTION_TWO`
- 虽然 `ACTION_THREE` 枚举和 `arm_task` 分支仍然存在，但**当前遥控路径下不可达**

因此做 MATLAB 仿真时，如果目标是“复现当前正式链”，应先把 `ACTION_THREE` 当作保留分支，而不是默认可被遥控触发。

### 6.5 兑换动作状态

- `MODE_EXCHANGE_UN_CMD`
- `MODE_EXCHANGE_PICK_ACTION1`
- `MODE_EXCHANGE_PICK_ACTION2`

---

## 7. 底盘模型数据

### 7.1 麦轮运动学参数

当前底盘参数：

- 轮周长：`478.0 mm`
- 左右轮距：`375.0 mm`
- 前后轴距：`365.0 mm`
- 轮电机到轮端减速比：`1 / 19`
- 最大单轮目标转速：`8500 rpm`
- 最大底盘前后速度：`3300 mm/s`
- 最大底盘左右速度：`3300 mm/s`
- 最大底盘角速度：`480 deg/s`

### 7.2 麦轮速度分配公式

当前实现位于 `app/algorithm/kinematics/kinematics.c` 的 `mecanum_calc(...)`。

定义：

- `rotate_ratio = ((wheel_base + wheel_track) / 2) / APP_RADIAN_COEF`
- `wheel_rpm_ratio = 60 / (wheel_perimeter * decel_ratio)`

四轮速度公式：

```text
front_right = (-vx - vy - vw * rotate_ratio) * wheel_rpm_ratio
front_left  = ( vx - vy - vw * rotate_ratio) * wheel_rpm_ratio
back_left   = ( vx + vy - vw * rotate_ratio) * wheel_rpm_ratio
back_right  = (-vx + vy - vw * rotate_ratio) * wheel_rpm_ratio
```

并带统一最大轮速缩放。

### 7.3 遥控与键鼠速度参数

- RC 满量程：`660`
- RC 旋转软限幅缩放：`0.5`
- RC 旋转持续打满解除时间：`2000 ms`
- 键鼠最大速度：
  - `X`: `1000 mm/s`
  - `Y`: `1000 mm/s`
  - `R`: `600 deg/s`
- 键鼠比例：
  - `X`: `1.0`
  - `Y`: `0.6`
  - `R`: `1.0`
- 鼠标旋转缩放：`0.2`

---

## 8. 后腿模型数据

### 8.1 腿部参考角生成

当前 `chassis_task` 内部用 `ch4` 积分生成 `pit_leg_cmd_deg`：

```text
pit_leg_cmd_deg += ch4 * 0.0001 deg/tick
pit_leg_cmd_deg clamp to [-1.82, 38.85] deg
```

左右腿参考角：

```text
left_leg_ref_deg  = clamp(-3.4 - pit_leg_cmd_deg, [-38.85, 1.82])
right_leg_ref_deg = clamp( 3.4 + pit_leg_cmd_deg, [-1.82, 38.85])
```

### 8.2 腿部控制结构

两条后腿当前都走 `CURRENT` 控制模式，`chassis_task` 内部做本地双环：

- 角度环 PID
- 速度环 PID

PID 参数：

#### 角度环

- `Kp = 8.0`
- `Ki = 0.0`
- `Kd = 0.0`
- `OutLimit = 960.0`
- `IntegralLimit = 200.0`

#### 速度环

- `Kp = 35.0`
- `Ki = 0.0`
- `Kd = 0.0`
- `OutLimit = 2000.0`
- `IntegralLimit = 800.0`

### 8.3 P1010B 正式反馈路径

两台 `P1010B` 当前使用 query-mode：

- `activeReport.enable = false`
- `motor_communications_task` 每 `10 ms` 轮询一台
- query 项：
  - absolute position
  - speed rpm
  - iq ampere
  - bus voltage

所以在 MATLAB 中，如果只复现当前控制环路，可以先把腿部反馈视为 `100 Hz`。

---

## 9. 机械臂模型数据

### 9.1 轴顺序

机械臂姿态表当前统一按如下机构轴顺序存储：

1. `big_yaw`
2. `pitch1`
3. `pitch2`
4. `roll2`
5. `pitch3`
6. `roll3`
7. `grip`

其中：

- `big_yaw / pitch1 / pitch2 / roll2 / pitch3 / grip` 单位为 `rad`
- `roll3` 单位为 `deg`

### 9.2 姿态语义

当前 `arm_task` 已恢复成旧工程的：

```text
final_machine_pose = normal_pose + mode_delta_pose
```

不是“姿态表即最终绝对目标”。

### 9.3 常驻基础姿态 `normal`

```text
big_yaw = 0
pitch1  = 0
pitch2  = 0
roll2   = 0
pitch3  = 0.1
roll3   = 148 deg
grip    = 1.8
```

### 9.4 主要模式增量姿态表

#### `GET_ENERGY_UNIT`

```text
big_yaw = 0
pitch1  = 1.24218
pitch2  = 1.19447
roll2   = 0
pitch3  = 0
roll3   = 0
grip    = -1.8
```

#### `GET_ENERGY_UNIT1`

```text
big_yaw = -0.00667
pitch1  = 1.035
pitch2  = (5.53 / 6.33) + 0.34 + 0.1
roll2   = 0.6178
pitch3  = -0.194
roll3   = -90.39 deg
grip    = -1.8
```

#### `GET_ENERGY_UNIT2`

```text
big_yaw = 0.148584366
pitch1  = 0.99088
pitch2  = 1.04010 + 0.1
roll2   = 0.093270302
pitch3  = 0.07834
roll3   = -54.396 deg
grip    = -1.8
```

#### `EXCHANGE`

```text
big_yaw = 0
pitch1  = 0.64218
pitch2  = 1.0447
roll2   = 0
pitch3  = 0
roll3   = 0
grip    = 0
```

#### `PRIMARY`

```text
big_yaw = 0
pitch1  = 1.46691
pitch2  = 2.0053
roll2   = 0.1192
pitch3  = -1.6
roll3   = 180 deg
grip    = 0
```

#### `SECONDARY_ORE`

```text
big_yaw = 0
pitch1  = 1.48691
pitch2  = 0.85
roll2   = -1.57
pitch3  = 0
roll3   = 0
grip    = 0
```

### 9.5 机械臂模式动作时序

#### `GET_ENERGY_UNIT`

- `UN_CMD`：保持 `get_energy`
- `ACTION_ONE`：`grip = 0`
- `ACTION_TWO`：
  - `pitch3 = 0.43`
  - `pitch2 = -0.43 + 1.19447`
- `ACTION_THREE`：
  - 切换到 `store_energy`
  - `elapsed >= 800 ms` 后 `grip = -1.8`

#### `GET_ENERGY_UNIT1`

- `ACTION_ONE`：`grip = 0`

#### `GET_ENERGY_UNIT2`

- `ACTION_ONE`：`grip = 0`
- `ACTION_TWO`：
  - 切换到 `store_energy1`
  - `elapsed >= 1150 ms` 后 `grip = -1.8`

#### `EXCHANGE`

兑换动作不读 `clamp_action`，只读 `exchange_action`。

`PICK_ACTION1` 时间窗：

- `>= 100 ms` -> `store_energy`
- `>= 1200 ms` -> `exchange_pick`
- `>= 1400 ms` -> `pitch2 = 1.04067`, `roll2 = 0.05`
- `>= 1550 ms` -> `pitch2 = 0.82`, `pitch3 = -1.0`, `roll2 = -0.04512`, `roll3 = -49.3427`
- `>= 1800 ms` -> `grip = 0`
- `>= 2000 ms` -> `pitch2 = 1.3`

`PICK_ACTION2` 时间窗：

- `>= 100 ms` -> `store_energy1`
- `>= 1200 ms` -> `exchange_pick1`
- `>= 1300 ms` -> `pitch2 = 0.87`, `pitch3 = -1.19`
- `>= 1460 ms` -> `big_yaw = -2.00189481`, `pitch2 = 0.810012383`, `pitch3 = -1.11`, `pitch1 = 1.20`
- `>= 2170 ms` -> `grip = 0`
- `>= 2390 ms` -> `pitch2 = 1.6`, `pitch3 = -1.45`

### 9.6 机械臂电机目标映射

当前 `arm_task` 的机构角到电机目标角映射：

```text
big_yaw_target = final_big_yaw
pitch1_target  = APP_ARM_PITCH1_TARGET_RATIO * final_pitch1
pitch2_target  = pitch2_zero_angle + final_pitch2_joint * (-APP_ARM_PITCH2_GEAR_RATIO)
roll2_target   = final_roll2
pitch3_target  = final_pitch3
roll3_target   = deg2rad(final_roll3_deg)
grip_target    = final_grip
```

其中：

- `APP_ARM_PITCH1_TARGET_RATIO = -1.0`
- `APP_ARM_PITCH2_GEAR_RATIO = 6.33`
- `pitch2_zero_angle` 由 GO8010 首次在线反馈捕获

### 9.7 机械臂控制模式

当前正式链：

- `big_yaw`：`ANGLE`
- `pitch1`：`ANGLE`
- `pitch2`：`ANGLE`
- `roll2`：`ANGLE`
- `pitch3`：`ANGLE`
- `grip`：`ANGLE`
- `roll3`：`CURRENT`，本地双环 PID

### 9.8 机械臂各轴参数

#### 角度环 / 速率限制

| 轴 | Kp | Kd | 最大目标变化速率 |
|---|---:|---:|---:|
| `big_yaw` | 30.0 | 0.01 | 2.0 rad/s |
| `pitch1` | 63.0 | 0.07 | 0.8 rad/s |
| `pitch2` | 1.0 | 0.06 | 3.0 rad/s |
| `roll2` | 7.0 | 0.01 | 2.0 rad/s |
| `pitch3` | 20.0 | 0.01 | 2.0 rad/s |
| `grip` | 18.0 | 0.10 | 4.0 rad/s |
| `roll3` | 本地双环 | 本地双环 | 4.0 rad/s |

#### `roll3` 双环 PID

角度环：

- `Kp = 5.0`
- `Ki = 0.0`
- `Kd = 0.0`
- `OutLimit = 100.0`
- `IntegralLimit = 10.0`

速度环：

- `Kp = 35.0`
- `Ki = 0.0`
- `Kd = 0.0`
- `OutLimit = 15000.0`
- `IntegralLimit = 500.0`

### 9.9 `pitch2` 零位与限幅

- `APP_ARM_PITCH1_ZERO_OFFSET_RAD = 0.2793`
- `APP_ARM_PITCH2_ZERO_OFFSET_RAD = 1.85`
- `APP_ARM_PITCH1_MIN_RAD = -2.5133`
- `APP_ARM_PITCH1_MAX_RAD = 0.0`
- `APP_ARM_PITCH2_MIN_RAD = -2.0`
- `APP_ARM_PITCH2_MAX_RAD = 0.0`

注意：

- 这组零位偏置主要用于 `Change_Position_to_Motor_Angle(...)`
- 当前正式 `arm_task` 主链是**姿态表驱动**
- 在线逆解函数目前仅存在于 `kinematics.c`，当前正式 `arm_task` 未直接调用

---

## 10. 逆运动学与几何参数

### 10.1 几何参数

- `D2 = 78 mm`
- `A1 = 400 mm`
- `D3 = 404 mm`
- `A2 = 65 mm`

### 10.2 当前仓库中的逆解函数

函数：`Change_Position_to_Motor_Angle(float x_mm, float z_mm, ...)`

输入：

- `x_mm`
- `z_mm`

输出：

- `pitch1_motor_angle_rad`
- `pitch2_motor_angle_rad`

求解过程：

```text
a = -2*A2*z - 2*D3*x
b = -2*A2*x + 2*D3*z
c = A1^2 - (z^2 + x^2 + A2^2 + D3^2)
den = sqrt(a^2 + b^2)
asin_input = clamp(c / den, -1, 1)
phi = atan2(b, a)
elbow = asin(asin_input) - phi
pitch1 = atan2(z - A2*sin(elbow) + D3*cos(elbow),
               x - D3*sin(elbow) - A2*cos(elbow))
pitch2 = elbow - pitch1
pitch1 = -normalize(pitch1) + pitch1_zero_offset
pitch2 =  normalize(pitch2) + pitch2_zero_offset
pitch1 = clamp(pitch1, [pitch1_min, pitch1_max])
pitch2 = clamp(pitch2, [pitch2_min, pitch2_max])
pitch2_motor = pitch2 * gear_ratio
```

重要事实：

- 当前仓库保留了逆解函数
- 但当前正式 `arm_task` 没有在线使用该函数
- 当前正式机械臂行为由“模式 -> 姿态表 -> 电机映射”驱动

因此后续 MATLAB 若想先复现当前软件行为，应优先复现姿态表控制；若要做末端位姿控制，可再单独接这套逆解。

---

## 11. 重力补偿模型

### 11.1 质量与偏移参数

- 重力加速度：`9.8 m/s^2`
- `M2 = 1.333 kg`
- `M3 = 0.70583 kg`
- `M4 = 0.59967 kg`
- `M6 = 0.37708 kg`
- `M7 = 0.631 kg`

角度偏置：

- `Q2_OFFSET = 0.261799 rad`
- `Q3_OFFSET = 0.383972 rad`
- `Q6_OFFSET = 0.1 rad`

Pitch1 等效参数：

- `PITCH1_EQUIVALENT_LEVER = 1.1728 m`
- `PITCH1_RY2 = -0.000476 m`

Pitch2 / Pitch3 / Roll2 模型参数：

- `LM56 = 0.16 m`
- `LM34 = 0.30228 m`
- `D5 = 0.14 m`
- `D4 = 0.16 m`
- `RX3 = 0.004006 m`
- `RX4 = 0.002649 m`
- `RY4 = 0.003381 m`
- `RZ7 = 0.062165 m`
- `RY6 = 0.05038 m`
- `D7 = 0.057 m`

### 11.2 当前代码中的角度解析

用于 `pitch1` / `pitch2` / `roll2` 链的角：

```text
q2 = Q2_OFFSET - pitch1_motor_angle
q3 = (pitch2_motor_angle - pitch2_zero_angle) / gear_ratio + Q3_OFFSET
q4 = -roll2_motor_angle
q6 = pitch3_motor_angle
```

用于 `pitch3` 补偿的角：

```text
q2 = Q2_OFFSET + pitch1_motor_angle
q3 = (-pitch2_motor_angle + pitch2_zero_angle) / gear_ratio - Q3_OFFSET
q4 = -roll2_motor_angle
q6 = pitch3_motor_angle - Q6_OFFSET
```

### 11.3 当前代码中的补偿函数

`pitch2_grav_torque_calculate(...)`：

```text
torque = g / -gear_ratio *
(
  cos(q2+q3) * (LM34 + LM56*cos(q6) + M6*(D5+D4) + M7*(D5+D4))
  +
  sin(q2+q3) * (-M3*RX3 + M4*(RX4*cos(q4) + RY4*sin(q4)) + LM56*cos(q4)*sin(q6))
)
```

`pitch3_grav_torque_calcuate(...)`：

```text
torque = g * LM56 * (cos(q6)*cos(q2+q3)*cos(q4) - sin(q6)*sin(q2+q3))
```

`roll2_grav_torque_calculate(...)`：

```text
torque = g * cos(q2+q3) *
(
  sin(q4) * (M6*RY6 + M7*(D7 + RZ7)*sin(q6))
)
```

`pitch1_grav_torque_calcuate(...)`：

```text
local_torque =
(
  PITCH1_EQUIVALENT_LEVER * cos(q2)
  - M2 * PITCH1_RY2 * sin(q2)
) * g

pitch1_torque = -(pitch2_torque * gear_ratio + local_torque)
```

### 11.4 当前正式链中前馈输出限幅

- `pitch1`：`[-20.0, 20.0]`
- `pitch2`：`[-1.42, 1.43]`

说明：

- `roll2` 与 `pitch3` 当前也有重力补偿计算
- 当前 `arm_task` 每个周期都会重新基于反馈计算这些前馈

---

## 12. 电机通信与 HIL 相关数据

### 12.1 Damiao 正式配置

当前 6 台 Damiao 的正式配置：

| 名称 | 型号 | `can_id` | `master_id` | `installed` |
|---|---|---:|---:|---|
| `big_yaw` | DM4340 | `0x00` | `0x10` | `true` |
| `pitch1` | DM10010L | `0x01` | `0x11` | `true` |
| `roll1` | DM4340 | `0x02` | `0x12` | `false` |
| `roll2` | DM4310 | `0x03` | `0x13` | `true` |
| `grip` | DM4310 | `0x04` | `0x14` | `true` |
| `pitch3` | DM4310 | `0x05` | `0x15` | `true` |

启动期 bring-up：

1. 写 `CTRL_MODE (RID 10) = MIT (1)`
2. 等待 `10 ms`
3. `enable`

这一步现在已经是正式链固定行为。

### 12.2 P1010B 正式配置

两台：

- `joint_leg_r` -> `id = 1`
- `joint_leg_l` -> `id = 2`

当前正式链：

- 采用 query-mode
- `activeReport.enable = false`
- 每 `10 ms` 轮询一台

### 12.3 GO8010 正式配置

- `pitch2`
- bus owner 是 `motor_communications_task`
- 当前注释语义：`GO8010` 通过 `USART6` 直接挂正式通信任务，不走 `serial_dispatch`

---

## 13. MATLAB 建模建议分层

这是基于当前软件架构给出的建模建议，不是仓库事实：

### 第 1 层：控制离散逻辑仿真

先只复现：

- `mode_task` 模式切换
- `chassis_task` 速度分配与腿部参考角
- `arm_task` 姿态表、动作时序、目标斜坡、重力补偿

此层不先引入完整电机电流环和通信延迟。

### 第 2 层：关节级执行器仿真

对每个电机建立简化的：

- 目标角 / 目标电流 -> 一阶或二阶响应
- 限速
- 饱和
- 前馈叠加

优先验证模式动作是否与现车一致。

### 第 3 层：几何与动力学增强

再逐步补：

- 机械臂末端工作空间
- 腿部动态
- 轮地接触
- 更完整的多体动力学

---

## 14. 当前仓库里没有、但高可信仿真仍需要补的实物数据

以下数据**当前仓库未完整提供**，若要做高保真 MATLAB / Simscape / Simulink 仿真，必须后续补实测或机械设计数据：

### 14.1 底盘本体

- 整机质量
- 整机转动惯量
- 质心位置
- 轮地摩擦参数
- 麦轮滑移参数

### 14.2 后腿机构

- 两条腿的真实机械结构尺寸
- 连杆/转轴惯量
- 电机到关节的真实传动参数
- 关节阻尼与摩擦

### 14.3 机械臂本体

- 各连杆完整惯量张量
- 各质心的完整三维坐标
- `roll3` 真实电机 / 负载等效惯量
- 夹爪端部负载变化范围

### 14.4 执行器本体

- DJI / P1010B / Damiao / GO8010 的真实电机常数
- 电流环带宽
- 转矩常数
- 内部限流 / 限速 / 保护阈值

### 14.5 传感器与延迟

- IMU 噪声模型
- 编码器噪声与量化
- 通信延迟抖动
- 反馈丢帧概率

---

## 15. 后续 MATLAB 建模时建议优先照搬的内容

优先直接照搬，不要二次“脑补”的内容：

1. `mode_task` 的模式切换规则
2. `chassis_task` 的麦轮公式与腿部参考角生成
3. `arm_task` 的 `normal + mode_delta` 姿态语义
4. `arm_task` 的动作时间窗
5. `pitch1` 的负号映射
6. `pitch2` 的 `-6.33` 电机映射
7. 当前重力补偿公式与参数
8. 当前各轴 `Kp / Kd / 限速`

这些是当前正式链行为的核心。

---

## 16. 建模时的已知坑

1. `roll1` 当前未安装，但仍占电机命名和 VOFA 通道位。
2. `ACTION_THREE` 仍存在于 `arm_task`，但当前遥控路径实际不会推进到它。
3. `pitch2` 的零位不是固定常量，而是**首个在线反馈时抓取**。
4. 当前正式机械臂主链不是在线逆解，而是姿态表驱动。
5. `roll3` 不是统一角度环，而是单独的双环 `CURRENT` 控制。
6. `BIG_YAW_TEMP_HOLD_ENABLE = 0`，说明当前正式 `big_yaw` owner 是 `arm_task`，不是 `chassis_task`。

---

## 17. 文档维护建议

后续只要以下任一内容变化，都应同步更新本文档：

- `app_config.h` 中的控制参数、几何参数、重力补偿参数
- `mode_task` 的模式切换规则
- `arm_task` 的姿态表或动作时间窗
- `motor_communications_task` 的正式电机配置
- `vofa_task` 的通道顺序
