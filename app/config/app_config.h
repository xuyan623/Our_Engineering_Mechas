#ifndef NEW_ROBOT_APP_CONFIG_H
#define NEW_ROBOT_APP_CONFIG_H

/* -------------------------------------------------------------------------- */
/* system_health 自测开关                                                     */
/* -------------------------------------------------------------------------- */

/* 关闭 system_health 自测注错。 */
#define APP_SYSTEM_HEALTH_SELF_TEST_MODE_OFF (0u)

/* 在烟测任务内停止心跳，验证 runtime timeout 报码。 */
#define APP_SYSTEM_HEALTH_SELF_TEST_MODE_RUNTIME_TIMEOUT (1u)

/* 在烟测任务内主动锁存 fatal，验证 fatal 报码与最终接管流程。 */
#define APP_SYSTEM_HEALTH_SELF_TEST_MODE_FATAL (2u)

/* 当前默认关闭；需要验证时改成上面的某一种模式。 */
#define APP_SYSTEM_HEALTH_SELF_TEST_MODE (APP_SYSTEM_HEALTH_SELF_TEST_MODE_OFF)

/* 进入注错前的等待时间，单位 ms。
 * 预留几秒给上电、串口观测和电机起振稳定。
 */
#define APP_SYSTEM_HEALTH_SELF_TEST_DELAY_MS (0u)

/* -------------------------------------------------------------------------- */
/* 电机自动恢复                                                               */
/* -------------------------------------------------------------------------- */

/* 编译期开关：
 * - 1：启用运行时自动恢复
 * - 0：关闭恢复模块，只保留启动期 bring-up
 */
#define APP_MOTOR_AUTO_RECOVERY_ENABLE (1u)

/* 所有电机统一使用的在线超时窗口，单位 ms。
 * 当前项目里达妙是经典 CAN 下的被动式一发一收，多电机同总线时
 * 反馈节拍会明显低于 DJI/P1010B。这里统一放宽到 500ms，避免
 * “反馈存在但 cadence 偏慢”时被恢复模块误判成离线。
 */
#define APP_MOTOR_RECOVERY_ONLINE_TIMEOUT_MS (500u)

/* 自动恢复重试间隔，单位 ms。 */
#define APP_MOTOR_RECOVERY_RETRY_INTERVAL_MS (50u)

/* 离线故障报码去抖时间，单位 ms。 */
#define APP_MOTOR_RECOVERY_FAULT_DEBOUNCE_MS (100u)

/* -------------------------------------------------------------------------- */
/* 基础数学常量                                                               */
/* -------------------------------------------------------------------------- */

/* 圆周率，供运动学与角度归一化计算使用。 */
#define APP_PI (3.1415926535f)

/* 角度与弧度换算系数：
 * 旧工程中的底盘旋转速度输入使用 deg/s，因此麦轮分解时沿用 57.3 的换算方式。
 */
#define APP_RADIAN_COEF (57.3f)

/* -------------------------------------------------------------------------- */
/* 底盘运动学参数                                                             */
/* -------------------------------------------------------------------------- */

/* 麦轮轮周长，单位 mm。 */
#define APP_CHASSIS_WHEEL_PERIMETER_MM (478.0f)

/* 左右轮中心距，单位 mm。 */
#define APP_CHASSIS_WHEEL_TRACK_MM (375.0f)

/* 前后轮中心距，单位 mm。 */
#define APP_CHASSIS_WHEEL_BASE_MM (365.0f)

/* 底盘轮电机减速比：
 * 3508 电机输出到轮端的等效比例为 1/19。
 */
#define APP_CHASSIS_DECEL_RATIO (1.0f / 19.0f)

/* 单轮允许的最大目标转速，单位 rpm。 */
#define APP_CHASSIS_MAX_WHEEL_RPM (8500.0f)

/* 底盘前后平移速度上限，单位 mm/s。 */
#define APP_CHASSIS_MAX_VX_MM_PER_S (3300.0f)

/* 底盘左右平移速度上限，单位 mm/s。 */
#define APP_CHASSIS_MAX_VY_MM_PER_S (3300.0f)

/* 底盘绕自身旋转角速度上限，单位 deg/s。 */
#define APP_CHASSIS_MAX_VW_DEG_PER_S (480.0f)

/* 遥控摇杆满量程分辨率。 */
#define APP_RC_RESOLUTION (660.0f)

/* 遥控旋转打满后的软限幅策略：
 * - 常规情况下旋转输入乘 0.5，降低操作灵敏度
 * - 持续打满超过该时间后解除缩放，保留旧工程的小陀螺体验
 */
#define APP_CHASSIS_RC_ROTATE_SOFTEN_SCALE (0.5f)
#define APP_CHASSIS_RC_ROTATE_SOFTEN_HOLD_MS (2000u)

/* 键鼠底盘控制速度参数，沿用旧工程当前实测值。 */
#define APP_CHASSIS_KB_MAX_SPEED_X_MM_PER_S (1000.0f)
#define APP_CHASSIS_KB_MAX_SPEED_Y_MM_PER_S (1000.0f)
#define APP_CHASSIS_KB_MAX_SPEED_R_DEG_PER_S (600.0f)
#define APP_CHASSIS_KB_MOVE_RATIO_X (1.0f)
#define APP_CHASSIS_KB_MOVE_RATIO_Y (0.6f)
#define APP_CHASSIS_KB_MOVE_RATIO_R (1.0f)
#define APP_CHASSIS_KB_MOUSE_ROTATE_SCALE (0.2f)

/* 腿部 pitch 命令在 chassis_task 内部按旧工程语义积分，单位 deg。 */
#define APP_CHASSIS_LEG_PIT_CMD_STEP_PER_TICK_DEG (0.0001f)
#define APP_CHASSIS_LEG_PIT_CMD_MIN_DEG (-1.82f)
#define APP_CHASSIS_LEG_PIT_CMD_MAX_DEG (38.85f)
#define APP_CHASSIS_LEFT_LEG_REF_BIAS_DEG (-3.4f)
#define APP_CHASSIS_RIGHT_LEG_REF_BIAS_DEG (3.4f)
#define APP_CHASSIS_LEFT_LEG_REF_MIN_DEG (-38.85f)
#define APP_CHASSIS_LEFT_LEG_REF_MAX_DEG (1.82f)
#define APP_CHASSIS_RIGHT_LEG_REF_MIN_DEG (-1.82f)
#define APP_CHASSIS_RIGHT_LEG_REF_MAX_DEG (38.85f)

/* 轮速 / 腿部双环 PID 参数。
 * 轮速环的 Ki 保持旧工程“每 tick 累积 0.05*error”的离散语义，
 * chassis_task 会按任务周期换算到 OMR PID 的秒制接口。
 */
#define APP_CHASSIS_WHEEL_SPEED_PID_KP (4.5f)
#define APP_CHASSIS_WHEEL_SPEED_PID_KI (0.05f)
#define APP_CHASSIS_WHEEL_SPEED_PID_KD (0.0f)
#define APP_CHASSIS_WHEEL_SPEED_PID_OUT_LIMIT (10000.0f)
#define APP_CHASSIS_WHEEL_SPEED_PID_INTEGRAL_LIMIT (2000.0f)

#define APP_CHASSIS_LEG_ANGLE_PID_KP (8.0f)
#define APP_CHASSIS_LEG_ANGLE_PID_KI (0.0f)
#define APP_CHASSIS_LEG_ANGLE_PID_KD (0.0f)
#define APP_CHASSIS_LEG_ANGLE_PID_OUT_LIMIT (960.0f)
#define APP_CHASSIS_LEG_ANGLE_PID_INTEGRAL_LIMIT (200.0f)

#define APP_CHASSIS_LEG_SPEED_PID_KP (35.0f)
#define APP_CHASSIS_LEG_SPEED_PID_KI (0.0f)
#define APP_CHASSIS_LEG_SPEED_PID_KD (0.0f)
#define APP_CHASSIS_LEG_SPEED_PID_OUT_LIMIT (2000.0f)
#define APP_CHASSIS_LEG_SPEED_PID_INTEGRAL_LIMIT (800.0f)

/* big_yaw 在 arm_task 落地前由 chassis_task 以低增益角度环做启动后保持。
 * arm_task 接管后应默认关闭，避免两个控制任务同时改写同一电机目标。
 */
#define BIG_YAW_TEMP_HOLD_ENABLE (0u)
#define APP_CHASSIS_BIG_YAW_HOLD_KP (3.0f)
#define APP_CHASSIS_BIG_YAW_HOLD_KD (0.12f)

/* -------------------------------------------------------------------------- */
/* 机械臂控制参数                                                             */
/* -------------------------------------------------------------------------- */

#define APP_ARM_BIG_YAW_KP (30.0f)
#define APP_ARM_BIG_YAW_KD (0.01f)
#define APP_ARM_BIG_YAW_MAX_RATE_RAD_PER_S (2.0f)

#define APP_ARM_PITCH1_KP (63.0f)
#define APP_ARM_PITCH1_KD (0.07f)
#define APP_ARM_PITCH1_MAX_RATE_RAD_PER_S (0.8f)
/* Pitch1 沿用旧工程的一对一负号映射：
 * - 姿态表里的机构角增量为正
 * - 下发到 DM10010L 的电机目标角必须取反
 *
 * 这里继续保留成显式比例，便于后续若现场方向再次变化时，只改配置不动控制逻辑。
 */
#define APP_ARM_PITCH1_TARGET_RATIO (-1.0f)

/* GO8010 走关节位置模式，沿用旧工程注释中已验证过的一组轻量参数。 */
#define APP_ARM_PITCH2_KP (1.0f)
#define APP_ARM_PITCH2_KD (0.06f)
#define APP_ARM_PITCH2_MAX_RATE_RAD_PER_S (3.0f)

#define APP_ARM_ROLL2_KP (7.0f)
#define APP_ARM_ROLL2_KD (0.01f)
#define APP_ARM_ROLL2_MAX_RATE_RAD_PER_S (2.0f)

#define APP_ARM_PITCH3_KP (20.0f)
#define APP_ARM_PITCH3_KD (0.01f)
#define APP_ARM_PITCH3_MAX_RATE_RAD_PER_S (2.0f)

#define APP_ARM_GRIP_KP (18.0f)
#define APP_ARM_GRIP_KD (0.10f)
#define APP_ARM_GRIP_MAX_RATE_RAD_PER_S (4.0f)

#define APP_ARM_PITCH1_GRAVITY_FF_MIN (-20.0f)
#define APP_ARM_PITCH1_GRAVITY_FF_MAX (20.0f)
#define APP_ARM_PITCH2_GRAVITY_FF_MIN (-1.42f)
#define APP_ARM_PITCH2_GRAVITY_FF_MAX (1.43f)

#define APP_ARM_ROLL3_ANGLE_PID_KP (5.0f)
#define APP_ARM_ROLL3_ANGLE_PID_KI (0.0f)
#define APP_ARM_ROLL3_ANGLE_PID_KD (0.0f)
#define APP_ARM_ROLL3_ANGLE_PID_OUT_LIMIT (100.0f)
#define APP_ARM_ROLL3_ANGLE_PID_INTEGRAL_LIMIT (10.0f)
#define APP_ARM_ROLL3_MAX_RATE_RAD_PER_S (4.0f)

#define APP_ARM_ROLL3_SPEED_PID_KP (35.0f)
#define APP_ARM_ROLL3_SPEED_PID_KI (0.0f)
#define APP_ARM_ROLL3_SPEED_PID_KD (0.0f)
#define APP_ARM_ROLL3_SPEED_PID_OUT_LIMIT (15000.0f)
#define APP_ARM_ROLL3_SPEED_PID_INTEGRAL_LIMIT (500.0f)

/* -------------------------------------------------------------------------- */
/* 机械臂逆运动学几何参数                                                     */
/* -------------------------------------------------------------------------- */

/* 机械臂基座到第一段参考点的偏移长度，单位 mm。 */
#define APP_ARM_LINK_D2_MM (78.0f)

/* 第一段主连杆长度，单位 mm。 */
#define APP_ARM_LINK_A1_MM (400.0f)

/* 第二段参考偏移长度，单位 mm。 */
#define APP_ARM_LINK_D3_MM (404.0f)

/* 末端短连杆等效长度，单位 mm。 */
#define APP_ARM_LINK_A2_MM (65.0f)

/* -------------------------------------------------------------------------- */
/* Pitch 关节零位偏置与限幅                                                   */
/* -------------------------------------------------------------------------- */

/* Pitch1 电机角度零位补偿，单位 rad。 */
#define APP_ARM_PITCH1_ZERO_OFFSET_RAD (0.2793f)

/* Pitch2 电机角度零位补偿，单位 rad。
 * 这里对应减速器输出侧角度，后续会在算法中乘以减速比得到电机参考角。
 */
#define APP_ARM_PITCH2_ZERO_OFFSET_RAD (1.85f)

/* Pitch1 允许的最小参考角，单位 rad。 */
#define APP_ARM_PITCH1_MIN_RAD (-2.5133f)

/* Pitch1 允许的最大参考角，单位 rad。 */
#define APP_ARM_PITCH1_MAX_RAD (0.0f)

/* Pitch2 允许的最小参考角，单位 rad。 */
#define APP_ARM_PITCH2_MIN_RAD (-2.0f)

/* Pitch2 允许的最大参考角，单位 rad。 */
#define APP_ARM_PITCH2_MAX_RAD (0.0f)

/* Pitch2 电机减速比。
 * 逆解先得到关节角，再乘以该比例换算为电机参考角。
 */
#define APP_ARM_PITCH2_GEAR_RATIO (6.33f)

/* -------------------------------------------------------------------------- */
/* 重力补偿模型参数                                                           */
/* -------------------------------------------------------------------------- */

/* 重力加速度，单位 m/s^2。 */
#define APP_GRAVITY_ACCELERATION (9.8f)

/* 机械臂各连杆的等效质量，单位 kg。 */
#define APP_GRAVITY_M2_KG (1.333f)
#define APP_GRAVITY_M3_KG (0.70583f)
#define APP_GRAVITY_M4_KG (0.59967f)
#define APP_GRAVITY_M6_KG (0.37708f)
#define APP_GRAVITY_M7_KG (0.631f)

/* 旧工程重力模型使用的关节零位偏置，单位 rad。 */
#define APP_GRAVITY_Q2_OFFSET_RAD (0.261799f)
#define APP_GRAVITY_Q3_OFFSET_RAD (0.383972f)
#define APP_GRAVITY_Q6_OFFSET_RAD (0.1f)

/* Pitch1 重力补偿使用的等效力臂与质心偏移，单位 m。 */
#define APP_GRAVITY_PITCH1_EQUIVALENT_LEVER_M (1.1728f)
#define APP_GRAVITY_PITCH1_RY2_M (-0.000476f)

/* Pitch2 重力补偿使用的等效参数，单位 m。 */
#define APP_GRAVITY_PITCH2_LM56_M (0.16f)
#define APP_GRAVITY_PITCH2_LM34_M (0.30228f)
#define APP_GRAVITY_PITCH2_D5_M (0.14f)
#define APP_GRAVITY_PITCH2_D4_M (0.16f)
#define APP_GRAVITY_PITCH2_RX3_M (0.004006f)
#define APP_GRAVITY_PITCH2_RX4_M (0.002649f)
#define APP_GRAVITY_PITCH2_RY4_M (0.003381f)

/* Pitch3 / Roll2 重力补偿使用的等效参数，单位 m。 */
#define APP_GRAVITY_PITCH3_RZ7_M (0.062165f)
#define APP_GRAVITY_PITCH3_RY6_M (0.05038f)
#define APP_GRAVITY_PITCH3_D7_M (0.057f)

#endif
