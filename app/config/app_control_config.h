#ifndef NEW_ROBOT_APP_CONTROL_CONFIG_H
#define NEW_ROBOT_APP_CONTROL_CONFIG_H

/* app_control_config.h 只放“正式控制会直接消费”的参数：
 * - 任务/通信节拍
 * - PID 参数
 * - 运动学/几何参数
 * - 力矩前馈/限幅/速率参数
 *
 * 调参原则：
 * - 想改 PID、限幅、速率、几何参数，改这里
 * - 不要把装配启用状态或 VOFA 布局塞回这个文件
 */

/* -------------------------------------------------------------------------- */
/* 任务与通信节拍                                                             */
/* -------------------------------------------------------------------------- */

#define APP_MCT_LOOP_PERIOD_MS                             (5u)
#define APP_MCT_OPERATIONAL_FORMAL_TRANSMIT_PERIOD_MS      (10u)
#define APP_MCT_OPERATIONAL_OBSERVATION_PERIOD_MS          (50u)
#define APP_MCT_NON_OPERATIONAL_PERIOD_MS                  (50u)
#define APP_MCT_NON_OPERATIONAL_P1010B_OBSERVE_PERIOD_MS   (50u)

#define APP_CHASSIS_TASK_PERIOD_MS                         (6u)
#define APP_CHASSIS_TX_REQUEST_PERIOD_MS                   (6u)

#define APP_ARM_TASK_PERIOD_MS                             (3u)
#define APP_ARM_BIG_YAW_CONTROL_PERIOD_MS                  (10u)
#define APP_ARM_PITCH1_CONTROL_PERIOD_MS                   (10u)
#define APP_ARM_PITCH2_CONTROL_PERIOD_MS                   (3u)
#define APP_ARM_ROLL2_CONTROL_PERIOD_MS                    (10u)
#define APP_ARM_PITCH3_CONTROL_PERIOD_MS                   (10u)
#define APP_ARM_ROLL3_CONTROL_PERIOD_MS                    (10u)
#define APP_ARM_GRIP_CONTROL_PERIOD_MS                     (10u)
#define APP_ARM_TX_REQUEST_PERIOD_MS                       (0u)

/* -------------------------------------------------------------------------- */
/* 基础数学常量                                                               */
/* -------------------------------------------------------------------------- */

#define APP_PI (3.1415926535f)
#define APP_RADIAN_COEF (57.3f)

/* -------------------------------------------------------------------------- */
/* 底盘运动学参数                                                             */
/* -------------------------------------------------------------------------- */

#define APP_CHASSIS_WHEEL_PERIMETER_MM (478.0f)
#define APP_CHASSIS_WHEEL_TRACK_MM (375.0f)
#define APP_CHASSIS_WHEEL_BASE_MM (365.0f)
#define APP_CHASSIS_DECEL_RATIO (1.0f / 19.0f)
#define APP_CHASSIS_MAX_WHEEL_RPM (8500.0f)
#define APP_CHASSIS_MAX_VX_MM_PER_S (3300.0f)
#define APP_CHASSIS_MAX_VY_MM_PER_S (3300.0f)
#define APP_CHASSIS_MAX_VW_DEG_PER_S (480.0f)
#define APP_CHASSIS_MAX_TOTAL_SPEED_MM_PER_S (1000.0f)
#define APP_CHASSIS_TOTAL_SPEED_LIMIT_ENABLE (1u)
#define APP_RC_RESOLUTION (660.0f)
#define APP_CHASSIS_RC_ROTATE_SOFTEN_SCALE (0.5f)
#define APP_CHASSIS_RC_ROTATE_SOFTEN_HOLD_MS (2000u)
#define APP_CHASSIS_KB_MAX_SPEED_X_MM_PER_S (1000.0f)
#define APP_CHASSIS_KB_MAX_SPEED_Y_MM_PER_S (1000.0f)
#define APP_CHASSIS_KB_MAX_SPEED_R_DEG_PER_S (600.0f)
#define APP_CHASSIS_KB_MOVE_RATIO_X (1.0f)
#define APP_CHASSIS_KB_MOVE_RATIO_Y (0.6f)
#define APP_CHASSIS_KB_MOVE_RATIO_R (1.0f)
#define APP_CHASSIS_KB_MOUSE_ROTATE_SCALE (0.2f)
#define APP_CHASSIS_LEG_PIT_CMD_STEP_PER_TICK_DEG (0.0001f)
#define APP_CHASSIS_LEG_PIT_CMD_MIN_DEG (-1.82f)
#define APP_CHASSIS_LEG_PIT_CMD_MAX_DEG (38.85f)
#define APP_CHASSIS_LEFT_LEG_REF_BIAS_DEG (-3.4f)
#define APP_CHASSIS_RIGHT_LEG_REF_BIAS_DEG (3.4f)
#define APP_CHASSIS_LEFT_LEG_REF_MIN_DEG (-38.85f)
#define APP_CHASSIS_LEFT_LEG_REF_MAX_DEG (1.82f)
#define APP_CHASSIS_RIGHT_LEG_REF_MIN_DEG (-1.82f)
#define APP_CHASSIS_RIGHT_LEG_REF_MAX_DEG (38.85f)

/* -------------------------------------------------------------------------- */
/* 底盘 PID 参数                                                              */
/* -------------------------------------------------------------------------- */

#define APP_CHASSIS_FRONT_WHEEL_SPEED_PID_KP (4.5f)
#define APP_CHASSIS_FRONT_WHEEL_SPEED_PID_KI (0.00f)
#define APP_CHASSIS_FRONT_WHEEL_SPEED_PID_KD (0.0f)
#define APP_CHASSIS_FRONT_WHEEL_SPEED_PID_OUT_LIMIT (10000.0f)
#define APP_CHASSIS_FRONT_WHEEL_SPEED_PID_INTEGRAL_LIMIT (2000.0f)

#define APP_CHASSIS_REAR_WHEEL_SPEED_PID_KP (3.0f)
#define APP_CHASSIS_REAR_WHEEL_SPEED_PID_KI (0.0f)
#define APP_CHASSIS_REAR_WHEEL_SPEED_PID_KD (0.0f)
#define APP_CHASSIS_REAR_WHEEL_SPEED_PID_OUT_LIMIT (10000.0f)
#define APP_CHASSIS_REAR_WHEEL_SPEED_PID_INTEGRAL_LIMIT (2000.0f)

/* 后腿采用单环 PID（与旧工程对齐）：角度误差直接出电流。 */
//#define APP_CHASSIS_LEG_ANGLE_PID_KP (210.0f)
#define APP_CHASSIS_LEG_ANGLE_PID_KP (0.0f)
#define APP_CHASSIS_LEG_ANGLE_PID_KI (0.0f)
//#define APP_CHASSIS_LEG_ANGLE_PID_KD (6.0f)
#define APP_CHASSIS_LEG_ANGLE_PID_KD (0.0f)
#define APP_CHASSIS_LEG_ANGLE_PID_OUT_LIMIT (3600.0f)
#define APP_CHASSIS_LEG_ANGLE_PID_INTEGRAL_LIMIT (200.0f)

/* -------------------------------------------------------------------------- */
/* 机械臂控制参数                                                             */
/* -------------------------------------------------------------------------- */

#define APP_ARM_BIG_YAW_KP (25.0f)
#define APP_ARM_BIG_YAW_KD (0.01f)
#define APP_ARM_BIG_YAW_MAX_RATE_RAD_PER_S (2.0f)

#define APP_ARM_PITCH1_KP (63.0f)
#define APP_ARM_PITCH1_KD (0.07f)
#define APP_ARM_PITCH1_MAX_RATE_RAD_PER_S (0.8f)
#define APP_ARM_PITCH1_TARGET_RATIO (-1.0f)

#define APP_ARM_PITCH2_KP (1.0f)
#define APP_ARM_PITCH2_KD (0.06f)
#define APP_ARM_PITCH2_MAX_RATE_RAD_PER_S (3.0f)

#define APP_ARM_CUSTOM_CONTROLLER_ALIGNMENT_TIMEOUT_MS (3000u)

#define APP_ARM_ROLL2_KP (7.0f)
#define APP_ARM_ROLL2_KD (0.01f)
#define APP_ARM_ROLL2_MAX_RATE_RAD_PER_S (2.0f)

#define APP_ARM_PITCH3_KP (10.0f)
#define APP_ARM_PITCH3_KD (0.02f)
#define APP_ARM_PITCH3_MAX_RATE_RAD_PER_S (2.0f)
#define APP_ARM_PITCH3_ENABLE_GRAVITY_FF (1u)

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

#define APP_ARM_ROLL3_SPEED_PID_KP (40.0f)
#define APP_ARM_ROLL3_SPEED_PID_KI (0.0f)
#define APP_ARM_ROLL3_SPEED_PID_KD (0.0f)
#define APP_ARM_ROLL3_SPEED_PID_OUT_LIMIT (15000.0f)
#define APP_ARM_ROLL3_SPEED_PID_INTEGRAL_LIMIT (500.0f)

/* -------------------------------------------------------------------------- */
/* 机械臂逆运动学几何参数                                                     */
/* -------------------------------------------------------------------------- */

#define APP_ARM_LINK_D2_MM (78.0f)
#define APP_ARM_LINK_A1_MM (400.0f)
#define APP_ARM_LINK_D3_MM (404.0f)
#define APP_ARM_LINK_A2_MM (65.0f)

/* -------------------------------------------------------------------------- */
/* Pitch 关节零位偏置与限幅                                                   */
/* -------------------------------------------------------------------------- */

#define APP_ARM_PITCH1_ZERO_OFFSET_RAD (0.2793f)
#define APP_ARM_PITCH2_ZERO_OFFSET_RAD (1.85f)
#define APP_ARM_PITCH1_MIN_RAD (-2.5133f)
#define APP_ARM_PITCH1_MAX_RAD (0.0f)
#define APP_ARM_PITCH2_MIN_RAD (-2.0f)
#define APP_ARM_PITCH2_MAX_RAD (0.0f)
#define APP_ARM_PITCH2_GEAR_RATIO (6.33f)

/* -------------------------------------------------------------------------- */
/* 重力补偿模型参数                                                           */
/* -------------------------------------------------------------------------- */

#define APP_GRAVITY_ACCELERATION (9.8f)
#define APP_GRAVITY_M2_KG (1.333f)
#define APP_GRAVITY_M3_KG (0.70583f)
#define APP_GRAVITY_M4_KG (0.59967f)
#define APP_GRAVITY_M6_KG (0.37708f)
#define APP_GRAVITY_M7_KG (0.631f)
#define APP_GRAVITY_Q2_OFFSET_RAD (0.261799f)
#define APP_GRAVITY_PITCH1_EQUIVALENT_LEVER_M (1.1728f)
#define APP_GRAVITY_PITCH1_RY2_M (-0.000476f)
#define APP_GRAVITY_PITCH2_Q2_OFFSET_RAD (0.366519f)
#define APP_GRAVITY_PITCH2_Q3_OFFSET_RAD (-0.353786f)
#define APP_GRAVITY_PITCH2_Q6_OFFSET_RAD (0.15f)
#define APP_GRAVITY_PITCH2_LM34_M (0.30228f)
#define APP_GRAVITY_PITCH2_LM56_M (0.13f)
#define APP_GRAVITY_PITCH2_D45_M (0.3f)
#define APP_GRAVITY_PITCH2_M67_KG (1.4327f)
#define APP_GRAVITY_PITCH2_RY4_M (0.003381f)
#define APP_GRAVITY_PITCH3_Q2_OFFSET_RAD (0.20944f)
#define APP_GRAVITY_PITCH3_Q3_OFFSET_RAD (0.079799f)
#define APP_GRAVITY_PITCH3_Q6_OFFSET_RAD (0.15f)
#define APP_GRAVITY_PITCH3_EFFECTIVE_LEVER_M (0.1525f)
#define APP_GRAVITY_PITCH3_RZ7_M (0.062165f)
#define APP_GRAVITY_PITCH3_RY6_M (0.05038f)
#define APP_GRAVITY_PITCH3_D7_M (0.057f)

#endif
