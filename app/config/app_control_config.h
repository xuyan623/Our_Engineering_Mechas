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
#define APP_MCT_OPERATIONAL_TX_MS      (10u)
#define APP_MCT_OPERATIONAL_OBSERVE_MS          (50u)
#define APP_MCT_IDLE_PERIOD_MS                  (50u)
#define APP_MCT_IDLE_P1010B_OBSERVE_MS   (50u)

#define APP_CT_TASK_PERIOD_MS                         (6u)
#define APP_CT_TX_REQUEST_PERIOD_MS                   (6u)

#define APP_AT_TASK_PERIOD_MS                             (3u)
#define APP_AT_BIG_YAW_LOOP_MS                  (10u)
#define APP_AT_PITCH1_LOOP_MS                   (10u)
#define APP_AT_PITCH2_LOOP_MS                   (3u)
#define APP_AT_ROLL2_LOOP_MS                    (10u)
#define APP_AT_PITCH3_LOOP_MS                   (10u)
#define APP_AT_ROLL3_LOOP_MS                    (10u)
#define APP_AT_GRIP_LOOP_MS                     (10u)
#define APP_AT_TX_REQUEST_PERIOD_MS                       (0u)

/* -------------------------------------------------------------------------- */
/* 基础数学常量                                                               */
/* -------------------------------------------------------------------------- */

#define APP_PI (3.1415926535f)
#define APP_RADIAN_COEF (57.3f)

/* -------------------------------------------------------------------------- */
/* 底盘运动学参数                                                             */
/* -------------------------------------------------------------------------- */

#define APP_CT_WHEEL_PERIMETER_MM (478.0f)
#define APP_CT_WHEEL_TRACK_MM (375.0f)
#define APP_CT_WHEEL_BASE_MM (365.0f)
#define APP_CT_DECEL_RATIO (1.0f / 19.0f)
#define APP_CT_MAX_WHEEL_RPM (8500.0f)
#define APP_CT_MAX_VX_MM_PER_S (3300.0f)
#define APP_CT_MAX_VY_MM_PER_S (3300.0f)
#define APP_CT_MAX_VW_DEG_PER_S (480.0f)
#define APP_CT_TOTAL_MAX_MM_S (1000.0f)
#define APP_CT_TOTAL_LIMIT_ENABLE (1u)
#define APP_RC_RESOLUTION (660.0f)
#define APP_CT_RC_ROTATE_SOFTEN_SCALE (0.5f)
#define APP_CT_RC_SOFTEN_HOLD_MS (2000u)
#define APP_CT_KB_MAX_SPEED_X_MM_PER_S (1000.0f)
#define APP_CT_KB_MAX_SPEED_Y_MM_PER_S (1000.0f)
#define APP_CT_KB_MAX_ROTATE_DEG_S (600.0f)
#define APP_CT_KB_MOVE_RATIO_X (1.0f)
#define APP_CT_KB_MOVE_RATIO_Y (0.6f)
#define APP_CT_KB_MOVE_RATIO_R (1.0f)
#define APP_CT_KB_MOUSE_ROTATE_SCALE (0.2f)
#define APP_CT_LEG_PIT_STEP_DEG (0.0001f)
#define APP_CT_LEG_PIT_CMD_MIN_DEG (-1.82f)
#define APP_CT_LEG_PIT_CMD_MAX_DEG (38.85f)
#define APP_CT_LEFT_LEG_BIAS_DEG (-3.4f)
#define APP_CT_RIGHT_LEG_BIAS_DEG (3.4f)
#define APP_CT_LEFT_LEG_MIN_DEG (-38.85f)
#define APP_CT_LEFT_LEG_MAX_DEG (1.82f)
#define APP_CT_RIGHT_LEG_MIN_DEG (-1.82f)
#define APP_CT_RIGHT_LEG_MAX_DEG (38.85f)

/* -------------------------------------------------------------------------- */
/* 底盘 PID 参数                                                              */
/* -------------------------------------------------------------------------- */

#define APP_CT_FRONT_VEL_PID_KP (4.5f)
#define APP_CT_FRONT_VEL_PID_KI (0.00f)
#define APP_CT_FRONT_VEL_PID_KD (0.0f)
#define APP_CT_FRONT_VEL_PID_OUT_MAX (10000.0f)
#define APP_CT_FRONT_VEL_PID_I_MAX (2000.0f)

#define APP_CT_REAR_VEL_PID_KP (3.0f)
#define APP_CT_REAR_VEL_PID_KI (0.0f)
#define APP_CT_REAR_VEL_PID_KD (0.0f)
#define APP_CT_REAR_VEL_PID_OUT_MAX (10000.0f)
#define APP_CT_REAR_VEL_PID_I_MAX (2000.0f)

/* 后腿采用单环 PID（与旧工程对齐）：角度误差直接出电流。 */
//#define APP_CT_LEG_ANGLE_PID_KP (210.0f)
#define APP_CT_LEG_ANGLE_PID_KP (0.0f)
#define APP_CT_LEG_ANGLE_PID_KI (0.0f)
//#define APP_CT_LEG_ANGLE_PID_KD (6.0f)
#define APP_CT_LEG_ANGLE_PID_KD (0.0f)
#define APP_CT_LEG_ANGLE_PID_OUT_MAX (3600.0f)
#define APP_CT_LEG_ANGLE_PID_I_MAX (200.0f)

/* -------------------------------------------------------------------------- */
/* 机械臂控制参数                                                             */
/* -------------------------------------------------------------------------- */

#define APP_AT_BIG_YAW_KP (25.0f)
#define APP_AT_BIG_YAW_KD (0.01f)
#define APP_AT_BIG_YAW_MAX_RATE_RAD_PER_S (2.0f)

#define APP_AT_PITCH1_KP (63.0f)
#define APP_AT_PITCH1_KD (0.07f)
#define APP_AT_PITCH1_MAX_RATE_RAD_PER_S (0.8f)
#define APP_AT_PITCH1_TARGET_RATIO (-1.0f)

#define APP_AT_PITCH2_KP (1.0f)
#define APP_AT_PITCH2_KD (0.06f)
#define APP_AT_PITCH2_MAX_RATE_RAD_PER_S (3.0f)

#define APP_AT_CUSTOM_ALIGN_TIMEOUT_MS (3000u)

#define APP_AT_ROLL2_KP (7.0f)
#define APP_AT_ROLL2_KD (0.01f)
#define APP_AT_ROLL2_MAX_RATE_RAD_PER_S (2.0f)

#define APP_AT_PITCH3_KP (10.0f)
#define APP_AT_PITCH3_KD (0.02f)
#define APP_AT_PITCH3_MAX_RATE_RAD_PER_S (2.0f)
#define APP_AT_PITCH3_ENABLE_GRAVITY_FF (1u)

#define APP_AT_GRIP_KP (18.0f)
#define APP_AT_GRIP_KD (0.10f)
#define APP_AT_GRIP_MAX_RATE_RAD_PER_S (4.0f)

#define APP_AT_PITCH1_GRAVITY_FF_MIN (-20.0f)
#define APP_AT_PITCH1_GRAVITY_FF_MAX (20.0f)
#define APP_AT_PITCH2_GRAVITY_FF_MIN (-1.42f)
#define APP_AT_PITCH2_GRAVITY_FF_MAX (1.43f)

#define APP_AT_ROLL3_ANGLE_PID_KP (5.0f)
#define APP_AT_ROLL3_ANGLE_PID_KI (0.0f)
#define APP_AT_ROLL3_ANGLE_PID_KD (0.0f)
#define APP_AT_ROLL3_ANGLE_PID_OUT_MAX (100.0f)
#define APP_AT_ROLL3_ANGLE_PID_I_MAX (10.0f)
#define APP_AT_ROLL3_MAX_RATE_RAD_PER_S (4.0f)

#define APP_AT_ROLL3_SPEED_PID_KP (40.0f)
#define APP_AT_ROLL3_SPEED_PID_KI (0.0f)
#define APP_AT_ROLL3_SPEED_PID_KD (0.0f)
#define APP_AT_ROLL3_SPEED_PID_OUT_MAX (15000.0f)
#define APP_AT_ROLL3_SPEED_PID_I_MAX (500.0f)

/* -------------------------------------------------------------------------- */
/* 机械臂统一最终机构角零点                                                    */
/* -------------------------------------------------------------------------- */

/* 统一最终机构角语义：
 * - machine_joint = 反馈逆映射后的机构角 - 该轴零点
 * - 动作表、FK/IK、VOFA layout0/layout4 都统一使用这套语义
 *
 * 其中：
 * - pitch2 的原始电机零点继续由 GO8010 owner 在运行时捕获
 * - 这里的 pitch2 机构角零点固定为 0
 */
#define APP_AT_JOINT_ZERO_BIG_YAW_RAD (0.0f)
#define APP_AT_JOINT_ZERO_PITCH1_RAD (0.0f)
#define APP_AT_JOINT_ZERO_PITCH2_RAD (0.0f)
#define APP_AT_JOINT_ZERO_ROLL2_RAD (0.0f)
#define APP_AT_JOINT_ZERO_PITCH3_RAD (0.1f)
#define APP_AT_JOINT_ZERO_ROLL3_RAD (3.7306414f)
#define APP_AT_JOINT_ZERO_GRIP_RAD (1.8f)

/* -------------------------------------------------------------------------- */
/* 6轴机械臂 IK / FK 参数                                                     */
/* -------------------------------------------------------------------------- */

/* URDF FK 末端工具偏置，直接对齐 simulation/kinematics_common.py::_TOOL_OFFSET。 */
#define APP_AT_IK_URDF_TOOL_OFFSET_M (0.12322f)

/* 理想控制模型参数，直接对齐 simulation/ideal_control_model.json。 */
#define APP_AT_IK_SHOULDER_HEIGHT_M (0.09000004f)
#define APP_AT_IK_SHOULDER_OFFSET_M (0.06950139f)
#define APP_AT_IK_UPPER_ARM_LENGTH_M (0.25872970f)
#define APP_AT_IK_FOREARM_LENGTH_M (0.27005011f)
#define APP_AT_IK_TOOL_LENGTH_M (0.17122f)

/* 仿真/URDF 的 solver 零位是“机械臂竖直拉直”。
 * 真实电机当前以 normal 姿态为零位，因此需要一组 solver 侧 home offset：
 * - 当 machine joint vector = 0（真实 normal 姿态）时
 * - 映射到仿真 Joint1~6 = [0, -66.5, -154.7, 0, 0, 0] deg
 */
#define APP_AT_IK_HOME_BIG_YAW_RAD (0.0f)
#define APP_AT_IK_HOME_PITCH1_RAD (-1.16064395f)
#define APP_AT_IK_HOME_PITCH2_RAD (-2.69995069f)
#define APP_AT_IK_HOME_ROLL2_RAD (0.0f)
#define APP_AT_IK_HOME_PITCH3_RAD (0.0f)
#define APP_AT_IK_HOME_ROLL3_RAD (0.0f)

/* 公开 machine pose 语义对应的 6 轴限位。 */
#define APP_AT_IK_BIG_YAW_MIN_RAD (-3.14f)
#define APP_AT_IK_BIG_YAW_MAX_RAD (3.14f)
#define APP_AT_IK_PITCH1_MIN_RAD (-1.16f)
#define APP_AT_IK_PITCH1_MAX_RAD (1.69f)
#define APP_AT_IK_PITCH2_MIN_RAD (0.0f)
#define APP_AT_IK_PITCH2_MAX_RAD (2.70f)
#define APP_AT_IK_ROLL2_MIN_RAD (-3.14f)
#define APP_AT_IK_ROLL2_MAX_RAD (3.14f)
#define APP_AT_IK_PITCH3_MIN_RAD (-1.57f)
#define APP_AT_IK_PITCH3_MAX_RAD (1.57f)

/* 内部 URDF joint 语义对应的限位。 */
#define APP_AT_IK_URDF_PITCH2_MIN_RAD (-2.70f)
#define APP_AT_IK_URDF_PITCH2_MAX_RAD (0.0f)
#define APP_AT_IK_URDF_ROLL3_MIN_RAD (-3.14f)
#define APP_AT_IK_URDF_ROLL3_MAX_RAD (3.14f)

/* realtime full-pose solver 参数。 */
#define APP_AT_IK_FULL_POSE_MAX_ITERATIONS (200u)
#define APP_AT_IK_FULL_POS_TOL_M (1e-5f)
#define APP_AT_IK_FULL_ORI_TOL_RAD (1e-4f)
#define APP_AT_IK_FULL_ACCEPT_POS_ERR_M (1e-4f)
#define APP_AT_IK_FULL_ACCEPT_ORI_ERR_RAD (2e-3f)
#define APP_AT_IK_FULL_POSE_DAMPING (5e-3f)
#define APP_AT_IK_FULL_POSE_STEP_LIMIT_RAD (0.25f)
#define APP_AT_IK_FULL_POSE_EPSILON_RAD (1e-5f)

/* realtime position-priority solver 参数。 */
#define APP_AT_IK_POS_PRI_MAX_ITERS (200u)
#define APP_AT_IK_POS_PRI_TOL_M (1e-5f)
#define APP_AT_IK_POS_PRI_ACCEPT_POS_ERR_M (1e-4f)
#define APP_AT_IK_POS_PRI_DAMPING (5e-3f)
#define APP_AT_IK_POS_PRI_STEP_MAX_RAD (0.25f)
#define APP_AT_IK_POS_PRI_EPS_RAD (1e-5f)

/* 遥控器 IK 模式：满杆对应的目标 pose 速度。
 * 位置单位 mm/s，姿态单位 deg/s；arm_task 内部会换算到 m/s 与 rad/s。
 */
#define APP_AT_RC_IK_POS_X_MM_PER_S (120.0f)
#define APP_AT_RC_IK_POS_Y_MM_PER_S (120.0f)
#define APP_AT_RC_IK_POS_Z_MM_PER_S (120.0f)
#define APP_AT_RC_IK_ROLL_DEG_PER_S (35.0f)
#define APP_AT_RC_IK_PITCH_DEG_PER_S (35.0f)
#define APP_AT_RC_IK_YAW_DEG_PER_S (35.0f)

/* 夹爪两态目标使用统一最终机构角语义：
 * - OPEN：normal 姿态，因此为 0
 * - CLOSED：当前物理闭合角 0 减去 grip 零点 1.8
 */
#define APP_AT_GRIP_OPEN_TARGET_RAD (0.0f)
#define APP_AT_GRIP_CLOSED_TARGET_RAD (-1.8f)

/* -------------------------------------------------------------------------- */
/* 机械臂逆运动学几何参数                                                     */
/* -------------------------------------------------------------------------- */

/* 旧的 2 连杆几何 helper，当前仅保留给 legacy kin_pos_to_motor()。 */
#define APP_AT_LINK_D2_MM (78.0f)
#define APP_AT_LINK_A1_MM (400.0f)
#define APP_AT_LINK_D3_MM (404.0f)
#define APP_AT_LINK_A2_MM (65.0f)

/* -------------------------------------------------------------------------- */
/* Pitch 关节零位偏置与限幅                                                   */
/* -------------------------------------------------------------------------- */

#define APP_AT_PITCH1_ZERO_OFFSET_RAD (0.2793f)
#define APP_AT_PITCH2_ZERO_OFFSET_RAD (1.85f)
#define APP_AT_PITCH1_MIN_RAD (-2.5133f)
#define APP_AT_PITCH1_MAX_RAD (0.0f)
#define APP_AT_PITCH2_MIN_RAD (-2.0f)
#define APP_AT_PITCH2_MAX_RAD (0.0f)
#define APP_AT_PITCH2_GEAR_RATIO (6.33f)

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
#define APP_GRAVITY_PITCH1_LEVER_M (1.1728f)
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
#define APP_GRAVITY_PITCH3_LEVER_M (0.1525f)
#define APP_GRAVITY_PITCH3_RZ7_M (0.062165f)
#define APP_GRAVITY_PITCH3_RY6_M (0.05038f)
#define APP_GRAVITY_PITCH3_D7_M (0.057f)

#endif
