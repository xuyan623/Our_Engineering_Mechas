#ifndef NEW_ROBOT_APP_DEBUG_CONFIG_H
#define NEW_ROBOT_APP_DEBUG_CONFIG_H

/* app_debug_config.h 只放“调试和观测面参数”：
 * - system_health 自测
 * - 恢复模块调试节拍
 * - 临时观测开关
 * - VOFA 默认布局
 *
 * 使用方式：
 * - 想切 VOFA 默认显示内容，改 APP_VOFA_DEFAULT_LAYOUT_ID
 * - 想改 observer 发送频率，改 APP_VOFA_TASK_PERIOD_MS
 * - 不要把正式控制 PID 参数放到这里
 */

/* -------------------------------------------------------------------------- */
/* system_health 自测开关                                                     */
/* -------------------------------------------------------------------------- */

#define APP_SH_SELF_TEST_MODE_OFF (0u)
#define APP_SH_SELF_TEST_MODE_RT_TIMEOUT (1u)
#define APP_SH_SELF_TEST_MODE_FATAL (2u)
#define APP_SH_SELF_TEST_MODE (APP_SH_SELF_TEST_MODE_OFF)
#define APP_SH_SELF_TEST_DELAY_MS (0u)

/* -------------------------------------------------------------------------- */
/* 电机自动恢复                                                               */
/* -------------------------------------------------------------------------- */

#define APP_MOTOR_AUTO_RECOVERY_ENABLE (0u)
#define APP_MR_ONLINE_AGE_MS (500u)
#define APP_MR_RETRY_MS (50u)
#define APP_MR_TICK_PERIOD_MS (50u)
#define APP_MR_P1010B_REPORT_MS (50u)
#define APP_MR_FAULT_DELAY_MS (100u)

/* -------------------------------------------------------------------------- */
/* 过渡期调试/观测开关                                                        */
/* -------------------------------------------------------------------------- */

/* big_yaw 在 arm_task 完整接管前，允许 chassis_task 做临时低增益保持。 */
#define BIG_YAW_TEMP_HOLD_ENABLE (0u)
#define APP_CT_BIG_YAW_HOLD_KP (3.0f)
#define APP_CT_BIG_YAW_HOLD_KD (0.12f)

/* VOFA observer 面默认布局。
 * 当前内置布局：
 * - 0：arm_machine_angles
 * - 1：chassis_debug
 * - 2：mct_runtime
 * - 3：arm_fk_pose
 * - 4：arm_ik_joints
 */
#define APP_VOFA_TASK_PERIOD_MS (10u)
#define APP_VOFA_DEFAULT_LAYOUT_ID (3u)

#endif
