#ifndef NEW_ROBOT_APP_HW_PROFILE_CONFIG_H
#define NEW_ROBOT_APP_HW_PROFILE_CONFIG_H

#include "core/om_def.h"
#include <stdint.h>

/* app_hw_profile_config.h 只放“当前应用装配事实”：
 * - 正式电机命名
 * - 是否接入当前应用
 * - 是否允许正式控制
 * - vendor id / can id / master id
 *
 * 使用方式：
 * - 想禁用某个电机，优先改 APP_MR_*
 * - 想改电机正式名字或 vendor 映射 id，也改这里
 * - 不要在 task 源码里再手写名字和 installed 开关
 */

/* -------------------------------------------------------------------------- */
/* 电机装配 profile 语义                                                      */
/* -------------------------------------------------------------------------- */

/* 角色语义：
 * - DISABLED：当前应用不正式接入该电机
 * - OBSERVE_ONLY：正式接入并保留反馈观测，但不允许 control task 下发正式控制
 * - CONTROL：正式接入并允许 control task 控制
 *
 * 当前建议：
 * - 对已纳入正式 arm/chassis 控制链的电机，若只想停控，优先改成 OBSERVE_ONLY
 * - DISABLED 更适合当前不接线或未纳入正式控制链的电机，例如 roll1
 */
#define APP_MR_DISABLED   (0u)
#define APP_MR_OBSERVE_ONLY   (1u)
#define APP_MR_CONTROL    (2u)

static inline OmBool app_motor_role_is_present(uint8_t role)
{
    return (role != APP_MR_DISABLED) ? OM_TRUE : OM_FALSE;
}

static inline OmBool app_motor_role_allows_control(uint8_t role)
{
    return (role == APP_MR_CONTROL) ? OM_TRUE : OM_FALSE;
}

/* -------------------------------------------------------------------------- */
/* 正式电机命名                                                               */
/* -------------------------------------------------------------------------- */

#define APP_MN_CHASSIS_FR   "chassis_fr"
#define APP_MN_CHASSIS_FL   "chassis_fl"
#define APP_MN_CHASSIS_BL   "chassis_bl"
#define APP_MN_CHASSIS_BR   "chassis_br"
#define APP_MN_JOINT_LEG_R  "joint_leg_r"
#define APP_MN_JOINT_LEG_L  "joint_leg_l"
#define APP_MN_BIG_YAW      "big_yaw"
#define APP_MN_PITCH1       "pitch1"
#define APP_MN_PITCH2       "pitch2"
#define APP_MN_ROLL1        "roll1"
#define APP_MN_ROLL2        "roll2"
#define APP_MN_PITCH3       "pitch3"
#define APP_MN_ROLL3        "roll3"
#define APP_MN_GRIP         "grip"

/* -------------------------------------------------------------------------- */
/* 正式电机装配角色                                                           */
/* -------------------------------------------------------------------------- */

/* 当前 role 是应用级事实，不是 driver 事实。
 * 例如：
 * - roll1 当前未纳入正式控制链，因此设为 DISABLED
 * - 若后续只想保留反馈观测、不下正式控制，可改成 OBSERVE_ONLY
 */

#define APP_MR_CHASSIS_FR   APP_MR_CONTROL
#define APP_MR_CHASSIS_FL   APP_MR_CONTROL
#define APP_MR_CHASSIS_BL   APP_MR_CONTROL
#define APP_MR_CHASSIS_BR   APP_MR_CONTROL

#define APP_MR_JOINT_LEG_R  APP_MR_CONTROL
#define APP_MR_JOINT_LEG_L  APP_MR_CONTROL

#define APP_MR_BIG_YAW      APP_MR_CONTROL
#define APP_MR_PITCH1       APP_MR_CONTROL
#define APP_MR_PITCH2       APP_MR_CONTROL
#define APP_MR_ROLL1        APP_MR_DISABLED
#define APP_MR_ROLL2        APP_MR_CONTROL
#define APP_MR_PITCH3       APP_MR_CONTROL
#define APP_MR_ROLL3        APP_MR_CONTROL
#define APP_MR_GRIP         APP_MR_CONTROL

/* -------------------------------------------------------------------------- */
/* vendor 映射 id                                                             */
/* -------------------------------------------------------------------------- */

#define APP_MDJI_ID_CHASSIS_FR     (1u)
#define APP_MDJI_ID_CHASSIS_FL     (2u)
#define APP_MDJI_ID_CHASSIS_BL     (3u)
#define APP_MDJI_ID_CHASSIS_BR     (4u)
#define APP_MDJI_ID_ROLL3          (5u)

#define APP_MP10_ID_JOINT_LEG_R (1u)
#define APP_MP10_ID_JOINT_LEG_L (2u)

#define APP_MD_CAN_ID_BIG_YAW  (0x00u)
#define APP_MD_CAN_ID_PITCH1   (0x01u)
#define APP_MD_CAN_ID_ROLL1    (0x02u)
#define APP_MD_CAN_ID_ROLL2    (0x03u)
#define APP_MD_CAN_ID_GRIP     (0x04u)
#define APP_MD_CAN_ID_PITCH3   (0x05u)

#define APP_MD_MASTER_ID_BIG_YAW  (0x10u)
#define APP_MD_MASTER_ID_PITCH1   (0x11u)
#define APP_MD_MASTER_ID_ROLL1    (0x12u)
#define APP_MD_MASTER_ID_ROLL2    (0x13u)
#define APP_MD_MASTER_ID_GRIP     (0x14u)
#define APP_MD_MASTER_ID_PITCH3   (0x15u)

#define APP_MG8_ID_PITCH2 (1u)

#endif
