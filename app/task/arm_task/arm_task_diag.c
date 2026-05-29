/* arm_task 观测接口实现。
 * 从 arm_task.c 抽离，通过 arm_task_internal.h 访问运行时上下文与内部 helper。
 */

#include "task/arm_task/arm_task_diag.h"
#include "task/arm_task/arm_task_internal.h"
#include "function/math_utils/math_utils.h"
#include "task/motor_communications_task/mct.h"

uint8_t arm_task_get_custom_controller_alignment_done(void)
{
    return g_arm_task_custom_controller_alignment_done_debug;
}

uint8_t arm_task_get_custom_controller_online(void)
{
    if (g_arm_task_owner_context == OM_NULL)
    {
        return 0u;
    }

    return g_arm_task_owner_context->latest_custom_controller_snapshot.online;
}

uint8_t arm_task_get_custom_controller_takeover_bit(void)
{
    if (g_arm_task_owner_context == OM_NULL)
    {
        return 0u;
    }

    return (g_arm_task_owner_context->latest_mode_snapshot.chassis_mode ==
                MODE_CHASSIS_CUSTOM_CONTROLLER_NORMAL &&
            g_arm_task_owner_context->custom_controller_alignment_done == OM_TRUE &&
            g_arm_task_owner_context->latest_custom_controller_snapshot.online != 0u &&
            g_arm_task_owner_context->latest_custom_controller_snapshot.work_mode ==
                ARM_TASK_CUSTOM_CONTROLLER_WORK_MODE_ENCODER)
               ? 1u
               : 0u;
}


OmBool arm_task_get_pitch2_debug_snapshot(
    float* pitch2_feedback_deg,
    float* pitch2_feedback_rpm,
    float* pitch2_feedback_current,
    float* pitch2_feedback_torque,
    float* pitch2_feedback_online,
    float* pitch2_target_deg)
{
    const MotorFeedback* pitch2_feedback = OM_NULL;
    OmBool export_targets = OM_FALSE;

    if (g_arm_task_owner_context == OM_NULL ||
        pitch2_feedback_deg == OM_NULL ||
        pitch2_feedback_rpm == OM_NULL ||
        pitch2_feedback_current == OM_NULL ||
        pitch2_feedback_torque == OM_NULL ||
        pitch2_feedback_online == OM_NULL ||
        pitch2_target_deg == OM_NULL)
    {
        return OM_FALSE;
    }

    export_targets =
        (mct_is_operational_active() == OM_TRUE &&
         g_arm_task_owner_context->latest_mode_snapshot.chassis_mode != MODE_CHASSIS_RELEASE)
            ? OM_TRUE
            : OM_FALSE;

    pitch2_feedback = motor_get_feedback(g_arm_task_owner_context->pitch2_motor);
    *pitch2_feedback_deg =
        (pitch2_feedback != OM_NULL) ? math_utils_rad_to_deg(pitch2_feedback->angle) : 0.0f;
    *pitch2_feedback_rpm =
        (pitch2_feedback != OM_NULL) ? math_utils_rad_per_s_to_rpm(pitch2_feedback->speed) : 0.0f;
    *pitch2_feedback_current =
        (pitch2_feedback != OM_NULL) ? pitch2_feedback->current : 0.0f;
    *pitch2_feedback_torque =
        (pitch2_feedback != OM_NULL) ? pitch2_feedback->torque : 0.0f;
    *pitch2_feedback_online =
        (motor_is_feedback_recent(
             g_arm_task_owner_context->pitch2_motor,
             ARM_TASK_GO8010_RECENT_TIMEOUT_MS) == OM_TRUE)
            ? 1.0f
            : 0.0f;
    *pitch2_target_deg =
        (export_targets == OM_TRUE)
            ? math_utils_rad_to_deg(g_arm_task_owner_context->smoothed_targets.pitch2_rad)
            : 0.0f;

    return OM_TRUE;
}

OmBool arm_task_get_custom_controller_feedback_snapshot(
    float* axis0_feedback_deg,
    float* axis1_feedback_deg,
    float* axis2_feedback_deg)
{
    if (g_arm_task_owner_context == OM_NULL ||
        axis0_feedback_deg == OM_NULL ||
        axis1_feedback_deg == OM_NULL ||
        axis2_feedback_deg == OM_NULL)
    {
        return OM_FALSE;
    }

    *axis0_feedback_deg =
        g_arm_task_owner_context->latest_custom_controller_snapshot.angle_deg[ARM_TASK_CUSTOM_AXIS_Y];
    *axis1_feedback_deg =
        g_arm_task_owner_context->latest_custom_controller_snapshot.angle_deg[ARM_TASK_CUSTOM_AXIS_Z];
    *axis2_feedback_deg =
        g_arm_task_owner_context->latest_custom_controller_snapshot.angle_deg[ARM_TASK_CUSTOM_AXIS_X];
    return OM_TRUE;
}

OmBool arm_task_get_custom_controller_pitch_axis_feedback(float* pitch_axis_feedback_deg)
{
    if (g_arm_task_owner_context == OM_NULL || pitch_axis_feedback_deg == OM_NULL)
    {
        return OM_FALSE;
    }

    *pitch_axis_feedback_deg =
        g_arm_task_owner_context->latest_custom_controller_snapshot.angle_deg[ARM_TASK_CUSTOM_AXIS_PITCH];
    return OM_TRUE;
}

OmBool arm_task_get_arm_motor_machine_angle_rad_snapshot(
    float machine_angle_rad[7])
{
    const MotorFeedback* fb = OM_NULL;
    float pitch2_zero_rad = 0.0f;

    if (g_arm_task_owner_context == OM_NULL || machine_angle_rad == OM_NULL)
    {
        return OM_FALSE;
    }

    /* big_yaw: zhi jie ni xiang */
    fb = motor_get_feedback(g_arm_task_owner_context->big_yaw_motor);
    machine_angle_rad[0] = (fb != OM_NULL) ?
        (fb->angle - g_arm_pose_normal.machine_values[ARM_TASK_MACHINE_BIG_YAW]) : 0.0f;

    /* pitch1: xian chu yi ratio zai jian normal */
    fb = motor_get_feedback(g_arm_task_owner_context->pitch1_motor);
    machine_angle_rad[1] = (fb != OM_NULL) ?
        (fb->angle / APP_ARM_PITCH1_TARGET_RATIO -
         g_arm_pose_normal.machine_values[ARM_TASK_MACHINE_PITCH1]) : 0.0f;

    /* pitch2: xian jian zero, zai chu yi -gear_ratio, zai jian normal */
    fb = motor_get_feedback(g_arm_task_owner_context->pitch2_motor);
    if (fb != OM_NULL)
    {
        if (arm_task_get_pitch2_zero_angle_rad(g_arm_task_owner_context, &pitch2_zero_rad) != OM_TRUE)
        {
            pitch2_zero_rad = fb->angle;
        }
        machine_angle_rad[2] =
            (fb->angle - pitch2_zero_rad) / (-APP_ARM_PITCH2_GEAR_RATIO) -
            g_arm_pose_normal.machine_values[ARM_TASK_MACHINE_PITCH2];
    }
    else
    {
        machine_angle_rad[2] = 0.0f;
    }

    /* roll2: zhi jie ni xiang */
    fb = motor_get_feedback(g_arm_task_owner_context->roll2_motor);
    machine_angle_rad[3] = (fb != OM_NULL) ?
        (fb->angle - g_arm_pose_normal.machine_values[ARM_TASK_MACHINE_ROLL2]) : 0.0f;

    /* pitch3: zhi jie ni xiang */
    fb = motor_get_feedback(g_arm_task_owner_context->pitch3_motor);
    machine_angle_rad[4] = (fb != OM_NULL) ?
        (fb->angle - g_arm_pose_normal.machine_values[ARM_TASK_MACHINE_PITCH3]) : 0.0f;

    /* roll3: GM6020 bian ma qi fan wei [-pi, pi), zi tai biao yong [0, 2*pi).
     * Ni xiang shi tou chuan, dan yao zuo [0, 2*pi) unwrap cai neng yu dong zuo biao wan quan dui qi.
     */
    fb = motor_get_feedback(g_arm_task_owner_context->roll3_motor);
    if (fb != OM_NULL)
    {
        float roll3_unwrapped = fb->angle;
        while (roll3_unwrapped < 0.0f)
        {
            roll3_unwrapped += 2.0f * 3.14159265358979323846f;
        }
        while (roll3_unwrapped >= 2.0f * 3.14159265358979323846f)
        {
            roll3_unwrapped -= 2.0f * 3.14159265358979323846f;
        }
        machine_angle_rad[5] = roll3_unwrapped;
    }
    else
    {
        machine_angle_rad[5] = 0.0f;
    }

    /* grip: zhi jie ni xiang */
    fb = motor_get_feedback(g_arm_task_owner_context->grip_motor);
    machine_angle_rad[6] = (fb != OM_NULL) ?
        (fb->angle - g_arm_pose_normal.machine_values[ARM_TASK_MACHINE_GRIP]) : 0.0f;

    return OM_TRUE;
}

OmBool arm_task_get_arm_motor_feedback_rad_snapshot(
    float arm_feedback_rad[7])
{
    const MotorFeedback* fb = OM_NULL;

    if (g_arm_task_owner_context == OM_NULL || arm_feedback_rad == OM_NULL)
    {
        return OM_FALSE;
    }

    fb = motor_get_feedback(g_arm_task_owner_context->big_yaw_motor);
    arm_feedback_rad[0] = (fb != OM_NULL) ? fb->angle : 0.0f;

    fb = motor_get_feedback(g_arm_task_owner_context->pitch1_motor);
    arm_feedback_rad[1] = (fb != OM_NULL) ? fb->angle : 0.0f;

    fb = motor_get_feedback(g_arm_task_owner_context->pitch2_motor);
    arm_feedback_rad[2] = (fb != OM_NULL) ? fb->angle : 0.0f;

    fb = motor_get_feedback(g_arm_task_owner_context->roll2_motor);
    arm_feedback_rad[3] = (fb != OM_NULL) ? fb->angle : 0.0f;

    fb = motor_get_feedback(g_arm_task_owner_context->pitch3_motor);
    arm_feedback_rad[4] = (fb != OM_NULL) ? fb->angle : 0.0f;

    fb = motor_get_feedback(g_arm_task_owner_context->roll3_motor);
    arm_feedback_rad[5] = (fb != OM_NULL) ? fb->angle : 0.0f;

    fb = motor_get_feedback(g_arm_task_owner_context->grip_motor);
    arm_feedback_rad[6] = (fb != OM_NULL) ? fb->angle : 0.0f;

    return OM_TRUE;
}

OmBool arm_task_get_pitch1_debug_snapshot(
    float* pitch1_feedback_deg,
    float* pitch1_feedback_rpm,
    float* pitch1_feedback_torque,
    float* pitch1_feedback_online,
    float* pitch1_target_deg)
{
    const MotorFeedback* pitch1_feedback = OM_NULL;
    OmBool export_targets = OM_FALSE;

    if (g_arm_task_owner_context == OM_NULL ||
        pitch1_feedback_deg == OM_NULL ||
        pitch1_feedback_rpm == OM_NULL ||
        pitch1_feedback_torque == OM_NULL ||
        pitch1_feedback_online == OM_NULL ||
        pitch1_target_deg == OM_NULL)
    {
        return OM_FALSE;
    }

    export_targets =
        (mct_is_operational_active() == OM_TRUE &&
         g_arm_task_owner_context->latest_mode_snapshot.chassis_mode != MODE_CHASSIS_RELEASE)
            ? OM_TRUE
            : OM_FALSE;

    pitch1_feedback = motor_get_feedback(g_arm_task_owner_context->pitch1_motor);
    *pitch1_feedback_deg =
        (pitch1_feedback != OM_NULL) ? math_utils_rad_to_deg(pitch1_feedback->angle) : 0.0f;
    *pitch1_feedback_rpm =
        (pitch1_feedback != OM_NULL) ? math_utils_rad_per_s_to_rpm(pitch1_feedback->speed) : 0.0f;
    *pitch1_feedback_torque =
        (pitch1_feedback != OM_NULL) ? pitch1_feedback->torque : 0.0f;
    *pitch1_feedback_online =
        (motor_is_feedback_recent(
             g_arm_task_owner_context->pitch1_motor,
             ARM_TASK_DAMIAO_RECENT_TIMEOUT_MS) == OM_TRUE)
            ? 1.0f
            : 0.0f;
    *pitch1_target_deg =
        (export_targets == OM_TRUE)
            ? math_utils_rad_to_deg(g_arm_task_owner_context->smoothed_targets.pitch1_rad)
            : 0.0f;

    return OM_TRUE;
}

OmBool arm_task_get_pitch3_debug_snapshot(
    float* pitch3_feedback_deg,
    float* pitch3_feedback_rpm,
    float* pitch3_feedback_torque,
    float* pitch3_feedback_online,
    float* pitch3_target_deg)
{
    const MotorFeedback* pitch3_feedback = OM_NULL;
    OmBool export_targets = OM_FALSE;

    if (g_arm_task_owner_context == OM_NULL ||
        pitch3_feedback_deg == OM_NULL ||
        pitch3_feedback_rpm == OM_NULL ||
        pitch3_feedback_torque == OM_NULL ||
        pitch3_feedback_online == OM_NULL ||
        pitch3_target_deg == OM_NULL)
    {
        return OM_FALSE;
    }

    export_targets =
        (mct_is_operational_active() == OM_TRUE &&
         g_arm_task_owner_context->latest_mode_snapshot.chassis_mode != MODE_CHASSIS_RELEASE)
            ? OM_TRUE
            : OM_FALSE;

    pitch3_feedback = motor_get_feedback(g_arm_task_owner_context->pitch3_motor);
    *pitch3_feedback_deg =
        (pitch3_feedback != OM_NULL) ? math_utils_rad_to_deg(pitch3_feedback->angle) : 0.0f;
    *pitch3_feedback_rpm =
        (pitch3_feedback != OM_NULL) ? math_utils_rad_per_s_to_rpm(pitch3_feedback->speed) : 0.0f;
    *pitch3_feedback_torque =
        (pitch3_feedback != OM_NULL) ? pitch3_feedback->torque : 0.0f;
    *pitch3_feedback_online =
        (motor_is_feedback_recent(
             g_arm_task_owner_context->pitch3_motor,
             ARM_TASK_DAMIAO_RECENT_TIMEOUT_MS) == OM_TRUE)
            ? 1.0f
            : 0.0f;
    *pitch3_target_deg =
        (export_targets == OM_TRUE)
            ? math_utils_rad_to_deg(g_arm_task_owner_context->smoothed_targets.pitch3_rad)
            : 0.0f;

    return OM_TRUE;
}

OmBool arm_task_get_debug_snapshot(
    float* big_yaw_feedback_deg,
    float* big_yaw_target_deg,
    float* big_yaw_force_feedback,
    float* pitch1_feedback_deg,
    float* pitch1_target_deg,
    float* pitch1_force_feedback,
    float* pitch2_feedback_deg,
    float* pitch2_target_deg,
    float* pitch2_force_feedback,
    float* pitch3_feedback_deg,
    float* pitch3_target_deg,
    float* pitch3_force_feedback,
    float* grip_feedback_deg,
    float* grip_target_deg,
    float* grip_force_feedback)
{
    const MotorFeedback* big_yaw_feedback = OM_NULL;
    const MotorFeedback* pitch1_feedback = OM_NULL;
    const MotorFeedback* pitch2_feedback = OM_NULL;
    const MotorFeedback* pitch3_feedback = OM_NULL;
    const MotorFeedback* grip_feedback = OM_NULL;
    OmBool export_targets = OM_FALSE;

    if (g_arm_task_owner_context == OM_NULL ||
        big_yaw_feedback_deg == OM_NULL ||
        big_yaw_target_deg == OM_NULL ||
        big_yaw_force_feedback == OM_NULL ||
        pitch1_feedback_deg == OM_NULL ||
        pitch1_target_deg == OM_NULL ||
        pitch1_force_feedback == OM_NULL ||
        pitch2_feedback_deg == OM_NULL ||
        pitch2_target_deg == OM_NULL ||
        pitch2_force_feedback == OM_NULL ||
        pitch3_feedback_deg == OM_NULL ||
        pitch3_target_deg == OM_NULL ||
        pitch3_force_feedback == OM_NULL ||
        grip_feedback_deg == OM_NULL ||
        grip_target_deg == OM_NULL ||
        grip_force_feedback == OM_NULL)
    {
        return OM_FALSE;
    }

    export_targets =
        (mct_is_operational_active() == OM_TRUE &&
         g_arm_task_owner_context->latest_mode_snapshot.chassis_mode != MODE_CHASSIS_RELEASE)
            ? OM_TRUE
            : OM_FALSE;

    /* big_yaw：反馈角、目标角、力矩反馈 */
    big_yaw_feedback = motor_get_feedback(g_arm_task_owner_context->big_yaw_motor);
    *big_yaw_feedback_deg =
        (big_yaw_feedback != OM_NULL) ? math_utils_rad_to_deg(big_yaw_feedback->angle) : 0.0f;
    *big_yaw_target_deg =
        (export_targets == OM_TRUE)
            ? math_utils_rad_to_deg(g_arm_task_owner_context->smoothed_targets.big_yaw_rad)
            : 0.0f;
    *big_yaw_force_feedback =
        (big_yaw_feedback != OM_NULL) ? big_yaw_feedback->torque : 0.0f;

    /* pitch1：反馈角、目标角、力矩反馈 */
    pitch1_feedback = motor_get_feedback(g_arm_task_owner_context->pitch1_motor);
    *pitch1_feedback_deg =
        (pitch1_feedback != OM_NULL) ? math_utils_rad_to_deg(pitch1_feedback->angle) : 0.0f;
    *pitch1_target_deg =
        (export_targets == OM_TRUE)
            ? math_utils_rad_to_deg(g_arm_task_owner_context->smoothed_targets.pitch1_rad)
            : 0.0f;
    *pitch1_force_feedback =
        (pitch1_feedback != OM_NULL) ? pitch1_feedback->torque : 0.0f;

    /* pitch2：反馈角、目标角、力矩反馈 */
    pitch2_feedback = motor_get_feedback(g_arm_task_owner_context->pitch2_motor);
    *pitch2_feedback_deg =
        (pitch2_feedback != OM_NULL) ? math_utils_rad_to_deg(pitch2_feedback->angle) : 0.0f;
    *pitch2_target_deg =
        (export_targets == OM_TRUE)
            ? math_utils_rad_to_deg(g_arm_task_owner_context->smoothed_targets.pitch2_rad)
            : 0.0f;
    *pitch2_force_feedback =
        (pitch2_feedback != OM_NULL) ? pitch2_feedback->torque : 0.0f;

    /* pitch3：反馈角、目标角、力矩反馈 */
    pitch3_feedback = motor_get_feedback(g_arm_task_owner_context->pitch3_motor);
    *pitch3_feedback_deg =
        (pitch3_feedback != OM_NULL) ? math_utils_rad_to_deg(pitch3_feedback->angle) : 0.0f;
    *pitch3_target_deg =
        (export_targets == OM_TRUE)
            ? math_utils_rad_to_deg(g_arm_task_owner_context->smoothed_targets.pitch3_rad)
            : 0.0f;
    *pitch3_force_feedback =
        (pitch3_feedback != OM_NULL) ? pitch3_feedback->torque : 0.0f;

    /* grip：反馈角、目标角、力矩反馈 */
    grip_feedback = motor_get_feedback(g_arm_task_owner_context->grip_motor);
    *grip_feedback_deg =
        (grip_feedback != OM_NULL) ? math_utils_rad_to_deg(grip_feedback->angle) : 0.0f;
    *grip_target_deg =
        (export_targets == OM_TRUE)
            ? math_utils_rad_to_deg(g_arm_task_owner_context->smoothed_targets.grip_rad)
            : 0.0f;
    *grip_force_feedback =
        (grip_feedback != OM_NULL) ? grip_feedback->torque : 0.0f;
    return OM_TRUE;
}
