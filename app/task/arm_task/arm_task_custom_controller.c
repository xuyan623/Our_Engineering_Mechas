#include "task/arm_task/arm_task_internal.h"

#include "algorithm/gravity_comp/gravity_comp.h"
#include "config/app_config.h"
#include "driver/motor/motor.h"
#include "function/math_utils/math_utils.h"
#include "module/motor_tx_dispatch/motor_tx_dispatch.h"
#include "module/system_health/system_health.h"
#include "osal/osal.h"
#include "osal/osal_config.h"
#include "osal/osal_time.h"
#include "task/mode_task/mode_task.h"
#include "task/motor_communications_task/mct.h"
#include <string.h>

volatile uint8_t g_arm_task_custom_controller_alignment_done_debug = 0u;
/* 自定义控制器原始角度直接进机械臂时，pitch2 会被 6.33 齿比放大。
 * 这里在 arm_task 内先做每轴前处理：
 * - 死区：压掉控制器静止时的小抖动
 * - 一阶低通：避免原始角度噪声直接打到关节目标
 *
 * 当前只在自定义控制器接管路径生效，不影响遥控器或动作表。
 */
const float g_arm_task_custom_controller_deadband_deg[ARM_TASK_CUSTOM_CONTROLLER_AXIS_COUNT] = {
    0.8f, 0.8f, 1.5f, 0.8f, 0.8f, 0.8f};
const float g_arm_task_custom_controller_filter_alpha[ARM_TASK_CUSTOM_CONTROLLER_AXIS_COUNT] = {
    0.18f, 0.18f, 0.10f, 0.18f, 0.18f, 0.18f};
void arm_task_preprocess_custom_controller_delta_deg(
    ArmTaskContext* context,
    const ArmTaskCustomControllerSnapshot* controller_snapshot,
    float filtered_delta_deg[ARM_TASK_CUSTOM_CONTROLLER_AXIS_COUNT])
{
    uint32_t axis_index = 0u;

    if (context == OM_NULL || controller_snapshot == OM_NULL || filtered_delta_deg == OM_NULL)
    {
        return;
    }

    for (axis_index = 0u; axis_index < ARM_TASK_CUSTOM_CONTROLLER_AXIS_COUNT; axis_index++)
    {
        const float raw_delta_deg =
            controller_snapshot->angle_deg[axis_index] -
            context->custom_controller_neutral_deg[axis_index];
        const float deadbanded_delta_deg =
            math_utils_apply_symmetric_deadband(
                raw_delta_deg,
                g_arm_task_custom_controller_deadband_deg[axis_index]);

        // 首次运行时直接使用死区处理后的值初始化滤波器
        if (!(context->flags & ARM_TASK_FLAG_CUSTOM_FILTER_INIT))
        {
            context->custom_controller_filtered_delta_deg[axis_index] =
                deadbanded_delta_deg;
        }
        else
        {
            // 使用一阶低通滤波器平滑角度变化
            const float current_filtered_deg =
                context->custom_controller_filtered_delta_deg[axis_index];
            const float alpha =
                g_arm_task_custom_controller_filter_alpha[axis_index];

            context->custom_controller_filtered_delta_deg[axis_index] =
                current_filtered_deg +
                alpha * (deadbanded_delta_deg - current_filtered_deg);
        }

        filtered_delta_deg[axis_index] =
            context->custom_controller_filtered_delta_deg[axis_index];
    }

    context->flags |= ARM_TASK_FLAG_CUSTOM_FILTER_INIT;
}
void arm_task_refresh_smoothed_targets_from_feedback(ArmTaskContext* context)
{
    const MotorFeedback* feedback = OM_NULL;

    if (context == OM_NULL || (context->flags & ARM_TASK_FLAG_SMOOTHED_TARGETS_INIT))
    {
        return;
    }

    feedback = motor_get_feedback(arm_task_get_motor(ARM_TASK_MACHINE_BIG_YAW));
    context->smoothed_targets.big_yaw_rad = (feedback != OM_NULL) ? feedback->angle : 0.0f;

    feedback = motor_get_feedback(arm_task_get_motor(ARM_TASK_MACHINE_PITCH1));
    context->smoothed_targets.pitch1_rad = (feedback != OM_NULL) ? feedback->angle : 0.0f;

    feedback = motor_get_feedback(arm_task_get_motor(ARM_TASK_MACHINE_PITCH2));
    if (feedback != OM_NULL)
    {
        context->smoothed_targets.pitch2_rad = feedback->angle;
    }
    else if (arm_task_get_pitch2_zero_angle_rad(context, &context->smoothed_targets.pitch2_rad) != OM_TRUE)
    {
        context->smoothed_targets.pitch2_rad = 0.0f;
    }

    feedback = motor_get_feedback(arm_task_get_motor(ARM_TASK_MACHINE_ROLL2));
    context->smoothed_targets.roll2_rad = (feedback != OM_NULL) ? feedback->angle : 0.0f;

    feedback = motor_get_feedback(arm_task_get_motor(ARM_TASK_MACHINE_PITCH3));
    context->smoothed_targets.pitch3_rad = (feedback != OM_NULL) ? feedback->angle : 0.0f;

    if (arm_task_get_roll3_feedback_angle_rad(context, &context->smoothed_targets.roll3_rad) != OM_TRUE)
    {
        context->smoothed_targets.roll3_rad = 0.0f;
    }

    feedback = motor_get_feedback(arm_task_get_motor(ARM_TASK_MACHINE_GRIP));
    context->smoothed_targets.grip_rad = (feedback != OM_NULL) ? feedback->angle : 0.0f;

    context->flags |= ARM_TASK_FLAG_SMOOTHED_TARGETS_INIT;
}

void arm_task_clear_smoothed_targets(ArmTaskContext* context)
{
    if (context == OM_NULL)
    {
        return;
    }

    memset(&context->smoothed_targets, 0, sizeof(context->smoothed_targets));
    context->flags &= ~ARM_TASK_FLAG_SMOOTHED_TARGETS_INIT;
}

void arm_task_reset_custom_controller_state(ArmTaskContext* context)
{
    uint32_t axis_index = 0u;

    if (context == OM_NULL)
    {
        return;
    }

    context->flags &= ~ARM_TASK_FLAG_CUSTOM_ALIGNMENT_DONE;
    context->flags &= ~ARM_TASK_FLAG_CUSTOM_ALIGNMENT_FAILED;
    context->flags &= ~ARM_TASK_FLAG_CUSTOM_REF_CAPTURED;
    context->flags &= ~ARM_TASK_FLAG_CUSTOM_WAS_ACTIVE;
    context->flags &= ~ARM_TASK_FLAG_CUSTOM_FILTER_INIT;
    context->custom_controller_alignment_started_ms = 0u;
    for (axis_index = 0u; axis_index < ARM_TASK_CUSTOM_CONTROLLER_AXIS_COUNT; axis_index++)
    {
        context->custom_controller_filtered_delta_deg[axis_index] = 0.0f;
    }
    g_arm_task_custom_controller_alignment_done_debug = 0u;
}

void arm_task_apply_custom_controller_alignment_pose(
    ArmTaskContext* context,
    ArmTaskMachinePose* pose)
{
    float big_yaw_target_rad = 0.0f;
    float pitch1_target_motor_rad = 0.0f;
    float pitch2_target_motor_rad = 0.0f;
    float roll2_target_rad = 0.0f;
    float pitch3_target_rad = 0.0f;
    float roll3_target_rad = 0.0f;
    float grip_target_rad = 0.0f;
    float pitch2_zero_angle_rad = 0.0f;
    const MotorFeedback* big_yaw_feedback = OM_NULL;
    const MotorFeedback* pitch1_feedback = OM_NULL;
    const MotorFeedback* pitch2_feedback = OM_NULL;
    const MotorFeedback* roll2_feedback = OM_NULL;
    const MotorFeedback* pitch3_feedback = OM_NULL;
    const MotorFeedback* grip_feedback = OM_NULL;

    if (context == OM_NULL || pose == OM_NULL)
    {
        return;
    }

    arm_task_assign_pose(pose, &g_arm_pose_zero);

    if ((context->flags & ARM_TASK_FLAG_SMOOTHED_TARGETS_INIT))
    {
        big_yaw_target_rad = context->smoothed_targets.big_yaw_rad;
        pitch1_target_motor_rad = context->smoothed_targets.pitch1_rad;
        pitch2_target_motor_rad = context->smoothed_targets.pitch2_rad;
        roll2_target_rad = context->smoothed_targets.roll2_rad;
        pitch3_target_rad = context->smoothed_targets.pitch3_rad;
        roll3_target_rad = context->smoothed_targets.roll3_rad;
        grip_target_rad = context->smoothed_targets.grip_rad;
    }
    else
    {
        big_yaw_feedback = motor_get_feedback(arm_task_get_motor(ARM_TASK_MACHINE_BIG_YAW));
        big_yaw_target_rad =
            (big_yaw_feedback != OM_NULL) ? big_yaw_feedback->angle : 0.0f;

        pitch1_feedback = motor_get_feedback(arm_task_get_motor(ARM_TASK_MACHINE_PITCH1));
        pitch1_target_motor_rad =
            (pitch1_feedback != OM_NULL) ? pitch1_feedback->angle : 0.0f;

        pitch2_feedback = motor_get_feedback(arm_task_get_motor(ARM_TASK_MACHINE_PITCH2));
        pitch2_target_motor_rad =
            (pitch2_feedback != OM_NULL) ? pitch2_feedback->angle : 0.0f;

        roll2_feedback = motor_get_feedback(arm_task_get_motor(ARM_TASK_MACHINE_ROLL2));
        roll2_target_rad =
            (roll2_feedback != OM_NULL) ? roll2_feedback->angle : 0.0f;

        pitch3_feedback = motor_get_feedback(arm_task_get_motor(ARM_TASK_MACHINE_PITCH3));
        pitch3_target_rad =
            (pitch3_feedback != OM_NULL) ? pitch3_feedback->angle : 0.0f;

        if (arm_task_get_roll3_feedback_angle_rad(context, &roll3_target_rad) != OM_TRUE)
        {
            roll3_target_rad = g_arm_pose_normal.machine_values[ARM_TASK_MACHINE_ROLL3];
        }

        grip_feedback = motor_get_feedback(arm_task_get_motor(ARM_TASK_MACHINE_GRIP));
        grip_target_rad =
            (grip_feedback != OM_NULL) ? grip_feedback->angle : g_arm_pose_normal.machine_values[ARM_TASK_MACHINE_GRIP];
    }

    if (arm_task_get_pitch2_zero_angle_rad(context, &pitch2_zero_angle_rad) != OM_TRUE)
    {
        pitch2_zero_angle_rad = pitch2_target_motor_rad;
    }

    /* 校准期保持当前整臂姿态：
     * - 不再把手臂拖到固定对齐姿态
     * - 等当前姿态稳定后，再以此姿态捕获 controller neutral 并接管
     */
    pose->machine_values[ARM_TASK_MACHINE_BIG_YAW] =
        big_yaw_target_rad - g_arm_pose_normal.machine_values[ARM_TASK_MACHINE_BIG_YAW];
    pose->machine_values[ARM_TASK_MACHINE_PITCH1] =
        (pitch1_target_motor_rad / APP_ARM_PITCH1_TARGET_RATIO) -
        g_arm_pose_normal.machine_values[ARM_TASK_MACHINE_PITCH1];
    pose->machine_values[ARM_TASK_MACHINE_PITCH2] =
        ((pitch2_target_motor_rad - pitch2_zero_angle_rad) / (-APP_ARM_PITCH2_GEAR_RATIO)) -
        g_arm_pose_normal.machine_values[ARM_TASK_MACHINE_PITCH2];
    pose->machine_values[ARM_TASK_MACHINE_ROLL2] =
        roll2_target_rad - g_arm_pose_normal.machine_values[ARM_TASK_MACHINE_ROLL2];
    pose->machine_values[ARM_TASK_MACHINE_PITCH3] =
        pitch3_target_rad - g_arm_pose_normal.machine_values[ARM_TASK_MACHINE_PITCH3];
    pose->machine_values[ARM_TASK_MACHINE_ROLL3] = roll3_target_rad;
    pose->machine_values[ARM_TASK_MACHINE_GRIP] =
        grip_target_rad - g_arm_pose_normal.machine_values[ARM_TASK_MACHINE_GRIP];
}

OmBool arm_task_custom_controller_alignment_reached(ArmTaskContext* context)
{
    ArmTaskMachinePose alignment_pose = {0};
    ArmTaskMotorTargets targets = {0};
    const MotorFeedback* big_yaw_feedback = OM_NULL;
    const MotorFeedback* pitch1_feedback = OM_NULL;
    const MotorFeedback* pitch2_feedback = OM_NULL;
    const MotorFeedback* roll2_feedback = OM_NULL;
    const MotorFeedback* pitch3_feedback = OM_NULL;
    float roll3_feedback_rad = 0.0f;
    float pitch2_zero_angle_rad = 0.0f;
    const float alignment_threshold_rad = 0.10471976f;
    float roll3_target_rad = 0.0f;

    if (context == OM_NULL)
    {
        return OM_FALSE;
    }

    big_yaw_feedback = motor_get_feedback(arm_task_get_motor(ARM_TASK_MACHINE_BIG_YAW));
    pitch1_feedback = motor_get_feedback(arm_task_get_motor(ARM_TASK_MACHINE_PITCH1));
    pitch2_feedback = motor_get_feedback(arm_task_get_motor(ARM_TASK_MACHINE_PITCH2));
    roll2_feedback = motor_get_feedback(arm_task_get_motor(ARM_TASK_MACHINE_ROLL2));
    pitch3_feedback = motor_get_feedback(arm_task_get_motor(ARM_TASK_MACHINE_PITCH3));
    if (arm_task_motor_online(arm_task_get_motor(ARM_TASK_MACHINE_BIG_YAW)) != OM_TRUE ||
        arm_task_motor_online(arm_task_get_motor(ARM_TASK_MACHINE_PITCH1)) != OM_TRUE ||
        arm_task_motor_online(arm_task_get_motor(ARM_TASK_MACHINE_PITCH2)) != OM_TRUE ||
        arm_task_motor_online(arm_task_get_motor(ARM_TASK_MACHINE_ROLL2)) != OM_TRUE ||
        arm_task_motor_online(arm_task_get_motor(ARM_TASK_MACHINE_PITCH3)) != OM_TRUE ||
        arm_task_roll3_online(context) != OM_TRUE)
    {
        return OM_FALSE;
    }

    arm_task_apply_custom_controller_alignment_pose(context, &alignment_pose);
    arm_task_resolve_motor_targets(context, &alignment_pose, &targets);

    if (arm_task_get_pitch2_zero_angle_rad(context, &pitch2_zero_angle_rad) != OM_TRUE)
    {
        return OM_FALSE;
    }

    if (math_utils_abs_float(targets.big_yaw_rad - big_yaw_feedback->angle) > alignment_threshold_rad ||
        math_utils_abs_float(targets.pitch1_rad - pitch1_feedback->angle) > alignment_threshold_rad ||
        math_utils_abs_float(targets.pitch2_rad - pitch2_feedback->angle) > alignment_threshold_rad ||
        math_utils_abs_float(targets.roll2_rad - roll2_feedback->angle) > alignment_threshold_rad ||
        math_utils_abs_float(targets.pitch3_rad - pitch3_feedback->angle) > alignment_threshold_rad)
    {
        return OM_FALSE;
    }

    if (arm_task_get_roll3_feedback_angle_rad(context, &roll3_feedback_rad) != OM_TRUE)
    {
        return OM_FALSE;
    }

    roll3_target_rad =
        math_utils_resolve_nearest_equivalent_rad(targets.roll3_rad, roll3_feedback_rad);
    return (math_utils_abs_float(roll3_target_rad - roll3_feedback_rad) <= alignment_threshold_rad)
               ? OM_TRUE
               : OM_FALSE;
}

void arm_task_capture_custom_controller_reference(
    ArmTaskContext* context,
    const ArmTaskCustomControllerSnapshot* controller_snapshot)
{
    uint32_t axis_index = 0u;
    float roll3_feedback_rad = 0.0f;
    const MotorFeedback* grip_feedback = OM_NULL;

    if (context == OM_NULL || controller_snapshot == OM_NULL)
    {
        return;
    }

    for (axis_index = 0u; axis_index < ARM_TASK_CUSTOM_CONTROLLER_AXIS_COUNT; axis_index++)
    {
        context->custom_controller_neutral_deg[axis_index] = controller_snapshot->angle_deg[axis_index];
        context->custom_controller_filtered_delta_deg[axis_index] = 0.0f;
    }
    context->flags &= ~ARM_TASK_FLAG_CUSTOM_FILTER_INIT;

    if ((context->flags & ARM_TASK_FLAG_SMOOTHED_TARGETS_INIT))
    {
        context->custom_controller_roll3_reference_rad =
            context->smoothed_targets.roll3_rad;
        context->custom_controller_grip_reference_rad =
            context->smoothed_targets.grip_rad;
    }
    else
    {
        if (arm_task_get_roll3_feedback_angle_rad(context, &roll3_feedback_rad) == OM_TRUE)
        {
            context->custom_controller_roll3_reference_rad =
                roll3_feedback_rad;
        }
        else
        {
            context->custom_controller_roll3_reference_rad =
                g_arm_pose_normal.machine_values[ARM_TASK_MACHINE_ROLL3];
        }

        grip_feedback = motor_get_feedback(arm_task_get_motor(ARM_TASK_MACHINE_GRIP));
        context->custom_controller_grip_reference_rad =
            (grip_feedback != OM_NULL) ? grip_feedback->angle : g_arm_pose_normal.machine_values[ARM_TASK_MACHINE_GRIP];
    }

    context->flags |= ARM_TASK_FLAG_CUSTOM_REF_CAPTURED;
}

void arm_task_update_custom_controller_reference_state(
    ArmTaskContext* context,
    OmBool custom_controller_mode_selected,
    OmBool custom_controller_active,
    const ArmTaskCustomControllerSnapshot* controller_snapshot)
{
    if (context == OM_NULL)
    {
        return;
    }

    if (custom_controller_mode_selected != OM_TRUE)
    {
        arm_task_reset_custom_controller_state(context);
        return;
    }

    if (custom_controller_active != OM_TRUE)
    {
        if ((context->flags & ARM_TASK_FLAG_CUSTOM_ALIGNMENT_DONE))
        {
            arm_task_reset_custom_controller_state(context);
        }
        return;
    }

    if (!(context->flags & ARM_TASK_FLAG_CUSTOM_WAS_ACTIVE) ||
        !(context->flags & ARM_TASK_FLAG_CUSTOM_REF_CAPTURED))
    {
        arm_task_capture_custom_controller_reference(context, controller_snapshot);
    }

    context->flags |= ARM_TASK_FLAG_CUSTOM_WAS_ACTIVE;
}

