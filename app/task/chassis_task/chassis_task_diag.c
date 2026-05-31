/* chassis_task 观测接口实现。
 * 从 chassis_task.c 抽离，通过 chassis_task_internal.h 访问运行时上下文。
 */

#include "task/chassis_task/chassis_task_diag.h"
#include "task/chassis_task/chassis_task_internal.h"
#include "function/math_utils/math_utils.h"

OmBool chassis_task_get_debug_snapshot(
    float wheel_feedback_rpm[CHASSIS_TASK_WHEEL_COUNT],
    float wheel_command_current[CHASSIS_TASK_WHEEL_COUNT],
    float leg_feedback_deg[CHASSIS_TASK_LEG_COUNT],
    float leg_command_current[CHASSIS_TASK_LEG_COUNT])
{
    uint32_t index = 0u;

    if (g_chassis_task_owner_context == OM_NULL ||
        wheel_feedback_rpm == OM_NULL ||
        wheel_command_current == OM_NULL ||
        leg_feedback_deg == OM_NULL ||
        leg_command_current == OM_NULL)
    {
        return OM_FALSE;
    }

    /* 四轮：反馈转速（rpm）、命令电流 */
    for (index = 0u; index < CHASSIS_TASK_WHEEL_COUNT; index++)
    {
        const MotorFeedback* feedback = motor_get_feedback(chassis_task_get_wheel_motor(index));
        wheel_feedback_rpm[index] =
            (feedback != OM_NULL) ? math_utils_rad_per_s_to_rpm(feedback->speed) : 0.0f;
        wheel_command_current[index] =
            motor_get_output(chassis_task_get_wheel_motor(index));
    }

    /* 两腿：反馈角度（deg）、命令电流 */
    for (index = 0u; index < CHASSIS_TASK_LEG_COUNT; index++)
    {
        const MotorFeedback* feedback = motor_get_feedback(chassis_task_get_leg_motor(index));
        leg_feedback_deg[index] =
            (feedback != OM_NULL) ? math_utils_normalize_deg(math_utils_rad_to_deg(feedback->angle)) : 0.0f;
        leg_command_current[index] =
            motor_get_output(chassis_task_get_leg_motor(index));
    }

    return OM_TRUE;
}

OmBool chassis_task_get_debug_chassis_mode(
    uint8_t* chassis_mode)
{
    if (g_chassis_task_owner_context == OM_NULL || chassis_mode == OM_NULL)
    {
        return OM_FALSE;
    }

    *chassis_mode = g_chassis_task_owner_context->latest_mode_snapshot.chassis_mode;
    return ((g_chassis_task_owner_context->flags & CHASSIS_TASK_FLAG_MODE_SNAPSHOT_READY)) ? OM_TRUE : OM_FALSE;
}
