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

const char* g_arm_task_motor_names[AT_MACHINE_COUNT] = {
    APP_MN_BIG_YAW,
    APP_MN_PITCH1,
    APP_MN_PITCH2,
    APP_MN_ROLL2,
    APP_MN_PITCH3,
    APP_MN_ROLL3,
    APP_MN_GRIP,
};
const uint8_t g_arm_task_motor_roles[AT_MACHINE_COUNT] = {
    APP_MR_BIG_YAW,
    APP_MR_PITCH1,
    APP_MR_PITCH2,
    APP_MR_ROLL2,
    APP_MR_PITCH3,
    APP_MR_ROLL3,
    APP_MR_GRIP,
};
PidController g_roll3_angle_pid = {0};
PidController g_roll3_speed_pid = {0};
Motor* g_arm_task_motor_cache[AT_MACHINE_COUNT] = {0};

OmBool arm_task_motor_present(ArmTaskMachineAxis axis)
{
    if (axis >= AT_MACHINE_COUNT)
    {
        return OM_FALSE;
    }

    return app_motor_role_is_present(g_arm_task_motor_roles[axis]);
}

OmBool arm_task_motor_allows(ArmTaskMachineAxis axis)
{
    if (axis >= AT_MACHINE_COUNT)
    {
        return OM_FALSE;
    }

    return app_motor_role_allows_control(g_arm_task_motor_roles[axis]);
}

MotorControlMode arm_task_profile_mode(ArmTaskMachineAxis axis)
{
    if (arm_task_motor_allows(axis) != OM_TRUE)
    {
        return MOTOR_CONTROL_MODE_DISABLED;
    }

    return (axis == AT_MACHINE_ROLL3) ?
               MOTOR_CONTROL_MODE_CURRENT :
               MOTOR_CONTROL_MODE_ANGLE;
}
void arm_task_drain_mode_snapshots(ArmTaskContext* context)
{
    ArmTaskModeSnapshot snapshot = {0};

    if (context == OM_NULL)
    {
        return;
    }

    while (tmpsc_receive(&context->mode_channel, &snapshot) == OM_OK)
    {
        context->latest_mode_snapshot = snapshot;
        context->flags |= AT_FLAG_MODE_SNAPSHOT_READY;
    }
}

void arm_task_drain_rc_snapshots(ArmTaskContext* context)
{
    InputRcSnapshot snapshot = {0};

    if (context == OM_NULL)
    {
        return;
    }

    while (task_pipe_channel_receive(&context->rc_channel, &snapshot, 0u) == OM_OK)
    {
        context->latest_rc_snapshot = snapshot;
        context->flags |= AT_FLAG_RC_SNAPSHOT_READY;
    }
}

void arm_task_drain_custom(ArmTaskContext* context)
{
    InputCustomSnapshot snapshot = {0};

    if (context == OM_NULL)
    {
        return;
    }

    while (task_pipe_channel_receive(&context->custom_channel, &snapshot, 0u) == OM_OK)
    {
        context->latest_custom_snapshot = snapshot;
    }
}

/* 每轮只读一次本地 latest-cache，正式输入不再从任何全局共享真源回读。 */
OmBool arm_task_load_snapshot(
    const ArmTaskContext* context,
    ArmTaskSnapshot* snapshot)
{
    if (context == OM_NULL || snapshot == OM_NULL || !(context->flags & AT_FLAG_MODE_SNAPSHOT_READY))
    {
        return OM_FALSE;
    }

    snapshot->arm_mode = (ArmTaskMode)context->latest_mode_snapshot.arm_mode;
    snapshot->grip_state = context->latest_mode_snapshot.grip_state;
    snapshot->ik_solver_mode = context->latest_mode_snapshot.ik_solver_mode;
    snapshot->ik_control_bank = context->latest_mode_snapshot.ik_control_bank;
    snapshot->chassis_mode =
        (ChassisMode)context->latest_mode_snapshot.preset_action.chassis_mode;
    snapshot->clamp_action =
        (ClampAction)context->latest_mode_snapshot.preset_action.clamp_action;
    snapshot->exchange_action =
        (ExchangeAction)context->latest_mode_snapshot.preset_action.exchange_action;
    snapshot->primary_turn_ore_flag =
        context->latest_mode_snapshot.preset_action.primary_turn_ore_flag;
    return OM_TRUE;
}

void arm_task_load_rc_snapshot(
    const ArmTaskContext* context,
    InputRcSnapshot* snapshot)
{
    if (context == OM_NULL || snapshot == OM_NULL)
    {
        return;
    }

    *snapshot = context->latest_rc_snapshot;
}

void arm_task_load_custom_snapshot(
    const ArmTaskContext* context,
    ArmCustomSnapshot* snapshot)
{
    uint32_t axis_index = 0u;

    if (context == OM_NULL || snapshot == OM_NULL)
    {
        return;
    }

    snapshot->online = context->latest_custom_snapshot.online;
    snapshot->work_mode = context->latest_custom_snapshot.work_mode;
    for (axis_index = 0u; axis_index < AT_CUSTOM_AXIS_COUNT; axis_index++)
    {
        snapshot->angle_deg[axis_index] =
            context->latest_custom_snapshot.angle_deg[axis_index];
    }
}

OmBool arm_task_snapshot_changed(const ArmTaskSnapshot* lhs, const ArmTaskSnapshot* rhs)
{
    if (lhs == OM_NULL || rhs == OM_NULL)
    {
        return OM_TRUE;
    }

    return (lhs->arm_mode != rhs->arm_mode || lhs->chassis_mode != rhs->chassis_mode ||
            lhs->clamp_action != rhs->clamp_action ||
            lhs->exchange_action != rhs->exchange_action ||
            lhs->primary_turn_ore_flag != rhs->primary_turn_ore_flag ||
            lhs->ik_solver_mode != rhs->ik_solver_mode ||
            lhs->ik_control_bank != rhs->ik_control_bank ||
            lhs->grip_state != rhs->grip_state)
               ? OM_TRUE
               : OM_FALSE;
}

OmBool arm_task_custom_active(
    const ArmTaskContext* context,
    const ArmTaskSnapshot* arm_snapshot,
    const ArmCustomSnapshot* controller_snapshot)
{
    if (context == OM_NULL || arm_snapshot == OM_NULL || controller_snapshot == OM_NULL)
    {
        return OM_FALSE;
    }

    return (arm_snapshot->arm_mode == AT_MODE_CUSTOM_TAKEOVER &&
            controller_snapshot->online != 0u &&
            controller_snapshot->work_mode == AT_CUSTOM_WORK_ENCODER)
               ? OM_TRUE
               : OM_FALSE;
}

OmBool arm_task_feedback_online(const MotorFeedback* feedback)
{
    return (feedback != OM_NULL && feedback->online == OM_TRUE) ? OM_TRUE : OM_FALSE;
}

OmBool arm_task_motor_online(const Motor* motor)
{
    if (motor == OM_NULL)
    {
        return OM_FALSE;
    }

    if (motor->config.vendor == MOTOR_VENDOR_GO8010)
    {
        return motor_is_feedback_recent(
                   motor,
                   AT_GO8010_RECENT_TIMEOUT_MS) == OM_TRUE
                   ? OM_TRUE
                   : OM_FALSE;
    }

    return arm_task_feedback_online(motor_get_feedback(motor));
}

OmBool arm_task_roll3_online(const ArmTaskContext* context)
{
    if (context == OM_NULL)
    {
        return OM_FALSE;
    }

    return (arm_task_motor_online(arm_task_get_motor(AT_MACHINE_ROLL3)) == OM_TRUE) ?
               OM_TRUE :
               OM_FALSE;
}

OmBool arm_task_roll3_feedback_rad(
    const ArmTaskContext* context,
    float* angle_rad)
{
    if (context == OM_NULL || angle_rad == OM_NULL || arm_task_get_motor(AT_MACHINE_ROLL3) == OM_NULL)
    {
        return OM_FALSE;
    }

    return motor_turn_rad(arm_task_get_motor(AT_MACHINE_ROLL3), angle_rad);
}

/* roll3 双环 PID 的通用初始化 helper。 */
OmRet arm_task_init_pid(
    PidController* pid,
    float kp,
    float ki,
    float kd,
    float output_limit,
    float integral_limit)
{
    if (pid == OM_NULL)
    {
        return OM_ERROR_NULL;
    }

    if (!pid_init(pid, PID_POSITIONAL_MODE, kp, ki, kd))
    {
        return OM_ERROR;
    }

    pid_set_output_limit(pid, -output_limit, output_limit);
    pid_set_integral_limit(pid, integral_limit);
    return OM_OK;
}

/* 当前 arm_task 只有 roll3 需要本地 PID，其余轴都直接走 motor 层 angle target。 */
OmRet arm_task_init_pids(void)
{
    OmRet ret = OM_OK;

    ret = arm_task_init_pid(
        &g_roll3_angle_pid,
        APP_AT_ROLL3_ANGLE_PID_KP,
        APP_AT_ROLL3_ANGLE_PID_KI,
        APP_AT_ROLL3_ANGLE_PID_KD,
        APP_AT_ROLL3_ANGLE_PID_OUT_MAX,
        APP_AT_ROLL3_ANGLE_PID_I_MAX);
    if (ret != OM_OK)
    {
        return ret;
    }

    return arm_task_init_pid(
        &g_roll3_speed_pid,
        APP_AT_ROLL3_SPEED_PID_KP,
        APP_AT_ROLL3_SPEED_PID_KI,
        APP_AT_ROLL3_SPEED_PID_KD,
        APP_AT_ROLL3_SPEED_PID_OUT_MAX,
        APP_AT_ROLL3_SPEED_PID_I_MAX);
}

OmRet arm_task_try_bind_motors(ArmTaskContext* context)
{
    uint32_t axis = 0u;

    if (context == OM_NULL)
    {
        return OM_ERROR_NULL;
    }

    context->flags &= ~AT_FLAG_MOTORS_BOUND;

    for (axis = 0u; axis < AT_MACHINE_COUNT; axis++)
    {
        if (arm_task_motor_present((ArmTaskMachineAxis)axis) != OM_TRUE)
        {
            g_arm_task_motor_cache[axis] = OM_NULL;
            continue;
        }

        g_arm_task_motor_cache[axis] = motor_find_by_name(g_arm_task_motor_names[axis]);
        if (g_arm_task_motor_cache[axis] == OM_NULL)
        {
            return OM_ERROR_NULL;
        }
    }

    for (axis = 0u; axis < AT_MACHINE_COUNT; axis++)
    {
        Motor* motor = g_arm_task_motor_cache[axis];

        if (motor == OM_NULL)
        {
            continue;
        }

        if (motor_set_control_mode(
                motor,
                arm_task_profile_mode((ArmTaskMachineAxis)axis)) != OM_OK)
        {
            return OM_ERROR;
        }
    }

    context->flags |= AT_FLAG_MOTORS_BOUND;
    return OM_OK;
}

OmRet arm_task_restore_modes(ArmTaskContext* context)
{
    uint32_t axis = 0u;

    if (context == OM_NULL || !(context->flags & AT_FLAG_MOTORS_BOUND))
    {
        return OM_ERROR_NULL;
    }

    for (axis = 0u; axis < AT_MACHINE_COUNT; axis++)
    {
        Motor* motor = g_arm_task_motor_cache[axis];

        if (motor == OM_NULL)
        {
            continue;
        }

        if (motor_set_control_mode(
                motor,
                arm_task_profile_mode((ArmTaskMachineAxis)axis)) != OM_OK)
        {
            return OM_ERROR;
        }
    }

    return OM_OK;
}

/* pitch2 的绝对位置零位由 GO8010 owner 在正式通信 bring-up 中锁存。
 * arm_task 只读这个基准，不再自己维护初始化事实。
 */
OmBool arm_task_pitch2_zero_rad(
    const ArmTaskContext* context,
    float* pitch2_zero_angle_rad)
{
    if (context == OM_NULL || pitch2_zero_angle_rad == OM_NULL ||
        arm_task_get_motor(AT_MACHINE_PITCH2) == OM_NULL)
    {
        return OM_FALSE;
    }

    return motor_zero_rad(arm_task_get_motor(AT_MACHINE_PITCH2), pitch2_zero_angle_rad);
}

OmBool arm_task_pitch2_feedback_rad(
    const ArmTaskContext* context,
    float* pitch2_joint_angle_rad)
{
    const MotorFeedback* pitch2_feedback = OM_NULL;
    float pitch2_zero_angle_rad = 0.0f;

    if (context == OM_NULL || pitch2_joint_angle_rad == OM_NULL)
    {
        return OM_FALSE;
    }

    pitch2_feedback = motor_get_feedback(arm_task_get_motor(AT_MACHINE_PITCH2));
    if (pitch2_feedback == OM_NULL)
    {
        return OM_FALSE;
    }

    if (arm_task_pitch2_zero_rad(context, &pitch2_zero_angle_rad) != OM_TRUE)
    {
        return OM_FALSE;
    }

    *pitch2_joint_angle_rad =
        (pitch2_zero_angle_rad - pitch2_feedback->angle) /
        APP_AT_PITCH2_GEAR_RATIO;
    return OM_TRUE;
}

OmBool arm_task_feedback_to_joint(
    const ArmTaskContext* context,
    ArmTaskMachineAxis axis,
    float feedback_angle_rad,
    float* machine_joint_rad)
{
    float pitch2_zero_angle_rad = 0.0f;

    if (context == OM_NULL || machine_joint_rad == OM_NULL)
    {
        return OM_FALSE;
    }

    switch (axis)
    {
    case AT_MACHINE_BIG_YAW:
        *machine_joint_rad =
            feedback_angle_rad - arm_task_joint_zero_rad(axis);
        return OM_TRUE;

    case AT_MACHINE_PITCH1:
        *machine_joint_rad =
            (feedback_angle_rad / APP_AT_PITCH1_TARGET_RATIO) -
            arm_task_joint_zero_rad(axis);
        return OM_TRUE;

    case AT_MACHINE_PITCH2:
        if (arm_task_pitch2_zero_rad(context, &pitch2_zero_angle_rad) != OM_TRUE)
        {
            pitch2_zero_angle_rad = feedback_angle_rad;
        }
        *machine_joint_rad =
            (feedback_angle_rad - pitch2_zero_angle_rad) / (-APP_AT_PITCH2_GEAR_RATIO);
        return OM_TRUE;

    case AT_MACHINE_ROLL2:
    case AT_MACHINE_PITCH3:
    case AT_MACHINE_GRIP:
        *machine_joint_rad =
            feedback_angle_rad - arm_task_joint_zero_rad(axis);
        return OM_TRUE;

    case AT_MACHINE_ROLL3:
        *machine_joint_rad =
            math_utils_resolve_rad(
                feedback_angle_rad - arm_task_joint_zero_rad(axis),
                0.0f);
        return OM_TRUE;

    default:
        return OM_FALSE;
    }
}

OmBool arm_task_joint_to_target(
    const ArmTaskContext* context,
    ArmTaskMachineAxis axis,
    float machine_joint_rad,
    float* motor_target_rad)
{
    float pitch2_zero_angle_rad = 0.0f;

    if (context == OM_NULL || motor_target_rad == OM_NULL)
    {
        return OM_FALSE;
    }

    switch (axis)
    {
    case AT_MACHINE_BIG_YAW:
    case AT_MACHINE_ROLL2:
    case AT_MACHINE_PITCH3:
    case AT_MACHINE_ROLL3:
    case AT_MACHINE_GRIP:
        *motor_target_rad =
            arm_task_get_motor_zero_angle(axis) + machine_joint_rad;
        return OM_TRUE;

    case AT_MACHINE_PITCH1:
        *motor_target_rad =
            APP_AT_PITCH1_TARGET_RATIO *
            (arm_task_joint_zero_rad(axis) + machine_joint_rad);
        return OM_TRUE;

    case AT_MACHINE_PITCH2:
        if (arm_task_pitch2_zero_rad(context, &pitch2_zero_angle_rad) != OM_TRUE)
        {
            const MotorFeedback* feedback = motor_get_feedback(arm_task_get_motor(axis));
            pitch2_zero_angle_rad = (feedback != OM_NULL) ? feedback->angle : 0.0f;
        }
        *motor_target_rad =
            pitch2_zero_angle_rad + machine_joint_rad * (-APP_AT_PITCH2_GEAR_RATIO);
        return OM_TRUE;

    default:
        return OM_FALSE;
    }
}

OmBool arm_task_get_ik_joint_vector(
    const ArmTaskContext* context,
    ArmIkJointVector* joint_vector)
{
    const MotorFeedback* feedback = OM_NULL;

    if (context == OM_NULL || joint_vector == OM_NULL)
    {
        return OM_FALSE;
    }

    memset(joint_vector, 0, sizeof(*joint_vector));

    feedback = motor_get_feedback(arm_task_get_motor(AT_MACHINE_BIG_YAW));
    joint_vector->joint_rad[AT_MACHINE_BIG_YAW] =
        (feedback != OM_NULL &&
         arm_task_feedback_to_joint(
             context,
             AT_MACHINE_BIG_YAW,
             feedback->angle,
             &joint_vector->joint_rad[AT_MACHINE_BIG_YAW]) == OM_TRUE)
            ? joint_vector->joint_rad[AT_MACHINE_BIG_YAW]
            : 0.0f;

    feedback = motor_get_feedback(arm_task_get_motor(AT_MACHINE_PITCH1));
    joint_vector->joint_rad[AT_MACHINE_PITCH1] =
        (feedback != OM_NULL &&
         arm_task_feedback_to_joint(
             context,
             AT_MACHINE_PITCH1,
             feedback->angle,
             &joint_vector->joint_rad[AT_MACHINE_PITCH1]) == OM_TRUE)
            ? joint_vector->joint_rad[AT_MACHINE_PITCH1]
            : 0.0f;

    feedback = motor_get_feedback(arm_task_get_motor(AT_MACHINE_PITCH2));
    if (feedback != OM_NULL)
    {
        (void)arm_task_feedback_to_joint(
            context,
            AT_MACHINE_PITCH2,
            feedback->angle,
            &joint_vector->joint_rad[AT_MACHINE_PITCH2]);
    }

    feedback = motor_get_feedback(arm_task_get_motor(AT_MACHINE_ROLL2));
    joint_vector->joint_rad[AT_MACHINE_ROLL2] =
        (feedback != OM_NULL &&
         arm_task_feedback_to_joint(
             context,
             AT_MACHINE_ROLL2,
             feedback->angle,
             &joint_vector->joint_rad[AT_MACHINE_ROLL2]) == OM_TRUE)
            ? joint_vector->joint_rad[AT_MACHINE_ROLL2]
            : 0.0f;

    feedback = motor_get_feedback(arm_task_get_motor(AT_MACHINE_PITCH3));
    joint_vector->joint_rad[AT_MACHINE_PITCH3] =
        (feedback != OM_NULL &&
         arm_task_feedback_to_joint(
             context,
             AT_MACHINE_PITCH3,
             feedback->angle,
             &joint_vector->joint_rad[AT_MACHINE_PITCH3]) == OM_TRUE)
            ? joint_vector->joint_rad[AT_MACHINE_PITCH3]
            : 0.0f;

    feedback = motor_get_feedback(arm_task_get_motor(AT_MACHINE_ROLL3));
    if (feedback != OM_NULL)
    {
        (void)arm_task_feedback_to_joint(
            context,
            AT_MACHINE_ROLL3,
            feedback->angle,
            &joint_vector->joint_rad[AT_MACHINE_ROLL3]);
    }

    return OM_TRUE;
}

void arm_task_pose_from_ik(
    const ArmIkJointVector* joint_vector,
    ArmTaskMachinePose* pose)
{
    if (joint_vector == OM_NULL || pose == OM_NULL)
    {
        return;
    }

    memset(pose, 0, sizeof(*pose));

    pose->machine_values[AT_MACHINE_BIG_YAW] =
        joint_vector->joint_rad[AT_MACHINE_BIG_YAW];
    pose->machine_values[AT_MACHINE_PITCH1] =
        joint_vector->joint_rad[AT_MACHINE_PITCH1];
    pose->machine_values[AT_MACHINE_PITCH2] =
        joint_vector->joint_rad[AT_MACHINE_PITCH2];
    pose->machine_values[AT_MACHINE_ROLL2] =
        joint_vector->joint_rad[AT_MACHINE_ROLL2];
    pose->machine_values[AT_MACHINE_PITCH3] =
        joint_vector->joint_rad[AT_MACHINE_PITCH3];
    pose->machine_values[AT_MACHINE_ROLL3] =
        joint_vector->joint_rad[AT_MACHINE_ROLL3];
    pose->machine_values[AT_MACHINE_GRIP] = 0.0f;
}

void arm_task_apply_angle_target(
    Motor* motor,
    float target_angle_rad,
    float kp,
    float kd,
    float torque_feedforward)
{
    if (motor == OM_NULL)
    {
        return;
    }

    (void)motor_set_angle(motor, target_angle_rad);
    (void)motor_set_speed(motor, 0.0f);
    (void)motor_set_position_gains(motor, kp, kd);
    (void)motor_set_torque_feedforward(motor, torque_feedforward);
    (void)motor_control_compute(motor);
}

void arm_task_apply_hold_target(
    Motor* motor,
    float kp,
    float kd)
{
    const MotorFeedback* feedback = OM_NULL;

    if (motor == OM_NULL)
    {
        return;
    }

    feedback = motor_get_feedback(motor);
    if (arm_task_motor_online(motor) != OM_TRUE)
    {
        arm_task_apply_angle_target(motor, 0.0f, 0.0f, 0.0f, 0.0f);
        return;
    }

    arm_task_apply_angle_target(motor, feedback->angle, kp, kd, 0.0f);
}

void arm_task_reset_axis_state(ArmTaskContext* context)
{
    if (context == OM_NULL)
    {
        return;
    }


    context->last_control_ms[AT_MACHINE_BIG_YAW] = 0u;
    context->last_control_ms[AT_MACHINE_PITCH1] = 0u;
    context->last_control_ms[AT_MACHINE_PITCH2] = 0u;
    context->last_control_ms[AT_MACHINE_ROLL2] = 0u;
    context->last_control_ms[AT_MACHINE_PITCH3] = 0u;
    context->last_control_ms[AT_MACHINE_ROLL3] = 0u;
    context->last_control_ms[AT_MACHINE_GRIP] = 0u;
    pid_reset(&g_roll3_angle_pid);
    pid_reset(&g_roll3_speed_pid);
}

OmBool arm_task_should_big_yaw(
    ArmTaskContext* context,
    OsalTimeMs now_ms)
{
    if (context == OM_NULL)
    {
        return OM_FALSE;
    }

    if (context->last_control_ms[AT_MACHINE_BIG_YAW] != 0u &&
        (uint32_t)(now_ms - context->last_control_ms[AT_MACHINE_BIG_YAW]) <
            AT_BIG_YAW_LOOP_MS)
    {
        return OM_FALSE;
    }

    context->last_control_ms[AT_MACHINE_BIG_YAW] = now_ms;
    return OM_TRUE;
}

OmBool arm_task_should_pitch1(
    ArmTaskContext* context,
    OsalTimeMs now_ms)
{
    if (context == OM_NULL)
    {
        return OM_FALSE;
    }

    if (context->last_control_ms[AT_MACHINE_PITCH1] != 0u &&
        (uint32_t)(now_ms - context->last_control_ms[AT_MACHINE_PITCH1]) <
            AT_PITCH1_LOOP_MS)
    {
        return OM_FALSE;
    }

    context->last_control_ms[AT_MACHINE_PITCH1] = now_ms;
    return OM_TRUE;
}

OmBool arm_task_should_pitch2(
    ArmTaskContext* context,
    OsalTimeMs now_ms)
{
    if (context == OM_NULL)
    {
        return OM_FALSE;
    }

    if (context->last_control_ms[AT_MACHINE_PITCH2] != 0u &&
        (uint32_t)(now_ms - context->last_control_ms[AT_MACHINE_PITCH2]) <
            AT_PITCH2_LOOP_MS)
    {
        return OM_FALSE;
    }

    context->last_control_ms[AT_MACHINE_PITCH2] = now_ms;
    return OM_TRUE;
}

OmBool arm_task_should_roll2(
    ArmTaskContext* context,
    OsalTimeMs now_ms)
{
    if (context == OM_NULL)
    {
        return OM_FALSE;
    }

    if (context->last_control_ms[AT_MACHINE_ROLL2] != 0u &&
        (uint32_t)(now_ms - context->last_control_ms[AT_MACHINE_ROLL2]) <
            AT_ROLL2_LOOP_MS)
    {
        return OM_FALSE;
    }

    context->last_control_ms[AT_MACHINE_ROLL2] = now_ms;
    return OM_TRUE;
}

OmBool arm_task_should_pitch3(
    ArmTaskContext* context,
    OsalTimeMs now_ms)
{
    if (context == OM_NULL)
    {
        return OM_FALSE;
    }

    if (context->last_control_ms[AT_MACHINE_PITCH3] != 0u &&
        (uint32_t)(now_ms - context->last_control_ms[AT_MACHINE_PITCH3]) <
            AT_PITCH3_LOOP_MS)
    {
        return OM_FALSE;
    }

    context->last_control_ms[AT_MACHINE_PITCH3] = now_ms;
    return OM_TRUE;
}

OmBool arm_task_should_roll3(
    ArmTaskContext* context,
    OsalTimeMs now_ms)
{
    if (context == OM_NULL)
    {
        return OM_FALSE;
    }

    if (context->last_control_ms[AT_MACHINE_ROLL3] != 0u &&
        (uint32_t)(now_ms - context->last_control_ms[AT_MACHINE_ROLL3]) <
            AT_ROLL3_LOOP_MS)
    {
        return OM_FALSE;
    }

    context->last_control_ms[AT_MACHINE_ROLL3] = now_ms;
    return OM_TRUE;
}

OmBool arm_task_pitch2_zero_ready(const ArmTaskContext* context)
{
    float zero_angle_rad = 0.0f;

    if (context == OM_NULL)
    {
        return OM_FALSE;
    }

    return arm_task_pitch2_zero_rad(context, &zero_angle_rad);
}

OmBool arm_task_should_grip(
    ArmTaskContext* context,
    OsalTimeMs now_ms)
{
    if (context == OM_NULL)
    {
        return OM_FALSE;
    }

    if (context->last_control_ms[AT_MACHINE_GRIP] != 0u &&
        (uint32_t)(now_ms - context->last_control_ms[AT_MACHINE_GRIP]) <
            AT_GRIP_LOOP_MS)
    {
        return OM_FALSE;
    }

    context->last_control_ms[AT_MACHINE_GRIP] = now_ms;
    return OM_TRUE;
}

void arm_task_apply_current(
    Motor* motor,
    float current)
{
    if (motor == OM_NULL)
    {
        return;
    }

    (void)motor_set_current(motor, current);
    (void)motor_control_compute(motor);
}

void arm_task_apply_roll3_target(
    ArmTaskContext* context,
    float target_angle_rad,
    float current_tick_s)
{
    const MotorFeedback* feedback = OM_NULL;
    float feedback_angle_rad = 0.0f;
    float nearest_target_angle_rad = 0.0f;
    float target_speed_rpm = 0.0f;
    float feedback_angle_deg = 0.0f;
    float target_angle_deg = 0.0f;
    float feedback_speed_rpm = 0.0f;
    float command_current = 0.0f;

    if (context == OM_NULL || arm_task_get_motor(AT_MACHINE_ROLL3) == OM_NULL)
    {
        return;
    }

    feedback = motor_get_feedback(arm_task_get_motor(AT_MACHINE_ROLL3));
    if (arm_task_feedback_online(feedback) != OM_TRUE ||
        arm_task_roll3_feedback_rad(context, &feedback_angle_rad) != OM_TRUE)
    {
        arm_task_apply_current(arm_task_get_motor(AT_MACHINE_ROLL3), 0.0f);
        return;
    }

    nearest_target_angle_rad =
        math_utils_resolve_rad(
            target_angle_rad,
            feedback_angle_rad);
    feedback_angle_deg = math_utils_rad_to_deg(feedback_angle_rad);
    target_angle_deg = math_utils_rad_to_deg(nearest_target_angle_rad);
    feedback_speed_rpm = math_utils_rad_per_s_to_rpm(feedback->speed);

    /* roll3/GM6020 延续旧工程已验证的单位语义：
     * - 外环：角度用 deg
     * - 内环：速度用 rpm
     * 这样现有 PID 量级才与 6020 raw current 输出匹配。
     */
    target_speed_rpm = pid_compute(
        &g_roll3_angle_pid,
        target_angle_deg,
        feedback_angle_deg,
        current_tick_s);
    command_current = pid_compute(
        &g_roll3_speed_pid,
        target_speed_rpm,
        feedback_speed_rpm,
        current_tick_s);
    arm_task_apply_current(
        arm_task_get_motor(AT_MACHINE_ROLL3),
        command_current);
}

OmBool arm_task_should_tx(
    ArmTaskContext* context,
    ModeTaskPhaseState operational_phase,
    OsalTimeMs now_ms)
{
    if (context == OM_NULL)
    {
        return OM_FALSE;
    }

    if (mct_is_operational_active() != OM_TRUE ||
        operational_phase == MT_OPERATIONAL_PHASE_RELEASE)
    {
        context->last_tx_request_ms = 0u;
        return OM_FALSE;
    }

    if (context->last_tx_request_ms != 0u &&
        (uint32_t)(now_ms - context->last_tx_request_ms) <
            AT_TX_REQUEST_PERIOD_MS)
    {
        return OM_FALSE;
    }

    context->last_tx_request_ms = now_ms;
    return OM_TRUE;
}

/* RELEASE 模式不推进动作，只把各角轴保持在当前反馈位置。 */
void arm_task_apply_release_output(ArmTaskContext* context)
{
    if (context == OM_NULL)
    {
        return;
    }

    arm_task_reset_axis_state(context);
    arm_task_apply_hold_target(
        arm_task_get_motor(AT_MACHINE_BIG_YAW),
        APP_AT_BIG_YAW_KP,
        APP_AT_BIG_YAW_KD);
    arm_task_apply_hold_target(
        arm_task_get_motor(AT_MACHINE_PITCH1),
        APP_AT_PITCH1_KP,
        APP_AT_PITCH1_KD);
    arm_task_apply_hold_target(
        arm_task_get_motor(AT_MACHINE_PITCH2),
        APP_AT_PITCH2_KP,
        APP_AT_PITCH2_KD);
    arm_task_apply_hold_target(
        arm_task_get_motor(AT_MACHINE_ROLL2),
        APP_AT_ROLL2_KP,
        APP_AT_ROLL2_KD);
    arm_task_apply_hold_target(
        arm_task_get_motor(AT_MACHINE_PITCH3),
        APP_AT_PITCH3_KP,
        APP_AT_PITCH3_KD);
    arm_task_apply_current(arm_task_get_motor(AT_MACHINE_ROLL3), 0.0f);
    arm_task_apply_hold_target(
        arm_task_get_motor(AT_MACHINE_GRIP),
        APP_AT_GRIP_KP,
        APP_AT_GRIP_KD);
}

/* 带时间窗的动作从“共享控制事实发生变化”那一刻重新计时。 */
void arm_task_update_command_timer(ArmTaskContext* context, const ArmTaskSnapshot* snapshot)
{
    if (context == OM_NULL || snapshot == OM_NULL)
    {
        return;
    }

    if (!(context->flags & AT_FLAG_SNAPSHOT_INITIALIZED) ||
        arm_task_snapshot_changed(&context->last_snapshot, snapshot) == OM_TRUE)
    {
        context->last_snapshot = *snapshot;
        context->command_since_ms = osal_time_now_monotonic();
        context->flags |= AT_FLAG_SNAPSHOT_INITIALIZED;
    }
}

static void arm_task_reset_ik_target(ArmTaskContext* context)
{
    if (context == OM_NULL)
    {
        return;
    }

    memset(&context->ik_target_pose, 0, sizeof(context->ik_target_pose));
    memset(&context->last_ik_solved_joint_vector, 0, sizeof(context->last_ik_solved_joint_vector));
    context->last_ik_solve_ms = 0u;
    context->flags &= ~AT_FLAG_IK_TARGET_POSE_READY;
    context->flags &= ~AT_FLAG_IK_SOLUTION_READY;
}

static OmBool arm_task_capture_ik_target(ArmTaskContext* context)
{
    ArmIkJointVector joint_vector = {0};

    if (context == OM_NULL)
    {
        return OM_FALSE;
    }

    if (arm_task_get_ik_joint_vector(context, &joint_vector) != OM_TRUE)
    {
        return OM_FALSE;
    }

    if (arm_kinematics_forward(&joint_vector, &context->ik_target_pose) != OM_OK)
    {
        return OM_FALSE;
    }

    context->last_ik_solved_joint_vector = joint_vector;
    context->last_ik_solve_ms = 0u;
    context->flags |= AT_FLAG_IK_TARGET_POSE_READY;
    context->flags |= AT_FLAG_IK_SOLUTION_READY;
    return OM_TRUE;
}

static void arm_task_apply_grip_state(
    const ArmTaskSnapshot* snapshot,
    ArmTaskMachinePose* pose)
{
    const float grip_target_rad =
        (snapshot != OM_NULL && snapshot->grip_state == MT_GRIP_CLOSED)
            ? APP_AT_GRIP_CLOSED_TARGET_RAD
            : APP_AT_GRIP_OPEN_TARGET_RAD;

    if (pose == OM_NULL)
    {
        return;
    }

    pose->machine_values[AT_MACHINE_GRIP] = grip_target_rad;
}

static void arm_task_integrate_ik(
    ArmTaskContext* context,
    const ArmTaskSnapshot* snapshot,
    const InputRcSnapshot* rc_snapshot,
    float dt_s)
{
    const float x_speed_m_per_s =
        APP_AT_RC_IK_POS_X_MM_PER_S / 1000.0f;
    const float y_speed_m_per_s =
        APP_AT_RC_IK_POS_Y_MM_PER_S / 1000.0f;
    const float z_speed_m_per_s =
        APP_AT_RC_IK_POS_Z_MM_PER_S / 1000.0f;
    const float roll_speed_rad_per_s =
        math_utils_deg_to_rad(APP_AT_RC_IK_ROLL_DEG_PER_S);
    const float pitch_speed_rad_per_s =
        math_utils_deg_to_rad(APP_AT_RC_IK_PITCH_DEG_PER_S);
    const float yaw_speed_rad_per_s =
        math_utils_deg_to_rad(APP_AT_RC_IK_YAW_DEG_PER_S);
    const float ch1_norm = ((float)rc_snapshot->ch1) / APP_RC_RESOLUTION;
    const float ch2_norm = ((float)rc_snapshot->ch2) / APP_RC_RESOLUTION;
    const float ch4_norm = ((float)rc_snapshot->ch4) / APP_RC_RESOLUTION;

    if (context == OM_NULL || snapshot == OM_NULL || rc_snapshot == OM_NULL ||
        !(context->flags & AT_FLAG_IK_TARGET_POSE_READY))
    {
        return;
    }

    if (snapshot->ik_control_bank == MT_IK_BANK_POS_XYZ)
    {
        context->ik_target_pose.position_m[0] += ch2_norm * x_speed_m_per_s * dt_s;
        context->ik_target_pose.position_m[1] += -ch1_norm * y_speed_m_per_s * dt_s;
        context->ik_target_pose.position_m[2] += ch4_norm * z_speed_m_per_s * dt_s;
    }
    else
    {
        context->ik_target_pose.orientation_rpy_rad[0] =
            math_utils_resolve_rad(
                context->ik_target_pose.orientation_rpy_rad[0] +
                    ch2_norm * roll_speed_rad_per_s * dt_s,
                0.0f);
        context->ik_target_pose.orientation_rpy_rad[1] =
            math_utils_resolve_rad(
                context->ik_target_pose.orientation_rpy_rad[1] +
                    ch4_norm * pitch_speed_rad_per_s * dt_s,
                0.0f);
        context->ik_target_pose.orientation_rpy_rad[2] =
            math_utils_resolve_rad(
                context->ik_target_pose.orientation_rpy_rad[2] +
                    -ch1_norm * yaw_speed_rad_per_s * dt_s,
                0.0f);
    }
}

static void arm_task_resolve_ik(
    ArmTaskContext* context,
    const ArmTaskSnapshot* snapshot,
    const InputRcSnapshot* rc_snapshot,
    float current_tick_s,
    OsalTimeMs now_ms,
    ArmTaskMachinePose* pose)
{
    ArmIkJointVector reference_joint_vector = {0};
    ArmIkJointVector solved_joint_vector = {0};
    ArmIkPoseErr pose_error_snapshot = {0};
    ArmIkSolveDiag solve_debug_snapshot = {0};
    OmRet ret = OM_ERROR;
    OmBool should_run_solver = OM_TRUE;

    if (context == OM_NULL || snapshot == OM_NULL || rc_snapshot == OM_NULL || pose == OM_NULL)
    {
        return;
    }

    if (!(context->flags & AT_FLAG_IK_TARGET_POSE_READY))
    {
        if (arm_task_capture_ik_target(context) != OM_TRUE)
        {
            arm_task_assign_pose(pose, &g_arm_pose_zero);
            return;
        }
    }

    if (rc_snapshot->sw1 == RC_SWITCH_DN)
    {
        arm_task_integrate_ik(context, snapshot, rc_snapshot, current_tick_s);
    }

    if (arm_task_get_ik_joint_vector(context, &reference_joint_vector) != OM_TRUE)
    {
        arm_task_assign_pose(pose, &g_arm_pose_zero);
        return;
    }

    if ((context->flags & AT_FLAG_IK_SOLUTION_READY) != 0u &&
        context->last_ik_solve_ms != 0u &&
        (uint32_t)(now_ms - context->last_ik_solve_ms) < APP_AT_IK_SOLVER_PERIOD_MS)
    {
        should_run_solver = OM_FALSE;
    }

    if (should_run_solver != OM_TRUE)
    {
        arm_task_pose_from_ik(&context->last_ik_solved_joint_vector, pose);
        return;
    }

    if (snapshot->ik_solver_mode == MT_IK_SOLVER_POSITION_PRIORITY)
    {
        ret = aik_inverse_pos_local(
            &context->ik_target_pose,
            &reference_joint_vector,
            &solved_joint_vector,
            &pose_error_snapshot,
            &solve_debug_snapshot);
    }
    else
    {
        ret = aik_inverse_full_local(
            &context->ik_target_pose,
            &reference_joint_vector,
            &solved_joint_vector,
            &pose_error_snapshot,
            &solve_debug_snapshot);
    }

    if (ret == OM_OK)
    {
        context->last_ik_solved_joint_vector = solved_joint_vector;
        context->last_ik_solve_ms = now_ms;
        context->flags |= AT_FLAG_IK_SOLUTION_READY;
        arm_task_pose_from_ik(&context->last_ik_solved_joint_vector, pose);
    }
    else
    {
        context->last_ik_solved_joint_vector = reference_joint_vector;
        context->last_ik_solve_ms = now_ms;
        context->flags |= AT_FLAG_IK_SOLUTION_READY;
        arm_task_pose_from_ik(&context->last_ik_solved_joint_vector, pose);
    }
}

/* 机械臂控制主循环：
 * 读快照 -> 生成姿态表 -> 映射电机目标 -> 下发到 motor 抽象层 -> 发布 TX 请求。
 */
void arm_task_run_once(ArmTaskContext* context)
{
    ArmTaskSnapshot snapshot = {0};
    InputRcSnapshot rc_snapshot = {0};
    ArmCustomSnapshot controller_snapshot = {0};
    ArmTaskMachinePose pose = {0};
    ArmTaskMotorTargets targets = {0};
    const float current_tick_s = ((float)AT_PERIOD_MS) / 1000.0f;
    const OsalTimeMs now_ms = osal_time_now_monotonic();
    OsalTimeMs elapsed_ms = 0u;
    float pitch1_torque_ff = 0.0f;
    float pitch2_torque_ff = 0.0f;
    float roll2_torque_ff = 0.0f;
    float pitch3_torque_ff = 0.0f;
    OmBool custom_mode_selected = OM_FALSE;
    OmBool custom_input_ready = OM_FALSE;
    OmBool custom_active = OM_FALSE;
    OmBool ik_mode_selected = OM_FALSE;
    OmBool ik_mode_just_entered = OM_FALSE;

    if (context == OM_NULL)
    {
        return;
    }

    arm_task_drain_mode_snapshots(context);
    arm_task_drain_rc_snapshots(context);
    arm_task_drain_custom(context);
    if (arm_task_load_snapshot(context, &snapshot) != OM_TRUE)
    {
        return;
    }
    arm_task_load_rc_snapshot(context, &rc_snapshot);
    arm_task_load_custom_snapshot(context, &controller_snapshot);

    /* 确保电机已绑定，如果未绑定则尝试绑定 */
    if (!(context->flags & AT_FLAG_MOTORS_BOUND))
    {
        if (arm_task_try_bind_motors(context) != OM_OK)
        {
            return;
        }
    }
    if (mct_is_operational_active() != OM_TRUE)
    {
        context->flags &= ~AT_FLAG_CONTROL_MODES_ARMED;
    }
    else if (!(context->flags & AT_FLAG_CONTROL_MODES_ARMED))
    {
        if (arm_task_restore_modes(context) != OM_OK)
        {
            return;
        }
        context->flags |= AT_FLAG_CONTROL_MODES_ARMED;
    }

    /* 判断自定义控制器模式是否选中、输入是否就绪、是否请求强制接管 */
    custom_mode_selected =
        (snapshot.arm_mode == AT_MODE_CUSTOM_TAKEOVER)
            ? OM_TRUE
            : OM_FALSE;
    custom_input_ready =
        (controller_snapshot.online != 0u &&
         controller_snapshot.work_mode == AT_CUSTOM_WORK_ENCODER)
            ? OM_TRUE
            : OM_FALSE;
    ik_mode_selected =
        (snapshot.arm_mode == AT_MODE_RC_IK)
            ? OM_TRUE
            : OM_FALSE;
    ik_mode_just_entered =
        (ik_mode_selected == OM_TRUE &&
         (!(context->flags & AT_FLAG_SNAPSHOT_INITIALIZED) ||
          context->last_snapshot.arm_mode != snapshot.arm_mode))
            ? OM_TRUE
            : OM_FALSE;

    /* 检查自定义控制器是否处于主动接管状态 */
    custom_active =
        arm_task_custom_active(context, &snapshot, &controller_snapshot);
    g_arm_task_custom_align_done_dbg =
        (custom_active == OM_TRUE) ? 1u : 0u;
    
    /* 更新自定义控制器参考状态 */
    arm_task_update_custom(
        context,
        custom_mode_selected,
        custom_active,
        &controller_snapshot);
    
    /* 更新命令计时器 */
    arm_task_update_command_timer(context, &snapshot);

    if (ik_mode_selected == OM_TRUE && ik_mode_just_entered == OM_TRUE)
    {
        (void)arm_task_capture_ik_target(context);
    }
    else if (ik_mode_selected != OM_TRUE)
    {
        arm_task_reset_ik_target(context);
    }
    
    /* 机械臂目标链不应被单个轴的在线状态整体拖死。
     * 这里先继续推进共享姿态 -> 电机目标 -> 平滑目标，
     * 再在各轴输出点各自判断 online / hold。
     */
    if (snapshot.arm_mode == AT_MODE_RELEASE)
    {
        /* RELEASE 会让机械臂失力。重新回到可控模式时，必须从当前反馈重新建目标：
         * - 清掉动作计时，避免时间窗继续沿用上一次动作
         * - 清掉平滑目标，避免 first tick 先冲向旧目标再回 normal
         * - 清掉自定义控制器接管状态，避免 release 前后的参考残留
         */
        context->flags &= ~AT_FLAG_SNAPSHOT_INITIALIZED;
        arm_task_clear_smoothed(context);
        arm_task_reset_custom_state(context);

        /* 底盘释放模式下，应用释放输出 */
        arm_task_apply_release_output(context);
    }
    else
    {
        /* 非 RELEASE：根据相位和运动模式生成目标姿态。 */
        elapsed_ms = now_ms - context->command_since_ms;

        if (snapshot.arm_mode == AT_MODE_NORMAL)
        {
            arm_task_assign_pose(&pose, &g_arm_pose_zero);
        }
        else if (snapshot.arm_mode == AT_MODE_PRESET_ACTION)
        {
            clamp_angle_handle(&snapshot, elapsed_ms, &pose);
        }
        else if (snapshot.arm_mode == AT_MODE_CUSTOM_TAKEOVER)
        {
            if (custom_input_ready == OM_TRUE &&
                custom_active == OM_TRUE)
            {
                arm_task_apply_custom_pose(context, &controller_snapshot, &pose);
            }
            else
            {
                arm_task_assign_pose(&pose, &g_arm_pose_zero);
            }
        }
        else if (snapshot.arm_mode == AT_MODE_RC_IK)
        {
            arm_task_resolve_ik(
                context,
                &snapshot,
                &rc_snapshot,
                current_tick_s,
                now_ms,
                &pose);
        }
        else
        {
            arm_task_assign_pose(&pose, &g_arm_pose_zero);
        }

        arm_task_apply_grip_state(&snapshot, &pose);
        
        /* 将机器姿态解析为各电机的目标值 */
        arm_task_resolve_targets(context, &pose, &targets);
        /* 对电机目标进行平滑滤波处理 */
        arm_task_update_smoothed(context, &targets, current_tick_s);
        arm_task_gravity_feedforward(
            context,
            &pitch1_torque_ff,
            &pitch2_torque_ff,
            &roll2_torque_ff,
            &pitch3_torque_ff);
        if (arm_task_motor_allows(AT_MACHINE_BIG_YAW) == OM_TRUE &&
            arm_task_should_big_yaw(context, now_ms) == OM_TRUE)
        {
            if (arm_task_motor_online(arm_task_get_motor(AT_MACHINE_BIG_YAW)) == OM_TRUE)
            {
                arm_task_apply_angle_target(
                    arm_task_get_motor(AT_MACHINE_BIG_YAW),
                    context->smoothed_targets.big_yaw_rad,
                    APP_AT_BIG_YAW_KP,
                    APP_AT_BIG_YAW_KD,
                    0.0f);
            }
            else
            {
                arm_task_apply_hold_target(
                    arm_task_get_motor(AT_MACHINE_BIG_YAW),
                    APP_AT_BIG_YAW_KP,
                    APP_AT_BIG_YAW_KD);
            }
        }
        if (arm_task_motor_allows(AT_MACHINE_PITCH1) == OM_TRUE &&
            arm_task_should_pitch1(context, now_ms) == OM_TRUE)
        {
            if (arm_task_motor_online(arm_task_get_motor(AT_MACHINE_PITCH1)) == OM_TRUE)
            {
                arm_task_apply_angle_target(
                    arm_task_get_motor(AT_MACHINE_PITCH1),
                    context->smoothed_targets.pitch1_rad,
                    APP_AT_PITCH1_KP,
                    APP_AT_PITCH1_KD,
                    pitch1_torque_ff);
            }
            else
            {
                arm_task_apply_hold_target(
                    arm_task_get_motor(AT_MACHINE_PITCH1),
                    APP_AT_PITCH1_KP,
                    APP_AT_PITCH1_KD);
            }
        }
        if (arm_task_motor_allows(AT_MACHINE_PITCH2) == OM_TRUE &&
            arm_task_should_pitch2(context, now_ms) == OM_TRUE)
        {
            if (arm_task_motor_online(arm_task_get_motor(AT_MACHINE_PITCH2)) == OM_TRUE &&
                arm_task_pitch2_zero_ready(context) == OM_TRUE)
            {
                arm_task_apply_angle_target(
                    arm_task_get_motor(AT_MACHINE_PITCH2),
                    context->smoothed_targets.pitch2_rad,
                    APP_AT_PITCH2_KP,
                    APP_AT_PITCH2_KD,
                    pitch2_torque_ff);
            }
            else
            {
                arm_task_apply_hold_target(
                    arm_task_get_motor(AT_MACHINE_PITCH2),
                    APP_AT_PITCH2_KP,
                    APP_AT_PITCH2_KD);
            }
        }
        if (arm_task_motor_allows(AT_MACHINE_ROLL2) == OM_TRUE &&
            arm_task_should_roll2(context, now_ms) == OM_TRUE)
        {
            if (arm_task_motor_online(arm_task_get_motor(AT_MACHINE_ROLL2)) == OM_TRUE)
            {
                arm_task_apply_angle_target(
                    arm_task_get_motor(AT_MACHINE_ROLL2),
                    context->smoothed_targets.roll2_rad,
                    APP_AT_ROLL2_KP,
                    APP_AT_ROLL2_KD,
                    roll2_torque_ff);
            }
            else
            {
                arm_task_apply_hold_target(
                    arm_task_get_motor(AT_MACHINE_ROLL2),
                    APP_AT_ROLL2_KP,
                    APP_AT_ROLL2_KD);
            }
        }
        if (arm_task_motor_allows(AT_MACHINE_PITCH3) == OM_TRUE &&
            arm_task_should_pitch3(context, now_ms) == OM_TRUE)
        {
            if (arm_task_motor_online(arm_task_get_motor(AT_MACHINE_PITCH3)) == OM_TRUE)
            {
                arm_task_apply_angle_target(
                    arm_task_get_motor(AT_MACHINE_PITCH3),
                    context->smoothed_targets.pitch3_rad,
                    APP_AT_PITCH3_KP,
                    APP_AT_PITCH3_KD,
                    pitch3_torque_ff);
            }
            else
            {
                arm_task_apply_hold_target(
                    arm_task_get_motor(AT_MACHINE_PITCH3),
                    APP_AT_PITCH3_KP,
                    APP_AT_PITCH3_KD);
            }
        }
        if (arm_task_motor_allows(AT_MACHINE_ROLL3) == OM_TRUE &&
            arm_task_should_roll3(context, now_ms) == OM_TRUE)
        {
            arm_task_apply_roll3_target(
                context,
                context->smoothed_targets.roll3_rad,
                current_tick_s);
        }
        if (arm_task_motor_allows(AT_MACHINE_GRIP) == OM_TRUE &&
            arm_task_should_grip(context, now_ms) == OM_TRUE)
        {
            if (arm_task_motor_online(arm_task_get_motor(AT_MACHINE_GRIP)) == OM_TRUE)
            {
                arm_task_apply_angle_target(
                    arm_task_get_motor(AT_MACHINE_GRIP),
                    context->smoothed_targets.grip_rad,
                    APP_AT_GRIP_KP,
                    APP_AT_GRIP_KD,
                    0.0f);
            }
            else
            {
                arm_task_apply_hold_target(
                    arm_task_get_motor(AT_MACHINE_GRIP),
                    APP_AT_GRIP_KP,
                    APP_AT_GRIP_KD);
            }
        }
    }

    if (arm_task_should_tx(
            context,
            (snapshot.arm_mode == AT_MODE_RELEASE)
                ? MT_OPERATIONAL_PHASE_RELEASE
                : MT_OPERATIONAL_PHASE_FORMAL,
            now_ms) != OM_TRUE)
    {
        return;
    }

    (void)motor_tx_dispatch_submit(MOTOR_TX_SOURCE_ARM);
}

    /* 正式输入已经改走通道和 owner latest-cache；这里固定节拍运行。 */
