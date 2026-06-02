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

const char* g_arm_task_motor_names[ARM_TASK_MACHINE_COUNT] = {
    APP_MOTOR_NAME_BIG_YAW,
    APP_MOTOR_NAME_PITCH1,
    APP_MOTOR_NAME_PITCH2,
    APP_MOTOR_NAME_ROLL2,
    APP_MOTOR_NAME_PITCH3,
    APP_MOTOR_NAME_ROLL3,
    APP_MOTOR_NAME_GRIP,
};
const uint8_t g_arm_task_motor_roles[ARM_TASK_MACHINE_COUNT] = {
    APP_MOTOR_ROLE_BIG_YAW,
    APP_MOTOR_ROLE_PITCH1,
    APP_MOTOR_ROLE_PITCH2,
    APP_MOTOR_ROLE_ROLL2,
    APP_MOTOR_ROLE_PITCH3,
    APP_MOTOR_ROLE_ROLL3,
    APP_MOTOR_ROLE_GRIP,
};
PidController g_roll3_angle_pid = {0};
PidController g_roll3_speed_pid = {0};
Motor* g_arm_task_motor_cache[ARM_TASK_MACHINE_COUNT] = {0};

OmBool arm_task_motor_profile_is_present(ArmTaskMachineAxis axis)
{
    if (axis >= ARM_TASK_MACHINE_COUNT)
    {
        return OM_FALSE;
    }

    return app_motor_profile_is_present(g_arm_task_motor_roles[axis]);
}

OmBool arm_task_motor_profile_allows_control(ArmTaskMachineAxis axis)
{
    if (axis >= ARM_TASK_MACHINE_COUNT)
    {
        return OM_FALSE;
    }

    return app_motor_profile_allows_control(g_arm_task_motor_roles[axis]);
}

MotorControlMode arm_task_get_profile_control_mode(ArmTaskMachineAxis axis)
{
    if (arm_task_motor_profile_allows_control(axis) != OM_TRUE)
    {
        return MOTOR_CONTROL_MODE_DISABLED;
    }

    return (axis == ARM_TASK_MACHINE_ROLL3) ?
               MOTOR_CONTROL_MODE_CURRENT :
               MOTOR_CONTROL_MODE_ANGLE;
}
void arm_task_drain_mode_snapshots(ArmTaskContext* context)
{
    ModeTaskControlSnapshot snapshot = {0};

    if (context == OM_NULL)
    {
        return;
    }

    while (task_mpsc_channel_receive_nonblocking(&context->mode_channel, &snapshot) == OM_OK)
    {
        context->latest_mode_snapshot = snapshot;
        context->flags |= ARM_TASK_FLAG_MODE_SNAPSHOT_READY;
    }
}

void arm_task_drain_custom_controller_snapshots(ArmTaskContext* context)
{
    DpCustomControllerSnapshot snapshot = {0};

    if (context == OM_NULL)
    {
        return;
    }

    while (task_pipe_channel_receive(&context->custom_controller_channel, &snapshot, 0u) == OM_OK)
    {
        context->latest_custom_controller_snapshot = snapshot;
    }
}

/* 每轮只读一次本地 latest-cache，正式输入不再从 DataPool 取。 */
OmBool arm_task_load_snapshot(
    const ArmTaskContext* context,
    ArmTaskSnapshot* snapshot)
{
    if (context == OM_NULL || snapshot == OM_NULL || !(context->flags & ARM_TASK_FLAG_MODE_SNAPSHOT_READY))
    {
        return OM_FALSE;
    }

    snapshot->chassis_mode = (ChassisMode)context->latest_mode_snapshot.chassis_mode;
    snapshot->clamp_action = (ClampAction)context->latest_mode_snapshot.clamp_action;
    snapshot->exchange_action = (ExchangeAction)context->latest_mode_snapshot.exchange_action;
    snapshot->primary_turn_ore_flag = context->latest_mode_snapshot.primary_turn_ore_flag;
    snapshot->custom_controller_force_takeover_flag =
        context->latest_mode_snapshot.custom_controller_force_takeover_flag;
    return OM_TRUE;
}

void arm_task_load_custom_controller_snapshot(
    const ArmTaskContext* context,
    ArmTaskCustomControllerSnapshot* snapshot)
{
    uint32_t axis_index = 0u;

    if (context == OM_NULL || snapshot == OM_NULL)
    {
        return;
    }

    snapshot->online = context->latest_custom_controller_snapshot.online;
    snapshot->work_mode = context->latest_custom_controller_snapshot.work_mode;
    for (axis_index = 0u; axis_index < ARM_TASK_CUSTOM_CONTROLLER_AXIS_COUNT; axis_index++)
    {
        snapshot->angle_deg[axis_index] =
            context->latest_custom_controller_snapshot.angle_deg[axis_index];
    }
}

OmBool arm_task_snapshot_changed(const ArmTaskSnapshot* lhs, const ArmTaskSnapshot* rhs)
{
    if (lhs == OM_NULL || rhs == OM_NULL)
    {
        return OM_TRUE;
    }

    return (lhs->chassis_mode != rhs->chassis_mode || lhs->clamp_action != rhs->clamp_action ||
            lhs->exchange_action != rhs->exchange_action ||
            lhs->primary_turn_ore_flag != rhs->primary_turn_ore_flag ||
            lhs->custom_controller_force_takeover_flag != rhs->custom_controller_force_takeover_flag)
               ? OM_TRUE
               : OM_FALSE;
}

OmBool arm_task_custom_controller_takeover_active(
    const ArmTaskContext* context,
    const ArmTaskSnapshot* arm_snapshot,
    const ArmTaskCustomControllerSnapshot* controller_snapshot)
{
    if (context == OM_NULL || arm_snapshot == OM_NULL || controller_snapshot == OM_NULL)
    {
        return OM_FALSE;
    }

    return (arm_snapshot->chassis_mode == MODE_CHASSIS_CUSTOM_CONTROLLER_NORMAL &&
            (context->flags & ARM_TASK_FLAG_CUSTOM_ALIGNMENT_DONE) &&
            controller_snapshot->online != 0u &&
            controller_snapshot->work_mode == ARM_TASK_CUSTOM_CONTROLLER_WORK_MODE_ENCODER)
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
                   ARM_TASK_GO8010_RECENT_TIMEOUT_MS) == OM_TRUE
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

    return (arm_task_motor_online(arm_task_get_motor(ARM_TASK_MACHINE_ROLL3)) == OM_TRUE) ?
               OM_TRUE :
               OM_FALSE;
}

OmBool arm_task_get_roll3_feedback_angle_rad(
    const ArmTaskContext* context,
    float* angle_rad)
{
    if (context == OM_NULL || angle_rad == OM_NULL || arm_task_get_motor(ARM_TASK_MACHINE_ROLL3) == OM_NULL)
    {
        return OM_FALSE;
    }

    return motor_get_single_turn_angle_rad(arm_task_get_motor(ARM_TASK_MACHINE_ROLL3), angle_rad);
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
        APP_ARM_ROLL3_ANGLE_PID_KP,
        APP_ARM_ROLL3_ANGLE_PID_KI,
        APP_ARM_ROLL3_ANGLE_PID_KD,
        APP_ARM_ROLL3_ANGLE_PID_OUT_LIMIT,
        APP_ARM_ROLL3_ANGLE_PID_INTEGRAL_LIMIT);
    if (ret != OM_OK)
    {
        return ret;
    }

    return arm_task_init_pid(
        &g_roll3_speed_pid,
        APP_ARM_ROLL3_SPEED_PID_KP,
        APP_ARM_ROLL3_SPEED_PID_KI,
        APP_ARM_ROLL3_SPEED_PID_KD,
        APP_ARM_ROLL3_SPEED_PID_OUT_LIMIT,
        APP_ARM_ROLL3_SPEED_PID_INTEGRAL_LIMIT);
}

OmRet arm_task_try_bind_motors(ArmTaskContext* context)
{
    uint32_t axis = 0u;

    if (context == OM_NULL)
    {
        return OM_ERROR_NULL;
    }

    context->flags &= ~ARM_TASK_FLAG_MOTORS_BOUND;

    for (axis = 0u; axis < ARM_TASK_MACHINE_COUNT; axis++)
    {
        if (arm_task_motor_profile_is_present((ArmTaskMachineAxis)axis) != OM_TRUE)
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

    for (axis = 0u; axis < ARM_TASK_MACHINE_COUNT; axis++)
    {
        Motor* motor = g_arm_task_motor_cache[axis];

        if (motor == OM_NULL)
        {
            continue;
        }

        if (motor_set_control_mode(
                motor,
                arm_task_get_profile_control_mode((ArmTaskMachineAxis)axis)) != OM_OK)
        {
            return OM_ERROR;
        }
    }

    context->flags |= ARM_TASK_FLAG_MOTORS_BOUND;
    return OM_OK;
}

OmRet arm_task_restore_control_modes(ArmTaskContext* context)
{
    uint32_t axis = 0u;

    if (context == OM_NULL || !(context->flags & ARM_TASK_FLAG_MOTORS_BOUND))
    {
        return OM_ERROR_NULL;
    }

    for (axis = 0u; axis < ARM_TASK_MACHINE_COUNT; axis++)
    {
        Motor* motor = g_arm_task_motor_cache[axis];

        if (motor == OM_NULL)
        {
            continue;
        }

        if (motor_set_control_mode(
                motor,
                arm_task_get_profile_control_mode((ArmTaskMachineAxis)axis)) != OM_OK)
        {
            return OM_ERROR;
        }
    }

    return OM_OK;
}

/* pitch2 的绝对位置零位由 GO8010 owner 在正式通信 bring-up 中锁存。
 * arm_task 只读这个基准，不再自己维护初始化事实。
 */
OmBool arm_task_get_pitch2_zero_angle_rad(
    const ArmTaskContext* context,
    float* pitch2_zero_angle_rad)
{
    if (context == OM_NULL || pitch2_zero_angle_rad == OM_NULL ||
        arm_task_get_motor(ARM_TASK_MACHINE_PITCH2) == OM_NULL)
    {
        return OM_FALSE;
    }

    return motor_get_initial_zero_angle_rad(arm_task_get_motor(ARM_TASK_MACHINE_PITCH2), pitch2_zero_angle_rad);
}

OmBool arm_task_get_pitch2_joint_feedback_rad(
    const ArmTaskContext* context,
    float* pitch2_joint_angle_rad)
{
    const MotorFeedback* pitch2_feedback = OM_NULL;
    float pitch2_zero_angle_rad = 0.0f;

    if (context == OM_NULL || pitch2_joint_angle_rad == OM_NULL)
    {
        return OM_FALSE;
    }

    pitch2_feedback = motor_get_feedback(arm_task_get_motor(ARM_TASK_MACHINE_PITCH2));
    if (pitch2_feedback == OM_NULL)
    {
        return OM_FALSE;
    }

    if (arm_task_get_pitch2_zero_angle_rad(context, &pitch2_zero_angle_rad) != OM_TRUE)
    {
        return OM_FALSE;
    }

    *pitch2_joint_angle_rad =
        (pitch2_zero_angle_rad - pitch2_feedback->angle) /
        APP_ARM_PITCH2_GEAR_RATIO;
    return OM_TRUE;
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

void arm_task_apply_hold_angle_target(
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

void arm_task_reset_axis_control_state(ArmTaskContext* context)
{
    if (context == OM_NULL)
    {
        return;
    }


    context->last_control_ms[ARM_TASK_MACHINE_BIG_YAW] = 0u;
    context->last_control_ms[ARM_TASK_MACHINE_PITCH1] = 0u;
    context->last_control_ms[ARM_TASK_MACHINE_PITCH2] = 0u;
    context->last_control_ms[ARM_TASK_MACHINE_ROLL2] = 0u;
    context->last_control_ms[ARM_TASK_MACHINE_PITCH3] = 0u;
    context->last_control_ms[ARM_TASK_MACHINE_ROLL3] = 0u;
    context->last_control_ms[ARM_TASK_MACHINE_GRIP] = 0u;
    pid_reset(&g_roll3_angle_pid);
    pid_reset(&g_roll3_speed_pid);
}

OmBool arm_task_should_run_big_yaw_control(
    ArmTaskContext* context,
    OsalTimeMs now_ms)
{
    if (context == OM_NULL)
    {
        return OM_FALSE;
    }

    if (context->last_control_ms[ARM_TASK_MACHINE_BIG_YAW] != 0u &&
        (uint32_t)(now_ms - context->last_control_ms[ARM_TASK_MACHINE_BIG_YAW]) <
            ARM_TASK_BIG_YAW_CONTROL_PERIOD_MS)
    {
        return OM_FALSE;
    }

    context->last_control_ms[ARM_TASK_MACHINE_BIG_YAW] = now_ms;
    return OM_TRUE;
}

OmBool arm_task_should_run_pitch1_control(
    ArmTaskContext* context,
    OsalTimeMs now_ms)
{
    if (context == OM_NULL)
    {
        return OM_FALSE;
    }

    if (context->last_control_ms[ARM_TASK_MACHINE_PITCH1] != 0u &&
        (uint32_t)(now_ms - context->last_control_ms[ARM_TASK_MACHINE_PITCH1]) <
            ARM_TASK_PITCH1_CONTROL_PERIOD_MS)
    {
        return OM_FALSE;
    }

    context->last_control_ms[ARM_TASK_MACHINE_PITCH1] = now_ms;
    return OM_TRUE;
}

OmBool arm_task_should_run_pitch2_control(
    ArmTaskContext* context,
    OsalTimeMs now_ms)
{
    if (context == OM_NULL)
    {
        return OM_FALSE;
    }

    if (context->last_control_ms[ARM_TASK_MACHINE_PITCH2] != 0u &&
        (uint32_t)(now_ms - context->last_control_ms[ARM_TASK_MACHINE_PITCH2]) <
            ARM_TASK_PITCH2_CONTROL_PERIOD_MS)
    {
        return OM_FALSE;
    }

    context->last_control_ms[ARM_TASK_MACHINE_PITCH2] = now_ms;
    return OM_TRUE;
}

OmBool arm_task_should_run_roll2_control(
    ArmTaskContext* context,
    OsalTimeMs now_ms)
{
    if (context == OM_NULL)
    {
        return OM_FALSE;
    }

    if (context->last_control_ms[ARM_TASK_MACHINE_ROLL2] != 0u &&
        (uint32_t)(now_ms - context->last_control_ms[ARM_TASK_MACHINE_ROLL2]) <
            ARM_TASK_ROLL2_CONTROL_PERIOD_MS)
    {
        return OM_FALSE;
    }

    context->last_control_ms[ARM_TASK_MACHINE_ROLL2] = now_ms;
    return OM_TRUE;
}

OmBool arm_task_should_run_pitch3_control(
    ArmTaskContext* context,
    OsalTimeMs now_ms)
{
    if (context == OM_NULL)
    {
        return OM_FALSE;
    }

    if (context->last_control_ms[ARM_TASK_MACHINE_PITCH3] != 0u &&
        (uint32_t)(now_ms - context->last_control_ms[ARM_TASK_MACHINE_PITCH3]) <
            ARM_TASK_PITCH3_CONTROL_PERIOD_MS)
    {
        return OM_FALSE;
    }

    context->last_control_ms[ARM_TASK_MACHINE_PITCH3] = now_ms;
    return OM_TRUE;
}

OmBool arm_task_should_run_roll3_control(
    ArmTaskContext* context,
    OsalTimeMs now_ms)
{
    if (context == OM_NULL)
    {
        return OM_FALSE;
    }

    if (context->last_control_ms[ARM_TASK_MACHINE_ROLL3] != 0u &&
        (uint32_t)(now_ms - context->last_control_ms[ARM_TASK_MACHINE_ROLL3]) <
            ARM_TASK_ROLL3_CONTROL_PERIOD_MS)
    {
        return OM_FALSE;
    }

    context->last_control_ms[ARM_TASK_MACHINE_ROLL3] = now_ms;
    return OM_TRUE;
}

OmBool arm_task_pitch2_zero_ready(const ArmTaskContext* context)
{
    float zero_angle_rad = 0.0f;

    if (context == OM_NULL)
    {
        return OM_FALSE;
    }

    return arm_task_get_pitch2_zero_angle_rad(context, &zero_angle_rad);
}

OmBool arm_task_should_run_grip_control(
    ArmTaskContext* context,
    OsalTimeMs now_ms)
{
    if (context == OM_NULL)
    {
        return OM_FALSE;
    }

    if (context->last_control_ms[ARM_TASK_MACHINE_GRIP] != 0u &&
        (uint32_t)(now_ms - context->last_control_ms[ARM_TASK_MACHINE_GRIP]) <
            ARM_TASK_GRIP_CONTROL_PERIOD_MS)
    {
        return OM_FALSE;
    }

    context->last_control_ms[ARM_TASK_MACHINE_GRIP] = now_ms;
    return OM_TRUE;
}

void arm_task_apply_current_command(
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

    if (context == OM_NULL || arm_task_get_motor(ARM_TASK_MACHINE_ROLL3) == OM_NULL)
    {
        return;
    }

    feedback = motor_get_feedback(arm_task_get_motor(ARM_TASK_MACHINE_ROLL3));
    if (arm_task_feedback_online(feedback) != OM_TRUE ||
        arm_task_get_roll3_feedback_angle_rad(context, &feedback_angle_rad) != OM_TRUE)
    {
        arm_task_apply_current_command(arm_task_get_motor(ARM_TASK_MACHINE_ROLL3), 0.0f);
        return;
    }

    nearest_target_angle_rad =
        math_utils_resolve_nearest_equivalent_rad(
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
    arm_task_apply_current_command(
        arm_task_get_motor(ARM_TASK_MACHINE_ROLL3),
        command_current);
}

OmBool arm_task_should_submit_tx_request(
    ArmTaskContext* context,
    ChassisMode chassis_mode,
    OsalTimeMs now_ms)
{
    if (context == OM_NULL)
    {
        return OM_FALSE;
    }

    if (mct_is_operational_active() != OM_TRUE ||
        chassis_mode == MODE_CHASSIS_RELEASE)
    {
        context->last_tx_request_ms = 0u;
        return OM_FALSE;
    }

    if (context->last_tx_request_ms != 0u &&
        (uint32_t)(now_ms - context->last_tx_request_ms) <
            ARM_TASK_TX_REQUEST_PERIOD_MS)
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

    arm_task_reset_axis_control_state(context);
    arm_task_apply_hold_angle_target(
        arm_task_get_motor(ARM_TASK_MACHINE_BIG_YAW),
        APP_ARM_BIG_YAW_KP,
        APP_ARM_BIG_YAW_KD);
    arm_task_apply_hold_angle_target(
        arm_task_get_motor(ARM_TASK_MACHINE_PITCH1),
        APP_ARM_PITCH1_KP,
        APP_ARM_PITCH1_KD);
    arm_task_apply_hold_angle_target(
        arm_task_get_motor(ARM_TASK_MACHINE_PITCH2),
        APP_ARM_PITCH2_KP,
        APP_ARM_PITCH2_KD);
    arm_task_apply_hold_angle_target(
        arm_task_get_motor(ARM_TASK_MACHINE_ROLL2),
        APP_ARM_ROLL2_KP,
        APP_ARM_ROLL2_KD);
    arm_task_apply_hold_angle_target(
        arm_task_get_motor(ARM_TASK_MACHINE_PITCH3),
        APP_ARM_PITCH3_KP,
        APP_ARM_PITCH3_KD);
    arm_task_apply_current_command(arm_task_get_motor(ARM_TASK_MACHINE_ROLL3), 0.0f);
    arm_task_apply_hold_angle_target(
        arm_task_get_motor(ARM_TASK_MACHINE_GRIP),
        APP_ARM_GRIP_KP,
        APP_ARM_GRIP_KD);
}

/* 带时间窗的动作从“共享控制事实发生变化”那一刻重新计时。 */
void arm_task_update_command_timer(ArmTaskContext* context, const ArmTaskSnapshot* snapshot)
{
    if (context == OM_NULL || snapshot == OM_NULL)
    {
        return;
    }

    if (!(context->flags & ARM_TASK_FLAG_SNAPSHOT_INITIALIZED) ||
        arm_task_snapshot_changed(&context->last_snapshot, snapshot) == OM_TRUE)
    {
        context->last_snapshot = *snapshot;
        context->command_since_ms = osal_time_now_monotonic();
        context->flags |= ARM_TASK_FLAG_SNAPSHOT_INITIALIZED;
    }
}

/* 机械臂控制主循环：
 * 读快照 -> 生成姿态表 -> 映射电机目标 -> 下发到 motor 抽象层 -> 发布 TX 请求。
 */
void arm_task_run_once(ArmTaskContext* context)
{
    ArmTaskSnapshot snapshot = {0};
    ArmTaskCustomControllerSnapshot controller_snapshot = {0};
    ArmTaskMachinePose pose = {0};
    ArmTaskMotorTargets targets = {0};
    const float current_tick_s = ((float)ARM_TASK_PERIOD_MS) / 1000.0f;
    const OsalTimeMs now_ms = osal_time_now_monotonic();
    OsalTimeMs elapsed_ms = 0u;
    float pitch1_torque_ff = 0.0f;
    float pitch2_torque_ff = 0.0f;
    float roll2_torque_ff = 0.0f;
    float pitch3_torque_ff = 0.0f;
    OmBool custom_controller_mode_selected = OM_FALSE;
    OmBool custom_controller_input_ready = OM_FALSE;
    OmBool custom_controller_force_takeover_requested = OM_FALSE;
    OmBool custom_controller_active = OM_FALSE;

    if (context == OM_NULL)
    {
        return;
    }

    arm_task_drain_mode_snapshots(context);
    arm_task_drain_custom_controller_snapshots(context);
    if (arm_task_load_snapshot(context, &snapshot) != OM_TRUE)
    {
        return;
    }
    arm_task_load_custom_controller_snapshot(context, &controller_snapshot);

    /* 确保电机已绑定，如果未绑定则尝试绑定 */
    if (!(context->flags & ARM_TASK_FLAG_MOTORS_BOUND))
    {
        if (arm_task_try_bind_motors(context) != OM_OK)
        {
            return;
        }
    }
    if (mct_is_operational_active() != OM_TRUE)
    {
        context->flags &= ~ARM_TASK_FLAG_CONTROL_MODES_ARMED;
    }
    else if (!(context->flags & ARM_TASK_FLAG_CONTROL_MODES_ARMED))
    {
        if (arm_task_restore_control_modes(context) != OM_OK)
        {
            return;
        }
        context->flags |= ARM_TASK_FLAG_CONTROL_MODES_ARMED;
    }

    /* 判断自定义控制器模式是否选中、输入是否就绪、是否请求强制接管 */
    custom_controller_mode_selected =
        (snapshot.chassis_mode == MODE_CHASSIS_CUSTOM_CONTROLLER_NORMAL) ? OM_TRUE : OM_FALSE;
    custom_controller_input_ready =
        (controller_snapshot.online != 0u &&
         controller_snapshot.work_mode == ARM_TASK_CUSTOM_CONTROLLER_WORK_MODE_ENCODER)
            ? OM_TRUE
            : OM_FALSE;
    custom_controller_force_takeover_requested =
        (snapshot.custom_controller_force_takeover_flag != 0u) ? OM_TRUE : OM_FALSE;

    if (custom_controller_mode_selected == OM_TRUE &&
        !(context->flags & ARM_TASK_FLAG_CUSTOM_ALIGNMENT_DONE))
    {
        if (context->custom_controller_alignment_started_ms == 0u)
        {
            context->custom_controller_alignment_started_ms = now_ms;
        }
        else if (!(context->flags & ARM_TASK_FLAG_CUSTOM_ALIGNMENT_FAILED) &&
                 (OsalTimeMs)(now_ms - context->custom_controller_alignment_started_ms) >=
                     APP_ARM_CUSTOM_CONTROLLER_ALIGNMENT_TIMEOUT_MS)
        {
            context->flags |= ARM_TASK_FLAG_CUSTOM_ALIGNMENT_FAILED;
            sh_set_custom_controller_calibration_failed();
        }
    }
    
    /* 强制接管是独立动作，不等同于“自动校准失败”。
     * 如果之前还没判失败，这里只结束 pending 指示并进入接管；
     * 若已经超时失败，则保留红灯，直到退出该模式。
     */
    if (custom_controller_mode_selected == OM_TRUE &&
        custom_controller_input_ready == OM_TRUE &&
        custom_controller_force_takeover_requested == OM_TRUE &&
        !(context->flags & ARM_TASK_FLAG_CUSTOM_ALIGNMENT_DONE))
    {
        if (!(context->flags & ARM_TASK_FLAG_CUSTOM_ALIGNMENT_FAILED))
        {
            sh_clear_custom_controller_calibration_indicator();
        }
        context->flags |= ARM_TASK_FLAG_CUSTOM_ALIGNMENT_DONE;
        context->flags &= ~ARM_TASK_FLAG_CUSTOM_REF_CAPTURED;
        context->flags &= ~ARM_TASK_FLAG_CUSTOM_WAS_ACTIVE;
        g_arm_task_custom_controller_alignment_done_debug = 1u;
    }
    
    /* 检查自定义控制器是否处于主动接管状态 */
    custom_controller_active =
        arm_task_custom_controller_takeover_active(context, &snapshot, &controller_snapshot);
    
    /* 更新自定义控制器参考状态 */
    arm_task_update_custom_controller_reference_state(
        context,
        custom_controller_mode_selected,
        custom_controller_active,
        &controller_snapshot);
    
    /* 更新命令计时器 */
    arm_task_update_command_timer(context, &snapshot);
    
    /* 机械臂目标链不应被单个轴的在线状态整体拖死。
     * 这里先继续推进共享姿态 -> 电机目标 -> 平滑目标，
     * 再在各轴输出点各自判断 online / hold。
     */
    if (snapshot.chassis_mode == MODE_CHASSIS_RELEASE)
    {
        /* RELEASE 会让机械臂失力。重新回到可控模式时，必须从当前反馈重新建目标：
         * - 清掉动作计时，避免时间窗继续沿用上一次动作
         * - 清掉平滑目标，避免 first tick 先冲向旧目标再回 normal
         * - 清掉自定义控制器接管状态，避免 release 前后的参考残留
         */
        context->flags &= ~ARM_TASK_FLAG_SNAPSHOT_INITIALIZED;
        arm_task_clear_smoothed_targets(context);
        arm_task_reset_custom_controller_state(context);

        /* 底盘释放模式下，应用释放输出 */
        arm_task_apply_release_output(context);
    }
    else
    {
        /* 正常控制模式：计算姿态、解析目标、应用控制 */
        elapsed_ms = now_ms - context->command_since_ms;
        
        /* 根据是否选择自定义控制器模式，采用不同的姿态生成策略 */
        if (custom_controller_mode_selected == OM_TRUE)
        {
            /* 检查自定义控制器对齐是否完成 */
            if (custom_controller_input_ready == OM_TRUE &&
                !(context->flags & ARM_TASK_FLAG_CUSTOM_ALIGNMENT_DONE) &&
                arm_task_custom_controller_alignment_reached(context) == OM_TRUE)
            {
                context->flags &= ~ARM_TASK_FLAG_CUSTOM_ALIGNMENT_FAILED;
                sh_set_custom_controller_calibration_success();
                context->flags |= ARM_TASK_FLAG_CUSTOM_ALIGNMENT_DONE;
                g_arm_task_custom_controller_alignment_done_debug = 1u;
            }

            /* 根据对齐状态和接管状态选择姿态来源 */
            if ((context->flags & ARM_TASK_FLAG_CUSTOM_ALIGNMENT_DONE) &&
                custom_controller_active == OM_TRUE)
            {
                /* 对齐完成且控制器主动接管时，使用自定义控制器姿态 */
                arm_task_apply_custom_controller_pose(context, &controller_snapshot, &pose);
            }
            else
            {
                /* 否则使用对齐姿态 */
                arm_task_apply_custom_controller_alignment_pose(context, &pose);
            }
        }
        else
        {
            /* 常规模式下，使用角度钳位处理 */
            clamp_angle_handle(&snapshot, elapsed_ms, &pose);
        }
        
        /* 将机器姿态解析为各电机的目标值 */
        arm_task_resolve_motor_targets(context, &pose, &targets);
        /* 对电机目标进行平滑滤波处理 */
        arm_task_update_smoothed_targets(context, &targets, current_tick_s);
        arm_task_compute_gravity_feedforward(
            context,
            &pitch1_torque_ff,
            &pitch2_torque_ff,
            &roll2_torque_ff,
            &pitch3_torque_ff);
        if (arm_task_motor_profile_allows_control(ARM_TASK_MACHINE_BIG_YAW) == OM_TRUE &&
            arm_task_should_run_big_yaw_control(context, now_ms) == OM_TRUE)
        {
            if (arm_task_motor_online(arm_task_get_motor(ARM_TASK_MACHINE_BIG_YAW)) == OM_TRUE)
            {
                arm_task_apply_angle_target(
                    arm_task_get_motor(ARM_TASK_MACHINE_BIG_YAW),
                    context->smoothed_targets.big_yaw_rad,
                    APP_ARM_BIG_YAW_KP,
                    APP_ARM_BIG_YAW_KD,
                    0.0f);
            }
            else
            {
                arm_task_apply_hold_angle_target(
                    arm_task_get_motor(ARM_TASK_MACHINE_BIG_YAW),
                    APP_ARM_BIG_YAW_KP,
                    APP_ARM_BIG_YAW_KD);
            }
        }
        if (arm_task_motor_profile_allows_control(ARM_TASK_MACHINE_PITCH1) == OM_TRUE &&
            arm_task_should_run_pitch1_control(context, now_ms) == OM_TRUE)
        {
            if (arm_task_motor_online(arm_task_get_motor(ARM_TASK_MACHINE_PITCH1)) == OM_TRUE)
            {
                arm_task_apply_angle_target(
                    arm_task_get_motor(ARM_TASK_MACHINE_PITCH1),
                    context->smoothed_targets.pitch1_rad,
                    APP_ARM_PITCH1_KP,
                    APP_ARM_PITCH1_KD,
                    pitch1_torque_ff);
            }
            else
            {
                arm_task_apply_hold_angle_target(
                    arm_task_get_motor(ARM_TASK_MACHINE_PITCH1),
                    APP_ARM_PITCH1_KP,
                    APP_ARM_PITCH1_KD);
            }
        }
        if (arm_task_motor_profile_allows_control(ARM_TASK_MACHINE_PITCH2) == OM_TRUE &&
            arm_task_should_run_pitch2_control(context, now_ms) == OM_TRUE)
        {
            if (arm_task_motor_online(arm_task_get_motor(ARM_TASK_MACHINE_PITCH2)) == OM_TRUE &&
                arm_task_pitch2_zero_ready(context) == OM_TRUE)
            {
                arm_task_apply_angle_target(
                    arm_task_get_motor(ARM_TASK_MACHINE_PITCH2),
                    context->smoothed_targets.pitch2_rad,
                    APP_ARM_PITCH2_KP,
                    APP_ARM_PITCH2_KD,
                    pitch2_torque_ff);
            }
            else
            {
                arm_task_apply_hold_angle_target(
                    arm_task_get_motor(ARM_TASK_MACHINE_PITCH2),
                    APP_ARM_PITCH2_KP,
                    APP_ARM_PITCH2_KD);
            }
        }
        if (arm_task_motor_profile_allows_control(ARM_TASK_MACHINE_ROLL2) == OM_TRUE &&
            arm_task_should_run_roll2_control(context, now_ms) == OM_TRUE)
        {
            if (arm_task_motor_online(arm_task_get_motor(ARM_TASK_MACHINE_ROLL2)) == OM_TRUE)
            {
                arm_task_apply_angle_target(
                    arm_task_get_motor(ARM_TASK_MACHINE_ROLL2),
                    context->smoothed_targets.roll2_rad,
                    APP_ARM_ROLL2_KP,
                    APP_ARM_ROLL2_KD,
                    roll2_torque_ff);
            }
            else
            {
                arm_task_apply_hold_angle_target(
                    arm_task_get_motor(ARM_TASK_MACHINE_ROLL2),
                    APP_ARM_ROLL2_KP,
                    APP_ARM_ROLL2_KD);
            }
        }
        if (arm_task_motor_profile_allows_control(ARM_TASK_MACHINE_PITCH3) == OM_TRUE &&
            arm_task_should_run_pitch3_control(context, now_ms) == OM_TRUE)
        {
            if (arm_task_motor_online(arm_task_get_motor(ARM_TASK_MACHINE_PITCH3)) == OM_TRUE)
            {
                arm_task_apply_angle_target(
                    arm_task_get_motor(ARM_TASK_MACHINE_PITCH3),
                    context->smoothed_targets.pitch3_rad,
                    APP_ARM_PITCH3_KP,
                    APP_ARM_PITCH3_KD,
                    pitch3_torque_ff);
            }
            else
            {
                arm_task_apply_hold_angle_target(
                    arm_task_get_motor(ARM_TASK_MACHINE_PITCH3),
                    APP_ARM_PITCH3_KP,
                    APP_ARM_PITCH3_KD);
            }
        }
        if (arm_task_motor_profile_allows_control(ARM_TASK_MACHINE_ROLL3) == OM_TRUE &&
            arm_task_should_run_roll3_control(context, now_ms) == OM_TRUE)
        {
            arm_task_apply_roll3_target(
                context,
                context->smoothed_targets.roll3_rad,
                current_tick_s);
        }
        if (arm_task_motor_profile_allows_control(ARM_TASK_MACHINE_GRIP) == OM_TRUE &&
            arm_task_should_run_grip_control(context, now_ms) == OM_TRUE)
        {
            if (arm_task_motor_online(arm_task_get_motor(ARM_TASK_MACHINE_GRIP)) == OM_TRUE)
            {
                arm_task_apply_angle_target(
                    arm_task_get_motor(ARM_TASK_MACHINE_GRIP),
                    context->smoothed_targets.grip_rad,
                    APP_ARM_GRIP_KP,
                    APP_ARM_GRIP_KD,
                    0.0f);
            }
            else
            {
                arm_task_apply_hold_angle_target(
                    arm_task_get_motor(ARM_TASK_MACHINE_GRIP),
                    APP_ARM_GRIP_KP,
                    APP_ARM_GRIP_KD);
            }
        }
    }

    if (arm_task_should_submit_tx_request(
            context,
            snapshot.chassis_mode,
            now_ms) != OM_TRUE)
    {
        return;
    }

    (void)motor_tx_dispatch_submit(MOTOR_TX_SOURCE_ARM);
}

    /* 正式输入已经改走通道和 owner latest-cache；这里固定节拍运行。 */
