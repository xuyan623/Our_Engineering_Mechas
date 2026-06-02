#include "task/chassis_task/chassis_task_internal.h"

#include "algorithm/kinematics/kinematics.h"
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
#include <math.h>
#include <string.h>

float chassis_task_now_s(void)
{
    return ((float)osal_time_now_monotonic()) / 1000.0f;
}

OmBool chassis_task_is_front_wheel_index(uint32_t wheel_index)
{
    return (wheel_index < CHASSIS_TASK_FRONT_WHEEL_COUNT) ? OM_TRUE : OM_FALSE;
}

OmBool chassis_task_wheel_profile_is_present(uint32_t wheel_index)
{
    if (wheel_index >= CHASSIS_TASK_WHEEL_COUNT)
    {
        return OM_FALSE;
    }

    return app_motor_profile_is_present(g_chassis_task_wheel_roles[wheel_index]);
}

OmBool chassis_task_wheel_profile_allows_control(uint32_t wheel_index)
{
    if (wheel_index >= CHASSIS_TASK_WHEEL_COUNT)
    {
        return OM_FALSE;
    }

    return app_motor_profile_allows_control(g_chassis_task_wheel_roles[wheel_index]);
}

OmBool chassis_task_leg_profile_is_present(uint32_t leg_index)
{
    if (leg_index >= CHASSIS_TASK_LEG_COUNT)
    {
        return OM_FALSE;
    }

    return app_motor_profile_is_present(g_chassis_task_leg_roles[leg_index]);
}

OmBool chassis_task_leg_profile_allows_control(uint32_t leg_index)
{
    if (leg_index >= CHASSIS_TASK_LEG_COUNT)
    {
        return OM_FALSE;
    }

    return app_motor_profile_allows_control(g_chassis_task_leg_roles[leg_index]);
}

PidController* chassis_task_get_wheel_speed_pid(
    ChassisTaskContext* context,
    uint32_t wheel_index)
{
    if (context == OM_NULL || wheel_index >= CHASSIS_TASK_WHEEL_COUNT)
    {
        return OM_NULL;
    }

    if (chassis_task_is_front_wheel_index(wheel_index) == OM_TRUE)
    {
        return &g_front_wheel_speed_pids[wheel_index];
    }

    return &g_rear_wheel_speed_pids[wheel_index - CHASSIS_TASK_FRONT_WHEEL_COUNT];
}

OmBool chassis_task_motor_feedback_recent(const Motor* motor, uint32_t timeout_ms)
{
    if (motor == OM_NULL)
    {
        return OM_FALSE;
    }

    return motor_is_feedback_recent(motor, timeout_ms);
}

OmBool chassis_task_key_is_down(uint16_t keyboard_bits, uint16_t mask)
{
    return ((keyboard_bits & mask) != 0u) ? OM_TRUE : OM_FALSE;
}

void chassis_task_drain_mode_snapshots(ChassisTaskContext* context)
{
    ModeTaskControlSnapshot snapshot = {0};

    if (context == OM_NULL)
    {
        return;
    }

    while (task_mpsc_channel_receive_nonblocking(&context->mode_channel, &snapshot) == OM_OK)
    {
        context->latest_mode_snapshot = snapshot;
        context->flags |= CHASSIS_TASK_FLAG_MODE_SNAPSHOT_READY;
    }
}

void chassis_task_drain_rc_snapshots(ChassisTaskContext* context)
{
    DpRcSnapshot snapshot = {0};

    if (context == OM_NULL)
    {
        return;
    }

    while (task_pipe_channel_receive(&context->rc_channel, &snapshot, 0u) == OM_OK)
    {
        context->latest_rc_snapshot = snapshot;
        context->flags |= CHASSIS_TASK_FLAG_RC_SNAPSHOT_READY;
    }
}

void chassis_task_drain_imu_snapshots(ChassisTaskContext* context)
{
    DpImuSnapshot snapshot = {0};

    if (context == OM_NULL)
    {
        return;
    }

    while (task_pipe_channel_receive(&context->imu_channel, &snapshot, 0u) == OM_OK)
    {
        context->latest_imu_snapshot = snapshot;
        context->flags |= CHASSIS_TASK_FLAG_IMU_SNAPSHOT_READY;
    }
}

OmBool chassis_task_load_snapshot(
    const ChassisTaskContext* context,
    ChassisTaskInputSnapshot* snapshot)
{
    if (context == OM_NULL || snapshot == OM_NULL ||
        !(context->flags & CHASSIS_TASK_FLAG_MODE_SNAPSHOT_READY) ||
        !(context->flags & CHASSIS_TASK_FLAG_RC_SNAPSHOT_READY))
    {
        return OM_FALSE;
    }

    snapshot->ch1 = context->latest_rc_snapshot.ch1;
    snapshot->ch2 = context->latest_rc_snapshot.ch2;
    snapshot->ch3 = context->latest_rc_snapshot.ch3;
    snapshot->ch4 = context->latest_rc_snapshot.ch4;
    snapshot->mouse_x = context->latest_rc_snapshot.mouse.x;
    snapshot->keyboard_bits = context->latest_rc_snapshot.keyboard_bits;
    snapshot->system_state = (ModeTaskSystemState)context->latest_mode_snapshot.system_state;
    snapshot->control_domain_state =
        (ModeTaskControlDomainState)context->latest_mode_snapshot.control_domain_state;
    snapshot->global_mode = (GlobalMode)context->latest_mode_snapshot.global_mode;
    snapshot->chassis_mode = (ChassisMode)context->latest_mode_snapshot.chassis_mode;
    snapshot->imu_pitch_deg =
        ((context->flags & CHASSIS_TASK_FLAG_IMU_SNAPSHOT_READY)) ? context->latest_imu_snapshot.pitch : 0.0f;
    return OM_TRUE;
}

void chassis_task_compute_keyboard_velocity(
    const ChassisTaskInputSnapshot* snapshot,
    float* vx_mm_per_s,
    float* vy_mm_per_s,
    float* vw_deg_per_s)
{
    OmBool shift_pressed = OM_FALSE;
    OmBool ctrl_pressed = OM_FALSE;

    if (snapshot == OM_NULL || vx_mm_per_s == OM_NULL || vy_mm_per_s == OM_NULL || vw_deg_per_s == OM_NULL)
    {
        return;
    }

    *vx_mm_per_s = 0.0f;
    *vy_mm_per_s = 0.0f;
    *vw_deg_per_s =
        ((float)snapshot->mouse_x / APP_RC_RESOLUTION) *
        APP_CHASSIS_KB_MAX_SPEED_R_DEG_PER_S *
        APP_CHASSIS_KB_MOVE_RATIO_R *
        APP_CHASSIS_KB_MOUSE_ROTATE_SCALE;

    shift_pressed = chassis_task_key_is_down(snapshot->keyboard_bits, CHASSIS_TASK_KEY_SHIFT_MASK);
    ctrl_pressed = chassis_task_key_is_down(snapshot->keyboard_bits, CHASSIS_TASK_KEY_CTRL_MASK);

    if (shift_pressed == OM_TRUE || ctrl_pressed == OM_TRUE)
    {
        return;
    }

    if (chassis_task_key_is_down(snapshot->keyboard_bits, CHASSIS_TASK_KEY_W_MASK) == OM_TRUE)
    {
        *vy_mm_per_s = -APP_CHASSIS_KB_MAX_SPEED_Y_MM_PER_S * APP_CHASSIS_KB_MOVE_RATIO_Y;
    }
    else if (chassis_task_key_is_down(snapshot->keyboard_bits, CHASSIS_TASK_KEY_S_MASK) == OM_TRUE)
    {
        *vy_mm_per_s = APP_CHASSIS_KB_MAX_SPEED_Y_MM_PER_S * APP_CHASSIS_KB_MOVE_RATIO_Y;
    }

    if (chassis_task_key_is_down(snapshot->keyboard_bits, CHASSIS_TASK_KEY_A_MASK) == OM_TRUE)
    {
        *vx_mm_per_s = -APP_CHASSIS_KB_MAX_SPEED_X_MM_PER_S * APP_CHASSIS_KB_MOVE_RATIO_X;
    }
    else if (chassis_task_key_is_down(snapshot->keyboard_bits, CHASSIS_TASK_KEY_D_MASK) == OM_TRUE)
    {
        *vx_mm_per_s = APP_CHASSIS_KB_MAX_SPEED_X_MM_PER_S * APP_CHASSIS_KB_MOVE_RATIO_X;
    }
}

float chassis_task_compute_rc_rotate_velocity_deg_per_s(
    ChassisTaskContext* context,
    int16_t ch3,
    OsalTimeMs now_ms)
{
    float rotate_velocity_deg_per_s = 0.0f;

    if (context == OM_NULL)
    {
        return 0.0f;
    }

    if (math_utils_abs_float((float)ch3) < APP_RC_RESOLUTION)
    {
        context->rc_rotate_saturation_since_ms = now_ms;
    }

    rotate_velocity_deg_per_s =
        ((float)ch3 / APP_RC_RESOLUTION) *
        APP_CHASSIS_MAX_VW_DEG_PER_S;

    if ((OsalTimeMs)(now_ms - context->rc_rotate_saturation_since_ms) <=
        APP_CHASSIS_RC_ROTATE_SOFTEN_HOLD_MS)
    {
        rotate_velocity_deg_per_s *= APP_CHASSIS_RC_ROTATE_SOFTEN_SCALE;
    }

    return rotate_velocity_deg_per_s;
}

void chassis_task_compute_chassis_velocity(
    ChassisTaskContext* context,
    const ChassisTaskInputSnapshot* snapshot,
    float* vx_mm_per_s,
    float* vy_mm_per_s,
    float* vw_deg_per_s)
{
    float rc_vx_mm_per_s = 0.0f;
    float rc_vy_mm_per_s = 0.0f;
    float rc_vw_deg_per_s = 0.0f;
    float kb_vx_mm_per_s = 0.0f;
    float kb_vy_mm_per_s = 0.0f;
    float kb_vw_deg_per_s = 0.0f;
    const OsalTimeMs now_ms = osal_time_now_monotonic();

    if (context == OM_NULL || snapshot == OM_NULL || vx_mm_per_s == OM_NULL ||
        vy_mm_per_s == OM_NULL || vw_deg_per_s == OM_NULL)
    {
        return;
    }

    rc_vx_mm_per_s = ((float)snapshot->ch1 / APP_RC_RESOLUTION) * APP_CHASSIS_MAX_VX_MM_PER_S;
    rc_vy_mm_per_s = -((float)snapshot->ch2 / APP_RC_RESOLUTION) * APP_CHASSIS_MAX_VY_MM_PER_S;
    rc_vw_deg_per_s = chassis_task_compute_rc_rotate_velocity_deg_per_s(context, snapshot->ch3, now_ms);

    chassis_task_compute_keyboard_velocity(
        snapshot,
        &kb_vx_mm_per_s,
        &kb_vy_mm_per_s,
        &kb_vw_deg_per_s);

    *vy_mm_per_s = -(rc_vx_mm_per_s + kb_vx_mm_per_s);
    *vx_mm_per_s = -(rc_vy_mm_per_s + kb_vy_mm_per_s);
    *vw_deg_per_s = -(rc_vw_deg_per_s + kb_vw_deg_per_s);
}

