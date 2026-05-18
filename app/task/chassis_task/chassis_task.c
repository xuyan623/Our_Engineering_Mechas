#include "task/chassis_task/chassis_task.h"

#include "algorithm/kinematics/kinematics.h"
#include "config/app_config.h"
#include "core/algorithm/controller/pid.h"
#include "driver/motor/motor.h"
#include "module/data_pool/data_pool.h"
#include "module/event_bus/event_bus.h"
#include "module/system_health/system_health.h"
#include "task/mode_task/mode_task.h"
#include "osal/osal.h"
#include "osal/osal_config.h"
#include "osal/osal_time.h"
#include <math.h>
#include <string.h>

#define CHASSIS_TASK_PERIOD_MS        (6u)
#define CHASSIS_TASK_STACK_BYTES      (512u * OSAL_STACK_WORD_BYTES)
#define CHASSIS_TASK_PRIORITY         (4u)
#define CHASSIS_TASK_WHEEL_COUNT      (MECANUM_WHEEL_COUNT)
#define CHASSIS_TASK_LEG_COUNT        (2u)
#define CHASSIS_TASK_RIGHT_LEG_INDEX  (0u)
#define CHASSIS_TASK_LEFT_LEG_INDEX   (1u)

#define CHASSIS_TASK_KEY_W_MASK       (1u << 0u)
#define CHASSIS_TASK_KEY_S_MASK       (1u << 1u)
#define CHASSIS_TASK_KEY_A_MASK       (1u << 2u)
#define CHASSIS_TASK_KEY_D_MASK       (1u << 3u)
#define CHASSIS_TASK_KEY_SHIFT_MASK   (1u << 4u)
#define CHASSIS_TASK_KEY_CTRL_MASK    (1u << 5u)

static const uint32_t g_chassis_task_wheel_feedback_timeout_ms = 20u;

typedef struct
{
    int16_t ch1;
    int16_t ch2;
    int16_t ch3;
    int16_t ch4;
    int16_t mouse_x;
    uint16_t keyboard_bits;
    ChassisMode chassis_mode;
    float imu_pitch_deg;
} ChassisTaskInputSnapshot;

typedef struct
{
    Motor* wheel_motors[CHASSIS_TASK_WHEEL_COUNT];
    Motor* leg_motors[CHASSIS_TASK_LEG_COUNT];
    Motor* big_yaw_motor;
    PidController wheel_speed_pids[CHASSIS_TASK_WHEEL_COUNT];
    PidController leg_angle_pids[CHASSIS_TASK_LEG_COUNT];
    PidController leg_speed_pids[CHASSIS_TASK_LEG_COUNT];
    float leg_speed_filtered_rpm[CHASSIS_TASK_LEG_COUNT];
    float pit_leg_cmd_deg;
    float big_yaw_hold_angle_rad;
    OsalTimeMs rc_rotate_saturation_since_ms;
    OmBool big_yaw_hold_initialized;
    OmBool motors_bound_flag;
} ChassisTaskContext;

static const char* g_chassis_task_wheel_names[CHASSIS_TASK_WHEEL_COUNT] = {
    "chassis_fr",
    "chassis_fl",
    "chassis_bl",
    "chassis_br",
};

static const char* g_chassis_task_leg_names[CHASSIS_TASK_LEG_COUNT] = {
    "joint_leg_r",
    "joint_leg_l",
};
static const char* g_chassis_task_big_yaw_name = "big_yaw";

static float chassis_task_now_s(void)
{
    return ((float)osal_time_now_monotonic()) / 1000.0f;
}

static float chassis_task_clamp_float(float value, float min_value, float max_value)
{
    if (value < min_value)
    {
        return min_value;
    }
    if (value > max_value)
    {
        return max_value;
    }
    return value;
}

static float chassis_task_abs_float(float value)
{
    return (value < 0.0f) ? -value : value;
}

static float chassis_task_rad_to_deg(float angle_rad)
{
    return angle_rad * (180.0f / APP_PI);
}

static float chassis_task_rad_per_s_to_rpm(float speed_rad_per_s)
{
    return speed_rad_per_s * (60.0f / (2.0f * APP_PI));
}

static float chassis_task_normalize_deg(float angle_deg)
{
    while (angle_deg > 180.0f)
    {
        angle_deg -= 360.0f;
    }
    while (angle_deg < -180.0f)
    {
        angle_deg += 360.0f;
    }
    return angle_deg;
}

static OmBool chassis_task_feedback_recent(const MotorFeedback* feedback, uint32_t timeout_ms)
{
    if (feedback == OM_NULL || feedback->online != OM_TRUE || feedback->timestamp_ms == 0u)
    {
        return OM_FALSE;
    }

    return ((uint32_t)(osal_time_now_monotonic() - feedback->timestamp_ms) <= timeout_ms) ? OM_TRUE : OM_FALSE;
}

static OmBool chassis_task_key_is_down(uint16_t keyboard_bits, uint16_t mask)
{
    return ((keyboard_bits & mask) != 0u) ? OM_TRUE : OM_FALSE;
}

static void chassis_task_load_snapshot(ChassisTaskInputSnapshot* snapshot)
{
    if (snapshot == OM_NULL)
    {
        return;
    }

    snapshot->ch1 = DP_LOAD_INT16(&g_data_pool.rc.ch1);
    snapshot->ch2 = DP_LOAD_INT16(&g_data_pool.rc.ch2);
    snapshot->ch3 = DP_LOAD_INT16(&g_data_pool.rc.ch3);
    snapshot->ch4 = DP_LOAD_INT16(&g_data_pool.rc.ch4);
    snapshot->mouse_x = DP_LOAD_INT16(&g_data_pool.rc.mouse.x);
    snapshot->keyboard_bits = DP_LOAD_UINT16(&g_data_pool.rc.keyboard_bits);
    snapshot->chassis_mode = (ChassisMode)DP_LOAD_UINT8(&g_data_pool.mode.chassis_mode);
    snapshot->imu_pitch_deg = DP_LOAD_FLOAT(&g_data_pool.imu.pitch);
}

static OmRet chassis_task_init_pid(
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

static OmRet chassis_task_init_pids(ChassisTaskContext* context)
{
    const float task_period_s = ((float)CHASSIS_TASK_PERIOD_MS) / 1000.0f;
    const float wheel_speed_ki = APP_CHASSIS_WHEEL_SPEED_PID_KI / task_period_s;
    uint32_t index = 0u;
    OmRet ret = OM_OK;

    if (context == OM_NULL)
    {
        return OM_ERROR_NULL;
    }

    for (index = 0u; index < CHASSIS_TASK_WHEEL_COUNT; index++)
    {
        ret = chassis_task_init_pid(
            &context->wheel_speed_pids[index],
            APP_CHASSIS_WHEEL_SPEED_PID_KP,
            wheel_speed_ki,
            APP_CHASSIS_WHEEL_SPEED_PID_KD,
            APP_CHASSIS_WHEEL_SPEED_PID_OUT_LIMIT,
            APP_CHASSIS_WHEEL_SPEED_PID_INTEGRAL_LIMIT);
        if (ret != OM_OK)
        {
            return ret;
        }
    }

    for (index = 0u; index < CHASSIS_TASK_LEG_COUNT; index++)
    {
        ret = chassis_task_init_pid(
            &context->leg_angle_pids[index],
            APP_CHASSIS_LEG_ANGLE_PID_KP,
            APP_CHASSIS_LEG_ANGLE_PID_KI,
            APP_CHASSIS_LEG_ANGLE_PID_KD,
            APP_CHASSIS_LEG_ANGLE_PID_OUT_LIMIT,
            APP_CHASSIS_LEG_ANGLE_PID_INTEGRAL_LIMIT);
        if (ret != OM_OK)
        {
            return ret;
        }

        ret = chassis_task_init_pid(
            &context->leg_speed_pids[index],
            APP_CHASSIS_LEG_SPEED_PID_KP,
            APP_CHASSIS_LEG_SPEED_PID_KI,
            APP_CHASSIS_LEG_SPEED_PID_KD,
            APP_CHASSIS_LEG_SPEED_PID_OUT_LIMIT,
            APP_CHASSIS_LEG_SPEED_PID_INTEGRAL_LIMIT);
        if (ret != OM_OK)
        {
            return ret;
        }
    }

    return OM_OK;
}

static OmRet chassis_task_try_bind_motors(ChassisTaskContext* context)
{
    uint32_t index = 0u;

    if (context == OM_NULL)
    {
        return OM_ERROR_NULL;
    }

    context->motors_bound_flag = OM_FALSE;

    for (index = 0u; index < CHASSIS_TASK_WHEEL_COUNT; index++)
    {
        context->wheel_motors[index] = motor_find_by_name(g_chassis_task_wheel_names[index]);
        if (context->wheel_motors[index] == OM_NULL)
        {
            return OM_ERROR_NULL;
        }
    }

    for (index = 0u; index < CHASSIS_TASK_LEG_COUNT; index++)
    {
        context->leg_motors[index] = motor_find_by_name(g_chassis_task_leg_names[index]);
        if (context->leg_motors[index] == OM_NULL)
        {
            return OM_ERROR_NULL;
        }
    }

#if (BIG_YAW_TEMP_HOLD_ENABLE == 1u)
    context->big_yaw_motor = motor_find_by_name(g_chassis_task_big_yaw_name);
#else
    context->big_yaw_motor = OM_NULL;
#endif

    for (index = 0u; index < CHASSIS_TASK_WHEEL_COUNT; index++)
    {
        if (motor_set_control_mode(context->wheel_motors[index], MOTOR_CONTROL_MODE_CURRENT) != OM_OK)
        {
            return OM_ERROR;
        }
    }

    for (index = 0u; index < CHASSIS_TASK_LEG_COUNT; index++)
    {
        if (motor_set_control_mode(context->leg_motors[index], MOTOR_CONTROL_MODE_CURRENT) != OM_OK)
        {
            return OM_ERROR;
        }
    }

    if (context->big_yaw_motor != OM_NULL)
    {
        if (motor_set_control_mode(context->big_yaw_motor, MOTOR_CONTROL_MODE_ANGLE) != OM_OK)
        {
            return OM_ERROR;
        }
        context->big_yaw_hold_initialized = OM_FALSE;
    }

    context->motors_bound_flag = OM_TRUE;
    return OM_OK;
}

static void chassis_task_compute_keyboard_velocity(
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

static float chassis_task_compute_rc_rotate_velocity_deg_per_s(
    ChassisTaskContext* context,
    int16_t ch3,
    OsalTimeMs now_ms)
{
    float rotate_velocity_deg_per_s = 0.0f;

    if (context == OM_NULL)
    {
        return 0.0f;
    }

    if (chassis_task_abs_float((float)ch3) < APP_RC_RESOLUTION)
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

static void chassis_task_compute_chassis_velocity(
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

static OmBool chassis_task_mode_allows_chassis_motion(ChassisMode chassis_mode)
{
    switch (chassis_mode)
    {
    case MODE_CHASSIS_NORMAL:
    case MODE_CHASSIS_EXCHANGE:
    case MODE_CHASSIS_PRIMARY:
    case MODE_CHASSIS_GET_ENERGY_UNIT:
    case MODE_CHASSIS_GET_ENERGY_UNIT1:
    case MODE_CHASSIS_GET_ENERGY_UNIT2:
    case MODE_CHASSIS_SECONDARY_ORE:
        return OM_TRUE;
    default:
        return OM_FALSE;
    }
}

static OmBool chassis_task_mode_allows_leg_control(ChassisMode chassis_mode)
{
    switch (chassis_mode)
    {
    case MODE_CHASSIS_NORMAL:
    case MODE_CHASSIS_EXCHANGE:
    case MODE_CHASSIS_PRIMARY:
    case MODE_CHASSIS_GET_ENERGY_UNIT:
    case MODE_CHASSIS_GET_ENERGY_UNIT1:
    case MODE_CHASSIS_GET_ENERGY_UNIT2:
    case MODE_CHASSIS_SECONDARY_ORE:
    case MODE_CHASSIS_PITCH3_TORQUE_COLLECTION:
    case MODE_CHASSIS_CHECK:
    case MODE_CHASSIS_URGENT_MEASURE:
        return OM_TRUE;
    default:
        return OM_FALSE;
    }
}

static void chassis_task_update_leg_reference_deg(
    ChassisTaskContext* context,
    int16_t ch4,
    float leg_reference_deg[CHASSIS_TASK_LEG_COUNT])
{
    if (context == OM_NULL || leg_reference_deg == OM_NULL)
    {
        return;
    }

    context->pit_leg_cmd_deg += ((float)ch4) * APP_CHASSIS_LEG_PIT_CMD_STEP_PER_TICK_DEG;
    context->pit_leg_cmd_deg = chassis_task_clamp_float(
        context->pit_leg_cmd_deg,
        APP_CHASSIS_LEG_PIT_CMD_MIN_DEG,
        APP_CHASSIS_LEG_PIT_CMD_MAX_DEG);

    leg_reference_deg[CHASSIS_TASK_LEFT_LEG_INDEX] =
        chassis_task_clamp_float(
            APP_CHASSIS_LEFT_LEG_REF_BIAS_DEG - context->pit_leg_cmd_deg,
            APP_CHASSIS_LEFT_LEG_REF_MIN_DEG,
            APP_CHASSIS_LEFT_LEG_REF_MAX_DEG);
    leg_reference_deg[CHASSIS_TASK_RIGHT_LEG_INDEX] =
        chassis_task_clamp_float(
            APP_CHASSIS_RIGHT_LEG_REF_BIAS_DEG + context->pit_leg_cmd_deg,
            APP_CHASSIS_RIGHT_LEG_REF_MIN_DEG,
            APP_CHASSIS_RIGHT_LEG_REF_MAX_DEG);
}

static void chassis_task_reset_leg_command(ChassisTaskContext* context)
{
    uint32_t index = 0u;

    if (context == OM_NULL)
    {
        return;
    }

    context->pit_leg_cmd_deg = 0.0f;
    for (index = 0u; index < CHASSIS_TASK_LEG_COUNT; index++)
    {
        context->leg_speed_filtered_rpm[index] = 0.0f;
    }
}

static void chassis_task_apply_current_command(Motor* motor, float current)
{
    if (motor == OM_NULL)
    {
        return;
    }

    (void)motor_set_current(motor, current);
    (void)motor_control_compute(motor);
}

static void chassis_task_apply_zero_output(ChassisTaskContext* context)
{
    uint32_t index = 0u;

    if (context == OM_NULL)
    {
        return;
    }

    for (index = 0u; index < CHASSIS_TASK_WHEEL_COUNT; index++)
    {
        chassis_task_apply_current_command(context->wheel_motors[index], 0.0f);
    }

    for (index = 0u; index < CHASSIS_TASK_LEG_COUNT; index++)
    {
        chassis_task_apply_current_command(context->leg_motors[index], 0.0f);
    }
}

static void chassis_task_apply_big_yaw_hold(ChassisTaskContext* context)
{
#if (BIG_YAW_TEMP_HOLD_ENABLE == 1u)
    const MotorFeedback* feedback = OM_NULL;

    if (context == OM_NULL || context->big_yaw_motor == OM_NULL)
    {
        return;
    }

    feedback = motor_get_feedback(context->big_yaw_motor);
    if (context->big_yaw_hold_initialized != OM_TRUE)
    {
        if (feedback == OM_NULL || feedback->online != OM_TRUE)
        {
            return;
        }

        context->big_yaw_hold_angle_rad = feedback->angle;
        context->big_yaw_hold_initialized = OM_TRUE;
    }

    motor_set_angle(context->big_yaw_motor, context->big_yaw_hold_angle_rad);
    motor_set_speed(context->big_yaw_motor, 0.0f);
    motor_set_position_gains(
        context->big_yaw_motor,
        APP_CHASSIS_BIG_YAW_HOLD_KP,
        APP_CHASSIS_BIG_YAW_HOLD_KD);
    motor_set_torque(context->big_yaw_motor, 0.0f);
    (void)motor_control_compute(context->big_yaw_motor);
#else
    (void)context;
#endif
}

static void chassis_task_resolve_wheel_online_flags(
    ChassisTaskContext* context,
    OmBool wheel_online_flags[CHASSIS_TASK_WHEEL_COUNT],
    uint32_t* online_wheel_count,
    OmBool* degraded_mode_enabled,
    MecanumWheelId* offline_wheel_id)
{
    uint32_t index = 0u;
    uint32_t offline_wheel_count = 0u;
    MecanumWheelId last_offline_wheel_id = MECANUM_WHEEL_FRONT_RIGHT;

    if (wheel_online_flags == OM_NULL || online_wheel_count == OM_NULL ||
        degraded_mode_enabled == OM_NULL || offline_wheel_id == OM_NULL)
    {
        return;
    }

    *online_wheel_count = 0u;
    *degraded_mode_enabled = OM_FALSE;
    *offline_wheel_id = MECANUM_WHEEL_FRONT_RIGHT;

    for (index = 0u; index < CHASSIS_TASK_WHEEL_COUNT; index++)
    {
        const MotorFeedback* feedback = OM_NULL;

        wheel_online_flags[index] = OM_FALSE;

        if (context == OM_NULL || context->wheel_motors[index] == OM_NULL)
        {
            offline_wheel_count++;
            last_offline_wheel_id = (MecanumWheelId)index;
            continue;
        }

        feedback = motor_get_feedback(context->wheel_motors[index]);
        if (chassis_task_feedback_recent(feedback, g_chassis_task_wheel_feedback_timeout_ms) == OM_TRUE)
        {
            wheel_online_flags[index] = OM_TRUE;
            (*online_wheel_count)++;
            continue;
        }

        offline_wheel_count++;
        last_offline_wheel_id = (MecanumWheelId)index;
    }

    if (offline_wheel_count == 1u)
    {
        /* 恰好掉 1 个轮电机时启用三麦轮降级，2 个及以上则不再维持底盘运动。 */
        *degraded_mode_enabled = OM_TRUE;
        *offline_wheel_id = last_offline_wheel_id;
    }
}

static void chassis_task_apply_wheel_control(
    ChassisTaskContext* context,
    const int16_t wheel_speed_ref_rpm[CHASSIS_TASK_WHEEL_COUNT],
    const OmBool wheel_active_flags[CHASSIS_TASK_WHEEL_COUNT],
    float current_tick_s)
{
    uint32_t index = 0u;

    if (context == OM_NULL || wheel_speed_ref_rpm == OM_NULL || wheel_active_flags == OM_NULL)
    {
        return;
    }

    for (index = 0u; index < CHASSIS_TASK_WHEEL_COUNT; index++)
    {
        const MotorFeedback* feedback = motor_get_feedback(context->wheel_motors[index]);
        float wheel_speed_fdb_rpm = 0.0f;
        float current_cmd = 0.0f;

        if (wheel_active_flags[index] != OM_TRUE ||
            chassis_task_feedback_recent(feedback, g_chassis_task_wheel_feedback_timeout_ms) != OM_TRUE)
        {
            pid_reset(&context->wheel_speed_pids[index]);
            chassis_task_apply_current_command(context->wheel_motors[index], 0.0f);
            continue;
        }

        wheel_speed_fdb_rpm = chassis_task_rad_per_s_to_rpm(feedback->speed);
        current_cmd =
            pid_compute(
                &context->wheel_speed_pids[index],
                (float)wheel_speed_ref_rpm[index],
                wheel_speed_fdb_rpm,
                current_tick_s);

        chassis_task_apply_current_command(context->wheel_motors[index], current_cmd);
    }
}

static void chassis_task_apply_leg_control(
    ChassisTaskContext* context,
    const float leg_reference_deg[CHASSIS_TASK_LEG_COUNT],
    float current_tick_s)
{
    uint32_t index = 0u;

    if (context == OM_NULL || leg_reference_deg == OM_NULL)
    {
        return;
    }

    for (index = 0u; index < CHASSIS_TASK_LEG_COUNT; index++)
    {
        const MotorFeedback* feedback = motor_get_feedback(context->leg_motors[index]);
        const float leg_angle_fdb_deg =
            (feedback != OM_NULL) ? chassis_task_normalize_deg(chassis_task_rad_to_deg(feedback->angle)) : 0.0f;
        const float leg_speed_fdb_rpm =
            (feedback != OM_NULL) ? chassis_task_rad_per_s_to_rpm(feedback->speed) : 0.0f;
        float leg_speed_ref_rpm = 0.0f;
        float leg_current_raw = 0.0f;

        context->leg_speed_filtered_rpm[index] =
            0.16f * leg_speed_fdb_rpm + 0.84f * context->leg_speed_filtered_rpm[index];

        leg_speed_ref_rpm =
            pid_compute(
                &context->leg_angle_pids[index],
                leg_reference_deg[index],
                leg_angle_fdb_deg,
                current_tick_s);
        leg_current_raw =
            pid_compute(
                &context->leg_speed_pids[index],
                leg_speed_ref_rpm,
                context->leg_speed_filtered_rpm[index],
                current_tick_s);

        chassis_task_apply_current_command(context->leg_motors[index], leg_current_raw / 100.0f);
    }
}

static void chassis_task_run_once(ChassisTaskContext* context)
{
    ChassisTaskInputSnapshot snapshot = {0};
    float vx_mm_per_s = 0.0f;
    float vy_mm_per_s = 0.0f;
    float vw_deg_per_s = 0.0f;
    int16_t wheel_speed_ref_rpm[CHASSIS_TASK_WHEEL_COUNT] = {0};
    OmBool wheel_online_flags[CHASSIS_TASK_WHEEL_COUNT] = {OM_FALSE};
    float leg_reference_deg[CHASSIS_TASK_LEG_COUNT] = {0.0f};
    uint32_t online_wheel_count = 0u;
    OmBool degraded_mode_enabled = OM_FALSE;
    MecanumWheelId offline_wheel_id = MECANUM_WHEEL_FRONT_RIGHT;
    const float current_tick_s = chassis_task_now_s();

    if (context == OM_NULL)
    {
        return;
    }

    if (context->motors_bound_flag != OM_TRUE)
    {
        if (chassis_task_try_bind_motors(context) != OM_OK)
        {
            return;
        }
    }

    chassis_task_load_snapshot(&snapshot);
    chassis_task_resolve_wheel_online_flags(
        context,
        wheel_online_flags,
        &online_wheel_count,
        &degraded_mode_enabled,
        &offline_wheel_id);

    if (snapshot.chassis_mode == MODE_CHASSIS_RELEASE)
    {
        chassis_task_reset_leg_command(context);
        chassis_task_apply_zero_output(context);
    }
    else if (chassis_task_mode_allows_chassis_motion(snapshot.chassis_mode) == OM_TRUE)
    {
        chassis_task_compute_chassis_velocity(context, &snapshot, &vx_mm_per_s, &vy_mm_per_s, &vw_deg_per_s);
        if (online_wheel_count >= CHASSIS_TASK_WHEEL_COUNT)
        {
            mecanum_calc(vx_mm_per_s, vy_mm_per_s, vw_deg_per_s, wheel_speed_ref_rpm);
        }
        else if (degraded_mode_enabled == OM_TRUE)
        {
            mecanum_calc_three_wheel(vx_mm_per_s, vy_mm_per_s, vw_deg_per_s, offline_wheel_id, wheel_speed_ref_rpm);
        }

        chassis_task_update_leg_reference_deg(context, snapshot.ch4, leg_reference_deg);
        chassis_task_apply_wheel_control(context, wheel_speed_ref_rpm, wheel_online_flags, current_tick_s);
        chassis_task_apply_leg_control(context, leg_reference_deg, current_tick_s);
    }
    else if (chassis_task_mode_allows_leg_control(snapshot.chassis_mode) == OM_TRUE)
    {
        chassis_task_update_leg_reference_deg(context, snapshot.ch4, leg_reference_deg);
        chassis_task_apply_wheel_control(context, wheel_speed_ref_rpm, wheel_online_flags, current_tick_s);
        chassis_task_apply_leg_control(context, leg_reference_deg, current_tick_s);
    }
    else
    {
        chassis_task_apply_zero_output(context);
    }

    chassis_task_apply_big_yaw_hold(context);

    if (event_bus_publish(&g_event_bus, EVT_MOTOR_TX_REQUEST) != OSAL_OK)
    {
        sh_report_fatal(
            SH_ERR_EVT_MOTOR_TX_REQUEST_PUBLISH_FAIL,
            "event_bus_publish EVT_MOTOR_TX_REQUEST failed");
        for (;;)
        {
            osal_sleep_ms(1000u);
        }
    }
}

static void chassis_task_entry(void* arg)
{
    ChassisTaskContext* context = (ChassisTaskContext*)arg;
    OsalTimeMs deadline_cursor_ms = 0u;

    while (1)
    {
        chassis_task_run_once(context);
        (void)sh_beat(SH_TASK_CHASSIS);
        (void)osal_delay_until(&deadline_cursor_ms, CHASSIS_TASK_PERIOD_MS, OM_NULL);
    }
}

OmRet chassis_task_start(void)
{
    static OsalThread* chassis_task_thread = OM_NULL;
    static ChassisTaskContext chassis_task_context = {0};
    const OsalThreadAttr chassis_task_attr = {
        "chassis_task",
        CHASSIS_TASK_STACK_BYTES,
        CHASSIS_TASK_PRIORITY};
    OsalStatus status = OSAL_INVALID;
    OmRet ret = OM_OK;

    if (chassis_task_thread != OM_NULL)
    {
        return OM_ERR_CONFLICT;
    }

    memset(&chassis_task_context, 0, sizeof(chassis_task_context));
    ret = chassis_task_init_pids(&chassis_task_context);
    if (ret != OM_OK)
    {
        return ret;
    }

    status = osal_thread_create(
        &chassis_task_thread,
        &chassis_task_attr,
        chassis_task_entry,
        &chassis_task_context);
    if (status != OSAL_OK)
    {
        chassis_task_thread = OM_NULL;
        return OM_ERROR;
    }

    return OM_OK;
}
