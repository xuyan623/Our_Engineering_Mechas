#include "task/chassis_task/chassis_task.h"

#include "algorithm/kinematics/kinematics.h"
#include "config/app_config.h"
#include "core/algorithm/controller/pid.h"
#include "driver/motor/motor.h"
#include "module/data_pool/data_pool.h"
#include "module/event_bus/event_bus.h"
#include "module/motor_tx_dispatch/motor_tx_dispatch.h"
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

/* 底盘轮控需要比通用 online 判据更快地切断 stale feedback，
 * 但 20ms 在当前 5ms 通信循环 + CAN1 多电机负载下过紧，容易把瞬时调度抖动误判成四轮同时离线。
 * 这里放宽到 50ms，仍显著快于 driver 层 100ms online 超时，同时避免遥控正常但底盘被误清零。
 */
static const uint32_t g_chassis_task_wheel_feedback_timeout_ms = 50u;

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

static OmBool chassis_task_motor_feedback_recent(const Motor* motor, uint32_t timeout_ms)
{
    const MotorFeedback* feedback = OM_NULL;
    uint32_t timestamp_ms = 0u;

    if (motor == OM_NULL)
    {
        return OM_FALSE;
    }

    if (motor->config.vendor == MOTOR_VENDOR_DJI && motor->binding.dji.driver != OM_NULL)
    {
        timestamp_ms = dji_motor_get_feedback_timestamp_ms(motor->binding.dji.driver);
        return (timestamp_ms != 0u && (uint32_t)(osal_time_now_monotonic() - timestamp_ms) <= timeout_ms) ? OM_TRUE : OM_FALSE;
    }

    feedback = motor_get_feedback(motor);
    return chassis_task_feedback_recent(feedback, timeout_ms);
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
    case MODE_CHASSIS_CUSTOM_CONTROLLER_NORMAL:
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
    case MODE_CHASSIS_CUSTOM_CONTROLLER_NORMAL:
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

static void chassis_task_reset_wheel_control_state(ChassisTaskContext* context)
{
    uint32_t index = 0u;

    if (context == OM_NULL)
    {
        return;
    }

    for (index = 0u; index < CHASSIS_TASK_WHEEL_COUNT; index++)
    {
        pid_reset(&context->wheel_speed_pids[index]);
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

    chassis_task_reset_wheel_control_state(context);

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

/**
 * @brief 解析轮子在线状态标志，检测离线轮子并判断是否启用降级模式
 * 
 * 该函数遍历所有麦轮电机，检查每个电机的反馈数据是否有效（在线），
 * 统计在线和离线的轮子数量。当恰好有1个轮子离线时，启用三麦轮降级模式；
 * 当2个及以上轮子离线时，不启用降级模式，底盘将停止运动。
 * 
 * @param context 底盘任务上下文指针，包含轮子电机信息
 * @param wheel_online_flags 输出参数，长度为CHASSIS_TASK_WHEEL_COUNT的布尔数组，
 *                           用于存储每个轮子的在线状态（OM_TRUE表示在线）
 * @param online_wheel_count 输出参数，指向在线轮子数量的指针
 * @param degraded_mode_enabled 输出参数，指向是否启用降级模式的指针，
 *                              OM_TRUE表示启用三麦轮降级模式
 * @param offline_wheel_id 输出参数，指向离线轮子ID的指针，
 *                         仅在恰好1个轮子离线时有效
 * 
 * @note 如果任一输出参数为OM_NULL，函数将直接返回而不执行任何操作
 * @note 降级模式仅在恰好1个轮子离线时启用，多个轮子离线时底盘将停止运动
 */
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

    /* 参数有效性检查 */
    if (wheel_online_flags == OM_NULL || online_wheel_count == OM_NULL ||
        degraded_mode_enabled == OM_NULL || offline_wheel_id == OM_NULL)
    {
        return;
    }

    /* 初始化输出参数 */
    *online_wheel_count = 0u;
    *degraded_mode_enabled = OM_FALSE;
    *offline_wheel_id = MECANUM_WHEEL_FRONT_RIGHT;

    /* 遍历所有轮子，检测每个轮子的在线状态 */
    for (index = 0u; index < CHASSIS_TASK_WHEEL_COUNT; index++)
    {
        const MotorFeedback* feedback = OM_NULL;

        wheel_online_flags[index] = OM_FALSE;

        /* 检查上下文和电机指针是否有效 */
        if (context == OM_NULL || context->wheel_motors[index] == OM_NULL)
        {
            offline_wheel_count++;
            last_offline_wheel_id = (MecanumWheelId)index;
            continue;
        }

        feedback = motor_get_feedback(context->wheel_motors[index]);
        /* 检查电机反馈数据是否在有效时间范围内 */
        if (chassis_task_motor_feedback_recent(context->wheel_motors[index], g_chassis_task_wheel_feedback_timeout_ms) == OM_TRUE)
        {
            wheel_online_flags[index] = OM_TRUE;
            (*online_wheel_count)++;
            continue;
        }

        offline_wheel_count++;
        last_offline_wheel_id = (MecanumWheelId)index;
    }

    /* 根据离线轮子数量决定是否启用降级模式 */
    if (offline_wheel_count == 1u)
    {
        /* 恰好掉 1 个轮电机时启用三麦轮降级，2 个及以上则不再维持底盘运动。 */
        *degraded_mode_enabled = OM_TRUE;
        *offline_wheel_id = last_offline_wheel_id;
    }
}

/**
 * @brief 应用底盘轮速控制指令到各个电机
 * 
 * 该函数遍历所有轮子电机，根据参考转速和反馈信息计算并应用电流控制指令。
 * 对于非活跃或反馈超时的电机会重置PID控制器并停止输出。
 * 
 * @param context 底盘任务上下文指针，包含电机句柄和PID控制器等状态信息
 * @param wheel_speed_ref_rpm 各轮子的参考转速数组（单位：RPM），长度为CHASSIS_TASK_WHEEL_COUNT
 * @param wheel_active_flags 各轮子的激活标志数组，OM_TRUE表示该轮子需要控制
 * @param current_tick_s 当前时间戳（单位：秒），用于PID微分计算
 * 
 * @return 无返回值
 */
static void chassis_task_apply_wheel_control(
    ChassisTaskContext* context,
    const int16_t wheel_speed_ref_rpm[CHASSIS_TASK_WHEEL_COUNT],
    const OmBool wheel_active_flags[CHASSIS_TASK_WHEEL_COUNT],
    float current_tick_s)
{
    uint32_t index = 0u;

    /* 参数有效性检查 */
    if (context == OM_NULL || wheel_speed_ref_rpm == OM_NULL || wheel_active_flags == OM_NULL)
    {
        return;
    }

    /* 遍历所有轮子电机，逐个应用速度控制 */
    for (index = 0u; index < CHASSIS_TASK_WHEEL_COUNT; index++)
    {
        const MotorFeedback* feedback = motor_get_feedback(context->wheel_motors[index]);
        float wheel_speed_fdb_rpm = 0.0f;
        float current_cmd = 0.0f;

        /* 检查轮子是否激活且反馈数据有效，否则重置PID并停止电机 */
        if (wheel_active_flags[index] != OM_TRUE ||
            chassis_task_motor_feedback_recent(context->wheel_motors[index], g_chassis_task_wheel_feedback_timeout_ms) != OM_TRUE)
        {
            pid_reset(&context->wheel_speed_pids[index]);
            chassis_task_apply_current_command(context->wheel_motors[index], 0.0f);
            continue;
        }

        /* 将角速度反馈转换为RPM单位 */
        wheel_speed_fdb_rpm = chassis_task_rad_per_s_to_rpm(feedback->speed);
        
        /* 使用PID控制器计算电流控制指令 */
        current_cmd =
            pid_compute(
                &context->wheel_speed_pids[index],
                (float)wheel_speed_ref_rpm[index],
                wheel_speed_fdb_rpm,
                current_tick_s);

        /* 应用电流控制指令到电机 */
        chassis_task_apply_current_command(context->wheel_motors[index], current_cmd);
    }
}

/**
 * @brief 应用腿部电机的控制指令，通过级联PID实现角度闭环控制
 * 
 * 该函数对每个腿部电机执行以下控制流程：
 * 1. 获取电机反馈（角度和速度）
 * 2. 对速度反馈进行一阶低通滤波
 * 3. 外环角度PID计算期望转速
 * 4. 内环速度PID计算期望电流
 * 5. 下发最终电流指令
 * 
 * @param context 底盘任务上下文指针，包含电机句柄、PID控制器等状态信息
 * @param leg_reference_deg 腿部目标角度数组（单位：度），长度为CHASSIS_TASK_LEG_COUNT
 * @param current_tick_s 当前时间戳（单位：秒），用于PID积分项计算
 */
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

        /* 对速度反馈进行一阶低通滤波，抑制高频噪声 */
        context->leg_speed_filtered_rpm[index] =
            0.16f * leg_speed_fdb_rpm + 0.84f * context->leg_speed_filtered_rpm[index];

        /* 外环：角度PID控制器计算期望转速 */
        leg_speed_ref_rpm =
            pid_compute(
                &context->leg_angle_pids[index],
                leg_reference_deg[index],
                leg_angle_fdb_deg,
                current_tick_s);
        
        /* 内环：速度PID控制器计算期望电流 */
        leg_current_raw =
            pid_compute(
                &context->leg_speed_pids[index],
                leg_speed_ref_rpm,
                context->leg_speed_filtered_rpm[index],
                current_tick_s);

        chassis_task_apply_current_command(context->leg_motors[index], leg_current_raw / 100.0f);
    }
}

/**
 * @brief 底盘任务单次执行主循环
 * 
 * 该函数是底盘控制任务的核心执行函数，负责：
 * 1. 电机绑定与初始化检查
 * 2. 输入数据采集与轮子在线状态检测
 * 3. 根据底盘模式计算运动学解算（支持四轮全向和三轮降级模式）
 * 4. 腿部关节角度控制
 * 5. 大云台保持控制
 * 6. 电机控制指令下发
 * 
 * @param context 底盘任务上下文指针，包含电机句柄、PID控制器状态等信息
 *                如果为NULL则直接返回
 * 
 * @return 无返回值
 * 
 * @note 函数内部会根据不同的底盘模式执行不同的控制逻辑：
 *       - MODE_CHASSIS_RELEASE: 释放模式，清零所有输出
 *       - 允许底盘运动的模式: 执行速度解算和轮控/腿控
 *       - 仅允许腿控的模式: 仅执行腿控
 *       - 其他模式: 清零输出
 * 
 * @note 支持降级运行模式：当检测到某个轮子离线时，自动切换到三轮运动学解算
 * 
 * @note 如果事件总线发布失败，会触发致命错误并进入死循环
 */
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

    /* 确保电机已绑定，未绑定时尝试绑定 */
    if (context->motors_bound_flag != OM_TRUE)
    {
        if (chassis_task_try_bind_motors(context) != OM_OK)
        {
            return;
        }
    }

    /* 加载当前输入快照并解析轮子在线状态 */
    chassis_task_load_snapshot(&snapshot);
    chassis_task_resolve_wheel_online_flags(
        context,
        wheel_online_flags,
        &online_wheel_count,
        &degraded_mode_enabled,
        &offline_wheel_id);

    /* 根据底盘模式执行相应的控制逻辑 */
    if (snapshot.chassis_mode == MODE_CHASSIS_RELEASE)
    {
        /* 释放模式：重置腿部命令并清零所有输出 */
        chassis_task_reset_leg_command(context);
        chassis_task_apply_zero_output(context);
    }
    else if (chassis_task_mode_allows_chassis_motion(snapshot.chassis_mode) == OM_TRUE)
    {
        /* 计算底盘期望速度并进行运动学解算 */
        chassis_task_compute_chassis_velocity(context, &snapshot, &vx_mm_per_s, &vy_mm_per_s, &vw_deg_per_s);
        if (online_wheel_count >= CHASSIS_TASK_WHEEL_COUNT)
        {
            /* 四轮在线：使用标准四轮全向运动学解算 */
            mecanum_calc(vx_mm_per_s, vy_mm_per_s, vw_deg_per_s, wheel_speed_ref_rpm);
        }
        else if (degraded_mode_enabled == OM_TRUE)
        {
            /* 降级模式：使用三轮运动学解算（排除离线轮） */
            mecanum_calc_three_wheel(vx_mm_per_s, vy_mm_per_s, vw_deg_per_s, offline_wheel_id, wheel_speed_ref_rpm);
        }
        else
        {
            /* 异常状态：清除在线标志并重置轮控状态 */
            memset(wheel_online_flags, 0, sizeof(wheel_online_flags));
            chassis_task_reset_wheel_control_state(context);
        }

        /* 更新腿部参考角度并应用轮控和腿控 */
        chassis_task_update_leg_reference_deg(context, snapshot.ch4, leg_reference_deg);
        chassis_task_apply_wheel_control(context, wheel_speed_ref_rpm, wheel_online_flags, current_tick_s);
        chassis_task_apply_leg_control(context, leg_reference_deg, current_tick_s);
    }
    else if (chassis_task_mode_allows_leg_control(snapshot.chassis_mode) == OM_TRUE)
    {
        /* 仅腿控模式：更新腿部参考角度并应用腿控 */
        chassis_task_update_leg_reference_deg(context, snapshot.ch4, leg_reference_deg);
        chassis_task_apply_wheel_control(context, wheel_speed_ref_rpm, wheel_online_flags, current_tick_s);
        chassis_task_apply_leg_control(context, leg_reference_deg, current_tick_s);
    }
    else
    {
        /* 其他模式：清零所有输出 */
        chassis_task_apply_zero_output(context);
    }

    /* 应用大云台保持控制 */
    chassis_task_apply_big_yaw_hold(context);

    /* 提交电机发送请求到分发器 */
    (void)motor_tx_dispatch_submit(MOTOR_TX_SOURCE_CHASSIS);

    /* 发布电机发送请求事件，失败则触发致命错误 */
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

/**
 * @brief 启动底盘控制任务
 * 
 * 该函数负责初始化并启动底盘控制任务线程。主要完成以下工作：
 * 1. 检查任务是否已启动，防止重复创建
 * 2. 初始化任务上下文和PID控制器参数
 * 3. 创建底盘任务线程并开始执行
 * 
 * @return OmRet 返回操作结果
 *         - OM_OK: 任务启动成功
 *         - OM_ERR_CONFLICT: 任务已经启动，不能重复启动
 *         - OM_ERROR: 线程创建失败
 *         - 其他错误码: PID初始化失败时返回的具体错误码
 */
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

    /* 检查任务是否已经启动，防止重复创建 */
    if (chassis_task_thread != OM_NULL)
    {
        return OM_ERR_CONFLICT;
    }

    /* 初始化任务上下文并配置PID控制器 */
    memset(&chassis_task_context, 0, sizeof(chassis_task_context));
    ret = chassis_task_init_pids(&chassis_task_context);
    if (ret != OM_OK)
    {
        return ret;
    }

    /* 创建底盘任务线程 */
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
