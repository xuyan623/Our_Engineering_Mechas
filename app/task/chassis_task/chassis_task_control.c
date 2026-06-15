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

/* 底盘轮控需要比通用 online 判据更快地切断 stale feedback，
 * 但 20ms 在当前 5ms 通信循环 + CAN1 多电机负载下过紧，容易把瞬时调度抖动误判成四轮同时离线。
 * 这里放宽到 50ms，仍显著快于 driver 层 100ms online 超时，同时避免遥控正常但底盘被误清零。
 */
const uint32_t g_chassis_task_wheel_feedback_timeout_ms = 50u;

const char* g_chassis_task_wheel_names[CHASSIS_TASK_WHEEL_COUNT] = {
    APP_MOTOR_NAME_CHASSIS_FR,
    APP_MOTOR_NAME_CHASSIS_FL,
    APP_MOTOR_NAME_CHASSIS_BL,
    APP_MOTOR_NAME_CHASSIS_BR,
};
const uint8_t g_chassis_task_wheel_roles[CHASSIS_TASK_WHEEL_COUNT] = {
    APP_MOTOR_ROLE_CHASSIS_FR,
    APP_MOTOR_ROLE_CHASSIS_FL,
    APP_MOTOR_ROLE_CHASSIS_BL,
    APP_MOTOR_ROLE_CHASSIS_BR,
};

const char* g_chassis_task_leg_names[CHASSIS_TASK_LEG_COUNT] = {
    APP_MOTOR_NAME_JOINT_LEG_R,
    APP_MOTOR_NAME_JOINT_LEG_L,
};
const uint8_t g_chassis_task_leg_roles[CHASSIS_TASK_LEG_COUNT] = {
    APP_MOTOR_ROLE_JOINT_LEG_R,
    APP_MOTOR_ROLE_JOINT_LEG_L,
};
const char* g_chassis_task_big_yaw_name = APP_MOTOR_NAME_BIG_YAW;
Motor* g_chassis_task_wheel_motor_cache[CHASSIS_TASK_WHEEL_COUNT] = {0};
Motor* g_chassis_task_leg_motor_cache[CHASSIS_TASK_LEG_COUNT] = {0};
Motor* g_chassis_task_big_yaw_motor = OM_NULL;
PidController g_front_wheel_speed_pids[CHASSIS_TASK_FRONT_WHEEL_COUNT] = {0};
PidController g_rear_wheel_speed_pids[CHASSIS_TASK_REAR_WHEEL_COUNT] = {0};
PidController g_leg_angle_pids[CHASSIS_TASK_LEG_COUNT] = {0};

OmRet chassis_task_init_pid(
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

OmRet chassis_task_init_pids(ChassisTaskContext* context)
{
    const float task_period_s = ((float)CHASSIS_TASK_PERIOD_MS) / 1000.0f;
    const float front_wheel_speed_ki = APP_CHASSIS_FRONT_WHEEL_SPEED_PID_KI / task_period_s;
    const float rear_wheel_speed_ki = APP_CHASSIS_REAR_WHEEL_SPEED_PID_KI / task_period_s;
    uint32_t index = 0u;
    OmRet ret = OM_OK;

    if (context == OM_NULL)
    {
        return OM_ERROR_NULL;
    }

    for (index = 0u; index < CHASSIS_TASK_FRONT_WHEEL_COUNT; index++)
    {
        ret = chassis_task_init_pid(
            &g_front_wheel_speed_pids[index],
            APP_CHASSIS_FRONT_WHEEL_SPEED_PID_KP,
            front_wheel_speed_ki,
            APP_CHASSIS_FRONT_WHEEL_SPEED_PID_KD,
            APP_CHASSIS_FRONT_WHEEL_SPEED_PID_OUT_LIMIT,
            APP_CHASSIS_FRONT_WHEEL_SPEED_PID_INTEGRAL_LIMIT);
        if (ret != OM_OK)
        {
            return ret;
        }
    }

    for (index = 0u; index < CHASSIS_TASK_REAR_WHEEL_COUNT; index++)
    {
        ret = chassis_task_init_pid(
            &g_rear_wheel_speed_pids[index],
            APP_CHASSIS_REAR_WHEEL_SPEED_PID_KP,
            rear_wheel_speed_ki,
            APP_CHASSIS_REAR_WHEEL_SPEED_PID_KD,
            APP_CHASSIS_REAR_WHEEL_SPEED_PID_OUT_LIMIT,
            APP_CHASSIS_REAR_WHEEL_SPEED_PID_INTEGRAL_LIMIT);
        if (ret != OM_OK)
        {
            return ret;
        }
    }

    for (index = 0u; index < CHASSIS_TASK_LEG_COUNT; index++)
    {
        ret = chassis_task_init_pid(
            &g_leg_angle_pids[index],
            APP_CHASSIS_LEG_ANGLE_PID_KP,
            APP_CHASSIS_LEG_ANGLE_PID_KI,
            APP_CHASSIS_LEG_ANGLE_PID_KD,
            APP_CHASSIS_LEG_ANGLE_PID_OUT_LIMIT,
            APP_CHASSIS_LEG_ANGLE_PID_INTEGRAL_LIMIT);
        if (ret != OM_OK)
        {
            return ret;
        }
    }

    return OM_OK;
}

OmRet chassis_task_try_bind_motors(ChassisTaskContext* context)
{
    uint32_t index = 0u;

    if (context == OM_NULL)
    {
        return OM_ERROR_NULL;
    }

    context->flags &= ~CHASSIS_TASK_FLAG_MOTORS_BOUND;

    for (index = 0u; index < CHASSIS_TASK_WHEEL_COUNT; index++)
    {
        if (chassis_task_wheel_profile_is_present(index) != OM_TRUE)
        {
            g_chassis_task_wheel_motor_cache[index] = OM_NULL;
            continue;
        }

        g_chassis_task_wheel_motor_cache[index] = motor_find_by_name(g_chassis_task_wheel_names[index]);
        if (chassis_task_get_wheel_motor(index) == OM_NULL)
        {
            return OM_ERROR_NULL;
        }
    }

    for (index = 0u; index < CHASSIS_TASK_LEG_COUNT; index++)
    {
        if (chassis_task_leg_profile_is_present(index) != OM_TRUE)
        {
            g_chassis_task_leg_motor_cache[index] = OM_NULL;
            continue;
        }

        g_chassis_task_leg_motor_cache[index] = motor_find_by_name(g_chassis_task_leg_names[index]);
        if (chassis_task_get_leg_motor(index) == OM_NULL)
        {
            return OM_ERROR_NULL;
        }
    }

#if (BIG_YAW_TEMP_HOLD_ENABLE == 1u)
    chassis_task_get_big_yaw_motor() = motor_find_by_name(g_chassis_task_big_yaw_name);
#else
    g_chassis_task_big_yaw_motor = OM_NULL;
#endif

    for (index = 0u; index < CHASSIS_TASK_WHEEL_COUNT; index++)
    {
        MotorControlMode control_mode =
            (chassis_task_wheel_profile_allows_control(index) == OM_TRUE) ?
                MOTOR_CONTROL_MODE_CURRENT :
                MOTOR_CONTROL_MODE_DISABLED;

        if (chassis_task_get_wheel_motor(index) == OM_NULL)
        {
            continue;
        }

        if (motor_set_control_mode(chassis_task_get_wheel_motor(index), control_mode) != OM_OK)
        {
            return OM_ERROR;
        }
    }

    for (index = 0u; index < CHASSIS_TASK_LEG_COUNT; index++)
    {
        MotorControlMode control_mode =
            (chassis_task_leg_profile_allows_control(index) == OM_TRUE) ?
                MOTOR_CONTROL_MODE_CURRENT :
                MOTOR_CONTROL_MODE_DISABLED;

        if (chassis_task_get_leg_motor(index) == OM_NULL)
        {
            continue;
        }

        if (motor_set_control_mode(chassis_task_get_leg_motor(index), control_mode) != OM_OK)
        {
            return OM_ERROR;
        }
    }

    if (chassis_task_get_big_yaw_motor() != OM_NULL)
    {
        if (motor_set_control_mode(chassis_task_get_big_yaw_motor(), MOTOR_CONTROL_MODE_DISABLED) != OM_OK)
        {
            return OM_ERROR;
        }
        context->flags &= ~CHASSIS_TASK_FLAG_BIG_YAW_HOLD_INIT;
    }

    context->flags |= CHASSIS_TASK_FLAG_MOTORS_BOUND;
    return OM_OK;
}

OmRet chassis_task_restore_control_modes(ChassisTaskContext* context)
{
    uint32_t index = 0u;

    if (context == OM_NULL || !(context->flags & CHASSIS_TASK_FLAG_MOTORS_BOUND))
    {
        return OM_ERROR_NULL;
    }

    for (index = 0u; index < CHASSIS_TASK_WHEEL_COUNT; index++)
    {
        MotorControlMode control_mode =
            (chassis_task_wheel_profile_allows_control(index) == OM_TRUE) ?
                MOTOR_CONTROL_MODE_CURRENT :
                MOTOR_CONTROL_MODE_DISABLED;

        if (chassis_task_get_wheel_motor(index) == OM_NULL)
        {
            continue;
        }

        if (motor_set_control_mode(chassis_task_get_wheel_motor(index), control_mode) != OM_OK)
        {
            return OM_ERROR;
        }
    }

    for (index = 0u; index < CHASSIS_TASK_LEG_COUNT; index++)
    {
        MotorControlMode control_mode =
            (chassis_task_leg_profile_allows_control(index) == OM_TRUE) ?
                MOTOR_CONTROL_MODE_CURRENT :
                MOTOR_CONTROL_MODE_DISABLED;

        if (chassis_task_get_leg_motor(index) == OM_NULL)
        {
            continue;
        }

        if (motor_set_control_mode(chassis_task_get_leg_motor(index), control_mode) != OM_OK)
        {
            return OM_ERROR;
        }
    }

    if (chassis_task_get_big_yaw_motor() != OM_NULL)
    {
        if (motor_set_control_mode(chassis_task_get_big_yaw_motor(), MOTOR_CONTROL_MODE_DISABLED) != OM_OK)
        {
            return OM_ERROR;
        }
    }

    return OM_OK;
}

void chassis_task_update_leg_reference_deg(
    ChassisTaskContext* context,
    int16_t ch4,
    float leg_reference_deg[CHASSIS_TASK_LEG_COUNT])
{
    if (context == OM_NULL || leg_reference_deg == OM_NULL)
    {
        return;
    }

    context->pit_leg_cmd_deg += ((float)ch4) * APP_CHASSIS_LEG_PIT_CMD_STEP_PER_TICK_DEG;
    context->pit_leg_cmd_deg = math_utils_clamp_float(
        context->pit_leg_cmd_deg,
        APP_CHASSIS_LEG_PIT_CMD_MIN_DEG,
        APP_CHASSIS_LEG_PIT_CMD_MAX_DEG);

    leg_reference_deg[CHASSIS_TASK_LEFT_LEG_INDEX] =
        math_utils_clamp_float(
            APP_CHASSIS_LEFT_LEG_REF_BIAS_DEG - context->pit_leg_cmd_deg,
            APP_CHASSIS_LEFT_LEG_REF_MIN_DEG,
            APP_CHASSIS_LEFT_LEG_REF_MAX_DEG);
    leg_reference_deg[CHASSIS_TASK_RIGHT_LEG_INDEX] =
        math_utils_clamp_float(
            APP_CHASSIS_RIGHT_LEG_REF_BIAS_DEG + context->pit_leg_cmd_deg,
            APP_CHASSIS_RIGHT_LEG_REF_MIN_DEG,
            APP_CHASSIS_RIGHT_LEG_REF_MAX_DEG);
}

void chassis_task_reset_leg_command(ChassisTaskContext* context)
{
    uint32_t index = 0u;

    if (context == OM_NULL)
    {
        return;
    }

    context->pit_leg_cmd_deg = 0.0f;
}

void chassis_task_reset_wheel_control_state(ChassisTaskContext* context)
{
    uint32_t index = 0u;

    if (context == OM_NULL)
    {
        return;
    }

    for (index = 0u; index < CHASSIS_TASK_WHEEL_COUNT; index++)
    {
        PidController* wheel_speed_pid = chassis_task_get_wheel_speed_pid(context, index);

        if (wheel_speed_pid != OM_NULL)
        {
            pid_reset(wheel_speed_pid);
        }
    }
}

void chassis_task_apply_current_command(Motor* motor, float current)
{
    if (motor == OM_NULL)
    {
        return;
    }

    (void)motor_set_current(motor, current);
    (void)motor_control_compute(motor);
}

void chassis_task_apply_zero_output(ChassisTaskContext* context)
{
    uint32_t index = 0u;

    if (context == OM_NULL)
    {
        return;
    }

    chassis_task_reset_wheel_control_state(context);

    for (index = 0u; index < CHASSIS_TASK_WHEEL_COUNT; index++)
    {
        chassis_task_apply_current_command(chassis_task_get_wheel_motor(index), 0.0f);
    }

    for (index = 0u; index < CHASSIS_TASK_LEG_COUNT; index++)
    {
        chassis_task_apply_current_command(chassis_task_get_leg_motor(index), 0.0f);
    }
}

OmBool chassis_task_should_submit_tx_request(
    ChassisTaskContext* context,
    ModeTaskOperationalPhaseState operational_phase,
    OsalTimeMs now_ms)
{
    if (context == OM_NULL)
    {
        return OM_FALSE;
    }

    if (mct_is_operational_active() != OM_TRUE ||
        operational_phase == MODE_TASK_OPERATIONAL_PHASE_RELEASE)
    {
        context->last_tx_request_ms = 0u;
        return OM_FALSE;
    }

    if (context->last_tx_request_ms != 0u &&
        (uint32_t)(now_ms - context->last_tx_request_ms) < CHASSIS_TASK_TX_REQUEST_PERIOD_MS)
    {
        return OM_FALSE;
    }

    context->last_tx_request_ms = now_ms;
    return OM_TRUE;
}

void chassis_task_apply_big_yaw_hold(ChassisTaskContext* context)
{
#if (BIG_YAW_TEMP_HOLD_ENABLE == 1u)
    const MotorFeedback* feedback = OM_NULL;

    if (context == OM_NULL || chassis_task_get_big_yaw_motor() == OM_NULL)
    {
        return;
    }

    feedback = motor_get_feedback(chassis_task_get_big_yaw_motor());
    if (!(context->flags & CHASSIS_TASK_FLAG_BIG_YAW_HOLD_INIT))
    {
        if (feedback == OM_NULL || feedback->online != OM_TRUE)
        {
            return;
        }

        context->big_yaw_hold_angle_rad = feedback->angle;
        context->flags |= CHASSIS_TASK_FLAG_BIG_YAW_HOLD_INIT;
    }

    motor_set_angle(chassis_task_get_big_yaw_motor(), context->big_yaw_hold_angle_rad);
    motor_set_speed(chassis_task_get_big_yaw_motor(), 0.0f);
    motor_set_position_gains(
        chassis_task_get_big_yaw_motor(),
        APP_CHASSIS_BIG_YAW_HOLD_KP,
        APP_CHASSIS_BIG_YAW_HOLD_KD);
    motor_set_torque(chassis_task_get_big_yaw_motor(), 0.0f);
    (void)motor_control_compute(chassis_task_get_big_yaw_motor());
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
void chassis_task_resolve_wheel_online_flags(
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
        if (context == OM_NULL || chassis_task_get_wheel_motor(index) == OM_NULL)
        {
            offline_wheel_count++;
            last_offline_wheel_id = (MecanumWheelId)index;
            continue;
        }

        feedback = motor_get_feedback(chassis_task_get_wheel_motor(index));
        /* 检查电机反馈数据是否在有效时间范围内 */
        if (chassis_task_motor_feedback_recent(chassis_task_get_wheel_motor(index), g_chassis_task_wheel_feedback_timeout_ms) == OM_TRUE)
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
void chassis_task_apply_wheel_control(
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
        const MotorFeedback* feedback = motor_get_feedback(chassis_task_get_wheel_motor(index));
        PidController* wheel_speed_pid = chassis_task_get_wheel_speed_pid(context, index);
        float wheel_speed_fdb_rpm = 0.0f;
        float current_cmd = 0.0f;

        /* 检查轮子是否激活且反馈数据有效，否则重置PID并停止电机 */
        if (wheel_active_flags[index] != OM_TRUE ||
            wheel_speed_pid == OM_NULL ||
            chassis_task_motor_feedback_recent(chassis_task_get_wheel_motor(index), g_chassis_task_wheel_feedback_timeout_ms) != OM_TRUE)
        {
            pid_reset(wheel_speed_pid);
            chassis_task_apply_current_command(chassis_task_get_wheel_motor(index), 0.0f);
            continue;
        }

        /* 将角速度反馈转换为RPM单位 */
        wheel_speed_fdb_rpm = math_utils_rad_per_s_to_rpm(feedback->speed);
        
        /* 使用PID控制器计算电流控制指令 */
        current_cmd =
            pid_compute(
                wheel_speed_pid,
                (float)wheel_speed_ref_rpm[index],
                wheel_speed_fdb_rpm,
                current_tick_s);

        /* 应用电流控制指令到电机 */
        chassis_task_apply_current_command(chassis_task_get_wheel_motor(index), current_cmd);
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
void chassis_task_apply_leg_control(
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
        const MotorFeedback* feedback = motor_get_feedback(chassis_task_get_leg_motor(index));
        const float leg_angle_fdb_deg =
            (feedback != OM_NULL) ? math_utils_normalize_deg(math_utils_rad_to_deg(feedback->angle)) : 0.0f;
        float leg_current_raw = 0.0f;

        /* 单环：角度 PID 直接出电流（与旧工程对齐） */
        leg_current_raw =
            pid_compute(
                &g_leg_angle_pids[index],
                leg_reference_deg[index],
                leg_angle_fdb_deg,
                current_tick_s);

        chassis_task_apply_current_command(
            chassis_task_get_leg_motor(index),
            leg_current_raw / 100.0f);
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
void chassis_task_run_once(ChassisTaskContext* context)
{
    ChassisTaskInputSnapshot snapshot = {0};
    float vx_mm_per_s = 0.0f;
    float vy_mm_per_s = 0.0f;
    float vw_deg_per_s = 0.0f;
    float leg_reference_deg[CHASSIS_TASK_LEG_COUNT] = {0.0f};
    int16_t wheel_speed_ref_rpm[CHASSIS_TASK_WHEEL_COUNT] = {0};
    OmBool wheel_online_flags[CHASSIS_TASK_WHEEL_COUNT] = {OM_FALSE};
    uint32_t online_wheel_count = 0u;
    OmBool degraded_mode_enabled = OM_FALSE;
    MecanumWheelId offline_wheel_id = MECANUM_WHEEL_FRONT_RIGHT;
    const float current_tick_s = chassis_task_now_s();
    const OsalTimeMs now_ms = osal_time_now_monotonic();
    uint32_t index = 0u;
    OmBool allow_chassis_motion = OM_FALSE;
    OmBool allow_leg_control = OM_FALSE;

    if (context == OM_NULL)
    {
        return;
    }

    chassis_task_drain_mode_snapshots(context);
    chassis_task_drain_rc_snapshots(context);
    chassis_task_drain_imu_snapshots(context);
    if (chassis_task_load_snapshot(context, &snapshot) != OM_TRUE)
    {
        return;
    }

    /* 确保电机已绑定，未绑定时尝试绑定 */
    if (!(context->flags & CHASSIS_TASK_FLAG_MOTORS_BOUND))
    {
        if (chassis_task_try_bind_motors(context) != OM_OK)
        {
            return;
        }
    }
    if (mct_is_operational_active() != OM_TRUE)
    {
        context->flags &= ~CHASSIS_TASK_FLAG_CONTROL_MODES_ARMED;
        context->last_tx_request_ms = 0u;
    }
    else if (!(context->flags & CHASSIS_TASK_FLAG_CONTROL_MODES_ARMED))
    {
        if (chassis_task_restore_control_modes(context) != OM_OK)
        {
            return;
        }
        context->flags |= CHASSIS_TASK_FLAG_CONTROL_MODES_ARMED;
    }

    chassis_task_resolve_wheel_online_flags(
        context,
        wheel_online_flags,
        &online_wheel_count,
        &degraded_mode_enabled,
        &offline_wheel_id);

    if (snapshot.operational_phase == MODE_TASK_OPERATIONAL_PHASE_MODE_SELECTION)
    {
        allow_chassis_motion = (snapshot.allow_rc_drive != 0u) ? OM_TRUE : OM_FALSE;
        allow_leg_control = (snapshot.leg_enable != 0u) ? OM_TRUE : OM_FALSE;
    }
    else if (snapshot.operational_phase == MODE_TASK_OPERATIONAL_PHASE_FORMAL_CONTROL)
    {
        allow_chassis_motion = (snapshot.allow_rc_drive != 0u) ? OM_TRUE : OM_FALSE;
        allow_leg_control = (snapshot.leg_enable != 0u) ? OM_TRUE : OM_FALSE;
    }

    /* 根据正式相位与当前模式执行底盘/后腿控制。 */
    if (snapshot.operational_phase == MODE_TASK_OPERATIONAL_PHASE_RELEASE)
    {
        /* 释放模式：重置腿部命令并清零所有输出 */
        chassis_task_reset_leg_command(context);
        chassis_task_apply_zero_output(context);
    }
    else if (allow_chassis_motion == OM_TRUE)
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

        chassis_task_apply_wheel_control(context, wheel_speed_ref_rpm, wheel_online_flags, current_tick_s);
        if (allow_leg_control == OM_TRUE)
        {
            chassis_task_update_leg_reference_deg(context, snapshot.ch4, leg_reference_deg);
            chassis_task_apply_leg_control(context, leg_reference_deg, current_tick_s);
        }
        else
        {
            for (index = 0u; index < CHASSIS_TASK_LEG_COUNT; index++)
            {
                chassis_task_apply_current_command(chassis_task_get_leg_motor(index), 0.0f);
            }
        }
        chassis_task_apply_big_yaw_hold(context);
    }
    else if (allow_leg_control == OM_TRUE)
    {
        chassis_task_apply_zero_output(context);
        chassis_task_update_leg_reference_deg(context, snapshot.ch4, leg_reference_deg);
        chassis_task_apply_leg_control(context, leg_reference_deg, current_tick_s);
        chassis_task_apply_big_yaw_hold(context);
    }
    else
    {
        /* 其他模式：清零所有输出 */
        chassis_task_apply_zero_output(context);
    }

    for (index = 0u; index < CHASSIS_TASK_WHEEL_COUNT; index++)
    {
        context->last_wheel_speed_ref_rpm[index] = (float)wheel_speed_ref_rpm[index];
    }

    if (chassis_task_should_submit_tx_request(context, snapshot.operational_phase, now_ms) != OM_TRUE)
    {
        return;
    }

    (void)motor_tx_dispatch_submit(MOTOR_TX_SOURCE_CHASSIS);
}

