#include "task/arm_task/arm_task.h"

#include "algorithm/gravity_comp/gravity_comp.h"
#include "config/app_config.h"
#include "core/algorithm/controller/pid.h"
#include "driver/motor/motor.h"
#include "module/data_pool/data_pool.h"
#include "module/event_bus/event_bus.h"
#include "module/motor_tx_dispatch/motor_tx_dispatch.h"
#include "module/system_health/system_health.h"
#include "osal/osal.h"
#include "osal/osal_config.h"
#include "osal/osal_time.h"
#include "task/mode_task/mode_task.h"
#include <string.h>

#define ARM_TASK_PERIOD_MS           (6u)
#define ARM_TASK_STACK_BYTES         (1024u * OSAL_STACK_WORD_BYTES)
#define ARM_TASK_PRIORITY            (4u)
#define ARM_TASK_POSE_MACHINE_COUNT  (7u)
#define ARM_TASK_CUSTOM_CONTROLLER_WORK_MODE_ENCODER (0u)
#define ARM_TASK_CUSTOM_CONTROLLER_AXIS_COUNT        (6u)
#define ARM_TASK_CUSTOM_ALIGNMENT_RAD_THRESHOLD      (0.08f)

/* 机构角姿态表的索引顺序。
 * 保持与旧工程的 Machine_angle 轴顺序一致，便于直接迁移姿态表。
 */
typedef enum
{
    ARM_TASK_MACHINE_BIG_YAW = 0u,
    ARM_TASK_MACHINE_PITCH1,
    ARM_TASK_MACHINE_PITCH2,
    ARM_TASK_MACHINE_ROLL2,
    ARM_TASK_MACHINE_PITCH3,
    ARM_TASK_MACHINE_ROLL3,
    ARM_TASK_MACHINE_GRIP,
    ARM_TASK_MACHINE_COUNT
} ArmTaskMachineAxis;

typedef struct
{
    ChassisMode chassis_mode;
    ClampAction clamp_action;
    ExchangeAction exchange_action;
    uint8_t primary_turn_ore_flag;
    uint8_t custom_controller_force_takeover_flag;
} ArmTaskSnapshot;

typedef struct
{
    uint8_t online;
    uint8_t work_mode;
    float angle_deg[ARM_TASK_CUSTOM_CONTROLLER_AXIS_COUNT];
} ArmTaskCustomControllerSnapshot;

/* 机械臂姿态使用当前正式链的“机构角”定义：
 * - big_yaw / pitch1 / pitch2 / roll2 / pitch3 / roll3 / grip：单位 rad
 * - roll3 仍保持 GM6020 单圈物理角语义，但在 arm_task 内部统一存成 rad
 *
 * 后续统一在一个地方映射到当前 motor 抽象层的绝对目标值。
 */
typedef struct
{
    float machine_values[ARM_TASK_MACHINE_COUNT];
} ArmTaskMachinePose;

typedef struct
{
    float big_yaw_rad;
    float pitch1_rad;
    float pitch2_rad;
    float roll2_rad;
    float pitch3_rad;
    float roll3_rad;
    float grip_rad;
} ArmTaskMotorTargets;

/* arm_task 本地上下文：
 * - 直接持有机械臂各轴电机句柄
 * - 维护 roll3 的双环 PID
 * - 保存最近一次共享控制事实，用于动作时间窗推进
 * - pitch2 零位由 GO8010 owner 锁存，这里只读消费
 */
typedef struct
{
    Motor* big_yaw_motor;
    Motor* pitch1_motor;
    Motor* pitch2_motor;
    Motor* roll2_motor;
    Motor* pitch3_motor;
    Motor* roll3_motor;
    Motor* grip_motor;
    PidController roll3_angle_pid;
    PidController roll3_speed_pid;
    ArmTaskSnapshot last_snapshot;
    ArmTaskMotorTargets smoothed_targets;
    OsalTimeMs command_since_ms;
    OmBool motors_bound_flag;
    OmBool snapshot_initialized;
    OmBool smoothed_targets_initialized;
    OmBool custom_controller_alignment_done;
    OmBool custom_controller_alignment_failed;
    OmBool custom_controller_reference_captured;
    OmBool custom_controller_was_active;
    OmBool custom_controller_filter_initialized;
    OsalTimeMs custom_controller_alignment_started_ms;
    float custom_controller_neutral_deg[ARM_TASK_CUSTOM_CONTROLLER_AXIS_COUNT];
    float custom_controller_filtered_delta_deg[ARM_TASK_CUSTOM_CONTROLLER_AXIS_COUNT];
    float custom_controller_roll3_reference_rad;
    float custom_controller_grip_reference_rad;
} ArmTaskContext;

typedef enum
{
    ARM_TASK_CUSTOM_AXIS_Y = 0u,
    ARM_TASK_CUSTOM_AXIS_Z,
    ARM_TASK_CUSTOM_AXIS_X,
    ARM_TASK_CUSTOM_AXIS_YAW,
    ARM_TASK_CUSTOM_AXIS_PITCH,
    ARM_TASK_CUSTOM_AXIS_ROLL,
} ArmTaskCustomControllerAxis;

static const char* g_arm_task_big_yaw_name = "big_yaw";
static const char* g_arm_task_pitch1_name = "pitch1";
static const char* g_arm_task_pitch2_name = "pitch2";
static const char* g_arm_task_roll2_name = "roll2";
static const char* g_arm_task_pitch3_name = "pitch3";
static const char* g_arm_task_roll3_name = "roll3";
static const char* g_arm_task_grip_name = "grip";
static volatile uint8_t g_arm_task_custom_controller_alignment_done_debug = 0u;
/* 自定义控制器原始角度直接进机械臂时，pitch2 会被 6.33 齿比放大。
 * 这里在 arm_task 内先做每轴前处理：
 * - 死区：压掉控制器静止时的小抖动
 * - 一阶低通：避免原始角度噪声直接打到关节目标
 *
 * 当前只在自定义控制器接管路径生效，不影响遥控器或动作表。
 */
static const float g_arm_task_custom_controller_deadband_deg[ARM_TASK_CUSTOM_CONTROLLER_AXIS_COUNT] = {
    0.8f, 0.8f, 1.5f, 0.8f, 0.8f, 0.8f};
static const float g_arm_task_custom_controller_filter_alpha[ARM_TASK_CUSTOM_CONTROLLER_AXIS_COUNT] = {
    0.18f, 0.18f, 0.10f, 0.18f, 0.18f, 0.18f};

static void arm_task_assign_pose(ArmTaskMachinePose* target, const ArmTaskMachinePose* source);
static void arm_task_apply_custom_controller_alignment_pose(
    ArmTaskContext* context,
    ArmTaskMachinePose* pose);
static void arm_task_resolve_motor_targets(
    const ArmTaskContext* context,
    const ArmTaskMachinePose* pose,
    ArmTaskMotorTargets* targets);

/* 旧工程姿态表迁移结果。
 * 语义拆成两层：
 * - g_arm_pose_normal：常驻基础姿态
 * - 其余 g_arm_pose_*：模式增量（mode angle）
 *
 * 最终机构角 = normal + mode_delta
 *
 * 这样才能与旧工程的
 *   angle_ref = normal_angle + mode_angle + offset_angle
 * 保持一致。
 *
 * 注意：
 * - roll3 当前不再沿用“normal + mode_delta”的相对平衡角写法
 * - 源数据仍来自 GM6020 单圈物理目标角（deg），但落到 arm_task 姿态表时统一转成 rad
 * - 旧工程 roll3 基准是 normal_angle = 271 deg，当前新电机的平衡位改为 213.75 deg
 * - 因此各动作 roll3 目标先按“213.75 + 旧 mode_angle”重算，再转成 rad 存储
 */
static const ArmTaskMachinePose g_arm_pose_zero = {
    {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 3.7306414f, 0.0f}};
static const ArmTaskMachinePose g_arm_pose_normal = {
    {0.0f, 0.0f, 0.0f, 0.0f, 0.1f, 0.0f, 1.8f}};
static const ArmTaskMachinePose g_arm_pose_get_energy = {
    {0.0f, 1.24218f, 1.19447f, 0.0f, 0.0f, 3.7306414f, -1.8f}};
static const ArmTaskMachinePose g_arm_pose_get_energy1 = {
    {-0.00667f, 1.035f, (5.53f / 6.33f) + 0.34f + 0.1f, 0.6178f, -0.194f, 2.1530383f, -1.8f}};
static const ArmTaskMachinePose g_arm_pose_get_energy2 = {
    {0.148584366f, 0.99088f, 1.04010f + 0.1f, 0.093270302f, 0.07834f, 2.7812520f, -1.8f}};
static const ArmTaskMachinePose g_arm_pose_store_energy = {
    {1.141f, 1.1042f, 1.18567f, 0.016975f, -1.55555f, 2.8489184f, 0.0f}};
static const ArmTaskMachinePose g_arm_pose_store_energy1 = {
    {-1.9018459f, 1.00080109f, 1.008974f, 0.09136295f, -1.222925131f, 4.0876327f, 0.0f}};
static const ArmTaskMachinePose g_arm_pose_exchange = {
    {0.0f, 0.64218f, 1.0447f, 0.0f, 0.0f, 3.7306414f, 0.0f}};
static const ArmTaskMachinePose g_arm_pose_exchange_pick = {
    {1.041f, 1.2042f, 1.08567f, 0.016975f, -1.55555f, 2.8489184f, -1.8f}};
static const ArmTaskMachinePose g_arm_pose_exchange_pick1 = {
    {-1.90189481f, 1.1f, 1.00948341f, 0.092153325f, -1.363282f, 4.0026160f, -1.8f}};
static const ArmTaskMachinePose g_arm_pose_primary = {
    {0.0f, 1.46691f, 2.0053f, 0.1192f, -1.6f, 0.5890486f, 0.0f}};
static const ArmTaskMachinePose g_arm_pose_secondary_ore = {
    {0.0f, 1.48691f, 0.85f, -1.57f, 0.0f, 3.7306414f, 0.0f}};

static float arm_task_clamp_float(float value, float min_value, float max_value)
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

static float arm_task_rad_to_deg(float angle_rad)
{
    return angle_rad * (180.0f / APP_PI);
}

static float arm_task_deg_to_rad(float angle_deg)
{
    return angle_deg * (APP_PI / 180.0f);
}

static float arm_task_apply_symmetric_deadband_deg(float value_deg, float deadband_deg)
{
    if (value_deg > deadband_deg)
    {
        return value_deg - deadband_deg;
    }
    if (value_deg < -deadband_deg)
    {
        return value_deg + deadband_deg;
    }

    return 0.0f;
}

static void arm_task_preprocess_custom_controller_delta_deg(
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
            arm_task_apply_symmetric_deadband_deg(
                raw_delta_deg,
                g_arm_task_custom_controller_deadband_deg[axis_index]);

        if (context->custom_controller_filter_initialized != OM_TRUE)
        {
            context->custom_controller_filtered_delta_deg[axis_index] =
                deadbanded_delta_deg;
        }
        else
        {
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

    context->custom_controller_filter_initialized = OM_TRUE;
}

static float arm_task_normalize_deg(float angle_deg)
{
    while (angle_deg > 180.0f)
    {
        angle_deg -= 360.0f;
    }
    while (angle_deg <= -180.0f)
    {
        angle_deg += 360.0f;
    }
    return angle_deg;
}

static float arm_task_resolve_nearest_equivalent_deg(float target_deg, float reference_deg)
{
    float target_normalized_deg = arm_task_normalize_deg(target_deg);
    float reference_normalized_deg = arm_task_normalize_deg(reference_deg);
    float delta_deg = target_normalized_deg - reference_normalized_deg;

    while (delta_deg > 180.0f)
    {
        target_normalized_deg -= 360.0f;
        delta_deg -= 360.0f;
    }
    while (delta_deg < -180.0f)
    {
        target_normalized_deg += 360.0f;
        delta_deg += 360.0f;
    }

    return target_normalized_deg + (reference_deg - reference_normalized_deg);
}

static float arm_task_resolve_nearest_equivalent_rad(float target_rad, float reference_rad)
{
    return arm_task_deg_to_rad(
        arm_task_resolve_nearest_equivalent_deg(
            arm_task_rad_to_deg(target_rad),
            arm_task_rad_to_deg(reference_rad)));
}

static float arm_task_rad_per_s_to_rpm(float speed_rad_per_s)
{
    return speed_rad_per_s * (60.0f / (2.0f * APP_PI));
}

static float arm_task_slew_value(float current_value, float target_value, float max_rate_rad_per_s, float dt_s)
{
    float max_step = 0.0f;
    float delta = 0.0f;

    if (dt_s <= 0.0f || max_rate_rad_per_s <= 0.0f)
    {
        return target_value;
    }

    max_step = max_rate_rad_per_s * dt_s;
    delta = target_value - current_value;

    if (delta > max_step)
    {
        return current_value + max_step;
    }
    if (delta < -max_step)
    {
        return current_value - max_step;
    }
    return target_value;
}

/* 每轮只读一次共享池快照，后续动作逻辑全部基于这份本地副本展开。 */
static void arm_task_load_snapshot(ArmTaskSnapshot* snapshot)
{
    if (snapshot == OM_NULL)
    {
        return;
    }

    snapshot->chassis_mode = (ChassisMode)DP_LOAD_UINT8(&g_data_pool.mode.chassis_mode);
    snapshot->clamp_action = (ClampAction)DP_LOAD_UINT8(&g_data_pool.action.clamp_action);
    snapshot->exchange_action = (ExchangeAction)DP_LOAD_UINT8(&g_data_pool.action.exchange_action);
    snapshot->primary_turn_ore_flag = DP_LOAD_UINT8(&g_data_pool.action.primary_turn_ore_flag);
    snapshot->custom_controller_force_takeover_flag =
        DP_LOAD_UINT8(&g_data_pool.action.custom_controller_force_takeover_flag);
}

static void arm_task_load_custom_controller_snapshot(ArmTaskCustomControllerSnapshot* snapshot)
{
    uint32_t axis_index = 0u;

    if (snapshot == OM_NULL)
    {
        return;
    }

    snapshot->online = DP_LOAD_UINT8(&g_data_pool.custom_controller.online);
    snapshot->work_mode = DP_LOAD_UINT8(&g_data_pool.custom_controller.work_mode);
    for (axis_index = 0u; axis_index < ARM_TASK_CUSTOM_CONTROLLER_AXIS_COUNT; axis_index++)
    {
        snapshot->angle_deg[axis_index] = DP_LOAD_FLOAT(&g_data_pool.custom_controller.angle_deg[axis_index]);
    }
}

static OmBool arm_task_snapshot_changed(const ArmTaskSnapshot* lhs, const ArmTaskSnapshot* rhs)
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

static OmBool arm_task_custom_controller_takeover_active(
    const ArmTaskContext* context,
    const ArmTaskSnapshot* arm_snapshot,
    const ArmTaskCustomControllerSnapshot* controller_snapshot)
{
    if (context == OM_NULL || arm_snapshot == OM_NULL || controller_snapshot == OM_NULL)
    {
        return OM_FALSE;
    }

    return (arm_snapshot->chassis_mode == MODE_CHASSIS_CUSTOM_CONTROLLER_NORMAL &&
            context->custom_controller_alignment_done == OM_TRUE &&
            controller_snapshot->online != 0u &&
            controller_snapshot->work_mode == ARM_TASK_CUSTOM_CONTROLLER_WORK_MODE_ENCODER)
               ? OM_TRUE
               : OM_FALSE;
}

static OmBool arm_task_feedback_online(const MotorFeedback* feedback)
{
    return (feedback != OM_NULL && feedback->online == OM_TRUE) ? OM_TRUE : OM_FALSE;
}

static OmBool arm_task_motor_online(const Motor* motor)
{
    return arm_task_feedback_online(motor_get_feedback(motor));
}

static OmBool arm_task_all_motors_online(const ArmTaskContext* context)
{
    if (context == OM_NULL)
    {
        return OM_FALSE;
    }

    return (arm_task_motor_online(context->big_yaw_motor) == OM_TRUE &&
            arm_task_motor_online(context->pitch1_motor) == OM_TRUE &&
            arm_task_motor_online(context->pitch2_motor) == OM_TRUE &&
            arm_task_motor_online(context->roll2_motor) == OM_TRUE &&
            arm_task_motor_online(context->pitch3_motor) == OM_TRUE &&
            arm_task_motor_online(context->roll3_motor) == OM_TRUE &&
            arm_task_motor_online(context->grip_motor) == OM_TRUE)
               ? OM_TRUE
               : OM_FALSE;
}

static OmBool arm_task_get_roll3_feedback_angle_rad(
    const ArmTaskContext* context,
    float* angle_rad)
{
    if (context == OM_NULL || angle_rad == OM_NULL || context->roll3_motor == OM_NULL)
    {
        return OM_FALSE;
    }

    return motor_get_single_turn_angle_rad(context->roll3_motor, angle_rad);
}

/* roll3 双环 PID 的通用初始化 helper。 */
static OmRet arm_task_init_pid(
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
static OmRet arm_task_init_pids(ArmTaskContext* context)
{
    OmRet ret = OM_OK;

    if (context == OM_NULL)
    {
        return OM_ERROR_NULL;
    }

    ret = arm_task_init_pid(
        &context->roll3_angle_pid,
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
        &context->roll3_speed_pid,
        APP_ARM_ROLL3_SPEED_PID_KP,
        APP_ARM_ROLL3_SPEED_PID_KI,
        APP_ARM_ROLL3_SPEED_PID_KD,
        APP_ARM_ROLL3_SPEED_PID_OUT_LIMIT,
        APP_ARM_ROLL3_SPEED_PID_INTEGRAL_LIMIT);
}

/* 机械臂控制 owner 绑定：
 * - 只查已经注册进 motor registry 的对象
 * - 只设定 arm_task 所需的 control mode
 * - 不碰任何物理总线初始化
 */
static OmRet arm_task_try_bind_motors(ArmTaskContext* context)
{
    if (context == OM_NULL)
    {
        return OM_ERROR_NULL;
    }

    context->motors_bound_flag = OM_FALSE;

    context->big_yaw_motor = motor_find_by_name(g_arm_task_big_yaw_name);
    context->pitch1_motor = motor_find_by_name(g_arm_task_pitch1_name);
    context->pitch2_motor = motor_find_by_name(g_arm_task_pitch2_name);
    context->roll2_motor = motor_find_by_name(g_arm_task_roll2_name);
    context->pitch3_motor = motor_find_by_name(g_arm_task_pitch3_name);
    context->roll3_motor = motor_find_by_name(g_arm_task_roll3_name);
    context->grip_motor = motor_find_by_name(g_arm_task_grip_name);

    if (context->big_yaw_motor == OM_NULL || context->pitch1_motor == OM_NULL ||
        context->pitch2_motor == OM_NULL || context->roll2_motor == OM_NULL ||
        context->pitch3_motor == OM_NULL || context->roll3_motor == OM_NULL ||
        context->grip_motor == OM_NULL)
    {
        return OM_ERROR_NULL;
    }

    if (motor_set_control_mode(context->big_yaw_motor, MOTOR_CONTROL_MODE_ANGLE) != OM_OK ||
        motor_set_control_mode(context->pitch1_motor, MOTOR_CONTROL_MODE_ANGLE) != OM_OK ||
        motor_set_control_mode(context->pitch2_motor, MOTOR_CONTROL_MODE_ANGLE) != OM_OK ||
        motor_set_control_mode(context->roll2_motor, MOTOR_CONTROL_MODE_ANGLE) != OM_OK ||
        motor_set_control_mode(context->pitch3_motor, MOTOR_CONTROL_MODE_ANGLE) != OM_OK ||
        motor_set_control_mode(context->grip_motor, MOTOR_CONTROL_MODE_ANGLE) != OM_OK ||
        motor_set_control_mode(context->roll3_motor, MOTOR_CONTROL_MODE_CURRENT) != OM_OK)
    {
        return OM_ERROR;
    }

    context->motors_bound_flag = OM_TRUE;
    return OM_OK;
}

/* pitch2 的绝对位置零位由 GO8010 owner 在正式通信 bring-up 中锁存。
 * arm_task 只读这个基准，不再自己维护初始化事实。
 */
static OmBool arm_task_get_pitch2_zero_angle_rad(
    const ArmTaskContext* context,
    float* pitch2_zero_angle_rad)
{
    if (context == OM_NULL || pitch2_zero_angle_rad == OM_NULL ||
        context->pitch2_motor == OM_NULL)
    {
        return OM_FALSE;
    }

    return motor_get_initial_zero_angle_rad(context->pitch2_motor, pitch2_zero_angle_rad);
}

static OmBool arm_task_get_pitch2_joint_feedback_rad(
    const ArmTaskContext* context,
    float* pitch2_joint_angle_rad)
{
    const MotorFeedback* pitch2_feedback = OM_NULL;
    float pitch2_zero_angle_rad = 0.0f;

    if (context == OM_NULL || pitch2_joint_angle_rad == OM_NULL)
    {
        return OM_FALSE;
    }

    pitch2_feedback = motor_get_feedback(context->pitch2_motor);
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

static void arm_task_refresh_smoothed_targets_from_feedback(ArmTaskContext* context)
{
    const MotorFeedback* feedback = OM_NULL;

    if (context == OM_NULL || context->smoothed_targets_initialized == OM_TRUE)
    {
        return;
    }

    feedback = motor_get_feedback(context->big_yaw_motor);
    context->smoothed_targets.big_yaw_rad = (feedback != OM_NULL) ? feedback->angle : 0.0f;

    feedback = motor_get_feedback(context->pitch1_motor);
    context->smoothed_targets.pitch1_rad = (feedback != OM_NULL) ? feedback->angle : 0.0f;

    feedback = motor_get_feedback(context->pitch2_motor);
    if (feedback != OM_NULL)
    {
        context->smoothed_targets.pitch2_rad = feedback->angle;
    }
    else if (arm_task_get_pitch2_zero_angle_rad(context, &context->smoothed_targets.pitch2_rad) != OM_TRUE)
    {
        context->smoothed_targets.pitch2_rad = 0.0f;
    }

    feedback = motor_get_feedback(context->roll2_motor);
    context->smoothed_targets.roll2_rad = (feedback != OM_NULL) ? feedback->angle : 0.0f;

    feedback = motor_get_feedback(context->pitch3_motor);
    context->smoothed_targets.pitch3_rad = (feedback != OM_NULL) ? feedback->angle : 0.0f;

    if (arm_task_get_roll3_feedback_angle_rad(context, &context->smoothed_targets.roll3_rad) != OM_TRUE)
    {
        context->smoothed_targets.roll3_rad = 0.0f;
    }

    feedback = motor_get_feedback(context->grip_motor);
    context->smoothed_targets.grip_rad = (feedback != OM_NULL) ? feedback->angle : 0.0f;

    context->smoothed_targets_initialized = OM_TRUE;
}

static void arm_task_reset_custom_controller_state(ArmTaskContext* context)
{
    uint32_t axis_index = 0u;

    if (context == OM_NULL)
    {
        return;
    }

    context->custom_controller_alignment_done = OM_FALSE;
    context->custom_controller_alignment_failed = OM_FALSE;
    context->custom_controller_reference_captured = OM_FALSE;
    context->custom_controller_was_active = OM_FALSE;
    context->custom_controller_filter_initialized = OM_FALSE;
    context->custom_controller_alignment_started_ms = 0u;
    for (axis_index = 0u; axis_index < ARM_TASK_CUSTOM_CONTROLLER_AXIS_COUNT; axis_index++)
    {
        context->custom_controller_filtered_delta_deg[axis_index] = 0.0f;
    }
    g_arm_task_custom_controller_alignment_done_debug = 0u;
}

static void arm_task_apply_custom_controller_alignment_pose(
    ArmTaskContext* context,
    ArmTaskMachinePose* pose)
{
    float grip_target_rad = 0.0f;
    const MotorFeedback* grip_feedback = OM_NULL;

    if (context == OM_NULL || pose == OM_NULL)
    {
        return;
    }

    arm_task_assign_pose(pose, &g_arm_pose_zero);

    if (context->smoothed_targets_initialized == OM_TRUE)
    {
        grip_target_rad = context->smoothed_targets.grip_rad;
    }
    else
    {
        grip_feedback = motor_get_feedback(context->grip_motor);
        grip_target_rad =
            (grip_feedback != OM_NULL) ? grip_feedback->angle : g_arm_pose_normal.machine_values[ARM_TASK_MACHINE_GRIP];
    }

    pose->machine_values[ARM_TASK_MACHINE_GRIP] =
        grip_target_rad - g_arm_pose_normal.machine_values[ARM_TASK_MACHINE_GRIP];
}

static OmBool arm_task_custom_controller_alignment_reached(ArmTaskContext* context)
{
    ArmTaskMachinePose alignment_pose = {0};
    ArmTaskMotorTargets targets = {0};
    const MotorFeedback* feedback = OM_NULL;
    float error_rad = 0.0f;
    float pitch2_feedback_joint_rad = 0.0f;
    float pitch2_target_joint_rad = 0.0f;
    float roll3_feedback_rad = 0.0f;
    float roll3_target_rad = 0.0f;
    float roll3_error_rad = 0.0f;

    if (context == OM_NULL)
    {
        return OM_FALSE;
    }

    if (arm_task_all_motors_online(context) != OM_TRUE)
    {
        return OM_FALSE;
    }

    arm_task_apply_custom_controller_alignment_pose(context, &alignment_pose);
    arm_task_resolve_motor_targets(context, &alignment_pose, &targets);
    pitch2_target_joint_rad =
        g_arm_pose_normal.machine_values[ARM_TASK_MACHINE_PITCH2] +
        alignment_pose.machine_values[ARM_TASK_MACHINE_PITCH2];

    feedback = motor_get_feedback(context->big_yaw_motor);
    if (feedback == OM_NULL)
    {
        return OM_FALSE;
    }
    error_rad = feedback->angle - targets.big_yaw_rad;
    if (error_rad > ARM_TASK_CUSTOM_ALIGNMENT_RAD_THRESHOLD ||
        error_rad < -ARM_TASK_CUSTOM_ALIGNMENT_RAD_THRESHOLD)
    {
        return OM_FALSE;
    }

    feedback = motor_get_feedback(context->pitch1_motor);
    if (feedback == OM_NULL)
    {
        return OM_FALSE;
    }
    error_rad = feedback->angle - targets.pitch1_rad;
    if (error_rad > ARM_TASK_CUSTOM_ALIGNMENT_RAD_THRESHOLD ||
        error_rad < -ARM_TASK_CUSTOM_ALIGNMENT_RAD_THRESHOLD)
    {
        return OM_FALSE;
    }

    if (arm_task_get_pitch2_joint_feedback_rad(context, &pitch2_feedback_joint_rad) != OM_TRUE)
    {
        return OM_FALSE;
    }
    error_rad = pitch2_feedback_joint_rad - pitch2_target_joint_rad;
    /* pitch2 走 GO8010 + 6.33 齿比，回位时关节侧会有更明显的零位抖动和齿隙残差。
     * 自动对齐这里放宽一点，只影响“是否允许进入接管”，不改变正式控制精度。
     */
    if (error_rad > 0.10f ||
        error_rad < -0.10f)
    {
        return OM_FALSE;
    }

    feedback = motor_get_feedback(context->roll2_motor);
    if (feedback == OM_NULL)
    {
        return OM_FALSE;
    }
    error_rad = feedback->angle - targets.roll2_rad;
    if (error_rad > ARM_TASK_CUSTOM_ALIGNMENT_RAD_THRESHOLD ||
        error_rad < -ARM_TASK_CUSTOM_ALIGNMENT_RAD_THRESHOLD)
    {
        return OM_FALSE;
    }

    feedback = motor_get_feedback(context->pitch3_motor);
    if (feedback == OM_NULL)
    {
        return OM_FALSE;
    }
    error_rad = feedback->angle - targets.pitch3_rad;
    if (error_rad > ARM_TASK_CUSTOM_ALIGNMENT_RAD_THRESHOLD ||
        error_rad < -ARM_TASK_CUSTOM_ALIGNMENT_RAD_THRESHOLD)
    {
        return OM_FALSE;
    }

    if (arm_task_get_roll3_feedback_angle_rad(context, &roll3_feedback_rad) != OM_TRUE)
    {
        return OM_FALSE;
    }

    roll3_target_rad =
        arm_task_resolve_nearest_equivalent_rad(targets.roll3_rad, roll3_feedback_rad);
    roll3_error_rad = roll3_target_rad - roll3_feedback_rad;

    return (roll3_error_rad <= 0.10471976f &&
            roll3_error_rad >= -0.10471976f)
               ? OM_TRUE
               : OM_FALSE;
}

static void arm_task_capture_custom_controller_reference(
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
    context->custom_controller_filter_initialized = OM_FALSE;

    if (context->smoothed_targets_initialized == OM_TRUE)
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

        grip_feedback = motor_get_feedback(context->grip_motor);
        context->custom_controller_grip_reference_rad =
            (grip_feedback != OM_NULL) ? grip_feedback->angle : g_arm_pose_normal.machine_values[ARM_TASK_MACHINE_GRIP];
    }

    context->custom_controller_reference_captured = OM_TRUE;
}

static void arm_task_update_custom_controller_reference_state(
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
        if (context->custom_controller_alignment_done == OM_TRUE)
        {
            arm_task_reset_custom_controller_state(context);
        }
        return;
    }

    if (context->custom_controller_was_active != OM_TRUE ||
        context->custom_controller_reference_captured != OM_TRUE)
    {
        arm_task_capture_custom_controller_reference(context, controller_snapshot);
    }

    context->custom_controller_was_active = OM_TRUE;
}

static void arm_task_assign_pose(ArmTaskMachinePose* target, const ArmTaskMachinePose* source)
{
    if (target == OM_NULL || source == OM_NULL)
    {
        return;
    }

    memcpy(target, source, sizeof(*target));
}

static void arm_task_apply_custom_controller_pose(
    ArmTaskContext* context,
    const ArmTaskCustomControllerSnapshot* controller_snapshot,
    ArmTaskMachinePose* pose)
{
    float filtered_delta_deg[ARM_TASK_CUSTOM_CONTROLLER_AXIS_COUNT] = {0.0f};

    if (context == OM_NULL || controller_snapshot == OM_NULL || pose == OM_NULL)
    {
        return;
    }

    arm_task_assign_pose(pose, &g_arm_pose_zero);
    arm_task_preprocess_custom_controller_delta_deg(
        context,
        controller_snapshot,
        filtered_delta_deg);

    /* 当前控制器原始 6 轴在 VOFA 上仍按 Y/Z/X/Yaw/Pitch/Roll 展示。
     * 这里按现场已核对的机械臂语义接管：
     * - I2(Y)   -> big_yaw
     * - I3(Z)   -> pitch1，方向取反
     * - I4(X)   -> pitch2，方向取反
     * - I5(Yaw) -> roll2
     * - I6(Pitch) -> pitch3
     * - I7(Roll)  -> roll3
     */
    pose->machine_values[ARM_TASK_MACHINE_BIG_YAW] =
        arm_task_deg_to_rad(filtered_delta_deg[ARM_TASK_CUSTOM_AXIS_Y]);
    pose->machine_values[ARM_TASK_MACHINE_PITCH1] =
        arm_task_deg_to_rad(-filtered_delta_deg[ARM_TASK_CUSTOM_AXIS_Z]);
    pose->machine_values[ARM_TASK_MACHINE_PITCH2] =
        arm_task_deg_to_rad(-filtered_delta_deg[ARM_TASK_CUSTOM_AXIS_X]);
    pose->machine_values[ARM_TASK_MACHINE_ROLL2] =
        arm_task_deg_to_rad(filtered_delta_deg[ARM_TASK_CUSTOM_AXIS_YAW]);
    pose->machine_values[ARM_TASK_MACHINE_PITCH3] =
        arm_task_deg_to_rad(filtered_delta_deg[ARM_TASK_CUSTOM_AXIS_PITCH]);
    pose->machine_values[ARM_TASK_MACHINE_ROLL3] =
        context->custom_controller_roll3_reference_rad +
        arm_task_deg_to_rad(filtered_delta_deg[ARM_TASK_CUSTOM_AXIS_ROLL]);
    pose->machine_values[ARM_TASK_MACHINE_GRIP] =
        context->custom_controller_grip_reference_rad -
        g_arm_pose_normal.machine_values[ARM_TASK_MACHINE_GRIP];
}

/* EXCHANGE/PICK_ACTION1 的分段姿态推进。 */
static void arm_task_apply_exchange_pick_action_one(ArmTaskMachinePose* pose, OsalTimeMs elapsed_ms)
{
    if (pose == OM_NULL)
    {
        return;
    }

    arm_task_assign_pose(pose, &g_arm_pose_exchange);

    if (elapsed_ms >= 100u)
    {
        arm_task_assign_pose(pose, &g_arm_pose_store_energy);
    }
    if (elapsed_ms >= 1200u)
    {
        arm_task_assign_pose(pose, &g_arm_pose_exchange_pick);
    }
    if (elapsed_ms >= 1400u)
    {
        pose->machine_values[ARM_TASK_MACHINE_PITCH2] = 1.04067f;
        pose->machine_values[ARM_TASK_MACHINE_ROLL2] = 0.05f;
    }
    if (elapsed_ms >= 1550u)
    {
        pose->machine_values[ARM_TASK_MACHINE_PITCH2] = 0.82f;
        pose->machine_values[ARM_TASK_MACHINE_PITCH3] = -1.0f;
        pose->machine_values[ARM_TASK_MACHINE_ROLL2] = -0.04512f;
        pose->machine_values[ARM_TASK_MACHINE_ROLL3] = 2.8694487f;
    }
    if (elapsed_ms >= 1800u)
    {
        pose->machine_values[ARM_TASK_MACHINE_GRIP] = 0.0f;
    }
    if (elapsed_ms >= 2000u)
    {
        pose->machine_values[ARM_TASK_MACHINE_PITCH2] = 1.3f;
    }
}

/* EXCHANGE/PICK_ACTION2 的分段姿态推进。 */
static void arm_task_apply_exchange_pick_action_two(ArmTaskMachinePose* pose, OsalTimeMs elapsed_ms)
{
    if (pose == OM_NULL)
    {
        return;
    }

    arm_task_assign_pose(pose, &g_arm_pose_exchange);

    if (elapsed_ms >= 100u)
    {
        arm_task_assign_pose(pose, &g_arm_pose_store_energy1);
    }
    if (elapsed_ms >= 1200u)
    {
        arm_task_assign_pose(pose, &g_arm_pose_exchange_pick1);
    }
    if (elapsed_ms >= 1300u)
    {
        pose->machine_values[ARM_TASK_MACHINE_PITCH2] = 0.87f;
        pose->machine_values[ARM_TASK_MACHINE_PITCH3] = -1.19f;
    }
    if (elapsed_ms >= 1460u)
    {
        pose->machine_values[ARM_TASK_MACHINE_BIG_YAW] = -2.00189481f;
        pose->machine_values[ARM_TASK_MACHINE_PITCH2] = 0.810012383f;
        pose->machine_values[ARM_TASK_MACHINE_PITCH3] = -1.11f;
        pose->machine_values[ARM_TASK_MACHINE_PITCH1] = 1.20f;
    }
    if (elapsed_ms >= 2170u)
    {
        pose->machine_values[ARM_TASK_MACHINE_GRIP] = 0.0f;
    }
    if (elapsed_ms >= 2390u)
    {
        pose->machine_values[ARM_TASK_MACHINE_PITCH2] = 1.6f;
        pose->machine_values[ARM_TASK_MACHINE_PITCH3] = -1.45f;
    }
}

/* GET_ENERGY_UNIT 三段动作：
 * ACTION_ONE 开夹爪，ACTION_TWO 调整 pitch2/pitch3，ACTION_THREE 切到存放姿态。
 */
static void arm_task_apply_get_energy_unit(ArmTaskMachinePose* pose, ClampAction action, OsalTimeMs elapsed_ms)
{
    arm_task_assign_pose(pose, &g_arm_pose_get_energy);

    switch (action)
    {
    case MODE_CLAMP_ACTION_ONE:
        pose->machine_values[ARM_TASK_MACHINE_GRIP] = 0.0f;
        break;
    case MODE_CLAMP_ACTION_TWO:
        pose->machine_values[ARM_TASK_MACHINE_PITCH3] = 0.43f;
        pose->machine_values[ARM_TASK_MACHINE_PITCH2] = -0.43f + 1.19447f;
        break;
    case MODE_CLAMP_ACTION_THREE:
        arm_task_assign_pose(pose, &g_arm_pose_store_energy);
        if (elapsed_ms >= 800u)
        {
            pose->machine_values[ARM_TASK_MACHINE_GRIP] = -1.8f;
        }
        break;
    default:
        break;
    }
}

/* GET_ENERGY_UNIT1 当前只保留旧工程里已经明确的 ACTION_ONE 语义。 */
static void arm_task_apply_get_energy_unit1(ArmTaskMachinePose* pose, ClampAction action)
{
    arm_task_assign_pose(pose, &g_arm_pose_get_energy1);

    if (action == MODE_CLAMP_ACTION_ONE)
    {
        pose->machine_values[ARM_TASK_MACHINE_GRIP] = 0.0f;
    }
}

/* GET_ENERGY_UNIT2 的第二段动作直接切到另一套存放姿态。 */
static void arm_task_apply_get_energy_unit2(ArmTaskMachinePose* pose, ClampAction action, OsalTimeMs elapsed_ms)
{
    arm_task_assign_pose(pose, &g_arm_pose_get_energy2);

    switch (action)
    {
    case MODE_CLAMP_ACTION_ONE:
        pose->machine_values[ARM_TASK_MACHINE_GRIP] = 0.0f;
        break;
    case MODE_CLAMP_ACTION_TWO:
        arm_task_assign_pose(pose, &g_arm_pose_store_energy1);
        if (elapsed_ms >= 1150u)
        {
            pose->machine_values[ARM_TASK_MACHINE_GRIP] = -1.8f;
        }
        break;
    default:
        break;
    }
}

/* PRIMARY 模式里，primary_turn_ore_flag 只影响 roll3 的翻转目标。 */
static void arm_task_apply_primary(ArmTaskMachinePose* pose, ClampAction action, uint8_t primary_turn_ore_flag)
{
    if (pose == OM_NULL)
    {
        return;
    }

    switch (action)
    {
    case MODE_CLAMP_UN_CMD:
        arm_task_assign_pose(pose, &g_arm_pose_zero);
        pose->machine_values[ARM_TASK_MACHINE_PITCH1] = 0.6f;
        pose->machine_values[ARM_TASK_MACHINE_PITCH2] = 1.0f;
        break;
    case MODE_CLAMP_ACTION_ONE:
        arm_task_assign_pose(pose, &g_arm_pose_primary);
        break;
    case MODE_CLAMP_ACTION_TWO:
        arm_task_assign_pose(pose, &g_arm_pose_primary);
        pose->machine_values[ARM_TASK_MACHINE_ROLL3] =
            (primary_turn_ore_flag != 0u) ?
                0.5890486f :
                3.7306414f;
        break;
    case MODE_CLAMP_ACTION_THREE:
        arm_task_assign_pose(pose, &g_arm_pose_primary);
        pose->machine_values[ARM_TASK_MACHINE_GRIP] = -1.8f;
        pose->machine_values[ARM_TASK_MACHINE_PITCH2] = 2.5f;
        break;
    default:
        arm_task_assign_pose(pose, &g_arm_pose_primary);
        break;
    }
}

/* SECONDARY_ORE 保留旧工程里最小可靠动作链。 */
static void arm_task_apply_secondary_ore(ArmTaskMachinePose* pose, ClampAction action, OsalTimeMs elapsed_ms)
{
    if (pose == OM_NULL)
    {
        return;
    }

    switch (action)
    {
    case MODE_CLAMP_UN_CMD:
        arm_task_assign_pose(pose, &g_arm_pose_zero);
        pose->machine_values[ARM_TASK_MACHINE_PITCH1] = 1.38691f;
        pose->machine_values[ARM_TASK_MACHINE_PITCH2] = 1.0f;
        break;
    case MODE_CLAMP_ACTION_ONE:
        arm_task_assign_pose(pose, &g_arm_pose_zero);
        pose->machine_values[ARM_TASK_MACHINE_PITCH1] = 1.38691f;
        pose->machine_values[ARM_TASK_MACHINE_PITCH2] = 1.0f;
        if (elapsed_ms >= 50u)
        {
            arm_task_assign_pose(pose, &g_arm_pose_secondary_ore);
        }
        if (elapsed_ms >= 400u)
        {
            pose->machine_values[ARM_TASK_MACHINE_PITCH3] = -1.6f;
        }
        break;
    default:
        arm_task_assign_pose(pose, &g_arm_pose_secondary_ore);
        break;
    }
}

/* EXCHANGE 模式只关心兑换子动作，不消费 clamp_action。 */
static void arm_task_apply_exchange(ArmTaskMachinePose* pose, ExchangeAction action, OsalTimeMs elapsed_ms)
{
    if (pose == OM_NULL)
    {
        return;
    }

    switch (action)
    {
    case MODE_EXCHANGE_PICK_ACTION1:
        arm_task_apply_exchange_pick_action_one(pose, elapsed_ms);
        break;
    case MODE_EXCHANGE_PICK_ACTION2:
        arm_task_apply_exchange_pick_action_two(pose, elapsed_ms);
        break;
    case MODE_EXCHANGE_UN_CMD:
    default:
        arm_task_assign_pose(pose, &g_arm_pose_exchange);
        break;
    }
}

static void clamp_angle_handle(
    const ArmTaskSnapshot* snapshot,
    OsalTimeMs elapsed_ms,
    ArmTaskMachinePose* pose)
{
    if (snapshot == OM_NULL || pose == OM_NULL)
    {
        return;
    }

    /* 这是 C6 计划里要求承接的 clamp_angle_handle() 语义入口。
     * 输入是当前共享控制事实，输出是本轮机械臂机构角姿态表。
     */

    switch (snapshot->chassis_mode)
    {
    case MODE_CHASSIS_GET_ENERGY_UNIT:
        arm_task_apply_get_energy_unit(pose, snapshot->clamp_action, elapsed_ms);
        break;
    case MODE_CHASSIS_GET_ENERGY_UNIT1:
        arm_task_apply_get_energy_unit1(pose, snapshot->clamp_action);
        break;
    case MODE_CHASSIS_GET_ENERGY_UNIT2:
        arm_task_apply_get_energy_unit2(pose, snapshot->clamp_action, elapsed_ms);
        break;
    case MODE_CHASSIS_EXCHANGE:
        arm_task_apply_exchange(pose, snapshot->exchange_action, elapsed_ms);
        break;
    case MODE_CHASSIS_PRIMARY:
        arm_task_apply_primary(pose, snapshot->clamp_action, snapshot->primary_turn_ore_flag);
        break;
    case MODE_CHASSIS_SECONDARY_ORE:
        arm_task_apply_secondary_ore(pose, snapshot->clamp_action, elapsed_ms);
        break;
    case MODE_CHASSIS_CUSTOM_CONTROLLER_NORMAL:
    case MODE_CHASSIS_PITCH3_TORQUE_COLLECTION:
    case MODE_CHASSIS_URGENT_MEASURE:
    case MODE_CHASSIS_CHECK:
    case MODE_CHASSIS_NORMAL:
    case MODE_CHASSIS_RELEASE:
    default:
        arm_task_assign_pose(pose, &g_arm_pose_zero);
        break;
    }
}

/* 机构角 -> 电机目标角：
 * - pitch1 使用 app_config 中的目标比例映射
 * - pitch2 使用旧工程 -6.33 映射并叠加零位
 * - roll3 在 arm_task 内部已经统一成 GM6020 单圈物理角（rad），这里直接透传
 */
static void arm_task_resolve_motor_targets(
    const ArmTaskContext* context,
    const ArmTaskMachinePose* pose,
    ArmTaskMotorTargets* targets)
{
    float pitch2_zero_angle_rad = 0.0f;
    float final_big_yaw_rad = 0.0f;
    float final_pitch1_rad = 0.0f;
    float final_pitch2_joint_rad = 0.0f;
    float final_roll2_rad = 0.0f;
    float final_pitch3_rad = 0.0f;
    float final_roll3_rad = 0.0f;
    float final_grip_rad = 0.0f;

    if (context == OM_NULL || pose == OM_NULL || targets == OM_NULL)
    {
        return;
    }

    if (arm_task_get_pitch2_zero_angle_rad(context, &pitch2_zero_angle_rad) != OM_TRUE &&
        context->pitch2_motor != OM_NULL && motor_get_feedback(context->pitch2_motor) != OM_NULL)
    {
        pitch2_zero_angle_rad = motor_get_feedback(context->pitch2_motor)->angle;
    }

    final_big_yaw_rad =
        g_arm_pose_normal.machine_values[ARM_TASK_MACHINE_BIG_YAW] +
        pose->machine_values[ARM_TASK_MACHINE_BIG_YAW];
    final_pitch1_rad =
        g_arm_pose_normal.machine_values[ARM_TASK_MACHINE_PITCH1] +
        pose->machine_values[ARM_TASK_MACHINE_PITCH1];
    final_pitch2_joint_rad =
        g_arm_pose_normal.machine_values[ARM_TASK_MACHINE_PITCH2] +
        pose->machine_values[ARM_TASK_MACHINE_PITCH2];
    final_roll2_rad =
        g_arm_pose_normal.machine_values[ARM_TASK_MACHINE_ROLL2] +
        pose->machine_values[ARM_TASK_MACHINE_ROLL2];
    final_pitch3_rad =
        g_arm_pose_normal.machine_values[ARM_TASK_MACHINE_PITCH3] +
        pose->machine_values[ARM_TASK_MACHINE_PITCH3];
    final_roll3_rad = pose->machine_values[ARM_TASK_MACHINE_ROLL3];
    final_grip_rad =
        g_arm_pose_normal.machine_values[ARM_TASK_MACHINE_GRIP] +
        pose->machine_values[ARM_TASK_MACHINE_GRIP];

    targets->big_yaw_rad = final_big_yaw_rad;
    targets->pitch1_rad = APP_ARM_PITCH1_TARGET_RATIO * final_pitch1_rad;
    targets->pitch2_rad =
        pitch2_zero_angle_rad +
        final_pitch2_joint_rad * (-APP_ARM_PITCH2_GEAR_RATIO);
    targets->roll2_rad = final_roll2_rad;
    targets->pitch3_rad = final_pitch3_rad;
    targets->roll3_rad = final_roll3_rad;
    targets->grip_rad = final_grip_rad;
}

/* 重力补偿统一按当前反馈角现算。
 * 姿态表只负责几何目标，补偿力矩则由当前实际构型决定。
 */
static void arm_task_compute_gravity_feedforward(
    ArmTaskContext* context,
    float* pitch1_torque_ff,
    float* pitch2_torque_ff,
    float* roll2_torque_ff,
    float* pitch3_torque_ff)
{
    const MotorFeedback* pitch1_feedback = OM_NULL;
    const MotorFeedback* pitch2_feedback = OM_NULL;
    const MotorFeedback* roll2_feedback = OM_NULL;
    const MotorFeedback* pitch3_feedback = OM_NULL;
    float pitch1_angle_rad = 0.0f;
    float pitch2_zero_angle_rad = 0.0f;
    float pitch2_angle_rad = 0.0f;
    float roll2_angle_rad = 0.0f;
    float pitch3_angle_rad = 0.0f;

    if (context == OM_NULL || pitch1_torque_ff == OM_NULL || pitch2_torque_ff == OM_NULL ||
        roll2_torque_ff == OM_NULL || pitch3_torque_ff == OM_NULL)
    {
        return;
    }

    pitch1_feedback = motor_get_feedback(context->pitch1_motor);
    pitch2_feedback = motor_get_feedback(context->pitch2_motor);
    roll2_feedback = motor_get_feedback(context->roll2_motor);
    pitch3_feedback = motor_get_feedback(context->pitch3_motor);

    /* Pitch1 在旧工程里通过 `config_full_mapping_one(..., -1)` 做了方向映射。
     * 当前新链里电机目标已经按 APP_ARM_PITCH1_TARGET_RATIO 取反，但反馈角默认仍是
     * 电机原始坐标。重力补偿需要的是机构语义下的 Pitch1 角，因此这里按同一比例
     * 把反馈角转换回旧工程的机械臂符号约定，避免前馈方向与实际重力方向相反。
     */
    pitch1_angle_rad = (pitch1_feedback != OM_NULL) ? (pitch1_feedback->angle * APP_ARM_PITCH1_TARGET_RATIO) : 0.0f;
    if (arm_task_get_pitch2_zero_angle_rad(context, &pitch2_zero_angle_rad) != OM_TRUE && pitch2_feedback != OM_NULL)
    {
        pitch2_zero_angle_rad = pitch2_feedback->angle;
    }

    pitch2_angle_rad = (pitch2_feedback != OM_NULL) ? pitch2_feedback->angle : pitch2_zero_angle_rad;
    roll2_angle_rad = (roll2_feedback != OM_NULL) ? roll2_feedback->angle : 0.0f;
    pitch3_angle_rad = (pitch3_feedback != OM_NULL) ? pitch3_feedback->angle : 0.0f;

    *pitch2_torque_ff = pitch2_grav_torque_calculate(
        pitch1_angle_rad,
        pitch2_angle_rad,
        pitch2_zero_angle_rad,
        pitch3_angle_rad,
        roll2_angle_rad);
    *pitch2_torque_ff = arm_task_clamp_float(
        *pitch2_torque_ff,
        APP_ARM_PITCH2_GRAVITY_FF_MIN,
        APP_ARM_PITCH2_GRAVITY_FF_MAX);

    /* 旧工程 main 的 Pitch1 实际发送链走 Position 帧，
     * Motor_mit_tff_caculation() 里对应的 pitch1 tff 也处于注释关闭状态。
     * 当前正式链保留这一路为 0，不改变现有控制语义。
     */
    *pitch1_torque_ff = 0.0f;

    *pitch3_torque_ff = pitch3_grav_torque_calcuate(
        pitch1_angle_rad,
        pitch2_angle_rad,
        pitch2_zero_angle_rad,
        pitch3_angle_rad,
        roll2_angle_rad);
    *roll2_torque_ff = roll2_grav_torque_calculate(
        pitch1_angle_rad,
        pitch2_angle_rad,
        pitch2_zero_angle_rad,
        pitch3_angle_rad,
        roll2_angle_rad);
}

static void arm_task_update_smoothed_targets(
    ArmTaskContext* context,
    const ArmTaskMotorTargets* desired_targets,
    float current_tick_s)
{
    if (context == OM_NULL || desired_targets == OM_NULL)
    {
        return;
    }

    arm_task_refresh_smoothed_targets_from_feedback(context);

    context->smoothed_targets.big_yaw_rad =
        arm_task_slew_value(
            context->smoothed_targets.big_yaw_rad,
            desired_targets->big_yaw_rad,
            APP_ARM_BIG_YAW_MAX_RATE_RAD_PER_S,
            current_tick_s);
    context->smoothed_targets.pitch1_rad =
        arm_task_slew_value(
            context->smoothed_targets.pitch1_rad,
            desired_targets->pitch1_rad,
            APP_ARM_PITCH1_MAX_RATE_RAD_PER_S,
            current_tick_s);
    context->smoothed_targets.pitch2_rad =
        arm_task_slew_value(
            context->smoothed_targets.pitch2_rad,
            desired_targets->pitch2_rad,
            APP_ARM_PITCH2_MAX_RATE_RAD_PER_S,
            current_tick_s);
    context->smoothed_targets.roll2_rad =
        arm_task_slew_value(
            context->smoothed_targets.roll2_rad,
            desired_targets->roll2_rad,
            APP_ARM_ROLL2_MAX_RATE_RAD_PER_S,
            current_tick_s);
    context->smoothed_targets.pitch3_rad =
        arm_task_slew_value(
            context->smoothed_targets.pitch3_rad,
            desired_targets->pitch3_rad,
            APP_ARM_PITCH3_MAX_RATE_RAD_PER_S,
            current_tick_s);
    context->smoothed_targets.roll3_rad =
        arm_task_slew_value(
            context->smoothed_targets.roll3_rad,
            arm_task_resolve_nearest_equivalent_rad(
                desired_targets->roll3_rad,
                context->smoothed_targets.roll3_rad),
            APP_ARM_ROLL3_MAX_RATE_RAD_PER_S,
            current_tick_s);
    context->smoothed_targets.grip_rad =
        arm_task_slew_value(
            context->smoothed_targets.grip_rad,
            desired_targets->grip_rad,
            APP_ARM_GRIP_MAX_RATE_RAD_PER_S,
            current_tick_s);
}

/* 统一的 angle-mode 下发 helper。 */
static void arm_task_apply_angle_target(
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

static void arm_task_apply_hold_angle_target(
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
    if (arm_task_feedback_online(feedback) != OM_TRUE)
    {
        arm_task_apply_angle_target(motor, 0.0f, 0.0f, 0.0f, 0.0f);
        return;
    }

    arm_task_apply_angle_target(motor, feedback->angle, kp, kd, 0.0f);
}

/* roll3 仍沿用旧工程的双环思路。
 * 当前 motor 层没有 DJI 的 angle mode，因此这里直接算到电流目标。
 */
static void arm_task_apply_roll3_target(ArmTaskContext* context, float target_roll3_rad, float current_tick_s)
{
    const MotorFeedback* feedback = OM_NULL;
    float angle_feedback_rad = 0.0f;
    float angle_feedback_deg = 0.0f;
    float target_roll3_deg = 0.0f;
    float speed_feedback_rpm = 0.0f;
    float speed_reference_rpm = 0.0f;
    float current_command = 0.0f;

    if (context == OM_NULL || context->roll3_motor == OM_NULL)
    {
        return;
    }

    feedback = motor_get_feedback(context->roll3_motor);
    if (feedback == OM_NULL || arm_task_feedback_online(feedback) != OM_TRUE ||
        arm_task_get_roll3_feedback_angle_rad(context, &angle_feedback_rad) != OM_TRUE)
    {
        arm_task_apply_angle_target(context->roll3_motor, 0.0f, 0.0f, 0.0f, 0.0f);
        return;
    }

    angle_feedback_deg = arm_task_rad_to_deg(angle_feedback_rad);
    target_roll3_deg =
        arm_task_resolve_nearest_equivalent_deg(
            arm_task_rad_to_deg(target_roll3_rad),
            angle_feedback_deg);
    speed_feedback_rpm = arm_task_rad_per_s_to_rpm(feedback->speed);
    speed_reference_rpm =
        pid_compute(
            &context->roll3_angle_pid,
            target_roll3_deg,
            angle_feedback_deg,
            current_tick_s);
    current_command =
        pid_compute(
            &context->roll3_speed_pid,
            speed_reference_rpm,
            speed_feedback_rpm,
            current_tick_s);

    (void)motor_set_current(context->roll3_motor, current_command);
    (void)motor_control_compute(context->roll3_motor);
}

static void arm_task_apply_offline_guard_output(ArmTaskContext* context, float current_tick_s)
{
    const MotorFeedback* roll3_feedback = OM_NULL;
    float roll3_angle_rad = 0.0f;
    float roll3_target_rad = 0.0f;

    if (context == OM_NULL)
    {
        return;
    }

    arm_task_apply_hold_angle_target(
        context->big_yaw_motor,
        APP_ARM_BIG_YAW_KP,
        APP_ARM_BIG_YAW_KD);
    arm_task_apply_hold_angle_target(
        context->pitch1_motor,
        APP_ARM_PITCH1_KP,
        APP_ARM_PITCH1_KD);
    arm_task_apply_hold_angle_target(
        context->pitch2_motor,
        APP_ARM_PITCH2_KP,
        APP_ARM_PITCH2_KD);
    arm_task_apply_hold_angle_target(
        context->roll2_motor,
        APP_ARM_ROLL2_KP,
        APP_ARM_ROLL2_KD);
    arm_task_apply_hold_angle_target(
        context->pitch3_motor,
        APP_ARM_PITCH3_KP,
        APP_ARM_PITCH3_KD);
    arm_task_apply_hold_angle_target(
        context->grip_motor,
        APP_ARM_GRIP_KP,
        APP_ARM_GRIP_KD);

    roll3_feedback = motor_get_feedback(context->roll3_motor);
    if (arm_task_feedback_online(roll3_feedback) == OM_TRUE &&
        arm_task_get_roll3_feedback_angle_rad(context, &roll3_angle_rad) == OM_TRUE)
    {
        /* 别的机械臂轴离线时，roll3 若仍在线，应继续保持“上一份有效目标”，
         * 不能把目标直接改成当前角，否则 6020 会在离线保护期间变成跟手模式：
         * 外力偏转时没有反向力，直到整臂恢复后才突然重新给力。
         */
        roll3_target_rad = (context->smoothed_targets_initialized == OM_TRUE) ?
                               context->smoothed_targets.roll3_rad :
                               roll3_angle_rad;
        arm_task_apply_roll3_target(context, roll3_target_rad, current_tick_s);
    }
    else
    {
        (void)motor_set_current(context->roll3_motor, 0.0f);
        (void)motor_control_compute(context->roll3_motor);
    }
}

/* RELEASE 模式不推进动作，只把各角轴保持在当前反馈位置。 */
static void arm_task_apply_release_output(ArmTaskContext* context)
{
    const MotorFeedback* feedback = OM_NULL;

    if (context == OM_NULL)
    {
        return;
    }

    feedback = motor_get_feedback(context->big_yaw_motor);
    arm_task_apply_angle_target(
        context->big_yaw_motor,
        (feedback != OM_NULL) ? feedback->angle : 0.0f,
        0.0f,
        0.0f,
        0.0f);

    feedback = motor_get_feedback(context->pitch1_motor);
    arm_task_apply_angle_target(
        context->pitch1_motor,
        (feedback != OM_NULL) ? feedback->angle : 0.0f,
        0.0f,
        0.0f,
        0.0f);

    feedback = motor_get_feedback(context->pitch2_motor);
    arm_task_apply_angle_target(
        context->pitch2_motor,
        (feedback != OM_NULL) ? feedback->angle : 0.0f,
        0.0f,
        0.0f,
        0.0f);

    feedback = motor_get_feedback(context->roll2_motor);
    arm_task_apply_angle_target(
        context->roll2_motor,
        (feedback != OM_NULL) ? feedback->angle : 0.0f,
        0.0f,
        0.0f,
        0.0f);

    feedback = motor_get_feedback(context->pitch3_motor);
    arm_task_apply_angle_target(
        context->pitch3_motor,
        (feedback != OM_NULL) ? feedback->angle : 0.0f,
        0.0f,
        0.0f,
        0.0f);

    feedback = motor_get_feedback(context->grip_motor);
    arm_task_apply_angle_target(
        context->grip_motor,
        (feedback != OM_NULL) ? feedback->angle : 0.0f,
        0.0f,
        0.0f,
        0.0f);

    (void)motor_set_current(context->roll3_motor, 0.0f);
    (void)motor_control_compute(context->roll3_motor);
}

/* 带时间窗的动作从“共享控制事实发生变化”那一刻重新计时。 */
static void arm_task_update_command_timer(ArmTaskContext* context, const ArmTaskSnapshot* snapshot)
{
    if (context == OM_NULL || snapshot == OM_NULL)
    {
        return;
    }

    if (context->snapshot_initialized != OM_TRUE ||
        arm_task_snapshot_changed(&context->last_snapshot, snapshot) == OM_TRUE)
    {
        context->last_snapshot = *snapshot;
        context->command_since_ms = osal_time_now_monotonic();
        context->snapshot_initialized = OM_TRUE;
    }
}

/* 机械臂控制主循环：
 * 读快照 -> 生成姿态表 -> 映射电机目标 -> 下发到 motor 抽象层 -> 发布 TX 请求。
 */
static void arm_task_run_once(ArmTaskContext* context)
{
    ArmTaskSnapshot snapshot = {0};
    ArmTaskCustomControllerSnapshot controller_snapshot = {0};
    ArmTaskMachinePose pose = {0};
    ArmTaskMotorTargets targets = {0};
    float pitch1_torque_ff = 0.0f;
    float pitch2_torque_ff = 0.0f;
    float roll2_torque_ff = 0.0f;
    float pitch3_torque_ff = 0.0f;
    const float current_tick_s = ((float)ARM_TASK_PERIOD_MS) / 1000.0f;
    const OsalTimeMs now_ms = osal_time_now_monotonic();
    OsalTimeMs elapsed_ms = 0u;
    OmBool custom_controller_mode_selected = OM_FALSE;
    OmBool custom_controller_input_ready = OM_FALSE;
    OmBool custom_controller_force_takeover_requested = OM_FALSE;
    OmBool custom_controller_active = OM_FALSE;

    if (context == OM_NULL)
    {
        return;
    }

    /* 确保电机已绑定，如果未绑定则尝试绑定 */
    if (context->motors_bound_flag != OM_TRUE)
    {
        if (arm_task_try_bind_motors(context) != OM_OK)
        {
            return;
        }
    }

    /* 加载系统状态快照和自定义控制器状态 */
    arm_task_load_snapshot(&snapshot);
    arm_task_load_custom_controller_snapshot(&controller_snapshot);
    
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
        context->custom_controller_alignment_done != OM_TRUE)
    {
        if (context->custom_controller_alignment_started_ms == 0u)
        {
            context->custom_controller_alignment_started_ms = now_ms;
        }
        else if (context->custom_controller_alignment_failed != OM_TRUE &&
                 (OsalTimeMs)(now_ms - context->custom_controller_alignment_started_ms) >=
                     APP_ARM_CUSTOM_CONTROLLER_ALIGNMENT_TIMEOUT_MS)
        {
            context->custom_controller_alignment_failed = OM_TRUE;
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
        context->custom_controller_alignment_done != OM_TRUE)
    {
        if (context->custom_controller_alignment_failed != OM_TRUE)
        {
            sh_clear_custom_controller_calibration_indicator();
        }
        context->custom_controller_alignment_done = OM_TRUE;
        context->custom_controller_reference_captured = OM_FALSE;
        context->custom_controller_was_active = OM_FALSE;
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
    
    /* 根据电机在线状态和底盘模式选择不同的控制策略 */
    if (arm_task_all_motors_online(context) != OM_TRUE)
    {
        /* 电机离线时，应用离线保护输出 */
        context->snapshot_initialized = OM_FALSE;
        context->smoothed_targets_initialized = OM_FALSE;
        arm_task_apply_offline_guard_output(context, current_tick_s);
    }
    else if (snapshot.chassis_mode == MODE_CHASSIS_RELEASE)
    {
        /* RELEASE 会让机械臂失力。重新回到可控模式时，必须从当前反馈重新建目标：
         * - 清掉动作计时，避免时间窗继续沿用上一次动作
         * - 清掉平滑目标，避免 first tick 先冲向旧目标再回 normal
         * - 清掉自定义控制器接管状态，避免 release 前后的参考残留
         */
        context->snapshot_initialized = OM_FALSE;
        context->smoothed_targets_initialized = OM_FALSE;
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
                context->custom_controller_alignment_done != OM_TRUE &&
                arm_task_custom_controller_alignment_reached(context) == OM_TRUE)
            {
                context->custom_controller_alignment_failed = OM_FALSE;
                sh_set_custom_controller_calibration_success();
                context->custom_controller_alignment_done = OM_TRUE;
                g_arm_task_custom_controller_alignment_done_debug = 1u;
            }

            /* 根据对齐状态和接管状态选择姿态来源 */
            if (context->custom_controller_alignment_done == OM_TRUE &&
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
        
        /* 计算重力前馈力矩补偿（pitch1-3和roll2） */
        arm_task_compute_gravity_feedforward(
            context,
            &pitch1_torque_ff,
            &pitch2_torque_ff,
            &roll2_torque_ff,
            &pitch3_torque_ff);

        /* 应用大yaw轴角度控制 */
        arm_task_apply_angle_target(
            context->big_yaw_motor,
            context->smoothed_targets.big_yaw_rad,
            APP_ARM_BIG_YAW_KP,
            APP_ARM_BIG_YAW_KD,
            0.0f);
        
        /* 应用pitch1轴角度控制（含重力前馈） */
        arm_task_apply_angle_target(
            context->pitch1_motor,
            context->smoothed_targets.pitch1_rad,
            APP_ARM_PITCH1_KP,
            APP_ARM_PITCH1_KD,
            pitch1_torque_ff);
        
        /* 应用pitch2轴角度控制（含重力前馈） */
        arm_task_apply_angle_target(
            context->pitch2_motor,
            context->smoothed_targets.pitch2_rad,
            APP_ARM_PITCH2_KP,
            APP_ARM_PITCH2_KD,
            pitch2_torque_ff);
        
        /* 应用roll2轴角度控制（含重力前馈） */
        arm_task_apply_angle_target(
            context->roll2_motor,
            context->smoothed_targets.roll2_rad,
            APP_ARM_ROLL2_KP,
            APP_ARM_ROLL2_KD,
            roll2_torque_ff);
        
        /* 应用pitch3轴角度控制（含重力前馈） */
        arm_task_apply_angle_target(
            context->pitch3_motor,
            context->smoothed_targets.pitch3_rad,
            APP_ARM_PITCH3_KP,
            APP_ARM_PITCH3_KD,
            pitch3_torque_ff);
        
        /* 应用roll3轴目标控制 */
        arm_task_apply_roll3_target(context, context->smoothed_targets.roll3_rad, current_tick_s);
        
        /* 应用夹爪电机角度控制 */
        arm_task_apply_angle_target(
            context->grip_motor,
            context->smoothed_targets.grip_rad,
            APP_ARM_GRIP_KP,
            APP_ARM_GRIP_KD,
            0.0f);
    }

    /* 提交电机TX数据到发送队列 */
    (void)motor_tx_dispatch_submit(MOTOR_TX_SOURCE_ARM);

    /* 发布电机TX请求事件，如果失败则上报致命错误并进入死循环 */
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

uint8_t arm_task_get_custom_controller_alignment_done(void)
{
    return g_arm_task_custom_controller_alignment_done_debug;
}

/* 固定周期执行，不阻塞等总线反馈。 */
static void arm_task_entry(void* arg)
{
    ArmTaskContext* context = (ArmTaskContext*)arg;
    OsalTimeMs deadline_cursor_ms = 0u;

    while (1)
    {
        arm_task_run_once(context);
        (void)sh_beat(SH_TASK_ARM);
        (void)osal_delay_until(&deadline_cursor_ms, ARM_TASK_PERIOD_MS, OM_NULL);
    }
}

/* 启动入口只负责上下文初始化和任务创建。 */
OmRet arm_task_start(void)
{
    static OsalThread* arm_task_thread = OM_NULL;
    static ArmTaskContext arm_task_context = {0};
    const OsalThreadAttr arm_task_attr = {
        "arm_task",
        ARM_TASK_STACK_BYTES,
        ARM_TASK_PRIORITY};
    OsalStatus status = OSAL_INVALID;
    OmRet ret = OM_OK;

    if (arm_task_thread != OM_NULL)
    {
        return OM_ERR_CONFLICT;
    }

    memset(&arm_task_context, 0, sizeof(arm_task_context));
    ret = arm_task_init_pids(&arm_task_context);
    if (ret != OM_OK)
    {
        return ret;
    }

    status = osal_thread_create(
        &arm_task_thread,
        &arm_task_attr,
        arm_task_entry,
        &arm_task_context);
    if (status != OSAL_OK)
    {
        arm_task_thread = OM_NULL;
        return OM_ERROR;
    }

    return OM_OK;
}
