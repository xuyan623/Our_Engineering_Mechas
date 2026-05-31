#include "task/arm_task/arm_task.h"

#include "algorithm/gravity_comp/gravity_comp.h"
#include "config/app_config.h"
#include "driver/motor/motor.h"
#include "function/math_utils/math_utils.h"
#include "module/motor_tx_dispatch/motor_tx_dispatch.h"
#include "module/system_health/system_health.h"
#include "osal/osal.h"
#include "osal/osal_config.h"
#include "osal/osal_time.h"
#include "task/arm_task/arm_task_internal.h"
#include "task/mode_task/mode_task.h"
#include "task/motor_communications_task/mct.h"
#include <string.h>

#define ARM_TASK_PERIOD_MS                           (3u)
#define ARM_TASK_BIG_YAW_CONTROL_PERIOD_MS           (10u)
#define ARM_TASK_PITCH1_CONTROL_PERIOD_MS            (10u)
#define ARM_TASK_PITCH2_CONTROL_PERIOD_MS            (3u)
#define ARM_TASK_ROLL2_CONTROL_PERIOD_MS             (10u)
#define ARM_TASK_PITCH3_CONTROL_PERIOD_MS            (10u)
#define ARM_TASK_ROLL3_CONTROL_PERIOD_MS             (10u)
#define ARM_TASK_GRIP_CONTROL_PERIOD_MS              (10u)
#define ARM_TASK_TX_REQUEST_PERIOD_MS                (0u)
#define ARM_TASK_STACK_BYTES                         (1024u * OSAL_STACK_WORD_BYTES)
#define ARM_TASK_PRIORITY                            (4u)
#define ARM_TASK_POSE_MACHINE_COUNT                  (7u)
#define ARM_TASK_CUSTOM_CONTROLLER_WORK_MODE_ENCODER (0u)
#define ARM_TASK_CUSTOM_ALIGNMENT_RAD_THRESHOLD      (0.08f)
#define ARM_TASK_CUSTOM_CONTROLLER_CHANNEL_CAPACITY_BYTES (256u)

static const char* g_arm_task_big_yaw_name = "big_yaw";
static const char* g_arm_task_pitch1_name = "pitch1";
static const char* g_arm_task_pitch2_name = "pitch2";
static const char* g_arm_task_roll2_name = "roll2";
static const char* g_arm_task_pitch3_name = "pitch3";
static const char* g_arm_task_roll3_name = "roll3";
static const char* g_arm_task_grip_name = "grip";
volatile uint8_t g_arm_task_custom_controller_alignment_done_debug = 0u;
TaskContextSlotId g_arm_task_slot_id = 0;
static uint8_t g_arm_task_mode_channel_storage
    [sizeof(ModeTaskControlSnapshot) * ARM_TASK_MODE_CHANNEL_CAPACITY] = {0};
static OmAtomicU8 g_arm_task_mode_channel_ready_flags[ARM_TASK_MODE_CHANNEL_CAPACITY] = {0};
static uint8_t g_arm_task_custom_controller_channel_storage
    [ARM_TASK_CUSTOM_CONTROLLER_CHANNEL_CAPACITY_BYTES] = {0};
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
/* sw2=DN / sw2=UP 默认 / CHECK / PITCH3_TORQUE_COLLECTION / CUSTOM_CONTROLLER_NORMAL */
static const ArmTaskMachinePose g_arm_pose_zero = {
    {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 3.7306414f, 0.0f}};
/* 常驻基础姿态，所有模式均叠加此偏移 */
const ArmTaskMachinePose g_arm_pose_normal = {
    {0.0f, 0.0f, 0.0f, 0.0f, 0.1f, 0.0f, 1.8f}};
/* sw2=MI + sw1=UP + iw上边沿 -> GET_ENERGY_UNIT */
// static const ArmTaskMachinePose g_arm_pose_get_energy = {
//     {0.0f, 1.24218f, 1.19447f, 0.0f, 0.0f, 3.7306414f, -1.8f}};
static const ArmTaskMachinePose g_arm_pose_get_energy = {
    {0.0f, 0.64218f, 1.0447f, 0.8f, 1.3f, 3.7306414f, 0.0f}};
/* sw2=MI + sw1=UP + iw下边沿 -> GET_ENERGY_UNIT1 */
// static const ArmTaskMachinePose g_arm_pose_get_energy1 = {
//     {-0.00667f, 1.035f, (5.53f / 6.33f) + 0.34f + 0.1f, 0.6178f, -0.194f, 2.1530383f, -1.8f}};
static const ArmTaskMachinePose g_arm_pose_get_energy1 = {
    {0.0f, 0.64218f, 1.0447f, -0.8f, 1.3f, 3.7306414f, 0.0f}};
/* sw2=MI + sw1=MI + iw下边沿 -> GET_ENERGY_UNIT2 */
static const ArmTaskMachinePose g_arm_pose_get_energy2 = {
    {0.148584366f, 0.99088f, 1.04010f + 0.1f, 0.093270302f, 0.07834f, 2.7812520f, -1.8f}};
/* GET_ENERGY_UNIT 三段动作之第三段 / EXCHANGE PICK_ACTION1 中间态 */
static const ArmTaskMachinePose g_arm_pose_store_energy = {
    {1.141f, 1.1042f, 1.18567f, 0.016975f, -1.55555f, 2.8489184f, 0.0f}};
/* GET_ENERGY_UNIT2 二段动作之第二段 / EXCHANGE PICK_ACTION2 中间态 */
static const ArmTaskMachinePose g_arm_pose_store_energy1 = {
    {-1.9018459f, 1.00080109f, 1.008974f, 0.09136295f, -1.222925131f, 4.0876327f, 0.0f}};
/* sw2=MI + sw1=MI + iw上边沿 -> EXCHANGE */
static const ArmTaskMachinePose g_arm_pose_exchange = {
    {0.0f, 0.64218f, 1.0447f, 0.0f, 0.0f, 3.7306414f, 0.0f}};
/* EXCHANGE 子动作 PICK_ACTION1 后半段（sw1=DN 边沿触发） */
static const ArmTaskMachinePose g_arm_pose_exchange_pick = {
    {1.041f, 1.2042f, 1.08567f, 0.016975f, -1.55555f, 2.8489184f, -1.8f}};
/* EXCHANGE 子动作 PICK_ACTION2 后半段（sw1=UP 边沿触发） */
static const ArmTaskMachinePose g_arm_pose_exchange_pick1 = {
    {-1.90189481f, 1.1f, 1.00948341f, 0.092153325f, -1.363282f, 4.0026160f, -1.8f}};
/* sw2=MI + sw1=DN + iw上边沿 -> PRIMARY */
static const ArmTaskMachinePose g_arm_pose_primary = {
    {0.0f, 1.46691f, 2.0053f, 0.1192f, -1.6f, 0.5890486f, 0.0f}};
/* sw2=UP + sw1=MI + iw上边沿 -> SECONDARY_ORE */
static const ArmTaskMachinePose g_arm_pose_secondary_ore = {
    {0.0f, 1.48691f, 0.85f, -1.57f, 0.0f, 3.7306414f, 0.0f}};

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
            math_utils_apply_symmetric_deadband(
                raw_delta_deg,
                g_arm_task_custom_controller_deadband_deg[axis_index]);

        // 首次运行时直接使用死区处理后的值初始化滤波器
        if (context->custom_controller_filter_initialized != OM_TRUE)
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

    context->custom_controller_filter_initialized = OM_TRUE;
}

static void arm_task_drain_mode_snapshots(ArmTaskContext* context)
{
    ModeTaskControlSnapshot snapshot = {0};

    if (context == OM_NULL)
    {
        return;
    }

    while (task_mpsc_channel_receive_nonblocking(&context->mode_channel, &snapshot) == OM_OK)
    {
        context->latest_mode_snapshot = snapshot;
        context->mode_snapshot_ready = OM_TRUE;
    }
}

static void arm_task_drain_custom_controller_snapshots(ArmTaskContext* context)
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
static OmBool arm_task_load_snapshot(
    const ArmTaskContext* context,
    ArmTaskSnapshot* snapshot)
{
    if (context == OM_NULL || snapshot == OM_NULL || context->mode_snapshot_ready != OM_TRUE)
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

static void arm_task_load_custom_controller_snapshot(
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

static OmBool arm_task_roll3_online(const ArmTaskContext* context)
{
    if (context == OM_NULL)
    {
        return OM_FALSE;
    }

    return (arm_task_motor_online(context->roll3_motor) == OM_TRUE) ?
               OM_TRUE :
               OM_FALSE;
}

OmBool arm_task_get_roll3_feedback_angle_rad(
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

static OmRet arm_task_restore_control_modes(ArmTaskContext* context)
{
    if (context == OM_NULL || context->motors_bound_flag != OM_TRUE)
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

static void arm_task_clear_smoothed_targets(ArmTaskContext* context)
{
    if (context == OM_NULL)
    {
        return;
    }

    memset(&context->smoothed_targets, 0, sizeof(context->smoothed_targets));
    context->smoothed_targets_initialized = OM_FALSE;
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

    if (context->smoothed_targets_initialized == OM_TRUE)
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
        big_yaw_feedback = motor_get_feedback(context->big_yaw_motor);
        big_yaw_target_rad =
            (big_yaw_feedback != OM_NULL) ? big_yaw_feedback->angle : 0.0f;

        pitch1_feedback = motor_get_feedback(context->pitch1_motor);
        pitch1_target_motor_rad =
            (pitch1_feedback != OM_NULL) ? pitch1_feedback->angle : 0.0f;

        pitch2_feedback = motor_get_feedback(context->pitch2_motor);
        pitch2_target_motor_rad =
            (pitch2_feedback != OM_NULL) ? pitch2_feedback->angle : 0.0f;

        roll2_feedback = motor_get_feedback(context->roll2_motor);
        roll2_target_rad =
            (roll2_feedback != OM_NULL) ? roll2_feedback->angle : 0.0f;

        pitch3_feedback = motor_get_feedback(context->pitch3_motor);
        pitch3_target_rad =
            (pitch3_feedback != OM_NULL) ? pitch3_feedback->angle : 0.0f;

        if (arm_task_get_roll3_feedback_angle_rad(context, &roll3_target_rad) != OM_TRUE)
        {
            roll3_target_rad = g_arm_pose_normal.machine_values[ARM_TASK_MACHINE_ROLL3];
        }

        grip_feedback = motor_get_feedback(context->grip_motor);
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

static OmBool arm_task_custom_controller_alignment_reached(ArmTaskContext* context)
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

    big_yaw_feedback = motor_get_feedback(context->big_yaw_motor);
    pitch1_feedback = motor_get_feedback(context->pitch1_motor);
    pitch2_feedback = motor_get_feedback(context->pitch2_motor);
    roll2_feedback = motor_get_feedback(context->roll2_motor);
    pitch3_feedback = motor_get_feedback(context->pitch3_motor);
    if (arm_task_motor_online(context->big_yaw_motor) != OM_TRUE ||
        arm_task_motor_online(context->pitch1_motor) != OM_TRUE ||
        arm_task_motor_online(context->pitch2_motor) != OM_TRUE ||
        arm_task_motor_online(context->roll2_motor) != OM_TRUE ||
        arm_task_motor_online(context->pitch3_motor) != OM_TRUE ||
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
        math_utils_deg_to_rad(filtered_delta_deg[ARM_TASK_CUSTOM_AXIS_Y]);
    pose->machine_values[ARM_TASK_MACHINE_PITCH1] =
        math_utils_deg_to_rad(-filtered_delta_deg[ARM_TASK_CUSTOM_AXIS_Z]);
    pose->machine_values[ARM_TASK_MACHINE_PITCH2] =
        math_utils_deg_to_rad(-filtered_delta_deg[ARM_TASK_CUSTOM_AXIS_X]);
    pose->machine_values[ARM_TASK_MACHINE_ROLL2] =
        math_utils_deg_to_rad(filtered_delta_deg[ARM_TASK_CUSTOM_AXIS_YAW]);
    pose->machine_values[ARM_TASK_MACHINE_PITCH3] =
        math_utils_deg_to_rad(filtered_delta_deg[ARM_TASK_CUSTOM_AXIS_PITCH]);
    pose->machine_values[ARM_TASK_MACHINE_ROLL3] =
        context->custom_controller_roll3_reference_rad +
        math_utils_deg_to_rad(filtered_delta_deg[ARM_TASK_CUSTOM_AXIS_ROLL]);
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
    *pitch2_torque_ff = math_utils_clamp_float(
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
#if (APP_ARM_PITCH3_ENABLE_GRAVITY_FF == 0u)
    *pitch3_torque_ff = 0.0f;
#endif
    *roll2_torque_ff = roll2_grav_torque_calculate(
        pitch1_angle_rad,
        pitch2_angle_rad,
        pitch2_zero_angle_rad,
        pitch3_angle_rad,
        roll2_angle_rad);
}

/**
 * @brief 更新机械臂平滑目标值，对各个关节轴的目标角度进行速率限制平滑处理
 * 
 * 该函数使用斜坡滤波(slew rate limiting)算法，将期望的目标角度按照各关节的最大角速度限制
 * 进行平滑过渡，避免电机控制指令突变。同时会先从反馈刷新平滑目标值的基准状态。
 * 
 * 对于roll3轴，会先解析最近等效角度以处理角度环绕问题，确保平滑过渡的正确性。
 * 
 * @param context 机械臂任务上下文指针，包含当前平滑目标状态和各轴配置信息
 * @param desired_targets 期望的电机目标值指针，包含所有关节轴的目标角度(rad)
 * @param current_tick_s 当前时间步长(秒)，用于计算允许的最大角度变化量
 * 
 * @note 如果context或desired_targets为空指针，函数直接返回不执行任何操作
 * @note 各轴的最大角速度限制由APP_ARM_*_MAX_RATE_RAD_PER_S宏定义指定
 */
static void arm_task_update_smoothed_targets(
    ArmTaskContext* context,
    const ArmTaskMotorTargets* desired_targets,
    float current_tick_s)
{
    /* 参数有效性检查 */
    if (context == OM_NULL || desired_targets == OM_NULL)
    {
        return;
    }

    /* 从电机反馈刷新平滑目标值的基准状态 */
    arm_task_refresh_smoothed_targets_from_feedback(context);

    /* 对各关节轴应用斜坡滤波，限制角速度在允许范围内 */
    context->smoothed_targets.big_yaw_rad =
        math_utils_slew_value(
            context->smoothed_targets.big_yaw_rad,
            desired_targets->big_yaw_rad,
            APP_ARM_BIG_YAW_MAX_RATE_RAD_PER_S,
            current_tick_s);
    context->smoothed_targets.pitch1_rad =
        math_utils_slew_value(
            context->smoothed_targets.pitch1_rad,
            desired_targets->pitch1_rad,
            APP_ARM_PITCH1_MAX_RATE_RAD_PER_S,
            current_tick_s);
    context->smoothed_targets.pitch2_rad =
        math_utils_slew_value(
            context->smoothed_targets.pitch2_rad,
            desired_targets->pitch2_rad,
            APP_ARM_PITCH2_MAX_RATE_RAD_PER_S,
            current_tick_s);
    context->smoothed_targets.roll2_rad =
        math_utils_slew_value(
            context->smoothed_targets.roll2_rad,
            desired_targets->roll2_rad,
            APP_ARM_ROLL2_MAX_RATE_RAD_PER_S,
            current_tick_s);
    context->smoothed_targets.pitch3_rad =
        math_utils_slew_value(
            context->smoothed_targets.pitch3_rad,
            math_utils_resolve_nearest_equivalent_rad(
                desired_targets->pitch3_rad,
                context->smoothed_targets.pitch3_rad),
            APP_ARM_PITCH3_MAX_RATE_RAD_PER_S,
            current_tick_s);

    /* roll3轴需要特殊处理：先解析最近等效角度以处理角度环绕问题 */
    context->smoothed_targets.roll3_rad =
        math_utils_slew_value(
            context->smoothed_targets.roll3_rad,
            math_utils_resolve_nearest_equivalent_rad(
                desired_targets->roll3_rad,
                context->smoothed_targets.roll3_rad),
            APP_ARM_ROLL3_MAX_RATE_RAD_PER_S,
            current_tick_s);
    context->smoothed_targets.grip_rad =
        math_utils_slew_value(
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
    if (arm_task_motor_online(motor) != OM_TRUE)
    {
        arm_task_apply_angle_target(motor, 0.0f, 0.0f, 0.0f, 0.0f);
        return;
    }

    arm_task_apply_angle_target(motor, feedback->angle, kp, kd, 0.0f);
}

static void arm_task_reset_axis_control_state(ArmTaskContext* context)
{
    if (context == OM_NULL)
    {
        return;
    }

    pid_reset(&context->roll3_angle_pid);
    pid_reset(&context->roll3_speed_pid);
    context->last_big_yaw_control_ms = 0u;
    context->last_pitch1_control_ms = 0u;
    context->last_pitch2_control_ms = 0u;
    context->last_roll2_control_ms = 0u;
    context->last_pitch3_control_ms = 0u;
    context->last_roll3_control_ms = 0u;
    context->last_grip_control_ms = 0u;
}

static OmBool arm_task_should_run_big_yaw_control(
    ArmTaskContext* context,
    OsalTimeMs now_ms)
{
    if (context == OM_NULL)
    {
        return OM_FALSE;
    }

    if (context->last_big_yaw_control_ms != 0u &&
        (uint32_t)(now_ms - context->last_big_yaw_control_ms) <
            ARM_TASK_BIG_YAW_CONTROL_PERIOD_MS)
    {
        return OM_FALSE;
    }

    context->last_big_yaw_control_ms = now_ms;
    return OM_TRUE;
}

static OmBool arm_task_should_run_pitch1_control(
    ArmTaskContext* context,
    OsalTimeMs now_ms)
{
    if (context == OM_NULL)
    {
        return OM_FALSE;
    }

    if (context->last_pitch1_control_ms != 0u &&
        (uint32_t)(now_ms - context->last_pitch1_control_ms) <
            ARM_TASK_PITCH1_CONTROL_PERIOD_MS)
    {
        return OM_FALSE;
    }

    context->last_pitch1_control_ms = now_ms;
    return OM_TRUE;
}

static OmBool arm_task_should_run_pitch2_control(
    ArmTaskContext* context,
    OsalTimeMs now_ms)
{
    if (context == OM_NULL)
    {
        return OM_FALSE;
    }

    if (context->last_pitch2_control_ms != 0u &&
        (uint32_t)(now_ms - context->last_pitch2_control_ms) <
            ARM_TASK_PITCH2_CONTROL_PERIOD_MS)
    {
        return OM_FALSE;
    }

    context->last_pitch2_control_ms = now_ms;
    return OM_TRUE;
}

static OmBool arm_task_should_run_roll2_control(
    ArmTaskContext* context,
    OsalTimeMs now_ms)
{
    if (context == OM_NULL)
    {
        return OM_FALSE;
    }

    if (context->last_roll2_control_ms != 0u &&
        (uint32_t)(now_ms - context->last_roll2_control_ms) <
            ARM_TASK_ROLL2_CONTROL_PERIOD_MS)
    {
        return OM_FALSE;
    }

    context->last_roll2_control_ms = now_ms;
    return OM_TRUE;
}

static OmBool arm_task_should_run_pitch3_control(
    ArmTaskContext* context,
    OsalTimeMs now_ms)
{
    if (context == OM_NULL)
    {
        return OM_FALSE;
    }

    if (context->last_pitch3_control_ms != 0u &&
        (uint32_t)(now_ms - context->last_pitch3_control_ms) <
            ARM_TASK_PITCH3_CONTROL_PERIOD_MS)
    {
        return OM_FALSE;
    }

    context->last_pitch3_control_ms = now_ms;
    return OM_TRUE;
}

static OmBool arm_task_should_run_roll3_control(
    ArmTaskContext* context,
    OsalTimeMs now_ms)
{
    if (context == OM_NULL)
    {
        return OM_FALSE;
    }

    if (context->last_roll3_control_ms != 0u &&
        (uint32_t)(now_ms - context->last_roll3_control_ms) <
            ARM_TASK_ROLL3_CONTROL_PERIOD_MS)
    {
        return OM_FALSE;
    }

    context->last_roll3_control_ms = now_ms;
    return OM_TRUE;
}

static OmBool arm_task_pitch2_zero_ready(const ArmTaskContext* context)
{
    float zero_angle_rad = 0.0f;

    if (context == OM_NULL)
    {
        return OM_FALSE;
    }

    return arm_task_get_pitch2_zero_angle_rad(context, &zero_angle_rad);
}

static OmBool arm_task_should_run_grip_control(
    ArmTaskContext* context,
    OsalTimeMs now_ms)
{
    if (context == OM_NULL)
    {
        return OM_FALSE;
    }

    if (context->last_grip_control_ms != 0u &&
        (uint32_t)(now_ms - context->last_grip_control_ms) <
            ARM_TASK_GRIP_CONTROL_PERIOD_MS)
    {
        return OM_FALSE;
    }

    context->last_grip_control_ms = now_ms;
    return OM_TRUE;
}

static void arm_task_apply_current_command(
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

static void arm_task_apply_roll3_target(
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

    if (context == OM_NULL || context->roll3_motor == OM_NULL)
    {
        return;
    }

    feedback = motor_get_feedback(context->roll3_motor);
    if (arm_task_feedback_online(feedback) != OM_TRUE ||
        arm_task_get_roll3_feedback_angle_rad(context, &feedback_angle_rad) != OM_TRUE)
    {
        arm_task_apply_current_command(context->roll3_motor, 0.0f);
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
        &context->roll3_angle_pid,
        target_angle_deg,
        feedback_angle_deg,
        current_tick_s);
    command_current = pid_compute(
        &context->roll3_speed_pid,
        target_speed_rpm,
        feedback_speed_rpm,
        current_tick_s);
    arm_task_apply_current_command(
        context->roll3_motor,
        command_current);
}

static OmBool arm_task_should_submit_tx_request(
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
static void arm_task_apply_release_output(ArmTaskContext* context)
{
    if (context == OM_NULL)
    {
        return;
    }

    arm_task_reset_axis_control_state(context);
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
    arm_task_apply_current_command(context->roll3_motor, 0.0f);
    arm_task_apply_hold_angle_target(
        context->grip_motor,
        APP_ARM_GRIP_KP,
        APP_ARM_GRIP_KD);
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
    if (context->motors_bound_flag != OM_TRUE)
    {
        if (arm_task_try_bind_motors(context) != OM_OK)
        {
            return;
        }
    }
    if (mct_is_operational_active() != OM_TRUE)
    {
        context->control_modes_armed_for_operational = OM_FALSE;
    }
    else if (context->control_modes_armed_for_operational != OM_TRUE)
    {
        if (arm_task_restore_control_modes(context) != OM_OK)
        {
            return;
        }
        context->control_modes_armed_for_operational = OM_TRUE;
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
        context->snapshot_initialized = OM_FALSE;
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
        arm_task_compute_gravity_feedforward(
            context,
            &pitch1_torque_ff,
            &pitch2_torque_ff,
            &roll2_torque_ff,
            &pitch3_torque_ff);
        if (arm_task_should_run_big_yaw_control(context, now_ms) == OM_TRUE)
        {
            if (arm_task_motor_online(context->big_yaw_motor) == OM_TRUE)
            {
                arm_task_apply_angle_target(
                    context->big_yaw_motor,
                    context->smoothed_targets.big_yaw_rad,
                    APP_ARM_BIG_YAW_KP,
                    APP_ARM_BIG_YAW_KD,
                    0.0f);
            }
            else
            {
                arm_task_apply_hold_angle_target(
                    context->big_yaw_motor,
                    APP_ARM_BIG_YAW_KP,
                    APP_ARM_BIG_YAW_KD);
            }
        }
        if (arm_task_should_run_pitch1_control(context, now_ms) == OM_TRUE)
        {
            if (arm_task_motor_online(context->pitch1_motor) == OM_TRUE)
            {
                arm_task_apply_angle_target(
                    context->pitch1_motor,
                    context->smoothed_targets.pitch1_rad,
                    APP_ARM_PITCH1_KP,
                    APP_ARM_PITCH1_KD,
                    pitch1_torque_ff);
            }
            else
            {
                arm_task_apply_hold_angle_target(
                    context->pitch1_motor,
                    APP_ARM_PITCH1_KP,
                    APP_ARM_PITCH1_KD);
            }
        }
        if (arm_task_should_run_pitch2_control(context, now_ms) == OM_TRUE)
        {
            if (arm_task_motor_online(context->pitch2_motor) == OM_TRUE &&
                arm_task_pitch2_zero_ready(context) == OM_TRUE)
            {
                arm_task_apply_angle_target(
                    context->pitch2_motor,
                    context->smoothed_targets.pitch2_rad,
                    APP_ARM_PITCH2_KP,
                    APP_ARM_PITCH2_KD,
                    pitch2_torque_ff);
            }
            else
            {
                arm_task_apply_hold_angle_target(
                    context->pitch2_motor,
                    APP_ARM_PITCH2_KP,
                    APP_ARM_PITCH2_KD);
            }
        }
        if (arm_task_should_run_roll2_control(context, now_ms) == OM_TRUE)
        {
            if (arm_task_motor_online(context->roll2_motor) == OM_TRUE)
            {
                arm_task_apply_angle_target(
                    context->roll2_motor,
                    context->smoothed_targets.roll2_rad,
                    APP_ARM_ROLL2_KP,
                    APP_ARM_ROLL2_KD,
                    roll2_torque_ff);
            }
            else
            {
                arm_task_apply_hold_angle_target(
                    context->roll2_motor,
                    APP_ARM_ROLL2_KP,
                    APP_ARM_ROLL2_KD);
            }
        }
        if (arm_task_should_run_pitch3_control(context, now_ms) == OM_TRUE)
        {
            if (arm_task_motor_online(context->pitch3_motor) == OM_TRUE)
            {
                arm_task_apply_angle_target(
                    context->pitch3_motor,
                    context->smoothed_targets.pitch3_rad,
                    APP_ARM_PITCH3_KP,
                    APP_ARM_PITCH3_KD,
                    pitch3_torque_ff);
            }
            else
            {
                arm_task_apply_hold_angle_target(
                    context->pitch3_motor,
                    APP_ARM_PITCH3_KP,
                    APP_ARM_PITCH3_KD);
            }
        }
        if (arm_task_should_run_roll3_control(context, now_ms) == OM_TRUE)
        {
            arm_task_apply_roll3_target(
                context,
                context->smoothed_targets.roll3_rad,
                current_tick_s);
        }
        if (arm_task_should_run_grip_control(context, now_ms) == OM_TRUE)
        {
            if (arm_task_motor_online(context->grip_motor) == OM_TRUE)
            {
                arm_task_apply_angle_target(
                    context->grip_motor,
                    context->smoothed_targets.grip_rad,
                    APP_ARM_GRIP_KP,
                    APP_ARM_GRIP_KD,
                    0.0f);
            }
            else
            {
                arm_task_apply_hold_angle_target(
                    context->grip_motor,
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
static void arm_task_entry(void* arg)
{
    ArmTaskContext* context = (ArmTaskContext*)arg;
    OsalTimeMs deadline_cursor_ms = 0u;

    if (context == OM_NULL)
    {
        for (;;)
        {
            (void)osal_sleep_ms(1000u);
        }
    }

    while (1)
    {
        arm_task_run_once(context);
        (void)sh_beat(SH_TASK_ARM);
        (void)osal_delay_until(&deadline_cursor_ms, ARM_TASK_PERIOD_MS, OM_NULL);
    }
}

/* 启动入口只负责上下文初始化和任务创建。 */
/* VTable for arm_task context pool. */
static void arm_task_ctx_init(void* ctx)
{
    ArmTaskContext* self = (ArmTaskContext*)ctx;
    memset(self, 0, sizeof(ArmTaskContext));
    self->latest_custom_controller_snapshot.online = 0u;
    self->latest_custom_controller_snapshot.work_mode = 0u;
}

static void arm_task_ctx_reset(void* ctx)
{
    ArmTaskContext* self = (ArmTaskContext*)ctx;
    memset(&self->last_snapshot, 0, sizeof(self->last_snapshot));
    memset(&self->smoothed_targets, 0, sizeof(self->smoothed_targets));
    memset(&self->latest_mode_snapshot, 0, sizeof(self->latest_mode_snapshot));
    memset(&self->latest_custom_controller_snapshot, 0, sizeof(self->latest_custom_controller_snapshot));
    self->command_since_ms = 0u;
    self->last_tx_request_ms = 0u;
    self->last_big_yaw_control_ms = 0u;
    self->last_pitch1_control_ms = 0u;
    self->last_pitch2_control_ms = 0u;
    self->last_roll2_control_ms = 0u;
    self->last_pitch3_control_ms = 0u;
    self->last_roll3_control_ms = 0u;
    self->last_grip_control_ms = 0u;
    self->snapshot_initialized = OM_FALSE;
    self->smoothed_targets_initialized = OM_FALSE;
    self->custom_controller_alignment_done = OM_FALSE;
    self->custom_controller_alignment_failed = OM_FALSE;
    self->custom_controller_reference_captured = OM_FALSE;
    self->custom_controller_was_active = OM_FALSE;
    self->custom_controller_filter_initialized = OM_FALSE;
    self->custom_controller_alignment_started_ms = 0u;
    self->mode_snapshot_ready = OM_FALSE;
}

static void arm_task_ctx_cleanup(void* ctx)
{
    (void)ctx;
}

static const TaskContextVTable g_arm_task_vtable = {
    .task_name = "arm_task",
    .init = arm_task_ctx_init,
    .reset = arm_task_ctx_reset,
    .cleanup = arm_task_ctx_cleanup,
    .diag_online = OM_NULL,
    .diag_snapshot = OM_NULL,
};

/* 启动入口只负责上下文初始化和任务创建。 */
OmRet arm_task_start(void)
{
    static OsalThread* arm_task_thread = OM_NULL;
    const OsalThreadAttr arm_task_attr = {
        "arm_task",
        ARM_TASK_STACK_BYTES,
        ARM_TASK_PRIORITY};
    OsalStatus status = OSAL_INVALID;
    OmRet ret = OM_OK;
    ArmTaskContext* ctx = OM_NULL;

    if (arm_task_thread != OM_NULL)
    {
        return OM_ERR_CONFLICT;
    }

    g_arm_task_slot_id = task_context_pool_alloc("arm_task", sizeof(ArmTaskContext), &g_arm_task_vtable);
    if (g_arm_task_slot_id == 0u)
    {
        return OM_ERROR;
    }

    ctx = (ArmTaskContext*)task_context_pool_get_ptr(g_arm_task_slot_id);

    ret = task_mpsc_channel_init(
        &ctx->mode_channel,
        g_arm_task_mode_channel_storage,
        g_arm_task_mode_channel_ready_flags,
        sizeof(ModeTaskControlSnapshot),
        ARM_TASK_MODE_CHANNEL_CAPACITY);
    if (ret != OM_OK)
    {
        task_context_pool_free(g_arm_task_slot_id);
        g_arm_task_slot_id = 0u;
        return ret;
    }

    /* mode shi zheng shi zhuang tai owner de dang qian shi shi, yun xu zai qi dong shi zhi jie zhong ru dang qian formal kuai zhao. */
    if (mode_task_copy_control_snapshot(&ctx->latest_mode_snapshot) == OM_TRUE)
    {
        ctx->mode_snapshot_ready = OM_TRUE;
    }

    ret = task_pipe_channel_init(
        &ctx->custom_controller_channel,
        g_arm_task_custom_controller_channel_storage,
        ARM_TASK_CUSTOM_CONTROLLER_CHANNEL_CAPACITY_BYTES,
        sizeof(DpCustomControllerSnapshot));
    if (ret != OM_OK)
    {
        task_mpsc_channel_deinit(&ctx->mode_channel);
        task_context_pool_free(g_arm_task_slot_id);
        g_arm_task_slot_id = 0u;
        return ret;
    }

    ret = arm_task_init_pids(ctx);
    if (ret != OM_OK)
    {
        task_pipe_channel_deinit(&ctx->custom_controller_channel);
        task_mpsc_channel_deinit(&ctx->mode_channel);
        task_context_pool_free(g_arm_task_slot_id);
        g_arm_task_slot_id = 0u;
        return ret;
    }

    status = osal_thread_create(
        &arm_task_thread,
        &arm_task_attr,
        arm_task_entry,
        ctx);
    if (status != OSAL_OK)
    {
        task_pipe_channel_deinit(&ctx->custom_controller_channel);
        task_mpsc_channel_deinit(&ctx->mode_channel);
        task_context_pool_free(g_arm_task_slot_id);
        g_arm_task_slot_id = 0u;
        arm_task_thread = OM_NULL;
        return OM_ERROR;
    }

    return OM_OK;
}
OmRet arm_task_submit_mode_control_snapshot(
    const ModeTaskControlSnapshot* snapshot)
{
    if (snapshot == OM_NULL || g_arm_task_owner_context == OM_NULL)
    {
        return OM_ERROR_PARAM;
    }

    return task_mpsc_channel_submit_nonblocking(
        &g_arm_task_owner_context->mode_channel,
        snapshot);
}

OmRet arm_task_submit_custom_controller_snapshot(
    const DpCustomControllerSnapshot* snapshot)
{
    if (snapshot == OM_NULL || g_arm_task_owner_context == OM_NULL)
    {
        return OM_ERROR_PARAM;
    }

    return task_pipe_channel_submit_nonblocking(
        &g_arm_task_owner_context->custom_controller_channel,
        snapshot,
        OM_TRUE);
}
