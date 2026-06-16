#ifndef NEW_ROBOT_AT_INTERNAL_H
#define NEW_ROBOT_AT_INTERNAL_H

/* arm_task 内部共享头文件。
 * 职责：存放 arm_task 模块内部的类型定义、上下文结构体和共享 helper 声明，
 * 供 arm_task.c 与 arm_task_diag.c 共同使用。
 * 对外不暴露，调用方应使用 arm_task.h 或 arm_task_diag.h。
 */

#include "config/app_config.h"
#include "algorithm/arm_kinematics/arm_kinematics.h"
#include "core/algorithm/controller/pid.h"
#include "driver/motor/motor.h"
#include "function/math_utils/math_utils.h"
#include "module/task_channel/task_channel.h"
#include "osal/osal_time.h"
#include "task/input_task/input_task_snapshot.h"
#include "task/mode_task/mode_task.h"
#include "module/task_context_pool/task_context_pool.h"
#include <atomic/atomic.h>
#include <stdint.h>

#define AT_PERIOD_MS                           APP_AT_TASK_PERIOD_MS
#define AT_BIG_YAW_LOOP_MS           APP_AT_BIG_YAW_LOOP_MS
#define AT_PITCH1_LOOP_MS            APP_AT_PITCH1_LOOP_MS
#define AT_PITCH2_LOOP_MS            APP_AT_PITCH2_LOOP_MS
#define AT_ROLL2_LOOP_MS             APP_AT_ROLL2_LOOP_MS
#define AT_PITCH3_LOOP_MS            APP_AT_PITCH3_LOOP_MS
#define AT_ROLL3_LOOP_MS             APP_AT_ROLL3_LOOP_MS
#define AT_GRIP_LOOP_MS              APP_AT_GRIP_LOOP_MS
#define AT_TX_REQUEST_PERIOD_MS                APP_AT_TX_REQUEST_PERIOD_MS
#define AT_STACK_BYTES                         (1024u * OSAL_STACK_WORD_BYTES)
#define AT_PRIORITY                            (4u)
#define AT_POSE_MACHINE_COUNT                  (7u)
#define AT_CUSTOM_ALIGN_MAX_RAD      (0.08f)
#define AT_CUSTOM_CH_BYTES (256u)
#define AT_RC_CHANNEL_BYTES               (256u)

/* 机构角轴数量（big_yaw / pitch1 / pitch2 / roll2 / pitch3 / roll3 / grip）。 */
#define AT_MACHINE_COUNT                   (7u)

/* 自定义控制器输入轴数量（Y / Z / X / Yaw / Pitch / Roll）。 */
#define AT_CUSTOM_AXIS_COUNT    (6u)
#define AT_CUSTOM_WORK_ENCODER (0u)
#define AT_GO8010_RECENT_TIMEOUT_MS        (300u)
#define AT_DAMIAO_RECENT_TIMEOUT_MS        (300u)

/* formal mode 快照通道容量。 */
#define AT_MODE_CHANNEL_CAPACITY           (8u)

/* 机构角姿态表的索引顺序。
 * 保持与旧工程的 Machine_angle 轴顺序一致，便于直接迁移姿态表。
 */
typedef enum
{
    AT_MACHINE_BIG_YAW = 0u,
    AT_MACHINE_PITCH1,
    AT_MACHINE_PITCH2,
    AT_MACHINE_ROLL2,
    AT_MACHINE_PITCH3,
    AT_MACHINE_ROLL3,
    AT_MACHINE_GRIP,
} ArmTaskMachineAxis;

/* 来自 mode_task 的共享控制事实快照。 */
typedef struct
{
    ArmTaskMode arm_mode;
    uint8_t grip_state;
    uint8_t ik_solver_mode;
    uint8_t ik_control_bank;
    ChassisMode chassis_mode;
    ClampAction clamp_action;
    ExchangeAction exchange_action;
    uint8_t primary_turn_ore_flag;
} ArmTaskSnapshot;

/* 自定义控制器原始输入快照。 */
typedef struct
{
    uint8_t online;
    uint8_t work_mode;
    float angle_deg[AT_CUSTOM_AXIS_COUNT];
} ArmCustomSnapshot;

/* 机械臂动作姿态表使用当前正式链的"机构角增量"定义：
 * - 前 5 轴最终目标会与 g_arm_pose_normal 相加
 * - roll3 仍直接保存 GM6020 单圈物理角语义
 * - grip 当前仍沿用动作表本地语义，不并入本轮 IK
 *
 * 本轮 arm_kinematics 的 6 轴 joint vector 语义固定为：
 * - 与动作表机构角语义完全一致
 * - 每一轴都表示“反馈逆映射后的机构角 - 该轴零点”
 * - normal 姿态对应 joint 约为 0
 */
typedef struct
{
    float machine_values[AT_MACHINE_COUNT];
} ArmTaskMachinePose;

/* 机构角经映射后得到的各电机目标角（rad）。 */
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

/* 自定义控制器 6 轴到机构角的映射索引。 */
typedef enum
{
    AT_CUSTOM_AXIS_Y = 0u,
    AT_CUSTOM_AXIS_Z,
    AT_CUSTOM_AXIS_X,
    AT_CUSTOM_AXIS_YAW,
    AT_CUSTOM_AXIS_PITCH,
    AT_CUSTOM_AXIS_ROLL,
} ArmCustomAxis;

/* 电机句柄查询：由 arm_task.c 内部缓存表支持。 */
static inline Motor* arm_task_get_motor(ArmTaskMachineAxis axis)
{
    extern Motor* g_arm_task_motor_cache[];
    return (axis < AT_MACHINE_COUNT) ? g_arm_task_motor_cache[axis] : OM_NULL;
}

static inline float arm_task_joint_zero_rad(ArmTaskMachineAxis axis)
{
    switch (axis)
    {
    case AT_MACHINE_BIG_YAW:
        return APP_AT_JOINT_ZERO_BIG_YAW_RAD;
    case AT_MACHINE_PITCH1:
        return APP_AT_JOINT_ZERO_PITCH1_RAD;
    case AT_MACHINE_PITCH2:
        return APP_AT_JOINT_ZERO_PITCH2_RAD;
    case AT_MACHINE_ROLL2:
        return APP_AT_JOINT_ZERO_ROLL2_RAD;
    case AT_MACHINE_PITCH3:
        return APP_AT_JOINT_ZERO_PITCH3_RAD;
    case AT_MACHINE_ROLL3:
        return APP_AT_JOINT_ZERO_ROLL3_RAD;
    case AT_MACHINE_GRIP:
        return APP_AT_JOINT_ZERO_GRIP_RAD;
    default:
        return 0.0f;
    }
}

static inline float arm_task_get_motor_zero_angle(ArmTaskMachineAxis axis)
{
    switch (axis)
    {
    case AT_MACHINE_BIG_YAW:
        return APP_AT_JOINT_ZERO_BIG_YAW_RAD;
    case AT_MACHINE_PITCH1:
        return APP_AT_PITCH1_TARGET_RATIO * APP_AT_JOINT_ZERO_PITCH1_RAD;
    case AT_MACHINE_ROLL2:
        return APP_AT_JOINT_ZERO_ROLL2_RAD;
    case AT_MACHINE_PITCH3:
        return APP_AT_JOINT_ZERO_PITCH3_RAD;
    case AT_MACHINE_ROLL3:
        return APP_AT_JOINT_ZERO_ROLL3_RAD;
    case AT_MACHINE_GRIP:
        return APP_AT_JOINT_ZERO_GRIP_RAD;
    default:
        return 0.0f;
    }
}

/* arm_task 本地上下文：
 * - 电机句柄已外迁至模块局部缓存表，不再内嵌
 * - roll3 双环 PID 为模块局部静态实例，由 arm_task 直接计算电流下发
 * - 保存最近一次共享控制事实，用于动作时间窗推进
 * - pitch2 零位由 GO8010 owner 锁存，这里只读消费
 */
typedef struct
{
    /* 通信通道 */
    TaskMpscChannel mode_channel;
    TaskPipeChannel rc_channel;
    TaskPipeChannel custom_channel;

    /* 输入快照 */
    ArmTaskModeSnapshot latest_mode_snapshot;
    InputRcSnapshot latest_rc_snapshot;
    InputCustomSnapshot latest_custom_snapshot;
    ArmTaskSnapshot last_snapshot;
    ArmIkPose ik_target_pose;
    ArmIkJointVector last_ik_solved_joint_vector;

    /* 平滑目标值 */
    ArmTaskMotorTargets smoothed_targets;

    /* 时间戳：command + tx_request + 各轴控制 */
    OsalTimeMs command_since_ms;
    OsalTimeMs last_tx_request_ms;
    OsalTimeMs last_ik_solve_ms;
    OsalTimeMs last_control_ms[AT_MACHINE_COUNT];

    /* 自定义控制器状态 */
    float custom_neutral_deg[AT_CUSTOM_AXIS_COUNT];
    float custom_filtered_delta_deg[AT_CUSTOM_AXIS_COUNT];
    float custom_roll3_reference_rad;
    float custom_grip_reference_rad;
    OsalTimeMs custom_alignment_started_ms;

    /* 标志位打包 */
    uint16_t flags;
    /* bit 0: motors_bound */
    /* bit 1: control_modes_armed */
    /* bit 2: snapshot_initialized */
    /* bit 3: smoothed_targets_initialized */
    /* bit 4: custom_alignment_done */
    /* bit 5: custom_alignment_failed */
    /* bit 6: custom_reference_captured */
    /* bit 7: custom_was_active */
    /* bit 8: custom_filter_initialized */
    /* bit 9: mode_snapshot_ready */
    /* bit 10: rc_snapshot_ready */
    /* bit 11: ik_target_pose_initialized */
    /* bit 12: ik_last_solution_ready */
} ArmTaskContext;

#define AT_FLAG_MOTORS_BOUND               (1u << 0u)
#define AT_FLAG_CONTROL_MODES_ARMED        (1u << 1u)
#define AT_FLAG_SNAPSHOT_INITIALIZED       (1u << 2u)
#define AT_FLAG_SMOOTHED_TARGETS_INIT      (1u << 3u)
#define AT_FLAG_CUSTOM_ALIGNMENT_DONE      (1u << 4u)
#define AT_FLAG_CUSTOM_ALIGN_FAIL    (1u << 5u)
#define AT_FLAG_CUSTOM_CAPTURED        (1u << 6u)
#define AT_FLAG_CUSTOM_WAS_ACTIVE          (1u << 7u)
#define AT_FLAG_CUSTOM_FILTER_INIT         (1u << 8u)
#define AT_FLAG_MODE_SNAPSHOT_READY       (1u << 9u)
#define AT_FLAG_RC_SNAPSHOT_READY         (1u << 10u)
#define AT_FLAG_IK_TARGET_POSE_READY      (1u << 11u)
#define AT_FLAG_IK_SOLUTION_READY    (1u << 12u)

/* arm_task 运行时上下文单例，由 arm_task.c 定义。 */
extern TaskContextSlotId g_arm_task_slot_id;
extern Motor* g_arm_task_motor_cache[AT_MACHINE_COUNT];
static inline ArmTaskContext* arm_task_get_owner_context(void)
{
    return (ArmTaskContext*)task_context_pool_get_ptr(g_arm_task_slot_id);
}
#define g_arm_task_owner_context arm_task_get_owner_context()

/* 自定义控制器对齐完成调试指示，由 arm_task.c 定义。 */
extern volatile uint8_t g_arm_task_custom_align_done_dbg;

/* 获取 pitch2 GO8010 零位角（rad），失败返回 OM_FALSE。 */
OmBool arm_task_pitch2_zero_rad(
    const ArmTaskContext* context,
    float* pitch2_zero_angle_rad);

/* 获取 roll3 单圈角度反馈（rad），失败返回 OM_FALSE。 */
OmBool arm_task_roll3_feedback_rad(
    const ArmTaskContext* context,
    float* angle_rad);


/* 机构角姿态表常量，供跨文件姿态推进与逆向映射使用。 */
extern const ArmTaskMachinePose g_arm_pose_zero;
extern const ArmTaskMachinePose g_arm_pose_normal;

OmBool arm_task_load_snapshot(
    const ArmTaskContext* context,
    ArmTaskSnapshot* snapshot);
void arm_task_load_custom_snapshot(
    const ArmTaskContext* context,
    ArmCustomSnapshot* snapshot);
void arm_task_load_rc_snapshot(
    const ArmTaskContext* context,
    InputRcSnapshot* snapshot);
void arm_task_drain_mode_snapshots(ArmTaskContext* context);
void arm_task_drain_rc_snapshots(ArmTaskContext* context);
void arm_task_drain_custom(ArmTaskContext* context);
OmBool arm_task_snapshot_changed(const ArmTaskSnapshot* lhs, const ArmTaskSnapshot* rhs);
OmBool arm_task_custom_active(
    const ArmTaskContext* context,
    const ArmTaskSnapshot* arm_snapshot,
    const ArmCustomSnapshot* custom_snapshot);
OmBool arm_task_feedback_online(const MotorFeedback* feedback);
OmBool arm_task_motor_online(const Motor* motor);
OmBool arm_task_roll3_online(const ArmTaskContext* context);
OmRet arm_task_init_pids(void);
OmRet arm_task_try_bind_motors(ArmTaskContext* context);
OmRet arm_task_restore_modes(ArmTaskContext* context);
OmBool arm_task_pitch2_feedback_rad(
    const ArmTaskContext* context,
    float* pitch2_joint_feedback_rad);
OmBool arm_task_feedback_to_joint(
    const ArmTaskContext* context,
    ArmTaskMachineAxis axis,
    float feedback_angle_rad,
    float* machine_joint_rad);
OmBool arm_task_joint_to_target(
    const ArmTaskContext* context,
    ArmTaskMachineAxis axis,
    float machine_joint_rad,
    float* motor_target_rad);
void arm_task_sync_smooth_targets(ArmTaskContext* context);
void arm_task_clear_smoothed(ArmTaskContext* context);
void arm_task_reset_custom_state(ArmTaskContext* context);
void arm_task_filter_custom_deg(
    ArmTaskContext* context,
    const ArmCustomSnapshot* input_snapshot,
    float output_delta_deg[AT_CUSTOM_AXIS_COUNT]);
void arm_task_apply_align_pose(
    ArmTaskContext* context,
    ArmTaskMachinePose* pose);
OmBool arm_task_custom_align_reached(ArmTaskContext* context);
void arm_task_capture_custom_ref(
    ArmTaskContext* context,
    const ArmCustomSnapshot* custom_snapshot);
void arm_task_update_custom(
    ArmTaskContext* context,
    OmBool custom_mode_selected,
    OmBool custom_active,
    const ArmCustomSnapshot* custom_snapshot);
void arm_task_assign_pose(ArmTaskMachinePose* target, const ArmTaskMachinePose* source);
OmBool arm_task_get_ik_joint_vector(
    const ArmTaskContext* context,
    ArmIkJointVector* joint_vector);
void arm_task_pose_from_ik(
    const ArmIkJointVector* joint_vector,
    ArmTaskMachinePose* pose);
void arm_task_apply_custom_pose(
    ArmTaskContext* context,
    const ArmCustomSnapshot* custom_snapshot,
    ArmTaskMachinePose* pose);
void clamp_angle_handle(
    const ArmTaskSnapshot* snapshot,
    OsalTimeMs elapsed_ms,
    ArmTaskMachinePose* pose);
void arm_task_resolve_targets(
    const ArmTaskContext* context,
    const ArmTaskMachinePose* pose,
    ArmTaskMotorTargets* targets);
void arm_task_gravity_feedforward(
    ArmTaskContext* context,
    float* pitch1_torque_ff,
    float* pitch2_torque_ff,
    float* roll2_torque_ff,
    float* pitch3_torque_ff);
void arm_task_update_smoothed(
    ArmTaskContext* context,
    const ArmTaskMotorTargets* desired_targets,
    float current_tick_s);
void arm_task_run_once(ArmTaskContext* context);

#endif
