#ifndef NEW_ROBOT_ARM_TASK_INTERNAL_H
#define NEW_ROBOT_ARM_TASK_INTERNAL_H

/* arm_task 内部共享头文件。
 * 职责：存放 arm_task 模块内部的类型定义、上下文结构体和共享 helper 声明，
 * 供 arm_task.c 与 arm_task_diag.c 共同使用。
 * 对外不暴露，调用方应使用 arm_task.h 或 arm_task_diag.h。
 */

#include "config/app_config.h"
#include "core/algorithm/controller/pid.h"
#include "driver/motor/motor.h"
#include "function/math_utils/math_utils.h"
#include "module/data_pool/data_pool.h"
#include "module/task_channel/task_channel.h"
#include "osal/osal_time.h"
#include "task/mode_task/mode_task.h"
#include "module/task_context_pool/task_context_pool.h"
#include <atomic/atomic.h>
#include <stdint.h>

#define ARM_TASK_PERIOD_MS                           APP_ARM_TASK_PERIOD_MS
#define ARM_TASK_BIG_YAW_CONTROL_PERIOD_MS           APP_ARM_BIG_YAW_CONTROL_PERIOD_MS
#define ARM_TASK_PITCH1_CONTROL_PERIOD_MS            APP_ARM_PITCH1_CONTROL_PERIOD_MS
#define ARM_TASK_PITCH2_CONTROL_PERIOD_MS            APP_ARM_PITCH2_CONTROL_PERIOD_MS
#define ARM_TASK_ROLL2_CONTROL_PERIOD_MS             APP_ARM_ROLL2_CONTROL_PERIOD_MS
#define ARM_TASK_PITCH3_CONTROL_PERIOD_MS            APP_ARM_PITCH3_CONTROL_PERIOD_MS
#define ARM_TASK_ROLL3_CONTROL_PERIOD_MS             APP_ARM_ROLL3_CONTROL_PERIOD_MS
#define ARM_TASK_GRIP_CONTROL_PERIOD_MS              APP_ARM_GRIP_CONTROL_PERIOD_MS
#define ARM_TASK_TX_REQUEST_PERIOD_MS                APP_ARM_TX_REQUEST_PERIOD_MS
#define ARM_TASK_STACK_BYTES                         (1024u * OSAL_STACK_WORD_BYTES)
#define ARM_TASK_PRIORITY                            (4u)
#define ARM_TASK_POSE_MACHINE_COUNT                  (7u)
#define ARM_TASK_CUSTOM_ALIGNMENT_RAD_THRESHOLD      (0.08f)
#define ARM_TASK_CUSTOM_CONTROLLER_CHANNEL_CAPACITY_BYTES (256u)

/* 机构角轴数量（big_yaw / pitch1 / pitch2 / roll2 / pitch3 / roll3 / grip）。 */
#define ARM_TASK_MACHINE_COUNT                   (7u)

/* 自定义控制器输入轴数量（Y / Z / X / Yaw / Pitch / Roll）。 */
#define ARM_TASK_CUSTOM_CONTROLLER_AXIS_COUNT    (6u)
#define ARM_TASK_CUSTOM_CONTROLLER_WORK_MODE_ENCODER (0u)
#define ARM_TASK_GO8010_RECENT_TIMEOUT_MS        (300u)
#define ARM_TASK_DAMIAO_RECENT_TIMEOUT_MS        (300u)

/* formal mode 快照通道容量。 */
#define ARM_TASK_MODE_CHANNEL_CAPACITY           (8u)

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
} ArmTaskMachineAxis;

/* 来自 mode_task 的共享控制事实快照。 */
typedef struct
{
    ChassisMode chassis_mode;
    ClampAction clamp_action;
    ExchangeAction exchange_action;
    uint8_t primary_turn_ore_flag;
    uint8_t custom_controller_force_takeover_flag;
} ArmTaskSnapshot;

/* 自定义控制器原始输入快照。 */
typedef struct
{
    uint8_t online;
    uint8_t work_mode;
    float angle_deg[ARM_TASK_CUSTOM_CONTROLLER_AXIS_COUNT];
} ArmTaskCustomControllerSnapshot;

/* 机械臂姿态使用当前正式链的"机构角"定义：
 * - big_yaw / pitch1 / pitch2 / roll2 / pitch3 / roll3 / grip：单位 rad
 * - roll3 仍保持 GM6020 单圈物理角语义，但在 arm_task 内部统一存成 rad
 *
 * 后续统一在一个地方映射到当前 motor 抽象层的绝对目标值。
 */
typedef struct
{
    float machine_values[ARM_TASK_MACHINE_COUNT];
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
    ARM_TASK_CUSTOM_AXIS_Y = 0u,
    ARM_TASK_CUSTOM_AXIS_Z,
    ARM_TASK_CUSTOM_AXIS_X,
    ARM_TASK_CUSTOM_AXIS_YAW,
    ARM_TASK_CUSTOM_AXIS_PITCH,
    ARM_TASK_CUSTOM_AXIS_ROLL,
} ArmTaskCustomControllerAxis;

/* 电机句柄查询：由 arm_task.c 内部缓存表支持。 */
static inline Motor* arm_task_get_motor(ArmTaskMachineAxis axis)
{
    extern Motor* g_arm_task_motor_cache[];
    return (axis < ARM_TASK_MACHINE_COUNT) ? g_arm_task_motor_cache[axis] : OM_NULL;
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
    TaskPipeChannel custom_controller_channel;

    /* 输入快照 */
    ModeTaskControlSnapshot latest_mode_snapshot;
    DpCustomControllerSnapshot latest_custom_controller_snapshot;
    ArmTaskSnapshot last_snapshot;

    /* 平滑目标值 */
    ArmTaskMotorTargets smoothed_targets;

    /* 时间戳：command + tx_request + 各轴控制 */
    OsalTimeMs command_since_ms;
    OsalTimeMs last_tx_request_ms;
    OsalTimeMs last_control_ms[ARM_TASK_MACHINE_COUNT];

    /* 自定义控制器状态 */
    float custom_controller_neutral_deg[ARM_TASK_CUSTOM_CONTROLLER_AXIS_COUNT];
    float custom_controller_filtered_delta_deg[ARM_TASK_CUSTOM_CONTROLLER_AXIS_COUNT];
    float custom_controller_roll3_reference_rad;
    float custom_controller_grip_reference_rad;
    OsalTimeMs custom_controller_alignment_started_ms;

    /* 标志位打包 */
    uint16_t flags;
    /* bit 0: motors_bound */
    /* bit 1: control_modes_armed */
    /* bit 2: snapshot_initialized */
    /* bit 3: smoothed_targets_initialized */
    /* bit 4: custom_controller_alignment_done */
    /* bit 5: custom_controller_alignment_failed */
    /* bit 6: custom_controller_reference_captured */
    /* bit 7: custom_controller_was_active */
    /* bit 8: custom_controller_filter_initialized */
    /* bit 9: mode_snapshot_ready */
} ArmTaskContext;

#define ARM_TASK_FLAG_MOTORS_BOUND               (1u << 0u)
#define ARM_TASK_FLAG_CONTROL_MODES_ARMED        (1u << 1u)
#define ARM_TASK_FLAG_SNAPSHOT_INITIALIZED       (1u << 2u)
#define ARM_TASK_FLAG_SMOOTHED_TARGETS_INIT      (1u << 3u)
#define ARM_TASK_FLAG_CUSTOM_ALIGNMENT_DONE      (1u << 4u)
#define ARM_TASK_FLAG_CUSTOM_ALIGNMENT_FAILED    (1u << 5u)
#define ARM_TASK_FLAG_CUSTOM_REF_CAPTURED        (1u << 6u)
#define ARM_TASK_FLAG_CUSTOM_WAS_ACTIVE          (1u << 7u)
#define ARM_TASK_FLAG_CUSTOM_FILTER_INIT         (1u << 8u)
#define ARM_TASK_FLAG_MODE_SNAPSHOT_READY       (1u << 9u)

/* arm_task 运行时上下文单例，由 arm_task.c 定义。 */
extern TaskContextSlotId g_arm_task_slot_id;
extern Motor* g_arm_task_motor_cache[ARM_TASK_MACHINE_COUNT];
static inline ArmTaskContext* arm_task_get_owner_context(void)
{
    return (ArmTaskContext*)task_context_pool_get_ptr(g_arm_task_slot_id);
}
#define g_arm_task_owner_context arm_task_get_owner_context()

/* 自定义控制器对齐完成调试指示，由 arm_task.c 定义。 */
extern volatile uint8_t g_arm_task_custom_controller_alignment_done_debug;

/* 获取 pitch2 GO8010 零位角（rad），失败返回 OM_FALSE。 */
OmBool arm_task_get_pitch2_zero_angle_rad(
    const ArmTaskContext* context,
    float* pitch2_zero_angle_rad);

/* 获取 roll3 单圈角度反馈（rad），失败返回 OM_FALSE。 */
OmBool arm_task_get_roll3_feedback_angle_rad(
    const ArmTaskContext* context,
    float* angle_rad);


/* 机构角姿态表常量，供跨文件姿态推进与逆向映射使用。 */
extern const ArmTaskMachinePose g_arm_pose_zero;
extern const ArmTaskMachinePose g_arm_pose_normal;

OmBool arm_task_load_snapshot(
    const ArmTaskContext* context,
    ArmTaskSnapshot* snapshot);
void arm_task_load_custom_controller_snapshot(
    const ArmTaskContext* context,
    ArmTaskCustomControllerSnapshot* snapshot);
void arm_task_drain_mode_snapshots(ArmTaskContext* context);
void arm_task_drain_custom_controller_snapshots(ArmTaskContext* context);
OmBool arm_task_snapshot_changed(const ArmTaskSnapshot* lhs, const ArmTaskSnapshot* rhs);
OmBool arm_task_custom_controller_takeover_active(
    const ArmTaskContext* context,
    const ArmTaskSnapshot* arm_snapshot,
    const ArmTaskCustomControllerSnapshot* custom_controller_snapshot);
OmBool arm_task_feedback_online(const MotorFeedback* feedback);
OmBool arm_task_motor_online(const Motor* motor);
OmBool arm_task_roll3_online(const ArmTaskContext* context);
OmRet arm_task_init_pids(void);
OmRet arm_task_try_bind_motors(ArmTaskContext* context);
OmRet arm_task_restore_control_modes(ArmTaskContext* context);
OmBool arm_task_get_pitch2_joint_feedback_rad(
    const ArmTaskContext* context,
    float* pitch2_joint_feedback_rad);
void arm_task_refresh_smoothed_targets_from_feedback(ArmTaskContext* context);
void arm_task_clear_smoothed_targets(ArmTaskContext* context);
void arm_task_reset_custom_controller_state(ArmTaskContext* context);
void arm_task_preprocess_custom_controller_delta_deg(
    ArmTaskContext* context,
    const ArmTaskCustomControllerSnapshot* input_snapshot,
    float output_delta_deg[ARM_TASK_CUSTOM_CONTROLLER_AXIS_COUNT]);
void arm_task_apply_custom_controller_alignment_pose(
    ArmTaskContext* context,
    ArmTaskMachinePose* pose);
OmBool arm_task_custom_controller_alignment_reached(ArmTaskContext* context);
void arm_task_capture_custom_controller_reference(
    ArmTaskContext* context,
    const ArmTaskCustomControllerSnapshot* custom_controller_snapshot);
void arm_task_update_custom_controller_reference_state(
    ArmTaskContext* context,
    OmBool custom_controller_mode_selected,
    OmBool custom_controller_active,
    const ArmTaskCustomControllerSnapshot* custom_controller_snapshot);
void arm_task_assign_pose(ArmTaskMachinePose* target, const ArmTaskMachinePose* source);
void arm_task_apply_custom_controller_pose(
    ArmTaskContext* context,
    const ArmTaskCustomControllerSnapshot* custom_controller_snapshot,
    ArmTaskMachinePose* pose);
void clamp_angle_handle(
    const ArmTaskSnapshot* snapshot,
    OsalTimeMs elapsed_ms,
    ArmTaskMachinePose* pose);
void arm_task_resolve_motor_targets(
    const ArmTaskContext* context,
    const ArmTaskMachinePose* pose,
    ArmTaskMotorTargets* targets);
void arm_task_compute_gravity_feedforward(
    ArmTaskContext* context,
    float* pitch1_torque_ff,
    float* pitch2_torque_ff,
    float* roll2_torque_ff,
    float* pitch3_torque_ff);
void arm_task_update_smoothed_targets(
    ArmTaskContext* context,
    const ArmTaskMotorTargets* desired_targets,
    float current_tick_s);
void arm_task_run_once(ArmTaskContext* context);

#endif
