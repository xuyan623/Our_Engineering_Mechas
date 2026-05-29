#ifndef NEW_ROBOT_ARM_TASK_INTERNAL_H
#define NEW_ROBOT_ARM_TASK_INTERNAL_H

/* arm_task 内部共享头文件。
 * 职责：存放 arm_task 模块内部的类型定义、上下文结构体和共享 helper 声明，
 * 供 arm_task.c 与 arm_task_diag.c 共同使用。
 * 对外不暴露，调用方应使用 arm_task.h 或 arm_task_diag.h。
 */

#include "core/algorithm/controller/pid.h"
#include "driver/motor/motor.h"
#include "function/math_utils/math_utils.h"
#include "module/data_pool/data_pool.h"
#include "module/task_channel/task_channel.h"
#include "osal/osal_time.h"
#include "task/mode_task/mode_task.h"
#include <atomic/atomic.h>
#include <stdint.h>

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
    TaskMpscChannel mode_channel;
    TaskPipeChannel custom_controller_channel;
    ModeTaskControlSnapshot latest_mode_snapshot;
    DpCustomControllerSnapshot latest_custom_controller_snapshot;
    ArmTaskSnapshot last_snapshot;
    ArmTaskMotorTargets smoothed_targets;
    OsalTimeMs command_since_ms;
    OsalTimeMs last_tx_request_ms;
    OsalTimeMs last_big_yaw_control_ms;
    OsalTimeMs last_pitch1_control_ms;
    OsalTimeMs last_pitch2_control_ms;
    OsalTimeMs last_roll2_control_ms;
    OsalTimeMs last_pitch3_control_ms;
    OsalTimeMs last_roll3_control_ms;
    OsalTimeMs last_grip_control_ms;
    OmBool motors_bound_flag;
    OmBool control_modes_armed_for_operational;
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
    OmBool mode_snapshot_ready;
} ArmTaskContext;

/* arm_task 运行时上下文单例，由 arm_task.c 定义。 */
extern ArmTaskContext* g_arm_task_owner_context;

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


/* Ji gou jiao chang zhu ji chu zi tai biao, gong ni xiang ying she shi yong. */
extern const ArmTaskMachinePose g_arm_pose_normal;

#endif
