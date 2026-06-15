#ifndef NEW_ROBOT_MODE_TASK_INTERNAL_H
#define NEW_ROBOT_MODE_TASK_INTERNAL_H

#include "core/om_cpu.h"
#include "module/state_machine/state_machine.h"
#include "module/task_channel/task_channel.h"
#include "module/task_context_pool/task_context_pool.h"
#include "task/mode_task/mode_task.h"
#include <stdint.h>

#define MODE_TASK_PERIOD_MS                                (6u)
#define MODE_TASK_INIT_PROGRESS_CHANNEL_CAPACITY           (8u)
#define MODE_TASK_RC_CHANNEL_CAPACITY_BYTES                (256u)
#define MODE_TASK_CUSTOM_CONTROLLER_CHANNEL_CAPACITY_BYTES (256u)
#define RC_IW_UP_THRESHOLD                                 (694u)
#define RC_IW_DN_THRESHOLD                                 (1354u)

/* 任务内部的 RC 快照。
 * 每轮只从共享输入取一份，后续所有判断都基于这份局部事实展开。
 */
typedef struct
{
    int16_t ch1;
    int16_t ch2;
    int16_t ch3;
    int16_t ch4;
    uint8_t sw1;
    uint8_t sw2;
    uint16_t iw;
    uint8_t online;
} ModeTaskRcSnapshot;

typedef struct
{
    uint8_t selected_motion_mode_id;
} ModeTaskModeSelectionRuntime;

typedef struct
{
    ChassisMode chassis_mode;
    ClampAction clamp_action;
    ExchangeAction exchange_action;
    uint8_t primary_turn_ore_flag;
} ModeTaskPresetActionRuntime;

typedef struct
{
    uint8_t ik_solver_mode;
    uint8_t ik_control_bank;
} ModeTaskRcIkRuntime;

typedef struct
{
    uint8_t grip_state;
} ModeTaskGripRuntime;

typedef struct
{
    ModeTaskBoardInitState state;
} ModeTaskBoardInitContext;

typedef struct
{
    ModeTaskMotorInitState state;
} ModeTaskMotorInitContext;

typedef struct
{
    ModeTaskControlDomainState domain_state;
    ModeTaskControlLinkState rc_link_state;
    ModeTaskControlLinkState custom_link_state;
    ModeTaskCustomControlState custom_control_state;
    OmBool action_enabled;
} ModeTaskOperationalContext;

typedef struct
{
    ModeTaskSystemState system_state;
    ModeTaskBoardInitContext board_init;
    ModeTaskMotorInitContext motor_init;
    ModeTaskOperationalContext operational;
} ModeTaskHierarchyContext;

typedef struct
{
    uint8_t last_sw1;
    uint8_t last_last_sw1;
    uint8_t last_sw2;
    uint16_t last_iw;
    uint8_t confirmed_motion_mode_id;
    ModeTaskSystemSnapshot system_snapshot;
    ArmTaskModeSnapshot arm_mode_snapshot;
    ChassisTaskModeSnapshot chassis_mode_snapshot;
    ModeTaskModeSelectionRuntime mode_selection_runtime;
    ModeTaskPresetActionRuntime preset_action_runtime;
    ModeTaskRcIkRuntime rc_ik_runtime;
    ModeTaskGripRuntime grip_runtime;
    ModeTaskHierarchyContext hierarchy_state;
    TaskPipeChannel rc_channel;
    TaskPipeChannel custom_controller_channel;
    InputRcSnapshot latest_rc_snapshot;
    InputCustomControllerSnapshot latest_custom_controller_snapshot;
    uint16_t flags;
    StateMachine operational_phase_machine;
    StateMachine motion_mode_machine;
} ModeTaskContext;

#define MODE_TASK_FLAG_INITIALIZED               (1u << 0u)
#define MODE_TASK_FLAG_RC_SNAPSHOT_READY         (1u << 1u)
#define MODE_TASK_FLAG_INIT_CAN_READY            (1u << 2u)
#define MODE_TASK_FLAG_INIT_SERIAL_READY         (1u << 3u)
#define MODE_TASK_FLAG_INIT_IMU_READY            (1u << 4u)
#define MODE_TASK_FLAG_INIT_CHASSIS_MOTOR_READY  (1u << 5u)
#define MODE_TASK_FLAG_INIT_ARM_MOTOR_READY      (1u << 6u)

extern ModeTaskDebugState g_mode_task_debug;
extern TaskMpscChannel g_mode_task_init_progress_channel;
extern TaskContextSlotId g_mode_task_slot_id;

static inline ModeTaskContext* mode_task_get_owner_context(void)
{
    return (ModeTaskContext*)task_context_pool_get_ptr(g_mode_task_slot_id);
}

#define g_mode_task_owner_context mode_task_get_owner_context()

void mode_task_load_rc_snapshot(ModeTaskRcSnapshot* snapshot);
void mode_task_drain_rc_snapshots(ModeTaskContext* context);
void mode_task_load_custom_controller_snapshot(
    const ModeTaskContext* context,
    InputCustomControllerSnapshot* snapshot);
void mode_task_drain_custom_controller_snapshots(ModeTaskContext* context);

void mode_task_build_system_snapshot(
    const ModeTaskContext* context,
    ModeTaskSystemSnapshot* snapshot);
void mode_task_build_arm_mode_snapshot(
    const ModeTaskContext* context,
    ArmTaskModeSnapshot* snapshot);
void mode_task_build_chassis_mode_snapshot(
    const ModeTaskContext* context,
    ChassisTaskModeSnapshot* snapshot);

OmBool mode_task_system_snapshot_changed(
    const ModeTaskSystemSnapshot* lhs,
    const ModeTaskSystemSnapshot* rhs);
OmBool mode_task_arm_mode_snapshot_changed(
    const ArmTaskModeSnapshot* lhs,
    const ArmTaskModeSnapshot* rhs);
OmBool mode_task_chassis_mode_snapshot_changed(
    const ChassisTaskModeSnapshot* lhs,
    const ChassisTaskModeSnapshot* rhs);

void mode_task_publish_control_snapshots(
    const ArmTaskModeSnapshot* arm_snapshot,
    const ChassisTaskModeSnapshot* chassis_snapshot);

void mode_task_reset_mode_selection_runtime(ModeTaskContext* context);
void mode_task_reset_preset_action_runtime(ModeTaskContext* context);
void mode_task_reset_rc_ik_runtime(ModeTaskContext* context);
void mode_task_reset_grip_runtime(ModeTaskContext* context);

void mode_task_enter_release(StateMachine* state_machine, void* context);
void mode_task_enter_mode_selection(StateMachine* state_machine, void* context);
void mode_task_enter_formal_control(StateMachine* state_machine, void* context);
void mode_task_enter_preset_action(StateMachine* state_machine, void* context);
void mode_task_exit_preset_action(StateMachine* state_machine, void* context);
void mode_task_enter_custom_takeover(StateMachine* state_machine, void* context);
void mode_task_exit_custom_takeover(StateMachine* state_machine, void* context);
void mode_task_enter_rc_ik(StateMachine* state_machine, void* context);
void mode_task_exit_rc_ik(StateMachine* state_machine, void* context);

void mode_task_board_init_context_reset(ModeTaskBoardInitContext* context);
void mode_task_motor_init_context_reset(ModeTaskMotorInitContext* context);
void mode_task_operational_context_reset(ModeTaskOperationalContext* context);
void mode_task_update_bootstrap_state_from_progress(ModeTaskContext* context);
void mode_task_update_phase_state_from_rc(
    ModeTaskContext* context,
    const ModeTaskRcSnapshot* rc_snapshot);
void mode_task_update_operational_system_state(
    ModeTaskContext* context,
    const ModeTaskRcSnapshot* rc_snapshot,
    const InputCustomControllerSnapshot* custom_snapshot);
void mode_task_process_mct_lifecycle_requests(
    ModeTaskContext* context,
    const ModeTaskRcSnapshot* rc_snapshot);
void mode_task_update_operational_domain(
    ModeTaskContext* context,
    const ModeTaskRcSnapshot* rc_snapshot,
    const InputCustomControllerSnapshot* custom_snapshot);
void mode_task_refresh_output_snapshots(ModeTaskContext* context);
OmBool mode_task_bootstrap_allows_control(
    const ModeTaskContext* context);
void mode_task_update_debug_state(const ModeTaskContext* context);
void mode_task_drain_init_progress_messages(ModeTaskContext* context);
void mode_task_ctx_init(void* ctx);
void mode_task_ctx_reset(void* ctx);
void mode_task_ctx_cleanup(void* ctx);
void mode_task_run_once(ModeTaskContext* context);

#endif
