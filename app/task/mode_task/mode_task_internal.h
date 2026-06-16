#ifndef NEW_ROBOT_MT_INTERNAL_H
#define NEW_ROBOT_MT_INTERNAL_H

#include "core/om_cpu.h"
#include "module/state_machine/state_machine.h"
#include "module/task_channel/task_channel.h"
#include "module/task_context_pool/task_context_pool.h"
#include "task/mode_task/mode_task.h"
#include <stdint.h>

#define MT_PERIOD_MS                                (6u)
#define MT_INIT_MSG_CAP           (8u)
#define MT_RC_CHANNEL_BYTES                (256u)
#define MT_CUSTOM_CH_BYTES (256u)
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
} ModeTaskSelectRuntime;

typedef struct
{
    ChassisMode chassis_mode;
    ClampAction clamp_action;
    ExchangeAction exchange_action;
    uint8_t primary_turn_ore_flag;
} ModeTaskPresetRuntime;

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
    ModeBoardInitState state;
} ModeBoardInitCtx;

typedef struct
{
    ModeMotorInitState state;
} ModeMotorInitCtx;

typedef struct
{
    ModeTaskDomainState domain_state;
    ModeLinkState rc_link_state;
    ModeLinkState custom_link_state;
    ModeTaskCustomState custom_control_state;
    OmBool action_enabled;
} ModeTaskPhaseContext;

typedef struct
{
    ModeTaskSystemState system_state;
    ModeBoardInitCtx board_init;
    ModeMotorInitCtx motor_init;
    ModeTaskPhaseContext operational;
} ModeHierarchyCtx;

typedef struct
{
    uint8_t last_sw1;
    uint8_t last_last_sw1;
    uint8_t last_sw2;
    uint16_t last_iw;
    uint8_t confirmed_motion_mode_id;
    ModeSystemSnap system_snapshot;
    ArmTaskModeSnapshot arm_mode_snapshot;
    ChassisModeSnap chassis_mode_snapshot;
    ModeTaskSelectRuntime mode_selection_runtime;
    ModeTaskPresetRuntime preset_action_runtime;
    ModeTaskRcIkRuntime rc_ik_runtime;
    ModeTaskGripRuntime grip_runtime;
    ModeHierarchyCtx hierarchy_state;
    TaskPipeChannel rc_channel;
    TaskPipeChannel custom_channel;
    InputRcSnapshot latest_rc_snapshot;
    InputCustomSnapshot latest_custom_snapshot;
    uint16_t flags;
    StateMachine operational_phase_machine;
    StateMachine motion_mode_machine;
} ModeTaskContext;

#define MT_FLAG_INITIALIZED               (1u << 0u)
#define MT_FLAG_RC_SNAPSHOT_READY         (1u << 1u)
#define MT_FLAG_INIT_CAN_READY            (1u << 2u)
#define MT_FLAG_INIT_SERIAL_READY         (1u << 3u)
#define MT_FLAG_INIT_IMU_READY            (1u << 4u)
#define MT_FLAG_CHASSIS_MOTOR_READY  (1u << 5u)
#define MT_FLAG_ARM_MOTOR_READY      (1u << 6u)

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
void mode_task_load_custom(
    const ModeTaskContext* context,
    InputCustomSnapshot* snapshot);
void mode_task_drain_custom(ModeTaskContext* context);

void mode_task_build_system(
    const ModeTaskContext* context,
    ModeSystemSnap* snapshot);
void mode_task_build_arm_mode(
    const ModeTaskContext* context,
    ArmTaskModeSnapshot* snapshot);
void mode_task_build_chassis(
    const ModeTaskContext* context,
    ChassisModeSnap* snapshot);

OmBool mode_task_system_changed(
    const ModeSystemSnap* lhs,
    const ModeSystemSnap* rhs);
OmBool mode_task_arm_changed(
    const ArmTaskModeSnapshot* lhs,
    const ArmTaskModeSnapshot* rhs);
OmBool mode_task_chassis_changed(
    const ChassisModeSnap* lhs,
    const ChassisModeSnap* rhs);

void mode_task_publish_snapshots(
    const ArmTaskModeSnapshot* arm_snapshot,
    const ChassisModeSnap* chassis_snapshot);

void mode_task_reset_select(ModeTaskContext* context);
void mode_task_reset_preset(ModeTaskContext* context);
void mode_task_reset_rc_ik_runtime(ModeTaskContext* context);
void mode_task_reset_grip_runtime(ModeTaskContext* context);

void mode_task_enter_release(StateMachine* state_machine, void* context);
void mode_task_enter_select(StateMachine* state_machine, void* context);
void mode_task_enter_formal(StateMachine* state_machine, void* context);
void mode_task_enter_preset_action(StateMachine* state_machine, void* context);
void mode_task_exit_preset_action(StateMachine* state_machine, void* context);
void mode_task_enter_custom(StateMachine* state_machine, void* context);
void mode_task_exit_custom(StateMachine* state_machine, void* context);
void mode_task_enter_rc_ik(StateMachine* state_machine, void* context);
void mode_task_exit_rc_ik(StateMachine* state_machine, void* context);

void mode_task_reset_board_init(ModeBoardInitCtx* context);
void mode_task_reset_motor_init(ModeMotorInitCtx* context);
void mode_task_reset_phase_context(ModeTaskPhaseContext* context);
void mode_task_update_bootstrap(ModeTaskContext* context);
void mode_task_update_phase(
    ModeTaskContext* context,
    const ModeTaskRcSnapshot* rc_snapshot);
void mode_task_update_system_state(
    ModeTaskContext* context,
    const ModeTaskRcSnapshot* rc_snapshot,
    const InputCustomSnapshot* custom_snapshot);
void mode_task_process_mct(
    ModeTaskContext* context,
    const ModeTaskRcSnapshot* rc_snapshot);
void mode_task_update_domain(
    ModeTaskContext* context,
    const ModeTaskRcSnapshot* rc_snapshot,
    const InputCustomSnapshot* custom_snapshot);
void mode_task_refresh_snapshots(ModeTaskContext* context);
OmBool mode_task_bootstrap_allows(
    const ModeTaskContext* context);
void mode_task_update_debug_state(const ModeTaskContext* context);
void mode_task_drain_init_messages(ModeTaskContext* context);
void mode_task_ctx_init(void* ctx);
void mode_task_ctx_reset(void* ctx);
void mode_task_ctx_cleanup(void* ctx);
void mode_task_run_once(ModeTaskContext* context);

#endif
