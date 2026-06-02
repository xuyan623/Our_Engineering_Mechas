#ifndef NEW_ROBOT_MODE_TASK_INTERNAL_H
#define NEW_ROBOT_MODE_TASK_INTERNAL_H

#include "core/om_cpu.h"
#include "module/state_machine/state_machine.h"
#include "module/task_channel/task_channel.h"
#include "module/task_context_pool/task_context_pool.h"
#include "task/mode_task/mode_task.h"
#include <stdint.h>

#define MODE_TASK_PERIOD_MS                              (6u)
#define MODE_TASK_INIT_PROGRESS_CHANNEL_CAPACITY         (8u)
#define MODE_TASK_RC_CHANNEL_CAPACITY_BYTES              (256u)
#define MODE_TASK_CUSTOM_CONTROLLER_CHANNEL_CAPACITY_BYTES (256u)
#define RC_SWITCH_UP                                     (1u)
#define RC_SWITCH_DN                                     (2u)
#define RC_SWITCH_MI                                     (3u)
#define RC_IW_UP_THRESHOLD                               (694u)
#define RC_IW_DN_THRESHOLD                               (1354u)

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

/* 真正跨任务共享的控制事实。
 * 只保留下游任务会消费的正式结果，不把边沿历史提升到共享层。
 */
typedef struct
{
    GlobalMode global_mode;
    ChassisMode chassis_mode;
    ClampAction clamp_action;
    ExchangeAction exchange_action;
    uint8_t primary_turn_ore_flag;
    uint8_t custom_controller_force_takeover_flag;
} ModeTaskSharedState;

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

/* mode_task 的本地上下文。
 * 只保存当前任务自己的历史、共享结果真源和输入 channel。
 */
typedef struct
{
    uint8_t last_sw1;
    uint8_t last_last_sw1;
    uint8_t last_sw2;
    uint16_t last_iw;
    GlobalMode last_global_mode;
    ChassisMode last_chassis_mode;
    ModeTaskSharedState shared_state;
    ModeTaskHierarchyContext hierarchy_state;
    TaskPipeChannel rc_channel;
    TaskPipeChannel custom_controller_channel;
    DpRcSnapshot latest_rc_snapshot;
    DpCustomControllerSnapshot latest_custom_controller_snapshot;
    uint16_t flags;
    StateMachine global_machine;
    StateMachine chassis_machine;
} ModeTaskContext;

#define MODE_TASK_FLAG_INITIALIZED               (1u << 0u)
#define MODE_TASK_FLAG_CLAMP_READY               (1u << 1u)
#define MODE_TASK_FLAG_EXCHANGE_READY            (1u << 2u)
#define MODE_TASK_FLAG_RC_SNAPSHOT_READY         (1u << 3u)
#define MODE_TASK_FLAG_INIT_CAN_READY            (1u << 4u)
#define MODE_TASK_FLAG_INIT_SERIAL_READY         (1u << 5u)
#define MODE_TASK_FLAG_INIT_IMU_READY            (1u << 6u)
#define MODE_TASK_FLAG_INIT_CHASSIS_MOTOR_READY  (1u << 7u)
#define MODE_TASK_FLAG_INIT_ARM_MOTOR_READY      (1u << 8u)

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
    DpCustomControllerSnapshot* snapshot);
void mode_task_drain_custom_controller_snapshots(ModeTaskContext* context);
void mode_task_build_control_snapshot(
    const ModeTaskContext* context,
    const ModeTaskSharedState* state,
    ModeTaskControlSnapshot* snapshot);
OmBool mode_task_control_snapshot_changed(
    const ModeTaskControlSnapshot* lhs,
    const ModeTaskControlSnapshot* rhs);
void mode_task_publish_control_snapshot(
    const ModeTaskControlSnapshot* snapshot);
OmBool mode_task_shared_state_changed(
    const ModeTaskSharedState* lhs,
    const ModeTaskSharedState* rhs);
void mode_task_fill_release_shared_state(ModeTaskSharedState* state);
void mode_task_board_init_context_reset(ModeTaskBoardInitContext* context);
void mode_task_motor_init_context_reset(ModeTaskMotorInitContext* context);
void mode_task_operational_context_reset(ModeTaskOperationalContext* context);
void mode_task_update_bootstrap_state_from_progress(ModeTaskContext* context);
void mode_task_update_operational_system_state(
    ModeTaskContext* context,
    const ModeTaskRcSnapshot* rc_snapshot,
    const DpCustomControllerSnapshot* custom_snapshot);
void mode_task_process_mct_lifecycle_requests(
    ModeTaskContext* context,
    const ModeTaskRcSnapshot* rc_snapshot);
void mode_task_update_operational_domain(
    ModeTaskContext* context,
    const ModeTaskRcSnapshot* rc_snapshot,
    const DpCustomControllerSnapshot* custom_snapshot,
    const ModeTaskSharedState* shared_state);
OmBool mode_task_bootstrap_allows_control(
    const ModeTaskContext* context);
void mode_task_update_debug_state(const ModeTaskContext* context);
void mode_task_drain_init_progress_messages(ModeTaskContext* context);
void mode_task_ctx_init(void* ctx);
void mode_task_ctx_reset(void* ctx);
void mode_task_ctx_cleanup(void* ctx);
void mode_task_run_once(ModeTaskContext* context);

#endif
