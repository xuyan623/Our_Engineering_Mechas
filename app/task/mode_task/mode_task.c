#include "task/mode_task/mode_task.h"

#include "core/om_cpu.h"
#include "module/data_pool/data_pool.h"
#include "module/event_bus/event_bus.h"
#include "module/state_machine/state_machine.h"
#include "module/system_health/system_health.h"
#include "module/task_channel/task_channel.h"
#include "osal/osal.h"
#include "osal/osal_config.h"
#include "osal/osal_time.h"
#include "task/arm_task/arm_task.h"
#include "task/chassis_task/chassis_task.h"
#include "task/motor_communications_task/mct.h"
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>

#define MODE_TASK_PERIOD_MS     (6u)
#define MODE_TASK_INIT_PROGRESS_CHANNEL_CAPACITY (8u)
#define MODE_TASK_RC_CHANNEL_CAPACITY_BYTES (256u)
#define MODE_TASK_CUSTOM_CONTROLLER_CHANNEL_CAPACITY_BYTES (256u)
#define RC_SWITCH_UP            (1u)
#define RC_SWITCH_DN            (2u)
#define RC_SWITCH_MI            (3u)
#define RC_IW_UP_THRESHOLD      (694u)
#define RC_IW_DN_THRESHOLD      (1354u)

/* 任务内部的 rc 快照：
 * 每轮从共享池读一次，后续所有模式判断都基于这份快照完成，
 * 避免一轮逻辑里多次读共享池导致前后不一致。
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

/* 被其他任务消费的共享控制事实。
 * 这是 mode_task 每轮真正要写回 DataPool 的内容，
 * 边沿历史、ready 标志等局部状态都不允许提升到这里。
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

typedef struct
{
    OmBool can_ready;
    OmBool serial_ready;
    OmBool imu_ready;
    OmBool chassis_motor_ready;
    OmBool arm_motor_ready;
} ModeTaskInitProgressContext;

/* mode_task 的本地上下文：
 * - last_* 保存边沿历史，完全留在任务内部
 * - shared_state 保存当前导出的控制结果
 * - hierarchy_state 保存新的分层状态真源
 * - 两个旧状态机当前继续承担兼容输出记录器角色
 */
typedef struct
{
    OmBool initialized;
    uint8_t last_sw1;
    uint8_t last_last_sw1;
    uint8_t last_sw2;
    uint16_t last_iw;
    GlobalMode last_global_mode;
    ChassisMode last_chassis_mode;
    OmBool clamp_ready_to_change;
    OmBool exchange_ready_to_change;
    ModeTaskSharedState shared_state;
    ModeTaskHierarchyContext hierarchy_state;
    ModeTaskInitProgressContext init_progress;
    TaskPipeChannel rc_channel;
    TaskPipeChannel custom_controller_channel;
    DpRcSnapshot latest_rc_snapshot;
    DpCustomControllerSnapshot latest_custom_controller_snapshot;
    OmBool rc_snapshot_ready;
    StateMachine global_machine;
    StateMachine chassis_machine;
} ModeTaskContext;

ModeTaskDebugState g_mode_task_debug = {0};
static TaskMpscChannel g_mode_task_init_progress_channel = {0};
static uint8_t g_mode_task_init_progress_storage
    [sizeof(ModeTaskInitProgressMessage) * MODE_TASK_INIT_PROGRESS_CHANNEL_CAPACITY] = {0};
static OmAtomicU8 g_mode_task_init_progress_ready_flags[MODE_TASK_INIT_PROGRESS_CHANNEL_CAPACITY] = {0};
static uint8_t g_mode_task_rc_channel_storage[MODE_TASK_RC_CHANNEL_CAPACITY_BYTES] = {0};
static uint8_t g_mode_task_custom_controller_channel_storage
    [MODE_TASK_CUSTOM_CONTROLLER_CHANNEL_CAPACITY_BYTES] = {0};
static ModeTaskContext* g_mode_task_owner_context = OM_NULL;
static OmBool mode_task_bootstrap_allows_compat_control(
    const ModeTaskContext* context);

/* 当前全局/底盘状态机先不挂 enter/execute/exit 动作，
 * 只用它来统一记录 current/previous 状态，避免后续再引入第二套模式事实。
 */
static const State g_mode_global_states[] = {
    {.id = (StateId)MODE_GLOBAL_RELEASE_CTRL, .on_enter = OM_NULL, .on_execute = OM_NULL, .on_exit = OM_NULL, .name = "release"},
    {.id = (StateId)MODE_GLOBAL_MANUAL_CTRL, .on_enter = OM_NULL, .on_execute = OM_NULL, .on_exit = OM_NULL, .name = "manual"},
    {.id = (StateId)MODE_GLOBAL_ENGINEER_CTRL, .on_enter = OM_NULL, .on_execute = OM_NULL, .on_exit = OM_NULL, .name = "engineer"},
};

static const State g_mode_chassis_states[] = {
    {.id = (StateId)MODE_CHASSIS_RELEASE, .on_enter = OM_NULL, .on_execute = OM_NULL, .on_exit = OM_NULL, .name = "release"},
    {.id = (StateId)MODE_CHASSIS_NORMAL, .on_enter = OM_NULL, .on_execute = OM_NULL, .on_exit = OM_NULL, .name = "normal"},
    {.id = (StateId)MODE_CHASSIS_PITCH3_TORQUE_COLLECTION, .on_enter = OM_NULL, .on_execute = OM_NULL, .on_exit = OM_NULL, .name = "pitch3"},
    {.id = (StateId)MODE_CHASSIS_URGENT_MEASURE, .on_enter = OM_NULL, .on_execute = OM_NULL, .on_exit = OM_NULL, .name = "urgent"},
    {.id = (StateId)MODE_CHASSIS_EXCHANGE, .on_enter = OM_NULL, .on_execute = OM_NULL, .on_exit = OM_NULL, .name = "exchange"},
    {.id = (StateId)MODE_CHASSIS_PRIMARY, .on_enter = OM_NULL, .on_execute = OM_NULL, .on_exit = OM_NULL, .name = "primary"},
    {.id = (StateId)MODE_CHASSIS_GET_ENERGY_UNIT, .on_enter = OM_NULL, .on_execute = OM_NULL, .on_exit = OM_NULL, .name = "get_energy"},
    {.id = (StateId)MODE_CHASSIS_GET_ENERGY_UNIT1, .on_enter = OM_NULL, .on_execute = OM_NULL, .on_exit = OM_NULL, .name = "get_energy1"},
    {.id = (StateId)MODE_CHASSIS_GET_ENERGY_UNIT2, .on_enter = OM_NULL, .on_execute = OM_NULL, .on_exit = OM_NULL, .name = "get_energy2"},
    {.id = (StateId)MODE_CHASSIS_SECONDARY_ORE, .on_enter = OM_NULL, .on_execute = OM_NULL, .on_exit = OM_NULL, .name = "secondary_ore"},
    {.id = (StateId)MODE_CHASSIS_CHECK, .on_enter = OM_NULL, .on_execute = OM_NULL, .on_exit = OM_NULL, .name = "check"},
    {.id = (StateId)MODE_CHASSIS_CUSTOM_CONTROLLER_NORMAL,
     .on_enter = OM_NULL,
     .on_execute = OM_NULL,
     .on_exit = OM_NULL,
     .name = "custom_controller_normal"},
};

/* 当前模式与控制操作对照整理：
 * 1. 全局模式由 sw2 直接决定：
 *    - DN -> RELEASE_CTRL：安全态，后续任务应清输出/停泵/发零命令
 *    - UP -> MANUAL_CTRL：遥控直接控制底盘与部分机构
 *    - MI -> ENGINEER_CTRL：进入取矿/兑换/一矿等动作模式域
 *
 * 2. 手动模式（MANUAL_CTRL）下的底盘模式：
 *    - sw1=UP  且 iw 上边沿 -> PITCH3_TORQUE_COLLECTION
 *    - sw1=MI  且 iw 上边沿 -> SECONDARY_ORE
 *    - sw1=DN  且 iw 上边沿 -> CHECK
 *    - sw1=MI  且 iw 下边沿 -> 从 NORMAL/SECONDARY_ORE 进入 CUSTOM_CONTROLLER_NORMAL
 *    - CUSTOM_CONTROLLER_NORMAL 下再次收到 iw 下边沿 -> 回到 NORMAL
 *    - 其余几个保持型特殊模式收到 iw 下边沿时回到 NORMAL
 *    - 不处于保持型特殊模式时，默认底盘模式为 NORMAL
 *
 * 3. 工程模式（ENGINEER_CTRL）下的底盘模式：
 *    - sw1=UP  且 iw 上边沿 -> GET_ENERGY_UNIT
 *    - sw1=UP  且 iw 下边沿 -> GET_ENERGY_UNIT1
 *    - sw1=MI  且 iw 上边沿 -> EXCHANGE
 *    - sw1=MI  且 iw 下边沿 -> GET_ENERGY_UNIT2
 *    - sw1=DN  且 iw 上边沿 -> PRIMARY
 *
 * 4. 共享动作状态的控制语义：
 *    - clamp_action：只在 GET_ENERGY_UNIT/1/2、PRIMARY、SECONDARY_ORE 中有效
 *      先要求 sw1 回到中位解锁，再由 sw1 的 MI->DN / MI->UP 边沿推进动作
 *    - exchange_action：只在 EXCHANGE 中有效，同样要求 sw1 先回中位，再由边沿选择 PICK_ACTION1/2
 *    - primary_turn_ore_flag：只在 PRIMARY 中有效，由 iw 下边沿触发，离开 PRIMARY 后立即清零
 *
 * 5. 这份 mode_task 当前只负责“模式/动作结果”的推导与共享，
 *    不在此处直接做底盘、电机、机械臂控制计算。
 *    真正的控制操作要由后续 C5/C6 任务根据这些共享状态去实现。
 */

static void mode_task_load_rc_snapshot(ModeTaskRcSnapshot* snapshot)
{
    if (snapshot == OM_NULL)
    {
        return;
    }

    memset(snapshot, 0, sizeof(*snapshot));
    if (g_mode_task_owner_context == OM_NULL ||
        g_mode_task_owner_context->rc_snapshot_ready != OM_TRUE)
    {
        return;
    }

    snapshot->ch1 = g_mode_task_owner_context->latest_rc_snapshot.ch1;
    snapshot->ch2 = g_mode_task_owner_context->latest_rc_snapshot.ch2;
    snapshot->ch3 = g_mode_task_owner_context->latest_rc_snapshot.ch3;
    snapshot->ch4 = g_mode_task_owner_context->latest_rc_snapshot.ch4;
    snapshot->sw1 = g_mode_task_owner_context->latest_rc_snapshot.sw1;
    snapshot->sw2 = g_mode_task_owner_context->latest_rc_snapshot.sw2;
    snapshot->iw = g_mode_task_owner_context->latest_rc_snapshot.iw;
    snapshot->online = g_mode_task_owner_context->latest_rc_snapshot.online;
}

static void mode_task_drain_rc_snapshots(ModeTaskContext* context)
{
    DpRcSnapshot snapshot = {0};

    if (context == OM_NULL)
    {
        return;
    }

    while (task_pipe_channel_receive(&context->rc_channel, &snapshot, 0u) == OM_OK)
    {
        context->latest_rc_snapshot = snapshot;
        context->rc_snapshot_ready = OM_TRUE;
    }
}

static void mode_task_load_custom_controller_snapshot(
    const ModeTaskContext* context,
    DpCustomControllerSnapshot* snapshot)
{
    if (context == OM_NULL || snapshot == OM_NULL)
    {
        return;
    }

    *snapshot = context->latest_custom_controller_snapshot;
}

static void mode_task_drain_custom_controller_snapshots(ModeTaskContext* context)
{
    DpCustomControllerSnapshot snapshot = {0};

    if (context == OM_NULL)
    {
        return;
    }

    while (task_pipe_channel_receive(
               &context->custom_controller_channel,
               &snapshot,
               0u) == OM_OK)
    {
        context->latest_custom_controller_snapshot = snapshot;
    }
}

static void mode_task_store_shared_state(const ModeTaskSharedState* state)
{
    DpModeCompatSnapshot snapshot = {0};

    if (state == OM_NULL)
    {
        return;
    }

    /* mode/action 是跨任务共享的最终结果，
     * 写回统一收敛在这个函数里，避免多个位置维护同一份事实。
     */
    snapshot.global_mode = (uint8_t)state->global_mode;
    snapshot.chassis_mode = (uint8_t)state->chassis_mode;
    snapshot.clamp_action = (uint8_t)state->clamp_action;
    snapshot.exchange_action = (uint8_t)state->exchange_action;
    snapshot.primary_turn_ore_flag = state->primary_turn_ore_flag;
    snapshot.custom_controller_force_takeover_flag =
        state->custom_controller_force_takeover_flag;
    dp_store_mode_compat_snapshot(&snapshot);
}

static void mode_task_build_control_snapshot(
    const ModeTaskContext* context,
    const ModeTaskSharedState* state,
    ModeTaskControlSnapshot* snapshot)
{
    if (context == OM_NULL || state == OM_NULL || snapshot == OM_NULL)
    {
        return;
    }

    snapshot->system_state = (uint8_t)context->hierarchy_state.system_state;
    snapshot->control_domain_state =
        (uint8_t)context->hierarchy_state.operational.domain_state;
    snapshot->global_mode = (uint8_t)state->global_mode;
    snapshot->chassis_mode = (uint8_t)state->chassis_mode;
    snapshot->clamp_action = (uint8_t)state->clamp_action;
    snapshot->exchange_action = (uint8_t)state->exchange_action;
    snapshot->primary_turn_ore_flag = state->primary_turn_ore_flag;
    snapshot->custom_controller_force_takeover_flag =
        state->custom_controller_force_takeover_flag;
}

static OmBool mode_task_control_snapshot_changed(
    const ModeTaskControlSnapshot* lhs,
    const ModeTaskControlSnapshot* rhs)
{
    if (lhs == OM_NULL || rhs == OM_NULL)
    {
        return OM_FALSE;
    }

    return (lhs->system_state != rhs->system_state ||
            lhs->control_domain_state != rhs->control_domain_state ||
            lhs->global_mode != rhs->global_mode ||
            lhs->chassis_mode != rhs->chassis_mode ||
            lhs->clamp_action != rhs->clamp_action ||
            lhs->exchange_action != rhs->exchange_action ||
            lhs->primary_turn_ore_flag != rhs->primary_turn_ore_flag ||
            lhs->custom_controller_force_takeover_flag !=
                rhs->custom_controller_force_takeover_flag)
               ? OM_TRUE :
               OM_FALSE;
}

static void mode_task_publish_control_snapshot(
    const ModeTaskControlSnapshot* snapshot)
{
    if (snapshot == OM_NULL)
    {
        return;
    }

    (void)chassis_task_submit_mode_control_snapshot(snapshot);
    (void)arm_task_submit_mode_control_snapshot(snapshot);
}

static OmBool mode_task_shared_state_changed(const ModeTaskSharedState* lhs, const ModeTaskSharedState* rhs)
{
    if (lhs == OM_NULL || rhs == OM_NULL)
    {
        return OM_FALSE;
    }

    return (lhs->global_mode != rhs->global_mode || lhs->chassis_mode != rhs->chassis_mode || lhs->clamp_action != rhs->clamp_action ||
            lhs->exchange_action != rhs->exchange_action || lhs->primary_turn_ore_flag != rhs->primary_turn_ore_flag ||
            lhs->custom_controller_force_takeover_flag != rhs->custom_controller_force_takeover_flag)
               ? OM_TRUE
               : OM_FALSE;
}

static void mode_task_fill_release_shared_state(ModeTaskSharedState* state)
{
    if (state == OM_NULL)
    {
        return;
    }

    state->global_mode = MODE_GLOBAL_RELEASE_CTRL;
    state->chassis_mode = MODE_CHASSIS_RELEASE;
    state->clamp_action = MODE_CLAMP_UN_CMD;
    state->exchange_action = MODE_EXCHANGE_UN_CMD;
    state->primary_turn_ore_flag = 0u;
    state->custom_controller_force_takeover_flag = 0u;
}

static void mode_task_init_progress_context_reset(
    ModeTaskInitProgressContext* context)
{
    if (context == OM_NULL)
    {
        return;
    }

    memset(context, 0, sizeof(*context));
}

static void mode_task_board_init_context_reset(ModeTaskBoardInitContext* context)
{
    if (context == OM_NULL)
    {
        return;
    }

    memset(context, 0, sizeof(*context));
    context->state = MODE_TASK_BOARD_INIT_CAN_INITIALIZING;
}

static void mode_task_motor_init_context_reset(ModeTaskMotorInitContext* context)
{
    if (context == OM_NULL)
    {
        return;
    }

    memset(context, 0, sizeof(*context));
    context->state = MODE_TASK_MOTOR_INIT_CHASSIS_INITIALIZING;
}

static void mode_task_rc_control_context_reset(ModeTaskOperationalContext* context)
{
    if (context == OM_NULL)
    {
        return;
    }

    context->domain_state = MODE_TASK_CONTROL_DOMAIN_RC;
    context->rc_link_state = MODE_TASK_CONTROL_LINK_OFFLINE;
    context->action_enabled = OM_FALSE;
}

static void mode_task_custom_control_context_reset(ModeTaskOperationalContext* context)
{
    if (context == OM_NULL)
    {
        return;
    }

    context->domain_state = MODE_TASK_CONTROL_DOMAIN_CUSTOM;
    context->custom_link_state = MODE_TASK_CONTROL_LINK_OFFLINE;
    context->custom_control_state = MODE_TASK_CUSTOM_CONTROL_ALIGNING;
    context->action_enabled = OM_FALSE;
}

static void mode_task_operational_context_reset(ModeTaskOperationalContext* context)
{
    if (context == OM_NULL)
    {
        return;
    }

    memset(context, 0, sizeof(*context));
    mode_task_rc_control_context_reset(context);
    mode_task_custom_control_context_reset(context);
}

static void mode_task_clear_contexts_from_system_state(
    ModeTaskHierarchyContext* hierarchy_state,
    ModeTaskSystemState next_system_state)
{
    if (hierarchy_state == OM_NULL)
    {
        return;
    }

    switch (next_system_state)
    {
    case MODE_TASK_SYSTEM_UNINITIALIZED:
        mode_task_board_init_context_reset(&hierarchy_state->board_init);
        mode_task_motor_init_context_reset(&hierarchy_state->motor_init);
        mode_task_operational_context_reset(&hierarchy_state->operational);
        break;

    case MODE_TASK_SYSTEM_BOARD_INITIALIZING:
        mode_task_board_init_context_reset(&hierarchy_state->board_init);
        mode_task_motor_init_context_reset(&hierarchy_state->motor_init);
        mode_task_operational_context_reset(&hierarchy_state->operational);
        break;

    case MODE_TASK_SYSTEM_MOTOR_INITIALIZING:
        mode_task_motor_init_context_reset(&hierarchy_state->motor_init);
        mode_task_operational_context_reset(&hierarchy_state->operational);
        break;

    case MODE_TASK_SYSTEM_RELEASE:
        mode_task_operational_context_reset(&hierarchy_state->operational);
        break;

    case MODE_TASK_SYSTEM_OPERATIONAL:
    default:
        break;
    }
}

static void mode_task_set_system_state(
    ModeTaskContext* context,
    ModeTaskSystemState next_system_state)
{
    if (context == OM_NULL ||
        context->hierarchy_state.system_state == next_system_state)
    {
        return;
    }

    mode_task_clear_contexts_from_system_state(
        &context->hierarchy_state,
        next_system_state);
    context->hierarchy_state.system_state = next_system_state;
}

static void mode_task_update_bootstrap_state_from_progress(ModeTaskContext* context)
{
    if (context == OM_NULL)
    {
        return;
    }

    if (context->hierarchy_state.system_state == MODE_TASK_SYSTEM_UNINITIALIZED)
    {
        mode_task_set_system_state(context, MODE_TASK_SYSTEM_BOARD_INITIALIZING);
    }

    if (context->hierarchy_state.system_state == MODE_TASK_SYSTEM_BOARD_INITIALIZING)
    {
        if (context->init_progress.can_ready != OM_TRUE)
        {
            context->hierarchy_state.board_init.state =
                MODE_TASK_BOARD_INIT_CAN_INITIALIZING;
            return;
        }
        if (context->init_progress.serial_ready != OM_TRUE)
        {
            context->hierarchy_state.board_init.state =
                MODE_TASK_BOARD_INIT_SERIAL_INITIALIZING;
            return;
        }
        if (context->init_progress.imu_ready != OM_TRUE)
        {
            context->hierarchy_state.board_init.state =
                MODE_TASK_BOARD_INIT_IMU_INITIALIZING;
            return;
        }

        mode_task_set_system_state(context, MODE_TASK_SYSTEM_MOTOR_INITIALIZING);
    }

    if (context->hierarchy_state.system_state == MODE_TASK_SYSTEM_MOTOR_INITIALIZING)
    {
        if (context->init_progress.chassis_motor_ready != OM_TRUE)
        {
            context->hierarchy_state.motor_init.state =
                MODE_TASK_MOTOR_INIT_CHASSIS_INITIALIZING;
            return;
        }
        if (context->init_progress.arm_motor_ready != OM_TRUE)
        {
            context->hierarchy_state.motor_init.state =
                MODE_TASK_MOTOR_INIT_ARM_INITIALIZING;
            return;
        }

        mode_task_set_system_state(context, MODE_TASK_SYSTEM_RELEASE);
    }
}

static void mode_task_update_operational_system_state(
    ModeTaskContext* context,
    const ModeTaskRcSnapshot* rc_snapshot,
    const DpCustomControllerSnapshot* custom_snapshot)
{
    const OmBool rc_online =
        (rc_snapshot != OM_NULL && rc_snapshot->online != 0u) ? OM_TRUE : OM_FALSE;
    const OmBool custom_online =
        (custom_snapshot != OM_NULL && custom_snapshot->online != 0u) ? OM_TRUE : OM_FALSE;
    const OmBool custom_mode_active =
        (context != OM_NULL &&
         context->shared_state.chassis_mode == MODE_CHASSIS_CUSTOM_CONTROLLER_NORMAL)
            ? OM_TRUE
            : OM_FALSE;
    const OmBool operational_active = mct_is_operational_active();

    if (context == OM_NULL || rc_snapshot == OM_NULL || custom_snapshot == OM_NULL)
    {
        return;
    }

    if (mode_task_bootstrap_allows_compat_control(context) != OM_TRUE)
    {
        return;
    }

    if (rc_snapshot->sw2 == RC_SWITCH_DN || operational_active != OM_TRUE)
    {
        mode_task_set_system_state(context, MODE_TASK_SYSTEM_RELEASE);
        return;
    }

    if (custom_mode_active == OM_TRUE)
    {
        if (custom_online == OM_TRUE || rc_online == OM_TRUE)
        {
            mode_task_set_system_state(context, MODE_TASK_SYSTEM_OPERATIONAL);
        }
        else
        {
            mode_task_set_system_state(context, MODE_TASK_SYSTEM_RELEASE);
        }
        return;
    }

    if (rc_online == OM_TRUE)
    {
        mode_task_set_system_state(context, MODE_TASK_SYSTEM_OPERATIONAL);
    }
    else
    {
        mode_task_set_system_state(context, MODE_TASK_SYSTEM_RELEASE);
    }
}

static void mode_task_process_mct_lifecycle_requests(
    ModeTaskContext* context,
    const ModeTaskRcSnapshot* rc_snapshot)
{
    if (context == OM_NULL || rc_snapshot == OM_NULL)
    {
        return;
    }

    if (mode_task_bootstrap_allows_compat_control(context) != OM_TRUE)
    {
        return;
    }

    if (context->last_sw2 != RC_SWITCH_DN && rc_snapshot->sw2 == RC_SWITCH_DN)
    {
        context->hierarchy_state.operational.action_enabled = OM_FALSE;
        (void)mct_request_leave_operational_state();
        return;
    }

    if (context->last_sw2 == RC_SWITCH_DN && rc_snapshot->sw2 == RC_SWITCH_MI)
    {
        context->hierarchy_state.operational.action_enabled = OM_FALSE;
        (void)mct_request_enter_operational_state();
        return;
    }

    if (context->last_sw2 == RC_SWITCH_MI && rc_snapshot->sw2 == RC_SWITCH_UP)
    {
        context->hierarchy_state.operational.action_enabled = OM_TRUE;
        return;
    }
}

static void mode_task_update_operational_domain(
    ModeTaskContext* context,
    const ModeTaskRcSnapshot* rc_snapshot,
    const DpCustomControllerSnapshot* custom_snapshot,
    const ModeTaskSharedState* shared_state)
{
    const OmBool previous_action_enabled =
        (context != OM_NULL) ? context->hierarchy_state.operational.action_enabled : OM_FALSE;
    const OmBool rc_online =
        (rc_snapshot != OM_NULL && rc_snapshot->online != 0u) ? OM_TRUE : OM_FALSE;
    const OmBool custom_online =
        (custom_snapshot != OM_NULL && custom_snapshot->online != 0u) ? OM_TRUE : OM_FALSE;
    const OmBool custom_domain =
        (shared_state != OM_NULL &&
         shared_state->chassis_mode == MODE_CHASSIS_CUSTOM_CONTROLLER_NORMAL)
            ? OM_TRUE
            : OM_FALSE;

    if (context == OM_NULL || rc_snapshot == OM_NULL || custom_snapshot == OM_NULL ||
        shared_state == OM_NULL)
    {
        return;
    }

    if (context->hierarchy_state.system_state != MODE_TASK_SYSTEM_OPERATIONAL)
    {
        mode_task_operational_context_reset(&context->hierarchy_state.operational);
        return;
    }

    context->hierarchy_state.operational.rc_link_state =
        (rc_online == OM_TRUE) ? MODE_TASK_CONTROL_LINK_ONLINE : MODE_TASK_CONTROL_LINK_OFFLINE;
    context->hierarchy_state.operational.custom_link_state =
        (custom_online == OM_TRUE) ? MODE_TASK_CONTROL_LINK_ONLINE : MODE_TASK_CONTROL_LINK_OFFLINE;

    if (custom_domain == OM_TRUE)
    {
        if (context->hierarchy_state.operational.domain_state != MODE_TASK_CONTROL_DOMAIN_CUSTOM)
        {
            mode_task_custom_control_context_reset(&context->hierarchy_state.operational);
            context->hierarchy_state.operational.action_enabled = previous_action_enabled;
        }

        context->hierarchy_state.operational.domain_state = MODE_TASK_CONTROL_DOMAIN_CUSTOM;
        context->hierarchy_state.operational.custom_link_state =
            (custom_online == OM_TRUE) ? MODE_TASK_CONTROL_LINK_ONLINE : MODE_TASK_CONTROL_LINK_OFFLINE;
        context->hierarchy_state.operational.custom_control_state =
            ((arm_task_get_custom_controller_alignment_done() != 0u) ||
             shared_state->custom_controller_force_takeover_flag != 0u)
                ? MODE_TASK_CUSTOM_CONTROL_TAKEOVER
                : MODE_TASK_CUSTOM_CONTROL_ALIGNING;
    }
    else
    {
        if (context->hierarchy_state.operational.domain_state != MODE_TASK_CONTROL_DOMAIN_RC)
        {
            mode_task_rc_control_context_reset(&context->hierarchy_state.operational);
            context->hierarchy_state.operational.action_enabled = previous_action_enabled;
        }

        context->hierarchy_state.operational.domain_state = MODE_TASK_CONTROL_DOMAIN_RC;
        context->hierarchy_state.operational.rc_link_state =
            (rc_online == OM_TRUE) ? MODE_TASK_CONTROL_LINK_ONLINE : MODE_TASK_CONTROL_LINK_OFFLINE;
        context->hierarchy_state.operational.custom_control_state =
            MODE_TASK_CUSTOM_CONTROL_ALIGNING;
    }
}

static OmBool mode_task_bootstrap_allows_compat_control(
    const ModeTaskContext* context)
{
    if (context == OM_NULL)
    {
        return OM_FALSE;
    }

    /* step4 迁移期里，第一层状态先只覆盖冷启动到 RELEASE。
     * 一旦进入 RELEASE，旧 mode/action 兼容输出继续按当前逻辑派生；
     * 真正的 OPERATIONAL 分域留到后续 step7 再接。
     */
    return (context->hierarchy_state.system_state == MODE_TASK_SYSTEM_RELEASE ||
            context->hierarchy_state.system_state == MODE_TASK_SYSTEM_OPERATIONAL)
               ? OM_TRUE
               : OM_FALSE;
}

static void mode_task_update_debug_state(const ModeTaskContext* context)
{
    if (context == OM_NULL)
    {
        return;
    }

    g_mode_task_debug.system_state = (uint8_t)context->hierarchy_state.system_state;
    g_mode_task_debug.board_init_state = (uint8_t)context->hierarchy_state.board_init.state;
    g_mode_task_debug.motor_init_state = (uint8_t)context->hierarchy_state.motor_init.state;
    g_mode_task_debug.control_domain_state = (uint8_t)context->hierarchy_state.operational.domain_state;
    g_mode_task_debug.rc_link_state = (uint8_t)context->hierarchy_state.operational.rc_link_state;
    g_mode_task_debug.custom_link_state = (uint8_t)context->hierarchy_state.operational.custom_link_state;
    g_mode_task_debug.custom_control_state = (uint8_t)context->hierarchy_state.operational.custom_control_state;
}

static void mode_task_apply_init_progress_message(
    ModeTaskContext* context,
    const ModeTaskInitProgressMessage* message)
{
    if (context == OM_NULL || message == OM_NULL)
    {
        return;
    }

    switch ((ModeTaskInitProgressKind)message->kind)
    {
    case MODE_TASK_INIT_PROGRESS_CAN_READY:
        context->init_progress.can_ready = (message->value != 0u) ? OM_TRUE : OM_FALSE;
        break;
    case MODE_TASK_INIT_PROGRESS_SERIAL_READY:
        context->init_progress.serial_ready = (message->value != 0u) ? OM_TRUE : OM_FALSE;
        break;
    case MODE_TASK_INIT_PROGRESS_IMU_READY:
        context->init_progress.imu_ready = (message->value != 0u) ? OM_TRUE : OM_FALSE;
        break;
    case MODE_TASK_INIT_PROGRESS_CHASSIS_MOTOR_READY:
        context->init_progress.chassis_motor_ready = (message->value != 0u) ? OM_TRUE : OM_FALSE;
        break;
    case MODE_TASK_INIT_PROGRESS_ARM_MOTOR_READY:
        context->init_progress.arm_motor_ready = (message->value != 0u) ? OM_TRUE : OM_FALSE;
        break;
    default:
        break;
    }
}

static void mode_task_drain_init_progress_messages(ModeTaskContext* context)
{
    ModeTaskInitProgressMessage message = {0};

    if (context == OM_NULL)
    {
        return;
    }

    while (task_mpsc_channel_receive_nonblocking(
               &g_mode_task_init_progress_channel,
               &message) == OM_OK)
    {
        mode_task_apply_init_progress_message(context, &message);
    }
}

static OmBool mode_task_is_iw_up_edge(const ModeTaskContext* context, const ModeTaskRcSnapshot* snapshot)
{
    /* 拨轮从“大于阈值”跨到“小于等于阈值”视为一次上边沿事件。 */
    return (context->last_iw > RC_IW_UP_THRESHOLD && snapshot->iw <= RC_IW_UP_THRESHOLD) ? OM_TRUE : OM_FALSE;
}

static OmBool mode_task_is_iw_dn_edge(const ModeTaskContext* context, const ModeTaskRcSnapshot* snapshot)
{
    /* 拨轮从“小于阈值”跨到“大于等于阈值”视为一次下边沿事件。 */
    return (context->last_iw < RC_IW_DN_THRESHOLD && snapshot->iw >= RC_IW_DN_THRESHOLD) ? OM_TRUE : OM_FALSE;
}

static OmBool mode_task_is_sw1_to_dn_edge(const ModeTaskContext* context, const ModeTaskRcSnapshot* snapshot)
{
    return (context->last_sw1 == RC_SWITCH_MI && snapshot->sw1 == RC_SWITCH_DN) ? OM_TRUE : OM_FALSE;
}

static OmBool mode_task_is_sw1_to_up_edge(const ModeTaskContext* context, const ModeTaskRcSnapshot* snapshot)
{
    return (context->last_sw1 == RC_SWITCH_MI && snapshot->sw1 == RC_SWITCH_UP) ? OM_TRUE : OM_FALSE;
}

static GlobalMode mode_task_resolve_global_mode(
    const ModeTaskContext* context,
    const ModeTaskRcSnapshot* snapshot)
{
    if (context == OM_NULL || snapshot == OM_NULL)
    {
        return MODE_GLOBAL_RELEASE_CTRL;
    }

    /* 自定义控制器模式一旦进入，就锁在手动域内。
     * 动作总门控改成解锁锁存：
     * - DN 清除解锁
     * - MI -> UP 置位解锁
     * - 解锁后直到下一次 DN 前，都保持可动作
     */
    if (context->shared_state.chassis_mode == MODE_CHASSIS_CUSTOM_CONTROLLER_NORMAL)
    {
        return (context->hierarchy_state.operational.action_enabled == OM_TRUE) ?
                   MODE_GLOBAL_MANUAL_CTRL :
                   MODE_GLOBAL_RELEASE_CTRL;
    }

    /* sw2 语义：
     * - DN：退出 operational
     * - DN -> MI：只做 enable，不直接放行动作
     * - MI -> UP：解锁动作
     * - 解锁后直到下一次 DN 前，MI/UP 都允许进入动作域
     */
    if (context->hierarchy_state.operational.action_enabled != OM_TRUE)
    {
        return MODE_GLOBAL_RELEASE_CTRL;
    }

    switch (snapshot->sw2)
    {
    case RC_SWITCH_UP:
        return MODE_GLOBAL_MANUAL_CTRL;
    case RC_SWITCH_MI:
        return MODE_GLOBAL_ENGINEER_CTRL;
    case RC_SWITCH_DN:
    default:
        return MODE_GLOBAL_RELEASE_CTRL;
    }
}

static ChassisMode mode_task_resolve_manual_chassis_mode(
    const ModeTaskContext* context,
    const ModeTaskRcSnapshot* snapshot,
    OmBool custom_controller_online)
{
    ChassisMode current_mode = context->shared_state.chassis_mode;

    /* 自定义控制器模式是手动模式里的一个“独占子模式”：
     * - 进入后，普通遥控动作入口一律失效
     * - 只有专用退出手势（sw2=UP, sw1=MI, iw 下边沿）能退回 NORMAL
     */
    if (current_mode == MODE_CHASSIS_CUSTOM_CONTROLLER_NORMAL)
    {
        if (custom_controller_online != OM_TRUE)
        {
            return MODE_CHASSIS_NORMAL;
        }

        if (mode_task_is_iw_dn_edge(context, snapshot) == OM_TRUE &&
            snapshot->sw2 == RC_SWITCH_UP &&
            snapshot->sw1 == RC_SWITCH_MI)
        {
            return MODE_CHASSIS_NORMAL;
        }

        return current_mode;
    }

    /* 手动模式下依赖“当前 sw1 + iw 边沿”做模式切换。 */
    if (mode_task_is_iw_up_edge(context, snapshot) == OM_TRUE && snapshot->sw1 == RC_SWITCH_UP)
    {
        return MODE_CHASSIS_PITCH3_TORQUE_COLLECTION;
    }
    if (mode_task_is_iw_up_edge(context, snapshot) == OM_TRUE && snapshot->sw1 == RC_SWITCH_DN)
    {
        return MODE_CHASSIS_CHECK;
    }
    if (mode_task_is_iw_up_edge(context, snapshot) == OM_TRUE &&
        snapshot->sw2 == RC_SWITCH_UP &&
        snapshot->sw1 == RC_SWITCH_MI &&
        current_mode != MODE_CHASSIS_CUSTOM_CONTROLLER_NORMAL)
    {
        return MODE_CHASSIS_SECONDARY_ORE;
    }
    if (mode_task_is_iw_dn_edge(context, snapshot) == OM_TRUE &&
        snapshot->sw2 == RC_SWITCH_UP &&
        snapshot->sw1 == RC_SWITCH_MI &&
        custom_controller_online == OM_TRUE &&
        current_mode != MODE_CHASSIS_CUSTOM_CONTROLLER_NORMAL)
    {
        return MODE_CHASSIS_CUSTOM_CONTROLLER_NORMAL;
    }

    if (mode_task_is_iw_dn_edge(context, snapshot) == OM_TRUE &&
        (current_mode == MODE_CHASSIS_PITCH3_TORQUE_COLLECTION || current_mode == MODE_CHASSIS_URGENT_MEASURE ||
         current_mode == MODE_CHASSIS_SECONDARY_ORE))
    {
        return MODE_CHASSIS_NORMAL;
    }

    /* 不在几个保持型特殊模式里时，回落到普通底盘模式。 */
    if (current_mode != MODE_CHASSIS_PITCH3_TORQUE_COLLECTION && current_mode != MODE_CHASSIS_SECONDARY_ORE &&
        current_mode != MODE_CHASSIS_URGENT_MEASURE &&
        current_mode != MODE_CHASSIS_CUSTOM_CONTROLLER_NORMAL)
    {
        return MODE_CHASSIS_NORMAL;
    }

    return current_mode;
}

static ChassisMode mode_task_resolve_engineer_chassis_mode(const ModeTaskContext* context, const ModeTaskRcSnapshot* snapshot)
{
    const ChassisMode current_mode = context->shared_state.chassis_mode;

    /* 工程模式下，sw1 选择“哪一类动作模式”，iw 边沿选择具体入口。 */
    if (mode_task_is_iw_up_edge(context, snapshot) == OM_TRUE && snapshot->sw2 == RC_SWITCH_MI && snapshot->sw1 == RC_SWITCH_UP)
    {
        return MODE_CHASSIS_GET_ENERGY_UNIT;
    }
    if (mode_task_is_iw_dn_edge(context, snapshot) == OM_TRUE && snapshot->sw2 == RC_SWITCH_MI && snapshot->sw1 == RC_SWITCH_UP)
    {
        return MODE_CHASSIS_GET_ENERGY_UNIT1;
    }
    if (mode_task_is_iw_up_edge(context, snapshot) == OM_TRUE && snapshot->sw2 == RC_SWITCH_MI && snapshot->sw1 == RC_SWITCH_MI)
    {
        return MODE_CHASSIS_EXCHANGE;
    }
    if (mode_task_is_iw_dn_edge(context, snapshot) == OM_TRUE && snapshot->sw2 == RC_SWITCH_MI && snapshot->sw1 == RC_SWITCH_MI)
    {
        return MODE_CHASSIS_GET_ENERGY_UNIT2;
    }
    if (mode_task_is_iw_up_edge(context, snapshot) == OM_TRUE && snapshot->sw2 == RC_SWITCH_MI && snapshot->sw1 == RC_SWITCH_DN)
    {
        return MODE_CHASSIS_PRIMARY;
    }

    /* 工程动作模式需要保持，否则 arm_task 的时序姿态链只会收到一拍入口态，
     * 下一轮立即掉回 NORMAL，动作就看起来“完全没反应”。
     */
    switch (current_mode)
    {
    case MODE_CHASSIS_GET_ENERGY_UNIT:
    case MODE_CHASSIS_GET_ENERGY_UNIT1:
    case MODE_CHASSIS_GET_ENERGY_UNIT2:
    case MODE_CHASSIS_EXCHANGE:
    case MODE_CHASSIS_PRIMARY:
        return current_mode;
    default:
        return MODE_CHASSIS_NORMAL;
    }
}

static ChassisMode mode_task_resolve_chassis_mode(
    const ModeTaskContext* context,
    const ModeTaskRcSnapshot* snapshot,
    const DpCustomControllerSnapshot* custom_snapshot,
    GlobalMode global_mode)
{
    const OmBool custom_controller_online =
        (custom_snapshot != OM_NULL && custom_snapshot->online != 0u) ? OM_TRUE : OM_FALSE;

    switch (global_mode)
    {
    case MODE_GLOBAL_RELEASE_CTRL:
        return MODE_CHASSIS_RELEASE;
    case MODE_GLOBAL_MANUAL_CTRL:
        return mode_task_resolve_manual_chassis_mode(
            context,
            snapshot,
            custom_controller_online);
    case MODE_GLOBAL_ENGINEER_CTRL:
        return mode_task_resolve_engineer_chassis_mode(context, snapshot);
    default:
        return MODE_CHASSIS_RELEASE;
    }
}

static OmBool mode_task_is_clamp_related_mode(ChassisMode chassis_mode)
{
    switch (chassis_mode)
    {
    case MODE_CHASSIS_GET_ENERGY_UNIT:
    case MODE_CHASSIS_GET_ENERGY_UNIT1:
    case MODE_CHASSIS_GET_ENERGY_UNIT2:
    case MODE_CHASSIS_PRIMARY:
    case MODE_CHASSIS_SECONDARY_ORE:
        return OM_TRUE;
    default:
        return OM_FALSE;
    }
}

static void mode_task_update_clamp_action(ModeTaskContext* context, const ModeTaskRcSnapshot* snapshot, ModeTaskSharedState* state)
{
    /* 夹取动作只在取矿相关底盘模式中有效；
     * 一旦离开相关模式，动作状态直接收回到未命令态，避免陈旧动作残留。
     */
    if (mode_task_is_clamp_related_mode(state->chassis_mode) == OM_FALSE)
    {
        state->clamp_action = MODE_CLAMP_UN_CMD;
        context->clamp_ready_to_change = OM_FALSE;
        return;
    }

    /* 旧工程语义：sw1 回到中位后，才允许下一次动作切换。 */
    if (snapshot->sw1 == RC_SWITCH_MI)
    {
        context->clamp_ready_to_change = OM_TRUE;
    }

    if (context->clamp_ready_to_change != OM_TRUE)
    {
        return;
    }

    if (mode_task_is_sw1_to_dn_edge(context, snapshot) == OM_TRUE)
    {
        if (state->clamp_action < MODE_CLAMP_ACTION_TWO)
        {
            state->clamp_action = (ClampAction)((uint8_t)state->clamp_action + 1u);
        }
        context->clamp_ready_to_change = OM_FALSE;
    }
    else if (mode_task_is_sw1_to_up_edge(context, snapshot) == OM_TRUE)
    {
        if (state->clamp_action > MODE_CLAMP_UN_CMD)
        {
            state->clamp_action = (ClampAction)((uint8_t)state->clamp_action - 1u);
        }
        context->clamp_ready_to_change = OM_FALSE;
    }
}

static void mode_task_update_exchange_action(ModeTaskContext* context, const ModeTaskRcSnapshot* snapshot, ModeTaskSharedState* state)
{
    /* 兑换动作和夹取动作一样，都是“进入模式后由 sw1 中位解锁，再由边沿触发动作”。 */
    if (state->chassis_mode != MODE_CHASSIS_EXCHANGE)
    {
        state->exchange_action = MODE_EXCHANGE_UN_CMD;
        context->exchange_ready_to_change = OM_FALSE;
        return;
    }

    if (snapshot->sw1 == RC_SWITCH_MI)
    {
        context->exchange_ready_to_change = OM_TRUE;
        state->exchange_action = MODE_EXCHANGE_UN_CMD;
    }

    if (context->exchange_ready_to_change != OM_TRUE)
    {
        return;
    }

    if (mode_task_is_sw1_to_dn_edge(context, snapshot) == OM_TRUE)
    {
        state->exchange_action = MODE_EXCHANGE_PICK_ACTION1;
        context->exchange_ready_to_change = OM_FALSE;
    }
    else if (mode_task_is_sw1_to_up_edge(context, snapshot) == OM_TRUE)
    {
        state->exchange_action = MODE_EXCHANGE_PICK_ACTION2;
        context->exchange_ready_to_change = OM_FALSE;
    }
}

static void mode_task_update_primary_flag(ModeTaskContext* context, const ModeTaskRcSnapshot* snapshot, ModeTaskSharedState* state)
{
    /* 一矿模式下，拨轮下边沿触发一次“转矿”标志；
     * 离开该模式后立即清零，保证它是一个模式域内的共享事实。
     */
    if (state->chassis_mode == MODE_CHASSIS_PRIMARY)
    {
        if (mode_task_is_iw_dn_edge(context, snapshot) == OM_TRUE)
        {
            state->primary_turn_ore_flag = 1u;
        }
    }
    else
    {
        state->primary_turn_ore_flag = 0u;
    }
}

static void mode_task_update_custom_controller_force_takeover(
    ModeTaskContext* context,
    const ModeTaskRcSnapshot* snapshot,
    ModeTaskSharedState* state)
{
    if (context == OM_NULL || snapshot == OM_NULL || state == OM_NULL)
    {
        return;
    }

    if (state->chassis_mode != MODE_CHASSIS_CUSTOM_CONTROLLER_NORMAL)
    {
        state->custom_controller_force_takeover_flag = 0u;
        return;
    }

    if (snapshot->sw1 == RC_SWITCH_MI)
    {
        state->custom_controller_force_takeover_flag = 0u;
    }

    if (mode_task_is_sw1_to_dn_edge(context, snapshot) == OM_TRUE)
    {
        state->custom_controller_force_takeover_flag = 1u;
    }
}

static void mode_task_notify_custom_controller_transition(
    ChassisMode previous_mode,
    ChassisMode next_mode)
{
    OmBool previous_custom = (previous_mode == MODE_CHASSIS_CUSTOM_CONTROLLER_NORMAL) ? OM_TRUE : OM_FALSE;
    OmBool next_custom = (next_mode == MODE_CHASSIS_CUSTOM_CONTROLLER_NORMAL) ? OM_TRUE : OM_FALSE;

    if (previous_custom != OM_TRUE && next_custom == OM_TRUE)
    {
        sh_set_custom_controller_calibration_pending();
    }
    else if (previous_custom == OM_TRUE && next_custom != OM_TRUE)
    {
        sh_clear_custom_controller_calibration_indicator();
    }
}

static void mode_task_sync_context_history(ModeTaskContext* context, const ModeTaskRcSnapshot* snapshot)
{
    if (context == OM_NULL || snapshot == OM_NULL)
    {
        return;
    }

    /* sw1 额外保留两拍历史，是因为旧工程里部分动作逻辑需要区分更早一拍的中间态。 */
    if (context->last_sw1 != snapshot->sw1)
    {
        context->last_last_sw1 = context->last_sw1;
        context->last_sw1 = snapshot->sw1;
    }

    if (context->last_sw2 != snapshot->sw2)
    {
        context->last_sw2 = snapshot->sw2;
    }

    context->last_iw = snapshot->iw;
    context->last_global_mode = context->shared_state.global_mode;
    context->last_chassis_mode = context->shared_state.chassis_mode;
}

static void mode_task_run_once(ModeTaskContext* context)
{
    ModeTaskRcSnapshot rc_snapshot = {0};
    DpCustomControllerSnapshot custom_controller_snapshot = {0};
    ModeTaskSharedState next_state = {0};
    OmBool state_changed = OM_FALSE;
    ModeTaskControlSnapshot previous_control_snapshot = {0};
    ModeTaskControlSnapshot next_control_snapshot = {0};
    OmBool control_snapshot_changed = OM_FALSE;
    ChassisMode previous_chassis_mode = MODE_CHASSIS_RELEASE;

    /* 每轮先锁定一份输入快照，再用它推导整轮共享控制结果。 */
    mode_task_drain_rc_snapshots(context);
    mode_task_drain_custom_controller_snapshots(context);
    mode_task_load_rc_snapshot(&rc_snapshot);
    mode_task_load_custom_controller_snapshot(context, &custom_controller_snapshot);

    if (context->initialized != OM_TRUE)
    {
        context->last_sw1 = rc_snapshot.sw1;
        context->last_last_sw1 = rc_snapshot.sw1;
        context->last_sw2 = rc_snapshot.sw2;
        context->last_iw = rc_snapshot.iw;
        context->initialized = OM_TRUE;
    }

    mode_task_drain_init_progress_messages(context);
    mode_task_update_bootstrap_state_from_progress(context);
    mode_task_process_mct_lifecycle_requests(context, &rc_snapshot);
    mode_task_update_operational_system_state(
        context,
        &rc_snapshot,
        &custom_controller_snapshot);

    /* 先从上一轮共享结果出发，只覆盖本轮真正变化的字段。 */
    next_state = context->shared_state;
    previous_chassis_mode = context->shared_state.chassis_mode;
    mode_task_build_control_snapshot(
        context,
        &context->shared_state,
        &previous_control_snapshot);

    if (mode_task_bootstrap_allows_compat_control(context) != OM_TRUE ||
        context->hierarchy_state.system_state != MODE_TASK_SYSTEM_OPERATIONAL)
    {
        mode_task_fill_release_shared_state(&next_state);
    }
    else
    {
        next_state.global_mode = mode_task_resolve_global_mode(context, &rc_snapshot);
        next_state.chassis_mode = mode_task_resolve_chassis_mode(
            context,
            &rc_snapshot,
            &custom_controller_snapshot,
            next_state.global_mode);

        mode_task_update_clamp_action(context, &rc_snapshot, &next_state);
        mode_task_update_exchange_action(context, &rc_snapshot, &next_state);
        mode_task_update_primary_flag(context, &rc_snapshot, &next_state);
        mode_task_update_custom_controller_force_takeover(context, &rc_snapshot, &next_state);
    }

    mode_task_update_operational_domain(
        context,
        &rc_snapshot,
        &custom_controller_snapshot,
        &next_state);

    /* 当前实现里状态机主要承担“状态记录器”职责，
     * 真正的模式判定仍由显式条件函数决定。
     */
    if (sm_get_current(&context->global_machine) != (StateId)next_state.global_mode)
    {
        (void)sm_force_transition(&context->global_machine, (StateId)next_state.global_mode);
    }
    if (sm_get_current(&context->chassis_machine) != (StateId)next_state.chassis_mode)
    {
        (void)sm_force_transition(&context->chassis_machine, (StateId)next_state.chassis_mode);
    }

    /* 只有真正发生共享结果变化时才发 EVT_MODE_CHANGED，
     * 避免下游任务被无意义重复唤醒。
     */
    state_changed = mode_task_shared_state_changed(&context->shared_state, &next_state);
    mode_task_notify_custom_controller_transition(previous_chassis_mode, next_state.chassis_mode);
    context->shared_state = next_state;
    mode_task_build_control_snapshot(
        context,
        &context->shared_state,
        &next_control_snapshot);
    control_snapshot_changed = mode_task_control_snapshot_changed(
        &previous_control_snapshot,
        &next_control_snapshot);
    if (state_changed == OM_TRUE)
    {
        mode_task_store_shared_state(&context->shared_state);
    }
    if (control_snapshot_changed == OM_TRUE)
    {
        mode_task_publish_control_snapshot(&next_control_snapshot);
    }
    mode_task_sync_context_history(context, &rc_snapshot);
    mode_task_update_debug_state(context);

    if (state_changed == OM_TRUE)
    {
        if (event_bus_publish(&g_event_bus, EVT_MODE_CHANGED) != OSAL_OK)
        {
            sh_report_fatal(SH_ERR_EVT_MODE_CHANGED_PUBLISH_FAIL, "event_bus_publish EVT_MODE_CHANGED failed");
            for (;;)
            {
                osal_sleep_ms(1000U);
            }
        }
        g_mode_task_debug.publish_count++;
    }
}

static void mode_task_entry(void* arg)
{
    ModeTaskContext* context = (ModeTaskContext*)arg;
    OsalTimeMs deadline_cursor_ms = 0u;

    while (1)
    {
        g_mode_task_debug.loop_count++;
        mode_task_run_once(context);
        /* 6 ms 固定周期，对齐 build_steps 里的调度要求。 */
        (void)osal_delay_until(&deadline_cursor_ms, MODE_TASK_PERIOD_MS, OM_NULL);
    }
}

OmRet mode_task_start(void)
{
    static OsalThread* mode_task_thread = OM_NULL;
    static ModeTaskContext mode_task_context = {0};
    const OsalThreadAttr mode_task_attr = {"mode_task", 768u * OSAL_STACK_WORD_BYTES, 4u};
    OsalStatus status = OSAL_INVALID;
    OmRet ret = OM_OK;

    if (mode_task_thread != OM_NULL)
    {
        return OM_ERR_CONFLICT;
    }

    memset(&g_mode_task_debug, 0, sizeof(g_mode_task_debug));
    memset(&mode_task_context, 0, sizeof(mode_task_context));
    g_mode_task_owner_context = &mode_task_context;

    /* 启动时所有共享控制结果先落到安全默认态。 */
    mode_task_context.shared_state.global_mode = MODE_GLOBAL_RELEASE_CTRL;
    mode_task_context.shared_state.chassis_mode = MODE_CHASSIS_RELEASE;
    mode_task_context.shared_state.clamp_action = MODE_CLAMP_UN_CMD;
    mode_task_context.shared_state.exchange_action = MODE_EXCHANGE_UN_CMD;
    mode_task_context.shared_state.primary_turn_ore_flag = 0u;
    mode_task_context.shared_state.custom_controller_force_takeover_flag = 0u;
    mode_task_context.last_global_mode = MODE_GLOBAL_RELEASE_CTRL;
    mode_task_context.last_chassis_mode = MODE_CHASSIS_RELEASE;
    mode_task_context.hierarchy_state.system_state = MODE_TASK_SYSTEM_UNINITIALIZED;
    mode_task_board_init_context_reset(&mode_task_context.hierarchy_state.board_init);
    mode_task_motor_init_context_reset(&mode_task_context.hierarchy_state.motor_init);
    mode_task_operational_context_reset(&mode_task_context.hierarchy_state.operational);
    mode_task_init_progress_context_reset(&mode_task_context.init_progress);
    mode_task_context.rc_snapshot_ready = OM_FALSE;
    mode_task_update_debug_state(&mode_task_context);

    ret = task_mpsc_channel_init(
        &g_mode_task_init_progress_channel,
        g_mode_task_init_progress_storage,
        g_mode_task_init_progress_ready_flags,
        sizeof(ModeTaskInitProgressMessage),
        MODE_TASK_INIT_PROGRESS_CHANNEL_CAPACITY);
    if (ret != OM_OK)
    {
        g_mode_task_owner_context = OM_NULL;
        return ret;
    }

    ret = task_pipe_channel_init(
        &mode_task_context.rc_channel,
        g_mode_task_rc_channel_storage,
        MODE_TASK_RC_CHANNEL_CAPACITY_BYTES,
        sizeof(DpRcSnapshot));
    if (ret != OM_OK)
    {
        task_mpsc_channel_deinit(&g_mode_task_init_progress_channel);
        g_mode_task_owner_context = OM_NULL;
        return ret;
    }

    ret = task_pipe_channel_init(
        &mode_task_context.custom_controller_channel,
        g_mode_task_custom_controller_channel_storage,
        MODE_TASK_CUSTOM_CONTROLLER_CHANNEL_CAPACITY_BYTES,
        sizeof(DpCustomControllerSnapshot));
    if (ret != OM_OK)
    {
        task_pipe_channel_deinit(&mode_task_context.rc_channel);
        task_mpsc_channel_deinit(&g_mode_task_init_progress_channel);
        g_mode_task_owner_context = OM_NULL;
        return ret;
    }

    ret = sm_init(&mode_task_context.global_machine, g_mode_global_states, (uint8_t)(sizeof(g_mode_global_states) / sizeof(g_mode_global_states[0])),
                  OM_NULL, 0u, (StateId)MODE_GLOBAL_RELEASE_CTRL, &mode_task_context);
    if (ret != OM_OK)
    {
        task_pipe_channel_deinit(&mode_task_context.custom_controller_channel);
        task_pipe_channel_deinit(&mode_task_context.rc_channel);
        task_mpsc_channel_deinit(&g_mode_task_init_progress_channel);
        g_mode_task_owner_context = OM_NULL;
        return ret;
    }

    ret = sm_init(&mode_task_context.chassis_machine, g_mode_chassis_states, (uint8_t)(sizeof(g_mode_chassis_states) / sizeof(g_mode_chassis_states[0])),
                  OM_NULL, 0u, (StateId)MODE_CHASSIS_RELEASE, &mode_task_context);
    if (ret != OM_OK)
    {
        task_pipe_channel_deinit(&mode_task_context.custom_controller_channel);
        task_pipe_channel_deinit(&mode_task_context.rc_channel);
        task_mpsc_channel_deinit(&g_mode_task_init_progress_channel);
        g_mode_task_owner_context = OM_NULL;
        return ret;
    }

    /* 在任务真正启动前先把默认模式写进共享池，
     * 这样下游即使比 mode_task 更早运行，也不会读到未初始化模式值。
     */
    mode_task_store_shared_state(&mode_task_context.shared_state);

    status = osal_thread_create(&mode_task_thread, &mode_task_attr, mode_task_entry, &mode_task_context);
    if (status != OSAL_OK)
    {
        task_pipe_channel_deinit(&mode_task_context.custom_controller_channel);
        task_pipe_channel_deinit(&mode_task_context.rc_channel);
        task_mpsc_channel_deinit(&g_mode_task_init_progress_channel);
        mode_task_thread = OM_NULL;
        g_mode_task_owner_context = OM_NULL;
        return OM_ERROR;
    }

    return OM_OK;
}

OmRet mode_task_submit_init_progress(
    const ModeTaskInitProgressMessage* message)
{
    if (message == OM_NULL || g_mode_task_init_progress_channel.read_sem == OM_NULL)
    {
        return OM_ERROR;
    }

    return task_mpsc_channel_submit_nonblocking(
        &g_mode_task_init_progress_channel,
        message);
}

OmRet mode_task_submit_rc_snapshot(
    const DpRcSnapshot* snapshot)
{
    if (snapshot == OM_NULL || g_mode_task_owner_context == OM_NULL)
    {
        return OM_ERROR_PARAM;
    }

    return task_pipe_channel_submit_nonblocking(
        &g_mode_task_owner_context->rc_channel,
        snapshot,
        OM_TRUE);
}

OmRet mode_task_submit_custom_controller_snapshot(
    const DpCustomControllerSnapshot* snapshot)
{
    if (snapshot == OM_NULL || g_mode_task_owner_context == OM_NULL)
    {
        return OM_ERROR_PARAM;
    }

    return task_pipe_channel_submit_nonblocking(
        &g_mode_task_owner_context->custom_controller_channel,
        snapshot,
        OM_TRUE);
}

OmBool mode_task_copy_control_snapshot(
    ModeTaskControlSnapshot* snapshot)
{
    if (snapshot == OM_NULL || g_mode_task_owner_context == OM_NULL)
    {
        return OM_FALSE;
    }

    taskENTER_CRITICAL();
    mode_task_build_control_snapshot(
        g_mode_task_owner_context,
        &g_mode_task_owner_context->shared_state,
        snapshot);
    taskEXIT_CRITICAL();
    return OM_TRUE;
}
