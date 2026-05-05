#include "task/mode_task/mode_task.h"

#include "core/om_cpu.h"
#include "module/data_pool/data_pool.h"
#include "module/event_bus/event_bus.h"
#include "module/state_machine/state_machine.h"
#include "module/system_health/system_health.h"
#include "osal/osal.h"
#include "osal/osal_config.h"
#include "osal/osal_time.h"
#include <string.h>

#define MODE_TASK_PERIOD_MS     (6u)
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
} ModeTaskSharedState;

/* mode_task 的本地上下文：
 * - last_* 保存边沿历史，完全留在任务内部
 * - shared_state 保存当前导出的控制结果
 * - 两个状态机目前作为“当前模式记录器”存在，为后续扩展留接口
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
    StateMachine global_machine;
    StateMachine chassis_machine;
} ModeTaskContext;

ModeTaskDebugState g_mode_task_debug = {0};

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
 *    - 当特殊模式收到 iw 下边沿时回到 NORMAL
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

    snapshot->ch1 = DP_LOAD_INT16(&g_data_pool.rc.ch1);
    snapshot->ch2 = DP_LOAD_INT16(&g_data_pool.rc.ch2);
    snapshot->ch3 = DP_LOAD_INT16(&g_data_pool.rc.ch3);
    snapshot->ch4 = DP_LOAD_INT16(&g_data_pool.rc.ch4);
    snapshot->sw1 = DP_LOAD_UINT8(&g_data_pool.rc.sw1);
    snapshot->sw2 = DP_LOAD_UINT8(&g_data_pool.rc.sw2);
    snapshot->iw = DP_LOAD_UINT16(&g_data_pool.rc.iw);
}

static void mode_task_store_shared_state(const ModeTaskSharedState* state)
{
    if (state == OM_NULL)
    {
        return;
    }

    /* mode/action 是跨任务共享的最终结果，
     * 写回统一收敛在这个函数里，避免多个位置维护同一份事实。
     */
    DP_STORE_UINT8(&g_data_pool.mode.global_mode, (uint8_t)state->global_mode);
    DP_STORE_UINT8(&g_data_pool.mode.chassis_mode, (uint8_t)state->chassis_mode);
    DP_STORE_UINT8(&g_data_pool.action.clamp_action, (uint8_t)state->clamp_action);
    DP_STORE_UINT8(&g_data_pool.action.exchange_action, (uint8_t)state->exchange_action);
    DP_STORE_UINT8(&g_data_pool.action.primary_turn_ore_flag, state->primary_turn_ore_flag);
}

static OmBool mode_task_shared_state_changed(const ModeTaskSharedState* lhs, const ModeTaskSharedState* rhs)
{
    if (lhs == OM_NULL || rhs == OM_NULL)
    {
        return OM_FALSE;
    }

    return (lhs->global_mode != rhs->global_mode || lhs->chassis_mode != rhs->chassis_mode || lhs->clamp_action != rhs->clamp_action ||
            lhs->exchange_action != rhs->exchange_action || lhs->primary_turn_ore_flag != rhs->primary_turn_ore_flag)
               ? OM_TRUE
               : OM_FALSE;
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

static GlobalMode mode_task_resolve_global_mode(const ModeTaskRcSnapshot* snapshot)
{
    /* 旧工程里 sw2 直接决定全局模式，这里保持同样的主开关语义。 */
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

static ChassisMode mode_task_resolve_manual_chassis_mode(const ModeTaskContext* context, const ModeTaskRcSnapshot* snapshot)
{
    ChassisMode current_mode = context->shared_state.chassis_mode;

    /* 手动模式下依赖“当前 sw1 + iw 边沿”做模式切换。 */
    if (mode_task_is_iw_up_edge(context, snapshot) == OM_TRUE && snapshot->sw1 == RC_SWITCH_UP)
    {
        return MODE_CHASSIS_PITCH3_TORQUE_COLLECTION;
    }
    if (mode_task_is_iw_up_edge(context, snapshot) == OM_TRUE && snapshot->sw1 == RC_SWITCH_DN)
    {
        return MODE_CHASSIS_CHECK;
    }
    if (mode_task_is_iw_up_edge(context, snapshot) == OM_TRUE && snapshot->sw2 == RC_SWITCH_UP && snapshot->sw1 == RC_SWITCH_MI)
    {
        return MODE_CHASSIS_SECONDARY_ORE;
    }

    if (mode_task_is_iw_dn_edge(context, snapshot) == OM_TRUE &&
        (current_mode == MODE_CHASSIS_PITCH3_TORQUE_COLLECTION || current_mode == MODE_CHASSIS_URGENT_MEASURE ||
         current_mode == MODE_CHASSIS_SECONDARY_ORE))
    {
        return MODE_CHASSIS_NORMAL;
    }

    /* 不在几个保持型特殊模式里时，回落到普通底盘模式。 */
    if (current_mode != MODE_CHASSIS_PITCH3_TORQUE_COLLECTION && current_mode != MODE_CHASSIS_SECONDARY_ORE &&
        current_mode != MODE_CHASSIS_URGENT_MEASURE)
    {
        return MODE_CHASSIS_NORMAL;
    }

    return current_mode;
}

static ChassisMode mode_task_resolve_engineer_chassis_mode(const ModeTaskContext* context, const ModeTaskRcSnapshot* snapshot)
{
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

    return context->shared_state.chassis_mode;
}

static ChassisMode mode_task_resolve_chassis_mode(const ModeTaskContext* context, const ModeTaskRcSnapshot* snapshot, GlobalMode global_mode)
{
    switch (global_mode)
    {
    case MODE_GLOBAL_RELEASE_CTRL:
        return MODE_CHASSIS_RELEASE;
    case MODE_GLOBAL_MANUAL_CTRL:
        return mode_task_resolve_manual_chassis_mode(context, snapshot);
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
    ModeTaskSharedState next_state = {0};
    OmBool state_changed = OM_FALSE;

    /* 每轮先锁定一份输入快照，再用它推导整轮共享控制结果。 */
    mode_task_load_rc_snapshot(&rc_snapshot);

    if (context->initialized != OM_TRUE)
    {
        context->last_sw1 = rc_snapshot.sw1;
        context->last_last_sw1 = rc_snapshot.sw1;
        context->last_sw2 = rc_snapshot.sw2;
        context->last_iw = rc_snapshot.iw;
        context->initialized = OM_TRUE;
    }

    /* 先从上一轮共享结果出发，只覆盖本轮真正变化的字段。 */
    next_state = context->shared_state;
    next_state.global_mode = mode_task_resolve_global_mode(&rc_snapshot);
    next_state.chassis_mode = mode_task_resolve_chassis_mode(context, &rc_snapshot, next_state.global_mode);

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

    mode_task_update_clamp_action(context, &rc_snapshot, &next_state);
    mode_task_update_exchange_action(context, &rc_snapshot, &next_state);
    mode_task_update_primary_flag(context, &rc_snapshot, &next_state);

    /* 只有真正发生共享结果变化时才发 EVT_MODE_CHANGED，
     * 避免下游任务被无意义重复唤醒。
     */
    state_changed = mode_task_shared_state_changed(&context->shared_state, &next_state);
    context->shared_state = next_state;
    mode_task_store_shared_state(&context->shared_state);
    mode_task_sync_context_history(context, &rc_snapshot);

    if (state_changed == OM_TRUE)
    {
        if (event_bus_publish(&g_event_bus, EVT_MODE_CHANGED) != OSAL_OK)
        {
            system_health_report_fatal(SYSTEM_HEALTH_ERR_EVT_MODE_CHANGED_PUBLISH_FAIL, "event_bus_publish EVT_MODE_CHANGED failed");
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

    /* 启动时所有共享控制结果先落到安全默认态。 */
    mode_task_context.shared_state.global_mode = MODE_GLOBAL_RELEASE_CTRL;
    mode_task_context.shared_state.chassis_mode = MODE_CHASSIS_RELEASE;
    mode_task_context.shared_state.clamp_action = MODE_CLAMP_UN_CMD;
    mode_task_context.shared_state.exchange_action = MODE_EXCHANGE_UN_CMD;
    mode_task_context.shared_state.primary_turn_ore_flag = 0u;
    mode_task_context.last_global_mode = MODE_GLOBAL_RELEASE_CTRL;
    mode_task_context.last_chassis_mode = MODE_CHASSIS_RELEASE;

    ret = sm_init(&mode_task_context.global_machine, g_mode_global_states, (uint8_t)(sizeof(g_mode_global_states) / sizeof(g_mode_global_states[0])),
                  OM_NULL, 0u, (StateId)MODE_GLOBAL_RELEASE_CTRL, &mode_task_context);
    if (ret != OM_OK)
    {
        return ret;
    }

    ret = sm_init(&mode_task_context.chassis_machine, g_mode_chassis_states, (uint8_t)(sizeof(g_mode_chassis_states) / sizeof(g_mode_chassis_states[0])),
                  OM_NULL, 0u, (StateId)MODE_CHASSIS_RELEASE, &mode_task_context);
    if (ret != OM_OK)
    {
        return ret;
    }

    /* 在任务真正启动前先把默认模式写进共享池，
     * 这样下游即使比 mode_task 更早运行，也不会读到未初始化模式值。
     */
    mode_task_store_shared_state(&mode_task_context.shared_state);

    status = osal_thread_create(&mode_task_thread, &mode_task_attr, mode_task_entry, &mode_task_context);
    if (status != OSAL_OK)
    {
        mode_task_thread = OM_NULL;
        return OM_ERROR;
    }

    return OM_OK;
}
