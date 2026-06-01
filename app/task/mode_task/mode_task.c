#include "task/mode_task/mode_task_internal.h"

#include "FreeRTOS.h"
#include "osal/osal.h"
#include "osal/osal_config.h"
#include "task.h"
#include <string.h>

TaskMpscChannel g_mode_task_init_progress_channel = {0};
static uint8_t g_mode_task_init_progress_storage
    [sizeof(ModeTaskInitProgressMessage) * MODE_TASK_INIT_PROGRESS_CHANNEL_CAPACITY] = {0};
static OmAtomicU8 g_mode_task_init_progress_ready_flags[MODE_TASK_INIT_PROGRESS_CHANNEL_CAPACITY] = {0};
static uint8_t g_mode_task_rc_channel_storage[MODE_TASK_RC_CHANNEL_CAPACITY_BYTES] = {0};
static uint8_t g_mode_task_custom_controller_channel_storage
    [MODE_TASK_CUSTOM_CONTROLLER_CHANNEL_CAPACITY_BYTES] = {0};

ModeTaskDebugState g_mode_task_debug = {0};
TaskContextSlotId g_mode_task_slot_id = 0;

/* 当前全局/底盘状态机先只承担状态记录职责，
 * enter/execute/exit 动作仍保持为空，避免在这轮拆分里扩大行为面。
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

static void mode_task_entry(void* arg)
{
    ModeTaskContext* context = (ModeTaskContext*)arg;
    OsalTimeMs deadline_cursor_ms = 0u;

    while (1)
    {
        g_mode_task_debug.loop_count++;
        mode_task_run_once(context);
        (void)osal_delay_until(&deadline_cursor_ms, MODE_TASK_PERIOD_MS, OM_NULL);
    }
}

static const TaskContextVTable g_mode_task_vtable = {
    .task_name = "mode_task",
    .init = mode_task_ctx_init,
    .reset = mode_task_ctx_reset,
    .cleanup = mode_task_ctx_cleanup,
    .diag_online = OM_NULL,
    .diag_snapshot = OM_NULL,
};

OmRet mode_task_start(void)
{
    static OsalThread* mode_task_thread = OM_NULL;
    const OsalThreadAttr mode_task_attr = {"mode_task", 768u * OSAL_STACK_WORD_BYTES, 4u};
    OsalStatus status = OSAL_INVALID;
    OmRet ret = OM_OK;
    ModeTaskContext* ctx = OM_NULL;

    if (mode_task_thread != OM_NULL)
    {
        return OM_ERR_CONFLICT;
    }

    memset(&g_mode_task_debug, 0, sizeof(g_mode_task_debug));

    g_mode_task_slot_id = task_context_pool_alloc(
        "mode_task",
        sizeof(ModeTaskContext),
        &g_mode_task_vtable);
    if (g_mode_task_slot_id == 0u)
    {
        return OM_ERROR;
    }

    ctx = mode_task_get_owner_context();

    ctx->shared_state.global_mode = MODE_GLOBAL_RELEASE_CTRL;
    ctx->shared_state.chassis_mode = MODE_CHASSIS_RELEASE;
    ctx->shared_state.clamp_action = MODE_CLAMP_UN_CMD;
    ctx->shared_state.exchange_action = MODE_EXCHANGE_UN_CMD;
    ctx->shared_state.primary_turn_ore_flag = 0u;
    ctx->shared_state.custom_controller_force_takeover_flag = 0u;
    ctx->last_global_mode = MODE_GLOBAL_RELEASE_CTRL;
    ctx->last_chassis_mode = MODE_CHASSIS_RELEASE;
    ctx->hierarchy_state.system_state = MODE_TASK_SYSTEM_UNINITIALIZED;
    mode_task_board_init_context_reset(&ctx->hierarchy_state.board_init);
    mode_task_motor_init_context_reset(&ctx->hierarchy_state.motor_init);
    mode_task_operational_context_reset(&ctx->hierarchy_state.operational);
    ctx->flags = 0u;
    mode_task_update_debug_state(ctx);

    ret = task_mpsc_channel_init(
        &g_mode_task_init_progress_channel,
        g_mode_task_init_progress_storage,
        g_mode_task_init_progress_ready_flags,
        sizeof(ModeTaskInitProgressMessage),
        MODE_TASK_INIT_PROGRESS_CHANNEL_CAPACITY);
    if (ret != OM_OK)
    {
        task_context_pool_free(g_mode_task_slot_id);
        g_mode_task_slot_id = 0u;
        return ret;
    }

    ret = task_pipe_channel_init(
        &ctx->rc_channel,
        g_mode_task_rc_channel_storage,
        MODE_TASK_RC_CHANNEL_CAPACITY_BYTES,
        sizeof(DpRcSnapshot));
    if (ret != OM_OK)
    {
        task_mpsc_channel_deinit(&g_mode_task_init_progress_channel);
        task_context_pool_free(g_mode_task_slot_id);
        g_mode_task_slot_id = 0u;
        return ret;
    }

    ret = task_pipe_channel_init(
        &ctx->custom_controller_channel,
        g_mode_task_custom_controller_channel_storage,
        MODE_TASK_CUSTOM_CONTROLLER_CHANNEL_CAPACITY_BYTES,
        sizeof(DpCustomControllerSnapshot));
    if (ret != OM_OK)
    {
        task_pipe_channel_deinit(&ctx->rc_channel);
        task_mpsc_channel_deinit(&g_mode_task_init_progress_channel);
        task_context_pool_free(g_mode_task_slot_id);
        g_mode_task_slot_id = 0u;
        return ret;
    }

    ret = sm_init(
        &ctx->global_machine,
        g_mode_global_states,
        (uint8_t)(sizeof(g_mode_global_states) / sizeof(g_mode_global_states[0])),
        OM_NULL,
        0u,
        (StateId)MODE_GLOBAL_RELEASE_CTRL,
        ctx);
    if (ret != OM_OK)
    {
        task_pipe_channel_deinit(&ctx->custom_controller_channel);
        task_pipe_channel_deinit(&ctx->rc_channel);
        task_mpsc_channel_deinit(&g_mode_task_init_progress_channel);
        task_context_pool_free(g_mode_task_slot_id);
        g_mode_task_slot_id = 0u;
        return ret;
    }

    ret = sm_init(
        &ctx->chassis_machine,
        g_mode_chassis_states,
        (uint8_t)(sizeof(g_mode_chassis_states) / sizeof(g_mode_chassis_states[0])),
        OM_NULL,
        0u,
        (StateId)MODE_CHASSIS_RELEASE,
        ctx);
    if (ret != OM_OK)
    {
        task_pipe_channel_deinit(&ctx->custom_controller_channel);
        task_pipe_channel_deinit(&ctx->rc_channel);
        task_mpsc_channel_deinit(&g_mode_task_init_progress_channel);
        task_context_pool_free(g_mode_task_slot_id);
        g_mode_task_slot_id = 0u;
        return ret;
    }

    mode_task_store_shared_state(&ctx->shared_state);

    status = osal_thread_create(&mode_task_thread, &mode_task_attr, mode_task_entry, ctx);
    if (status != OSAL_OK)
    {
        task_pipe_channel_deinit(&ctx->custom_controller_channel);
        task_pipe_channel_deinit(&ctx->rc_channel);
        task_mpsc_channel_deinit(&g_mode_task_init_progress_channel);
        mode_task_thread = OM_NULL;
        task_context_pool_free(g_mode_task_slot_id);
        g_mode_task_slot_id = 0u;
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
