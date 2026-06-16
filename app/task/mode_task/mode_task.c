#include "task/mode_task/mode_task_internal.h"

#include "FreeRTOS.h"
#include "osal/osal.h"
#include "osal/osal_config.h"
#include "task.h"
#include <string.h>

TaskMpscChannel g_mode_task_init_progress_channel = {0};
static uint8_t g_mode_task_init_progress_storage
    [sizeof(ModeTaskInitMessage) * MT_INIT_MSG_CAP] = {0};
static OmAtomicU8 g_mode_task_init_progress_ready_flags[MT_INIT_MSG_CAP] = {0};
static uint8_t g_mode_task_rc_channel_storage[MT_RC_CHANNEL_BYTES] = {0};
static uint8_t g_mode_task_custom_ch_storage
    [MT_CUSTOM_CH_BYTES] = {0};

ModeTaskDebugState g_mode_task_debug = {0};
TaskContextSlotId g_mode_task_slot_id = 0;

static const State g_mode_operational_phase_states[] = {
    {.id = (StateId)MT_OPERATIONAL_PHASE_RELEASE,
     .on_enter = mode_task_enter_release,
     .on_execute = OM_NULL,
     .on_exit = OM_NULL,
     .name = "release"},
    {.id = (StateId)MT_OPERATIONAL_PHASE_SELECT,
     .on_enter = mode_task_enter_select,
     .on_execute = OM_NULL,
     .on_exit = OM_NULL,
     .name = "mode_selection"},
    {.id = (StateId)MT_OPERATIONAL_PHASE_FORMAL,
     .on_enter = mode_task_enter_formal,
     .on_execute = OM_NULL,
     .on_exit = OM_NULL,
     .name = "formal_control"},
};

static const State g_mode_motion_mode_states[] = {
    {.id = (StateId)MT_MOTION_MODE_PRESET_ACTION,
     .on_enter = mode_task_enter_preset_action,
     .on_execute = OM_NULL,
     .on_exit = mode_task_exit_preset_action,
     .name = "preset_action"},
    {.id = (StateId)MT_MOTION_MODE_CUSTOM_TAKEOVER,
     .on_enter = mode_task_enter_custom,
     .on_execute = OM_NULL,
     .on_exit = mode_task_exit_custom,
     .name = "custom_takeover"},
    {.id = (StateId)MT_MOTION_MODE_RC_IK,
     .on_enter = mode_task_enter_rc_ik,
     .on_execute = OM_NULL,
     .on_exit = mode_task_exit_rc_ik,
     .name = "rc_ik"},
};

static void mode_task_entry(void* arg)
{
    ModeTaskContext* context = (ModeTaskContext*)arg;
    OsalTimeMs deadline_cursor_ms = 0u;

    while (1)
    {
        g_mode_task_debug.loop_count++;
        mode_task_run_once(context);
        (void)osal_delay_until(&deadline_cursor_ms, MT_PERIOD_MS, OM_NULL);
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
    ctx->confirmed_motion_mode_id = MT_MOTION_MODE_PRESET_ACTION;
    mode_task_reset_select(ctx);
    mode_task_reset_preset(ctx);
    mode_task_reset_rc_ik_runtime(ctx);
    mode_task_reset_grip_runtime(ctx);
    ctx->hierarchy_state.system_state = MT_SYSTEM_UNINITIALIZED;
    mode_task_reset_board_init(&ctx->hierarchy_state.board_init);
    mode_task_reset_motor_init(&ctx->hierarchy_state.motor_init);
    mode_task_reset_phase_context(&ctx->hierarchy_state.operational);

    ret = task_mpsc_channel_init(
        &g_mode_task_init_progress_channel,
        g_mode_task_init_progress_storage,
        g_mode_task_init_progress_ready_flags,
        sizeof(ModeTaskInitMessage),
        MT_INIT_MSG_CAP);
    if (ret != OM_OK)
    {
        task_context_pool_free(g_mode_task_slot_id);
        g_mode_task_slot_id = 0u;
        return ret;
    }

    ret = task_pipe_channel_init(
        &ctx->rc_channel,
        g_mode_task_rc_channel_storage,
        MT_RC_CHANNEL_BYTES,
        sizeof(InputRcSnapshot));
    if (ret != OM_OK)
    {
        task_mpsc_channel_deinit(&g_mode_task_init_progress_channel);
        task_context_pool_free(g_mode_task_slot_id);
        g_mode_task_slot_id = 0u;
        return ret;
    }

    ret = task_pipe_channel_init(
        &ctx->custom_channel,
        g_mode_task_custom_ch_storage,
        MT_CUSTOM_CH_BYTES,
        sizeof(InputCustomSnapshot));
    if (ret != OM_OK)
    {
        task_pipe_channel_deinit(&ctx->rc_channel);
        task_mpsc_channel_deinit(&g_mode_task_init_progress_channel);
        task_context_pool_free(g_mode_task_slot_id);
        g_mode_task_slot_id = 0u;
        return ret;
    }

    ret = sm_init(
        &ctx->operational_phase_machine,
        g_mode_operational_phase_states,
        (uint8_t)(sizeof(g_mode_operational_phase_states) / sizeof(g_mode_operational_phase_states[0])),
        OM_NULL,
        0u,
        (StateId)MT_OPERATIONAL_PHASE_RELEASE,
        ctx);
    if (ret != OM_OK)
    {
        task_pipe_channel_deinit(&ctx->custom_channel);
        task_pipe_channel_deinit(&ctx->rc_channel);
        task_mpsc_channel_deinit(&g_mode_task_init_progress_channel);
        task_context_pool_free(g_mode_task_slot_id);
        g_mode_task_slot_id = 0u;
        return ret;
    }

    ret = sm_init(
        &ctx->motion_mode_machine,
        g_mode_motion_mode_states,
        (uint8_t)(sizeof(g_mode_motion_mode_states) / sizeof(g_mode_motion_mode_states[0])),
        OM_NULL,
        0u,
        (StateId)MT_MOTION_MODE_PRESET_ACTION,
        ctx);
    if (ret != OM_OK)
    {
        task_pipe_channel_deinit(&ctx->custom_channel);
        task_pipe_channel_deinit(&ctx->rc_channel);
        task_mpsc_channel_deinit(&g_mode_task_init_progress_channel);
        task_context_pool_free(g_mode_task_slot_id);
        g_mode_task_slot_id = 0u;
        return ret;
    }

    mode_task_refresh_snapshots(ctx);
    mode_task_update_debug_state(ctx);

    status = osal_thread_create(&mode_task_thread, &mode_task_attr, mode_task_entry, ctx);
    if (status != OSAL_OK)
    {
        task_pipe_channel_deinit(&ctx->custom_channel);
        task_pipe_channel_deinit(&ctx->rc_channel);
        task_mpsc_channel_deinit(&g_mode_task_init_progress_channel);
        mode_task_thread = OM_NULL;
        task_context_pool_free(g_mode_task_slot_id);
        g_mode_task_slot_id = 0u;
        return OM_ERROR;
    }

    return OM_OK;
}

OmRet mode_task_submit_init(
    const ModeTaskInitMessage* message)
{
    if (message == OM_NULL || g_mode_task_init_progress_channel.read_sem == OM_NULL)
    {
        return OM_ERROR;
    }

    return tmpsc_submit(
        &g_mode_task_init_progress_channel,
        message);
}

OmRet mode_task_submit_rc_snapshot(
    const InputRcSnapshot* snapshot)
{
    if (snapshot == OM_NULL || g_mode_task_owner_context == OM_NULL)
    {
        return OM_ERROR_PARAM;
    }

    return tpipe_submit(
        &g_mode_task_owner_context->rc_channel,
        snapshot,
        OM_TRUE);
}

OmRet mode_task_submit_custom(
    const InputCustomSnapshot* snapshot)
{
    if (snapshot == OM_NULL || g_mode_task_owner_context == OM_NULL)
    {
        return OM_ERROR_PARAM;
    }

    return tpipe_submit(
        &g_mode_task_owner_context->custom_channel,
        snapshot,
        OM_TRUE);
}

OmBool mode_task_copy_system(
    ModeSystemSnap* snapshot)
{
    if (snapshot == OM_NULL || g_mode_task_owner_context == OM_NULL)
    {
        return OM_FALSE;
    }

    taskENTER_CRITICAL();
    mode_task_build_system(g_mode_task_owner_context, snapshot);
    taskEXIT_CRITICAL();
    return OM_TRUE;
}

OmBool mode_task_copy_arm_mode(
    ArmTaskModeSnapshot* snapshot)
{
    if (snapshot == OM_NULL || g_mode_task_owner_context == OM_NULL)
    {
        return OM_FALSE;
    }

    taskENTER_CRITICAL();
    mode_task_build_arm_mode(g_mode_task_owner_context, snapshot);
    taskEXIT_CRITICAL();
    return OM_TRUE;
}

OmBool mode_task_copy_chassis_mode(
    ChassisModeSnap* snapshot)
{
    if (snapshot == OM_NULL || g_mode_task_owner_context == OM_NULL)
    {
        return OM_FALSE;
    }

    taskENTER_CRITICAL();
    mode_task_build_chassis(g_mode_task_owner_context, snapshot);
    taskEXIT_CRITICAL();
    return OM_TRUE;
}
