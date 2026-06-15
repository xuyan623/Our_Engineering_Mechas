#include "task/arm_task/arm_task.h"

#include "module/system_health/system_health.h"
#include "osal/osal.h"
#include "osal/osal_config.h"
#include "osal/osal_time.h"
#include "task/arm_task/arm_task_internal.h"
#include "task/mode_task/mode_task.h"
#include <string.h>

TaskContextSlotId g_arm_task_slot_id = 0;
uint8_t g_arm_task_mode_channel_storage[sizeof(ArmTaskModeSnapshot) * ARM_TASK_MODE_CHANNEL_CAPACITY] = {0};
OmAtomicU8 g_arm_task_mode_channel_ready_flags[ARM_TASK_MODE_CHANNEL_CAPACITY] = {0};
uint8_t g_arm_task_rc_channel_storage[ARM_TASK_RC_CHANNEL_CAPACITY_BYTES] = {0};
uint8_t g_arm_task_custom_controller_channel_storage[ARM_TASK_CUSTOM_CONTROLLER_CHANNEL_CAPACITY_BYTES] = {0};

static void arm_task_entry(void* arg)
{
    ArmTaskContext* context = (ArmTaskContext*)arg;
    OsalTimeMs deadline_cursor_ms = 0u;

    if (context == OM_NULL)
    {
        for (;;)
        {
            (void)osal_sleep_ms(1000u);
        }
    }

    while (1)
    {
        arm_task_run_once(context);
        (void)sh_beat(SH_TASK_ARM);
        (void)osal_delay_until(&deadline_cursor_ms, ARM_TASK_PERIOD_MS, OM_NULL);
    }
}

static void arm_task_ctx_init(void* ctx)
{
    ArmTaskContext* self = (ArmTaskContext*)ctx;
    memset(self, 0, sizeof(ArmTaskContext));
    self->latest_custom_controller_snapshot.online = 0u;
    self->latest_custom_controller_snapshot.work_mode = 0u;
}

static void arm_task_ctx_reset(void* ctx)
{
    ArmTaskContext* self = (ArmTaskContext*)ctx;
    memset(&self->last_snapshot, 0, sizeof(self->last_snapshot));
    memset(&self->smoothed_targets, 0, sizeof(self->smoothed_targets));
    memset(&self->latest_mode_snapshot, 0, sizeof(self->latest_mode_snapshot));
    memset(&self->latest_rc_snapshot, 0, sizeof(self->latest_rc_snapshot));
    memset(&self->latest_custom_controller_snapshot, 0, sizeof(self->latest_custom_controller_snapshot));
    memset(&self->ik_target_pose, 0, sizeof(self->ik_target_pose));
    self->command_since_ms = 0u;
    self->last_tx_request_ms = 0u;
    memset(self->last_control_ms, 0, sizeof(self->last_control_ms));
    self->flags = 0u;
    self->custom_controller_alignment_started_ms = 0u;
}

static void arm_task_ctx_cleanup(void* ctx)
{
    (void)ctx;
}

static const TaskContextVTable g_arm_task_vtable = {
    .task_name = "arm_task",
    .init = arm_task_ctx_init,
    .reset = arm_task_ctx_reset,
    .cleanup = arm_task_ctx_cleanup,
    .diag_online = arm_task_diag_online,
    .diag_snapshot = arm_task_diag_snapshot,
};

OmRet arm_task_start(void)
{
    static OsalThread* arm_task_thread = OM_NULL;
    const OsalThreadAttr arm_task_attr = {
        "arm_task",
        ARM_TASK_STACK_BYTES,
        ARM_TASK_PRIORITY};
    OsalStatus status = OSAL_INVALID;
    OmRet ret = OM_OK;
    ArmTaskContext* ctx = OM_NULL;

    if (arm_task_thread != OM_NULL)
    {
        return OM_ERR_CONFLICT;
    }

    g_arm_task_slot_id = task_context_pool_alloc("arm_task", sizeof(ArmTaskContext), &g_arm_task_vtable);
    if (g_arm_task_slot_id == 0u)
    {
        return OM_ERROR;
    }

    ctx = (ArmTaskContext*)task_context_pool_get_ptr(g_arm_task_slot_id);

    ret = task_mpsc_channel_init(
        &ctx->mode_channel,
        g_arm_task_mode_channel_storage,
        g_arm_task_mode_channel_ready_flags,
        sizeof(ArmTaskModeSnapshot),
        ARM_TASK_MODE_CHANNEL_CAPACITY);
    if (ret != OM_OK)
    {
        task_context_pool_free(g_arm_task_slot_id);
        g_arm_task_slot_id = 0u;
        return ret;
    }

    if (mode_task_copy_arm_mode_snapshot(&ctx->latest_mode_snapshot) == OM_TRUE)
    {
        ctx->flags |= ARM_TASK_FLAG_MODE_SNAPSHOT_READY;
    }

    ret = task_pipe_channel_init(
        &ctx->rc_channel,
        g_arm_task_rc_channel_storage,
        ARM_TASK_RC_CHANNEL_CAPACITY_BYTES,
        sizeof(InputRcSnapshot));
    if (ret != OM_OK)
    {
        task_mpsc_channel_deinit(&ctx->mode_channel);
        task_context_pool_free(g_arm_task_slot_id);
        g_arm_task_slot_id = 0u;
        return ret;
    }

    ret = task_pipe_channel_init(
        &ctx->custom_controller_channel,
        g_arm_task_custom_controller_channel_storage,
        ARM_TASK_CUSTOM_CONTROLLER_CHANNEL_CAPACITY_BYTES,
        sizeof(InputCustomControllerSnapshot));
    if (ret != OM_OK)
    {
        task_pipe_channel_deinit(&ctx->rc_channel);
        task_mpsc_channel_deinit(&ctx->mode_channel);
        task_context_pool_free(g_arm_task_slot_id);
        g_arm_task_slot_id = 0u;
        return ret;
    }

    ret = arm_task_init_pids();
    if (ret != OM_OK)
    {
        task_pipe_channel_deinit(&ctx->custom_controller_channel);
        task_pipe_channel_deinit(&ctx->rc_channel);
        task_mpsc_channel_deinit(&ctx->mode_channel);
        task_context_pool_free(g_arm_task_slot_id);
        g_arm_task_slot_id = 0u;
        return ret;
    }

    status = osal_thread_create(
        &arm_task_thread,
        &arm_task_attr,
        arm_task_entry,
        ctx);
    if (status != OSAL_OK)
    {
        task_pipe_channel_deinit(&ctx->custom_controller_channel);
        task_pipe_channel_deinit(&ctx->rc_channel);
        task_mpsc_channel_deinit(&ctx->mode_channel);
        task_context_pool_free(g_arm_task_slot_id);
        g_arm_task_slot_id = 0u;
        arm_task_thread = OM_NULL;
        return OM_ERROR;
    }

    return OM_OK;
}

OmRet arm_task_submit_mode_control_snapshot(
    const ArmTaskModeSnapshot* snapshot)
{
    if (snapshot == OM_NULL || g_arm_task_owner_context == OM_NULL)
    {
        return OM_ERROR_PARAM;
    }

    return task_mpsc_channel_submit_nonblocking(
        &g_arm_task_owner_context->mode_channel,
        snapshot);
}

OmRet arm_task_submit_rc_snapshot(
    const InputRcSnapshot* snapshot)
{
    if (snapshot == OM_NULL || g_arm_task_owner_context == OM_NULL)
    {
        return OM_ERROR_PARAM;
    }

    return task_pipe_channel_submit_nonblocking(
        &g_arm_task_owner_context->rc_channel,
        snapshot,
        OM_TRUE);
}

OmRet arm_task_submit_custom_controller_snapshot(
    const InputCustomControllerSnapshot* snapshot)
{
    if (snapshot == OM_NULL || g_arm_task_owner_context == OM_NULL)
    {
        return OM_ERROR_PARAM;
    }

    return task_pipe_channel_submit_nonblocking(
        &g_arm_task_owner_context->custom_controller_channel,
        snapshot,
        OM_TRUE);
}
