#include "task/chassis_task/chassis_task.h"

#include "module/system_health/system_health.h"
#include "osal/osal.h"
#include "osal/osal_config.h"
#include "osal/osal_time.h"
#include "task/chassis_task/chassis_task_internal.h"
#include "task/mode_task/mode_task.h"
#include <string.h>

TaskContextSlotId g_chassis_task_slot_id = 0;
uint8_t g_chassis_task_mode_channel_storage[sizeof(ChassisModeSnap) * CT_MODE_CHANNEL_CAPACITY] = {0};
OmAtomicU8 g_chassis_task_mode_channel_ready_flags[CT_MODE_CHANNEL_CAPACITY] = {0};
uint8_t g_chassis_task_rc_channel_storage[CT_RC_BYTES] = {0};
uint8_t g_chassis_task_imu_channel_storage[CT_IMU_BYTES] = {0};

static void chassis_task_entry(void* arg)
{
    ChassisTaskContext* context = (ChassisTaskContext*)arg;
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
        chassis_task_run_once(context);
        (void)sh_beat(SH_TASK_CHASSIS);
        (void)osal_delay_until(&deadline_cursor_ms, CT_PERIOD_MS, OM_NULL);
    }
}

static void chassis_task_ctx_init(void* ctx)
{
    ChassisTaskContext* self = (ChassisTaskContext*)ctx;
    memset(self, 0, sizeof(ChassisTaskContext));
}

static void chassis_task_ctx_reset(void* ctx)
{
    ChassisTaskContext* self = (ChassisTaskContext*)ctx;
    memset(&self->latest_mode_snapshot, 0, sizeof(self->latest_mode_snapshot));
    memset(&self->latest_rc_snapshot, 0, sizeof(self->latest_rc_snapshot));
    memset(&self->latest_imu_snapshot, 0, sizeof(self->latest_imu_snapshot));
    memset(self->last_wheel_speed_reference_rpm, 0, sizeof(self->last_wheel_speed_reference_rpm));
    self->pit_leg_cmd_deg = 0.0f;
    self->big_yaw_hold_angle_rad = 0.0f;
    self->last_tx_request_ms = 0u;
    self->rc_rotate_saturation_since_ms = 0u;
    self->flags = 0u;
}

static void chassis_task_ctx_cleanup(void* ctx)
{
    (void)ctx;
}

static const TaskContextVTable g_chassis_task_vtable = {
    .task_name = "chassis_task",
    .init = chassis_task_ctx_init,
    .reset = chassis_task_ctx_reset,
    .cleanup = chassis_task_ctx_cleanup,
    .diag_online = chassis_task_diag_online,
    .diag_snapshot = chassis_task_diag_snapshot,
};

OmRet chassis_task_start(void)
{
    static OsalThread* chassis_task_thread = OM_NULL;
    const OsalThreadAttr chassis_task_attr = {
        "chassis_task",
        CT_STACK_BYTES,
        CT_PRIORITY};
    OsalStatus status = OSAL_INVALID;
    OmRet ret = OM_OK;
    ChassisTaskContext* ctx = OM_NULL;

    if (chassis_task_thread != OM_NULL)
    {
        return OM_ERR_CONFLICT;
    }

    g_chassis_task_slot_id = task_context_pool_alloc("chassis_task", sizeof(ChassisTaskContext), &g_chassis_task_vtable);
    if (g_chassis_task_slot_id == 0u)
    {
        return OM_ERROR;
    }

    ctx = (ChassisTaskContext*)task_context_pool_get_ptr(g_chassis_task_slot_id);

    ret = task_mpsc_channel_init(
        &ctx->mode_channel,
        g_chassis_task_mode_channel_storage,
        g_chassis_task_mode_channel_ready_flags,
        sizeof(ChassisModeSnap),
        CT_MODE_CHANNEL_CAPACITY);
    if (ret != OM_OK)
    {
        task_context_pool_free(g_chassis_task_slot_id);
        g_chassis_task_slot_id = 0u;
        return ret;
    }

    if (mode_task_copy_chassis_mode(&ctx->latest_mode_snapshot) == OM_TRUE)
    {
        ctx->flags |= CT_FLAG_MODE_READY;
    }

    ret = task_pipe_channel_init(
        &ctx->rc_channel,
        g_chassis_task_rc_channel_storage,
        CT_RC_BYTES,
        sizeof(InputRcSnapshot));
    if (ret != OM_OK)
    {
        task_mpsc_channel_deinit(&ctx->mode_channel);
        task_context_pool_free(g_chassis_task_slot_id);
        g_chassis_task_slot_id = 0u;
        return ret;
    }

    ret = task_pipe_channel_init(
        &ctx->imu_channel,
        g_chassis_task_imu_channel_storage,
        CT_IMU_BYTES,
        sizeof(ImuTaskSnapshot));
    if (ret != OM_OK)
    {
        task_pipe_channel_deinit(&ctx->rc_channel);
        task_mpsc_channel_deinit(&ctx->mode_channel);
        task_context_pool_free(g_chassis_task_slot_id);
        g_chassis_task_slot_id = 0u;
        return ret;
    }

    ret = chassis_task_init_pids(ctx);
    if (ret != OM_OK)
    {
        task_pipe_channel_deinit(&ctx->imu_channel);
        task_pipe_channel_deinit(&ctx->rc_channel);
        task_mpsc_channel_deinit(&ctx->mode_channel);
        task_context_pool_free(g_chassis_task_slot_id);
        g_chassis_task_slot_id = 0u;
        return ret;
    }

    status = osal_thread_create(
        &chassis_task_thread,
        &chassis_task_attr,
        chassis_task_entry,
        ctx);
    if (status != OSAL_OK)
    {
        task_pipe_channel_deinit(&ctx->imu_channel);
        task_pipe_channel_deinit(&ctx->rc_channel);
        task_mpsc_channel_deinit(&ctx->mode_channel);
        task_context_pool_free(g_chassis_task_slot_id);
        g_chassis_task_slot_id = 0u;
        chassis_task_thread = OM_NULL;
        return OM_ERROR;
    }

    return OM_OK;
}

OmRet chassis_task_submit_mode(
    const ChassisModeSnap* snapshot)
{
    if (snapshot == OM_NULL || g_chassis_task_owner_context == OM_NULL)
    {
        return OM_ERROR_PARAM;
    }

    return tmpsc_submit(
        &g_chassis_task_owner_context->mode_channel,
        snapshot);
}

OmRet chassis_task_submit_rc(
    const InputRcSnapshot* snapshot)
{
    if (snapshot == OM_NULL || g_chassis_task_owner_context == OM_NULL)
    {
        return OM_ERROR_PARAM;
    }

    return tpipe_submit(
        &g_chassis_task_owner_context->rc_channel,
        snapshot,
        OM_TRUE);
}

OmRet chassis_task_submit_imu(
    const ImuTaskSnapshot* snapshot)
{
    if (snapshot == OM_NULL || g_chassis_task_owner_context == OM_NULL)
    {
        return OM_ERROR_PARAM;
    }

    return tpipe_submit(
        &g_chassis_task_owner_context->imu_channel,
        snapshot,
        OM_TRUE);
}
