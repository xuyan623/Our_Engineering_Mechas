#include "task/motor_communications_task/mct_internal.h"
#include "module/motor_tx_dispatch/motor_tx_dispatch.h"
#include "module/system_health/system_health.h"
#include "task/mode_task/mode_task.h"

/* mct.c 只保留 façade：
 * - 任务主循环
 * - 线程创建入口
 *
 * owner bring-up、vendor 启动期逻辑与诊断导出都已经拆去其他文件。
 */

static OmBool mct_owner_operational_active(const MctRuntime* runtime)
{
    if (runtime == OM_NULL)
    {
        return OM_FALSE;
    }

    return (OM_LOAD_ACQ(&runtime->operational_active) != 0u) ? OM_TRUE : OM_FALSE;
}

static OmBool mct_sources_include_formal_transmit(uint32_t sources_mask)
{
    const uint32_t formal_sources_mask =
        (1u << (uint32_t)MOTOR_TX_SOURCE_ARM) |
        (1u << (uint32_t)MOTOR_TX_SOURCE_CHASSIS);

    return ((sources_mask & formal_sources_mask) != 0u) ? OM_TRUE : OM_FALSE;
}

static void mct_run_operational_observation_only_transmit(MctRuntime* runtime)
{
    const OsalTimeMs now_ms = osal_time_now_monotonic();

    if (runtime == OM_NULL)
    {
        return;
    }

    if (runtime->last_operational_observation_ms != 0u &&
        (uint32_t)(now_ms - runtime->last_operational_observation_ms) < MCT_OPERATIONAL_OBSERVATION_PERIOD_MS)
    {
        return;
    }

    runtime->last_operational_observation_ms = now_ms;
    (void)motor_transmit_observation_only();
}

static void mct_run_operational_formal_transmit(
    MctRuntime* runtime,
    uint32_t sources_mask)
{
    const OsalTimeMs now_ms = osal_time_now_monotonic();

    if (runtime == OM_NULL)
    {
        return;
    }

    if (mct_sources_include_formal_transmit(sources_mask) == OM_TRUE)
    {
        runtime->operational_formal_transmit_pending = OM_TRUE;
    }

    if (runtime->operational_formal_transmit_pending != OM_TRUE)
    {
        return;
    }

    if (runtime->last_operational_formal_transmit_ms != 0u &&
        (uint32_t)(now_ms - runtime->last_operational_formal_transmit_ms) <
            MCT_OPERATIONAL_FORMAL_TRANSMIT_PERIOD_MS)
    {
        return;
    }

    runtime->last_operational_formal_transmit_ms = now_ms;
    runtime->operational_formal_transmit_pending = OM_FALSE;
    (void)motor_transmit_all();
}

static OmRet mct_submit_owner_command(
    MctRuntime* runtime,
    MctOwnerCommand command,
    OmBool replace_pending)
{
    OmRet ret = OM_OK;

    if (runtime == OM_NULL || command == MCT_OWNER_COMMAND_NONE)
    {
        return OM_ERROR_PARAM;
    }

    if (replace_pending == OM_TRUE)
    {
        ret = task_command_mailbox_reset(&runtime->owner_command_mailbox);
        if (ret != OM_OK)
        {
            return ret;
        }
    }

    return task_command_mailbox_submit_nonblocking(
        &runtime->owner_command_mailbox,
        &command);
}

static OmBool mct_process_owner_requests(MctRuntime* runtime)
{
    MctOwnerCommand command = MCT_OWNER_COMMAND_NONE;
    OmRet ret = OM_OK;

    if (runtime == OM_NULL)
    {
        return OM_FALSE;
    }

    ret = task_command_mailbox_receive(
        &runtime->owner_command_mailbox,
        &command,
        0u);
    if (ret == OM_ERROR_WOULD_BLOCK || ret == OM_ERROR_TIMEOUT)
    {
        return OM_FALSE;
    }

    if (ret != OM_OK)
    {
        return OM_FALSE;
    }

    if (command == MCT_OWNER_COMMAND_RESET_OPERATIONAL)
    {
        OM_STORE_REL(&runtime->operational_active, 0u);
        ret = mct_runtime_leave_operational_state(runtime);
        if (ret == OM_OK)
        {
            ret = mct_runtime_enter_operational_state(runtime);
            if (ret == OM_OK)
            {
                OM_STORE_REL(&runtime->operational_active, 1u);
            }
        }

        return OM_TRUE;
    }

    if (command == MCT_OWNER_COMMAND_ENTER_OPERATIONAL)
    {
        if (mct_runtime_enter_operational_state(runtime) == OM_OK)
        {
            OM_STORE_REL(&runtime->operational_active, 1u);
        }
        return OM_TRUE;
    }

    if (command == MCT_OWNER_COMMAND_LEAVE_OPERATIONAL)
    {
        OM_STORE_REL(&runtime->operational_active, 0u);
        if (mct_runtime_leave_operational_state(runtime) == OM_OK)
        {
            return OM_TRUE;
        }
        return OM_TRUE;
    }

    return OM_TRUE;
}

/* 主循环职责非常固定：
 * 1. 等待事件或周期超时
 * 2. 优先处理 owner request（软件重进正式可控态）
 * 2. 先打一拍心跳，避免后面同步调用过长时直接触发 timeout
 * 3. 统一发送所有 vendor
 * 4. 统一刷新所有 vendor 反馈
 * 5. 若收到新反馈，发布 EVT_MOTOR_FEEDBACK_READY
 * 6. 基于最新反馈推进恢复模块与 runtime fault
 * 7. 再打一拍心跳，表示本轮物理通信已完整走完
 */
static void mct_entry(void* arg)
{
    MctRuntime* runtime = (MctRuntime*)arg;
    OsalTimeMs deadline_cursor_ms = 0u;
    uint32_t sources_mask = 0u;
    OmBool overflowed = OM_FALSE;

    if (runtime == OM_NULL)
    {
        for (;;)
        {
            (void)osal_sleep_ms(1000u);
        }
    }

    while (1)
    {
        sources_mask = motor_tx_dispatch_drain_sources_mask();
        overflowed = motor_tx_dispatch_take_overflow_flag();
        runtime->last_tx_request_sources_mask = sources_mask;
        runtime->last_tx_request_overflowed = overflowed;

        if (mct_process_owner_requests(runtime) == OM_TRUE)
        {
            (void)sh_beat(SH_TASK_MOTOR_COMMUNICATIONS);
            continue;
        }

        if (mct_owner_operational_active(runtime) != OM_TRUE)
        {
            (void)sh_beat(SH_TASK_MOTOR_COMMUNICATIONS);
            (void)mct_runtime_run_non_operational_cycle(runtime);
            (void)sh_beat(SH_TASK_MOTOR_COMMUNICATIONS);
            continue;
        }

        (void)sh_beat(SH_TASK_MOTOR_COMMUNICATIONS);
        /* 临时调试：当前正式发送只在明确收到 control task 请求时执行。
         * 没有正式发送请求时，仍只走低频 observation-only 刷反馈。 */
        if (mct_sources_include_formal_transmit(sources_mask) == OM_TRUE ||
            runtime->operational_formal_transmit_pending == OM_TRUE)
        {
            mct_run_operational_formal_transmit(runtime, sources_mask);
        }
        else
        {
            mct_run_operational_observation_only_transmit(runtime);
        }

        if (motor_receive_all() == OM_OK)
        {
            mct_capture_go8010_zero(runtime);
        }
        motor_recovery_tick();

        (void)sh_beat(SH_TASK_MOTOR_COMMUNICATIONS);
        (void)osal_delay_until(&deadline_cursor_ms, MCT_LOOP_PERIOD_MS, OM_NULL);
    }
}

/* 任务启动入口只负责：
 * - 防重入
 * - 调 runtime_init 完成 owner 接线
 * - 创建正式通信线程
 */
/* VTable for mct context pool. */
static void mct_ctx_init(void* ctx)
{
    (void)ctx;
}

static void mct_ctx_reset(void* ctx)
{
    (void)ctx;
}

static void mct_ctx_cleanup(void* ctx)
{
    MctRuntime* self = (MctRuntime*)ctx;
    task_command_mailbox_deinit(&self->owner_command_mailbox);
}

static void mct_diag_online(void* ctx, uint8_t* out_online)
{
    MctRuntime* runtime = (MctRuntime*)ctx;
    uint8_t online = 0u;
    uint32_t i = 0u;

    if (runtime == OM_NULL || out_online == OM_NULL)
    {
        return;
    }

    /* P1010B 在线状态 */
    for (i = 0u; i < MCT_P1010B_COUNT; i++)
    {
        if (p1010b_is_online(&runtime->p1010b_drivers[i]) == OM_TRUE)
        {
            online |= (1u << i);
        }
    }

    /* Damiao 在线状态 */
    for (i = 0u; i < MCT_DAMIAO_COUNT; i++)
    {
        if (motor_is_feedback_recent(&runtime->damiao_motors[i], 100u) == OM_TRUE)
        {
            online |= (1u << (MCT_P1010B_COUNT + i));
        }
    }

    *out_online = online;
}

static void mct_diag_snapshot(void* ctx, float* out_buf, uint32_t cap, uint32_t* out_count)
{
    MctRuntime* runtime = (MctRuntime*)ctx;
    uint32_t idx = 0u;

    if (out_buf == OM_NULL || out_count == OM_NULL)
    {
        return;
    }

    *out_count = 0u;

    if (cap < 4u || runtime == OM_NULL)
    {
        return;
    }

    out_buf[idx++] = (float)(OM_LOAD_ACQ(&runtime->operational_active));
    out_buf[idx++] = (float)(runtime->last_tx_request_sources_mask);
    out_buf[idx++] = (float)(runtime->last_tx_request_overflowed);
    out_buf[idx++] = (float)(runtime->last_operational_formal_transmit_ms);

    *out_count = idx;
}

static const TaskContextVTable g_mct_vtable = {
    .task_name = "mct",
    .init = mct_ctx_init,
    .reset = mct_ctx_reset,
    .cleanup = mct_ctx_cleanup,
    .diag_online = mct_diag_online,
    .diag_snapshot = mct_diag_snapshot,
};

OmRet mct_start(const BspDeviceRegistry* devices)
{
    static OsalThread* mct_thread = OM_NULL;
    const OsalThreadAttr mct_thread_attr = {
        "motor_comm",
        MCT_STACK_WORDS * OSAL_STACK_WORD_BYTES,
        MCT_PRIORITY};
    OsalStatus status = OSAL_INVALID;
    OmRet ret = OM_OK;
    MctRuntime* ctx = OM_NULL;

    if (devices == OM_NULL)
    {
        return OM_ERROR_NULL;
    }

    if (mct_thread != OM_NULL)
    {
        return OM_ERR_CONFLICT;
    }

    g_mct_slot_id = task_context_pool_alloc("mct", sizeof(MctRuntime), &g_mct_vtable);
    if (g_mct_slot_id == 0u)
    {
        return OM_ERROR;
    }

    ctx = (MctRuntime*)task_context_pool_get_ptr(g_mct_slot_id);

    ret = mct_runtime_init(ctx, devices);
    if (ret != OM_OK)
    {
        task_context_pool_free(g_mct_slot_id);
        g_mct_slot_id = 0u;
        return ret;
    }

    status = osal_thread_create(
        &mct_thread,
        &mct_thread_attr,
        mct_entry,
        ctx);
    if (status != OSAL_OK)
    {
        task_context_pool_free(g_mct_slot_id);
        g_mct_slot_id = 0u;
        mct_thread = OM_NULL;
        return OM_ERROR;
    }

    {
        const ModeTaskInitProgressMessage can_ready = {
            .kind = (uint8_t)MODE_TASK_INIT_PROGRESS_CAN_READY,
            .value = 1u};
        const ModeTaskInitProgressMessage chassis_motor_ready = {
            .kind = (uint8_t)MODE_TASK_INIT_PROGRESS_CHASSIS_MOTOR_READY,
            .value = 1u};
        const ModeTaskInitProgressMessage arm_motor_ready = {
            .kind = (uint8_t)MODE_TASK_INIT_PROGRESS_ARM_MOTOR_READY,
            .value = 1u};

        (void)mode_task_submit_init_progress(&can_ready);
        (void)mode_task_submit_init_progress(&chassis_motor_ready);
        (void)mode_task_submit_init_progress(&arm_motor_ready);
    }

    return OM_OK;
}

OmRet mct_request_enter_operational_state(void)
{
    OmRet ret = OM_OK;

    ret = mct_submit_owner_command(
        &g_mct_runtime,
        MCT_OWNER_COMMAND_ENTER_OPERATIONAL,
        OM_TRUE);
    if (ret != OM_OK)
    {
        return ret;
    }

    return OM_OK;
}

OmRet mct_request_leave_operational_state(void)
{
    OmRet ret = OM_OK;

    ret = mct_submit_owner_command(
        &g_mct_runtime,
        MCT_OWNER_COMMAND_LEAVE_OPERATIONAL,
        OM_TRUE);
    if (ret != OM_OK)
    {
        return ret;
    }

    return OM_OK;
}

OmRet mct_request_reset_operational_state(void)
{
    OmRet ret = OM_OK;

    ret = mct_submit_owner_command(
        &g_mct_runtime,
        MCT_OWNER_COMMAND_RESET_OPERATIONAL,
        OM_TRUE);
    if (ret != OM_OK)
    {
        return ret;
    }

    return OM_OK;
}

OmBool mct_is_operational_active(void)
{
    return mct_owner_operational_active(&g_mct_runtime);
}

OmRet mct_copy_runtime_debug_snapshot(
    MctRuntimeDebugSnapshot* snapshot)
{
    if (snapshot == OM_NULL)
    {
        return OM_ERROR_NULL;
    }

    snapshot->operational_active =
        (uint8_t)((mct_owner_operational_active(&g_mct_runtime) == OM_TRUE) ? 1u : 0u);
    snapshot->last_tx_request_sources_mask = g_mct_runtime.last_tx_request_sources_mask;
    snapshot->last_tx_request_overflowed =
        (uint8_t)((g_mct_runtime.last_tx_request_overflowed == OM_TRUE) ? 1u : 0u);
    return OM_OK;
}
