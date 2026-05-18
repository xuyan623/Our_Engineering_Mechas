#include "task/motor_communications_task/mct_internal.h"
#include "module/data_pool/data_pool.h"
#include "module/system_health/system_health.h"
#include "task/mode_task/mode_task.h"

/* mct.c 只保留 façade：
 * - 任务主循环
 * - 线程创建入口
 *
 * owner bring-up、vendor 启动期逻辑、query 与诊断导出都已经拆去其他文件。
 */

/* 主循环职责非常固定：
 * 1. 等待事件或周期超时
 * 2. 先打一拍心跳，避免后面同步调用过长时直接触发 timeout
 * 3. 轮询一台 P1010B 的 active_query
 * 4. 统一发送所有 vendor
 * 5. 统一刷新所有 vendor 反馈
 * 6. 若收到新反馈，发布 EVT_MOTOR_FEEDBACK_READY
 * 7. 基于最新反馈推进恢复模块与 runtime fault
 * 8. 再打一拍心跳，表示本轮物理通信已完整走完
 */
static OmBool mct_release_active(void)
{
    return ((GlobalMode)DP_LOAD_UINT8(&g_data_pool.mode.global_mode) == MODE_GLOBAL_RELEASE_CTRL) ? OM_TRUE : OM_FALSE;
}

static void mct_entry(void* arg)
{
    MctRuntime* runtime = (MctRuntime*)arg;
    OsalStatus wait_status = OSAL_INVALID;
    OmBool release_active = OM_FALSE;

    if (runtime == OM_NULL)
    {
        for (;;)
        {
            (void)osal_sleep_ms(1000u);
        }
    }

    while (1)
    {
        /* 有发送请求就尽快苏醒；没有请求也按固定周期轮询，
         * 保证接收刷新、P1010B query 与 recovery 都持续推进。
         */
        wait_status = osal_event_flags_wait(
            runtime->tx_request_subscription.flags,
            runtime->tx_request_subscription.waitMask,
            OM_NULL,
            MCT_LOOP_PERIOD_MS,
            0u);

        (void)wait_status;
        release_active = mct_release_active();
        mct_apply_release_gate(runtime, release_active);
        runtime->release_active = release_active;
        (void)sh_beat(SH_TASK_MOTOR_COMMUNICATIONS);
        mct_query_one_p1010b(runtime);
        (void)motor_transmit_all();

        if (motor_receive_all() == OM_OK)
        {
            mct_capture_go8010_zero(runtime);
            (void)event_bus_publish(&g_event_bus, EVT_MOTOR_FEEDBACK_READY);
        }
        if (release_active != OM_TRUE)
        {
            motor_recovery_tick();
        }
        else
        {
            (void)sh_clear_runtime_fault(SH_ERR_MOTOR_RECOVERY_DEGRADED);
        }

        (void)sh_beat(SH_TASK_MOTOR_COMMUNICATIONS);
    }
}

/* 任务启动入口只负责：
 * - 防重入
 * - 调 runtime_init 完成 owner 接线
 * - 创建正式通信线程
 */
OmRet mct_start(const BspDeviceRegistry* devices)
{
    static OsalThread* mct_thread = OM_NULL;
    const OsalThreadAttr mct_thread_attr = {
        "motor_comm",
        MCT_STACK_WORDS * OSAL_STACK_WORD_BYTES,
        MCT_PRIORITY};
    OsalStatus status = OSAL_INVALID;
    OmRet ret = OM_OK;

    if (devices == OM_NULL)
    {
        return OM_ERROR_NULL;
    }

    if (mct_thread != OM_NULL)
    {
        return OM_ERR_CONFLICT;
    }

    ret = mct_runtime_init(&g_mct_runtime, devices);
    if (ret != OM_OK)
    {
        return ret;
    }

    status = osal_thread_create(
        &mct_thread,
        &mct_thread_attr,
        mct_entry,
        &g_mct_runtime);
    if (status != OSAL_OK)
    {
        mct_thread = OM_NULL;
        return OM_ERROR;
    }

    return OM_OK;
}
