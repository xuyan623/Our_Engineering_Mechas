#include "task/motor_communications_task/mct_internal.h"

#include <string.h>

/* mct_diag.c 承接两类职责：
 * 1. P1010B query-mode 的正式轮询与反馈回填
 * 2. 对外最小诊断快照导出
 */

/* 把 active_query 返回值回填进 driver telemetry。
 * 这里只回填正式链需要消费的最小字段，不做额外派生状态。
 */
static void mct_write_p1010b_query_feedback(
    P1010BDriver* driver,
    const P1010BResponse* response)
{
    if (driver == OM_NULL || response == OM_NULL)
    {
        return;
    }

    driver->telemetry.feedback.absolutePosition =
        (uint16_t)response->data.activeQueryValues[0];
    driver->telemetry.feedback.speedRpm =
        ((float)response->data.activeQueryValues[1]) / 10.0f;
    driver->telemetry.feedback.iqAmpere =
        ((float)response->data.activeQueryValues[2]) / 100.0f;
    driver->telemetry.feedback.busVoltage =
        ((float)response->data.activeQueryValues[3]) / 10.0f;
    driver->telemetry.feedback.timestampMs = response->timestampMs;
    driver->telemetry.lastSuccessRxTimestampMs = response->timestampMs;
    driver->telemetry.online = true;
}

void mct_query_one_p1010b(MctRuntime* runtime)
{
    P1010BDriver* driver = OM_NULL;
    P1010BResponse response = {0};
    OsalTimeMs now_ms = 0u;
    OsalTimeMs query_ok_ms = 0u;
    uint32_t index = 0u;
    OmRet query_ret = OM_ERROR_EMPTY;

    if (runtime == OM_NULL)
    {
        return;
    }

    now_ms = osal_time_now_monotonic();
    /* 正式链采用 query-mode，固定 10ms 只轮询一台，
     * 避免把两台 P1010B 的同步请求挤在同一轮里。
     */
    if ((uint32_t)(now_ms - runtime->last_p1010b_query_ms) <
        MCT_P1010B_QUERY_PERIOD_MS)
    {
        return;
    }

    index = runtime->next_p1010b_query_index;
    runtime->next_p1010b_query_index =
        (runtime->next_p1010b_query_index + 1u) % MCT_P1010B_COUNT;
    runtime->last_p1010b_query_ms = now_ms;
    driver = &runtime->p1010b_drivers[index];

    query_ret = p1010b_active_query_slots(
        driver,
        P1010B_REPORT_DATA_ABSOLUTE_POSITION,
        P1010B_REPORT_DATA_SPEED_RPM,
        P1010B_REPORT_DATA_IQ_AMPERE,
        P1010B_REPORT_DATA_BUS_VOLTAGE,
        0u,
        &response);
    runtime->p1010b_last_query_ret[index] = query_ret;
    if (query_ret == OM_OK)
    {
        query_ok_ms = osal_time_now_monotonic();
        runtime->p1010b_last_query_ok_ms[index] = query_ok_ms;
        /* query 成功时间戳同时作为 recovery 的在线依据。 */
        motor_recovery_notify_p1010b_query_ok(driver, query_ok_ms);
        mct_write_p1010b_query_feedback(driver, &response);
    }
}

/* recovery 快照直接转发给 motor_recovery 模块。 */
OmRet mct_copy_recovery_snapshots(
    MotorRecoverySnapshot* snapshots,
    uint32_t capacity,
    uint32_t* snapshot_count)
{
    return motor_recovery_copy_snapshots(snapshots, capacity, snapshot_count);
}

OmRet mct_copy_p1010b_predicate_snapshots(
    MotorRecoveryP1010BPredicateSnapshot* snapshots,
    uint32_t capacity,
    uint32_t* snapshot_count)
{
    return motor_recovery_copy_p1010b_predicate_snapshots(
        snapshots,
        capacity,
        snapshot_count);
}

/* P1010B 诊断快照只反映 owner task 本地 query 调度状态。 */
OmRet mct_copy_p1010b_diag_snapshots(
    MctP1010BDiagSnapshot* snapshots,
    uint32_t capacity,
    uint32_t* snapshot_count)
{
    uint32_t index = 0u;
    OsalTimeMs now_ms = 0u;

    if (snapshots == OM_NULL || snapshot_count == OM_NULL)
    {
        return OM_ERROR_NULL;
    }

    *snapshot_count = 0u;
    now_ms = osal_time_now_monotonic();

    for (index = 0u; index < MCT_P1010B_COUNT; index++)
    {
        uint32_t last_query_ok_ms =
            g_mct_runtime.p1010b_last_query_ok_ms[index];
        uint32_t query_ok_age_ms = 0u;

        if (*snapshot_count >= capacity)
        {
            break;
        }

        if (last_query_ok_ms != 0u)
        {
            query_ok_age_ms = (uint32_t)(now_ms - last_query_ok_ms);
        }

        snapshots[*snapshot_count].name = g_mct_p1010b_configs[index].name;
        snapshots[*snapshot_count].last_query_ok_ms = last_query_ok_ms;
        snapshots[*snapshot_count].query_ok_age_ms = query_ok_age_ms;
        snapshots[*snapshot_count].last_query_ret =
            g_mct_runtime.p1010b_last_query_ret[index];
        (*snapshot_count)++;
    }

    return OM_OK;
}

/* 达妙诊断快照提供：
 * - bus 级 raw rx 观测
 * - 每台正式达妙驱动的 feedback_sequence
 */
OmRet mct_copy_damiao_diag(
    MctDamiaoDiagSnapshot* snapshot)
{
    uint32_t index = 0u;

    if (snapshot == OM_NULL)
    {
        return OM_ERROR_NULL;
    }

    memset(snapshot, 0, sizeof(*snapshot));
    snapshot->raw_rx_count =
        damiao_motor_bus_get_raw_rx_count(&g_mct_runtime.damiao_bus);
    snapshot->last_raw_stdid =
        damiao_motor_bus_get_last_raw_stdid(&g_mct_runtime.damiao_bus);

    for (index = 0u; index < MCT_DAMIAO_COUNT; index++)
    {
        snapshot->feedback_sequence[index] =
            damiao_motor_get_feedback_sequence(&g_mct_runtime.damiao_drivers[index]);
    }

    return OM_OK;
}
