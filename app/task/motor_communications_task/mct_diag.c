#include "task/motor_communications_task/mct_internal.h"

#include "drivers/peripheral/can/pal_can_dev.h"
#include "module/system_health/system_health.h"
#include "task/input_task/input_task.h"
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
    uint32_t attempt = 0u;
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

    runtime->last_p1010b_query_ms = now_ms;

    for (attempt = 0u; attempt < MCT_P1010B_COUNT; attempt++)
    {
        index = runtime->next_p1010b_query_index;
        runtime->next_p1010b_query_index =
            (runtime->next_p1010b_query_index + 1u) % MCT_P1010B_COUNT;
        driver = &runtime->p1010b_drivers[index];

        if (motor_recovery_should_defer_p1010b_query(driver) == OM_TRUE)
        {
            runtime->p1010b_last_query_ret[index] = OM_ERROR_BUSY;
            driver = OM_NULL;
            continue;
        }

        break;
    }

    if (driver == OM_NULL)
    {
        return;
    }

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
    CanErrCounter can_status = {0};
    const Go8010Feedback* go8010_feedback = OM_NULL;
    MotorRecoveryP1010BPredicateSnapshot p1010b_predicates[MCT_P1010B_COUNT] = {0};
    MctP1010BDiagSnapshot p1010b_diags[MCT_P1010B_COUNT] = {0};
    uint32_t p1010b_predicate_count = 0u;
    uint32_t p1010b_diag_count = 0u;
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
    snapshot->recovery_runtime_fault_active =
        (motor_recovery_is_runtime_fault_active() == OM_TRUE) ? 1u : 0u;
    snapshot->can_restart_count = motor_recovery_get_damiao_can_restart_count();
    go8010_feedback = go8010_get_feedback(&g_mct_runtime.go8010_pitch2_driver);

    if (g_mct_runtime.damiao_bus.canDev != OM_NULL &&
        device_ctrl(g_mct_runtime.damiao_bus.canDev, CAN_CMD_GET_STATUS, &can_status) == OM_OK)
    {
        snapshot->can2_tx_err_count = (uint32_t)can_status.txErrCnt;
        snapshot->can2_rx_err_count = (uint32_t)can_status.rxErrCnt;
    }

    for (index = 0u; index < MCT_DAMIAO_COUNT; index++)
    {
        snapshot->raw_rx_by_stdid[index] =
            damiao_motor_bus_get_raw_rx_by_stdid(&g_mct_runtime.damiao_bus, index);
        snapshot->feedback_timestamp_ms[index] =
            damiao_motor_get_feedback_timestamp_ms(&g_mct_runtime.damiao_drivers[index]);
        snapshot->feedback_sequence[index] =
            damiao_motor_get_feedback_sequence(&g_mct_runtime.damiao_drivers[index]);
        snapshot->damiao_status[index] =
            (uint32_t)damiao_motor_get_status(&g_mct_runtime.damiao_drivers[index]);
    }

    if (go8010_feedback != OM_NULL)
    {
        snapshot->go8010_feedback_timestamp_ms = go8010_feedback->timestampMs;
        snapshot->go8010_feedback_sequence = go8010_feedback->sequence;
    }

    for (index = 0u; index < MCT_DJI_CHASSIS_COUNT; index++)
    {
        snapshot->dji_feedback_timestamp_ms[index] =
            dji_motor_get_feedback_timestamp_ms(&g_mct_runtime.dji_chassis_drivers[index]);
        snapshot->dji_error_code[index] =
            (uint32_t)dji_motor_get_error_code(&g_mct_runtime.dji_chassis_drivers[index]);
    }

    snapshot->go8010_bus_rx_frame_count = g_mct_runtime.go8010_bus.rxFrameCount;
    snapshot->go8010_bus_last_rx_timestamp_ms = g_mct_runtime.go8010_bus.lastRxTimestampMs;
    snapshot->input_frame_count = g_input_task_runtime.frame_count;
    snapshot->input_invalid_frame_count = g_input_task_runtime.invalid_frame_count;
    snapshot->system_health_state = sh_get_state_value();
    snapshot->system_health_active_display_code = sh_get_active_display_code_value();

    (void)mct_copy_p1010b_predicate_snapshots(
        p1010b_predicates,
        MCT_P1010B_COUNT,
        &p1010b_predicate_count);
    (void)mct_copy_p1010b_diag_snapshots(
        p1010b_diags,
        MCT_P1010B_COUNT,
        &p1010b_diag_count);

    for (index = 0u; index < MCT_P1010B_COUNT; index++)
    {
        if (index < p1010b_predicate_count)
        {
            snapshot->p1010b_online[index] =
                (p1010b_predicates[index].online == OM_TRUE) ? 1u : 0u;
            snapshot->p1010b_state_enabled[index] =
                (p1010b_predicates[index].state_enabled == OM_TRUE) ? 1u : 0u;
            snapshot->p1010b_fault_clear[index] =
                (p1010b_predicates[index].fault_clear == OM_TRUE) ? 1u : 0u;
            snapshot->p1010b_healthy[index] =
                (p1010b_predicates[index].healthy == OM_TRUE) ? 1u : 0u;
        }

        if (index < p1010b_diag_count)
        {
            snapshot->p1010b_query_ok_age_ms[index] =
                p1010b_diags[index].query_ok_age_ms;
            snapshot->p1010b_last_query_ret[index] =
                p1010b_diags[index].last_query_ret;
        }
    }

    return OM_OK;
}
