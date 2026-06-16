#include "task/motor_communications_task/mct_internal.h"

#include <string.h>

/* mct_diag.c 只承接对外最小诊断快照导出。 */

/* recovery 快照直接转发给 motor_recovery 模块。 */
OmRet mct_copy_recovery_snapshots(
    MotorRecoverySnapshot* snapshots,
    uint32_t capacity,
    uint32_t* snapshot_count)
{
    return motor_recovery_copy(snapshots, capacity, snapshot_count);
}

OmRet mct_copy_p1010b_checks(
    MotorRecoveryP1010BInfo* snapshots,
    uint32_t capacity,
    uint32_t* snapshot_count)
{
    return motor_recovery_copy_p1010b(
        snapshots,
        capacity,
        snapshot_count);
}

/* P1010B 诊断快照只反映 active report 的反馈新鲜度。 */
OmRet mct_copy_p1010b_diag(
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
        const Motor* motor = &g_mct_runtime.p1010b_motors[index];
        const P1010BDriver* driver = motor->binding.p1010b.driver;
        uint32_t last_feedback_ms = 0u;
        uint32_t feedback_age_ms = 0u;
        uint8_t online = 0u;

        if (*snapshot_count >= capacity)
        {
            break;
        }

        if (driver != OM_NULL)
        {
            last_feedback_ms = driver->telemetry.lastSuccessRxTimestampMs;
        }

        if (last_feedback_ms != 0u)
        {
            feedback_age_ms = (uint32_t)(now_ms - last_feedback_ms);
        }

        if (motor->feedback.online == OM_TRUE)
        {
            online = 1u;
        }

        snapshots[*snapshot_count].name = g_mct_p1010b_configs[index].name;
        snapshots[*snapshot_count].last_feedback_ms = last_feedback_ms;
        snapshots[*snapshot_count].feedback_age_ms = feedback_age_ms;
        snapshots[*snapshot_count].online = online;
        snapshots[*snapshot_count].absolute_position_raw =
            (driver != OM_NULL) ? driver->telemetry.feedback.absolutePosition : 0u;
        snapshots[*snapshot_count].feedback_angle_rad =
            (motor->feedback.timestamp_ms != 0u) ? motor->feedback.angle : 0.0f;
        (*snapshot_count)++;
    }

    return OM_OK;
}

/* 达妙诊断快照提供：
 * - bus 级 raw rx 观测
 * - 每台正式达妙驱动的 feedback_sequence
 */
OmRet mct_copy_damiao_bus_diag(
    MctDamiaoDiagSnapshot* snapshot)
{
    uint32_t index = 0u;

    if (snapshot == OM_NULL)
    {
        return OM_ERROR_NULL;
    }

    memset(snapshot, 0, sizeof(*snapshot));
    snapshot->raw_rx_count =
        dm_raw_rx_cnt(&g_mct_runtime.damiao_bus);
    snapshot->last_raw_stdid =
        dm_last_raw_id(&g_mct_runtime.damiao_bus);

    for (index = 0u; index < MCT_DAMIAO_COUNT; index++)
    {
        snapshot->raw_rx_by_stdid[index] =
            dm_raw_rx_id(&g_mct_runtime.damiao_bus, index);
        snapshot->raw_tx_by_stdid[index] =
            dm_raw_tx_id(&g_mct_runtime.damiao_bus, index);
        snapshot->feedback_sequence[index] =
            dm_fb_seq(&g_mct_runtime.damiao_drivers[index]);
    }

    return OM_OK;
}

OmRet mct_copy_damiao_diag_items(
    MctDamiaoDiagItem* snapshots,
    uint32_t capacity,
    uint32_t* snapshot_count)
{
    uint32_t index = 0u;
    const OsalTimeMs now_ms = osal_time_now_monotonic();

    if (snapshots == OM_NULL || snapshot_count == OM_NULL)
    {
        return OM_ERROR_NULL;
    }

    *snapshot_count = 0u;
    if (capacity == 0u)
    {
        return OM_ERROR_PARAM;
    }

    memset(snapshots, 0, sizeof(*snapshots) * capacity);

    for (index = 0u; index < MCT_DAMIAO_COUNT; index++)
    {
        if (index >= capacity)
        {
            break;
        }

        snapshots[index].name = g_mct_damiao_configs[index].name;
        snapshots[index].online =
            (g_mct_runtime.damiao_motors[index].feedback.online == OM_TRUE) ? 1u : 0u;
        snapshots[index].age_ms =
            (g_mct_runtime.damiao_motors[index].feedback.timestamp_ms != 0u) ?
                (uint32_t)(now_ms - g_mct_runtime.damiao_motors[index].feedback.timestamp_ms) :
                0u;
        snapshots[index].feedback_sequence =
            dm_fb_seq(&g_mct_runtime.damiao_drivers[index]);
        snapshots[index].status =
            damiao_motor_get_status(&g_mct_runtime.damiao_drivers[index]);
        (*snapshot_count)++;
    }

    return OM_OK;
}
