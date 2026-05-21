#include "module/motor_recovery/motor_recovery.h"

#include "config/app_config.h"
#include "drivers/peripheral/can/pal_can_dev.h"
#include "module/system_health/system_health.h"
#include "osal/osal_time.h"
#include <string.h>

/* 该模块是 app 层的运行时自动恢复 owner：
 * - 不拥有物理总线
 * - 不直接启动任务
 * - 只维护“哪台电机离线、何时重试、何时报码”的状态机
 *
 * 真正的收发仍然由 motor_communications_task 调 motor_transmit_all()/motor_receive_all() 执行。
 */

#define MOTOR_RECOVERY_CAPACITY (MOTOR_REGISTRY_CAPACITY)
#define MOTOR_RECOVERY_DAMIAO_MODE_SETTLE_MS (10u)
#define MOTOR_RECOVERY_DAMIAO_ENABLE_SETTLE_MS (50u)
#define MOTOR_RECOVERY_DAMIAO_FEEDBACK_PROBE_SETTLE_MS (10u)
#define MOTOR_RECOVERY_DAMIAO_CAN_RESTART_THROTTLE_MS (200u)
#define MOTOR_RECOVERY_CAN1_RESTART_THROTTLE_MS (200u)
#define MOTOR_RECOVERY_DJI_CAN1_RESTART_MIN_GAP_MS (600u)
#define MOTOR_RECOVERY_DJI_CAN1_RESTART_EPSILON (1.0f)
#define MOTOR_RECOVERY_P1010B_ENABLE_SETTLE_MS (150u)
#define MOTOR_RECOVERY_P1010B_FAULT_DEBOUNCE_MS (300u)
#define MOTOR_RECOVERY_P1010B_SYNC_TIMEOUT_MS (20u)
#define MOTOR_RECOVERY_P1010B_MAX_RETRY_COUNT (1u)
#define MOTOR_RECOVERY_P1010B_REPORT_PERIOD_MS (10u)
#define MOTOR_RECOVERY_DAMIAO_CTRL_MODE_RID (10u)
#define MOTOR_RECOVERY_DAMIAO_CTRL_MODE_MIT (1u)

/* 恢复子状态机只在模块内部使用：
 * - P1010B 需要多步同步恢复
 * - Damiao 当前只区分“等待 enable”与“正常 observation”
 */
typedef enum
{
    MOTOR_RECOVERY_VENDOR_SUBSTATE_NONE = 0u,
    MOTOR_RECOVERY_VENDOR_SUBSTATE_P1010B_DISABLE,
    MOTOR_RECOVERY_VENDOR_SUBSTATE_P1010B_SET_MODE,
    MOTOR_RECOVERY_VENDOR_SUBSTATE_P1010B_SET_ACTIVE_REPORT,
    MOTOR_RECOVERY_VENDOR_SUBSTATE_P1010B_ENABLE,
    MOTOR_RECOVERY_VENDOR_SUBSTATE_DAMIAO_WRITE_MIT,
    MOTOR_RECOVERY_VENDOR_SUBSTATE_DAMIAO_WAIT_MIT_SETTLE,
    MOTOR_RECOVERY_VENDOR_SUBSTATE_DAMIAO_SEND_ENABLE,
    MOTOR_RECOVERY_VENDOR_SUBSTATE_DAMIAO_WAIT_ENABLE_SETTLE,
    MOTOR_RECOVERY_VENDOR_SUBSTATE_DAMIAO_SEND_FEEDBACK_PROBE,
    MOTOR_RECOVERY_VENDOR_SUBSTATE_DAMIAO_WAIT_FEEDBACK_PROBE,
} MotorRecoveryVendorSubstate;

/* 每个条目的恢复策略。
 * 当前所有电机共享在线超时 / 重试间隔 / 报错去抖常量，
 * 若后续要做 vendor 特例，也应只在这里扩展。
 */
typedef struct
{
    uint16_t online_timeout_ms;
    uint16_t retry_interval_ms;
    uint16_t fault_debounce_ms;
} MotorRecoveryPolicy;

/* 模块内部的完整恢复条目。
 * 对外只暴露 MotorRecoverySnapshot，避免上层依赖内部状态机细节。
 */
typedef struct
{
    const char* name;
    MotorVendor vendor;
    Motor* motor;
    void* driver;
    MotorRecoveryPolicy policy;
    MotorRecoveryState state;
    MotorRecoveryVendorSubstate vendor_substate;
    OmBool degraded_flag;
    int32_t last_recover_ret;
    OsalTimeMs last_recover_ms;
    OsalTimeMs offline_since_ms;
    OsalTimeMs observe_gate_until_ms;
    OsalTimeMs damiao_mode_settle_until_ms;
    OsalTimeMs p1010b_enable_settle_until_ms;
    OsalTimeMs p1010b_last_query_ok_ms;
    OmBool damiao_enable_completed;
    uint32_t damiao_enable_sequence_baseline;
    uint32_t recover_count;
} MotorRecoveryEntry;

/* 模块上下文：
 * - entries：所有需要被自动恢复监督的电机
 * - runtime_fault_active：当前是否已经置位 263，用来做清故障的边沿判断
 */
typedef struct
{
    MotorRecoveryEntry entries[MOTOR_RECOVERY_CAPACITY];
    uint32_t entry_count;
    OmBool runtime_fault_active;
    OsalTimeMs damiao_can_restart_not_before_ms;
    OsalTimeMs damiao_can_last_restart_ms;
    uint32_t damiao_can_restart_count;
    OsalTimeMs can1_restart_not_before_ms;
    OsalTimeMs can1_last_restart_ms;
    uint32_t can1_restart_count;
} MotorRecoveryContext;

static MotorRecoveryContext g_motor_recovery = {0};

static OmBool motor_recovery_is_p1010b_rx_recent(
    const MotorRecoveryEntry* entry,
    uint32_t now_ms,
    uint32_t timeout_ms);

/* P1010B 恢复统一走 query-mode。
 * 这样运行期在线判据不依赖主动上报链，而是和现有正式通信逻辑保持一致。
 */
static P1010BActiveReportConfig motor_recovery_make_p1010b_active_report_config(void)
{
    return (P1010BActiveReportConfig){
        .enable = true,
        .periodMs = MOTOR_RECOVERY_P1010B_REPORT_PERIOD_MS,
        .dataTypeSlots = {
            (uint8_t)P1010B_REPORT_DATA_ABSOLUTE_POSITION,
            (uint8_t)P1010B_REPORT_DATA_SPEED_RPM,
            (uint8_t)P1010B_REPORT_DATA_IQ_AMPERE,
            (uint8_t)P1010B_REPORT_DATA_BUS_VOLTAGE,
        },
    };
}

/* 所有条目默认共享同一恢复策略。 */
static MotorRecoveryPolicy motor_recovery_make_default_policy(void)
{
    return (MotorRecoveryPolicy){
        .online_timeout_ms = APP_MOTOR_RECOVERY_ONLINE_TIMEOUT_MS,
        .retry_interval_ms = APP_MOTOR_RECOVERY_RETRY_INTERVAL_MS,
        .fault_debounce_ms = APP_MOTOR_RECOVERY_FAULT_DEBOUNCE_MS,
    };
}

static MotorRecoveryPolicy motor_recovery_make_policy_for_vendor(MotorVendor vendor)
{
    MotorRecoveryPolicy policy = motor_recovery_make_default_policy();

    if (vendor == MOTOR_VENDOR_P1010B)
    {
        policy.fault_debounce_ms = MOTOR_RECOVERY_P1010B_FAULT_DEBOUNCE_MS;
    }

    return policy;
}

/* vendor 默认子状态只表达“首次恢复时从哪一步开始”。 */
static MotorRecoveryVendorSubstate motor_recovery_get_default_vendor_substate(MotorVendor vendor)
{
    switch (vendor)
    {
    case MOTOR_VENDOR_P1010B:
        return MOTOR_RECOVERY_VENDOR_SUBSTATE_P1010B_DISABLE;
    case MOTOR_VENDOR_DAMIAO:
        return MOTOR_RECOVERY_VENDOR_SUBSTATE_DAMIAO_WRITE_MIT;
    default:
        return MOTOR_RECOVERY_VENDOR_SUBSTATE_NONE;
    }
}

static const MotorRecoveryEntry* motor_recovery_find_entry_by_motor(const Motor* motor)
{
    uint32_t index = 0u;

    if (motor == OM_NULL)
    {
        return OM_NULL;
    }

    for (index = 0u; index < g_motor_recovery.entry_count; index++)
    {
        if (g_motor_recovery.entries[index].motor == motor)
        {
            return &g_motor_recovery.entries[index];
        }
    }

    return OM_NULL;
}

static DamiaoMotorBus* motor_recovery_get_damiao_bus(const MotorRecoveryEntry* entry);
static OmBool motor_recovery_damiao_group_has_unhealthy_peer(const MotorRecoveryEntry* entry);
static Device* motor_recovery_get_can1_device(const MotorRecoveryEntry* entry);
static OmBool motor_recovery_are_can1_p1010b_healthy(const Device* can_dev);
static OmBool motor_recovery_is_can1_dji_group_idle(const Device* can_dev);
static OmBool motor_recovery_is_can1_dji_group_leader(const MotorRecoveryEntry* entry, const Device* can_dev);

static MotorRecoveryEntry* motor_recovery_find_mutable_entry_by_motor(Motor* motor)
{
    return (MotorRecoveryEntry*)motor_recovery_find_entry_by_motor(motor);
}

static void motor_recovery_set_regular_target_blocked(MotorRecoveryEntry* entry, OmBool blocked)
{
    if (entry == OM_NULL || entry->motor == OM_NULL)
    {
        return;
    }

    entry->motor->regular_target_blocked = blocked;
}

static void motor_recovery_set_damiao_group_regular_target_blocked(
    const MotorRecoveryEntry* entry,
    OmBool blocked)
{
    DamiaoMotorBus* bus = OM_NULL;
    uint32_t index = 0u;

    bus = motor_recovery_get_damiao_bus(entry);
    if (bus == OM_NULL)
    {
        return;
    }

    for (index = 0u; index < g_motor_recovery.entry_count; index++)
    {
        MotorRecoveryEntry* peer = &g_motor_recovery.entries[index];
        if (peer->vendor != MOTOR_VENDOR_DAMIAO ||
            motor_recovery_get_damiao_bus(peer) != bus)
        {
            continue;
        }

        motor_recovery_set_regular_target_blocked(peer, blocked);
    }
}

static void motor_recovery_reset_damiao_cycle(MotorRecoveryEntry* entry)
{
    if (entry == OM_NULL || entry->vendor != MOTOR_VENDOR_DAMIAO)
    {
        return;
    }

    entry->damiao_enable_completed = OM_FALSE;
    entry->damiao_enable_sequence_baseline = 0u;
    entry->damiao_mode_settle_until_ms = 0u;
    entry->observe_gate_until_ms = 0u;
    entry->vendor_substate = MOTOR_RECOVERY_VENDOR_SUBSTATE_DAMIAO_WRITE_MIT;
    motor_recovery_set_damiao_group_regular_target_blocked(entry, OM_TRUE);
}

/* 编译期开关，方便整套恢复逻辑一键裁掉。 */
static OmBool motor_recovery_enabled(void)
{
    return (APP_MOTOR_AUTO_RECOVERY_ENABLE != 0u) ? OM_TRUE : OM_FALSE;
}

/* 在线判据在恢复模块里重新计算一次，而不是直接复用 motor->feedback.online。
 * 这样恢复模块只依赖“最近有效反馈时间戳”，不受别的上层语义耦合。
 */
static OmBool motor_recovery_is_entry_online(const MotorRecoveryEntry* entry)
{
    uint32_t now_ms = 0u;
    uint32_t timeout_ms = 0u;

    if (entry == OM_NULL || entry->motor == OM_NULL || entry->driver == OM_NULL)
    {
        return OM_FALSE;
    }

    now_ms = osal_time_now_monotonic();
    timeout_ms = entry->policy.online_timeout_ms;

    switch (entry->vendor)
    {
    case MOTOR_VENDOR_DJI:
        return (dji_motor_get_feedback_timestamp_ms((DJIMotorDrv*)entry->driver) != 0u &&
                (uint32_t)(now_ms - dji_motor_get_feedback_timestamp_ms((DJIMotorDrv*)entry->driver)) <= timeout_ms) ?
                   OM_TRUE :
                   OM_FALSE;

    case MOTOR_VENDOR_DAMIAO:
        return (damiao_motor_get_feedback_sequence((DamiaoMotorDrv*)entry->driver) != 0u &&
                damiao_motor_get_feedback_timestamp_ms((DamiaoMotorDrv*)entry->driver) != 0u &&
                (uint32_t)(now_ms - damiao_motor_get_feedback_timestamp_ms((DamiaoMotorDrv*)entry->driver)) <= timeout_ms) ?
                   OM_TRUE :
                   OM_FALSE;

    case MOTOR_VENDOR_P1010B:
        return ((entry->p1010b_last_query_ok_ms != 0u &&
                 (uint32_t)(now_ms - entry->p1010b_last_query_ok_ms) <= timeout_ms) ||
                motor_recovery_is_p1010b_rx_recent(entry, now_ms, timeout_ms) == OM_TRUE) ?
                   OM_TRUE :
                   OM_FALSE;

    case MOTOR_VENDOR_GO8010:
        return go8010_is_online((Go8010MotorDrv*)entry->driver, timeout_ms);

    default:
        return OM_FALSE;
    }
}

/* P1010B 除了要在线，还必须真的处于 ENABLED 且没有 fault code。 */
static OmBool motor_recovery_is_p1010b_healthy(const MotorRecoveryEntry* entry)
{
    P1010BDriver* driver = OM_NULL;

    if (entry == OM_NULL || entry->driver == OM_NULL)
    {
        return OM_FALSE;
    }

    driver = (P1010BDriver*)entry->driver;
    if (driver->runtime.state != P1010B_STATE_ENABLED)
    {
        return OM_FALSE;
    }

    if (driver->telemetry.faultState.faultCode != 0u)
    {
        return OM_FALSE;
    }

    return motor_recovery_is_entry_online(entry);
}

static OmBool motor_recovery_is_damiao_enabled_status(const MotorRecoveryEntry* entry)
{
    uint8_t status = 0u;

    if (entry == OM_NULL || entry->driver == OM_NULL)
    {
        return OM_FALSE;
    }

    status = damiao_motor_get_status((DamiaoMotorDrv*)entry->driver);
    return (status == 1u) ? OM_TRUE : OM_FALSE;
}

/* vendor 无关的健康判定入口。 */
static OmBool motor_recovery_is_p1010b_rx_recent(
    const MotorRecoveryEntry* entry,
    uint32_t now_ms,
    uint32_t timeout_ms)
{
    const P1010BDriver* driver = OM_NULL;
    uint32_t last_rx_ms = 0u;

    if (entry == OM_NULL || entry->driver == OM_NULL)
    {
        return OM_FALSE;
    }

    driver = (const P1010BDriver*)entry->driver;
    last_rx_ms = driver->telemetry.lastSuccessRxTimestampMs;
    return (last_rx_ms != 0u && (uint32_t)(now_ms - last_rx_ms) <= timeout_ms) ? OM_TRUE : OM_FALSE;
}

static OmBool motor_recovery_is_entry_healthy(const MotorRecoveryEntry* entry)
{
    if (entry == OM_NULL)
    {
        return OM_FALSE;
    }

    switch (entry->vendor)
    {
    case MOTOR_VENDOR_P1010B:
        return motor_recovery_is_p1010b_healthy(entry);
    case MOTOR_VENDOR_GO8010:
    case MOTOR_VENDOR_DJI:
        return motor_recovery_is_entry_online(entry);
    case MOTOR_VENDOR_DAMIAO:
        return (motor_recovery_is_entry_online(entry) == OM_TRUE &&
                entry->damiao_enable_completed == OM_TRUE &&
                motor_recovery_is_damiao_enabled_status(entry) == OM_TRUE) ?
                   OM_TRUE :
                   OM_FALSE;
    default:
        return OM_FALSE;
    }
}

/* 记录一次恢复尝试的结果与时间，用于上层观测与下一轮节流。 */
static void motor_recovery_mark_attempt(MotorRecoveryEntry* entry, OmRet ret, OsalTimeMs now_ms)
{
    if (entry == OM_NULL)
    {
        return;
    }

    entry->recover_count++;
    entry->last_recover_ret = (int32_t)ret;
    entry->last_recover_ms = now_ms;
}

/* 一旦恢复健康，清掉 degraded 相关状态，并把 vendor 子状态复位。 */
static void motor_recovery_mark_healthy(MotorRecoveryEntry* entry)
{
    if (entry == OM_NULL)
    {
        return;
    }

    entry->state = MOTOR_RECOVERY_STATE_HEALTHY;
    entry->degraded_flag = OM_FALSE;
    entry->offline_since_ms = 0u;
    entry->damiao_mode_settle_until_ms = 0u;
    entry->p1010b_enable_settle_until_ms = 0u;
    entry->observe_gate_until_ms = 0u;
    entry->damiao_enable_sequence_baseline = 0u;
    entry->vendor_substate = motor_recovery_get_default_vendor_substate(entry->vendor);
    if (entry->vendor == MOTOR_VENDOR_DAMIAO)
    {
        motor_recovery_set_damiao_group_regular_target_blocked(
            entry,
            (motor_recovery_damiao_group_has_unhealthy_peer(entry) == OM_TRUE) ? OM_TRUE : OM_FALSE);
    }
    else
    {
        motor_recovery_set_regular_target_blocked(entry, OM_FALSE);
    }
}

static OmBool motor_recovery_is_p1010b_in_enable_settle_window(
    const MotorRecoveryEntry* entry,
    OsalTimeMs now_ms)
{
    if (entry == OM_NULL || entry->vendor != MOTOR_VENDOR_P1010B)
    {
        return OM_FALSE;
    }

    return osal_time_before(now_ms, entry->p1010b_enable_settle_until_ms);
}

static OmBool motor_recovery_can_fast_step_p1010b(const MotorRecoveryEntry* entry)
{
    if (entry == OM_NULL || entry->vendor != MOTOR_VENDOR_P1010B)
    {
        return OM_FALSE;
    }

    return (entry->vendor_substate != MOTOR_RECOVERY_VENDOR_SUBSTATE_P1010B_DISABLE &&
            entry->last_recover_ret == OM_OK) ? OM_TRUE : OM_FALSE;
}

static OmBool motor_recovery_is_damiao_in_mode_settle_window(
    const MotorRecoveryEntry* entry,
    OsalTimeMs now_ms)
{
    if (entry == OM_NULL || entry->vendor != MOTOR_VENDOR_DAMIAO)
    {
        return OM_FALSE;
    }

    return (entry->vendor_substate == MOTOR_RECOVERY_VENDOR_SUBSTATE_DAMIAO_WAIT_MIT_SETTLE &&
            osal_time_before(now_ms, entry->damiao_mode_settle_until_ms)) ?
               OM_TRUE :
               OM_FALSE;
}

static OmBool motor_recovery_is_damiao_in_enable_settle_window(
    const MotorRecoveryEntry* entry,
    OsalTimeMs now_ms)
{
    if (entry == OM_NULL || entry->vendor != MOTOR_VENDOR_DAMIAO)
    {
        return OM_FALSE;
    }

    return (entry->vendor_substate == MOTOR_RECOVERY_VENDOR_SUBSTATE_DAMIAO_WAIT_ENABLE_SETTLE &&
            osal_time_before(now_ms, entry->observe_gate_until_ms)) ?
               OM_TRUE :
               OM_FALSE;
}

static OmBool motor_recovery_is_damiao_in_feedback_probe_window(
    const MotorRecoveryEntry* entry,
    OsalTimeMs now_ms)
{
    if (entry == OM_NULL || entry->vendor != MOTOR_VENDOR_DAMIAO)
    {
        return OM_FALSE;
    }

    return (entry->vendor_substate == MOTOR_RECOVERY_VENDOR_SUBSTATE_DAMIAO_WAIT_FEEDBACK_PROBE &&
            osal_time_before(now_ms, entry->observe_gate_until_ms)) ? OM_TRUE : OM_FALSE;
}

static DamiaoMotorBus* motor_recovery_get_damiao_bus(const MotorRecoveryEntry* entry)
{
    if (entry == OM_NULL || entry->vendor != MOTOR_VENDOR_DAMIAO || entry->motor == OM_NULL)
    {
        return OM_NULL;
    }

    return entry->motor->binding.damiao.bus;
}

static Device* motor_recovery_get_can1_device(const MotorRecoveryEntry* entry)
{
    if (entry == OM_NULL || entry->motor == OM_NULL)
    {
        return OM_NULL;
    }

    switch (entry->vendor)
    {
    case MOTOR_VENDOR_DJI:
        return (entry->motor->binding.dji.bus != OM_NULL) ? entry->motor->binding.dji.bus->canDev : OM_NULL;
    case MOTOR_VENDOR_P1010B:
        return (entry->motor->binding.p1010b.bus != OM_NULL) ? entry->motor->binding.p1010b.bus->canDevice : OM_NULL;
    default:
        return OM_NULL;
    }
}

static OmBool motor_recovery_damiao_group_has_unhealthy_peer(const MotorRecoveryEntry* entry)
{
    DamiaoMotorBus* bus = OM_NULL;
    uint32_t index = 0u;

    bus = motor_recovery_get_damiao_bus(entry);
    if (bus == OM_NULL)
    {
        return OM_FALSE;
    }

    for (index = 0u; index < g_motor_recovery.entry_count; index++)
    {
        const MotorRecoveryEntry* peer = &g_motor_recovery.entries[index];
        if (peer->vendor != MOTOR_VENDOR_DAMIAO ||
            motor_recovery_get_damiao_bus(peer) != bus)
        {
            continue;
        }

        if (motor_recovery_is_entry_healthy(peer) != OM_TRUE)
        {
            return OM_TRUE;
        }
    }

    return OM_FALSE;
}

static OmBool motor_recovery_is_damiao_enable_feedback_advanced(const MotorRecoveryEntry* entry)
{
    uint32_t current_sequence = 0u;

    if (entry == OM_NULL || entry->vendor != MOTOR_VENDOR_DAMIAO || entry->driver == OM_NULL)
    {
        return OM_FALSE;
    }

    current_sequence = damiao_motor_get_feedback_sequence((DamiaoMotorDrv*)entry->driver);
    return (current_sequence != 0u && current_sequence != entry->damiao_enable_sequence_baseline) ? OM_TRUE : OM_FALSE;
}

static OmBool motor_recovery_is_damiao_group_pending(const MotorRecoveryEntry* entry, const DamiaoMotorBus* bus)
{
    if (entry == OM_NULL || bus == OM_NULL || entry->vendor != MOTOR_VENDOR_DAMIAO || entry->motor == OM_NULL)
    {
        return OM_FALSE;
    }

    return (entry->motor->binding.damiao.bus == bus &&
            motor_recovery_is_entry_healthy(entry) != OM_TRUE) ?
               OM_TRUE :
               OM_FALSE;
}

static OmBool motor_recovery_is_damiao_group_leader(
    const MotorRecoveryEntry* entry,
    const DamiaoMotorBus* bus)
{
    uint32_t index = 0u;

    if (entry == OM_NULL || bus == OM_NULL)
    {
        return OM_FALSE;
    }

    for (index = 0u; index < g_motor_recovery.entry_count; index++)
    {
        const MotorRecoveryEntry* peer = &g_motor_recovery.entries[index];

        if (motor_recovery_is_damiao_group_pending(peer, bus) != OM_TRUE)
        {
            continue;
        }

        return (peer == entry) ? OM_TRUE : OM_FALSE;
    }

    return OM_FALSE;
}

static OmBool motor_recovery_is_damiao_group_ready_for_enable(
    const MotorRecoveryEntry* entry,
    const DamiaoMotorBus* bus,
    OsalTimeMs now_ms)
{
    if (motor_recovery_is_damiao_group_pending(entry, bus) != OM_TRUE)
    {
        return OM_FALSE;
    }

    if (entry->vendor_substate == MOTOR_RECOVERY_VENDOR_SUBSTATE_DAMIAO_SEND_ENABLE)
    {
        return OM_TRUE;
    }

    if (entry->vendor_substate == MOTOR_RECOVERY_VENDOR_SUBSTATE_DAMIAO_WAIT_MIT_SETTLE &&
        osal_time_before(now_ms, entry->damiao_mode_settle_until_ms) != OM_TRUE)
    {
        return OM_TRUE;
    }

    return OM_FALSE;
}

static OmBool motor_recovery_is_damiao_group_ready_for_feedback_probe(
    const MotorRecoveryEntry* entry,
    const DamiaoMotorBus* bus)
{
    if (motor_recovery_is_damiao_group_pending(entry, bus) != OM_TRUE)
    {
        return OM_FALSE;
    }

    return (entry->vendor_substate == MOTOR_RECOVERY_VENDOR_SUBSTATE_DAMIAO_SEND_FEEDBACK_PROBE) ? OM_TRUE : OM_FALSE;
}

static float motor_recovery_get_damiao_probe_position(const MotorRecoveryEntry* entry)
{
    DamiaoMotorDrv* driver = OM_NULL;

    if (entry == OM_NULL || entry->driver == OM_NULL)
    {
        return 0.0f;
    }

    driver = (DamiaoMotorDrv*)entry->driver;
    if (damiao_motor_get_feedback_sequence(driver) == 0u)
    {
        return 0.0f;
    }

    return damiao_motor_get_position(driver);
}

static OmBool motor_recovery_is_can1_group_pending(const MotorRecoveryEntry* entry, const Device* can_dev)
{
    if (entry == OM_NULL || can_dev == OM_NULL)
    {
        return OM_FALSE;
    }

    if ((entry->vendor != MOTOR_VENDOR_DJI && entry->vendor != MOTOR_VENDOR_P1010B) ||
        motor_recovery_get_can1_device(entry) != can_dev)
    {
        return OM_FALSE;
    }

    return (motor_recovery_is_entry_healthy(entry) != OM_TRUE) ? OM_TRUE : OM_FALSE;
}

static OmBool motor_recovery_is_can1_group_leader(const MotorRecoveryEntry* entry, const Device* can_dev)
{
    uint32_t index = 0u;

    if (entry == OM_NULL || can_dev == OM_NULL)
    {
        return OM_FALSE;
    }

    for (index = 0u; index < g_motor_recovery.entry_count; index++)
    {
        const MotorRecoveryEntry* peer = &g_motor_recovery.entries[index];
        if (peer->vendor != MOTOR_VENDOR_P1010B)
        {
            continue;
        }

        if (motor_recovery_is_can1_group_pending(peer, can_dev) != OM_TRUE)
        {
            continue;
        }

        return (peer == entry) ? OM_TRUE : OM_FALSE;
    }

    return OM_FALSE;
}

static OmBool motor_recovery_are_can1_p1010b_healthy(const Device* can_dev)
{
    uint32_t index = 0u;
    OmBool found = OM_FALSE;

    if (can_dev == OM_NULL)
    {
        return OM_FALSE;
    }

    for (index = 0u; index < g_motor_recovery.entry_count; index++)
    {
        const MotorRecoveryEntry* entry = &g_motor_recovery.entries[index];
        if (entry->vendor != MOTOR_VENDOR_P1010B ||
            motor_recovery_get_can1_device(entry) != can_dev)
        {
            continue;
        }

        found = OM_TRUE;
        if (motor_recovery_is_entry_healthy(entry) != OM_TRUE)
        {
            return OM_FALSE;
        }
    }

    return found;
}

static OmBool motor_recovery_is_dji_entry_idle(const MotorRecoveryEntry* entry)
{
    const Motor* motor = OM_NULL;

    if (entry == OM_NULL || entry->vendor != MOTOR_VENDOR_DJI || entry->motor == OM_NULL)
    {
        return OM_FALSE;
    }

    motor = entry->motor;
    switch (motor->config.control_mode)
    {
    case MOTOR_CONTROL_MODE_CURRENT:
        return (motor->target_current <= MOTOR_RECOVERY_DJI_CAN1_RESTART_EPSILON &&
                motor->target_current >= -MOTOR_RECOVERY_DJI_CAN1_RESTART_EPSILON) ? OM_TRUE : OM_FALSE;
    case MOTOR_CONTROL_MODE_TORQUE:
        return (motor->target_torque <= MOTOR_RECOVERY_DJI_CAN1_RESTART_EPSILON &&
                motor->target_torque >= -MOTOR_RECOVERY_DJI_CAN1_RESTART_EPSILON) ? OM_TRUE : OM_FALSE;
    case MOTOR_CONTROL_MODE_DISABLED:
        return OM_TRUE;
    default:
        return OM_FALSE;
    }
}

static OmBool motor_recovery_is_can1_dji_group_idle(const Device* can_dev)
{
    uint32_t index = 0u;
    OmBool found = OM_FALSE;

    if (can_dev == OM_NULL)
    {
        return OM_FALSE;
    }

    for (index = 0u; index < g_motor_recovery.entry_count; index++)
    {
        const MotorRecoveryEntry* entry = &g_motor_recovery.entries[index];
        if (entry->vendor != MOTOR_VENDOR_DJI ||
            motor_recovery_get_can1_device(entry) != can_dev)
        {
            continue;
        }

        found = OM_TRUE;
        if (motor_recovery_is_dji_entry_idle(entry) != OM_TRUE)
        {
            return OM_FALSE;
        }
    }

    return found;
}

static OmBool motor_recovery_is_can1_dji_group_leader(const MotorRecoveryEntry* entry, const Device* can_dev)
{
    uint32_t index = 0u;

    if (entry == OM_NULL || can_dev == OM_NULL)
    {
        return OM_FALSE;
    }

    for (index = 0u; index < g_motor_recovery.entry_count; index++)
    {
        const MotorRecoveryEntry* peer = &g_motor_recovery.entries[index];
        if (peer->vendor != MOTOR_VENDOR_DJI ||
            motor_recovery_get_can1_device(peer) != can_dev)
        {
            continue;
        }

        if (motor_recovery_is_entry_healthy(peer) == OM_TRUE)
        {
            continue;
        }

        return (peer == entry) ? OM_TRUE : OM_FALSE;
    }

    return OM_FALSE;
}

static OmRet motor_recovery_restart_can1_bus(const MotorRecoveryEntry* entry, OsalTimeMs now_ms)
{
    Device* can_dev = OM_NULL;
    CanCfg can_cfg = CAN_DEFUALT_CFG;
    uint32_t io_type = 0u;
    OmRet ret = OM_OK;

    can_dev = motor_recovery_get_can1_device(entry);
    if (can_dev == OM_NULL)
    {
        return OM_ERROR_PARAM;
    }

    g_motor_recovery.can1_restart_count++;
    g_motor_recovery.can1_last_restart_ms = now_ms;
    g_motor_recovery.can1_restart_not_before_ms =
        now_ms + MOTOR_RECOVERY_CAN1_RESTART_THROTTLE_MS;

    ret = device_ctrl(can_dev, CAN_CMD_SUSPEND, OM_NULL);
    if (ret != OM_OK)
    {
        return ret;
    }

    (void)device_ctrl(can_dev, CAN_CMD_FLUSH, OM_NULL);

    can_cfg.normalTimeCfg.baudRate = CAN_BAUD_1M;
    ret = device_ctrl(can_dev, CAN_CMD_CFG, &can_cfg);
    if (ret != OM_OK)
    {
        return ret;
    }

    io_type = CAN_REG_INT_RX;
    ret = device_ctrl(can_dev, CAN_CMD_SET_IOTYPE, &io_type);
    if (ret != OM_OK)
    {
        return ret;
    }

    io_type = CAN_REG_INT_TX;
    ret = device_ctrl(can_dev, CAN_CMD_SET_IOTYPE, &io_type);
    if (ret != OM_OK)
    {
        return ret;
    }

    ret = device_ctrl(can_dev, CAN_CMD_START, OM_NULL);
    if (ret != OM_OK)
    {
        return ret;
    }

    (void)device_ctrl(can_dev, CAN_CMD_FLUSH, OM_NULL);
    return OM_OK;
}

/* 晚供电场景里，reset 之所以稳定，是因为 CAN2 外设也一起被拉回了干净状态。
 * 这里在应用层补一条等价路径：stop/flush/re-config/start，再继续达妙 bring-up。 */
static OmRet motor_recovery_restart_damiao_can_bus(
    const MotorRecoveryEntry* entry,
    OsalTimeMs now_ms)
{
    DamiaoMotorBus* bus = OM_NULL;
    Device* can_dev = OM_NULL;
    CanCfg can_cfg = CAN_DEFUALT_CFG;
    uint32_t io_type = 0u;
    OmRet ret = OM_OK;

    if (entry == OM_NULL)
    {
        return OM_ERROR_PARAM;
    }

    bus = motor_recovery_get_damiao_bus(entry);
    if (bus == OM_NULL || bus->canDev == OM_NULL)
    {
        return OM_ERROR_PARAM;
    }

    can_dev = bus->canDev;
    g_motor_recovery.damiao_can_restart_count++;
    g_motor_recovery.damiao_can_last_restart_ms = now_ms;
    g_motor_recovery.damiao_can_restart_not_before_ms =
        now_ms + MOTOR_RECOVERY_DAMIAO_CAN_RESTART_THROTTLE_MS;

    ret = device_ctrl(can_dev, CAN_CMD_SUSPEND, OM_NULL);
    if (ret != OM_OK)
    {
        return ret;
    }

    (void)device_ctrl(can_dev, CAN_CMD_FLUSH, OM_NULL);

    can_cfg.normalTimeCfg.baudRate = CAN_BAUD_1M;
    ret = device_ctrl(can_dev, CAN_CMD_CFG, &can_cfg);
    if (ret != OM_OK)
    {
        return ret;
    }

    io_type = CAN_REG_INT_RX;
    ret = device_ctrl(can_dev, CAN_CMD_SET_IOTYPE, &io_type);
    if (ret != OM_OK)
    {
        return ret;
    }

    io_type = CAN_REG_INT_TX;
    ret = device_ctrl(can_dev, CAN_CMD_SET_IOTYPE, &io_type);
    if (ret != OM_OK)
    {
        return ret;
    }

    ret = device_ctrl(can_dev, CAN_CMD_START, OM_NULL);
    if (ret != OM_OK)
    {
        return ret;
    }

    (void)device_ctrl(can_dev, CAN_CMD_FLUSH, OM_NULL);
    return OM_OK;
}

/* 当前是否到了允许再次重试的时间窗口。 */
static OmBool motor_recovery_retry_due(const MotorRecoveryEntry* entry, OsalTimeMs now_ms)
{
    if (entry == OM_NULL)
    {
        return OM_FALSE;
    }

    return ((uint32_t)(now_ms - entry->last_recover_ms) >= entry->policy.retry_interval_ms) ? OM_TRUE : OM_FALSE;
}

static OmBool motor_recovery_entry_affects_runtime_fault(const MotorRecoveryEntry* entry)
{
    return (entry != OM_NULL) ? OM_TRUE : OM_FALSE;
}

/* 离线进入恢复后，先走去抖，再升级为 DEGRADED。
 * 这里的 degraded 语义直接对应 263 的 runtime fault。
 */
static void motor_recovery_update_state(MotorRecoveryEntry* entry, OsalTimeMs now_ms)
{
    if (entry == OM_NULL)
    {
        return;
    }

    if (entry->offline_since_ms == 0u)
    {
        entry->offline_since_ms = now_ms;
    }

    if ((uint32_t)(now_ms - entry->offline_since_ms) >= entry->policy.fault_debounce_ms)
    {
        entry->degraded_flag = OM_TRUE;
        entry->state = MOTOR_RECOVERY_STATE_DEGRADED;
    }
    else
    {
        entry->degraded_flag = OM_FALSE;
        entry->state = MOTOR_RECOVERY_STATE_RECOVERING;
    }
}

/* P1010B 恢复是固定四步同步状态机：
 * disable -> set_mode -> set_active_report -> enable
 */
static OmRet motor_recovery_recover_p1010b(MotorRecoveryEntry* entry, OsalTimeMs now_ms)
{
    P1010BDriver* driver = OM_NULL;
    Device* can_dev = OM_NULL;
    P1010BResponse response = {0};
    OmRet ret = OM_OK;

    if (entry == OM_NULL || entry->driver == OM_NULL)
    {
        return OM_ERROR_PARAM;
    }

    driver = (P1010BDriver*)entry->driver;
    can_dev = motor_recovery_get_can1_device(entry);
    if (can_dev != OM_NULL &&
        motor_recovery_is_can1_group_leader(entry, can_dev) == OM_TRUE &&
        osal_time_before(now_ms, g_motor_recovery.can1_restart_not_before_ms) != OM_TRUE)
    {
        ret = motor_recovery_restart_can1_bus(entry, now_ms);
        if (ret != OM_OK)
        {
            motor_recovery_mark_attempt(entry, ret, now_ms);
            return ret;
        }
    }

    switch (entry->vendor_substate)
    {
    case MOTOR_RECOVERY_VENDOR_SUBSTATE_P1010B_DISABLE:
        motor_recovery_configure_p1010b_driver(driver);
        ret = p1010b_disable(driver, 0u, &response);
        if (ret == OM_OK)
        {
            entry->vendor_substate = MOTOR_RECOVERY_VENDOR_SUBSTATE_P1010B_SET_MODE;
        }
        break;
    case MOTOR_RECOVERY_VENDOR_SUBSTATE_P1010B_SET_MODE:
        ret = p1010b_set_mode(driver, driver->config.defaultMode, 0u, &response);
        if (ret == OM_OK)
        {
            entry->vendor_substate = MOTOR_RECOVERY_VENDOR_SUBSTATE_P1010B_SET_ACTIVE_REPORT;
        }
        break;
    case MOTOR_RECOVERY_VENDOR_SUBSTATE_P1010B_SET_ACTIVE_REPORT:
        ret = p1010b_set_active_report(driver, &driver->runtime.activeReport, 0u, &response);
        if (ret == OM_OK)
        {
            entry->vendor_substate = MOTOR_RECOVERY_VENDOR_SUBSTATE_P1010B_ENABLE;
        }
        break;
    case MOTOR_RECOVERY_VENDOR_SUBSTATE_P1010B_ENABLE:
        ret = p1010b_enable(driver, 0u, &response);
        if (ret == OM_OK)
        {
            entry->p1010b_enable_settle_until_ms = now_ms + MOTOR_RECOVERY_P1010B_ENABLE_SETTLE_MS;
            entry->vendor_substate = MOTOR_RECOVERY_VENDOR_SUBSTATE_P1010B_DISABLE;
        }
        break;
    default:
        entry->vendor_substate = MOTOR_RECOVERY_VENDOR_SUBSTATE_P1010B_DISABLE;
        ret = OM_ERROR_PARAM;
        break;
    }

    motor_recovery_mark_attempt(entry, ret, now_ms);
    return ret;
}

/* 达妙恢复需要覆盖“电机晚于主控上电”的场景。
 * 这时只发 enable 不够，必须先重写 MIT 模式，再在下一轮重试里真正 enable。
 */
static OmRet motor_recovery_recover_damiao(MotorRecoveryEntry* entry, OsalTimeMs now_ms)
{
    DamiaoMotorBus* bus = OM_NULL;
    uint32_t index = 0u;
    uint32_t success_count = 0u;
    OmRet ret = OM_OK;
    OmRet group_ret = OM_OK;

    if (entry == OM_NULL || entry->driver == OM_NULL || entry->motor == OM_NULL)
    {
        return OM_ERROR_PARAM;
    }

    bus = motor_recovery_get_damiao_bus(entry);
    motor_recovery_set_regular_target_blocked(entry, OM_TRUE);

    switch (entry->vendor_substate)
    {
    case MOTOR_RECOVERY_VENDOR_SUBSTATE_DAMIAO_WRITE_MIT:
        if (bus == OM_NULL || bus->canDev == OM_NULL)
        {
            ret = OM_ERROR_PARAM;
            break;
        }

        if (motor_recovery_is_damiao_group_leader(entry, bus) == OM_TRUE &&
            osal_time_before(now_ms, g_motor_recovery.damiao_can_restart_not_before_ms) != OM_TRUE)
        {
            ret = motor_recovery_restart_damiao_can_bus(entry, now_ms);
            if (ret != OM_OK)
            {
                break;
            }
        }

        for (index = 0u; index < g_motor_recovery.entry_count; index++)
        {
            MotorRecoveryEntry* peer = &g_motor_recovery.entries[index];
            DamiaoMotorDrv* peer_driver = OM_NULL;

            if (motor_recovery_is_damiao_group_pending(peer, bus) != OM_TRUE)
            {
                continue;
            }

            peer_driver = (DamiaoMotorDrv*)peer->driver;
            if (peer_driver == OM_NULL)
            {
                ret = OM_ERROR_PARAM;
                continue;
            }

            group_ret = damiao_motor_write_register_u32(
                bus->canDev,
                peer_driver->link.txId,
                MOTOR_RECOVERY_DAMIAO_CTRL_MODE_RID,
                MOTOR_RECOVERY_DAMIAO_CTRL_MODE_MIT);
            if (group_ret == OM_OK)
            {
                peer->damiao_enable_completed = OM_FALSE;
                peer->damiao_enable_sequence_baseline = 0u;
                peer->damiao_mode_settle_until_ms = now_ms + MOTOR_RECOVERY_DAMIAO_MODE_SETTLE_MS;
                peer->vendor_substate = MOTOR_RECOVERY_VENDOR_SUBSTATE_DAMIAO_WAIT_MIT_SETTLE;
                motor_recovery_set_regular_target_blocked(peer, OM_TRUE);
                success_count++;
            }
            else
            {
                ret = group_ret;
            }
        }

        if (success_count > 0u)
        {
            ret = OM_OK;
        }
        else if (ret == OM_OK)
        {
            ret = OM_ERROR;
        }
        break;

    case MOTOR_RECOVERY_VENDOR_SUBSTATE_DAMIAO_SEND_ENABLE:
        for (index = 0u; index < g_motor_recovery.entry_count; index++)
        {
            MotorRecoveryEntry* peer = &g_motor_recovery.entries[index];
            DamiaoMotorDrv* peer_driver = OM_NULL;

            if (motor_recovery_is_damiao_group_ready_for_enable(peer, bus, now_ms) != OM_TRUE)
            {
                continue;
            }

            peer_driver = (DamiaoMotorDrv*)peer->driver;
            if (peer_driver == OM_NULL)
            {
                ret = OM_ERROR_PARAM;
                continue;
            }

            damiao_motor_enable(peer_driver);
            motor_recovery_notify_damiao_enabled(peer->motor);
            peer->vendor_substate = MOTOR_RECOVERY_VENDOR_SUBSTATE_DAMIAO_WAIT_ENABLE_SETTLE;
            success_count++;
        }

        if (success_count == 0u)
        {
            ret = OM_ERROR;
            break;
        }

        damiao_motor_bus_sync(bus);
        ret = OM_OK;
        break;

    case MOTOR_RECOVERY_VENDOR_SUBSTATE_DAMIAO_SEND_FEEDBACK_PROBE:
        for (index = 0u; index < g_motor_recovery.entry_count; index++)
        {
            MotorRecoveryEntry* peer = &g_motor_recovery.entries[index];
            DamiaoMotorDrv* peer_driver = OM_NULL;

            if (motor_recovery_is_damiao_group_ready_for_feedback_probe(peer, bus) != OM_TRUE)
            {
                continue;
            }

            peer_driver = (DamiaoMotorDrv*)peer->driver;
            if (peer_driver == OM_NULL)
            {
                ret = OM_ERROR_PARAM;
                continue;
            }

            damiao_motor_set_mit(
                peer_driver,
                motor_recovery_get_damiao_probe_position(peer),
                0.0f,
                0.0f,
                0.0f,
                0.0f);
            success_count++;
        }

        if (success_count == 0u)
        {
            ret = OM_ERROR;
            break;
        }

        damiao_motor_bus_sync(bus);

        for (index = 0u; index < g_motor_recovery.entry_count; index++)
        {
            MotorRecoveryEntry* peer = &g_motor_recovery.entries[index];

            if (motor_recovery_is_damiao_group_ready_for_feedback_probe(peer, bus) != OM_TRUE)
            {
                continue;
            }

            peer->observe_gate_until_ms = now_ms + MOTOR_RECOVERY_DAMIAO_FEEDBACK_PROBE_SETTLE_MS;
            peer->vendor_substate = MOTOR_RECOVERY_VENDOR_SUBSTATE_DAMIAO_WAIT_FEEDBACK_PROBE;
            motor_recovery_set_regular_target_blocked(peer, OM_TRUE);
        }
        ret = OM_OK;
        break;

    case MOTOR_RECOVERY_VENDOR_SUBSTATE_DAMIAO_WAIT_MIT_SETTLE:
    case MOTOR_RECOVERY_VENDOR_SUBSTATE_DAMIAO_WAIT_ENABLE_SETTLE:
    case MOTOR_RECOVERY_VENDOR_SUBSTATE_DAMIAO_WAIT_FEEDBACK_PROBE:
        ret = OM_ERROR_BUSY;
        break;

    default:
        motor_recovery_reset_damiao_cycle(entry);
        ret = OM_ERROR_PARAM;
        break;
    }

    motor_recovery_mark_attempt(entry, ret, now_ms);
    return ret;
}

/* DJI / GO8010 当前不重开链路，只重新下发当前 target。 */
static OmRet motor_recovery_recover_reassert_target(MotorRecoveryEntry* entry, OsalTimeMs now_ms)
{
    OmRet ret = OM_OK;

    if (entry == OM_NULL || entry->motor == OM_NULL)
    {
        return OM_ERROR_PARAM;
    }

    /* DJI 底盘轮与 P1010B 共用 CAN1。
     * 实测由单轮掉线触发的 CAN1 restart 会把两条后腿一并拖进 query timeout，
     * 但并不能稳定拉回缺反馈的单轮。这里先退回为“仅重发当前 target”，
     * 保住 CAN1 其余电机的稳定性；单轮无反馈再通过诊断区分为 wiring/ID/物理层问题。 */
    ret = motor_control_compute(entry->motor);
    motor_recovery_mark_attempt(entry, ret, now_ms);
    return ret;
}

static OmRet motor_recovery_recover_go8010(MotorRecoveryEntry* entry, OsalTimeMs now_ms)
{
    Go8010Bus* bus = OM_NULL;
    OmRet ret = OM_OK;

    if (entry == OM_NULL || entry->motor == OM_NULL || entry->driver == OM_NULL)
    {
        return OM_ERROR_PARAM;
    }

    bus = entry->motor->binding.go8010.bus;
    if (bus == OM_NULL)
    {
        return OM_ERROR_PARAM;
    }

    ret = go8010_bus_recover(bus);
    if (ret == OM_OK)
    {
        ret = motor_control_compute(entry->motor);
        if (ret == OM_OK)
        {
            go8010_bus_sync(bus);
        }
    }

    motor_recovery_mark_attempt(entry, ret, now_ms);
    return ret;
}

/* 单条恢复推进逻辑：
 * 1. 先检查是否已经 healthy
 * 2. 不 healthy 则更新 state/degraded
 * 3. 到达重试时间窗口才执行 vendor 恢复动作
 */
static void motor_recovery_tick_entry(MotorRecoveryEntry* entry, OsalTimeMs now_ms)
{
    OmBool entry_online = OM_FALSE;
    OmBool damiao_retry_immediately = OM_FALSE;
    OmBool p1010b_retry_immediately = OM_FALSE;

    if (entry == OM_NULL)
    {
        return;
    }

    entry_online = motor_recovery_is_entry_online(entry);
    if (entry->vendor == MOTOR_VENDOR_DAMIAO &&
        entry_online != OM_TRUE &&
        entry->damiao_enable_completed == OM_TRUE)
    {
        motor_recovery_reset_damiao_cycle(entry);
    }

    if (motor_recovery_is_entry_healthy(entry) == OM_TRUE)
    {
        motor_recovery_mark_healthy(entry);
        return;
    }

    if (motor_recovery_is_p1010b_in_enable_settle_window(entry, now_ms) == OM_TRUE)
    {
        entry->state = MOTOR_RECOVERY_STATE_RECOVERING;
        entry->degraded_flag = OM_FALSE;
        entry->offline_since_ms = 0u;
        return;
    }

    motor_recovery_update_state(entry, now_ms);
    if (entry->vendor == MOTOR_VENDOR_DAMIAO)
    {
        if (motor_recovery_is_damiao_in_mode_settle_window(entry, now_ms) == OM_TRUE ||
            motor_recovery_is_damiao_in_enable_settle_window(entry, now_ms) == OM_TRUE ||
            motor_recovery_is_damiao_in_feedback_probe_window(entry, now_ms) == OM_TRUE)
        {
            return;
        }

        if (entry->vendor_substate == MOTOR_RECOVERY_VENDOR_SUBSTATE_DAMIAO_WAIT_MIT_SETTLE)
        {
            entry->vendor_substate = MOTOR_RECOVERY_VENDOR_SUBSTATE_DAMIAO_SEND_ENABLE;
            damiao_retry_immediately = OM_TRUE;
        }
        else if (entry->vendor_substate == MOTOR_RECOVERY_VENDOR_SUBSTATE_DAMIAO_WAIT_ENABLE_SETTLE)
        {
            if (entry_online == OM_TRUE &&
                motor_recovery_is_damiao_enable_feedback_advanced(entry) == OM_TRUE)
            {
                entry->damiao_enable_completed = OM_TRUE;
                motor_recovery_mark_healthy(entry);
                return;
            }

            entry->vendor_substate = MOTOR_RECOVERY_VENDOR_SUBSTATE_DAMIAO_SEND_FEEDBACK_PROBE;
            damiao_retry_immediately = OM_TRUE;
        }
        else if (entry->vendor_substate == MOTOR_RECOVERY_VENDOR_SUBSTATE_DAMIAO_WAIT_FEEDBACK_PROBE)
        {
            if (entry_online == OM_TRUE &&
                motor_recovery_is_damiao_enable_feedback_advanced(entry) == OM_TRUE)
            {
                entry->damiao_enable_completed = OM_TRUE;
                motor_recovery_mark_healthy(entry);
                return;
            }

            motor_recovery_reset_damiao_cycle(entry);
            damiao_retry_immediately = OM_TRUE;
        }
    }

    if (motor_recovery_can_fast_step_p1010b(entry) == OM_TRUE)
    {
        p1010b_retry_immediately = OM_TRUE;
    }

    if (motor_recovery_retry_due(entry, now_ms) != OM_TRUE)
    {
        if (damiao_retry_immediately != OM_TRUE &&
            p1010b_retry_immediately != OM_TRUE)
        {
            return;
        }
    }

    switch (entry->vendor)
    {
    case MOTOR_VENDOR_P1010B:
        (void)motor_recovery_recover_p1010b(entry, now_ms);
        break;
    case MOTOR_VENDOR_DAMIAO:
        (void)motor_recovery_recover_damiao(entry, now_ms);
        break;
    case MOTOR_VENDOR_DJI:
        (void)motor_recovery_recover_reassert_target(entry, now_ms);
        break;
    case MOTOR_VENDOR_GO8010:
        (void)motor_recovery_recover_go8010(entry, now_ms);
        break;
    default:
        break;
    }
}

/* 将所有 entry 的 degraded 状态聚合成 263 runtime fault。 */
static void motor_recovery_update_fault(void)
{
    OmBool any_degraded = OM_FALSE;
    uint32_t index = 0u;

    for (index = 0u; index < g_motor_recovery.entry_count; index++)
    {
        if (motor_recovery_entry_affects_runtime_fault(&g_motor_recovery.entries[index]) != OM_TRUE)
        {
            continue;
        }

        if (g_motor_recovery.entries[index].degraded_flag == OM_TRUE)
        {
            any_degraded = OM_TRUE;
            break;
        }
    }

    if (any_degraded == OM_TRUE)
    {
        (void)sh_report_runtime_fault(SH_ERR_MOTOR_RECOVERY_DEGRADED);
        g_motor_recovery.runtime_fault_active = OM_TRUE;
    }
    else if (g_motor_recovery.runtime_fault_active == OM_TRUE)
    {
        (void)sh_clear_runtime_fault(SH_ERR_MOTOR_RECOVERY_DEGRADED);
        g_motor_recovery.runtime_fault_active = OM_FALSE;
    }
}

void motor_recovery_reset(void)
{
    memset(&g_motor_recovery, 0, sizeof(g_motor_recovery));
}

/* 统一把恢复期要求写回 P1010B driver 配置。 */
void motor_recovery_configure_p1010b_driver(P1010BDriver* driver)
{
    if (driver == OM_NULL)
    {
        return;
    }

    driver->config.requestTimeoutMs = MOTOR_RECOVERY_P1010B_SYNC_TIMEOUT_MS;
    driver->config.maxRetryCount = MOTOR_RECOVERY_P1010B_MAX_RETRY_COUNT;
    driver->config.activeReport = motor_recovery_make_p1010b_active_report_config();
    driver->runtime.activeReport = driver->config.activeReport;
    driver->runtime.currentMode = driver->config.defaultMode;
}

/* 注册一台要参与自动恢复的电机。
 * 若宏关闭，则该函数静默返回 OM_OK，让正式通信任务无需分叉处理。
 */
OmRet motor_recovery_register_entry(
    const char* name,
    MotorVendor vendor,
    Motor* motor,
    void* driver)
{
    MotorRecoveryEntry* entry = OM_NULL;

    if (motor_recovery_enabled() != OM_TRUE)
    {
        return OM_OK;
    }

    if (name == OM_NULL || motor == OM_NULL || driver == OM_NULL)
    {
        return OM_ERROR_PARAM;
    }

    if (g_motor_recovery.entry_count >= MOTOR_RECOVERY_CAPACITY)
    {
        return OM_ERROR_MEMORY;
    }

    entry = &g_motor_recovery.entries[g_motor_recovery.entry_count++];
    memset(entry, 0, sizeof(*entry));
    entry->name = name;
    entry->vendor = vendor;
    entry->motor = motor;
    entry->driver = driver;
    entry->policy = motor_recovery_make_policy_for_vendor(vendor);
    entry->state = MOTOR_RECOVERY_STATE_RECOVERING;
    entry->vendor_substate = motor_recovery_get_default_vendor_substate(vendor);
    entry->last_recover_ret = OM_OK;

    if (vendor == MOTOR_VENDOR_DAMIAO)
    {
        /* 达妙 observation 静置窗口存放在 entry 私有字段里，
         * motor 只拿到一个 vendor_context 指针，不直接感知恢复模块状态。
         */
        motor_recovery_reset_damiao_cycle(entry);
        return motor_set_vendor_context(motor, &entry->observe_gate_until_ms);
    }

    return OM_OK;
}

/* 达妙每次 enable 后都需要重新打一个 settle gate。 */
void motor_recovery_notify_damiao_enabled(Motor* motor)
{
    OsalTimeMs* gate_until_ptr = OM_NULL;
    MotorRecoveryEntry* entry = OM_NULL;
    OsalTimeMs now_ms = 0u;

    if (motor_recovery_enabled() != OM_TRUE)
    {
        return;
    }

    if (motor == OM_NULL || motor->config.vendor_context == OM_NULL)
    {
        return;
    }

    now_ms = osal_time_now_monotonic();
    gate_until_ptr = (OsalTimeMs*)motor->config.vendor_context;
    *gate_until_ptr = now_ms + MOTOR_RECOVERY_DAMIAO_ENABLE_SETTLE_MS;

    entry = motor_recovery_find_mutable_entry_by_motor(motor);
    if (entry == OM_NULL || entry->vendor != MOTOR_VENDOR_DAMIAO)
    {
        return;
    }

    entry->damiao_enable_completed = OM_FALSE;
    entry->damiao_enable_sequence_baseline = damiao_motor_get_feedback_sequence((DamiaoMotorDrv*)entry->driver);
    entry->damiao_mode_settle_until_ms = 0u;
    entry->observe_gate_until_ms = *gate_until_ptr;
    entry->vendor_substate = MOTOR_RECOVERY_VENDOR_SUBSTATE_DAMIAO_WAIT_ENABLE_SETTLE;
    motor_recovery_set_regular_target_blocked(entry, OM_TRUE);
}

void motor_recovery_notify_p1010b_enabled(P1010BDriver* driver)
{
    uint32_t index = 0u;
    OsalTimeMs now_ms = 0u;

    if (motor_recovery_enabled() != OM_TRUE || driver == OM_NULL)
    {
        return;
    }

    now_ms = osal_time_now_monotonic();
    for (index = 0u; index < g_motor_recovery.entry_count; index++)
    {
        MotorRecoveryEntry* entry = &g_motor_recovery.entries[index];
        if (entry->vendor != MOTOR_VENDOR_P1010B || entry->driver != driver)
        {
            continue;
        }

        entry->p1010b_enable_settle_until_ms = now_ms + MOTOR_RECOVERY_P1010B_ENABLE_SETTLE_MS;
        entry->state = MOTOR_RECOVERY_STATE_RECOVERING;
        entry->degraded_flag = OM_FALSE;
        entry->offline_since_ms = 0u;
        break;
    }
}

void motor_recovery_notify_p1010b_query_ok(P1010BDriver* driver, OsalTimeMs timestamp_ms)
{
    uint32_t index = 0u;

    if (motor_recovery_enabled() != OM_TRUE || driver == OM_NULL || timestamp_ms == 0u)
    {
        return;
    }

    for (index = 0u; index < g_motor_recovery.entry_count; index++)
    {
        MotorRecoveryEntry* entry = &g_motor_recovery.entries[index];
        if (entry->vendor != MOTOR_VENDOR_P1010B || entry->driver != driver)
        {
            continue;
        }

        entry->p1010b_last_query_ok_ms = timestamp_ms;
        break;
    }
}

/* 启动期宽限：避免刚注册好就立刻进入第一次重试。 */
OmBool motor_recovery_should_defer_p1010b_query(const P1010BDriver* driver)
{
    OsalTimeMs now_ms = 0u;
    uint32_t index = 0u;

    if (motor_recovery_enabled() != OM_TRUE || driver == OM_NULL)
    {
        return OM_FALSE;
    }

    now_ms = osal_time_now_monotonic();
    for (index = 0u; index < g_motor_recovery.entry_count; index++)
    {
        const MotorRecoveryEntry* entry = &g_motor_recovery.entries[index];
        if (entry->vendor != MOTOR_VENDOR_P1010B || entry->driver != driver)
        {
            continue;
        }

        if (motor_recovery_is_p1010b_in_enable_settle_window(entry, now_ms) == OM_TRUE)
        {
            return OM_TRUE;
        }

        if (entry->state != MOTOR_RECOVERY_STATE_HEALTHY &&
            entry->vendor_substate != MOTOR_RECOVERY_VENDOR_SUBSTATE_P1010B_DISABLE)
        {
            return OM_TRUE;
        }

        return OM_FALSE;
    }

    return OM_FALSE;
}

void motor_recovery_arm_initial_grace(void)
{
    OsalTimeMs now_ms = 0u;
    uint32_t index = 0u;

    if (motor_recovery_enabled() != OM_TRUE)
    {
        return;
    }

    now_ms = osal_time_now_monotonic();
    for (index = 0u; index < g_motor_recovery.entry_count; index++)
    {
        g_motor_recovery.entries[index].last_recover_ms = now_ms;
    }
}

/* 模块主 tick。
 * 正式通信任务应在“本轮反馈已刷新”之后调用它。
 */
void motor_recovery_tick(void)
{
    OsalTimeMs now_ms = 0u;
    uint32_t index = 0u;

    if (motor_recovery_enabled() != OM_TRUE)
    {
        return;
    }

    now_ms = osal_time_now_monotonic();
    for (index = 0u; index < g_motor_recovery.entry_count; index++)
    {
        motor_recovery_tick_entry(&g_motor_recovery.entries[index], now_ms);
    }

    motor_recovery_update_fault();
}

OmBool motor_recovery_is_motor_ready(const Motor* motor)
{
    const MotorRecoveryEntry* entry = OM_NULL;
    const MotorFeedback* feedback = OM_NULL;

    if (motor == OM_NULL)
    {
        return OM_FALSE;
    }

    entry = motor_recovery_find_entry_by_motor(motor);
    if (entry != OM_NULL)
    {
        return motor_recovery_is_entry_healthy(entry);
    }

    feedback = motor_get_feedback(motor);
    return (feedback != OM_NULL && feedback->online == OM_TRUE) ? OM_TRUE : OM_FALSE;
}

OmBool motor_recovery_is_runtime_fault_active(void)
{
    return (g_motor_recovery.runtime_fault_active == OM_TRUE) ? OM_TRUE : OM_FALSE;
}

uint32_t motor_recovery_get_damiao_can_restart_count(void)
{
    return g_motor_recovery.damiao_can_restart_count;
}

/* 对外导出最小恢复快照，供 VOFA / 调试读取。 */
OmRet motor_recovery_copy_snapshots(
    MotorRecoverySnapshot* snapshots,
    uint32_t capacity,
    uint32_t* snapshot_count)
{
    uint32_t index = 0u;

    if (snapshots == OM_NULL || snapshot_count == OM_NULL)
    {
        return OM_ERROR_NULL;
    }

    *snapshot_count = 0u;
    if (capacity == 0u)
    {
        return OM_ERROR_PARAM;
    }

    if (motor_recovery_enabled() != OM_TRUE)
    {
        return OM_OK;
    }

    for (index = 0u; index < g_motor_recovery.entry_count; index++)
    {
        const MotorRecoveryEntry* entry = &g_motor_recovery.entries[index];

        if (*snapshot_count >= capacity)
        {
            break;
        }

        snapshots[*snapshot_count].name = entry->name;
        snapshots[*snapshot_count].vendor = (uint8_t)entry->vendor;
        snapshots[*snapshot_count].online = motor_recovery_is_entry_online(entry);
        snapshots[*snapshot_count].state = entry->state;
        snapshots[*snapshot_count].degraded_flag = entry->degraded_flag;
        snapshots[*snapshot_count].recover_count = entry->recover_count;
        snapshots[*snapshot_count].last_recover_ret = entry->last_recover_ret;
        snapshots[*snapshot_count].last_recover_ms = entry->last_recover_ms;
        (*snapshot_count)++;
    }

    return OM_OK;
}

OmRet motor_recovery_copy_p1010b_predicate_snapshots(
    MotorRecoveryP1010BPredicateSnapshot* snapshots,
    uint32_t capacity,
    uint32_t* snapshot_count)
{
    uint32_t index = 0u;

    if (snapshots == OM_NULL || snapshot_count == OM_NULL)
    {
        return OM_ERROR_NULL;
    }

    *snapshot_count = 0u;
    if (motor_recovery_enabled() == OM_FALSE)
    {
        return OM_OK;
    }

    for (index = 0u; index < g_motor_recovery.entry_count; index++)
    {
        const MotorRecoveryEntry* entry = &g_motor_recovery.entries[index];
        P1010BDriver* driver = OM_NULL;

        if (entry->vendor != MOTOR_VENDOR_P1010B)
        {
            continue;
        }

        if (*snapshot_count >= capacity)
        {
            break;
        }

        driver = (P1010BDriver*)entry->driver;
        snapshots[*snapshot_count].name = entry->name;
        snapshots[*snapshot_count].online = motor_recovery_is_entry_online(entry);
        snapshots[*snapshot_count].state_enabled =
            (driver != OM_NULL && driver->runtime.state == P1010B_STATE_ENABLED) ? OM_TRUE : OM_FALSE;
        snapshots[*snapshot_count].fault_clear =
            (driver != OM_NULL && driver->telemetry.faultState.faultCode == 0u) ? OM_TRUE : OM_FALSE;
        snapshots[*snapshot_count].healthy = motor_recovery_is_p1010b_healthy(entry);
        (*snapshot_count)++;
    }

    return OM_OK;
}
