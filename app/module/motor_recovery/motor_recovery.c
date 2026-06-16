#include "module/motor_recovery/motor_recovery.h"

#include "config/app_config.h"
#include "module/system_health/system_health.h"
#include "osal/osal_time.h"
#include <string.h>

#if (APP_MOTOR_AUTO_RECOVERY_ENABLE != 0u)

/* 该模块是 app 层的运行时自动恢复 owner：
 * - 不拥有物理总线
 * - 不直接启动任务
 * - 只维护“哪台电机离线、何时重试、何时报码”的状态机
 *
 * 真正的收发仍然由 motor_communications_task 调 motor_transmit_all()/motor_receive_all() 执行。
 */

#define MR_CAPACITY (MOTOR_REGISTRY_CAPACITY)
#define MR_DAMIAO_SETTLE_MS (50u)
#define MR_P1010B_SETTLE_MS (150u)
#define MR_P1010B_SYNC_MS (5u)
#define MR_P1010B_RETRY_MAX (0u)
#define MR_P1010B_REPORT_MS (APP_MR_P1010B_REPORT_MS)

/* 恢复子状态机只在模块内部使用：
 * - P1010B 需要多步同步恢复
 * - Damiao 当前只区分“等待 enable”与“正常 observation”
 */
typedef enum
{
    MR_VENDOR_SUBSTATE_NONE = 0u,
    MR_VENDOR_SUBSTATE_P1010B_DISABLE,
    MR_VENDOR_SUBSTATE_P1010B_SET_MODE,
    MR_VENDOR_SUBSTATE_P1010B_SET_ACTIVE_REPORT,
    MR_VENDOR_SUBSTATE_P1010B_ENABLE,
    MR_SUB_DAMIAO_EN_PENDING,
    MR_SUB_DAMIAO_OBS,
} MotorRecoverySubstate;

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
    Motor* motor;
    MotorRecoveryPolicy policy;
    MotorRecoveryState state;
    MotorRecoverySubstate vendor_substate;
    OmBool degraded_flag;
    int32_t last_recover_ret;
    OsalTimeMs last_recover_ms;
    OsalTimeMs offline_since_ms;
    OsalTimeMs observe_gate_until_ms;
    OsalTimeMs p1010b_enable_settle_until_ms;
    uint32_t recover_count;
} MotorRecoveryEntry;

/* 模块上下文：
 * - entries：所有需要被自动恢复监督的电机
 * - runtime_fault_active：当前是否已经置位 263，用来做清故障的边沿判断
 */
typedef struct
{
    MotorRecoveryEntry entries[MR_CAPACITY];
    uint32_t entry_count;
    OmBool runtime_fault_active;
    OsalTimeMs last_tick_ms;
} MotorRecoveryContext;

static MotorRecoveryContext g_motor_recovery = {0};

static MotorVendor motor_recovery_entry_vendor(const MotorRecoveryEntry* entry)
{
    return (entry != OM_NULL && entry->motor != OM_NULL) ?
               entry->motor->config.vendor :
               MOTOR_VENDOR_DJI;
}

static MotorRecoveryEntry* motor_recovery_find_mut(Motor* motor)
{
    uint32_t index = 0u;

    if (motor == OM_NULL)
    {
        return OM_NULL;
    }

    for (index = 0u; index < g_motor_recovery.entry_count; index++)
    {
        MotorRecoveryEntry* entry = &g_motor_recovery.entries[index];
        if (entry->motor == motor)
        {
            return entry;
        }
    }

    return OM_NULL;
}

static const MotorRecoveryEntry* motor_recovery_find(const Motor* motor)
{
    return motor_recovery_find_mut((Motor*)motor);
}

static P1010BDriver* motor_recovery_get_p1010b(const MotorRecoveryEntry* entry)
{
    if (entry == OM_NULL || entry->motor == OM_NULL ||
        entry->motor->config.vendor != MOTOR_VENDOR_P1010B)
    {
        return OM_NULL;
    }

    return entry->motor->binding.p1010b.driver;
}

static DamiaoMotorDrv* motor_recovery_get_damiao(const MotorRecoveryEntry* entry)
{
    if (entry == OM_NULL || entry->motor == OM_NULL ||
        entry->motor->config.vendor != MOTOR_VENDOR_DAMIAO)
    {
        return OM_NULL;
    }

    return entry->motor->binding.damiao.driver;
}

static DamiaoMotorBus* recovery_damiao_bus(const MotorRecoveryEntry* entry)
{
    if (entry == OM_NULL || entry->motor == OM_NULL ||
        entry->motor->config.vendor != MOTOR_VENDOR_DAMIAO)
    {
        return OM_NULL;
    }

    return entry->motor->binding.damiao.bus;
}

/* P1010B 运行期正式反馈改由 active report 承载。
 * 当前按规格书固定反馈顺序配置：
 * speed -> iq current -> bus voltage -> absolute position。
 */
static P1010BActiveReportConfig recovery_report_cfg(void)
{
    return (P1010BActiveReportConfig){
        .enable = true,
        .periodMs = MR_P1010B_REPORT_MS,
        .dataTypeSlots = {
            (uint8_t)P1010B_REPORT_DATA_SPEED_RPM,
            (uint8_t)P1010B_REPORT_DATA_IQ_AMPERE,
            (uint8_t)P1010B_REPORT_DATA_BUS_VOLTAGE,
            (uint8_t)P1010B_REPORT_DATA_ABSOLUTE_POSITION,
        },
    };
}

/* 所有条目默认共享同一恢复策略。 */
static MotorRecoveryPolicy recovery_policy(void)
{
    return (MotorRecoveryPolicy){
        .online_timeout_ms = APP_MR_ONLINE_AGE_MS,
        .retry_interval_ms = APP_MR_RETRY_MS,
        .fault_debounce_ms = APP_MR_FAULT_DELAY_MS,
    };
}

/* vendor 默认子状态只表达“首次恢复时从哪一步开始”。 */
static MotorRecoverySubstate recovery_substate(MotorVendor vendor)
{
    switch (vendor)
    {
    case MOTOR_VENDOR_P1010B:
        return MR_VENDOR_SUBSTATE_P1010B_DISABLE;
    case MOTOR_VENDOR_DAMIAO:
        return MR_SUB_DAMIAO_EN_PENDING;
    default:
        return MR_VENDOR_SUBSTATE_NONE;
    }
}

/* 编译期开关，方便整套恢复逻辑一键裁掉。 */
static OmBool motor_recovery_enabled(void)
{
    return (APP_MOTOR_AUTO_RECOVERY_ENABLE != 0u) ? OM_TRUE : OM_FALSE;
}

/* 在线判据在恢复模块里重新计算一次，而不是直接复用 motor->feedback.online。
 * 这样恢复模块只依赖“最近有效反馈时间戳”，不受底层 driver 自己的 online 位语义影响。
 */
static OmBool recovery_online(const MotorRecoveryEntry* entry)
{
    const MotorFeedback* feedback = OM_NULL;

    if (entry == OM_NULL || entry->motor == OM_NULL)
    {
        return OM_FALSE;
    }

    switch (motor_recovery_entry_vendor(entry))
    {
    case MOTOR_VENDOR_DJI:
    case MOTOR_VENDOR_DAMIAO:
    case MOTOR_VENDOR_GO8010:
        return motor_is_feedback_recent(entry->motor, entry->policy.online_timeout_ms);

    case MOTOR_VENDOR_P1010B:
        feedback = motor_get_feedback(entry->motor);
        if (feedback == OM_NULL || feedback->timestamp_ms == 0u)
        {
            return OM_FALSE;
        }
        return ((uint32_t)(osal_time_now_monotonic() - feedback->timestamp_ms) <= entry->policy.online_timeout_ms) ?
                   OM_TRUE :
                   OM_FALSE;

    default:
        return OM_FALSE;
    }
}

/* P1010B 除了要在线，还必须真的处于 ENABLED 且没有 fault code。 */
static OmBool recovery_p1010b_ok(const MotorRecoveryEntry* entry)
{
    P1010BDriver* driver = OM_NULL;

    if (entry == OM_NULL)
    {
        return OM_FALSE;
    }

    driver = motor_recovery_get_p1010b(entry);
    if (driver == OM_NULL)
    {
        return OM_FALSE;
    }

    if (driver->runtime.state != P1010B_STATE_ENABLED)
    {
        return OM_FALSE;
    }

    if (driver->telemetry.faultState.faultCode != 0u)
    {
        return OM_FALSE;
    }

    return recovery_online(entry);
}

/* vendor 无关的健康判定入口。 */
static OmBool recovery_ready(const MotorRecoveryEntry* entry)
{
    if (entry == OM_NULL)
    {
        return OM_FALSE;
    }

    switch (motor_recovery_entry_vendor(entry))
    {
    case MOTOR_VENDOR_P1010B:
        return recovery_p1010b_ok(entry);
    case MOTOR_VENDOR_DAMIAO:
    case MOTOR_VENDOR_GO8010:
    case MOTOR_VENDOR_DJI:
        return recovery_online(entry);
    default:
        return OM_FALSE;
    }
}

/* 记录一次恢复尝试的结果与时间，用于上层观测与下一轮节流。 */
static void recovery_mark_attempt(MotorRecoveryEntry* entry, OmRet ret, OsalTimeMs now_ms)
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
static void recovery_mark_healthy(MotorRecoveryEntry* entry)
{
    if (entry == OM_NULL)
    {
        return;
    }

    entry->state = MR_STATE_HEALTHY;
    entry->degraded_flag = OM_FALSE;
    entry->offline_since_ms = 0u;
    entry->p1010b_enable_settle_until_ms = 0u;
    entry->vendor_substate = recovery_substate(
        motor_recovery_entry_vendor(entry));
}

static OmBool recovery_p1010b_settling(
    const MotorRecoveryEntry* entry,
    OsalTimeMs now_ms)
{
    if (entry == OM_NULL || motor_recovery_entry_vendor(entry) != MOTOR_VENDOR_P1010B)
    {
        return OM_FALSE;
    }

    return osal_time_before(now_ms, entry->p1010b_enable_settle_until_ms);
}

/* 当前是否到了允许再次重试的时间窗口。 */
static OmBool recovery_retry_due(const MotorRecoveryEntry* entry, OsalTimeMs now_ms)
{
    if (entry == OM_NULL)
    {
        return OM_FALSE;
    }

    return ((uint32_t)(now_ms - entry->last_recover_ms) >= entry->policy.retry_interval_ms) ? OM_TRUE : OM_FALSE;
}

/* 离线进入恢复后，先走去抖，再升级为 DEGRADED。
 * 这里的 degraded 语义直接对应 263 的 runtime fault。
 */
static void recovery_update_state(MotorRecoveryEntry* entry, OsalTimeMs now_ms)
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
        entry->state = MR_STATE_DEGRADED;
    }
    else
    {
        entry->degraded_flag = OM_FALSE;
        entry->state = MR_STATE_RECOVERING;
    }
}

/* P1010B 恢复是固定三步同步状态机：
 * disable -> prepare_working_state -> enable
 */
static OmRet recovery_step_p1010b(MotorRecoveryEntry* entry, OsalTimeMs now_ms)
{
    P1010BDriver* driver = OM_NULL;
    P1010BResponse response = {0};
    OmRet ret = OM_OK;

    if (entry == OM_NULL || entry->motor == OM_NULL)
    {
        return OM_ERROR_PARAM;
    }

    driver = motor_recovery_get_p1010b(entry);
    if (driver == OM_NULL)
    {
        return OM_ERROR_PARAM;
    }

    switch (entry->vendor_substate)
    {
    case MR_VENDOR_SUBSTATE_P1010B_DISABLE:
        motor_recovery_bind_p1010b(driver);
        ret = p1010b_disable(driver, 0u, &response);
        if (ret == OM_OK)
        {
            entry->vendor_substate = MR_VENDOR_SUBSTATE_P1010B_SET_MODE;
        }
        break;
    case MR_VENDOR_SUBSTATE_P1010B_SET_MODE:
        ret = p1010b_set_mode(driver, driver->config.defaultMode, 0u, &response);
        if (ret == OM_OK)
        {
            entry->vendor_substate = MR_VENDOR_SUBSTATE_P1010B_SET_ACTIVE_REPORT;
        }
        break;
    case MR_VENDOR_SUBSTATE_P1010B_SET_ACTIVE_REPORT:
        ret = p1010b_set_active_report(driver, &driver->runtime.activeReport, 0u, &response);
        if (ret == OM_OK)
        {
            entry->vendor_substate = MR_VENDOR_SUBSTATE_P1010B_ENABLE;
        }
        break;
    case MR_VENDOR_SUBSTATE_P1010B_ENABLE:
        ret = p1010b_enable(driver, 0u, &response);
        if (ret == OM_OK)
        {
            entry->p1010b_enable_settle_until_ms = now_ms + MR_P1010B_SETTLE_MS;
            entry->vendor_substate = MR_VENDOR_SUBSTATE_P1010B_DISABLE;
        }
        break;
    default:
        entry->vendor_substate = MR_VENDOR_SUBSTATE_P1010B_DISABLE;
        ret = OM_ERROR_PARAM;
        break;
    }

    recovery_mark_attempt(entry, ret, now_ms);
    return ret;
}

/* 达妙恢复需要覆盖“电机晚于主控上电”的场景。
 * 这时只发 enable 不够，必须先重写 MIT 模式，再在下一轮重试里真正 enable。
 */
static OmRet recovery_step_damiao(MotorRecoveryEntry* entry, OsalTimeMs now_ms)
{
    Motor* motor = OM_NULL;
    DamiaoMotorBus* bus = OM_NULL;
    OmRet ret = OM_OK;

    if (entry == OM_NULL || entry->motor == OM_NULL)
    {
        return OM_ERROR_PARAM;
    }

    motor = entry->motor;
    bus = recovery_damiao_bus(entry);

    switch (entry->vendor_substate)
    {
    case MR_SUB_DAMIAO_EN_PENDING:
        if (bus == OM_NULL || bus->canDev == OM_NULL)
        {
            ret = OM_ERROR_PARAM;
            break;
        }

        ret = motor_owner_prepare_work(motor);
        if (ret == OM_OK)
        {
            entry->vendor_substate = MR_SUB_DAMIAO_OBS;
        }
        break;

    case MR_SUB_DAMIAO_OBS:
        ret = motor_owner_enable(motor);
        if (ret == OM_OK && bus != OM_NULL)
        {
            ret = motor_owner_sync_bus(motor);
        }
        motor_recovery_mark_damiao(motor);
        entry->vendor_substate = MR_SUB_DAMIAO_EN_PENDING;
        break;

    default:
        entry->vendor_substate = MR_SUB_DAMIAO_EN_PENDING;
        ret = OM_ERROR_PARAM;
        break;
    }

    recovery_mark_attempt(entry, ret, now_ms);
    return ret;
}

/* DJI / GO8010 当前不重开链路，只重新下发当前 target。 */
static OmRet recovery_reassert_target(MotorRecoveryEntry* entry, OsalTimeMs now_ms)
{
    OmRet ret = OM_OK;

    if (entry == OM_NULL || entry->motor == OM_NULL)
    {
        return OM_ERROR_PARAM;
    }

    ret = motor_control_compute(entry->motor);
    recovery_mark_attempt(entry, ret, now_ms);
    return ret;
}

/* 单条恢复推进逻辑：
 * 1. 先检查是否已经 healthy
 * 2. 不 healthy 则更新 state/degraded
 * 3. 到达重试时间窗口才执行 vendor 恢复动作
 */
static void recovery_tick_entry(MotorRecoveryEntry* entry, OsalTimeMs now_ms)
{
    if (entry == OM_NULL)
    {
        return;
    }

    if (recovery_ready(entry) == OM_TRUE)
    {
        recovery_mark_healthy(entry);
        return;
    }

    if (recovery_p1010b_settling(entry, now_ms) == OM_TRUE)
    {
        entry->state = MR_STATE_RECOVERING;
        entry->degraded_flag = OM_FALSE;
        entry->offline_since_ms = 0u;
        return;
    }

    recovery_update_state(entry, now_ms);
    if (recovery_retry_due(entry, now_ms) != OM_TRUE)
    {
        return;
    }

    switch (motor_recovery_entry_vendor(entry))
    {
    case MOTOR_VENDOR_P1010B:
        (void)recovery_step_p1010b(entry, now_ms);
        break;
    case MOTOR_VENDOR_DAMIAO:
        (void)recovery_step_damiao(entry, now_ms);
        break;
    case MOTOR_VENDOR_DJI:
    case MOTOR_VENDOR_GO8010:
        (void)recovery_reassert_target(entry, now_ms);
        break;
    default:
        break;
    }
}

/* 将所有 entry 的 degraded 状态聚合成 263 runtime fault。 */
static void recovery_update_fault(void)
{
    OmBool any_degraded = OM_FALSE;
    uint32_t index = 0u;

    for (index = 0u; index < g_motor_recovery.entry_count; index++)
    {
        if (g_motor_recovery.entries[index].degraded_flag == OM_TRUE)
        {
            any_degraded = OM_TRUE;
            break;
        }
    }

    if (any_degraded == OM_TRUE)
    {
        (void)sh_report_runtime_fault(SH_ERR_MR_DEGRADED);
        g_motor_recovery.runtime_fault_active = OM_TRUE;
    }
    else if (g_motor_recovery.runtime_fault_active == OM_TRUE)
    {
        (void)sh_clear_runtime_fault(SH_ERR_MR_DEGRADED);
        g_motor_recovery.runtime_fault_active = OM_FALSE;
    }
}

void motor_recovery_reset(void)
{
    memset(&g_motor_recovery, 0, sizeof(g_motor_recovery));
}

/* 统一把恢复期要求写回 P1010B driver 配置。 */
void motor_recovery_bind_p1010b(P1010BDriver* driver)
{
    if (driver == OM_NULL)
    {
        return;
    }

    driver->config.requestTimeoutMs = MR_P1010B_SYNC_MS;
    driver->config.maxRetryCount = MR_P1010B_RETRY_MAX;
    driver->config.activeReport = recovery_report_cfg();
    driver->runtime.activeReport = driver->config.activeReport;
    driver->runtime.currentMode = driver->config.defaultMode;
}

/* 注册一台要参与自动恢复的电机。
 * 若宏关闭，则该函数静默返回 OM_OK，让正式通信任务无需分叉处理。
 */
OmRet motor_recovery_register(
    Motor* motor)
{
    MotorRecoveryEntry* entry = OM_NULL;

    if (motor_recovery_enabled() != OM_TRUE)
    {
        return OM_OK;
    }

    if (motor == OM_NULL)
    {
        return OM_ERROR_PARAM;
    }

    if (g_motor_recovery.entry_count >= MR_CAPACITY)
    {
        return OM_ERROR_MEMORY;
    }

    entry = &g_motor_recovery.entries[g_motor_recovery.entry_count++];
    memset(entry, 0, sizeof(*entry));
    entry->motor = motor;
    entry->policy = recovery_policy();
    entry->state = MR_STATE_RECOVERING;
    entry->vendor_substate = recovery_substate(motor->config.vendor);
    entry->last_recover_ret = OM_OK;

    if (motor->config.vendor == MOTOR_VENDOR_DAMIAO)
    {
        /* 达妙 observation 静置窗口存放在 entry 私有字段里，
         * motor 只拿到一个 vendor_context 指针，不直接感知恢复模块状态。
         */
        return motor_set_vendor_context(motor, &entry->observe_gate_until_ms);
    }

    return OM_OK;
}

/* 达妙每次 enable 后都需要重新打一个 settle gate。 */
void motor_recovery_mark_damiao(Motor* motor)
{
    OsalTimeMs* gate_until_ptr = OM_NULL;

    if (motor_recovery_enabled() != OM_TRUE)
    {
        return;
    }

    if (motor == OM_NULL || motor->config.vendor_context == OM_NULL)
    {
        return;
    }

    gate_until_ptr = (OsalTimeMs*)motor->config.vendor_context;
    *gate_until_ptr = osal_time_now_monotonic() + MR_DAMIAO_SETTLE_MS;
}

void motor_recovery_mark_p1010b(Motor* motor)
{
    MotorRecoveryEntry* entry = OM_NULL;
    OsalTimeMs now_ms = 0u;

    if (motor_recovery_enabled() != OM_TRUE || motor == OM_NULL)
    {
        return;
    }

    now_ms = osal_time_now_monotonic();
    entry = motor_recovery_find_mut(motor);
    if (entry == OM_NULL)
    {
        return;
    }

    entry->p1010b_enable_settle_until_ms = now_ms + MR_P1010B_SETTLE_MS;
    entry->state = MR_STATE_RECOVERING;
    entry->degraded_flag = OM_FALSE;
    entry->offline_since_ms = 0u;
}

OmBool motor_recovery_block_damiao(const Motor* motor)
{
    const MotorRecoveryEntry* entry = OM_NULL;
    OsalTimeMs now_ms = 0u;

    if (motor_recovery_enabled() != OM_TRUE || motor == OM_NULL)
    {
        return OM_FALSE;
    }

    entry = motor_recovery_find(motor);
    if (entry == OM_NULL || motor_recovery_entry_vendor(entry) != MOTOR_VENDOR_DAMIAO)
    {
        return OM_FALSE;
    }

    now_ms = osal_time_now_monotonic();
    if (entry->state == MR_STATE_HEALTHY)
    {
        return OM_FALSE;
    }

    if (entry->vendor_substate == MR_SUB_DAMIAO_OBS)
    {
        return OM_TRUE;
    }

    if (osal_time_before(now_ms, entry->observe_gate_until_ms) == OM_TRUE)
    {
        return OM_TRUE;
    }

    return OM_FALSE;
}

/* 启动期宽限：避免刚注册好就立刻进入第一次重试。 */
void motor_recovery_initial_grace(void)
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

void motor_recovery_rearm_all(void)
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
        MotorRecoveryEntry* entry = &g_motor_recovery.entries[index];

        entry->state = MR_STATE_RECOVERING;
        entry->vendor_substate =
            recovery_substate(motor_recovery_entry_vendor(entry));
        entry->degraded_flag = OM_FALSE;
        entry->last_recover_ret = OM_OK;
        entry->last_recover_ms = now_ms;
        entry->offline_since_ms = 0u;
        entry->observe_gate_until_ms = 0u;
        entry->p1010b_enable_settle_until_ms = 0u;
        entry->recover_count = 0u;
    }

    if (g_motor_recovery.runtime_fault_active == OM_TRUE)
    {
        (void)sh_clear_runtime_fault(SH_ERR_MR_DEGRADED);
        g_motor_recovery.runtime_fault_active = OM_FALSE;
    }

    g_motor_recovery.last_tick_ms = 0u;
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
    if (g_motor_recovery.last_tick_ms != 0u &&
        (uint32_t)(now_ms - g_motor_recovery.last_tick_ms) < APP_MR_TICK_PERIOD_MS)
    {
        return;
    }

    g_motor_recovery.last_tick_ms = now_ms;
    for (index = 0u; index < g_motor_recovery.entry_count; index++)
    {
        recovery_tick_entry(&g_motor_recovery.entries[index], now_ms);
    }

    recovery_update_fault();
}

/* 对外导出最小恢复快照，供 VOFA / 调试读取。 */
OmRet motor_recovery_copy(
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

        snapshots[*snapshot_count].name = entry->motor->config.name;
        snapshots[*snapshot_count].vendor = (uint8_t)entry->motor->config.vendor;
        snapshots[*snapshot_count].online = recovery_online(entry);
        snapshots[*snapshot_count].state = entry->state;
        snapshots[*snapshot_count].degraded_flag = entry->degraded_flag;
        snapshots[*snapshot_count].recover_count = entry->recover_count;
        snapshots[*snapshot_count].last_recover_ret = entry->last_recover_ret;
        snapshots[*snapshot_count].last_recover_ms = entry->last_recover_ms;
        (*snapshot_count)++;
    }

    return OM_OK;
}

OmRet motor_recovery_copy_p1010b(
    MotorRecoveryP1010BInfo* snapshots,
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

        if (motor_recovery_entry_vendor(entry) != MOTOR_VENDOR_P1010B)
        {
            continue;
        }

        if (*snapshot_count >= capacity)
        {
            break;
        }

        driver = motor_recovery_get_p1010b(entry);
        snapshots[*snapshot_count].name = entry->motor->config.name;
        snapshots[*snapshot_count].online = recovery_online(entry);
        snapshots[*snapshot_count].state_enabled =
            (driver != OM_NULL && driver->runtime.state == P1010B_STATE_ENABLED) ? OM_TRUE : OM_FALSE;
        snapshots[*snapshot_count].fault_clear =
            (driver != OM_NULL && driver->telemetry.faultState.faultCode == 0u) ? OM_TRUE : OM_FALSE;
        snapshots[*snapshot_count].healthy = recovery_p1010b_ok(entry);
        (*snapshot_count)++;
    }

    return OM_OK;
}

#endif
