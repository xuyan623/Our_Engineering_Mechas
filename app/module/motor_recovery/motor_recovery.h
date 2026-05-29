#ifndef NEW_ROBOT_MOTOR_RECOVERY_H
#define NEW_ROBOT_MOTOR_RECOVERY_H

#include "config/app_config.h"
#include "core/om_def.h"
#include "driver/motor/motor.h"
#include "osal/osal_time.h"
#include <stdint.h>

/* 运行时恢复状态只表达“当前恢复进展”，不区分具体 vendor 步骤。
 * 具体恢复步骤由 .c 文件中的 vendor substate 状态机维护。
 */
typedef enum
{
    MOTOR_RECOVERY_STATE_HEALTHY = 0u,
    MOTOR_RECOVERY_STATE_RECOVERING,
    MOTOR_RECOVERY_STATE_DEGRADED,
} MotorRecoveryState;

/* 对外暴露的最小恢复快照。
 * 该结构用于 VOFA/调试读取，不泄露内部恢复状态机细节。
 */
typedef struct
{
    const char* name;
    uint8_t vendor; /* MotorVendor */
    OmBool online;
    MotorRecoveryState state;
    OmBool degraded_flag;
    uint32_t recover_count;
    int32_t last_recover_ret;
    uint32_t last_recover_ms;
} MotorRecoverySnapshot;

/* P1010B 离线判据快照。
 * 这里直接展开恢复模块对 P1010B 的 4 个判断结果，便于 VOFA 逐项观察：
 * - online：最近反馈时间戳是否仍在在线窗口内
 * - state_enabled：driver 运行态是否已经进入 ENABLED
 * - fault_clear：faultCode 是否为 0
 * - healthy：最终总判据，等价于上面三项同时成立
 */
typedef struct
{
    const char* name;
    OmBool online;
    OmBool state_enabled;
    OmBool fault_clear;
    OmBool healthy;
} MotorRecoveryP1010BPredicateSnapshot;

#if (APP_MOTOR_AUTO_RECOVERY_ENABLE != 0u)

/* 清空整个恢复模块上下文。
 * 正式通信任务在 runtime_init 早期调用，确保每次上电从干净状态开始。
 */
void motor_recovery_reset(void);

/* 将 app 侧对 P1010B 的恢复期默认约束写回 driver。
 * 这里收敛的是同步请求超时、重试次数和 active-report 配置。
 */
void motor_recovery_configure_p1010b_driver(P1010BDriver* driver);

/* 向恢复模块注册一台需要被运行时监督的电机。
 * 注意：这里只注册“要参与自动恢复”的电机；预留但未安装的电机不应注册进来。
 */
OmRet motor_recovery_register_entry(
    Motor* motor);

/* 达妙在 enable 成功后需要短暂静置窗口，避免刚使能就被 observation 帧打断。
 * 正式通信任务在启动期和运行期 re-enable 后都应调用它。
 */
void motor_recovery_notify_damiao_enabled(Motor* motor);

/* P1010B 在 enable 成功后需要一个短暂稳定窗口，
 * 这段时间只允许 active report 自然重建在线时间戳，不应立刻再进恢复或报码。
 */
void motor_recovery_notify_p1010b_enabled(Motor* motor);

/* 达妙正在跑恢复步骤时，常规 MIT 目标应短暂让位给恢复帧。
 * 这里只回答“当前这台电机是否该阻断常规目标”，不扩散成公共 Motor 状态。
 */
OmBool motor_recovery_should_block_damiao_regular_target(const Motor* motor);

/* 给所有已注册条目打一层“初始恢复宽限期”。
 * 用于避免任务刚启动时立刻把首轮离线判成 fault。
 */
void motor_recovery_arm_initial_grace(void);

/* 保留已注册条目，只把运行期状态重置成“刚完成 bring-up”的干净状态。
 * 这一步是后续软件侧重进正式可控态的基础，不需要重新注册电机。
 */
void motor_recovery_rearm_registered_entries(void);

/* 推进一次恢复状态机并根据当前条目状态更新 runtime fault。 */
void motor_recovery_tick(void);

/* 拷贝当前恢复快照。
 * 该接口保证只输出稳定的最小诊断信息，不暴露内部 entry 存储布局。
 */
OmRet motor_recovery_copy_snapshots(
    MotorRecoverySnapshot* snapshots,
    uint32_t capacity,
    uint32_t* snapshot_count);

/* 拷贝所有已注册 P1010B 的离线判据快照。
 * 当前正式工程里会返回两台腿电机，顺序与恢复条目注册顺序一致。
 */
OmRet motor_recovery_copy_p1010b_predicate_snapshots(
    MotorRecoveryP1010BPredicateSnapshot* snapshots,
    uint32_t capacity,
    uint32_t* snapshot_count);

#else

static inline void motor_recovery_reset(void)
{
}

static inline void motor_recovery_configure_p1010b_driver(P1010BDriver* driver)
{
    if (driver == OM_NULL)
    {
        return;
    }

    driver->config.requestTimeoutMs = 5u;
    driver->config.maxRetryCount = 0u;
    driver->config.activeReport = (P1010BActiveReportConfig){
        .enable = true,
        .periodMs = APP_MOTOR_RECOVERY_P1010B_REPORT_PERIOD_MS,
        .dataTypeSlots = {
            (uint8_t)P1010B_REPORT_DATA_SPEED_RPM,
            (uint8_t)P1010B_REPORT_DATA_IQ_AMPERE,
            (uint8_t)P1010B_REPORT_DATA_BUS_VOLTAGE,
            (uint8_t)P1010B_REPORT_DATA_ABSOLUTE_POSITION,
        },
    };
    driver->runtime.activeReport = driver->config.activeReport;
    driver->runtime.currentMode = driver->config.defaultMode;
}

static inline OmRet motor_recovery_register_entry(Motor* motor)
{
    static OsalTimeMs g_motor_recovery_disabled_damiao_observe_gate_ms = 0u;

    if (motor != OM_NULL && motor->config.vendor == MOTOR_VENDOR_DAMIAO)
    {
        return motor_set_vendor_context(
            motor,
            &g_motor_recovery_disabled_damiao_observe_gate_ms);
    }

    return OM_OK;
}

static inline void motor_recovery_notify_damiao_enabled(Motor* motor)
{
    (void)motor;
}

static inline void motor_recovery_notify_p1010b_enabled(Motor* motor)
{
    (void)motor;
}

static inline OmBool motor_recovery_should_block_damiao_regular_target(const Motor* motor)
{
    (void)motor;
    return OM_FALSE;
}

static inline void motor_recovery_arm_initial_grace(void)
{
}

static inline void motor_recovery_rearm_registered_entries(void)
{
}

static inline void motor_recovery_tick(void)
{
}

static inline OmRet motor_recovery_copy_snapshots(
    MotorRecoverySnapshot* snapshots,
    uint32_t capacity,
    uint32_t* snapshot_count)
{
    (void)snapshots;
    (void)capacity;

    if (snapshot_count == OM_NULL)
    {
        return OM_ERROR_NULL;
    }

    *snapshot_count = 0u;
    return OM_OK;
}

static inline OmRet motor_recovery_copy_p1010b_predicate_snapshots(
    MotorRecoveryP1010BPredicateSnapshot* snapshots,
    uint32_t capacity,
    uint32_t* snapshot_count)
{
    (void)snapshots;
    (void)capacity;

    if (snapshot_count == OM_NULL)
    {
        return OM_ERROR_NULL;
    }

    *snapshot_count = 0u;
    return OM_OK;
}

#endif

#endif
