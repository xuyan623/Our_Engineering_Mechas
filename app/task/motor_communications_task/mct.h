#ifndef NEW_ROBOT_MCT_H
#define NEW_ROBOT_MCT_H

#include "bsp/bsp_init.h"
#include "core/om_def.h"
#include "module/motor_recovery/motor_recovery.h"
#include <stdint.h>

/* 达妙最小诊断快照：
 * - raw_rx_count / last_raw_stdid：总线级原始接收观测
 * - feedback_sequence[]：6 台正式达妙驱动各自的反馈序号
 *
 * 这个结构只面向调试/观测层，不参与正式控制。
 */
typedef struct
{
    uint32_t raw_rx_count;
    uint32_t last_raw_stdid;
    uint32_t raw_rx_by_stdid[6];
    uint32_t raw_tx_by_stdid[6];
    uint32_t feedback_sequence[6];
} MctDamiaoDiagSnapshot;

typedef struct
{
    const char* name;
    uint8_t online;
    uint32_t age_ms;
    uint32_t feedback_sequence;
    uint8_t status;
} MctDamiaoMotorDiagSnapshot;

/* P1010B 最小诊断快照：
 * - last_feedback_ms：最近一次 active report/反馈帧时间戳
 * - feedback_age_ms：距最近一次反馈的年龄
 * - online：当前反馈新鲜度折算出的在线位
 * - absolute_position_raw：原始绝对位置编码值
 * - feedback_angle_rad：当前角度反馈（弧度）
 */
typedef struct
{
    const char* name;
    uint32_t last_feedback_ms;
    uint32_t feedback_age_ms;
    uint8_t online;
    uint16_t absolute_position_raw;
    float feedback_angle_rad;
} MctP1010BDiagSnapshot;

typedef struct
{
    int16_t target_output_raw;
    uint32_t feedback_timestamp_ms;
    uint32_t raw_rx_count;
} MctDjiRoll3DiagSnapshot;

typedef struct
{
    uint32_t raw_rx_count;
    uint32_t tx_attempt_count;
    uint32_t tx_success_count;
    uint32_t tx_fail_count;
    uint32_t dirty_unit_count;
} MctDjiBusDiagSnapshot;

typedef struct
{
    uint32_t raw_rx_count;
    uint32_t raw_feedback_count;
    uint32_t raw_rx_motor1_count;
    uint32_t raw_rx_motor2_count;
} MctP1010BBusDiagSnapshot;

typedef struct
{
    uint8_t operational_active;
    uint32_t last_tx_request_sources_mask;
    uint8_t last_tx_request_overflowed;
} MctRuntimeDebugSnapshot;

/* 正式电机通信任务启动入口。
 * 该任务拥有：
 * - CAN1：DJI + P1010B
 * - CAN2：Damiao
 * - USART6：GO8010
 */
OmRet mct_start(const BspDeviceRegistry* devices);

/* 请求 owner 线程重新进入“正式可控态”。
 * 调用方只提交请求；真正的 bring-up 只在 mct 线程里执行。
 */
OmRet mct_request_enter_operational_state(void);

/* 请求 owner 线程退出“正式可控态”并进入持续失能路径。
 * 调用方只提交请求；真正的 leave 只在 mct 线程里执行。
 */
OmRet mct_request_leave_operational_state(void);

/* 请求 owner 线程执行一次 leave -> enter 的软件重置。
 * 调用方只提交请求；真正的 reset 只在 mct 线程里执行。
 */
OmRet mct_request_reset_operational_state(void);

/* 只读 owner 事实：当前是否已经进入正式可控态。 */
OmBool mct_is_operational_active(void);
OmRet mct_copy_runtime_debug_snapshot(
    MctRuntimeDebugSnapshot* snapshot);

/* 拷贝当前运行期恢复快照。
 * 该接口只暴露最小诊断信息，不改变现有 VOFA 通道布局。
 */
OmRet mct_copy_recovery_snapshots(
    MotorRecoverySnapshot* snapshots,
    uint32_t capacity,
    uint32_t* snapshot_count);

OmRet mct_copy_p1010b_predicate_snapshots(
    MotorRecoveryP1010BPredicateSnapshot* snapshots,
    uint32_t capacity,
    uint32_t* snapshot_count);

/* 拷贝 P1010B active-report 诊断快照。
 * 该接口只反映 owner task 当前看到的反馈新鲜度，不等同于 vendor driver 的全部内部状态。
 */
OmRet mct_copy_p1010b_diag_snapshots(
    MctP1010BDiagSnapshot* snapshots,
    uint32_t capacity,
    uint32_t* snapshot_count);
OmRet mct_copy_dji_roll3_diag(
    MctDjiRoll3DiagSnapshot* snapshot);
OmRet mct_copy_dji_bus_diag(
    MctDjiBusDiagSnapshot* snapshot);
OmRet mct_copy_p1010b_bus_diag(
    MctP1010BBusDiagSnapshot* snapshot);

/* 拷贝达妙总线级诊断快照。 */
OmRet mct_copy_damiao_diag(
    MctDamiaoDiagSnapshot* snapshot);

OmRet mct_copy_damiao_motor_diag_snapshots(
    MctDamiaoMotorDiagSnapshot* snapshots,
    uint32_t capacity,
    uint32_t* snapshot_count);

#endif
