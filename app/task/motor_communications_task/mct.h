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
    uint32_t feedback_sequence[6];
} MctDamiaoDiagSnapshot;

/* P1010B query-mode 的最小诊断快照：
 * - last_query_ok_ms：最近一次 query 成功的本地时间戳
 * - query_ok_age_ms：距最近成功 query 的年龄
 * - last_query_ret：最近一次 active_query 的返回值
 */
typedef struct
{
    const char* name;
    uint32_t last_query_ok_ms;
    uint32_t query_ok_age_ms;
    int32_t last_query_ret;
} MctP1010BDiagSnapshot;

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

/* 请求 owner 线程执行一次 leave -> enter 的软件重置。
 * 调用方只提交请求；真正的 reset 只在 mct 线程里执行。
 */
OmRet mct_request_reset_operational_state(void);

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

/* 拷贝 P1010B query-mode 诊断快照。
 * 该接口只反映 owner task 内部 query 结果，不等同于 vendor driver 的全部内部状态。
 */
OmRet mct_copy_p1010b_diag_snapshots(
    MctP1010BDiagSnapshot* snapshots,
    uint32_t capacity,
    uint32_t* snapshot_count);

/* 拷贝达妙总线级诊断快照。 */
OmRet mct_copy_damiao_diag(
    MctDamiaoDiagSnapshot* snapshot);

#endif
