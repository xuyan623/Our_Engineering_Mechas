#ifndef NEW_ROBOT_MOTOR_COMMUNICATIONS_TASK_H
#define NEW_ROBOT_MOTOR_COMMUNICATIONS_TASK_H

#include "bsp/bsp_init.h"
#include "core/om_def.h"
#include "module/motor_recovery/motor_recovery.h"
#include <stdint.h>

typedef struct
{
    uint32_t raw_rx_count;
    uint32_t last_raw_stdid;
    uint32_t feedback_sequence[6];
} MotorCommunicationsTaskDamiaoDiagSnapshot;

typedef struct
{
    const char* name;
    uint32_t last_query_ok_ms;
    uint32_t query_ok_age_ms;
    int32_t last_query_ret;
} MotorCommunicationsTaskP1010BDiagSnapshot;

/* 正式电机通信任务启动入口。
 * 该任务拥有：
 * - CAN1：DJI + P1010B
 * - CAN2：Damiao
 * - USART6：GO8010
 */
OmRet motor_communications_task_start(const BspDeviceRegistry* devices);

/* 拷贝当前运行期恢复快照。
 * 该接口只暴露最小诊断信息，不改变现有 VOFA 通道布局。
 */
OmRet motor_communications_task_copy_recovery_snapshots(
    MotorRecoverySnapshot* snapshots,
    uint32_t capacity,
    uint32_t* snapshot_count);

OmRet motor_communications_task_copy_p1010b_predicate_snapshots(
    MotorRecoveryP1010BPredicateSnapshot* snapshots,
    uint32_t capacity,
    uint32_t* snapshot_count);

OmRet motor_communications_task_copy_p1010b_diag_snapshots(
    MotorCommunicationsTaskP1010BDiagSnapshot* snapshots,
    uint32_t capacity,
    uint32_t* snapshot_count);

OmRet motor_communications_task_copy_damiao_diag(
    MotorCommunicationsTaskDamiaoDiagSnapshot* snapshot);

#endif
