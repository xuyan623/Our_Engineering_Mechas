#ifndef NEW_ROBOT_CHASSIS_TASK_H
#define NEW_ROBOT_CHASSIS_TASK_H

#include "core/om_def.h"
#include "task/input_task/input_task_snapshot.h"
#include "task/imu_task/imu_task_snapshot.h"
#include "task/chassis_task/chassis_task_diag.h"
#include "task/mode_task/mode_task.h"

/**
 * @brief 启动底盘控制任务
 * @details 消费 mode_task 导出的 ChassisTaskModeSnapshot，
 *          只根据 wheel_enable / leg_enable / allow_rc_drive 做底盘输出分流。
 * @return `OM_OK` 表示启动成功，其他返回值表示重复启动、PID 初始化失败或任务创建失败
 */
OmRet chassis_task_start(void);
OmRet chassis_task_submit_mode_control_snapshot(
    const ChassisTaskModeSnapshot* snapshot);
OmRet chassis_task_submit_rc_snapshot(
    const InputRcSnapshot* snapshot);
OmRet chassis_task_submit_imu_snapshot(
    const ImuTaskSnapshot* snapshot);

#endif
