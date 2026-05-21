#ifndef NEW_ROBOT_CHASSIS_TASK_H
#define NEW_ROBOT_CHASSIS_TASK_H

#include "core/om_def.h"
#include <stdint.h>

#define CHASSIS_TASK_DIAG_WHEEL_COUNT (4u)

typedef struct
{
    float vx_mm_per_s;
    float vy_mm_per_s;
    float vw_deg_per_s;
    int16_t wheel_speed_ref_rpm[CHASSIS_TASK_DIAG_WHEEL_COUNT];
    float wheel_speed_fdb_rpm[CHASSIS_TASK_DIAG_WHEEL_COUNT];
    float wheel_current_cmd[CHASSIS_TASK_DIAG_WHEEL_COUNT];
    uint32_t online_wheel_count;
} ChassisTaskDiagSnapshot;

/**
 * @brief 启动底盘控制任务
 * @return `OM_OK` 表示启动成功，其他返回值表示重复启动、PID 初始化失败或任务创建失败
 */
OmRet chassis_task_start(void);
OmRet chassis_task_copy_diag_snapshot(ChassisTaskDiagSnapshot* snapshot);

#endif
