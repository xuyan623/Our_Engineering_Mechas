#ifndef NEW_ROBOT_CHASSIS_TASK_H
#define NEW_ROBOT_CHASSIS_TASK_H

#include "core/om_def.h"

/**
 * @brief 启动底盘控制任务
 * @return `OM_OK` 表示启动成功，其他返回值表示重复启动、PID 初始化失败或任务创建失败
 */
OmRet chassis_task_start(void);

#endif
