#ifndef NEW_ROBOT_VOFA_TASK_H
#define NEW_ROBOT_VOFA_TASK_H

#include "bsp/bsp_init.h"
#include "core/om_def.h"

/**
 * @brief 启动 UART7 VOFA 调试任务
 * @param devices BSP 设备注册表，至少需要有效的 `uart7`
 * @return `OM_OK` 表示启动成功，其他返回值表示参数错误、重复启动或任务创建失败
 */
OmRet vofa_task_start(const BspDeviceRegistry* devices);

#endif
