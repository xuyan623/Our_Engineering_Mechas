#ifndef NEW_ROBOT_INPUT_TASK_H
#define NEW_ROBOT_INPUT_TASK_H

#include "bsp/bsp_init.h"
#include "core/om_def.h"

typedef struct
{
    volatile uint32_t rx_available_hint;
    volatile uint32_t frame_count;
    volatile uint32_t invalid_frame_count;
} InputTaskDebugState;

extern InputTaskDebugState g_input_task_runtime;

/**
 * @brief 启动 DBUS 输入采集任务
 * @param devices BSP 设备注册表，至少需要有效的 `usart1`
 * @return `OM_OK` 表示启动成功，其他返回值表示参数错误、重复启动或任务创建失败
 */
OmRet input_task_start(const BspDeviceRegistry* devices);

#endif
