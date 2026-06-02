#ifndef NEW_ROBOT_VOFA_TASK_H
#define NEW_ROBOT_VOFA_TASK_H

/* vofa_task 的职责边界：
 * - observer only：只读诊断快照并上传到 UART7
 * - 不拥有正式控制状态
 * - 不参与模式仲裁和控制输出
 *
 * 当前它消费 task_context_pool 暴露的 diag 面，并通过
 * vofa_layout registry 决定默认上传布局；
 * 后续即使扩成多布局切换，也仍然只能停留在 observer 层。
 */

#include "bsp/bsp_init.h"
#include "core/om_def.h"

/**
 * @brief 启动 UART7 VOFA 调试任务
 * @param devices BSP 设备注册表，至少需要有效的 `uart7`
 * @return `OM_OK` 表示启动成功，其他返回值表示参数错误、重复启动或任务创建失败
 */
OmRet vofa_task_start(const BspDeviceRegistry* devices);

#endif
