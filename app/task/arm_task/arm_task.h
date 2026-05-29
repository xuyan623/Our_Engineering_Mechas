#ifndef NEW_ROBOT_ARM_TASK_H
#define NEW_ROBOT_ARM_TASK_H

#include "core/om_def.h"
#include "module/data_pool/data_pool.h"
#include "task/arm_task/arm_task_diag.h"
#include "task/mode_task/mode_task.h"
#include <stdint.h>

/* 机械臂控制任务：
 * - 消费 mode_task 导出的 chassis_mode / action
 * - 在任务内部把动作模式翻译成各轴目标位姿
 * - 只在 motor 抽象层写目标，不直接碰物理总线
 * - 当前调试阶段恢复整条机械臂正式控制
 *
 * 当前 owner 边界：
 * - grip：由 arm_task 直接写 angle target
 * - roll3：由 arm_task 维护本地双环 PID 并直接写 current target
 * - 物理收发仍只归 motor_communications_task
 */
OmRet arm_task_start(void);
OmRet arm_task_submit_mode_control_snapshot(
    const ModeTaskControlSnapshot* snapshot);
OmRet arm_task_submit_custom_controller_snapshot(
    const DpCustomControllerSnapshot* snapshot);

#endif
