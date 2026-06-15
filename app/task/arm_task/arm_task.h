#ifndef NEW_ROBOT_ARM_TASK_H
#define NEW_ROBOT_ARM_TASK_H

#include "core/om_def.h"
#include "task/input_task/input_task_snapshot.h"
#include "task/arm_task/arm_task_diag.h"
#include "task/mode_task/mode_task.h"
#include <stdint.h>

/* 机械臂控制任务：
 * - 消费 mode_task 导出的 ArmTaskModeSnapshot
 * - 在任务内部按 arm_mode 分流到 normal / preset / custom / rc_ik
 * - 只在 motor 抽象层写目标，不直接碰物理总线
 *
 * 当前 owner 边界：
 * - grip：由 arm_task 直接写 angle target
 * - roll3：由 arm_task 维护本地双环 PID 并直接写 current target
 * - 物理收发仍只归 motor_communications_task
 */
OmRet arm_task_start(void);
OmRet arm_task_submit_mode_control_snapshot(
    const ArmTaskModeSnapshot* snapshot);
OmRet arm_task_submit_rc_snapshot(
    const InputRcSnapshot* snapshot);
OmRet arm_task_submit_custom_controller_snapshot(
    const InputCustomControllerSnapshot* snapshot);

#endif
