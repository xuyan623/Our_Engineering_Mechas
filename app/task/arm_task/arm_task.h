#ifndef NEW_ROBOT_ARM_TASK_H
#define NEW_ROBOT_ARM_TASK_H

#include "core/om_def.h"
#include <stdint.h>

/* 机械臂控制任务：
 * - 消费 mode_task 导出的 chassis_mode / action
 * - 在任务内部把动作模式翻译成各轴目标位姿
 * - 只在 motor 抽象层写目标，不直接碰物理总线
 * - 完成一轮控制后发布 EVT_MOTOR_TX_REQUEST
 *
 * 当前 owner 边界：
 * - big_yaw / pitch1 / pitch2 / roll2 / pitch3 / grip：由 arm_task 直接写目标
 * - roll3：由 arm_task 内部双环 PID 计算后写电流目标
 * - 物理收发仍只归 motor_communications_task
 */
OmRet arm_task_start(void);

/* 当前自定义控制器接管是否已经完成对齐并进入可接管态。
 * 这是只读调试接口，供 VOFA 等观测路径使用。
 */
uint8_t arm_task_get_custom_controller_alignment_done(void);

#endif
