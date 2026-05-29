#ifndef NEW_ROBOT_CHASSIS_TASK_DIAG_H
#define NEW_ROBOT_CHASSIS_TASK_DIAG_H

/* chassis_task 观测接口。
 * 职责：为 VOFA 等外部观测路径提供只读诊断快照。
 * 这些接口不参与正式控制，仅用于调试与状态展示。
 */

#include "core/om_def.h"
#include <stdint.h>

/* 采集底盘四轮与两腿的反馈、目标与输出电流快照。
 * - wheel_feedback_rpm / wheel_command_current：长度 4（四轮）
 * - leg_feedback_deg / leg_command_current：长度 2（两腿）
 * 任一参数为 NULL 或上下文未初始化时返回 OM_FALSE。
 */
OmBool chassis_task_get_debug_snapshot(
    float wheel_feedback_rpm[4],
    float wheel_command_current[4],
    float leg_feedback_deg[2],
    float leg_command_current[2]);

/* 读取当前底盘模式。
 * - chassis_mode：输出当前 ChassisMode 枚举值
 * - 返回 OM_TRUE 表示模式快照已就绪，OM_FALSE 表示尚未收到首帧
 */
OmBool chassis_task_get_debug_chassis_mode(
    uint8_t* chassis_mode);

#endif
