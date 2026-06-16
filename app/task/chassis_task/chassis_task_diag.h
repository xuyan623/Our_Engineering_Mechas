#ifndef NEW_ROBOT_CT_DIAG_H
#define NEW_ROBOT_CT_DIAG_H

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
OmBool chassis_task_debug_snapshot(
    float wheel_feedback_rpm[4],
    float wheel_command_current[4],
    float leg_feedback_deg[2],
    float leg_command_current[2]);

/* 读取当前底盘正式控制相位。
 * - chassis_mode：输出当前 operational phase（0=release,1=selection,2=formal）
 * - 返回 OM_TRUE 表示模式快照已就绪，OM_FALSE 表示尚未收到首帧
 */
OmBool chassis_task_get_phase_debug(
    uint8_t* chassis_mode);

/* VTable 诊断回调：
 * - diag_online 返回轮/腿电机在线状态 bitmask
 * - diag_snapshot 返回固定 12 个 float：
 *   [0]  wheel_fr_feedback_rpm
 *   [1]  wheel_fl_feedback_rpm
 *   [2]  wheel_bl_feedback_rpm
 *   [3]  wheel_br_feedback_rpm
 *   [4]  wheel_fr_command_current
 *   [5]  wheel_fl_command_current
 *   [6]  wheel_bl_command_current
 *   [7]  wheel_br_command_current
 *   [8]  joint_leg_r_feedback_deg
 *   [9]  joint_leg_l_feedback_deg
 *   [10] joint_leg_r_command_current
 *   [11] joint_leg_l_command_current
 */
void chassis_task_diag_online(void* ctx, uint8_t* out_online);
void chassis_task_diag_snapshot(void* ctx, float* out_buf, uint32_t cap, uint32_t* out_count);

#endif
