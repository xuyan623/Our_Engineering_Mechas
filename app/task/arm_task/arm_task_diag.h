#ifndef NEW_ROBOT_ARM_TASK_DIAG_H
#define NEW_ROBOT_ARM_TASK_DIAG_H

/* arm_task 观测接口。
 * 职责：为 VOFA、mode_task 等外部观测路径提供只读诊断快照。
 * 这些接口不参与正式控制，仅用于调试与状态展示。
 */

#include "core/om_def.h"
#include <stdint.h>

/* 当前自定义控制器接管是否已经完成对齐并进入可接管态。
 * 这是只读调试接口，供 VOFA 等观测路径使用。
 */
uint8_t arm_task_get_custom_controller_alignment_done(void);

/* 自定义控制器最近一次接收快照的在线位。
 * 这是接收链语义，不依赖 mode_task 当前是否处于 CUSTOM domain。
 */
uint8_t arm_task_get_custom_controller_online(void);

/* 自定义控制器是否正处于接管态（alignment 完成 + 模式匹配 + 在线 + 工作模式正确）。
 * 返回 1 表示正在接管，0 表示未接管。
 */
uint8_t arm_task_get_custom_controller_takeover_bit(void);



/* 采集自定义控制器前三个轴的最近反馈值。
 * 当前按 Y / Z / X 三轴导出，单位为度（deg）。
 * 上下文未初始化或任一参数为 NULL 时返回 OM_FALSE。
 */
OmBool arm_task_get_custom_controller_feedback_snapshot(
    float* axis0_feedback_deg,
    float* axis1_feedback_deg,
    float* axis2_feedback_deg);

/* 采集自定义控制器 pitch 轴（控制 pitch3）的角度反馈。
 * 单位为度（deg）。
 */
OmBool arm_task_get_custom_controller_pitch_axis_feedback(float* pitch_axis_feedback_deg);

/* 采集机械臂全部 7 轴电机角度反馈快照。
 * 顺序和单位与 g_arm_pose_* 动作表一致：
 *   [0] big_yaw   (rad)
 *   [1] pitch1    (rad)
 *   [2] pitch2    (rad)
 *   [3] roll2     (rad)
 *   [4] pitch3    (rad)
 *   [5] roll3     (rad)
 *   [6] grip      (rad)
 */
/* 采集机械臂全部 7 轴电机反馈的机构角快照（逆映射）。
 * 顺序与 g_arm_pose_* 动作表一致：
 *   [0] big_yaw   (rad)
 *   [1] pitch1    (rad)
 *   [2] pitch2    (rad)
 *   [3] roll2     (rad)
 *   [4] pitch3    (rad)
 *   [5] roll3     (rad)
 *   [6] grip      (rad)
 *
 * 该接口将电机原始反馈角逆向映射回机构角语义，
 * 便于在 VOFA 上直接与 g_arm_pose_* 动作表数值对比。
 */
OmBool arm_task_get_arm_motor_machine_angle_rad_snapshot(
    float machine_angle_rad[7]);

OmBool arm_task_get_arm_motor_feedback_rad_snapshot(
    float arm_feedback_rad[7]);



/* 采集 pitch2/GO8010 的反馈与目标快照。
 * - 角度单位：deg
 * - 速度单位：rpm
 * - 电流单位：A
 * - 力矩单位：N·m
 * - online：0/1
 */
OmBool arm_task_get_pitch2_debug_snapshot(
    float* pitch2_feedback_deg,
    float* pitch2_feedback_rpm,
    float* pitch2_feedback_current,
    float* pitch2_feedback_torque,
    float* pitch2_feedback_online,
    float* pitch2_target_deg);

/* 采集 pitch1/Damiao 的反馈与目标快照。
 * - 角度单位：deg
 * - 速度单位：rpm
 * - 力矩单位：N·m
 * - online：0/1
 */
OmBool arm_task_get_pitch1_debug_snapshot(
    float* pitch1_feedback_deg,
    float* pitch1_feedback_rpm,
    float* pitch1_feedback_torque,
    float* pitch1_feedback_online,
    float* pitch1_target_deg);

/* 采集 pitch3/Damiao 的反馈与目标快照。
 * - 角度单位：deg
 * - 速度单位：rpm
 * - 力矩单位：N·m
 * - online：0/1
 */
OmBool arm_task_get_pitch3_debug_snapshot(
    float* pitch3_feedback_deg,
    float* pitch3_feedback_rpm,
    float* pitch3_feedback_torque,
    float* pitch3_feedback_online,
    float* pitch3_target_deg);



/* 采集 big_yaw / pitch1 / pitch2 / pitch3 / grip 五轴的反馈、目标与控制量快照。
 * 所有角度单位为度（deg）；达妙轴导出力矩反馈。
 * 任一参数为 NULL 或上下文未初始化时返回 OM_FALSE。
 */
OmBool arm_task_get_debug_snapshot(
    float* big_yaw_feedback_deg,
    float* big_yaw_target_deg,
    float* big_yaw_force_feedback,
    float* pitch1_feedback_deg,
    float* pitch1_target_deg,
    float* pitch1_force_feedback,
    float* pitch2_feedback_deg,
    float* pitch2_target_deg,
    float* pitch2_force_feedback,
    float* pitch3_feedback_deg,
    float* pitch3_target_deg,
    float* pitch3_force_feedback,
    float* grip_feedback_deg,
    float* grip_target_deg,
    float* grip_force_feedback);

/* VTable 诊断回调：
 * - diag_online 返回 7 轴在线状态 bitmask
 * - diag_snapshot 返回固定 8 个 float：
 *   [0] big_yaw   (rad)
 *   [1] pitch1    (rad)
 *   [2] pitch2    (rad)
 *   [3] roll2     (rad)
 *   [4] pitch3    (rad)
 *   [5] roll3     (rad)
 *   [6] grip      (rad)
 *   [7] custom_controller_takeover_bit (bit)
 */
void arm_task_diag_online(void* ctx, uint8_t* out_online);
void arm_task_diag_snapshot(void* ctx, float* out_buf, uint32_t cap, uint32_t* out_count);

#endif
