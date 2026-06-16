#ifndef NEW_ROBOT_KINEMATICS_H
#define NEW_ROBOT_KINEMATICS_H

#include "core/om_def.h"
#include <stdint.h>

#define MECANUM_WHEEL_COUNT (4u)

typedef enum
{
    MECANUM_WHEEL_FRONT_RIGHT = 0,
    MECANUM_WHEEL_FRONT_LEFT,
    MECANUM_WHEEL_BACK_LEFT,
    MECANUM_WHEEL_BACK_RIGHT
} MecanumWheelId;

void mecanum_calc(float vx_mm_per_s, float vy_mm_per_s, float vw_deg_per_s, int16_t wheel_speeds_rpm[MECANUM_WHEEL_COUNT]);
/* 三麦轮降级解算：
 * - offline_wheel_id 指明当前掉线轮位
 * - 仅对剩余三轮输出目标转速
 * - 掉线轮参考强制为 0，限幅只按存活三轮计算
 */
void mecanum_calc_three_wheel(
    float vx_mm_per_s,
    float vy_mm_per_s,
    float vw_deg_per_s,
    MecanumWheelId offline_wheel_id,
    int16_t wheel_speeds_rpm[MECANUM_WHEEL_COUNT]);

/* legacy 2 连杆逆解 helper，当前不参与 6 轴机械臂实时 IK。
 * 输出的是电机参考角：
 * - pitch1_motor_angle_rad：Pitch1 电机参考角（rad）
 * - pitch2_motor_angle_rad：Pitch2 电机参考角（rad），已乘减速比
 */
OmRet kin_pos_to_motor(float x_mm, float z_mm, float* pitch1_motor_angle_rad, float* pitch2_motor_angle_rad);

#endif
