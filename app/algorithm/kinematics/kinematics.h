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

/* 输出的是电机参考角：
 * - pitch1_motor_angle_rad：Pitch1 电机参考角（rad）
 * - pitch2_motor_angle_rad：Pitch2 电机参考角（rad），已乘减速比
 */
OmRet Change_Position_to_Motor_Angle(float x_mm, float z_mm, float* pitch1_motor_angle_rad, float* pitch2_motor_angle_rad);

#endif
