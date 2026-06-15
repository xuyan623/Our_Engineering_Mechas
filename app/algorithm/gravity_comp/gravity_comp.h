#ifndef NEW_ROBOT_GRAVITY_COMP_H
#define NEW_ROBOT_GRAVITY_COMP_H

#include "core/om_def.h"

typedef struct
{
    float pitch1_torque_nm;
    float pitch2_torque_nm;
    float roll2_torque_nm;
    float pitch3_torque_nm;
} GravityCompTorqueSnapshot;

/* 输入角度均为当前电机反馈角，单位 rad：
 * - pitch1_motor_angle_rad：Pitch1 电机角
 * - pitch2_motor_angle_rad：Pitch2 电机角
 * - pitch2_zero_angle_rad：Pitch2 零位角（对应旧工程 GO8010_init_angle1）
 * - pitch3_motor_angle_rad：Pitch3 电机角
 * - roll2_motor_angle_rad：Roll2 电机角
 */
OmRet gravity_comp_compute_torque_snapshot(float pitch1_motor_angle_rad, float pitch2_motor_angle_rad,
                                           float pitch2_zero_angle_rad, float pitch3_motor_angle_rad,
                                           float roll2_motor_angle_rad, GravityCompTorqueSnapshot* snapshot);
float pitch1_grav_torque_calcuate(float pitch1_motor_angle_rad, float pitch2_motor_angle_rad, float pitch2_zero_angle_rad,
                                  float pitch3_motor_angle_rad, float roll2_motor_angle_rad);
float pitch2_grav_torque_calculate(float pitch1_motor_angle_rad, float pitch2_motor_angle_rad, float pitch2_zero_angle_rad,
                                   float pitch3_motor_angle_rad, float roll2_motor_angle_rad);
float pitch3_grav_torque_calcuate(float pitch1_motor_angle_rad, float pitch2_motor_angle_rad, float pitch2_zero_angle_rad,
                                  float pitch3_motor_angle_rad, float roll2_motor_angle_rad);
float roll2_grav_torque_calculate(float pitch1_motor_angle_rad, float pitch2_motor_angle_rad, float pitch2_zero_angle_rad,
                                  float pitch3_motor_angle_rad, float roll2_motor_angle_rad);

#endif
