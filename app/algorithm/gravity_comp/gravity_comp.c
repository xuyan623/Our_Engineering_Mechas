#include "algorithm/gravity_comp/gravity_comp.h"

#include "config/app_config.h"
#include <math.h>

static void gravity_comp_resolve_pitch_chain_angles(float pitch1_motor_angle_rad, float pitch2_motor_angle_rad,
                                                    float pitch2_zero_angle_rad, float pitch3_motor_angle_rad,
                                                    float roll2_motor_angle_rad, float* q2, float* q3, float* q4,
                                                    float* q6)
{
    *q2 = APP_GRAVITY_PITCH2_Q2_OFFSET_RAD - pitch1_motor_angle_rad;
    *q3 =
        (pitch2_motor_angle_rad - pitch2_zero_angle_rad) / APP_ARM_PITCH2_GEAR_RATIO +
        APP_GRAVITY_PITCH2_Q3_OFFSET_RAD;
    *q4 = -roll2_motor_angle_rad;
    *q6 = pitch3_motor_angle_rad + APP_GRAVITY_PITCH2_Q6_OFFSET_RAD;
}

static void gravity_comp_resolve_pitch3_angles(float pitch1_motor_angle_rad, float pitch2_motor_angle_rad,
                                               float pitch2_zero_angle_rad, float pitch3_motor_angle_rad,
                                               float roll2_motor_angle_rad, float* q2, float* q3, float* q4, float* q6)
{
    *q2 = APP_GRAVITY_PITCH3_Q2_OFFSET_RAD + pitch1_motor_angle_rad;
    *q3 =
        (-pitch2_motor_angle_rad + pitch2_zero_angle_rad) / APP_ARM_PITCH2_GEAR_RATIO -
        APP_GRAVITY_PITCH3_Q3_OFFSET_RAD;
    *q4 = -roll2_motor_angle_rad;
    *q6 = pitch3_motor_angle_rad - APP_GRAVITY_PITCH3_Q6_OFFSET_RAD;
}

float pitch2_grav_torque_calculate(float pitch1_motor_angle_rad, float pitch2_motor_angle_rad, float pitch2_zero_angle_rad,
                                   float pitch3_motor_angle_rad, float roll2_motor_angle_rad)
{
    float q2 = 0.0f;
    float q3 = 0.0f;
    float q4 = 0.0f;
    float q6 = 0.0f;
    float torque = 0.0f;

    gravity_comp_resolve_pitch_chain_angles(pitch1_motor_angle_rad, pitch2_motor_angle_rad, pitch2_zero_angle_rad,
                                            pitch3_motor_angle_rad, roll2_motor_angle_rad, &q2, &q3, &q4, &q6);

    torque = APP_GRAVITY_ACCELERATION / -APP_ARM_PITCH2_GEAR_RATIO *
             (cosf(q2 + q3) * (APP_GRAVITY_PITCH2_LM34_M + APP_GRAVITY_PITCH2_LM56_M * cosf(q6) +
                               APP_GRAVITY_PITCH2_M67_KG * APP_GRAVITY_PITCH2_D45_M) +
              sinf(q2 + q3) * (APP_GRAVITY_M4_KG * APP_GRAVITY_PITCH2_RY4_M * sinf(q4) +
                               APP_GRAVITY_PITCH2_LM56_M * cosf(q4) * sinf(q6)));

    return torque;
}

float pitch3_grav_torque_calcuate(float pitch1_motor_angle_rad, float pitch2_motor_angle_rad, float pitch2_zero_angle_rad,
                                  float pitch3_motor_angle_rad, float roll2_motor_angle_rad)
{
    float q2 = 0.0f;
    float q3 = 0.0f;
    float q4 = 0.0f;
    float q6 = 0.0f;

    gravity_comp_resolve_pitch3_angles(pitch1_motor_angle_rad, pitch2_motor_angle_rad, pitch2_zero_angle_rad,
                                       pitch3_motor_angle_rad, roll2_motor_angle_rad, &q2, &q3, &q4, &q6);

    return APP_GRAVITY_ACCELERATION * APP_GRAVITY_PITCH3_EFFECTIVE_LEVER_M *
           (cosf(q6) * cosf(q2 + q3) * cosf(q4) - sinf(q6) * sinf(q2 + q3));
}

float roll2_grav_torque_calculate(float pitch1_motor_angle_rad, float pitch2_motor_angle_rad, float pitch2_zero_angle_rad,
                                  float pitch3_motor_angle_rad, float roll2_motor_angle_rad)
{
    float q2 = 0.0f;
    float q3 = 0.0f;
    float q4 = 0.0f;
    float q6 = 0.0f;

    gravity_comp_resolve_pitch_chain_angles(pitch1_motor_angle_rad, pitch2_motor_angle_rad, pitch2_zero_angle_rad,
                                            pitch3_motor_angle_rad, roll2_motor_angle_rad, &q2, &q3, &q4, &q6);

    return APP_GRAVITY_ACCELERATION * cosf(q2 + q3) * (sinf(q4) * (APP_GRAVITY_PITCH2_LM56_M * sinf(q6)));
}

float pitch1_grav_torque_calcuate(float pitch1_motor_angle_rad, float pitch2_motor_angle_rad, float pitch2_zero_angle_rad,
                                  float pitch3_motor_angle_rad, float roll2_motor_angle_rad)
{
    const float q2 = APP_GRAVITY_Q2_OFFSET_RAD - pitch1_motor_angle_rad;
    const float pitch2_torque =
        pitch2_grav_torque_calculate(pitch1_motor_angle_rad, pitch2_motor_angle_rad, pitch2_zero_angle_rad,
                                     pitch3_motor_angle_rad, roll2_motor_angle_rad);
    const float local_torque =
        (APP_GRAVITY_PITCH1_EQUIVALENT_LEVER_M * cosf(q2) - APP_GRAVITY_M2_KG * APP_GRAVITY_PITCH1_RY2_M * sinf(q2)) *
        APP_GRAVITY_ACCELERATION;

    return -(pitch2_torque * APP_ARM_PITCH2_GEAR_RATIO + local_torque);
}
