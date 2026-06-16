#include "algorithm/gravity_comp/gravity_comp.h"

#include "config/app_config.h"
#include "dsp/fast_math_functions.h"
#include "function/math_utils/math_utils.h"
#include <math.h>

typedef struct
{
    float q2;
    float q3;
    float q4;
    float q6;
    float sin_q23;
    float cos_q23;
    float sin_q4;
    float cos_q4;
    float sin_q6;
    float cos_q6;
} GravityPitchState;

static void gravity_chain_angles(float pitch1_motor_angle_rad, float pitch2_motor_angle_rad,
                                                    float pitch2_zero_angle_rad, float pitch3_motor_angle_rad,
                                                    float roll2_motor_angle_rad, float* q2, float* q3, float* q4,
                                                    float* q6)
{
    *q2 = APP_GRAVITY_PITCH2_Q2_OFFSET_RAD - pitch1_motor_angle_rad;
    *q3 =
        (pitch2_motor_angle_rad - pitch2_zero_angle_rad) / APP_AT_PITCH2_GEAR_RATIO +
        APP_GRAVITY_PITCH2_Q3_OFFSET_RAD;
    *q4 = -roll2_motor_angle_rad;
    *q6 = pitch3_motor_angle_rad + APP_GRAVITY_PITCH2_Q6_OFFSET_RAD;
}

static void gravity_pitch3_angles(float pitch1_motor_angle_rad, float pitch2_motor_angle_rad,
                                               float pitch2_zero_angle_rad, float pitch3_motor_angle_rad,
                                               float roll2_motor_angle_rad, float* q2, float* q3, float* q4, float* q6)
{
    *q2 = APP_GRAVITY_PITCH3_Q2_OFFSET_RAD + pitch1_motor_angle_rad;
    *q3 =
        (-pitch2_motor_angle_rad + pitch2_zero_angle_rad) / APP_AT_PITCH2_GEAR_RATIO -
        APP_GRAVITY_PITCH3_Q3_OFFSET_RAD;
    *q4 = -roll2_motor_angle_rad;
    *q6 = pitch3_motor_angle_rad - APP_GRAVITY_PITCH3_Q6_OFFSET_RAD;
}

static void gravity_build_chain_state(float pitch1_motor_angle_rad, float pitch2_motor_angle_rad,
                                                 float pitch2_zero_angle_rad, float pitch3_motor_angle_rad,
                                                 float roll2_motor_angle_rad, GravityPitchState* state)
{
    float q23 = 0.0f;

    if (state == OM_NULL)
    {
        return;
    }

    gravity_chain_angles(pitch1_motor_angle_rad, pitch2_motor_angle_rad, pitch2_zero_angle_rad,
                                            pitch3_motor_angle_rad, roll2_motor_angle_rad, &state->q2, &state->q3,
                                            &state->q4, &state->q6);

    q23 = state->q2 + state->q3;
    state->sin_q23 = arm_sin_f32(q23);
    state->cos_q23 = arm_cos_f32(q23);
    state->sin_q4 = arm_sin_f32(state->q4);
    state->cos_q4 = arm_cos_f32(state->q4);
    state->sin_q6 = arm_sin_f32(state->q6);
    state->cos_q6 = arm_cos_f32(state->q6);
}

static float gravity_pitch2_torque(const GravityPitchState* state)
{
    if (state == OM_NULL)
    {
        return 0.0f;
    }

    return APP_GRAVITY_ACCELERATION / -APP_AT_PITCH2_GEAR_RATIO *
           (state->cos_q23 * (APP_GRAVITY_PITCH2_LM34_M + APP_GRAVITY_PITCH2_LM56_M * state->cos_q6 +
                              APP_GRAVITY_PITCH2_M67_KG * APP_GRAVITY_PITCH2_D45_M) +
            state->sin_q23 * (APP_GRAVITY_M4_KG * APP_GRAVITY_PITCH2_RY4_M * state->sin_q4 +
                              APP_GRAVITY_PITCH2_LM56_M * state->cos_q4 * state->sin_q6));
}

static float gravity_roll2_torque(const GravityPitchState* state)
{
    if (state == OM_NULL)
    {
        return 0.0f;
    }

    return APP_GRAVITY_ACCELERATION * state->cos_q23 *
           (state->sin_q4 * (APP_GRAVITY_PITCH2_LM56_M * state->sin_q6));
}

static float gravity_pitch3_torque(float pitch1_motor_angle_rad, float pitch2_motor_angle_rad,
                                                            float pitch2_zero_angle_rad, float pitch3_motor_angle_rad,
                                                            float roll2_motor_angle_rad)
{
    float q2 = 0.0f;
    float q3 = 0.0f;
    float q4 = 0.0f;
    float q6 = 0.0f;
    float sin_q23 = 0.0f;
    float cos_q23 = 0.0f;
    float sin_q6 = 0.0f;
    float cos_q6 = 0.0f;
    float sin_q4 = 0.0f;
    float cos_q4 = 0.0f;

    gravity_pitch3_angles(pitch1_motor_angle_rad, pitch2_motor_angle_rad, pitch2_zero_angle_rad,
                                       pitch3_motor_angle_rad, roll2_motor_angle_rad, &q2, &q3, &q4, &q6);

    sin_q23 = arm_sin_f32(q2 + q3);
    cos_q23 = arm_cos_f32(q2 + q3);
    sin_q6 = arm_sin_f32(q6);
    cos_q6 = arm_cos_f32(q6);
    sin_q4 = arm_sin_f32(q4);
    cos_q4 = arm_cos_f32(q4);

    return APP_GRAVITY_ACCELERATION * APP_GRAVITY_PITCH3_LEVER_M *
           (cos_q6 * cos_q23 * cos_q4 - sin_q6 * sin_q23);
}

static float gravity_pitch1_torque(float pitch1_motor_angle_rad,
                                                           const GravityPitchState* state)
{
    const float q2 = APP_GRAVITY_Q2_OFFSET_RAD - pitch1_motor_angle_rad;
    float sin_q2 = 0.0f;
    float cos_q2 = 0.0f;
    const float pitch2_torque = gravity_pitch2_torque(state);
    float local_torque = 0.0f;

    sin_q2 = arm_sin_f32(q2);
    cos_q2 = arm_cos_f32(q2);
    local_torque =
        (APP_GRAVITY_PITCH1_LEVER_M * cos_q2 - APP_GRAVITY_M2_KG * APP_GRAVITY_PITCH1_RY2_M * sin_q2) *
        APP_GRAVITY_ACCELERATION;

    return -(pitch2_torque * APP_AT_PITCH2_GEAR_RATIO + local_torque);
}

OmRet gravity_torque_snapshot(float pitch1_motor_angle_rad, float pitch2_motor_angle_rad,
                                           float pitch2_zero_angle_rad, float pitch3_motor_angle_rad,
                                           float roll2_motor_angle_rad, GravityTorqueSnap* snapshot)
{
    GravityPitchState state = {0};

    if (snapshot == OM_NULL)
    {
        return OM_ERROR_NULL;
    }

    gravity_build_chain_state(pitch1_motor_angle_rad, pitch2_motor_angle_rad, pitch2_zero_angle_rad,
                                         pitch3_motor_angle_rad, roll2_motor_angle_rad, &state);

    snapshot->pitch2_torque_nm = gravity_pitch2_torque(&state);
    snapshot->roll2_torque_nm = gravity_roll2_torque(&state);
    snapshot->pitch3_torque_nm = gravity_pitch3_torque(
        pitch1_motor_angle_rad, pitch2_motor_angle_rad, pitch2_zero_angle_rad, pitch3_motor_angle_rad,
        roll2_motor_angle_rad);
    snapshot->pitch1_torque_nm = gravity_pitch1_torque(pitch1_motor_angle_rad, &state);
    return OM_OK;
}

float pitch2_grav_torque_calculate(float pitch1_motor_angle_rad, float pitch2_motor_angle_rad, float pitch2_zero_angle_rad,
                                   float pitch3_motor_angle_rad, float roll2_motor_angle_rad)
{
    GravityPitchState state = {0};

    gravity_build_chain_state(pitch1_motor_angle_rad, pitch2_motor_angle_rad, pitch2_zero_angle_rad,
                                         pitch3_motor_angle_rad, roll2_motor_angle_rad, &state);
    return gravity_pitch2_torque(&state);
}

float pitch3_grav_torque_calcuate(float pitch1_motor_angle_rad, float pitch2_motor_angle_rad, float pitch2_zero_angle_rad,
                                  float pitch3_motor_angle_rad, float roll2_motor_angle_rad)
{
    return gravity_pitch3_torque(
        pitch1_motor_angle_rad, pitch2_motor_angle_rad, pitch2_zero_angle_rad, pitch3_motor_angle_rad,
        roll2_motor_angle_rad);
}

float roll2_grav_torque_calculate(float pitch1_motor_angle_rad, float pitch2_motor_angle_rad, float pitch2_zero_angle_rad,
                                  float pitch3_motor_angle_rad, float roll2_motor_angle_rad)
{
    GravityPitchState state = {0};

    gravity_build_chain_state(pitch1_motor_angle_rad, pitch2_motor_angle_rad, pitch2_zero_angle_rad,
                                         pitch3_motor_angle_rad, roll2_motor_angle_rad, &state);
    return gravity_roll2_torque(&state);
}

float pitch1_grav_torque_calcuate(float pitch1_motor_angle_rad, float pitch2_motor_angle_rad, float pitch2_zero_angle_rad,
                                  float pitch3_motor_angle_rad, float roll2_motor_angle_rad)
{
    GravityPitchState state = {0};

    gravity_build_chain_state(pitch1_motor_angle_rad, pitch2_motor_angle_rad, pitch2_zero_angle_rad,
                                         pitch3_motor_angle_rad, roll2_motor_angle_rad, &state);
    return gravity_pitch1_torque(pitch1_motor_angle_rad, &state);
}
