/* 通用数学工具函数库实现。
 * 所有函数均为无副作用的纯函数，不依赖任何运行时状态。
 */

#include "function/math_utils/math_utils.h"
#include "config/app_config.h"
#include <math.h>

float math_utils_clamp_float(float value, float min_value, float max_value)
{
    if (value < min_value)
    {
        return min_value;
    }
    if (value > max_value)
    {
        return max_value;
    }
    return value;
}

float math_utils_abs_float(float value)
{
    return (value < 0.0f) ? -value : value;
}

float math_utils_rad_to_deg(float angle_rad)
{
    return angle_rad * (180.0f / APP_PI);
}

float math_utils_deg_to_rad(float angle_deg)
{
    return angle_deg * (APP_PI / 180.0f);
}

float math_utils_wrap_pi_f32(float angle_rad)
{
    const float two_pi = 2.0f * APP_PI;

    if (angle_rad > APP_PI)
    {
        if (angle_rad <= 3.0f * APP_PI)
        {
            return angle_rad - two_pi;
        }

        angle_rad = fmodf(angle_rad + APP_PI, two_pi);
        if (angle_rad < 0.0f)
        {
            angle_rad += two_pi;
        }
        return angle_rad - APP_PI;
    }

    if (angle_rad <= -APP_PI)
    {
        if (angle_rad > -3.0f * APP_PI)
        {
            return angle_rad + two_pi;
        }

        angle_rad = fmodf(angle_rad - APP_PI, two_pi);
        if (angle_rad > 0.0f)
        {
            angle_rad -= two_pi;
        }
        return angle_rad + APP_PI;
    }

    return angle_rad;
}

float math_utils_normalize_deg(float angle_deg)
{
    while (angle_deg > 180.0f)
    {
        angle_deg -= 360.0f;
    }
    while (angle_deg <= -180.0f)
    {
        angle_deg += 360.0f;
    }
    return angle_deg;
}

float math_utils_resolve_nearest_equivalent_deg(float target_deg, float reference_deg)
{
    float target_normalized_deg = math_utils_normalize_deg(target_deg);
    float reference_normalized_deg = math_utils_normalize_deg(reference_deg);
    float delta_deg = target_normalized_deg - reference_normalized_deg;

    while (delta_deg > 180.0f)
    {
        target_normalized_deg -= 360.0f;
        delta_deg -= 360.0f;
    }
    while (delta_deg < -180.0f)
    {
        target_normalized_deg += 360.0f;
        delta_deg += 360.0f;
    }

    return target_normalized_deg + (reference_deg - reference_normalized_deg);
}

float math_utils_resolve_nearest_equivalent_rad(float target_rad, float reference_rad)
{
    return reference_rad + math_utils_wrap_pi_f32(target_rad - reference_rad);
}

float math_utils_slew_value(
    float current_value,
    float target_value,
    float max_rate,
    float dt_s)
{
    float max_step = 0.0f;
    float delta = 0.0f;

    if (dt_s <= 0.0f || max_rate <= 0.0f)
    {
        return target_value;
    }

    max_step = max_rate * dt_s;
    delta = target_value - current_value;

    if (delta > max_step)
    {
        return current_value + max_step;
    }
    if (delta < -max_step)
    {
        return current_value - max_step;
    }
    return target_value;
}

float math_utils_rad_per_s_to_rpm(float angular_velocity_rad_per_s)
{
    return angular_velocity_rad_per_s * (60.0f / (2.0f * APP_PI));
}

float math_utils_apply_symmetric_deadband(float value_deg, float deadband_deg)
{
    if (value_deg > deadband_deg)
    {
        return value_deg - deadband_deg;
    }
    if (value_deg < -deadband_deg)
    {
        return value_deg + deadband_deg;
    }

    return 0.0f;
}
