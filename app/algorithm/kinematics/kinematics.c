#include "algorithm/kinematics/kinematics.h"

#include "config/app_config.h"
#include <math.h>
#include <string.h>

static float kinematics_clamp_float(float value, float min_value, float max_value)
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

static float kinematics_abs_float(float value)
{
    return (value < 0.0f) ? -value : value;
}

static float kinematics_normalize_angle(float angle_rad)
{
    const float two_pi = 2.0f * APP_PI;

    angle_rad = fmodf(angle_rad, two_pi);
    if (angle_rad > APP_PI)
    {
        angle_rad -= two_pi;
    }
    else if (angle_rad < -APP_PI)
    {
        angle_rad += two_pi;
    }

    return angle_rad;
}

static void kinematics_clamp_chassis_velocity(
    float* vx_mm_per_s,
    float* vy_mm_per_s,
    float* vw_deg_per_s)
{
    float linear_speed = 0.0f;
    float vw_linear_mm_per_s = 0.0f;
    float total_speed = 0.0f;
    float scale = 1.0f;

    if (vx_mm_per_s == OM_NULL || vy_mm_per_s == OM_NULL || vw_deg_per_s == OM_NULL)
    {
        return;
    }

    *vx_mm_per_s = kinematics_clamp_float(*vx_mm_per_s, -APP_CHASSIS_MAX_VX_MM_PER_S, APP_CHASSIS_MAX_VX_MM_PER_S);
    *vy_mm_per_s = kinematics_clamp_float(*vy_mm_per_s, -APP_CHASSIS_MAX_VY_MM_PER_S, APP_CHASSIS_MAX_VY_MM_PER_S);
    *vw_deg_per_s = kinematics_clamp_float(*vw_deg_per_s, -APP_CHASSIS_MAX_VW_DEG_PER_S, APP_CHASSIS_MAX_VW_DEG_PER_S);

#if (APP_CHASSIS_TOTAL_SPEED_LIMIT_ENABLE == 1u)
    {
        const float equivalent_radius_mm = (APP_CHASSIS_WHEEL_TRACK_MM + APP_CHASSIS_WHEEL_BASE_MM) / 2.0f;

        linear_speed = sqrtf((*vx_mm_per_s) * (*vx_mm_per_s) + (*vy_mm_per_s) * (*vy_mm_per_s));
        vw_linear_mm_per_s = (*vw_deg_per_s / APP_RADIAN_COEF) * equivalent_radius_mm;
        total_speed = sqrtf(linear_speed * linear_speed + vw_linear_mm_per_s * vw_linear_mm_per_s);

        if (total_speed > APP_CHASSIS_MAX_TOTAL_SPEED_MM_PER_S)
        {
            scale = APP_CHASSIS_MAX_TOTAL_SPEED_MM_PER_S / total_speed;
            *vx_mm_per_s *= scale;
            *vy_mm_per_s *= scale;
            *vw_deg_per_s *= scale;
        }
    }
#endif
}

static void kinematics_compute_mecanum_wheel_rpm_float(
    float vx_mm_per_s,
    float vy_mm_per_s,
    float vw_deg_per_s,
    float wheel_rpm_float[MECANUM_WHEEL_COUNT])
{
    const float rotate_ratio = ((APP_CHASSIS_WHEEL_BASE_MM + APP_CHASSIS_WHEEL_TRACK_MM) / 2.0f) / APP_RADIAN_COEF;
    const float wheel_rpm_ratio = 60.0f / (APP_CHASSIS_WHEEL_PERIMETER_MM * APP_CHASSIS_DECEL_RATIO);

    if (wheel_rpm_float == OM_NULL)
    {
        return;
    }

    wheel_rpm_float[MECANUM_WHEEL_FRONT_RIGHT] =
        (-vx_mm_per_s - vy_mm_per_s - vw_deg_per_s * rotate_ratio) * wheel_rpm_ratio;
    wheel_rpm_float[MECANUM_WHEEL_FRONT_LEFT] =
        (vx_mm_per_s - vy_mm_per_s - vw_deg_per_s * rotate_ratio) * wheel_rpm_ratio;
    wheel_rpm_float[MECANUM_WHEEL_BACK_LEFT] =
        (vx_mm_per_s + vy_mm_per_s - vw_deg_per_s * rotate_ratio) * wheel_rpm_ratio;
    wheel_rpm_float[MECANUM_WHEEL_BACK_RIGHT] =
        (-vx_mm_per_s + vy_mm_per_s - vw_deg_per_s * rotate_ratio) * wheel_rpm_ratio;
}

static void kinematics_scale_wheel_rpm_float(
    float wheel_rpm_float[MECANUM_WHEEL_COUNT],
    const OmBool active_wheel_flags[MECANUM_WHEEL_COUNT])
{
    float max_abs_rpm = 0.0f;
    uint32_t index = 0u;

    if (wheel_rpm_float == OM_NULL || active_wheel_flags == OM_NULL)
    {
        return;
    }

    for (index = 0u; index < MECANUM_WHEEL_COUNT; index++)
    {
        const float abs_value = kinematics_abs_float(wheel_rpm_float[index]);

        if (active_wheel_flags[index] != OM_TRUE)
        {
            continue;
        }

        if (abs_value > max_abs_rpm)
        {
            max_abs_rpm = abs_value;
        }
    }

    if (max_abs_rpm > APP_CHASSIS_MAX_WHEEL_RPM)
    {
        const float scale = APP_CHASSIS_MAX_WHEEL_RPM / max_abs_rpm;

        for (index = 0u; index < MECANUM_WHEEL_COUNT; index++)
        {
            if (active_wheel_flags[index] != OM_TRUE)
            {
                continue;
            }

            wheel_rpm_float[index] *= scale;
        }
    }
}

static void kinematics_write_wheel_rpm_int16(
    const float wheel_rpm_float[MECANUM_WHEEL_COUNT],
    int16_t wheel_speeds_rpm[MECANUM_WHEEL_COUNT])
{
    uint32_t index = 0u;

    if (wheel_rpm_float == OM_NULL || wheel_speeds_rpm == OM_NULL)
    {
        return;
    }

    for (index = 0u; index < MECANUM_WHEEL_COUNT; index++)
    {
        wheel_speeds_rpm[index] = (int16_t)wheel_rpm_float[index];
    }
}

/**
 * @brief 麦轮运动学正解算 - 将底盘速度转换为各轮转速
 * 
 * @details 根据给定的底盘线速度和角速度，计算麦轮底盘四个轮子各自的目标转速。
 *          该函数实现了麦轮运动学的正解算，考虑了底盘几何参数和减速比。
 *          输出结果会自动限幅，确保不超过最大轮速限制。
 * 
 * @param vx_mm_per_s 底盘X轴方向线速度（单位：mm/s），正值表示向前
 * @param vy_mm_per_s 底盘Y轴方向线速度（单位：mm/s），正值表示向左
 * @param vw_deg_per_s 底盘旋转角速度（单位：deg/s），正值表示逆时针旋转
 * @param wheel_speeds_rpm 输出的四轮转速数组（单位：RPM），数组大小为MECANUM_WHEEL_COUNT
 *                         索引对应关系：
 *                         - MECANUM_WHEEL_FRONT_RIGHT: 右前轮
 *                         - MECANUM_WHEEL_FRONT_LEFT: 左前轮
 *                         - MECANUM_WHEEL_BACK_LEFT: 左后轮
 *                         - MECANUM_WHEEL_BACK_RIGHT: 右后轮
 * 
 * @note 如果wheel_speeds_rpm为NULL，函数直接返回
 * @note 输入速度会被自动限制在允许的最大范围内
 * @note 当计算出的轮速超过最大值时，会按比例缩放所有轮速以保持运动方向
 */
void mecanum_calc(float vx_mm_per_s, float vy_mm_per_s, float vw_deg_per_s, int16_t wheel_speeds_rpm[MECANUM_WHEEL_COUNT])
{
    float wheel_rpm_float[MECANUM_WHEEL_COUNT] = {0.0f};
    const OmBool active_wheel_flags[MECANUM_WHEEL_COUNT] = {OM_TRUE, OM_TRUE, OM_TRUE, OM_TRUE};

    if (wheel_speeds_rpm == OM_NULL)
    {
        return;
    }

    kinematics_clamp_chassis_velocity(&vx_mm_per_s, &vy_mm_per_s, &vw_deg_per_s);
    kinematics_compute_mecanum_wheel_rpm_float(vx_mm_per_s, vy_mm_per_s, vw_deg_per_s, wheel_rpm_float);
    kinematics_scale_wheel_rpm_float(wheel_rpm_float, active_wheel_flags);
    kinematics_write_wheel_rpm_int16(wheel_rpm_float, wheel_speeds_rpm);
}

void mecanum_calc_three_wheel(
    float vx_mm_per_s,
    float vy_mm_per_s,
    float vw_deg_per_s,
    MecanumWheelId offline_wheel_id,
    int16_t wheel_speeds_rpm[MECANUM_WHEEL_COUNT])
{
    float wheel_rpm_float[MECANUM_WHEEL_COUNT] = {0.0f};
    OmBool active_wheel_flags[MECANUM_WHEEL_COUNT] = {OM_TRUE, OM_TRUE, OM_TRUE, OM_TRUE};

    if (wheel_speeds_rpm == OM_NULL)
    {
        return;
    }

    switch (offline_wheel_id)
    {
    case MECANUM_WHEEL_FRONT_RIGHT:
        active_wheel_flags[MECANUM_WHEEL_FRONT_RIGHT] = OM_FALSE;
        break;
    case MECANUM_WHEEL_FRONT_LEFT:
        active_wheel_flags[MECANUM_WHEEL_FRONT_LEFT] = OM_FALSE;
        break;
    case MECANUM_WHEEL_BACK_LEFT:
        active_wheel_flags[MECANUM_WHEEL_BACK_LEFT] = OM_FALSE;
        break;
    case MECANUM_WHEEL_BACK_RIGHT:
        active_wheel_flags[MECANUM_WHEEL_BACK_RIGHT] = OM_FALSE;
        break;
    default:
        memset(wheel_speeds_rpm, 0, sizeof(int16_t) * MECANUM_WHEEL_COUNT);
        return;
    }

    kinematics_clamp_chassis_velocity(&vx_mm_per_s, &vy_mm_per_s, &vw_deg_per_s);
    kinematics_compute_mecanum_wheel_rpm_float(vx_mm_per_s, vy_mm_per_s, vw_deg_per_s, wheel_rpm_float);
    wheel_rpm_float[offline_wheel_id] = 0.0f;
    kinematics_scale_wheel_rpm_float(wheel_rpm_float, active_wheel_flags);
    kinematics_write_wheel_rpm_int16(wheel_rpm_float, wheel_speeds_rpm);
}

/**
 * @brief 机械臂逆运动学解算 - 将末端位置转换为关节电机角度
 * 
 * @details 根据给定的机械臂末端在XZ平面的笛卡尔坐标，计算两个俯仰关节（pitch1和pitch2）
 *          的目标电机角度。该函数实现了二连杆机械臂的几何逆解，采用解析法求解。
 *          
 *          解算过程：
 *          1. 基于连杆长度和末端位置构建几何方程
 *          2. 求解肘关节角度（elbow angle）
 *          3. 计算两个俯仰关节的角度
 *          4. 进行角度归一化和零点偏移补偿
 *          5. 限幅到允许的关节角度范围
 *          
 *          注意：当存在多解时，选择"肘关节在下"的配置以保持与旧工程一致。
 * 
 * @param x_mm 机械臂末端在X轴方向的坐标（单位：mm），相对于基座坐标系
 * @param z_mm 机械臂末端在Z轴方向的坐标（单位：mm），相对于基座坐标系
 * @param pitch1_motor_angle_rad 输出的第一俯仰关节电机角度（单位：rad），已包含零点偏移
 * @param pitch2_motor_angle_rad 输出的第二俯仰关节电机角度（单位：rad），已考虑减速比
 * 
 * @return OmRet 返回执行结果
 *         - OM_OK: 解算成功
 *         - OM_ERROR_NULL: 输出指针为空
 *         - OM_ERROR_PARAM: 参数无效（目标位置超出工作空间）
 * 
 * @note pitch2_motor_angle_rad 输出的是电机轴角度，已乘以减速比 APP_ARM_PITCH2_GEAR_RATIO
 * @note 如果目标位置超出机械臂可达工作空间，函数会返回 OM_ERROR_PARAM
 * @note 输出角度会自动限制在 [MIN_RAD, MAX_RAD] 范围内
 */
OmRet Change_Position_to_Motor_Angle(float x_mm, float z_mm, float* pitch1_motor_angle_rad, float* pitch2_motor_angle_rad)
{
    const float arm_coefficient_a = -2.0f * APP_ARM_LINK_A2_MM * z_mm - 2.0f * APP_ARM_LINK_D3_MM * x_mm;
    const float arm_coefficient_b = -2.0f * APP_ARM_LINK_A2_MM * x_mm + 2.0f * APP_ARM_LINK_D3_MM * z_mm;
    const float arm_coefficient_c =
        APP_ARM_LINK_A1_MM * APP_ARM_LINK_A1_MM -
        (z_mm * z_mm + x_mm * x_mm + APP_ARM_LINK_A2_MM * APP_ARM_LINK_A2_MM + APP_ARM_LINK_D3_MM * APP_ARM_LINK_D3_MM);
    const float denominator = sqrtf(arm_coefficient_a * arm_coefficient_a + arm_coefficient_b * arm_coefficient_b);
    float phi = 0.0f;
    float elbow_angle = 0.0f;
    float pitch1_angle = 0.0f;
    float pitch2_angle = 0.0f;
    float asin_input = 0.0f;

    if (pitch1_motor_angle_rad == OM_NULL || pitch2_motor_angle_rad == OM_NULL)
    {
        return OM_ERROR_NULL;
    }

    if (denominator <= 0.0f)
    {
        return OM_ERROR_PARAM;
    }

    asin_input = kinematics_clamp_float(arm_coefficient_c / denominator, -1.0f, 1.0f);
    phi = atan2f(arm_coefficient_b, arm_coefficient_a);

    /* 保持与旧工程一致，选择“肘关节在下”的解。 */
    elbow_angle = asinf(asin_input) - phi;
    pitch1_angle = atan2f(z_mm - APP_ARM_LINK_A2_MM * sinf(elbow_angle) + APP_ARM_LINK_D3_MM * cosf(elbow_angle),
                          x_mm - APP_ARM_LINK_D3_MM * sinf(elbow_angle) - APP_ARM_LINK_A2_MM * cosf(elbow_angle));
    pitch2_angle = elbow_angle - pitch1_angle;

    pitch1_angle = -kinematics_normalize_angle(pitch1_angle) + APP_ARM_PITCH1_ZERO_OFFSET_RAD;
    pitch2_angle = kinematics_normalize_angle(pitch2_angle) + APP_ARM_PITCH2_ZERO_OFFSET_RAD;

    pitch1_angle = kinematics_clamp_float(pitch1_angle, APP_ARM_PITCH1_MIN_RAD, APP_ARM_PITCH1_MAX_RAD);
    pitch2_angle = kinematics_clamp_float(pitch2_angle, APP_ARM_PITCH2_MIN_RAD, APP_ARM_PITCH2_MAX_RAD);

    *pitch1_motor_angle_rad = pitch1_angle;
    *pitch2_motor_angle_rad = pitch2_angle * APP_ARM_PITCH2_GEAR_RATIO;

    return OM_OK;
}
