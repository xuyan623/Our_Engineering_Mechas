#ifndef NEW_ROBOT_MATH_UTILS_H
#define NEW_ROBOT_MATH_UTILS_H

/* 通用数学工具函数库。
 * 职责：提供各任务/模块共享的纯数学运算，不依赖任何运行时状态。
 *
 * 设计原则：
 * - 所有函数均为无副作用的纯函数
 * - 不依赖 OSAL、硬件或业务上下文
 * - 角度相关函数统一使用 APP_PI 常量（来自 config/app_config.h）
 */

#include <stdint.h>

/* 值裁剪：将 value 限制在 [min_value, max_value] 范围内。 */
float math_utils_clamp_float(float value, float min_value, float max_value);

/* 浮点绝对值。 */
float math_utils_abs_float(float value);

/* 弧度转角度。 */
float math_utils_rad_to_deg(float angle_rad);

/* 角度转弧度。 */
float math_utils_deg_to_rad(float angle_deg);

/* 角度归一化到 (-180, 180] 区间。 */
float math_utils_normalize_deg(float angle_deg);

/* 在参考角附近找到与目标角最近的等效角（单位：度）。
 * 用于处理角度环绕问题，例如 roll3 单圈物理角。
 */
float math_utils_resolve_nearest_equivalent_deg(float target_deg, float reference_deg);

/* 在参考角附近找到与目标角最近的等效角（单位：弧度）。 */
float math_utils_resolve_nearest_equivalent_rad(float target_rad, float reference_rad);

/* 斜坡限速：将 current_value 以不超过 max_rate * dt 的步长逼近 target_value。
 * 当 dt_s <= 0 或 max_rate <= 0 时直接返回 target_value。
 */
float math_utils_slew_value(
    float current_value,
    float target_value,
    float max_rate,
    float dt_s);

/* 角速度单位换算：rad/s → rpm。 */
float math_utils_rad_per_s_to_rpm(float angular_velocity_rad_per_s);

/* 对称死区：当 |value_deg| <= deadband_deg 时返回 0，否则减去死区偏移。 */
float math_utils_apply_symmetric_deadband(float value_deg, float deadband_deg);

#endif
