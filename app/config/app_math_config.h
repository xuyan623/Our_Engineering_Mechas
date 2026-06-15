#ifndef NEW_ROBOT_APP_MATH_CONFIG_H
#define NEW_ROBOT_APP_MATH_CONFIG_H

/* app_math_config.h 只放数学实现与性能调优参数：
 * - IK 求解节拍
 *
 * 约束：
 * - trig / atan2 / sqrt 直接使用 CMSIS-DSP fast math
 * - 不再保留“是否启用 CMSIS”这种配置开关
 */

/* IK 求解节拍独立于 arm_task 主循环。
 * arm_task 仍以 3ms 跑控制，但高成本 IK 默认只按更低频率触发。
 */
#ifndef APP_ARM_IK_SOLVER_PERIOD_MS
#define APP_ARM_IK_SOLVER_PERIOD_MS (10u)
#endif

/* 正解开关：
 * - 1：允许 FK 公共入口与基于 FK 的上层能力
 * - 0：禁用 FK，相关调用返回失败
 *
 * 注意：
 * - RC_IK 模式进入时需要基于当前反馈做一次 FK 捕获目标 pose
 * - 因此关闭后，RC_IK 模式将无法建立目标 pose
 */
#ifndef APP_ARM_IK_FORWARD_ENABLE
#define APP_ARM_IK_FORWARD_ENABLE (1u)
#endif

#endif
