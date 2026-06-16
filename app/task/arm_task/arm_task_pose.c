#include "task/arm_task/arm_task_internal.h"

#include "algorithm/gravity_comp/gravity_comp.h"
#include "config/app_config.h"
#include "driver/motor/motor.h"
#include "function/math_utils/math_utils.h"
#include "module/motor_tx_dispatch/motor_tx_dispatch.h"
#include "module/system_health/system_health.h"
#include "osal/osal.h"
#include "osal/osal_config.h"
#include "osal/osal_time.h"
#include "task/mode_task/mode_task.h"
#include "task/motor_communications_task/mct.h"
#include <string.h>

/* 机械臂动作表现已统一成最终机构角语义：
 * - 每一轴都表示“反馈逆映射后的机构角 - 该轴零点”
 * - 不再存在 normal + delta 的二层拼接
 * - normal 姿态固定为全 0
 */
/* sw2=DN / sw2=UP 默认 / CHECK / PITCH3_TORQUE_COLLECTION / CUSTOM_CONTROLLER_NORMAL */
const ArmTaskMachinePose g_arm_pose_zero = {
    {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f}};
/* normal 姿态在统一最终机构角语义下也是全 0。 */
const ArmTaskMachinePose g_arm_pose_normal = {
    {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f}};
/* sw2=MI + sw1=UP + iw上边沿 -> GET_ENERGY_UNIT */
// static const ArmTaskMachinePose g_arm_pose_get_energy = {
//     {0.0f, 1.24218f, 1.19447f, 0.0f, 0.0f, 3.7306414f, -1.8f}};
const ArmTaskMachinePose g_arm_pose_get_energy = {
    {0.0f, 0.64218f, 1.0447f, 0.8f, 1.3f, 0.0f, 0.0f}};
/* sw2=MI + sw1=DN + iw上边沿：与 GET_ENERGY_UNIT2 交替切换 */
// static const ArmTaskMachinePose g_arm_pose_get_energy1 = {
//     {-0.00667f, 1.035f, (5.53f / 6.33f) + 0.34f + 0.1f, 0.6178f, -0.194f, 2.1530383f, -1.8f}};
const ArmTaskMachinePose g_arm_pose_get_energy1 = {
    {0.0f, 0.64218f, 1.0447f, -0.8f, 1.3f, 0.0f, 0.0f}};
/* 保留旧的 GET_ENERGY_UNIT2 姿态定义；当前 formal 触发链不再直接在 1/2 间交替。 */
const ArmTaskMachinePose g_arm_pose_get_energy2 = {
    {0.148584366f, 0.99088f, 1.14010f, 0.093270302f, 0.07834f, -0.9493894f, -1.8f}};
/* GET_ENERGY_UNIT 三段动作之第三段 / EXCHANGE PICK_ACTION1 中间态 */
const ArmTaskMachinePose g_arm_pose_store_energy = {
    {1.141f, 1.1042f, 1.18567f, 0.016975f, -1.55555f, -0.8817230f, 0.0f}};
/* GET_ENERGY_UNIT2 二段动作之第二段 / EXCHANGE PICK_ACTION2 中间态 */
const ArmTaskMachinePose g_arm_pose_store_energy1 = {
    {-1.9018459f, 1.00080109f, 1.008974f, 0.09136295f, -1.222925131f, 0.3569913f, 0.0f}};
/* sw2=MI + sw1=MI + iw上边沿 -> EXCHANGE */
const ArmTaskMachinePose g_arm_pose_exchange = {
    {0.0f, 0.64218f, 1.0447f, 0.0f, 0.0f, 0.0f, 0.0f}};
/* EXCHANGE 子动作 PICK_ACTION1 后半段（sw1=DN 边沿触发） */
const ArmTaskMachinePose g_arm_pose_exchange_pick = {
    {1.041f, 1.2042f, 1.08567f, 0.016975f, -1.55555f, -0.8817230f, -1.8f}};
/* EXCHANGE 子动作 PICK_ACTION2 后半段（sw1=UP 边沿触发） */
const ArmTaskMachinePose g_arm_pose_exchange_pick1 = {
    {-1.90189481f, 1.1f, 1.00948341f, 0.092153325f, -1.363282f, 0.2719746f, -1.8f}};
/* sw2=MI + sw1=DN + iw上边沿 -> PRIMARY */
const ArmTaskMachinePose g_arm_pose_primary = {
    {0.0f, 1.46691f, 2.0053f, 0.1192f, -1.6f, -3.1415928f, 0.0f}};
/* sw2=UP + sw1=MI + iw上边沿 -> SECONDARY_ORE */
const ArmTaskMachinePose g_arm_pose_secondary_ore = {
    {0.0f, 1.48691f, 0.85f, -1.57f, 0.0f, 0.0f, 0.0f}};

void arm_task_assign_pose(ArmTaskMachinePose* target, const ArmTaskMachinePose* source)
{
    if (target == OM_NULL || source == OM_NULL)
    {
        return;
    }

    memcpy(target, source, sizeof(*target));
}

void arm_task_apply_custom_pose(
    ArmTaskContext* context,
    const ArmCustomSnapshot* controller_snapshot,
    ArmTaskMachinePose* pose)
{
    float filtered_delta_deg[AT_CUSTOM_AXIS_COUNT] = {0.0f};

    if (context == OM_NULL || controller_snapshot == OM_NULL || pose == OM_NULL)
    {
        return;
    }

    arm_task_assign_pose(pose, &g_arm_pose_zero);
    arm_task_filter_custom_deg(
        context,
        controller_snapshot,
        filtered_delta_deg);

    /* 当前控制器原始 6 轴在 VOFA 上仍按 Y/Z/X/Yaw/Pitch/Roll 展示。
     * 这里按现场已核对的机械臂语义接管：
     * - I2(Y)   -> big_yaw
     * - I3(Z)   -> pitch1，方向取反
     * - I4(X)   -> pitch2，方向取反
     * - I5(Yaw) -> roll2
     * - I6(Pitch) -> pitch3
     * - I7(Roll)  -> roll3
     */
    pose->machine_values[AT_MACHINE_BIG_YAW] =
        math_utils_deg_to_rad(filtered_delta_deg[AT_CUSTOM_AXIS_Y]);
    pose->machine_values[AT_MACHINE_PITCH1] =
        math_utils_deg_to_rad(-filtered_delta_deg[AT_CUSTOM_AXIS_Z]);
    pose->machine_values[AT_MACHINE_PITCH2] =
        math_utils_deg_to_rad(-filtered_delta_deg[AT_CUSTOM_AXIS_X]);
    pose->machine_values[AT_MACHINE_ROLL2] =
        math_utils_deg_to_rad(filtered_delta_deg[AT_CUSTOM_AXIS_YAW]);
    pose->machine_values[AT_MACHINE_PITCH3] =
        math_utils_deg_to_rad(filtered_delta_deg[AT_CUSTOM_AXIS_PITCH]);
    pose->machine_values[AT_MACHINE_ROLL3] =
        context->custom_roll3_reference_rad +
        math_utils_deg_to_rad(filtered_delta_deg[AT_CUSTOM_AXIS_ROLL]);
    pose->machine_values[AT_MACHINE_GRIP] =
        context->custom_grip_reference_rad;
}

/* EXCHANGE/PICK_ACTION1 的分段姿态推进。 */
void arm_task_exchange_pick_one(ArmTaskMachinePose* pose, OsalTimeMs elapsed_ms)
{
    if (pose == OM_NULL)
    {
        return;
    }

    arm_task_assign_pose(pose, &g_arm_pose_exchange);

    if (elapsed_ms >= 100u)
    {
        arm_task_assign_pose(pose, &g_arm_pose_store_energy);
    }
    if (elapsed_ms >= 1200u)
    {
        arm_task_assign_pose(pose, &g_arm_pose_exchange_pick);
    }
    if (elapsed_ms >= 1400u)
    {
        pose->machine_values[AT_MACHINE_PITCH2] = 1.04067f;
        pose->machine_values[AT_MACHINE_ROLL2] = 0.05f;
    }
    if (elapsed_ms >= 1550u)
    {
        pose->machine_values[AT_MACHINE_PITCH2] = 0.82f;
        pose->machine_values[AT_MACHINE_PITCH3] = -1.0f;
        pose->machine_values[AT_MACHINE_ROLL2] = -0.04512f;
        pose->machine_values[AT_MACHINE_ROLL3] = -0.8611927f;
    }
    if (elapsed_ms >= 1800u)
    {
        pose->machine_values[AT_MACHINE_GRIP] = 0.0f;
    }
    if (elapsed_ms >= 2000u)
    {
        pose->machine_values[AT_MACHINE_PITCH2] = 1.3f;
    }
}

/* EXCHANGE/PICK_ACTION2 的分段姿态推进。 */
void arm_task_exchange_pick_two(ArmTaskMachinePose* pose, OsalTimeMs elapsed_ms)
{
    if (pose == OM_NULL)
    {
        return;
    }

    arm_task_assign_pose(pose, &g_arm_pose_exchange);

    if (elapsed_ms >= 100u)
    {
        arm_task_assign_pose(pose, &g_arm_pose_store_energy1);
    }
    if (elapsed_ms >= 1200u)
    {
        arm_task_assign_pose(pose, &g_arm_pose_exchange_pick1);
    }
    if (elapsed_ms >= 1300u)
    {
        pose->machine_values[AT_MACHINE_PITCH2] = 0.87f;
        pose->machine_values[AT_MACHINE_PITCH3] = -1.19f;
    }
    if (elapsed_ms >= 1460u)
    {
        pose->machine_values[AT_MACHINE_BIG_YAW] = -2.00189481f;
        pose->machine_values[AT_MACHINE_PITCH2] = 0.810012383f;
        pose->machine_values[AT_MACHINE_PITCH3] = -1.11f;
        pose->machine_values[AT_MACHINE_PITCH1] = 1.20f;
    }
    if (elapsed_ms >= 2170u)
    {
        pose->machine_values[AT_MACHINE_GRIP] = 0.0f;
    }
    if (elapsed_ms >= 2390u)
    {
        pose->machine_values[AT_MACHINE_PITCH2] = 1.6f;
        pose->machine_values[AT_MACHINE_PITCH3] = -1.45f;
    }
}

/* GET_ENERGY_UNIT 三段动作：
 * ACTION_ONE 开夹爪，ACTION_TWO 调整 pitch2/pitch3，ACTION_THREE 切到存放姿态。
 */
void arm_task_apply_energy(ArmTaskMachinePose* pose, ClampAction action, OsalTimeMs elapsed_ms)
{
    arm_task_assign_pose(pose, &g_arm_pose_get_energy);

    switch (action)
    {
    case MODE_CLAMP_ACTION_ONE:
        pose->machine_values[AT_MACHINE_GRIP] = 0.0f;
        break;
    case MODE_CLAMP_ACTION_TWO:
        pose->machine_values[AT_MACHINE_PITCH3] = 0.43f;
        pose->machine_values[AT_MACHINE_PITCH2] = -0.43f + 1.19447f;
        break;
    case MODE_CLAMP_ACTION_THREE:
        arm_task_assign_pose(pose, &g_arm_pose_store_energy);
        if (elapsed_ms >= 800u)
        {
            pose->machine_values[AT_MACHINE_GRIP] = -1.8f;
        }
        break;
    default:
        break;
    }
}

/* GET_ENERGY_UNIT1 当前只保留旧工程里已经明确的 ACTION_ONE 语义。 */
void arm_task_apply_energy_1(ArmTaskMachinePose* pose, ClampAction action)
{
    arm_task_assign_pose(pose, &g_arm_pose_get_energy1);

    if (action == MODE_CLAMP_ACTION_ONE)
    {
        pose->machine_values[AT_MACHINE_GRIP] = 0.0f;
    }
}

/* GET_ENERGY_UNIT2 的第二段动作直接切到另一套存放姿态。 */
void arm_task_apply_energy_2(ArmTaskMachinePose* pose, ClampAction action, OsalTimeMs elapsed_ms)
{
    arm_task_assign_pose(pose, &g_arm_pose_get_energy2);

    switch (action)
    {
    case MODE_CLAMP_ACTION_ONE:
        pose->machine_values[AT_MACHINE_GRIP] = 0.0f;
        break;
    case MODE_CLAMP_ACTION_TWO:
        arm_task_assign_pose(pose, &g_arm_pose_store_energy1);
        if (elapsed_ms >= 1150u)
        {
            pose->machine_values[AT_MACHINE_GRIP] = -1.8f;
        }
        break;
    default:
        break;
    }
}

/* PRIMARY 模式里，primary_turn_ore_flag 只影响 roll3 的翻转目标。 */
void arm_task_apply_primary(ArmTaskMachinePose* pose, ClampAction action, uint8_t primary_turn_ore_flag)
{
    if (pose == OM_NULL)
    {
        return;
    }

    switch (action)
    {
    case MODE_CLAMP_UN_CMD:
        arm_task_assign_pose(pose, &g_arm_pose_zero);
        pose->machine_values[AT_MACHINE_PITCH1] = 0.6f;
        pose->machine_values[AT_MACHINE_PITCH2] = 1.0f;
        break;
    case MODE_CLAMP_ACTION_ONE:
        arm_task_assign_pose(pose, &g_arm_pose_primary);
        break;
    case MODE_CLAMP_ACTION_TWO:
        arm_task_assign_pose(pose, &g_arm_pose_primary);
        pose->machine_values[AT_MACHINE_ROLL3] =
            (primary_turn_ore_flag != 0u) ?
                -3.1415928f :
                0.0f;
        break;
    case MODE_CLAMP_ACTION_THREE:
        arm_task_assign_pose(pose, &g_arm_pose_primary);
        pose->machine_values[AT_MACHINE_GRIP] = -1.8f;
        pose->machine_values[AT_MACHINE_PITCH2] = 2.5f;
        break;
    default:
        arm_task_assign_pose(pose, &g_arm_pose_primary);
        break;
    }
}

/* SECONDARY_ORE 保留旧工程里最小可靠动作链。 */
void arm_task_apply_secondary_ore(ArmTaskMachinePose* pose, ClampAction action, OsalTimeMs elapsed_ms)
{
    if (pose == OM_NULL)
    {
        return;
    }

    switch (action)
    {
    case MODE_CLAMP_UN_CMD:
        arm_task_assign_pose(pose, &g_arm_pose_zero);
        pose->machine_values[AT_MACHINE_PITCH1] = 1.38691f;
        pose->machine_values[AT_MACHINE_PITCH2] = 1.0f;
        break;
    case MODE_CLAMP_ACTION_ONE:
        arm_task_assign_pose(pose, &g_arm_pose_zero);
        pose->machine_values[AT_MACHINE_PITCH1] = 1.38691f;
        pose->machine_values[AT_MACHINE_PITCH2] = 1.0f;
        if (elapsed_ms >= 50u)
        {
            arm_task_assign_pose(pose, &g_arm_pose_secondary_ore);
        }
        if (elapsed_ms >= 400u)
        {
            pose->machine_values[AT_MACHINE_PITCH3] = -1.6f;
        }
        break;
    default:
        arm_task_assign_pose(pose, &g_arm_pose_secondary_ore);
        break;
    }
}

/* EXCHANGE 模式只关心兑换子动作，不消费 clamp_action。 */
void arm_task_apply_exchange(ArmTaskMachinePose* pose, ExchangeAction action, OsalTimeMs elapsed_ms)
{
    if (pose == OM_NULL)
    {
        return;
    }

    switch (action)
    {
    case MODE_EXCHANGE_PICK_ACTION1:
        arm_task_exchange_pick_one(pose, elapsed_ms);
        break;
    case MODE_EXCHANGE_PICK_ACTION2:
        arm_task_exchange_pick_two(pose, elapsed_ms);
        break;
    case MODE_EXCHANGE_UN_CMD:
    default:
        arm_task_assign_pose(pose, &g_arm_pose_exchange);
        break;
    }
}

void clamp_angle_handle(
    const ArmTaskSnapshot* snapshot,
    OsalTimeMs elapsed_ms,
    ArmTaskMachinePose* pose)
{
    if (snapshot == OM_NULL || pose == OM_NULL)
    {
        return;
    }

    switch (snapshot->chassis_mode)
    {
    case MODE_CHASSIS_EXCHANGE:
        arm_task_apply_exchange(pose, snapshot->exchange_action, elapsed_ms);
        break;
    case MODE_CHASSIS_GET_ENERGY_UNIT:
        arm_task_apply_energy(pose, snapshot->clamp_action, elapsed_ms);
        break;
    case MODE_CHASSIS_GET_ENERGY_UNIT1:
        arm_task_apply_energy_1(pose, snapshot->clamp_action);
        break;
    case MODE_CHASSIS_GET_ENERGY_UNIT2:
        arm_task_apply_energy_2(pose, snapshot->clamp_action, elapsed_ms);
        break;
    case MODE_CHASSIS_PRIMARY:
        arm_task_apply_primary(
            pose,
            snapshot->clamp_action,
            snapshot->primary_turn_ore_flag);
        break;
    case MODE_CHASSIS_SECONDARY_ORE:
        arm_task_apply_secondary_ore(pose, snapshot->clamp_action, elapsed_ms);
        break;
    case MODE_CHASSIS_NORMAL:
    default:
        arm_task_assign_pose(pose, &g_arm_pose_zero);
        break;
    }
}

/* 统一最终机构角 -> 电机目标角。
 * 所有轴都通过各自零点和映射关系还原到电机控制角。
 */
void arm_task_resolve_targets(
    const ArmTaskContext* context,
    const ArmTaskMachinePose* pose,
    ArmTaskMotorTargets* targets)
{
    if (context == OM_NULL || pose == OM_NULL || targets == OM_NULL)
    {
        return;
    }

    (void)arm_task_joint_to_target(
        context,
        AT_MACHINE_BIG_YAW,
        pose->machine_values[AT_MACHINE_BIG_YAW],
        &targets->big_yaw_rad);
    (void)arm_task_joint_to_target(
        context,
        AT_MACHINE_PITCH1,
        pose->machine_values[AT_MACHINE_PITCH1],
        &targets->pitch1_rad);
    (void)arm_task_joint_to_target(
        context,
        AT_MACHINE_PITCH2,
        pose->machine_values[AT_MACHINE_PITCH2],
        &targets->pitch2_rad);
    (void)arm_task_joint_to_target(
        context,
        AT_MACHINE_ROLL2,
        pose->machine_values[AT_MACHINE_ROLL2],
        &targets->roll2_rad);
    (void)arm_task_joint_to_target(
        context,
        AT_MACHINE_PITCH3,
        pose->machine_values[AT_MACHINE_PITCH3],
        &targets->pitch3_rad);
    (void)arm_task_joint_to_target(
        context,
        AT_MACHINE_ROLL3,
        pose->machine_values[AT_MACHINE_ROLL3],
        &targets->roll3_rad);
    (void)arm_task_joint_to_target(
        context,
        AT_MACHINE_GRIP,
        pose->machine_values[AT_MACHINE_GRIP],
        &targets->grip_rad);
}

/* 重力补偿统一按当前反馈角现算。
 * 姿态表只负责几何目标，补偿力矩则由当前实际构型决定。
 */
void arm_task_gravity_feedforward(
    ArmTaskContext* context,
    float* pitch1_torque_ff,
    float* pitch2_torque_ff,
    float* roll2_torque_ff,
    float* pitch3_torque_ff)
{
    const MotorFeedback* pitch1_feedback = OM_NULL;
    const MotorFeedback* pitch2_feedback = OM_NULL;
    const MotorFeedback* roll2_feedback = OM_NULL;
    const MotorFeedback* pitch3_feedback = OM_NULL;
    float pitch1_angle_rad = 0.0f;
    float pitch2_zero_angle_rad = 0.0f;
    float pitch2_angle_rad = 0.0f;
    float roll2_angle_rad = 0.0f;
    float pitch3_angle_rad = 0.0f;
    GravityTorqueSnap gravity_snapshot = {0};

    if (context == OM_NULL || pitch1_torque_ff == OM_NULL || pitch2_torque_ff == OM_NULL ||
        roll2_torque_ff == OM_NULL || pitch3_torque_ff == OM_NULL)
    {
        return;
    }

    pitch1_feedback = motor_get_feedback(arm_task_get_motor(AT_MACHINE_PITCH1));
    pitch2_feedback = motor_get_feedback(arm_task_get_motor(AT_MACHINE_PITCH2));
    roll2_feedback = motor_get_feedback(arm_task_get_motor(AT_MACHINE_ROLL2));
    pitch3_feedback = motor_get_feedback(arm_task_get_motor(AT_MACHINE_PITCH3));

    /* Pitch1 在旧工程里通过 `config_full_mapping_one(..., -1)` 做了方向映射。
     * 当前新链里电机目标已经按 APP_AT_PITCH1_TARGET_RATIO 取反，但反馈角默认仍是
     * 电机原始坐标。重力补偿需要的是机构语义下的 Pitch1 角，因此这里按同一比例
     * 把反馈角转换回旧工程的机械臂符号约定，避免前馈方向与实际重力方向相反。
     */
    pitch1_angle_rad = (pitch1_feedback != OM_NULL) ? (pitch1_feedback->angle * APP_AT_PITCH1_TARGET_RATIO) : 0.0f;
    if (arm_task_pitch2_zero_rad(context, &pitch2_zero_angle_rad) != OM_TRUE && pitch2_feedback != OM_NULL)
    {
        pitch2_zero_angle_rad = pitch2_feedback->angle;
    }

    pitch2_angle_rad = (pitch2_feedback != OM_NULL) ? pitch2_feedback->angle : pitch2_zero_angle_rad;
    roll2_angle_rad = (roll2_feedback != OM_NULL) ? roll2_feedback->angle : 0.0f;
    pitch3_angle_rad = (pitch3_feedback != OM_NULL) ? pitch3_feedback->angle : 0.0f;

    (void)gravity_torque_snapshot(
        pitch1_angle_rad,
        pitch2_angle_rad,
        pitch2_zero_angle_rad,
        pitch3_angle_rad,
        roll2_angle_rad,
        &gravity_snapshot);
    *pitch2_torque_ff = gravity_snapshot.pitch2_torque_nm;
    *pitch2_torque_ff = math_utils_clamp_float(
        *pitch2_torque_ff,
        APP_AT_PITCH2_GRAVITY_FF_MIN,
        APP_AT_PITCH2_GRAVITY_FF_MAX);

    /* 旧工程 main 的 Pitch1 实际发送链走 Position 帧，
     * Motor_mit_tff_caculation() 里对应的 pitch1 tff 也处于注释关闭状态。
     * 当前正式链保留这一路为 0，不改变现有控制语义。
     */
    *pitch1_torque_ff = 0.0f;

    *pitch3_torque_ff = gravity_snapshot.pitch3_torque_nm;
#if (APP_AT_PITCH3_ENABLE_GRAVITY_FF == 0u)
    *pitch3_torque_ff = 0.0f;
#endif
    *roll2_torque_ff = gravity_snapshot.roll2_torque_nm;
}

/**
 * @brief 更新机械臂平滑目标值，对各个关节轴的目标角度进行速率限制平滑处理
 * 
 * 该函数使用斜坡滤波(slew rate limiting)算法，将期望的目标角度按照各关节的最大角速度限制
 * 进行平滑过渡，避免电机控制指令突变。同时会先从反馈刷新平滑目标值的基准状态。
 * 
 * 对于roll3轴，会先解析最近等效角度以处理角度环绕问题，确保平滑过渡的正确性。
 * 
 * @param context 机械臂任务上下文指针，包含当前平滑目标状态和各轴配置信息
 * @param desired_targets 期望的电机目标值指针，包含所有关节轴的目标角度(rad)
 * @param current_tick_s 当前时间步长(秒)，用于计算允许的最大角度变化量
 * 
 * @note 如果context或desired_targets为空指针，函数直接返回不执行任何操作
 * @note 各轴的最大角速度限制由APP_AT_*_MAX_RATE_RAD_PER_S宏定义指定
 */
void arm_task_update_smoothed(
    ArmTaskContext* context,
    const ArmTaskMotorTargets* desired_targets,
    float current_tick_s)
{
    /* 参数有效性检查 */
    if (context == OM_NULL || desired_targets == OM_NULL)
    {
        return;
    }

    /* 从电机反馈刷新平滑目标值的基准状态 */
    arm_task_sync_smooth_targets(context);

    /* 对各关节轴应用斜坡滤波，限制角速度在允许范围内 */
    context->smoothed_targets.big_yaw_rad =
        math_utils_slew_value(
            context->smoothed_targets.big_yaw_rad,
            desired_targets->big_yaw_rad,
            APP_AT_BIG_YAW_MAX_RATE_RAD_PER_S,
            current_tick_s);
    context->smoothed_targets.pitch1_rad =
        math_utils_slew_value(
            context->smoothed_targets.pitch1_rad,
            desired_targets->pitch1_rad,
            APP_AT_PITCH1_MAX_RATE_RAD_PER_S,
            current_tick_s);
    context->smoothed_targets.pitch2_rad =
        math_utils_slew_value(
            context->smoothed_targets.pitch2_rad,
            desired_targets->pitch2_rad,
            APP_AT_PITCH2_MAX_RATE_RAD_PER_S,
            current_tick_s);
    context->smoothed_targets.roll2_rad =
        math_utils_slew_value(
            context->smoothed_targets.roll2_rad,
            desired_targets->roll2_rad,
            APP_AT_ROLL2_MAX_RATE_RAD_PER_S,
            current_tick_s);
    context->smoothed_targets.pitch3_rad =
        math_utils_slew_value(
            context->smoothed_targets.pitch3_rad,
            math_utils_resolve_rad(
                desired_targets->pitch3_rad,
                context->smoothed_targets.pitch3_rad),
            APP_AT_PITCH3_MAX_RATE_RAD_PER_S,
            current_tick_s);

    /* roll3轴需要特殊处理：先解析最近等效角度以处理角度环绕问题 */
    context->smoothed_targets.roll3_rad =
        math_utils_slew_value(
            context->smoothed_targets.roll3_rad,
            math_utils_resolve_rad(
                desired_targets->roll3_rad,
                context->smoothed_targets.roll3_rad),
            APP_AT_ROLL3_MAX_RATE_RAD_PER_S,
            current_tick_s);
    context->smoothed_targets.grip_rad =
        math_utils_slew_value(
            context->smoothed_targets.grip_rad,
            desired_targets->grip_rad,
            APP_AT_GRIP_MAX_RATE_RAD_PER_S,
            current_tick_s);
}

/* 统一的 angle-mode 下发 helper。 */
