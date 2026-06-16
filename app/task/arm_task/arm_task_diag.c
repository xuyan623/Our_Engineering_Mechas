/* arm_task 观测接口实现。
 * 从 arm_task.c 抽离，通过 arm_task_internal.h 访问运行时上下文与内部 helper。
 */

#include "task/arm_task/arm_task_diag.h"
#include "task/arm_task/arm_task_internal.h"
#include "function/math_utils/math_utils.h"
#include "task/motor_communications_task/mct.h"

uint8_t arm_task_custom_align_done(void)
{
    return g_arm_task_custom_align_done_dbg;
}

uint8_t arm_task_get_custom_online(void)
{
    if (g_arm_task_owner_context == OM_NULL)
    {
        return 0u;
    }

    return g_arm_task_owner_context->latest_custom_snapshot.online;
}

uint8_t arm_task_get_custom_takeover(void)
{
    if (g_arm_task_owner_context == OM_NULL)
    {
        return 0u;
    }

    return (g_arm_task_owner_context->latest_mode_snapshot.arm_mode ==
                AT_MODE_CUSTOM_TAKEOVER &&
            g_arm_task_owner_context->latest_custom_snapshot.online != 0u &&
            g_arm_task_owner_context->latest_custom_snapshot.work_mode ==
                AT_CUSTOM_WORK_ENCODER)
               ? 1u
               : 0u;
}


OmBool arm_task_pitch2_debug(
    float* pitch2_feedback_deg,
    float* pitch2_feedback_rpm,
    float* pitch2_feedback_current,
    float* pitch2_feedback_torque,
    float* pitch2_feedback_online,
    float* pitch2_target_deg)
{
    const MotorFeedback* pitch2_feedback = OM_NULL;
    OmBool export_targets = OM_FALSE;

    if (g_arm_task_owner_context == OM_NULL ||
        pitch2_feedback_deg == OM_NULL ||
        pitch2_feedback_rpm == OM_NULL ||
        pitch2_feedback_current == OM_NULL ||
        pitch2_feedback_torque == OM_NULL ||
        pitch2_feedback_online == OM_NULL ||
        pitch2_target_deg == OM_NULL)
    {
        return OM_FALSE;
    }

    export_targets =
        (mct_is_operational_active() == OM_TRUE &&
         g_arm_task_owner_context->latest_mode_snapshot.arm_mode != AT_MODE_RELEASE)
            ? OM_TRUE
            : OM_FALSE;

    pitch2_feedback = motor_get_feedback(arm_task_get_motor(AT_MACHINE_PITCH2));
    *pitch2_feedback_deg =
        (pitch2_feedback != OM_NULL) ? math_utils_rad_to_deg(pitch2_feedback->angle) : 0.0f;
    *pitch2_feedback_rpm =
        (pitch2_feedback != OM_NULL) ? math_utils_rad_per_s_to_rpm(pitch2_feedback->speed) : 0.0f;
    *pitch2_feedback_current =
        (pitch2_feedback != OM_NULL) ? pitch2_feedback->current : 0.0f;
    *pitch2_feedback_torque =
        (pitch2_feedback != OM_NULL) ? pitch2_feedback->torque : 0.0f;
    *pitch2_feedback_online =
        (motor_is_feedback_recent(
             arm_task_get_motor(AT_MACHINE_PITCH2),
             AT_GO8010_RECENT_TIMEOUT_MS) == OM_TRUE)
            ? 1.0f
            : 0.0f;
    *pitch2_target_deg =
        (export_targets == OM_TRUE)
            ? math_utils_rad_to_deg(g_arm_task_owner_context->smoothed_targets.pitch2_rad)
            : 0.0f;

    return OM_TRUE;
}

OmBool arm_task_copy_custom_feedback(
    float* axis0_feedback_deg,
    float* axis1_feedback_deg,
    float* axis2_feedback_deg)
{
    if (g_arm_task_owner_context == OM_NULL ||
        axis0_feedback_deg == OM_NULL ||
        axis1_feedback_deg == OM_NULL ||
        axis2_feedback_deg == OM_NULL)
    {
        return OM_FALSE;
    }

    *axis0_feedback_deg =
        g_arm_task_owner_context->latest_custom_snapshot.angle_deg[AT_CUSTOM_AXIS_Y];
    *axis1_feedback_deg =
        g_arm_task_owner_context->latest_custom_snapshot.angle_deg[AT_CUSTOM_AXIS_Z];
    *axis2_feedback_deg =
        g_arm_task_owner_context->latest_custom_snapshot.angle_deg[AT_CUSTOM_AXIS_X];
    return OM_TRUE;
}

OmBool arm_task_get_custom_pitch_fb(float* pitch_axis_feedback_deg)
{
    if (g_arm_task_owner_context == OM_NULL || pitch_axis_feedback_deg == OM_NULL)
    {
        return OM_FALSE;
    }

    *pitch_axis_feedback_deg =
        g_arm_task_owner_context->latest_custom_snapshot.angle_deg[AT_CUSTOM_AXIS_PITCH];
    return OM_TRUE;
}

OmBool arm_task_joint_snapshot(
    float machine_angle_rad[7])
{
    ArmIkJointVector joint_vector = {0};
    const MotorFeedback* grip_feedback = OM_NULL;

    if (g_arm_task_owner_context == OM_NULL || machine_angle_rad == OM_NULL)
    {
        return OM_FALSE;
    }

    (void)arm_task_get_ik_joint_vector(g_arm_task_owner_context, &joint_vector);

    machine_angle_rad[0] = joint_vector.joint_rad[AT_MACHINE_BIG_YAW];
    machine_angle_rad[1] = joint_vector.joint_rad[AT_MACHINE_PITCH1];
    machine_angle_rad[2] = joint_vector.joint_rad[AT_MACHINE_PITCH2];
    machine_angle_rad[3] = joint_vector.joint_rad[AT_MACHINE_ROLL2];
    machine_angle_rad[4] = joint_vector.joint_rad[AT_MACHINE_PITCH3];
    machine_angle_rad[5] = joint_vector.joint_rad[AT_MACHINE_ROLL3];

    grip_feedback = motor_get_feedback(arm_task_get_motor(AT_MACHINE_GRIP));
    machine_angle_rad[6] = (grip_feedback != OM_NULL) ?
        (grip_feedback->angle - APP_AT_JOINT_ZERO_GRIP_RAD) : 0.0f;

    return OM_TRUE;
}

OmBool arm_task_ik_snapshot(
    ArmIkJointVector* joint_vector)
{
    if (g_arm_task_owner_context == OM_NULL || joint_vector == OM_NULL)
    {
        return OM_FALSE;
    }

    return arm_task_get_ik_joint_vector(g_arm_task_owner_context, joint_vector);
}

OmBool arm_task_get_fk_pose_snapshot(
    ArmIkPose* pose)
{
    ArmIkJointVector joint_vector = {0};

    if (pose == OM_NULL)
    {
        return OM_FALSE;
    }

    if (arm_task_ik_snapshot(&joint_vector) != OM_TRUE)
    {
        return OM_FALSE;
    }

    return (arm_kinematics_forward(&joint_vector, pose) == OM_OK) ? OM_TRUE : OM_FALSE;
}

OmBool arm_task_get_ik_target_pose(
    ArmIkPose* pose)
{
    if (g_arm_task_owner_context == OM_NULL || pose == OM_NULL)
    {
        return OM_FALSE;
    }

    if (g_arm_task_owner_context->latest_mode_snapshot.arm_mode != AT_MODE_RC_IK ||
        !(g_arm_task_owner_context->flags & AT_FLAG_IK_TARGET_POSE_READY))
    {
        return OM_FALSE;
    }

    *pose = g_arm_task_owner_context->ik_target_pose;
    return OM_TRUE;
}

OmBool arm_task_get_ik_feat_snapshot(
    ArmIkPoseFeat* pose_feature_snapshot)
{
    ArmIkJointVector joint_vector = {0};

    if (pose_feature_snapshot == OM_NULL)
    {
        return OM_FALSE;
    }

    if (arm_task_ik_snapshot(&joint_vector) != OM_TRUE)
    {
        return OM_FALSE;
    }

    return (aik_classify_pose(
                &joint_vector,
                pose_feature_snapshot) == OM_OK)
               ? OM_TRUE
               : OM_FALSE;
}

OmBool arm_task_mode_snapshot(
    uint8_t* arm_mode)
{
    if (g_arm_task_owner_context == OM_NULL || arm_mode == OM_NULL)
    {
        return OM_FALSE;
    }

    *arm_mode = g_arm_task_owner_context->latest_mode_snapshot.arm_mode;
    return OM_TRUE;
}

OmBool arm_task_copy_motor_feedback(
    float arm_feedback_rad[7])
{
    const MotorFeedback* feedback = OM_NULL;

    if (g_arm_task_owner_context == OM_NULL || arm_feedback_rad == OM_NULL)
    {
        return OM_FALSE;
    }

    feedback = motor_get_feedback(arm_task_get_motor(AT_MACHINE_BIG_YAW));
    arm_feedback_rad[0] = (feedback != OM_NULL) ? feedback->angle : 0.0f;

    feedback = motor_get_feedback(arm_task_get_motor(AT_MACHINE_PITCH1));
    arm_feedback_rad[1] = (feedback != OM_NULL) ? feedback->angle : 0.0f;

    feedback = motor_get_feedback(arm_task_get_motor(AT_MACHINE_PITCH2));
    arm_feedback_rad[2] = (feedback != OM_NULL) ? feedback->angle : 0.0f;

    feedback = motor_get_feedback(arm_task_get_motor(AT_MACHINE_ROLL2));
    arm_feedback_rad[3] = (feedback != OM_NULL) ? feedback->angle : 0.0f;

    feedback = motor_get_feedback(arm_task_get_motor(AT_MACHINE_PITCH3));
    arm_feedback_rad[4] = (feedback != OM_NULL) ? feedback->angle : 0.0f;

    feedback = motor_get_feedback(arm_task_get_motor(AT_MACHINE_ROLL3));
    arm_feedback_rad[5] = (feedback != OM_NULL) ? feedback->angle : 0.0f;

    feedback = motor_get_feedback(arm_task_get_motor(AT_MACHINE_GRIP));
    arm_feedback_rad[6] = (feedback != OM_NULL) ? feedback->angle : 0.0f;

    return OM_TRUE;
}

OmBool arm_task_pitch1_debug(
    float* pitch1_feedback_deg,
    float* pitch1_feedback_rpm,
    float* pitch1_feedback_torque,
    float* pitch1_feedback_online,
    float* pitch1_target_deg)
{
    const MotorFeedback* pitch1_feedback = OM_NULL;
    OmBool export_targets = OM_FALSE;

    if (g_arm_task_owner_context == OM_NULL ||
        pitch1_feedback_deg == OM_NULL ||
        pitch1_feedback_rpm == OM_NULL ||
        pitch1_feedback_torque == OM_NULL ||
        pitch1_feedback_online == OM_NULL ||
        pitch1_target_deg == OM_NULL)
    {
        return OM_FALSE;
    }

    export_targets =
        (mct_is_operational_active() == OM_TRUE &&
         g_arm_task_owner_context->latest_mode_snapshot.arm_mode != AT_MODE_RELEASE)
            ? OM_TRUE
            : OM_FALSE;

    pitch1_feedback = motor_get_feedback(arm_task_get_motor(AT_MACHINE_PITCH1));
    *pitch1_feedback_deg =
        (pitch1_feedback != OM_NULL) ? math_utils_rad_to_deg(pitch1_feedback->angle) : 0.0f;
    *pitch1_feedback_rpm =
        (pitch1_feedback != OM_NULL) ? math_utils_rad_per_s_to_rpm(pitch1_feedback->speed) : 0.0f;
    *pitch1_feedback_torque =
        (pitch1_feedback != OM_NULL) ? pitch1_feedback->torque : 0.0f;
    *pitch1_feedback_online =
        (motor_is_feedback_recent(
             arm_task_get_motor(AT_MACHINE_PITCH1),
             AT_DAMIAO_RECENT_TIMEOUT_MS) == OM_TRUE)
            ? 1.0f
            : 0.0f;
    *pitch1_target_deg =
        (export_targets == OM_TRUE)
            ? math_utils_rad_to_deg(g_arm_task_owner_context->smoothed_targets.pitch1_rad)
            : 0.0f;

    return OM_TRUE;
}

OmBool arm_task_pitch3_debug(
    float* pitch3_feedback_deg,
    float* pitch3_feedback_rpm,
    float* pitch3_feedback_torque,
    float* pitch3_feedback_online,
    float* pitch3_target_deg)
{
    const MotorFeedback* pitch3_feedback = OM_NULL;
    OmBool export_targets = OM_FALSE;

    if (g_arm_task_owner_context == OM_NULL ||
        pitch3_feedback_deg == OM_NULL ||
        pitch3_feedback_rpm == OM_NULL ||
        pitch3_feedback_torque == OM_NULL ||
        pitch3_feedback_online == OM_NULL ||
        pitch3_target_deg == OM_NULL)
    {
        return OM_FALSE;
    }

    export_targets =
        (mct_is_operational_active() == OM_TRUE &&
         g_arm_task_owner_context->latest_mode_snapshot.arm_mode != AT_MODE_RELEASE)
            ? OM_TRUE
            : OM_FALSE;

    pitch3_feedback = motor_get_feedback(arm_task_get_motor(AT_MACHINE_PITCH3));
    *pitch3_feedback_deg =
        (pitch3_feedback != OM_NULL) ? math_utils_rad_to_deg(pitch3_feedback->angle) : 0.0f;
    *pitch3_feedback_rpm =
        (pitch3_feedback != OM_NULL) ? math_utils_rad_per_s_to_rpm(pitch3_feedback->speed) : 0.0f;
    *pitch3_feedback_torque =
        (pitch3_feedback != OM_NULL) ? pitch3_feedback->torque : 0.0f;
    *pitch3_feedback_online =
        (motor_is_feedback_recent(
             arm_task_get_motor(AT_MACHINE_PITCH3),
             AT_DAMIAO_RECENT_TIMEOUT_MS) == OM_TRUE)
            ? 1.0f
            : 0.0f;
    *pitch3_target_deg =
        (export_targets == OM_TRUE)
            ? math_utils_rad_to_deg(g_arm_task_owner_context->smoothed_targets.pitch3_rad)
            : 0.0f;

    return OM_TRUE;
}

OmBool arm_task_get_debug_snapshot(
    float* big_yaw_feedback_deg,
    float* big_yaw_target_deg,
    float* big_yaw_force_feedback,
    float* pitch1_feedback_deg,
    float* pitch1_target_deg,
    float* pitch1_force_feedback,
    float* pitch2_feedback_deg,
    float* pitch2_target_deg,
    float* pitch2_force_feedback,
    float* pitch3_feedback_deg,
    float* pitch3_target_deg,
    float* pitch3_force_feedback,
    float* grip_feedback_deg,
    float* grip_target_deg,
    float* grip_force_feedback)
{
    const MotorFeedback* big_yaw_feedback = OM_NULL;
    const MotorFeedback* pitch1_feedback = OM_NULL;
    const MotorFeedback* pitch2_feedback = OM_NULL;
    const MotorFeedback* pitch3_feedback = OM_NULL;
    const MotorFeedback* grip_feedback = OM_NULL;
    OmBool export_targets = OM_FALSE;

    if (g_arm_task_owner_context == OM_NULL ||
        big_yaw_feedback_deg == OM_NULL ||
        big_yaw_target_deg == OM_NULL ||
        big_yaw_force_feedback == OM_NULL ||
        pitch1_feedback_deg == OM_NULL ||
        pitch1_target_deg == OM_NULL ||
        pitch1_force_feedback == OM_NULL ||
        pitch2_feedback_deg == OM_NULL ||
        pitch2_target_deg == OM_NULL ||
        pitch2_force_feedback == OM_NULL ||
        pitch3_feedback_deg == OM_NULL ||
        pitch3_target_deg == OM_NULL ||
        pitch3_force_feedback == OM_NULL ||
        grip_feedback_deg == OM_NULL ||
        grip_target_deg == OM_NULL ||
        grip_force_feedback == OM_NULL)
    {
        return OM_FALSE;
    }

    export_targets =
        (mct_is_operational_active() == OM_TRUE &&
         g_arm_task_owner_context->latest_mode_snapshot.arm_mode != AT_MODE_RELEASE)
            ? OM_TRUE
            : OM_FALSE;

    /* big_yaw：反馈角、目标角、力矩反馈 */
    big_yaw_feedback = motor_get_feedback(arm_task_get_motor(AT_MACHINE_BIG_YAW));
    *big_yaw_feedback_deg =
        (big_yaw_feedback != OM_NULL) ? math_utils_rad_to_deg(big_yaw_feedback->angle) : 0.0f;
    *big_yaw_target_deg =
        (export_targets == OM_TRUE)
            ? math_utils_rad_to_deg(g_arm_task_owner_context->smoothed_targets.big_yaw_rad)
            : 0.0f;
    *big_yaw_force_feedback =
        (big_yaw_feedback != OM_NULL) ? big_yaw_feedback->torque : 0.0f;

    /* pitch1：反馈角、目标角、力矩反馈 */
    pitch1_feedback = motor_get_feedback(arm_task_get_motor(AT_MACHINE_PITCH1));
    *pitch1_feedback_deg =
        (pitch1_feedback != OM_NULL) ? math_utils_rad_to_deg(pitch1_feedback->angle) : 0.0f;
    *pitch1_target_deg =
        (export_targets == OM_TRUE)
            ? math_utils_rad_to_deg(g_arm_task_owner_context->smoothed_targets.pitch1_rad)
            : 0.0f;
    *pitch1_force_feedback =
        (pitch1_feedback != OM_NULL) ? pitch1_feedback->torque : 0.0f;

    /* pitch2：反馈角、目标角、力矩反馈 */
    pitch2_feedback = motor_get_feedback(arm_task_get_motor(AT_MACHINE_PITCH2));
    *pitch2_feedback_deg =
        (pitch2_feedback != OM_NULL) ? math_utils_rad_to_deg(pitch2_feedback->angle) : 0.0f;
    *pitch2_target_deg =
        (export_targets == OM_TRUE)
            ? math_utils_rad_to_deg(g_arm_task_owner_context->smoothed_targets.pitch2_rad)
            : 0.0f;
    *pitch2_force_feedback =
        (pitch2_feedback != OM_NULL) ? pitch2_feedback->torque : 0.0f;

    /* pitch3：反馈角、目标角、力矩反馈 */
    pitch3_feedback = motor_get_feedback(arm_task_get_motor(AT_MACHINE_PITCH3));
    *pitch3_feedback_deg =
        (pitch3_feedback != OM_NULL) ? math_utils_rad_to_deg(pitch3_feedback->angle) : 0.0f;
    *pitch3_target_deg =
        (export_targets == OM_TRUE)
            ? math_utils_rad_to_deg(g_arm_task_owner_context->smoothed_targets.pitch3_rad)
            : 0.0f;
    *pitch3_force_feedback =
        (pitch3_feedback != OM_NULL) ? pitch3_feedback->torque : 0.0f;

    /* grip：反馈角、目标角、力矩反馈 */
    grip_feedback = motor_get_feedback(arm_task_get_motor(AT_MACHINE_GRIP));
    *grip_feedback_deg =
        (grip_feedback != OM_NULL) ? math_utils_rad_to_deg(grip_feedback->angle) : 0.0f;
    *grip_target_deg =
        (export_targets == OM_TRUE)
            ? math_utils_rad_to_deg(g_arm_task_owner_context->smoothed_targets.grip_rad)
            : 0.0f;
    *grip_force_feedback =
        (grip_feedback != OM_NULL) ? grip_feedback->torque : 0.0f;
    return OM_TRUE;
}
/* -------------------------------------------------------------------------- */
/* VTable 诊断回调实现                                                        */
/* -------------------------------------------------------------------------- */

void arm_task_diag_online(void* ctx, uint8_t* out_online)
{
    ArmTaskContext* context = (ArmTaskContext*)ctx;
    uint8_t online = 0u;
    uint32_t i = 0u;

    if (context == OM_NULL || out_online == OM_NULL)
    {
        return;
    }

    for (i = 0u; i < AT_MACHINE_COUNT; i++)
    {
        if (motor_is_feedback_recent(arm_task_get_motor((ArmTaskMachineAxis)i), 100u) == OM_TRUE)
        {
            online |= (1u << i);
        }
    }

    *out_online = online;
}

void arm_task_diag_snapshot(void* ctx, float* out_buf, uint32_t cap, uint32_t* out_count)
{
    float machine_angle_rad[7] = {0.0f};

    (void)ctx;

    if (out_buf == OM_NULL || out_count == OM_NULL)
    {
        return;
    }

    *out_count = 0u;

    if (cap < 8u)
    {
        return;
    }

    (void)arm_task_joint_snapshot(machine_angle_rad);

    out_buf[0] = machine_angle_rad[0]; /* big_yaw   */
    out_buf[1] = machine_angle_rad[1]; /* pitch1    */
    out_buf[2] = machine_angle_rad[2]; /* pitch2    */
    out_buf[3] = machine_angle_rad[3]; /* roll2     */
    out_buf[4] = machine_angle_rad[4]; /* pitch3    */
    out_buf[5] = machine_angle_rad[5]; /* roll3     */
    out_buf[6] = machine_angle_rad[6]; /* grip      */
    out_buf[7] = (float)arm_task_get_custom_takeover();
    *out_count = 8u;
}
