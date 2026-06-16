#include "algorithm/arm_kinematics/arm_kinematics.h"

#include "arm_math.h"
#include "config/app_config.h"
#include "function/math_utils/math_utils.h"
#include <math.h>
#include <string.h>

#define ARM_IK_DIM_3 (3u)
#define ARM_IK_DIM_6 (6u)
#define ARM_IK_EPSILON_REACHABILITY (1e-6f)
#define ARM_IK_EPSILON_BRANCH (1e-6f)
#define ARM_IK_LARGE_VALUE (1e30f)

typedef struct
{
    float lower;
    float upper;
} ArmIkJointLimit;

typedef struct
{
    float xyz[3];
    float rpy[3];
} ArmIkUrdfJointOrigin;

typedef struct
{
    float shoulder_height_m;
    float shoulder_offset_m;
    float upper_arm_length_m;
    float forearm_length_m;
    float tool_length_m;
} ArmIkIdealModel;

static const ArmIkIdealModel g_arm_ik_model = {
    APP_AT_IK_SHOULDER_HEIGHT_M,
    APP_AT_IK_SHOULDER_OFFSET_M,
    APP_AT_IK_UPPER_ARM_LENGTH_M,
    APP_AT_IK_FOREARM_LENGTH_M,
    APP_AT_IK_TOOL_LENGTH_M,
};

typedef struct
{
    float origin_position[3];
    float axis_world[3];
} ArmIkJointFrameCache;

typedef struct
{
    float end_position[3];
    float end_rotation[3][3];
    ArmIkJointFrameCache joints[ARM_IK_JOINT_COUNT];
} ArmIkForwardCache;

static const float g_arm_ik_urdf_tool_offset_m = APP_AT_IK_URDF_TOOL_OFFSET_M;
static const float g_arm_ik_home_offset_rad[ARM_IK_JOINT_COUNT] = {
    APP_AT_IK_HOME_BIG_YAW_RAD,
    APP_AT_IK_HOME_PITCH1_RAD,
    APP_AT_IK_HOME_PITCH2_RAD,
    APP_AT_IK_HOME_ROLL2_RAD,
    APP_AT_IK_HOME_PITCH3_RAD,
    APP_AT_IK_HOME_ROLL3_RAD,
};

static const ArmIkJointLimit g_arm_ik_urdf_limits[ARM_IK_JOINT_COUNT] = {
    {APP_AT_IK_BIG_YAW_MIN_RAD, APP_AT_IK_BIG_YAW_MAX_RAD},
    {APP_AT_IK_PITCH1_MIN_RAD, APP_AT_IK_PITCH1_MAX_RAD},
    {APP_AT_IK_URDF_PITCH2_MIN_RAD, APP_AT_IK_URDF_PITCH2_MAX_RAD},
    {APP_AT_IK_ROLL2_MIN_RAD, APP_AT_IK_ROLL2_MAX_RAD},
    {APP_AT_IK_PITCH3_MIN_RAD, APP_AT_IK_PITCH3_MAX_RAD},
    {APP_AT_IK_URDF_ROLL3_MIN_RAD, APP_AT_IK_URDF_ROLL3_MAX_RAD},
};

static const ArmIkUrdfJointOrigin g_arm_ik_urdf_origins[ARM_IK_JOINT_COUNT] = {
    {{0.0f, 0.0f, 0.01f}, {0.0f, 0.0f, 0.0f}},
    {{0.0f, 0.01f, 0.08f}, {-1.5708f, 0.0f, 0.0f}},
    {{0.0f, -0.25873f, 0.019f}, {-3.1416f, 0.0f, 0.0f}},
    {{0.0f, 0.132f, -0.0405f}, {-1.5708f, 0.0f, 3.1416f}},
    {{0.0f, 0.03f, -0.13805f}, {-1.5708f, 0.0f, 3.1416f}},
    {{0.0f, 0.048f, 0.03f}, {-1.5708f, 0.0f, 0.0f}},
};

static float arm_ik_normalize_angle(float angle_rad)
{
    return math_utils_wrap_pi_f32(angle_rad);
}

static void arm_ik_rot_z(float angle_rad, float matrix[3][3])
{
    const float c = arm_cos_f32(angle_rad);
    const float s = arm_sin_f32(angle_rad);

    matrix[0][0] = c;
    matrix[0][1] = -s;
    matrix[0][2] = 0.0f;
    matrix[1][0] = s;
    matrix[1][1] = c;
    matrix[1][2] = 0.0f;
    matrix[2][0] = 0.0f;
    matrix[2][1] = 0.0f;
    matrix[2][2] = 1.0f;
}

static void arm_ik_rpy_to_matrix(
    float roll_rad,
    float pitch_rad,
    float yaw_rad,
    float matrix[3][3])
{
    const float cr = arm_cos_f32(roll_rad);
    const float sr = arm_sin_f32(roll_rad);
    const float cp = arm_cos_f32(pitch_rad);
    const float sp = arm_sin_f32(pitch_rad);
    const float cy = arm_cos_f32(yaw_rad);
    const float sy = arm_sin_f32(yaw_rad);

    matrix[0][0] = cy * cp;
    matrix[0][1] = cy * sp * sr - sy * cr;
    matrix[0][2] = cy * sp * cr + sy * sr;
    matrix[1][0] = sy * cp;
    matrix[1][1] = sy * sp * sr + cy * cr;
    matrix[1][2] = sy * sp * cr - cy * sr;
    matrix[2][0] = -sp;
    matrix[2][1] = cp * sr;
    matrix[2][2] = cp * cr;
}

static void arm_ik_matrix_to_rpy(
    const float matrix[3][3],
    float rpy_rad[3])
{
    const float pitch = asinf(math_utils_clamp_float(-matrix[2][0], -1.0f, 1.0f));
    const float pitch_cos = arm_cos_f32(pitch);

    rpy_rad[1] = pitch;
    if (fabsf(pitch_cos) > ARM_IK_EPSILON_BRANCH)
    {
        (void)arm_atan2_f32(matrix[2][1], matrix[2][2], &rpy_rad[0]);
        (void)arm_atan2_f32(matrix[1][0], matrix[0][0], &rpy_rad[2]);
    }
    else
    {
        (void)arm_atan2_f32(-matrix[1][2], matrix[1][1], &rpy_rad[0]);
        rpy_rad[2] = 0.0f;
    }
}

static void arm_ik_mat3_mul(
    const float lhs[3][3],
    const float rhs[3][3],
    float out[3][3])
{
    arm_matrix_instance_f32 lhs_matrix = {0};
    arm_matrix_instance_f32 rhs_matrix = {0};
    arm_matrix_instance_f32 out_matrix = {0};

    arm_mat_init_f32(&lhs_matrix, ARM_IK_DIM_3, ARM_IK_DIM_3, (float*)lhs);
    arm_mat_init_f32(&rhs_matrix, ARM_IK_DIM_3, ARM_IK_DIM_3, (float*)rhs);
    arm_mat_init_f32(&out_matrix, ARM_IK_DIM_3, ARM_IK_DIM_3, (float*)out);
    (void)arm_mat_mult_f32(&lhs_matrix, &rhs_matrix, &out_matrix);
}

static void arm_ik_mat3_transpose(
    const float matrix[3][3],
    float out[3][3])
{
    arm_matrix_instance_f32 matrix_instance = {0};
    arm_matrix_instance_f32 out_instance = {0};

    arm_mat_init_f32(&matrix_instance, ARM_IK_DIM_3, ARM_IK_DIM_3, (float*)matrix);
    arm_mat_init_f32(&out_instance, ARM_IK_DIM_3, ARM_IK_DIM_3, (float*)out);
    (void)arm_mat_trans_f32(&matrix_instance, &out_instance);
}

static void arm_ik_mat3_vec3_mul(
    const float matrix[3][3],
    const float vector[3],
    float out[3])
{
    out[0] = matrix[0][0] * vector[0] + matrix[0][1] * vector[1] + matrix[0][2] * vector[2];
    out[1] = matrix[1][0] * vector[0] + matrix[1][1] * vector[1] + matrix[1][2] * vector[2];
    out[2] = matrix[2][0] * vector[0] + matrix[2][1] * vector[1] + matrix[2][2] * vector[2];
}

static void arm_ik_vec3_cross(
    const float lhs[3],
    const float rhs[3],
    float out[3])
{
    out[0] = lhs[1] * rhs[2] - lhs[2] * rhs[1];
    out[1] = lhs[2] * rhs[0] - lhs[0] * rhs[2];
    out[2] = lhs[0] * rhs[1] - lhs[1] * rhs[0];
}

static void arm_ik_vec3_sub(
    const float lhs[3],
    const float rhs[3],
    float out[3])
{
    out[0] = lhs[0] - rhs[0];
    out[1] = lhs[1] - rhs[1];
    out[2] = lhs[2] - rhs[2];
}

static float arm_ik_vec3_norm(const float vector[3])
{
    float norm = 0.0f;

    (void)arm_sqrt_f32(
        vector[0] * vector[0] +
            vector[1] * vector[1] +
            vector[2] * vector[2],
        &norm);
    return norm;
}

static float arm_ik_vecn_norm(const float* vector, uint32_t size)
{
    uint32_t index = 0u;
    float sum = 0.0f;

    for (index = 0u; index < size; index++)
    {
        sum += vector[index] * vector[index];
    }

    {
        float norm = 0.0f;

        (void)arm_sqrt_f32(sum, &norm);
        return norm;
    }
}

static void arm_ik_copy_vec(const float* source, float* destination, uint32_t size)
{
    uint32_t index = 0u;

    for (index = 0u; index < size; index++)
    {
        destination[index] = source[index];
    }
}

static void aik_fill_pose_err(
    const float position_error[3],
    const float orientation_error[3],
    ArmIkPoseErr* snapshot)
{
    if (snapshot == OM_NULL)
    {
        return;
    }

    snapshot->position_error_m[0] = position_error[0];
    snapshot->position_error_m[1] = position_error[1];
    snapshot->position_error_m[2] = position_error[2];
    snapshot->orientation_error_rad[0] = orientation_error[0];
    snapshot->orientation_error_rad[1] = orientation_error[1];
    snapshot->orientation_error_rad[2] = orientation_error[2];
    snapshot->position_error_norm_m = arm_ik_vec3_norm(position_error);
    snapshot->orientation_error_norm_rad = arm_ik_vec3_norm(orientation_error);
}

static void arm_ik_fill_debug_snapshot(
    ArmIkSolverKind solver_kind,
    ArmIkFailureReason failure_reason,
    uint16_t iteration_count,
    uint16_t candidate_count,
    uint16_t valid_candidate_count,
    ArmIkSolveDiag* snapshot)
{
    if (snapshot == OM_NULL)
    {
        return;
    }

    snapshot->solver_kind = (uint8_t)solver_kind;
    snapshot->failure_reason = (uint8_t)failure_reason;
    snapshot->iteration_count = iteration_count;
    snapshot->candidate_count = candidate_count;
    snapshot->valid_candidate_count = valid_candidate_count;
}

static float arm_ik_fit_angle_to_limits(
    float angle_rad,
    const ArmIkJointLimit* limit,
    float reference_rad,
    OmBool* ok)
{
    int32_t turn = 0;
    float best_candidate = 0.0f;
    float best_distance = ARM_IK_LARGE_VALUE;

    if (ok == OM_NULL || limit == OM_NULL)
    {
        return 0.0f;
    }

    *ok = OM_FALSE;
    for (turn = -2; turn <= 2; turn++)
    {
        const float candidate = angle_rad + (float)turn * 2.0f * APP_PI;

        if (candidate < limit->lower - ARM_IK_EPSILON_BRANCH ||
            candidate > limit->upper + ARM_IK_EPSILON_BRANCH)
        {
            continue;
        }

        {
            const float clamped = math_utils_clamp_float(
                candidate,
                limit->lower,
                limit->upper);
            const float distance = fabsf(clamped - reference_rad);

            if (distance < best_distance)
            {
                best_distance = distance;
                best_candidate = clamped;
                *ok = OM_TRUE;
            }
        }
    }

    return best_candidate;
}

static void arm_ik_machine_to_urdf(
    const ArmIkJointVector* machine,
    float urdf_joint_rad[ARM_IK_JOINT_COUNT])
{
    urdf_joint_rad[0] = machine->joint_rad[0] + g_arm_ik_home_offset_rad[0];
    urdf_joint_rad[1] = machine->joint_rad[1] + g_arm_ik_home_offset_rad[1];
    urdf_joint_rad[2] = -machine->joint_rad[2] + g_arm_ik_home_offset_rad[2];
    urdf_joint_rad[3] = machine->joint_rad[3] + g_arm_ik_home_offset_rad[3];
    urdf_joint_rad[4] = machine->joint_rad[4] + g_arm_ik_home_offset_rad[4];
    urdf_joint_rad[5] = arm_ik_normalize_angle(
        machine->joint_rad[5] + g_arm_ik_home_offset_rad[5]);
}

static void arm_ik_urdf_to_machine(
    const float urdf_joint_rad[ARM_IK_JOINT_COUNT],
    const ArmIkJointVector* reference_machine,
    ArmIkJointVector* machine)
{
    machine->joint_rad[0] = urdf_joint_rad[0] - g_arm_ik_home_offset_rad[0];
    machine->joint_rad[1] = urdf_joint_rad[1] - g_arm_ik_home_offset_rad[1];
    machine->joint_rad[2] = -(urdf_joint_rad[2] - g_arm_ik_home_offset_rad[2]);
    machine->joint_rad[3] = urdf_joint_rad[3] - g_arm_ik_home_offset_rad[3];
    machine->joint_rad[4] = urdf_joint_rad[4] - g_arm_ik_home_offset_rad[4];
    machine->joint_rad[5] = math_utils_resolve_rad(
        urdf_joint_rad[5] - g_arm_ik_home_offset_rad[5],
        reference_machine->joint_rad[5]);
}

static void arm_ik_build_fk_cache(
    const float urdf_joint_rad[ARM_IK_JOINT_COUNT],
    ArmIkForwardCache* cache)
{
    uint32_t index = 0u;
    float current_rotation[3][3] = {
        {1.0f, 0.0f, 0.0f},
        {0.0f, 1.0f, 0.0f},
        {0.0f, 0.0f, 1.0f},
    };
    float current_position[3] = {0.0f, 0.0f, 0.0f};

    if (cache == OM_NULL)
    {
        return;
    }

    memset(cache, 0, sizeof(*cache));
    for (index = 0u; index < ARM_IK_JOINT_COUNT; index++)
    {
        float fixed_rotation[3][3] = {{0.0f}};
        float origin_rotation[3][3] = {{0.0f}};
        float joint_rotation[3][3] = {{0.0f}};
        float rotated_translation[3] = {0.0f};
        const float local_z_axis[3] = {0.0f, 0.0f, 1.0f};
        float next_axis[3] = {0.0f};

        arm_ik_rpy_to_matrix(
            g_arm_ik_urdf_origins[index].rpy[0],
            g_arm_ik_urdf_origins[index].rpy[1],
            g_arm_ik_urdf_origins[index].rpy[2],
            fixed_rotation);

        arm_ik_mat3_vec3_mul(current_rotation, g_arm_ik_urdf_origins[index].xyz, rotated_translation);
        cache->joints[index].origin_position[0] = current_position[0] + rotated_translation[0];
        cache->joints[index].origin_position[1] = current_position[1] + rotated_translation[1];
        cache->joints[index].origin_position[2] = current_position[2] + rotated_translation[2];

        arm_ik_mat3_mul(current_rotation, fixed_rotation, origin_rotation);
        arm_ik_mat3_vec3_mul(origin_rotation, local_z_axis, next_axis);
        memcpy(cache->joints[index].axis_world, next_axis, sizeof(next_axis));

        arm_ik_rot_z(urdf_joint_rad[index], joint_rotation);
        arm_ik_mat3_mul(origin_rotation, joint_rotation, current_rotation);
        memcpy(current_position, cache->joints[index].origin_position, sizeof(current_position));
    }

    {
        const float tool_translation[3] = {
            0.0f,
            0.0f,
            g_arm_ik_urdf_tool_offset_m,
        };
        float rotated_tool_translation[3] = {0.0f};

        arm_ik_mat3_vec3_mul(current_rotation, tool_translation, rotated_tool_translation);
        cache->end_position[0] = current_position[0] + rotated_tool_translation[0];
        cache->end_position[1] = current_position[1] + rotated_tool_translation[1];
        cache->end_position[2] = current_position[2] + rotated_tool_translation[2];
        memcpy(cache->end_rotation, current_rotation, sizeof(cache->end_rotation));
    }
}

static void aik_pose_from_fk(
    const ArmIkForwardCache* cache,
    ArmIkPose* pose)
{
    if (cache == OM_NULL || pose == OM_NULL)
    {
        return;
    }

    memcpy(pose->position_m, cache->end_position, sizeof(cache->end_position));
    arm_ik_matrix_to_rpy(cache->end_rotation, pose->orientation_rpy_rad);
}

static void arm_ik_rotation_error_vector(
    const float current_rotation[3][3],
    const float target_rotation[3][3],
    float error_vector[3])
{
    float current_transpose[3][3] = {{0.0f}};
    float delta[3][3] = {{0.0f}};
    float cos_theta = 0.0f;
    float theta = 0.0f;
    float sin_theta = 0.0f;

    arm_ik_mat3_transpose(current_rotation, current_transpose);
    arm_ik_mat3_mul(target_rotation, current_transpose, delta);

    cos_theta = math_utils_clamp_float(
        0.5f * (delta[0][0] + delta[1][1] + delta[2][2] - 1.0f),
        -1.0f,
        1.0f);
    theta = acosf(cos_theta);
    if (theta < ARM_IK_EPSILON_BRANCH)
    {
        error_vector[0] = 0.0f;
        error_vector[1] = 0.0f;
        error_vector[2] = 0.0f;
        return;
    }

    if (fabsf(APP_PI - theta) < 1e-6f)
    {
        float axis[3] = {0.0f};
        float diagonal[3] = {
            fmaxf((delta[0][0] + 1.0f) * 0.5f, 0.0f),
            fmaxf((delta[1][1] + 1.0f) * 0.5f, 0.0f),
            fmaxf((delta[2][2] + 1.0f) * 0.5f, 0.0f),
        };
        float norm = 0.0f;

        (void)arm_sqrt_f32(diagonal[0], &axis[0]);
        (void)arm_sqrt_f32(diagonal[1], &axis[1]);
        (void)arm_sqrt_f32(diagonal[2], &axis[2]);

        if (axis[0] > ARM_IK_EPSILON_BRANCH)
        {
            axis[1] = copysignf(axis[1], delta[0][1] + delta[1][0]);
            axis[2] = copysignf(axis[2], delta[0][2] + delta[2][0]);
        }
        else if (axis[1] > ARM_IK_EPSILON_BRANCH)
        {
            axis[0] = copysignf(axis[0], delta[0][1] + delta[1][0]);
            axis[2] = copysignf(axis[2], delta[1][2] + delta[2][1]);
        }
        else
        {
            axis[0] = copysignf(axis[0], delta[0][2] + delta[2][0]);
            axis[1] = copysignf(axis[1], delta[1][2] + delta[2][1]);
        }

        norm = arm_ik_vec3_norm(axis);
        if (norm <= ARM_IK_EPSILON_BRANCH)
        {
            error_vector[0] = 0.0f;
            error_vector[1] = 0.0f;
            error_vector[2] = 0.0f;
            return;
        }

        error_vector[0] = axis[0] / norm * theta;
        error_vector[1] = axis[1] / norm * theta;
        error_vector[2] = axis[2] / norm * theta;
        return;
    }

    sin_theta = arm_sin_f32(theta);
    error_vector[0] = (delta[2][1] - delta[1][2]) / (2.0f * sin_theta) * theta;
    error_vector[1] = (delta[0][2] - delta[2][0]) / (2.0f * sin_theta) * theta;
    error_vector[2] = (delta[1][0] - delta[0][1]) / (2.0f * sin_theta) * theta;
}

static void aik_build_jac(
    const ArmIkForwardCache* cache,
    float jacobian[ARM_IK_DIM_6][ARM_IK_DIM_6])
{
    uint32_t index = 0u;

    if (cache == OM_NULL || jacobian == OM_NULL)
    {
        return;
    }

    memset(jacobian, 0, sizeof(float) * ARM_IK_DIM_6 * ARM_IK_DIM_6);
    for (index = 0u; index < ARM_IK_DIM_6; index++)
    {
        float position_delta[3] = {0.0f};
        float linear_component[3] = {0.0f};

        arm_ik_vec3_sub(cache->end_position, cache->joints[index].origin_position, position_delta);
        arm_ik_vec3_cross(cache->joints[index].axis_world, position_delta, linear_component);

        jacobian[0][index] = linear_component[0];
        jacobian[1][index] = linear_component[1];
        jacobian[2][index] = linear_component[2];
        jacobian[3][index] = cache->joints[index].axis_world[0];
        jacobian[4][index] = cache->joints[index].axis_world[1];
        jacobian[5][index] = cache->joints[index].axis_world[2];
    }
}

static OmBool aik_solve_spd(
    const float* matrix_in,
    const float* rhs_in,
    uint32_t dimension,
    float* solution_out)
{
    float lower_storage[ARM_IK_DIM_6 * ARM_IK_DIM_6] = {0.0f};
    float lower_transpose_storage[ARM_IK_DIM_6 * ARM_IK_DIM_6] = {0.0f};
    float rhs_storage[ARM_IK_DIM_6] = {0.0f};
    float forward_solution_storage[ARM_IK_DIM_6] = {0.0f};
    arm_matrix_instance_f32 matrix = {0};
    arm_matrix_instance_f32 lower = {0};
    arm_matrix_instance_f32 lower_transpose = {0};
    arm_matrix_instance_f32 rhs = {0};
    arm_matrix_instance_f32 forward_solution = {0};
    arm_matrix_instance_f32 solution = {0};

    if (matrix_in == OM_NULL || rhs_in == OM_NULL || solution_out == OM_NULL ||
        dimension == 0u || dimension > ARM_IK_DIM_6)
    {
        return OM_FALSE;
    }

    memcpy(rhs_storage, rhs_in, sizeof(float) * dimension);
    arm_mat_init_f32(&matrix, (uint16_t)dimension, (uint16_t)dimension, (float*)matrix_in);
    arm_mat_init_f32(&lower, (uint16_t)dimension, (uint16_t)dimension, lower_storage);
    arm_mat_init_f32(&lower_transpose, (uint16_t)dimension, (uint16_t)dimension, lower_transpose_storage);
    arm_mat_init_f32(&rhs, (uint16_t)dimension, 1u, rhs_storage);
    arm_mat_init_f32(&forward_solution, (uint16_t)dimension, 1u, forward_solution_storage);
    arm_mat_init_f32(&solution, (uint16_t)dimension, 1u, solution_out);

    if (arm_mat_cholesky_f32(&matrix, &lower) != ARM_MATH_SUCCESS)
    {
        return OM_FALSE;
    }

    if (arm_mat_solve_lower_triangular_f32(&lower, &rhs, &forward_solution) != ARM_MATH_SUCCESS)
    {
        return OM_FALSE;
    }

    if (arm_mat_trans_f32(&lower, &lower_transpose) != ARM_MATH_SUCCESS)
    {
        return OM_FALSE;
    }

    if (arm_mat_solve_upper_triangular_f32(&lower_transpose, &forward_solution, &solution) != ARM_MATH_SUCCESS)
    {
        return OM_FALSE;
    }

    return OM_TRUE;
}

static OmBool arm_ik_full_pose_reach(
    const ArmIkPose* target_pose)
{
    float target_rotation[3][3] = {{0.0f}};
    float wrist_center[3] = {0.0f};
    float radial_sq = 0.0f;
    float x_plane_mag = 0.0f;
    float z_plane = 0.0f;
    float reach_sq = 0.0f;
    float cos_theta3 = 0.0f;
    const float denominator =
        2.0f * g_arm_ik_model.upper_arm_length_m * g_arm_ik_model.forearm_length_m;

    arm_ik_rpy_to_matrix(
        target_pose->orientation_rpy_rad[0],
        target_pose->orientation_rpy_rad[1],
        target_pose->orientation_rpy_rad[2],
        target_rotation);

    wrist_center[0] = target_pose->position_m[0] - g_arm_ik_model.tool_length_m * target_rotation[0][2];
    wrist_center[1] = target_pose->position_m[1] - g_arm_ik_model.tool_length_m * target_rotation[1][2];
    wrist_center[2] = target_pose->position_m[2] - g_arm_ik_model.tool_length_m * target_rotation[2][2];

    radial_sq = wrist_center[0] * wrist_center[0] + wrist_center[1] * wrist_center[1];
    if (radial_sq < g_arm_ik_model.shoulder_offset_m * g_arm_ik_model.shoulder_offset_m -
                        ARM_IK_EPSILON_REACHABILITY)
    {
        return OM_FALSE;
    }

    (void)arm_sqrt_f32(
        fmaxf(
            0.0f,
            radial_sq - g_arm_ik_model.shoulder_offset_m * g_arm_ik_model.shoulder_offset_m),
        &x_plane_mag);
    z_plane = wrist_center[2] - g_arm_ik_model.shoulder_height_m;
    reach_sq = x_plane_mag * x_plane_mag + z_plane * z_plane;
    cos_theta3 = (
        reach_sq -
        g_arm_ik_model.upper_arm_length_m * g_arm_ik_model.upper_arm_length_m -
        g_arm_ik_model.forearm_length_m * g_arm_ik_model.forearm_length_m) /
        denominator;

    return (cos_theta3 >= -1.0f - ARM_IK_EPSILON_REACHABILITY &&
            cos_theta3 <= 1.0f + ARM_IK_EPSILON_REACHABILITY)
               ? OM_TRUE
               : OM_FALSE;
}

static OmBool arm_ik_pos_reach(
    const ArmIkPose* target_pose,
    const ArmIkJointVector* reference_joint_vector)
{
    float q1_candidates[3] = {0.0f};
    uint32_t candidate_count = 0u;
    float radial_sq = 0.0f;
    const float link_min = fabsf(
        g_arm_ik_model.forearm_length_m - g_arm_ik_model.tool_length_m);
    const float link_max =
        g_arm_ik_model.forearm_length_m + g_arm_ik_model.tool_length_m;
    const float reference_pitch3_cos = arm_cos_f32(reference_joint_vector->joint_rad[4]);
    float current_effective = 0.0f;

    (void)arm_sqrt_f32(
        g_arm_ik_model.forearm_length_m * g_arm_ik_model.forearm_length_m +
            g_arm_ik_model.tool_length_m * g_arm_ik_model.tool_length_m +
            2.0f * g_arm_ik_model.forearm_length_m * g_arm_ik_model.tool_length_m *
                reference_pitch3_cos,
        &current_effective);

    q1_candidates[candidate_count++] = reference_joint_vector->joint_rad[0];

    radial_sq =
        target_pose->position_m[0] * target_pose->position_m[0] +
        target_pose->position_m[1] * target_pose->position_m[1];
    if (radial_sq >=
        g_arm_ik_model.shoulder_offset_m * g_arm_ik_model.shoulder_offset_m -
            ARM_IK_EPSILON_REACHABILITY)
    {
        float x_plane_mag = 0.0f;
        float atan_base = 0.0f;
        float atan_front = 0.0f;
        float atan_back = 0.0f;

        (void)arm_sqrt_f32(
            fmaxf(
                0.0f,
                radial_sq - g_arm_ik_model.shoulder_offset_m *
                                g_arm_ik_model.shoulder_offset_m),
            &x_plane_mag);
        (void)arm_atan2_f32(target_pose->position_m[1], target_pose->position_m[0], &atan_base);
        (void)arm_atan2_f32(g_arm_ik_model.shoulder_offset_m, x_plane_mag, &atan_front);
        (void)arm_atan2_f32(g_arm_ik_model.shoulder_offset_m, -x_plane_mag, &atan_back);

        q1_candidates[candidate_count++] = arm_ik_normalize_angle(atan_base - atan_front);
        q1_candidates[candidate_count++] = arm_ik_normalize_angle(atan_base - atan_back);
    }

    {
        uint32_t index = 0u;
        for (index = 0u; index < candidate_count; index++)
        {
            const float theta1 = q1_candidates[index];
            float theta1_sin = 0.0f;
            float theta1_cos = 0.0f;
            float target_in_shoulder_x = 0.0f;
            float target_in_shoulder_y = 0.0f;
            const float target_in_shoulder_z =
                target_pose->position_m[2] - g_arm_ik_model.shoulder_height_m;
            float planar_distance = 0.0f;
            float effective_low_inner = 0.0f;
            float effective_low = 0.0f;
            float effective_high = 0.0f;

            theta1_sin = arm_sin_f32(theta1);
            theta1_cos = arm_cos_f32(theta1);
            target_in_shoulder_x =
                theta1_cos * target_pose->position_m[0] +
                theta1_sin * target_pose->position_m[1];
            target_in_shoulder_y =
                -theta1_sin * target_pose->position_m[0] +
                theta1_cos * target_pose->position_m[1] -
                g_arm_ik_model.shoulder_offset_m;
            (void)arm_sqrt_f32(
                target_in_shoulder_x * target_in_shoulder_x +
                    target_in_shoulder_z * target_in_shoulder_z,
                &planar_distance);
            (void)arm_sqrt_f32(
                fabsf(planar_distance - g_arm_ik_model.upper_arm_length_m) *
                    fabsf(planar_distance - g_arm_ik_model.upper_arm_length_m) +
                target_in_shoulder_y * target_in_shoulder_y,
                &effective_low_inner);
            effective_low = fmaxf(
                link_min,
                fmaxf(
                    fabsf(target_in_shoulder_y),
                    effective_low_inner));
            {
                float tmp = 0.0f;
                (void)arm_sqrt_f32(
                    (planar_distance + g_arm_ik_model.upper_arm_length_m) *
                        (planar_distance + g_arm_ik_model.upper_arm_length_m) +
                    target_in_shoulder_y * target_in_shoulder_y,
                    &tmp);
                effective_high = fminf(link_max, tmp);
            }

            if (current_effective >= effective_low - ARM_IK_EPSILON_REACHABILITY &&
                current_effective <= effective_high + ARM_IK_EPSILON_REACHABILITY)
            {
                return OM_TRUE;
            }

            if (effective_low <= effective_high + ARM_IK_EPSILON_REACHABILITY)
            {
                return OM_TRUE;
            }
        }
    }

    return OM_FALSE;
}

static void arm_ik_pose_error_inner(
    const ArmIkPose* target_pose,
    const ArmIkPose* current_pose,
    float position_error[3],
    float orientation_error[3])
{
    float current_rotation[3][3] = {{0.0f}};
    float target_rotation[3][3] = {{0.0f}};

    position_error[0] = target_pose->position_m[0] - current_pose->position_m[0];
    position_error[1] = target_pose->position_m[1] - current_pose->position_m[1];
    position_error[2] = target_pose->position_m[2] - current_pose->position_m[2];

    arm_ik_rpy_to_matrix(
        current_pose->orientation_rpy_rad[0],
        current_pose->orientation_rpy_rad[1],
        current_pose->orientation_rpy_rad[2],
        current_rotation);
    arm_ik_rpy_to_matrix(
        target_pose->orientation_rpy_rad[0],
        target_pose->orientation_rpy_rad[1],
        target_pose->orientation_rpy_rad[2],
        target_rotation);
    arm_ik_rotation_error_vector(current_rotation, target_rotation, orientation_error);
}

static void aik_forward_urdf(
    const float urdf_joint_rad[ARM_IK_JOINT_COUNT],
    ArmIkPose* pose)
{
    ArmIkForwardCache cache = {0};

    arm_ik_build_fk_cache(urdf_joint_rad, &cache);
    aik_pose_from_fk(&cache, pose);
}

static OmRet aik_inverse_full(
    const ArmIkPose* target_pose,
    const ArmIkJointVector* reference_joint_vector,
    ArmIkJointVector* solved_joint_vector,
    ArmIkPoseErr* pose_error_snapshot,
    ArmIkSolveDiag* solve_debug_snapshot)
{
    float solution_urdf[ARM_IK_JOINT_COUNT] = {0.0f};
    uint32_t iteration = 0u;

    arm_ik_machine_to_urdf(reference_joint_vector, solution_urdf);

    for (iteration = 0u; iteration < APP_AT_IK_FULL_POSE_MAX_ITERATIONS; iteration++)
    {
        ArmIkForwardCache current_cache = {0};
        ArmIkPose current_pose = {0};
        float position_error[3] = {0.0f};
        float orientation_error[3] = {0.0f};
        float error[ARM_IK_DIM_6] = {0.0f};
        float jacobian[ARM_IK_DIM_6][ARM_IK_DIM_6] = {{0.0f}};
        float jacobian_transpose[ARM_IK_DIM_6][ARM_IK_DIM_6] = {{0.0f}};
        float damping_matrix[ARM_IK_DIM_6][ARM_IK_DIM_6] = {{0.0f}};
        float damping_rhs[ARM_IK_DIM_6] = {0.0f};
        float damping_solution[ARM_IK_DIM_6] = {0.0f};
        float step[ARM_IK_DIM_6] = {0.0f};
        float position_error_norm = 0.0f;
        float orientation_error_norm = 0.0f;
        float step_norm = 0.0f;
        uint32_t row = 0u;
        uint32_t col = 0u;

        arm_ik_build_fk_cache(solution_urdf, &current_cache);
        aik_pose_from_fk(&current_cache, &current_pose);
        arm_ik_pose_error_inner(
            target_pose,
            &current_pose,
            position_error,
            orientation_error);
        position_error_norm = arm_ik_vec3_norm(position_error);
        orientation_error_norm = arm_ik_vec3_norm(orientation_error);

        error[0] = position_error[0];
        error[1] = position_error[1];
        error[2] = position_error[2];
        error[3] = orientation_error[0];
        error[4] = orientation_error[1];
        error[5] = orientation_error[2];

        if (position_error_norm <= APP_AT_IK_FULL_POS_TOL_M &&
            orientation_error_norm <= APP_AT_IK_FULL_ORI_TOL_RAD)
        {
            arm_ik_urdf_to_machine(solution_urdf, reference_joint_vector, solved_joint_vector);
            aik_fill_pose_err(
                position_error,
                orientation_error,
                pose_error_snapshot);
            arm_ik_fill_debug_snapshot(
                ARM_IK_SOLVER_FULL_POSE_LOCAL,
                ARM_IK_FAILURE_NONE,
                (uint16_t)iteration,
                1u,
                1u,
                solve_debug_snapshot);
            return OM_OK;
        }

        aik_build_jac(&current_cache, jacobian);

        for (row = 0u; row < ARM_IK_DIM_6; row++)
        {
            for (col = 0u; col < ARM_IK_DIM_6; col++)
            {
                jacobian_transpose[row][col] = jacobian[col][row];
            }
        }

        for (row = 0u; row < ARM_IK_DIM_6; row++)
        {
            for (col = 0u; col < ARM_IK_DIM_6; col++)
            {
                uint32_t k = 0u;
                float value = 0.0f;
                for (k = 0u; k < ARM_IK_DIM_6; k++)
                {
                    value += jacobian[row][k] * jacobian_transpose[k][col];
                }
                if (row == col)
                {
                    value += APP_AT_IK_FULL_POSE_DAMPING * APP_AT_IK_FULL_POSE_DAMPING;
                }
                damping_matrix[row][col] = value;
            }
            damping_rhs[row] = error[row];
        }

        if (aik_solve_spd(
                &damping_matrix[0][0],
                damping_rhs,
                ARM_IK_DIM_6,
                damping_solution) != OM_TRUE)
        {
            break;
        }

        for (row = 0u; row < ARM_IK_DIM_6; row++)
        {
            uint32_t k = 0u;
            float value = 0.0f;
            for (k = 0u; k < ARM_IK_DIM_6; k++)
            {
                value += jacobian_transpose[row][k] * damping_solution[k];
            }
            step[row] = value;
        }

        step_norm = arm_ik_vecn_norm(step, ARM_IK_DIM_6);
        if (step_norm > APP_AT_IK_FULL_POSE_STEP_LIMIT_RAD)
        {
            const float scale = APP_AT_IK_FULL_POSE_STEP_LIMIT_RAD / step_norm;
            for (row = 0u; row < ARM_IK_DIM_6; row++)
            {
                step[row] *= scale;
            }
        }

        for (row = 0u; row < ARM_IK_DIM_6; row++)
        {
            solution_urdf[row] += step[row];
            solution_urdf[row] = math_utils_clamp_float(
                solution_urdf[row],
                g_arm_ik_urdf_limits[row].lower,
                g_arm_ik_urdf_limits[row].upper);
        }
    }

    {
        ArmIkPose current_pose = {0};
        float position_error[3] = {0.0f};
        float orientation_error[3] = {0.0f};

        aik_forward_urdf(solution_urdf, &current_pose);
        arm_ik_pose_error_inner(
            target_pose,
            &current_pose,
            position_error,
            orientation_error);

        if (arm_ik_vec3_norm(position_error) <=
                APP_AT_IK_FULL_ACCEPT_POS_ERR_M &&
            arm_ik_vec3_norm(orientation_error) <=
                APP_AT_IK_FULL_ACCEPT_ORI_ERR_RAD)
        {
            arm_ik_urdf_to_machine(solution_urdf, reference_joint_vector, solved_joint_vector);
            aik_fill_pose_err(
                position_error,
                orientation_error,
                pose_error_snapshot);
            arm_ik_fill_debug_snapshot(
                ARM_IK_SOLVER_FULL_POSE_LOCAL,
                ARM_IK_FAILURE_NONE,
                APP_AT_IK_FULL_POSE_MAX_ITERATIONS,
                1u,
                1u,
                solve_debug_snapshot);
            return OM_OK;
        }

        aik_fill_pose_err(
            position_error,
            orientation_error,
            pose_error_snapshot);
        arm_ik_fill_debug_snapshot(
            ARM_IK_SOLVER_FULL_POSE_LOCAL,
            ARM_IK_FAILURE_LOCAL_SOLVER_FAILED,
            APP_AT_IK_FULL_POSE_MAX_ITERATIONS,
            0u,
            0u,
            solve_debug_snapshot);
    }

    return OM_ERROR;
}

static OmRet aik_inverse_pos(
    const ArmIkPose* target_pose,
    const ArmIkJointVector* reference_joint_vector,
    ArmIkJointVector* solved_joint_vector,
    ArmIkPoseErr* pose_error_snapshot,
    ArmIkSolveDiag* solve_debug_snapshot)
{
    float solution_urdf[ARM_IK_JOINT_COUNT] = {0.0f};
    float fixed_roll3_urdf = 0.0f;
    uint32_t iteration = 0u;

    arm_ik_machine_to_urdf(reference_joint_vector, solution_urdf);
    fixed_roll3_urdf = solution_urdf[5];

    for (iteration = 0u; iteration < APP_AT_IK_POS_PRI_MAX_ITERS; iteration++)
    {
        ArmIkForwardCache current_cache = {0};
        ArmIkPose current_pose = {0};
        float position_error[3] = {0.0f};
        float zero_orientation_error[3] = {0.0f, 0.0f, 0.0f};
        float full_jacobian[ARM_IK_DIM_6][ARM_IK_DIM_6] = {{0.0f}};
        float jacobian[ARM_IK_DIM_3][ARM_IK_DIM_6] = {{0.0f}};
        float jacobian_transpose[ARM_IK_DIM_6][ARM_IK_DIM_3] = {{0.0f}};
        float damping_matrix[ARM_IK_DIM_3][ARM_IK_DIM_3] = {{0.0f}};
        float damping_solution[ARM_IK_DIM_3] = {0.0f};
        float step[ARM_IK_DIM_6] = {0.0f};
        float position_error_norm = 0.0f;
        float step_norm = 0.0f;
        uint32_t row = 0u;
        uint32_t col = 0u;

        arm_ik_build_fk_cache(solution_urdf, &current_cache);
        aik_pose_from_fk(&current_cache, &current_pose);
        position_error[0] = target_pose->position_m[0] - current_pose.position_m[0];
        position_error[1] = target_pose->position_m[1] - current_pose.position_m[1];
        position_error[2] = target_pose->position_m[2] - current_pose.position_m[2];
        position_error_norm = arm_ik_vec3_norm(position_error);

        if (position_error_norm <= APP_AT_IK_POS_PRI_TOL_M)
        {
            arm_ik_urdf_to_machine(solution_urdf, reference_joint_vector, solved_joint_vector);
            aik_fill_pose_err(
                position_error,
                zero_orientation_error,
                pose_error_snapshot);
            arm_ik_fill_debug_snapshot(
                ARM_IK_SOLVER_POSITION_PRIORITY_LOCAL,
                ARM_IK_FAILURE_NONE,
                (uint16_t)iteration,
                1u,
                1u,
                solve_debug_snapshot);
            return OM_OK;
        }

        aik_build_jac(&current_cache, full_jacobian);
        for (row = 0u; row < ARM_IK_DIM_3; row++)
        {
            for (col = 0u; col < ARM_IK_DIM_6; col++)
            {
                jacobian[row][col] = full_jacobian[row][col];
            }
        }

        for (row = 0u; row < ARM_IK_DIM_6; row++)
        {
            for (col = 0u; col < ARM_IK_DIM_3; col++)
            {
                jacobian_transpose[row][col] = jacobian[col][row];
            }
        }

        for (row = 0u; row < ARM_IK_DIM_3; row++)
        {
            for (col = 0u; col < ARM_IK_DIM_3; col++)
            {
                uint32_t k = 0u;
                float value = 0.0f;
                for (k = 0u; k < ARM_IK_DIM_6; k++)
                {
                    value += jacobian[row][k] * jacobian_transpose[k][col];
                }
                if (row == col)
                {
                    value +=
                        APP_AT_IK_POS_PRI_DAMPING *
                        APP_AT_IK_POS_PRI_DAMPING;
                }
                damping_matrix[row][col] = value;
            }
        }

        if (aik_solve_spd(
                &damping_matrix[0][0],
                position_error,
                ARM_IK_DIM_3,
                damping_solution) != OM_TRUE)
        {
            break;
        }

        for (row = 0u; row < ARM_IK_DIM_6; row++)
        {
            uint32_t k = 0u;
            float value = 0.0f;
            for (k = 0u; k < ARM_IK_DIM_3; k++)
            {
                value += jacobian_transpose[row][k] * damping_solution[k];
            }
            step[row] = value;
        }

        step_norm = arm_ik_vecn_norm(step, ARM_IK_DIM_6);
        if (step_norm > APP_AT_IK_POS_PRI_STEP_MAX_RAD)
        {
            const float scale = APP_AT_IK_POS_PRI_STEP_MAX_RAD / step_norm;
            for (row = 0u; row < ARM_IK_DIM_6; row++)
            {
                step[row] *= scale;
            }
        }

        for (row = 0u; row < ARM_IK_DIM_6; row++)
        {
            solution_urdf[row] += step[row];
            solution_urdf[row] = math_utils_clamp_float(
                solution_urdf[row],
                g_arm_ik_urdf_limits[row].lower,
                g_arm_ik_urdf_limits[row].upper);
        }
        solution_urdf[5] = fixed_roll3_urdf;
    }

    {
        ArmIkPose current_pose = {0};
        float position_error[3] = {0.0f};
        float zero_orientation_error[3] = {0.0f, 0.0f, 0.0f};

        aik_forward_urdf(solution_urdf, &current_pose);
        position_error[0] = target_pose->position_m[0] - current_pose.position_m[0];
        position_error[1] = target_pose->position_m[1] - current_pose.position_m[1];
        position_error[2] = target_pose->position_m[2] - current_pose.position_m[2];

        if (arm_ik_vec3_norm(position_error) <=
            APP_AT_IK_POS_PRI_ACCEPT_POS_ERR_M)
        {
            arm_ik_urdf_to_machine(solution_urdf, reference_joint_vector, solved_joint_vector);
            aik_fill_pose_err(
                position_error,
                zero_orientation_error,
                pose_error_snapshot);
            arm_ik_fill_debug_snapshot(
                ARM_IK_SOLVER_POSITION_PRIORITY_LOCAL,
                ARM_IK_FAILURE_NONE,
                APP_AT_IK_POS_PRI_MAX_ITERS,
                1u,
                1u,
                solve_debug_snapshot);
            return OM_OK;
        }

        aik_fill_pose_err(
            position_error,
            zero_orientation_error,
            pose_error_snapshot);
        arm_ik_fill_debug_snapshot(
            ARM_IK_SOLVER_POSITION_PRIORITY_LOCAL,
            ARM_IK_FAILURE_LOCAL_SOLVER_FAILED,
            APP_AT_IK_POS_PRI_MAX_ITERS,
            0u,
            0u,
            solve_debug_snapshot);
    }

    return OM_ERROR;
}

OmRet arm_kinematics_forward(
    const ArmIkJointVector* joint_vector,
    ArmIkPose* pose)
{
    float urdf_joint_rad[ARM_IK_JOINT_COUNT] = {0.0f};

#if (APP_AT_IK_FORWARD_ENABLE != 1u)
    (void)joint_vector;
    (void)pose;
    return OM_ERROR_UNSUPPORTED;
#else
    if (joint_vector == OM_NULL || pose == OM_NULL)
    {
        return OM_ERROR_NULL;
    }

    arm_ik_machine_to_urdf(joint_vector, urdf_joint_rad);
    aik_forward_urdf(urdf_joint_rad, pose);
    return OM_OK;
#endif
}

OmRet aik_inverse_full_local(
    const ArmIkPose* target_pose,
    const ArmIkJointVector* reference_joint_vector,
    ArmIkJointVector* solved_joint_vector,
    ArmIkPoseErr* pose_error_snapshot,
    ArmIkSolveDiag* solve_debug_snapshot)
{
    if (target_pose == OM_NULL || reference_joint_vector == OM_NULL ||
        solved_joint_vector == OM_NULL)
    {
        return OM_ERROR_NULL;
    }

    if (arm_ik_full_pose_reach(target_pose) != OM_TRUE)
    {
        if (pose_error_snapshot != OM_NULL)
        {
            memset(pose_error_snapshot, 0, sizeof(*pose_error_snapshot));
        }
        arm_ik_fill_debug_snapshot(
            ARM_IK_SOLVER_FULL_POSE_LOCAL,
            ARM_IK_FAILURE_POSITION_UNREACHABLE,
            0u,
            0u,
            0u,
            solve_debug_snapshot);
        return OM_ERROR_PARAM;
    }

    return aik_inverse_full(
        target_pose,
        reference_joint_vector,
        solved_joint_vector,
        pose_error_snapshot,
        solve_debug_snapshot);
}

OmRet aik_inverse_pos_local(
    const ArmIkPose* target_pose,
    const ArmIkJointVector* reference_joint_vector,
    ArmIkJointVector* solved_joint_vector,
    ArmIkPoseErr* pose_error_snapshot,
    ArmIkSolveDiag* solve_debug_snapshot)
{
    if (target_pose == OM_NULL || reference_joint_vector == OM_NULL ||
        solved_joint_vector == OM_NULL)
    {
        return OM_ERROR_NULL;
    }

    if (arm_ik_pos_reach(target_pose, reference_joint_vector) != OM_TRUE)
    {
        if (pose_error_snapshot != OM_NULL)
        {
            memset(pose_error_snapshot, 0, sizeof(*pose_error_snapshot));
        }
        arm_ik_fill_debug_snapshot(
            ARM_IK_SOLVER_POSITION_PRIORITY_LOCAL,
            ARM_IK_FAILURE_POSITION_UNREACHABLE,
            0u,
            0u,
            0u,
            solve_debug_snapshot);
        return OM_ERROR_PARAM;
    }

    return aik_inverse_pos(
        target_pose,
        reference_joint_vector,
        solved_joint_vector,
        pose_error_snapshot,
        solve_debug_snapshot);
}

OmRet aik_pose_error(
    const ArmIkPose* target_pose,
    const ArmIkPose* current_pose,
    ArmIkPoseErr* pose_error_snapshot)
{
    float position_error[3] = {0.0f};
    float orientation_error[3] = {0.0f};

    if (target_pose == OM_NULL || current_pose == OM_NULL ||
        pose_error_snapshot == OM_NULL)
    {
        return OM_ERROR_NULL;
    }

    arm_ik_pose_error_inner(
        target_pose,
        current_pose,
        position_error,
        orientation_error);
    aik_fill_pose_err(
        position_error,
        orientation_error,
        pose_error_snapshot);
    return OM_OK;
}

OmRet aik_classify_pose(
    const ArmIkJointVector* joint_vector,
    ArmIkPoseFeat* pose_feature_snapshot)
{
    ArmIkPose pose = {0};
    float target_rotation[3][3] = {{0.0f}};
    float wrist_center[3] = {0.0f};
    float urdf_joint_rad[ARM_IK_JOINT_COUNT] = {0.0f};
    float target_in_shoulder_x = 0.0f;

    if (joint_vector == OM_NULL || pose_feature_snapshot == OM_NULL)
    {
        return OM_ERROR_NULL;
    }

    if (arm_kinematics_forward(joint_vector, &pose) != OM_OK)
    {
        return OM_ERROR;
    }

    arm_ik_machine_to_urdf(joint_vector, urdf_joint_rad);
    arm_ik_rpy_to_matrix(
        pose.orientation_rpy_rad[0],
        pose.orientation_rpy_rad[1],
        pose.orientation_rpy_rad[2],
        target_rotation);

    wrist_center[0] = pose.position_m[0] - g_arm_ik_model.tool_length_m * target_rotation[0][2];
    wrist_center[1] = pose.position_m[1] - g_arm_ik_model.tool_length_m * target_rotation[1][2];
    wrist_center[2] = pose.position_m[2] - g_arm_ik_model.tool_length_m * target_rotation[2][2];

    {
        const float yaw_sin = arm_sin_f32(urdf_joint_rad[0]);
        const float yaw_cos = arm_cos_f32(urdf_joint_rad[0]);
        target_in_shoulder_x =
            yaw_cos * wrist_center[0] +
            yaw_sin * wrist_center[1];
    }

    if (fabsf(target_in_shoulder_x) <= ARM_IK_EPSILON_BRANCH)
    {
        pose_feature_snapshot->shoulder_branch = ARM_IK_SHOULDER_UNKNOWN;
    }
    else
    {
        pose_feature_snapshot->shoulder_branch =
            (target_in_shoulder_x >= 0.0f) ? ARM_IK_SHOULDER_FRONT : ARM_IK_SHOULDER_BACK;
    }

    if (fabsf(urdf_joint_rad[2]) <= ARM_IK_EPSILON_BRANCH)
    {
        pose_feature_snapshot->elbow_branch = ARM_IK_ELBOW_UNKNOWN;
    }
    else
    {
        pose_feature_snapshot->elbow_branch =
            (urdf_joint_rad[2] < 0.0f) ? ARM_IK_ELBOW_DOWN : ARM_IK_ELBOW_UP;
    }

    {
        const float wrist_sin = arm_sin_f32(urdf_joint_rad[4]);
        if (fabsf(wrist_sin) <= ARM_IK_EPSILON_BRANCH)
        {
            pose_feature_snapshot->wrist_branch = ARM_IK_WRIST_SINGULAR;
        }
        else
        {
            pose_feature_snapshot->wrist_branch =
                (urdf_joint_rad[4] > 0.0f) ? ARM_IK_WRIST_NONFLIP : ARM_IK_WRIST_FLIP;
        }
    }

    return OM_OK;
}
