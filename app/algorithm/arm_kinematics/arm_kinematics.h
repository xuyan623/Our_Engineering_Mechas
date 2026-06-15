#ifndef NEW_ROBOT_ARM_KINEMATICS_H
#define NEW_ROBOT_ARM_KINEMATICS_H

#include "core/om_def.h"
#include <stdint.h>

#define ARM_IK_JOINT_COUNT (6u)

typedef enum
{
    ARM_IK_SOLVER_NONE = 0u,
    ARM_IK_SOLVER_FULL_POSE_LOCAL,
    ARM_IK_SOLVER_POSITION_PRIORITY_LOCAL,
} ArmIkSolverKind;

typedef enum
{
    ARM_IK_FAILURE_NONE = 0u,
    ARM_IK_FAILURE_POSITION_UNREACHABLE,
    ARM_IK_FAILURE_LOCAL_SOLVER_FAILED,
} ArmIkFailureReason;

typedef enum
{
    ARM_IK_SHOULDER_UNKNOWN = 0u,
    ARM_IK_SHOULDER_FRONT,
    ARM_IK_SHOULDER_BACK,
} ArmIkShoulderBranch;

typedef enum
{
    ARM_IK_ELBOW_UNKNOWN = 0u,
    ARM_IK_ELBOW_DOWN,
    ARM_IK_ELBOW_UP,
} ArmIkElbowBranch;

typedef enum
{
    ARM_IK_WRIST_UNKNOWN = 0u,
    ARM_IK_WRIST_NONFLIP,
    ARM_IK_WRIST_FLIP,
    ARM_IK_WRIST_SINGULAR,
} ArmIkWristBranch;

/* 公开关节语义固定沿用当前固件 machine pose：
 * [0] big_yaw
 * [1] pitch1
 * [2] pitch2
 * [3] roll2
 * [4] pitch3
 * [5] roll3
 *
 * 单位均为 rad，其中 roll3 允许保持 machine 侧绝对角语义，
 * 算法内部会自行 wrap 到 [-pi, pi]。
 */
typedef struct
{
    float joint_rad[ARM_IK_JOINT_COUNT];
} ArmIkJointVector;

/* 末端位姿：
 * - position_m：x/y/z，单位 m
 * - orientation_rpy_rad：roll/pitch/yaw，单位 rad
 */
typedef struct
{
    float position_m[3];
    float orientation_rpy_rad[3];
} ArmIkPose;

typedef struct
{
    float position_error_m[3];
    float orientation_error_rad[3];
    float position_error_norm_m;
    float orientation_error_norm_rad;
} ArmIkPoseErrorSnapshot;

typedef struct
{
    uint8_t shoulder_branch;
    uint8_t elbow_branch;
    uint8_t wrist_branch;
} ArmIkPoseFeatureSnapshot;

typedef struct
{
    uint8_t solver_kind;
    uint8_t failure_reason;
    uint16_t iteration_count;
    uint16_t candidate_count;
    uint16_t valid_candidate_count;
} ArmIkSolveDebugSnapshot;

OmRet arm_kinematics_forward(
    const ArmIkJointVector* joint_vector,
    ArmIkPose* pose);

OmRet arm_kinematics_inverse_full_pose_local(
    const ArmIkPose* target_pose,
    const ArmIkJointVector* reference_joint_vector,
    ArmIkJointVector* solved_joint_vector,
    ArmIkPoseErrorSnapshot* pose_error_snapshot,
    ArmIkSolveDebugSnapshot* solve_debug_snapshot);

OmRet arm_kinematics_inverse_position_priority_local(
    const ArmIkPose* target_pose,
    const ArmIkJointVector* reference_joint_vector,
    ArmIkJointVector* solved_joint_vector,
    ArmIkPoseErrorSnapshot* pose_error_snapshot,
    ArmIkSolveDebugSnapshot* solve_debug_snapshot);

OmRet arm_kinematics_compute_pose_error(
    const ArmIkPose* target_pose,
    const ArmIkPose* current_pose,
    ArmIkPoseErrorSnapshot* pose_error_snapshot);

OmRet arm_kinematics_classify_pose_features(
    const ArmIkJointVector* joint_vector,
    ArmIkPoseFeatureSnapshot* pose_feature_snapshot);

#endif
