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

/* 公开关节语义固定与当前动作表机构角语义一致：
 * [0] big_yaw
 * [1] pitch1
 * [2] pitch2
 * [3] roll2
 * [4] pitch3
 * [5] roll3
 *
 * 单位均为 rad：
 * - 所有轴都表示“反馈逆映射后的机构角 - 该轴零点”
 * - 在当前项目里，normal 姿态对应各轴 joint 约为 0
 *
 * 算法内部会统一做 normal/home offset 与 wrap 适配。
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
} ArmIkPoseErr;

typedef struct
{
    uint8_t shoulder_branch;
    uint8_t elbow_branch;
    uint8_t wrist_branch;
} ArmIkPoseFeat;

typedef struct
{
    uint8_t solver_kind;
    uint8_t failure_reason;
    uint16_t iteration_count;
    uint16_t candidate_count;
    uint16_t valid_candidate_count;
} ArmIkSolveDiag;

OmRet arm_kinematics_forward(
    const ArmIkJointVector* joint_vector,
    ArmIkPose* pose);

OmRet aik_inverse_full_local(
    const ArmIkPose* target_pose,
    const ArmIkJointVector* reference_joint_vector,
    ArmIkJointVector* solved_joint_vector,
    ArmIkPoseErr* pose_error_snapshot,
    ArmIkSolveDiag* solve_debug_snapshot);

OmRet aik_inverse_pos_local(
    const ArmIkPose* target_pose,
    const ArmIkJointVector* reference_joint_vector,
    ArmIkJointVector* solved_joint_vector,
    ArmIkPoseErr* pose_error_snapshot,
    ArmIkSolveDiag* solve_debug_snapshot);

OmRet aik_pose_error(
    const ArmIkPose* target_pose,
    const ArmIkPose* current_pose,
    ArmIkPoseErr* pose_error_snapshot);

OmRet aik_classify_pose(
    const ArmIkJointVector* joint_vector,
    ArmIkPoseFeat* pose_feature_snapshot);

#endif
