#include "task/vofa_task/vofa_layout.h"

#include "config/app_config.h"

/* 当前内置布局 0：
 * - 对齐 arm_task diag_snapshot 的固定 8 路输出
 * - 直接对应“7 轴机构角 + 接管标志”
 */
static const VofaChannelDescriptor g_vofa_layout_arm_machine_angles[] = {
    {.source_kind = VL_SRC_TASK_SNAPSHOT, .task_name = "arm_task", .snapshot_index = 0u, .label = "big_yaw_angle", .unit = "rad"},
    {.source_kind = VL_SRC_TASK_SNAPSHOT, .task_name = "arm_task", .snapshot_index = 1u, .label = "pitch1_angle", .unit = "rad"},
    {.source_kind = VL_SRC_TASK_SNAPSHOT, .task_name = "arm_task", .snapshot_index = 2u, .label = "pitch2_angle", .unit = "rad"},
    {.source_kind = VL_SRC_TASK_SNAPSHOT, .task_name = "arm_task", .snapshot_index = 3u, .label = "roll2_angle", .unit = "rad"},
    {.source_kind = VL_SRC_TASK_SNAPSHOT, .task_name = "arm_task", .snapshot_index = 4u, .label = "pitch3_angle", .unit = "rad"},
    {.source_kind = VL_SRC_TASK_SNAPSHOT, .task_name = "arm_task", .snapshot_index = 5u, .label = "roll3_angle", .unit = "rad"},
    {.source_kind = VL_SRC_TASK_SNAPSHOT, .task_name = "arm_task", .snapshot_index = 6u, .label = "grip_angle", .unit = "rad"},
    {.source_kind = VL_SRC_TASK_SNAPSHOT, .task_name = "arm_task", .snapshot_index = 7u, .label = "custom_takeover", .unit = "bit"},
};

/* 当前内置布局 1：
 * - 对齐 chassis_task diag_snapshot 的固定 12 路输出
 * - 顺序必须与 chassis_task_diag.h 文档保持一致
 */
static const VofaChannelDescriptor g_vofa_layout_chassis_debug[] = {
    {.source_kind = VL_SRC_TASK_SNAPSHOT, .task_name = "chassis_task", .snapshot_index = 0u, .label = "wheel_fr_rpm", .unit = "rpm"},
    {.source_kind = VL_SRC_TASK_SNAPSHOT, .task_name = "chassis_task", .snapshot_index = 1u, .label = "wheel_fl_rpm", .unit = "rpm"},
    {.source_kind = VL_SRC_TASK_SNAPSHOT, .task_name = "chassis_task", .snapshot_index = 2u, .label = "wheel_bl_rpm", .unit = "rpm"},
    {.source_kind = VL_SRC_TASK_SNAPSHOT, .task_name = "chassis_task", .snapshot_index = 3u, .label = "wheel_br_rpm", .unit = "rpm"},
    {.source_kind = VL_SRC_TASK_SNAPSHOT, .task_name = "chassis_task", .snapshot_index = 4u, .label = "wheel_fr_current", .unit = "A_like"},
    {.source_kind = VL_SRC_TASK_SNAPSHOT, .task_name = "chassis_task", .snapshot_index = 5u, .label = "wheel_fl_current", .unit = "A_like"},
    {.source_kind = VL_SRC_TASK_SNAPSHOT, .task_name = "chassis_task", .snapshot_index = 6u, .label = "wheel_bl_current", .unit = "A_like"},
    {.source_kind = VL_SRC_TASK_SNAPSHOT, .task_name = "chassis_task", .snapshot_index = 7u, .label = "wheel_br_current", .unit = "A_like"},
    {.source_kind = VL_SRC_TASK_SNAPSHOT, .task_name = "chassis_task", .snapshot_index = 8u, .label = "leg_r_angle", .unit = "deg"},
    {.source_kind = VL_SRC_TASK_SNAPSHOT, .task_name = "chassis_task", .snapshot_index = 9u, .label = "leg_l_angle", .unit = "deg"},
    {.source_kind = VL_SRC_TASK_SNAPSHOT, .task_name = "chassis_task", .snapshot_index = 10u, .label = "leg_r_current", .unit = "A_like"},
    {.source_kind = VL_SRC_TASK_SNAPSHOT, .task_name = "chassis_task", .snapshot_index = 11u, .label = "leg_l_current", .unit = "A_like"},
};

/* 当前内置布局 2：
 * - 前 4 路来自 mct diag_snapshot
 * - 后 2 路是 VOFA observer 自己补采的 CAN FIFO 使用量
 */
static const VofaChannelDescriptor g_vofa_layout_mct_runtime[] = {
    {.source_kind = VL_SRC_TASK_SNAPSHOT, .task_name = "mct", .snapshot_index = 0u, .label = "operational_active", .unit = "bit"},
    {.source_kind = VL_SRC_TASK_SNAPSHOT, .task_name = "mct", .snapshot_index = 1u, .label = "tx_request_sources_mask", .unit = "mask"},
    {.source_kind = VL_SRC_TASK_SNAPSHOT, .task_name = "mct", .snapshot_index = 2u, .label = "tx_request_overflowed", .unit = "bit"},
    {.source_kind = VL_SRC_TASK_SNAPSHOT, .task_name = "mct", .snapshot_index = 3u, .label = "last_formal_tx_ms", .unit = "ms"},
    {.source_kind = VL_SRC_CAN1_TX_USED, .task_name = OM_NULL, .snapshot_index = 0u, .label = "can1_tx_fifo_used", .unit = "slot"},
    {.source_kind = VL_SRC_CAN2_TX_USED, .task_name = OM_NULL, .snapshot_index = 0u, .label = "can2_tx_fifo_used", .unit = "slot"},
};

/* 当前内置布局 3：
 * - 前 6 路：当前反馈做 FK 得到的末端 pose
 * - 第 7 路：当前 arm_mode
 * - 最后 3 路：当前 IK 目标 xyz，仅在 RC_IK 模式且 target 已初始化时有效
 */
static const VofaChannelDescriptor g_vofa_layout_arm_fk_pose[] = {
    {.source_kind = VOFA_SRC_ARM_FK_POSE, .task_name = OM_NULL, .snapshot_index = 0u, .label = "fk_x", .unit = "m"},
    {.source_kind = VOFA_SRC_ARM_FK_POSE, .task_name = OM_NULL, .snapshot_index = 1u, .label = "fk_y", .unit = "m"},
    {.source_kind = VOFA_SRC_ARM_FK_POSE, .task_name = OM_NULL, .snapshot_index = 2u, .label = "fk_z", .unit = "m"},
    {.source_kind = VOFA_SRC_ARM_FK_POSE, .task_name = OM_NULL, .snapshot_index = 3u, .label = "fk_roll", .unit = "rad"},
    {.source_kind = VOFA_SRC_ARM_FK_POSE, .task_name = OM_NULL, .snapshot_index = 4u, .label = "fk_pitch", .unit = "rad"},
    {.source_kind = VOFA_SRC_ARM_FK_POSE, .task_name = OM_NULL, .snapshot_index = 5u, .label = "fk_yaw", .unit = "rad"},
    {.source_kind = VL_SRC_ARM_MODE, .task_name = OM_NULL, .snapshot_index = 0u, .label = "arm_mode", .unit = "enum"},
    {.source_kind = VOFA_SRC_ARM_IK_TARGET_POS, .task_name = OM_NULL, .snapshot_index = 0u, .label = "target_x", .unit = "m"},
    {.source_kind = VOFA_SRC_ARM_IK_TARGET_POS, .task_name = OM_NULL, .snapshot_index = 1u, .label = "target_y", .unit = "m"},
    {.source_kind = VOFA_SRC_ARM_IK_TARGET_POS, .task_name = OM_NULL, .snapshot_index = 2u, .label = "target_z", .unit = "m"},
};

/* 当前内置布局 4：
 * - 直接调用 arm_task IK joint 只读接口
 * - 直接对应当前反馈逆映射得到的 IK joint vector
 */
static const VofaChannelDescriptor g_vofa_layout_arm_ik_joints[] = {
    {.source_kind = VOFA_SRC_ARM_IK_JOINT, .task_name = OM_NULL, .snapshot_index = 0u, .label = "ik_big_yaw", .unit = "rad"},
    {.source_kind = VOFA_SRC_ARM_IK_JOINT, .task_name = OM_NULL, .snapshot_index = 1u, .label = "ik_pitch1", .unit = "rad"},
    {.source_kind = VOFA_SRC_ARM_IK_JOINT, .task_name = OM_NULL, .snapshot_index = 2u, .label = "ik_pitch2", .unit = "rad"},
    {.source_kind = VOFA_SRC_ARM_IK_JOINT, .task_name = OM_NULL, .snapshot_index = 3u, .label = "ik_roll2", .unit = "rad"},
    {.source_kind = VOFA_SRC_ARM_IK_JOINT, .task_name = OM_NULL, .snapshot_index = 4u, .label = "ik_pitch3", .unit = "rad"},
    {.source_kind = VOFA_SRC_ARM_IK_JOINT, .task_name = OM_NULL, .snapshot_index = 5u, .label = "ik_roll3", .unit = "rad"},
};

/* 所有预置布局都集中注册在这里。
 * 后续若增加新布局，优先补一个新的 descriptor 数组和一个新的 VofaLayoutDef，
 * 而不是在 vofa_task.c 里重新写硬编码拼帧逻辑。
 */
static const VofaLayoutDef g_vofa_layout_defs[] = {
    {
        .id = VL_ID_ARM_MACHINE,
        .name = "arm_machine_angles",
        .channel_count = (uint32_t)(sizeof(g_vofa_layout_arm_machine_angles) / sizeof(g_vofa_layout_arm_machine_angles[0])),
        .channels = g_vofa_layout_arm_machine_angles,
    },
    {
        .id = VL_ID_CHASSIS_DEBUG,
        .name = "chassis_debug",
        .channel_count = (uint32_t)(sizeof(g_vofa_layout_chassis_debug) / sizeof(g_vofa_layout_chassis_debug[0])),
        .channels = g_vofa_layout_chassis_debug,
    },
    {
        .id = VL_ID_MCT_RUNTIME,
        .name = "mct_runtime",
        .channel_count = (uint32_t)(sizeof(g_vofa_layout_mct_runtime) / sizeof(g_vofa_layout_mct_runtime[0])),
        .channels = g_vofa_layout_mct_runtime,
    },
    {
        .id = VL_ID_ARM_FK_POSE,
        .name = "arm_fk_pose",
        .channel_count = (uint32_t)(sizeof(g_vofa_layout_arm_fk_pose) / sizeof(g_vofa_layout_arm_fk_pose[0])),
        .channels = g_vofa_layout_arm_fk_pose,
    },
    {
        .id = VL_ID_ARM_IK_JOINTS,
        .name = "arm_ik_joints",
        .channel_count = (uint32_t)(sizeof(g_vofa_layout_arm_ik_joints) / sizeof(g_vofa_layout_arm_ik_joints[0])),
        .channels = g_vofa_layout_arm_ik_joints,
    },
};

const VofaLayoutDef* vofa_layout_find_by_id(uint32_t layout_id)
{
    uint32_t index = 0u;

    for (index = 0u; index < (uint32_t)(sizeof(g_vofa_layout_defs) / sizeof(g_vofa_layout_defs[0])); index++)
    {
        if (g_vofa_layout_defs[index].id == layout_id)
        {
            return &g_vofa_layout_defs[index];
        }
    }

    return OM_NULL;
}

const VofaLayoutDef* vofa_layout_get_default(void)
{
    return vofa_layout_find_by_id(APP_VOFA_DEFAULT_LAYOUT_ID);
}
