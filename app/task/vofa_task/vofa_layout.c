#include "task/vofa_task/vofa_layout.h"

#include "config/app_config.h"

/* 当前内置布局 0：
 * - 对齐 arm_task diag_snapshot 的固定 8 路输出
 * - 直接对应“7 轴机构角 + 接管标志”
 */
static const VofaChannelDescriptor g_vofa_layout_arm_machine_angles[] = {
    {.source_kind = VOFA_LAYOUT_SOURCE_TASK_SNAPSHOT, .task_name = "arm_task", .snapshot_index = 0u, .label = "big_yaw_angle", .unit = "rad"},
    {.source_kind = VOFA_LAYOUT_SOURCE_TASK_SNAPSHOT, .task_name = "arm_task", .snapshot_index = 1u, .label = "pitch1_angle", .unit = "rad"},
    {.source_kind = VOFA_LAYOUT_SOURCE_TASK_SNAPSHOT, .task_name = "arm_task", .snapshot_index = 2u, .label = "pitch2_angle", .unit = "rad"},
    {.source_kind = VOFA_LAYOUT_SOURCE_TASK_SNAPSHOT, .task_name = "arm_task", .snapshot_index = 3u, .label = "roll2_angle", .unit = "rad"},
    {.source_kind = VOFA_LAYOUT_SOURCE_TASK_SNAPSHOT, .task_name = "arm_task", .snapshot_index = 4u, .label = "pitch3_angle", .unit = "rad"},
    {.source_kind = VOFA_LAYOUT_SOURCE_TASK_SNAPSHOT, .task_name = "arm_task", .snapshot_index = 5u, .label = "roll3_angle", .unit = "rad"},
    {.source_kind = VOFA_LAYOUT_SOURCE_TASK_SNAPSHOT, .task_name = "arm_task", .snapshot_index = 6u, .label = "grip_angle", .unit = "rad"},
    {.source_kind = VOFA_LAYOUT_SOURCE_TASK_SNAPSHOT, .task_name = "arm_task", .snapshot_index = 7u, .label = "custom_takeover", .unit = "bit"},
};

/* 当前内置布局 1：
 * - 对齐 chassis_task diag_snapshot 的固定 12 路输出
 * - 顺序必须与 chassis_task_diag.h 文档保持一致
 */
static const VofaChannelDescriptor g_vofa_layout_chassis_debug[] = {
    {.source_kind = VOFA_LAYOUT_SOURCE_TASK_SNAPSHOT, .task_name = "chassis_task", .snapshot_index = 0u, .label = "wheel_fr_rpm", .unit = "rpm"},
    {.source_kind = VOFA_LAYOUT_SOURCE_TASK_SNAPSHOT, .task_name = "chassis_task", .snapshot_index = 1u, .label = "wheel_fl_rpm", .unit = "rpm"},
    {.source_kind = VOFA_LAYOUT_SOURCE_TASK_SNAPSHOT, .task_name = "chassis_task", .snapshot_index = 2u, .label = "wheel_bl_rpm", .unit = "rpm"},
    {.source_kind = VOFA_LAYOUT_SOURCE_TASK_SNAPSHOT, .task_name = "chassis_task", .snapshot_index = 3u, .label = "wheel_br_rpm", .unit = "rpm"},
    {.source_kind = VOFA_LAYOUT_SOURCE_TASK_SNAPSHOT, .task_name = "chassis_task", .snapshot_index = 4u, .label = "wheel_fr_current", .unit = "A_like"},
    {.source_kind = VOFA_LAYOUT_SOURCE_TASK_SNAPSHOT, .task_name = "chassis_task", .snapshot_index = 5u, .label = "wheel_fl_current", .unit = "A_like"},
    {.source_kind = VOFA_LAYOUT_SOURCE_TASK_SNAPSHOT, .task_name = "chassis_task", .snapshot_index = 6u, .label = "wheel_bl_current", .unit = "A_like"},
    {.source_kind = VOFA_LAYOUT_SOURCE_TASK_SNAPSHOT, .task_name = "chassis_task", .snapshot_index = 7u, .label = "wheel_br_current", .unit = "A_like"},
    {.source_kind = VOFA_LAYOUT_SOURCE_TASK_SNAPSHOT, .task_name = "chassis_task", .snapshot_index = 8u, .label = "leg_r_angle", .unit = "deg"},
    {.source_kind = VOFA_LAYOUT_SOURCE_TASK_SNAPSHOT, .task_name = "chassis_task", .snapshot_index = 9u, .label = "leg_l_angle", .unit = "deg"},
    {.source_kind = VOFA_LAYOUT_SOURCE_TASK_SNAPSHOT, .task_name = "chassis_task", .snapshot_index = 10u, .label = "leg_r_current", .unit = "A_like"},
    {.source_kind = VOFA_LAYOUT_SOURCE_TASK_SNAPSHOT, .task_name = "chassis_task", .snapshot_index = 11u, .label = "leg_l_current", .unit = "A_like"},
};

/* 当前内置布局 2：
 * - 前 4 路来自 mct diag_snapshot
 * - 后 2 路是 VOFA observer 自己补采的 CAN FIFO 使用量
 */
static const VofaChannelDescriptor g_vofa_layout_mct_runtime[] = {
    {.source_kind = VOFA_LAYOUT_SOURCE_TASK_SNAPSHOT, .task_name = "mct", .snapshot_index = 0u, .label = "operational_active", .unit = "bit"},
    {.source_kind = VOFA_LAYOUT_SOURCE_TASK_SNAPSHOT, .task_name = "mct", .snapshot_index = 1u, .label = "tx_request_sources_mask", .unit = "mask"},
    {.source_kind = VOFA_LAYOUT_SOURCE_TASK_SNAPSHOT, .task_name = "mct", .snapshot_index = 2u, .label = "tx_request_overflowed", .unit = "bit"},
    {.source_kind = VOFA_LAYOUT_SOURCE_TASK_SNAPSHOT, .task_name = "mct", .snapshot_index = 3u, .label = "last_formal_tx_ms", .unit = "ms"},
    {.source_kind = VOFA_LAYOUT_SOURCE_CAN1_TX_FIFO_USED, .task_name = OM_NULL, .snapshot_index = 0u, .label = "can1_tx_fifo_used", .unit = "slot"},
    {.source_kind = VOFA_LAYOUT_SOURCE_CAN2_TX_FIFO_USED, .task_name = OM_NULL, .snapshot_index = 0u, .label = "can2_tx_fifo_used", .unit = "slot"},
};

/* 所有预置布局都集中注册在这里。
 * 后续若增加新布局，优先补一个新的 descriptor 数组和一个新的 VofaLayoutDef，
 * 而不是在 vofa_task.c 里重新写硬编码拼帧逻辑。
 */
static const VofaLayoutDef g_vofa_layout_defs[] = {
    {
        .id = VOFA_LAYOUT_ID_ARM_MACHINE_ANGLES,
        .name = "arm_machine_angles",
        .channel_count = (uint32_t)(sizeof(g_vofa_layout_arm_machine_angles) / sizeof(g_vofa_layout_arm_machine_angles[0])),
        .channels = g_vofa_layout_arm_machine_angles,
    },
    {
        .id = VOFA_LAYOUT_ID_CHASSIS_DEBUG,
        .name = "chassis_debug",
        .channel_count = (uint32_t)(sizeof(g_vofa_layout_chassis_debug) / sizeof(g_vofa_layout_chassis_debug[0])),
        .channels = g_vofa_layout_chassis_debug,
    },
    {
        .id = VOFA_LAYOUT_ID_MCT_RUNTIME,
        .name = "mct_runtime",
        .channel_count = (uint32_t)(sizeof(g_vofa_layout_mct_runtime) / sizeof(g_vofa_layout_mct_runtime[0])),
        .channels = g_vofa_layout_mct_runtime,
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
