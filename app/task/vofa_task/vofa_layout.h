#ifndef NEW_ROBOT_VOFA_LAYOUT_H
#define NEW_ROBOT_VOFA_LAYOUT_H

#include "core/om_def.h"
#include <stdint.h>

/* VOFA 布局注册表是 observer 面的正式描述层：
 * - 定义“当前上传哪些通道”
 * - 定义“每个通道来自哪个 task diag_snapshot 的哪个索引”
 * - 定义“每个通道的标签与单位”
 *
 * 它不负责：
 * - 生成业务数据
 * - 维护正式控制状态
 * - 改写 task 的诊断语义
 */

#define VL_MAX_CHANNELS (16u)

typedef enum
{
    VL_SRC_TASK_SNAPSHOT = 0u,
    VL_SRC_CAN1_TX_USED,
    VL_SRC_CAN2_TX_USED,
    VOFA_SRC_ARM_FK_POSE,
    VOFA_SRC_ARM_IK_JOINT,
    VL_SRC_ARM_MODE,
    VOFA_SRC_ARM_IK_TARGET_POS,
    VL_SRC_CONST_ZERO,
} VofaLayoutSourceKind;

typedef struct
{
    VofaLayoutSourceKind source_kind;
    const char* task_name;
    uint32_t snapshot_index;
    const char* label;
    const char* unit;
} VofaChannelDescriptor;

typedef struct
{
    uint32_t id;
    const char* name;
    uint32_t channel_count;
    const VofaChannelDescriptor* channels;
} VofaLayoutDef;

/* 预置布局 ID。
 * - 0：机械臂 7 轴机构角 + 接管标志
 * - 1：底盘 4 轮反馈/电流 + 2 腿角度/电流
 * - 2：mct owner 运行态 + CAN1/CAN2 FIFO used
 * - 3：机械臂当前 FK pose + 当前 arm_mode + IK 目标 xyz
 * - 4：机械臂当前 IK joint（big_yaw/pitch1/pitch2/roll2/pitch3/roll3）
 */
#define VL_ID_ARM_MACHINE   (0u)
#define VL_ID_CHASSIS_DEBUG        (1u)
#define VL_ID_MCT_RUNTIME          (2u)
#define VL_ID_ARM_FK_POSE          (3u)
#define VL_ID_ARM_IK_JOINTS        (4u)

/* 返回编译期默认布局。 */
const VofaLayoutDef* vofa_layout_get_default(void);

/* 按布局 ID 查找描述结构；找不到返回 OM_NULL。 */
const VofaLayoutDef* vofa_layout_find_by_id(uint32_t layout_id);

#endif
