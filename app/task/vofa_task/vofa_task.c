#include "task/vofa_task/vofa_task.h"

#include "bsp/bsp_init.h"
#include "config/app_config.h"
#include "drivers/peripheral/can/pal_can_dev.h"
#include "drivers/peripheral/serial/pal_serial_dev.h"
#include "function/vofa/vofa.h"
#include "task/arm_task/arm_task_diag.h"
#include "task/vofa_task/vofa_layout.h"
#include "osal/osal.h"
#include "osal/osal_config.h"
#include "osal/osal_time.h"
#include "module/task_context_pool/task_context_pool.h"
#include <string.h>

/* 当前 VOFA 任务仍是纯观察链：
 * - 上游数据来自 task_context_pool 的 diag_snapshot
 * - 下游只做 UART7 输出
 *
 * 它不能：
 * - 写回正式控制链
 * - 维护任何控制真源
 * - 越过 owner task 直接读取驱动内部状态
 */

#define VOFA_TASK_PERIOD_MS                        APP_VOFA_TASK_PERIOD_MS
#define VOFA_TASK_UART7_BAUDRATE                   (115200u)
#define VOFA_TASK_UART7_TX_BUFSIZE                 (128u)
#define VOFA_TASK_UART7_RX_BUFSIZE                 (64u)
#define VOFA_TASK_UART7_RX_DRAIN_BUDGET            (32u)
#define VOFA_TASK_STACK_BYTES                      (512u * OSAL_STACK_WORD_BYTES)

typedef struct
{
    const char* task_name;
    TaskContextSlotId slot_id;
    float snapshot[VL_MAX_CHANNELS];
    uint32_t snapshot_count;
} VofaResolvedTask;

static float g_vofa_frame[VL_MAX_CHANNELS] = {0.0f};
static const VofaLayoutDef* g_vofa_layout = OM_NULL;
static VofaResolvedTask g_vofa_resolved_tasks[VL_MAX_CHANNELS] = {0};
static uint32_t g_vofa_resolved_task_count = 0u;

static OmRet vofa_task_prepare_uart7(Device* uart7_device)
{
    SerialCfg serial_cfg = SERIAL_DEFAULT_CFG;
    HalSerial* hal_serial = OM_NULL;

    if (uart7_device == OM_NULL)
    {
        return OM_ERROR_NULL;
    }

    serial_cfg.baudrate = VOFA_TASK_UART7_BAUDRATE;
    serial_cfg.txBufSize = VOFA_TASK_UART7_TX_BUFSIZE;
    serial_cfg.rxBufSize = VOFA_TASK_UART7_RX_BUFSIZE;

    hal_serial = (HalSerial*)uart7_device;
    hal_serial->cfg = serial_cfg;

    return device_open(uart7_device, SERIAL_O_NBLCK_RX | SERIAL_O_NBLCK_TX);
}

static void vofa_task_drain_uart7_rx(Device* uart7_device)
{
    uint8_t byte = 0u;
    uint32_t count = 0u;

    if (uart7_device == OM_NULL)
    {
        return;
    }

    while (count < VOFA_TASK_UART7_RX_DRAIN_BUDGET)
    {
        if (device_read(uart7_device, 0, &byte, 1u) != 1u)
        {
            break;
        }
        count++;
    }
}

static float vt_can_tx(Device* can_device)
{
    const HalCanHandler* can = (const HalCanHandler*)can_device;
    size_t total_count = 0u;

    if (can_device == OM_NULL)
    {
        return 0.0f;
    }

    total_count = can->cfg.txMsgListBufSize;
    if (total_count == 0u || can->txHandler.txFifo.freeCount > total_count)
    {
        return 0.0f;
    }

    return (float)(total_count - can->txHandler.txFifo.freeCount);
}

static TaskContextSlotId vofa_task_find_slot_by_name(const char* task_name)
{
    uint32_t i = 0;
    uint32_t count = task_context_pool_count();

    if (task_name == OM_NULL)
    {
        return 0u;
    }

    for (i = 0; i < count; i++)
    {
        TaskContextSlotId slot = task_context_pool_slot_at(i);
        const char* name = task_context_pool_get_name(slot);
        if (name != OM_NULL && strcmp(name, task_name) == 0)
        {
            return slot;
        }
    }

    return 0u;
}

static VofaResolvedTask* vofa_task_find_resolved_task(const char* task_name)
{
    uint32_t index = 0u;

    if (task_name == OM_NULL)
    {
        return OM_NULL;
    }

    for (index = 0u; index < g_vofa_resolved_task_count; index++)
    {
        if (g_vofa_resolved_tasks[index].task_name != OM_NULL &&
            strcmp(g_vofa_resolved_tasks[index].task_name, task_name) == 0)
        {
            return &g_vofa_resolved_tasks[index];
        }
    }

    return OM_NULL;
}

static OmRet vofa_task_prepare_layout(void)
{
    uint32_t channel_index = 0u;

    g_vofa_layout = vofa_layout_get_default();
    if (g_vofa_layout == OM_NULL || g_vofa_layout->channels == OM_NULL ||
        g_vofa_layout->channel_count == 0u ||
        g_vofa_layout->channel_count > VL_MAX_CHANNELS)
    {
        return OM_ERROR_PARAM;
    }

    memset(g_vofa_resolved_tasks, 0, sizeof(g_vofa_resolved_tasks));
    g_vofa_resolved_task_count = 0u;

    for (channel_index = 0u; channel_index < g_vofa_layout->channel_count; channel_index++)
    {
        const VofaChannelDescriptor* descriptor = &g_vofa_layout->channels[channel_index];
        TaskContextSlotId slot_id = 0u;

        if (descriptor->source_kind != VL_SRC_TASK_SNAPSHOT ||
            descriptor->task_name == OM_NULL)
        {
            continue;
        }

        if (vofa_task_find_resolved_task(descriptor->task_name) != OM_NULL)
        {
            continue;
        }

        if (g_vofa_resolved_task_count >= VL_MAX_CHANNELS)
        {
            return OM_ERROR;
        }

        slot_id = vofa_task_find_slot_by_name(descriptor->task_name);
        if (slot_id == 0u)
        {
            return OM_ERROR;
        }

        g_vofa_resolved_tasks[g_vofa_resolved_task_count].task_name = descriptor->task_name;
        g_vofa_resolved_tasks[g_vofa_resolved_task_count].slot_id = slot_id;
        g_vofa_resolved_tasks[g_vofa_resolved_task_count].snapshot_count = 0u;
        g_vofa_resolved_task_count++;
    }

    return OM_OK;
}

static void vt_refresh(void)
{
    uint32_t index = 0u;

    for (index = 0u; index < g_vofa_resolved_task_count; index++)
    {
        memset(g_vofa_resolved_tasks[index].snapshot, 0, sizeof(g_vofa_resolved_tasks[index].snapshot));
        g_vofa_resolved_tasks[index].snapshot_count = 0u;
        if (g_vofa_resolved_tasks[index].slot_id != 0u)
        {
            task_context_pool_diag_snap(
                g_vofa_resolved_tasks[index].slot_id,
                g_vofa_resolved_tasks[index].snapshot,
                VL_MAX_CHANNELS,
                &g_vofa_resolved_tasks[index].snapshot_count);
        }
    }
}

static void vofa_task_fill_frame(
    float frame[VL_MAX_CHANNELS],
    uint32_t* frame_count,
    const BspDeviceRegistry* devices)
{
    uint32_t channel_index = 0u;
    ArmIkPose arm_fk_pose = {0};
    ArmIkPose arm_ik_target_pose = {0};
    ArmIkJointVector arm_ik_joint_vector = {0};
    OmBool arm_fk_pose_ready = OM_FALSE;
    OmBool arm_ik_target_pose_ready = OM_FALSE;
    OmBool arm_ik_joint_ready = OM_FALSE;
    uint8_t arm_mode = 0u;
    OmBool arm_mode_ready = OM_FALSE;

    if (frame == OM_NULL || frame_count == OM_NULL)
    {
        return;
    }

    *frame_count = 0u;
    memset(frame, 0, sizeof(float) * VL_MAX_CHANNELS);

    if (devices == OM_NULL || g_vofa_layout == OM_NULL)
    {
        return;
    }

    vt_refresh();

    for (channel_index = 0u; channel_index < g_vofa_layout->channel_count; channel_index++)
    {
        const VofaChannelDescriptor* descriptor = &g_vofa_layout->channels[channel_index];
        float value = 0.0f;

        switch (descriptor->source_kind)
        {
        case VL_SRC_TASK_SNAPSHOT:
        {
            VofaResolvedTask* resolved_task = vofa_task_find_resolved_task(descriptor->task_name);
            if (resolved_task != OM_NULL &&
                descriptor->snapshot_index < resolved_task->snapshot_count)
            {
                value = resolved_task->snapshot[descriptor->snapshot_index];
            }
            break;
        }

        case VL_SRC_CAN1_TX_USED:
            value = vt_can_tx(devices->can1);
            break;

        case VL_SRC_CAN2_TX_USED:
            value = vt_can_tx(devices->can2);
            break;

        case VOFA_SRC_ARM_FK_POSE:
            if (arm_fk_pose_ready != OM_TRUE)
            {
                arm_fk_pose_ready = arm_task_get_fk_pose_snapshot(&arm_fk_pose);
            }
            if (arm_fk_pose_ready == OM_TRUE && descriptor->snapshot_index < 6u)
            {
                if (descriptor->snapshot_index < 3u)
                {
                    value = arm_fk_pose.position_m[descriptor->snapshot_index];
                }
                else
                {
                    value = arm_fk_pose.orientation_rpy_rad[descriptor->snapshot_index - 3u];
                }
            }
            break;

        case VOFA_SRC_ARM_IK_JOINT:
            if (arm_ik_joint_ready != OM_TRUE)
            {
                arm_ik_joint_ready = arm_task_ik_snapshot(&arm_ik_joint_vector);
            }
            if (arm_ik_joint_ready == OM_TRUE && descriptor->snapshot_index < 6u)
            {
                value = arm_ik_joint_vector.joint_rad[descriptor->snapshot_index];
            }
            break;

        case VL_SRC_ARM_MODE:
            if (arm_mode_ready != OM_TRUE)
            {
                arm_mode_ready = arm_task_mode_snapshot(&arm_mode);
            }
            if (arm_mode_ready == OM_TRUE)
            {
                value = (float)arm_mode;
            }
            break;

        case VOFA_SRC_ARM_IK_TARGET_POS:
            if (arm_ik_target_pose_ready != OM_TRUE)
            {
                arm_ik_target_pose_ready = arm_task_get_ik_target_pose(&arm_ik_target_pose);
            }
            if (arm_ik_target_pose_ready == OM_TRUE && descriptor->snapshot_index < 3u)
            {
                value = arm_ik_target_pose.position_m[descriptor->snapshot_index];
            }
            break;

        case VL_SRC_CONST_ZERO:
        default:
            value = 0.0f;
            break;
        }

        frame[channel_index] = value;
    }

    *frame_count = g_vofa_layout->channel_count;
}

static void vofa_task_entry(void* arg)
{
    const BspDeviceRegistry* devices = (const BspDeviceRegistry*)arg;
    OsalTimeMs deadline_cursor_ms = 0u;
    uint32_t frame_count = 0u;

    while (1)
    {
        vofa_task_drain_uart7_rx(devices->uart7);
        vofa_task_fill_frame(g_vofa_frame, &frame_count, devices);
        if (frame_count != 0u)
        {
            vofa_justfloat_send(devices->uart7, g_vofa_frame, (uint16_t)frame_count);
        }
        (void)osal_delay_until(&deadline_cursor_ms, VOFA_TASK_PERIOD_MS, OM_NULL);
    }
}

OmRet vofa_task_start(const BspDeviceRegistry* devices)
{
    static OsalThread* vofa_task_thread = OM_NULL;
    const OsalThreadAttr vofa_task_attr = {"vofa_task", VOFA_TASK_STACK_BYTES, 3u};
    OsalStatus status = OSAL_INVALID;

    if (devices == OM_NULL || devices->uart7 == OM_NULL)
    {
        return OM_ERROR_NULL;
    }

    if (vofa_task_thread != OM_NULL)
    {
        return OM_ERR_CONFLICT;
    }

    if (vofa_task_prepare_uart7(devices->uart7) != OM_OK)
    {
        return OM_ERROR;
    }

    if (vofa_task_prepare_layout() != OM_OK)
    {
        return OM_ERROR;
    }

    status = osal_thread_create(&vofa_task_thread, &vofa_task_attr, vofa_task_entry, (void*)devices);
    if (status != OSAL_OK)
    {
        vofa_task_thread = OM_NULL;
        return OM_ERROR;
    }

    return OM_OK;
}
