#ifndef NEW_ROBOT_INPUT_TASK_H
#define NEW_ROBOT_INPUT_TASK_H

#include "bsp/bsp_init.h"
#include "core/om_def.h"

typedef struct
{
    volatile uint32_t rx_available_hint;
    volatile uint32_t frame_count;
    volatile uint32_t invalid_frame_count;
    volatile uint32_t last_frame_ms;
    volatile uint32_t last_frame_age_ms;
    volatile uint8_t online;
} InputTaskRcDebugState;

/* 自定义控制器输入的最小调试视图：
 * - rx_available_hint：读回调提示的“可能有新字节”
 * - frame_count：成功收下的 0x0302 帧数量
 * - crc8/crc16/cmd_mismatch：协议层诊断计数
 * - degraded_start：UART8 未成功打开时置 1，表示当前按降级模式运行
 */
typedef struct
{
    volatile uint32_t rx_available_hint;
    volatile uint32_t frame_count;
    volatile uint32_t crc8_fail_count;
    volatile uint32_t crc16_fail_count;
    volatile uint32_t cmd_mismatch_count;
    volatile uint8_t last_seq;
    volatile uint32_t last_frame_age_ms;
    volatile uint8_t degraded_start;
} InputTaskCustomControllerDebugState;

/* 裁判系统输入本轮尚未正式接线。
 * 这里先保留一个最小 runtime 槽位，下一轮真正接 USART3 RX 时
 * 不用再改 input_task 的总调度骨架。
 */
typedef struct
{
    volatile uint32_t rx_available_hint;
    volatile uint32_t reserved_frame_count;
} InputTaskJudgeStubDebugState;

/* input_task 当前是外部控制输入的唯一 owner。
 * 所有来自 USART1/UART8 的控制输入调试状态都统一收进这一个 runtime，
 * 其它任务只消费 formal input snapshot，不再直接拥有输入串口。
 */
typedef struct
{
    InputTaskRcDebugState rc;
    InputTaskCustomControllerDebugState custom_controller;
    InputTaskJudgeStubDebugState judge;
} InputTaskDebugState;

extern InputTaskDebugState g_input_task_runtime;

/**
 * @brief 启动统一外部控制输入任务
 * @param devices BSP 设备注册表，至少需要有效的 `usart1`
 * @return `OM_OK` 表示启动成功，其他返回值表示参数错误、重复启动或任务创建失败
 *
 * 当前 owner 范围：
 * - USART1：DBUS 遥控器
 * - UART8：自定义控制器
 * - USART3：本轮只预留 owner 位，不正式接线
 */
OmRet input_task_start(const BspDeviceRegistry* devices);

#endif
