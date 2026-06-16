#ifndef NEW_ROBOT_TCH_H
#define NEW_ROBOT_TCH_H

/* task_channel 的职责边界：
 * - 它只统一 transport 契约：非阻塞提交、接收、统计字段
 * - 它不拥有任何业务真源，也不保存“最新控制状态”
 *
 * 正确用法：
 * - producer 只负责投递消息或命令
 * - consumer task 把收到的数据 drain 到自己的 latest-cache
 * - 真正的业务判断永远发生在 owner / consumer task 本地 context 中
 */

#include "core/om_def.h"
#include "data_struct/mpsc_ringbuf.h"
#include "ipc/pipe.h"
#include "osal/osal_queue.h"
#include "osal/osal_sem.h"
#include <stdint.h>

typedef struct
{
    /* 这些计数只用于诊断与观测，不参与正式控制语义。 */
    volatile uint32_t submit_ok_count;
    volatile uint32_t submit_would_block_count;
    volatile uint32_t submit_drop_count;
    volatile uint32_t submit_keep_pending_count;
    volatile uint32_t recv_ok_count;
    volatile uint32_t recv_would_block_count;
    volatile uint32_t error_count;
} TaskChannelStats;

#define TASK_MAILBOX_CMD_MAX_BYTES (16u)

/* 固定帧长度的 SPSC pipe 包装。
 * 语义固定为：
 * - 生产者非阻塞整帧提交
 * - 可选“满时丢一帧最旧数据后重试”
 * - 不允许部分帧写入
 */
typedef struct
{
    Pipe pipe;
    uint32_t frame_size_bytes;
    uint32_t capacity_bytes;
    TaskChannelStats stats;
} TaskPipeChannel;

OmRet task_pipe_channel_init(
    TaskPipeChannel* channel,
    uint8_t* storage,
    uint32_t capacity_bytes,
    uint32_t frame_size_bytes);
void task_pipe_channel_deinit(TaskPipeChannel* channel);
OmRet tpipe_submit(
    TaskPipeChannel* channel,
    const void* frame,
    OmBool drop_oldest_on_full);
OmRet task_pipe_channel_receive(
    TaskPipeChannel* channel,
    void* frame,
    uint32_t timeout_ms);

/* 固定消息大小的 MPSC 包装。
 * 语义固定为：
 * - 多生产者非阻塞投递
 * - 满时直接返回 would-block，不等待消费者
 * - consumer 应在本地 context 中维护 latest-cache；
 *   channel 自身不承担“最新状态真源”的职责
 */
typedef struct
{
    MpscRingbuf ringbuf;
    OsalSem* read_sem;
    uint32_t message_size_bytes;
    uint32_t message_capacity;
    TaskChannelStats stats;
} TaskMpscChannel;

OmRet task_mpsc_channel_init(
    TaskMpscChannel* channel,
    uint8_t* storage,
    OmAtomicU8* ready_flags,
    uint32_t message_size_bytes,
    uint32_t message_capacity);
void task_mpsc_channel_deinit(TaskMpscChannel* channel);
OmRet tmpsc_submit(
    TaskMpscChannel* channel,
    const void* message);
/* receive(timeout_ms) 适合“等待下一条消息”的消费者。 */
OmRet task_mpsc_channel_receive(
    TaskMpscChannel* channel,
    void* message,
    uint32_t timeout_ms);
/* receive_nonblocking() 适合任务主循环内 drain 到 latest-cache。 */
OmRet tmpsc_receive(
    TaskMpscChannel* channel,
    void* message);

/* 只用于低频幂等命令的邮箱包装。
 * 语义固定为：
 * - 队列长度 1
 * - 非阻塞提交
 * - 已有 pending 命令时，只有“原始命令字节完全相同”的重复提交
 *   才视为“已经成功挂起”
 *
 * 注意：
 * - 这里当前实现的“同类命令”不是业务语义等价，而是字节完全相等
 * - 若调用方需要更宽松的“业务等价命令合并”，应在更上层先做归一化
 */
typedef struct
{
    OsalQueue* queue;
    uint32_t command_size_bytes;
    uint8_t pending_command_bytes[TASK_MAILBOX_CMD_MAX_BYTES];
    uint8_t has_pending_command;
    TaskChannelStats stats;
} TaskCommandMailbox;

OmRet task_command_mailbox_init(
    TaskCommandMailbox* mailbox,
    uint32_t command_size_bytes);
void task_command_mailbox_deinit(TaskCommandMailbox* mailbox);
OmRet tmail_submit(
    TaskCommandMailbox* mailbox,
    const void* command);
OmRet tmail_receive(
    TaskCommandMailbox* mailbox,
    void* command,
    uint32_t timeout_ms);
OmRet task_command_mailbox_reset(TaskCommandMailbox* mailbox);

#endif
