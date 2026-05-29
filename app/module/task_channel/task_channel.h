#ifndef NEW_ROBOT_TASK_CHANNEL_H
#define NEW_ROBOT_TASK_CHANNEL_H

#include "core/om_def.h"
#include "data_struct/mpsc_ringbuf.h"
#include "ipc/pipe.h"
#include "osal/osal_queue.h"
#include "osal/osal_sem.h"
#include <stdint.h>

typedef struct
{
    volatile uint32_t submit_ok_count;
    volatile uint32_t submit_would_block_count;
    volatile uint32_t submit_drop_count;
    volatile uint32_t submit_keep_pending_count;
    volatile uint32_t recv_ok_count;
    volatile uint32_t recv_would_block_count;
    volatile uint32_t error_count;
} TaskChannelStats;

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
OmRet task_pipe_channel_submit_nonblocking(
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
OmRet task_mpsc_channel_submit_nonblocking(
    TaskMpscChannel* channel,
    const void* message);
OmRet task_mpsc_channel_receive(
    TaskMpscChannel* channel,
    void* message,
    uint32_t timeout_ms);
OmRet task_mpsc_channel_receive_nonblocking(
    TaskMpscChannel* channel,
    void* message);

/* 只用于低频幂等命令的邮箱包装。
 * 语义固定为：
 * - 队列长度 1
 * - 非阻塞提交
 * - 已有 pending 命令时，新的同类/等价命令允许“视为已挂起成功”
 */
typedef struct
{
    OsalQueue* queue;
    uint32_t command_size_bytes;
    TaskChannelStats stats;
} TaskCommandMailbox;

OmRet task_command_mailbox_init(
    TaskCommandMailbox* mailbox,
    uint32_t command_size_bytes);
void task_command_mailbox_deinit(TaskCommandMailbox* mailbox);
OmRet task_command_mailbox_submit_nonblocking(
    TaskCommandMailbox* mailbox,
    const void* command);
OmRet task_command_mailbox_receive(
    TaskCommandMailbox* mailbox,
    void* command,
    uint32_t timeout_ms);
OmRet task_command_mailbox_reset(TaskCommandMailbox* mailbox);

#endif
