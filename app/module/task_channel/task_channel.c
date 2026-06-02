#include "module/task_channel/task_channel.h"

#include "FreeRTOS.h"
#include "task.h"
#include <string.h>

static OmBool task_channel_is_power_of_two(uint32_t value)
{
    return (value != 0u && (value & (value - 1u)) == 0u) ? OM_TRUE : OM_FALSE;
}

static void task_channel_reset_stats(TaskChannelStats* stats)
{
    if (stats == OM_NULL)
    {
        return;
    }

    memset((void*)stats, 0, sizeof(*stats));
}

static OmRet task_channel_translate_osal_status(OsalStatus status)
{
    switch (status)
    {
    case OSAL_OK:
        return OM_OK;
    case OSAL_WOULD_BLOCK:
        return OM_ERROR_WOULD_BLOCK;
    case OSAL_TIMEOUT:
        return OM_ERROR_TIMEOUT;
    default:
        return OM_ERROR;
    }
}

OmRet task_pipe_channel_init(
    TaskPipeChannel* channel,
    uint8_t* storage,
    uint32_t capacity_bytes,
    uint32_t frame_size_bytes)
{
    OmRet ret = OM_OK;

    if (channel == OM_NULL || storage == OM_NULL || frame_size_bytes == 0u ||
        capacity_bytes == 0u || capacity_bytes < frame_size_bytes ||
        task_channel_is_power_of_two(capacity_bytes) != OM_TRUE)
    {
        return OM_ERROR_PARAM;
    }

    memset(channel, 0, sizeof(*channel));
    channel->frame_size_bytes = frame_size_bytes;
    channel->capacity_bytes = capacity_bytes;
    task_channel_reset_stats(&channel->stats);

    ret = pipe_init(&channel->pipe, storage, capacity_bytes);
    return ret;
}

void task_pipe_channel_deinit(TaskPipeChannel* channel)
{
    if (channel == OM_NULL)
    {
        return;
    }

    pipe_deinit(&channel->pipe);
    channel->frame_size_bytes = 0u;
    channel->capacity_bytes = 0u;
    task_channel_reset_stats(&channel->stats);
}

OmRet task_pipe_channel_submit_nonblocking(
    TaskPipeChannel* channel,
    const void* frame,
    OmBool drop_oldest_on_full)
{
    int write_ret = 0;

    if (channel == OM_NULL || frame == OM_NULL || channel->frame_size_bytes == 0u)
    {
        return OM_ERROR_PARAM;
    }

    while ((uint32_t)pipe_avail(&channel->pipe) < channel->frame_size_bytes)
    {
        if (drop_oldest_on_full != OM_TRUE)
        {
            channel->stats.submit_would_block_count++;
            return OM_ERROR_WOULD_BLOCK;
        }

        if (pipe_skip(&channel->pipe, (int)channel->frame_size_bytes) != OM_OK)
        {
            channel->stats.submit_would_block_count++;
            return OM_ERROR_WOULD_BLOCK;
        }

        channel->stats.submit_drop_count++;
    }

    write_ret = pipe_write(&channel->pipe, frame, (int)channel->frame_size_bytes, 0u);
    if (write_ret == (int)channel->frame_size_bytes)
    {
        channel->stats.submit_ok_count++;
        return OM_OK;
    }

    if (write_ret == OM_ERROR_WOULD_BLOCK)
    {
        channel->stats.submit_would_block_count++;
        return OM_ERROR_WOULD_BLOCK;
    }

    channel->stats.error_count++;
    return OM_ERROR;
}

OmRet task_pipe_channel_receive(
    TaskPipeChannel* channel,
    void* frame,
    uint32_t timeout_ms)
{
    int read_ret = 0;

    if (channel == OM_NULL || frame == OM_NULL || channel->frame_size_bytes == 0u)
    {
        return OM_ERROR_PARAM;
    }

    if (timeout_ms == 0u && (uint32_t)pipe_len(&channel->pipe) < channel->frame_size_bytes)
    {
        channel->stats.recv_would_block_count++;
        return OM_ERROR_WOULD_BLOCK;
    }

    read_ret = pipe_read(&channel->pipe, frame, (int)channel->frame_size_bytes, timeout_ms);
    if (read_ret == (int)channel->frame_size_bytes)
    {
        channel->stats.recv_ok_count++;
        return OM_OK;
    }

    if (read_ret == OM_ERROR_WOULD_BLOCK)
    {
        channel->stats.recv_would_block_count++;
        return OM_ERROR_WOULD_BLOCK;
    }

    if (read_ret == OM_ERROR_TIMEOUT)
    {
        return OM_ERROR_TIMEOUT;
    }

    channel->stats.error_count++;
    return OM_ERROR;
}

OmRet task_mpsc_channel_init(
    TaskMpscChannel* channel,
    uint8_t* storage,
    OmAtomicU8* ready_flags,
    uint32_t message_size_bytes,
    uint32_t message_capacity)
{
    if (channel == OM_NULL || storage == OM_NULL || ready_flags == OM_NULL ||
        message_size_bytes == 0u || message_capacity == 0u)
    {
        return OM_ERROR_PARAM;
    }

    memset(channel, 0, sizeof(*channel));
    channel->message_size_bytes = message_size_bytes;
    channel->message_capacity = message_capacity;
    task_channel_reset_stats(&channel->stats);

    if (osal_sem_create(&channel->read_sem, 1u, 0u) != OSAL_OK)
    {
        return OM_ERROR_MEMORY;
    }

    if (mpscrb_init(
            &channel->ringbuf,
            storage,
            ready_flags,
            message_size_bytes,
            message_capacity) != true)
    {
        (void)osal_sem_delete(channel->read_sem);
        channel->read_sem = OM_NULL;
        return OM_ERROR_PARAM;
    }

    return OM_OK;
}

void task_mpsc_channel_deinit(TaskMpscChannel* channel)
{
    if (channel == OM_NULL)
    {
        return;
    }

    if (channel->read_sem != OM_NULL)
    {
        (void)osal_sem_delete(channel->read_sem);
        channel->read_sem = OM_NULL;
    }

    memset(&channel->ringbuf, 0, sizeof(channel->ringbuf));
    channel->message_size_bytes = 0u;
    channel->message_capacity = 0u;
    task_channel_reset_stats(&channel->stats);
}

OmRet task_mpsc_channel_submit_nonblocking(
    TaskMpscChannel* channel,
    const void* message)
{
    if (channel == OM_NULL || message == OM_NULL)
    {
        return OM_ERROR_PARAM;
    }

    if (mpscrb_in(&channel->ringbuf, message) != true)
    {
        channel->stats.submit_would_block_count++;
        return OM_ERROR_WOULD_BLOCK;
    }

    (void)osal_sem_post(channel->read_sem);
    channel->stats.submit_ok_count++;
    return OM_OK;
}

OmRet task_mpsc_channel_receive(
    TaskMpscChannel* channel,
    void* message,
    uint32_t timeout_ms)
{
    OsalStatus wait_status = OSAL_INVALID;

    if (channel == OM_NULL || message == OM_NULL)
    {
        return OM_ERROR_PARAM;
    }

    if (mpscrb_out(&channel->ringbuf, message) == true)
    {
        channel->stats.recv_ok_count++;
        return OM_OK;
    }

    if (timeout_ms == 0u)
    {
        channel->stats.recv_would_block_count++;
        return OM_ERROR_WOULD_BLOCK;
    }

    wait_status = osal_sem_wait(channel->read_sem, timeout_ms);
    if (wait_status != OSAL_OK)
    {
        if (wait_status == OSAL_WOULD_BLOCK)
        {
            channel->stats.recv_would_block_count++;
            return OM_ERROR_WOULD_BLOCK;
        }

        if (wait_status == OSAL_TIMEOUT)
        {
            return OM_ERROR_TIMEOUT;
        }

        channel->stats.error_count++;
        return OM_ERROR;
    }

    if (mpscrb_out(&channel->ringbuf, message) == true)
    {
        channel->stats.recv_ok_count++;
        return OM_OK;
    }

    channel->stats.recv_would_block_count++;
    return OM_ERROR_WOULD_BLOCK;
}

OmRet task_mpsc_channel_receive_nonblocking(
    TaskMpscChannel* channel,
    void* message)
{
    if (channel == OM_NULL || message == OM_NULL)
    {
        return OM_ERROR_PARAM;
    }

    if (mpscrb_out(&channel->ringbuf, message) != true)
    {
        channel->stats.recv_would_block_count++;
        return OM_ERROR_WOULD_BLOCK;
    }

    channel->stats.recv_ok_count++;
    return OM_OK;
}

OmRet task_command_mailbox_init(
    TaskCommandMailbox* mailbox,
    uint32_t command_size_bytes)
{
    OsalStatus status = OSAL_INVALID;

    if (mailbox == OM_NULL || command_size_bytes == 0u ||
        command_size_bytes > TASK_COMMAND_MAILBOX_MAX_COMMAND_BYTES)
    {
        return OM_ERROR_PARAM;
    }

    memset(mailbox, 0, sizeof(*mailbox));
    mailbox->command_size_bytes = command_size_bytes;
    task_channel_reset_stats(&mailbox->stats);

    status = osal_queue_create(&mailbox->queue, 1u, command_size_bytes);
    return task_channel_translate_osal_status(status);
}

void task_command_mailbox_deinit(TaskCommandMailbox* mailbox)
{
    if (mailbox == OM_NULL)
    {
        return;
    }

    if (mailbox->queue != OM_NULL)
    {
        (void)osal_queue_delete(mailbox->queue);
        mailbox->queue = OM_NULL;
    }

    mailbox->command_size_bytes = 0u;
    mailbox->has_pending_command = 0u;
    memset(mailbox->pending_command_bytes, 0, sizeof(mailbox->pending_command_bytes));
    task_channel_reset_stats(&mailbox->stats);
}

OmRet task_command_mailbox_submit_nonblocking(
    TaskCommandMailbox* mailbox,
    const void* command)
{
    OsalStatus status = OSAL_INVALID;
    OmBool same_pending_command = OM_FALSE;

    if (mailbox == OM_NULL || mailbox->queue == OM_NULL || command == OM_NULL)
    {
        return OM_ERROR_PARAM;
    }

    status = osal_queue_send(mailbox->queue, command, 0u);
    if (status == OSAL_OK)
    {
        taskENTER_CRITICAL();
        memcpy(
            mailbox->pending_command_bytes,
            command,
            (size_t)mailbox->command_size_bytes);
        mailbox->has_pending_command = 1u;
        taskEXIT_CRITICAL();
        mailbox->stats.submit_ok_count++;
        return OM_OK;
    }

    if (status == OSAL_WOULD_BLOCK)
    {
        /* 队列长度固定为 1：
         * - 同类命令重复提交，视为“已经成功挂起”
         * - 异类命令不覆盖已有 pending，由调用方下次再试
         */
        taskENTER_CRITICAL();
        if (mailbox->has_pending_command != 0u &&
            memcmp(
                mailbox->pending_command_bytes,
                command,
                (size_t)mailbox->command_size_bytes) == 0)
        {
            same_pending_command = OM_TRUE;
        }
        taskEXIT_CRITICAL();

        if (same_pending_command == OM_TRUE)
        {
            mailbox->stats.submit_keep_pending_count++;
            return OM_OK;
        }

        mailbox->stats.submit_would_block_count++;
        return OM_ERROR_WOULD_BLOCK;
    }

    mailbox->stats.error_count++;
    return task_channel_translate_osal_status(status);
}

OmRet task_command_mailbox_receive(
    TaskCommandMailbox* mailbox,
    void* command,
    uint32_t timeout_ms)
{
    OsalStatus status = OSAL_INVALID;

    if (mailbox == OM_NULL || mailbox->queue == OM_NULL || command == OM_NULL)
    {
        return OM_ERROR_PARAM;
    }

    status = osal_queue_recv(mailbox->queue, command, timeout_ms);
    if (status == OSAL_OK)
    {
        taskENTER_CRITICAL();
        mailbox->has_pending_command = 0u;
        memset(mailbox->pending_command_bytes, 0, sizeof(mailbox->pending_command_bytes));
        taskEXIT_CRITICAL();
        mailbox->stats.recv_ok_count++;
        return OM_OK;
    }

    if (status == OSAL_WOULD_BLOCK)
    {
        mailbox->stats.recv_would_block_count++;
        return OM_ERROR_WOULD_BLOCK;
    }

    if (status == OSAL_TIMEOUT)
    {
        return OM_ERROR_TIMEOUT;
    }

    mailbox->stats.error_count++;
    return OM_ERROR;
}

OmRet task_command_mailbox_reset(TaskCommandMailbox* mailbox)
{
    OsalStatus status = OSAL_INVALID;

    if (mailbox == OM_NULL || mailbox->queue == OM_NULL)
    {
        return OM_ERROR_PARAM;
    }

    status = osal_queue_reset(mailbox->queue);
    if (status == OSAL_OK)
    {
        taskENTER_CRITICAL();
        mailbox->has_pending_command = 0u;
        memset(mailbox->pending_command_bytes, 0, sizeof(mailbox->pending_command_bytes));
        taskEXIT_CRITICAL();
    }
    return task_channel_translate_osal_status(status);
}
