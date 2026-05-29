#include "task/input_task/input_task_rc.h"

#include "module/data_pool/data_pool.h"
#include <stdlib.h>
#include <string.h>

/* 旧 DBUS 中位附近的小抖动直接压到 0，
 * 避免 mode_task / chassis_task 每轮都看到噪声输入。
 */
static int16_t input_task_rc_apply_deadband(int16_t value)
{
    if (value <= INPUT_TASK_DBUS_CHANNEL_DEADBAND &&
        value >= -INPUT_TASK_DBUS_CHANNEL_DEADBAND)
    {
        return 0;
    }

    return value;
}

void input_task_rc_reset_runtime(InputTaskRcDebugState* runtime)
{
    if (runtime == OM_NULL)
    {
        return;
    }

    memset((void*)runtime, 0, sizeof(*runtime));
}

OmBool input_task_rc_decode_frame(
    const uint8_t raw_frame[INPUT_TASK_DBUS_FRAME_LEN],
    InputTaskRcFrame* frame)
{
    if (raw_frame == OM_NULL || frame == OM_NULL)
    {
        return OM_FALSE;
    }

    memset(frame, 0, sizeof(*frame));

    /* DBUS 的 4 个摇杆通道都是 11 bit 压缩编码，
     * 这里完全按旧工程位布局展开，不引入新的协议解释。
     */
    frame->ch1 =
        (int16_t)(((raw_frame[0] | (raw_frame[1] << 8)) &
                   INPUT_TASK_DBUS_11BIT_MASK) -
                  INPUT_TASK_DBUS_CHANNEL_CENTER);
    frame->ch2 =
        (int16_t)((((raw_frame[1] >> 3) | (raw_frame[2] << 5)) &
                   INPUT_TASK_DBUS_11BIT_MASK) -
                  INPUT_TASK_DBUS_CHANNEL_CENTER);
    frame->ch3 =
        (int16_t)((((raw_frame[2] >> 6) | (raw_frame[3] << 2) |
                    (raw_frame[4] << 10)) &
                   INPUT_TASK_DBUS_11BIT_MASK) -
                  INPUT_TASK_DBUS_CHANNEL_CENTER);
    frame->ch4 =
        (int16_t)((((raw_frame[4] >> 1) | (raw_frame[5] << 7)) &
                   INPUT_TASK_DBUS_11BIT_MASK) -
                  INPUT_TASK_DBUS_CHANNEL_CENTER);

    frame->ch1 = input_task_rc_apply_deadband(frame->ch1);
    frame->ch2 = input_task_rc_apply_deadband(frame->ch2);
    frame->ch3 = input_task_rc_apply_deadband(frame->ch3);
    frame->ch4 = input_task_rc_apply_deadband(frame->ch4);

    frame->sw1 = (uint8_t)(((raw_frame[5] >> 4) & 0x0Cu) >> 2);
    frame->sw2 = (uint8_t)((raw_frame[5] >> 4) & 0x03u);
    frame->iw =
        (uint16_t)((raw_frame[16] | (raw_frame[17] << 8)) &
                   INPUT_TASK_DBUS_11BIT_MASK);

    frame->mouse.x = (int16_t)(raw_frame[6] | (raw_frame[7] << 8));
    frame->mouse.y = (int16_t)(raw_frame[8] | (raw_frame[9] << 8));
    frame->mouse.z = (int16_t)(raw_frame[10] | (raw_frame[11] << 8));
    frame->mouse.l = raw_frame[12];
    frame->mouse.r = raw_frame[13];
    frame->keyboard_bits = (uint16_t)(raw_frame[14] | (raw_frame[15] << 8));

    /* 摇杆绝对值超出旧工程经验范围时，认为当前帧已错位或损坏。
     * 这里直接清零该帧，保持下游控制逻辑看到的是“安全输入”。
     */
    if ((abs(frame->ch1) > INPUT_TASK_DBUS_CHANNEL_MAX_ABS) ||
        (abs(frame->ch2) > INPUT_TASK_DBUS_CHANNEL_MAX_ABS) ||
        (abs(frame->ch3) > INPUT_TASK_DBUS_CHANNEL_MAX_ABS) ||
        (abs(frame->ch4) > INPUT_TASK_DBUS_CHANNEL_MAX_ABS))
    {
        memset(frame, 0, sizeof(*frame));
        return OM_FALSE;
    }

    return OM_TRUE;
}

void input_task_rc_store_to_data_pool(const InputTaskRcFrame* frame)
{
    DpRcSnapshot snapshot = {0};

    if (frame == OM_NULL)
    {
        return;
    }

    input_task_rc_fill_snapshot(frame, &snapshot);
    dp_store_rc_snapshot(&snapshot);
}

void input_task_rc_fill_snapshot(
    const InputTaskRcFrame* frame,
    DpRcSnapshot* snapshot)
{
    if (frame == OM_NULL || snapshot == OM_NULL)
    {
        return;
    }

    memset(snapshot, 0, sizeof(*snapshot));
    snapshot->ch1 = frame->ch1;
    snapshot->ch2 = frame->ch2;
    snapshot->ch3 = frame->ch3;
    snapshot->ch4 = frame->ch4;
    snapshot->sw1 = frame->sw1;
    snapshot->sw2 = frame->sw2;
    snapshot->iw = frame->iw;
    snapshot->online = 1u;
    snapshot->mouse.x = frame->mouse.x;
    snapshot->mouse.y = frame->mouse.y;
    snapshot->mouse.z = frame->mouse.z;
    snapshot->mouse.l = frame->mouse.l;
    snapshot->mouse.r = frame->mouse.r;
    snapshot->keyboard_bits = frame->keyboard_bits;
}

void input_task_rc_update_online_state(
    InputTaskRcDebugState* runtime,
    OsalTimeMs now_ms)
{
    DpRcSnapshot snapshot = {0};

    if (runtime == OM_NULL)
    {
        return;
    }

    dp_copy_rc_snapshot(&snapshot);

    if (runtime->last_frame_ms == 0u)
    {
        runtime->last_frame_age_ms = 0u;
        runtime->online = 0u;
        snapshot.online = 0u;
        dp_store_rc_snapshot(&snapshot);
        return;
    }

    runtime->last_frame_age_ms = (uint32_t)(now_ms - runtime->last_frame_ms);
    runtime->online =
        (runtime->last_frame_age_ms <= INPUT_TASK_DBUS_FRAME_TIMEOUT_MS) ? 1u : 0u;
    snapshot.online = runtime->online;
    dp_store_rc_snapshot(&snapshot);
}
