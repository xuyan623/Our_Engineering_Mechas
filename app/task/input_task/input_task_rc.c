#include "task/input_task/input_task_rc.h"

#include <stdlib.h>
#include <string.h>

static InputRcSnapshot g_input_task_rc_snapshot = {0};

/* 旧 DBUS 中位附近的小抖动直接压到 0，
 * 避免 mode_task / chassis_task 每轮都看到噪声输入。
 */
static int16_t input_task_rc_apply_deadband(int16_t value)
{
    if (value <= IT_DBUS_CHANNEL_DEADBAND &&
        value >= -IT_DBUS_CHANNEL_DEADBAND)
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

void input_task_rc_reset_latest(void)
{
    memset(&g_input_task_rc_snapshot, 0, sizeof(g_input_task_rc_snapshot));
}

OmBool input_task_rc_decode_frame(
    const uint8_t raw_frame[IT_DBUS_FRAME_LEN],
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
                   IT_DBUS_11BIT_MASK) -
                  IT_DBUS_CHANNEL_CENTER);
    frame->ch2 =
        (int16_t)((((raw_frame[1] >> 3) | (raw_frame[2] << 5)) &
                   IT_DBUS_11BIT_MASK) -
                  IT_DBUS_CHANNEL_CENTER);
    frame->ch3 =
        (int16_t)((((raw_frame[2] >> 6) | (raw_frame[3] << 2) |
                    (raw_frame[4] << 10)) &
                   IT_DBUS_11BIT_MASK) -
                  IT_DBUS_CHANNEL_CENTER);
    frame->ch4 =
        (int16_t)((((raw_frame[4] >> 1) | (raw_frame[5] << 7)) &
                   IT_DBUS_11BIT_MASK) -
                  IT_DBUS_CHANNEL_CENTER);

    frame->ch1 = input_task_rc_apply_deadband(frame->ch1);
    frame->ch2 = input_task_rc_apply_deadband(frame->ch2);
    frame->ch3 = input_task_rc_apply_deadband(frame->ch3);
    frame->ch4 = input_task_rc_apply_deadband(frame->ch4);

    frame->sw1 = (uint8_t)(((raw_frame[5] >> 4) & 0x0Cu) >> 2);
    frame->sw2 = (uint8_t)((raw_frame[5] >> 4) & 0x03u);
    frame->iw =
        (uint16_t)((raw_frame[16] | (raw_frame[17] << 8)) &
                   IT_DBUS_11BIT_MASK);

    frame->mouse.x = (int16_t)(raw_frame[6] | (raw_frame[7] << 8));
    frame->mouse.y = (int16_t)(raw_frame[8] | (raw_frame[9] << 8));
    frame->mouse.z = (int16_t)(raw_frame[10] | (raw_frame[11] << 8));
    frame->mouse.l = raw_frame[12];
    frame->mouse.r = raw_frame[13];
    frame->keyboard_bits = (uint16_t)(raw_frame[14] | (raw_frame[15] << 8));

    /* 摇杆绝对值超出旧工程经验范围时，认为当前帧已错位或损坏。
     * 这里直接清零该帧，保持下游控制逻辑看到的是“安全输入”。
     */
    if ((abs(frame->ch1) > IT_DBUS_CHANNEL_MAX_ABS) ||
        (abs(frame->ch2) > IT_DBUS_CHANNEL_MAX_ABS) ||
        (abs(frame->ch3) > IT_DBUS_CHANNEL_MAX_ABS) ||
        (abs(frame->ch4) > IT_DBUS_CHANNEL_MAX_ABS))
    {
        memset(frame, 0, sizeof(*frame));
        return OM_FALSE;
    }

    return OM_TRUE;
}

void input_task_rc_fill_snapshot(
    const InputTaskRcFrame* frame,
    InputRcSnapshot* snapshot)
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

void input_task_rc_commit_snapshot(const InputRcSnapshot* snapshot)
{
    if (snapshot == OM_NULL)
    {
        return;
    }

    g_input_task_rc_snapshot = *snapshot;
}

void input_task_rc_copy_snapshot(InputRcSnapshot* snapshot)
{
    if (snapshot == OM_NULL)
    {
        return;
    }

    *snapshot = g_input_task_rc_snapshot;
}

OmBool input_task_rc_update_online(
    InputTaskRcDebugState* runtime,
    OsalTimeMs now_ms)
{
    const uint8_t previous_online = g_input_task_rc_snapshot.online;

    if (runtime == OM_NULL)
    {
        return OM_FALSE;
    }

    if (runtime->last_frame_ms == 0u)
    {
        runtime->last_frame_age_ms = 0u;
        runtime->online = 0u;
        g_input_task_rc_snapshot.online = 0u;
        return (previous_online != g_input_task_rc_snapshot.online) ? OM_TRUE : OM_FALSE;
    }

    runtime->last_frame_age_ms = (uint32_t)(now_ms - runtime->last_frame_ms);
    runtime->online =
        (runtime->last_frame_age_ms <= IT_DBUS_FRAME_TIMEOUT_MS) ? 1u : 0u;
    g_input_task_rc_snapshot.online = runtime->online;
    return (previous_online != g_input_task_rc_snapshot.online) ? OM_TRUE : OM_FALSE;
}
