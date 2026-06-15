#ifndef NEW_ROBOT_INPUT_TASK_RC_H
#define NEW_ROBOT_INPUT_TASK_RC_H

#include "osal/osal_time.h"
#include "task/input_task/input_task.h"
#include "task/input_task/input_task_snapshot.h"
#include <stdint.h>

#define INPUT_TASK_DBUS_FRAME_LEN               (18u)
#define INPUT_TASK_DBUS_CHANNEL_CENTER          (1024)
#define INPUT_TASK_DBUS_CHANNEL_MAX_ABS         (660)
#define INPUT_TASK_DBUS_CHANNEL_DEADBAND        (5)
#define INPUT_TASK_DBUS_11BIT_MASK              (0x07FFu)
#define INPUT_TASK_DBUS_FRAME_TIMEOUT_MS        (200u)
#define INPUT_TASK_USART1_BAUDRATE              (100000u)
#define INPUT_TASK_USART1_TX_BUFSIZE            (128u)
#define INPUT_TASK_USART1_RX_BUFSIZE            (1024u)

/* 这份结构只服务于 input_task 内部：
 * - 用于承接一帧 DBUS 原始数据的解析结果
 * - 不向外暴露，对外正式发布的是 InputRcSnapshot
 */
typedef struct
{
    int16_t ch1;
    int16_t ch2;
    int16_t ch3;
    int16_t ch4;
    uint8_t sw1;
    uint8_t sw2;
    uint16_t iw;
    struct
    {
        int16_t x;
        int16_t y;
        int16_t z;
        uint8_t l;
        uint8_t r;
    } mouse;
    uint16_t keyboard_bits;
} InputTaskRcFrame;

/* 只清 DBUS 这一源自己的 runtime，不碰其它输入源。 */
void input_task_rc_reset_runtime(InputTaskRcDebugState* runtime);
/* 清 input_task 持有的 RC latest-cache，不影响其它输入源。 */
void input_task_rc_reset_latest_snapshot(void);
/* 按旧工程 DBUS 位布局把 18 字节整帧解成一份 RC 快照。 */
OmBool input_task_rc_decode_frame(const uint8_t raw_frame[INPUT_TASK_DBUS_FRAME_LEN], InputTaskRcFrame* frame);
void input_task_rc_fill_snapshot(const InputTaskRcFrame* frame, InputRcSnapshot* snapshot);
/* owner 在发布前先更新本地 latest-cache；正式跨任务分发仍由 input_task 完成。 */
void input_task_rc_commit_snapshot(const InputRcSnapshot* snapshot);
void input_task_rc_copy_snapshot(InputRcSnapshot* snapshot);
/* 根据最近一次帧时间更新 latest-cache.online；若 online 发生变化则返回 OM_TRUE。 */
OmBool input_task_rc_update_online_state(
    InputTaskRcDebugState* runtime,
    OsalTimeMs now_ms);

#endif
