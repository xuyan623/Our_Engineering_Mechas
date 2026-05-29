#ifndef NEW_ROBOT_INPUT_TASK_RC_H
#define NEW_ROBOT_INPUT_TASK_RC_H

#include "module/data_pool/data_pool.h"
#include "osal/osal_time.h"
#include "task/input_task/input_task.h"
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
 * - 不向外暴露，真正跨任务共享的数据仍然只进 DataPool.rc
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
/* 按旧工程 DBUS 位布局把 18 字节整帧解成一份 RC 快照。 */
OmBool input_task_rc_decode_frame(const uint8_t raw_frame[INPUT_TASK_DBUS_FRAME_LEN], InputTaskRcFrame* frame);
void input_task_rc_fill_snapshot(const InputTaskRcFrame* frame, DpRcSnapshot* snapshot);
/* 把解析后的“原始输入事实”写回共享池，不在这里做模式解释。 */
void input_task_rc_store_to_data_pool(const InputTaskRcFrame* frame);
void input_task_rc_update_online_state(
    InputTaskRcDebugState* runtime,
    OsalTimeMs now_ms);

#endif
