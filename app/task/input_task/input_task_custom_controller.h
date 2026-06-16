#ifndef NEW_ROBOT_IC_H
#define NEW_ROBOT_IC_H

#include "osal/osal_time.h"
#include "task/input_task/input_task.h"
#include "task/input_task/input_task_snapshot.h"
#include <stdint.h>

#define IC_FRAME_SOF        (0xA5u)
#define IC_CMD_ID           (0x0302u)
#define IC_PAYLOAD_LEN      (30u)
#define IC_HEADER_LEN       (5u)
#define IC_CMD_LEN          (2u)
#define IC_CRC16_LEN        (2u)
#define IC_FRAME_SIZE   (IC_HEADER_LEN + IC_CMD_LEN + IC_PAYLOAD_LEN + IC_CRC16_LEN)
#define IC_UART8_BAUD   (9600u)
#define IC_UART8_TX_BUF (128u)
#define IC_UART8_RX_BUF (256u)
#define IC_FRAME_TIMEOUT_MS (200u)

/* 自定义控制器字节流解析状态机：
 * 按帧头 -> 长度 -> 序号 -> CRC8 -> 数据与 CRC16 的顺序推进。
 */
typedef enum
{
    IC_STEP_WAIT_SOF = 0u,
    IC_STEP_LEN_LO,
    IC_STEP_LEN_HI,
    IC_STEP_SEQ,
    IC_STEP_HEADER_CRC8,
    IC_STEP_BODY_CRC16,
} InputCustomParseStep;

typedef struct
{
    uint8_t buffer[IC_FRAME_SIZE];
    uint16_t index;
    uint16_t data_length;
    uint16_t expected_frame_len;
    InputCustomParseStep step;
    OsalTimeMs last_frame_ms;
} InputCustomParser;

/* 只清自定义控制器这一源的 runtime 计数，不影响 DBUS / judge 槽位。 */
void input_custom_reset_runtime(
    InputCustomDebugState* runtime);
/* 当 UART8 降级或刚启动时，把 owner 持有的 latest-cache 重置到离线态。 */
void input_custom_reset_latest(void);
/* 解析器复位到等待帧头状态；若当前字节本身就是 SOF，则直接从长度阶段开始。 */
void input_custom_reset_parser(
    InputCustomParser* parser,
    uint8_t maybe_sof);
/* 向解析器喂一个字节，必要时更新 latest-cache 与 runtime 统计。 */
OmRet input_custom_accept_byte(
    InputCustomDebugState* runtime,
    InputCustomParser* parser,
    uint8_t byte,
    OsalTimeMs now_ms);
/* 根据最近一次成功帧的时间戳更新 online 状态；若 online 变化则返回 OM_TRUE。 */
OmBool input_custom_update_online(
    InputCustomDebugState* runtime,
    const InputCustomParser* parser,
    OsalTimeMs now_ms);
void input_custom_copy_snapshot(
    InputCustomSnapshot* snapshot);

#endif
