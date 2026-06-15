#ifndef NEW_ROBOT_INPUT_TASK_CUSTOM_CONTROLLER_H
#define NEW_ROBOT_INPUT_TASK_CUSTOM_CONTROLLER_H

#include "osal/osal_time.h"
#include "task/input_task/input_task.h"
#include "task/input_task/input_task_snapshot.h"
#include <stdint.h>

#define INPUT_TASK_CUSTOM_CONTROLLER_FRAME_SOF        (0xA5u)
#define INPUT_TASK_CUSTOM_CONTROLLER_CMD_ID           (0x0302u)
#define INPUT_TASK_CUSTOM_CONTROLLER_PAYLOAD_LEN      (30u)
#define INPUT_TASK_CUSTOM_CONTROLLER_HEADER_LEN       (5u)
#define INPUT_TASK_CUSTOM_CONTROLLER_CMD_LEN          (2u)
#define INPUT_TASK_CUSTOM_CONTROLLER_CRC16_LEN        (2u)
#define INPUT_TASK_CUSTOM_CONTROLLER_FRAME_MAX_SIZE   (INPUT_TASK_CUSTOM_CONTROLLER_HEADER_LEN + INPUT_TASK_CUSTOM_CONTROLLER_CMD_LEN + INPUT_TASK_CUSTOM_CONTROLLER_PAYLOAD_LEN + INPUT_TASK_CUSTOM_CONTROLLER_CRC16_LEN)
#define INPUT_TASK_CUSTOM_CONTROLLER_UART8_BAUDRATE   (9600u)
#define INPUT_TASK_CUSTOM_CONTROLLER_UART8_TX_BUFSIZE (128u)
#define INPUT_TASK_CUSTOM_CONTROLLER_UART8_RX_BUFSIZE (256u)
#define INPUT_TASK_CUSTOM_CONTROLLER_FRAME_TIMEOUT_MS (200u)

/* 自定义控制器字节流解析状态机：
 * 按帧头 -> 长度 -> 序号 -> CRC8 -> 数据与 CRC16 的顺序推进。
 */
typedef enum
{
    INPUT_TASK_CUSTOM_CONTROLLER_STEP_WAIT_SOF = 0u,
    INPUT_TASK_CUSTOM_CONTROLLER_STEP_LENGTH_LOW,
    INPUT_TASK_CUSTOM_CONTROLLER_STEP_LENGTH_HIGH,
    INPUT_TASK_CUSTOM_CONTROLLER_STEP_SEQ,
    INPUT_TASK_CUSTOM_CONTROLLER_STEP_HEADER_CRC8,
    INPUT_TASK_CUSTOM_CONTROLLER_STEP_BODY_AND_CRC16,
} InputTaskCustomControllerParseStep;

typedef struct
{
    uint8_t buffer[INPUT_TASK_CUSTOM_CONTROLLER_FRAME_MAX_SIZE];
    uint16_t index;
    uint16_t data_length;
    uint16_t expected_frame_len;
    InputTaskCustomControllerParseStep step;
    OsalTimeMs last_frame_ms;
} InputTaskCustomControllerParser;

/* 只清自定义控制器这一源的 runtime 计数，不影响 DBUS / judge 槽位。 */
void input_task_custom_controller_reset_runtime(
    InputTaskCustomControllerDebugState* runtime);
/* 当 UART8 降级或刚启动时，把 owner 持有的 latest-cache 重置到离线态。 */
void input_task_custom_controller_reset_latest_snapshot(void);
/* 解析器复位到等待帧头状态；若当前字节本身就是 SOF，则直接从长度阶段开始。 */
void input_task_custom_controller_reset_parser(
    InputTaskCustomControllerParser* parser,
    uint8_t maybe_sof);
/* 向解析器喂一个字节，必要时更新 latest-cache 与 runtime 统计。 */
OmRet input_task_custom_controller_accept_byte(
    InputTaskCustomControllerDebugState* runtime,
    InputTaskCustomControllerParser* parser,
    uint8_t byte,
    OsalTimeMs now_ms);
/* 根据最近一次成功帧的时间戳更新 online 状态；若 online 变化则返回 OM_TRUE。 */
OmBool input_task_custom_controller_update_online_state(
    InputTaskCustomControllerDebugState* runtime,
    const InputTaskCustomControllerParser* parser,
    OsalTimeMs now_ms);
void input_task_custom_controller_copy_snapshot(
    InputCustomControllerSnapshot* snapshot);

#endif
