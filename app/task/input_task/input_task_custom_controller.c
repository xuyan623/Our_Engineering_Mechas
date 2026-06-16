#include "task/input_task/input_task_custom_controller.h"

#include "core/algorithm/protocol/crc.h"
#include <string.h>

typedef struct
{
    uint8_t work_mode;
    float angle_deg[IC_ANGLE_COUNT];
    uint8_t reserved[IC_PAYLOAD_LEN - sizeof(uint8_t) - sizeof(float) * IC_ANGLE_COUNT];
} __attribute__((__packed__)) InputCustomPayload;

static InputCustomSnapshot g_input_custom_snapshot = {0};

/* 只有整帧 CRC16 和 cmd_id 都通过后，才刷新 owner 持有的 latest-cache。
 * 这保证 arm_task / mode_task 看到的始终是一份“最近一次合法帧”快照。
 */

/**
 * @brief 处理并验证接收到的自定义控制器数据帧，将有效数据写入 latest-cache
 * 
 * 该函数对接收到的完整数据帧进行多层验证（空指针检查、帧长度检查、CRC16校验、命令ID匹配），
 * 只有通过所有验证的帧才会被解析并将其中的工作模式和角度数据更新到 owner latest-cache。
 * 同时更新解析器状态和运行时统计信息。
 * 
 * @param runtime 运行时调试状态指针，用于记录统计信息和序列号等
 * @param parser 解析器状态指针，用于更新最后接收帧的时间戳
 * @param frame 指向完整数据帧的指针，包含帧头、命令、载荷和CRC校验
 * @param frame_len 数据帧的长度，必须等于IC_FRAME_SIZE
 * @param now_ms 当前系统时间戳（毫秒），用于更新帧接收时间和计算帧龄
 * 
 * @return 无返回值，失败时直接返回，成功时更新数据池和统计信息
 * 
 * @note 函数采用快速失败策略，任何验证失败都会立即返回并递增相应的错误计数
 * @note 只有在CRC16校验和命令ID都通过的情况下，才会更新控制器正式快照
 */
static void input_custom_consume_frame(
    InputCustomDebugState* runtime,
    InputCustomParser* parser,
    const uint8_t frame[IC_FRAME_SIZE],
    uint16_t frame_len,
    OsalTimeMs now_ms)
{
    uint16_t cmd_id = 0u;
    InputCustomPayload payload = {0};
    InputCustomSnapshot snapshot = {0};
    uint32_t angle_index = 0u;

    /* 参数有效性检查：确保所有必需指针非空 */
    if (runtime == OM_NULL || parser == OM_NULL || frame == OM_NULL)
    {
        return;
    }

    /* 帧长度验证：确保接收到的帧长度与预期完全一致 */
    if (frame_len != IC_FRAME_SIZE)
    {
        return;
    }

    /* CRC16校验：验证整个数据帧的完整性，失败则递增错误计数 */
    if (verify_crc16_check_sum((uint8_t*)frame, frame_len) == 0u)
    {
        runtime->crc16_fail_count++;
        return;
    }

    /* 提取并验证命令ID：从帧中解析16位命令ID并与期望值比较 */
    cmd_id = (uint16_t)(frame[IC_HEADER_LEN] |
                        (frame[IC_HEADER_LEN + 1u]
                         << 8u));
    if (cmd_id != IC_CMD_ID)
    {
        runtime->cmd_mismatch_count++;
        return;
    }

    /* 解析载荷数据：从帧中提取工作模式和角度数组 */
    memcpy(
        &payload,
        &frame[IC_HEADER_LEN +
               IC_CMD_LEN],
        sizeof(payload));

    snapshot.online = 1u;
    snapshot.work_mode = payload.work_mode;
    for (angle_index = 0u; angle_index < IC_ANGLE_COUNT;
         angle_index++)
    {
        snapshot.angle_deg[angle_index] = payload.angle_deg[angle_index];
    }

    /* 标记控制器在线状态，更新解析器时间戳和运行时统计信息。 */
    g_input_custom_snapshot = snapshot;
    parser->last_frame_ms = now_ms;
    runtime->frame_count++;
    runtime->last_seq = frame[3];
    runtime->last_frame_age_ms = 0u;
}

void input_custom_reset_runtime(
    InputCustomDebugState* runtime)
{
    if (runtime == OM_NULL)
    {
        return;
    }

    memset((void*)runtime, 0, sizeof(*runtime));
}

void input_custom_reset_latest(void)
{
    InputCustomSnapshot snapshot = {0};
    uint32_t angle_index = 0u;

    /* 降级启动或控制器掉线时，统一把 owner latest-cache 收回到“离线 + 0 值”。
     * 这样其它任务不需要再区分“从没启动成功”和“运行时断开”。
     */
    for (angle_index = 0u; angle_index < IC_ANGLE_COUNT;
         angle_index++)
    {
        snapshot.angle_deg[angle_index] = 0.0f;
    }
    g_input_custom_snapshot = snapshot;
}

void input_custom_reset_parser(
    InputCustomParser* parser,
    uint8_t maybe_sof)
{
    if (parser == OM_NULL)
    {
        return;
    }

    memset(parser->buffer, 0, sizeof(parser->buffer));
    parser->index = 0u;
    parser->data_length = 0u;
    parser->expected_frame_len = 0u;
    parser->step = IC_STEP_WAIT_SOF;

    if (maybe_sof == IC_FRAME_SOF)
    {
        parser->buffer[0] = maybe_sof;
        parser->index = 1u;
        parser->step = IC_STEP_LEN_LO;
    }
}

/**
 * @brief 接收并解析自定义控制器的单个字节，实现状态机驱动的帧解析
 * 
 * 该函数按照协议规范逐步解析数据帧，支持以下阶段：
 * - 等待帧起始符(SOF)
 * - 解析数据长度(低字节和高字节)
 * - 解析序列号
 * - 验证帧头CRC8校验和
 * - 接收帧体和CRC16校验码
 * 
 * 当完整帧接收完毕后，自动调用consume_frame进行进一步处理
 * 
 * @param runtime 运行时调试状态指针，用于统计错误计数等信息，不能为NULL
 * @param parser 解析器状态指针，维护当前解析进度和缓冲区，不能为NULL
 * @param byte 待处理的输入字节
 * @param now_ms 当前时间戳(毫秒)，用于帧处理时的时间记录
 * @return OmRet 返回处理结果：
 *         - OM_OK: 字节处理成功，继续等待下一个字节
 *         - OM_ERROR_NULL: 参数为空指针
 *         - OM_ERROR_PARAM: 参数错误或帧长度不匹配
 *         - OM_ERROR: CRC8校验失败
 *         - OM_ERR_OVERFLOW: 缓冲区溢出
 */
OmRet input_custom_accept_byte(
    InputCustomDebugState* runtime,
    InputCustomParser* parser,
    uint8_t byte,
    OsalTimeMs now_ms)
{
    if (runtime == OM_NULL || parser == OM_NULL)
    {
        return OM_ERROR_NULL;
    }

    switch (parser->step)
    {
    case IC_STEP_WAIT_SOF:
        /* 只接受 0xA5 作为新帧起点；其它字节全部丢掉。 */
        if (byte == IC_FRAME_SOF)
        {
            parser->buffer[0] = byte;
            parser->index = 1u;
            parser->step = IC_STEP_LEN_LO;
        }
        return OM_OK;

    case IC_STEP_LEN_LO:
        parser->buffer[parser->index++] = byte;
        parser->data_length = byte;
        parser->step = IC_STEP_LEN_HI;
        return OM_OK;

    case IC_STEP_LEN_HI:
        parser->buffer[parser->index++] = byte;
        parser->data_length = (uint16_t)(parser->data_length | (byte << 8u));
        
        /* 验证数据长度是否符合预期，不符合则重置解析器 */
        if (parser->data_length != IC_PAYLOAD_LEN)
        {
            input_custom_reset_parser(parser, byte);
            return OM_ERROR_PARAM;
        }
        
        /* 计算期望的完整帧长度 */
        parser->expected_frame_len =
            (uint16_t)(IC_HEADER_LEN +
                       IC_CMD_LEN +
                       parser->data_length +
                       IC_CRC16_LEN);
        parser->step = IC_STEP_SEQ;
        return OM_OK;

    case IC_STEP_SEQ:
        parser->buffer[parser->index++] = byte;
        parser->step = IC_STEP_HEADER_CRC8;
        return OM_OK;

    case IC_STEP_HEADER_CRC8:
        parser->buffer[parser->index++] = byte;
        
        /* 验证帧头CRC8校验和，失败则丢弃当前帧并重新寻找SOF */
        if (verify_crc8_check_sum(
                parser->buffer, IC_HEADER_LEN) == 0u)
        {
            runtime->crc8_fail_count++;
            input_custom_reset_parser(parser, byte);
            return OM_ERROR;
        }
        parser->step = IC_STEP_BODY_CRC16;
        return OM_OK;

    case IC_STEP_BODY_CRC16:
        /* 检查缓冲区是否溢出 */
        if (parser->index >= IC_FRAME_SIZE)
        {
            input_custom_reset_parser(parser, byte);
            return OM_ERR_OVERFLOW;
        }

        parser->buffer[parser->index++] = byte;
        
        /* 如果还未收满整帧，继续等待后续字节 */
        if (parser->index < parser->expected_frame_len)
        {
            return OM_OK;
        }

        /* 到这里说明一帧字节数已经收满，剩下的合法性由 consume_frame 统一判断。 */
        input_custom_consume_frame(
            runtime, parser, parser->buffer, parser->expected_frame_len, now_ms);
        input_custom_reset_parser(parser, 0u);
        return OM_OK;

    default:
        input_custom_reset_parser(parser, byte);
        return OM_ERROR_PARAM;
    }
}

OmBool input_custom_update_online(
    InputCustomDebugState* runtime,
    const InputCustomParser* parser,
    OsalTimeMs now_ms)
{
    const uint8_t previous_online = g_input_custom_snapshot.online;

    if (runtime == OM_NULL || parser == OM_NULL)
    {
        return OM_FALSE;
    }

    if (parser->last_frame_ms == 0u)
    {
        g_input_custom_snapshot.online = 0u;
        runtime->last_frame_age_ms = 0u;
        return (previous_online != g_input_custom_snapshot.online) ? OM_TRUE : OM_FALSE;
    }

    /* 控制器 online 只由“最近是否收到合法帧”决定，
     * 不与 mode_task / arm_task 的更高层状态混用。
     */
    runtime->last_frame_age_ms = (uint32_t)(now_ms - parser->last_frame_ms);
    if (runtime->last_frame_age_ms >
        IC_FRAME_TIMEOUT_MS)
    {
        g_input_custom_snapshot.online = 0u;
    }
    else
    {
        g_input_custom_snapshot.online = 1u;
    }

    return (previous_online != g_input_custom_snapshot.online) ? OM_TRUE : OM_FALSE;
}

void input_custom_copy_snapshot(
    InputCustomSnapshot* snapshot)
{
    if (snapshot == OM_NULL)
    {
        return;
    }

    *snapshot = g_input_custom_snapshot;
}
