#include "task/input_task/input_task.h"

#include "core/om_cpu.h"
#include "drivers/model/device.h"
#include "drivers/peripheral/serial/pal_serial_dev.h"
#include "module/data_pool/data_pool.h"
#include "module/event_bus/event_bus.h"
#include "module/system_health/system_health.h"
#include "osal/osal.h"
#include "osal/osal_config.h"
#include "osal/osal_time.h"
#include <stdlib.h>
#include <string.h>

#define INPUT_TASK_PERIOD_MS         (1u)
#define DBUS_FRAME_LEN               (18u)
#define DBUS_CHANNEL_CENTER          (1024)
#define DBUS_CHANNEL_MAX_ABS         (660)
#define DBUS_CHANNEL_DEADBAND        (5)
#define DBUS_11BIT_MASK              (0x07FFu)
#define INPUT_TASK_USART1_BAUDRATE   (100000u)
#define INPUT_TASK_USART1_TX_BUFSIZE (128u)
#define INPUT_TASK_USART1_RX_BUFSIZE (1024u)
#define INPUT_TASK_STREAM_BUF_SIZE   (128u)

/* 这份结构只服务于 input_task 内部�?
 * - 用于承接一�?DBUS 原始数据的解析结�?
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

InputTaskDebugState g_input_task_runtime = {0};

static void input_task_read_callback(Device* dev, void* param, size_t paramsz);

static OmRet input_task_prepare_usart1(Device* usart1_device)
{
    SerialCfg serial_cfg = SERIAL_DEFAULT_CFG;
    HalSerial* hal_serial = OM_NULL;

    if (usart1_device == OM_NULL)
    {
        return OM_ERROR_NULL;
    }

    serial_cfg.baudrate = INPUT_TASK_USART1_BAUDRATE;
    serial_cfg.parity = PARITY_EVEN;
    serial_cfg.txBufSize = INPUT_TASK_USART1_TX_BUFSIZE;
    serial_cfg.rxBufSize = INPUT_TASK_USART1_RX_BUFSIZE;

    hal_serial = (HalSerial*)usart1_device;
    hal_serial->cfg = serial_cfg;

    device_set_param(usart1_device, &g_input_task_runtime);
    device_set_read_cb(usart1_device, input_task_read_callback);

    return device_open(usart1_device, SERIAL_O_NBLCK_RX);
}

static void input_task_read_callback(Device* dev, void* param, size_t paramsz)
{
    InputTaskDebugState* runtime = (InputTaskDebugState*)param;

    (void)dev;

    if (runtime == OM_NULL)
    {
        return;
    }

    runtime->rx_available_hint = (uint32_t)paramsz;
}

static int16_t input_task_apply_deadband(int16_t value)
{
    if (value <= DBUS_CHANNEL_DEADBAND && value >= -DBUS_CHANNEL_DEADBAND)
    {
        return 0;
    }

    return value;
}

static void input_task_consume_stream(uint8_t* stream_buf, size_t* stream_len, size_t consume_len)
{
    if (stream_buf == OM_NULL || stream_len == OM_NULL || consume_len == 0u || *stream_len == 0u)
    {
        return;
    }

    if (consume_len >= *stream_len)
    {
        *stream_len = 0u;
        return;
    }

    memmove(stream_buf, stream_buf + consume_len, *stream_len - consume_len);
    *stream_len -= consume_len;
}

static OmBool input_task_decode_frame(const uint8_t raw_frame[DBUS_FRAME_LEN], InputTaskRcFrame* frame)
{
    if (raw_frame == OM_NULL || frame == OM_NULL)
    {
        return OM_FALSE;
    }

    memset(frame, 0, sizeof(*frame));

    /* DBUS �?4 个摇杆通道都是 11 bit 压缩编码�?
     * 这里完全按旧工程位布局展开，不引入新的协议解释�?
     */
    frame->ch1 = (int16_t)(((raw_frame[0] | (raw_frame[1] << 8)) & DBUS_11BIT_MASK) - DBUS_CHANNEL_CENTER);
    frame->ch2 = (int16_t)((((raw_frame[1] >> 3) | (raw_frame[2] << 5)) & DBUS_11BIT_MASK) - DBUS_CHANNEL_CENTER);
    frame->ch3 =
        (int16_t)((((raw_frame[2] >> 6) | (raw_frame[3] << 2) | (raw_frame[4] << 10)) & DBUS_11BIT_MASK) - DBUS_CHANNEL_CENTER);
    frame->ch4 = (int16_t)((((raw_frame[4] >> 1) | (raw_frame[5] << 7)) & DBUS_11BIT_MASK) - DBUS_CHANNEL_CENTER);

    frame->ch1 = input_task_apply_deadband(frame->ch1);
    frame->ch2 = input_task_apply_deadband(frame->ch2);
    frame->ch3 = input_task_apply_deadband(frame->ch3);
    frame->ch4 = input_task_apply_deadband(frame->ch4);

    frame->sw1 = (uint8_t)(((raw_frame[5] >> 4) & 0x0Cu) >> 2);
    frame->sw2 = (uint8_t)((raw_frame[5] >> 4) & 0x03u);
    frame->iw = (uint16_t)((raw_frame[16] | (raw_frame[17] << 8)) & DBUS_11BIT_MASK);

    frame->mouse.x = (int16_t)(raw_frame[6] | (raw_frame[7] << 8));
    frame->mouse.y = (int16_t)(raw_frame[8] | (raw_frame[9] << 8));
    frame->mouse.z = (int16_t)(raw_frame[10] | (raw_frame[11] << 8));
    frame->mouse.l = raw_frame[12];
    frame->mouse.r = raw_frame[13];
    frame->keyboard_bits = (uint16_t)(raw_frame[14] | (raw_frame[15] << 8));

    /* 摇杆绝对值超出旧工程经验范围时，认为当前帧已错位或损坏�?
     * 这里直接清零该帧，保持下游控制逻辑看到的是“安全输入”�?
     */
    if ((abs(frame->ch1) > DBUS_CHANNEL_MAX_ABS) || (abs(frame->ch2) > DBUS_CHANNEL_MAX_ABS) ||
        (abs(frame->ch3) > DBUS_CHANNEL_MAX_ABS) || (abs(frame->ch4) > DBUS_CHANNEL_MAX_ABS))
    {
        memset(frame, 0, sizeof(*frame));
        return OM_FALSE;
    }

    return OM_TRUE;
}

static void input_task_store_to_data_pool(const InputTaskRcFrame* frame)
{
    if (frame == OM_NULL)
    {
        return;
    }

    /* input_task 只负责把“原始输入事实”写入共享池�?
     * 不在这里做边沿历史、模式判断、动作推进�?
     */
    DP_STORE_INT16(&g_data_pool.rc.ch1, frame->ch1);
    DP_STORE_INT16(&g_data_pool.rc.ch2, frame->ch2);
    DP_STORE_INT16(&g_data_pool.rc.ch3, frame->ch3);
    DP_STORE_INT16(&g_data_pool.rc.ch4, frame->ch4);

    DP_STORE_UINT8(&g_data_pool.rc.sw1, frame->sw1);
    DP_STORE_UINT8(&g_data_pool.rc.sw2, frame->sw2);
    DP_STORE_UINT16(&g_data_pool.rc.iw, frame->iw);

    DP_STORE_INT16(&g_data_pool.rc.mouse.x, frame->mouse.x);
    DP_STORE_INT16(&g_data_pool.rc.mouse.y, frame->mouse.y);
    DP_STORE_INT16(&g_data_pool.rc.mouse.z, frame->mouse.z);
    DP_STORE_UINT8(&g_data_pool.rc.mouse.l, frame->mouse.l);
    DP_STORE_UINT8(&g_data_pool.rc.mouse.r, frame->mouse.r);

    DP_STORE_UINT16(&g_data_pool.rc.keyboard_bits, frame->keyboard_bits);
}

static OmBool input_task_try_parse_stream(uint8_t* stream_buf, size_t* stream_len, InputTaskRcFrame* parsed_frame)
{
    if (stream_buf == OM_NULL || stream_len == OM_NULL || parsed_frame == OM_NULL)
    {
        return OM_FALSE;
    }

    while (*stream_len >= DBUS_FRAME_LEN)
    {
        if (input_task_decode_frame(stream_buf, parsed_frame) == OM_TRUE)
        {
            input_task_consume_stream(stream_buf, stream_len, DBUS_FRAME_LEN);
            return OM_TRUE;
        }

        g_input_task_runtime.invalid_frame_count++;
        g_input_task_runtime.resync_drop_count++;
        input_task_consume_stream(stream_buf, stream_len, 1u);
    }

    return OM_FALSE;
}

static void input_task_entry(void* arg)
{
    const BspDeviceRegistry* devices = (const BspDeviceRegistry*)arg;
    OsalTimeMs deadline_cursor_ms = 0u;
    uint8_t stream_buf[INPUT_TASK_STREAM_BUF_SIZE] = {0};
    InputTaskRcFrame parsed_frame = {0};
    size_t read_len = 0u;
    size_t request_len = 0u;
    size_t stream_len = 0u;
    OmBool has_valid_frame = OM_FALSE;

    while (1)
    {
        has_valid_frame = OM_FALSE;

        if (g_input_task_runtime.rx_available_hint > 0u)
        {
            request_len = (size_t)g_input_task_runtime.rx_available_hint;
            if (request_len > (INPUT_TASK_STREAM_BUF_SIZE - stream_len))
            {
                request_len = INPUT_TASK_STREAM_BUF_SIZE - stream_len;
            }

            if (request_len > 0u)
            {
                read_len = device_read(devices->usart1, 0, stream_buf + stream_len, request_len);
                if (read_len > 0u)
                {
                    stream_len += read_len;
                }
            }

            g_input_task_runtime.rx_available_hint = 0u;
        }

        while (input_task_try_parse_stream(stream_buf, &stream_len, &parsed_frame) == OM_TRUE)
        {
            input_task_store_to_data_pool(&parsed_frame);
            g_input_task_runtime.frame_count++;
            has_valid_frame = OM_TRUE;
        }

        if (stream_len == INPUT_TASK_STREAM_BUF_SIZE)
        {
            /* һֱû����ɹ�ʱ������ 1 �ֽڣ����������������ÿ��ڴ�����λ�� */
            g_input_task_runtime.resync_drop_count++;
            input_task_consume_stream(stream_buf, &stream_len, 1u);
        }

        if (has_valid_frame == OM_TRUE)
        {
            if (event_bus_publish(&g_event_bus, EVT_RC_DATA_READY) != OSAL_OK)
            {
                sh_report_fatal(SH_ERR_EVT_RC_DATA_READY_PUBLISH_FAIL, "event_bus_publish EVT_RC_DATA_READY failed");
                for (;;)
                {
                    osal_sleep_ms(1000U);
                }
            }
        }

        (void)osal_delay_until(&deadline_cursor_ms, INPUT_TASK_PERIOD_MS, OM_NULL);
    }
}
OmRet input_task_start(const BspDeviceRegistry* devices)
{
    static OsalThread* input_task_thread = OM_NULL;
    const OsalThreadAttr input_task_attr = {"input_task", 512u * OSAL_STACK_WORD_BYTES, 4u};
    OsalStatus status = OSAL_INVALID;

    if (devices == OM_NULL || devices->usart1 == OM_NULL)
    {
        return OM_ERROR_NULL;
    }

    if (input_task_thread != OM_NULL)
    {
        return OM_ERR_CONFLICT;
    }

    memset((void*)&g_input_task_runtime, 0, sizeof(g_input_task_runtime));

    /* USART1 的读回调只用于“唤醒提示”，不在回调里直接读帧�?
     * 这里�?input_task 自己拥有并初始化串口，而不是依�?BSP 预开设备�?
     */
    if (input_task_prepare_usart1(devices->usart1) != OM_OK)
    {
        return OM_ERROR;
    }

    status = osal_thread_create(&input_task_thread, &input_task_attr, input_task_entry, (void*)devices);
    if (status != OSAL_OK)
    {
        input_task_thread = OM_NULL;
        return OM_ERROR;
    }

    return OM_OK;
}
