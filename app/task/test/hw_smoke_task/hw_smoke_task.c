#include "task/test/hw_smoke_task/hw_smoke_task.h"

#include "drivers/peripheral/can/pal_can_dev.h"
#include "drivers/peripheral/serial/pal_serial_dev.h"
#include "function/vofa/vofa.h"
#include "osal/osal.h"
#include "osal/osal_config.h"
#include <string.h>

#define HW_SMOKE_TASK_PERIOD_MS       (100u)
#define HW_SMOKE_HEARTBEAT_PERIOD_MS  (500u)
#define HW_SMOKE_RX_BUFFER_SIZE       (32u)
#define HW_SMOKE_CAN_TEST_DLC         (8u)
#define HW_SMOKE_CAN1_TEST_ID         (0x321u)
#define HW_SMOKE_CAN2_TEST_ID         (0x322u)
#define HW_SMOKE_USART3_BAUDRATE      (115200u)
#define HW_SMOKE_USART6_BAUDRATE      (4000000u)
#define HW_SMOKE_UART7_BAUDRATE       (115200u)
#define HW_SMOKE_UART8_BAUDRATE       (921600u)

typedef struct
{
    const BspDeviceRegistry* devices;
    uint8_t imu_init_ret;
    CanFilterHandle handles[HW_SMOKE_CAN_COUNT];
    uint8_t rx_storage[HW_SMOKE_CAN_COUNT][HW_SMOKE_CAN_TEST_DLC];
    CanUserMsg rx_msgs[HW_SMOKE_CAN_COUNT];
} HwSmokeRuntime;

typedef struct
{
    HwSmokeSerialId serial_id;
    volatile uint8_t rx_pending;
} HwSmokeSerialRuntime;

static HwSmokeRuntime g_hw_smoke_runtime = {0};
static HwSmokeSerialRuntime g_hw_smoke_serial_runtime[HW_SMOKE_SERIAL_COUNT] = {
    {.serial_id = HW_SMOKE_SERIAL_USART1, .rx_pending = 0u},
    {.serial_id = HW_SMOKE_SERIAL_USART3, .rx_pending = 0u},
    {.serial_id = HW_SMOKE_SERIAL_USART6, .rx_pending = 0u},
    {.serial_id = HW_SMOKE_SERIAL_UART7, .rx_pending = 0u},
    {.serial_id = HW_SMOKE_SERIAL_UART8, .rx_pending = 0u},
};
HwSmokeSnapshot g_hw_smoke_snapshot = {0};

static void hw_smoke_serial_read_callback(Device* dev, void* param, size_t paramsz)
{
    HwSmokeSerialRuntime* runtime = (HwSmokeSerialRuntime*)param;

    (void)dev;
    (void)paramsz;

    if (runtime == 0)
    {
        return;
    }

    runtime->rx_pending = 1u;
}

static void hw_smoke_can_callback(Device* dev, void* param, CanFilterHandle filter_handle, size_t msg_count)
{
    HwSmokeCanId can_id = (HwSmokeCanId)(uintptr_t)param;
    CanUserMsg rx_msg = {0};
    uint32_t expected_id = 0u;
    size_t read_count = 0u;

    (void)filter_handle;

    rx_msg.filterHandle = g_hw_smoke_runtime.handles[can_id];
    rx_msg.userBuf = g_hw_smoke_runtime.rx_storage[can_id];
    read_count = device_read(dev, 0, &rx_msg, 1u);
    if (read_count == 0u)
    {
        g_hw_smoke_snapshot.last_error_code = 0xE200u + (uint32_t)can_id;
        return;
    }

    expected_id = (can_id == HW_SMOKE_CAN1) ? HW_SMOKE_CAN1_TEST_ID : HW_SMOKE_CAN2_TEST_ID;
    g_hw_smoke_snapshot.cans[can_id].rx_count++;
    g_hw_smoke_snapshot.cans[can_id].last_rx_seq = g_hw_smoke_runtime.rx_storage[can_id][1];
    g_hw_smoke_snapshot.cans[can_id].last_payload_ok =
        (rx_msg.dsc.id == expected_id && g_hw_smoke_runtime.rx_storage[can_id][0] == expected_id &&
         g_hw_smoke_runtime.rx_storage[can_id][7] == (uint8_t)(g_hw_smoke_runtime.rx_storage[can_id][1] ^ 0xA5u))
            ? OM_TRUE
            : OM_FALSE;
}

static OmRet hw_smoke_prepare_serial_device(
    Device* serial_dev,
    uint32_t baudrate,
    uint32_t open_flags)
{
    SerialCfg serial_cfg = SERIAL_DEFAULT_CFG;
    HalSerial* hal_serial = OM_NULL;

    if (serial_dev == OM_NULL)
    {
        return OM_ERROR_NULL;
    }

    serial_cfg.baudrate = baudrate;
    serial_cfg.txBufSize = 1024u;
    serial_cfg.rxBufSize = 128u;

    hal_serial = (HalSerial*)serial_dev;
    hal_serial->cfg = serial_cfg;

    return device_open(serial_dev, open_flags);
}

static OmRet hw_smoke_configure_loopback_can(Device* can_dev, HwSmokeCanId can_id, uint32_t can_id_value)
{
    CanCfg can_cfg = CAN_DEFUALT_CFG;
    CanFilterAllocArg alloc_arg = {
        .request = CAN_FILTER_REQUEST_INIT(CAN_FILTER_MODE_MASK, CAN_FILTER_ID_STD, can_id_value, 0x7FFu,
                                           hw_smoke_can_callback, (void*)(uintptr_t)can_id),
    };
    OmRet ret = OM_OK;
    uint32_t io_type = 0u;

    if (!device_check_status(can_dev, DEV_STATUS_OPENED))
    {
        ret = device_open(can_dev, CAN_O_INT_RX | CAN_O_INT_TX);
        if (ret != OM_OK)
        {
            return ret;
        }
    }

    can_cfg.workMode = CAN_WORK_LOOPBACK;
    can_cfg.normalTimeCfg.baudRate = CAN_BAUD_500K;

    ret = device_ctrl(can_dev, CAN_CMD_CFG, &can_cfg);
    g_hw_smoke_snapshot.cans[can_id].cfg_ret = ret;
    if (ret != OM_OK)
    {
        g_hw_smoke_snapshot.last_error_code = 0xC100u + (uint32_t)can_id;
        return ret;
    }

    ret = device_ctrl(can_dev, CAN_CMD_FILTER_ALLOC, &alloc_arg);
    g_hw_smoke_snapshot.cans[can_id].filter_ret = ret;
    if (ret != OM_OK)
    {
        g_hw_smoke_snapshot.last_error_code = 0xC200u + (uint32_t)can_id;
        return ret;
    }

    io_type = CAN_REG_INT_RX;
    ret = device_ctrl(can_dev, CAN_CMD_SET_IOTYPE, &io_type);
    if (ret != OM_OK)
    {
        g_hw_smoke_snapshot.last_error_code = 0xC250u + (uint32_t)can_id;
        return ret;
    }

    io_type = CAN_REG_INT_TX;
    ret = device_ctrl(can_dev, CAN_CMD_SET_IOTYPE, &io_type);
    if (ret != OM_OK)
    {
        g_hw_smoke_snapshot.last_error_code = 0xC260u + (uint32_t)can_id;
        return ret;
    }

    g_hw_smoke_runtime.handles[can_id] = alloc_arg.handle;
    g_hw_smoke_runtime.rx_msgs[can_id].filterHandle = alloc_arg.handle;
    g_hw_smoke_runtime.rx_msgs[can_id].userBuf = g_hw_smoke_runtime.rx_storage[can_id];

    /* CAN_CMD_CFG 只重新装载配置，不会替代启动动作。
     * 这里必须按框架样例显式 START，否则环回帧不会真正进收发通路，
     * 最终表现就是 tx 软 FIFO 越积越多，rx_count 一直为 0。
     */
    ret = device_ctrl(can_dev, CAN_CMD_START, 0);
    if (ret != OM_OK)
    {
        g_hw_smoke_snapshot.last_error_code = 0xC400u + (uint32_t)can_id;
        return ret;
    }

    return OM_OK;
}

static void hw_smoke_send_can_frame(Device* can_dev, HwSmokeCanId can_id, uint32_t can_id_value)
{
    CanUserMsg tx_msg = {0};
    uint8_t payload[HW_SMOKE_CAN_TEST_DLC] = {0};
    size_t write_count = 0u;

    g_hw_smoke_snapshot.cans[can_id].last_tx_seq++;
    payload[0] = (uint8_t)can_id_value;
    payload[1] = (uint8_t)g_hw_smoke_snapshot.cans[can_id].last_tx_seq;
    payload[2] = 0x11u;
    payload[3] = 0x22u;
    payload[4] = 0x33u;
    payload[5] = 0x44u;
    payload[6] = 0x55u;
    payload[7] = (uint8_t)(payload[1] ^ 0xA5u);

    tx_msg.dsc = CAN_DATA_MSG_DSC_INIT(can_id_value, CAN_IDE_STD, HW_SMOKE_CAN_TEST_DLC);
    tx_msg.filterHandle = 0u;
    tx_msg.userBuf = payload;
    write_count = device_write(can_dev, 0, &tx_msg, 1u);
    if (write_count == 1u)
    {
        g_hw_smoke_snapshot.cans[can_id].tx_count++;
    }
    else
    {
        g_hw_smoke_snapshot.last_error_code = 0xC300u + (uint32_t)can_id;
    }
}

static void hw_smoke_send_serial_heartbeat(Device* serial_dev, HwSmokeSerialId serial_id)
{
    static const char* heartbeat_names[HW_SMOKE_SERIAL_COUNT] = {"usart1", "usart3", "usart6", "uart7", "uart8"};
    char payload[32] = {0};
    size_t length = 0u;
    size_t write_count = 0u;
    size_t name_index = 0u;

    if (serial_dev == 0 || serial_id == HW_SMOKE_SERIAL_USART1 || serial_id == HW_SMOKE_SERIAL_UART7)
    {
        return;
    }

    for (name_index = 0u; heartbeat_names[serial_id][name_index] != '\0'; name_index++)
    {
        payload[name_index] = heartbeat_names[serial_id][name_index];
    }
    payload[name_index++] = ' ';
    payload[name_index++] = 's';
    payload[name_index++] = 'm';
    payload[name_index++] = 'o';
    payload[name_index++] = 'k';
    payload[name_index++] = 'e';
    payload[name_index++] = '\r';
    payload[name_index++] = '\n';
    length = name_index;

    write_count = device_write(serial_dev, 0, payload, length);
    if (write_count == length)
    {
        g_hw_smoke_snapshot.serials[serial_id].tx_count += (uint32_t)write_count;
    }
    else
    {
        g_hw_smoke_snapshot.serials[serial_id].write_fail_count++;
        g_hw_smoke_snapshot.last_error_code = 0xD100u + (uint32_t)serial_id;
    }
}

static void hw_smoke_poll_serial_rx(Device* serial_dev, HwSmokeSerialId serial_id)
{
    uint8_t rx_buffer[HW_SMOKE_RX_BUFFER_SIZE] = {0};
    static const uint8_t ok_reply[] = {'o', 'k', '\r', '\n'};
    size_t read_count = 0u;
    size_t total_read_count = 0u;
    size_t write_count = 0u;

    if (serial_dev == 0 || serial_id == HW_SMOKE_SERIAL_USART1 || serial_id == HW_SMOKE_SERIAL_UART7)
    {
        return;
    }

    if (g_hw_smoke_serial_runtime[serial_id].rx_pending == 0u)
    {
        return;
    }

    /* 当前串口 PAL 的非阻塞读语义是“len 个字节全部满足才返回”，
     * 因此这里按 1 字节轮询抽空，才能满足“上位机发任意内容都回 ok”。
     */
    do
    {
        read_count = device_read(serial_dev, 0, rx_buffer, 1u);
        total_read_count += read_count;
    } while (read_count == 1u && total_read_count < HW_SMOKE_RX_BUFFER_SIZE);

    if (total_read_count == 0u)
    {
        return;
    }

    g_hw_smoke_serial_runtime[serial_id].rx_pending = 0u;
    g_hw_smoke_snapshot.serials[serial_id].rx_count += (uint32_t)total_read_count;
    write_count = device_write(serial_dev, 0, (void*)ok_reply, sizeof(ok_reply));
    if (write_count == sizeof(ok_reply))
    {
        g_hw_smoke_snapshot.serials[serial_id].tx_count += (uint32_t)write_count;
        g_hw_smoke_snapshot.serials[serial_id].ok_reply_count++;
    }
    else
    {
        g_hw_smoke_snapshot.serials[serial_id].write_fail_count++;
        g_hw_smoke_snapshot.last_error_code = 0xD200u + (uint32_t)serial_id;
    }
}

static void hw_smoke_fill_vofa_frame(float frame[HW_SMOKE_VOFA_CHANNEL_COUNT])
{
    uint32_t index = 0u;

    /* 先清零所有通道。
     * 这样如果你手动注释掉下面某一行赋值，该通道会稳定显示为 0，
     * 便于通过排除法确认上位机曲线对应的具体数据。
     */
    for (index = 0u; index < HW_SMOKE_VOFA_CHANNEL_COUNT; index++)
    {
        frame[index] = 0.0f;
    }

    /* CH1: IMU 初始化返回值，正常应为 0。 */
    //frame[0] = (float)g_hw_smoke_snapshot.imu_init_ret;

    /* CH2: IMU 更新计数，每个周期递增。 */
    //frame[1] = (float)g_hw_smoke_snapshot.imu_update_count;

    /* CH3: IMU 温度。 */
    //frame[2] = g_hw_smoke_snapshot.last_imu.temp;

    /* CH4: Roll 角。 */
    //frame[3] = g_hw_smoke_snapshot.last_imu.rol;

    /* CH5: Pitch 角。 */
    //frame[4] = g_hw_smoke_snapshot.last_imu.pit;

    /* CH6: Yaw 角。 */
    //frame[5] = g_hw_smoke_snapshot.last_imu.yaw;

    /* CH7: Roll 角速度。 */
    //frame[6] = g_hw_smoke_snapshot.last_imu.rol_rate;

    /* CH8: Pitch 角速度。 */
    //frame[7] = g_hw_smoke_snapshot.last_imu.pit_rate;

    /* CH9: Yaw 角速度。 */
    //frame[8] = g_hw_smoke_snapshot.last_imu.yaw_rate;

    /* CH10: CAN1 发送计数。 */
    frame[9] = (float)g_hw_smoke_snapshot.cans[HW_SMOKE_CAN1].tx_count;

    /* CH11: CAN1 接收计数。 */
    frame[10] = (float)g_hw_smoke_snapshot.cans[HW_SMOKE_CAN1].rx_count;

    /* CH12: CAN1 最近一次 payload 校验是否成功，0/1。 */
    frame[11] = (g_hw_smoke_snapshot.cans[HW_SMOKE_CAN1].last_payload_ok == OM_TRUE) ? 1.0f : 0.0f;

    /* CH13: CAN2 发送计数。 */
    frame[12] = (float)g_hw_smoke_snapshot.cans[HW_SMOKE_CAN2].tx_count;

    /* CH14: CAN2 接收计数。 */
    frame[13] = (float)g_hw_smoke_snapshot.cans[HW_SMOKE_CAN2].rx_count;

    /* CH15: CAN2 最近一次 payload 校验是否成功，0/1。 */
    frame[14] = (g_hw_smoke_snapshot.cans[HW_SMOKE_CAN2].last_payload_ok == OM_TRUE) ? 1.0f : 0.0f;

    /* CH16: 最近一次错误码，正常应为 0。 */
    frame[15] = (float)g_hw_smoke_snapshot.last_error_code;
}

static void hw_smoke_task_entry(void* arg)
{
    uint32_t heartbeat_divider = 0u;
    float vofa_frame[HW_SMOKE_VOFA_CHANNEL_COUNT] = {0.0f};
    imu_data_t* imu_data = 0;
    const BspDeviceRegistry* devices = (const BspDeviceRegistry*)arg;

    while (1)
    {
        mpu_get_data();
        update_attitude(-1.0f);
        imu_data = get_imu_data();
        if (imu_data != 0)
        {
            g_hw_smoke_snapshot.last_imu = *imu_data;
            g_hw_smoke_snapshot.imu_update_count++;
        }

        hw_smoke_poll_serial_rx(devices->usart3, HW_SMOKE_SERIAL_USART3);
        hw_smoke_poll_serial_rx(devices->usart6, HW_SMOKE_SERIAL_USART6);
        hw_smoke_poll_serial_rx(devices->uart8, HW_SMOKE_SERIAL_UART8);

        hw_smoke_send_can_frame(devices->can1, HW_SMOKE_CAN1, HW_SMOKE_CAN1_TEST_ID);
        hw_smoke_send_can_frame(devices->can2, HW_SMOKE_CAN2, HW_SMOKE_CAN2_TEST_ID);

        heartbeat_divider++;
        if ((heartbeat_divider % (HW_SMOKE_HEARTBEAT_PERIOD_MS / HW_SMOKE_TASK_PERIOD_MS)) == 0u)
        {
            hw_smoke_send_serial_heartbeat(devices->usart3, HW_SMOKE_SERIAL_USART3);
            hw_smoke_send_serial_heartbeat(devices->usart6, HW_SMOKE_SERIAL_USART6);
            hw_smoke_send_serial_heartbeat(devices->uart8, HW_SMOKE_SERIAL_UART8);
        }

        hw_smoke_fill_vofa_frame(vofa_frame);
        vofa_justfloat_send(devices->uart7, vofa_frame, HW_SMOKE_VOFA_CHANNEL_COUNT);
        osal_sleep_ms(HW_SMOKE_TASK_PERIOD_MS);
    }
}

OmRet hw_smoke_task_start(const BspDeviceRegistry* devices, uint8_t imu_init_ret)
{
    static OsalThread* smoke_task = 0;
    const OsalThreadAttr smoke_attr = {"hw_smoke_task", 1024u * OSAL_STACK_WORD_BYTES, 4u};
    OmRet can_ret = OM_OK;
    OsalStatus thread_ret = OSAL_INVALID;
    size_t serial_index = 0u;

    if (devices == 0)
    {
        return OM_ERROR_NULL;
    }

    memset(&g_hw_smoke_snapshot, 0, sizeof(g_hw_smoke_snapshot));
    g_hw_smoke_runtime.devices = devices;
    g_hw_smoke_runtime.imu_init_ret = imu_init_ret;
    g_hw_smoke_snapshot.bsp_init_ok = OM_TRUE;
    g_hw_smoke_snapshot.imu_init_ret = imu_init_ret;

    g_hw_smoke_snapshot.serials[HW_SMOKE_SERIAL_USART1].open_ret = (devices->usart1 != 0) ? OM_OK : OM_ERROR_NULL;
    g_hw_smoke_snapshot.serials[HW_SMOKE_SERIAL_USART3].open_ret =
        hw_smoke_prepare_serial_device(
            devices->usart3,
            HW_SMOKE_USART3_BAUDRATE,
            SERIAL_O_NBLCK_RX | SERIAL_O_NBLCK_TX);
    g_hw_smoke_snapshot.serials[HW_SMOKE_SERIAL_USART6].open_ret =
        hw_smoke_prepare_serial_device(
            devices->usart6,
            HW_SMOKE_USART6_BAUDRATE,
            SERIAL_O_NBLCK_RX | SERIAL_O_NBLCK_TX);
    g_hw_smoke_snapshot.serials[HW_SMOKE_SERIAL_UART7].open_ret =
        hw_smoke_prepare_serial_device(
            devices->uart7,
            HW_SMOKE_UART7_BAUDRATE,
            SERIAL_O_NBLCK_TX);
    g_hw_smoke_snapshot.serials[HW_SMOKE_SERIAL_UART8].open_ret =
        hw_smoke_prepare_serial_device(
            devices->uart8,
            HW_SMOKE_UART8_BAUDRATE,
            SERIAL_O_NBLCK_RX | SERIAL_O_NBLCK_TX);

    if (devices->usart3 != 0)
    {
        device_set_param(devices->usart3, &g_hw_smoke_serial_runtime[HW_SMOKE_SERIAL_USART3]);
        device_set_read_cb(devices->usart3, hw_smoke_serial_read_callback);
    }
    if (devices->usart6 != 0)
    {
        device_set_param(devices->usart6, &g_hw_smoke_serial_runtime[HW_SMOKE_SERIAL_USART6]);
        device_set_read_cb(devices->usart6, hw_smoke_serial_read_callback);
    }
    if (devices->uart8 != 0)
    {
        device_set_param(devices->uart8, &g_hw_smoke_serial_runtime[HW_SMOKE_SERIAL_UART8]);
        device_set_read_cb(devices->uart8, hw_smoke_serial_read_callback);
    }

    for (serial_index = 0u; serial_index < HW_SMOKE_SERIAL_COUNT; serial_index++)
    {
        if (g_hw_smoke_snapshot.serials[serial_index].open_ret != OM_OK)
        {
            g_hw_smoke_snapshot.last_error_code = 0xD000u + (uint32_t)serial_index;
        }
    }

    if (devices->can1 != 0)
    {
        can_ret = hw_smoke_configure_loopback_can(devices->can1, HW_SMOKE_CAN1, HW_SMOKE_CAN1_TEST_ID);
        if (can_ret != OM_OK)
        {
            return can_ret;
        }
    }

    if (devices->can2 != 0)
    {
        can_ret = hw_smoke_configure_loopback_can(devices->can2, HW_SMOKE_CAN2, HW_SMOKE_CAN2_TEST_ID);
        if (can_ret != OM_OK)
        {
            return can_ret;
        }
    }

    thread_ret = osal_thread_create(&smoke_task, &smoke_attr, hw_smoke_task_entry, (void*)devices);
    if (thread_ret != OSAL_OK)
    {
        g_hw_smoke_snapshot.last_error_code = 0xE000u;
        return OM_ERROR;
    }

    return OM_OK;
}
