#include "task/input_task/input_task.h"

#include "drivers/model/device.h"
#include "drivers/peripheral/serial/pal_serial_dev.h"
#include "module/system_health/system_health.h"
#include "osal/osal.h"
#include "osal/osal_config.h"
#include "osal/osal_time.h"
#include "task/arm_task/arm_task.h"
#include "task/chassis_task/chassis_task.h"
#include "task/mode_task/mode_task.h"
#include "task/input_task/input_task_custom_controller.h"
#include "task/input_task/input_task_judge_stub.h"
#include "task/input_task/input_task_rc.h"
#include <string.h>

#define INPUT_TASK_PERIOD_MS (1u)

/* input_task 当前是外部控制输入的唯一 owner：
 * - USART1：DBUS 遥控器
 * - UART8：自定义控制器
 * - USART3：本轮只保留 stub 位，不正式接线
 *
 * mode_task / arm_task / chassis_task 后续都只消费 formal input snapshot，
 * 不再直接拥有这些输入串口。
 */
InputTaskDebugState g_input_task_runtime = {0};

static void input_task_usart1_read_callback(Device* dev, void* param, size_t paramsz);
static void input_task_uart8_read_callback(Device* dev, void* param, size_t paramsz);
static OmRet input_task_prepare_usart1(Device* usart1_device);
static OmRet input_task_prepare_uart8(Device* uart8_device);

/* 串口回调只负责提示“可能有新数据”，
 * 不在 ISR / 回调上下文里直接解协议。
 */
static void input_task_usart1_read_callback(Device* dev, void* param, size_t paramsz)
{
    InputTaskRcDebugState* runtime = (InputTaskRcDebugState*)param;

    (void)dev;

    if (runtime == OM_NULL)
    {
        return;
    }

    runtime->rx_available_hint = (uint32_t)paramsz;
}

static void input_task_uart8_read_callback(Device* dev, void* param, size_t paramsz)
{
    InputTaskCustomControllerDebugState* runtime =
        (InputTaskCustomControllerDebugState*)param;

    (void)dev;

    if (runtime == OM_NULL)
    {
        return;
    }

    runtime->rx_available_hint = (uint32_t)paramsz;
}

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

    device_set_param(usart1_device, &g_input_task_runtime.rc);
    device_set_read_cb(usart1_device, input_task_usart1_read_callback);

    return device_open(usart1_device, SERIAL_O_NBLCK_RX);
}

/* UART8 是自定义控制器输入 owner 的物理入口。
 * 本轮即使它打开失败，也只会让控制器输入降级离线，不会拖死整个 input_task。
 */
static OmRet input_task_prepare_uart8(Device* uart8_device)
{
    SerialCfg serial_cfg = SERIAL_DEFAULT_CFG;
    HalSerial* hal_serial = OM_NULL;

    if (uart8_device == OM_NULL)
    {
        return OM_ERROR_NULL;
    }

    serial_cfg.baudrate = INPUT_TASK_CUSTOM_CONTROLLER_UART8_BAUDRATE;
    serial_cfg.txBufSize = INPUT_TASK_CUSTOM_CONTROLLER_UART8_TX_BUFSIZE;
    serial_cfg.rxBufSize = INPUT_TASK_CUSTOM_CONTROLLER_UART8_RX_BUFSIZE;

    hal_serial = (HalSerial*)uart8_device;
    hal_serial->cfg = serial_cfg;

    device_set_param(uart8_device, &g_input_task_runtime.custom_controller);
    device_set_read_cb(uart8_device, input_task_uart8_read_callback);

    return device_open(uart8_device, SERIAL_O_NBLCK_RX);
}

static void input_task_entry(void* arg)
{
    const BspDeviceRegistry* devices = (const BspDeviceRegistry*)arg;
    OsalTimeMs deadline_cursor_ms = 0u;
    OsalTimeMs now_ms = 0u;
    uint8_t raw_frame[INPUT_TASK_DBUS_FRAME_LEN] = {0};
    InputTaskRcFrame parsed_frame = {0};
    InputTaskCustomControllerParser custom_controller_parser = {0};
    InputRcSnapshot rc_snapshot = {0};
    InputCustomControllerSnapshot custom_controller_snapshot = {0};
    size_t read_len = 0u;
    uint8_t byte = 0u;
    OmBool rc_online_changed = OM_FALSE;
    OmBool custom_controller_snapshot_changed = OM_FALSE;
    uint32_t previous_custom_controller_frame_count = 0u;

    input_task_custom_controller_reset_parser(&custom_controller_parser, 0u);

    while (1)
    {
        rc_online_changed = OM_FALSE;
        custom_controller_snapshot_changed = OM_FALSE;
        previous_custom_controller_frame_count =
            g_input_task_runtime.custom_controller.frame_count;
        input_task_custom_controller_copy_snapshot(&custom_controller_snapshot);

        if (g_input_task_runtime.rc.rx_available_hint >= INPUT_TASK_DBUS_FRAME_LEN)
        {
            do
            {
                /* 当前串口 PAL 的非阻塞读语义要求：
                 * 缓冲区中至少有 len 个字节时，这次读取才会成功返回。
                 * 因此这里按 18 字节整帧读取，和旧工程的 DBUS 边界保持一致。
                 */
                read_len = device_read(
                    devices->usart1, 0, raw_frame, INPUT_TASK_DBUS_FRAME_LEN);
                if (read_len == INPUT_TASK_DBUS_FRAME_LEN)
                {
                    if (input_task_rc_decode_frame(raw_frame, &parsed_frame) ==
                        OM_FALSE)
                    {
                        g_input_task_runtime.rc.invalid_frame_count++;
                    }

                    input_task_rc_fill_snapshot(&parsed_frame, &rc_snapshot);
                    now_ms = osal_time_now_monotonic();
                    g_input_task_runtime.rc.last_frame_ms = (uint32_t)now_ms;
                    g_input_task_runtime.rc.online = 1u;
                    input_task_rc_commit_snapshot(&rc_snapshot);
                    (void)mode_task_submit_rc_snapshot(&rc_snapshot);
                    (void)chassis_task_submit_rc_snapshot(&rc_snapshot);
                    (void)arm_task_submit_rc_snapshot(&rc_snapshot);
                    g_input_task_runtime.rc.frame_count++;
                }
            } while (read_len == INPUT_TASK_DBUS_FRAME_LEN);

            g_input_task_runtime.rc.rx_available_hint = 0u;
        }

        if (devices->uart8 != OM_NULL &&
            g_input_task_runtime.custom_controller.degraded_start == 0u &&
            g_input_task_runtime.custom_controller.rx_available_hint > 0u)
        {
            /* 自定义控制器协议按字节流推进状态机：
             * - 不要求 UART8 一次凑齐整帧
             * - 每收到一个字节就喂给 parser
             */
            while (device_read(devices->uart8, 0, &byte, 1u) == 1u)
            {
                now_ms = osal_time_now_monotonic();
                (void)input_task_custom_controller_accept_byte(
                    &g_input_task_runtime.custom_controller,
                    &custom_controller_parser,
                    byte,
                    now_ms);
            }

            g_input_task_runtime.custom_controller.rx_available_hint = 0u;
        }

        now_ms = osal_time_now_monotonic();
        rc_online_changed =
            input_task_rc_update_online_state(&g_input_task_runtime.rc, now_ms);
        if (rc_online_changed == OM_TRUE)
        {
            input_task_rc_copy_snapshot(&rc_snapshot);
            (void)mode_task_submit_rc_snapshot(&rc_snapshot);
            (void)chassis_task_submit_rc_snapshot(&rc_snapshot);
            (void)arm_task_submit_rc_snapshot(&rc_snapshot);
        }

        custom_controller_snapshot_changed =
            input_task_custom_controller_update_online_state(
            &g_input_task_runtime.custom_controller,
            &custom_controller_parser,
            now_ms);

        input_task_custom_controller_copy_snapshot(&custom_controller_snapshot);
        if (g_input_task_runtime.custom_controller.frame_count !=
                previous_custom_controller_frame_count ||
            custom_controller_snapshot_changed == OM_TRUE)
        {
            custom_controller_snapshot_changed = OM_TRUE;
        }

        if (custom_controller_snapshot_changed == OM_TRUE)
        {
            (void)mode_task_submit_custom_controller_snapshot(
                &custom_controller_snapshot);
            (void)arm_task_submit_custom_controller_snapshot(
                &custom_controller_snapshot);
        }

        (void)osal_delay_until(&deadline_cursor_ms, INPUT_TASK_PERIOD_MS, OM_NULL);
    }
}

OmRet input_task_start(const BspDeviceRegistry* devices)
{
    static OsalThread* input_task_thread = OM_NULL;
    const OsalThreadAttr input_task_attr = {
        "input_task", 512u * OSAL_STACK_WORD_BYTES, 4u};
    OsalStatus status = OSAL_INVALID;

    if (devices == OM_NULL || devices->usart1 == OM_NULL)
    {
        return OM_ERROR_NULL;
    }

    if (input_task_thread != OM_NULL)
    {
        return OM_ERR_CONFLICT;
    }

    input_task_rc_reset_runtime(&g_input_task_runtime.rc);
    input_task_rc_reset_latest_snapshot();
    input_task_custom_controller_reset_runtime(
        &g_input_task_runtime.custom_controller);
    input_task_judge_stub_reset_runtime(&g_input_task_runtime.judge);
    input_task_custom_controller_reset_latest_snapshot();

    /* USART1 是本轮唯一必选输入源：它失败时 input_task 整体启动失败。 */
    if (input_task_prepare_usart1(devices->usart1) != OM_OK)
    {
        return OM_ERROR;
    }

    /* UART8 采用降级启动：
     * - 能打开就正式接入自定义控制器
     * - 打不开就把控制器 formal snapshot 钳成离线，不影响 DBUS 和整机其余链路
     */
    if (devices->uart8 == OM_NULL || input_task_prepare_uart8(devices->uart8) != OM_OK)
    {
        g_input_task_runtime.custom_controller.degraded_start = 1u;
        input_task_custom_controller_reset_latest_snapshot();
    }

    status = osal_thread_create(
        &input_task_thread, &input_task_attr, input_task_entry, (void*)devices);
    if (status != OSAL_OK)
    {
        input_task_thread = OM_NULL;
        return OM_ERROR;
    }

    {
        const ModeTaskInitProgressMessage init_progress = {
            .kind = (uint8_t)MODE_TASK_INIT_PROGRESS_SERIAL_READY,
            .value = 1u};
        (void)mode_task_submit_init_progress(&init_progress);
    }

    return OM_OK;
}
