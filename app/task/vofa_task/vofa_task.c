#include "task/vofa_task/vofa_task.h"

#include "driver/motor/motor.h"
#include "drivers/peripheral/serial/pal_serial_dev.h"
#include "function/vofa/vofa.h"
#include "osal/osal.h"
#include "osal/osal_config.h"
#include "osal/osal_time.h"
#include <string.h>

#define VOFA_TASK_PERIOD_MS             (10u)
#define VOFA_MOTOR_FEEDBACK_COUNT       (14u)
#define VOFA_CHANNEL_COUNT              (VOFA_MOTOR_FEEDBACK_COUNT)
#define VOFA_TASK_UART7_BAUDRATE        (115200u)
#define VOFA_TASK_UART7_TX_BUFSIZE      (128u)
#define VOFA_TASK_UART7_RX_BUFSIZE      (64u)
#define VOFA_TASK_UART7_RX_DRAIN_BUDGET (32u)
#define VOFA_TASK_STACK_BYTES           (512u * OSAL_STACK_WORD_BYTES)

static MotorFeedbackSnapshot g_vofa_feedback_snapshots[VOFA_MOTOR_FEEDBACK_COUNT] = {0};
static float g_vofa_frame[VOFA_CHANNEL_COUNT] = {0.0f};

static OmRet vofa_task_prepare_uart7(Device* uart7_device)
{
    SerialCfg serial_cfg = SERIAL_DEFAULT_CFG;
    HalSerial* hal_serial = OM_NULL;

    if (uart7_device == OM_NULL)
    {
        return OM_ERROR_NULL;
    }

    serial_cfg.baudrate = VOFA_TASK_UART7_BAUDRATE;
    serial_cfg.txBufSize = VOFA_TASK_UART7_TX_BUFSIZE;
    serial_cfg.rxBufSize = VOFA_TASK_UART7_RX_BUFSIZE;

    hal_serial = (HalSerial*)uart7_device;
    hal_serial->cfg = serial_cfg;

    return device_open(uart7_device, SERIAL_O_NBLCK_RX | SERIAL_O_NBLCK_TX);
}

static void vofa_task_drain_uart7_rx(Device* uart7_device)
{
    uint8_t byte = 0u;
    uint32_t count = 0u;

    if (uart7_device == OM_NULL)
    {
        return;
    }

    while (count < VOFA_TASK_UART7_RX_DRAIN_BUDGET)
    {
        if (device_read(uart7_device, 0, &byte, 1u) != 1u)
        {
            break;
        }
        count++;
    }
}

static void vofa_task_fill_frame(float frame[VOFA_CHANNEL_COUNT])
{
    uint32_t index = 0u;
    uint32_t snapshot_count = 0u;

    for (index = 0u; index < VOFA_CHANNEL_COUNT; index++)
    {
        frame[index] = 0.0f;
    }

    memset(g_vofa_feedback_snapshots, 0, sizeof(g_vofa_feedback_snapshots));
    if (motor_copy_feedback_snapshots(
            g_vofa_feedback_snapshots,
            VOFA_MOTOR_FEEDBACK_COUNT,
            &snapshot_count) != OM_OK)
    {
        return;
    }

    for (index = 0u; index < snapshot_count; index++)
    {
        frame[index] = g_vofa_feedback_snapshots[index].feedback.angle;
    }
}

static void vofa_task_entry(void* arg)
{
    const BspDeviceRegistry* devices = (const BspDeviceRegistry*)arg;
    OsalTimeMs deadline_cursor_ms = 0u;

    while (1)
    {
        vofa_task_drain_uart7_rx(devices->uart7);
        vofa_task_fill_frame(g_vofa_frame);
        vofa_justfloat_send(devices->uart7, g_vofa_frame, VOFA_CHANNEL_COUNT);
        (void)osal_delay_until(&deadline_cursor_ms, VOFA_TASK_PERIOD_MS, OM_NULL);
    }
}

OmRet vofa_task_start(const BspDeviceRegistry* devices)
{
    static OsalThread* vofa_task_thread = OM_NULL;
    const OsalThreadAttr vofa_task_attr = {"vofa_task", VOFA_TASK_STACK_BYTES, 3u};
    OsalStatus status = OSAL_INVALID;

    if (devices == OM_NULL || devices->uart7 == OM_NULL)
    {
        return OM_ERROR_NULL;
    }

    if (vofa_task_thread != OM_NULL)
    {
        return OM_ERR_CONFLICT;
    }

    if (vofa_task_prepare_uart7(devices->uart7) != OM_OK)
    {
        return OM_ERROR;
    }

    status = osal_thread_create(&vofa_task_thread, &vofa_task_attr, vofa_task_entry, (void*)devices);
    if (status != OSAL_OK)
    {
        vofa_task_thread = OM_NULL;
        return OM_ERROR;
    }

    return OM_OK;
}
