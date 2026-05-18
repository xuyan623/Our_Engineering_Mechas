#include "task/vofa_task/vofa_task.h"

#include "config/app_config.h"
#include "module/data_pool/data_pool.h"
#include "driver/motor/motor.h"
#include "drivers/peripheral/serial/pal_serial_dev.h"
#include "function/vofa/vofa.h"
#include "osal/osal.h"
#include "osal/osal_config.h"
#include "osal/osal_time.h"
#include <string.h>

#define VOFA_TASK_PERIOD_MS             (10u)
#define VOFA_MOTOR_FEEDBACK_COUNT       (14u)
#define VOFA_RC_CHANNEL_COUNT           (7u)
#define VOFA_EXTRA_CHANNEL_COUNT        (1u + VOFA_RC_CHANNEL_COUNT)
#define VOFA_CHANNEL_INDEX_PITCH2_ZERO  (VOFA_MOTOR_FEEDBACK_COUNT)
#define VOFA_CHANNEL_INDEX_RC_CH1       (VOFA_CHANNEL_INDEX_PITCH2_ZERO + 1u)
#define VOFA_CHANNEL_INDEX_RC_CH2       (VOFA_CHANNEL_INDEX_RC_CH1 + 1u)
#define VOFA_CHANNEL_INDEX_RC_CH3       (VOFA_CHANNEL_INDEX_RC_CH2 + 1u)
#define VOFA_CHANNEL_INDEX_RC_CH4       (VOFA_CHANNEL_INDEX_RC_CH3 + 1u)
#define VOFA_CHANNEL_INDEX_RC_SW1       (VOFA_CHANNEL_INDEX_RC_CH4 + 1u)
#define VOFA_CHANNEL_INDEX_RC_SW2       (VOFA_CHANNEL_INDEX_RC_SW1 + 1u)
#define VOFA_CHANNEL_INDEX_RC_IW        (VOFA_CHANNEL_INDEX_RC_SW2 + 1u)
#define VOFA_CHANNEL_COUNT              (VOFA_MOTOR_FEEDBACK_COUNT + VOFA_EXTRA_CHANNEL_COUNT)
#define VOFA_TASK_UART7_BAUDRATE        (115200u)
#define VOFA_TASK_UART7_TX_BUFSIZE      (128u)
#define VOFA_TASK_UART7_RX_BUFSIZE      (64u)
#define VOFA_TASK_UART7_RX_DRAIN_BUDGET (32u)
#define VOFA_TASK_STACK_BYTES           (512u * OSAL_STACK_WORD_BYTES)

static MotorFeedbackSnapshot g_vofa_feedback_snapshots[VOFA_MOTOR_FEEDBACK_COUNT] = {0};
static float g_vofa_frame[VOFA_CHANNEL_COUNT] = {0.0f};

static float vofa_task_rad_to_deg(float angle_rad)
{
    return angle_rad * (180.0f / APP_PI);
}

static float vofa_task_resolve_pitch2_zero_angle_rad(void)
{
    Motor* pitch2_motor = OM_NULL;
    float pitch2_zero_angle_rad = 0.0f;

    pitch2_motor = motor_find_by_name("pitch2");
    if (pitch2_motor == OM_NULL || pitch2_motor->binding.go8010.driver == OM_NULL)
    {
        return 0.0f;
    }

    if (go8010_get_initial_position_zero(
            pitch2_motor->binding.go8010.driver,
            &pitch2_zero_angle_rad) != OM_TRUE)
    {
        return 0.0f;
    }

    return pitch2_zero_angle_rad;
}

static float vofa_task_resolve_roll3_single_turn_deg(void)
{
    Motor* roll3_motor = OM_NULL;

    roll3_motor = motor_find_by_name("roll3");
    if (roll3_motor == OM_NULL || roll3_motor->binding.dji.driver == OM_NULL)
    {
        return 0.0f;
    }

    return dji_motor_get_singgle_angle(roll3_motor->binding.dji.driver);
}

static float vofa_task_convert_feedback_angle_to_action_unit(const MotorFeedbackSnapshot* snapshot)
{
    float pitch2_zero_angle_rad = 0.0f;

    if (snapshot == OM_NULL || snapshot->name == OM_NULL)
    {
        return 0.0f;
    }

    if (strcmp(snapshot->name, "pitch1") == 0)
    {
        return snapshot->feedback.angle * APP_ARM_PITCH1_TARGET_RATIO;
    }

    if (strcmp(snapshot->name, "pitch2") == 0)
    {
        pitch2_zero_angle_rad = vofa_task_resolve_pitch2_zero_angle_rad();
        return (pitch2_zero_angle_rad - snapshot->feedback.angle) / APP_ARM_PITCH2_GEAR_RATIO;
    }

    if (strcmp(snapshot->name, "roll3") == 0)
    {
        return vofa_task_resolve_roll3_single_turn_deg();
    }

    return snapshot->feedback.angle;
}

static void vofa_task_fill_pitch2_zero(float frame[VOFA_CHANNEL_COUNT])
{
    if (frame == OM_NULL)
    {
        return;
    }

    frame[VOFA_CHANNEL_INDEX_PITCH2_ZERO] = vofa_task_resolve_pitch2_zero_angle_rad();
}

static void vofa_task_fill_rc_snapshot(float frame[VOFA_CHANNEL_COUNT])
{
    if (frame == OM_NULL)
    {
        return;
    }

    frame[VOFA_CHANNEL_INDEX_RC_CH1] = (float)DP_LOAD_INT16(&g_data_pool.rc.ch1);
    frame[VOFA_CHANNEL_INDEX_RC_CH2] = (float)DP_LOAD_INT16(&g_data_pool.rc.ch2);
    frame[VOFA_CHANNEL_INDEX_RC_CH3] = (float)DP_LOAD_INT16(&g_data_pool.rc.ch3);
    frame[VOFA_CHANNEL_INDEX_RC_CH4] = (float)DP_LOAD_INT16(&g_data_pool.rc.ch4);
    frame[VOFA_CHANNEL_INDEX_RC_SW1] = (float)DP_LOAD_UINT8(&g_data_pool.rc.sw1);
    frame[VOFA_CHANNEL_INDEX_RC_SW2] = (float)DP_LOAD_UINT8(&g_data_pool.rc.sw2);
    frame[VOFA_CHANNEL_INDEX_RC_IW] = (float)DP_LOAD_UINT16(&g_data_pool.rc.iw);
}

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
        frame[index] = vofa_task_convert_feedback_angle_to_action_unit(&g_vofa_feedback_snapshots[index]);
    }

    vofa_task_fill_pitch2_zero(frame);
    vofa_task_fill_rc_snapshot(frame);
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
