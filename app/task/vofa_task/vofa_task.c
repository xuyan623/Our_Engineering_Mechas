#include "task/vofa_task/vofa_task.h"

#include "config/app_config.h"
#include "module/data_pool/data_pool.h"
#include "task/arm_task/arm_task.h"
#include "driver/motor/motor.h"
#include "drivers/peripheral/serial/pal_serial_dev.h"
#include "function/vofa/vofa.h"
#include "osal/osal.h"
#include "osal/osal_config.h"
#include "osal/osal_time.h"
#include <string.h>

#define VOFA_TASK_PERIOD_MS                        (10u)
#define VOFA_CUSTOM_CONTROLLER_CHANNEL_COUNT       (8u)
#define VOFA_MOTOR_FEEDBACK_COUNT                  (14u)
#define VOFA_DEBUG_CHANNEL_COUNT                   (3u)
#define VOFA_CHANNEL_INDEX_CONTROLLER_ONLINE       (0u)
#define VOFA_CHANNEL_INDEX_WORK_MODE               (1u)
#define VOFA_CHANNEL_INDEX_ANGLE_Y                 (2u)
#define VOFA_CHANNEL_INDEX_ANGLE_Z                 (3u)
#define VOFA_CHANNEL_INDEX_ANGLE_X                 (4u)
#define VOFA_CHANNEL_INDEX_ANGLE_YAW               (5u)
#define VOFA_CHANNEL_INDEX_ANGLE_PITCH             (6u)
#define VOFA_CHANNEL_INDEX_ANGLE_ROLL              (7u)
#define VOFA_CHANNEL_INDEX_MOTOR_FEEDBACK_BASE     (VOFA_CUSTOM_CONTROLLER_CHANNEL_COUNT)
#define VOFA_CHANNEL_INDEX_DEBUG_BASE              (VOFA_CHANNEL_INDEX_MOTOR_FEEDBACK_BASE + VOFA_MOTOR_FEEDBACK_COUNT)
#define VOFA_CHANNEL_INDEX_DEBUG_CHASSIS_MODE      (VOFA_CHANNEL_INDEX_DEBUG_BASE + 0u)
#define VOFA_CHANNEL_INDEX_DEBUG_ALIGNMENT_DONE    (VOFA_CHANNEL_INDEX_DEBUG_BASE + 1u)
#define VOFA_CHANNEL_INDEX_DEBUG_CONTROLLER_ONLINE (VOFA_CHANNEL_INDEX_DEBUG_BASE + 2u)
#define VOFA_CHANNEL_COUNT                         (VOFA_CUSTOM_CONTROLLER_CHANNEL_COUNT + VOFA_MOTOR_FEEDBACK_COUNT + VOFA_DEBUG_CHANNEL_COUNT)
#define VOFA_TASK_UART7_BAUDRATE                   (115200u)
#define VOFA_TASK_UART7_TX_BUFSIZE                 (128u)
#define VOFA_TASK_UART7_RX_BUFSIZE                 (64u)
#define VOFA_TASK_UART7_RX_DRAIN_BUDGET            (32u)
#define VOFA_TASK_STACK_BYTES                      (512u * OSAL_STACK_WORD_BYTES)

static MotorFeedbackSnapshot g_vofa_feedback_snapshots[VOFA_MOTOR_FEEDBACK_COUNT] = {0};
static float g_vofa_frame[VOFA_CHANNEL_COUNT] = {0.0f};

static float vofa_task_resolve_pitch2_zero_angle_rad(void)
{
    Motor* pitch2_motor = OM_NULL;
    float pitch2_zero_angle_rad = 0.0f;

    pitch2_motor = motor_find_by_name("pitch2");
    if (pitch2_motor == OM_NULL)
    {
        return 0.0f;
    }

    if (motor_get_initial_zero_angle_rad(
            pitch2_motor,
            &pitch2_zero_angle_rad) != OM_TRUE)
    {
        return 0.0f;
    }

    return pitch2_zero_angle_rad;
}

static float vofa_task_resolve_roll3_single_turn_rad(void)
{
    Motor* roll3_motor = OM_NULL;
    float roll3_angle_rad = 0.0f;

    roll3_motor = motor_find_by_name("roll3");
    if (roll3_motor == OM_NULL)
    {
        return 0.0f;
    }

    if (motor_get_single_turn_angle_rad(roll3_motor, &roll3_angle_rad) != OM_TRUE)
    {
        return 0.0f;
    }

    return roll3_angle_rad;
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
        return vofa_task_resolve_roll3_single_turn_rad();
    }

    return snapshot->feedback.angle;
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

    frame[VOFA_CHANNEL_INDEX_CONTROLLER_ONLINE] = (float)DP_LOAD_UINT8(&g_data_pool.custom_controller.online);
    frame[VOFA_CHANNEL_INDEX_WORK_MODE] = (float)DP_LOAD_UINT8(&g_data_pool.custom_controller.work_mode);
    frame[VOFA_CHANNEL_INDEX_ANGLE_Y] = DP_LOAD_FLOAT(&g_data_pool.custom_controller.angle_deg[0]);
    frame[VOFA_CHANNEL_INDEX_ANGLE_Z] = DP_LOAD_FLOAT(&g_data_pool.custom_controller.angle_deg[1]);
    frame[VOFA_CHANNEL_INDEX_ANGLE_X] = DP_LOAD_FLOAT(&g_data_pool.custom_controller.angle_deg[2]);
    frame[VOFA_CHANNEL_INDEX_ANGLE_YAW] = DP_LOAD_FLOAT(&g_data_pool.custom_controller.angle_deg[3]);
    frame[VOFA_CHANNEL_INDEX_ANGLE_PITCH] = DP_LOAD_FLOAT(&g_data_pool.custom_controller.angle_deg[4]);
    frame[VOFA_CHANNEL_INDEX_ANGLE_ROLL] = DP_LOAD_FLOAT(&g_data_pool.custom_controller.angle_deg[5]);

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
        frame[VOFA_CHANNEL_INDEX_MOTOR_FEEDBACK_BASE + index] =
            vofa_task_convert_feedback_angle_to_action_unit(&g_vofa_feedback_snapshots[index]);
    }

    frame[VOFA_CHANNEL_INDEX_DEBUG_CHASSIS_MODE] =
        (float)DP_LOAD_UINT8(&g_data_pool.mode.chassis_mode);
    frame[VOFA_CHANNEL_INDEX_DEBUG_ALIGNMENT_DONE] =
        (float)arm_task_get_custom_controller_alignment_done();
    frame[VOFA_CHANNEL_INDEX_DEBUG_CONTROLLER_ONLINE] =
        (float)DP_LOAD_UINT8(&g_data_pool.custom_controller.online);
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
