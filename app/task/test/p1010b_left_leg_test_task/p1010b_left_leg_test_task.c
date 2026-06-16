#include "task/test/p1010b_left_leg_test_task/p1010b_left_leg_test_task.h"

#include "driver/p1010b/P1010B.h"
#include "drivers/peripheral/can/pal_can_dev.h"
#include "module/system_health/system_health.h"
#include "osal/osal.h"
#include "osal/osal_config.h"
#include "osal/osal_time.h"
#include <string.h>

#define P1010B_LEFT_LEG_TEST_LOOP_PERIOD_MS               (10u)
#define P1010B_LEFT_LEG_TEST_RETRY_DELAY_MS               (50u)
#define P1010B_LEFT_LEG_TEST_RETRY_LOOPS (10u)
#define P1010B_LEFT_LEG_TEST_CAN_BAUDRATE                 (CAN_BAUD_1M)
#define P1010B_LEFT_LEG_TEST_MOTOR_ID                     (2u)
#define P1010B_LEFT_LEG_TEST_DEFAULT_MODE                 (P1010B_MODE_CURRENT)
#define P1010B_LEFT_TEST_QUERY_MS       (1u)
#define P1010B_LEFT_TEST_ONLINE_MS            (100u)
#define P1010B_LEFT_LEG_TEST_ENCODER_WRAP                 (32768)
#define P1010B_LEFT_TEST_HALF_WRAP            (16384)
#define P1010B_LEFT_TEST_SCALE_DEG              (360.0f / 32768.0f)

typedef struct
{
    Device* can_device;
    P1010BBus can_bus;
    P1010BDriver driver;
    uint16_t last_absolute_position_raw;
    uint16_t offset_absolute_position_raw;
    int32_t round_count;
    int32_t total_encoder_counts;
    OmBool offset_initialized_flag;
    OmBool bus_initialized_flag;
    uint32_t last_feedback_timestamp_ms;
} P1010BLeftRuntime;

static P1010BLeftRuntime g_p1010b_left_leg_test_runtime = {0};
P1010BLeftDebug g_p1010b_left_leg_test_debug = {0};

static P1010BActiveReportConfig p10lt_query_cfg(void)
{
    return (P1010BActiveReportConfig){
        /* 查询模式下驱动仍要求 periodMs 非 0，这里给 1ms 占位。 */
        .enable = false,
        .periodMs = P1010B_LEFT_TEST_QUERY_MS,
        .dataTypeSlots = {
            (uint8_t)P1010B_REPORT_DATA_ABSOLUTE_POSITION,
            (uint8_t)P1010B_REPORT_DATA_SPEED_RPM,
            (uint8_t)P1010B_REPORT_DATA_IQ_AMPERE,
            (uint8_t)P1010B_REPORT_DATA_BUS_VOLTAGE,
        },
    };
}

static OmRet p10lt_can(Device* can_device)
{
    CanCfg can_cfg = CAN_DEFUALT_CFG;
    OmRet ret = OM_OK;
    uint32_t io_type = 0u;

    if (can_device == OM_NULL)
    {
        return OM_ERROR_NULL;
    }

    if (!device_check_status(can_device, DEV_STATUS_OPENED))
    {
        ret = device_open(can_device, CAN_O_INT_RX | CAN_O_INT_TX);
        if (ret != OM_OK)
        {
            return ret;
        }
    }

    ret = device_ctrl(can_device, CAN_CMD_SUSPEND, OM_NULL);
    if (ret != OM_OK)
    {
        return ret;
    }

    can_cfg.normalTimeCfg.baudRate = P1010B_LEFT_LEG_TEST_CAN_BAUDRATE;
    ret = device_ctrl(can_device, CAN_CMD_CFG, &can_cfg);
    if (ret != OM_OK)
    {
        return ret;
    }

    io_type = CAN_REG_INT_RX;
    ret = device_ctrl(can_device, CAN_CMD_SET_IOTYPE, &io_type);
    if (ret != OM_OK)
    {
        return ret;
    }

    io_type = CAN_REG_INT_TX;
    ret = device_ctrl(can_device, CAN_CMD_SET_IOTYPE, &io_type);
    if (ret != OM_OK)
    {
        return ret;
    }

    return OM_OK;
}

static void p10lt_track_pos(
    P1010BLeftRuntime* runtime,
    uint16_t absolute_position_raw)
{
    if (runtime == OM_NULL)
    {
        return;
    }

    if (runtime->offset_initialized_flag != OM_TRUE)
    {
        runtime->offset_absolute_position_raw = absolute_position_raw;
        runtime->last_absolute_position_raw = absolute_position_raw;
        runtime->round_count = 0;
        runtime->total_encoder_counts = 0;
        runtime->offset_initialized_flag = OM_TRUE;
        return;
    }

    {
        int32_t encoder_delta =
            (int32_t)absolute_position_raw - (int32_t)runtime->last_absolute_position_raw;

        if (encoder_delta > P1010B_LEFT_TEST_HALF_WRAP)
        {
            runtime->round_count--;
        }
        else if (encoder_delta < -P1010B_LEFT_TEST_HALF_WRAP)
        {
            runtime->round_count++;
        }

        runtime->total_encoder_counts =
            runtime->round_count * P1010B_LEFT_LEG_TEST_ENCODER_WRAP +
            (int32_t)absolute_position_raw -
            (int32_t)runtime->offset_absolute_position_raw;
        runtime->last_absolute_position_raw = absolute_position_raw;
    }
}

static void p10lt_write_fb(
    P1010BLeftRuntime* runtime,
    uint16_t absolute_position_raw,
    int16_t speed_raw,
    int16_t iq_raw,
    int16_t bus_voltage_raw,
    uint32_t timestamp_ms)
{
    if (runtime == OM_NULL)
    {
        return;
    }

    runtime->last_feedback_timestamp_ms = timestamp_ms;
    g_p1010b_left_leg_test_debug.feedback_count++;
    p10lt_track_pos(runtime, absolute_position_raw);

    g_p1010b_left_leg_test_debug.absolute_position_raw = (float)absolute_position_raw;
    g_p1010b_left_leg_test_debug.total_angle_deg =
        ((float)runtime->total_encoder_counts) * P1010B_LEFT_TEST_SCALE_DEG;
    g_p1010b_left_leg_test_debug.speed_rpm = ((float)speed_raw) / 10.0f;
    g_p1010b_left_leg_test_debug.speed_deg_per_s =
        g_p1010b_left_leg_test_debug.speed_rpm * 6.0f;
    g_p1010b_left_leg_test_debug.iq_current_amp = ((float)iq_raw) / 100.0f;
    g_p1010b_left_leg_test_debug.bus_voltage_v = ((float)bus_voltage_raw) / 10.0f;
}

static OmRet p10lt_query_fb(P1010BLeftRuntime* runtime)
{
    P1010BResponse response = {0};
    OmRet ret = OM_OK;
    uint16_t absolute_position_raw = 0u;
    int16_t speed_raw = 0;
    int16_t iq_raw = 0;
    int16_t bus_voltage_raw = 0;

    if (runtime == OM_NULL)
    {
        return OM_ERROR_NULL;
    }

    ret = p1010b_active_query_slots(
        &runtime->driver,
        P1010B_REPORT_DATA_ABSOLUTE_POSITION,
        P1010B_REPORT_DATA_SPEED_RPM,
        P1010B_REPORT_DATA_IQ_AMPERE,
        P1010B_REPORT_DATA_BUS_VOLTAGE,
        0u,
        &response);
    if (ret != OM_OK)
    {
        return ret;
    }

    absolute_position_raw = (uint16_t)response.data.activeQueryValues[0];
    speed_raw = response.data.activeQueryValues[1];
    iq_raw = response.data.activeQueryValues[2];
    bus_voltage_raw = response.data.activeQueryValues[3];

    p10lt_write_fb(
        runtime,
        absolute_position_raw,
        speed_raw,
        iq_raw,
        bus_voltage_raw,
        response.timestampMs);
    return OM_OK;
}

static void p10lt_online(P1010BLeftRuntime* runtime)
{
    uint32_t now_ms = 0u;
    uint32_t age_ms = 0u;

    if (runtime == OM_NULL)
    {
        return;
    }

    now_ms = osal_time_now_monotonic();
    if (runtime->last_feedback_timestamp_ms > 0u)
    {
        age_ms = now_ms - runtime->last_feedback_timestamp_ms;
    }

    g_p1010b_left_leg_test_debug.last_rx_age_ms = age_ms;
    g_p1010b_left_leg_test_debug.online_flag =
        (runtime->last_feedback_timestamp_ms > 0u &&
         age_ms <= P1010B_LEFT_TEST_ONLINE_MS)
            ? 1u
            : 0u;
}

static OmRet p10lt_prep_motor(P1010BLeftRuntime* runtime)
{
    const P1010BActiveReportConfig query_mode_config =
        p10lt_query_cfg();
    P1010BResponse response = {0};
    OmRet ret = OM_OK;

    if (runtime == OM_NULL)
    {
        return OM_ERROR_NULL;
    }

    ret = p1010b_disable(&runtime->driver, 0u, &response);
    if (ret != OM_OK)
    {
        return ret;
    }

    ret = p1010b_set_mode(&runtime->driver, P1010B_LEFT_LEG_TEST_DEFAULT_MODE, 0u, &response);
    if (ret != OM_OK)
    {
        return ret;
    }

    ret = p1010b_set_active_report(&runtime->driver, &query_mode_config, 0u, &response);
    if (ret != OM_OK)
    {
        return ret;
    }

    ret = p1010b_enable(&runtime->driver, 0u, &response);
    return ret;
}

static OmRet p10lt_init(
    P1010BLeftRuntime* runtime,
    const BspDeviceRegistry* devices)
{
    P1010BConfig driver_config = P1010B_DEFAULT_CONFIG(P1010B_LEFT_LEG_TEST_MOTOR_ID);
    OmRet ret = OM_OK;

    if (runtime == OM_NULL || devices == OM_NULL || devices->can1 == OM_NULL)
    {
        return OM_ERROR_NULL;
    }

    memset(runtime, 0, sizeof(*runtime));
    runtime->can_device = devices->can1;

    driver_config.defaultMode = P1010B_LEFT_LEG_TEST_DEFAULT_MODE;
    driver_config.activeReport = p10lt_query_cfg();

    ret = p10lt_can(runtime->can_device);
    if (ret != OM_OK)
    {
        return ret;
    }

    ret = p1010b_bus_init(&runtime->can_bus, runtime->can_device);
    if (ret != OM_OK)
    {
        return ret;
    }
    runtime->bus_initialized_flag = OM_TRUE;

    ret = device_ctrl(runtime->can_device, CAN_CMD_START, OM_NULL);
    if (ret != OM_OK)
    {
        (void)p1010b_bus_deinit(&runtime->can_bus);
        memset(runtime, 0, sizeof(*runtime));
        return ret;
    }

    ret = p1010b_register(&runtime->can_bus, &runtime->driver, &driver_config);
    if (ret != OM_OK)
    {
        (void)p1010b_bus_deinit(&runtime->can_bus);
        memset(runtime, 0, sizeof(*runtime));
        return ret;
    }

    return OM_OK;
}

static void p10lt_entry(void* arg)
{
    P1010BLeftRuntime* runtime = (P1010BLeftRuntime*)arg;
    OsalTimeMs deadline_cursor_ms = 0u;
    uint32_t retry_loop_count = 0u;

    if (runtime == OM_NULL)
    {
        for (;;)
        {
            (void)osal_sleep_ms(1000u);
        }
    }

    g_p1010b_left_leg_test_debug.prepare_ret =
        (int32_t)p10lt_prep_motor(runtime);

    while (1)
    {
        retry_loop_count++;
        g_p1010b_left_leg_test_debug.driver_state = (uint8_t)p1010b_get_state(&runtime->driver);

        if (p10lt_query_fb(runtime) != OM_OK &&
            (retry_loop_count % P1010B_LEFT_LEG_TEST_RETRY_LOOPS) == 0u)
        {
            g_p1010b_left_leg_test_debug.prepare_ret =
                (int32_t)p10lt_prep_motor(runtime);
            (void)osal_sleep_ms(P1010B_LEFT_LEG_TEST_RETRY_DELAY_MS);
            continue;
        }

        p10lt_online(runtime);
        (void)sh_beat(SH_TASK_P1010B_LEFT_LEG_SMOKE);
        (void)osal_delay_until(&deadline_cursor_ms, P1010B_LEFT_LEG_TEST_LOOP_PERIOD_MS, OM_NULL);
    }
}

OmRet p10lt_start(const BspDeviceRegistry* devices)
{
    static OsalThread* p1010b_left_leg_test_task = OM_NULL;
    const OsalThreadAttr p1010b_left_leg_test_attr = {
        "p1010b_left_leg_test",
        768u * OSAL_STACK_WORD_BYTES,
        4u};
    OsalStatus status = OSAL_INVALID;
    OmRet ret = OM_OK;

    if (devices == OM_NULL)
    {
        return OM_ERROR_NULL;
    }

    if (p1010b_left_leg_test_task != OM_NULL)
    {
        return OM_ERR_CONFLICT;
    }

    memset(&g_p1010b_left_leg_test_debug, 0, sizeof(g_p1010b_left_leg_test_debug));

    ret = p10lt_init(&g_p1010b_left_leg_test_runtime, devices);
    g_p1010b_left_leg_test_debug.init_ret = (int32_t)ret;
    if (ret != OM_OK)
    {
        return ret;
    }

    status = osal_thread_create(
        &p1010b_left_leg_test_task,
        &p1010b_left_leg_test_attr,
        p10lt_entry,
        &g_p1010b_left_leg_test_runtime);
    if (status != OSAL_OK)
    {
        p1010b_left_leg_test_task = OM_NULL;
        if (g_p1010b_left_leg_test_runtime.bus_initialized_flag == OM_TRUE)
        {
            (void)p1010b_bus_deinit(&g_p1010b_left_leg_test_runtime.can_bus);
        }
        memset(&g_p1010b_left_leg_test_runtime, 0, sizeof(g_p1010b_left_leg_test_runtime));
        return OM_ERROR;
    }

    return OM_OK;
}
