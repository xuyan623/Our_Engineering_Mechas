#include "task/test/dji_m3508_test_task/dji_m3508_test_task.h"

#include "driver/dji/dji_motor.h"
#include "drivers/peripheral/can/pal_can_dev.h"
#include "osal/osal.h"
#include "osal/osal_config.h"
#include "osal/osal_time.h"
#include <string.h>

#define DJI_M3508_TEST_LOOP_PERIOD_MS      (5u)
#define DJI_M3508_TEST_STEP_INTERVAL_MS    (1000u)
#define DJI_M3508_TEST_OUTPUT_AMPLITUDE    (1000)
#define DJI_M3508_TEST_MOTOR_ID            (1u)

typedef struct
{
    Device* can_device;
    DJIMotorBus can_bus;
    DJIMotorDrv motor;
    float last_angle_deg;
    float last_speed_rpm;
    float last_current_amp;
    float last_temp_deg;
} DjiM3508TestRuntime;

static DjiM3508TestRuntime g_dji_m3508_test_runtime = {0};
DjiM3508TestDebugState g_dji_m3508_test_debug = {0};

static OmRet dji_m3508_test_configure_can(Device* can_device)
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

    /* 测试任务切回 OMR 标准链路后，仍需要把 can1 从 BSP 默认的 500K
     * 收敛到 DJI 电调使用的 1M，避免与正式电机协议不一致。
     */
    ret = device_ctrl(can_device, CAN_CMD_SUSPEND, OM_NULL);
    if (ret != OM_OK)
    {
        return ret;
    }

    can_cfg.normalTimeCfg.baudRate = CAN_BAUD_1M;
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

    ret = device_ctrl(can_device, CAN_CMD_START, OM_NULL);
    if (ret != OM_OK)
    {
        return ret;
    }

    return OM_OK;
}

static OmRet dji_m3508_test_runtime_init(DjiM3508TestRuntime* runtime, const BspDeviceRegistry* devices)
{
    OmRet ret = OM_OK;

    if (runtime == OM_NULL || devices == OM_NULL || devices->can1 == OM_NULL)
    {
        return OM_ERROR_NULL;
    }

    memset(runtime, 0, sizeof(*runtime));
    runtime->can_device = devices->can1;

    ret = dji_m3508_test_configure_can(runtime->can_device);
    if (ret != OM_OK)
    {
        return ret;
    }

    ret = dji_motor_bus_init(&runtime->can_bus, runtime->can_device);
    if (ret != OM_OK)
    {
        return ret;
    }

    ret = dji_motor_register(
        &runtime->can_bus,
        &runtime->motor,
        DJI_MOTOR_TYPE_C620,
        DJI_M3508_TEST_MOTOR_ID,
        DJI_CTRL_MODE_CURRENT);
    if (ret != OM_OK)
    {
        return ret;
    }

    return OM_OK;
}

static void dji_m3508_test_refresh_debug(DjiM3508TestRuntime* runtime, int16_t output_command)
{
    float angle_deg = 0.0f;
    float speed_rpm = 0.0f;
    float current_amp = 0.0f;
    float temp_deg = 0.0f;

    if (runtime == OM_NULL)
    {
        return;
    }

    angle_deg = dji_motor_get_total_angle(&runtime->motor);
    speed_rpm = dji_motor_get_velocity(&runtime->motor);
    current_amp = dji_motor_get_current(&runtime->motor);
    temp_deg = dji_motor_get_temp(&runtime->motor);

    g_dji_m3508_test_debug.output_command = output_command;
    g_dji_m3508_test_debug.angle_deg = angle_deg;
    g_dji_m3508_test_debug.speed_rpm = speed_rpm;
    g_dji_m3508_test_debug.current_amp = current_amp;
    g_dji_m3508_test_debug.temp_deg = temp_deg;

    /* 这里不直接读驱动内部回调计数，改为以“观测到反馈状态变化”近似统计反馈到达次数。
     * 对最小单电机测试足够，也避免把临时调试字段重新提升为公共接口。
     */
    if ((angle_deg != runtime->last_angle_deg) || (speed_rpm != runtime->last_speed_rpm) || (current_amp != runtime->last_current_amp) ||
        (temp_deg != runtime->last_temp_deg))
    {
        g_dji_m3508_test_debug.feedback_seen_count++;
        runtime->last_angle_deg = angle_deg;
        runtime->last_speed_rpm = speed_rpm;
        runtime->last_current_amp = current_amp;
        runtime->last_temp_deg = temp_deg;
    }
}

static void dji_m3508_test_task_entry(void* arg)
{
    DjiM3508TestRuntime* runtime = (DjiM3508TestRuntime*)arg;
    OsalTimeMs deadline_cursor_ms = 0u;
    uint32_t phase_tick = 0u;
    int16_t output_command = 0;

    while (1)
    {
        phase_tick++;

        if (((phase_tick / (DJI_M3508_TEST_STEP_INTERVAL_MS / DJI_M3508_TEST_LOOP_PERIOD_MS)) % 2u) == 0u)
        {
            output_command = DJI_M3508_TEST_OUTPUT_AMPLITUDE;
        }
        else
        {
            output_command = (int16_t)(-DJI_M3508_TEST_OUTPUT_AMPLITUDE);
        }

        dji_motor_set_output(&runtime->motor, output_command);
        dji_motor_bus_sync(&runtime->can_bus);

        g_dji_m3508_test_debug.tx_count++;
        g_dji_m3508_test_debug.loop_count++;
        dji_m3508_test_refresh_debug(runtime, output_command);

        (void)osal_delay_until(&deadline_cursor_ms, DJI_M3508_TEST_LOOP_PERIOD_MS, OM_NULL);
    }
}

OmRet dji_m3508_test_task_start(const BspDeviceRegistry* devices)
{
    static OsalThread* dji_test_task = OM_NULL;
    const OsalThreadAttr dji_test_attr = {"dji_m3508_test", 768u * OSAL_STACK_WORD_BYTES, 4u};
    OsalStatus status = OSAL_INVALID;
    OmRet ret = OM_OK;

    if (devices == OM_NULL)
    {
        return OM_ERROR_NULL;
    }

    if (dji_test_task != OM_NULL)
    {
        return OM_ERR_CONFLICT;
    }

    memset(&g_dji_m3508_test_debug, 0, sizeof(g_dji_m3508_test_debug));

    ret = dji_m3508_test_runtime_init(&g_dji_m3508_test_runtime, devices);
    if (ret != OM_OK)
    {
        return ret;
    }

    status = osal_thread_create(&dji_test_task, &dji_test_attr, dji_m3508_test_task_entry, &g_dji_m3508_test_runtime);
    if (status != OSAL_OK)
    {
        dji_test_task = OM_NULL;
        return OM_ERROR;
    }

    return OM_OK;
}
