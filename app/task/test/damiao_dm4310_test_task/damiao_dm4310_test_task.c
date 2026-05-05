#include "task/test/damiao_dm4310_test_task/damiao_dm4310_test_task.h"

#include "driver/damiao/damiao.h"
#include "module/system_health/system_health.h"
#include "drivers/peripheral/can/pal_can_dev.h"
#include "osal/osal.h"
#include "osal/osal_config.h"
#include "osal/osal_time.h"
#include <string.h>

#define DAMIAO_DM4310_TEST_LOOP_PERIOD_MS      (5u)
#define DAMIAO_DM4310_TEST_STEP_INTERVAL_MS    (1000u)
#define DAMIAO_DM4310_TEST_CAN_ID              (0x03u)
#define DAMIAO_DM4310_TEST_MASTER_ID           (0x13u)
#define DAMIAO_DM4310_TEST_POSITION_AMPLITUDE  (1.0f)
#define DAMIAO_DM4310_TEST_KP                  (5.0f)
#define DAMIAO_DM4310_TEST_KD                  (0.0f)

typedef struct
{
    Device* can_device;
    DamiaoMotorBus can_bus;
    DamiaoMotorDrv motor;
    float last_position_rad;
    float last_velocity_rad_s;
    float last_torque_nm;
    uint8_t last_status_code;
} DamiaoDm4310TestRuntime;

static DamiaoDm4310TestRuntime g_damiao_dm4310_test_runtime = {0};
DamiaoDm4310TestDebugState g_damiao_dm4310_test_debug = {0};

static OmRet damiao_dm4310_test_configure_can(Device* can_device)
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

static OmRet damiao_dm4310_test_runtime_init(DamiaoDm4310TestRuntime* runtime, const BspDeviceRegistry* devices)
{
    OmRet ret = OM_OK;

    if (runtime == OM_NULL || devices == OM_NULL || devices->can1 == OM_NULL)
    {
        return OM_ERROR_NULL;
    }

    memset(runtime, 0, sizeof(*runtime));
    runtime->can_device = devices->can1;

    ret = damiao_dm4310_test_configure_can(runtime->can_device);
    if (ret != OM_OK)
    {
        return ret;
    }

    ret = damiao_motor_bus_init(&runtime->can_bus, runtime->can_device);
    if (ret != OM_OK)
    {
        return ret;
    }

    ret = damiao_motor_register(
        &runtime->can_bus,
        &runtime->motor,
        DAMIAO_MOTOR_TYPE_DM4310,
        DAMIAO_DM4310_TEST_CAN_ID,
        DAMIAO_DM4310_TEST_MASTER_ID);
    if (ret != OM_OK)
    {
        return ret;
    }

    damiao_motor_enable(&runtime->motor);
    damiao_motor_bus_sync(&runtime->can_bus);

    return OM_OK;
}

static void damiao_dm4310_test_refresh_debug(DamiaoDm4310TestRuntime* runtime, float target_position_rad)
{
    float position_rad = 0.0f;
    float velocity_rad_s = 0.0f;
    float torque_nm = 0.0f;
    uint8_t status_code = 0u;

    if (runtime == OM_NULL)
    {
        return;
    }

    position_rad = damiao_motor_get_position(&runtime->motor);
    velocity_rad_s = damiao_motor_get_velocity(&runtime->motor);
    torque_nm = damiao_motor_get_torque(&runtime->motor);
    status_code = damiao_motor_get_status(&runtime->motor);

    g_damiao_dm4310_test_debug.target_position_rad = target_position_rad;
    g_damiao_dm4310_test_debug.position_rad = position_rad;
    g_damiao_dm4310_test_debug.velocity_rad_s = velocity_rad_s;
    g_damiao_dm4310_test_debug.torque_nm = torque_nm;
    g_damiao_dm4310_test_debug.status_code = status_code;

    if ((position_rad != runtime->last_position_rad) || (velocity_rad_s != runtime->last_velocity_rad_s) ||
        (torque_nm != runtime->last_torque_nm) || (status_code != runtime->last_status_code))
    {
        g_damiao_dm4310_test_debug.feedback_seen_count++;
        runtime->last_position_rad = position_rad;
        runtime->last_velocity_rad_s = velocity_rad_s;
        runtime->last_torque_nm = torque_nm;
        runtime->last_status_code = status_code;
    }
}

static void damiao_dm4310_test_task_entry(void* arg)
{
    DamiaoDm4310TestRuntime* runtime = (DamiaoDm4310TestRuntime*)arg;
    OsalTimeMs deadline_cursor_ms = 0u;
    uint32_t phase_tick = 0u;
    float target_position_rad = 0.0f;

    osal_sleep_ms(50u);

    while (1)
    {
        phase_tick++;

        if (((phase_tick / (DAMIAO_DM4310_TEST_STEP_INTERVAL_MS / DAMIAO_DM4310_TEST_LOOP_PERIOD_MS)) % 2u) == 0u)
        {
            target_position_rad = DAMIAO_DM4310_TEST_POSITION_AMPLITUDE;
        }
        else
        {
            target_position_rad = -DAMIAO_DM4310_TEST_POSITION_AMPLITUDE;
        }

        damiao_motor_set_mit(&runtime->motor, target_position_rad, 0.0f, DAMIAO_DM4310_TEST_KP, DAMIAO_DM4310_TEST_KD, 0.0f);
        damiao_motor_bus_sync(&runtime->can_bus);

        g_damiao_dm4310_test_debug.tx_count++;
        g_damiao_dm4310_test_debug.loop_count++;
        damiao_dm4310_test_refresh_debug(runtime, target_position_rad);
        (void)system_health_beat(SYSTEM_HEALTH_TASK_DAMIAO_SMOKE);

        (void)osal_delay_until(&deadline_cursor_ms, DAMIAO_DM4310_TEST_LOOP_PERIOD_MS, OM_NULL);
    }
}

OmRet damiao_dm4310_test_task_start(const BspDeviceRegistry* devices)
{
    static OsalThread* damiao_test_task = OM_NULL;
    const OsalThreadAttr damiao_test_attr = {"damiao_dm4310_test", 768u * OSAL_STACK_WORD_BYTES, 4u};
    OsalStatus status = OSAL_INVALID;
    OmRet ret = OM_OK;

    if (devices == OM_NULL)
    {
        return OM_ERROR_NULL;
    }

    if (damiao_test_task != OM_NULL)
    {
        return OM_ERR_CONFLICT;
    }

    memset(&g_damiao_dm4310_test_debug, 0, sizeof(g_damiao_dm4310_test_debug));

    ret = damiao_dm4310_test_runtime_init(&g_damiao_dm4310_test_runtime, devices);
    if (ret != OM_OK)
    {
        return ret;
    }

    status = osal_thread_create(&damiao_test_task, &damiao_test_attr, damiao_dm4310_test_task_entry, &g_damiao_dm4310_test_runtime);
    if (status != OSAL_OK)
    {
        damiao_test_task = OM_NULL;
        return OM_ERROR;
    }

    return OM_OK;
}
