#include "task/motor_communications_task/mct_internal.h"

#include "module/motor_tx_dispatch/motor_tx_dispatch.h"
#include "module/system_health/system_health.h"
#include "drivers/peripheral/can/pal_can_dev.h"
#include <string.h>

/* 运行时文件承接两类事实：
 * 1. 静态配置表（正式命名与 vendor id 映射）
 * 2. owner wiring（CAN/USART6 bring-up 与 runtime init）
 */

const MctDjiChassisConfig g_mct_dji_chassis_configs[MCT_DJI_CHASSIS_COUNT] = {
    {.name = "chassis_fr", .id = 1u},
    {.name = "chassis_fl", .id = 2u},
    {.name = "chassis_bl", .id = 3u},
    {.name = "chassis_br", .id = 4u},
};

const MctP1010BConfig g_mct_p1010b_configs[MCT_P1010B_COUNT] = {
    {.name = "joint_leg_r", .id = 1u},
    {.name = "joint_leg_l", .id = 2u},
};

const MctDamiaoConfig g_mct_damiao_configs[MCT_DAMIAO_COUNT] = {
    {.name = "big_yaw", .type = DAMIAO_MOTOR_TYPE_DM4340, .can_id = 0x00u, .master_id = 0x10u, .installed = OM_TRUE},
    {.name = "pitch1", .type = DAMIAO_MOTOR_TYPE_DM10010L, .can_id = 0x01u, .master_id = 0x11u, .installed = OM_TRUE},
    {.name = "roll1", .type = DAMIAO_MOTOR_TYPE_DM4340, .can_id = 0x02u, .master_id = 0x12u, .installed = OM_FALSE},
    {.name = "roll2", .type = DAMIAO_MOTOR_TYPE_DM4310, .can_id = 0x03u, .master_id = 0x13u, .installed = OM_TRUE},
    {.name = "grip", .type = DAMIAO_MOTOR_TYPE_DM4310, .can_id = 0x04u, .master_id = 0x14u, .installed = OM_TRUE},
    {.name = "pitch3", .type = DAMIAO_MOTOR_TYPE_DM4310, .can_id = 0x05u, .master_id = 0x15u, .installed = OM_TRUE},
};

TaskContextSlotId g_mct_slot_id = 0;

/* owner 侧 CAN bring-up：
 * - open
 * - 配置 1Mbps
 * - 打开 RX/TX 中断模式
 *
 * 这一步不在 BSP 预开，保持 owner-based initialization 边界。
 */
static OmRet mct_prepare_can(Device* can_device)
{
    CanCfg can_cfg = CAN_DEFUALT_CFG;
    uint32_t io_type = 0u;
    OmRet ret = OM_OK;

    if (can_device == OM_NULL)
    {
        return OM_ERROR_NULL;
    }

    if (device_check_status(can_device, DEV_STATUS_OPENED) != 0u)
    {
        return OM_ERR_CONFLICT;
    }

    ret = device_open(can_device, CAN_O_INT_RX | CAN_O_INT_TX);
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
    return device_ctrl(can_device, CAN_CMD_SET_IOTYPE, &io_type);
}

static OmRet mct_start_can(Device* can_device)
{
    if (can_device == OM_NULL)
    {
        return OM_ERROR_NULL;
    }

    return device_ctrl(can_device, CAN_CMD_START, OM_NULL);
}

static void mct_runtime_reset_loop_state(MctRuntime* runtime)
{
    uint32_t index = 0u;

    if (runtime == OM_NULL)
    {
        return;
    }

    runtime->last_operational_observation_ms = 0u;
    runtime->last_operational_formal_transmit_ms = 0u;
    runtime->last_non_operational_cycle_ms = 0u;
    runtime->last_non_operational_p1010b_observation_ms = 0u;
    runtime->next_non_operational_p1010b_observation_index = 0u;
    runtime->operational_formal_transmit_pending = OM_FALSE;
    runtime->last_tx_request_sources_mask = 0u;
    runtime->last_tx_request_overflowed = OM_FALSE;

    for (index = 0u; index < MCT_P1010B_COUNT; index++)
    {
        runtime->p1010b_non_operational_disable_confirmed[index] = OM_FALSE;
    }

    for (index = 0u; index < MCT_DAMIAO_COUNT; index++)
    {
        runtime->damiao_non_operational_disable_confirmed[index] = OM_FALSE;
        runtime->damiao_non_operational_disable_sequence_base[index] = 0u;
    }
}

static void mct_runtime_clear_tx_dispatch_state(void)
{
    (void)motor_tx_dispatch_drain_sources_mask();
    (void)motor_tx_dispatch_take_overflow_flag();
}

static void mct_runtime_set_all_motors_control_mode(
    MctRuntime* runtime,
    MotorControlMode control_mode)
{
    uint32_t index = 0u;

    if (runtime == OM_NULL)
    {
        return;
    }

    for (index = 0u; index < MCT_DJI_CHASSIS_COUNT; index++)
    {
        (void)motor_set_control_mode(&runtime->dji_chassis_motors[index], control_mode);
    }
    (void)motor_set_control_mode(&runtime->dji_roll3_motor, control_mode);

    for (index = 0u; index < MCT_P1010B_COUNT; index++)
    {
        (void)motor_set_control_mode(&runtime->p1010b_motors[index], control_mode);
    }

    for (index = 0u; index < MCT_DAMIAO_COUNT; index++)
    {
        (void)motor_set_control_mode(&runtime->damiao_motors[index], control_mode);
    }

    (void)motor_set_control_mode(&runtime->go8010_pitch2_motor, control_mode);
}

static OmRet mct_runtime_prepare_owner_devices(const BspDeviceRegistry* devices)
{
    OmRet ret = OM_OK;

    if (devices == OM_NULL || devices->can1 == OM_NULL || devices->can2 == OM_NULL)
    {
        return OM_ERROR_NULL;
    }

    ret = mct_prepare_can(devices->can1);
    if (ret != OM_OK)
    {
        return ret;
    }

    return mct_prepare_can(devices->can2);
}

static OmRet mct_runtime_init_vendor_buses(
    MctRuntime* runtime,
    const BspDeviceRegistry* devices)
{
    OmRet ret = OM_OK;

    if (runtime == OM_NULL || devices == OM_NULL || devices->can1 == OM_NULL ||
        devices->can2 == OM_NULL || devices->usart6 == OM_NULL)
    {
        return OM_ERROR_NULL;
    }

    ret = dji_motor_bus_init(&runtime->dji_bus, devices->can1);
    if (ret != OM_OK)
    {
        return ret;
    }

    ret = p1010b_bus_init(&runtime->p1010b_bus, devices->can1);
    if (ret != OM_OK)
    {
        return ret;
    }

    ret = damiao_motor_bus_init(&runtime->damiao_bus, devices->can2);
    if (ret != OM_OK)
    {
        return ret;
    }

    return go8010_init(&runtime->go8010_bus, devices->usart6);
}

static OmRet mct_runtime_start_owner_devices(const BspDeviceRegistry* devices)
{
    OmRet ret = OM_OK;

    if (devices == OM_NULL || devices->can1 == OM_NULL || devices->can2 == OM_NULL)
    {
        return OM_ERROR_NULL;
    }

    ret = mct_start_can(devices->can1);
    if (ret != OM_OK)
    {
        return ret;
    }

    return mct_start_can(devices->can2);
}

static void mct_runtime_update_non_operational_disable_confirmation(MctRuntime* runtime)
{
    uint32_t index = 0u;

    if (runtime == OM_NULL)
    {
        return;
    }

    for (index = 0u; index < MCT_P1010B_COUNT; index++)
    {
        if (runtime->p1010b_non_operational_disable_confirmed[index] == OM_TRUE)
        {
            continue;
        }

        if (runtime->p1010b_drivers[index].runtime.state == P1010B_STATE_DISABLED)
        {
            runtime->p1010b_non_operational_disable_confirmed[index] = OM_TRUE;
        }
    }

    for (index = 0u; index < MCT_DAMIAO_COUNT; index++)
    {
        if (g_mct_damiao_configs[index].installed != OM_TRUE ||
            runtime->damiao_non_operational_disable_confirmed[index] == OM_TRUE)
        {
            continue;
        }

        if (damiao_motor_get_feedback_sequence(&runtime->damiao_drivers[index]) >
                runtime->damiao_non_operational_disable_sequence_base[index] &&
            damiao_motor_get_status(&runtime->damiao_drivers[index]) == 0u)
        {
            runtime->damiao_non_operational_disable_confirmed[index] = OM_TRUE;
        }
    }
}

static OmRet mct_runtime_observe_non_operational_p1010b(MctRuntime* runtime)
{
    OsalTimeMs now_ms = 0u;
    uint32_t attempt_index = 0u;

    if (runtime == OM_NULL)
    {
        return OM_ERROR_NULL;
    }

    now_ms = osal_time_now_monotonic();
    if (runtime->last_non_operational_p1010b_observation_ms != 0u &&
        (uint32_t)(now_ms - runtime->last_non_operational_p1010b_observation_ms) <
            MCT_NON_OPERATIONAL_P1010B_OBSERVE_PERIOD_MS)
    {
        return OM_OK;
    }

    for (attempt_index = 0u; attempt_index < MCT_P1010B_COUNT; attempt_index++)
    {
        uint32_t index =
            (uint32_t)((runtime->next_non_operational_p1010b_observation_index + attempt_index) % MCT_P1010B_COUNT);

        if (runtime->p1010b_non_operational_disable_confirmed[index] != OM_TRUE)
        {
            continue;
        }

        runtime->last_non_operational_p1010b_observation_ms = now_ms;
        runtime->next_non_operational_p1010b_observation_index =
            (uint8_t)((index + 1u) % MCT_P1010B_COUNT);
        return motor_owner_query_feedback(&runtime->p1010b_motors[index]);
    }

    return OM_OK;
}

static OmRet mct_runtime_apply_non_operational_outputs(MctRuntime* runtime)
{
    uint32_t index = 0u;
    OmRet ret = OM_OK;
    OmRet last_error = OM_OK;
    Motor* damiao_sync_motor = OM_NULL;
    Motor* dji_sync_motor = OM_NULL;
    Motor* go8010_sync_motor = OM_NULL;

    if (runtime == OM_NULL)
    {
        return OM_ERROR_NULL;
    }

    runtime->last_non_operational_cycle_ms = osal_time_now_monotonic();
    mct_runtime_set_all_motors_control_mode(runtime, MOTOR_CONTROL_MODE_DISABLED);
    mct_runtime_update_non_operational_disable_confirmation(runtime);

    for (index = 0u; index < MCT_DJI_CHASSIS_COUNT; index++)
    {
        ret = motor_control_compute(&runtime->dji_chassis_motors[index]);
        if (ret != OM_OK)
        {
            last_error = ret;
        }
        else if (dji_sync_motor == OM_NULL)
        {
            dji_sync_motor = &runtime->dji_chassis_motors[index];
        }
    }

    ret = motor_control_compute(&runtime->dji_roll3_motor);
    if (ret != OM_OK)
    {
        last_error = ret;
    }
    else if (dji_sync_motor == OM_NULL)
    {
        dji_sync_motor = &runtime->dji_roll3_motor;
    }

    if (dji_sync_motor != OM_NULL)
    {
        ret = motor_owner_sync_bus(dji_sync_motor);
        if (ret != OM_OK)
        {
            last_error = ret;
        }
    }

    ret = motor_control_compute(&runtime->go8010_pitch2_motor);
    if (ret != OM_OK)
    {
        last_error = ret;
    }
    else
    {
        go8010_sync_motor = &runtime->go8010_pitch2_motor;
    }

    if (go8010_sync_motor != OM_NULL)
    {
        ret = motor_owner_sync_bus(go8010_sync_motor);
        if (ret != OM_OK)
        {
            last_error = ret;
        }
    }

    go8010_rx_service(&runtime->go8010_bus);

    for (index = 0u; index < MCT_P1010B_COUNT; index++)
    {
        if (runtime->p1010b_non_operational_disable_confirmed[index] == OM_TRUE)
        {
            continue;
        }

        ret = motor_owner_disable(&runtime->p1010b_motors[index]);
        if (ret != OM_OK)
        {
            last_error = ret;
            continue;
        }

        if (runtime->p1010b_drivers[index].runtime.state == P1010B_STATE_DISABLED)
        {
            runtime->p1010b_non_operational_disable_confirmed[index] = OM_TRUE;
        }
    }

    for (index = 0u; index < MCT_DAMIAO_COUNT; index++)
    {
        if (g_mct_damiao_configs[index].installed != OM_TRUE)
        {
            continue;
        }

        runtime->damiao_non_operational_disable_sequence_base[index] =
            damiao_motor_get_feedback_sequence(&runtime->damiao_drivers[index]);
        ret = motor_owner_disable(&runtime->damiao_motors[index]);
        if (ret != OM_OK)
        {
            last_error = ret;
        }
        else if (damiao_sync_motor == OM_NULL)
        {
            damiao_sync_motor = &runtime->damiao_motors[index];
        }
    }

    if (damiao_sync_motor != OM_NULL)
    {
        ret = motor_owner_sync_bus(damiao_sync_motor);
        if (ret != OM_OK)
        {
            last_error = ret;
        }
    }

    ret = mct_runtime_observe_non_operational_p1010b(runtime);
    if (ret != OM_OK)
    {
        last_error = ret;
    }

    if (motor_receive_all() != OM_OK)
    {
        last_error = OM_ERROR;
    }

    mct_runtime_update_non_operational_disable_confirmation(runtime);

    (void)sh_clear_runtime_fault(SH_ERR_MOTOR_RECOVERY_DEGRADED);
    return last_error;
}

OmRet mct_runtime_enter_operational_state(MctRuntime* runtime)
{
    if (runtime == OM_NULL)
    {
        return OM_ERROR_NULL;
    }

    mct_runtime_reset_loop_state(runtime);
    motor_tx_dispatch_init();
    mct_runtime_clear_tx_dispatch_state();
    motor_recovery_rearm_registered_entries();
    (void)mct_prepare_startup_motors(runtime);
    motor_recovery_arm_initial_grace();
    return OM_OK;
}

OmRet mct_runtime_leave_operational_state(MctRuntime* runtime)
{
    if (runtime == OM_NULL)
    {
        return OM_ERROR_NULL;
    }

    mct_runtime_reset_loop_state(runtime);
    mct_runtime_clear_tx_dispatch_state();
    return mct_runtime_apply_non_operational_outputs(runtime);
}

OmRet mct_runtime_run_non_operational_cycle(MctRuntime* runtime)
{
    OsalTimeMs now_ms = 0u;

    if (runtime == OM_NULL)
    {
        return OM_ERROR_NULL;
    }

    now_ms = osal_time_now_monotonic();
    if (runtime->last_non_operational_cycle_ms != 0u &&
        (uint32_t)(now_ms - runtime->last_non_operational_cycle_ms) < MCT_NON_OPERATIONAL_PERIOD_MS)
    {
        return OM_OK;
    }

    return mct_runtime_apply_non_operational_outputs(runtime);
}

OmRet mct_runtime_init(
    MctRuntime* runtime,
    const BspDeviceRegistry* devices)
{
    OmRet ret = OM_OK;

    if (runtime == OM_NULL || devices == OM_NULL || devices->can1 == OM_NULL || devices->can2 == OM_NULL ||
        devices->usart6 == OM_NULL)
    {
        return OM_ERROR_NULL;
    }

    /* 冷启动时先清整个 runtime 与 recovery 注册表。 */
    memset(runtime, 0, sizeof(*runtime));
    motor_recovery_reset();

    ret = mct_runtime_prepare_owner_devices(devices);
    if (ret != OM_OK)
    {
        return ret;
    }

    /* 先 init vendor bus，再统一 register motors。 */
    ret = mct_runtime_init_vendor_buses(runtime, devices);
    if (ret != OM_OK)
    {
        return ret;
    }

    ret = mct_register_vendors(runtime);
    if (ret != OM_OK)
    {
        return ret;
    }

    ret = mct_runtime_start_owner_devices(devices);
    if (ret != OM_OK)
    {
        return ret;
    }

    ret = task_command_mailbox_init(
        &runtime->owner_command_mailbox,
        sizeof(MctOwnerCommand));
    if (ret != OM_OK)
    {
        return ret;
    }

    OM_STORE_REL(&runtime->operational_active, 0u);

    return OM_OK;
}

void mct_capture_go8010_zero(MctRuntime* runtime)
{
    if (runtime == OM_NULL)
    {
        return;
    }

    /* GO8010 的零位属于设备初始化事实，只应在 owner 侧锁存一次。 */
    (void)motor_capture_initial_zero(&runtime->go8010_pitch2_motor);
}
