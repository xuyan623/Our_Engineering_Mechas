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
    {.name = APP_MN_CHASSIS_FR, .id = APP_MDJI_ID_CHASSIS_FR, .profile_role = APP_MR_CHASSIS_FR},
    {.name = APP_MN_CHASSIS_FL, .id = APP_MDJI_ID_CHASSIS_FL, .profile_role = APP_MR_CHASSIS_FL},
    {.name = APP_MN_CHASSIS_BL, .id = APP_MDJI_ID_CHASSIS_BL, .profile_role = APP_MR_CHASSIS_BL},
    {.name = APP_MN_CHASSIS_BR, .id = APP_MDJI_ID_CHASSIS_BR, .profile_role = APP_MR_CHASSIS_BR},
};

const MctP1010BConfig g_mct_p1010b_configs[MCT_P1010B_COUNT] = {
    {.name = APP_MN_JOINT_LEG_R, .id = APP_MP10_ID_JOINT_LEG_R, .profile_role = APP_MR_JOINT_LEG_R},
    {.name = APP_MN_JOINT_LEG_L, .id = APP_MP10_ID_JOINT_LEG_L, .profile_role = APP_MR_JOINT_LEG_L},
};

const MctDamiaoConfig g_mct_damiao_configs[MCT_DAMIAO_COUNT] = {
    {.name = APP_MN_BIG_YAW, .type = DAMIAO_MOTOR_TYPE_DM4340, .can_id = APP_MD_CAN_ID_BIG_YAW, .master_id = APP_MD_MASTER_ID_BIG_YAW, .profile_role = APP_MR_BIG_YAW},
    {.name = APP_MN_PITCH1, .type = DAMIAO_MOTOR_TYPE_DM10010L, .can_id = APP_MD_CAN_ID_PITCH1, .master_id = APP_MD_MASTER_ID_PITCH1, .profile_role = APP_MR_PITCH1},
    {.name = APP_MN_ROLL1, .type = DAMIAO_MOTOR_TYPE_DM4340, .can_id = APP_MD_CAN_ID_ROLL1, .master_id = APP_MD_MASTER_ID_ROLL1, .profile_role = APP_MR_ROLL1},
    {.name = APP_MN_ROLL2, .type = DAMIAO_MOTOR_TYPE_DM4310, .can_id = APP_MD_CAN_ID_ROLL2, .master_id = APP_MD_MASTER_ID_ROLL2, .profile_role = APP_MR_ROLL2},
    {.name = APP_MN_GRIP, .type = DAMIAO_MOTOR_TYPE_DM4310, .can_id = APP_MD_CAN_ID_GRIP, .master_id = APP_MD_MASTER_ID_GRIP, .profile_role = APP_MR_GRIP},
    {.name = APP_MN_PITCH3, .type = DAMIAO_MOTOR_TYPE_DM4310, .can_id = APP_MD_CAN_ID_PITCH3, .master_id = APP_MD_MASTER_ID_PITCH3, .profile_role = APP_MR_PITCH3},
};

TaskContextSlotId g_mct_slot_id = 0;

static OmBool mct_profile_role_is_present(uint8_t profile_role)
{
    return app_motor_role_is_present(profile_role);
}

static OmBool mct_dji_roll3_is_present(void)
{
    return app_motor_role_is_present(APP_MR_ROLL3);
}

static OmBool mct_go8010_pitch2_is_present(void)
{
    return app_motor_role_is_present(APP_MR_PITCH2);
}

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
        runtime->damiao_idle_dis_seq_base[index] = 0u;
    }
}

static void mct_runtime_clear_tx_state(void)
{
    (void)mtx_drain();
    (void)mtx_take_overflow();
}

static void mct_runtime_set_control_mode(
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
        if (mct_profile_role_is_present(g_mct_dji_chassis_configs[index].profile_role) != OM_TRUE)
        {
            continue;
        }
        (void)motor_set_control_mode(&runtime->dji_chassis_motors[index], control_mode);
    }
    if (mct_dji_roll3_is_present() == OM_TRUE)
    {
        (void)motor_set_control_mode(&runtime->dji_roll3_motor, control_mode);
    }

    for (index = 0u; index < MCT_P1010B_COUNT; index++)
    {
        if (mct_profile_role_is_present(g_mct_p1010b_configs[index].profile_role) != OM_TRUE)
        {
            continue;
        }
        (void)motor_set_control_mode(&runtime->p1010b_motors[index], control_mode);
    }

    for (index = 0u; index < MCT_DAMIAO_COUNT; index++)
    {
        if (mct_profile_role_is_present(g_mct_damiao_configs[index].profile_role) != OM_TRUE)
        {
            continue;
        }
        (void)motor_set_control_mode(&runtime->damiao_motors[index], control_mode);
    }

    if (mct_go8010_pitch2_is_present() == OM_TRUE)
    {
        (void)motor_set_control_mode(&runtime->go8010_pitch2_motor, control_mode);
    }
}

static OmRet mct_runtime_prepare_devices(const BspDeviceRegistry* devices)
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

static OmRet mct_runtime_init_buses(
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

static OmRet mct_runtime_start_devices(const BspDeviceRegistry* devices)
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

static void mct_runtime_idle_confirm(MctRuntime* runtime)
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

        if (mct_profile_role_is_present(g_mct_p1010b_configs[index].profile_role) != OM_TRUE)
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
        if (mct_profile_role_is_present(g_mct_damiao_configs[index].profile_role) != OM_TRUE ||
            runtime->damiao_non_operational_disable_confirmed[index] == OM_TRUE)
        {
            continue;
        }

        if (dm_fb_seq(&runtime->damiao_drivers[index]) >
                runtime->damiao_idle_dis_seq_base[index] &&
            damiao_motor_get_status(&runtime->damiao_drivers[index]) == 0u)
        {
            runtime->damiao_non_operational_disable_confirmed[index] = OM_TRUE;
        }
    }
}

static OmRet mct_runtime_p1010b_observe(MctRuntime* runtime)
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
            MCT_IDLE_P1010B_OBSERVE_MS)
    {
        return OM_OK;
    }

    for (attempt_index = 0u; attempt_index < MCT_P1010B_COUNT; attempt_index++)
    {
        uint32_t index =
            (uint32_t)((runtime->next_non_operational_p1010b_observation_index + attempt_index) % MCT_P1010B_COUNT);

        if (mct_profile_role_is_present(g_mct_p1010b_configs[index].profile_role) != OM_TRUE)
        {
            continue;
        }

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

static OmRet mct_runtime_idle_outputs(MctRuntime* runtime)
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
    mct_runtime_set_control_mode(runtime, MOTOR_CONTROL_MODE_DISABLED);
    mct_runtime_idle_confirm(runtime);

    for (index = 0u; index < MCT_DJI_CHASSIS_COUNT; index++)
    {
        if (mct_profile_role_is_present(g_mct_dji_chassis_configs[index].profile_role) != OM_TRUE)
        {
            continue;
        }

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

    if (mct_dji_roll3_is_present() == OM_TRUE)
    {
        ret = motor_control_compute(&runtime->dji_roll3_motor);
        if (ret != OM_OK)
        {
            last_error = ret;
        }
        else if (dji_sync_motor == OM_NULL)
        {
            dji_sync_motor = &runtime->dji_roll3_motor;
        }
    }

    if (dji_sync_motor != OM_NULL)
    {
        ret = motor_owner_sync_bus(dji_sync_motor);
        if (ret != OM_OK)
        {
            last_error = ret;
        }
    }

    if (mct_go8010_pitch2_is_present() == OM_TRUE)
    {
        ret = motor_control_compute(&runtime->go8010_pitch2_motor);
        if (ret != OM_OK)
        {
            last_error = ret;
        }
        else
        {
            go8010_sync_motor = &runtime->go8010_pitch2_motor;
        }
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
        if (mct_profile_role_is_present(g_mct_p1010b_configs[index].profile_role) != OM_TRUE)
        {
            continue;
        }

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
        if (mct_profile_role_is_present(g_mct_damiao_configs[index].profile_role) != OM_TRUE)
        {
            continue;
        }

        runtime->damiao_idle_dis_seq_base[index] =
            dm_fb_seq(&runtime->damiao_drivers[index]);
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

    ret = mct_runtime_p1010b_observe(runtime);
    if (ret != OM_OK)
    {
        last_error = ret;
    }

    if (motor_receive_all() != OM_OK)
    {
        last_error = OM_ERROR;
    }

    mct_runtime_idle_confirm(runtime);

    (void)sh_clear_runtime_fault(SH_ERR_MR_DEGRADED);
    return last_error;
}

OmRet mct_runtime_enter_active(MctRuntime* runtime)
{
    if (runtime == OM_NULL)
    {
        return OM_ERROR_NULL;
    }

    mct_runtime_reset_loop_state(runtime);
    motor_tx_dispatch_init();
    mct_runtime_clear_tx_state();
    motor_recovery_rearm_all();
    (void)mct_prepare_startup_motors(runtime);
    motor_recovery_initial_grace();
    return OM_OK;
}

OmRet mct_runtime_leave_active(MctRuntime* runtime)
{
    if (runtime == OM_NULL)
    {
        return OM_ERROR_NULL;
    }

    mct_runtime_reset_loop_state(runtime);
    mct_runtime_clear_tx_state();
    return mct_runtime_idle_outputs(runtime);
}

OmRet mct_runtime_run_idle(MctRuntime* runtime)
{
    OsalTimeMs now_ms = 0u;

    if (runtime == OM_NULL)
    {
        return OM_ERROR_NULL;
    }

    now_ms = osal_time_now_monotonic();
    if (runtime->last_non_operational_cycle_ms != 0u &&
        (uint32_t)(now_ms - runtime->last_non_operational_cycle_ms) < MCT_IDLE_PERIOD_MS)
    {
        return OM_OK;
    }

    return mct_runtime_idle_outputs(runtime);
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

    ret = mct_runtime_prepare_devices(devices);
    if (ret != OM_OK)
    {
        return ret;
    }

    /* 先 init vendor bus，再统一 register motors。 */
    ret = mct_runtime_init_buses(runtime, devices);
    if (ret != OM_OK)
    {
        return ret;
    }

    ret = mct_register_vendors(runtime);
    if (ret != OM_OK)
    {
        return ret;
    }

    ret = mct_runtime_start_devices(devices);
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
    if (runtime == OM_NULL || mct_go8010_pitch2_is_present() != OM_TRUE)
    {
        return;
    }

    /* GO8010 的零位属于设备初始化事实，只应在 owner 侧锁存一次。 */
    (void)motor_capture_initial_zero(&runtime->go8010_pitch2_motor);
}
