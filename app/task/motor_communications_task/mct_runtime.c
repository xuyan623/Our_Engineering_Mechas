#include "task/motor_communications_task/mct_internal.h"

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

MctRuntime g_mct_runtime = {0};

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

    /* 运行时状态每次启动都从干净上下文开始。 */
    memset(runtime, 0, sizeof(*runtime));
    runtime->p1010b_last_query_ret[0] = OM_ERROR_EMPTY;
    runtime->p1010b_last_query_ret[1] = OM_ERROR_EMPTY;
    motor_recovery_reset();

    ret = mct_prepare_can(devices->can1);
    if (ret != OM_OK)
    {
        return ret;
    }

    ret = mct_prepare_can(devices->can2);
    if (ret != OM_OK)
    {
        return ret;
    }

    /* 先 init vendor bus，再统一 register motors。 */
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

    ret = go8010_init(&runtime->go8010_bus, devices->usart6);
    if (ret != OM_OK)
    {
        return ret;
    }

    ret = mct_register_vendors(runtime);
    if (ret != OM_OK)
    {
        return ret;
    }

    ret = mct_start_can(devices->can1);
    if (ret != OM_OK)
    {
        return ret;
    }

    ret = mct_start_can(devices->can2);
    if (ret != OM_OK)
    {
        return ret;
    }

    /* 启动期 bring-up 失败不阻塞 owner 任务启动；
     * 缺失电机会在后续反馈/恢复链里体现为 offline。
     */
    (void)mct_prepare_startup_motors(runtime);
    motor_recovery_arm_initial_grace();

    if (event_bus_subscribe(&g_event_bus, EVT_MOTOR_TX_REQUEST, &runtime->tx_request_subscription) != OSAL_OK)
    {
        return OM_ERROR;
    }

    return OM_OK;
}

void mct_capture_go8010_zero(MctRuntime* runtime)
{
    if (runtime == OM_NULL)
    {
        return;
    }

    /* GO8010 的零位属于设备初始化事实，只应在 owner 侧锁存一次。 */
    go8010_capture_initial_position_zero(&runtime->go8010_pitch2_driver);
}
