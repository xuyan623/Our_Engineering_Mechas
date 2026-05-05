#include "task/motor_communications_task/motor_communications_task.h"

#include "driver/motor/motor.h"
#include "drivers/peripheral/can/pal_can_dev.h"
#include "module/event_bus/event_bus.h"
#include "module/system_health/system_health.h"
#include "osal/osal.h"
#include "osal/osal_config.h"
#include "osal/osal_event.h"
#include "osal/osal_time.h"
#include <string.h>

/* motor_communications_task 是当前 app 的正式电机通信 owner。
 * 它的职责边界固定为：
 * 1. 自己完成 CAN1/CAN2/USART6 的 bring-up；
 * 2. 持有各 vendor bus 与 driver 实例；
 * 3. 成为唯一调用 motor_transmit_all()/motor_receive_all() 的任务；
 * 4. 负责把各 vendor 的启动期 bring-up 收敛到统一 owner 路径。
 *
 * 这里不承载上层控制算法，也不在此处计算目标值。
 * 控制任务只写 motor 层目标缓存，真正的物理总线交互统一收敛到本任务。
 */

#define MOTOR_COMMUNICATIONS_TASK_LOOP_PERIOD_MS         (5u)
#define MOTOR_COMMUNICATIONS_TASK_STACK_WORDS            (1024u)
#define MOTOR_COMMUNICATIONS_TASK_PRIORITY               (4u)
#define MOTOR_COMMUNICATIONS_TASK_DJI_CHASSIS_COUNT      (4u)
#define MOTOR_COMMUNICATIONS_TASK_P1010B_COUNT           (2u)
#define MOTOR_COMMUNICATIONS_TASK_DAMIAO_COUNT           (6u)
#define MOTOR_COMMUNICATIONS_TASK_DJI_ROLL3_ID           (6u)
#define MOTOR_COMMUNICATIONS_TASK_GO8010_PITCH2_ID       (1u)
#define MOTOR_COMMUNICATIONS_TASK_P1010B_QUERY_PERIOD_MS         (10u)
#define MOTOR_COMMUNICATIONS_TASK_DAMIAO_CTRL_MODE_RID           (10u)
#define MOTOR_COMMUNICATIONS_TASK_DAMIAO_CTRL_MODE_MIT           (1u)
#define MOTOR_COMMUNICATIONS_TASK_DAMIAO_MODE_SETTLE_MS          (10u)

typedef struct
{
    const char* name;
    uint8_t id;
} MotorCommunicationsTaskDjiChassisConfig;

typedef struct
{
    const char* name;
    uint8_t id;
} MotorCommunicationsTaskP1010BConfig;

typedef struct
{
    const char* name;
    DamiaoMotorType type;
    uint16_t can_id;
    uint16_t master_id;
    OmBool installed;
} MotorCommunicationsTaskDamiaoConfig;

typedef struct
{
    /* 总线对象 + 电机对象按 vendor 聚合。
     * 这样正式通信任务内部可以清楚表达“谁拥有哪条物理总线”。
     */
    DJIMotorBus dji_bus;
    DJIMotorDrv dji_chassis_drivers[MOTOR_COMMUNICATIONS_TASK_DJI_CHASSIS_COUNT];
    Motor dji_chassis_motors[MOTOR_COMMUNICATIONS_TASK_DJI_CHASSIS_COUNT];
    DJIMotorDrv dji_roll3_driver;
    Motor dji_roll3_motor;

    P1010BBus p1010b_bus;
    P1010BDriver p1010b_drivers[MOTOR_COMMUNICATIONS_TASK_P1010B_COUNT];
    Motor p1010b_motors[MOTOR_COMMUNICATIONS_TASK_P1010B_COUNT];

    DamiaoMotorBus damiao_bus;
    DamiaoMotorDrv damiao_drivers[MOTOR_COMMUNICATIONS_TASK_DAMIAO_COUNT];
    Motor damiao_motors[MOTOR_COMMUNICATIONS_TASK_DAMIAO_COUNT];

    Go8010Bus go8010_bus;
    Go8010MotorDrv go8010_pitch2_driver;
    Motor go8010_pitch2_motor;

    /* 事件订阅只用来做“有请求就尽快跑一轮”的唤醒提示。
     * 本任务仍保持固定周期轮询，因此即使没有事件，也会继续维护发送与接收刷新。
     */
    EventSubscription tx_request_subscription;
    OsalTimeMs last_p1010b_query_ms;
    uint32_t next_p1010b_query_index;
    int32_t p1010b_last_query_ret[MOTOR_COMMUNICATIONS_TASK_P1010B_COUNT];
    OsalTimeMs p1010b_last_query_ok_ms[MOTOR_COMMUNICATIONS_TASK_P1010B_COUNT];
} MotorCommunicationsTaskRuntime;

static const MotorCommunicationsTaskDjiChassisConfig g_dji_chassis_configs[MOTOR_COMMUNICATIONS_TASK_DJI_CHASSIS_COUNT] = {
    {.name = "chassis_fr", .id = 1u},
    {.name = "chassis_fl", .id = 2u},
    {.name = "chassis_bl", .id = 3u},
    {.name = "chassis_br", .id = 4u},
};

static const MotorCommunicationsTaskP1010BConfig g_p1010b_configs[MOTOR_COMMUNICATIONS_TASK_P1010B_COUNT] = {
    {.name = "joint_leg_r", .id = 1u},
    {.name = "joint_leg_l", .id = 2u},
};

static const MotorCommunicationsTaskDamiaoConfig g_damiao_configs[MOTOR_COMMUNICATIONS_TASK_DAMIAO_COUNT] = {
    {.name = "big_yaw", .type = DAMIAO_MOTOR_TYPE_DM4340, .can_id = 0x00u, .master_id = 0x10u, .installed = OM_TRUE},
    {.name = "pitch1", .type = DAMIAO_MOTOR_TYPE_DM10010L, .can_id = 0x01u, .master_id = 0x11u, .installed = OM_TRUE},
    {.name = "roll1", .type = DAMIAO_MOTOR_TYPE_DM4340, .can_id = 0x02u, .master_id = 0x12u, .installed = OM_FALSE},
    {.name = "roll2", .type = DAMIAO_MOTOR_TYPE_DM4310, .can_id = 0x03u, .master_id = 0x13u, .installed = OM_TRUE},
    {.name = "grip", .type = DAMIAO_MOTOR_TYPE_DM4310, .can_id = 0x04u, .master_id = 0x14u, .installed = OM_TRUE},
    {.name = "pitch3", .type = DAMIAO_MOTOR_TYPE_DM4310, .can_id = 0x05u, .master_id = 0x15u, .installed = OM_TRUE},
};

static MotorCommunicationsTaskRuntime g_motor_communications_task_runtime = {0};

/* 本任务拥有 CAN owner bring-up，因此这里负责：
 * - open
 * - 配置波特率
 * - 打开 RX/TX 中断模式
 *
 * 不在 BSP 层预开 CAN，保持 owner-based initialization 的边界。
 */
static OmRet motor_communications_task_prepare_can(Device* can_device)
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

/* 物理总线启动动作与前面的 open/config 分开，便于总线对象先完成 register/filter 分配，
 * 再统一进入 START。
 */
static OmRet motor_communications_task_start_can(Device* can_device)
{
    if (can_device == OM_NULL)
    {
        return OM_ERROR_NULL;
    }

    return device_ctrl(can_device, CAN_CMD_START, OM_NULL);
}

/* DJI 只在正式通信任务里作为 CAN1 上的一个 vendor bus 被接入。
 * 上层任务不直接碰 DJI 的总线对象。
 */
static OmRet motor_communications_task_register_dji(MotorCommunicationsTaskRuntime* runtime)
{
    uint32_t index = 0u;
    OmRet ret = OM_OK;

    if (runtime == OM_NULL)
    {
        return OM_ERROR_NULL;
    }

    for (index = 0u; index < MOTOR_COMMUNICATIONS_TASK_DJI_CHASSIS_COUNT; index++)
    {
        ret = motor_attach_dji(
            &runtime->dji_chassis_motors[index],
            g_dji_chassis_configs[index].name,
            &runtime->dji_bus,
            &runtime->dji_chassis_drivers[index],
            DJI_MOTOR_TYPE_C620,
            g_dji_chassis_configs[index].id,
            DJI_CTRL_MODE_CURRENT,
            MOTOR_CONTROL_MODE_DISABLED);
        if (ret != OM_OK)
        {
            return ret;
        }

        ret = motor_recovery_register_entry(
            g_dji_chassis_configs[index].name,
            MOTOR_VENDOR_DJI,
            &runtime->dji_chassis_motors[index],
            &runtime->dji_chassis_drivers[index]);
        if (ret != OM_OK)
        {
            return ret;
        }
    }

    ret = motor_attach_dji(
        &runtime->dji_roll3_motor,
        "roll3",
        &runtime->dji_bus,
        &runtime->dji_roll3_driver,
        DJI_MOTOR_TYPE_GM6020,
        MOTOR_COMMUNICATIONS_TASK_DJI_ROLL3_ID,
        DJI_CTRL_MODE_CURRENT,
        MOTOR_CONTROL_MODE_DISABLED);
    if (ret != OM_OK)
    {
        return ret;
    }

    return motor_recovery_register_entry(
        "roll3",
        MOTOR_VENDOR_DJI,
        &runtime->dji_roll3_motor,
        &runtime->dji_roll3_driver);
}

/* P1010B 注册阶段只完成：
 * - bus/driver/motor 对象绑定
 * - 默认配置写入
 * - 默认主动上报配置写入
 *
 * 真正的 disable/set_mode/set_active_report/enable 由后面的启动期 prepare 执行。
 */
static OmRet motor_communications_task_register_p1010b(MotorCommunicationsTaskRuntime* runtime)
{
    uint32_t index = 0u;
    OmRet ret = OM_OK;

    if (runtime == OM_NULL)
    {
        return OM_ERROR_NULL;
    }

    for (index = 0u; index < MOTOR_COMMUNICATIONS_TASK_P1010B_COUNT; index++)
    {
        ret = motor_attach_p1010b(
            &runtime->p1010b_motors[index],
            g_p1010b_configs[index].name,
            &runtime->p1010b_bus,
            &runtime->p1010b_drivers[index],
            g_p1010b_configs[index].id,
            P1010B_MODE_CURRENT,
            MOTOR_CONTROL_MODE_DISABLED);
        if (ret != OM_OK)
        {
            return ret;
        }

        motor_recovery_configure_p1010b_driver(&runtime->p1010b_drivers[index]);

        ret = motor_recovery_register_entry(
            g_p1010b_configs[index].name,
            MOTOR_VENDOR_P1010B,
            &runtime->p1010b_motors[index],
            &runtime->p1010b_drivers[index]);
        if (ret != OM_OK)
        {
            return ret;
        }
    }

    return OM_OK;
}

/* Damiao 当前仍按统一 bus owner 接入，但默认保持 disabled。
 * 这样正式通信任务不会在上电后立刻给达妙电机下保持命令。
 */
static OmRet motor_communications_task_register_damiao(MotorCommunicationsTaskRuntime* runtime)
{
    uint32_t index = 0u;
    OmRet ret = OM_OK;

    if (runtime == OM_NULL)
    {
        return OM_ERROR_NULL;
    }

    for (index = 0u; index < MOTOR_COMMUNICATIONS_TASK_DAMIAO_COUNT; index++)
    {
        ret = motor_attach_damiao(
            &runtime->damiao_motors[index],
            g_damiao_configs[index].name,
            &runtime->damiao_bus,
            &runtime->damiao_drivers[index],
            g_damiao_configs[index].type,
            g_damiao_configs[index].can_id,
            g_damiao_configs[index].master_id,
            MOTOR_CONTROL_MODE_DISABLED);
        if (ret != OM_OK)
        {
            return ret;
        }

        if (g_damiao_configs[index].installed != OM_TRUE)
        {
            continue;
        }

        ret = motor_recovery_register_entry(
            g_damiao_configs[index].name,
            MOTOR_VENDOR_DAMIAO,
            &runtime->damiao_motors[index],
            &runtime->damiao_drivers[index]);
        if (ret != OM_OK)
        {
            return ret;
        }
    }

    return OM_OK;
}

/* GO8010 通过 USART6 直接挂到正式通信任务，不走 serial_dispatch。 */
static OmRet motor_communications_task_register_go8010(MotorCommunicationsTaskRuntime* runtime)
{
    if (runtime == OM_NULL)
    {
        return OM_ERROR_NULL;
    }

    OmRet ret = motor_attach_go8010(
        &runtime->go8010_pitch2_motor,
        "pitch2",
        &runtime->go8010_bus,
        &runtime->go8010_pitch2_driver,
        MOTOR_COMMUNICATIONS_TASK_GO8010_PITCH2_ID,
        MOTOR_CONTROL_MODE_DISABLED);
    if (ret != OM_OK)
    {
        return ret;
    }

    return motor_recovery_register_entry(
        "pitch2",
        MOTOR_VENDOR_GO8010,
        &runtime->go8010_pitch2_motor,
        &runtime->go8010_pitch2_driver);
}

/* P1010B 的恢复动作固定为标准四步：
 * disable -> set_mode -> set_active_report -> enable
 *
 * 启动期直接走这一 helper；运行期恢复则复用同样的原子步骤，但由统一恢复表分步推进。
 */
static OmRet motor_communications_task_prepare_p1010b_driver(P1010BDriver* driver)
{
    P1010BResponse response = {0};
    OmRet ret = OM_OK;

    if (driver == OM_NULL)
    {
        return OM_ERROR_NULL;
    }

    motor_recovery_configure_p1010b_driver(driver);

    ret = p1010b_disable(driver, 0u, &response);
    if (ret != OM_OK)
    {
        return ret;
    }

    ret = p1010b_set_mode(driver, driver->config.defaultMode, 0u, &response);
    if (ret != OM_OK)
    {
        return ret;
    }

    ret = p1010b_set_active_report(
        driver,
        &driver->runtime.activeReport,
        0u,
        &response);
    if (ret != OM_OK)
    {
        return ret;
    }

    ret = p1010b_enable(driver, 0u, &response);
    if (ret == OM_OK)
    {
        motor_recovery_notify_p1010b_enabled(driver);
    }

    return ret;
}

/* 启动期对两台 P1010B 都尝试做一次完整 bring-up。
 * 任务启动本身不因为单台电机缺席而失败；后续由统一恢复表继续兜底。
 */
static OmRet motor_communications_task_prepare_p1010b(MotorCommunicationsTaskRuntime* runtime)
{
    uint32_t index = 0u;
    OmRet last_error = OM_OK;

    if (runtime == OM_NULL)
    {
        return OM_ERROR_NULL;
    }

    for (index = 0u; index < MOTOR_COMMUNICATIONS_TASK_P1010B_COUNT; index++)
    {
        OmRet ret = motor_communications_task_prepare_p1010b_driver(&runtime->p1010b_drivers[index]);
        if (ret != OM_OK)
        {
            last_error = ret;
        }
    }

    return last_error;
}

/* 达妙反馈要持续刷新，就必须先进入协议使能态。
 * 这里启动期只做一次 enable；后续 disabled 态的零增益 MIT 空闲帧由统一发送路径维持。
 */
static OmRet motor_communications_task_prepare_damiao(MotorCommunicationsTaskRuntime* runtime)
{
    uint32_t index = 0u;
    OmRet last_error = OM_OK;
    OmRet ret = OM_OK;

    if (runtime == OM_NULL)
    {
        return OM_ERROR_NULL;
    }

    for (index = 0u; index < MOTOR_COMMUNICATIONS_TASK_DAMIAO_COUNT; index++)
    {
        if (g_damiao_configs[index].installed != OM_TRUE)
        {
            continue;
        }

        ret = damiao_motor_write_register_u32(
            runtime->damiao_bus.canDev,
            g_damiao_configs[index].can_id,
            MOTOR_COMMUNICATIONS_TASK_DAMIAO_CTRL_MODE_RID,
            MOTOR_COMMUNICATIONS_TASK_DAMIAO_CTRL_MODE_MIT);
        if (ret != OM_OK)
        {
            last_error = ret;
        }
    }

    osal_sleep_ms(MOTOR_COMMUNICATIONS_TASK_DAMIAO_MODE_SETTLE_MS);

    for (index = 0u; index < MOTOR_COMMUNICATIONS_TASK_DAMIAO_COUNT; index++)
    {
        if (g_damiao_configs[index].installed != OM_TRUE)
        {
            continue;
        }

        damiao_motor_enable(&runtime->damiao_drivers[index]);
        motor_recovery_notify_damiao_enabled(&runtime->damiao_motors[index]);
    }

    damiao_motor_bus_sync(&runtime->damiao_bus);
    return last_error;
}

/* P1010B 正式任务回到仓库里已经验证过的 query-mode 路径：
 * - activeReport.enable = false
 * - 周期用 active_query 拉反馈
 * 这样不依赖电机主动上报链，也更贴近现有左腿测试任务。
 */
static void motor_communications_task_write_p1010b_query_feedback(
    P1010BDriver* driver,
    const P1010BResponse* response)
{
    if (driver == OM_NULL || response == OM_NULL)
    {
        return;
    }

    driver->telemetry.feedback.absolutePosition =
        (uint16_t)response->data.activeQueryValues[0];
    driver->telemetry.feedback.speedRpm =
        ((float)response->data.activeQueryValues[1]) / 10.0f;
    driver->telemetry.feedback.iqAmpere =
        ((float)response->data.activeQueryValues[2]) / 100.0f;
    driver->telemetry.feedback.busVoltage =
        ((float)response->data.activeQueryValues[3]) / 10.0f;
    driver->telemetry.feedback.timestampMs = response->timestampMs;
    driver->telemetry.lastSuccessRxTimestampMs = response->timestampMs;
    driver->telemetry.online = true;
}

static void motor_communications_task_query_one_p1010b(MotorCommunicationsTaskRuntime* runtime)
{
    P1010BDriver* driver = OM_NULL;
    P1010BResponse response = {0};
    OsalTimeMs now_ms = 0u;
    OsalTimeMs query_ok_ms = 0u;
    uint32_t index = 0u;
    OmRet query_ret = OM_ERROR_EMPTY;

    if (runtime == OM_NULL)
    {
        return;
    }

    now_ms = osal_time_now_monotonic();
    if ((uint32_t)(now_ms - runtime->last_p1010b_query_ms) <
        MOTOR_COMMUNICATIONS_TASK_P1010B_QUERY_PERIOD_MS)
    {
        return;
    }

    index = runtime->next_p1010b_query_index;
    runtime->next_p1010b_query_index =
        (runtime->next_p1010b_query_index + 1u) % MOTOR_COMMUNICATIONS_TASK_P1010B_COUNT;
    runtime->last_p1010b_query_ms = now_ms;
    driver = &runtime->p1010b_drivers[index];

    if (driver->runtime.state != P1010B_STATE_ENABLED)
    {
        runtime->p1010b_last_query_ret[index] = OM_ERROR_BUSY;
        return;
    }

    query_ret = p1010b_active_query_slots(
        driver,
        P1010B_REPORT_DATA_ABSOLUTE_POSITION,
        P1010B_REPORT_DATA_SPEED_RPM,
        P1010B_REPORT_DATA_IQ_AMPERE,
        P1010B_REPORT_DATA_BUS_VOLTAGE,
        0u,
        &response);
    runtime->p1010b_last_query_ret[index] = query_ret;
    if (query_ret == OM_OK)
    {
        query_ok_ms = osal_time_now_monotonic();
        runtime->p1010b_last_query_ok_ms[index] = query_ok_ms;
        motor_recovery_notify_p1010b_query_ok(driver, query_ok_ms);
        motor_communications_task_write_p1010b_query_feedback(driver, &response);
    }
}

/* 正式通信任务启动顺序：
 * 1. bring-up CAN1/CAN2
 * 2. 初始化各 vendor bus
 * 3. 注册各 motor/driver
 * 4. START 物理总线
 * 5. 做首轮 P1010B / Damiao bring-up
 * 6. 订阅 EVT_MOTOR_TX_REQUEST
 *
 * 这里是本任务最核心的 owner 接线点。
 */
static OmRet motor_communications_task_runtime_init(
    MotorCommunicationsTaskRuntime* runtime,
    const BspDeviceRegistry* devices)
{
    OmRet ret = OM_OK;

    if (runtime == OM_NULL || devices == OM_NULL || devices->can1 == OM_NULL || devices->can2 == OM_NULL ||
        devices->usart6 == OM_NULL)
    {
        return OM_ERROR_NULL;
    }

    memset(runtime, 0, sizeof(*runtime));
    runtime->p1010b_last_query_ret[0] = OM_ERROR_EMPTY;
    runtime->p1010b_last_query_ret[1] = OM_ERROR_EMPTY;
    motor_recovery_reset();

    ret = motor_communications_task_prepare_can(devices->can1);
    if (ret != OM_OK)
    {
        return ret;
    }

    ret = motor_communications_task_prepare_can(devices->can2);
    if (ret != OM_OK)
    {
        return ret;
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

    ret = go8010_init(&runtime->go8010_bus, devices->usart6);
    if (ret != OM_OK)
    {
        return ret;
    }

    ret = motor_communications_task_register_dji(runtime);
    if (ret != OM_OK)
    {
        return ret;
    }

    ret = motor_communications_task_register_p1010b(runtime);
    if (ret != OM_OK)
    {
        return ret;
    }

    ret = motor_communications_task_register_damiao(runtime);
    if (ret != OM_OK)
    {
        return ret;
    }

    ret = motor_communications_task_register_go8010(runtime);
    if (ret != OM_OK)
    {
        return ret;
    }

    ret = motor_communications_task_start_can(devices->can1);
    if (ret != OM_OK)
    {
        return ret;
    }

    ret = motor_communications_task_start_can(devices->can2);
    if (ret != OM_OK)
    {
        return ret;
    }

    /* Per-motor bring-up depends on each motor replying. A missing motor must
     * be reflected as offline feedback, not block the bus owner task startup.
     */
    (void)motor_communications_task_prepare_p1010b(runtime);
    (void)motor_communications_task_prepare_damiao(runtime);
    motor_recovery_arm_initial_grace();

    if (event_bus_subscribe(&g_event_bus, EVT_MOTOR_TX_REQUEST, &runtime->tx_request_subscription) != OSAL_OK)
    {
        return OM_ERROR;
    }

    return OM_OK;
}

/* 主循环职责非常固定：
 * 1. 等待事件或周期超时
 * 2. 先打一拍心跳，避免后面同步调用过长时直接触发 timeout
 * 3. 轮询一台 P1010B 的 active_query
 * 4. 统一发送所有 vendor
 * 5. 统一刷新所有 vendor 反馈
 * 6. 若收到新反馈，发布 EVT_MOTOR_FEEDBACK_READY
 * 7. 基于最新反馈推进恢复模块与 runtime fault
 * 8. 再打一拍心跳，表示本轮物理通信已完整走完
 */
static void motor_communications_task_entry(void* arg)
{
    MotorCommunicationsTaskRuntime* runtime = (MotorCommunicationsTaskRuntime*)arg;
    OsalStatus wait_status = OSAL_INVALID;

    if (runtime == OM_NULL)
    {
        for (;;)
        {
            (void)osal_sleep_ms(1000u);
        }
    }

    while (1)
    {
        wait_status = osal_event_flags_wait(
            runtime->tx_request_subscription.flags,
            runtime->tx_request_subscription.waitMask,
            OM_NULL,
            MOTOR_COMMUNICATIONS_TASK_LOOP_PERIOD_MS,
            0u);

        (void)wait_status;
        (void)system_health_beat(SYSTEM_HEALTH_TASK_MOTOR_COMMUNICATIONS);
        motor_communications_task_query_one_p1010b(runtime);
        (void)motor_transmit_all();

        if (motor_receive_all() == OM_OK)
        {
            (void)event_bus_publish(&g_event_bus, EVT_MOTOR_FEEDBACK_READY);
        }
        motor_recovery_tick();

        (void)system_health_beat(SYSTEM_HEALTH_TASK_MOTOR_COMMUNICATIONS);
    }
}

/* 任务启动入口只负责：
 * - 防重入
 * - 调 runtime_init 完成 owner 接线
 * - 创建正式通信线程
 */
OmRet motor_communications_task_start(const BspDeviceRegistry* devices)
{
    static OsalThread* motor_communications_task = OM_NULL;
    const OsalThreadAttr motor_communications_task_attr = {
        "motor_comm",
        MOTOR_COMMUNICATIONS_TASK_STACK_WORDS * OSAL_STACK_WORD_BYTES,
        MOTOR_COMMUNICATIONS_TASK_PRIORITY};
    OsalStatus status = OSAL_INVALID;
    OmRet ret = OM_OK;

    if (devices == OM_NULL)
    {
        return OM_ERROR_NULL;
    }

    if (motor_communications_task != OM_NULL)
    {
        return OM_ERR_CONFLICT;
    }

    ret = motor_communications_task_runtime_init(&g_motor_communications_task_runtime, devices);
    if (ret != OM_OK)
    {
        return ret;
    }

    status = osal_thread_create(
        &motor_communications_task,
        &motor_communications_task_attr,
        motor_communications_task_entry,
        &g_motor_communications_task_runtime);
    if (status != OSAL_OK)
    {
        motor_communications_task = OM_NULL;
        return OM_ERROR;
    }

    return OM_OK;
}

OmRet motor_communications_task_copy_recovery_snapshots(
    MotorRecoverySnapshot* snapshots,
    uint32_t capacity,
    uint32_t* snapshot_count)
{
    return motor_recovery_copy_snapshots(snapshots, capacity, snapshot_count);
}

OmRet motor_communications_task_copy_p1010b_predicate_snapshots(
    MotorRecoveryP1010BPredicateSnapshot* snapshots,
    uint32_t capacity,
    uint32_t* snapshot_count)
{
    return motor_recovery_copy_p1010b_predicate_snapshots(
        snapshots,
        capacity,
        snapshot_count);
}

OmRet motor_communications_task_copy_p1010b_diag_snapshots(
    MotorCommunicationsTaskP1010BDiagSnapshot* snapshots,
    uint32_t capacity,
    uint32_t* snapshot_count)
{
    uint32_t index = 0u;
    OsalTimeMs now_ms = 0u;

    if (snapshots == OM_NULL || snapshot_count == OM_NULL)
    {
        return OM_ERROR_NULL;
    }

    *snapshot_count = 0u;
    now_ms = osal_time_now_monotonic();

    for (index = 0u; index < MOTOR_COMMUNICATIONS_TASK_P1010B_COUNT; index++)
    {
        uint32_t last_query_ok_ms =
            g_motor_communications_task_runtime.p1010b_last_query_ok_ms[index];
        uint32_t query_ok_age_ms = 0u;

        if (*snapshot_count >= capacity)
        {
            break;
        }

        if (last_query_ok_ms != 0u)
        {
            query_ok_age_ms = (uint32_t)(now_ms - last_query_ok_ms);
        }

        snapshots[*snapshot_count].name = g_p1010b_configs[index].name;
        snapshots[*snapshot_count].last_query_ok_ms = last_query_ok_ms;
        snapshots[*snapshot_count].query_ok_age_ms = query_ok_age_ms;
        snapshots[*snapshot_count].last_query_ret =
            g_motor_communications_task_runtime.p1010b_last_query_ret[index];
        (*snapshot_count)++;
    }

    return OM_OK;
}

OmRet motor_communications_task_copy_damiao_diag(
    MotorCommunicationsTaskDamiaoDiagSnapshot* snapshot)
{
    uint32_t index = 0u;

    if (snapshot == OM_NULL)
    {
        return OM_ERROR_NULL;
    }

    memset(snapshot, 0, sizeof(*snapshot));
    snapshot->raw_rx_count =
        damiao_motor_bus_get_raw_rx_count(&g_motor_communications_task_runtime.damiao_bus);
    snapshot->last_raw_stdid =
        damiao_motor_bus_get_last_raw_stdid(&g_motor_communications_task_runtime.damiao_bus);

    for (index = 0u; index < MOTOR_COMMUNICATIONS_TASK_DAMIAO_COUNT; index++)
    {
        snapshot->feedback_sequence[index] =
            damiao_motor_get_feedback_sequence(&g_motor_communications_task_runtime.damiao_drivers[index]);
    }

    return OM_OK;
}
