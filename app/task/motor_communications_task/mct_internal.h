#ifndef NEW_ROBOT_MCT_INTERNAL_H
#define NEW_ROBOT_MCT_INTERNAL_H

#include "driver/motor/motor.h"
#include "module/event_bus/event_bus.h"
#include "task/motor_communications_task/mct.h"
#include <stdint.h>

/* mct 的固定调度参数。
 * 这里集中放 owner 任务节拍、设备数量和 vendor 常量，
 * 供 runtime/vendor/diag 三个实现文件共享。
 */
#define MCT_LOOP_PERIOD_MS                 (5u)
#define MCT_STACK_WORDS                    (1024u)
#define MCT_PRIORITY                       (4u)
#define MCT_DJI_CHASSIS_COUNT              (4u)
#define MCT_P1010B_COUNT                   (2u)
#define MCT_DAMIAO_COUNT                   (6u)
#define MCT_DJI_ROLL3_ID                   (5u)
#define MCT_GO8010_PITCH2_ID               (1u)
#define MCT_P1010B_QUERY_PERIOD_MS         (10u)
#define MCT_DAMIAO_CTRL_MODE_RID           (10u)
#define MCT_DAMIAO_CTRL_MODE_MIT           (1u)
#define MCT_DAMIAO_MODE_SETTLE_MS          (10u)

/* 下列配置表只描述“正式电机命名 -> vendor 内部 id”的静态事实，
 * 不承载运行时状态。
 */
typedef struct
{
    const char* name;
    uint8_t id;
} MctDjiChassisConfig;

typedef struct
{
    const char* name;
    uint8_t id;
} MctP1010BConfig;

typedef struct
{
    const char* name;
    DamiaoMotorType type;
    uint16_t can_id;
    uint16_t master_id;
    OmBool installed;
} MctDamiaoConfig;

/* mct 的运行时上下文。
 * 职责边界固定为：
 * - 持有各 vendor bus / driver / motor 实例
 * - 持有事件订阅句柄
 * - 持有 P1010B query-mode 的调度与最小诊断状态
 *
 * 上层控制目标不放在这里，仍由 chassis_task / arm_task 写入 motor 抽象层。
 */
typedef struct
{
    DJIMotorBus dji_bus;
    DJIMotorDrv dji_chassis_drivers[MCT_DJI_CHASSIS_COUNT];
    Motor dji_chassis_motors[MCT_DJI_CHASSIS_COUNT];
    DJIMotorDrv dji_roll3_driver;
    Motor dji_roll3_motor;

    P1010BBus p1010b_bus;
    P1010BDriver p1010b_drivers[MCT_P1010B_COUNT];
    Motor p1010b_motors[MCT_P1010B_COUNT];

    DamiaoMotorBus damiao_bus;
    DamiaoMotorDrv damiao_drivers[MCT_DAMIAO_COUNT];
    Motor damiao_motors[MCT_DAMIAO_COUNT];

    Go8010Bus go8010_bus;
    Go8010MotorDrv go8010_pitch2_driver;
    Motor go8010_pitch2_motor;

    EventSubscription tx_request_subscription;
    OsalTimeMs last_p1010b_query_ms;
    uint32_t next_p1010b_query_index;
    int32_t p1010b_last_query_ret[MCT_P1010B_COUNT];
    OsalTimeMs p1010b_last_query_ok_ms[MCT_P1010B_COUNT];
    uint32_t last_tx_request_sources_mask;
    OmBool last_tx_request_overflowed;
} MctRuntime;

extern const MctDjiChassisConfig g_mct_dji_chassis_configs[MCT_DJI_CHASSIS_COUNT];
extern const MctP1010BConfig g_mct_p1010b_configs[MCT_P1010B_COUNT];
extern const MctDamiaoConfig g_mct_damiao_configs[MCT_DAMIAO_COUNT];
extern MctRuntime g_mct_runtime;

/* owner 接线：
 * - bring-up CAN / USART6
 * - init vendor bus
 * - register motors
 * - start physical bus
 * - 做启动期 vendor prepare
 * - 订阅 EVT_MOTOR_TX_REQUEST
 */
OmRet mct_runtime_init(
    MctRuntime* runtime,
    const BspDeviceRegistry* devices);

/* GO8010 零位捕获：
 * 首个有效反馈到来后，在 owner 侧锁存初始零位，供 arm_task 只读消费。
 */
void mct_capture_go8010_zero(MctRuntime* runtime);

/* vendor 注册阶段：
 * 只做 bus/driver/motor 对象绑定与 recovery entry 注册，
 * 不做依赖电机应答的启动期动作。
 */
OmRet mct_register_vendors(MctRuntime* runtime);

/* 启动期 bring-up：
 * - P1010B：固定四步
 * - Damiao：写 MIT 模式后 enable
 */
OmRet mct_prepare_startup_motors(MctRuntime* runtime);

/* 正式链里的 P1010B query-mode 轮询入口。 */
void mct_query_one_p1010b(MctRuntime* runtime);

#endif
