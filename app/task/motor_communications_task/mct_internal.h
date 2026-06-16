#ifndef NEW_ROBOT_MCT_INTERNAL_H
#define NEW_ROBOT_MCT_INTERNAL_H

#include "config/app_config.h"
#include "driver/motor/motor.h"
#include "module/task_channel/task_channel.h"
#include "task/motor_communications_task/mct.h"
#include "module/task_context_pool/task_context_pool.h"
#include <stdint.h>

/* mct 的固定调度参数。
 * 这里集中放 owner 任务节拍、设备数量和 vendor 常量，
 * 供 runtime/vendor/diag 三个实现文件共享。
 */
#define MCT_LOOP_PERIOD_MS                              APP_MCT_LOOP_PERIOD_MS
#define MCT_OPERATIONAL_TX_MS      APP_MCT_OPERATIONAL_TX_MS
#define MCT_OPERATIONAL_OBSERVE_MS          APP_MCT_OPERATIONAL_OBSERVE_MS
#define MCT_IDLE_PERIOD_MS                  APP_MCT_IDLE_PERIOD_MS
#define MCT_IDLE_P1010B_OBSERVE_MS   APP_MCT_IDLE_P1010B_OBSERVE_MS
#define MCT_STACK_WORDS                             (1024u)
#define MCT_PRIORITY                                (4u)
#define MCT_DJI_CHASSIS_COUNT                       (4u)
#define MCT_P1010B_COUNT                            (2u)
#define MCT_DAMIAO_COUNT                            (6u)
#define MCT_DJI_ROLL3_ID                            APP_MDJI_ID_ROLL3
#define MCT_GO8010_PITCH2_ID                        APP_MG8_ID_PITCH2
#define MCT_DAMIAO_MODE_SETTLE_MS                   (10u)

/* 下列配置表只描述“正式电机命名 -> vendor 内部 id”的静态事实，
 * 不承载运行时状态。
 */
typedef struct
{
    const char* name;
    uint8_t id;
    uint8_t profile_role;
} MctDjiChassisConfig;

typedef struct
{
    const char* name;
    uint8_t id;
    uint8_t profile_role;
} MctP1010BConfig;

typedef struct
{
    const char* name;
    DamiaoMotorType type;
    uint16_t can_id;
    uint16_t master_id;
    uint8_t profile_role;
} MctDamiaoConfig;

/* 只允许在 mct owner 线程里执行的低频生命周期命令。 */
typedef enum
{
    MCT_OWNER_COMMAND_NONE = 0u,
    MCT_OWNER_COMMAND_ENTER_OPERATIONAL = 1u,
    MCT_OWNER_COMMAND_LEAVE_OPERATIONAL = 2u,
    MCT_OWNER_COMMAND_RESET_OPERATIONAL = 3u,
} MctOwnerCommand;

/* mct 的运行时上下文。
 * 职责边界固定为：
 * - 持有各 vendor bus / driver / motor 实例
 * - 持有 owner lifecycle / vendor runtime 状态
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

    TaskCommandMailbox owner_command_mailbox;
    OmAtomicUint operational_active;
    OsalTimeMs last_operational_formal_transmit_ms;
    OsalTimeMs last_operational_observation_ms;
    OsalTimeMs last_non_operational_cycle_ms;
    OsalTimeMs last_non_operational_p1010b_observation_ms;
    OmBool operational_formal_transmit_pending;
    OmBool p1010b_non_operational_disable_confirmed[MCT_P1010B_COUNT];
    uint8_t next_non_operational_p1010b_observation_index;
    OmBool damiao_non_operational_disable_confirmed[MCT_DAMIAO_COUNT];
    uint32_t damiao_idle_dis_seq_base[MCT_DAMIAO_COUNT];
    uint32_t last_tx_request_sources_mask;
    OmBool last_tx_request_overflowed;
} MctRuntime;

extern const MctDjiChassisConfig g_mct_dji_chassis_configs[MCT_DJI_CHASSIS_COUNT];
extern const MctP1010BConfig g_mct_p1010b_configs[MCT_P1010B_COUNT];
extern const MctDamiaoConfig g_mct_damiao_configs[MCT_DAMIAO_COUNT];
extern TaskContextSlotId g_mct_slot_id;
#define g_mct_runtime (*(MctRuntime*)task_context_pool_get_ptr(g_mct_slot_id))

/* owner 接线：
 * - bring-up CAN / USART6
 * - init vendor bus
 * - register motors
 * - start physical bus
 * - 保持 non-operational，等待 owner request
 */
OmRet mct_runtime_init(
    MctRuntime* runtime,
    const BspDeviceRegistry* devices);

/* 保留 owner 接线与电机注册不变，只把运行时重新推进到“正式可控态”：
 * - 清 owner loop 状态
 * - 重新做启动期 vendor bring-up
 * - 重新 arm recovery 宽限期
 *
 * 这是后续“遥控器触发的软件重置”要复用的核心入口。
 */
OmRet mct_runtime_enter_active(MctRuntime* runtime);

/* 保留 wiring 和注册表不变，只把正式通信 owner 退回到“安全退出”状态：
 * - 清 loop/query/dispatch 运行态
 * - 下发安全零目标或 vendor disable
 * - 清当前 recovery runtime fault
 *
 * 当前设计面向“立即重进”的软件 bring-up 复用，不是长期 disabled 模式机。
 */
OmRet mct_runtime_leave_active(MctRuntime* runtime);

/* non-operational owner 路径：
 * - 持续把正式电机保持在 disabled / safe output
 * - 不推进正常 query / receive / recovery
 */
OmRet mct_runtime_run_idle(MctRuntime* runtime);

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

#endif
