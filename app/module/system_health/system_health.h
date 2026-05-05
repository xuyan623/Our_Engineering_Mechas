#ifndef NEW_ROBOT_SYSTEM_HEALTH_H
#define NEW_ROBOT_SYSTEM_HEALTH_H

#include "core/om_def.h"
#include "osal/osal_time.h"

/* 被 system_health 监控的任务类型。
 * 这些枚举值只用于内部注册与心跳管理，不参与 LED 报码。
 */
typedef enum
{
    SYSTEM_HEALTH_TASK_DAMIAO_SMOKE = 0u,
    SYSTEM_HEALTH_TASK_P1010B_LEFT_LEG_SMOKE,
    SYSTEM_HEALTH_TASK_GO8010_SMOKE,
    SYSTEM_HEALTH_TASK_SELF_TEST,
    SYSTEM_HEALTH_TASK_IMU,
    SYSTEM_HEALTH_TASK_MODE,
    SYSTEM_HEALTH_TASK_CHASSIS,
    SYSTEM_HEALTH_TASK_ARM,
    SYSTEM_HEALTH_TASK_MOTOR_COMMUNICATIONS,
    SYSTEM_HEALTH_TASK_COUNT
} SystemHealthTaskId;

/* 三段语义码直接编码进错误码本身：
 * 0x0ABC 中，A/B/C 分别对应三段闪烁次数。
 * SYSTEM_HEALTH_ERR_NONE 保留为 0，不参与三段编码。
 * 这样后续增删错误时，只需要维护这一处枚举定义，
 * system_health.c 可直接从错误码中拆出三段闪烁次数。
 */
#define SYSTEM_HEALTH_CODE(a, b, c) \
    ((((uint16_t)((a) & 0x0Fu)) << 8) | (((uint16_t)((b) & 0x0Fu)) << 4) | ((uint16_t)((c) & 0x0Fu)))

/* 从编码值中反解出三段报码。 */
#define SYSTEM_HEALTH_CODE_P1(code)   (((uint16_t)(code) >> 8) & 0x0Fu)
#define SYSTEM_HEALTH_CODE_P2(code)   (((uint16_t)(code) >> 4) & 0x0Fu)
#define SYSTEM_HEALTH_CODE_P3(code)   ((uint16_t)(code) & 0x0Fu)

/* 系统健康错误码。
 * 约定：
 * - 1-x-x：启动/初始化 fatal
 * - 2-x-x：运行时超时
 * - 3-x-x：事件发布 fatal
 */
typedef enum
{
    SYSTEM_HEALTH_ERR_NONE = 0u,
    SYSTEM_HEALTH_ERR_BSP_INIT_FAIL = SYSTEM_HEALTH_CODE(1u, 1u, 1u),
    SYSTEM_HEALTH_ERR_DAMIAO_DM4310_TEST_TASK_START_FAIL = SYSTEM_HEALTH_CODE(1u, 7u, 1u),
    SYSTEM_HEALTH_ERR_VOFA_TASK_START_FAIL = SYSTEM_HEALTH_CODE(1u, 8u, 1u),
    SYSTEM_HEALTH_ERR_P1010B_LEFT_LEG_TEST_TASK_START_FAIL = SYSTEM_HEALTH_CODE(1u, 7u, 2u),
    SYSTEM_HEALTH_ERR_P1010B_VOFA_TASK_START_FAIL = SYSTEM_HEALTH_CODE(1u, 8u, 2u),
    SYSTEM_HEALTH_ERR_GO8010_TEST_TASK_START_FAIL = SYSTEM_HEALTH_CODE(1u, 7u, 3u),
    SYSTEM_HEALTH_ERR_GO8010_VOFA_TASK_START_FAIL = SYSTEM_HEALTH_CODE(1u, 8u, 3u),
    SYSTEM_HEALTH_ERR_SYSTEM_HEALTH_REGISTER_FAIL = SYSTEM_HEALTH_CODE(1u, 1u, 2u),
    SYSTEM_HEALTH_ERR_EVENT_BUS_INIT_FAIL = SYSTEM_HEALTH_CODE(1u, 1u, 3u),
    SYSTEM_HEALTH_ERR_MPU_DEVICE_INIT_FAIL = SYSTEM_HEALTH_CODE(1u, 2u, 1u),
    SYSTEM_HEALTH_ERR_IMU_TASK_START_FAIL = SYSTEM_HEALTH_CODE(1u, 2u, 2u),
    SYSTEM_HEALTH_ERR_INPUT_TASK_START_FAIL = SYSTEM_HEALTH_CODE(1u, 3u, 1u),
    SYSTEM_HEALTH_ERR_MODE_TASK_START_FAIL = SYSTEM_HEALTH_CODE(1u, 4u, 1u),
    SYSTEM_HEALTH_ERR_CHASSIS_TASK_START_FAIL = SYSTEM_HEALTH_CODE(1u, 5u, 2u),
    SYSTEM_HEALTH_ERR_ARM_TASK_START_FAIL = SYSTEM_HEALTH_CODE(1u, 6u, 1u),
    SYSTEM_HEALTH_ERR_DJI_M3508_TEST_TASK_START_FAIL = SYSTEM_HEALTH_CODE(1u, 5u, 1u),
    SYSTEM_HEALTH_ERR_MOTOR_COMMUNICATIONS_TASK_START_FAIL = SYSTEM_HEALTH_CODE(1u, 6u, 2u),
    SYSTEM_HEALTH_ERR_SELF_TEST_FATAL = SYSTEM_HEALTH_CODE(1u, 9u, 1u),
    SYSTEM_HEALTH_ERR_SELF_TEST_TASK_START_FAIL = SYSTEM_HEALTH_CODE(1u, 9u, 2u),
    SYSTEM_HEALTH_ERR_EVT_IMU_DATA_READY_PUBLISH_FAIL = SYSTEM_HEALTH_CODE(3u, 2u, 1u),
    SYSTEM_HEALTH_ERR_EVT_RC_DATA_READY_PUBLISH_FAIL = SYSTEM_HEALTH_CODE(3u, 3u, 1u),
    SYSTEM_HEALTH_ERR_EVT_MODE_CHANGED_PUBLISH_FAIL = SYSTEM_HEALTH_CODE(3u, 4u, 1u),
    SYSTEM_HEALTH_ERR_EVT_MOTOR_TX_REQUEST_PUBLISH_FAIL = SYSTEM_HEALTH_CODE(3u, 5u, 1u),
    SYSTEM_HEALTH_ERR_DAMIAO_SMOKE_TIMEOUT = SYSTEM_HEALTH_CODE(2u, 7u, 1u),
    SYSTEM_HEALTH_ERR_P1010B_LEFT_LEG_TEST_TIMEOUT = SYSTEM_HEALTH_CODE(2u, 7u, 2u),
    SYSTEM_HEALTH_ERR_GO8010_SMOKE_TIMEOUT = SYSTEM_HEALTH_CODE(2u, 7u, 3u),
    SYSTEM_HEALTH_ERR_SELF_TEST_TIMEOUT = SYSTEM_HEALTH_CODE(2u, 9u, 1u),
    SYSTEM_HEALTH_ERR_IMU_TASK_TIMEOUT = SYSTEM_HEALTH_CODE(2u, 2u, 1u),
    SYSTEM_HEALTH_ERR_MODE_TASK_TIMEOUT = SYSTEM_HEALTH_CODE(2u, 4u, 1u),
    SYSTEM_HEALTH_ERR_CHASSIS_TASK_TIMEOUT = SYSTEM_HEALTH_CODE(2u, 5u, 1u),
    SYSTEM_HEALTH_ERR_ARM_TASK_TIMEOUT = SYSTEM_HEALTH_CODE(2u, 6u, 1u),
    SYSTEM_HEALTH_ERR_MOTOR_COMMUNICATIONS_TIMEOUT = SYSTEM_HEALTH_CODE(2u, 6u, 2u),
    SYSTEM_HEALTH_ERR_MOTOR_RECOVERY_DEGRADED = SYSTEM_HEALTH_CODE(2u, 6u, 3u),
} SystemHealthErrorCode;

#define SYSTEM_HEALTH_ERR_RUNTIME_FAULT_MAX (SYSTEM_HEALTH_ERR_SELF_TEST_TIMEOUT)

/* 初始化系统健康模块。
 * 负责：
 * - 初始化项目内 LED GPIO
 * - 进入 BOOTING 状态
 * - 清空 runtime/fatal 故障集合
 */
OmRet system_health_init(void);

/* 注册一个需要被 watchdog 监督的任务。 */
OmRet system_health_register(SystemHealthTaskId task_id, OsalTimeMs timeout_ms, SystemHealthErrorCode fault_code);

/* 任务心跳上报接口。 */
OmRet system_health_beat(SystemHealthTaskId task_id);

/* 显式上报一个运行时故障。
 * 当前项目里主要还是依赖心跳超时自动生成 runtime 故障，
 * 但这个接口预留给未来主动上报场景。
 */
OmRet system_health_report_runtime_fault(SystemHealthErrorCode code);

/* 清除一个显式上报的运行时故障。 */
OmRet system_health_clear_runtime_fault(SystemHealthErrorCode code);

/* 上报一个 fatal 故障。
 * 注意：该接口只负责“锁存 fatal”，不直接闪灯，也不直接进入 OM_CPU_ERRHANDLER。
 * 最终的 fatal 仲裁与灯控动作由 start_task 周期调用 system_health_poll() 时统一执行。
 */
void system_health_report_fatal(SystemHealthErrorCode code, char* msg);

/* 切换到 RUNNING 状态，显示绿灯常亮。 */
void system_health_set_running(void);

/* 系统健康总裁轮询入口。
 * 这应该由 start_task 周期调用，作为唯一的健康监督执行者。
 */
void system_health_poll(void);

#define SYSTEM_HEALTH_REPORT_FATAL(code, msg) system_health_report_fatal(code, msg)

#endif
