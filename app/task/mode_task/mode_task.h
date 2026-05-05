#ifndef NEW_ROBOT_MODE_TASK_H
#define NEW_ROBOT_MODE_TASK_H

#include "core/om_def.h"
#include <stdint.h>

/* 全局控制模式：
 * - RELEASE：总控释放，控制输出应进入安全态
 * - MANUAL：遥控直接控制的手动模式
 * - ENGINEER：动作/取矿/兑换等工程模式
 */
typedef enum
{
    MODE_GLOBAL_RELEASE_CTRL = 0u,
    MODE_GLOBAL_MANUAL_CTRL,
    MODE_GLOBAL_ENGINEER_CTRL,
} GlobalMode;

/* 底盘子模式：
 * 这里保留旧工程里实际参与切换的模式枚举，供 chassis/arm 等后续任务直接消费。
 */
typedef enum
{
    MODE_CHASSIS_RELEASE = 0u,
    MODE_CHASSIS_NORMAL,
    MODE_CHASSIS_RESCUE,
    MODE_CHASSIS_SUPPLY,
    MODE_CHASSIS_PITCH3_TORQUE_COLLECTION,
    MODE_CHASSIS_URGENT_MEASURE,
    MODE_CHASSIS_EXCHANGE,
    MODE_CHASSIS_PRIMARY,
    MODE_CHASSIS_GET_ENERGY_UNIT,
    MODE_CHASSIS_GET_ENERGY_UNIT1,
    MODE_CHASSIS_GET_ENERGY_UNIT2,
    MODE_CHASSIS_CLAMP_CATCH,
    MODE_CHASSIS_SECONDARY_ORE,
    MODE_CHASSIS_STOP,
    MODE_CHASSIS_DEFEND,
    MODE_CHASSIS_CHECK,
} ChassisMode;

/* 机械臂夹取动作推进状态。 */
typedef enum
{
    MODE_CLAMP_UN_CMD = 0u,
    MODE_CLAMP_ACTION_ONE,
    MODE_CLAMP_ACTION_TWO,
    MODE_CLAMP_ACTION_THREE,
} ClampAction;

/* 兑换动作推进状态。 */
typedef enum
{
    MODE_EXCHANGE_UN_CMD = 0u,
    MODE_EXCHANGE_PICK_ACTION1,
    MODE_EXCHANGE_PICK_ACTION2,
} ExchangeAction;

/* mode_task 的轻量调试状态：
 * - loop_count：任务循环次数
 * - publish_count：共享模式结果变化并成功发出 EVT_MODE_CHANGED 的次数
 */
typedef struct
{
    volatile uint32_t loop_count;
    volatile uint32_t publish_count;
} ModeTaskDebugState;

extern ModeTaskDebugState g_mode_task_debug;

/**
 * @brief 启动模式切换任务
 * @return `OM_OK` 表示启动成功，其他返回值表示初始化或任务创建失败
 */
OmRet mode_task_start(void);

#endif
