#ifndef NEW_ROBOT_MT_H
#define NEW_ROBOT_MT_H

/* mode_task 的职责边界：
 * - 它是正式全局控制状态 owner
 * - formal 输出面拆成：
 *   - ModeSystemSnap
 *   - ArmTaskModeSnapshot
 *   - ChassisModeSnap
 */

#include "core/om_def.h"
#include "task/input_task/input_task_snapshot.h"
#include <stdint.h>

#define RC_SWITCH_UP  (1u)
#define RC_SWITCH_DN  (2u)
#define RC_SWITCH_MI  (3u)

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
    MODE_CHASSIS_CUSTOM_CONTROLLER_NORMAL,
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

/* 第一层：系统总状态。 */
typedef enum
{
    MT_SYSTEM_UNINITIALIZED = 0u,
    MT_SYSTEM_BOARD_INITIALIZING,
    MT_SYSTEM_MOTOR_INITIALIZING,
    MT_SYSTEM_RELEASE,
    MT_SYSTEM_OPERATIONAL,
} ModeTaskSystemState;

/* 第二层：板级初始化子状态。 */
typedef enum
{
    MT_BOARD_INIT_NONE = 0u,
    MT_BOARD_INIT_CAN_INITIALIZING,
    MT_BOARD_INIT_SERIAL_INITIALIZING,
    MT_BOARD_INIT_IMU_INITIALIZING,
} ModeBoardInitState;

/* 第二层：电机初始化子状态。 */
typedef enum
{
    MT_MOTOR_INIT_NONE = 0u,
    MT_MOTOR_INIT_CHASSIS_INIT,
    MT_MOTOR_INIT_ARM_INIT,
} ModeMotorInitState;

/* 第二层：正式控制域。 */
typedef enum
{
    MT_CONTROL_DOMAIN_NONE = 0u,
    MT_CONTROL_DOMAIN_RC,
    MT_CONTROL_DOMAIN_CUSTOM,
} ModeTaskDomainState;

/* 整车 operational 相位。 */
typedef enum
{
    MT_OPERATIONAL_PHASE_RELEASE = 0u,
    MT_OPERATIONAL_PHASE_SELECT,
    MT_OPERATIONAL_PHASE_FORMAL,
} ModeTaskPhaseState;

/* 已确认或待确认的运动模式编号。 */
typedef enum
{
    MT_MOTION_MODE_NONE = 0u,
    MT_MOTION_MODE_PRESET_ACTION = 1u,
    MT_MOTION_MODE_CUSTOM_TAKEOVER = 2u,
    MT_MOTION_MODE_RC_IK = 3u,
} ModeTaskMotionModeId;

typedef enum
{
    MT_IK_SOLVER_FULL_POSE = 0u,
    MT_IK_SOLVER_POSITION_PRIORITY,
} ModeTaskIkSolverMode;

typedef enum
{
    MT_IK_BANK_POS_XYZ = 0u,
    MT_IK_BANK_ORI_RPY,
} ModeTaskIkControlBank;

typedef enum
{
    MT_GRIP_OPEN = 0u,
    MT_GRIP_CLOSED,
} ModeTaskGripState;

typedef enum
{
    AT_MODE_RELEASE = 0u,
    AT_MODE_NORMAL,
    AT_MODE_PRESET_ACTION,
    AT_MODE_CUSTOM_TAKEOVER,
    AT_MODE_RC_IK,
} ArmTaskMode;

/* 第三层：控制链在线状态。 */
typedef enum
{
    MT_CONTROL_LINK_OFFLINE = 0u,
    MT_CONTROL_LINK_ONLINE,
} ModeLinkState;

/* 第四层：自定义控制器在线后的行为态。 */
typedef enum
{
    MT_CUSTOM_CONTROL_ALIGNING = 0u,
    MT_CUSTOM_CONTROL_TAKEOVER,
} ModeTaskCustomState;

typedef enum
{
    MODE_INIT_CAN_READY = 0u,
    MODE_INIT_SERIAL_READY,
    MODE_INIT_IMU_READY,
    MODE_INIT_CHASSIS_MOTOR_READY,
    MODE_INIT_ARM_MOTOR_READY,
} ModeTaskInitKind;

typedef struct
{
    uint8_t kind;
    uint8_t value;
} ModeTaskInitMessage;

typedef struct
{
    uint8_t operational_phase;
    uint8_t selected_motion_mode_id;
    uint8_t confirmed_motion_mode_id;
} ModeSystemSnap;

typedef struct
{
    uint8_t chassis_mode;
    uint8_t clamp_action;
    uint8_t exchange_action;
    uint8_t primary_turn_ore_flag;
} ArmPresetSnapshot;

typedef struct
{
    uint8_t arm_mode;
    uint8_t grip_state;
    uint8_t ik_solver_mode;
    uint8_t ik_control_bank;
    ArmPresetSnapshot preset_action;
} ArmTaskModeSnapshot;

typedef struct
{
    uint8_t operational_phase;
    uint8_t wheel_enable;
    uint8_t leg_enable;
    uint8_t allow_rc_drive;
} ChassisModeSnap;

/* mode_task 的轻量调试状态：
 * - loop_count：任务循环次数
 * - publish_count：共享模式结果变化次数
 *
 * 当前 event_bus 已退出正式链，因此 publish_count 只反映
 * “共享状态发生变化并完成一次对下游发布”的调试计数，不代表事件总线行为。
 */
typedef struct
{
    volatile uint32_t loop_count;
    volatile uint32_t publish_count;
    volatile uint8_t system_state;
    volatile uint8_t board_init_state;
    volatile uint8_t motor_init_state;
    volatile uint8_t operational_phase;
    volatile uint8_t control_domain_state;
    volatile uint8_t rc_link_state;
    volatile uint8_t custom_link_state;
    volatile uint8_t custom_control_state;
    volatile uint8_t selected_motion_mode_id;
    volatile uint8_t confirmed_motion_mode_id;
    volatile uint8_t ik_solver_mode;
    volatile uint8_t ik_control_bank;
    volatile uint8_t grip_state;
} ModeTaskDebugState;

extern ModeTaskDebugState g_mode_task_debug;

/**
 * @brief 启动模式切换任务
 * @return `OM_OK` 表示启动成功，其他返回值表示初始化或任务创建失败
 */
OmRet mode_task_start(void);

OmRet mode_task_submit_init(
    const ModeTaskInitMessage* message);

OmRet mode_task_submit_rc_snapshot(
    const InputRcSnapshot* snapshot);

OmRet mode_task_submit_custom(
    const InputCustomSnapshot* snapshot);

OmBool mode_task_copy_system(
    ModeSystemSnap* snapshot);
OmBool mode_task_copy_arm_mode(
    ArmTaskModeSnapshot* snapshot);
OmBool mode_task_copy_chassis_mode(
    ChassisModeSnap* snapshot);

#endif
