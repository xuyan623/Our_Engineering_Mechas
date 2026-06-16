#ifndef NEW_ROBOT_CT_INTERNAL_H
#define NEW_ROBOT_CT_INTERNAL_H

/* chassis_task 内部共享头文件。
 * 职责：存放 chassis_task 模块内部的类型定义、上下文结构体和共享常量，
 * 供 chassis_task.c 与 chassis_task_diag.c 共同使用。
 * 对外不暴露，调用方应使用 chassis_task.h 或 chassis_task_diag.h。
 */

#include "config/app_config.h"
#include "algorithm/kinematics/kinematics.h"
#include "core/algorithm/controller/pid.h"
#include "driver/motor/motor.h"
#include "module/task_channel/task_channel.h"
#include "osal/osal_time.h"
#include "task/input_task/input_task_snapshot.h"
#include "task/imu_task/imu_task_snapshot.h"
#include "task/mode_task/mode_task.h"
#include "module/task_context_pool/task_context_pool.h"
#include <atomic/atomic.h>
#include <stdint.h>

#define CT_PERIOD_MS        APP_CT_TASK_PERIOD_MS
#define CT_STACK_BYTES      (512u * OSAL_STACK_WORD_BYTES)
#define CT_PRIORITY         (4u)
#define CT_RIGHT_LEG_INDEX  (0u)
#define CT_LEFT_LEG_INDEX   (1u)
#define CT_TX_REQUEST_PERIOD_MS            APP_CT_TX_REQUEST_PERIOD_MS
#define CT_MODE_CHANNEL_CAPACITY         (8u)
#define CT_RC_BYTES     (256u)
#define CT_IMU_BYTES    (256u)

#define CT_KEY_W_MASK       (1u << 0u)
#define CT_KEY_S_MASK       (1u << 1u)
#define CT_KEY_A_MASK       (1u << 2u)
#define CT_KEY_D_MASK       (1u << 3u)
#define CT_KEY_SHIFT_MASK   (1u << 4u)
#define CT_KEY_CTRL_MASK    (1u << 5u)

#define CT_WHEEL_COUNT      (MECANUM_WHEEL_COUNT)
#define CT_FRONT_WHEEL_COUNT  (2u)
#define CT_REAR_WHEEL_COUNT   (2u)
#define CT_LEG_COUNT        (2u)

/* 底盘任务输入快照。
 * 从各通道 drain 出来的最新输入汇总。
 */
typedef struct
{
    int16_t ch1;
    int16_t ch2;
    int16_t ch3;
    int16_t ch4;
    int16_t mouse_x;
    uint16_t keyboard_bits;
    uint8_t sw1;
    uint8_t sw2;
    uint16_t iw;
    ModeTaskPhaseState operational_phase;
    uint8_t wheel_enable;
    uint8_t leg_enable;
    uint8_t allow_rc_drive;
    float imu_pitch_deg;
} ChassisInputSnap;

/* 底盘任务本地上下文：
 * - 持有底盘四轮、两腿、big_yaw 电机句柄
 * - 维护轮速 PID 和腿部角度/速度 PID
 * - 保存最新输入快照与通道状态
 */
typedef struct
{
    /* 电机句柄及 PID 控制器均已外迁至模块局部缓存/静态变量，此处不再内嵌 */
    TaskMpscChannel mode_channel;
    TaskPipeChannel rc_channel;
    TaskPipeChannel imu_channel;
    ChassisModeSnap latest_mode_snapshot;
    InputRcSnapshot latest_rc_snapshot;
    ImuTaskSnapshot latest_imu_snapshot;
    float last_wheel_speed_reference_rpm[CT_WHEEL_COUNT];
    float pit_leg_cmd_deg;
    float big_yaw_hold_angle_rad;
    OsalTimeMs last_tx_request_ms;
    OsalTimeMs rc_rotate_saturation_since_ms;
    uint16_t flags;
    /* bit 0: big_yaw_hold_initialized */
    /* bit 1: motors_bound */
    /* bit 2: control_modes_armed */
    /* bit 3: mode_snapshot_ready */
    /* bit 4: rc_snapshot_ready */
    /* bit 5: imu_snapshot_ready */
} ChassisTaskContext;

#define CT_FLAG_BIG_YAW_HOLD_INIT     (1u << 0u)
#define CT_FLAG_MOTORS_BOUND          (1u << 1u)
#define CT_FLAG_MODES_ARMED   (1u << 2u)
#define CT_FLAG_MODE_READY   (1u << 3u)
#define CT_FLAG_RC_SNAPSHOT_READY     (1u << 4u)
#define CT_FLAG_IMU_READY    (1u << 5u)

/* 电机句柄查询：由 chassis_task.c 内部缓存表支持。 */
static inline Motor* chassis_task_get_wheel_motor(uint32_t index)
{
    extern Motor* g_chassis_task_wheel_motor_cache[];
    return (index < CT_WHEEL_COUNT) ? g_chassis_task_wheel_motor_cache[index] : OM_NULL;
}

static inline Motor* chassis_task_get_leg_motor(uint32_t index)
{
    extern Motor* g_chassis_task_leg_motor_cache[];
    return (index < CT_LEG_COUNT) ? g_chassis_task_leg_motor_cache[index] : OM_NULL;
}

static inline Motor* chassis_task_big_yaw_motor(void)
{
    extern Motor* g_chassis_task_big_yaw_motor;
    return g_chassis_task_big_yaw_motor;
}

/* 底盘任务运行时上下文单例，由 chassis_task.c 定义。 */
extern TaskContextSlotId g_chassis_task_slot_id;
extern const uint8_t g_chassis_task_wheel_roles[CT_WHEEL_COUNT];
extern const uint8_t g_chassis_task_leg_roles[CT_LEG_COUNT];
extern Motor* g_chassis_task_wheel_motor_cache[CT_WHEEL_COUNT];
extern Motor* g_chassis_task_leg_motor_cache[CT_LEG_COUNT];
extern Motor* g_chassis_task_big_yaw_motor;
extern PidController g_front_wheel_speed_pids[CT_FRONT_WHEEL_COUNT];
extern PidController g_rear_wheel_speed_pids[CT_REAR_WHEEL_COUNT];
extern PidController g_leg_angle_pids[CT_LEG_COUNT];
static inline ChassisTaskContext* chassis_task_owner_context(void)
{
    return (ChassisTaskContext*)task_context_pool_get_ptr(g_chassis_task_slot_id);
}
#define g_chassis_task_owner_context chassis_task_owner_context()

float chassis_task_now_s(void);
OmBool chassis_task_front_wheel(uint32_t wheel_index);
OmBool chassis_task_wheel_present(uint32_t wheel_index);
OmBool chassis_task_wheel_allows(uint32_t wheel_index);
OmBool chassis_task_leg_present(uint32_t leg_index);
OmBool chassis_task_leg_allows(uint32_t leg_index);
PidController* chassis_task_wheel_pid(
    ChassisTaskContext* context,
    uint32_t wheel_index);
OmBool chassis_task_feedback_recent(const Motor* motor, uint32_t timeout_ms);
OmBool chassis_task_key_is_down(uint16_t keyboard_bits, uint16_t mask);
void chassis_task_drain_mode(ChassisTaskContext* context);
void chassis_task_drain_rc(ChassisTaskContext* context);
void chassis_task_drain_imu(ChassisTaskContext* context);
OmBool chassis_task_load_snapshot(
    const ChassisTaskContext* context,
    ChassisInputSnap* snapshot);
void chassis_task_chassis_velocity(
    ChassisTaskContext* context,
    const ChassisInputSnap* snapshot,
    float* vx_mm_per_s,
    float* vy_mm_per_s,
    float* vw_deg_per_s);
OmRet chassis_task_init_pids(ChassisTaskContext* context);
OmRet chassis_task_try_bind_motors(ChassisTaskContext* context);
OmRet chassis_task_restore_modes(ChassisTaskContext* context);
void chassis_task_apply_zero(ChassisTaskContext* context);
OmBool chassis_task_should_tx(
    ChassisTaskContext* context,
    ModeTaskPhaseState operational_phase,
    OsalTimeMs now_ms);
void chassis_task_run_once(ChassisTaskContext* context);

#endif
