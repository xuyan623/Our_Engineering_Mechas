#ifndef NEW_ROBOT_CHASSIS_TASK_INTERNAL_H
#define NEW_ROBOT_CHASSIS_TASK_INTERNAL_H

/* chassis_task 内部共享头文件。
 * 职责：存放 chassis_task 模块内部的类型定义、上下文结构体和共享常量，
 * 供 chassis_task.c 与 chassis_task_diag.c 共同使用。
 * 对外不暴露，调用方应使用 chassis_task.h 或 chassis_task_diag.h。
 */

#include "algorithm/kinematics/kinematics.h"
#include "core/algorithm/controller/pid.h"
#include "driver/motor/motor.h"
#include "module/data_pool/data_pool.h"
#include "module/task_channel/task_channel.h"
#include "osal/osal_time.h"
#include "task/mode_task/mode_task.h"
#include "module/task_context_pool/task_context_pool.h"
#include <atomic/atomic.h>
#include <stdint.h>

#define CHASSIS_TASK_WHEEL_COUNT      (MECANUM_WHEEL_COUNT)
#define CHASSIS_TASK_FRONT_WHEEL_COUNT  (2u)
#define CHASSIS_TASK_REAR_WHEEL_COUNT   (2u)
#define CHASSIS_TASK_LEG_COUNT        (2u)

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
    ModeTaskSystemState system_state;
    ModeTaskControlDomainState control_domain_state;
    GlobalMode global_mode;
    ChassisMode chassis_mode;
    float imu_pitch_deg;
} ChassisTaskInputSnapshot;

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
    ModeTaskControlSnapshot latest_mode_snapshot;
    DpRcSnapshot latest_rc_snapshot;
    DpImuSnapshot latest_imu_snapshot;
    float last_wheel_speed_ref_rpm[CHASSIS_TASK_WHEEL_COUNT];
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

#define CHASSIS_TASK_FLAG_BIG_YAW_HOLD_INIT     (1u << 0u)
#define CHASSIS_TASK_FLAG_MOTORS_BOUND          (1u << 1u)
#define CHASSIS_TASK_FLAG_CONTROL_MODES_ARMED   (1u << 2u)
#define CHASSIS_TASK_FLAG_MODE_SNAPSHOT_READY   (1u << 3u)
#define CHASSIS_TASK_FLAG_RC_SNAPSHOT_READY     (1u << 4u)
#define CHASSIS_TASK_FLAG_IMU_SNAPSHOT_READY    (1u << 5u)

/* 电机句柄查询：由 chassis_task.c 内部缓存表支持。 */
static inline Motor* chassis_task_get_wheel_motor(uint32_t index)
{
    extern Motor* g_chassis_task_wheel_motor_cache[];
    return (index < CHASSIS_TASK_WHEEL_COUNT) ? g_chassis_task_wheel_motor_cache[index] : OM_NULL;
}

static inline Motor* chassis_task_get_leg_motor(uint32_t index)
{
    extern Motor* g_chassis_task_leg_motor_cache[];
    return (index < CHASSIS_TASK_LEG_COUNT) ? g_chassis_task_leg_motor_cache[index] : OM_NULL;
}

static inline Motor* chassis_task_get_big_yaw_motor(void)
{
    extern Motor* g_chassis_task_big_yaw_motor;
    return g_chassis_task_big_yaw_motor;
}

/* 底盘任务运行时上下文单例，由 chassis_task.c 定义。 */
extern TaskContextSlotId g_chassis_task_slot_id;
static inline ChassisTaskContext* chassis_task_get_owner_context(void)
{
    return (ChassisTaskContext*)task_context_pool_get_ptr(g_chassis_task_slot_id);
}
#define g_chassis_task_owner_context chassis_task_get_owner_context()

#endif
