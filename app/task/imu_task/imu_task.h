#ifndef NEW_ROBOT_IMU_TASK_H
#define NEW_ROBOT_IMU_TASK_H

#include "core/om_def.h"

typedef struct
{
    volatile uint32_t loop_count;
    volatile uint32_t publish_count;
    volatile uint32_t no_new_sample_count;
    volatile uint32_t stack_reserved_bytes;
    volatile uint32_t stack_high_water_words;
    volatile uint32_t stack_min_free_bytes;
    volatile uint32_t stack_peak_used_bytes;
} ImuTaskDebugState;

extern ImuTaskDebugState g_imu_task_debug;

/**
 * @brief 启动 IMU 周期任务
 * @return `OM_OK` 表示启动成功，其他返回值表示任务创建失败或重复启动
 */
OmRet imu_task_start(void);

#endif
