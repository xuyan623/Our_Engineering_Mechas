#ifndef NEW_ROBOT_DJI_M3508_TEST_TASK_H
#define NEW_ROBOT_DJI_M3508_TEST_TASK_H

#include "bsp/bsp_init.h"
#include "core/om_def.h"

typedef struct
{
    volatile int16_t output_command;
    volatile float angle_deg;
    volatile float speed_rpm;
    volatile float current_amp;
    volatile float temp_deg;
    volatile uint32_t loop_count;
    volatile uint32_t tx_count;
    volatile uint32_t feedback_seen_count;
} DjiM3508TestDebugState;

extern DjiM3508TestDebugState g_dji_m3508_test_debug;

OmRet dji_m3508_test_task_start(const BspDeviceRegistry* devices);

#endif
