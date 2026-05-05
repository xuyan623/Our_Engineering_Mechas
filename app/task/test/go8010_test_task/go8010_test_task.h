#ifndef NEW_ROBOT_GO8010_TEST_TASK_H
#define NEW_ROBOT_GO8010_TEST_TASK_H

#include "bsp/bsp_init.h"
#include "core/om_def.h"

typedef struct
{
    volatile float target_position_rad;
    volatile float target_speed_rad_s;
    volatile float target_torque_nm;
    volatile float position_rad;
    volatile float relative_position_rad;
    volatile float speed_rad_s;
    volatile float torque_nm;
    volatile uint8_t mode;
    volatile uint8_t online_flag;
    volatile int32_t init_ret;
    volatile uint32_t feedback_seen_count;
    volatile uint32_t last_rx_age_ms;
} Go8010TestDebugState;

extern Go8010TestDebugState g_go8010_test_debug;

OmRet go8010_test_task_start(const BspDeviceRegistry* devices);

#endif
