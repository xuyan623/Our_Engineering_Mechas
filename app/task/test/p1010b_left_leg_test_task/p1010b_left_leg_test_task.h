#ifndef NEW_ROBOT_P1010B_LEFT_LEG_TEST_TASK_H
#define NEW_ROBOT_P1010B_LEFT_LEG_TEST_TASK_H

#include "bsp/bsp_init.h"
#include "core/om_def.h"
#include <stdint.h>

typedef struct
{
    volatile float absolute_position_raw;
    volatile float total_angle_deg;
    volatile float speed_rpm;
    volatile float speed_deg_per_s;
    volatile float iq_current_amp;
    volatile float bus_voltage_v;
    volatile uint8_t online_flag;
    volatile uint8_t driver_state;
    volatile int32_t init_ret;
    volatile int32_t prepare_ret;
    volatile uint32_t feedback_count;
    volatile uint32_t last_rx_age_ms;
} P1010BLeftDebug;

extern P1010BLeftDebug g_p1010b_left_leg_test_debug;

OmRet p10lt_start(const BspDeviceRegistry* devices);

#endif
