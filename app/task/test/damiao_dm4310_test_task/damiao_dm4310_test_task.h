#ifndef NEW_ROBOT_DAMIAO_DM4310_TEST_TASK_H
#define NEW_ROBOT_DAMIAO_DM4310_TEST_TASK_H

#include "bsp/bsp_init.h"
#include "core/om_def.h"

typedef struct
{
    volatile float target_position_rad;
    volatile float position_rad;
    volatile float velocity_rad_s;
    volatile float torque_nm;
    volatile uint8_t status_code;
    volatile uint32_t loop_count;
    volatile uint32_t tx_count;
    volatile uint32_t feedback_seen_count;
} DamiaoDm4310TestDebugState;

extern DamiaoDm4310TestDebugState g_damiao_dm4310_test_debug;

OmRet damiao_dm4310_test_task_start(const BspDeviceRegistry* devices);

#endif
