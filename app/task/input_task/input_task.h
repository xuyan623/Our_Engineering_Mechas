#ifndef NEW_ROBOT_INPUT_TASK_H
#define NEW_ROBOT_INPUT_TASK_H

#include "bsp/bsp_init.h"
#include "core/om_def.h"

typedef struct
{
    volatile uint32_t rx_available_hint;
    volatile uint32_t frame_count;
    volatile uint32_t invalid_frame_count;
    volatile uint32_t resync_drop_count;
} InputTaskDebugState;

extern InputTaskDebugState g_input_task_runtime;

OmRet input_task_start(const BspDeviceRegistry* devices);

#endif
