#ifndef NEW_ROBOT_IMU_BSP_H
#define NEW_ROBOT_IMU_BSP_H

#include "core/om_def.h"
#include <stdint.h>

#define IMU_BSP_RAW_PAYLOAD_LEN  (20u)

typedef struct
{
    volatile uint32_t drdy_irq_count;
    volatile uint32_t dma_start_count;
    volatile uint32_t dma_done_count;
    volatile uint32_t dma_drop_count;
    volatile uint32_t dma_error_count;
    volatile uint32_t latest_seq;
    volatile uint32_t last_processed_seq;
} ImuBspDebugState;

extern volatile ImuBspDebugState g_imu_bsp_debug;

OmRet imu_bsp_init(void);
OmBool imu_bsp_fetch_latest_raw(uint8_t out_payload[IMU_BSP_RAW_PAYLOAD_LEN], uint32_t* out_seq);

#endif
