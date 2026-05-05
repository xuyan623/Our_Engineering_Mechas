#ifndef NEW_ROBOT_HW_SMOKE_TASK_H
#define NEW_ROBOT_HW_SMOKE_TASK_H

#include "bsp/bsp_init.h"
#include "core/om_def.h"
#include "driver/imu/imu.h"
#include <stddef.h>
#include <stdint.h>

#define HW_SMOKE_SERIAL_COUNT (5u)
#define HW_SMOKE_CAN_COUNT    (2u)
#define HW_SMOKE_VOFA_CHANNEL_COUNT (16u)

typedef enum
{
    HW_SMOKE_SERIAL_USART1 = 0,
    HW_SMOKE_SERIAL_USART3,
    HW_SMOKE_SERIAL_USART6,
    HW_SMOKE_SERIAL_UART7,
    HW_SMOKE_SERIAL_UART8
} HwSmokeSerialId;

typedef enum
{
    HW_SMOKE_CAN1 = 0,
    HW_SMOKE_CAN2
} HwSmokeCanId;

typedef struct
{
    OmRet open_ret;
    uint32_t tx_count;
    uint32_t rx_count;
    uint32_t ok_reply_count;
    uint32_t write_fail_count;
} HwSmokeSerialSnapshot;

typedef struct
{
    OmRet cfg_ret;
    OmRet filter_ret;
    uint32_t tx_count;
    uint32_t rx_count;
    OmBool last_payload_ok;
    uint32_t last_tx_seq;
    uint32_t last_rx_seq;
} HwSmokeCanSnapshot;

typedef struct
{
    OmBool bsp_init_ok;
    uint8_t imu_init_ret;
    uint32_t imu_update_count;
    imu_data_t last_imu;
    HwSmokeSerialSnapshot serials[HW_SMOKE_SERIAL_COUNT];
    HwSmokeCanSnapshot cans[HW_SMOKE_CAN_COUNT];
    uint32_t last_error_code;
} HwSmokeSnapshot;

extern HwSmokeSnapshot g_hw_smoke_snapshot;

OmRet hw_smoke_task_start(const BspDeviceRegistry* devices, uint8_t imu_init_ret);

#endif
