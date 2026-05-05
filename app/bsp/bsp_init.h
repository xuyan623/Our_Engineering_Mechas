#ifndef NEW_ROBOT_BSP_INIT_H
#define NEW_ROBOT_BSP_INIT_H

#include "core/om_def.h"
#include "drivers/model/device.h"
#include <stdint.h>

typedef struct
{
    Device* can1;
    Device* can2;
    Device* usart1;
    Device* usart3;
    Device* usart6;
    Device* uart7;
    Device* uart8;
} BspDeviceRegistry;

/* 只收集设备句柄并设置板级安全默认态，不打开通信外设。 */
OmRet bsp_register_all(void);
/* 兼容旧 eager-init 路径；当前仅供过渡期旧入口使用。 */
OmRet bsp_init_all(void);
const BspDeviceRegistry* bsp_get_device_registry(void);
/* 由 IMU owner 显式初始化 SPI5 基础链路。 */
OmRet bsp_spi5_init(void);

/* A6 阶段由 mpu6500 驱动直接调用这些桥接函数，C1 在这里把 stub 替换为真实板级实现。 */
uint8_t SPI5_ReadWriteByte(uint8_t tx_data);
void mpu6500_SPI_NS_H(void);
void mpu6500_SPI_NS_L(void);
void mpu6500_delay_ms(uint16_t ms);
void mpu6500_delay_us(uint32_t us);

#endif
