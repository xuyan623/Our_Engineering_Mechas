#ifndef NEW_ROBOT_MPU6500_H
#define NEW_ROBOT_MPU6500_H

#include "driver/imu/imu.h"
#include <stdint.h>

typedef struct
{
    int16_t ax;
    int16_t ay;
    int16_t az;

#ifdef USE_MAGNETOMETER
    int16_t mx;
    int16_t my;
    int16_t mz;
#endif

    int16_t temp;
    int16_t gx;
    int16_t gy;
    int16_t gz;
    int16_t ax_offset;
    int16_t ay_offset;
    int16_t az_offset;
    int16_t gx_offset;
    int16_t gy_offset;
    int16_t gz_offset;
    float accel_scale;
} mpu_data_t;

mpu_data_t* get_mpu_data(void);
uint8_t mpu6500_init(void);
void mpu6500_delay_ms(uint16_t ms);
void mpu6500_delay_us(uint32_t us);
void mpu6500_SPI_NS_H(void);
void mpu6500_SPI_NS_L(void);
void mpu6500_write_single_reg(uint8_t reg, uint8_t data);
uint8_t mpu6500_read_single_reg(uint8_t reg);
void mpu6500_write_muli_reg(uint8_t reg, uint8_t* buf, uint8_t len);
void mpu6500_read_muli_reg(uint8_t reg, uint8_t* buf, uint8_t len);

#endif
