#ifndef NEW_ROBOT_IMU_H
#define NEW_ROBOT_IMU_H

#include "core/om_def.h"
#include <stdint.h>

/* 加速度计量程配置。 */
/* #define MPU6500_ACCEL_RANGE_16G */
#define MPU6500_ACCEL_RANGE_8G
/* #define MPU6500_ACCEL_RANGE_4G */
/* #define MPU6500_ACCEL_RANGE_2G */

/* 陀螺仪量程配置。 */
#define MPU6500_GYRO_RANGE_2000
/* #define MPU6500_GYRO_RANGE_1000 */
/* #define MPU6500_GYRO_RANGE_500 */
/* #define MPU6500_GYRO_RANGE_250 */

/* 磁力计开关。 */
#define USE_MAGNETOMETER
#ifdef USE_MAGNETOMETER
#define MAG_SEN 1320.0f
#endif

/* 校准开关：A6 默认关闭，沿用硬编码偏移。 */
/* #define USE_CALIBRATION */

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

    float temp;
    float temp_ref;
    float wx;
    float wy;
    float wz;
    float vx;
    float vy;
    float vz;
    float rol;
    float pit;
    float yaw;
    float rol_rate;
    float pit_rate;
    float yaw_rate;

#ifdef USE_MAGNETOMETER
    float mag_x;
    float mag_y;
    float mag_z;
#endif
} imu_data_t;

uint8_t mpu_device_init(float gravity);
void mpu_get_data(void);
OmBool imu_has_new_data(void);
uint8_t mpu_offset_cal(float gravity);
void update_attitude(float dt);
imu_data_t* get_imu_data(void);
void mpu_direct_value_cal(void);

#endif
