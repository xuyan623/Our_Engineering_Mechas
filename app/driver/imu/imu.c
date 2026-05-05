#include "driver/imu/imu.h"

#include "bsp/bsp_init.h"
#include "bsp/imu_bsp.h"
#include "driver/ahrs/MahonyAHRS.h"
#include "driver/ahrs/filters.h"
#include "driver/mpu6500/ist8310.h"
#include "driver/mpu6500/mpu6500.h"
#include "driver/mpu6500/mpu6500_reg.h"

#ifdef MPU6500_GYRO_RANGE_2000
#define GYRO_SEN 0.00106526443603169529841533860381f
#elif defined(MPU6500_GYRO_RANGE_1000)
#define GYRO_SEN 0.0005326322180158476492076f
#elif defined(MPU6500_GYRO_RANGE_500)
#define GYRO_SEN 0.0002663161090079238246038f
#elif defined(MPU6500_GYRO_RANGE_250)
#define GYRO_SEN 0.000133158054503961923019f
#else
#error "Please set the right range of gyro (2000, 1000, 500 or 250)"
#endif

#ifdef MPU6500_ACCEL_RANGE_2G
#define ACCEL_SEN 0.00059814453125f
#define ACCEL_GRAVITY_RAW 16384
#elif defined(MPU6500_ACCEL_RANGE_4G)
#define ACCEL_SEN 0.0011962890625f
#define ACCEL_GRAVITY_RAW 8192
#elif defined(MPU6500_ACCEL_RANGE_8G)
#define ACCEL_SEN 0.002392578125f
#define ACCEL_GRAVITY_RAW 4096
#elif defined(MPU6500_ACCEL_RANGE_16G)
#define ACCEL_SEN 0.00478515625f
#define ACCEL_GRAVITY_RAW 2048
#else
#error "Please set the right range of accel (16G, 8G, 4G or 2G)"
#endif

#ifndef USE_CALIBRATION
#define GX_OFFSET -25
#define GY_OFFSET 0
#define GZ_OFFSET 10
#define AX_OFFSET -24
#define AY_OFFSET -14
#define AZ_OFFSET 198
#endif

static s_FIRST_ORDER_LPF_t accel_lpf_data = {0};
static s_FIRST_ORDER_LPF_t gyro_lpf_data = {0};
static s_FIRST_ORDER_LPF_t euler_rate_lpf_data = {0};
#ifdef USE_MAGNETOMETER
static s_FIRST_ORDER_LPF_t mag_lpf_data = {0};
#endif

static imu_data_t imu = {0};
static int8_t yaw_init_offset = 0;
static mpu_data_t* mpu_data_p = 0;
static uint32_t g_imu_last_sample_seq = 0u;
static OmBool g_imu_has_new_sample = OM_FALSE;

imu_data_t* get_imu_data(void)
{
    return &imu;
}

OmBool imu_has_new_data(void)
{
    return g_imu_has_new_sample;
}

uint8_t mpu_device_init(float gravity)
{
    uint8_t ret = 0U;

    mpu_data_p = get_mpu_data();
    if (mpu_data_p == 0)
    {
        return 1U;
    }

    FirstOrderLPF_Init(&accel_lpf_data, 25.0f, 100.0f);
    FirstOrderLPF_Init(&gyro_lpf_data, 25.0f, 100.0f);
    FirstOrderLPF_Init(&euler_rate_lpf_data, 10.0f, 100.0f);
#ifdef USE_MAGNETOMETER
    FirstOrderLPF_Init(&mag_lpf_data, 25.0f, 100.0f);
#endif

    if (bsp_spi5_init() != OM_OK)
    {
        return 5U;
    }

    ret = mpu6500_init();
    if (ret != 0U)
    {
        return 2U;
    }

#ifdef USE_MAGNETOMETER
    ret = ist8310_init();
    if (ret != 0U)
    {
        return 3U;
    }
#endif

#ifdef USE_CALIBRATION
    ret = mpu_offset_cal(gravity);
    if (ret != 0U)
    {
        return 4U;
    }
#else
    mpu_direct_value_cal();
#endif

    Quaternion_AHRS_InitIMU(mpu_data_p->ax_offset * ACCEL_SEN, mpu_data_p->ay_offset * ACCEL_SEN,
                            mpu_data_p->az_offset * ACCEL_SEN, gravity);

    g_imu_last_sample_seq = 0u;
    g_imu_has_new_sample = OM_FALSE;

    if (imu_bsp_init() != OM_OK)
    {
        return 5U;
    }

    return 0U;
}

void mpu_get_data(void)
{
    uint8_t raw_payload[IMU_BSP_RAW_PAYLOAD_LEN] = {0};
    uint32_t raw_seq = 0u;
    float accel_raw[3] = {0.0f};
    float gyro_raw[3] = {0.0f};
#ifdef USE_MAGNETOMETER
    float mag_raw[3] = {0.0f};
#endif

    g_imu_has_new_sample = OM_FALSE;

    if (mpu_data_p == 0)
    {
        return;
    }

    if (imu_bsp_fetch_latest_raw(raw_payload, &raw_seq) == OM_FALSE)
    {
        return;
    }

    if (raw_seq == g_imu_last_sample_seq)
    {
        return;
    }

    g_imu_last_sample_seq = raw_seq;
    g_imu_bsp_debug.last_processed_seq = raw_seq;

    mpu_data_p->ax = (int16_t)((raw_payload[0] << 8) | raw_payload[1]) - mpu_data_p->ax_offset;
    mpu_data_p->ay = (int16_t)((raw_payload[2] << 8) | raw_payload[3]) - mpu_data_p->ay_offset;
    mpu_data_p->az = (int16_t)((raw_payload[4] << 8) | raw_payload[5]) - mpu_data_p->az_offset;
    mpu_data_p->temp = (int16_t)((raw_payload[6] << 8) | raw_payload[7]);
    mpu_data_p->gx = (int16_t)((raw_payload[8] << 8) | raw_payload[9]) - mpu_data_p->gx_offset;
    mpu_data_p->gy = (int16_t)((raw_payload[10] << 8) | raw_payload[11]) - mpu_data_p->gy_offset;
    mpu_data_p->gz = (int16_t)((raw_payload[12] << 8) | raw_payload[13]) - mpu_data_p->gz_offset;

    imu.ax = mpu_data_p->ax;
    imu.ay = mpu_data_p->ay;
    imu.az = mpu_data_p->az;
    imu.temp = 21.0f + mpu_data_p->temp / 333.87f;

#ifdef USE_MAGNETOMETER
    mpu_data_p->mx = (int16_t)((raw_payload[15] << 8) | raw_payload[14]);
    mpu_data_p->my = (int16_t)((raw_payload[17] << 8) | raw_payload[16]);
    mpu_data_p->mz = (int16_t)((raw_payload[19] << 8) | raw_payload[18]);
    imu.mx = mpu_data_p->mx;
    imu.my = mpu_data_p->my;
    imu.mz = mpu_data_p->mz;
#endif

    gyro_raw[0] = mpu_data_p->gx * GYRO_SEN;
    gyro_raw[1] = mpu_data_p->gy * GYRO_SEN;
    gyro_raw[2] = mpu_data_p->gz * GYRO_SEN;
    accel_raw[0] = mpu_data_p->ax * ACCEL_SEN;
    accel_raw[1] = mpu_data_p->ay * ACCEL_SEN;
    accel_raw[2] = mpu_data_p->az * ACCEL_SEN;

#ifdef USE_MAGNETOMETER
    mag_raw[0] = mpu_data_p->mx / MAG_SEN;
    mag_raw[1] = mpu_data_p->my / MAG_SEN;
    mag_raw[2] = mpu_data_p->mz / MAG_SEN;
#endif

    FirstOrderLPF_Update(accel_raw, &accel_lpf_data);
    FirstOrderLPF_Update(gyro_raw, &gyro_lpf_data);
#ifdef USE_MAGNETOMETER
    FirstOrderLPF_Update(mag_raw, &mag_lpf_data);
#endif

    imu.vx = accel_raw[0];
    imu.vy = accel_raw[1];
    imu.vz = accel_raw[2];
    imu.wx = gyro_raw[0];
    imu.wy = gyro_raw[1];
    imu.wz = gyro_raw[2];

#ifdef USE_MAGNETOMETER
    imu.mag_x = mag_raw[0];
    imu.mag_y = mag_raw[1];
    imu.mag_z = mag_raw[2];
#endif

    g_imu_has_new_sample = OM_TRUE;
}

void mpu_direct_value_cal(void)
{
    mpu_data_t* mpu_data = get_mpu_data();

    if (mpu_data == 0)
    {
        return;
    }

    mpu_data->gx_offset = GX_OFFSET;
    mpu_data->gy_offset = GY_OFFSET;
    mpu_data->gz_offset = GZ_OFFSET;
    mpu_data->ax_offset = AX_OFFSET;
    mpu_data->ay_offset = AY_OFFSET;
    mpu_data->az_offset = AZ_OFFSET;
}

uint8_t mpu_offset_cal(float gravity)
{
    const uint16_t sample_count = 200U;
    uint16_t index = 0U;
    uint8_t raw_buffer[14] = {0};
    int32_t ax_sum = 0;
    int32_t ay_sum = 0;
    int32_t az_sum = 0;
    int32_t gx_sum = 0;
    int32_t gy_sum = 0;
    int32_t gz_sum = 0;

    (void)gravity;

    if (mpu_data_p == 0)
    {
        return 1U;
    }

    for (index = 0U; index < sample_count; index++)
    {
        mpu6500_read_muli_reg(MPU_ACCEL_XOUT_H, raw_buffer, 14U);
        ax_sum += (int16_t)((raw_buffer[0] << 8) | raw_buffer[1]);
        ay_sum += (int16_t)((raw_buffer[2] << 8) | raw_buffer[3]);
        az_sum += (int16_t)((raw_buffer[4] << 8) | raw_buffer[5]);
        gx_sum += (int16_t)((raw_buffer[8] << 8) | raw_buffer[9]);
        gy_sum += (int16_t)((raw_buffer[10] << 8) | raw_buffer[11]);
        gz_sum += (int16_t)((raw_buffer[12] << 8) | raw_buffer[13]);
    }

    mpu_data_p->gx_offset = (int16_t)(gx_sum / sample_count);
    mpu_data_p->gy_offset = (int16_t)(gy_sum / sample_count);
    mpu_data_p->gz_offset = (int16_t)(gz_sum / sample_count);
    mpu_data_p->ax_offset = (int16_t)(ax_sum / sample_count);
    mpu_data_p->ay_offset = (int16_t)(ay_sum / sample_count);
    mpu_data_p->az_offset = (int16_t)(az_sum / sample_count) - ACCEL_GRAVITY_RAW;

    return 0U;
}

void update_attitude(float dt)
{
    float euler_rate_raw[3] = {0.0f};

    if (dt <= 0.0f || dt > 0.1f)
    {
        dt = 0.01f;
    }

#ifdef USE_MAGNETOMETER
    Quaternion_AHRS_Update(imu.wx, imu.wy, imu.wz, imu.vx, imu.vy, imu.vz, imu.mag_x, imu.mag_y, imu.mag_z, dt);
#else
    Quaternion_AHRS_Update(imu.wx, imu.wy, imu.wz, imu.vx, imu.vy, imu.vz, 0.0f, 0.0f, 0.0f, dt);
#endif

    Get_EulerAngle(AHRS.q);
    imu.rol = AHRS.Roll;
    imu.pit = AHRS.Pitch;
    imu.yaw = AHRS.Yaw - (float)yaw_init_offset;

    Get_EulerAngleRates(AHRS.q, imu.wx, imu.wy, imu.wz);
    euler_rate_raw[0] = AHRS.RollRate;
    euler_rate_raw[1] = AHRS.PitchRate;
    euler_rate_raw[2] = AHRS.YawRate;
    FirstOrderLPF_Update(euler_rate_raw, &euler_rate_lpf_data);
    imu.rol_rate = euler_rate_raw[0];
    imu.pit_rate = euler_rate_raw[1];
    imu.yaw_rate = euler_rate_raw[2];
}
