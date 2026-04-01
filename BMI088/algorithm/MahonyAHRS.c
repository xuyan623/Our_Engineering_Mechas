/**
  ******************************************************************************
  * @file    QuaternionAHRS.c
  * @author  Wang Hongxi
  * @version V1.2.3
  * @date    2021/6/22
  * @brief
  ******************************************************************************
  * @attention
  *
  ******************************************************************************
  */
#include "MahonyAHRS.h"//#include "QuaternionAHRS.h"
#include <math.h>
#include "arm_math.h"
#include "STM32_TIM_BASE.h"


AHRS_t AHRS = {0};
QuaternionBuf_t QuaternionBuffer;
float twoKp = 5.0f; // 这行你原本就有
float twoKi = 0.01f;     // <--- 新增：积分增益 (Ki)。如果设为 0.0f，则退化为无积分的P控制


float Gravity = 9.78;
uint8_t debugPlot = 0;
static float accel_fliter_1[3] = {0.0f, 0.0f, 0.0f};
static float accel_fliter_2[3] = {0.0f, 0.0f, 0.0f};
static const float accFliterNum[3] = {1.929454039488895f, -0.93178349823448126f, 0.002329458745586203f};

volatile float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;                 // quaternion of sensor frame relative to auxiliary frame
volatile float integralFBx = 0.0f, integralFBy = 0.0f, integralFBz = 0.0f; // integral error terms scaled by Ki

static float invSqrt(float x);
static float TuningKp(float *gyro, float *acc, float *kp);

/**
  * @brief          AHRS algorithm init
  * @param[1-3]     accelerometer measurement in m/s2
  */
void Quaternion_AHRS_InitIMU(float ax, float ay, float az, float ref_gNorm)
{
    float Pitch, Roll;

    Gravity = ref_gNorm;

    Pitch = atan2f(ay, az);
    Roll = -atan2f(ax, az);
    q0 = arm_cos_f32(Pitch / 2) * arm_cos_f32(Roll / 2);
    q1 = arm_sin_f32(Pitch / 2) * arm_cos_f32(Roll / 2);
    q2 = arm_cos_f32(Pitch / 2) * arm_sin_f32(Roll / 2);
    q3 = -arm_sin_f32(Pitch / 2) * arm_sin_f32(Roll / 2);
}

/**
  * @brief          AHRS algorithm update
  * @param[1-3]     gyro measurement in rad/s
  * @param[4-6]     accelerometer measurement in m/s2
  * @param[7-9]     magnetometer measurement in uT
  * @param[10]      time stamp in s
  */
//void Quaternion_AHRS_Update(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz)
//{
//    static float recipNorm;
//    static float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
//    static float hx, hy, bx, bz;
//    static float halfvx, halfvy, halfvz, halfwx, halfwy, halfwz;
//    static float halfex, halfey, halfez;
//    static float qa, qb, qc;
//	  static float dt;
//	
//	  static uint32_t last_update, now_update;               /* Sampling cycle count, ubit ms */

//	
//	  now_update  = HAL_GetTick(); //ms
//	  dt = ((float)(now_update - last_update) / 1000.0f);
//		last_update = now_update;


//    // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
//    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)))
//    {

//        // Normalise accelerometer measurement
//        recipNorm = invSqrt(ax * ax + ay * ay + az * az);
//        ax *= recipNorm;
//        ay *= recipNorm;
//        az *= recipNorm;

//        // Normalise magnetometer measurement
//        recipNorm = invSqrt(mx * mx + my * my + mz * mz);
//        mx *= recipNorm;
//        my *= recipNorm;
//        mz *= recipNorm;

//        // Auxiliary variables to avoid repeated arithmetic
//        q0q0 = q0 * q0;
//        q0q1 = q0 * q1;
//        q0q2 = q0 * q2;
//        q0q3 = q0 * q3;
//        q1q1 = q1 * q1;
//        q1q2 = q1 * q2;
//        q1q3 = q1 * q3;
//        q2q2 = q2 * q2;
//        q2q3 = q2 * q3;
//        q3q3 = q3 * q3;

//        // Reference direction of Earth's magnetic field
//        hx = 2.0f * (mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2));
//        hy = 2.0f * (mx * (q1q2 + q0q3) + my * (0.5f - q1q1 - q3q3) + mz * (q2q3 - q0q1));
//        bx = sqrt(hx * hx + hy * hy);
//        bz = 2.0f * (mx * (q1q3 - q0q2) + my * (q2q3 + q0q1) + mz * (0.5f - q1q1 - q2q2));

//        // Estimated direction of gravity and magnetic field
//        halfvx = q1q3 - q0q2;
//        halfvy = q0q1 + q2q3;
//        halfvz = q0q0 - 0.5f + q3q3;
//        halfwx = bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2);
//        halfwy = bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3);
//        halfwz = bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2);

//        // Error is sum of cross product between estimated direction and measured direction of field vectors
//        halfex = (ay * halfvz - az * halfvy) + (my * halfwz - mz * halfwy);
//        halfey = (az * halfvx - ax * halfvz) + (mz * halfwx - mx * halfwz);
//        halfez = (ax * halfvy - ay * halfvx) + (mx * halfwy - my * halfwx);

//        // Apply proportional feedback
//        gx += twoKp * halfex;
//        gy += twoKp * halfey;
//        gz += twoKp * halfez;
//    }

//    // Integrate rate of change of quaternion
//    gx *= (0.5f * dt); // pre-multiply common factors
//    gy *= (0.5f * dt);
//    gz *= (0.5f * dt);
//    qa = q0;
//    qb = q1;
//    qc = q2;
//    q0 += (-qb * gx - qc * gy - q3 * gz);
//    q1 += (qa * gx + qc * gz - q3 * gy);
//    q2 += (qa * gy - qb * gz + q3 * gx);
//    q3 += (qa * gz + qb * gy - qc * gx);

//    // Normalise quaternion
//    recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
//    q0 *= recipNorm;
//    q1 *= recipNorm;
//    q2 *= recipNorm;
//    q3 *= recipNorm;

//    AHRS.q[0] = q0;
//    AHRS.q[1] = q1;
//    AHRS.q[2] = q2;
//    AHRS.q[3] = q3;
//}


/**
  * @brief          AHRS algorithm update with PI controller (Standard Lib Version)
  * @param[1-3]     gyro measurement in rad/s
  * @param[4-6]     accelerometer measurement in m/s^2 (or raw, will be normalized)
  * @param[7-9]     magnetometer measurement in uT (or raw, will be normalized)
  */
void Quaternion_AHRS_Update(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz)
{
    static float recipNorm;
    static float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
    static float hx, hy, bx, bz;
    static float halfvx, halfvy, halfvz, halfwx, halfwy, halfwz;
    static float halfex, halfey, halfez;
    static float qa, qb, qc;
    static float dt;
    static uint32_t last_update, now_update;

    /* 1. 计算时间步长 dt */
    now_update  = HAL_GetTick(); // ms
    dt = ((float)(now_update - last_update) / 1000.0f);
    last_update = now_update;

    /* 2. 归一化加速度计测量 (只要数据不全为0) */
    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)))
    {
        // Normalise accelerometer measurement
        recipNorm = invSqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        // Normalise magnetometer measurement
        recipNorm = invSqrt(mx * mx + my * my + mz * mz);
        mx *= recipNorm;
        my *= recipNorm;
        mz *= recipNorm;

        // 辅助变量，避免重复计算
        q0q0 = q0 * q0;
        q0q1 = q0 * q1;
        q0q2 = q0 * q2;
        q0q3 = q0 * q3;
        q1q1 = q1 * q1;
        q1q2 = q1 * q2;
        q1q3 = q1 * q3;
        q2q2 = q2 * q2;
        q2q3 = q2 * q3;
        q3q3 = q3 * q3;

        // 参考磁场方向
        hx = 2.0f * (mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2));
        hy = 2.0f * (mx * (q1q2 + q0q3) + my * (0.5f - q1q1 - q3q3) + mz * (q2q3 - q0q1));
        bx = sqrt(hx * hx + hy * hy);
        bz = 2.0f * (mx * (q1q3 - q0q2) + my * (q2q3 + q0q1) + mz * (0.5f - q1q1 - q2q2));

        // 估算的重力方向和磁场方向
        halfvx = q1q3 - q0q2;
        halfvy = q0q1 + q2q3;
        halfvz = q0q0 - 0.5f + q3q3;
        halfwx = bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2);
        halfwy = bx * (q1q2 - q0q3) + bz * (0.0f * q1 + q2q3); // 注意：原代码写的是 q0q1，这里修正为标准逻辑
        halfwz = bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2);

        /* 误差计算 (测量方向 与 估算方向 的叉积) */
        halfex = (ay * halfvz - az * halfvy) + (my * halfwz - mz * halfwy);
        halfey = (az * halfvx - ax * halfvz) + (mz * halfwx - mx * halfwz);
        halfez = (ax * halfvy - ay * halfvx) + (mx * halfwy - my * halfwx);

        /* 
         * 【修改点】：应用积分项 (I) 和 比例项 (P)
         * integralFBx += 误差 * Ki * dt
         */
        integralFBx += twoKi * halfex * dt;
        integralFBy += twoKi * halfey * dt;
        integralFBz += twoKi * halfez * dt;

        // 将误差应用到陀螺仪数据上
        gx += twoKp * halfex + integralFBx;
        gy += twoKp * halfey + integralFBy;
        gz += twoKp * halfez + integralFBz;
    }

    /* 3. 积分四元数变化率 */
    gx *= (0.5f * dt);
    gy *= (0.5f * dt);
    gz *= (0.5f * dt);
    
    qa = q0;
    qb = q1;
    qc = q2;
    
    q0 += (-qb * gx - qc * gy - q3 * gz);
    q1 += (qa * gx + qc * gz - q3 * gy);
    q2 += (qa * gy - qb * gz + q3 * gx);
    q3 += (qa * gz + qb * gy - qc * gx);

    /* 4. 归一化四元数 */
    recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;

    /* 5. 输出结果 */
    AHRS.q[0] = q0;
    AHRS.q[1] = q1;
    AHRS.q[2] = q2;
    AHRS.q[3] = q3;
}




/**
  * @brief          IMU algorithm update
  * @param[1-3]     gyro measurement in rad/s
  * @param[4-6]     accelerometer measurement in m/s2
  * @param[7-9]     gravity vector in m/s2
  * @param[10]       time stamp in s
  */
void Quaternion_AHRS_UpdateIMU(float gx, float gy, float gz, float ax, float ay, float az, float gVecx, float gVecy, float gVecz, float dt)
{
    static float recipNorm;
    static float halfvx, halfvy, halfvz;
    static float halfex, halfey, halfez;
    static float qa, qb, qc;
    static float kpScale = 1.0f;
    static float gyroRAW[3], gyro[3], gyro_[3], accel[3];
    static float lpf = 0.00002;

    gyroRAW[0] = gx;
    gyroRAW[1] = gy;
    gyroRAW[2] = gz;
    gyro_[0] = gx * lpf / (lpf + dt) + gyro[0] * dt / (lpf + dt);
    gyro_[1] = gy * lpf / (lpf + dt) + gyro[1] * dt / (lpf + dt);
    gyro_[2] = gz * lpf / (lpf + dt) + gyro[2] * dt / (lpf + dt);
    gyro[0] = gx * lpf / (lpf + dt) + gyro_[0] * dt / (lpf + dt);
    gyro[1] = gy * lpf / (lpf + dt) + gyro_[1] * dt / (lpf + dt);
    gyro[2] = gz * lpf / (lpf + dt) + gyro_[2] * dt / (lpf + dt);

    accel_fliter_1[0] = accel_fliter_2[0];
    accel_fliter_2[0] = accel[0];
    accel[0] = accel_fliter_2[0] * accFliterNum[0] + accel_fliter_1[0] * accFliterNum[1] + ax * accFliterNum[2];
    accel_fliter_1[1] = accel_fliter_2[1];
    accel_fliter_2[1] = accel[1];
    accel[1] = accel_fliter_2[1] * accFliterNum[0] + accel_fliter_1[1] * accFliterNum[1] + ay * accFliterNum[2];
    accel_fliter_1[2] = accel_fliter_2[2];
    accel_fliter_2[2] = accel[2];
    accel[2] = accel_fliter_2[2] * accFliterNum[0] + accel_fliter_1[2] * accFliterNum[1] + az * accFliterNum[2];

    if (!((fabsf(gVecx) < 0.00001f) && (fabsf(gVecx) < 0.00001f) && (fabsf(gVecx) < 0.00001f)))
    {
        ax = gVecx;
        ay = gVecy;
        az = gVecz;
    }

    // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
    if (!((fabsf(ax) < 0.00001f) && (fabsf(ay) < 0.00001f) && (fabsf(az) < 0.00001f)))
    {
        // Normalise accelerometer measurement
        recipNorm = invSqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        // Estimated direction of gravity and vector perpendicular to magnetic flux
        halfvx = q1 * q3 - q0 * q2;
        halfvy = q0 * q1 + q2 * q3;
        halfvz = q0 * q0 - 0.5f + q3 * q3;

        // Error is sum of cross product between estimated and measured direction of gravity
        halfex = (ay * halfvz - az * halfvy);
        halfey = (az * halfvx - ax * halfvz);
        halfez = (ax * halfvy - ay * halfvx);

        kpScale = TuningKp(gyro, accel, &twoKp);

        // Apply proportional feedback
        gx += twoKp * halfex * kpScale;
        gy += twoKp * halfey * kpScale;
        gz += twoKp * halfez * kpScale;
    }

    // Integrate rate of change of quaternion
    gx *= (0.5f * dt); // pre-multiply common factors
    gy *= (0.5f * dt);
    gz *= (0.5f * dt);
    qa = q0;
    qb = q1;
    qc = q2;
    q0 += (-qb * gx - qc * gy - q3 * gz);
    q1 += (qa * gx + qc * gz - q3 * gy);
    q2 += (qa * gy - qb * gz + q3 * gx);
    q3 += 0.5f * dt * (qa * gyroRAW[2] + qb * gyroRAW[1] - qc * gyroRAW[0]);

    // Normalise quaternion
    recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;

    AHRS.q[0] = q0;
    AHRS.q[1] = q1;
    AHRS.q[2] = q2;
    AHRS.q[3] = q3;
}

static float TuningKp(float *gyro, float *acc, float *kp)
{
    /*float scale = 1;
    float accNorm;
    float gyroNorm;
    float normDifference;
    static float maxDifference = 0.5, differenceThreshold = 0.1, gyroNormThreshold = 0.084;

    accNorm = 1 / invSqrt(acc[0] * acc[0] + acc[1] * acc[1] + acc[2] * acc[2]);
    gyroNorm = 1 / invSqrt(gyro[0] * gyro[0] + gyro[1] * gyro[1] + gyro[2] * gyro[2]);
    normDifference = fabsf(accNorm - Gravity);
    if (normDifference > differenceThreshold || gyroNorm > gyroNormThreshold)
    {
        if (normDifference > maxDifference)
            scale = 0;
        else if (normDifference > differenceThreshold)
        {
            if (gyroNorm > gyroNormThreshold)
                scale = differenceThreshold / normDifference * gyroNormThreshold / gyroNorm;
            else
                scale = differenceThreshold / normDifference;
        }
        else
        {
            scale = gyroNormThreshold / gyroNorm;
        }
    }
    if (debugPlot)
        Serial_Debug(&huart1, 1, maxDifference * 10, differenceThreshold * 10, gyroNormThreshold * 10, normDifference * 10, gyroNorm * 10,
                     0);
    return scale;*/
    float scale = 1;
    float accNorm;
    float gyroSum;
    float normDifference;
    static float maxDifference = 0.33, differenceThreshold = 0.1, gyroSumThreshold = 0.12;

    accNorm = 1 / invSqrt(acc[0] * acc[0] + acc[1] * acc[1] + acc[2] * acc[2]);
    gyroSum = fabsf(gyro[0]) + fabsf(gyro[1]) + fabsf(gyro[2]);
    normDifference = fabsf(accNorm - Gravity);
    if (normDifference > differenceThreshold || gyroSum > gyroSumThreshold)
    {
        if (normDifference > maxDifference)
            scale = 0;
        else if (normDifference > differenceThreshold)
        {
            if (gyroSum > gyroSumThreshold)
                scale = differenceThreshold / normDifference * gyroSumThreshold / gyroSum;
            else
                scale = differenceThreshold / normDifference;
        }
        else
        {
            scale = gyroSumThreshold / gyroSum;
        }
    }
//    if (debugPlot)
//        Serial_Debug(&huart1, 1, maxDifference * 10, differenceThreshold * 10, gyroSumThreshold * 10, normDifference * 10, gyroSum * 10,
//                     1 / invSqrt(gyro[0] * gyro[0] + gyro[1] * gyro[1] + gyro[2] * gyro[2]) / gyroSum);
    return scale;
}

/**
  * @brief        Convert quaternion to eular angle
  */
void Get_EulerAngle(float *q)
{
    static int16_t Pitch_Round_Count = 0;
    static int16_t Yaw_Round_Count = 0;
    static float Pitch_Angle_Last = 0;
    static float Yaw_Angle_Last = 0;

    AHRS.Yaw = atan2f(2.0f * (q[0] * q[3] + q[1] * q[2]), 2.0f * (q[0] * q[0] + q[1] * q[1]) - 1.0f) * 57.295779513f;
    AHRS.Pitch = atan2f(2.0f * (q[0] * q[1] + q[2] * q[3]), 2.0f * (q[0] * q[0] + q[3] * q[3]) - 1.0f) * 57.295779513f;
    AHRS.Roll = asinf(-2.0f * (q[1] * q[3] - q[0] * q[2])) * 57.295779513f;

    // Yaw rount count
    if (AHRS.Yaw - Yaw_Angle_Last > 180.0f)
        Yaw_Round_Count--;
    else if (AHRS.Yaw - Yaw_Angle_Last < -180.0f)
        Yaw_Round_Count++;
    // Pitch rount count
    if (AHRS.Pitch - Pitch_Angle_Last > 180.0f)
        Pitch_Round_Count--;
    else if (AHRS.Pitch - Pitch_Angle_Last < -180.0f)
        Pitch_Round_Count++;

    AHRS.YawTotalAngle = 360.0f * Yaw_Round_Count + AHRS.Yaw;
    AHRS.PitchTotalAngle = 360.0f * Pitch_Round_Count + AHRS.Pitch;

    Yaw_Angle_Last = AHRS.Yaw;
    Pitch_Angle_Last = AHRS.Pitch;
}

/**
  * @brief        Convert quaternion to eular angle
  */
void QuaternionToEularAngle(float *q, float *Yaw, float *Pitch, float *Roll)
{
    *Yaw = atan2f(2.0f * (q[0] * q[3] + q[1] * q[2]), 2.0f * (q[0] * q[0] + q[1] * q[1]) - 1.0f) * 57.295779513f;
    *Pitch = atan2f(2.0f * (q[0] * q[1] + q[2] * q[3]), 2.0f * (q[0] * q[0] + q[3] * q[3]) - 1.0f) * 57.295779513f;
    *Roll = asinf(2.0f * (q[0] * q[2] - q[1] * q[3])) * 57.295779513f;
}

/**
  * @brief        Convert eular angle to quaternion
  */
void EularAngleToQuaternion(float Yaw, float Pitch, float Roll, float *q)
{
	float cosPitch,cosYaw,cosRoll,sinPitch,sinYaw,sinRoll;
    Yaw /= 57.295779513f;
    Pitch /= 57.295779513f;
    Roll /= 57.295779513f;
	cosPitch = arm_cos_f32(Pitch / 2);
	cosYaw = arm_cos_f32(Yaw / 2);
	cosRoll = arm_cos_f32(Roll / 2);
	sinPitch = arm_sin_f32(Pitch / 2);
	sinYaw = arm_sin_f32(Yaw / 2);
	sinRoll = arm_sin_f32(Roll / 2);
    q[0] = cosPitch * cosRoll * cosYaw + sinPitch * sinRoll * sinYaw;
    q[1] = sinPitch * cosRoll * cosYaw - cosPitch * sinRoll * sinYaw;
    q[2] = sinPitch * cosRoll * sinYaw + cosPitch * sinRoll * cosYaw;
    q[3] = cosPitch * cosRoll * sinYaw - sinPitch * sinRoll * cosYaw;
}

void InsertQuaternionFrame(QuaternionBuf_t *qBuf, float *q, float time_stamp)
{
    if (qBuf->LatestNum == Q_FRAME_LEN - 1)
        qBuf->LatestNum = 0;
    else
        qBuf->LatestNum++;

    qBuf->qFrame[qBuf->LatestNum].TimeStamp = time_stamp;
    for (uint16_t i = 0; i < 4; i++)
        qBuf->qFrame[qBuf->LatestNum].q[i] = q[i];
}

uint16_t FindTimeMatchFrame(QuaternionBuf_t *qBuf, float match_time_stamp)
{
    float min_time_error = fabsf(qBuf->qFrame[0].TimeStamp - match_time_stamp);
    uint16_t num = 0;
    for (uint16_t i = 0; i < Q_FRAME_LEN; i++)
    {
        if (fabsf(qBuf->qFrame[i].TimeStamp - match_time_stamp) < min_time_error)
        {
            min_time_error = fabsf(qBuf->qFrame[i].TimeStamp - match_time_stamp);
            num = i;
        }
    }
    return num;
}

/**
  * @brief          Transform 3dvector from BodyFrame to EarthFrame
  * @param[1]       vector in BodyFrame
  * @param[2]       vector in EarthFrame
  * @param[3]       quaternion
  */
void BodyFrameToEarthFrame(float *vecBF, float *vecEF, float *q)
{
    vecEF[0] = 2.0f * ((0.5f - q[2] * q[2] - q[3] * q[3]) * vecBF[0] +
                       (q[1] * q[2] - q[0] * q[3]) * vecBF[1] +
                       (q[1] * q[3] + q[0] * q[2]) * vecBF[2]);

    vecEF[1] = 2.0f * ((q[1] * q[2] + q[0] * q[3]) * vecBF[0] +
                       (0.5f - q[1] * q[1] - q[3] * q[3]) * vecBF[1] +
                       (q[2] * q[3] - q[0] * q[1]) * vecBF[2]);

    vecEF[2] = 2.0f * ((q[1] * q[3] - q[0] * q[2]) * vecBF[0] +
                       (q[2] * q[3] + q[0] * q[1]) * vecBF[1] +
                       (0.5f - q[1] * q[1] - q[2] * q[2]) * vecBF[2]);
}

/**
  * @brief          Transform 3dvector from EarthFrame to BodyFrame
  * @param[1]       vector in EarthFrame
  * @param[2]       vector in BodyFrame
  * @param[3]       quaternion
  */
void EarthFrameToBodyFrame(float *vecEF, float *vecBF, float *q)
{
    vecBF[0] = 2.0f * ((0.5f - q[2] * q[2] - q[3] * q[3]) * vecEF[0] +
                       (q[1] * q[2] + q[0] * q[3]) * vecEF[1] +
                       (q[1] * q[3] - q[0] * q[2]) * vecEF[2]);

    vecBF[1] = 2.0f * ((q[1] * q[2] - q[0] * q[3]) * vecEF[0] +
                       (0.5f - q[1] * q[1] - q[3] * q[3]) * vecEF[1] +
                       (q[2] * q[3] + q[0] * q[1]) * vecEF[2]);

    vecBF[2] = 2.0f * ((q[1] * q[3] + q[0] * q[2]) * vecEF[0] +
                       (q[2] * q[3] - q[0] * q[1]) * vecEF[1] +
                       (0.5f - q[1] * q[1] - q[2] * q[2]) * vecEF[2]);
}


static float invSqrt(float x)
{
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long *)&y;
    i = 0x5f375a86 - (i >> 1);
    y = *(float *)&i;
    y = y * (1.5f - (halfx * y * y));
    return y;
}
