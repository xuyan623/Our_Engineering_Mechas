#include "driver/ahrs/MahonyAHRS.h"

#include <math.h>

AHRS_t AHRS = {0};
QuaternionBuf_t QuaternionBuffer = {0};
float twoKp = 1.5f;
float twoKi = 0.01f;
static float Gravity = 0.0f;
volatile float q0 = 1.0f;
volatile float q1 = 0.0f;
volatile float q2 = 0.0f;
volatile float q3 = 0.0f;
static volatile float integralFBx = 0.0f;
static volatile float integralFBy = 0.0f;
static volatile float integralFBz = 0.0f;

static float invSqrt(float x)
{
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long*)&y;

    i = 0x5f375a86 - (i >> 1);
    y = *(float*)&i;
    y = y * (1.5f - (halfx * y * y));
    return y;
}

float get_gravity(void)
{
    return Gravity;
}

void Quaternion_AHRS_InitIMU(float ax, float ay, float az, float ref_gNorm)
{
    float pitch = 0.0f;
    float roll = 0.0f;

    Gravity = ref_gNorm;
    pitch = atan2f(ay, az);
    roll = -atan2f(ax, az);

    q0 = cosf(pitch / 2.0f) * cosf(roll / 2.0f);
    q1 = sinf(pitch / 2.0f) * cosf(roll / 2.0f);
    q2 = cosf(pitch / 2.0f) * sinf(roll / 2.0f);
    q3 = -sinf(pitch / 2.0f) * sinf(roll / 2.0f);
}

void Quaternion_AHRS_Update(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float dt)
{
    float recipNorm = 0.0f;
    float halfvx = 0.0f;
    float halfvy = 0.0f;
    float halfvz = 0.0f;
    float halfex = 0.0f;
    float halfey = 0.0f;
    float halfez = 0.0f;
    float qa = 0.0f;
    float qb = 0.0f;
    float qc = 0.0f;

#ifdef USE_MAGNETOMETER
    float hx = 0.0f;
    float hy = 0.0f;
    float bx = 0.0f;
    float bz = 0.0f;
    float halfwx = 0.0f;
    float halfwy = 0.0f;
    float halfwz = 0.0f;
    float q0q0 = 0.0f;
    float q0q1 = 0.0f;
    float q0q2 = 0.0f;
    float q0q3 = 0.0f;
    float q1q1 = 0.0f;
    float q1q2 = 0.0f;
    float q1q3 = 0.0f;
    float q2q2 = 0.0f;
    float q2q3 = 0.0f;
    float q3q3 = 0.0f;
#endif

    if (dt <= 0.0f || dt > 0.1f)
    {
        dt = 0.01f;
    }

    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)))
    {
        recipNorm = invSqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        halfvx = q1 * q3 - q0 * q2;
        halfvy = q0 * q1 + q2 * q3;
        halfvz = q0 * q0 - 0.5f + q3 * q3;

#ifdef USE_MAGNETOMETER
        if (!((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f)))
        {
            recipNorm = invSqrt(mx * mx + my * my + mz * mz);
            mx *= recipNorm;
            my *= recipNorm;
            mz *= recipNorm;

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

            hx = 2.0f * (mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2));
            hy = 2.0f * (mx * (q1q2 + q0q3) + my * (0.5f - q1q1 - q3q3) + mz * (q2q3 - q0q1));
            bx = sqrtf(hx * hx + hy * hy);
            bz = 2.0f * (mx * (q1q3 - q0q2) + my * (q2q3 + q0q1) + mz * (0.5f - q1q1 - q2q2));

            halfwx = bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2);
            halfwy = bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3);
            halfwz = bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2);

            halfex = (ay * halfvz - az * halfvy) + (my * halfwz - mz * halfwy);
            halfey = (az * halfvx - ax * halfvz) + (mz * halfwx - mx * halfwz);
            halfez = (ax * halfvy - ay * halfvx) + (mx * halfwy - my * halfwx);
        }
        else
#endif
        {
            halfex = (ay * halfvz - az * halfvy);
            halfey = (az * halfvx - ax * halfvz);
            halfez = (ax * halfvy - ay * halfvx);
        }

        integralFBx += twoKi * halfex * dt;
        integralFBy += twoKi * halfey * dt;
        integralFBz += twoKi * halfez * dt;

        gx += twoKp * halfex + integralFBx;
        gy += twoKp * halfey + integralFBy;
        gz += twoKp * halfez + integralFBz;
    }

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

void Quaternion_AHRS_UpdateIMU(float gx, float gy, float gz, float ax, float ay, float az, float gVecx, float gVecy, float gVecz, float dt)
{
    if (!((fabsf(gVecx) < 0.00001f) && (fabsf(gVecy) < 0.00001f) && (fabsf(gVecz) < 0.00001f)))
    {
        ax = gVecx;
        ay = gVecy;
        az = gVecz;
    }

    Quaternion_AHRS_Update(gx, gy, gz, ax, ay, az, 0.0f, 0.0f, 0.0f, dt);
}

void Get_EulerAngle(float* q)
{
    static int16_t pitch_round_count = 0;
    static int16_t yaw_round_count = 0;
    static float pitch_angle_last = 0.0f;
    static float yaw_angle_last = 0.0f;

    AHRS.Yaw = atan2f(2.0f * (q[0] * q[3] + q[1] * q[2]), 1.0f - 2.0f * (q[2] * q[2] + q[3] * q[3])) * 57.295779513f;
    AHRS.Pitch = asinf(2.0f * (q[0] * q[1] - q[2] * q[3])) * 57.295779513f;
    AHRS.Roll = atan2f(2.0f * (q[0] * q[2] + q[1] * q[3]), 1.0f - 2.0f * (q[1] * q[1] + q[2] * q[2])) * 57.295779513f;

    if (AHRS.Yaw - yaw_angle_last > 180.0f)
    {
        yaw_round_count--;
    }
    else if (AHRS.Yaw - yaw_angle_last < -180.0f)
    {
        yaw_round_count++;
    }

    if (AHRS.Pitch - pitch_angle_last > 180.0f)
    {
        pitch_round_count--;
    }
    else if (AHRS.Pitch - pitch_angle_last < -180.0f)
    {
        pitch_round_count++;
    }

    AHRS.YawTotalAngle = 360.0f * yaw_round_count + AHRS.Yaw;
    AHRS.PitchTotalAngle = 360.0f * pitch_round_count + AHRS.Pitch;
    yaw_angle_last = AHRS.Yaw;
    pitch_angle_last = AHRS.Pitch;
}

void Get_EulerAngleRates(float* q, float gx, float gy, float gz)
{
    float phi = 0.0f;
    float theta = 0.0f;
    float sin_phi = 0.0f;
    float cos_phi = 0.0f;
    float tan_theta = 0.0f;
    float sec_theta = 0.0f;
    float roll_rate = 0.0f;
    float pitch_rate = 0.0f;
    float yaw_rate = 0.0f;

    (void)q;

    phi = AHRS.Roll / 57.295779513f;
    theta = AHRS.Pitch / 57.295779513f;
    sin_phi = sinf(phi);
    cos_phi = cosf(phi);
    tan_theta = tanf(theta);
    sec_theta = 1.0f / cosf(theta);

    if (fabsf(sec_theta) > 10.0f)
    {
        sec_theta = (sec_theta > 0.0f) ? 10.0f : -10.0f;
    }

    roll_rate = gx + sin_phi * tan_theta * gy + cos_phi * tan_theta * gz;
    pitch_rate = cos_phi * gy - sin_phi * gz;
    yaw_rate = sin_phi * sec_theta * gy + cos_phi * sec_theta * gz;

    AHRS.RollRate = roll_rate * 57.295779513f;
    AHRS.PitchRate = pitch_rate * 57.295779513f;
    AHRS.YawRate = yaw_rate * 57.295779513f;
}

void QuaternionToEularAngle(float* q, float* Yaw, float* Pitch, float* Roll)
{
    *Yaw = atan2f(2.0f * (q[0] * q[3] + q[1] * q[2]), 1.0f - 2.0f * (q[2] * q[2] + q[3] * q[3])) * 57.295779513f;
    *Pitch = asinf(2.0f * (q[0] * q[1] - q[2] * q[3])) * 57.295779513f;
    *Roll = atan2f(2.0f * (q[0] * q[2] + q[1] * q[3]), 1.0f - 2.0f * (q[1] * q[1] + q[2] * q[2])) * 57.295779513f;
}

void EularAngleToQuaternion(float Yaw, float Pitch, float Roll, float* q)
{
    float cosPitch = 0.0f;
    float cosYaw = 0.0f;
    float cosRoll = 0.0f;
    float sinPitch = 0.0f;
    float sinYaw = 0.0f;
    float sinRoll = 0.0f;

    Yaw /= 57.295779513f;
    Pitch /= 57.295779513f;
    Roll /= 57.295779513f;
    cosPitch = cosf(Pitch / 2.0f);
    cosYaw = cosf(Yaw / 2.0f);
    cosRoll = cosf(Roll / 2.0f);
    sinPitch = sinf(Pitch / 2.0f);
    sinYaw = sinf(Yaw / 2.0f);
    sinRoll = sinf(Roll / 2.0f);

    q[0] = cosPitch * cosRoll * cosYaw + sinPitch * sinRoll * sinYaw;
    q[1] = sinPitch * cosRoll * cosYaw - cosPitch * sinRoll * sinYaw;
    q[2] = sinPitch * cosRoll * sinYaw + cosPitch * sinRoll * cosYaw;
    q[3] = cosPitch * cosRoll * sinYaw - sinPitch * sinRoll * cosYaw;
}

void InsertQuaternionFrame(QuaternionBuf_t* qBuf, float* q, float time_stamp)
{
    uint16_t index = 0U;

    if (qBuf->LatestNum == (Q_FRAME_LEN - 1U))
    {
        qBuf->LatestNum = 0U;
    }
    else
    {
        qBuf->LatestNum++;
    }

    qBuf->qFrame[qBuf->LatestNum].TimeStamp = time_stamp;
    for (index = 0U; index < 4U; index++)
    {
        qBuf->qFrame[qBuf->LatestNum].q[index] = q[index];
    }
}

uint16_t FindTimeMatchFrame(QuaternionBuf_t* qBuf, float match_time_stamp)
{
    float min_time_error = fabsf(qBuf->qFrame[0].TimeStamp - match_time_stamp);
    uint16_t number = 0U;
    uint16_t index = 0U;

    for (index = 0U; index < Q_FRAME_LEN; index++)
    {
        const float time_error = fabsf(qBuf->qFrame[index].TimeStamp - match_time_stamp);
        if (time_error < min_time_error)
        {
            min_time_error = time_error;
            number = index;
        }
    }
    return number;
}

void BodyFrameToEarthFrame(float* vecBF, float* vecEF, float* q)
{
    vecEF[0] = 2.0f * ((0.5f - q[2] * q[2] - q[3] * q[3]) * vecBF[0] + (q[1] * q[2] - q[0] * q[3]) * vecBF[1] +
                       (q[1] * q[3] + q[0] * q[2]) * vecBF[2]);
    vecEF[1] = 2.0f * ((q[1] * q[2] + q[0] * q[3]) * vecBF[0] + (0.5f - q[1] * q[1] - q[3] * q[3]) * vecBF[1] +
                       (q[2] * q[3] - q[0] * q[1]) * vecBF[2]);
    vecEF[2] = 2.0f * ((q[1] * q[3] - q[0] * q[2]) * vecBF[0] + (q[2] * q[3] + q[0] * q[1]) * vecBF[1] +
                       (0.5f - q[1] * q[1] - q[2] * q[2]) * vecBF[2]);
}

void EarthFrameToBodyFrame(float* vecEF, float* vecBF, float* q)
{
    vecBF[0] = 2.0f * ((0.5f - q[2] * q[2] - q[3] * q[3]) * vecEF[0] + (q[1] * q[2] + q[0] * q[3]) * vecEF[1] +
                       (q[1] * q[3] - q[0] * q[2]) * vecEF[2]);
    vecBF[1] = 2.0f * ((q[1] * q[2] - q[0] * q[3]) * vecEF[0] + (0.5f - q[1] * q[1] - q[3] * q[3]) * vecEF[1] +
                       (q[2] * q[3] + q[0] * q[1]) * vecEF[2]);
    vecBF[2] = 2.0f * ((q[1] * q[3] + q[0] * q[2]) * vecEF[0] + (q[2] * q[3] - q[0] * q[1]) * vecEF[1] +
                       (0.5f - q[1] * q[1] - q[2] * q[2]) * vecEF[2]);
}
