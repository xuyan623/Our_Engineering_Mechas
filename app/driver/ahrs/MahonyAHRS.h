#ifndef NEW_ROBOT_MAHONY_AHRS_H
#define NEW_ROBOT_MAHONY_AHRS_H

#include "driver/imu/imu.h"
#include <stdint.h>

typedef struct
{
    float q[4];
    float Accel[3];
    float Gyro[3];
    float Yaw;
    float Pitch;
    float Roll;
    float YawTotalAngle;
    float PitchTotalAngle;
    float YawRate;
    float PitchRate;
    float RollRate;
} AHRS_t;

typedef struct
{
    float q[4];
    float TimeStamp;
} QuaternionFrame_t;

#define Q_FRAME_LEN 50U

typedef struct
{
    QuaternionFrame_t qFrame[Q_FRAME_LEN];
    uint16_t LatestNum;
} QuaternionBuf_t;

extern float twoKp;
extern float twoKi;
extern volatile float q0;
extern volatile float q1;
extern volatile float q2;
extern volatile float q3;
extern AHRS_t AHRS;
extern QuaternionBuf_t QuaternionBuffer;

float get_gravity(void);
void Quaternion_AHRS_InitIMU(float ax, float ay, float az, float ref_gNorm);
void Quaternion_AHRS_Update(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float dt);
void Quaternion_AHRS_UpdateIMU(float gx, float gy, float gz, float ax, float ay, float az, float gVecx, float gVecy, float gVecz, float dt);
void Get_EulerAngle(float* q);
void Get_EulerAngleRates(float* q, float gx, float gy, float gz);
void QuaternionToEularAngle(float* q, float* Yaw, float* Pitch, float* Roll);
void EularAngleToQuaternion(float Yaw, float Pitch, float Roll, float* q);
void InsertQuaternionFrame(QuaternionBuf_t* qBuf, float* q, float time_stamp);
uint16_t FindTimeMatchFrame(QuaternionBuf_t* qBuf, float match_time_stamp);
void BodyFrameToEarthFrame(float* vecBF, float* vecEF, float* q);
void EarthFrameToBodyFrame(float* vecEF, float* vecBF, float* q);

#endif
