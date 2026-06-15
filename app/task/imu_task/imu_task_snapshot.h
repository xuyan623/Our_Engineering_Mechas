#ifndef NEW_ROBOT_IMU_TASK_SNAPSHOT_H
#define NEW_ROBOT_IMU_TASK_SNAPSHOT_H

typedef struct
{
    float yaw;
    float pitch;
    float roll;
    float yaw_rate;
    float pitch_rate;
    float roll_rate;
    float wx;
    float wy;
    float wz;
    float ax;
    float ay;
    float az;
    float temp;
} ImuTaskSnapshot;

#endif
