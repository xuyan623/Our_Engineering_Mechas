#ifndef NEW_ROBOT_DATA_POOL_H
#define NEW_ROBOT_DATA_POOL_H

#include <stdint.h>

#define DP_CUSTOM_CONTROLLER_ANGLE_COUNT (6u)

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
} DpImuSnapshot;

typedef struct
{
    int16_t x;
    int16_t y;
    int16_t z;
    uint8_t l;
    uint8_t r;
} DpMouseSnapshot;

typedef struct
{
    int16_t ch1;
    int16_t ch2;
    int16_t ch3;
    int16_t ch4;
    uint8_t sw1;
    uint8_t sw2;
    uint16_t iw;
    uint8_t online;
    DpMouseSnapshot mouse;
    uint16_t keyboard_bits;
} DpRcSnapshot;

typedef struct
{
    uint8_t global_mode;
    uint8_t chassis_mode;
} DpModeSnapshot;

typedef struct
{
    uint8_t clamp_action;
    uint8_t exchange_action;
    uint8_t primary_turn_ore_flag;
    uint8_t custom_controller_force_takeover_flag;
} DpActionSnapshot;

typedef struct
{
    uint8_t global_mode;
    uint8_t chassis_mode;
    uint8_t clamp_action;
    uint8_t exchange_action;
    uint8_t primary_turn_ore_flag;
    uint8_t custom_controller_force_takeover_flag;
} DpModeCompatSnapshot;

typedef struct
{
    uint8_t online;
    uint8_t work_mode;
    float angle_deg[DP_CUSTOM_CONTROLLER_ANGLE_COUNT];
} DpCustomControllerSnapshot;

typedef struct
{
    DpImuSnapshot imu;
    DpRcSnapshot rc;
    DpModeSnapshot mode;
    DpActionSnapshot action;
    DpCustomControllerSnapshot custom_controller;
} DataPool;

void dp_copy_imu_snapshot(DpImuSnapshot* snapshot);
void dp_store_imu_snapshot(const DpImuSnapshot* snapshot);

void dp_copy_rc_snapshot(DpRcSnapshot* snapshot);
void dp_store_rc_snapshot(const DpRcSnapshot* snapshot);

void dp_copy_mode_compat_snapshot(DpModeCompatSnapshot* snapshot);
void dp_store_mode_compat_snapshot(const DpModeCompatSnapshot* snapshot);

void dp_copy_custom_controller_snapshot(DpCustomControllerSnapshot* snapshot);
void dp_store_custom_controller_snapshot(const DpCustomControllerSnapshot* snapshot);

extern DataPool g_data_pool;

#endif
