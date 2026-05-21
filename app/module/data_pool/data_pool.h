#ifndef NEW_ROBOT_DATA_POOL_H
#define NEW_ROBOT_DATA_POOL_H

#include "atomic/atomic_simple.h"
#include <stdint.h>
#include <string.h>

typedef struct
{
    uint32_t v;
} DpFloat;

typedef struct
{
    int16_t v;
} DpInt16;

typedef struct
{
    uint8_t v;
} DpUint8;

typedef struct
{
    uint16_t v;
} DpUint16;

typedef struct
{
    struct
    {
        DpFloat yaw;
        DpFloat pitch;
        DpFloat roll;
        DpFloat yaw_rate;
        DpFloat pitch_rate;
        DpFloat roll_rate;
        DpFloat wx;
        DpFloat wy;
        DpFloat wz;
        DpFloat ax;
        DpFloat ay;
        DpFloat az;
        DpFloat temp;
    } imu;

    struct
    {
        DpInt16 ch1;
        DpInt16 ch2;
        DpInt16 ch3;
        DpInt16 ch4;
        DpUint8 sw1;
        DpUint8 sw2;
        /* Old DBUS protocol encodes the wheel as an unsigned 11-bit value. */
        DpUint16 iw;
        struct
        {
            DpInt16 x;
            DpInt16 y;
            DpInt16 z;
            DpUint8 l;
            DpUint8 r;
        } mouse;
        DpUint16 keyboard_bits;
    } rc;

    /* Shared control facts only; edge history stays inside task-local context. */
    struct
    {
        DpUint8 global_mode;
        DpUint8 chassis_mode;
    } mode;

    struct
    {
        DpUint8 clamp_action;
        DpUint8 exchange_action;
        DpUint8 primary_turn_ore_flag;
        DpUint8 custom_controller_force_takeover_flag;
    } action;

    struct
    {
        DpUint8 online;
        DpUint8 work_mode;
        DpFloat angle_deg[6];
    } custom_controller;
} DataPool;

static inline uint32_t dp_float_to_bits(float value)
{
    uint32_t raw = 0u;

    /* 共享池中的浮点量按 32 位原始比特存储。
     * 这样既能继续复用整数原子接口，也能让 IntelliSense 的类型检查成立。
     */
    memcpy(&raw, &value, sizeof(raw));
    return raw;
}

static inline float dp_float_from_bits(uint32_t raw)
{
    float value = 0.0f;

    memcpy(&value, &raw, sizeof(value));
    return value;
}

#define DP_LOAD_FLOAT(ptr) dp_float_from_bits(OM_LOAD_RLX(&(ptr)->v))
#define DP_STORE_FLOAT(ptr, value) OM_STORE_RLX(&(ptr)->v, dp_float_to_bits((value)))

#define DP_LOAD_INT16(ptr) OM_LOAD_RLX(&(ptr)->v)
#define DP_STORE_INT16(ptr, value) OM_STORE_RLX(&(ptr)->v, (value))

#define DP_LOAD_UINT8(ptr) OM_LOAD_RLX(&(ptr)->v)
#define DP_STORE_UINT8(ptr, value) OM_STORE_RLX(&(ptr)->v, (value))

#define DP_LOAD_UINT16(ptr) OM_LOAD_RLX(&(ptr)->v)
#define DP_STORE_UINT16(ptr, value) OM_STORE_RLX(&(ptr)->v, (value))

extern DataPool g_data_pool;

#endif
