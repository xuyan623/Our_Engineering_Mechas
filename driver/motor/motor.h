#ifndef NEW_ROBOT_MOTOR_H
#define NEW_ROBOT_MOTOR_H

#include "core/om_def.h"
#include "driver/damiao/damiao.h"
#include "driver/dji/dji_motor.h"
#include "driver/go8010/go8010.h"
#include "driver/p1010b/P1010B.h"
#include <stdint.h>

#define MOTOR_REGISTRY_CAPACITY  (32u)

typedef struct Motor Motor;

typedef enum
{
    MOTOR_VENDOR_DJI = 0u,
    MOTOR_VENDOR_DAMIAO,
    MOTOR_VENDOR_P1010B,
    MOTOR_VENDOR_GO8010,
} MotorVendor;

typedef enum
{
    MOTOR_CONTROL_MODE_DISABLED = 0u,
    MOTOR_CONTROL_MODE_CURRENT,
    MOTOR_CONTROL_MODE_SPEED,
    MOTOR_CONTROL_MODE_ANGLE,
    MOTOR_CONTROL_MODE_TORQUE,
} MotorControlMode;

typedef OmRet (*MotorComputeHook)(Motor* motor);
typedef OmRet (*MotorSyncHook)(Motor* motor);

typedef struct
{
    float angle;
    float speed;
    float current;
    float torque;
    float kp;
    float kd;
} MotorCommand;

typedef struct
{
    float angle;
    float speed;
    float current;
    float torque;
    OmBool online;
    uint32_t timestamp_ms;
} MotorFeedback;

typedef struct
{
    const char* name;
    MotorVendor vendor;
    MotorFeedback feedback;
} MotorFeedbackSnapshot;

typedef struct
{
    DJIMotorBus* bus;
    DJIMotorDrv* driver;
} MotorDjiBinding;

typedef struct
{
    DamiaoMotorBus* bus;
    DamiaoMotorDrv* driver;
} MotorDamiaoBinding;

typedef struct
{
    P1010BBus* bus;
    P1010BDriver* driver;
} MotorP1010BBinding;

typedef struct
{
    Go8010Bus* bus;
    Go8010MotorDrv* driver;
} MotorGo8010Binding;

typedef union
{
    MotorDjiBinding dji;
    MotorDamiaoBinding damiao;
    MotorP1010BBinding p1010b;
    MotorGo8010Binding go8010;
} MotorVendorBinding;

typedef struct
{
    const char* name;
    MotorVendor vendor;
    MotorControlMode control_mode;
    void* vendor_context;
    MotorComputeHook compute_hook;
    MotorSyncHook sync_hook;
    OmBool output_limit_enabled;
    float output_min;
    float output_max;
} MotorConfig;

struct Motor
{
    MotorConfig config;
    MotorVendorBinding binding;
    OmBool registered_flag;
    float target_current;
    float target_speed;
    float target_angle;
    float target_torque;
    float target_kp;
    float target_kd;
    float torque_feedforward;
    float computed_output;
    MotorCommand command;
    MotorFeedback feedback;
};

OmRet motor_register(Motor* motor, const MotorConfig* config);
OmRet motor_register_dji(Motor* motor, const char* name, DJIMotorBus* bus, DJIMotorDrv* driver, MotorControlMode control_mode);
OmRet motor_register_damiao(Motor* motor, const char* name, DamiaoMotorBus* bus, DamiaoMotorDrv* driver,
                            MotorControlMode control_mode);
OmRet motor_register_p1010b(Motor* motor, const char* name, P1010BBus* bus, P1010BDriver* driver,
                            MotorControlMode control_mode);
OmRet motor_register_go8010(Motor* motor, const char* name, Go8010Bus* bus, Go8010MotorDrv* driver,
                            MotorControlMode control_mode);
OmRet motor_attach_dji(Motor* motor, const char* name, DJIMotorBus* bus, DJIMotorDrv* driver, DJIMotorType type,
                       uint8_t id, DJIMotorCtrlMode dji_control_mode, MotorControlMode control_mode);
OmRet motor_attach_damiao(Motor* motor, const char* name, DamiaoMotorBus* bus, DamiaoMotorDrv* driver,
                          DamiaoMotorType type, uint16_t can_id, uint16_t master_id, MotorControlMode control_mode);
OmRet motor_attach_p1010b(Motor* motor, const char* name, P1010BBus* bus, P1010BDriver* driver, uint8_t id,
                          P1010BMode default_mode, MotorControlMode control_mode);
OmRet motor_attach_go8010(Motor* motor, const char* name, Go8010Bus* bus, Go8010MotorDrv* driver, uint8_t id,
                          MotorControlMode control_mode);

OmRet motor_set_control_mode(Motor* motor, MotorControlMode control_mode);
OmRet motor_set_current(Motor* motor, float current);
OmRet motor_set_speed(Motor* motor, float speed);
OmRet motor_set_angle(Motor* motor, float angle);
OmRet motor_set_torque(Motor* motor, float torque);
OmRet motor_set_position_gains(Motor* motor, float kp, float kd);
OmRet motor_set_torque_feedforward(Motor* motor, float torque_feedforward);
OmRet motor_set_output_limit(Motor* motor, float output_min, float output_max);
OmRet motor_set_feedback(Motor* motor, float angle, float speed, float current, float torque);
OmRet motor_refresh_feedback(Motor* motor);
const MotorFeedback* motor_get_feedback(const Motor* motor);
Motor* motor_find_by_name(const char* name);
OmRet motor_copy_feedback_snapshots(MotorFeedbackSnapshot* snapshots, uint32_t capacity, uint32_t* snapshot_count);
OmRet motor_set_compute_hook(Motor* motor, MotorComputeHook compute_hook);
OmRet motor_set_sync_hook(Motor* motor, MotorSyncHook sync_hook);
OmRet motor_set_vendor_context(Motor* motor, void* vendor_context);
OmRet motor_control_compute(Motor* motor);
float motor_get_output(const Motor* motor);
OmRet motor_transmit_all(void);
OmRet motor_receive_all(void);

#endif
