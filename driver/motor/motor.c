#include "driver/motor/motor.h"

#include "module/motor_recovery/motor_recovery.h"
#include "osal/osal_time.h"
#include <string.h>

#define MOTOR_PI                          (3.1415926f)
#define MOTOR_TWO_PI                      (2.0f * MOTOR_PI)
#define MOTOR_DEG_TO_RAD                  (MOTOR_PI / 180.0f)
#define MOTOR_RPM_TO_RAD_PER_SECOND       (MOTOR_TWO_PI / 60.0f)
#define MOTOR_P1010B_ENCODER_TO_RAD       (MOTOR_TWO_PI / 32768.0f)
#define MOTOR_DJI_ONLINE_TIMEOUT_MS       (100u)
#define MOTOR_P1010B_ONLINE_TIMEOUT_MS    (100u)
#define MOTOR_DAMIAO_ONLINE_TIMEOUT_MS    (100u)
#define MOTOR_GO8010_ONLINE_TIMEOUT_MS    (300u)
#define MOTOR_OWNER_DAMIAO_CTRL_MODE_RID  (10u)
#define MOTOR_OWNER_DAMIAO_CTRL_MODE_MIT  (1u)

static Motor* g_motor_registry[MOTOR_REGISTRY_CAPACITY] = {0};

static void motor_write_p1010b_query_feedback(
    P1010BDriver* driver,
    const P1010BResponse* response)
{
    if (driver == OM_NULL || response == OM_NULL)
    {
        return;
    }

    driver->telemetry.feedback.absolutePosition =
        (uint16_t)response->data.activeQueryValues[0];
    driver->telemetry.feedback.speedRpm =
        ((float)response->data.activeQueryValues[1]) / 10.0f;
    driver->telemetry.feedback.iqAmpere =
        ((float)response->data.activeQueryValues[2]) / 100.0f;
    driver->telemetry.feedback.busVoltage =
        ((float)response->data.activeQueryValues[3]) / 10.0f;
    driver->telemetry.feedback.timestampMs = response->timestampMs;
    driver->telemetry.lastSuccessRxTimestampMs = response->timestampMs;
    driver->telemetry.online = true;
}

static OmBool motor_is_registered_instance(const Motor* motor)
{
    uint32_t index = 0u;

    if (motor == OM_NULL)
    {
        return OM_FALSE;
    }

    for (index = 0u; index < MOTOR_REGISTRY_CAPACITY; index++)
    {
        if (g_motor_registry[index] == motor)
        {
            return OM_TRUE;
        }
    }

    return OM_FALSE;
}

static OmRet motor_register_instance(Motor* motor)
{
    uint32_t index = 0u;

    if (motor_is_registered_instance(motor) == OM_TRUE)
    {
        return OM_ERR_CONFLICT;
    }

    for (index = 0u; index < MOTOR_REGISTRY_CAPACITY; index++)
    {
        if (g_motor_registry[index] == OM_NULL)
        {
            g_motor_registry[index] = motor;
            return OM_OK;
        }
    }

    return OM_ERROR_MEMORY;
}

static OmBool motor_has_valid_binding(const Motor* motor)
{
    if (motor == OM_NULL || motor->registered_flag != OM_TRUE)
    {
        return OM_FALSE;
    }

    switch (motor->config.vendor)
    {
    case MOTOR_VENDOR_DJI:
        return (motor->binding.dji.bus != OM_NULL && motor->binding.dji.driver != OM_NULL) ? OM_TRUE : OM_FALSE;
    case MOTOR_VENDOR_DAMIAO:
        return (motor->binding.damiao.bus != OM_NULL && motor->binding.damiao.driver != OM_NULL) ? OM_TRUE : OM_FALSE;
    case MOTOR_VENDOR_P1010B:
        return (motor->binding.p1010b.bus != OM_NULL && motor->binding.p1010b.driver != OM_NULL) ? OM_TRUE : OM_FALSE;
    case MOTOR_VENDOR_GO8010:
        return (motor->binding.go8010.bus != OM_NULL && motor->binding.go8010.driver != OM_NULL) ? OM_TRUE : OM_FALSE;
    default:
        return OM_FALSE;
    }
}

static float motor_clamp_primary_output(const Motor* motor, float output)
{
    if (motor == OM_NULL || motor->config.output_limit_enabled != OM_TRUE)
    {
        return output;
    }

    if (output < motor->config.output_min)
    {
        return motor->config.output_min;
    }

    if (output > motor->config.output_max)
    {
        return motor->config.output_max;
    }

    return output;
}

static int16_t motor_scale_p1010b_target_raw(float scale, float target_value)
{
    float scaled = target_value * scale;

    if (scaled >= 0.0f)
    {
        scaled += 0.5f;
    }
    else
    {
        scaled -= 0.5f;
    }

    if (scaled > 32767.0f)
    {
        scaled = 32767.0f;
    }
    else if (scaled < -32768.0f)
    {
        scaled = -32768.0f;
    }

    return (int16_t)scaled;
}

static void motor_reset_command(Motor* motor)
{
    if (motor == OM_NULL)
    {
        return;
    }

    memset(&motor->command, 0, sizeof(motor->command));
    motor->computed_output = 0.0f;
}

static OmRet motor_apply_default_command(Motor* motor)
{
    float primary_output = 0.0f;

    if (motor == OM_NULL)
    {
        return OM_ERROR_NULL;
    }

    motor_reset_command(motor);

    switch (motor->config.control_mode)
    {
    case MOTOR_CONTROL_MODE_DISABLED:
        primary_output = 0.0f;
        break;

    case MOTOR_CONTROL_MODE_CURRENT:
        primary_output = motor->target_current + motor->torque_feedforward;
        primary_output = motor_clamp_primary_output(motor, primary_output);
        motor->command.current = primary_output;
        break;

    case MOTOR_CONTROL_MODE_SPEED:
        primary_output = motor_clamp_primary_output(motor, motor->target_speed);
        motor->command.speed = primary_output;
        motor->command.kd = motor->target_kd;
        motor->command.torque = motor->torque_feedforward;
        break;

    case MOTOR_CONTROL_MODE_ANGLE:
        primary_output = motor_clamp_primary_output(motor, motor->target_angle);
        motor->command.angle = primary_output;
        motor->command.speed = motor->target_speed;
        motor->command.kp = motor->target_kp;
        motor->command.kd = motor->target_kd;
        motor->command.torque = motor->torque_feedforward;
        break;

    case MOTOR_CONTROL_MODE_TORQUE:
        primary_output = motor->target_torque + motor->torque_feedforward;
        primary_output = motor_clamp_primary_output(motor, primary_output);
        motor->command.torque = primary_output;
        break;

    default:
        return OM_ERROR_NOT_SUPPORT;
    }

    motor->computed_output = primary_output;
    return OM_OK;
}

static OmRet motor_prepare_dji_target(Motor* motor)
{
    int16_t output = 0;

    if (motor == OM_NULL || motor->binding.dji.driver == OM_NULL)
    {
        return OM_ERROR_PARAM;
    }

    switch (motor->config.control_mode)
    {
    case MOTOR_CONTROL_MODE_DISABLED:
        output = 0;
        break;

    case MOTOR_CONTROL_MODE_CURRENT:
        output = (int16_t)motor->command.current;
        break;

    case MOTOR_CONTROL_MODE_TORQUE:
        output = (int16_t)motor->command.torque;
        break;

    case MOTOR_CONTROL_MODE_SPEED:
    case MOTOR_CONTROL_MODE_ANGLE:
    default:
        return OM_ERROR_NOT_SUPPORT;
    }

    dji_motor_set_output(motor->binding.dji.driver, output);
    return OM_OK;
}

static OmRet motor_prepare_damiao_target(Motor* motor)
{
    float position = 0.0f;
    float speed = 0.0f;
    float kp = 0.0f;
    float kd = 0.0f;
    float torque = 0.0f;

    if (motor == OM_NULL || motor->binding.damiao.driver == OM_NULL)
    {
        return OM_ERROR_PARAM;
    }

    if (motor_recovery_should_block_damiao_regular_target(motor) == OM_TRUE)
    {
        return OM_OK;
    }

    position = motor->feedback.angle;

    switch (motor->config.control_mode)
    {
    case MOTOR_CONTROL_MODE_DISABLED:
        break;

    case MOTOR_CONTROL_MODE_ANGLE:
        position = motor->command.angle;
        speed = motor->command.speed;
        kp = motor->command.kp;
        kd = motor->command.kd;
        torque = motor->command.torque;
        break;

    case MOTOR_CONTROL_MODE_SPEED:
        speed = motor->command.speed;
        kd = motor->command.kd;
        torque = motor->command.torque;
        break;

    case MOTOR_CONTROL_MODE_TORQUE:
        torque = motor->command.torque;
        break;

    case MOTOR_CONTROL_MODE_CURRENT:
    default:
        return OM_ERROR_NOT_SUPPORT;
    }

    damiao_motor_set_mit(motor->binding.damiao.driver, position, speed, kp, kd, torque);
    return OM_OK;
}

static OmRet motor_prepare_p1010b_target(Motor* motor)
{
    if (motor == OM_NULL || motor->binding.p1010b.driver == OM_NULL)
    {
        return OM_ERROR_PARAM;
    }

    switch (motor->config.control_mode)
    {
    case MOTOR_CONTROL_MODE_DISABLED:
    case MOTOR_CONTROL_MODE_CURRENT:
    case MOTOR_CONTROL_MODE_SPEED:
    case MOTOR_CONTROL_MODE_ANGLE:
        return OM_OK;

    case MOTOR_CONTROL_MODE_TORQUE:
    default:
        return OM_ERROR_NOT_SUPPORT;
    }
}

static OmRet motor_prepare_go8010_target(Motor* motor)
{
    float position = 0.0f;
    float speed = 0.0f;
    float kp = 0.0f;
    float kd = 0.0f;
    float torque = 0.0f;

    if (motor == OM_NULL || motor->binding.go8010.driver == OM_NULL)
    {
        return OM_ERROR_PARAM;
    }

    position = motor->feedback.angle;

    switch (motor->config.control_mode)
    {
    case MOTOR_CONTROL_MODE_DISABLED:
        break;

    case MOTOR_CONTROL_MODE_ANGLE:
        position = motor->command.angle;
        speed = motor->command.speed;
        kp = motor->command.kp;
        kd = motor->command.kd;
        torque = motor->command.torque;
        break;

    case MOTOR_CONTROL_MODE_SPEED:
        speed = motor->command.speed;
        kd = motor->command.kd;
        torque = motor->command.torque;
        break;

    case MOTOR_CONTROL_MODE_TORQUE:
        torque = motor->command.torque;
        break;

    case MOTOR_CONTROL_MODE_CURRENT:
    default:
        return OM_ERROR_NOT_SUPPORT;
    }

    go8010_set_target(motor->binding.go8010.driver, torque, position, speed, kp, kd);
    return OM_OK;
}

static OmRet motor_prepare_vendor_target(Motor* motor)
{
    if (motor == OM_NULL)
    {
        return OM_ERROR_NULL;
    }

    switch (motor->config.vendor)
    {
    case MOTOR_VENDOR_DJI:
        return motor_prepare_dji_target(motor);
    case MOTOR_VENDOR_DAMIAO:
        return motor_prepare_damiao_target(motor);
    case MOTOR_VENDOR_P1010B:
        return motor_prepare_p1010b_target(motor);
    case MOTOR_VENDOR_GO8010:
        return motor_prepare_go8010_target(motor);
    default:
        return OM_ERROR_NOT_SUPPORT;
    }
}

static OmRet motor_prepare_disabled_observation_target(Motor* motor)
{
    if (motor == OM_NULL)
    {
        return OM_ERROR_NULL;
    }

    if (motor->config.control_mode != MOTOR_CONTROL_MODE_DISABLED)
    {
        return OM_OK;
    }

    /* formal transmit 不再为 disabled 达妙合成 observation 空闲帧。
     * 达妙/GO8010 的反馈刷新统一走独立的 observation-only 路径，
     * 这里仅保留 GO8010 的最小接收驱动帧需求。
     */
    switch (motor->config.vendor)
    {
    case MOTOR_VENDOR_DAMIAO:
        return OM_OK;
    case MOTOR_VENDOR_GO8010:
        return motor_prepare_go8010_target(motor);
    default:
        return OM_OK;
    }
}

static OmRet motor_prepare_observation_only_target(Motor* motor)
{
    if (motor == OM_NULL)
    {
        return OM_ERROR_NULL;
    }

    switch (motor->config.vendor)
    {
    case MOTOR_VENDOR_DAMIAO:
        if (motor->binding.damiao.driver == OM_NULL)
        {
            return OM_ERROR_PARAM;
        }

        if (motor->config.vendor_context == OM_NULL)
        {
            return OM_OK;
        }

        {
            const OsalTimeMs* gate_until_ms = (const OsalTimeMs*)motor->config.vendor_context;
            if (osal_time_before(osal_time_now_monotonic(), *gate_until_ms))
            {
                return OM_OK;
            }
        }

        damiao_motor_set_mit(
            motor->binding.damiao.driver,
            motor->feedback.angle,
            0.0f,
            0.0f,
            0.0f,
            0.0f);
        return OM_OK;

    case MOTOR_VENDOR_GO8010:
        if (motor->binding.go8010.driver == OM_NULL)
        {
            return OM_ERROR_PARAM;
        }

        go8010_set_target(
            motor->binding.go8010.driver,
            0.0f,
            motor->feedback.angle,
            0.0f,
            0.0f,
            0.0f);
        return OM_OK;

    default:
        return OM_OK;
    }
}

static OmRet motor_collect_unique_pointer(void** pointers, uint32_t* count, void* target)
{
    uint32_t index = 0u;

    if (pointers == OM_NULL || count == OM_NULL || target == OM_NULL)
    {
        return OM_ERROR_PARAM;
    }

    for (index = 0u; index < *count; index++)
    {
        if (pointers[index] == target)
        {
            return OM_OK;
        }
    }

    if (*count >= MOTOR_REGISTRY_CAPACITY)
    {
        return OM_ERROR_MEMORY;
    }

    pointers[*count] = target;
    (*count)++;
    return OM_OK;
}

static int32_t motor_find_pointer_index(
    void** pointers,
    uint32_t count,
    void* target)
{
    uint32_t index = 0u;

    if (pointers == OM_NULL || target == OM_NULL)
    {
        return -1;
    }

    for (index = 0u; index < count; index++)
    {
        if (pointers[index] == target)
        {
            return (int32_t)index;
        }
    }

    return -1;
}

static void motor_pack_p1010b_group_payload(
    const P1010BBus* bus,
    uint8_t group_index,
    uint8_t payload[P1010B_CAN_DLC])
{
    uint8_t slot = 0u;

    if (bus == OM_NULL || payload == OM_NULL)
    {
        return;
    }

    for (slot = 0u; slot < 4u; slot++)
    {
        const int16_t raw_value = bus->groupTargetsRaw[group_index][slot];
        payload[(slot * 2u)] = (uint8_t)(((uint16_t)raw_value >> 8u) & 0xFFu);
        payload[(slot * 2u) + 1u] = (uint8_t)((uint16_t)raw_value & 0xFFu);
    }
}

static OmRet motor_flush_p1010b_group(
    P1010BBus* bus,
    uint8_t group_index)
{
    CanUserMsg message = {0};
    uint8_t payload[P1010B_CAN_DLC] = {0};

    if (bus == OM_NULL || bus->canDevice == OM_NULL || group_index >= 2u)
    {
        return OM_ERROR_PARAM;
    }

    motor_pack_p1010b_group_payload(bus, group_index, payload);
    message.dsc = CAN_DATA_MSG_DSC_INIT(
        (group_index == 0u) ? P1010B_CAN_CMD_DRIVE_GROUP_1_4 :
                               P1010B_CAN_CMD_DRIVE_GROUP_5_8,
        CAN_IDE_STD,
        P1010B_CAN_DLC);
    message.filterHandle = 0u;
    message.userBuf = payload;

    return (device_write(bus->canDevice, 0, &message, 1u) > 0) ? OM_OK : OM_ERROR;
}

static OmRet motor_sync_p1010b_target(
    Motor* motor,
    uint8_t* dirty_group_mask)
{
    float target_value = 0.0f;
    int16_t raw_target = 0;
    P1010BDriver* driver = OM_NULL;
    uint8_t group_index = 0u;
    uint8_t slot_index = 0u;

    if (motor == OM_NULL || motor->binding.p1010b.driver == OM_NULL ||
        dirty_group_mask == OM_NULL)
    {
        return OM_ERROR_PARAM;
    }

    driver = motor->binding.p1010b.driver;

    switch (motor->config.control_mode)
    {
    case MOTOR_CONTROL_MODE_DISABLED:
        target_value = 0.0f;
        break;

    case MOTOR_CONTROL_MODE_CURRENT:
        if (driver->runtime.currentMode != P1010B_MODE_CURRENT)
        {
            return OM_ERR_CONFLICT;
        }
        target_value = motor->command.current;
        break;

    case MOTOR_CONTROL_MODE_SPEED:
        if (driver->runtime.currentMode != P1010B_MODE_SPEED)
        {
            return OM_ERR_CONFLICT;
        }
        target_value = motor->command.speed / MOTOR_RPM_TO_RAD_PER_SECOND;
        break;

    case MOTOR_CONTROL_MODE_ANGLE:
        if (driver->runtime.currentMode != P1010B_MODE_POSITION)
        {
            return OM_ERR_CONFLICT;
        }
        target_value = motor->command.angle / MOTOR_TWO_PI;
        break;

    case MOTOR_CONTROL_MODE_TORQUE:
    default:
        return OM_ERROR_NOT_SUPPORT;
    }

    raw_target = motor_scale_p1010b_target_raw(
        driver->runtime.targetScale,
        target_value);
    if (motor->p1010b_last_synced_valid == OM_TRUE &&
        motor->p1010b_last_synced_mode == (uint8_t)driver->runtime.currentMode &&
        motor->p1010b_last_synced_raw_target == raw_target)
    {
        return OM_OK;
    }

    group_index = (driver->config.motorId <= 4u) ? 0u : 1u;
    slot_index = (uint8_t)((driver->config.motorId - 1u) % 4u);
    driver->bus->groupTargetsRaw[group_index][slot_index] = raw_target;
    *dirty_group_mask |= (uint8_t)(1u << group_index);
    driver->runtime.lastRejectReason = P1010B_REJECT_NONE;
    motor->p1010b_last_synced_raw_target = raw_target;
    motor->p1010b_last_synced_mode = (uint8_t)driver->runtime.currentMode;
    motor->p1010b_last_synced_valid = OM_TRUE;
    return OM_OK;
}

static OmRet motor_write_binding_after_register(Motor* motor, MotorVendor vendor, const void* bus, const void* driver)
{
    if (motor == OM_NULL || bus == OM_NULL || driver == OM_NULL)
    {
        return OM_ERROR_PARAM;
    }

    switch (vendor)
    {
    case MOTOR_VENDOR_DJI:
        motor->binding.dji.bus = (DJIMotorBus*)bus;
        motor->binding.dji.driver = (DJIMotorDrv*)driver;
        return OM_OK;
    case MOTOR_VENDOR_DAMIAO:
        motor->binding.damiao.bus = (DamiaoMotorBus*)bus;
        motor->binding.damiao.driver = (DamiaoMotorDrv*)driver;
        return OM_OK;
    case MOTOR_VENDOR_P1010B:
        motor->binding.p1010b.bus = (P1010BBus*)bus;
        motor->binding.p1010b.driver = (P1010BDriver*)driver;
        return OM_OK;
    case MOTOR_VENDOR_GO8010:
        motor->binding.go8010.bus = (Go8010Bus*)bus;
        motor->binding.go8010.driver = (Go8010MotorDrv*)driver;
        return OM_OK;
    default:
        return OM_ERROR_NOT_SUPPORT;
    }
}

OmRet motor_register(Motor* motor, const MotorConfig* config)
{
    OmRet ret = OM_OK;

    if (motor == OM_NULL || config == OM_NULL)
    {
        return OM_ERROR_NULL;
    }

    memset(motor, 0, sizeof(*motor));
    motor->config = *config;

    ret = motor_register_instance(motor);
    if (ret != OM_OK)
    {
        return ret;
    }

    motor->registered_flag = OM_TRUE;
    return OM_OK;
}

OmRet motor_register_dji(Motor* motor, const char* name, DJIMotorBus* bus, DJIMotorDrv* driver, MotorControlMode control_mode)
{
    MotorConfig config = {
        .name = name,
        .vendor = MOTOR_VENDOR_DJI,
        .control_mode = control_mode,
        .output_limit_enabled = OM_FALSE,
    };
    OmRet ret = motor_register(motor, &config);

    if (ret != OM_OK)
    {
        return ret;
    }

    return motor_write_binding_after_register(motor, MOTOR_VENDOR_DJI, bus, driver);
}

OmRet motor_register_damiao(Motor* motor, const char* name, DamiaoMotorBus* bus, DamiaoMotorDrv* driver,
                            MotorControlMode control_mode)
{
    MotorConfig config = {
        .name = name,
        .vendor = MOTOR_VENDOR_DAMIAO,
        .control_mode = control_mode,
        .output_limit_enabled = OM_FALSE,
    };
    OmRet ret = motor_register(motor, &config);

    if (ret != OM_OK)
    {
        return ret;
    }

    return motor_write_binding_after_register(motor, MOTOR_VENDOR_DAMIAO, bus, driver);
}

OmRet motor_register_p1010b(Motor* motor, const char* name, P1010BBus* bus, P1010BDriver* driver,
                            MotorControlMode control_mode)
{
    MotorConfig config = {
        .name = name,
        .vendor = MOTOR_VENDOR_P1010B,
        .control_mode = control_mode,
        .output_limit_enabled = OM_FALSE,
    };
    OmRet ret = motor_register(motor, &config);

    if (ret != OM_OK)
    {
        return ret;
    }

    return motor_write_binding_after_register(motor, MOTOR_VENDOR_P1010B, bus, driver);
}

OmRet motor_register_go8010(Motor* motor, const char* name, Go8010Bus* bus, Go8010MotorDrv* driver,
                            MotorControlMode control_mode)
{
    MotorConfig config = {
        .name = name,
        .vendor = MOTOR_VENDOR_GO8010,
        .control_mode = control_mode,
        .output_limit_enabled = OM_FALSE,
    };
    OmRet ret = motor_register(motor, &config);

    if (ret != OM_OK)
    {
        return ret;
    }

    return motor_write_binding_after_register(motor, MOTOR_VENDOR_GO8010, bus, driver);
}

OmRet motor_attach_dji(Motor* motor, const char* name, DJIMotorBus* bus, DJIMotorDrv* driver, DJIMotorType type,
                       uint8_t id, DJIMotorCtrlMode dji_control_mode, MotorControlMode control_mode)
{
    OmRet ret = dji_motor_register(bus, driver, type, id, dji_control_mode);

    if (ret != OM_OK)
    {
        return ret;
    }

    return motor_register_dji(motor, name, bus, driver, control_mode);
}

OmRet motor_attach_damiao(Motor* motor, const char* name, DamiaoMotorBus* bus, DamiaoMotorDrv* driver,
                          DamiaoMotorType type, uint16_t can_id, uint16_t master_id, MotorControlMode control_mode)
{
    OmRet ret = damiao_motor_register(bus, driver, type, can_id, master_id);

    if (ret != OM_OK)
    {
        return ret;
    }

    return motor_register_damiao(motor, name, bus, driver, control_mode);
}

OmRet motor_attach_p1010b(Motor* motor, const char* name, P1010BBus* bus, P1010BDriver* driver, uint8_t id,
                          P1010BMode default_mode, MotorControlMode control_mode)
{
    P1010BConfig config = P1010B_DEFAULT_CONFIG(id);
    OmRet ret = OM_OK;

    config.defaultMode = default_mode;
    ret = p1010b_register(bus, driver, &config);
    if (ret != OM_OK)
    {
        return ret;
    }

    return motor_register_p1010b(motor, name, bus, driver, control_mode);
}

OmRet motor_attach_go8010(Motor* motor, const char* name, Go8010Bus* bus, Go8010MotorDrv* driver, uint8_t id,
                          MotorControlMode control_mode)
{
    OmRet ret = go8010_register(bus, driver, id);

    if (ret != OM_OK)
    {
        return ret;
    }

    return motor_register_go8010(motor, name, bus, driver, control_mode);
}

OmRet motor_set_control_mode(Motor* motor, MotorControlMode control_mode)
{
    if (motor == OM_NULL || motor->registered_flag != OM_TRUE)
    {
        return OM_ERROR_PARAM;
    }

    motor->config.control_mode = control_mode;
    motor->p1010b_last_synced_valid = OM_FALSE;
    return OM_OK;
}

OmRet motor_set_current(Motor* motor, float current)
{
    if (motor == OM_NULL || motor->registered_flag != OM_TRUE)
    {
        return OM_ERROR_PARAM;
    }

    motor->target_current = current;
    return OM_OK;
}

OmRet motor_set_speed(Motor* motor, float speed)
{
    if (motor == OM_NULL || motor->registered_flag != OM_TRUE)
    {
        return OM_ERROR_PARAM;
    }

    motor->target_speed = speed;
    return OM_OK;
}

OmRet motor_set_angle(Motor* motor, float angle)
{
    if (motor == OM_NULL || motor->registered_flag != OM_TRUE)
    {
        return OM_ERROR_PARAM;
    }

    motor->target_angle = angle;
    return OM_OK;
}

OmRet motor_set_torque(Motor* motor, float torque)
{
    if (motor == OM_NULL || motor->registered_flag != OM_TRUE)
    {
        return OM_ERROR_PARAM;
    }

    motor->target_torque = torque;
    return OM_OK;
}

OmRet motor_set_position_gains(Motor* motor, float kp, float kd)
{
    if (motor == OM_NULL || motor->registered_flag != OM_TRUE)
    {
        return OM_ERROR_PARAM;
    }

    motor->target_kp = kp;
    motor->target_kd = kd;
    return OM_OK;
}

OmRet motor_set_torque_feedforward(Motor* motor, float torque_feedforward)
{
    if (motor == OM_NULL || motor->registered_flag != OM_TRUE)
    {
        return OM_ERROR_PARAM;
    }

    motor->torque_feedforward = torque_feedforward;
    return OM_OK;
}

OmRet motor_set_output_limit(Motor* motor, float output_min, float output_max)
{
    if (motor == OM_NULL || motor->registered_flag != OM_TRUE || output_min > output_max)
    {
        return OM_ERROR_PARAM;
    }

    motor->config.output_limit_enabled = OM_TRUE;
    motor->config.output_min = output_min;
    motor->config.output_max = output_max;
    return OM_OK;
}

OmRet motor_set_feedback(Motor* motor, float angle, float speed, float current, float torque)
{
    if (motor == OM_NULL || motor->registered_flag != OM_TRUE)
    {
        return OM_ERROR_PARAM;
    }

    motor->feedback.angle = angle;
    motor->feedback.speed = speed;
    motor->feedback.current = current;
    motor->feedback.torque = torque;
    return OM_OK;
}

OmRet motor_refresh_feedback(Motor* motor)
{
    const Go8010Feedback* go8010_feedback = OM_NULL;

    if (motor == OM_NULL || motor->registered_flag != OM_TRUE)
    {
        return OM_ERROR_PARAM;
    }

    if (motor_has_valid_binding(motor) != OM_TRUE)
    {
        return OM_ERROR_PARAM;
    }

    switch (motor->config.vendor)
    {
    case MOTOR_VENDOR_DJI:
        motor->feedback.angle = dji_motor_get_total_angle(motor->binding.dji.driver) * MOTOR_DEG_TO_RAD;
        motor->feedback.speed = dji_motor_get_velocity(motor->binding.dji.driver) * MOTOR_RPM_TO_RAD_PER_SECOND;
        motor->feedback.current = dji_motor_get_current(motor->binding.dji.driver);
        motor->feedback.torque = 0.0f;
        motor->feedback.timestamp_ms = dji_motor_get_feedback_timestamp_ms(motor->binding.dji.driver);
        motor->feedback.online =
            (motor->feedback.timestamp_ms != 0u &&
             (uint32_t)(osal_time_now_monotonic() - motor->feedback.timestamp_ms) <= MOTOR_DJI_ONLINE_TIMEOUT_MS) ?
                OM_TRUE :
                OM_FALSE;
        return OM_OK;

    case MOTOR_VENDOR_DAMIAO:
        motor->feedback.angle = damiao_motor_get_position(motor->binding.damiao.driver);
        motor->feedback.speed = damiao_motor_get_velocity(motor->binding.damiao.driver);
        motor->feedback.current = 0.0f;
        motor->feedback.torque = damiao_motor_get_torque(motor->binding.damiao.driver);
        motor->feedback.timestamp_ms = damiao_motor_get_feedback_timestamp_ms(motor->binding.damiao.driver);
        motor->feedback.online =
            (damiao_motor_get_feedback_sequence(motor->binding.damiao.driver) != 0u &&
             (uint32_t)(osal_time_now_monotonic() - motor->feedback.timestamp_ms) <= MOTOR_DAMIAO_ONLINE_TIMEOUT_MS) ?
                OM_TRUE :
                OM_FALSE;
        return OM_OK;

    case MOTOR_VENDOR_P1010B:
        motor->feedback.angle = (float)motor->binding.p1010b.driver->telemetry.feedback.absolutePosition * MOTOR_P1010B_ENCODER_TO_RAD;
        motor->feedback.speed = motor->binding.p1010b.driver->telemetry.feedback.speedRpm * MOTOR_RPM_TO_RAD_PER_SECOND;
        motor->feedback.current = motor->binding.p1010b.driver->telemetry.feedback.iqAmpere;
        motor->feedback.torque = 0.0f;
        motor->feedback.timestamp_ms = motor->binding.p1010b.driver->telemetry.lastSuccessRxTimestampMs;
        motor->feedback.online =
            (motor->feedback.timestamp_ms != 0u &&
             (uint32_t)(osal_time_now_monotonic() - motor->feedback.timestamp_ms) <= MOTOR_P1010B_ONLINE_TIMEOUT_MS) ?
                OM_TRUE :
                OM_FALSE;
        return OM_OK;

    case MOTOR_VENDOR_GO8010:
        go8010_feedback = go8010_get_feedback(motor->binding.go8010.driver);
        motor->feedback.angle = go8010_feedback->position;
        motor->feedback.speed = go8010_feedback->speed;
        motor->feedback.current = 0.0f;
        motor->feedback.torque = go8010_feedback->torque;
        motor->feedback.online = go8010_is_online(motor->binding.go8010.driver, MOTOR_GO8010_ONLINE_TIMEOUT_MS);
        motor->feedback.timestamp_ms = go8010_feedback->timestampMs;
        return OM_OK;

    default:
        return OM_ERROR_NOT_SUPPORT;
    }
}

const MotorFeedback* motor_get_feedback(const Motor* motor)
{
    if (motor == OM_NULL)
    {
        return OM_NULL;
    }

    return &motor->feedback;
}

uint32_t motor_get_feedback_timestamp_ms(const Motor* motor)
{
    const MotorFeedback* feedback = motor_get_feedback(motor);

    return (feedback != OM_NULL) ? feedback->timestamp_ms : 0u;
}

OmBool motor_is_feedback_recent(const Motor* motor, uint32_t timeout_ms)
{
    const MotorFeedback* feedback = motor_get_feedback(motor);

    if (feedback == OM_NULL || feedback->online != OM_TRUE || feedback->timestamp_ms == 0u)
    {
        return OM_FALSE;
    }

    return ((uint32_t)(osal_time_now_monotonic() - feedback->timestamp_ms) <= timeout_ms) ? OM_TRUE : OM_FALSE;
}

OmBool motor_get_single_turn_angle_rad(const Motor* motor, float* angle_rad)
{
    const MotorFeedback* feedback = OM_NULL;

    if (motor == OM_NULL || angle_rad == OM_NULL || motor->registered_flag != OM_TRUE)
    {
        return OM_FALSE;
    }

    if (motor->config.vendor == MOTOR_VENDOR_DJI && motor->binding.dji.driver != OM_NULL)
    {
        *angle_rad = dji_motor_get_singgle_angle(motor->binding.dji.driver) * MOTOR_DEG_TO_RAD;
        return OM_TRUE;
    }

    feedback = motor_get_feedback(motor);
    if (feedback == OM_NULL)
    {
        return OM_FALSE;
    }

    *angle_rad = feedback->angle;
    return OM_TRUE;
}

OmBool motor_get_initial_zero_angle_rad(const Motor* motor, float* zero_angle_rad)
{
    if (motor == OM_NULL || zero_angle_rad == OM_NULL || motor->registered_flag != OM_TRUE)
    {
        return OM_FALSE;
    }

    if (motor->config.vendor != MOTOR_VENDOR_GO8010 || motor->binding.go8010.driver == OM_NULL)
    {
        return OM_FALSE;
    }

    return go8010_get_initial_position_zero(motor->binding.go8010.driver, zero_angle_rad);
}

OmRet motor_capture_initial_zero(Motor* motor)
{
    if (motor == OM_NULL || motor->registered_flag != OM_TRUE)
    {
        return OM_ERROR_PARAM;
    }

    if (motor->config.vendor != MOTOR_VENDOR_GO8010 || motor->binding.go8010.driver == OM_NULL)
    {
        return OM_ERROR_NOT_SUPPORT;
    }

    go8010_capture_initial_position_zero(motor->binding.go8010.driver);
    return OM_OK;
}

OmRet motor_owner_prepare_working_state(Motor* motor)
{
    P1010BDriver* p1010b_driver = OM_NULL;
    P1010BResponse p1010b_response = {0};
    DamiaoMotorDrv* damiao_driver = OM_NULL;
    DamiaoMotorBus* damiao_bus = OM_NULL;

    if (motor == OM_NULL || motor->registered_flag != OM_TRUE)
    {
        return OM_ERROR_PARAM;
    }

    switch (motor->config.vendor)
    {
    case MOTOR_VENDOR_P1010B:
        p1010b_driver = motor->binding.p1010b.driver;
        if (p1010b_driver == OM_NULL)
        {
            return OM_ERROR_PARAM;
        }

        if (p1010b_set_mode(
                p1010b_driver,
                p1010b_driver->config.defaultMode,
                0u,
                &p1010b_response) != OM_OK)
        {
            return OM_ERROR;
        }

        return p1010b_set_active_report(
            p1010b_driver,
            &p1010b_driver->runtime.activeReport,
            0u,
            &p1010b_response);

    case MOTOR_VENDOR_DAMIAO:
        damiao_driver = motor->binding.damiao.driver;
        damiao_bus = motor->binding.damiao.bus;
        if (damiao_driver == OM_NULL || damiao_bus == OM_NULL || damiao_bus->canDev == OM_NULL)
        {
            return OM_ERROR_PARAM;
        }

        return damiao_motor_write_register_u32(
            damiao_bus->canDev,
            damiao_driver->link.txId,
            MOTOR_OWNER_DAMIAO_CTRL_MODE_RID,
            MOTOR_OWNER_DAMIAO_CTRL_MODE_MIT);

    default:
        return OM_ERROR_NOT_SUPPORT;
    }
}

OmRet motor_owner_enable(Motor* motor)
{
    P1010BDriver* p1010b_driver = OM_NULL;
    P1010BResponse p1010b_response = {0};
    DamiaoMotorDrv* damiao_driver = OM_NULL;

    if (motor == OM_NULL || motor->registered_flag != OM_TRUE)
    {
        return OM_ERROR_PARAM;
    }

    switch (motor->config.vendor)
    {
    case MOTOR_VENDOR_P1010B:
        p1010b_driver = motor->binding.p1010b.driver;
        if (p1010b_driver == OM_NULL)
        {
            return OM_ERROR_PARAM;
        }

        return p1010b_enable(p1010b_driver, 0u, &p1010b_response);

    case MOTOR_VENDOR_DAMIAO:
        damiao_driver = motor->binding.damiao.driver;
        if (damiao_driver == OM_NULL)
        {
            return OM_ERROR_PARAM;
        }

        damiao_motor_enable(damiao_driver);
        return OM_OK;

    default:
        return OM_ERROR_NOT_SUPPORT;
    }
}

OmRet motor_owner_disable(Motor* motor)
{
    P1010BDriver* p1010b_driver = OM_NULL;
    P1010BResponse p1010b_response = {0};
    DamiaoMotorDrv* damiao_driver = OM_NULL;

    if (motor == OM_NULL || motor->registered_flag != OM_TRUE)
    {
        return OM_ERROR_PARAM;
    }

    switch (motor->config.vendor)
    {
    case MOTOR_VENDOR_P1010B:
        p1010b_driver = motor->binding.p1010b.driver;
        if (p1010b_driver == OM_NULL)
        {
            return OM_ERROR_PARAM;
        }

        return p1010b_disable(p1010b_driver, 0u, &p1010b_response);

    case MOTOR_VENDOR_DAMIAO:
        damiao_driver = motor->binding.damiao.driver;
        if (damiao_driver == OM_NULL)
        {
            return OM_ERROR_PARAM;
        }

        damiao_motor_disable(damiao_driver);
        return OM_OK;

    default:
        return OM_ERROR_NOT_SUPPORT;
    }
}

OmRet motor_owner_query_feedback(Motor* motor)
{
    P1010BDriver* p1010b_driver = OM_NULL;
    P1010BResponse response = {0};
    OmRet ret = OM_OK;

    if (motor == OM_NULL || motor->registered_flag != OM_TRUE)
    {
        return OM_ERROR_PARAM;
    }

    if (motor->config.vendor != MOTOR_VENDOR_P1010B || motor->binding.p1010b.driver == OM_NULL)
    {
        return OM_ERROR_NOT_SUPPORT;
    }

    p1010b_driver = motor->binding.p1010b.driver;
    ret = p1010b_active_query_slots(
        p1010b_driver,
        P1010B_REPORT_DATA_ABSOLUTE_POSITION,
        P1010B_REPORT_DATA_SPEED_RPM,
        P1010B_REPORT_DATA_IQ_AMPERE,
        P1010B_REPORT_DATA_BUS_VOLTAGE,
        0u,
        &response);
    if (ret != OM_OK)
    {
        return ret;
    }

    motor_write_p1010b_query_feedback(p1010b_driver, &response);
    (void)motor_refresh_feedback(motor);
    return OM_OK;
}

OmRet motor_owner_sync_bus(Motor* motor)
{
    if (motor == OM_NULL || motor->registered_flag != OM_TRUE)
    {
        return OM_ERROR_PARAM;
    }

    switch (motor->config.vendor)
    {
    case MOTOR_VENDOR_DJI:
        if (motor->binding.dji.bus == OM_NULL)
        {
            return OM_ERROR_PARAM;
        }

        dji_motor_bus_sync(motor->binding.dji.bus);
        return OM_OK;

    case MOTOR_VENDOR_DAMIAO:
        if (motor->binding.damiao.bus == OM_NULL)
        {
            return OM_ERROR_PARAM;
        }

        damiao_motor_bus_sync(motor->binding.damiao.bus);
        return OM_OK;

    case MOTOR_VENDOR_GO8010:
        if (motor->binding.go8010.bus == OM_NULL)
        {
            return OM_ERROR_PARAM;
        }

        go8010_bus_sync(motor->binding.go8010.bus);
        return OM_OK;

    case MOTOR_VENDOR_P1010B:
        return OM_OK;

    default:
        return OM_ERROR_NOT_SUPPORT;
    }
}

Motor* motor_find_by_name(const char* name)
{
    uint32_t index = 0u;

    if (name == OM_NULL)
    {
        return OM_NULL;
    }

    for (index = 0u; index < MOTOR_REGISTRY_CAPACITY; index++)
    {
        Motor* motor = g_motor_registry[index];

        if (motor == OM_NULL || motor->registered_flag != OM_TRUE || motor->config.name == OM_NULL)
        {
            continue;
        }

        if (strcmp(motor->config.name, name) == 0)
        {
            return motor;
        }
    }

    return OM_NULL;
}

OmRet motor_copy_feedback_snapshots(MotorFeedbackSnapshot* snapshots, uint32_t capacity, uint32_t* snapshot_count)
{
    uint32_t index = 0u;
    uint32_t count = 0u;

    if (snapshots == OM_NULL || snapshot_count == OM_NULL)
    {
        return OM_ERROR_NULL;
    }

    *snapshot_count = 0u;
    if (capacity == 0u)
    {
        return OM_ERROR_PARAM;
    }

    for (index = 0u; index < MOTOR_REGISTRY_CAPACITY; index++)
    {
        Motor* motor = g_motor_registry[index];

        if (motor == OM_NULL || motor->registered_flag != OM_TRUE)
        {
            continue;
        }

        if (count >= capacity)
        {
            break;
        }

        snapshots[count].name = motor->config.name;
        snapshots[count].vendor = motor->config.vendor;
        snapshots[count].feedback = motor->feedback;
        count++;
    }

    *snapshot_count = count;
    return OM_OK;
}

OmRet motor_set_compute_hook(Motor* motor, MotorComputeHook compute_hook)
{
    if (motor == OM_NULL || motor->registered_flag != OM_TRUE)
    {
        return OM_ERROR_PARAM;
    }

    motor->config.compute_hook = compute_hook;
    return OM_OK;
}

OmRet motor_set_sync_hook(Motor* motor, MotorSyncHook sync_hook)
{
    if (motor == OM_NULL || motor->registered_flag != OM_TRUE)
    {
        return OM_ERROR_PARAM;
    }

    motor->config.sync_hook = sync_hook;
    return OM_OK;
}

OmRet motor_set_vendor_context(Motor* motor, void* vendor_context)
{
    if (motor == OM_NULL || motor->registered_flag != OM_TRUE)
    {
        return OM_ERROR_PARAM;
    }

    motor->config.vendor_context = vendor_context;
    return OM_OK;
}

OmRet motor_control_compute(Motor* motor)
{
    OmRet ret = OM_OK;

    if (motor == OM_NULL || motor->registered_flag != OM_TRUE)
    {
        return OM_ERROR_PARAM;
    }

    if (motor_has_valid_binding(motor) != OM_TRUE)
    {
        return OM_ERROR_PARAM;
    }

    ret = motor_refresh_feedback(motor);
    if (ret != OM_OK)
    {
        return ret;
    }

    if (motor->config.compute_hook != OM_NULL)
    {
        ret = motor->config.compute_hook(motor);
    }
    else
    {
        ret = motor_apply_default_command(motor);
    }

    if (ret != OM_OK)
    {
        return ret;
    }

    if (motor->config.sync_hook != OM_NULL)
    {
        return motor->config.sync_hook(motor);
    }

    return motor_prepare_vendor_target(motor);
}

float motor_get_output(const Motor* motor)
{
    if (motor == OM_NULL)
    {
        return 0.0f;
    }

    return motor->computed_output;
}

OmRet motor_transmit_all(void)
{
    DJIMotorBus* dji_buses[MOTOR_REGISTRY_CAPACITY] = {0};
    P1010BBus* p1010b_buses[MOTOR_REGISTRY_CAPACITY] = {0};
    DamiaoMotorBus* damiao_buses[MOTOR_REGISTRY_CAPACITY] = {0};
    Go8010Bus* go8010_buses[MOTOR_REGISTRY_CAPACITY] = {0};
    uint32_t dji_bus_count = 0u;
    uint32_t p1010b_bus_count = 0u;
    uint32_t damiao_bus_count = 0u;
    uint32_t go8010_bus_count = 0u;
    uint8_t p1010b_dirty_group_masks[MOTOR_REGISTRY_CAPACITY] = {0};
    uint32_t index = 0u;
    OmRet last_error = OM_OK;

    for (index = 0u; index < MOTOR_REGISTRY_CAPACITY; index++)
    {
        Motor* motor = g_motor_registry[index];

        if (motor == OM_NULL || motor->registered_flag != OM_TRUE)
        {
            continue;
        }

        if (motor_has_valid_binding(motor) != OM_TRUE)
        {
            last_error = OM_ERROR_PARAM;
            continue;
        }

        if (motor_prepare_disabled_observation_target(motor) != OM_OK)
        {
            last_error = OM_ERROR;
            continue;
        }

        switch (motor->config.vendor)
        {
        case MOTOR_VENDOR_DJI:
            if (motor_collect_unique_pointer((void**)dji_buses, &dji_bus_count, motor->binding.dji.bus) != OM_OK)
            {
                last_error = OM_ERROR;
            }
            break;

        case MOTOR_VENDOR_DAMIAO:
            if (motor_collect_unique_pointer((void**)damiao_buses, &damiao_bus_count, motor->binding.damiao.bus) != OM_OK)
            {
                last_error = OM_ERROR;
            }
            break;

        case MOTOR_VENDOR_GO8010:
            if (motor_collect_unique_pointer((void**)go8010_buses, &go8010_bus_count, motor->binding.go8010.bus) != OM_OK)
            {
                last_error = OM_ERROR;
            }
            break;

        case MOTOR_VENDOR_P1010B:
        {
            int32_t p1010b_bus_index =
                motor_find_pointer_index(
                    (void**)p1010b_buses,
                    p1010b_bus_count,
                    motor->binding.p1010b.bus);

            if (p1010b_bus_index < 0)
            {
                if (motor_collect_unique_pointer(
                        (void**)p1010b_buses,
                        &p1010b_bus_count,
                        motor->binding.p1010b.bus) != OM_OK)
                {
                    last_error = OM_ERROR;
                    break;
                }
                p1010b_bus_index = (int32_t)(p1010b_bus_count - 1u);
            }

            if (motor_sync_p1010b_target(
                    motor,
                    &p1010b_dirty_group_masks[(uint32_t)p1010b_bus_index]) != OM_OK)
            {
                last_error = OM_ERROR;
            }
            break;
        }

        default:
            last_error = OM_ERROR_NOT_SUPPORT;
            break;
        }
    }

    for (index = 0u; index < dji_bus_count; index++)
    {
        dji_motor_bus_sync(dji_buses[index]);
    }

    for (index = 0u; index < p1010b_bus_count; index++)
    {
        uint8_t group_index = 0u;

        for (group_index = 0u; group_index < 2u; group_index++)
        {
            if ((p1010b_dirty_group_masks[index] & (uint8_t)(1u << group_index)) == 0u)
            {
                continue;
            }

            if (motor_flush_p1010b_group(p1010b_buses[index], group_index) != OM_OK)
            {
                last_error = OM_ERROR;
            }
        }
    }

    for (index = 0u; index < damiao_bus_count; index++)
    {
        damiao_motor_bus_sync(damiao_buses[index]);
    }

    for (index = 0u; index < go8010_bus_count; index++)
    {
        go8010_tx_service(go8010_buses[index]);
    }

    return last_error;
}

OmRet motor_transmit_observation_only(void)
{
    DamiaoMotorBus* damiao_buses[MOTOR_REGISTRY_CAPACITY] = {0};
    Go8010Bus* go8010_buses[MOTOR_REGISTRY_CAPACITY] = {0};
    uint32_t damiao_bus_count = 0u;
    uint32_t go8010_bus_count = 0u;
    uint32_t index = 0u;
    OmRet last_error = OM_OK;

    for (index = 0u; index < MOTOR_REGISTRY_CAPACITY; index++)
    {
        Motor* motor = g_motor_registry[index];

        if (motor == OM_NULL || motor->registered_flag != OM_TRUE)
        {
            continue;
        }

        if (motor_has_valid_binding(motor) != OM_TRUE)
        {
            last_error = OM_ERROR_PARAM;
            continue;
        }

        if (motor_prepare_observation_only_target(motor) != OM_OK)
        {
            last_error = OM_ERROR;
            continue;
        }

        switch (motor->config.vendor)
        {
        case MOTOR_VENDOR_DAMIAO:
            if (motor_collect_unique_pointer((void**)damiao_buses, &damiao_bus_count, motor->binding.damiao.bus) != OM_OK)
            {
                last_error = OM_ERROR;
            }
            break;

        case MOTOR_VENDOR_GO8010:
            if (motor_collect_unique_pointer((void**)go8010_buses, &go8010_bus_count, motor->binding.go8010.bus) != OM_OK)
            {
                last_error = OM_ERROR;
            }
            break;

        default:
            break;
        }
    }

    for (index = 0u; index < damiao_bus_count; index++)
    {
        damiao_motor_bus_sync(damiao_buses[index]);
    }

    for (index = 0u; index < go8010_bus_count; index++)
    {
        go8010_tx_service(go8010_buses[index]);
    }

    return last_error;
}

OmRet motor_receive_all(void)
{
    Go8010Bus* go8010_buses[MOTOR_REGISTRY_CAPACITY] = {0};
    uint32_t go8010_bus_count = 0u;
    uint32_t index = 0u;
    OmRet last_error = OM_OK;

    for (index = 0u; index < MOTOR_REGISTRY_CAPACITY; index++)
    {
        Motor* motor = g_motor_registry[index];

        if (motor == OM_NULL || motor->registered_flag != OM_TRUE)
        {
            continue;
        }

        if (motor_has_valid_binding(motor) != OM_TRUE)
        {
            last_error = OM_ERROR_PARAM;
            continue;
        }

        if (motor->config.vendor == MOTOR_VENDOR_GO8010 &&
            motor_collect_unique_pointer((void**)go8010_buses, &go8010_bus_count, motor->binding.go8010.bus) != OM_OK)
        {
            last_error = OM_ERROR;
        }
    }

    for (index = 0u; index < go8010_bus_count; index++)
    {
        go8010_rx_service(go8010_buses[index]);
    }

    for (index = 0u; index < MOTOR_REGISTRY_CAPACITY; index++)
    {
        Motor* motor = g_motor_registry[index];

        if (motor == OM_NULL || motor->registered_flag != OM_TRUE)
        {
            continue;
        }

        if (motor_refresh_feedback(motor) != OM_OK)
        {
            last_error = OM_ERROR;
        }
    }

    return last_error;
}
