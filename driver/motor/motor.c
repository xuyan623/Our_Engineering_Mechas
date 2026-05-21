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
#define MOTOR_GO8010_ONLINE_TIMEOUT_MS    (100u)

static Motor* g_motor_registry[MOTOR_REGISTRY_CAPACITY] = {0};

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

    /* Disabled Damiao motors use zero-gain MIT idle frames to keep feedback refreshing.
     * GO8010 still needs a periodic command frame to drive its reply path, so
     * it keeps the observation-mode target synthesis as well.
     */
    switch (motor->config.vendor)
    {
    case MOTOR_VENDOR_DAMIAO:
        /* 只有被正式通信任务显式接入观察路径的达妙电机，
         * disabled 态才会合成零增益 MIT 空闲帧。
         * 预留但未安装的电机保持静默，避免无意义报码与总线占用。 */
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
        return motor_prepare_damiao_target(motor);
    case MOTOR_VENDOR_GO8010:
        return motor_prepare_go8010_target(motor);
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

static OmRet motor_sync_p1010b_target(Motor* motor)
{
    float target_value = 0.0f;
    P1010BDriver* driver = OM_NULL;

    if (motor == OM_NULL || motor->binding.p1010b.driver == OM_NULL)
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

    return p1010b_set_target(driver, target_value);
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
    DamiaoMotorBus* damiao_buses[MOTOR_REGISTRY_CAPACITY] = {0};
    Go8010Bus* go8010_buses[MOTOR_REGISTRY_CAPACITY] = {0};
    uint32_t dji_bus_count = 0u;
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
            if (motor_sync_p1010b_target(motor) != OM_OK)
            {
                last_error = OM_ERROR;
            }
            break;

        default:
            last_error = OM_ERROR_NOT_SUPPORT;
            break;
        }
    }

    for (index = 0u; index < dji_bus_count; index++)
    {
        dji_motor_bus_sync(dji_buses[index]);
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
