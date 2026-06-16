#include "driver/go8010/go8010.h"

#include "drivers/peripheral/serial/pal_serial_dev.h"
#include "osal/osal_time.h"
#include <math.h>
#include <string.h>

#define GO8010_DEVICE_NAME_USART6      ("usart6")
#define GO8010_PI_TIMES_TWO_APPROX     (6.2831f)
#define GO8010_SPEED_OFFSET_RAD_S      (0.122718468f)
#define GO8010_POSITION_MIN_RAD        (-95.5f)
#define GO8010_POSITION_MAX_RAD        (95.5f)
#define GO8010_SPEED_MIN_RAD_S         (-45.0f)
#define GO8010_SPEED_MAX_RAD_S         (45.0f)
#define GO8010_TORQUE_MIN_NM           (-5.0f)
#define GO8010_TORQUE_MAX_NM           (5.0f)
#define GO8010_KP_MIN                  (0.0f)
#define GO8010_KP_MAX                  (500.0f)
#define GO8010_KD_MIN                  (0.0f)
#define GO8010_KD_MAX                  (5.0f)
#define GO8010_FEEDBACK_TORQUE_EPS_NM  (0.5f)
#define GO8010_FEEDBACK_SPEED_EPS_RAD_S (2.0f)
#define GO8010_FEEDBACK_POSITION_EPS_RAD (2.0f)
#define GO8010_FEEDBACK_MAX_JUMP_MARGIN_RAD (1.0f)

/* 直接复用旧工程 PROTOCOL/crc.c 中的 CRC-CCITT 查表。 */
static const uint16_t g_go8010_crc_ccitt_table[256] = {
    0x0000, 0x1189, 0x2312, 0x329B, 0x4624, 0x57AD, 0x6536, 0x74BF, 0x8C48, 0x9DC1, 0xAF5A, 0xBED3, 0xCA6C,
    0xDBE5, 0xE97E, 0xF8F7, 0x1081, 0x0108, 0x3393, 0x221A, 0x56A5, 0x472C, 0x75B7, 0x643E, 0x9CC9, 0x8D40,
    0xBFDB, 0xAE52, 0xDAED, 0xCB64, 0xF9FF, 0xE876, 0x2102, 0x308B, 0x0210, 0x1399, 0x6726, 0x76AF, 0x4434,
    0x55BD, 0xAD4A, 0xBCC3, 0x8E58, 0x9FD1, 0xEB6E, 0xFAE7, 0xC87C, 0xD9F5, 0x3183, 0x200A, 0x1291, 0x0318,
    0x77A7, 0x662E, 0x54B5, 0x453C, 0xBDCB, 0xAC42, 0x9ED9, 0x8F50, 0xFBEF, 0xEA66, 0xD8FD, 0xC974, 0x4204,
    0x538D, 0x6116, 0x709F, 0x0420, 0x15A9, 0x2732, 0x36BB, 0xCE4C, 0xDFC5, 0xED5E, 0xFCD7, 0x8868, 0x99E1,
    0xAB7A, 0xBAF3, 0x5285, 0x430C, 0x7197, 0x601E, 0x14A1, 0x0528, 0x37B3, 0x263A, 0xDECD, 0xCF44, 0xFDDF,
    0xEC56, 0x98E9, 0x8960, 0xBBFB, 0xAA72, 0x6306, 0x728F, 0x4014, 0x519D, 0x2522, 0x34AB, 0x0630, 0x17B9,
    0xEF4E, 0xFEC7, 0xCC5C, 0xDDD5, 0xA96A, 0xB8E3, 0x8A78, 0x9BF1, 0x7387, 0x620E, 0x5095, 0x411C, 0x35A3,
    0x242A, 0x16B1, 0x0738, 0xFFCF, 0xEE46, 0xDCDD, 0xCD54, 0xB9EB, 0xA862, 0x9AF9, 0x8B70, 0x8408, 0x9581,
    0xA71A, 0xB693, 0xC22C, 0xD3A5, 0xE13E, 0xF0B7, 0x0840, 0x19C9, 0x2B52, 0x3ADB, 0x4E64, 0x5FED, 0x6D76,
    0x7CFF, 0x9489, 0x8500, 0xB79B, 0xA612, 0xD2AD, 0xC324, 0xF1BF, 0xE036, 0x18C1, 0x0948, 0x3BD3, 0x2A5A,
    0x5EE5, 0x4F6C, 0x7DF7, 0x6C7E, 0xA50A, 0xB483, 0x8618, 0x9791, 0xE32E, 0xF2A7, 0xC03C, 0xD1B5, 0x2942,
    0x38CB, 0x0A50, 0x1BD9, 0x6F66, 0x7EEF, 0x4C74, 0x5DFD, 0xB58B, 0xA402, 0x9699, 0x8710, 0xF3AF, 0xE226,
    0xD0BD, 0xC134, 0x39C3, 0x284A, 0x1AD1, 0x0B58, 0x7FE7, 0x6E6E, 0x5CF5, 0x4D7C, 0xC60C, 0xD785, 0xE51E,
    0xF497, 0x8028, 0x91A1, 0xA33A, 0xB2B3, 0x4A44, 0x5BCD, 0x6956, 0x78DF, 0x0C60, 0x1DE9, 0x2F72, 0x3EFB,
    0xD68D, 0xC704, 0xF59F, 0xE416, 0x90A9, 0x8120, 0xB3BB, 0xA232, 0x5AC5, 0x4B4C, 0x79D7, 0x685E, 0x1CE1,
    0x0D68, 0x3FF3, 0x2E7A, 0xE70E, 0xF687, 0xC41C, 0xD595, 0xA12A, 0xB0A3, 0x8238, 0x93B1, 0x6B46, 0x7ACF,
    0x4854, 0x59DD, 0x2D62, 0x3CEB, 0x0E70, 0x1FF9, 0xF78F, 0xE606, 0xD49D, 0xC514, 0xB1AB, 0xA022, 0x92B9,
    0x8330, 0x7BC7, 0x6A4E, 0x58D5, 0x495C, 0x3DE3, 0x2C6A, 0x1EF1, 0x0F78,
};

static float go8010_clamp(float value, float min_value, float max_value)
{
    if (value < min_value)
    {
        return min_value;
    }
    if (value > max_value)
    {
        return max_value;
    }
    return value;
}

static uint16_t go8010_crc_ccitt_byte(uint16_t crc, uint8_t data)
{
    return (uint16_t)((crc >> 8) ^ g_go8010_crc_ccitt_table[(crc ^ data) & 0xFFu]);
}

static uint16_t go8010_crc_ccitt(const uint8_t* buffer, size_t length)
{
    uint16_t crc = 0u;

    while (length-- > 0u)
    {
        crc = go8010_crc_ccitt_byte(crc, *buffer++);
    }

    return crc;
}

static OmBool go8010_feedback_values_sane(
    const Go8010MotorDrv* motor,
    uint8_t mode,
    float torque,
    float speed,
    float position,
    uint32_t now_ms)
{
    float max_position_step_rad = 0.0f;
    uint32_t dt_ms = 0u;

    if (motor == OM_NULL)
    {
        return OM_FALSE;
    }

    if (mode != GO8010_CONTROL_MODE_JOINT)
    {
        return OM_FALSE;
    }

    if (torque < (GO8010_TORQUE_MIN_NM - GO8010_FEEDBACK_TORQUE_EPS_NM) ||
        torque > (GO8010_TORQUE_MAX_NM + GO8010_FEEDBACK_TORQUE_EPS_NM))
    {
        return OM_FALSE;
    }

    if (speed < (GO8010_SPEED_MIN_RAD_S - GO8010_FEEDBACK_SPEED_EPS_RAD_S) ||
        speed > (GO8010_SPEED_MAX_RAD_S + GO8010_FEEDBACK_SPEED_EPS_RAD_S))
    {
        return OM_FALSE;
    }

    if (position < (GO8010_POSITION_MIN_RAD - GO8010_FEEDBACK_POSITION_EPS_RAD) ||
        position > (GO8010_POSITION_MAX_RAD + GO8010_FEEDBACK_POSITION_EPS_RAD))
    {
        return OM_FALSE;
    }

    if (motor->feedback.timestampMs == 0u)
    {
        return OM_TRUE;
    }

    dt_ms = now_ms - motor->feedback.timestampMs;
    if (dt_ms > 0u && dt_ms <= 200u)
    {
        max_position_step_rad =
            GO8010_SPEED_MAX_RAD_S * ((float)dt_ms / 1000.0f) +
            GO8010_FEEDBACK_MAX_JUMP_MARGIN_RAD;
        if (fabsf(position - motor->feedback.position) > max_position_step_rad)
        {
            return OM_FALSE;
        }
    }

    return OM_TRUE;
}

static void go8010_drop_rx_cache_prefix(Go8010Bus* bus, size_t drop_length)
{
    if (bus == OM_NULL || drop_length == 0u || bus->rxCacheLength == 0u)
    {
        return;
    }

    if (drop_length >= bus->rxCacheLength)
    {
        bus->rxCacheLength = 0u;
        return;
    }

    memmove(bus->rxCache, bus->rxCache + drop_length, bus->rxCacheLength - drop_length);
    bus->rxCacheLength -= drop_length;
}

static size_t go8010_find_frame_start(const uint8_t* buffer, size_t length)
{
    size_t index = 0u;

    if (buffer == OM_NULL || length < 2u)
    {
        return length;
    }

    for (index = 0u; index + 1u < length; index++)
    {
        if (buffer[index] == GO8010_PACKET_HEAD0 && buffer[index + 1u] == GO8010_PACKET_HEAD1)
        {
            return index;
        }
    }

    return length;
}

static OmBool go8010_is_valid_motor_id(uint8_t id)
{
    return (id <= GO8010_MOTOR_ID_MAX) ? OM_TRUE : OM_FALSE;
}

static OmBool go8010_is_valid_usart6(Device* serial_dev)
{
    char* device_name = OM_NULL;

    if (serial_dev == OM_NULL)
    {
        return OM_FALSE;
    }

    device_name = device_get_name(serial_dev);
    if (device_name == OM_NULL)
    {
        return OM_FALSE;
    }

    return (strcmp(device_name, GO8010_DEVICE_NAME_USART6) == 0) ? OM_TRUE : OM_FALSE;
}

static void go8010_read_callback(Device* dev, void* param, size_t paramsz)
{
    Go8010Bus* bus = (Go8010Bus*)param;

    (void)dev;

    if (bus == OM_NULL)
    {
        return;
    }

    bus->rxAvailableHint = (uint32_t)paramsz;
}

static void go8010_mark_online(Go8010MotorDrv* motor, uint32_t timestamp_ms)
{
    if (motor == OM_NULL)
    {
        return;
    }

    motor->onlineFlag = OM_TRUE;
    motor->feedback.timestampMs = timestamp_ms;
    motor->feedback.sequence++;
}

static void go8010_encode_tx_frame(Go8010MotorDrv* motor)
{
    int16_t torque_raw = 0;
    int16_t speed_raw = 0;
    int16_t kp_raw = 0;
    int16_t kd_raw = 0;
    int32_t position_raw = 0;
    uint16_t crc = 0u;

    motor->target.torque = go8010_clamp(motor->target.torque, GO8010_TORQUE_MIN_NM, GO8010_TORQUE_MAX_NM);
    motor->target.position = go8010_clamp(motor->target.position, GO8010_POSITION_MIN_RAD, GO8010_POSITION_MAX_RAD);
    motor->target.speed = go8010_clamp(motor->target.speed, GO8010_SPEED_MIN_RAD_S, GO8010_SPEED_MAX_RAD_S);
    motor->target.kp = go8010_clamp(motor->target.kp, GO8010_KP_MIN, GO8010_KP_MAX);
    motor->target.kd = go8010_clamp(motor->target.kd, GO8010_KD_MIN, GO8010_KD_MAX);

    /* 编码规则保持与旧工程 GO_M8010_send_data() 一致。 */
    torque_raw = (int16_t)(256.0f * motor->target.torque);
    speed_raw = (int16_t)(motor->target.speed * 256.0f / GO8010_PI_TIMES_TWO_APPROX);
    position_raw = (int32_t)(motor->target.position * 32768.0f / GO8010_PI_TIMES_TWO_APPROX);
    kp_raw = (int16_t)(motor->target.kp * 1280.0f);
    kd_raw = (int16_t)(motor->target.kd * 1280.0f);

    motor->txBuffer[0] = GO8010_PACKET_HEAD0;
    motor->txBuffer[1] = GO8010_PACKET_HEAD1;
    motor->txBuffer[2] = (uint8_t)(((GO8010_CONTROL_MODE_JOINT & 0x0Fu) << 4) | motor->id);
    motor->txBuffer[3] = (uint8_t)(torque_raw & 0xFF);
    motor->txBuffer[4] = (uint8_t)((uint16_t)torque_raw >> 8);
    motor->txBuffer[5] = (uint8_t)(speed_raw & 0xFF);
    motor->txBuffer[6] = (uint8_t)((uint16_t)speed_raw >> 8);
    motor->txBuffer[7] = (uint8_t)(position_raw & 0xFF);
    motor->txBuffer[8] = (uint8_t)(((uint32_t)position_raw >> 8) & 0xFFu);
    motor->txBuffer[9] = (uint8_t)(((uint32_t)position_raw >> 16) & 0xFFu);
    motor->txBuffer[10] = (uint8_t)(((uint32_t)position_raw >> 24) & 0xFFu);
    motor->txBuffer[11] = (uint8_t)(kp_raw & 0xFF);
    motor->txBuffer[12] = (uint8_t)((uint16_t)kp_raw >> 8);
    motor->txBuffer[13] = (uint8_t)(kd_raw & 0xFF);
    motor->txBuffer[14] = (uint8_t)((uint16_t)kd_raw >> 8);

    crc = go8010_crc_ccitt(motor->txBuffer, 15u);
    motor->txBuffer[15] = (uint8_t)(crc & 0xFFu);
    motor->txBuffer[16] = (uint8_t)(crc >> 8);
}

OmRet go8010_init(Go8010Bus* bus, Device* usart6_dev)
{
    SerialCfg serial_cfg = SERIAL_DEFAULT_CFG;
    HalSerial* hal_serial = OM_NULL;

    if (bus == OM_NULL || usart6_dev == OM_NULL)
    {
        return OM_ERROR_NULL;
    }

    if (go8010_is_valid_usart6(usart6_dev) != OM_TRUE)
    {
        return OM_ERROR_PARAM;
    }

    if (bus->serialDev == usart6_dev)
    {
        return OM_OK;
    }

    if (bus->serialDev != OM_NULL)
    {
        return OM_ERR_CONFLICT;
    }

    if (device_check_status(usart6_dev, DEV_STATUS_OPENED) != 0u)
    {
        return OM_ERR_CONFLICT;
    }

    memset(bus, 0, sizeof(*bus));
    bus->serialDev = usart6_dev;

    serial_cfg.baudrate = GO8010_USART6_BAUDRATE;
    serial_cfg.txBufSize = GO8010_USART6_TX_BUFSIZE;
    serial_cfg.rxBufSize = GO8010_USART6_RX_BUFSIZE;

    hal_serial = (HalSerial*)usart6_dev;
    hal_serial->cfg = serial_cfg;

    device_set_param(usart6_dev, bus);
    device_set_read_cb(usart6_dev, go8010_read_callback);

    if (device_open(usart6_dev, SERIAL_O_NBLCK_RX | SERIAL_O_NBLCK_TX) != OM_OK)
    {
        bus->serialDev = OM_NULL;
        return OM_ERROR;
    }

    return OM_OK;
}

OmRet go8010_register(Go8010Bus* bus, Go8010MotorDrv* motor, uint8_t id)
{
    if (bus == OM_NULL || motor == OM_NULL)
    {
        return OM_ERROR_NULL;
    }

    if (bus->serialDev == OM_NULL)
    {
        return OM_ERROR;
    }

    if (go8010_is_valid_motor_id(id) != OM_TRUE)
    {
        return OM_ERROR_PARAM;
    }

    if (bus->motorMap[id] != OM_NULL)
    {
        return OM_ERR_CONFLICT;
    }

    memset(motor, 0, sizeof(*motor));
    motor->id = id;
    motor->mode = GO8010_CONTROL_MODE_JOINT;
    bus->motorMap[id] = motor;

    return OM_OK;
}

void go8010_set_target(Go8010MotorDrv* motor, float torque, float position, float speed, float kp, float kd)
{
    if (motor == OM_NULL)
    {
        return;
    }

    motor->target.torque = torque;
    motor->target.position = position;
    motor->target.speed = speed;
    motor->target.kp = kp;
    motor->target.kd = kd;

    go8010_encode_tx_frame(motor);
    motor->isDirty = 1u;
}

OmRet go8010_send(Go8010Bus* bus, Go8010MotorDrv* motor)
{
    size_t write_count = 0u;

    if (bus == OM_NULL || motor == OM_NULL)
    {
        return OM_ERROR_NULL;
    }

    if (bus->serialDev == OM_NULL)
    {
        return OM_ERROR;
    }

    write_count = device_write(bus->serialDev, 0, motor->txBuffer, GO8010_FRAME_TX_SIZE);
    if (write_count != GO8010_FRAME_TX_SIZE)
    {
        return OM_ERROR;
    }

    motor->isDirty = 0u;
    return OM_OK;
}

OmRet go8010_parse_feedback(Go8010MotorDrv* motor, const uint8_t* frame, size_t frame_length)
{
    int16_t torque_raw = 0;
    int16_t speed_raw = 0;
    int32_t position_raw = 0;
    uint8_t frame_motor_id = 0u;
    uint8_t frame_mode = 0u;
    float torque = 0.0f;
    float speed = 0.0f;
    float position = 0.0f;
    uint32_t now_ms = 0u;

    if (motor == OM_NULL || frame == OM_NULL)
    {
        return OM_ERROR_NULL;
    }

    if (frame_length != GO8010_FRAME_RX_SIZE)
    {
        return OM_ERROR_PARAM;
    }

    frame_motor_id = (uint8_t)(frame[2] & 0x0Fu);
    if (frame_motor_id != motor->id)
    {
        return OM_ERR_CONFLICT;
    }
    frame_mode = (uint8_t)(frame[2] >> 4);

    torque_raw = (int16_t)((uint16_t)frame[3] | ((uint16_t)frame[4] << 8));
    speed_raw = (int16_t)((uint16_t)frame[5] | ((uint16_t)frame[6] << 8));
    position_raw =
        (int32_t)((uint32_t)frame[7] | ((uint32_t)frame[8] << 8) | ((uint32_t)frame[9] << 16) | ((uint32_t)frame[10] << 24));

    torque = (float)torque_raw / 256.0f;
    speed = ((float)speed_raw * GO8010_PI_TIMES_TWO_APPROX / 256.0f) - GO8010_SPEED_OFFSET_RAD_S;
    position = (float)position_raw * GO8010_PI_TIMES_TWO_APPROX / 32768.0f;
    now_ms = osal_time_now_monotonic();

    if (go8010_feedback_values_sane(
            motor,
            frame_mode,
            torque,
            speed,
            position,
            now_ms) != OM_TRUE)
    {
        return OM_ERROR;
    }

    motor->feedback.id = frame_motor_id;
    motor->feedback.mode = frame_mode;
    motor->feedback.torque = torque;
    motor->feedback.speed = speed;
    motor->feedback.position = position;
    motor->mode = motor->feedback.mode;
    go8010_mark_online(motor, now_ms);

    return OM_OK;
}

void go8010_rx_service(Go8010Bus* bus)
{
    size_t read_count = 0u;
    size_t frame_start = 0u;
    uint8_t motor_id = 0u;
    Go8010MotorDrv* motor = OM_NULL;

    if (bus == OM_NULL || bus->serialDev == OM_NULL)
    {
        return;
    }

    while (bus->rxCacheLength < GO8010_RX_CACHE_SIZE)
    {
        read_count = device_read(
            bus->serialDev,
            0,
            bus->rxCache + bus->rxCacheLength,
            1u);
        if (read_count == 0u)
        {
            break;
        }

        bus->rxCacheLength += read_count;
    }

    while (bus->rxCacheLength >= GO8010_FRAME_RX_SIZE)
    {
        frame_start = go8010_find_frame_start(bus->rxCache, bus->rxCacheLength);
        if (frame_start >= bus->rxCacheLength)
        {
            if (bus->rxCacheLength < GO8010_FRAME_RX_SIZE)
            {
                break;
            }

            frame_start = 0u;
        }

        if (frame_start > 0u)
        {
            go8010_drop_rx_cache_prefix(bus, frame_start);
            continue;
        }

        motor_id = (uint8_t)(bus->rxCache[2] & 0x0Fu);
        if (go8010_is_valid_motor_id(motor_id) != OM_TRUE)
        {
            if (bus->rxCacheLength > 1u)
            {
                go8010_drop_rx_cache_prefix(bus, 1u);
            }
            continue;
        }

        motor = bus->motorMap[motor_id];
        if (motor == OM_NULL)
        {
            go8010_drop_rx_cache_prefix(bus, 1u);
            continue;
        }

        if (go8010_parse_feedback(motor, bus->rxCache, GO8010_FRAME_RX_SIZE) == OM_OK)
        {
            bus->rxFrameCount++;
            bus->lastRxTimestampMs = osal_time_now_monotonic();
            go8010_drop_rx_cache_prefix(bus, GO8010_FRAME_RX_SIZE);
        }
        else
        {
            go8010_drop_rx_cache_prefix(bus, 1u);
        }
    }

    bus->rxAvailableHint = 0u;
}

void go8010_tx_service(Go8010Bus* bus)
{
    uint32_t motor_index = 0u;

    if (bus == OM_NULL || bus->serialDev == OM_NULL)
    {
        return;
    }

    for (motor_index = 0u; motor_index <= GO8010_MOTOR_ID_MAX; motor_index++)
    {
        Go8010MotorDrv* motor = bus->motorMap[motor_index];

        if (motor == OM_NULL || motor->isDirty == 0u)
        {
            continue;
        }

        (void)go8010_send(bus, motor);
    }
}

void go8010_bus_sync(Go8010Bus* bus)
{
    if (bus == OM_NULL)
    {
        return;
    }

    go8010_tx_service(bus);
    go8010_rx_service(bus);
}

const Go8010Feedback* go8010_get_feedback(const Go8010MotorDrv* motor)
{
    if (motor == OM_NULL)
    {
        return OM_NULL;
    }

    return &motor->feedback;
}

OmBool go8010_is_online(Go8010MotorDrv* motor, uint32_t timeout_ms)
{
    uint32_t now_ms = 0u;
    uint32_t age_ms = 0u;

    if (motor == OM_NULL)
    {
        return OM_FALSE;
    }

    if (motor->feedback.timestampMs == 0u)
    {
        motor->onlineFlag = OM_FALSE;
        return OM_FALSE;
    }

    now_ms = osal_time_now_monotonic();
    age_ms = now_ms - motor->feedback.timestampMs;
    motor->onlineFlag = (age_ms <= timeout_ms) ? OM_TRUE : OM_FALSE;
    return motor->onlineFlag;
}

void g8_capture_zero(Go8010MotorDrv* motor)
{
    if (motor == OM_NULL || motor->initialPositionCaptured == OM_TRUE)
    {
        return;
    }

    if (motor->feedback.timestampMs == 0u)
    {
        return;
    }

    motor->initialPositionZero = motor->feedback.position;
    motor->initialPositionCaptured = OM_TRUE;
}

OmBool g8_get_zero(const Go8010MotorDrv* motor, float* zero_angle_rad)
{
    if (motor == OM_NULL || zero_angle_rad == OM_NULL || motor->initialPositionCaptured != OM_TRUE)
    {
        return OM_FALSE;
    }

    *zero_angle_rad = motor->initialPositionZero;
    return OM_TRUE;
}
