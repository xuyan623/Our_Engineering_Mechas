#include "driver/damiao/damiao.h"

#include "bsp_dwt.h"
#include "osal/osal_time.h"
#include <string.h>

#define DAMIAO_CAN_CLASSIC_INTERFRAME_DELAY_US    (200u)
#define DAMIAO_PARAM_FRAME_STDID                  (0x7FFu)

/* 各型号 MIT 模式量程直接沿用旧工程定义。
 * 后续若引入更多型号，只在这里扩展，不把范围散落到各任务里。
 */
static const DamiaoMitLimits g_damiao_limits[] = {
    [DAMIAO_MOTOR_TYPE_DM10010L] = {
        .position_min = -12.5f,
        .position_max = 12.5f,
        .velocity_min = -25.0f,
        .velocity_max = 25.0f,
        .torque_min = -200.0f,
        .torque_max = 200.0f,
        .kp_min = 0.0f,
        .kp_max = 500.0f,
        .kd_min = 0.0f,
        .kd_max = 5.0f,
    },
    [DAMIAO_MOTOR_TYPE_DM4310] = {
        .position_min = -12.5f,
        .position_max = 12.5f,
        .velocity_min = -25.0f,
        .velocity_max = 25.0f,
        .torque_min = -10.0f,
        .torque_max = 10.0f,
        .kp_min = 0.0f,
        .kp_max = 500.0f,
        .kd_min = 0.0f,
        .kd_max = 5.0f,
    },
    [DAMIAO_MOTOR_TYPE_DM4340] = {
        .position_min = -12.5f,
        .position_max = 12.5f,
        .velocity_min = -25.0f,
        .velocity_max = 25.0f,
        .torque_min = -28.0f,
        .torque_max = 28.0f,
        .kp_min = 0.0f,
        .kp_max = 500.0f,
        .kd_min = 0.0f,
        .kd_max = 5.0f,
    },
};

static OmBool damiao_type_is_valid(DamiaoMotorType type)
{
    return (type == DAMIAO_MOTOR_TYPE_DM10010L || type == DAMIAO_MOTOR_TYPE_DM4310 || type == DAMIAO_MOTOR_TYPE_DM4340) ? OM_TRUE :
                                                                                                                              OM_FALSE;
}

/* 所有 MIT 输入在编码前都统一钳位，避免上层越界值直接溢出到位段。 */
static float damiao_clamp(float value, float min_value, float max_value)
{
    if (value < min_value)
        return min_value;
    if (value > max_value)
        return max_value;
    return value;
}

/* 旧工程 MIT 协议使用定点压缩编码：
 * - 位置 16bit
 * - 速度 12bit
 * - Kp  12bit
 * - Kd  12bit
 * - 力矩前馈 12bit
 */
static uint16_t damiao_float_to_uint(float value, float min_value, float max_value, uint8_t bits)
{
    float span = 0.0f;
    uint32_t max_code = 0u;

    span = max_value - min_value;
    max_code = (1u << bits) - 1u;
    value = damiao_clamp(value, min_value, max_value);

    return (uint16_t)(((value - min_value) * (float)max_code) / span);
}

OmRet damiao_motor_write_register_u32(Device* can_dev, uint16_t can_id, uint8_t register_id, uint32_t value)
{
    CanUserMsg msg = {0};
    uint8_t payload[8] = {0};

    if (can_dev == OM_NULL)
    {
        return OM_ERROR_NULL;
    }

    payload[0] = (uint8_t)(can_id & 0xFFu);
    payload[1] = (uint8_t)((can_id >> 8) & 0xFFu);
    payload[2] = 0x55u;
    payload[3] = register_id;
    payload[4] = (uint8_t)(value & 0xFFu);
    payload[5] = (uint8_t)((value >> 8) & 0xFFu);
    payload[6] = (uint8_t)((value >> 16) & 0xFFu);
    payload[7] = (uint8_t)((value >> 24) & 0xFFu);

    msg.dsc = CAN_DATA_MSG_DSC_INIT(DAMIAO_PARAM_FRAME_STDID, CAN_IDE_STD, 8u);
    msg.userBuf = payload;

    return ((int)device_write(can_dev, 0, &msg, 1) > 0) ? OM_OK : OM_ERROR;
}

/* 将反馈帧中的定点值还原为物理量。 */
static float damiao_uint_to_float(uint16_t value, float min_value, float max_value, uint8_t bits)
{
    float span = 0.0f;
    uint32_t max_code = 0u;

    span = max_value - min_value;
    max_code = (1u << bits) - 1u;

    return ((float)value * span) / (float)max_code + min_value;
}

/* 使能/失能命令不是普通 MIT 控制帧，而是 7 个 0xFF + 特殊尾字节。 */
static void damiao_fill_special_frame(DamiaoMotorDrv* motor, uint8_t tail_code)
{
    uint32_t index = 0u;

    for (index = 0u; index < 7u; index++)
    {
        motor->txBuffer[index] = 0xFFu;
    }
    motor->txBuffer[7] = tail_code;
    motor->isDirty = 1u;
}

/* 达妙反馈帧位段规则来自旧工程与电机手册：
 * - byte0: 状态码
 * - byte1~2: 位置 16bit
 * - byte3 + byte4[7:4]: 速度 12bit
 * - byte4[3:0] + byte5: 力矩 12bit
 */
static void damiao_parse_feedback_frame(DamiaoMotorDrv* motor, const uint8_t frame[8])
{
    motor->measure.lastStatus = motor->measure.status;
    /* 反馈帧 byte0 的高 4 位是 ERR，低 4 位是控制器 ID。 */
    motor->measure.status = (uint8_t)(frame[0] >> 4);
    motor->measure.positionRaw = (uint16_t)((frame[1] << 8) | frame[2]);
    motor->measure.velocityRaw = (uint16_t)((frame[3] << 4) | (frame[4] >> 4));
    motor->measure.torqueRaw = (uint16_t)(((frame[4] & 0x0Fu) << 8) | frame[5]);

    motor->measure.position =
        damiao_uint_to_float(motor->measure.positionRaw, motor->limits.position_min, motor->limits.position_max, 16u);
    motor->measure.velocity =
        damiao_uint_to_float(motor->measure.velocityRaw, motor->limits.velocity_min, motor->limits.velocity_max, 12u);
    motor->measure.torque = damiao_uint_to_float(motor->measure.torqueRaw, motor->limits.torque_min, motor->limits.torque_max, 12u);

    if (motor->errorCallback != OM_NULL && motor->measure.status != motor->measure.lastStatus)
    {
        motor->errorCallback(motor, motor->measure.status);
    }
}

/* 总线回调只做轻量分发：
 * 1. 从设备层读出一帧
 * 2. 按标准帧 ID 映射到电机对象
 * 3. 更新该电机的最近一次反馈
 */
static void damiao_rx_callback(Device* dev, void* param, CanFilterHandle filter_handle, size_t count)
{
    DamiaoMotorBus* bus = (DamiaoMotorBus*)param;
    CanUserMsg msg = {0};
    uint8_t buffer[8] = {0};

    msg.userBuf = buffer;
    msg.filterHandle = filter_handle;

    for (size_t index = 0u; index < count; index++)
    {
        if (device_read(dev, 0, &msg, 1) > 0)
        {
            uint16_t master_id = (uint16_t)(msg.dsc.id & 0xFFu);
            uint16_t raw_rx_diag_slot = 0xFFFFu;
            bus->rawRxCount++;
            bus->lastRawStdId = msg.dsc.id;
            if (master_id < 6u)
            {
                raw_rx_diag_slot = master_id;
            }
            else if (master_id >= 0x10u && master_id < 0x16u)
            {
                raw_rx_diag_slot = (uint16_t)(master_id - 0x10u);
            }

            if (raw_rx_diag_slot < 6u)
            {
                bus->rawRxByStdId[raw_rx_diag_slot]++;
            }
            DamiaoMotorDrv* motor = bus->rxMap[master_id - DAMIAO_MASTER_ID_MIN];
            if (motor == OM_NULL)
                continue;

            damiao_parse_feedback_frame(motor, buffer);
            motor->measure.timestampMs = osal_time_now_monotonic();
            motor->measure.sequence++;
        }
    }
}

const DamiaoMitLimits* damiao_motor_get_limits(DamiaoMotorType type)
{
    if (damiao_type_is_valid(type) != OM_TRUE)
        return OM_NULL;

    return &g_damiao_limits[type];
}

OmRet damiao_motor_bus_init(DamiaoMotorBus* bus, Device* can_dev)
{
    CanFilterAllocArg alloc_arg = {0};
    OmRet ret = OM_OK;

    if (!bus || !can_dev)
        return OM_ERROR_PARAM;

    memset(bus, 0, sizeof(*bus));
    bus->canDev = can_dev;
    INIT_LIST_HEAD(&bus->txList);

    /* 旧工程生产链只按反馈帧标准 ID 路由达妙反馈。
     * 这里保持相同语义：先接收所有标准帧，再在软件层按 Master ID 分发。
     */
    alloc_arg.request.workMode = CAN_FILTER_MODE_MASK;
    alloc_arg.request.idType = CAN_FILTER_ID_STD;
    alloc_arg.request.id = 0x000u;
    alloc_arg.request.mask = 0x000u;
    alloc_arg.request.rxCallback = damiao_rx_callback;
    alloc_arg.request.param = bus;

    ret = device_ctrl(can_dev, CAN_CMD_FILTER_ALLOC, &alloc_arg);
    if (ret != OM_OK)
        return ret;

    bus->filterHandle = alloc_arg.handle;
    return OM_OK;
}

OmRet damiao_motor_register(DamiaoMotorBus* bus, DamiaoMotorDrv* motor, DamiaoMotorType type, uint16_t can_id, uint16_t master_id)
{
    const DamiaoMitLimits* limits = OM_NULL;

    if (!bus || !motor)
        return OM_ERROR_PARAM;
    if (damiao_type_is_valid(type) != OM_TRUE)
        return OM_ERROR_PARAM;
    if (can_id > 0x7FFu)
        return OM_ERROR_PARAM;
    if (master_id < DAMIAO_MASTER_ID_MIN || master_id > DAMIAO_MASTER_ID_MAX)
        return OM_ERROR_PARAM;

    if (bus->rxMap[master_id - DAMIAO_MASTER_ID_MIN] != OM_NULL)
        return OM_ERR_CONFLICT;

    limits = damiao_motor_get_limits(type);
    if (limits == OM_NULL)
        return OM_ERROR_PARAM;

    memset(motor, 0, sizeof(*motor));
    /* 控制帧 ID 与反馈帧 Master ID 是两套独立参数：
     * - txId: 发给电机的控制帧 CAN ID
     * - rxId: 电机回传反馈时使用的 Master ID
     */
    motor->link.rxId = master_id;
    motor->link.txId = can_id;
    motor->link.type = type;
    motor->link.mode = DAMIAO_CTRL_MODE_MIT;
    motor->limits = *limits;
    INIT_LIST_HEAD(&motor->list);

    bus->rxMap[motor->link.rxId - DAMIAO_MASTER_ID_MIN] = motor;
    list_add_tail(&motor->list, &bus->txList);
    return OM_OK;
}

void damiao_motor_set_mit(DamiaoMotorDrv* motor, float position, float velocity, float kp, float kd, float torque_feedforward)
{
    uint16_t position_raw = 0u;
    uint16_t velocity_raw = 0u;
    uint16_t kp_raw = 0u;
    uint16_t kd_raw = 0u;
    uint16_t torque_raw = 0u;

    if (!motor)
        return;

    motor->target.position = damiao_clamp(position, motor->limits.position_min, motor->limits.position_max);
    motor->target.velocity = damiao_clamp(velocity, motor->limits.velocity_min, motor->limits.velocity_max);
    motor->target.kp = damiao_clamp(kp, motor->limits.kp_min, motor->limits.kp_max);
    motor->target.kd = damiao_clamp(kd, motor->limits.kd_min, motor->limits.kd_max);
    motor->target.torqueFeedforward =
        damiao_clamp(torque_feedforward, motor->limits.torque_min, motor->limits.torque_max);

    /* MIT 8 字节编码布局与旧工程 DM_MIT() 完全一致。 */
    position_raw = damiao_float_to_uint(motor->target.position, motor->limits.position_min, motor->limits.position_max, 16u);
    velocity_raw = damiao_float_to_uint(motor->target.velocity, motor->limits.velocity_min, motor->limits.velocity_max, 12u);
    kp_raw = damiao_float_to_uint(motor->target.kp, motor->limits.kp_min, motor->limits.kp_max, 12u);
    kd_raw = damiao_float_to_uint(motor->target.kd, motor->limits.kd_min, motor->limits.kd_max, 12u);
    torque_raw =
        damiao_float_to_uint(motor->target.torqueFeedforward, motor->limits.torque_min, motor->limits.torque_max, 12u);

    motor->txBuffer[0] = (uint8_t)(position_raw >> 8);
    motor->txBuffer[1] = (uint8_t)position_raw;
    motor->txBuffer[2] = (uint8_t)(velocity_raw >> 4);
    motor->txBuffer[3] = (uint8_t)((velocity_raw << 4) | (kp_raw >> 8));
    motor->txBuffer[4] = (uint8_t)kp_raw;
    motor->txBuffer[5] = (uint8_t)(kd_raw >> 4);
    motor->txBuffer[6] = (uint8_t)((kd_raw << 4) | (torque_raw >> 8));
    motor->txBuffer[7] = (uint8_t)torque_raw;
    motor->isDirty = 1u;
}

void damiao_motor_enable(DamiaoMotorDrv* motor)
{
    if (!motor)
        return;

    damiao_fill_special_frame(motor, DAMIAO_ENABLE_CODE);
}

void damiao_motor_disable(DamiaoMotorDrv* motor)
{
    if (!motor)
        return;

    damiao_fill_special_frame(motor, DAMIAO_DISABLE_CODE);
}

void damiao_motor_bus_sync(DamiaoMotorBus* bus)
{
    ListHead* pos = OM_NULL;

    if (!bus)
        return;

    list_for_each(pos, &bus->txList)
    {
        DamiaoMotorDrv* motor = list_entry(pos, DamiaoMotorDrv, list);

        /* 没有新目标就不重复发，保持“先写缓存，再统一同步”的发送模型。 */
        if (!motor || motor->isDirty == 0u)
            continue;

        CanUserMsg msg = {0};
        msg.dsc = CAN_DATA_MSG_DSC_INIT(motor->link.txId, CAN_IDE_STD, 8);
        msg.userBuf = motor->txBuffer;
        if (motor->link.txId < 6u)
        {
            bus->rawTxByStdId[motor->link.txId]++;
        }

        /* 只在设备层确认写入成功后清脏，失败则保留到下一周期自动重试。 */
        if ((int)device_write(bus->canDev, 0, &msg, 1) > 0)
        {
            motor->isDirty = 0u;

            /* 临时试验：反馈已经稳定后，先注释掉 200us 帧间延时，
             * 观察多电机经典 CAN 下的反馈 cadence、掉帧与总线错误是否回归。
             *
             * DWT_Delay((float)DAMIAO_CAN_CLASSIC_INTERFRAME_DELAY_US / 1000000.0f);
             */
        }
    }
}
