#ifndef NEW_ROBOT_DAMIAO_H
#define NEW_ROBOT_DAMIAO_H

#include "drivers/model/device.h"
#include "drivers/peripheral/can/pal_can_dev.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 达妙 MIT 协议来源于 old_robot_code/BSP_CONF/bsp_can.c
 * 与 TASK/info_get_task.c 的发送/反馈逻辑，不是从 OMR 现有驱动提取。
 * 这里按 DJI 驱动的组织方式收敛成总线对象 + 电机对象 + 统一同步发送。
 */

/* 达妙反馈帧的主机 ID（Master ID）使用标准帧范围 0x00 ~ 0x7FF。
 * 当前项目按低 8 位建表已经足够覆盖实机使用的 Master ID。
 */
#define DAMIAO_MASTER_ID_MIN    (0x00u)
#define DAMIAO_MASTER_ID_MAX    (0xFFu)
#define DAMIAO_RX_MAP_SIZE      (DAMIAO_MASTER_ID_MAX - DAMIAO_MASTER_ID_MIN + 1u)
/* 使能/失能命令沿用旧工程的特殊尾字节定义。 */
#define DAMIAO_ENABLE_CODE      (0xFCu)
#define DAMIAO_DISABLE_CODE     (0xFDu)

/* 达妙电机型号。
 * 不同型号只在 MIT 量程和反馈解码范围上不同。
 */
typedef enum
{
    DAMIAO_MOTOR_TYPE_DM10010L = 0u,  /* 大扭矩关节电机，力矩范围最大。 */
    DAMIAO_MOTOR_TYPE_DM4310,         /* 小力矩高速型号。 */
    DAMIAO_MOTOR_TYPE_DM4340,         /* 中等力矩型号。 */
    DAMIAO_MOTOR_TYPE_UNKNOWN,        /* 非法或占位类型。 */
} DamiaoMotorType;

/* 当前驱动阶段只支持 MIT 模式。
 * 后续若补位置速度模式，可在这里扩展模式枚举。
 */
typedef enum
{
    DAMIAO_CTRL_MODE_MIT = 0u,
} DamiaoMotorCtrlMode;

typedef struct DamiaoMotorDrv DamiaoMotorDrv;

/* 错误状态回调。
 * 当反馈帧中的状态码发生变化时触发，便于上层做边沿处理。
 */
typedef void (*DamiaoMotorErrorCallback)(DamiaoMotorDrv* motor, uint8_t status_code);

/* MIT 模式量程配置。
 * 同一型号的位置、速度、力矩、Kp、Kd 编码范围都由这一组常量统一描述。
 */
typedef struct
{
    float position_min;
    float position_max;
    float velocity_min;
    float velocity_max;
    float torque_min;
    float torque_max;
    float kp_min;
    float kp_max;
    float kd_min;
    float kd_max;
} DamiaoMitLimits;

/* 单个达妙电机驱动对象。 */
struct DamiaoMotorDrv
{
    /* 静态链路信息：注册时确定，不在运行时频繁变化。 */
    struct
    {
        uint16_t rxId;                 /* 反馈帧标准帧 ID（Master ID）。 */
        uint16_t txId;                 /* 控制帧标准帧 ID（CAN ID）。 */
        DamiaoMotorType type;          /* 电机型号。 */
        DamiaoMotorCtrlMode mode;      /* 当前控制模式。 */
    } link;

    /* 运行时测量值：由 CAN 反馈回调更新。 */
    struct
    {
        uint8_t status;                /* 当前状态码。 */
        uint8_t lastStatus;            /* 上一帧状态码，用于边沿判断。 */
        uint16_t positionRaw;          /* 原始位置编码。 */
        uint16_t velocityRaw;          /* 原始速度编码。 */
        uint16_t torqueRaw;            /* 原始力矩编码。 */
        float position;                /* 位置反馈，单位 rad。 */
        float velocity;                /* 速度反馈，单位 rad/s。 */
        float torque;                  /* 力矩反馈，单位 Nm 或协议标定值。 */
        uint32_t timestampMs;          /* 最近一次有效反馈时间戳。 */
        uint32_t sequence;             /* 有效反馈序号，用于任务态判断新帧。 */
    } measure;

    /* 目标缓存：控制任务只写目标，不直接发总线。 */
    struct
    {
        float position;                /* 目标位置。 */
        float velocity;                /* 目标速度。 */
        float kp;                      /* MIT 模式位置刚度。 */
        float kd;                      /* MIT 模式阻尼。 */
        float torqueFeedforward;       /* MIT 模式前馈力矩。 */
    } target;

    uint8_t txBuffer[8];               /* 已编码待发的 8 字节控制帧。 */
    uint8_t isDirty;                   /* 脏标记，1 表示本周期需要发送。 */
    DamiaoMitLimits limits;            /* 当前型号对应的量程配置。 */
    DamiaoMotorErrorCallback errorCallback;
    ListHead list;                     /* 挂入 bus->txList 的链表节点。 */
};

/* 一条 CAN 总线对应一个达妙总线对象。 */
typedef struct
{
    Device* canDev;                            /* 已打开的 CAN 设备。 */
    CanFilterHandle filterHandle;             /* 框架分配的接收过滤器句柄。 */
    DamiaoMotorDrv* rxMap[DAMIAO_RX_MAP_SIZE];/* Master ID 到电机对象的直接映射。 */
    ListHead txList;                          /* 待同步发送的电机链表。 */
    volatile uint32_t rawRxCount;            /* 收到的原始反馈帧总数（未做 rxMap 命中判断）。 */
    volatile uint32_t lastRawStdId;          /* 最近一帧原始反馈的标准帧 ID。 */
    volatile uint32_t rawRxByStdId[6];       /* StdId 0~5 的原始回包计数。 */
    volatile uint32_t rawTxByStdId[6];       /* StdId 0~5 的原始发送尝试计数。 */
} DamiaoMotorBus;

/* 初始化达妙总线对象并注册接收过滤器。 */
OmRet damiao_motor_bus_init(DamiaoMotorBus* bus, Device* can_dev);
/* 将单个达妙电机按显式 can_id + master_id 注册到总线上。 */
OmRet damiao_motor_register(DamiaoMotorBus* bus, DamiaoMotorDrv* motor, DamiaoMotorType type, uint16_t can_id, uint16_t master_id);
/* 设置 MIT 控制目标，只更新缓存不立即发送。 */
void damiao_motor_set_mit(DamiaoMotorDrv* motor, float position, float velocity, float kp, float kd, float torque_feedforward);
/* 通过 0x7FF 参数写帧写入一个 32 位寄存器值。 */
OmRet damiao_motor_write_register_u32(Device* can_dev, uint16_t can_id, uint8_t register_id, uint32_t value);
/* 编码使能命令，等待 bus_sync 统一发送。 */
void damiao_motor_enable(DamiaoMotorDrv* motor);
/* 编码失能命令，等待 bus_sync 统一发送。 */
void damiao_motor_disable(DamiaoMotorDrv* motor);
/* 同步发送当前周期所有脏电机的控制帧。 */
void damiao_motor_bus_sync(DamiaoMotorBus* bus);

/* 获取指定型号的 MIT 量程配置。 */
const DamiaoMitLimits* damiao_motor_get_limits(DamiaoMotorType type);

static inline void damiao_motor_config_error_callback(DamiaoMotorDrv* motor, DamiaoMotorErrorCallback callback)
{
    if (motor)
        motor->errorCallback = callback;
}

/* 获取最近一次反馈的位置值。 */
static inline float damiao_motor_get_position(DamiaoMotorDrv* motor)
{
    if (!motor)
        return 0.0f;
    return motor->measure.position;
}

/* 获取最近一次反馈的速度值。 */
static inline float damiao_motor_get_velocity(DamiaoMotorDrv* motor)
{
    if (!motor)
        return 0.0f;
    return motor->measure.velocity;
}

/* 获取最近一次反馈的力矩值。 */
static inline float damiao_motor_get_torque(DamiaoMotorDrv* motor)
{
    if (!motor)
        return 0.0f;
    return motor->measure.torque;
}

/* 获取最近一次反馈的状态码。 */
static inline uint8_t damiao_motor_get_status(DamiaoMotorDrv* motor)
{
    if (!motor)
        return 0u;
    return motor->measure.status;
}

/* 获取上一帧状态码。 */
static inline uint8_t damiao_motor_get_last_status(DamiaoMotorDrv* motor)
{
    if (!motor)
        return 0u;
    return motor->measure.lastStatus;
}

/* 获取最近一次有效反馈时间戳。 */
static inline uint32_t damiao_motor_get_feedback_timestamp_ms(DamiaoMotorDrv* motor)
{
    if (!motor)
        return 0u;
    return motor->measure.timestampMs;
}

/* 获取有效反馈序号。 */
static inline uint32_t damiao_motor_get_feedback_sequence(DamiaoMotorDrv* motor)
{
    if (!motor)
        return 0u;
    return motor->measure.sequence;
}

static inline uint32_t damiao_motor_bus_get_raw_rx_count(const DamiaoMotorBus* bus)
{
    if (!bus)
        return 0u;
    return bus->rawRxCount;
}

static inline uint32_t damiao_motor_bus_get_last_raw_stdid(const DamiaoMotorBus* bus)
{
    if (!bus)
        return 0u;
    return bus->lastRawStdId;
}

static inline uint32_t damiao_motor_bus_get_raw_rx_by_stdid(const DamiaoMotorBus* bus, uint32_t stdid)
{
    if (!bus || stdid >= 6u)
        return 0u;
    return bus->rawRxByStdId[stdid];
}

static inline uint32_t damiao_motor_bus_get_raw_tx_by_stdid(const DamiaoMotorBus* bus, uint32_t stdid)
{
    if (!bus || stdid >= 6u)
        return 0u;
    return bus->rawTxByStdId[stdid];
}

#ifdef __cplusplus
}
#endif

#endif
