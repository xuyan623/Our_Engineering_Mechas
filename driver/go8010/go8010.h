#ifndef NEW_ROBOT_GO8010_H
#define NEW_ROBOT_GO8010_H

#include "core/om_def.h"
#include "drivers/model/device.h"
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* GO-8010 USART 协议来源：
 * - old_robot_code/PROTOCOL/motor_8010.h
 * - old_robot_code/PROTOCOL/motor_8010.c
 * 当前驱动保持“USART6 专用 owner”模型，不接入 serial_dispatch。
 */

#define GO8010_FRAME_TX_SIZE          (17u)
#define GO8010_FRAME_RX_SIZE          (16u)
#define GO8010_PACKET_HEAD0           (0xFEu)
#define GO8010_PACKET_HEAD1           (0xEEu)
#define GO8010_CONTROL_MODE_JOINT     (0x01u)
#define GO8010_BROADCAST_ID           (0x0Fu)
#define GO8010_MOTOR_ID_MAX           (0x0Eu)
#define GO8010_RX_MAP_SIZE            (GO8010_BROADCAST_ID + 1u)
#define GO8010_USART6_BAUDRATE        (4000000u)
#define GO8010_USART6_TX_BUFSIZE      (256u)
#define GO8010_USART6_RX_BUFSIZE      (256u)
#define GO8010_RX_CACHE_SIZE          (64u)

typedef struct Go8010MotorDrv Go8010MotorDrv;

typedef struct
{
    Device* serialDev;
    Go8010MotorDrv* motorMap[GO8010_RX_MAP_SIZE];
    volatile uint32_t rxAvailableHint;
    volatile uint32_t rxFrameCount;
    volatile uint32_t lastRxTimestampMs;
    uint8_t rxCache[GO8010_RX_CACHE_SIZE];
    size_t rxCacheLength;
} Go8010Bus;

typedef struct
{
    uint8_t id;
    uint8_t mode;
    float torque;
    float speed;
    float position;
    uint32_t timestampMs;
    uint32_t sequence;
} Go8010Feedback;

struct Go8010MotorDrv
{
    uint8_t id;
    uint8_t mode;
    uint8_t isDirty;
    OmBool onlineFlag;

    struct
    {
        float torque;
        float position;
        float speed;
        float kp;
        float kd;
    } target;

    Go8010Feedback feedback;
    uint8_t txBuffer[GO8010_FRAME_TX_SIZE];
};

/* 绑定 USART6，并初始化 GO-8010 专用串口 owner。 */
OmRet go8010_init(Go8010Bus* bus, Device* usart6_dev);
/* 注册单个 GO-8010 电机到 USART6 总线上。 */
OmRet go8010_register(Go8010Bus* bus, Go8010MotorDrv* motor, uint8_t id);
/* 设置单个电机目标值，只更新缓存与待发帧。 */
void go8010_set_target(Go8010MotorDrv* motor, float torque, float position, float speed, float kp, float kd);
/* 发送单个电机待发帧。 */
OmRet go8010_send(Go8010Bus* bus, Go8010MotorDrv* motor);
/* 解析单帧反馈并更新最近一次测量值。 */
OmRet go8010_parse_feedback(Go8010MotorDrv* motor, const uint8_t* frame, size_t frame_length);
/* 发送当前周期的全部脏电机。 */
void go8010_tx_service(Go8010Bus* bus);
/* 轮询 USART6 的已到达反馈帧，不依赖 serial_dispatch。 */
void go8010_rx_service(Go8010Bus* bus);
/* 兼容 helper：按顺序执行 tx_service + rx_service。 */
void go8010_bus_sync(Go8010Bus* bus);
/* 获取最近一次反馈快照。 */
const Go8010Feedback* go8010_get_feedback(const Go8010MotorDrv* motor);
/* 按超时窗口判断当前电机是否在线。 */
OmBool go8010_is_online(Go8010MotorDrv* motor, uint32_t timeout_ms);

#ifdef __cplusplus
}
#endif

#endif
