#ifndef NEW_ROBOT_EVENT_BUS_H
#define NEW_ROBOT_EVENT_BUS_H

#include "atomic/atomic_simple.h"
#include "core/om_def.h"
#include "osal/osal_event.h"
#include <stdint.h>

typedef enum
{
    EVT_IMU_DATA_READY = 0,
    EVT_RC_DATA_READY,
    EVT_MODE_CHANGED,
    EVT_MOTOR_FEEDBACK_READY,
    EVT_MOTOR_TX_REQUEST,
    EVT_SERIAL_TX_REQUEST,
    EVT_COUNT
} EventId;

/* 单个订阅者的本地视图：
 * - flags/waitMask 预留给需要阻塞等待的单消费者任务
 * - generation/lastSeenGeneration 用于控制任务非阻塞轮询检查
 */
typedef struct
{
    OsalEventFlags* flags;
    OmAtomicUint* generation;
    uint32_t waitMask;
    uint32_t lastSeenGeneration;
} EventSubscription;

/* 每个事件维护两种同步信息：
 * - flags: OSAL 事件标志，适合调度任务阻塞等待
 * - generations: 发布代次，适合控制任务周期轮询
 */
typedef struct
{
    OsalEventFlags* flags[EVT_COUNT];
    OmAtomicUint generations[EVT_COUNT];
} EventBus;

extern EventBus g_event_bus;

OsalStatus event_bus_init(EventBus* bus);
OsalStatus event_bus_subscribe(const EventBus* bus, EventId event, EventSubscription* subscription);
/* 非阻塞检查接口：
 * 控制任务在固定周期内调用，判断“自上次检查以来是否出现过新事件”。
 */
OsalStatus event_bus_check(EventSubscription* subscription, OmBool* has_new_event);
OsalStatus event_bus_publish(const EventBus* bus, EventId event);
OsalStatus event_bus_publish_from_isr(const EventBus* bus, EventId event);

#endif
