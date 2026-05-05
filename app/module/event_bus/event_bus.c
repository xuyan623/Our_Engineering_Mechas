#include "module/event_bus/event_bus.h"

#define EVENT_BUS_SIGNAL_MASK (0x01u)

EventBus g_event_bus = {0};

static OmBool event_bus_is_valid_event(EventId event)
{
    return (event >= 0 && event < EVT_COUNT) ? OM_TRUE : OM_FALSE;
}

static void event_bus_reset_bus(EventBus* bus)
{
    uint32_t index = 0U;

    for (index = 0U; index < (uint32_t)EVT_COUNT; index++)
    {
        bus->flags[index] = OM_NULL;
        OM_STORE_RLX(&bus->generations[index], 0U);
    }
}

static void event_bus_delete_created_flags(EventBus* bus, uint32_t created_count)
{
    uint32_t index = 0U;

    for (index = 0U; index < created_count; index++)
    {
        if (bus->flags[index] != OM_NULL)
        {
            (void)osal_event_flags_delete(bus->flags[index]);
            bus->flags[index] = OM_NULL;
        }
    }
}

OsalStatus event_bus_init(EventBus* bus)
{
    uint32_t index = 0U;
    OsalStatus status = OSAL_INVALID;

    if (bus == OM_NULL)
    {
        return OSAL_INVALID;
    }

    event_bus_reset_bus(bus);

    for (index = 0U; index < (uint32_t)EVT_COUNT; index++)
    {
        status = osal_event_flags_create(&bus->flags[index]);
        if (status != OSAL_OK)
        {
            event_bus_delete_created_flags(bus, index);
            return status;
        }
    }

    return OSAL_OK;
}

OsalStatus event_bus_subscribe(const EventBus* bus, EventId event, EventSubscription* subscription)
{
    if (bus == OM_NULL || subscription == OM_NULL || event_bus_is_valid_event(event) == OM_FALSE)
    {
        return OSAL_INVALID;
    }

    if (bus->flags[event] == OM_NULL)
    {
        return OSAL_INVALID;
    }

    subscription->flags = bus->flags[event];
    subscription->generation = (OmAtomicUint*)&bus->generations[event];
    subscription->waitMask = EVENT_BUS_SIGNAL_MASK;
    /* 订阅建立时对齐到当前代次，避免把订阅前的历史事件误判为“新事件”。 */
    subscription->lastSeenGeneration = OM_LOAD_RLX(subscription->generation);

    return OSAL_OK;
}

OsalStatus event_bus_check(EventSubscription* subscription, OmBool* has_new_event)
{
    uint32_t current_generation = 0U;

    if (subscription == OM_NULL || has_new_event == OM_NULL || subscription->generation == OM_NULL)
    {
        return OSAL_INVALID;
    }

    current_generation = OM_LOAD_ACQ(subscription->generation);
    if (current_generation != subscription->lastSeenGeneration)
    {
        /* 发布代次发生变化，说明至少到过一次新事件。
         * 这里不关心事件发生了多少次，只关心“本周期是否更新过”。
         */
        subscription->lastSeenGeneration = current_generation;
        *has_new_event = OM_TRUE;
        return OSAL_OK;
    }

    *has_new_event = OM_FALSE;
    return OSAL_OK;
}

OsalStatus event_bus_publish(const EventBus* bus, EventId event)
{
    if (bus == OM_NULL || event_bus_is_valid_event(event) == OM_FALSE || bus->flags[event] == OM_NULL)
    {
        return OSAL_INVALID;
    }

    /* 先推进代次，再置位唤醒等待者。
     * 控制任务走 generation 轮询路径，调度任务走 flags 等待路径。
     */
    (void)OM_FAA_REL((OmAtomicUint*)&bus->generations[event], 1U);
    return osal_event_flags_set(bus->flags[event], EVENT_BUS_SIGNAL_MASK);
}

OsalStatus event_bus_publish_from_isr(const EventBus* bus, EventId event)
{
    if (bus == OM_NULL || event_bus_is_valid_event(event) == OM_FALSE || bus->flags[event] == OM_NULL)
    {
        return OSAL_INVALID;
    }

    /* ISR 路径保持与线程路径相同的发布顺序。 */
    (void)OM_FAA_REL((OmAtomicUint*)&bus->generations[event], 1U);
    return osal_event_flags_set_from_isr(bus->flags[event], EVENT_BUS_SIGNAL_MASK);
}
