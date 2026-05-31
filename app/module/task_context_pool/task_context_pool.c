#include "module/task_context_pool/task_context_pool.h"
#include <string.h>

typedef struct {
    OmBool allocated;
    const char* name;
    uint32_t size;
    uint32_t offset;
    const TaskContextVTable* vtable;
} TaskContextSlot;

static uint8_t g_task_context_pool_buffer[TASK_CONTEXT_POOL_BUFFER_BYTES];
static TaskContextSlot g_task_context_pool_slots[TASK_CONTEXT_POOL_MAX_SLOTS];
static uint32_t g_task_context_pool_used_bytes = 0u;

TaskContextSlotId task_context_pool_alloc(const char* name, uint32_t size, const TaskContextVTable* vtable)
{
    uint32_t i = 0u;

    if (name == OM_NULL || vtable == OM_NULL || size == 0u)
    {
        return 0u;
    }

    if (size > (TASK_CONTEXT_POOL_BUFFER_BYTES - g_task_context_pool_used_bytes))
    {
        return 0u;
    }

    for (i = 0u; i < TASK_CONTEXT_POOL_MAX_SLOTS; i++)
    {
        if (g_task_context_pool_slots[i].allocated == OM_FALSE)
        {
            g_task_context_pool_slots[i].allocated = OM_TRUE;
            g_task_context_pool_slots[i].name = name;
            g_task_context_pool_slots[i].size = size;
            g_task_context_pool_slots[i].offset = g_task_context_pool_used_bytes;
            g_task_context_pool_slots[i].vtable = vtable;

            memset(&g_task_context_pool_buffer[g_task_context_pool_used_bytes], 0, size);
            g_task_context_pool_used_bytes += size;

            if (vtable->init != OM_NULL)
            {
                vtable->init(&g_task_context_pool_buffer[g_task_context_pool_slots[i].offset]);
            }

            return (TaskContextSlotId)(i + 1u);
        }
    }

    return 0u;
}

void task_context_pool_reset(TaskContextSlotId slot_id)
{
    uint32_t index = (uint32_t)slot_id;

    if (index == 0u || index > TASK_CONTEXT_POOL_MAX_SLOTS)
    {
        return;
    }

    index--;

    if (g_task_context_pool_slots[index].allocated == OM_FALSE)
    {
        return;
    }

    if (g_task_context_pool_slots[index].vtable != OM_NULL &&
        g_task_context_pool_slots[index].vtable->reset != OM_NULL)
    {
        g_task_context_pool_slots[index].vtable->reset(
            &g_task_context_pool_buffer[g_task_context_pool_slots[index].offset]);
    }
}

void task_context_pool_free(TaskContextSlotId slot_id)
{
    uint32_t index = (uint32_t)slot_id;

    if (index == 0u || index > TASK_CONTEXT_POOL_MAX_SLOTS)
    {
        return;
    }

    index--;

    if (g_task_context_pool_slots[index].allocated == OM_FALSE)
    {
        return;
    }

    if (g_task_context_pool_slots[index].vtable != OM_NULL &&
        g_task_context_pool_slots[index].vtable->cleanup != OM_NULL)
    {
        g_task_context_pool_slots[index].vtable->cleanup(
            &g_task_context_pool_buffer[g_task_context_pool_slots[index].offset]);
    }

    memset(&g_task_context_pool_buffer[g_task_context_pool_slots[index].offset], 0,
           g_task_context_pool_slots[index].size);

    g_task_context_pool_slots[index].allocated = OM_FALSE;
    g_task_context_pool_slots[index].name = OM_NULL;
    g_task_context_pool_slots[index].vtable = OM_NULL;
}

OmBool task_context_pool_is_allocated(TaskContextSlotId slot_id)
{
    uint32_t index = (uint32_t)slot_id;

    if (index == 0u || index > TASK_CONTEXT_POOL_MAX_SLOTS)
    {
        return OM_FALSE;
    }

    index--;
    return g_task_context_pool_slots[index].allocated;
}

uint32_t task_context_pool_get_allocated_count(void)
{
    uint32_t count = 0u;
    uint32_t i = 0u;

    for (i = 0u; i < TASK_CONTEXT_POOL_MAX_SLOTS; i++)
    {
        if (g_task_context_pool_slots[i].allocated == OM_TRUE)
        {
            count++;
        }
    }

    return count;
}

TaskContextSlotId task_context_pool_get_slot_by_index(uint32_t index)
{
    uint32_t i = 0u;
    uint32_t allocated_seen = 0u;

    for (i = 0u; i < TASK_CONTEXT_POOL_MAX_SLOTS; i++)
    {
        if (g_task_context_pool_slots[i].allocated == OM_TRUE)
        {
            if (allocated_seen == index)
            {
                return (TaskContextSlotId)(i + 1u);
            }
            allocated_seen++;
        }
    }

    return 0u;
}

const char* task_context_pool_get_name(TaskContextSlotId slot_id)
{
    uint32_t index = (uint32_t)slot_id;

    if (index == 0u || index > TASK_CONTEXT_POOL_MAX_SLOTS)
    {
        return OM_NULL;
    }

    index--;
    return g_task_context_pool_slots[index].name;
}

void task_context_pool_call_diag_online(TaskContextSlotId slot_id, uint8_t* out_online)
{
    uint32_t index = (uint32_t)slot_id;

    if (out_online == OM_NULL)
    {
        return;
    }

    *out_online = 0u;

    if (index == 0u || index > TASK_CONTEXT_POOL_MAX_SLOTS)
    {
        return;
    }

    index--;

    if (g_task_context_pool_slots[index].allocated == OM_FALSE ||
        g_task_context_pool_slots[index].vtable == OM_NULL ||
        g_task_context_pool_slots[index].vtable->diag_online == OM_NULL)
    {
        return;
    }

    g_task_context_pool_slots[index].vtable->diag_online(
        &g_task_context_pool_buffer[g_task_context_pool_slots[index].offset],
        out_online);
}

void task_context_pool_call_diag_snapshot(TaskContextSlotId slot_id, float* out_buf, uint32_t cap, uint32_t* out_count)
{
    uint32_t index = (uint32_t)slot_id;

    if (out_buf == OM_NULL || out_count == OM_NULL)
    {
        return;
    }

    *out_count = 0u;

    if (index == 0u || index > TASK_CONTEXT_POOL_MAX_SLOTS)
    {
        return;
    }

    index--;

    if (g_task_context_pool_slots[index].allocated == OM_FALSE ||
        g_task_context_pool_slots[index].vtable == OM_NULL ||
        g_task_context_pool_slots[index].vtable->diag_snapshot == OM_NULL)
    {
        return;
    }

    g_task_context_pool_slots[index].vtable->diag_snapshot(
        &g_task_context_pool_buffer[g_task_context_pool_slots[index].offset],
        out_buf, cap, out_count);
}

void* task_context_pool_get_ptr(TaskContextSlotId slot_id)
{
    uint32_t index = (uint32_t)slot_id;

    if (index == 0u || index > TASK_CONTEXT_POOL_MAX_SLOTS)
    {
        return OM_NULL;
    }

    index--;

    if (g_task_context_pool_slots[index].allocated == OM_FALSE)
    {
        return OM_NULL;
    }

    return (void*)&g_task_context_pool_buffer[g_task_context_pool_slots[index].offset];
}
