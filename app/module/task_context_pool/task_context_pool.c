#include "module/task_context_pool/task_context_pool.h"
#include <string.h>

typedef union {
    void* ptr_align;
    uint64_t u64_align;
    double f64_align;
} TaskContextPoolAlignProbe;

typedef struct {
    OmBool allocated;
    const char* name;
    uint32_t size;
    uint32_t capacity;
    uint32_t offset;
    const TaskContextVTable* vtable;
} TaskContextSlot;

typedef union {
    TaskContextPoolAlignProbe align;
    uint8_t bytes[TASK_CONTEXT_POOL_BUFFER_BYTES];
} TaskContextPoolStorage;

#define TASK_CONTEXT_POOL_ALIGNMENT_BYTES ((uint32_t)__alignof__(TaskContextPoolAlignProbe))

static TaskContextPoolStorage g_task_context_pool_storage;
static TaskContextSlot g_task_context_pool_slots[TASK_CONTEXT_POOL_MAX_SLOTS];
static uint32_t g_task_context_pool_used_bytes = 0u;

static uint32_t task_context_pool_align_up(uint32_t value, uint32_t alignment)
{
    if (alignment == 0u)
    {
        return value;
    }

    return ((value + alignment - 1u) / alignment) * alignment;
}

static void* task_context_pool_slot_ptr(const TaskContextSlot* slot)
{
    if (slot == OM_NULL)
    {
        return OM_NULL;
    }

    return (void*)&g_task_context_pool_storage.bytes[slot->offset];
}

static void task_context_pool_prepare_slot(
    TaskContextSlot* slot,
    const char* name,
    uint32_t size,
    const TaskContextVTable* vtable)
{
    void* payload = OM_NULL;

    if (slot == OM_NULL)
    {
        return;
    }

    slot->allocated = OM_TRUE;
    slot->name = name;
    slot->size = size;
    slot->vtable = vtable;

    payload = task_context_pool_slot_ptr(slot);
    memset(payload, 0, slot->capacity);

    if (vtable->init != OM_NULL)
    {
        vtable->init(payload);
    }
}

TaskContextSlotId task_context_pool_alloc(const char* name, uint32_t size, const TaskContextVTable* vtable)
{
    uint32_t i = 0u;
    uint32_t aligned_offset = 0u;
    uint32_t aligned_capacity = 0u;

    if (name == OM_NULL || vtable == OM_NULL || size == 0u)
    {
        return 0u;
    }

    aligned_capacity = task_context_pool_align_up(size, TASK_CONTEXT_POOL_ALIGNMENT_BYTES);

    for (i = 0u; i < TASK_CONTEXT_POOL_MAX_SLOTS; i++)
    {
        if (g_task_context_pool_slots[i].allocated == OM_FALSE &&
            g_task_context_pool_slots[i].capacity >= aligned_capacity)
        {
            task_context_pool_prepare_slot(&g_task_context_pool_slots[i], name, size, vtable);
            return (TaskContextSlotId)(i + 1u);
        }
    }

    aligned_offset = task_context_pool_align_up(
        g_task_context_pool_used_bytes,
        TASK_CONTEXT_POOL_ALIGNMENT_BYTES);

    if (aligned_offset > TASK_CONTEXT_POOL_BUFFER_BYTES ||
        aligned_capacity > (TASK_CONTEXT_POOL_BUFFER_BYTES - aligned_offset))
    {
        return 0u;
    }

    for (i = 0u; i < TASK_CONTEXT_POOL_MAX_SLOTS; i++)
    {
        if (g_task_context_pool_slots[i].allocated == OM_FALSE &&
            g_task_context_pool_slots[i].capacity == 0u)
        {
            g_task_context_pool_slots[i].offset = aligned_offset;
            g_task_context_pool_slots[i].capacity = aligned_capacity;
            task_context_pool_prepare_slot(&g_task_context_pool_slots[i], name, size, vtable);
            g_task_context_pool_used_bytes = aligned_offset + aligned_capacity;
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
            task_context_pool_slot_ptr(&g_task_context_pool_slots[index]));
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
            task_context_pool_slot_ptr(&g_task_context_pool_slots[index]));
    }

    memset(
        task_context_pool_slot_ptr(&g_task_context_pool_slots[index]),
        0,
        g_task_context_pool_slots[index].capacity);

    g_task_context_pool_slots[index].allocated = OM_FALSE;
    g_task_context_pool_slots[index].name = OM_NULL;
    g_task_context_pool_slots[index].size = 0u;
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
        task_context_pool_slot_ptr(&g_task_context_pool_slots[index]),
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
        task_context_pool_slot_ptr(&g_task_context_pool_slots[index]),
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

    return task_context_pool_slot_ptr(&g_task_context_pool_slots[index]);
}
