#ifndef TASK_CONTEXT_POOL_H
#define TASK_CONTEXT_POOL_H

#include "core/om_def.h"
#include <stdint.h>

#define TASK_CONTEXT_POOL_MAX_SLOTS       (8u)
#define TASK_CONTEXT_POOL_BUFFER_BYTES    (16u * 1024u)

typedef uint8_t TaskContextSlotId;

typedef struct {
    const char* task_name;
    void (*init)(void* ctx);
    void (*reset)(void* ctx);
    void (*cleanup)(void* ctx);
    void (*diag_online)(void* ctx, uint8_t* out_online);
    void (*diag_snapshot)(void* ctx, float* out_buf, uint32_t cap, uint32_t* out_count);
} TaskContextVTable;

TaskContextSlotId task_context_pool_alloc(const char* name, uint32_t size, const TaskContextVTable* vtable);
void              task_context_pool_reset(TaskContextSlotId slot_id);
void              task_context_pool_free(TaskContextSlotId slot_id);
OmBool            task_context_pool_is_allocated(TaskContextSlotId slot_id);

uint32_t          task_context_pool_get_allocated_count(void);
TaskContextSlotId task_context_pool_get_slot_by_index(uint32_t index);
const char*       task_context_pool_get_name(TaskContextSlotId slot_id);
void              task_context_pool_call_diag_online(TaskContextSlotId slot_id, uint8_t* out_online);
void              task_context_pool_call_diag_snapshot(TaskContextSlotId slot_id, float* out_buf, uint32_t cap, uint32_t* out_count);

/* zhi xian ren wu mo kuai nei bu shi yong: na dao void* hou qiang zhuan cheng ju ti Context lei xing */
void*             task_context_pool_get_ptr(TaskContextSlotId slot_id);

#endif
