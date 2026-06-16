#ifndef TCP_H
#define TCP_H

/* task_context_pool 的职责边界：
 * - 这是“任务私有上下文注册表 + 诊断接口表”
 * - 它负责各 task local context 的 alloc / reset / cleanup
 * - 它统一暴露 diag_online / diag_snapshot 给 observer 使用
 *
 * 它不负责：
 * - 正式跨任务共享状态
 * - 正式控制真源
 * - 任何业务层状态仲裁
 *
 * 因此：
 * - owner task 仍然是各自 context 的唯一语义 owner
 * - 其它模块只能通过诊断接口只读观测，不能把这里当共享状态池使用
 */

#include "core/om_def.h"
#include <stdint.h>

/* slot id 约定：
 * - 0：无效 slot
 * - 1..TCP_MAX_SLOTS：有效 slot
 */
#define TCP_MAX_SLOTS       (8u)
#define TCP_BUFFER_BYTES    (16u * 1024u)

typedef uint8_t TaskContextSlotId;

/* 每个任务在注册时提交自己的生命周期回调和诊断回调。
 * 这些回调都只作用于该任务自己的 local context。
 *
 * 生命周期合同：
 * - init：只在 alloc 成功后调用一次；payload 已被清零
 * - reset：对“已分配且仍有效”的 context 做原地运行态清理；
 *          不会重新分配，也不会破坏已初始化好的通道/句柄成员
 * - cleanup：只在 free 前调用一次，用于释放 payload 外部资源
 */
typedef struct {
    const char* task_name;
    void (*init)(void* ctx);
    void (*reset)(void* ctx);
    void (*cleanup)(void* ctx);
    void (*diag_online)(void* ctx, uint8_t* out_online);
    void (*diag_snapshot)(void* ctx, float* out_buf, uint32_t cap, uint32_t* out_count);
} TaskContextVTable;

/* alloc 合同：
 * - payload 起始地址满足池内统一对齐要求
 * - pool 会优先复用已释放 slot 的空洞；若没有合适空洞，再从尾部扩展
 * - payload 在调用 init 前会被清零
 */
TaskContextSlotId task_context_pool_alloc(const char* name, uint32_t size, const TaskContextVTable* vtable);
void              task_context_pool_reset(TaskContextSlotId slot_id);
void              task_context_pool_free(TaskContextSlotId slot_id);
OmBool            task_context_pool_allocated(TaskContextSlotId slot_id);

uint32_t          task_context_pool_count(void);
TaskContextSlotId task_context_pool_slot_at(uint32_t index);
const char*       task_context_pool_get_name(TaskContextSlotId slot_id);
void              task_context_pool_diag_online(TaskContextSlotId slot_id, uint8_t* out_online);
void              task_context_pool_diag_snap(TaskContextSlotId slot_id, float* out_buf, uint32_t cap, uint32_t* out_count);

/* 只限任务模块内部使用：拿到 void* 后强转成具体 Context 类型。
 * 非 owner / 非模块内部代码不得把它当跨任务共享入口使用。
 */
void*             task_context_pool_get_ptr(TaskContextSlotId slot_id);

#endif
