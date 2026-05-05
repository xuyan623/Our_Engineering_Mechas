#ifndef NEW_ROBOT_STATE_MACHINE_H
#define NEW_ROBOT_STATE_MACHINE_H

#include "core/om_def.h"
#include <stdint.h>

typedef uint8_t StateId;

#define STATE_INVALID ((StateId)0xFFu)

typedef struct StateMachine StateMachine;

typedef void (*StateAction)(StateMachine* state_machine, void* context);
typedef OmBool (*TransitionCondition)(StateMachine* state_machine, void* context);

typedef struct
{
    StateId id;
    StateAction on_enter;
    StateAction on_execute;
    StateAction on_exit;
    /* 调试用状态名，运行逻辑不能依赖该字段。 */
    const char* name;
} State;

typedef struct
{
    StateId from;
    StateId to;
    TransitionCondition condition;
} Transition;

struct StateMachine
{
    const State* states;
    const Transition* transitions;
    uint8_t state_count;
    uint8_t transition_count;
    /* current_state 表示最近一次完整转换后的当前激活状态。 */
    StateId current_state;
    /* previous_state 只在真正发生状态转换时更新。 */
    StateId previous_state;
    void* context;
};

OmRet sm_init(StateMachine* state_machine, const State* states, uint8_t state_count, const Transition* transitions,
              uint8_t transition_count, StateId initial_state, void* context);
/* 按顺序扫描转换表：
 * 第一个命中的转换规则立即生效，随后执行转换后当前状态的 on_execute。
 */
OmRet sm_update(StateMachine* state_machine);
StateId sm_get_current(const StateMachine* state_machine);
StateId sm_get_previous(const StateMachine* state_machine);
/* 强制转换会跳过条件判断，但仍保持相同的 enter/exit 调用顺序。 */
OmRet sm_force_transition(StateMachine* state_machine, StateId target_state);

#endif
