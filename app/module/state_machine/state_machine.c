#include "module/state_machine/state_machine.h"

static const State* sm_find_state(const StateMachine* state_machine, StateId state_id)
{
    uint8_t index = 0U;

    if (state_machine == OM_NULL || state_machine->states == OM_NULL)
    {
        return OM_NULL;
    }

    for (index = 0U; index < state_machine->state_count; index++)
    {
        if (state_machine->states[index].id == state_id)
        {
            return &state_machine->states[index];
        }
    }

    return OM_NULL;
}

static OmRet sm_apply_transition(StateMachine* state_machine, StateId target_state)
{
    const State* current_state = OM_NULL;
    const State* next_state = OM_NULL;

    if (state_machine == OM_NULL)
    {
        return OM_ERROR_NULL;
    }

    if (state_machine->current_state == target_state)
    {
        return OM_OK;
    }

    current_state = sm_find_state(state_machine, state_machine->current_state);
    next_state = sm_find_state(state_machine, target_state);
    if (current_state == OM_NULL || next_state == OM_NULL)
    {
        return OM_ERROR_PARAM;
    }

    if (current_state->on_exit != OM_NULL)
    {
        current_state->on_exit(state_machine, state_machine->context);
    }

    /* previous_state 记录的是刚刚退出的状态，而不是初始化前的占位值。 */
    state_machine->previous_state = state_machine->current_state;
    state_machine->current_state = target_state;

    if (next_state->on_enter != OM_NULL)
    {
        next_state->on_enter(state_machine, state_machine->context);
    }

    return OM_OK;
}

OmRet sm_init(StateMachine* state_machine, const State* states, uint8_t state_count, const Transition* transitions,
              uint8_t transition_count, StateId initial_state, void* context)
{
    const State* initial_state_definition = OM_NULL;

    if (state_machine == OM_NULL || states == OM_NULL || state_count == 0U)
    {
        return OM_ERROR_NULL;
    }

    state_machine->states = states;
    state_machine->transitions = transitions;
    state_machine->state_count = state_count;
    state_machine->transition_count = transition_count;
    state_machine->current_state = initial_state;
    state_machine->previous_state = STATE_INVALID;
    state_machine->context = context;

    initial_state_definition = sm_find_state(state_machine, initial_state);
    if (initial_state_definition == OM_NULL)
    {
        state_machine->current_state = STATE_INVALID;
        return OM_ERROR_PARAM;
    }

    if (initial_state_definition->on_enter != OM_NULL)
    {
        initial_state_definition->on_enter(state_machine, state_machine->context);
    }

    return OM_OK;
}

OmRet sm_update(StateMachine* state_machine)
{
    const State* current_state = OM_NULL;
    uint8_t transition_index = 0U;
    OmRet transition_status = OM_OK;

    if (state_machine == OM_NULL)
    {
        return OM_ERROR_NULL;
    }

    current_state = sm_find_state(state_machine, state_machine->current_state);
    if (current_state == OM_NULL)
    {
        return OM_ERROR_PARAM;
    }

    /* 转换规则按表顺序判定，优先级更高的规则必须排在前面。 */
    for (transition_index = 0U; transition_index < state_machine->transition_count; transition_index++)
    {
        const Transition* transition = &state_machine->transitions[transition_index];

        if (transition->from != state_machine->current_state || transition->condition == OM_NULL)
        {
            continue;
        }

        if (transition->condition(state_machine, state_machine->context) == OM_TRUE)
        {
            transition_status = sm_apply_transition(state_machine, transition->to);
            if (transition_status != OM_OK)
            {
                return transition_status;
            }
            break;
        }
    }

    current_state = sm_find_state(state_machine, state_machine->current_state);
    if (current_state == OM_NULL)
    {
        return OM_ERROR_PARAM;
    }

    if (current_state->on_execute != OM_NULL)
    {
        /* on_execute 始终对“转换完成后的当前状态”执行。 */
        current_state->on_execute(state_machine, state_machine->context);
    }

    return OM_OK;
}

StateId sm_get_current(const StateMachine* state_machine)
{
    if (state_machine == OM_NULL)
    {
        return STATE_INVALID;
    }

    return state_machine->current_state;
}

StateId sm_get_previous(const StateMachine* state_machine)
{
    if (state_machine == OM_NULL)
    {
        return STATE_INVALID;
    }

    return state_machine->previous_state;
}

OmRet sm_force_transition(StateMachine* state_machine, StateId target_state)
{
    if (state_machine == OM_NULL)
    {
        return OM_ERROR_NULL;
    }

    return sm_apply_transition(state_machine, target_state);
}
