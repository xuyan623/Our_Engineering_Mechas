#include "task/mode_task/mode_task_internal.h"

#include "module/system_health/system_health.h"
#include "osal/osal.h"

static OmBool mode_task_is_iw_up_edge(
    const ModeTaskContext* context,
    const ModeTaskRcSnapshot* snapshot)
{
    return (context->last_iw > RC_IW_UP_THRESHOLD &&
            snapshot->iw <= RC_IW_UP_THRESHOLD)
               ? OM_TRUE
               : OM_FALSE;
}

static OmBool mode_task_is_iw_dn_edge(
    const ModeTaskContext* context,
    const ModeTaskRcSnapshot* snapshot)
{
    return (context->last_iw < RC_IW_DN_THRESHOLD &&
            snapshot->iw >= RC_IW_DN_THRESHOLD)
               ? OM_TRUE
               : OM_FALSE;
}

static OmBool mode_task_is_sw1_to_dn_edge(
    const ModeTaskContext* context,
    const ModeTaskRcSnapshot* snapshot)
{
    return (context->last_sw1 == RC_SWITCH_MI && snapshot->sw1 == RC_SWITCH_DN)
               ? OM_TRUE
               : OM_FALSE;
}

static OmBool mode_task_is_sw1_to_up_edge(
    const ModeTaskContext* context,
    const ModeTaskRcSnapshot* snapshot)
{
    return (context->last_sw1 == RC_SWITCH_MI && snapshot->sw1 == RC_SWITCH_UP)
               ? OM_TRUE
               : OM_FALSE;
}

static GlobalMode mode_task_resolve_global_mode(
    const ModeTaskContext* context,
    const ModeTaskRcSnapshot* snapshot)
{
    if (context == OM_NULL || snapshot == OM_NULL)
    {
        return MODE_GLOBAL_RELEASE_CTRL;
    }

    if (context->shared_state.chassis_mode == MODE_CHASSIS_CUSTOM_CONTROLLER_NORMAL)
    {
        return (context->hierarchy_state.operational.action_enabled == OM_TRUE)
                   ? MODE_GLOBAL_MANUAL_CTRL
                   : MODE_GLOBAL_RELEASE_CTRL;
    }

    if (context->hierarchy_state.operational.action_enabled != OM_TRUE)
    {
        return MODE_GLOBAL_RELEASE_CTRL;
    }

    switch (snapshot->sw2)
    {
    case RC_SWITCH_UP:
        return MODE_GLOBAL_MANUAL_CTRL;
    case RC_SWITCH_MI:
        return MODE_GLOBAL_ENGINEER_CTRL;
    case RC_SWITCH_DN:
    default:
        return MODE_GLOBAL_RELEASE_CTRL;
    }
}

static ChassisMode mode_task_resolve_manual_chassis_mode(
    const ModeTaskContext* context,
    const ModeTaskRcSnapshot* snapshot,
    OmBool custom_controller_online)
{
    ChassisMode current_mode = context->shared_state.chassis_mode;

    if (current_mode == MODE_CHASSIS_CUSTOM_CONTROLLER_NORMAL)
    {
        if (custom_controller_online != OM_TRUE)
        {
            return MODE_CHASSIS_NORMAL;
        }

        if (mode_task_is_iw_dn_edge(context, snapshot) == OM_TRUE &&
            snapshot->sw2 == RC_SWITCH_UP &&
            snapshot->sw1 == RC_SWITCH_MI)
        {
            return MODE_CHASSIS_NORMAL;
        }

        return current_mode;
    }

    if (mode_task_is_iw_up_edge(context, snapshot) == OM_TRUE &&
        snapshot->sw1 == RC_SWITCH_UP)
    {
        return MODE_CHASSIS_PITCH3_TORQUE_COLLECTION;
    }
    if (mode_task_is_iw_up_edge(context, snapshot) == OM_TRUE &&
        snapshot->sw1 == RC_SWITCH_DN)
    {
        return MODE_CHASSIS_CHECK;
    }
    if (mode_task_is_iw_up_edge(context, snapshot) == OM_TRUE &&
        snapshot->sw2 == RC_SWITCH_UP &&
        snapshot->sw1 == RC_SWITCH_MI &&
        current_mode != MODE_CHASSIS_CUSTOM_CONTROLLER_NORMAL)
    {
        return MODE_CHASSIS_SECONDARY_ORE;
    }
    if (mode_task_is_iw_dn_edge(context, snapshot) == OM_TRUE &&
        snapshot->sw2 == RC_SWITCH_UP &&
        snapshot->sw1 == RC_SWITCH_MI &&
        custom_controller_online == OM_TRUE &&
        current_mode != MODE_CHASSIS_CUSTOM_CONTROLLER_NORMAL)
    {
        return MODE_CHASSIS_CUSTOM_CONTROLLER_NORMAL;
    }

    if (mode_task_is_iw_dn_edge(context, snapshot) == OM_TRUE &&
        (current_mode == MODE_CHASSIS_PITCH3_TORQUE_COLLECTION ||
         current_mode == MODE_CHASSIS_URGENT_MEASURE ||
         current_mode == MODE_CHASSIS_SECONDARY_ORE))
    {
        return MODE_CHASSIS_NORMAL;
    }

    if (current_mode != MODE_CHASSIS_PITCH3_TORQUE_COLLECTION &&
        current_mode != MODE_CHASSIS_SECONDARY_ORE &&
        current_mode != MODE_CHASSIS_URGENT_MEASURE &&
        current_mode != MODE_CHASSIS_CUSTOM_CONTROLLER_NORMAL)
    {
        return MODE_CHASSIS_NORMAL;
    }

    return current_mode;
}

static ChassisMode mode_task_resolve_engineer_chassis_mode(
    const ModeTaskContext* context,
    const ModeTaskRcSnapshot* snapshot)
{
    const ChassisMode current_mode = context->shared_state.chassis_mode;

    if (mode_task_is_iw_up_edge(context, snapshot) == OM_TRUE &&
        snapshot->sw2 == RC_SWITCH_MI &&
        snapshot->sw1 == RC_SWITCH_UP)
    {
        return MODE_CHASSIS_GET_ENERGY_UNIT;
    }
    if (mode_task_is_iw_dn_edge(context, snapshot) == OM_TRUE &&
        snapshot->sw2 == RC_SWITCH_MI &&
        snapshot->sw1 == RC_SWITCH_UP)
    {
        return MODE_CHASSIS_GET_ENERGY_UNIT1;
    }
    if (mode_task_is_iw_up_edge(context, snapshot) == OM_TRUE &&
        snapshot->sw2 == RC_SWITCH_MI &&
        snapshot->sw1 == RC_SWITCH_MI)
    {
        return MODE_CHASSIS_EXCHANGE;
    }
    if (mode_task_is_iw_dn_edge(context, snapshot) == OM_TRUE &&
        snapshot->sw2 == RC_SWITCH_MI &&
        snapshot->sw1 == RC_SWITCH_MI)
    {
        return MODE_CHASSIS_GET_ENERGY_UNIT2;
    }
    if (mode_task_is_iw_up_edge(context, snapshot) == OM_TRUE &&
        snapshot->sw2 == RC_SWITCH_MI &&
        snapshot->sw1 == RC_SWITCH_DN)
    {
        return MODE_CHASSIS_PRIMARY;
    }

    switch (current_mode)
    {
    case MODE_CHASSIS_GET_ENERGY_UNIT:
    case MODE_CHASSIS_GET_ENERGY_UNIT1:
    case MODE_CHASSIS_GET_ENERGY_UNIT2:
    case MODE_CHASSIS_EXCHANGE:
    case MODE_CHASSIS_PRIMARY:
        return current_mode;
    default:
        return MODE_CHASSIS_NORMAL;
    }
}

static ChassisMode mode_task_resolve_chassis_mode(
    const ModeTaskContext* context,
    const ModeTaskRcSnapshot* snapshot,
    const DpCustomControllerSnapshot* custom_snapshot,
    GlobalMode global_mode)
{
    const OmBool custom_controller_online =
        (custom_snapshot != OM_NULL && custom_snapshot->online != 0u)
            ? OM_TRUE
            : OM_FALSE;

    switch (global_mode)
    {
    case MODE_GLOBAL_RELEASE_CTRL:
        return MODE_CHASSIS_RELEASE;
    case MODE_GLOBAL_MANUAL_CTRL:
        return mode_task_resolve_manual_chassis_mode(
            context,
            snapshot,
            custom_controller_online);
    case MODE_GLOBAL_ENGINEER_CTRL:
        return mode_task_resolve_engineer_chassis_mode(context, snapshot);
    default:
        return MODE_CHASSIS_RELEASE;
    }
}

static OmBool mode_task_is_clamp_related_mode(ChassisMode chassis_mode)
{
    switch (chassis_mode)
    {
    case MODE_CHASSIS_GET_ENERGY_UNIT:
    case MODE_CHASSIS_GET_ENERGY_UNIT1:
    case MODE_CHASSIS_GET_ENERGY_UNIT2:
    case MODE_CHASSIS_PRIMARY:
    case MODE_CHASSIS_SECONDARY_ORE:
        return OM_TRUE;
    default:
        return OM_FALSE;
    }
}

static void mode_task_update_clamp_action(
    ModeTaskContext* context,
    const ModeTaskRcSnapshot* snapshot,
    ModeTaskSharedState* state)
{
    if (mode_task_is_clamp_related_mode(state->chassis_mode) == OM_FALSE)
    {
        state->clamp_action = MODE_CLAMP_UN_CMD;
        context->flags &= ~MODE_TASK_FLAG_CLAMP_READY;
        return;
    }

    if (snapshot->sw1 == RC_SWITCH_MI)
    {
        context->flags |= MODE_TASK_FLAG_CLAMP_READY;
    }

    if (!(context->flags & MODE_TASK_FLAG_CLAMP_READY))
    {
        return;
    }

    if (mode_task_is_sw1_to_dn_edge(context, snapshot) == OM_TRUE)
    {
        if (state->clamp_action < MODE_CLAMP_ACTION_TWO)
        {
            state->clamp_action = (ClampAction)((uint8_t)state->clamp_action + 1u);
        }
        context->flags &= ~MODE_TASK_FLAG_CLAMP_READY;
    }
    else if (mode_task_is_sw1_to_up_edge(context, snapshot) == OM_TRUE)
    {
        if (state->clamp_action > MODE_CLAMP_UN_CMD)
        {
            state->clamp_action = (ClampAction)((uint8_t)state->clamp_action - 1u);
        }
        context->flags &= ~MODE_TASK_FLAG_CLAMP_READY;
    }
}

static void mode_task_update_exchange_action(
    ModeTaskContext* context,
    const ModeTaskRcSnapshot* snapshot,
    ModeTaskSharedState* state)
{
    if (state->chassis_mode != MODE_CHASSIS_EXCHANGE)
    {
        state->exchange_action = MODE_EXCHANGE_UN_CMD;
        context->flags &= ~MODE_TASK_FLAG_EXCHANGE_READY;
        return;
    }

    if (snapshot->sw1 == RC_SWITCH_MI)
    {
        context->flags |= MODE_TASK_FLAG_EXCHANGE_READY;
        state->exchange_action = MODE_EXCHANGE_UN_CMD;
    }

    if (!(context->flags & MODE_TASK_FLAG_EXCHANGE_READY))
    {
        return;
    }

    if (mode_task_is_sw1_to_dn_edge(context, snapshot) == OM_TRUE)
    {
        state->exchange_action = MODE_EXCHANGE_PICK_ACTION1;
        context->flags &= ~MODE_TASK_FLAG_EXCHANGE_READY;
    }
    else if (mode_task_is_sw1_to_up_edge(context, snapshot) == OM_TRUE)
    {
        state->exchange_action = MODE_EXCHANGE_PICK_ACTION2;
        context->flags &= ~MODE_TASK_FLAG_EXCHANGE_READY;
    }
}

static void mode_task_update_primary_flag(
    ModeTaskContext* context,
    const ModeTaskRcSnapshot* snapshot,
    ModeTaskSharedState* state)
{
    if (state->chassis_mode == MODE_CHASSIS_PRIMARY)
    {
        if (mode_task_is_iw_dn_edge(context, snapshot) == OM_TRUE)
        {
            state->primary_turn_ore_flag = 1u;
        }
    }
    else
    {
        state->primary_turn_ore_flag = 0u;
    }
}

static void mode_task_update_custom_controller_force_takeover(
    ModeTaskContext* context,
    const ModeTaskRcSnapshot* snapshot,
    ModeTaskSharedState* state)
{
    if (context == OM_NULL || snapshot == OM_NULL || state == OM_NULL)
    {
        return;
    }

    if (state->chassis_mode != MODE_CHASSIS_CUSTOM_CONTROLLER_NORMAL)
    {
        state->custom_controller_force_takeover_flag = 0u;
        return;
    }

    if (snapshot->sw1 == RC_SWITCH_MI)
    {
        state->custom_controller_force_takeover_flag = 0u;
    }

    if (mode_task_is_sw1_to_dn_edge(context, snapshot) == OM_TRUE)
    {
        state->custom_controller_force_takeover_flag = 1u;
    }
}

static void mode_task_notify_custom_controller_transition(
    ChassisMode previous_mode,
    ChassisMode next_mode)
{
    OmBool previous_custom =
        (previous_mode == MODE_CHASSIS_CUSTOM_CONTROLLER_NORMAL) ? OM_TRUE : OM_FALSE;
    OmBool next_custom =
        (next_mode == MODE_CHASSIS_CUSTOM_CONTROLLER_NORMAL) ? OM_TRUE : OM_FALSE;

    if (previous_custom != OM_TRUE && next_custom == OM_TRUE)
    {
        sh_set_custom_controller_calibration_pending();
    }
    else if (previous_custom == OM_TRUE && next_custom != OM_TRUE)
    {
        sh_clear_custom_controller_calibration_indicator();
    }
}

static void mode_task_sync_context_history(
    ModeTaskContext* context,
    const ModeTaskRcSnapshot* snapshot)
{
    if (context == OM_NULL || snapshot == OM_NULL)
    {
        return;
    }

    if (context->last_sw1 != snapshot->sw1)
    {
        context->last_last_sw1 = context->last_sw1;
        context->last_sw1 = snapshot->sw1;
    }

    if (context->last_sw2 != snapshot->sw2)
    {
        context->last_sw2 = snapshot->sw2;
    }

    context->last_iw = snapshot->iw;
    context->last_global_mode = context->shared_state.global_mode;
    context->last_chassis_mode = context->shared_state.chassis_mode;
}

void mode_task_run_once(ModeTaskContext* context)
{
    ModeTaskRcSnapshot rc_snapshot = {0};
    DpCustomControllerSnapshot custom_controller_snapshot = {0};
    ModeTaskSharedState next_state = {0};
    OmBool state_changed = OM_FALSE;
    ModeTaskControlSnapshot previous_control_snapshot = {0};
    ModeTaskControlSnapshot next_control_snapshot = {0};
    OmBool control_snapshot_changed = OM_FALSE;
    ChassisMode previous_chassis_mode = MODE_CHASSIS_RELEASE;

    mode_task_drain_rc_snapshots(context);
    mode_task_drain_custom_controller_snapshots(context);
    mode_task_load_rc_snapshot(&rc_snapshot);
    mode_task_load_custom_controller_snapshot(context, &custom_controller_snapshot);

    if (!(context->flags & MODE_TASK_FLAG_INITIALIZED))
    {
        context->last_sw1 = rc_snapshot.sw1;
        context->last_last_sw1 = rc_snapshot.sw1;
        context->last_sw2 = rc_snapshot.sw2;
        context->last_iw = rc_snapshot.iw;
        context->flags |= MODE_TASK_FLAG_INITIALIZED;
    }

    mode_task_drain_init_progress_messages(context);
    mode_task_update_bootstrap_state_from_progress(context);
    mode_task_process_mct_lifecycle_requests(context, &rc_snapshot);
    mode_task_update_operational_system_state(
        context,
        &rc_snapshot,
        &custom_controller_snapshot);

    next_state = context->shared_state;
    previous_chassis_mode = context->shared_state.chassis_mode;
    mode_task_build_control_snapshot(
        context,
        &context->shared_state,
        &previous_control_snapshot);

    if (mode_task_bootstrap_allows_control(context) != OM_TRUE ||
        context->hierarchy_state.system_state != MODE_TASK_SYSTEM_OPERATIONAL)
    {
        mode_task_fill_release_shared_state(&next_state);
    }
    else
    {
        next_state.global_mode = mode_task_resolve_global_mode(context, &rc_snapshot);
        next_state.chassis_mode = mode_task_resolve_chassis_mode(
            context,
            &rc_snapshot,
            &custom_controller_snapshot,
            next_state.global_mode);

        mode_task_update_clamp_action(context, &rc_snapshot, &next_state);
        mode_task_update_exchange_action(context, &rc_snapshot, &next_state);
        mode_task_update_primary_flag(context, &rc_snapshot, &next_state);
        mode_task_update_custom_controller_force_takeover(context, &rc_snapshot, &next_state);
    }

    mode_task_update_operational_domain(
        context,
        &rc_snapshot,
        &custom_controller_snapshot,
        &next_state);

    if (sm_get_current(&context->global_machine) != (StateId)next_state.global_mode)
    {
        (void)sm_force_transition(&context->global_machine, (StateId)next_state.global_mode);
    }
    if (sm_get_current(&context->chassis_machine) != (StateId)next_state.chassis_mode)
    {
        (void)sm_force_transition(&context->chassis_machine, (StateId)next_state.chassis_mode);
    }

    state_changed = mode_task_shared_state_changed(&context->shared_state, &next_state);
    mode_task_notify_custom_controller_transition(previous_chassis_mode, next_state.chassis_mode);
    context->shared_state = next_state;
    mode_task_build_control_snapshot(
        context,
        &context->shared_state,
        &next_control_snapshot);
    control_snapshot_changed = mode_task_control_snapshot_changed(
        &previous_control_snapshot,
        &next_control_snapshot);
    if (control_snapshot_changed == OM_TRUE)
    {
        mode_task_publish_control_snapshot(&next_control_snapshot);
    }
    mode_task_sync_context_history(context, &rc_snapshot);
    mode_task_update_debug_state(context);

    if (state_changed == OM_TRUE)
    {
        g_mode_task_debug.publish_count++;
    }
}
