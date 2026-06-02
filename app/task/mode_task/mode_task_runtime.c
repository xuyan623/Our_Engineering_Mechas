#include "task/mode_task/mode_task_internal.h"

#include "task/arm_task/arm_task.h"
#include "task/chassis_task/chassis_task.h"
#include "task/motor_communications_task/mct.h"
#include <string.h>

void mode_task_load_rc_snapshot(ModeTaskRcSnapshot* snapshot)
{
    if (snapshot == OM_NULL)
    {
        return;
    }

    memset(snapshot, 0, sizeof(*snapshot));
    if (g_mode_task_owner_context == OM_NULL ||
        !(g_mode_task_owner_context->flags & MODE_TASK_FLAG_RC_SNAPSHOT_READY))
    {
        return;
    }

    snapshot->ch1 = g_mode_task_owner_context->latest_rc_snapshot.ch1;
    snapshot->ch2 = g_mode_task_owner_context->latest_rc_snapshot.ch2;
    snapshot->ch3 = g_mode_task_owner_context->latest_rc_snapshot.ch3;
    snapshot->ch4 = g_mode_task_owner_context->latest_rc_snapshot.ch4;
    snapshot->sw1 = g_mode_task_owner_context->latest_rc_snapshot.sw1;
    snapshot->sw2 = g_mode_task_owner_context->latest_rc_snapshot.sw2;
    snapshot->iw = g_mode_task_owner_context->latest_rc_snapshot.iw;
    snapshot->online = g_mode_task_owner_context->latest_rc_snapshot.online;
}

void mode_task_drain_rc_snapshots(ModeTaskContext* context)
{
    DpRcSnapshot snapshot = {0};

    if (context == OM_NULL)
    {
        return;
    }

    while (task_pipe_channel_receive(&context->rc_channel, &snapshot, 0u) == OM_OK)
    {
        context->latest_rc_snapshot = snapshot;
        context->flags |= MODE_TASK_FLAG_RC_SNAPSHOT_READY;
    }
}

void mode_task_load_custom_controller_snapshot(
    const ModeTaskContext* context,
    DpCustomControllerSnapshot* snapshot)
{
    if (context == OM_NULL || snapshot == OM_NULL)
    {
        return;
    }

    *snapshot = context->latest_custom_controller_snapshot;
}

void mode_task_drain_custom_controller_snapshots(ModeTaskContext* context)
{
    DpCustomControllerSnapshot snapshot = {0};

    if (context == OM_NULL)
    {
        return;
    }

    while (task_pipe_channel_receive(
               &context->custom_controller_channel,
               &snapshot,
               0u) == OM_OK)
    {
        context->latest_custom_controller_snapshot = snapshot;
    }
}

void mode_task_build_control_snapshot(
    const ModeTaskContext* context,
    const ModeTaskSharedState* state,
    ModeTaskControlSnapshot* snapshot)
{
    if (context == OM_NULL || state == OM_NULL || snapshot == OM_NULL)
    {
        return;
    }

    snapshot->system_state = (uint8_t)context->hierarchy_state.system_state;
    snapshot->control_domain_state =
        (uint8_t)context->hierarchy_state.operational.domain_state;
    snapshot->global_mode = (uint8_t)state->global_mode;
    snapshot->chassis_mode = (uint8_t)state->chassis_mode;
    snapshot->clamp_action = (uint8_t)state->clamp_action;
    snapshot->exchange_action = (uint8_t)state->exchange_action;
    snapshot->primary_turn_ore_flag = state->primary_turn_ore_flag;
    snapshot->custom_controller_force_takeover_flag =
        state->custom_controller_force_takeover_flag;
}

OmBool mode_task_control_snapshot_changed(
    const ModeTaskControlSnapshot* lhs,
    const ModeTaskControlSnapshot* rhs)
{
    if (lhs == OM_NULL || rhs == OM_NULL)
    {
        return OM_FALSE;
    }

    return (lhs->system_state != rhs->system_state ||
            lhs->control_domain_state != rhs->control_domain_state ||
            lhs->global_mode != rhs->global_mode ||
            lhs->chassis_mode != rhs->chassis_mode ||
            lhs->clamp_action != rhs->clamp_action ||
            lhs->exchange_action != rhs->exchange_action ||
            lhs->primary_turn_ore_flag != rhs->primary_turn_ore_flag ||
            lhs->custom_controller_force_takeover_flag !=
                rhs->custom_controller_force_takeover_flag)
               ? OM_TRUE
               : OM_FALSE;
}

void mode_task_publish_control_snapshot(
    const ModeTaskControlSnapshot* snapshot)
{
    if (snapshot == OM_NULL)
    {
        return;
    }

    (void)chassis_task_submit_mode_control_snapshot(snapshot);
    (void)arm_task_submit_mode_control_snapshot(snapshot);
}

OmBool mode_task_shared_state_changed(
    const ModeTaskSharedState* lhs,
    const ModeTaskSharedState* rhs)
{
    if (lhs == OM_NULL || rhs == OM_NULL)
    {
        return OM_FALSE;
    }

    return (lhs->global_mode != rhs->global_mode ||
            lhs->chassis_mode != rhs->chassis_mode ||
            lhs->clamp_action != rhs->clamp_action ||
            lhs->exchange_action != rhs->exchange_action ||
            lhs->primary_turn_ore_flag != rhs->primary_turn_ore_flag ||
            lhs->custom_controller_force_takeover_flag !=
                rhs->custom_controller_force_takeover_flag)
               ? OM_TRUE
               : OM_FALSE;
}

void mode_task_fill_release_shared_state(ModeTaskSharedState* state)
{
    if (state == OM_NULL)
    {
        return;
    }

    state->global_mode = MODE_GLOBAL_RELEASE_CTRL;
    state->chassis_mode = MODE_CHASSIS_RELEASE;
    state->clamp_action = MODE_CLAMP_UN_CMD;
    state->exchange_action = MODE_EXCHANGE_UN_CMD;
    state->primary_turn_ore_flag = 0u;
    state->custom_controller_force_takeover_flag = 0u;
}

void mode_task_board_init_context_reset(ModeTaskBoardInitContext* context)
{
    if (context == OM_NULL)
    {
        return;
    }

    memset(context, 0, sizeof(*context));
    context->state = MODE_TASK_BOARD_INIT_CAN_INITIALIZING;
}

void mode_task_motor_init_context_reset(ModeTaskMotorInitContext* context)
{
    if (context == OM_NULL)
    {
        return;
    }

    memset(context, 0, sizeof(*context));
    context->state = MODE_TASK_MOTOR_INIT_CHASSIS_INITIALIZING;
}

static void mode_task_rc_control_context_reset(ModeTaskOperationalContext* context)
{
    if (context == OM_NULL)
    {
        return;
    }

    context->domain_state = MODE_TASK_CONTROL_DOMAIN_RC;
    context->rc_link_state = MODE_TASK_CONTROL_LINK_OFFLINE;
    context->action_enabled = OM_FALSE;
}

static void mode_task_custom_control_context_reset(ModeTaskOperationalContext* context)
{
    if (context == OM_NULL)
    {
        return;
    }

    context->domain_state = MODE_TASK_CONTROL_DOMAIN_CUSTOM;
    context->custom_link_state = MODE_TASK_CONTROL_LINK_OFFLINE;
    context->custom_control_state = MODE_TASK_CUSTOM_CONTROL_ALIGNING;
    context->action_enabled = OM_FALSE;
}

void mode_task_operational_context_reset(ModeTaskOperationalContext* context)
{
    if (context == OM_NULL)
    {
        return;
    }

    memset(context, 0, sizeof(*context));
    mode_task_rc_control_context_reset(context);
    mode_task_custom_control_context_reset(context);
}

static void mode_task_clear_contexts_from_system_state(
    ModeTaskHierarchyContext* hierarchy_state,
    ModeTaskSystemState next_system_state)
{
    if (hierarchy_state == OM_NULL)
    {
        return;
    }

    switch (next_system_state)
    {
    case MODE_TASK_SYSTEM_UNINITIALIZED:
        mode_task_board_init_context_reset(&hierarchy_state->board_init);
        mode_task_motor_init_context_reset(&hierarchy_state->motor_init);
        mode_task_operational_context_reset(&hierarchy_state->operational);
        break;

    case MODE_TASK_SYSTEM_BOARD_INITIALIZING:
        mode_task_board_init_context_reset(&hierarchy_state->board_init);
        mode_task_motor_init_context_reset(&hierarchy_state->motor_init);
        mode_task_operational_context_reset(&hierarchy_state->operational);
        break;

    case MODE_TASK_SYSTEM_MOTOR_INITIALIZING:
        mode_task_motor_init_context_reset(&hierarchy_state->motor_init);
        mode_task_operational_context_reset(&hierarchy_state->operational);
        break;

    case MODE_TASK_SYSTEM_RELEASE:
        mode_task_operational_context_reset(&hierarchy_state->operational);
        break;

    case MODE_TASK_SYSTEM_OPERATIONAL:
    default:
        break;
    }
}

static void mode_task_set_system_state(
    ModeTaskContext* context,
    ModeTaskSystemState next_system_state)
{
    if (context == OM_NULL ||
        context->hierarchy_state.system_state == next_system_state)
    {
        return;
    }

    mode_task_clear_contexts_from_system_state(
        &context->hierarchy_state,
        next_system_state);
    context->hierarchy_state.system_state = next_system_state;
}

void mode_task_update_bootstrap_state_from_progress(ModeTaskContext* context)
{
    if (context == OM_NULL)
    {
        return;
    }

    if (context->hierarchy_state.system_state == MODE_TASK_SYSTEM_UNINITIALIZED)
    {
        mode_task_set_system_state(context, MODE_TASK_SYSTEM_BOARD_INITIALIZING);
    }

    if (context->hierarchy_state.system_state == MODE_TASK_SYSTEM_BOARD_INITIALIZING)
    {
        if (!(context->flags & MODE_TASK_FLAG_INIT_CAN_READY))
        {
            context->hierarchy_state.board_init.state =
                MODE_TASK_BOARD_INIT_CAN_INITIALIZING;
            return;
        }
        if (!(context->flags & MODE_TASK_FLAG_INIT_SERIAL_READY))
        {
            context->hierarchy_state.board_init.state =
                MODE_TASK_BOARD_INIT_SERIAL_INITIALIZING;
            return;
        }
        if (!(context->flags & MODE_TASK_FLAG_INIT_IMU_READY))
        {
            context->hierarchy_state.board_init.state =
                MODE_TASK_BOARD_INIT_IMU_INITIALIZING;
            return;
        }

        mode_task_set_system_state(context, MODE_TASK_SYSTEM_MOTOR_INITIALIZING);
    }

    if (context->hierarchy_state.system_state == MODE_TASK_SYSTEM_MOTOR_INITIALIZING)
    {
        if (!(context->flags & MODE_TASK_FLAG_INIT_CHASSIS_MOTOR_READY))
        {
            context->hierarchy_state.motor_init.state =
                MODE_TASK_MOTOR_INIT_CHASSIS_INITIALIZING;
            return;
        }
        if (!(context->flags & MODE_TASK_FLAG_INIT_ARM_MOTOR_READY))
        {
            context->hierarchy_state.motor_init.state =
                MODE_TASK_MOTOR_INIT_ARM_INITIALIZING;
            return;
        }

        mode_task_set_system_state(context, MODE_TASK_SYSTEM_RELEASE);
    }
}

void mode_task_update_operational_system_state(
    ModeTaskContext* context,
    const ModeTaskRcSnapshot* rc_snapshot,
    const DpCustomControllerSnapshot* custom_snapshot)
{
    const OmBool rc_online =
        (rc_snapshot != OM_NULL && rc_snapshot->online != 0u) ? OM_TRUE : OM_FALSE;
    const OmBool custom_online =
        (custom_snapshot != OM_NULL && custom_snapshot->online != 0u) ? OM_TRUE : OM_FALSE;
    const OmBool custom_mode_active =
        (context != OM_NULL &&
         context->shared_state.chassis_mode == MODE_CHASSIS_CUSTOM_CONTROLLER_NORMAL)
            ? OM_TRUE
            : OM_FALSE;
    const OmBool operational_active = mct_is_operational_active();

    if (context == OM_NULL || rc_snapshot == OM_NULL || custom_snapshot == OM_NULL)
    {
        return;
    }

    if (mode_task_bootstrap_allows_control(context) != OM_TRUE)
    {
        return;
    }

    if (rc_snapshot->sw2 == RC_SWITCH_DN || operational_active != OM_TRUE)
    {
        mode_task_set_system_state(context, MODE_TASK_SYSTEM_RELEASE);
        return;
    }

    if (custom_mode_active == OM_TRUE)
    {
        if (custom_online == OM_TRUE || rc_online == OM_TRUE)
        {
            mode_task_set_system_state(context, MODE_TASK_SYSTEM_OPERATIONAL);
        }
        else
        {
            mode_task_set_system_state(context, MODE_TASK_SYSTEM_RELEASE);
        }
        return;
    }

    if (rc_online == OM_TRUE)
    {
        mode_task_set_system_state(context, MODE_TASK_SYSTEM_OPERATIONAL);
    }
    else
    {
        mode_task_set_system_state(context, MODE_TASK_SYSTEM_RELEASE);
    }
}

void mode_task_process_mct_lifecycle_requests(
    ModeTaskContext* context,
    const ModeTaskRcSnapshot* rc_snapshot)
{
    if (context == OM_NULL || rc_snapshot == OM_NULL)
    {
        return;
    }

    if (mode_task_bootstrap_allows_control(context) != OM_TRUE)
    {
        return;
    }

    if (context->last_sw2 != RC_SWITCH_DN && rc_snapshot->sw2 == RC_SWITCH_DN)
    {
        context->hierarchy_state.operational.action_enabled = OM_FALSE;
        (void)mct_request_leave_operational_state();
        return;
    }

    if (context->last_sw2 == RC_SWITCH_DN && rc_snapshot->sw2 == RC_SWITCH_MI)
    {
        context->hierarchy_state.operational.action_enabled = OM_FALSE;
        (void)mct_request_enter_operational_state();
        return;
    }

    if (context->last_sw2 == RC_SWITCH_MI && rc_snapshot->sw2 == RC_SWITCH_UP)
    {
        context->hierarchy_state.operational.action_enabled = OM_TRUE;
        return;
    }
}

void mode_task_update_operational_domain(
    ModeTaskContext* context,
    const ModeTaskRcSnapshot* rc_snapshot,
    const DpCustomControllerSnapshot* custom_snapshot,
    const ModeTaskSharedState* shared_state)
{
    const OmBool previous_action_enabled =
        (context != OM_NULL) ? context->hierarchy_state.operational.action_enabled : OM_FALSE;
    const OmBool rc_online =
        (rc_snapshot != OM_NULL && rc_snapshot->online != 0u) ? OM_TRUE : OM_FALSE;
    const OmBool custom_online =
        (custom_snapshot != OM_NULL && custom_snapshot->online != 0u) ? OM_TRUE : OM_FALSE;
    const OmBool custom_domain =
        (shared_state != OM_NULL &&
         shared_state->chassis_mode == MODE_CHASSIS_CUSTOM_CONTROLLER_NORMAL)
            ? OM_TRUE
            : OM_FALSE;

    if (context == OM_NULL || rc_snapshot == OM_NULL || custom_snapshot == OM_NULL ||
        shared_state == OM_NULL)
    {
        return;
    }

    if (context->hierarchy_state.system_state != MODE_TASK_SYSTEM_OPERATIONAL)
    {
        mode_task_operational_context_reset(&context->hierarchy_state.operational);
        return;
    }

    context->hierarchy_state.operational.rc_link_state =
        (rc_online == OM_TRUE) ? MODE_TASK_CONTROL_LINK_ONLINE : MODE_TASK_CONTROL_LINK_OFFLINE;
    context->hierarchy_state.operational.custom_link_state =
        (custom_online == OM_TRUE) ? MODE_TASK_CONTROL_LINK_ONLINE : MODE_TASK_CONTROL_LINK_OFFLINE;

    if (custom_domain == OM_TRUE)
    {
        if (context->hierarchy_state.operational.domain_state != MODE_TASK_CONTROL_DOMAIN_CUSTOM)
        {
            mode_task_custom_control_context_reset(&context->hierarchy_state.operational);
            context->hierarchy_state.operational.action_enabled = previous_action_enabled;
        }

        context->hierarchy_state.operational.domain_state = MODE_TASK_CONTROL_DOMAIN_CUSTOM;
        context->hierarchy_state.operational.custom_link_state =
            (custom_online == OM_TRUE) ? MODE_TASK_CONTROL_LINK_ONLINE : MODE_TASK_CONTROL_LINK_OFFLINE;
        context->hierarchy_state.operational.custom_control_state =
            ((arm_task_get_custom_controller_alignment_done() != 0u) ||
             shared_state->custom_controller_force_takeover_flag != 0u)
                ? MODE_TASK_CUSTOM_CONTROL_TAKEOVER
                : MODE_TASK_CUSTOM_CONTROL_ALIGNING;
    }
    else
    {
        if (context->hierarchy_state.operational.domain_state != MODE_TASK_CONTROL_DOMAIN_RC)
        {
            mode_task_rc_control_context_reset(&context->hierarchy_state.operational);
            context->hierarchy_state.operational.action_enabled = previous_action_enabled;
        }

        context->hierarchy_state.operational.domain_state = MODE_TASK_CONTROL_DOMAIN_RC;
        context->hierarchy_state.operational.rc_link_state =
            (rc_online == OM_TRUE) ? MODE_TASK_CONTROL_LINK_ONLINE : MODE_TASK_CONTROL_LINK_OFFLINE;
        context->hierarchy_state.operational.custom_control_state =
            MODE_TASK_CUSTOM_CONTROL_ALIGNING;
    }
}

OmBool mode_task_bootstrap_allows_control(
    const ModeTaskContext* context)
{
    if (context == OM_NULL)
    {
        return OM_FALSE;
    }

    return (context->hierarchy_state.system_state == MODE_TASK_SYSTEM_RELEASE ||
            context->hierarchy_state.system_state == MODE_TASK_SYSTEM_OPERATIONAL)
               ? OM_TRUE
               : OM_FALSE;
}

void mode_task_update_debug_state(const ModeTaskContext* context)
{
    if (context == OM_NULL)
    {
        return;
    }

    g_mode_task_debug.system_state = (uint8_t)context->hierarchy_state.system_state;
    g_mode_task_debug.board_init_state = (uint8_t)context->hierarchy_state.board_init.state;
    g_mode_task_debug.motor_init_state = (uint8_t)context->hierarchy_state.motor_init.state;
    g_mode_task_debug.control_domain_state = (uint8_t)context->hierarchy_state.operational.domain_state;
    g_mode_task_debug.rc_link_state = (uint8_t)context->hierarchy_state.operational.rc_link_state;
    g_mode_task_debug.custom_link_state = (uint8_t)context->hierarchy_state.operational.custom_link_state;
    g_mode_task_debug.custom_control_state = (uint8_t)context->hierarchy_state.operational.custom_control_state;
}

static void mode_task_apply_init_progress_message(
    ModeTaskContext* context,
    const ModeTaskInitProgressMessage* message)
{
    if (context == OM_NULL || message == OM_NULL)
    {
        return;
    }

    switch ((ModeTaskInitProgressKind)message->kind)
    {
    case MODE_TASK_INIT_PROGRESS_CAN_READY:
        if (message->value != 0u) { context->flags |= MODE_TASK_FLAG_INIT_CAN_READY; } else { context->flags &= ~MODE_TASK_FLAG_INIT_CAN_READY; }
        break;
    case MODE_TASK_INIT_PROGRESS_SERIAL_READY:
        if (message->value != 0u) { context->flags |= MODE_TASK_FLAG_INIT_SERIAL_READY; } else { context->flags &= ~MODE_TASK_FLAG_INIT_SERIAL_READY; }
        break;
    case MODE_TASK_INIT_PROGRESS_IMU_READY:
        if (message->value != 0u) { context->flags |= MODE_TASK_FLAG_INIT_IMU_READY; } else { context->flags &= ~MODE_TASK_FLAG_INIT_IMU_READY; }
        break;
    case MODE_TASK_INIT_PROGRESS_CHASSIS_MOTOR_READY:
        if (message->value != 0u) { context->flags |= MODE_TASK_FLAG_INIT_CHASSIS_MOTOR_READY; } else { context->flags &= ~MODE_TASK_FLAG_INIT_CHASSIS_MOTOR_READY; }
        break;
    case MODE_TASK_INIT_PROGRESS_ARM_MOTOR_READY:
        if (message->value != 0u) { context->flags |= MODE_TASK_FLAG_INIT_ARM_MOTOR_READY; } else { context->flags &= ~MODE_TASK_FLAG_INIT_ARM_MOTOR_READY; }
        break;
    default:
        break;
    }
}

void mode_task_drain_init_progress_messages(ModeTaskContext* context)
{
    ModeTaskInitProgressMessage message = {0};

    if (context == OM_NULL)
    {
        return;
    }

    while (task_mpsc_channel_receive_nonblocking(
               &g_mode_task_init_progress_channel,
               &message) == OM_OK)
    {
        mode_task_apply_init_progress_message(context, &message);
    }
}

void mode_task_ctx_init(void* ctx)
{
    ModeTaskContext* self = (ModeTaskContext*)ctx;

    memset(self, 0, sizeof(ModeTaskContext));
}

void mode_task_ctx_reset(void* ctx)
{
    ModeTaskContext* self = (ModeTaskContext*)ctx;

    self->last_sw1 = 0u;
    self->last_last_sw1 = 0u;
    self->last_sw2 = 0u;
    self->last_iw = 0u;
    self->last_global_mode = MODE_GLOBAL_RELEASE_CTRL;
    self->last_chassis_mode = MODE_CHASSIS_RELEASE;
    self->flags = 0u;
    memset(&self->shared_state, 0, sizeof(self->shared_state));
    self->shared_state.global_mode = MODE_GLOBAL_RELEASE_CTRL;
    self->shared_state.chassis_mode = MODE_CHASSIS_RELEASE;
    memset(&self->hierarchy_state, 0, sizeof(self->hierarchy_state));
    memset(&self->latest_rc_snapshot, 0, sizeof(self->latest_rc_snapshot));
    memset(&self->latest_custom_controller_snapshot, 0, sizeof(self->latest_custom_controller_snapshot));
}

void mode_task_ctx_cleanup(void* ctx)
{
    (void)ctx;
}
