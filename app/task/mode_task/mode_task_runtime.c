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
        !(g_mode_task_owner_context->flags & MT_FLAG_RC_SNAPSHOT_READY))
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
    InputRcSnapshot snapshot = {0};

    if (context == OM_NULL)
    {
        return;
    }

    while (task_pipe_channel_receive(&context->rc_channel, &snapshot, 0u) == OM_OK)
    {
        context->latest_rc_snapshot = snapshot;
        context->flags |= MT_FLAG_RC_SNAPSHOT_READY;
    }
}

void mode_task_load_custom(
    const ModeTaskContext* context,
    InputCustomSnapshot* snapshot)
{
    if (context == OM_NULL || snapshot == OM_NULL)
    {
        return;
    }

    *snapshot = context->latest_custom_snapshot;
}

void mode_task_drain_custom(ModeTaskContext* context)
{
    InputCustomSnapshot snapshot = {0};

    if (context == OM_NULL)
    {
        return;
    }

    while (task_pipe_channel_receive(
               &context->custom_channel,
               &snapshot,
               0u) == OM_OK)
    {
        context->latest_custom_snapshot = snapshot;
    }
}

void mode_task_build_system(
    const ModeTaskContext* context,
    ModeSystemSnap* snapshot)
{
    if (context == OM_NULL || snapshot == OM_NULL)
    {
        return;
    }

    snapshot->operational_phase =
        (uint8_t)sm_get_current(&context->operational_phase_machine);
    snapshot->selected_motion_mode_id =
        context->mode_selection_runtime.selected_motion_mode_id;
    snapshot->confirmed_motion_mode_id =
        context->confirmed_motion_mode_id;
}

void mode_task_build_arm_mode(
    const ModeTaskContext* context,
    ArmTaskModeSnapshot* snapshot)
{
    const uint8_t phase =
        (context != OM_NULL) ? (uint8_t)sm_get_current(&context->operational_phase_machine)
                             : MT_OPERATIONAL_PHASE_RELEASE;
    const uint8_t motion_mode =
        (context != OM_NULL) ? (uint8_t)sm_get_current(&context->motion_mode_machine)
                             : MT_MOTION_MODE_PRESET_ACTION;

    if (context == OM_NULL || snapshot == OM_NULL)
    {
        return;
    }

    memset(snapshot, 0, sizeof(*snapshot));

    if (phase == MT_OPERATIONAL_PHASE_RELEASE)
    {
        snapshot->arm_mode = AT_MODE_RELEASE;
    }
    else if (phase == MT_OPERATIONAL_PHASE_SELECT)
    {
        snapshot->arm_mode = AT_MODE_NORMAL;
    }
    else
    {
        switch ((ModeTaskMotionModeId)motion_mode)
        {
        case MT_MOTION_MODE_PRESET_ACTION:
            snapshot->arm_mode = AT_MODE_PRESET_ACTION;
            break;
        case MT_MOTION_MODE_CUSTOM_TAKEOVER:
            snapshot->arm_mode = AT_MODE_CUSTOM_TAKEOVER;
            break;
        case MT_MOTION_MODE_RC_IK:
            snapshot->arm_mode = AT_MODE_RC_IK;
            break;
        default:
            snapshot->arm_mode = AT_MODE_NORMAL;
            break;
        }
    }

    snapshot->grip_state = context->grip_runtime.grip_state;
    snapshot->ik_solver_mode = context->rc_ik_runtime.ik_solver_mode;
    snapshot->ik_control_bank = context->rc_ik_runtime.ik_control_bank;
    snapshot->preset_action.chassis_mode = (uint8_t)context->preset_action_runtime.chassis_mode;
    snapshot->preset_action.clamp_action = (uint8_t)context->preset_action_runtime.clamp_action;
    snapshot->preset_action.exchange_action = (uint8_t)context->preset_action_runtime.exchange_action;
    snapshot->preset_action.primary_turn_ore_flag = context->preset_action_runtime.primary_turn_ore_flag;
}

void mode_task_build_chassis(
    const ModeTaskContext* context,
    ChassisModeSnap* snapshot)
{
    const uint8_t phase =
        (context != OM_NULL) ? (uint8_t)sm_get_current(&context->operational_phase_machine)
                             : MT_OPERATIONAL_PHASE_RELEASE;
    const uint8_t motion_mode =
        (context != OM_NULL) ? (uint8_t)sm_get_current(&context->motion_mode_machine)
                             : MT_MOTION_MODE_PRESET_ACTION;

    if (context == OM_NULL || snapshot == OM_NULL)
    {
        return;
    }

    memset(snapshot, 0, sizeof(*snapshot));
    snapshot->operational_phase = phase;

    if (phase == MT_OPERATIONAL_PHASE_RELEASE)
    {
        snapshot->wheel_enable = 0u;
        snapshot->leg_enable = 0u;
        snapshot->allow_rc_drive = 0u;
        return;
    }

    if (phase == MT_OPERATIONAL_PHASE_SELECT)
    {
        snapshot->wheel_enable = 1u;
        snapshot->leg_enable = 1u;
        snapshot->allow_rc_drive = 1u;
        return;
    }

    if (motion_mode == MT_MOTION_MODE_RC_IK &&
        context->latest_rc_snapshot.sw1 == RC_SWITCH_DN)
    {
        snapshot->wheel_enable = 0u;
        snapshot->leg_enable = 0u;
        snapshot->allow_rc_drive = 0u;
        return;
    }

    snapshot->wheel_enable = 1u;
    snapshot->leg_enable = 1u;
    snapshot->allow_rc_drive = 1u;
}

OmBool mode_task_system_changed(
    const ModeSystemSnap* lhs,
    const ModeSystemSnap* rhs)
{
    if (lhs == OM_NULL || rhs == OM_NULL)
    {
        return OM_FALSE;
    }

    return (lhs->operational_phase != rhs->operational_phase ||
            lhs->selected_motion_mode_id != rhs->selected_motion_mode_id ||
            lhs->confirmed_motion_mode_id != rhs->confirmed_motion_mode_id)
               ? OM_TRUE
               : OM_FALSE;
}

OmBool mode_task_arm_changed(
    const ArmTaskModeSnapshot* lhs,
    const ArmTaskModeSnapshot* rhs)
{
    if (lhs == OM_NULL || rhs == OM_NULL)
    {
        return OM_FALSE;
    }

    return (lhs->arm_mode != rhs->arm_mode ||
            lhs->grip_state != rhs->grip_state ||
            lhs->ik_solver_mode != rhs->ik_solver_mode ||
            lhs->ik_control_bank != rhs->ik_control_bank ||
            lhs->preset_action.chassis_mode != rhs->preset_action.chassis_mode ||
            lhs->preset_action.clamp_action != rhs->preset_action.clamp_action ||
            lhs->preset_action.exchange_action != rhs->preset_action.exchange_action ||
            lhs->preset_action.primary_turn_ore_flag != rhs->preset_action.primary_turn_ore_flag)
               ? OM_TRUE
               : OM_FALSE;
}

OmBool mode_task_chassis_changed(
    const ChassisModeSnap* lhs,
    const ChassisModeSnap* rhs)
{
    if (lhs == OM_NULL || rhs == OM_NULL)
    {
        return OM_FALSE;
    }

    return (lhs->operational_phase != rhs->operational_phase ||
            lhs->wheel_enable != rhs->wheel_enable ||
            lhs->leg_enable != rhs->leg_enable ||
            lhs->allow_rc_drive != rhs->allow_rc_drive)
               ? OM_TRUE
               : OM_FALSE;
}

void mode_task_publish_snapshots(
    const ArmTaskModeSnapshot* arm_snapshot,
    const ChassisModeSnap* chassis_snapshot)
{
    if (arm_snapshot != OM_NULL)
    {
        (void)arm_task_submit_mode_snapshot(arm_snapshot);
    }

    if (chassis_snapshot != OM_NULL)
    {
        (void)chassis_task_submit_mode(chassis_snapshot);
    }
}

void mode_task_reset_select(ModeTaskContext* context)
{
    if (context == OM_NULL)
    {
        return;
    }

    context->mode_selection_runtime.selected_motion_mode_id =
        MT_MOTION_MODE_NONE;
}

void mode_task_reset_preset(ModeTaskContext* context)
{
    if (context == OM_NULL)
    {
        return;
    }

    context->preset_action_runtime.chassis_mode = MODE_CHASSIS_NORMAL;
    context->preset_action_runtime.clamp_action = MODE_CLAMP_UN_CMD;
    context->preset_action_runtime.exchange_action = MODE_EXCHANGE_UN_CMD;
    context->preset_action_runtime.primary_turn_ore_flag = 0u;
}

void mode_task_reset_rc_ik_runtime(ModeTaskContext* context)
{
    if (context == OM_NULL)
    {
        return;
    }

    context->rc_ik_runtime.ik_solver_mode = MT_IK_SOLVER_FULL_POSE;
    context->rc_ik_runtime.ik_control_bank = MT_IK_BANK_POS_XYZ;
}

void mode_task_reset_grip_runtime(ModeTaskContext* context)
{
    if (context == OM_NULL)
    {
        return;
    }

    context->grip_runtime.grip_state = MT_GRIP_OPEN;
}

void mode_task_enter_release(StateMachine* state_machine, void* user_context)
{
    ModeTaskContext* context = (ModeTaskContext*)user_context;

    (void)state_machine;

    if (context == OM_NULL)
    {
        return;
    }

    mode_task_reset_select(context);
    mode_task_reset_preset(context);
    mode_task_reset_rc_ik_runtime(context);
    if (context->motion_mode_machine.states != OM_NULL)
    {
        (void)sm_force_transition(
            &context->motion_mode_machine,
            (StateId)MT_MOTION_MODE_PRESET_ACTION);
    }
}

void mode_task_enter_select(StateMachine* state_machine, void* user_context)
{
    ModeTaskContext* context = (ModeTaskContext*)user_context;

    (void)state_machine;

    if (context == OM_NULL)
    {
        return;
    }

    mode_task_reset_preset(context);
    context->mode_selection_runtime.selected_motion_mode_id =
        context->confirmed_motion_mode_id;
}

void mode_task_enter_formal(StateMachine* state_machine, void* user_context)
{
    ModeTaskContext* context = (ModeTaskContext*)user_context;
    const uint8_t selected = context->mode_selection_runtime.selected_motion_mode_id;

    (void)state_machine;

    if (context == OM_NULL)
    {
        return;
    }

    if (selected >= MT_MOTION_MODE_PRESET_ACTION &&
        selected <= MT_MOTION_MODE_RC_IK)
    {
        context->confirmed_motion_mode_id = selected;
    }

    (void)sm_force_transition(
        &context->motion_mode_machine,
        (StateId)context->confirmed_motion_mode_id);
}

void mode_task_enter_preset_action(StateMachine* state_machine, void* user_context)
{
    (void)state_machine;
    mode_task_reset_preset((ModeTaskContext*)user_context);
}

void mode_task_exit_preset_action(StateMachine* state_machine, void* user_context)
{
    (void)state_machine;
    mode_task_reset_preset((ModeTaskContext*)user_context);
}

void mode_task_enter_custom(StateMachine* state_machine, void* user_context)
{
    (void)state_machine;
    (void)user_context;
}

void mode_task_exit_custom(StateMachine* state_machine, void* user_context)
{
    (void)state_machine;
    (void)user_context;
}

void mode_task_enter_rc_ik(StateMachine* state_machine, void* user_context)
{
    (void)state_machine;
    mode_task_reset_rc_ik_runtime((ModeTaskContext*)user_context);
}

void mode_task_exit_rc_ik(StateMachine* state_machine, void* user_context)
{
    (void)state_machine;
    mode_task_reset_rc_ik_runtime((ModeTaskContext*)user_context);
}

void mode_task_reset_board_init(ModeBoardInitCtx* context)
{
    if (context == OM_NULL)
    {
        return;
    }

    memset(context, 0, sizeof(*context));
    context->state = MT_BOARD_INIT_CAN_INITIALIZING;
}

void mode_task_reset_motor_init(ModeMotorInitCtx* context)
{
    if (context == OM_NULL)
    {
        return;
    }

    memset(context, 0, sizeof(*context));
    context->state = MT_MOTOR_INIT_CHASSIS_INIT;
}

static void mode_task_reset_rc(ModeTaskPhaseContext* context)
{
    if (context == OM_NULL)
    {
        return;
    }

    context->domain_state = MT_CONTROL_DOMAIN_RC;
    context->rc_link_state = MT_CONTROL_LINK_OFFLINE;
    context->action_enabled = OM_FALSE;
}

static void mode_task_reset_custom(ModeTaskPhaseContext* context)
{
    if (context == OM_NULL)
    {
        return;
    }

    context->domain_state = MT_CONTROL_DOMAIN_CUSTOM;
    context->custom_link_state = MT_CONTROL_LINK_OFFLINE;
    context->custom_control_state = MT_CUSTOM_CONTROL_ALIGNING;
    context->action_enabled = OM_FALSE;
}

void mode_task_reset_phase_context(ModeTaskPhaseContext* context)
{
    if (context == OM_NULL)
    {
        return;
    }

    memset(context, 0, sizeof(*context));
    mode_task_reset_rc(context);
    mode_task_reset_custom(context);
}

static void mode_task_clear_contexts(
    ModeHierarchyCtx* hierarchy_state,
    ModeTaskSystemState next_system_state)
{
    if (hierarchy_state == OM_NULL)
    {
        return;
    }

    switch (next_system_state)
    {
    case MT_SYSTEM_UNINITIALIZED:
    case MT_SYSTEM_BOARD_INITIALIZING:
        mode_task_reset_board_init(&hierarchy_state->board_init);
        mode_task_reset_motor_init(&hierarchy_state->motor_init);
        mode_task_reset_phase_context(&hierarchy_state->operational);
        break;

    case MT_SYSTEM_MOTOR_INITIALIZING:
        mode_task_reset_motor_init(&hierarchy_state->motor_init);
        mode_task_reset_phase_context(&hierarchy_state->operational);
        break;

    case MT_SYSTEM_RELEASE:
        mode_task_reset_phase_context(&hierarchy_state->operational);
        break;

    case MT_SYSTEM_OPERATIONAL:
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

    mode_task_clear_contexts(
        &context->hierarchy_state,
        next_system_state);
    context->hierarchy_state.system_state = next_system_state;
}

void mode_task_update_bootstrap(ModeTaskContext* context)
{
    if (context == OM_NULL)
    {
        return;
    }

    if (context->hierarchy_state.system_state == MT_SYSTEM_UNINITIALIZED)
    {
        mode_task_set_system_state(context, MT_SYSTEM_BOARD_INITIALIZING);
    }

    if (context->hierarchy_state.system_state == MT_SYSTEM_BOARD_INITIALIZING)
    {
        if (!(context->flags & MT_FLAG_INIT_CAN_READY))
        {
            context->hierarchy_state.board_init.state =
                MT_BOARD_INIT_CAN_INITIALIZING;
            return;
        }
        if (!(context->flags & MT_FLAG_INIT_SERIAL_READY))
        {
            context->hierarchy_state.board_init.state =
                MT_BOARD_INIT_SERIAL_INITIALIZING;
            return;
        }
        if (!(context->flags & MT_FLAG_INIT_IMU_READY))
        {
            context->hierarchy_state.board_init.state =
                MT_BOARD_INIT_IMU_INITIALIZING;
            return;
        }

        mode_task_set_system_state(context, MT_SYSTEM_MOTOR_INITIALIZING);
    }

    if (context->hierarchy_state.system_state == MT_SYSTEM_MOTOR_INITIALIZING)
    {
        if (!(context->flags & MT_FLAG_CHASSIS_MOTOR_READY))
        {
            context->hierarchy_state.motor_init.state =
                MT_MOTOR_INIT_CHASSIS_INIT;
            return;
        }
        if (!(context->flags & MT_FLAG_ARM_MOTOR_READY))
        {
            context->hierarchy_state.motor_init.state =
                MT_MOTOR_INIT_ARM_INIT;
            return;
        }

        mode_task_set_system_state(context, MT_SYSTEM_RELEASE);
    }
}

void mode_task_update_system_state(
    ModeTaskContext* context,
    const ModeTaskRcSnapshot* rc_snapshot,
    const InputCustomSnapshot* custom_snapshot)
{
    const OmBool rc_online =
        (rc_snapshot != OM_NULL && rc_snapshot->online != 0u) ? OM_TRUE : OM_FALSE;
    const OmBool operational_active = mct_is_operational_active();
    const ModeTaskPhaseState phase =
        (ModeTaskPhaseState)sm_get_current(&context->operational_phase_machine);

    (void)custom_snapshot;

    if (context == OM_NULL || rc_snapshot == OM_NULL)
    {
        return;
    }

    if (mode_task_bootstrap_allows(context) != OM_TRUE)
    {
        return;
    }

    if (operational_active != OM_TRUE ||
        rc_online != OM_TRUE ||
        phase == MT_OPERATIONAL_PHASE_RELEASE)
    {
        mode_task_set_system_state(context, MT_SYSTEM_RELEASE);
        return;
    }

    mode_task_set_system_state(context, MT_SYSTEM_OPERATIONAL);
}

void mode_task_process_mct(
    ModeTaskContext* context,
    const ModeTaskRcSnapshot* rc_snapshot)
{
    if (context == OM_NULL || rc_snapshot == OM_NULL)
    {
        return;
    }

    if (mode_task_bootstrap_allows(context) != OM_TRUE)
    {
        return;
    }

    if (context->last_sw2 != RC_SWITCH_DN && rc_snapshot->sw2 == RC_SWITCH_DN)
    {
        (void)mct_request_leave();
        return;
    }

    if (context->last_sw2 == RC_SWITCH_DN && rc_snapshot->sw2 == RC_SWITCH_MI)
    {
        (void)mct_request_enter();
        return;
    }
}

void mode_task_update_domain(
    ModeTaskContext* context,
    const ModeTaskRcSnapshot* rc_snapshot,
    const InputCustomSnapshot* custom_snapshot)
{
    const ModeTaskPhaseState phase =
        (ModeTaskPhaseState)sm_get_current(&context->operational_phase_machine);
    const ModeTaskMotionModeId motion_mode =
        (ModeTaskMotionModeId)sm_get_current(&context->motion_mode_machine);
    const OmBool rc_online =
        (rc_snapshot != OM_NULL && rc_snapshot->online != 0u) ? OM_TRUE : OM_FALSE;
    const OmBool custom_online =
        (custom_snapshot != OM_NULL && custom_snapshot->online != 0u) ? OM_TRUE : OM_FALSE;

    if (context == OM_NULL || rc_snapshot == OM_NULL || custom_snapshot == OM_NULL)
    {
        return;
    }

    if (context->hierarchy_state.system_state != MT_SYSTEM_OPERATIONAL)
    {
        mode_task_reset_phase_context(&context->hierarchy_state.operational);
        return;
    }

    context->hierarchy_state.operational.rc_link_state =
        (rc_online == OM_TRUE) ? MT_CONTROL_LINK_ONLINE : MT_CONTROL_LINK_OFFLINE;
    context->hierarchy_state.operational.custom_link_state =
        (custom_online == OM_TRUE) ? MT_CONTROL_LINK_ONLINE : MT_CONTROL_LINK_OFFLINE;
    context->hierarchy_state.operational.action_enabled =
        (phase == MT_OPERATIONAL_PHASE_FORMAL) ? OM_TRUE : OM_FALSE;

    if (phase == MT_OPERATIONAL_PHASE_FORMAL &&
        motion_mode == MT_MOTION_MODE_CUSTOM_TAKEOVER)
    {
        context->hierarchy_state.operational.domain_state = MT_CONTROL_DOMAIN_CUSTOM;
        context->hierarchy_state.operational.custom_control_state =
            (custom_online == OM_TRUE && custom_snapshot->work_mode == 0u)
                ? MT_CUSTOM_CONTROL_TAKEOVER
                : MT_CUSTOM_CONTROL_ALIGNING;
    }
    else
    {
        context->hierarchy_state.operational.domain_state = MT_CONTROL_DOMAIN_RC;
        context->hierarchy_state.operational.custom_control_state =
            MT_CUSTOM_CONTROL_ALIGNING;
    }
}

void mode_task_refresh_snapshots(ModeTaskContext* context)
{
    if (context == OM_NULL)
    {
        return;
    }

    mode_task_build_system(context, &context->system_snapshot);
    mode_task_build_arm_mode(context, &context->arm_mode_snapshot);
    mode_task_build_chassis(context, &context->chassis_mode_snapshot);
}

OmBool mode_task_bootstrap_allows(
    const ModeTaskContext* context)
{
    if (context == OM_NULL)
    {
        return OM_FALSE;
    }

    return (context->hierarchy_state.system_state == MT_SYSTEM_RELEASE ||
            context->hierarchy_state.system_state == MT_SYSTEM_OPERATIONAL)
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
    g_mode_task_debug.operational_phase = context->system_snapshot.operational_phase;
    g_mode_task_debug.control_domain_state = (uint8_t)context->hierarchy_state.operational.domain_state;
    g_mode_task_debug.rc_link_state = (uint8_t)context->hierarchy_state.operational.rc_link_state;
    g_mode_task_debug.custom_link_state = (uint8_t)context->hierarchy_state.operational.custom_link_state;
    g_mode_task_debug.custom_control_state = (uint8_t)context->hierarchy_state.operational.custom_control_state;
    g_mode_task_debug.selected_motion_mode_id = context->system_snapshot.selected_motion_mode_id;
    g_mode_task_debug.confirmed_motion_mode_id = context->system_snapshot.confirmed_motion_mode_id;
    g_mode_task_debug.ik_solver_mode = context->arm_mode_snapshot.ik_solver_mode;
    g_mode_task_debug.ik_control_bank = context->arm_mode_snapshot.ik_control_bank;
    g_mode_task_debug.grip_state = context->arm_mode_snapshot.grip_state;
}

static void mode_task_apply_init_msg(
    ModeTaskContext* context,
    const ModeTaskInitMessage* message)
{
    if (context == OM_NULL || message == OM_NULL)
    {
        return;
    }

    switch ((ModeTaskInitKind)message->kind)
    {
    case MODE_INIT_CAN_READY:
        if (message->value != 0u) { context->flags |= MT_FLAG_INIT_CAN_READY; } else { context->flags &= ~MT_FLAG_INIT_CAN_READY; }
        break;
    case MODE_INIT_SERIAL_READY:
        if (message->value != 0u) { context->flags |= MT_FLAG_INIT_SERIAL_READY; } else { context->flags &= ~MT_FLAG_INIT_SERIAL_READY; }
        break;
    case MODE_INIT_IMU_READY:
        if (message->value != 0u) { context->flags |= MT_FLAG_INIT_IMU_READY; } else { context->flags &= ~MT_FLAG_INIT_IMU_READY; }
        break;
    case MODE_INIT_CHASSIS_MOTOR_READY:
        if (message->value != 0u) { context->flags |= MT_FLAG_CHASSIS_MOTOR_READY; } else { context->flags &= ~MT_FLAG_CHASSIS_MOTOR_READY; }
        break;
    case MODE_INIT_ARM_MOTOR_READY:
        if (message->value != 0u) { context->flags |= MT_FLAG_ARM_MOTOR_READY; } else { context->flags &= ~MT_FLAG_ARM_MOTOR_READY; }
        break;
    default:
        break;
    }
}

void mode_task_drain_init_messages(ModeTaskContext* context)
{
    ModeTaskInitMessage message = {0};

    if (context == OM_NULL)
    {
        return;
    }

    while (tmpsc_receive(
               &g_mode_task_init_progress_channel,
               &message) == OM_OK)
    {
        mode_task_apply_init_msg(context, &message);
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
    self->confirmed_motion_mode_id = MT_MOTION_MODE_PRESET_ACTION;
    self->flags = 0u;
    memset(&self->system_snapshot, 0, sizeof(self->system_snapshot));
    memset(&self->arm_mode_snapshot, 0, sizeof(self->arm_mode_snapshot));
    memset(&self->chassis_mode_snapshot, 0, sizeof(self->chassis_mode_snapshot));
    memset(&self->mode_selection_runtime, 0, sizeof(self->mode_selection_runtime));
    memset(&self->preset_action_runtime, 0, sizeof(self->preset_action_runtime));
    memset(&self->rc_ik_runtime, 0, sizeof(self->rc_ik_runtime));
    memset(&self->grip_runtime, 0, sizeof(self->grip_runtime));
    memset(&self->hierarchy_state, 0, sizeof(self->hierarchy_state));
    memset(&self->latest_rc_snapshot, 0, sizeof(self->latest_rc_snapshot));
    memset(&self->latest_custom_snapshot, 0, sizeof(self->latest_custom_snapshot));

    mode_task_reset_select(self);
    mode_task_reset_preset(self);
    mode_task_reset_rc_ik_runtime(self);
    mode_task_reset_grip_runtime(self);
}

void mode_task_ctx_cleanup(void* ctx)
{
    (void)ctx;
}
