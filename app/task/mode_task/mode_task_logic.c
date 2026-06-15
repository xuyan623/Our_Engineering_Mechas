#include "task/mode_task/mode_task_internal.h"

#include "module/system_health/system_health.h"
#include "osal/osal.h"
#include "task/motor_communications_task/mct.h"

static OmBool mode_task_is_iw_dn_edge(
    const ModeTaskContext* context,
    const ModeTaskRcSnapshot* snapshot)
{
    return (context->last_iw < RC_IW_DN_THRESHOLD &&
            snapshot->iw >= RC_IW_DN_THRESHOLD)
               ? OM_TRUE
               : OM_FALSE;
}

static OmBool mode_task_is_iw_up_edge(
    const ModeTaskContext* context,
    const ModeTaskRcSnapshot* snapshot)
{
    return (context->last_iw > RC_IW_UP_THRESHOLD &&
            snapshot->iw <= RC_IW_UP_THRESHOLD)
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

static OmBool mode_task_is_sw2_to_dn_edge(
    const ModeTaskContext* context,
    const ModeTaskRcSnapshot* snapshot)
{
    return (context->last_sw2 != RC_SWITCH_DN && snapshot->sw2 == RC_SWITCH_DN)
               ? OM_TRUE
               : OM_FALSE;
}

static OmBool mode_task_is_sw2_to_up_from_mi_edge(
    const ModeTaskContext* context,
    const ModeTaskRcSnapshot* snapshot)
{
    return (context->last_sw2 == RC_SWITCH_MI && snapshot->sw2 == RC_SWITCH_UP)
               ? OM_TRUE
               : OM_FALSE;
}

static OmBool mode_task_is_sw2_to_mi_from_up_edge(
    const ModeTaskContext* context,
    const ModeTaskRcSnapshot* snapshot)
{
    return (context->last_sw2 == RC_SWITCH_UP && snapshot->sw2 == RC_SWITCH_MI)
               ? OM_TRUE
               : OM_FALSE;
}

static uint8_t mode_task_cycle_motion_mode(uint8_t current, int32_t delta)
{
    int32_t next = (int32_t)current;

    if (next < (int32_t)MODE_TASK_MOTION_MODE_PRESET_ACTION ||
        next > (int32_t)MODE_TASK_MOTION_MODE_RC_IK)
    {
        next = (int32_t)MODE_TASK_MOTION_MODE_PRESET_ACTION;
    }

    next += delta;
    if (next > (int32_t)MODE_TASK_MOTION_MODE_RC_IK)
    {
        next = (int32_t)MODE_TASK_MOTION_MODE_PRESET_ACTION;
    }
    else if (next < (int32_t)MODE_TASK_MOTION_MODE_PRESET_ACTION)
    {
        next = (int32_t)MODE_TASK_MOTION_MODE_RC_IK;
    }

    return (uint8_t)next;
}

static void mode_task_update_preset_action_runtime(
    ModeTaskContext* context,
    const ModeTaskRcSnapshot* rc_snapshot)
{
    const ModeTaskOperationalPhaseState phase =
        (ModeTaskOperationalPhaseState)sm_get_current(&context->operational_phase_machine);
    const ModeTaskMotionModeId motion_mode =
        (ModeTaskMotionModeId)sm_get_current(&context->motion_mode_machine);

    if (context == OM_NULL || rc_snapshot == OM_NULL)
    {
        return;
    }

    if (phase != MODE_TASK_OPERATIONAL_PHASE_FORMAL_CONTROL ||
        motion_mode != MODE_TASK_MOTION_MODE_PRESET_ACTION ||
        mode_task_is_iw_up_edge(context, rc_snapshot) != OM_TRUE)
    {
        return;
    }

    if (rc_snapshot->sw1 == RC_SWITCH_MI)
    {
        context->preset_action_runtime.chassis_mode = MODE_CHASSIS_EXCHANGE;
        context->preset_action_runtime.clamp_action = MODE_CLAMP_UN_CMD;
        context->preset_action_runtime.exchange_action = MODE_EXCHANGE_UN_CMD;
        context->preset_action_runtime.primary_turn_ore_flag = 0u;
    }
    else if (rc_snapshot->sw1 == RC_SWITCH_DN)
    {
        context->preset_action_runtime.chassis_mode =
            (context->preset_action_runtime.chassis_mode == MODE_CHASSIS_GET_ENERGY_UNIT1)
                ? MODE_CHASSIS_GET_ENERGY_UNIT
                : MODE_CHASSIS_GET_ENERGY_UNIT1;
        context->preset_action_runtime.clamp_action = MODE_CLAMP_UN_CMD;
        context->preset_action_runtime.exchange_action = MODE_EXCHANGE_UN_CMD;
        context->preset_action_runtime.primary_turn_ore_flag = 0u;
    }
}

void mode_task_update_phase_state_from_rc(
    ModeTaskContext* context,
    const ModeTaskRcSnapshot* rc_snapshot)
{
    const OmBool operational_active = mct_is_operational_active();
    const ModeTaskOperationalPhaseState current_phase =
        (ModeTaskOperationalPhaseState)sm_get_current(&context->operational_phase_machine);

    if (context == OM_NULL || rc_snapshot == OM_NULL)
    {
        return;
    }

    if (mode_task_is_sw2_to_dn_edge(context, rc_snapshot) == OM_TRUE)
    {
        (void)sm_force_transition(
            &context->operational_phase_machine,
            (StateId)MODE_TASK_OPERATIONAL_PHASE_RELEASE);
        sh_clear_custom_controller_calibration_indicator();
        return;
    }

    if (mode_task_is_sw2_to_up_from_mi_edge(context, rc_snapshot) == OM_TRUE &&
        operational_active == OM_TRUE)
    {
        (void)sm_force_transition(
            &context->operational_phase_machine,
            (StateId)MODE_TASK_OPERATIONAL_PHASE_MODE_SELECTION);
        return;
    }

    if (current_phase == MODE_TASK_OPERATIONAL_PHASE_MODE_SELECTION)
    {
        if (mode_task_is_sw1_to_up_edge(context, rc_snapshot) == OM_TRUE)
        {
            context->mode_selection_runtime.selected_motion_mode_id =
                mode_task_cycle_motion_mode(
                    context->mode_selection_runtime.selected_motion_mode_id,
                    1);
        }
        else if (mode_task_is_sw1_to_dn_edge(context, rc_snapshot) == OM_TRUE)
        {
            context->mode_selection_runtime.selected_motion_mode_id =
                mode_task_cycle_motion_mode(
                    context->mode_selection_runtime.selected_motion_mode_id,
                    -1);
        }

        if (mode_task_is_sw2_to_mi_from_up_edge(context, rc_snapshot) == OM_TRUE)
        {
            (void)sm_force_transition(
                &context->operational_phase_machine,
                (StateId)MODE_TASK_OPERATIONAL_PHASE_FORMAL_CONTROL);
            sh_clear_custom_controller_calibration_indicator();
        }
    }
}

static void mode_task_update_grip_runtime(
    ModeTaskContext* context,
    const ModeTaskRcSnapshot* rc_snapshot)
{
    const ModeTaskOperationalPhaseState phase =
        (ModeTaskOperationalPhaseState)sm_get_current(&context->operational_phase_machine);

    if (context == OM_NULL || rc_snapshot == OM_NULL)
    {
        return;
    }

    if (phase != MODE_TASK_OPERATIONAL_PHASE_FORMAL_CONTROL)
    {
        return;
    }

    if (mode_task_is_iw_dn_edge(context, rc_snapshot) == OM_TRUE)
    {
        context->grip_runtime.grip_state =
            (context->grip_runtime.grip_state == MODE_TASK_GRIP_OPEN)
                ? MODE_TASK_GRIP_CLOSED
                : MODE_TASK_GRIP_OPEN;
    }
}

static void mode_task_update_rc_ik_runtime(
    ModeTaskContext* context,
    const ModeTaskRcSnapshot* rc_snapshot)
{
    const ModeTaskOperationalPhaseState phase =
        (ModeTaskOperationalPhaseState)sm_get_current(&context->operational_phase_machine);
    const ModeTaskMotionModeId motion_mode =
        (ModeTaskMotionModeId)sm_get_current(&context->motion_mode_machine);

    if (context == OM_NULL || rc_snapshot == OM_NULL)
    {
        return;
    }

    if (phase != MODE_TASK_OPERATIONAL_PHASE_FORMAL_CONTROL ||
        motion_mode != MODE_TASK_MOTION_MODE_RC_IK)
    {
        return;
    }

    if (mode_task_is_sw1_to_up_edge(context, rc_snapshot) == OM_TRUE)
    {
        context->rc_ik_runtime.ik_solver_mode =
            (context->rc_ik_runtime.ik_solver_mode == MODE_TASK_IK_SOLVER_FULL_POSE)
                ? MODE_TASK_IK_SOLVER_POSITION_PRIORITY
                : MODE_TASK_IK_SOLVER_FULL_POSE;

        if (context->rc_ik_runtime.ik_solver_mode ==
            MODE_TASK_IK_SOLVER_POSITION_PRIORITY)
        {
            context->rc_ik_runtime.ik_control_bank =
                MODE_TASK_IK_CONTROL_BANK_POSITION_XYZ;
        }
    }
    else if (mode_task_is_sw1_to_dn_edge(context, rc_snapshot) == OM_TRUE &&
             context->rc_ik_runtime.ik_solver_mode ==
                 MODE_TASK_IK_SOLVER_FULL_POSE)
    {
        context->rc_ik_runtime.ik_control_bank =
            (context->rc_ik_runtime.ik_control_bank ==
             MODE_TASK_IK_CONTROL_BANK_POSITION_XYZ)
                ? MODE_TASK_IK_CONTROL_BANK_ORIENTATION_RPY
                : MODE_TASK_IK_CONTROL_BANK_POSITION_XYZ;
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
}

void mode_task_run_once(ModeTaskContext* context)
{
    ModeTaskRcSnapshot rc_snapshot = {0};
    InputCustomControllerSnapshot custom_controller_snapshot = {0};
    ModeTaskSystemSnapshot previous_system_snapshot = {0};
    ArmTaskModeSnapshot previous_arm_mode_snapshot = {0};
    ChassisTaskModeSnapshot previous_chassis_mode_snapshot = {0};
    ModeTaskSystemSnapshot next_system_snapshot = {0};
    ArmTaskModeSnapshot next_arm_mode_snapshot = {0};
    ChassisTaskModeSnapshot next_chassis_mode_snapshot = {0};
    OmBool system_snapshot_changed = OM_FALSE;
    OmBool arm_mode_snapshot_changed = OM_FALSE;
    OmBool chassis_mode_snapshot_changed = OM_FALSE;

    if (context == OM_NULL)
    {
        return;
    }

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

    mode_task_build_system_snapshot(context, &previous_system_snapshot);
    mode_task_build_arm_mode_snapshot(context, &previous_arm_mode_snapshot);
    mode_task_build_chassis_mode_snapshot(context, &previous_chassis_mode_snapshot);

    mode_task_drain_init_progress_messages(context);
    mode_task_update_bootstrap_state_from_progress(context);
    mode_task_process_mct_lifecycle_requests(context, &rc_snapshot);

    if (mode_task_bootstrap_allows_control(context) == OM_TRUE)
    {
        mode_task_update_phase_state_from_rc(context, &rc_snapshot);
    }

    if ((ModeTaskOperationalPhaseState)sm_get_current(&context->operational_phase_machine) ==
        MODE_TASK_OPERATIONAL_PHASE_FORMAL_CONTROL)
    {
        (void)sm_force_transition(
            &context->motion_mode_machine,
            (StateId)context->confirmed_motion_mode_id);
    }

    mode_task_update_preset_action_runtime(context, &rc_snapshot);
    mode_task_update_rc_ik_runtime(context, &rc_snapshot);
    mode_task_update_grip_runtime(context, &rc_snapshot);
    mode_task_update_operational_system_state(
        context,
        &rc_snapshot,
        &custom_controller_snapshot);

    if (context->hierarchy_state.system_state != MODE_TASK_SYSTEM_OPERATIONAL &&
        (ModeTaskOperationalPhaseState)sm_get_current(&context->operational_phase_machine) !=
            MODE_TASK_OPERATIONAL_PHASE_RELEASE)
    {
        (void)sm_force_transition(
            &context->operational_phase_machine,
            (StateId)MODE_TASK_OPERATIONAL_PHASE_RELEASE);
    }

    mode_task_update_operational_domain(
        context,
        &rc_snapshot,
        &custom_controller_snapshot);
    mode_task_refresh_output_snapshots(context);

    mode_task_build_system_snapshot(context, &next_system_snapshot);
    mode_task_build_arm_mode_snapshot(context, &next_arm_mode_snapshot);
    mode_task_build_chassis_mode_snapshot(context, &next_chassis_mode_snapshot);

    system_snapshot_changed = mode_task_system_snapshot_changed(
        &previous_system_snapshot,
        &next_system_snapshot);
    arm_mode_snapshot_changed = mode_task_arm_mode_snapshot_changed(
        &previous_arm_mode_snapshot,
        &next_arm_mode_snapshot);
    chassis_mode_snapshot_changed = mode_task_chassis_mode_snapshot_changed(
        &previous_chassis_mode_snapshot,
        &next_chassis_mode_snapshot);

    if (arm_mode_snapshot_changed == OM_TRUE ||
        chassis_mode_snapshot_changed == OM_TRUE)
    {
        mode_task_publish_control_snapshots(
            &next_arm_mode_snapshot,
            &next_chassis_mode_snapshot);
    }

    mode_task_sync_context_history(context, &rc_snapshot);
    mode_task_update_debug_state(context);

    if (system_snapshot_changed == OM_TRUE ||
        arm_mode_snapshot_changed == OM_TRUE ||
        chassis_mode_snapshot_changed == OM_TRUE)
    {
        g_mode_task_debug.publish_count++;
    }
}
