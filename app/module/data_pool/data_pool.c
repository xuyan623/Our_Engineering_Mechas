#include "module/data_pool/data_pool.h"

#include "FreeRTOS.h"
#include "task.h"
#include <string.h>

static void dp_copy_bytes(void* destination, const void* source, uint32_t size_bytes)
{
    if (destination == NULL || source == NULL || size_bytes == 0u)
    {
        return;
    }

    taskENTER_CRITICAL();
    memcpy(destination, source, (size_t)size_bytes);
    taskEXIT_CRITICAL();
}

static void dp_store_bytes(void* destination, const void* source, uint32_t size_bytes)
{
    if (destination == NULL || source == NULL || size_bytes == 0u)
    {
        return;
    }

    taskENTER_CRITICAL();
    memcpy(destination, source, (size_t)size_bytes);
    taskEXIT_CRITICAL();
}

void dp_copy_imu_snapshot(DpImuSnapshot* snapshot)
{
    dp_copy_bytes(snapshot, &g_data_pool.imu, sizeof(g_data_pool.imu));
}

void dp_store_imu_snapshot(const DpImuSnapshot* snapshot)
{
    dp_store_bytes(&g_data_pool.imu, snapshot, sizeof(g_data_pool.imu));
}

void dp_copy_rc_snapshot(DpRcSnapshot* snapshot)
{
    dp_copy_bytes(snapshot, &g_data_pool.rc, sizeof(g_data_pool.rc));
}

void dp_store_rc_snapshot(const DpRcSnapshot* snapshot)
{
    dp_store_bytes(&g_data_pool.rc, snapshot, sizeof(g_data_pool.rc));
}

void dp_copy_mode_compat_snapshot(DpModeCompatSnapshot* snapshot)
{
    if (snapshot == NULL)
    {
        return;
    }

    taskENTER_CRITICAL();
    snapshot->global_mode = g_data_pool.mode.global_mode;
    snapshot->chassis_mode = g_data_pool.mode.chassis_mode;
    snapshot->clamp_action = g_data_pool.action.clamp_action;
    snapshot->exchange_action = g_data_pool.action.exchange_action;
    snapshot->primary_turn_ore_flag = g_data_pool.action.primary_turn_ore_flag;
    snapshot->custom_controller_force_takeover_flag =
        g_data_pool.action.custom_controller_force_takeover_flag;
    taskEXIT_CRITICAL();
}

void dp_store_mode_compat_snapshot(const DpModeCompatSnapshot* snapshot)
{
    if (snapshot == NULL)
    {
        return;
    }

    taskENTER_CRITICAL();
    g_data_pool.mode.global_mode = snapshot->global_mode;
    g_data_pool.mode.chassis_mode = snapshot->chassis_mode;
    g_data_pool.action.clamp_action = snapshot->clamp_action;
    g_data_pool.action.exchange_action = snapshot->exchange_action;
    g_data_pool.action.primary_turn_ore_flag = snapshot->primary_turn_ore_flag;
    g_data_pool.action.custom_controller_force_takeover_flag =
        snapshot->custom_controller_force_takeover_flag;
    taskEXIT_CRITICAL();
}

void dp_copy_custom_controller_snapshot(DpCustomControllerSnapshot* snapshot)
{
    dp_copy_bytes(snapshot, &g_data_pool.custom_controller, sizeof(g_data_pool.custom_controller));
}

void dp_store_custom_controller_snapshot(const DpCustomControllerSnapshot* snapshot)
{
    dp_store_bytes(&g_data_pool.custom_controller, snapshot, sizeof(g_data_pool.custom_controller));
}
