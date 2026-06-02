#include "core/om_cpu.h"
#include "bsp/bsp_init.h"
#include "driver/imu/imu.h"
#include "config/app_config.h"
#include "module/data_pool/data_pool.h"
#include "module/system_health/system_health.h"
#include "task/arm_task/arm_task.h"
#include "task/chassis_task/chassis_task.h"
#include "task/input_task/input_task.h"
#include "task/imu_task/imu_task.h"
#include "task/mode_task/mode_task.h"
#include "task/motor_communications_task/mct.h"
#include "task/vofa_task/vofa_task.h"
#include "osal/osal.h"
#include "FreeRTOS.h"
#include "task.h"

DataPool g_data_pool = {0};
volatile const char* g_rtos_fault_task_name = OM_NULL;
volatile uint32_t g_rtos_fault_type = 0u;

static void start_task(void* arg);

void vApplicationStackOverflowHook(TaskHandle_t xTask, char* pcTaskName)
{
    (void)xTask;
    g_rtos_fault_type = 1u;
    g_rtos_fault_task_name = pcTaskName;
    taskDISABLE_INTERRUPTS();
    for (;;)
    {
    }
}

void vApplicationMallocFailedHook(void)
{
    g_rtos_fault_type = 2u;
    g_rtos_fault_task_name = "malloc_failed";
    taskDISABLE_INTERRUPTS();
    for (;;)
    {
    }
}

int main(void)
{
    om_board_init();
    om_core_init();

    OsalThread* start_thread_handle = NULL;
    const OsalThreadAttr start_thread_attr = {"start_thread", 0U, 1U};
    OsalStatus status = OSAL_INVALID;

    status = osal_thread_create(&start_thread_handle, &start_thread_attr, start_task, NULL);
    if (status != OSAL_OK)
    {
        return -1;
    }

    status = osal_kernel_start();
    if (status != OSAL_OK)
    {
        return -2;
    }

    while (1)
    {
    }
}

static void start_task(void* arg)
{
    OmRet init_ret = OM_OK;
    OmRet input_task_ret = OM_OK;
    OmRet imu_task_ret = OM_OK;
    OmRet mode_task_ret = OM_OK;
    OmRet chassis_task_ret = OM_OK;
    OmRet arm_task_ret = OM_OK;
    OmRet motor_communications_task_ret = OM_OK;
    OmRet vofa_task_ret = OM_OK;
    uint8_t imu_init_ret = 0U;
    const BspDeviceRegistry* devices = OM_NULL;

    (void)arg;
    (void)sh_init();

    init_ret = bsp_register_all();
    if (init_ret != OM_OK)
    {
        sh_report_fatal(SH_ERR_BSP_INIT_FAIL, "bsp_register_all failed");
        goto supervisor_loop;
    }

    devices = bsp_get_device_registry();

    mode_task_ret = mode_task_start();
    if (mode_task_ret != OM_OK)
    {
        sh_report_fatal(SH_ERR_MODE_TASK_START_FAIL, "mode_task_start failed");
        goto supervisor_loop;
    }

    // imu_init_ret = mpu_device_init(9.8f);
    // if (imu_init_ret != 0U)
    // {
    //     sh_report_fatal(SH_ERR_MPU_DEVICE_INIT_FAIL, "mpu_device_init failed");
    //     goto supervisor_loop;
    // }

    imu_task_ret = imu_task_start();
    if (imu_task_ret != OM_OK)
    {
        sh_report_fatal(SH_ERR_IMU_TASK_START_FAIL, "imu_task_start failed");
        goto supervisor_loop;
    }

    input_task_ret = input_task_start(devices);
    if (input_task_ret != OM_OK)
    {
        sh_report_fatal(SH_ERR_INPUT_TASK_START_FAIL, "input_task_start failed");
        goto supervisor_loop;
    }

    motor_communications_task_ret = mct_start(devices);
    if (motor_communications_task_ret != OM_OK)
    {
        sh_report_fatal(
            SH_ERR_MOTOR_COMMUNICATIONS_TASK_START_FAIL,
            "motor_communications_task_start failed");
        goto supervisor_loop;
    }

    if (sh_register(
            SH_TASK_MOTOR_COMMUNICATIONS,
            50u,
            SH_ERR_MOTOR_COMMUNICATIONS_TIMEOUT) != OM_OK)
    {
        sh_report_fatal(
            SH_ERR_SYSTEM_HEALTH_REGISTER_FAIL,
            "sh_register MOTOR_COMMUNICATIONS failed");
        goto supervisor_loop;
    }

    chassis_task_ret = chassis_task_start();
    if (chassis_task_ret != OM_OK)
    {
        sh_report_fatal(
            SH_ERR_CHASSIS_TASK_START_FAIL,
            "chassis_task_start failed");
        goto supervisor_loop;
    }

    if (sh_register(
            SH_TASK_CHASSIS,
            50u,
            SH_ERR_CHASSIS_TASK_TIMEOUT) != OM_OK)
    {
        sh_report_fatal(
            SH_ERR_SYSTEM_HEALTH_REGISTER_FAIL,
            "sh_register CHASSIS failed");
        goto supervisor_loop;
    }

    arm_task_ret = arm_task_start();
    if (arm_task_ret != OM_OK)
    {
        sh_report_fatal(
            SH_ERR_ARM_TASK_START_FAIL,
            "arm_task_start failed");
        goto supervisor_loop;
    }

    if (sh_register(
            SH_TASK_ARM,
            50u,
            SH_ERR_ARM_TASK_TIMEOUT) != OM_OK)
    {
        sh_report_fatal(
            SH_ERR_SYSTEM_HEALTH_REGISTER_FAIL,
            "sh_register ARM failed");
        goto supervisor_loop;
    }

    vofa_task_ret = vofa_task_start(devices);
    if (vofa_task_ret != OM_OK)
    {
        sh_report_fatal(SH_ERR_VOFA_TASK_START_FAIL, "vofa_task_start failed");
        goto supervisor_loop;
    }

    sh_set_running();

supervisor_loop:
    while (1)
    {
        sh_poll();
        osal_sleep_ms(20U);
    }
}
