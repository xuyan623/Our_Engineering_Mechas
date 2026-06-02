#include "task/imu_task/imu_task.h"

#include "core/om_cpu.h"
#include "driver/imu/imu.h"
#include "module/data_pool/data_pool.h"
#include "module/system_health/system_health.h"
#include "osal/osal.h"
#include "osal/osal_config.h"
#include "osal/osal_time.h"
#include "task/chassis_task/chassis_task.h"
#include "task/mode_task/mode_task.h"
#include "FreeRTOS.h"
#include "task.h"

#define IMU_TASK_PERIOD_MS        (10u)
#define IMU_TASK_UPDATE_DT_SEC    (0.01f)
#define IMU_TASK_STACK_BYTES      (512u * OSAL_STACK_WORD_BYTES)

/* 这组计数只用于调试器和 VOFA 观察任务是否按预期工作。 */
ImuTaskDebugState g_imu_task_debug = {0};

static void imu_task_fill_snapshot(
    const imu_data_t* imu_data,
    DpImuSnapshot* snapshot)
{
    if (imu_data == OM_NULL || snapshot == OM_NULL)
    {
        return;
    }

    /* DataPool 暴露给上层的是处理后的物理量，
     * 不回灌驱动层原始寄存器值，也不把驱动内部临时状态提升成共享事实。
     */
    snapshot->yaw = imu_data->yaw;
    snapshot->pitch = imu_data->pit;
    snapshot->roll = imu_data->rol;
    snapshot->yaw_rate = imu_data->yaw_rate;
    snapshot->pitch_rate = imu_data->pit_rate;
    snapshot->roll_rate = imu_data->rol_rate;
    snapshot->wx = imu_data->wx;
    snapshot->wy = imu_data->wy;
    snapshot->wz = imu_data->wz;
    snapshot->ax = imu_data->vx;
    snapshot->ay = imu_data->vy;
    snapshot->az = imu_data->vz;
    snapshot->temp = imu_data->temp;
}

static void imu_task_store_to_data_pool(const imu_data_t* imu_data)
{
    DpImuSnapshot snapshot = {0};

    if (imu_data == OM_NULL)
    {
        return;
    }

    imu_task_fill_snapshot(imu_data, &snapshot);
    dp_store_imu_snapshot(&snapshot);
    (void)chassis_task_submit_imu_snapshot(&snapshot);
}

static void imu_task_entry(void* arg)
{
    OsalTimeMs deadline_cursor_ms = 0u;
    imu_data_t* imu_data = OM_NULL;
    UBaseType_t high_water_words = 0u;

    (void)arg;

    g_imu_task_debug.stack_reserved_bytes = IMU_TASK_STACK_BYTES;

    while (1)
    {
        g_imu_task_debug.loop_count++;
        /* 这里的 mpu_get_data() 已经不再直接读 SPI，
         * 它只消费 imu_bsp 提供的“最近完成的一帧原始样本”。
         */
        mpu_get_data();

        if (imu_has_new_data() == OM_TRUE)
        {
            /* 只有拿到新样本才推进一次姿态融合。
             * 这样可以避免在采样停滞时重复积分旧陀螺数据。
             */
            update_attitude(IMU_TASK_UPDATE_DT_SEC);
            imu_data = get_imu_data();
            imu_task_store_to_data_pool(imu_data);

            g_imu_task_debug.publish_count++;
        }
        else
        {
            /* 本周期没有新采样属于允许状态，
             * 这里只做计数，不做兜底重算，不发布“数据已更新”事件。
             */
            g_imu_task_debug.no_new_sample_count++;
        }

        /* uxTaskGetStackHighWaterMark 返回的是“任务启动以来最小剩余栈空间”，
         * 因此它天然就是峰值占用统计所需的数据源。
         */
        high_water_words = uxTaskGetStackHighWaterMark(NULL);
        g_imu_task_debug.stack_high_water_words = (uint32_t)high_water_words;
        g_imu_task_debug.stack_min_free_bytes = ((uint32_t)high_water_words) * OSAL_STACK_WORD_BYTES;
        if (g_imu_task_debug.stack_reserved_bytes >= g_imu_task_debug.stack_min_free_bytes)
        {
            g_imu_task_debug.stack_peak_used_bytes =
                g_imu_task_debug.stack_reserved_bytes - g_imu_task_debug.stack_min_free_bytes;
        }
        else
        {
            g_imu_task_debug.stack_peak_used_bytes = 0u;
        }

        (void)osal_delay_until(&deadline_cursor_ms, IMU_TASK_PERIOD_MS, OM_NULL);
    }
}

OmRet imu_task_start(void)
{
    static OsalThread* imu_task_thread = OM_NULL;
    const OsalThreadAttr imu_task_attr = {"imu_task", 512u * OSAL_STACK_WORD_BYTES, 4u};
    OsalStatus status = OSAL_INVALID;

    if (imu_task_thread != OM_NULL)
    {
        return OM_ERR_CONFLICT;
    }

    status = osal_thread_create(&imu_task_thread, &imu_task_attr, imu_task_entry, OM_NULL);
    if (status != OSAL_OK)
    {
        imu_task_thread = OM_NULL;
        return OM_ERROR;
    }

    {
        const ModeTaskInitProgressMessage init_progress = {
            .kind = (uint8_t)MODE_TASK_INIT_PROGRESS_IMU_READY,
            .value = 1u};
        (void)mode_task_submit_init_progress(&init_progress);
    }

    return OM_OK;
}
