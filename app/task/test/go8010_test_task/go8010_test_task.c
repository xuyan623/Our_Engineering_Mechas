#include "task/test/go8010_test_task/go8010_test_task.h"

#include "driver/go8010/go8010.h"
#include "module/system_health/system_health.h"
#include "osal/osal.h"
#include "osal/osal_config.h"
#include "osal/osal_time.h"
#include <string.h>

#define GO8010_TEST_TASK_LOOP_PERIOD_MS     (5u)
#define GO8010_TEST_RETRY_DELAY_MS          (50u)
#define GO8010_TEST_MOTOR_ID                (1u)
#define GO8010_TEST_TARGET_TORQUE_NM        (0.0f)
#define GO8010_TEST_TARGET_POSITION_RAD     (0.0f)
#define GO8010_TEST_TARGET_SPEED_RAD_S      (0.0f)
#define GO8010_TEST_TARGET_KP               (0.0f)
#define GO8010_TEST_TARGET_KD               (0.0f)
#define GO8010_TEST_ONLINE_TIMEOUT_MS       (100u)

typedef struct
{
    Go8010Bus bus;
    Go8010MotorDrv motor;
    OmBool zero_offset_initialized_flag;
    float zero_offset_rad;
    uint32_t last_feedback_sequence;
} Go8010TestRuntime;

static Go8010TestRuntime g_go8010_test_runtime = {0};
Go8010TestDebugState g_go8010_test_debug = {0};

static void go8010_test_apply_idle_target(Go8010TestRuntime* runtime)
{
    if (runtime == OM_NULL)
    {
        return;
    }

    /* 这里只发“零力矩观测”命令：
     * - torque = 0
     * - kp/kd = 0
     * 不给电机任何位置保持力，避免测试任务主动驱动电机运动。
     */
    g_go8010_test_debug.target_torque_nm = GO8010_TEST_TARGET_TORQUE_NM;
    g_go8010_test_debug.target_position_rad = GO8010_TEST_TARGET_POSITION_RAD;
    g_go8010_test_debug.target_speed_rad_s = GO8010_TEST_TARGET_SPEED_RAD_S;

    go8010_set_target(
        &runtime->motor,
        GO8010_TEST_TARGET_TORQUE_NM,
        GO8010_TEST_TARGET_POSITION_RAD,
        GO8010_TEST_TARGET_SPEED_RAD_S,
        GO8010_TEST_TARGET_KP,
        GO8010_TEST_TARGET_KD);
}

static void go8010_test_refresh_feedback(Go8010TestRuntime* runtime)
{
    const Go8010Feedback* feedback = OM_NULL;

    if (runtime == OM_NULL)
    {
        return;
    }

    feedback = go8010_get_feedback(&runtime->motor);
    if (feedback == OM_NULL)
    {
        return;
    }

    if (feedback->sequence != runtime->last_feedback_sequence)
    {
        runtime->last_feedback_sequence = feedback->sequence;
        g_go8010_test_debug.feedback_seen_count = feedback->sequence;

        if (runtime->zero_offset_initialized_flag != OM_TRUE)
        {
            runtime->zero_offset_rad = feedback->position;
            runtime->zero_offset_initialized_flag = OM_TRUE;
        }
    }

    g_go8010_test_debug.mode = feedback->mode;
    g_go8010_test_debug.position_rad = feedback->position;
    g_go8010_test_debug.speed_rad_s = feedback->speed;
    g_go8010_test_debug.torque_nm = feedback->torque;

    if (runtime->zero_offset_initialized_flag == OM_TRUE)
    {
        g_go8010_test_debug.relative_position_rad =
            feedback->position - runtime->zero_offset_rad;
    }
}

static void go8010_test_update_online_flag(Go8010TestRuntime* runtime)
{
    uint32_t now_ms = 0u;
    uint32_t age_ms = 0u;

    if (runtime == OM_NULL)
    {
        return;
    }

    now_ms = osal_time_now_monotonic();
    if (runtime->motor.feedback.timestampMs > 0u)
    {
        age_ms = now_ms - runtime->motor.feedback.timestampMs;
    }

    g_go8010_test_debug.last_rx_age_ms = age_ms;
    g_go8010_test_debug.online_flag =
        (go8010_is_online(&runtime->motor, GO8010_TEST_ONLINE_TIMEOUT_MS) == OM_TRUE) ? 1u : 0u;
}

static OmRet go8010_test_runtime_init(Go8010TestRuntime* runtime, const BspDeviceRegistry* devices)
{
    OmRet ret = OM_OK;

    if (runtime == OM_NULL || devices == OM_NULL || devices->usart6 == OM_NULL)
    {
        return OM_ERROR_NULL;
    }

    memset(runtime, 0, sizeof(*runtime));

    ret = go8010_init(&runtime->bus, devices->usart6);
    if (ret != OM_OK)
    {
        return ret;
    }

    ret = go8010_register(&runtime->bus, &runtime->motor, GO8010_TEST_MOTOR_ID);
    if (ret != OM_OK)
    {
        return ret;
    }

    g_go8010_test_debug.mode = runtime->motor.mode;
    return OM_OK;
}

static void go8010_test_task_entry(void* arg)
{
    Go8010TestRuntime* runtime = (Go8010TestRuntime*)arg;
    OsalTimeMs deadline_cursor_ms = 0u;

    if (runtime == OM_NULL)
    {
        for (;;)
        {
            (void)osal_sleep_ms(1000u);
        }
    }

    while (1)
    {
        go8010_test_apply_idle_target(runtime);
        go8010_tx_service(&runtime->bus);
        go8010_rx_service(&runtime->bus);
        go8010_test_refresh_feedback(runtime);
        go8010_test_update_online_flag(runtime);
        (void)sh_beat(SH_TASK_GO8010_SMOKE);

        if (g_go8010_test_debug.online_flag == 0u &&
            g_go8010_test_debug.feedback_seen_count == 0u)
        {
            (void)osal_sleep_ms(GO8010_TEST_RETRY_DELAY_MS);
            continue;
        }

        (void)osal_delay_until(&deadline_cursor_ms, GO8010_TEST_TASK_LOOP_PERIOD_MS, OM_NULL);
    }
}

OmRet go8010_test_task_start(const BspDeviceRegistry* devices)
{
    static OsalThread* go8010_test_task = OM_NULL;
    const OsalThreadAttr go8010_test_attr = {"go8010_test", 768u * OSAL_STACK_WORD_BYTES, 4u};
    OsalStatus status = OSAL_INVALID;
    OmRet ret = OM_OK;

    if (devices == OM_NULL)
    {
        return OM_ERROR_NULL;
    }

    if (go8010_test_task != OM_NULL)
    {
        return OM_ERR_CONFLICT;
    }

    memset(&g_go8010_test_debug, 0, sizeof(g_go8010_test_debug));

    ret = go8010_test_runtime_init(&g_go8010_test_runtime, devices);
    g_go8010_test_debug.init_ret = (int32_t)ret;
    if (ret != OM_OK)
    {
        return ret;
    }

    status = osal_thread_create(
        &go8010_test_task,
        &go8010_test_attr,
        go8010_test_task_entry,
        &g_go8010_test_runtime);
    if (status != OSAL_OK)
    {
        go8010_test_task = OM_NULL;
        return OM_ERROR;
    }

    return OM_OK;
}
