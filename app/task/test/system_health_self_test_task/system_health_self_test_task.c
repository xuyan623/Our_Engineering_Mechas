#include "task/test/system_health_self_test_task/system_health_self_test_task.h"

#include "config/app_config.h"
#include "module/system_health/system_health.h"
#include "osal/osal.h"
#include "osal/osal_config.h"
#include "osal/osal_time.h"

#define SH_SELF_TEST_TASK_PERIOD_MS (20u)

static void sh_self_test_task_entry(void* arg)
{
    OsalTimeMs deadline_cursor_ms = 0u;
    OsalTimeMs start_time_ms = 0u;
    OsalTimeMs elapsed_ms = 0u;
    OmBool fault_injected = OM_FALSE;

    (void)arg;
    start_time_ms = osal_time_now_monotonic();

    while (1)
    {
        elapsed_ms = (OsalTimeMs)(osal_time_now_monotonic() - start_time_ms);

        if (fault_injected != OM_TRUE && elapsed_ms >= APP_SH_SELF_TEST_DELAY_MS)
        {
#if (APP_SH_SELF_TEST_MODE == APP_SH_SELF_TEST_MODE_RT_TIMEOUT)
            /* 专用自测任务停止心跳，验证 timeout -> runtime fault 链路。 */
            fault_injected = OM_TRUE;
#elif (APP_SH_SELF_TEST_MODE == APP_SH_SELF_TEST_MODE_FATAL)
            sh_report_fatal(SH_ERR_SELF_TEST_FATAL, "system_health self-test fatal injected");
            fault_injected = OM_TRUE;
#else
            return;
#endif
        }

        if (fault_injected != OM_TRUE)
        {
            (void)sh_beat(SH_TASK_SELF_TEST);
            (void)osal_delay_until(&deadline_cursor_ms, SH_SELF_TEST_TASK_PERIOD_MS, OM_NULL);
            continue;
        }

        /* 注错后保持任务存活，但不再恢复心跳。 */
        osal_sleep_ms(1000u);
    }
}

OmRet sh_self_test_task_start(void)
{
#if (APP_SH_SELF_TEST_MODE == APP_SH_SELF_TEST_MODE_OFF)
    return OM_OK;
#else
    static OsalThread* self_test_task_thread = OM_NULL;
    const OsalThreadAttr self_test_task_attr = {"sh_self_test", 512u * OSAL_STACK_WORD_BYTES, 3u};
    OsalStatus status = OSAL_INVALID;

    if (self_test_task_thread != OM_NULL)
    {
        return OM_ERR_CONFLICT;
    }

    status = osal_thread_create(&self_test_task_thread, &self_test_task_attr, sh_self_test_task_entry, OM_NULL);
    if (status != OSAL_OK)
    {
        self_test_task_thread = OM_NULL;
        return OM_ERROR;
    }

    return OM_OK;
#endif
}
