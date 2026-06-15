#include "module/system_health/system_health.h"

#include "bsp/board_led.h"
#include "core/om_cpu.h"
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>

/* 错误显示时序：
 * - 先 8 灯全亮作为一轮循环前导
 * - 再依次显示三位错误码
 * - fatal 显示固定 3 轮后进入最终错误处理
 */
#define SH_PREAMBLE_ON_MS       (600u)
#define SH_DIGIT_ON_MS          (600u)
#define SH_DIGIT_GAP_MS         (300u)
#define SH_CYCLE_GAP_MS         (700u)
#define SH_FATAL_DISPLAY_ROUNDS (3u)
#define SH_RUNNING_GREEN_FLASH_MS (120u)
#define SH_CAL_FAILED_RED_FLASH_MS (120u)
#define SH_CAL_PENDING_BLINK_PERIOD_MS (500u)

/* system_health 的高层状态。
 * - BOOTING：启动期，板载灯关闭
 * - RUNNING：系统健康，单绿灯常亮
 * - ERROR：存在 runtime fault 或 fatal，进入统一错误显示协议
 */
typedef enum
{
    SH_STATE_BOOTING = 0u,
    SH_STATE_RUNNING,
    SH_STATE_ERROR,
} SHState;

typedef enum
{
    SH_CUSTOM_CONTROLLER_CAL_IDLE = 0u,
    SH_CUSTOM_CONTROLLER_CAL_PENDING,
    SH_CUSTOM_CONTROLLER_CAL_FAILED,
} SHCustomControllerCalibrationState;

/* 统一错误显示状态机的内部阶段。 */
typedef enum
{
    SH_DISPLAY_PHASE_IDLE = 0u,
    SH_DISPLAY_PHASE_PREAMBLE_ON,
    SH_DISPLAY_PHASE_DIGIT_ON,
    SH_DISPLAY_PHASE_DIGIT_GAP,
    SH_DISPLAY_PHASE_CYCLE_GAP,
} SHDisplayPhase;

/* 由错误码解出来的“三段报码”。 */
typedef struct
{
    uint8_t phase1_count;
    uint8_t phase2_count;
    uint8_t phase3_count;
} SHBlinkPattern;

/* 被监督任务的运行时条目。
 * 每个条目只负责：
 * - 是否注册
 * - timeout 阈值
 * - 最近一次心跳时刻
 * - 若超时，应映射成哪个 runtime fault code
 */
typedef struct
{
    OmBool registered_flag;
    OsalTimeMs timeout_ms;
    volatile OsalTimeMs last_beat_ms;
    SHErrorCode fault_code;
} SHEntry;

/* fatal 锁存槽。
 * 采用 first fatal wins：
 * - 第一个 fatal 进入后锁存
 * - 后续 fatal 上报一律忽略
 */
typedef struct
{
    OmBool latched_flag;
    SHErrorCode code;
    char* msg;
} SHFatalLatch;

/* system_health 全局上下文。
 * 这是当前项目唯一的健康监督事实，不并入 mode_task，也不混入控制快照。
 */
typedef struct
{
    SHState state;
    SHEntry entries[SH_TASK_COUNT];
    OmBool runtime_fault_active[SH_ERR_RUNTIME_FAULT_MAX + 1u];
    SHErrorCode active_display_code;
    SHFatalLatch fatal;
    SHBlinkPattern active_pattern;
    SHDisplayPhase display_phase;
    OsalTimeMs next_transition_ms;
    OsalTimeMs running_green_flash_until_ms;
    OsalTimeMs cal_failed_red_flash_until_ms;
    SHCustomControllerCalibrationState custom_controller_calibration_state;
    uint8_t current_digit_index;
    uint32_t fatal_completed_display_rounds;
} SHContext;

static SHContext g_system_health = {0};

/* BOOTING 状态下，所有 system_health 相关灯都灭。 */
static void sh_set_booting_led(void)
{
    board_led_set_red(OM_FALSE);
    board_led_set_green(OM_FALSE);
    board_led_set_user8_all(OM_FALSE);
}

/* 把错误码直接反解成三段报码。
 * 当前方案里没有单独的 pattern 表，错误码本身就是模式。
 */
static SHBlinkPattern sh_decode_pattern(SHErrorCode code)
{
    if (code == SH_ERR_NONE)
    {
        return (SHBlinkPattern){8u, 8u, 8u};
    }

    return (SHBlinkPattern){
        .phase1_count = (uint8_t)SH_CODE_P1(code),
        .phase2_count = (uint8_t)SH_CODE_P2(code),
        .phase3_count = (uint8_t)SH_CODE_P3(code),
    };
}

/* 多个错误同时存在时的优先级比较。
 * 规则固定为三码从前到后逐段比较，数字小的优先。
 */
static OmBool sh_error_precedes(SHErrorCode lhs, SHErrorCode rhs)
{
    if (rhs == SH_ERR_NONE)
        return OM_TRUE;
    if (lhs == SH_ERR_NONE)
        return OM_FALSE;

    if (SH_CODE_P1(lhs) != SH_CODE_P1(rhs))
        return (SH_CODE_P1(lhs) < SH_CODE_P1(rhs)) ? OM_TRUE : OM_FALSE;
    if (SH_CODE_P2(lhs) != SH_CODE_P2(rhs))
        return (SH_CODE_P2(lhs) < SH_CODE_P2(rhs)) ? OM_TRUE : OM_FALSE;
    return (SH_CODE_P3(lhs) < SH_CODE_P3(rhs)) ? OM_TRUE : OM_FALSE;
}

/* 根据当前位索引，取出该位需要显示的数字。 */
static uint8_t sh_get_digit_value(const SHBlinkPattern* pattern, uint8_t digit_index)
{
    switch (digit_index)
    {
    case 0u:
        return pattern->phase1_count;
    case 1u:
        return pattern->phase2_count;
    case 2u:
        return pattern->phase3_count;
    default:
        return 0u;
    }
}

/* 统一错误显示中，红灯常亮、PF14 绿灯熄灭。 */
static void sh_set_error_led_base(void)
{
    board_led_set_green(OM_FALSE);
    board_led_set_red(OM_TRUE);
}

/* 把“当前是哪一位 + 当前位的数值”编码到 PG1-PG8：
 * - PG1-PG4: 当前数字的 4bit 值
 * - PG5-PG7: 第 1/2/3 位标识
 * - PG8: 当前位显示使能
 */
static uint8_t sh_build_digit_mask(const SHBlinkPattern* pattern, uint8_t digit_index)
{
    uint8_t digit_value = sh_get_digit_value(pattern, digit_index);

    return (uint8_t)((digit_value & 0x0Fu) | (1u << (4u + digit_index)) | (1u << 7u));
}

static void sh_show_preamble(void)
{
    sh_set_error_led_base();
    board_led_set_user8_all(OM_TRUE);
}

static void sh_show_digit(const SHBlinkPattern* pattern, uint8_t digit_index)
{
    sh_set_error_led_base();
    board_led_set_user8_mask(sh_build_digit_mask(pattern, digit_index));
}

static void sh_clear_error_detail_leds(void)
{
    sh_set_error_led_base();
    board_led_set_user8_all(OM_FALSE);
}

/* 高层状态到 LED 表现的直接映射。 */
static void sh_apply_led(SHState state)
{
    OsalTimeMs now_ms = osal_time_now_monotonic();

    switch (state)
    {
    case SH_STATE_RUNNING:
        if (g_system_health.custom_controller_calibration_state ==
            SH_CUSTOM_CONTROLLER_CAL_PENDING)
        {
            board_led_set_red(OM_FALSE);
            board_led_set_green(
                ((now_ms / SH_CAL_PENDING_BLINK_PERIOD_MS) % 2u == 0u) ? OM_TRUE : OM_FALSE);
        }
        else if (g_system_health.custom_controller_calibration_state ==
                 SH_CUSTOM_CONTROLLER_CAL_FAILED)
        {
            if (g_system_health.cal_failed_red_flash_until_ms != 0u &&
                osal_time_before(now_ms, g_system_health.cal_failed_red_flash_until_ms))
            {
                board_led_set_red(OM_TRUE);
                board_led_set_green(OM_FALSE);
            }
            else
            {
                g_system_health.cal_failed_red_flash_until_ms = 0u;
                g_system_health.custom_controller_calibration_state =
                    SH_CUSTOM_CONTROLLER_CAL_IDLE;
                board_led_set_running();
            }
        }
        else if (g_system_health.running_green_flash_until_ms != 0u &&
            osal_time_before(now_ms, g_system_health.running_green_flash_until_ms))
        {
            board_led_set_red(OM_FALSE);
            board_led_set_green(OM_FALSE);
        }
        else
        {
            g_system_health.running_green_flash_until_ms = 0u;
            board_led_set_running();
        }
        board_led_set_user8_all(OM_FALSE);
        break;
    case SH_STATE_ERROR:
        /* 错误态的具体数字显示由独立状态机驱动，这里不直接改 PG1-PG8。 */
        break;
    case SH_STATE_BOOTING:
    default:
        sh_set_booting_led();
        break;
    }
}

static void sh_start_error_display(SHErrorCode code, OsalTimeMs now_ms)
{
    g_system_health.active_display_code = code;
    g_system_health.active_pattern = sh_decode_pattern(code);
    g_system_health.display_phase = SH_DISPLAY_PHASE_PREAMBLE_ON;
    g_system_health.current_digit_index = 0u;
    g_system_health.fatal_completed_display_rounds = 0u;
    g_system_health.next_transition_ms = (OsalTimeMs)(now_ms + SH_PREAMBLE_ON_MS);

    sh_show_preamble();
}

/* fatal 最终执行动作。
 * 新协议下 fatal 先完成固定轮数的统一显示，再进入错误处理。
 */
static void sh_enter_fatal(char* msg)
{
    sh_set_error_led_base();
    OM_CPU_ERRHANDLER(msg, OM_LOG_LEVEL_FATAL);
}

/* 统一错误显示状态机。
 * runtime fault 与 fatal 共用同一套 PG1-PG8 显示协议。
 */
static void sh_update_error_display(OsalTimeMs now_ms)
{
    if (g_system_health.state != SH_STATE_ERROR)
        return;

    /* 这里必须用 OSAL 的回绕安全时间比较。
     * 之前直接做无符号减法会在“deadline 还没到”时产生下溢，
     * 结果被误判为已经到时，导致状态机几乎每次 poll 都推进。
     */
    if (osal_time_before(now_ms, g_system_health.next_transition_ms))
        return;

    switch (g_system_health.display_phase)
    {
    case SH_DISPLAY_PHASE_PREAMBLE_ON:
        g_system_health.current_digit_index = 0u;
        sh_show_digit(&g_system_health.active_pattern, g_system_health.current_digit_index);
        g_system_health.display_phase = SH_DISPLAY_PHASE_DIGIT_ON;
        g_system_health.next_transition_ms = (OsalTimeMs)(now_ms + SH_DIGIT_ON_MS);
        break;
    case SH_DISPLAY_PHASE_DIGIT_ON:
        if (g_system_health.current_digit_index < 2u)
        {
            sh_clear_error_detail_leds();
            g_system_health.display_phase = SH_DISPLAY_PHASE_DIGIT_GAP;
            g_system_health.next_transition_ms = (OsalTimeMs)(now_ms + SH_DIGIT_GAP_MS);
            break;
        }

        if (g_system_health.fatal.latched_flag == OM_TRUE)
        {
            g_system_health.fatal_completed_display_rounds++;
            if (g_system_health.fatal_completed_display_rounds >= SH_FATAL_DISPLAY_ROUNDS)
            {
                sh_enter_fatal(g_system_health.fatal.msg);
                return;
            }
        }

        sh_clear_error_detail_leds();
        g_system_health.display_phase = SH_DISPLAY_PHASE_CYCLE_GAP;
        g_system_health.next_transition_ms = (OsalTimeMs)(now_ms + SH_CYCLE_GAP_MS);
        break;
    case SH_DISPLAY_PHASE_DIGIT_GAP:
        g_system_health.current_digit_index++;
        sh_show_digit(&g_system_health.active_pattern, g_system_health.current_digit_index);
        g_system_health.display_phase = SH_DISPLAY_PHASE_DIGIT_ON;
        g_system_health.next_transition_ms = (OsalTimeMs)(now_ms + SH_DIGIT_ON_MS);
        break;
    case SH_DISPLAY_PHASE_CYCLE_GAP:
        g_system_health.current_digit_index = 0u;
        sh_show_preamble();
        g_system_health.display_phase = SH_DISPLAY_PHASE_PREAMBLE_ON;
        g_system_health.next_transition_ms = (OsalTimeMs)(now_ms + SH_PREAMBLE_ON_MS);
        break;
    case SH_DISPLAY_PHASE_IDLE:
    default:
        sh_start_error_display(g_system_health.active_display_code, now_ms);
        break;
    }
}

OmRet sh_init(void)
{
    /* 初始化唯一健康上下文，并把灯切到 BOOTING 默认态。 */
    memset(&g_system_health, 0, sizeof(g_system_health));
    (void)board_led_init();
    g_system_health.state = SH_STATE_BOOTING;
    g_system_health.active_display_code = SH_ERR_NONE;
    g_system_health.display_phase = SH_DISPLAY_PHASE_IDLE;
    sh_apply_led(g_system_health.state);
    return OM_OK;
}

OmRet sh_register(SHTaskId task_id, OsalTimeMs timeout_ms, SHErrorCode fault_code)
{
    if (task_id >= SH_TASK_COUNT || timeout_ms == 0u || fault_code == SH_ERR_NONE)
    {
        return OM_ERROR_PARAM;
    }

    g_system_health.entries[task_id].registered_flag = OM_TRUE;
    g_system_health.entries[task_id].timeout_ms = timeout_ms;
    g_system_health.entries[task_id].last_beat_ms = osal_time_now_monotonic();
    g_system_health.entries[task_id].fault_code = fault_code;
    g_system_health.runtime_fault_active[fault_code] = OM_FALSE;
    return OM_OK;
}

OmRet sh_beat(SHTaskId task_id)
{
    if (task_id >= SH_TASK_COUNT)
    {
        return OM_ERROR_PARAM;
    }
    if (g_system_health.entries[task_id].registered_flag != OM_TRUE)
    {
        return OM_ERROR_PARAM;
    }

    g_system_health.entries[task_id].last_beat_ms = osal_time_now_monotonic();
    return OM_OK;
}

/* 主动上报一个运行时故障。
 * 当前项目主要还是依靠心跳超时自动置位，但这里保留主动上报能力。
 */
OmRet sh_report_runtime_fault(SHErrorCode code)
{
    if (code == SH_ERR_NONE || code > SH_ERR_RUNTIME_FAULT_MAX)
    {
        return OM_ERROR_PARAM;
    }

    taskENTER_CRITICAL();
    g_system_health.runtime_fault_active[code] = OM_TRUE;
    taskEXIT_CRITICAL();
    return OM_OK;
}

/* 清除一个主动上报的运行时故障。 */
OmRet sh_clear_runtime_fault(SHErrorCode code)
{
    if (code == SH_ERR_NONE || code > SH_ERR_RUNTIME_FAULT_MAX)
    {
        return OM_ERROR_PARAM;
    }

    taskENTER_CRITICAL();
    g_system_health.runtime_fault_active[code] = OM_FALSE;
    taskEXIT_CRITICAL();
    return OM_OK;
}

void sh_report_fatal(SHErrorCode code, char* msg)
{
    /* first fatal wins：
     * 一旦已有 fatal 锁存，后续 fatal 一律忽略，不允许闪灯码切换。
     */
    taskENTER_CRITICAL();
    if (g_system_health.fatal.latched_flag == OM_TRUE)
    {
        taskEXIT_CRITICAL();
        return;
    }

    g_system_health.fatal.latched_flag = OM_TRUE;
    g_system_health.fatal.code = code;
    g_system_health.fatal.msg = msg;
    taskEXIT_CRITICAL();
}

void sh_set_running(void)
{
    /* 切回 RUNNING 时，错误显示上下文一并清掉。 */
    g_system_health.state = SH_STATE_RUNNING;
    g_system_health.active_display_code = SH_ERR_NONE;
    g_system_health.display_phase = SH_DISPLAY_PHASE_IDLE;
    g_system_health.current_digit_index = 0u;
    g_system_health.fatal_completed_display_rounds = 0u;
    sh_apply_led(g_system_health.state);
}

void sh_request_running_green_flash(void)
{
    OsalTimeMs now_ms = osal_time_now_monotonic();

    if (g_system_health.state != SH_STATE_RUNNING)
    {
        return;
    }

    g_system_health.running_green_flash_until_ms =
        (OsalTimeMs)(now_ms + SH_RUNNING_GREEN_FLASH_MS);
    sh_apply_led(g_system_health.state);
}

void sh_set_custom_controller_calibration_pending(void)
{
    if (g_system_health.state != SH_STATE_RUNNING)
    {
        return;
    }

    g_system_health.custom_controller_calibration_state =
        SH_CUSTOM_CONTROLLER_CAL_PENDING;
    sh_apply_led(g_system_health.state);
}

void sh_set_custom_controller_calibration_success(void)
{
    if (g_system_health.state != SH_STATE_RUNNING)
    {
        return;
    }

    g_system_health.custom_controller_calibration_state =
        SH_CUSTOM_CONTROLLER_CAL_IDLE;
    sh_apply_led(g_system_health.state);
}

void sh_set_custom_controller_calibration_failed(void)
{
    OsalTimeMs now_ms = osal_time_now_monotonic();

    if (g_system_health.state != SH_STATE_RUNNING)
    {
        return;
    }

    g_system_health.custom_controller_calibration_state =
        SH_CUSTOM_CONTROLLER_CAL_FAILED;
    g_system_health.cal_failed_red_flash_until_ms =
        (OsalTimeMs)(now_ms + SH_CAL_FAILED_RED_FLASH_MS);
    sh_apply_led(g_system_health.state);
}

void sh_clear_custom_controller_calibration_indicator(void)
{
    if (g_system_health.state != SH_STATE_RUNNING)
    {
        g_system_health.custom_controller_calibration_state =
            SH_CUSTOM_CONTROLLER_CAL_IDLE;
        return;
    }

    g_system_health.custom_controller_calibration_state =
        SH_CUSTOM_CONTROLLER_CAL_IDLE;
    sh_apply_led(g_system_health.state);
}

/* start_task 的唯一健康监督入口。
 * 仲裁顺序固定：
 * 1. BOOTING 显示
 * 2. 计算 runtime fault 集合中的最高优先级错误
 * 3. 若 fatal 已锁存，则 fatal 直接接管显示码
 * 4. 若都没有，则回 RUNNING
 */
void sh_poll(void)
{
    OsalTimeMs now_ms = osal_time_now_monotonic();
    SHErrorCode next_fault_code = SH_ERR_NONE;

    if (g_system_health.state == SH_STATE_BOOTING && g_system_health.fatal.latched_flag != OM_TRUE)
    {
        sh_apply_led(g_system_health.state);
        return;
    }

    for (uint32_t index = 0u; index < SH_TASK_COUNT; index++)
    {
        SHEntry* entry = &g_system_health.entries[index];

        if (entry->registered_flag != OM_TRUE)
        {
            continue;
        }

        /* 心跳超时直接映射为对应 runtime fault。 */
        if ((OsalTimeMs)(now_ms - entry->last_beat_ms) > entry->timeout_ms)
        {
            g_system_health.runtime_fault_active[entry->fault_code] = OM_TRUE;
        }
        else
        {
            g_system_health.runtime_fault_active[entry->fault_code] = OM_FALSE;
        }
    }

    for (uint32_t code = 1u; code <= SH_ERR_RUNTIME_FAULT_MAX; code++)
    {
        if (g_system_health.runtime_fault_active[code] == OM_TRUE)
        {
            if (sh_error_precedes((SHErrorCode)code, next_fault_code) == OM_TRUE)
            {
                next_fault_code = (SHErrorCode)code;
            }
        }
    }

    if (g_system_health.fatal.latched_flag == OM_TRUE)
    {
        next_fault_code = g_system_health.fatal.code;
    }

    if (next_fault_code == SH_ERR_NONE)
    {
        if (g_system_health.state != SH_STATE_RUNNING)
        {
            sh_set_running();
        }
        else
        {
            sh_apply_led(g_system_health.state);
        }
        return;
    }

    /* 错误显示只允许存在一个“当前显示中的错误码”。 */
    if (g_system_health.state != SH_STATE_ERROR || g_system_health.active_display_code != next_fault_code)
    {
        g_system_health.state = SH_STATE_ERROR;
        sh_start_error_display(next_fault_code, now_ms);
    }

    sh_update_error_display(now_ms);
}
