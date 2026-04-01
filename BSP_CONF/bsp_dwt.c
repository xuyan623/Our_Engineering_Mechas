/**
  ******************************************************************************
  * @file	 bsp_dwt.c
  * @author  Wang Hongxi
  * @version V1.0.0
  * @date    2020/2/14
  * @brief
  ******************************************************************************
  * @attention
  *
  ******************************************************************************
  */
#include "stm32f4xx.h"
#include "bsp_dwt.h"
#include "core_cm4.h"

static uint32_t CPU_FREQ_Hz;
static uint32_t CYCCNT_RountCount;
static float DWT_Timeline;


void DWT_Init(uint32_t CPU_Freq_mHz)
{
    /* 使能DWT外设 */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;

    /* DWT CYCCNT寄存器计数清0 */
    DWT->CYCCNT = (uint32_t)0u;

    /* 使能Cortex-M DWT CYCCNT寄存器 */
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    CPU_FREQ_Hz = CPU_Freq_mHz * 1000000;
    CYCCNT_RountCount = 0;
    DWT_Timeline = 0;
}

float DWT_GetDeltaT(uint32_t *cnt_last)
{
    volatile uint32_t cnt_now = DWT->CYCCNT;
    float dt = ((uint32_t)(cnt_now - *cnt_last)) / ((float)(CPU_FREQ_Hz));
    *cnt_last = cnt_now;
    return dt;
}

double DWT_GetDeltaT64(uint32_t *cnt_last)
{
    volatile uint32_t cnt_now = DWT->CYCCNT;
    double dt = ((uint32_t)(cnt_now - *cnt_last)) / ((double)(CPU_FREQ_Hz));
    *cnt_last = cnt_now;
    return dt;
}

float DWT_GetTimeline(void)
{
    volatile uint32_t cnt_now = DWT->CYCCNT;
    static uint32_t CYCCNT_Last;

    if (cnt_now < CYCCNT_Last)
        CYCCNT_RountCount++;

    DWT_Timeline = ((uint64_t)((uint64_t)CYCCNT_RountCount * (uint64_t)UINT32_MAX) + (uint64_t)cnt_now) / ((float)(CPU_FREQ_Hz));

    CYCCNT_Last = cnt_now;

    return DWT_Timeline;
}

void DWT_Delay(float Delay)
{
    uint32_t tickstart = DWT->CYCCNT;
    float wait = Delay;

    while((DWT->CYCCNT - tickstart) < wait * (float)CPU_FREQ_Hz)
    {
    }
}

