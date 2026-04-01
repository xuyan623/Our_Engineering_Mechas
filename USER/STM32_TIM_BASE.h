#ifndef _STM32_TIM_BASE_H
#define _STM32_TIM_BASE_H


#include "stm32f4xx.h"

void TIM_BASE_Init(u16 per,u16 psc);
void HAL_IncTick(void);
uint32_t HAL_GetTick(void);
uint32_t osKernelSysTick(void);
extern __IO uint32_t uwTick;

#endif

