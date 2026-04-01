#ifndef _delay_H
#define _delay_H

#include "stm32f4xx.h"

void SysTick_Init(u8 SYSCLK);
void delay_ms(u16 nms);
void delay_us(u32 nus);



#endif
