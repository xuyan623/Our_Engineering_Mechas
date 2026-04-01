#ifndef __VOFA_H
#define __VOFA_H

#include "stm32f4xx.h"


typedef union
{
	float fdata;
	unsigned long ldata;
}FloatLongType;


extern float vofa_debug[6];

void JustFloat_Send(float * fdata,uint16_t fdata_num,USART_TypeDef *Usart_choose);
void UART7_Test(void);


#endif
