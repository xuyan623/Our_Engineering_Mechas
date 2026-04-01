#ifndef _tim_H
#define _tim_H


#include "stm32f4xx.h"

//#define ENCODER_TIM_PSC  0          /*计数器分频*/
//#define ENCODER_TIM_PERIOD  65535   /*计数器最大值*/
//#define CNT_INIT 0                  /*计数器初值*/


void TIM5_DEVICE(int per,int psc);
void TIM4_DEVICE(int per,int psc);
void TIM2_DEVICE(int per,int psc);
void SERVO_INIT(TIM_TypeDef* TIM,uint32_t TIM_out,uint8_t Compare);
void PWM_PB5_Init(int per,int psc);
void Test_GPIO_PB5(void);

//void TIM4_ENCODER_Init(void);

#endif

