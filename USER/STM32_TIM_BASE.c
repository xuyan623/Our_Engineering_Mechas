#include "STM32_TIM_BASE.h"
#include "system_stm32f4xx.h"
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

__IO uint32_t uwTick;

/*10-1,9000-1*/
void TIM_BASE_Init(u16 per,u16 psc)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);//使能TIM3时钟
	
	TIM_TimeBaseInitStructure.TIM_Period        = per;   //自动装载值
	TIM_TimeBaseInitStructure.TIM_Prescaler     = psc*2; //分频系数
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInitStructure.TIM_CounterMode   = TIM_CounterMode_Up; //设置向上计数模式
	TIM_TimeBaseInit(TIM3,&TIM_TimeBaseInitStructure);
	
	TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE); //开启定时器中断
	TIM_ClearITPendingBit(TIM3,TIM_IT_Update);
	
	NVIC_InitStructure.NVIC_IRQChannel                   = TIM3_IRQn;//定时器中断通道
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;        //抢占优先级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0;		     //子优先级
	NVIC_InitStructure.NVIC_IRQChannelCmd                =  ENABLE;	 //IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	
	
	TIM_Cmd(TIM3,ENABLE); //使能定时器	
}

__weak void HAL_IncTick(void)
{
  uwTick++;
	if(uwTick == 0xffffffff)
		uwTick = 0;
}

__weak uint32_t HAL_GetTick(void)
{
  return uwTick;
}

/*确定我们是处于线程模式还是处理程序模式。*/
static int inHandlerMode (void)
{
  return __get_IPSR() != 0;
}

/******************************************
获取内核SysTick计时器的值(操作系统是用滴答定时器作为时间源)
osKernelSysTick应该在每个CMSIS-RTOS中保持一致
*******************************************/
uint32_t osKernelSysTick(void)
{
  if (inHandlerMode()) {
    return xTaskGetTickCountFromISR();
  }
  else {
    return xTaskGetTickCount();
  }
}

/*用定时器3记录代码运行时间*/
void TIM3_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM3,TIM_IT_Update))
	{
		HAL_IncTick();
	}
	TIM_ClearITPendingBit(TIM3,TIM_IT_Update);	
}


