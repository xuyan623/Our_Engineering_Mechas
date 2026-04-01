#include "pc_task.h"
#include "STM32_TIM_BASE.h"

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

#include "comm_task.h"
#include "detect_task.h"
#include "data_packet.h"
#include "pc_rx_data.h"
#include "pc_tx_data.h"
#include "dma.h"

extern TaskHandle_t pc_rx_Task_Handle;
	BaseType_t STAUS;
		uint32_t Signal;
UBaseType_t pc_tx_stack_surplus;
UBaseType_t pc_rx_stack_surplus;

void pc_tx_task(void *parm)
{
	uint32_t pc_wake_time = osKernelSysTick();
  while(1)
  {
    pc_send_data_packet_pack();
    send_packed_fifo_data(&pc_txdata_fifo, UP_REG_ID);
    
    pc_tx_stack_surplus = uxTaskGetStackHighWaterMark(NULL);    
    vTaskDelayUntil(&pc_wake_time, 20);
  }
}

void pc_rx_task(void *parm)
{
	uint32_t Signal;

  
  while(1)
  {
    STAUS = xTaskNotifyWait((uint32_t) NULL, 
                            (uint32_t) PC_UART_IDLE_SIGNAL, 
                            (uint32_t *)&Signal, 
                            (TickType_t) portMAX_DELAY );
    if(STAUS == pdTRUE)
		{
			if(Signal & PC_UART_IDLE_SIGNAL)
			{
        dma_buffer_to_unpack_buffer(&pc_rx_obj, UART_IDLE_IT,JUDGE_MAX_LEN);
        unpack_fifo_data(&pc_unpack_obj, DN_REG_ID,NORMAL);         
      }
    }
    pc_rx_stack_surplus = uxTaskGetStackHighWaterMark(NULL);  
  }
}


///*USART3 中断函数*/
//void USART3_IRQHandler(void)
//{
//  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
//  
//	if(USART_GetFlagStatus(USART3,USART_FLAG_IDLE) != RESET 
//		 && USART_GetITStatus(USART3,USART_IT_IDLE) != RESET)
//	{
//		USART_ReceiveData(USART3);		
//		USART_ClearFlag(USART3, USART_FLAG_IDLE);//清除空闲中断标志位
//    
//    err_detector_hook(PC_SYS_OFFLINE);
//    
//    if(pc_rx_Task_Handle != NULL)//避免任务没来得及创建就发送信号量，导致卡在断言机制中
//    {
//      xTaskNotifyFromISR((TaskHandle_t) pc_rx_Task_Handle, 
//                         (uint32_t) PC_UART_IDLE_SIGNAL, 
//                         (eNotifyAction) eSetBits, 
//                         (BaseType_t *)&xHigherPriorityTaskWoken);
//      
//      /*进行上下文切换*/
//      if(xHigherPriorityTaskWoken != pdFALSE)
//        portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
//    }
//	}
//}

