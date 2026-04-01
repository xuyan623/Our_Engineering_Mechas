#include "judge_task.h"
#include "STM32_TIM_BASE.h"

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

#include "comm_task.h"
#include "detect_task.h"
#include "data_packet.h"
#include "judge_rx_data.h"
#include "judge_tx_data.h"
#include <string.h> 
#include "dma.h"

extern TaskHandle_t judge_rx_Task_Handle;
extern uint8_t judge_rxbuf_test[256];
UBaseType_t judge_tx_stack_surplus;
UBaseType_t judge_rx_stack_surplus;
uint8_t DATA[113] = {"AwakeLion!!!"};//该数组用来储存机器人交互的数据，可用户自行修改


interaction_figure_t figure_line;

/*
		非图传链路的裁判系统，对于工程车来说，由单片机发送给裁判系统的数据主要作用就是：画UI
		而以下的发送任务就是调用了封装在judge_task_tx里的画ui的函数
		而接收主要就只有对 遥控器与控制器的连接成功的数据 发送给了单片机

*/


void judge_tx_task(void *parm)
{
	uint32_t judge_wake_time = osKernelSysTick();
	static uint8_t i;
  while(1)
  {
		/*车间通信*/
//			judgement_client_packet_pack(DATA);
//			send_packed_fifo_data(&judge_txdata_fifo, DN_REG_ID);
		 
		//  i++;
		
		
		    grapic_line_fun(&figure_line,31,Add,layer0,Yellow,1,660,240,1260,840);
		    data_packet_pack(STUDENT_INTERACTIVE_HEADER_DATA_ID,(uint8_t *)&figure_line,	sizeof(figure_line), UP_REG_ID);
	      send_packed_fifo_data(&judge_txdata_fifo, UP_REG_ID);			
		/*首先发送不需要变化的 发送成功后将不在刷新这些数据*/
			if(i < 5)
			{
				
				

				
				
		//		judgement_client_graphics_draw_pack(CONSTANT);//显示常量
		//		send_packed_fifo_data(&judge_txdata_fifo, DN_REG_ID);
			}
			else if(i < 10)
			{
				/*显示实时工程当前模式*/
				judgement_client_graphics_draw_pack(CHASSIS_MODE);//显示底盘模式
				send_packed_fifo_data(&judge_txdata_fifo, DN_REG_ID);
			}
			else if(i < 15)
			{
				/*实时显示夹取箱数*/
				judgement_client_graphics_draw_pack(PICK_BOX);//显示夹取箱数
				send_packed_fifo_data(&judge_txdata_fifo, DN_REG_ID);
			}
			else if(i < 20)
			{
				judgement_client_graphics_draw_pack(AUXILIARY_LINE);//显示空接
				send_packed_fifo_data(&judge_txdata_fifo, DN_REG_ID);				
			}
			else if(i < 25)
			{
				judgement_client_graphics_draw_pack(MODE_STATE);//显示吸盘状态
				send_packed_fifo_data(&judge_txdata_fifo, DN_REG_ID);								
			}
			if(i > 20)
					i = 6;	
    
  //  judge_tx_stack_surplus = uxTaskGetStackHighWaterMark(NULL);    
			
    vTaskDelayUntil(&judge_wake_time, 100);
  }
}


//fifo_register(&test_fifo,judge_rxbuf_test,39,NULL,NULL);
void judge_rx_task(void *parm)
{
  uint32_t Signal;
//	BaseType_t STAUS;
  while(1)
  {

		
      unpack_fifo_data(&judge_unpack_obj, DN_REG_ID,NORMAL);//同样再通过指针取址的方法把FIFO里的数据拿出来放进一个数组里
			unpack_fifo_data(&control_unpack_obj, DN_REG_ID,CONTROLLER);
  }
}

void DMA1_Stream1_IRQHandler(void)
{
	if(DMA_GetFlagStatus(DMA1_Stream1,DMA_FLAG_TCIF1) != RESET 
		 && DMA_GetITStatus(DMA1_Stream1,DMA_IT_TCIF1) != RESET)
	{
		dma_buffer_to_unpack_buffer(&judge_rx_obj,UART_DMA_FULL_IT,JUDGE_MAX_LEN);
		DMA_ClearFlag(DMA1_Stream1, DMA_FLAG_TCIF1);
	}
	
		if(DMA_GetFlagStatus(DMA1_Stream1,DMA_FLAG_HTIF1) != RESET 
		 && DMA_GetITStatus(DMA1_Stream1,DMA_IT_HTIF1) != RESET)
	{
		
		dma_buffer_to_unpack_buffer(&judge_rx_obj,UART_DMA_HALF_IT,JUDGE_MAX_LEN);
		DMA_ClearFlag(DMA1_Stream1, DMA_FLAG_HTIF1);
	}
	
	
}

void DMA1_Stream6_IRQHandler(void)
{
	if(DMA_GetFlagStatus(DMA1_Stream6,DMA_FLAG_TCIF6) != RESET 
		 && DMA_GetITStatus(DMA1_Stream6,DMA_IT_TCIF6) != RESET)
	{
		dma_buffer_to_unpack_buffer(&control_rx_obj,UART_DMA_FULL_IT,JUDGE_MAX_LEN);
		DMA_ClearFlag(DMA1_Stream6, DMA_FLAG_TCIF6);
	}
	
//		if(DMA_GetFlagStatus(DMA1_Stream6,DMA_FLAG_HTIF6) != RESET 
//		 && DMA_GetITStatus(DMA1_Stream6,DMA_IT_HTIF6) != RESET)
//	{
//		dma_buffer_to_unpack_buffer(&control_rx_obj,UART_DMA_HALF_IT,JUDGE_MAX_LEN);
//		DMA_ClearFlag(DMA1_Stream6, DMA_FLAG_HTIF6);
//	}
	
	
}
//用于传输自定义控制器的数据
void UART8_IRQHandler(void)
{
		if(USART_GetFlagStatus(UART8,USART_FLAG_IDLE) != RESET 
		 && USART_GetITStatus(UART8,USART_IT_IDLE) != RESET)
	{
		dma_buffer_to_unpack_buffer(&control_rx_obj, UART_IDLE_IT,JUDGE_MAX_LEN);
		USART_ReceiveData(UART8);
		USART_ClearFlag(UART8, USART_FLAG_IDLE);//清除空闲中断标志位
	}
}

//用于传输裁判系统数据，但其实这个串口没用到
void USART3_IRQHandler(void)
{
	 BaseType_t xHigherPriorityTaskWoken = pdFALSE;

	
	if(USART_GetFlagStatus(USART3,USART_FLAG_IDLE) != RESET 
		 && USART_GetITStatus(USART3,USART_IT_IDLE) != RESET)
	{
		dma_buffer_to_unpack_buffer(&judge_rx_obj, UART_IDLE_IT,JUDGE_MAX_LEN);
		USART_ReceiveData(USART3);
		USART_ClearFlag(USART3, USART_FLAG_IDLE);//清除空闲中断标志位
	}
	
	
	
	if(USART_GetITStatus(USART3,USART_IT_RXNE) != RESET)
	{
		USART_ReceiveData(USART3);
    //DMA_ClearITPendingBit(USART3, USART_IT_RXNE);//清除空闲中断标志位
    USART_ClearITPendingBit(USART3,USART_IT_RXNE);
    err_detector_hook(JUDGE_SYS_OFFLINE);
    
    if(judge_rx_Task_Handle != NULL) //避免任务没来得及创建就发送信号量，导致卡在断言机制中
    {
      xTaskNotifyFromISR((TaskHandle_t) judge_rx_Task_Handle, 
                         (uint32_t) JUDGE_UART_IDLE_SIGNAL,
                         (eNotifyAction) eSetBits,
                         (BaseType_t *)&xHigherPriorityTaskWoken);
      /*进行上下文切换*/
      if(xHigherPriorityTaskWoken != pdFALSE)
        portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
    }
	}
}

/*USART6 中断函数*/
//用于传输8010的数据
void USART6_IRQHandler(void)//当数据为达到包的一半时，通过串口空闲中断将这些不完整的数据进行接收
{	
	
		if(USART_GetFlagStatus(USART6,USART_FLAG_IDLE) != RESET //简单来说：USART_IT_IDLE告诉你"中断系统认为应该进入中断"，USART_FLAG_IDLE告诉你"硬件确实检测到了空闲"。两者都满足才是真正的"因为空闲检测而触发的中断"。
			 && USART_GetITStatus(USART6,USART_IT_IDLE) != RESET)
		{
			USART_ReceiveData(USART6);
			USART_ITConfig(USART6,USART_IT_IDLE,DISABLE);	//关闭空闲中断，防止处理数据时再有数据进来
			DMA_Cmd(DMA2_Stream1,DISABLE);	
			
			//这里DMA失能后，其实一般会通过计算接收到的数据长度，然后进行数据处理，然后在重新设置长度后使能DMA
			
			DMA_SetCurrDataCounter(DMA2_Stream1,16);
			DMA_Cmd(DMA2_Stream1,ENABLE);
			USART_ClearITPendingBit(USART6,USART_IT_IDLE);
			USART_ClearFlag(USART6, USART_FLAG_IDLE);//清除空闲中断标志位
		}
			if(USART_GetFlagStatus(USART6,USART_IT_ORE_RX) != RESET)//USART_IT_ORE_RX​ 是接收溢出错误中断标志，用于检测数据接收溢出。当接收数据寄存器（RDR）未及时读取，又有新数据到达时触发。必须及时清除该标志，否则会导致后续通信异常。
		{
			
			USART_ReceiveData(USART6);
			USART_ClearFlag(USART6, USART_IT_ORE_RX);               //这里清除的标志位可能是错的，应该清除的是IT也就是中断标志位
		}
		/*串口发送完成中断*/
			if(USART_GetITStatus(USART6,USART_IT_TC) != RESET)
		{
				
			USART_ClearFlag(USART6, USART_FLAG_TC);//清除空闲中断标志位
			USART_ITConfig(USART6,USART_IT_TC,DISABLE);		
		}
		

}


