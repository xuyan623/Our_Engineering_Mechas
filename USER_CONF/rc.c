#include "rc.h"
#include "remote_ctrl.h"
#include "usart.h"
#include "dma.h"
#include "detect_task.h"

rc_info_t rc;

/*重置DMA*/
void DMA_ReStart(void)
{
	USART_Cmd(USART1,DISABLE);
	DMA_Cmd(DMA2_Stream2,DISABLE);
	DMA_SetCurrDataCounter(DMA2_Stream2,DBUS_BUFLEN);
	
	USART_ClearFlag(USART1,USART_FLAG_IDLE);
	DMA_ClearFlag(DMA2_Stream2,DMA_FLAG_TCIF2);
	DMA_ClearITPendingBit(DMA2_Stream2,DMA_IT_TCIF2);
	
	USART_Cmd(USART1,ENABLE);
	DMA_Cmd(DMA2_Stream2,ENABLE);
	
}


/*USART1 中断函数*/
void USART1_IRQHandler(void)
{
	static uint16_t rbuf_size;
	if(USART_GetFlagStatus(USART1,USART_FLAG_IDLE) != RESET 
		 && USART_GetITStatus(USART1,USART_IT_IDLE) != RESET)
	{
		USART_ReceiveData(USART1);
		if(DMA_GetCurrentMemoryTarget(DMA2_Stream2) == 0)	//Memory 0
		{
			/*重置DMA*/
			DMA_Cmd(DMA2_Stream2,DISABLE);
			rbuf_size = DBUS_MAX_LEN - DMA_GetCurrDataCounter(DMA2_Stream2);
			if(rbuf_size == DBUS_BUFLEN*2)
			{
				remote_ctrl(&rc,dbus_buf[0]);
        err_detector_hook(REMOTE_CTRL_OFFLINE);
			}
			DMA_SetCurrDataCounter(DMA2_Stream2,DBUS_MAX_LEN);
			DMA2_Stream2->CR |= DMA_SxCR_CT;
			
			DMA_ClearFlag(DMA2_Stream2,DMA_FLAG_TCIF2|DMA_FLAG_HTIF2);
			DMA_Cmd(DMA2_Stream2,ENABLE);	
		}
		else																						 //Memory 1
		{
			/*重置DMA*/
			DMA_Cmd(DMA2_Stream2,ENABLE);
			rbuf_size = DBUS_MAX_LEN - DMA_GetCurrDataCounter(DMA1_Stream2);
			if(rbuf_size == DBUS_BUFLEN*2)
			{
				remote_ctrl(&rc,dbus_buf[1]);		
        err_detector_hook(REMOTE_CTRL_OFFLINE);
			}
			DMA_SetCurrDataCounter(DMA2_Stream2,DBUS_MAX_LEN);
			DMA2_Stream2 ->CR &= ~(DMA_SxCR_CT);
			
			DMA_ClearFlag(DMA2_Stream2,DMA_FLAG_TCIF2|DMA_FLAG_HTIF2);
			DMA_Cmd(DMA2_Stream2,ENABLE);
		}
		USART_ClearFlag(USART1, USART_FLAG_IDLE);//清除空闲中断标志位
		USART_ClearITPendingBit(USART1,USART_FLAG_IDLE);
	}
}
