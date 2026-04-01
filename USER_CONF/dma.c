#include "dma.h"
#include "motor_8010.h"
/*USART1*/
uint8_t  dbus_buf[2][DBUS_MAX_LEN];
/*USART2*/
uint8_t  usart2_TXbuf[USART2_BUFLEN];
uint8_t  usart2_RXbuf[USART2_BUFLEN];
/*USART3*/
uint8_t  pc_rxbuf[2][PC_MAX_LEN];
/*USART6*/
uint8_t  judge_rxbuf[2][JUDGE_MAX_LEN];

/*UART8*/
uint8_t judge_control_rxbuf[2][JUDGE_MAX_LEN];

uint8_t judge_rxbuf_test[JUDGE_MAX_LEN];

/*SPI5*/
uint8_t  spi5_rx_buf[SPI5_BUFLEN];
uint8_t  spi5_tx_buf[SPI5_BUFLEN];

uint8_t  motor_8010_TXbuf[TX_MAX_BUFLEN];
uint8_t  motor_8010_RXbuf[RX_MAX_BUFLEN];

extern uint8_t send1_8010_flag;
extern uint8_t send2_8010_flag; 

void USART1_DMA(void)
{
	DMA_InitTypeDef DMA_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
	USART_DMACmd(USART1, USART_DMAReq_Rx, ENABLE);
	DMA_DeInit(DMA2_Stream2);
	
	DMA_InitStructure.DMA_Channel = DMA_Channel_4;
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&(USART1->DR);
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)dbus_buf[0];
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
	DMA_InitStructure.DMA_BufferSize = DBUS_MAX_LEN;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_Init(DMA2_Stream2,&DMA_InitStructure);
	/*使用双缓冲模式*/
	DMA_DoubleBufferModeConfig(DMA2_Stream2,(uint32_t)dbus_buf[1],DMA_Memory_0);
	DMA_DoubleBufferModeCmd(DMA2_Stream2,ENABLE);
	
	DMA_Cmd(DMA2_Stream2, DISABLE); //Add a disable
	DMA_Cmd(DMA2_Stream2, ENABLE);

}


void USART3_DMA(void)
{
	DMA_InitTypeDef DMA_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);
	USART_DMACmd(USART3, USART_DMAReq_Rx, ENABLE);
	
	DMA_DeInit(DMA1_Stream1);
	/*接收*/
	DMA_InitStructure.DMA_Channel = DMA_Channel_4;
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&(USART3->DR);
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)judge_rxbuf[0];
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
	DMA_InitStructure.DMA_BufferSize = JUDGE_MAX_LEN;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_Init(DMA1_Stream1,&DMA_InitStructure);

  /*接收使用双缓冲模式*/
	DMA_DoubleBufferModeConfig(DMA1_Stream1,(uint32_t)judge_rxbuf[1],DMA_Memory_0);
	DMA_DoubleBufferModeCmd(DMA1_Stream1,ENABLE);
//	

	DMA_ITConfig(DMA1_Stream1,DMA_IT_TC,ENABLE);
	DMA_ITConfig(DMA1_Stream1,DMA_IT_HT,ENABLE);
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Stream1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 5;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
		
	DMA_ClearFlag(DMA1_Stream1, DMA_FLAG_HTIF1);

	DMA_Cmd(DMA1_Stream1, DISABLE);
	DMA_Cmd(DMA1_Stream1, ENABLE);
	
}






void USART6_DMA(void)
{
	
	
	NVIC_InitTypeDef NVIC_InitStructure;
	DMA_InitTypeDef DMA_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
	USART_DMACmd(USART6, USART_DMAReq_Rx, ENABLE);
	USART_DMACmd(USART6, USART_DMAReq_Tx, ENABLE);
	
	DMA_DeInit(DMA2_Stream1);
	DMA_InitStructure.DMA_Channel = DMA_Channel_5;
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&(USART6->DR);
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)motor_8010_RXbuf;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
	DMA_InitStructure.DMA_BufferSize = RX_MAX_BUFLEN;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_Init(DMA2_Stream1,&DMA_InitStructure);
  
	
	DMA_DeInit(DMA2_Stream6);
	DMA_InitTypeDef DMA_InitStructure2;
	DMA_InitStructure2.DMA_Channel = DMA_Channel_5;
	DMA_InitStructure2.DMA_PeripheralBaseAddr = (uint32_t)&(USART6->DR);
	DMA_InitStructure2.DMA_Memory0BaseAddr = (uint32_t)&sdata[0];
	DMA_InitStructure2.DMA_DIR = DMA_DIR_MemoryToPeripheral;
	DMA_InitStructure2.DMA_BufferSize =TX_MAX_BUFLEN; //TX_MAX_BUFLEN;
	DMA_InitStructure2.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure2.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure2.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure2.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure2.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStructure2.DMA_Priority = DMA_Priority_VeryHigh;
	DMA_InitStructure2.DMA_FIFOMode = DMA_FIFOMode_Disable;
	DMA_InitStructure2.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
	DMA_InitStructure2.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStructure2.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_Init(DMA2_Stream6,&DMA_InitStructure2);
	
	
	NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream6_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 4;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	
	DMA_ITConfig(DMA2_Stream1, DMA_IT_TC, ENABLE);
	DMA_ITConfig(DMA2_Stream1, DMA_IT_FE, ENABLE);
	NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 4;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
//   /*接收使用单缓冲模式*/
	DMA_DoubleBufferModeConfig(DMA2_Stream1,(uint32_t)motor_8010_RXbuf,DMA_Memory_0);
	DMA_DoubleBufferModeCmd(DMA2_Stream1,ENABLE);
	
	DMA_ITConfig(DMA2_Stream6,DMA_IT_TC,ENABLE);
	DMA_Cmd(DMA2_Stream6, DISABLE);

	DMA_Cmd(DMA2_Stream1, DISABLE);
	
}



void DMA2_Stream6_IRQHandler(void)
{
	if(DMA_GetFlagStatus(DMA2_Stream6,DMA_FLAG_TCIF6) != RESET    //传输完成标志
		 && DMA_GetITStatus(DMA2_Stream6,DMA_IT_TCIF6) != RESET)    //传输中断标志
	{
		DMA_Cmd(DMA2_Stream6,DISABLE);   //禁用DMA2_Stream6。在重新配置DMA之前，通常需要先禁用DMA。
		DMA_ClearFlag(DMA2_Stream6, DMA_FLAG_TCIF6);//清除DMA2_Stream6的传输完成标志位（TCIF6）。这是为了确保不会残留上一次传输完成的中断标志，避免误触发。
		DMA_SetCurrDataCounter(DMA2_Stream6,TX_MAX_BUFLEN); //设置DMA2_Stream6的当前数据计数器，即本次传输的数据长度
		USART_ITConfig(USART6,USART_IT_TC,ENABLE);
	}
}

	void DMA2_Stream1_IRQHandler(void)
{
		
		//传输完成中断
		if(DMA_GetFlagStatus(DMA2_Stream1,DMA_FLAG_TCIF1) != RESET )
		{		  	
			
		  DMA_ClearFlag(DMA2_Stream1, DMA_FLAG_TCIF1);		 
			USART_ITConfig(USART6,USART_IT_IDLE,ENABLE);
		}
		  

}


/*SPI5*/
void SPI5_DMA(void)
{

    NVIC_InitTypeDef NVIC_InitStructure;
    DMA_InitTypeDef DMA_InitStructure;
    /* -------------- Enable Module Clock Source ----------------------------*/
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
		SPI_DMACmd(SPI5,SPI_I2S_DMAReq_Rx,ENABLE);
		SPI_DMACmd(SPI5,SPI_I2S_DMAReq_Tx,ENABLE);

    DMA_DeInit(DMA2_Stream5);
		DMA_DeInit(DMA2_Stream4);
	
    while (DMA_GetCmdStatus(DMA2_Stream5) != DISABLE)
    {
        ;
    }
    DMA_InitStructure.DMA_Channel = DMA_Channel_7;
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) & (SPI5->DR);
    DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)spi5_rx_buf;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
    DMA_InitStructure.DMA_BufferSize = SPI5_BUFLEN;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
    DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
    DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
    DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
    DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
    DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
    DMA_Init(DMA2_Stream5, &DMA_InitStructure);

    DMA_ITConfig(DMA2_Stream5, DMA_IT_TC, ENABLE);

    while (DMA_GetCmdStatus(DMA2_Stream4) != DISABLE)
    {
        ;
    }
    DMA_InitStructure.DMA_Channel = DMA_Channel_2;
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) & (SPI5->DR);
    DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)spi5_tx_buf;
    DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
    DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;
    DMA_Init(DMA2_Stream4, &DMA_InitStructure);

		DMA_Cmd(DMA2_Stream5, DISABLE); //Add a disable
    DMA_Cmd(DMA2_Stream4, DISABLE); //Add a disable
		DMA_Cmd(DMA2_Stream5, ENABLE); //Add a enable
    DMA_Cmd(DMA2_Stream4, ENABLE); //Add a enable
		
		NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream5_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 5;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
		
}



void UART8_DMA()
{
	DMA_InitTypeDef DMA_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);
	USART_DMACmd(UART8, USART_DMAReq_Rx, ENABLE);
	
	DMA_DeInit(DMA1_Stream6);
	/*接收*/
	DMA_InitStructure.DMA_Channel = DMA_Channel_5;
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&(UART8->DR);
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)judge_control_rxbuf[0];
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
	DMA_InitStructure.DMA_BufferSize = JUDGE_MAX_LEN;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_Init(DMA1_Stream6,&DMA_InitStructure);

  /*接收使用双缓冲模式*/

//	

	DMA_ITConfig(DMA1_Stream6,DMA_IT_TC,ENABLE);
//	DMA_ITConfig(DMA1_Stream6,DMA_IT_HT,ENABLE);
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Stream6_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 5;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
		
	DMA_ClearFlag(DMA1_Stream6, DMA_FLAG_HTIF1);

	DMA_Cmd(DMA1_Stream6, DISABLE);
	DMA_Cmd(DMA1_Stream6, ENABLE);
}	
