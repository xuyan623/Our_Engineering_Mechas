

#include "gpio.h"
#include "delay.h"
#include "bsp_flash.h"
#include "bsp_can.h"

#include "IO.h"
static void KEY_EXTI(void);

/**************************************************
GPIOG:PIN1~PIN8  led灯
GPIOB:PIN2       key按键
GPIOF:PIN5       imu片选脚

I1->GPIOF PIN1   气泵开关
J1->GPIOE PIN5	 倾斜
K1->GPIOE PIN6   伸缩
L1->GPIOC PIN2	 复活
M1->GPIOC PIN3   障碍块

Q1->GPIOF PIN10		光电门信号

GPIOI PIN0 抬升按键压触

***************************************************/


/****************2025赛季pin口******************************

GPIOC 0——5 软件spi

**********************************************/




void GPIO_INIT(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE,ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF,ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOH,ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOI,ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG,ENABLE);
	
	/*LED灯*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Low_Speed;
	GPIO_Init(GPIOG,&GPIO_InitStructure);
  GPIO_SetBits(GPIOG,GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7);
	GPIO_ResetBits(GPIOG,GPIO_Pin_8);
	 
	/*GPIOA*/
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Low_Speed;
	GPIO_Init(GPIOA,&GPIO_InitStructure);
  
	GPIO_SetBits(GPIOA,GPIO_Pin_5);
	
  /*GPIOC*/
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2|GPIO_Pin_3;
//	GPIO_Init(GPIOC,&GPIO_InitStructure);
//  
//  GPIO_SetBits(GPIOC,GPIO_Pin_2|GPIO_Pin_3);
	
	/*限位开关供电段 PC5 O1*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
	GPIO_Init(GPIOC,&GPIO_InitStructure);
  
  //GPIO_SetBits(GPIOC,GPIO_Pin_5);
	
	/*限位开关供电段 PF10 Q1*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_Init(GPIOF,&GPIO_InitStructure);
  
  GPIO_SetBits(GPIOF,GPIO_Pin_10);
	
  /*GPIOE*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5|GPIO_Pin_6;
	GPIO_Init(GPIOE,&GPIO_InitStructure);
  
  GPIO_SetBits(GPIOE,GPIO_Pin_5|GPIO_Pin_6);
  
  /*GPIOF*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
	GPIO_Init(GPIOF,&GPIO_InitStructure);
  
//  GPIO_SetBits(GPIOF,GPIO_Pin_1);
  /*GPIOH   开发板的可控电源输出24V*/
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5;
//	GPIO_Init(GPIOH,&GPIO_InitStructure);
//  
//  GPIO_SetBits(GPIOH, GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5);
	
  /*GPIOI   控制显示屏切屏IO*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
	GPIO_Init(GPIOC,&GPIO_InitStructure);
	
	/*夹取IO配置（输出模式）*/
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Low_Speed;
	/*夹取头*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
	GPIO_Init(GPIOF,&GPIO_InitStructure);
	/*夹取尾*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
	GPIO_Init(GPIOE,&GPIO_InitStructure);
	/*伸缩头*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
	GPIO_Init(GPIOE,&GPIO_InitStructure);
	/*伸缩尾*/
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
//	GPIO_Init(GPIOC,&GPIO_InitStructure);
//	/*复活头*/
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
//	GPIO_Init(GPIOC,&GPIO_InitStructure);
//	/*复活尾*/
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
//	GPIO_Init(GPIOC,&GPIO_InitStructure);
//	/*障碍头*/
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
//	GPIO_Init(GPIOC,&GPIO_InitStructure);
	/*障碍尾*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
	GPIO_Init(GPIOA,&GPIO_InitStructure);
	
	
	/*显示器切换*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
	GPIO_Init(GPIOC,&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_Init(GPIOF,&GPIO_InitStructure);
//	  GPIO_SetBits(GPIOF,GPIO_Pin_0);

	
	/*IMU片选脚*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Low_Speed;
	GPIO_Init(GPIOF,&GPIO_InitStructure);
	GPIO_SetBits(GPIOF, GPIO_Pin_6); 
	
	
	/*KEY按键*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOB,&GPIO_InitStructure);
	
	/*夹取限位开关（输入模式）*/
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Low_Speed;
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_Init(GPIOF,&GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_Init(GPIOI,&GPIO_InitStructure);
	
		/*抬升限位开关（输入模式）*/
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Low_Speed;
	
	GPIO_InitStructure.GPIO_Pin = UPRISE_KEY1_PIN;
	GPIO_Init(UPRISE_KEY1_PORT,&GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = UPRISE_KEY2_PIN;
	GPIO_Init(UPRISE_KEY2_PORT,&GPIO_InitStructure);
	
//	KEY_EXTI();
}


/*****************************************************************
PB2:KEY按键外部外部中断配置
******************************************************************/
static void KEY_EXTI(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;
	EXTI_InitTypeDef  EXTI_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG,ENABLE);			//使能外部中断源时钟
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB,EXTI_PinSource2);	//映射到IO口
	
	//EXTI2 NVIC 配置
	NVIC_InitStructure.NVIC_IRQChannel = EXTI2_IRQn;//EXTI2中断通道
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=2;//抢占优先级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =2;		//子优先级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器
  
	EXTI_InitStructure.EXTI_Line=EXTI_Line2; 
	EXTI_InitStructure.EXTI_Mode=EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger=EXTI_Trigger_Rising;
	EXTI_InitStructure.EXTI_LineCmd=ENABLE;
	EXTI_Init(&EXTI_InitStructure);
}


uint8_t key_state;//按键状态标志位

//void EXTI2_IRQHandler(void)
//{
//	if(EXTI_GetITStatus(EXTI_Line2)==1)
//	{
//		delay_ms(10);
//		if(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_2)==Bit_SET)
//		{	
//			
//			cali_param.yaw_offset = moto_yaw.ecd;
//			cali_param.pitch_offset = moto_pit.ecd;
//      cali_param.cali_state = CALI_DONE;
//			key_state = BSP_FLASH_WRITE((uint8_t *)&cali_param,sizeof(cali_sys_t));
//			
//		}
//	}
//	EXTI_ClearITPendingBit(EXTI_Line2);    
//}
