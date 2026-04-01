#include "tim.h"

/*PI0   ------> TIM5_CH4   A  挡板右
  PH12  ------> TIM5_CH3   B  挡板左
	
	PH11 -------> TIM5_CH2   C  姿态右
	PH10 -------> TIM5_CH1   D  云台1PIT
	
	PD15  ------> TIM4_CH4   E  救援右
  PD14  ------> TIM4_CH3   F  救援左
	
	PD13  ------> TIM4_CH2   G  姿态左
	*/
	
void TIM5_DEVICE(int per,int psc)
{
  GPIO_InitTypeDef        GPIO_InitStructure;
  TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
  TIM_OCInitTypeDef       TIM_OCInitStructure;
  
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOI,ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOH,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5,ENABLE);
  
  GPIO_PinAFConfig(GPIOI,GPIO_PinSource0,GPIO_AF_TIM5);
	GPIO_PinAFConfig(GPIOH,GPIO_PinSource10,GPIO_AF_TIM5);
	GPIO_PinAFConfig(GPIOH,GPIO_PinSource11,GPIO_AF_TIM5);
	GPIO_PinAFConfig(GPIOH,GPIO_PinSource12,GPIO_AF_TIM5);
  
  GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_High_Speed;
	GPIO_Init(GPIOI,&GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_12;
	GPIO_Init(GPIOH,&GPIO_InitStructure);
  
  TIM_TimeBaseInitStructure.TIM_Period        = per;                //重装载值
  TIM_TimeBaseInitStructure.TIM_Prescaler     = psc;                //预分频系数
  TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;       //时钟分频因子
  TIM_TimeBaseInitStructure.TIM_CounterMode   = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM5,&TIM_TimeBaseInitStructure);
  
  TIM_OCInitStructure.TIM_OCMode      = TIM_OCMode_PWM1;          //小于CCR为有效值
  TIM_OCInitStructure.TIM_OCPolarity  = TIM_OCPolarity_High;      //有效电平为高电平
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;   //比较输出使能  
  TIM_OCInitStructure.TIM_Pulse       = 1000;                     //输出比较值CCR	
  
  TIM_OC4Init(TIM5,&TIM_OCInitStructure);
  TIM_OC3Init(TIM5,&TIM_OCInitStructure);
	TIM_OC2Init(TIM5,&TIM_OCInitStructure);
	TIM_OC1Init(TIM5,&TIM_OCInitStructure);
  
  TIM_OC4PreloadConfig(TIM5,TIM_OCPreload_Enable);
	TIM_OC3PreloadConfig(TIM5,TIM_OCPreload_Enable); //输出比较 4 预装载使能 OC4PE
	TIM_OC2PreloadConfig(TIM5,TIM_OCPreload_Enable); 
	TIM_OC1PreloadConfig(TIM5,TIM_OCPreload_Enable); 
	
  TIM_ARRPreloadConfig(TIM5,ENABLE);               //自动重载预装载使能 ARPE
  
  TIM_Cmd(TIM5,ENABLE);
}
void TIM4_DEVICE(int per,int psc)
{
  GPIO_InitTypeDef        GPIO_InitStructure;
  TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
  TIM_OCInitTypeDef       TIM_OCInitStructure;
  
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE);
  
  GPIO_PinAFConfig(GPIOD,GPIO_PinSource15,GPIO_AF_TIM4);
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource14,GPIO_AF_TIM4);
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource13,GPIO_AF_TIM4);
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource12,GPIO_AF_TIM4);
  
  GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_12|GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_High_Speed;
	GPIO_Init(GPIOD,&GPIO_InitStructure);
  
  TIM_TimeBaseInitStructure.TIM_Period        = per;                //重装载值
  TIM_TimeBaseInitStructure.TIM_Prescaler     = psc;                //预分频系数
  TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;       //时钟分频因子
  TIM_TimeBaseInitStructure.TIM_CounterMode   = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM4,&TIM_TimeBaseInitStructure);
  
  TIM_OCInitStructure.TIM_OCMode      = TIM_OCMode_PWM1;          //小于CCR为有效值
  TIM_OCInitStructure.TIM_OCPolarity  = TIM_OCPolarity_High;      //有效电平为高电平
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;   //比较输出使能  
  TIM_OCInitStructure.TIM_Pulse       = 1000;                     //输出比较值CCR	
  
  TIM_OC4Init(TIM4,&TIM_OCInitStructure);
  TIM_OC3Init(TIM4,&TIM_OCInitStructure);
	TIM_OC2Init(TIM4,&TIM_OCInitStructure);
  
  TIM_OC4PreloadConfig(TIM4,TIM_OCPreload_Enable);
	TIM_OC3PreloadConfig(TIM4,TIM_OCPreload_Enable); //输出比较 4 预装载使能 OC4PE
	TIM_OC2PreloadConfig(TIM4,TIM_OCPreload_Enable); 
	
  TIM_ARRPreloadConfig(TIM4,ENABLE);               //自动重载预装载使能 ARPE
  
  TIM_Cmd(TIM4,ENABLE);
}



/**
 * @brief  初始化 TIM10 的 PWM 输出（复用 PB5 引脚）
 * @param  per: 自动重装载值 (注意 TIM10 最大支持 65535)
 * @param  psc: 预分频系数
 * @note   引脚映射：PB5 -> TIM10_CH3 (实际上是 AF3)
 * @note   TIM10 是单通道定时器，只有 CH1
 */
void PWM_PB5_Init(int per, int psc)
{
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);   // 使能GPIOB时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM10, ENABLE);   // 使能TIM10时钟

    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;             // 复用功能模式
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;       // 高速输出
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;           // 推挽输出
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;             // 上拉
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    GPIO_PinAFConfig(GPIOB, GPIO_PinSource5, GPIO_AF_TIM10);

    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_TimeBaseStructure.TIM_Period = per;                  // 自动重装载值
    TIM_TimeBaseStructure.TIM_Prescaler = psc;                // 预分频器
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;   // 时钟分频
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; // 向上计数
    TIM_TimeBaseInit(TIM10, &TIM_TimeBaseStructure);         

   
    TIM_OCInitTypeDef TIM_OCInitStructure;
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;         // PWM1模式
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; // 使能输出
    TIM_OCInitStructure.TIM_Pulse = 0;                     // 初始占空比
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; // 输出极性：高电平有效
    
    TIM_OC1Init(TIM10, &TIM_OCInitStructure);                 

    TIM_Cmd(TIM10, ENABLE);                                     
}

/**
 * @brief  强制把 PB5 配置为普通 IO，并拉低/拉高进行测试
 * @param  mode: 0 = 强制低电平 (尝试加热), 1 = 强制高电平 (尝试停止)
 */
void Test_GPIO_PB5(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    // 1. 确保时钟已开启（保险起见再开一次）
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

    // 2. 先将 PB5 配置为“推挽输出”模式（注意：是 GPIO_Mode_OUT，不是 AF）
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;   // 【关键】改为普通输出模式
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;   // 推挽输出
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;     // 上拉
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    
    GPIO_ResetBits(GPIOB, GPIO_Pin_5); 
    
    
}


















/*PWM*/
void TIM2_DEVICE(int per,int psc)
{
  GPIO_InitTypeDef        GPIO_InitStructure;
  TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
  TIM_OCInitTypeDef       TIM_OCInitStructure;
  
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);
  
  GPIO_PinAFConfig(GPIOA,GPIO_PinSource1,GPIO_AF_TIM2);
  GPIO_PinAFConfig(GPIOA,GPIO_PinSource0,GPIO_AF_TIM2);
  GPIO_PinAFConfig(GPIOA,GPIO_PinSource3,GPIO_AF_TIM2);
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource2,GPIO_AF_TIM2);
	
  GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_0 | GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_High_Speed;
	GPIO_Init(GPIOA,&GPIO_InitStructure);
  
  TIM_TimeBaseInitStructure.TIM_Period        = per;
  TIM_TimeBaseInitStructure.TIM_Prescaler     = psc;
  TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
  TIM_TimeBaseInitStructure.TIM_CounterMode   = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM2,&TIM_TimeBaseInitStructure);
  
  TIM_OCInitStructure.TIM_OCMode      = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OCPolarity  = TIM_OCPolarity_High;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse       = 0;
  
  TIM_OC1Init(TIM2,&TIM_OCInitStructure);
  TIM_OC2Init(TIM2,&TIM_OCInitStructure);
	TIM_OC3Init(TIM2,&TIM_OCInitStructure);
 	TIM_OC4Init(TIM2,&TIM_OCInitStructure);
	
  TIM_OC1PreloadConfig(TIM2,TIM_OCPreload_Enable);
  TIM_OC2PreloadConfig(TIM2,TIM_OCPreload_Enable);
	TIM_OC3PreloadConfig(TIM2,TIM_OCPreload_Enable);
	TIM_OC4PreloadConfig(TIM2,TIM_OCPreload_Enable);
  
  TIM_ARRPreloadConfig(TIM2,ENABLE);
  
  TIM_Cmd(TIM2,ENABLE);
}

void SERVO_INIT(TIM_TypeDef* TIM,uint32_t TIM_out,uint8_t Compare)
{
	switch(Compare)
	{
		case 1:
		{
			TIM_SetCompare1(TIM,TIM_out);
		}break;
		case 2:
		{
			TIM_SetCompare2(TIM,TIM_out);
		}break;
		case 3:
		{
			TIM_SetCompare3(TIM,TIM_out);
		}break;
		case 4:
		{
			TIM_SetCompare4(TIM,TIM_out);
		}break;
	}
}
