#include "comm_task.h"
#include "STM32_TIM_BASE.h"
#include "comm_8010_task.h"
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

#include "bsp_can.h"
#include "pid.h"
#include "motor_task.h"
#include "motor_8010.h"
/***********************************

利用CAN发送函数向各个单元发送控制电流

***********************************/
UBaseType_t can_stack_surplus;
int16_t CAN1_current[9];
int16_t CAN2_current[9];
float target_pitch1_angle=0;
uint8_t ifflag[2]={0};
extern float pitch1_grav_torque;
extern float pid_chassis_pit_current;   //这是陀螺仪闭环的电流值
extern float Current_1010B[2];
float DM10010_l_output_angle=0;
extern uint8_t DM_ENABLE_flag;
extern float pitch3_cur;

void can_msg_send_task(void *parm)
{
	uint32_t Signal;
	BaseType_t STAUS;
  uint32_t comm_time = osKernelSysTick();

	
  while(1)
  {
				 STAUS = xTaskNotifyWait((uint32_t) NULL, 
													 (uint32_t) CHASSIS_MOTOR_MSG_SIGNAL | \
																			MODE_SWITCH_MSG_SIGNAL | \
																			INFO_SEND_MOTOR_SIGNAL ,
								(uint32_t *)&Signal, 
		 					(TickType_t) portMAX_DELAY );
		   if(STAUS == pdTRUE)
    {				
			/********************************************************
			can1底盘   can2机械臂		
			********************************************************/
			  //重新整合发送函数，结构更加合理并更好搭配motor.c
					 if(Signal & CHASSIS_MOTOR_MSG_SIGNAL)//发送底盘电流
		  {
   			//	   send_can1_low_cur(CAN1_current[1],CAN1_current[2],CAN1_current[3],CAN1_current[4]);
   			//	Motor10010B_Current(Current_1010B[0],Current_1010B[1],0,0);					
			}
					 if(Signal & INFO_SEND_MOTOR_SIGNAL)//发送除底盘电机的电流
			{
								 if(DM_ENABLE_flag == 1)
								 {			 
									   // DM_Enable();								 
									 	 DM_Enable_Send(0x05);
									 vTaskDelayUntil(&comm_time,1);		
										 DM_Enable_Send(0x04);
									 vTaskDelayUntil(&comm_time,1);
										 DM_CAN1_Enable_Send(0x03);
									 vTaskDelayUntil(&comm_time,1);
										 DM_Enable_Send(0x01);
		 							 vTaskDelayUntil(&comm_time,1);
								     DM_Enable_Send(0x00);
		 							 vTaskDelayUntil(&comm_time,1);		 								 
									   DM_ENABLE_flag = 0;
								 }
						 
				  //DM_Dis_Enable();
//							 
		  			send_6020_can1_high_cur(0,CAN1_current[7],0);   //6020
                                         								             //   速度控制：达妙应该是可以用斜坡函数的，但记得量纲要做好  切记角度控制用的是弧度为单位 ！！！			   			   
		  	//	DM_MIT_send(&Motor[Pitch1]);		
		    //  DM_Dis_Enable_Send(0x01);
			//			DM_Dis_Enable_Send(0x03);		 
						Motor10010L_Position(&Motor[Pitch1]);	 		 
  					DM_MIT_send(&Motor[Roll2]);	
		  			DM_MIT_send(&Motor[Pitch3]);		  
            DM_MIT_send(&Motor[Grip]);		
            DM_MIT_send(&Motor[Big_Yaw]);		
        		
			 
			}
      			if(Signal & MODE_SWITCH_MSG_SIGNAL)//关闭遥控 -- 发送-电流为零
			{      
				  if(DM_ENABLE_flag == 1)
					 {
						 DM_Dis_Enable();
						 DM_CAN1_Dis_Enable_Send(0x03);
					 }
						send_can1_low_cur(CAN1_current[1],CAN1_current[2],CAN1_current[3],CAN1_current[4]);
					  send_can1_high_cur(CAN1_current[5],CAN1_current[6],CAN1_current[7]);
						send_gear_can2_low_cur(CAN2_current[0],CAN2_current[1],CAN2_current[2],CAN2_current[3]);
					  send_6020_can2_high_cur(CAN2_current[4],CAN2_current[5],CAN2_current[6]);
						Motor10010B_Current(0,0,0,0);
				
			}				
				
			
			

			  
			  //vTaskDelayUntil(&mode_switch_wake_time, 100);
			   /*查询自身堆栈的高水位  判断堆栈是否溢出*/			
	      //can_stack_surplus = uxTaskGetStackHighWaterMark(NULL);//会占用时间 一般在调试的时候使用就好
        //检查PRIMASK是否屏蔽了可屏蔽中断
					 if (__get_PRIMASK() == 1) {
								 ifflag[0]=1;
						}

					 // 检查 FAULTMASK 是否屏蔽了所有中断（包括 NMI 和 HardFault）
					 if (__get_FAULTMASK() == 1){
								 ifflag[1]=1;
						}
	  }
  }	
} 



