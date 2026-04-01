#include "STM32_TIM_BASE.h"
#include "info_get_task.h"
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "math.h"
#include "bsp_can.h"
#include "pid.h"
#include "motor_8010.h"
#include "comm_8010_task.h"
#include "comm_task.h"
#include "modeswitch_task.h"
#include "comm_8010_task.h"
#include "motor_task.h"
#include "bsp_vofa.h"
#include "General_function.h"

float target_angle[2];
float receive_angle[2];
float tete=1;
extern TaskHandle_t motor_Task_Handle;
extern float GO8010_init_angle1;
extern float pitch2_grav_torque;

void coom_8010_task(void *parm)
{
	uint32_t Signal;
	BaseType_t STAUS;
	uint32_t comm_8010_time = osKernelSysTick();

	while (1)
	{
		STAUS = xTaskNotifyWait((uint32_t)NULL, // 等待前清零指定任务通知值的比特位（旧值对应bit清0）
								(uint32_t)INFO_SEND_MOTOR_SIGNAL | MODE_SWITCH_MSG_SIGNAL|GO_8010_INIT_SIGNAL,
								(uint32_t *)&Signal,		// 用来取出通知值（如果不需要取出，可设为NULL）
								(TickType_t)portMAX_DELAY); // 设置阻塞时间
		if (STAUS == pdTRUE)
		{

			   pitch2_grav_torque = pitch2_grav_torque_calculate();
			   pitch2_grav_torque=constrain(pitch2_grav_torque,-1.42,1.43);		  
	//		  JustFloat_Send(vofa_debug,6,UART7);   //vofa的发送，用大臂的时候记得注释掉

			 
			if (Signal == MODE_SWITCH_MSG_SIGNAL) 
			{				
			   go_8010_test_tesk1(GO_8010_1,0,0,0,0,0);		
			}
			if (Signal == INFO_SEND_MOTOR_SIGNAL) //motor_task 角度的赋值最终是传到了这里
			{				
					ref_8010_angle(target_angle);
					get_8010_angle(receive_angle);

				//id，力矩，位置，角速度，p，d   使用的是位置模式
       go_8010_test_tesk1(GO_8010_1,pitch2_grav_torque,tamp_task(receive_angle[0],tete,target_angle[0]),0,1,0.06);   //加这个斜坡函数一定要确保量纲一致，切记切记！！！   1， 0.06 -- 完整形态的pd参数
	
				//   go_8010_init(GO_8010_1,0.05);
			  //   go_8010_test_tesk1(GO_8010_1,0,0,0,0,0);
		 // 	 go_8010_test_tesk1(GO_8010_1,0,0,-0.27*6.33,0.02,0);  //  控制速度0.11(空转)  d别给0.18以上，会剧烈震荡，因为超出限幅了。	
				//	vTaskDelayUntil(&comm_8010_time, 2);
				
			}
		}		
	 }
}
 
/*初始化前馈补偿*/   //b=Izz3 + m3*(rx3^2 + ry3^2) + 2*m4*rz4*(rz4 + d4) + m4*(a4 + rx4)^2 + Iyy4   kg*m^2    仔细想了一下，加速度前馈补偿就先放一下吧，目前的机械臂只是测试板，后面ok了在来进行补偿吧，顺便带带小灯吧
void initFeedforwardParam(FFC *vFFC, float a, float b)
{

		vFFC->a = a;
		vFFC->b = b;
		vFFC->lastRin = 0;
		vFFC->perrRin = 0;
		vFFC->rin = 0;
}

/*实现前馈控制器(惯性补偿)*/   //前馈微分，这里的参数a就是关于关节速度的系数，参数b就是关于关节加速度的系数
float getFeedforwardControl(FFC *vFFC, float v) // yaw轴
{
		vFFC->rin = v;
		float result = vFFC->a * (vFFC->rin - vFFC->lastRin) + vFFC->b * (vFFC->rin - 2 * vFFC->lastRin + vFFC->perrRin);
		vFFC->perrRin = vFFC->lastRin;
		vFFC->lastRin = vFFC->rin;
		return result;
}   
  
float pitch2_grav_torque_calculate(void)
{
     float q_int2,q_int3,q_int6;    
     float m7,m6,m4,m3,rz7,ry6,rz4,ry4,rx4,ry3,rx3,d7,d5,d4,d45,m67;
     float q_6,q_3,q_2,q_4;
     float toq=0; 
	
	   float lm_56,lm_34 = 0;

     m7=0.89679; m6=0.5359;m4=0.59967; m3=0.70583;m67 = 1.4327;  
     rz7=0.062165; ry6=0.05038; rz4=0.057519; ry4=0.003381; rx4=0.002649; ry3=0.064597; rx3=0.004006; 
	   d7=0.057; d5=0.14; d4=0.16;  d45=0.3;
	 
	   lm_56 = 0.13 ;  //m7*d7+m7*rz7+m6*ry6;  0.0941904054
	   lm_34 = 0.30228;        //m3*ry3+m4*(rz4+d4)    0.17603411924 
	
	   q_int2 = 0.366519; q_int3 = -0.353786;	
     q_int6 = 0.15;
	
	   q_2 = q_int2 - Motor[Pitch1].Brushless.angle_fdb;  //Motor[Pitch1].Brushless.angle_fdb
		 q_3 =(Motor[Pitch2].Brushless.angle_fdb-GO8010_init_angle1)/6.33 + q_int3;	 
	   q_4 =-Motor[Roll2].Brushless.angle_fdb;
     q_6 = Motor[Pitch3].Brushless.angle_fdb+q_int6;
	
     toq = 9.80f/-6.33*(cos(q_2+q_3)*(lm_34+(lm_56)*cos(q_6)+m67*d45) + sin(q_2+q_3)*(m4*ry4*sin(q_4)+(lm_56)*cos(q_4)*sin(q_6)));
     
	   return toq;
}
 

float pitch2_friction_fitting(float pitch2_speed_fdb)//摩擦力补偿
{

   float a=0;
   static float f_torque2 =0;

	 
	  if(pitch2_speed_fdb>0.2152f){
		    a=0.015638815f;
		}
		else if(pitch2_speed_fdb<-0.2152f){
		    a=-0.015638815f;
		}
		else if((-0.2152f<= pitch2_speed_fdb) && (pitch2_speed_fdb <=0.2152f)){
			  a=0;
		}
		
    f_torque2=a;

    return f_torque2;
}


void go_8010_test_tesk1(int id, float T, float Pos, float W, float K_P, float K_W)
{
	GO_M8010_send_data(id, T, Pos, W, K_P, K_W); // 计算发数
	DMA_Cmd(DMA2_Stream1, ENABLE);
	DMA_Cmd(DMA2_Stream6, ENABLE);
}

 

void go_8010_init(int id,float torque)
{
	GO_M8010_send_data(id, torque, 0, 0, 0, 0);
	DMA_Cmd(DMA2_Stream1, ENABLE);
	DMA_Cmd(DMA2_Stream6, ENABLE);
	
}
int go_8010_stop(int id)
{
	GO_M8010_send_data(id, 0, 0, 0, 0, 0);
	DMA_Cmd(DMA2_Stream1, ENABLE);
	DMA_Cmd(DMA2_Stream6, ENABLE);
	return 1;
}



void ref_8010_angle(float get_angle[])
{
	for(int ID=0;ID<Motor_count;ID++)
	{
		switch(Motor[ID].Brushless.GO_ID)
		{
			case GO_8010_1:get_angle[0]=Motor[ID].Brushless.angle_ref+GO8010_init_angle1;break;  //GO8010_init_angle1：对零点进行补偿
			
			case GO_8010_2:get_angle[1]=Motor[ID].Brushless.angle_ref;break;
			
			
		}
	}
}

void get_8010_angle(float receive_angle[])
{
	for(int ID=0;ID<Motor_count;ID++)
	{
		switch(Motor[ID].Brushless.GO_ID)
		{
			case GO_8010_1:receive_angle[0]=Motor[ID].Brushless.angle_fdb;break;
			
			case GO_8010_2:receive_angle[1]=Motor[ID].Brushless.angle_fdb;break;
			
		}
	}
}




void go_8010_init_task(int GO_ID,float speed)
{
	 go_8010_init(GO_ID,speed);
	for(int ID=0;ID<Motor_count;ID++)
	{
		 if(Motor[ID].Brushless.GO_ID==GO_ID)
		{
			 if(Motor[ID].Brushless.offset_angle_init_flag == 0)
			{
			   
			   if(Motor[ID].Brushless.angle_fdb-Motor[ID].Brushless.l_angle_fdb==0)
				{
					Motor[ID].Brushless.err_count++;
	          if(Motor[ID].Brushless.err_count>300)   
					{
	     	    Motor[ID].Brushless.offset_angle_init_flag = 1;
	     	    Motor[ID].Angle.offset_angle=Motor[ID].Brushless.angle_fdb;		
	    
					}						
				}
			}
      else if(Motor[ID].Brushless.offset_angle_init_flag == 1)
			{
				go_8010_stop(GO_ID);
			}				
		}
	}
}



//	 float current_angle;
//   float angle1,angle2,angle3,angle4;
//	
//   current_angle=-(pitch2_angle_fdb-pitch2_init_angle)/6.33+16*6.28/360-pitch1_angle_fdb*6.28/360;
//	 pitch2_angle=current_angle;
//	
//	
//	 
//	 angle1=cos(current_angle);
//	 angle2=angle1*angle1;
//	 angle3=angle1*angle1*angle1;
//	 angle4=angle1*angle1*angle1*angle1;
//	 
//	 pitch2_cos_angle[1]=cos(current_angle);
//	       if(current_angle >= 3.14){
//			  current_angle=current_angle-3.14;
//			}
//    else if(current_angle < -3.14){
//			  current_angle=current_angle+3.14;
//			}
//			
//			  last_torque2 = torque2;

			
			
//   if(current_angle>0)
//      torque2= -0.1873*angle4+0.2173*angle3+0.1284*angle2-0.7071 *angle1-0.1848;
//   else if(current_angle<0)
//	     torque2= -1.033*angle1+0.4034;
//	
//			torque2 = tamp_task(last_torque2,0.07,torque2);	

 // torque= -1.063*angle1+0.4034;
 
 
 
 
    //torque=  -0.1873*angle4+0.2173*angle3+0.1284*angle2-0.7071 *angle1-0.1848;

// float pitch2_torque_calcuate (float pitch2_init_angle,float pitch2_angle_fdb,float pitch1_angle_fdb)
//{

//   float torque;
//	 float current_angle;

//	 current_angle=-(pitch2_angle_fdb-pitch2_init_angle)/6.33+16*6.28/360-pitch1_angle_fdb*6.28/360;
//	 pitch2_cos_angle[0]=cos(current_angle);
//   torque=0.67*cos(current_angle);
//   return torque;

//}
// 
 
 
