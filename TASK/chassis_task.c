#include "chassis_task.h"
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "modeswitch_task.h"
#include "comm_task.h"
#include "info_get_task.h"
#include "pid.h"
#include "sys_config.h"
#include "math.h"
#include "pc_rx_data.h"
#include "remote_ctrl.h"
#include "keyboard.h"
#include "bsp_can.h"
#include "bsp_vofa.h"
#include "keyboard.h"
#include "struct_typedef.h"
#include "power_control.h"
#include "General_function.h"
/*********************24注释****************************
	根据过去的设计习惯，x是前后y是左右

**************************************************/
UBaseType_t chassis_stack_surplus;
extern TaskHandle_t can_msg_send_Task_Handle;

Direction_t Last_chassi_direction=AHEAD_BACK;
Direction_t Now_chassi_direction=AHEAD_BACK;

int16_t exchange_x_angle_move =70;
int16_t exchange_y_angle_move;

uint8_t chassis_speed_or_angle_flag=0;
extern fp32 INS_angle_final[3];   //转换单位后的角度

/*底盘运动变量*/
chassis_t chassis;
float pid_chassis_pit_current=0;

float chassis_pid[6] = {40.0f, 0.05f, 0.0f, 4.5f, 0.05, 0};
float chassis_pit_pid[3] = {200.0, 0.0f ,0.1f};
float chassis_joint_leg_pid[6]={520,0,900,0,0,0};    //前面是角度环，后面是速度环   
float Current_1010B[2]={0};
float joint_leg_cur_G[2] = {0};

u8 chassis_retract_leg_flag = 0;

void chassis_task(void *parm)
{
  uint32_t Signal;
	BaseType_t STAUS;
  
  while(1)
  {
    STAUS = xTaskNotifyWait((uint32_t) NULL, 
										        (uint32_t) INFO_GET_CHASSIS_SIGNAL, 
									        	(uint32_t *)&Signal, 
									        	(TickType_t) portMAX_DELAY );
		extern u8 chassis_leg_flag;
      if(STAUS == pdTRUE)
    {
			   if(Signal & INFO_GET_CHASSIS_SIGNAL)
      {							  
				  chassis_param_init();			
  				rm.pit_leg=constrain(rm.pit_leg, -4.4 - chassis.up_angle,48.85 - chassis.up_angle);   //限制角度      //左边的要正值，右边的要负值
				 

					chassis.joint_leg_angle_ref[left_joint] =  -4.4  - rm.pit_leg - chassis.up_angle; 
					chassis.joint_leg_angle_ref[right_joint]=   4.4  + rm.pit_leg + chassis.up_angle;    //电机闭环的角度  
				
				  chassis.joint_leg_angle_ref[left_joint]  = constrain(chassis.joint_leg_angle_ref[left_joint],-46.85,1.82);     //这个限制函数记得前面的数小后面的数大
				  chassis.joint_leg_angle_ref[right_joint] = constrain(chassis.joint_leg_angle_ref[right_joint],-1.82,46.85);
							            		
           if(chassis_mode != CHASSIS_RELEASE )//云台归中之后底盘才能动 
        {
							switch(chassis_mode)
				  {

								case CHASSIS_NORMAL_MODE:
  							{
									chassis_ahead_to_back();
									chassis_retract_leg_flag = 0;
								}break;	

								case CHASSIS_GET_ENERGY_UNIT_MODE:
							  case CHASSIS_EXCHANGE_MODE:
								case CHASSIS_GET_ENERGY_UNIT1_MODE:
								case CLASSIS_PRIMARY_MODE:	
								case CHASSIS_SECONDDARY_ORE_MODE:
                case CHASSIS_GET_ENERGY_UNIT2_MODE:
								case CHASSIS_LEG_DOWN_MODE:
  							{
									chassis_ahead_to_back();
								}break;	
                case CHASSIS_LEG_UP_MODE:
 	              {
									chassis_ahead_to_back();
									 
  									if(clamp_action == ACTION_TWO)
									{
									     chassis_retract_leg_flag = 1;
										   chassis.up_angle = 0;
										   rm.pit_leg = 0;
									}
								}break;	
      
								/*底盘停止模式*/
								case CHASSIS_STOP_MODE:
								{
									chassis_stop_handler();
								}break;
								default:
								{
									chassis_stop_handler();
								}break;
						 						
					}
             mecanum_calc(chassis.vx, chassis.vy, chassis.vw, chassis.wheel_spd_ref);
							
							if(chassis_speed_or_angle_flag==0)//速度环，加上功率限制
						{
								for (int i = 0; i < 4; i++)
								{
									chassis.current[i] = pid_calc(&pid_spd[i], chassis.wheel_spd_fdb[i], chassis.wheel_spd_ref[i]);
									chassis.wheel_angle_offset[i]=chassis.wheel_angle_fdb[i];//刷新角度环				
						    }
						}
							else if(chassis_speed_or_angle_flag==1)//角度环
					  {
								for (int i = 0; i < 4; i++)
								{
									chassis.wheel_spd_ref[i]=pid_calc(&pid_chassis_angle,chassis.wheel_angle_fdb[i],chassis.wheel_angle_ref[i]);
									chassis.current[i] = pid_calc(&pid_spd[i], chassis.wheel_spd_fdb[i], chassis.wheel_spd_ref[i]);
								}
					  }					
						    for(int i = 0; i<2; i++)//底盘腿闭环 
						{ 
      			//	 joint_leg_G_torque();		 
						  chassis.leg_current[i] = pid_calc(&pid_chassis_joint_leg_angle[i],chassis.joint_leg_angle_fdb[i],chassis.joint_leg_angle_ref[i]);
						}							

              if(chassis_retract_leg_flag ==0)
					 {						
							Current_1010B[0]= chassis.leg_current[0];
						  Current_1010B[1]= chassis.leg_current[1];						
					 }
					    else
					 {
					          Current_1010B[0] = -900;     //配合收腿的动作
					          Current_1010B[1] = 900;   
					 }
								//		Chassis_Power_Control(&chassis);  功率控制，还没实践过						 												
							//底盘暂时不改，因此底盘can变动还需手动调整
							memcpy(&CAN1_current[1], chassis.current, sizeof(chassis.current));																																	
		   }				
					else
			{
						memset(&CAN1_current[1], 0, sizeof(chassis.current));//不出意外的话只会操作低8个字节
			      Current_1010B[0]=0;
					  Current_1010B[1]=0;
						rm.pit_leg=0;

			}
       
						xTaskGenericNotify( (TaskHandle_t) can_msg_send_Task_Handle, 
											(uint32_t) CHASSIS_MOTOR_MSG_SIGNAL, 
											(eNotifyAction) eSetBits, 
											(uint32_t *)NULL );
      }
    }       chassis_stack_surplus = uxTaskGetStackHighWaterMark(NULL);   
  }
}

float G_comper(float angle)
{

	float iq;
	float	angle_4 = angle*angle*angle*angle;
	float angle_3 = angle*angle*angle;
	float angle_2	= angle*angle;
	float width = 0.4f;
	    iq = 0.12944760941*angle_4
					-0.0425*angle_3
					+1.9726*angle_2
					+ -111.3105*angle
					+ 60.36306411;
	
	return iq;
	
}

float tilt_angle_fitting (float joint_angle)
{
	float tilt_angle = 0;
	float angle_2	= joint_angle*joint_angle;
	

 tilt_angle =	-0.0114*angle_2
					    +1.2137*joint_angle
	            +0.1390 ;
	return tilt_angle;
	
}

float a;
float tilt_cos_angle_fitting (float joint_angle)
{
	float cos_tilt_angle = 0;
	float l1,L;
  l1 = 190.4; L = 343.07;
	a = l1*sin((joint_angle+30)*3.14/180) - 0.5*l1;
	
	cos_tilt_angle = sqrt(L*L-a*a)/L ;
	return cos_tilt_angle;
	
}


float Gravity_compensation(float angle,float w)
{
	float iq;
	float	angle_4 = angle*angle*angle*angle;
	float angle_3 = angle*angle*angle;
	float angle_2	= angle*angle;
	float width = 0.4f;
	float iq_up = 0.12944760941*angle_4
					-0.0425*angle_3
					+1.9726*angle_2
					+ -111.3105*angle
					+ 60.36306411;

	
	
	float iq_down = -0.001862f*angle_4
					+ 0.02706f*angle_3
					+ 0.3094f*angle_2
					- 3.559f*angle
					+ 1791;
	
	float b = (iq_up + iq_down)/2;
	float k = (iq_up - b)/width;
	if(w >=0 )
	{
		if(w <= width)
			iq = b + k*w;
		else
			iq = iq_up;
	}
	else if(w < 0)
	{
		if(w >= -width)
			iq = b + k*w;
		else
			iq = iq_down;
	}
	return iq;
}



void joint_leg_G_torque(void)
{

  float joint_angle[2] = {0};
	joint_angle[0] = (chassis.joint_leg_angle_fdb[0] + 30)*3.14/180;
	joint_angle[1] = (-chassis.joint_leg_angle_fdb[1]+ 30)*3.14/180;
	
 // cos_tilt_angle = 	tilt_cos_angle_fitting(chassis.joint_leg_angle_fdb[0]);
  //  tilt_angle = tilt_angle_fitting(chassis.joint_leg_angle_fdb[0]);
	
//	joint_leg_cur_G[0] =  2213.6249 * cos(joint_angle[0]) *1.1f * cos(tilt_angle*3.14/180);    //(171.5+12)*270/343*(218-7)/436* (N) 0.19*2 (N*m) / 1.2(A) *100 ()
//	joint_leg_cur_G[1] = -2360.5 * cos(joint_angle[1]) * 1.1f  *  cos(tilt_angle*3.14/180);    //(171.5+12)*270/343*(218-7)/436* (N) 0.19*2 (N*m) / 1.2(A) *100 ()

}

void chassis_Trajectory_running(uint32_t t,double a0,double a1,double a2,double a3)
{
   uint32_t t2,t3;
   t2 = t*t;
	 t3 = t*t*t;
	chassis.up_angle = a0+a1*t+a2*t2+a3*t3;

}


uint32_t Turn_Record_times;
static uint8_t chassis_Auto_Turn()   //自动转向的功能函数 （once）
{
	uint8_t Turn_ready_flag=0;
	int16_t Turn_speed=300;
	if(Now_chassi_direction!=Last_chassi_direction)
	{
		Turn_Record_times=HAL_GetTick();
		Last_chassi_direction=Now_chassi_direction;
	}
	if(HAL_GetTick()-Turn_Record_times <= 700 && HAL_GetTick()>10000)
	{
		switch(Now_chassi_direction)
		{
			case AHEAD_BACK:
			{
				chassis.vw= -Turn_speed;
			}break;
			case AHEAD_FRONT:
			{
				chassis.vw= +Turn_speed;
			}break;
		}
		Turn_ready_flag=0;
	}
	else 
		Turn_ready_flag=1;
	return Turn_ready_flag;
}
static void chassis_ahead_to_back(void) //倒着开的功能函数 （once）
{
	#ifdef BACK_DRIVE
	Now_chassi_direction=AHEAD_BACK;
	chassis.vy = +(rm.vx * CHASSIS_RC_MOVE_RATIO_X + km.vx * CHASSIS_KB_MOVE_RATIO_X);
  chassis.vx = +(rm.vy * CHASSIS_RC_MOVE_RATIO_Y + km.vy * CHASSIS_KB_MOVE_RATIO_Y);
	if(chassis_Auto_Turn() || NOT_AUTO_TURN)
		chassis.vw = -rm.vw * CHASSIS_RC_MOVE_RATIO_R ;
	#else
		chassis_ahead_to_front();
	#endif

}
static void chassis_ahead_to_front(void)
{
	Now_chassi_direction=AHEAD_FRONT;
	chassis.vy = -(rm.vx * CHASSIS_RC_MOVE_RATIO_X + km.vx * CHASSIS_KB_MOVE_RATIO_X);
  chassis.vx = -(rm.vy * CHASSIS_RC_MOVE_RATIO_Y + km.vy * CHASSIS_KB_MOVE_RATIO_Y);
	#ifdef BACK_DRIVE
	if(chassis_Auto_Turn() || NOT_AUTO_TURN)
	#endif
	chassis.vw = -(rm.vw * CHASSIS_RC_MOVE_RATIO_R+ km.vw * CHASSIS_KB_MOVE_RATIO_R);
}
/*********************************************
	底盘角度环模式控制处理
***********************************************/
static void chassis_exchange_angle_handler(void)
{
	chassis.wheel_angle_ref[0]=(chassis.wheel_angle_offset[0]+exchange_y_angle_move-exchange_x_angle_move)*ANGLE_RATIO_FR;
	chassis.wheel_angle_ref[1]=(chassis.wheel_angle_offset[1]+exchange_y_angle_move+exchange_x_angle_move)*ANGLE_RATIO_FL;
	chassis.wheel_angle_ref[2]=(chassis.wheel_angle_offset[2]-exchange_y_angle_move+exchange_x_angle_move)*ANGLE_RATIO_BR;
	chassis.wheel_angle_ref[3]=(chassis.wheel_angle_offset[3]-exchange_y_angle_move-exchange_x_angle_move)*ANGLE_RATIO_BL;
	chassis_speed_or_angle_flag=1;
}
static void chassis_stop_handler(void)
{
  chassis.vy = 0;
  chassis.vx = 0;
  chassis.vw = 0;
}

void chassis_param_init(void)
{    
        	/*底盘vw旋转的pid*/
    PID_Struct_Init(&pid_chassis_angle,chassis_pid[0],chassis_pid[1],chassis_pid[2],1000, 50, DONE);				
				                        /*狗腿pid*/
    PID_Struct_Init(&pid_chassis_pit_angle,chassis_pit_pid[0],chassis_pit_pid[1],chassis_pit_pid[2],8000, 1200, DONE);			    
					/*底盘vx,vy平移的pid*/
		for(int i = 0; i < 4; i++)
	{
		PID_Struct_Init(&pid_spd[i],chassis_pid[3],chassis_pid[4],chassis_pid[5],10000, 2000, DONE);
	}											            
	  PID_Struct_Init(&pid_chassis_joint_leg_angle[0],chassis_joint_leg_pid[0],chassis_joint_leg_pid[1],chassis_joint_leg_pid[2],3600,200,DONE);
		PID_Struct_Init(&pid_chassis_joint_leg_spd[0],chassis_joint_leg_pid[3],chassis_joint_leg_pid[4],chassis_joint_leg_pid[5],3600,400,DONE);							
		PID_Struct_Init(&pid_chassis_joint_leg_angle[1],chassis_joint_leg_pid[0],chassis_joint_leg_pid[1],chassis_joint_leg_pid[2],3600,200,DONE);
		PID_Struct_Init(&pid_chassis_joint_leg_spd[1],chassis_joint_leg_pid[3],chassis_joint_leg_pid[4],chassis_joint_leg_pid[5],3500,400,DONE);			
}
/**
  * @brief mecanum chassis velocity decomposition  麦克纳姆轮解算
  * @param input : ↑=+vx(mm/s)  ←=+vy(mm/s)  ccw=+vw(deg/s)
  *        output: every wheel speed(rpm)
	* @trans 输入：		前后左右的量
	*				 输出：		每个轮子对应的速度
  * @note  1=FR 2=FL 3=BL 4=BR
	* @work	 分析演算公式计算的效率
  */
int rotation_center_gimbal = 0;  //我们默认机械臂在中心，观察情况是否符号预期在调整参数
static void mecanum_calc(float vx, float vy, float vw, int16_t speed[])
{
  static float rotate_ratio_fr;
  static float rotate_ratio_fl;
  static float rotate_ratio_bl;
  static float rotate_ratio_br;
  static float wheel_rpm_ratio;
  
  taskENTER_CRITICAL();
    if (rotation_center_gimbal)
    {
      chassis.rotate_x_offset = glb_struct.gimbal_x_offset;    //将云台（我们的是机械臂）的距离底盘中心的距离考虑进去
      chassis.rotate_y_offset = glb_struct.gimbal_y_offset;
    }
    else 
    {
      chassis.rotate_x_offset = 0;
      chassis.rotate_y_offset = 0;
    }
  rotate_ratio_fr = ((glb_struct.wheel_base+glb_struct.wheel_track)/2.0f \
                      - chassis.rotate_x_offset + chassis.rotate_y_offset)/RADIAN_COEF;
  rotate_ratio_fl = ((glb_struct.wheel_base+glb_struct.wheel_track)/2.0f \
                      - chassis.rotate_x_offset - chassis.rotate_y_offset)/RADIAN_COEF;
  rotate_ratio_bl = ((glb_struct.wheel_base+glb_struct.wheel_track)/2.0f \
                      + chassis.rotate_x_offset - chassis.rotate_y_offset)/RADIAN_COEF;
  rotate_ratio_br = ((glb_struct.wheel_base+glb_struct.wheel_track)/2.0f \
                      + chassis.rotate_x_offset + chassis.rotate_y_offset)/RADIAN_COEF;     //通过具体的参数估算出每个轮子该输出的速度

  wheel_rpm_ratio = 60.0f/(glb_struct.wheel_perimeter*CHASSIS_DECELE_RATIO);
  taskEXIT_CRITICAL();//离开中断豁免区
  

  VAL_LIMIT(vx, -MAX_CHASSIS_VX_SPEED, MAX_CHASSIS_VX_SPEED);  //mm/s         
  VAL_LIMIT(vy, -MAX_CHASSIS_VY_SPEED, MAX_CHASSIS_VY_SPEED);  //mm/s         
  VAL_LIMIT(vw, -MAX_CHASSIS_VR_SPEED, MAX_CHASSIS_VR_SPEED);  //deg/s         


  int16_t wheel_rpm[4];
  float   max = 0;
  
  wheel_rpm[0] = (-vx - vy - vw * rotate_ratio_fr) * wheel_rpm_ratio;
  wheel_rpm[1] = ( vx - vy - vw * rotate_ratio_fl) * wheel_rpm_ratio;
  wheel_rpm[2] = ( vx + vy - vw * rotate_ratio_bl) * wheel_rpm_ratio;
  wheel_rpm[3] = (-vx + vy - vw * rotate_ratio_br) * wheel_rpm_ratio;

  //find max item
  for (uint8_t i = 0; i < 4; i++)
  {
    if (abs(wheel_rpm[i]) > max)
      max = abs(wheel_rpm[i]);
  }
  //equal proportion
  if (max > MAX_WHEEL_RPM)
  {
    float rate = MAX_WHEEL_RPM / max;
    for (uint8_t i = 0; i < 4; i++)
      wheel_rpm[i] *= rate;
  }
  memcpy(speed, wheel_rpm, 4*sizeof(int16_t));
}
