#include "info_get_task.h"
#include "STM32_TIM_BASE.h"
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "comm_task.h"
#include "keyboard.h"
#include "remote_ctrl.h"
#include "bsp_can.h"
#include "bsp_vofa.h"
#include "chassis_task.h"
#include "modeswitch_task.h"
#include "imu_task.h"
#include "sys_config.h"
#include "pc_rx_data.h"
#include "stdlib.h"
#include "math.h"
#include "controller.h"
#include "motor_task.h"
#include "motor_8010.h"
#include "inverse_Kinematics.h"


UBaseType_t info_stack_surplus;

extern TaskHandle_t can_msg_send_Task_Handle;
extern TaskHandle_t chassis_Task_Handle;
extern GO_Motorfield motor_recevie;
extern TaskHandle_t motor_Task_Handle;
extern TaskHandle_t comm_8010_task_Handle;

uint8_t poweren_flag=0;
float GO8010_init_angle1=0;    //8010的初始偏移值，在后续角度赋值中减去，起到零点的作用
//接收由modeswitch_task的通知
extern float pitch2_cos_angle[2];
extern imu_data_t imu;
extern fp32 INS_angle_final[3] ;
extern fp32 INS_palstance_final[3]; //角速度
extern chassis_t chassis;
extern float pid_chassis_pit_current;
extern float chassis_joint_leg_current[2];
extern float lm2 ;
extern float Current_1010B[2];
extern float speed_filtered[2];
extern float joint_leg_cur_G[2];

float big_yaw_grav_torque = 0;
float pitch1_grav_torque =  0;
float pitch2_grav_torque =  0;
float pitch3_grav_torque =  0;
float roll1_grav_torque =   0;
float roll2_grav_torque =   0;
float roll3_grav_torque =   0;
extern float tilt_angle;
extern float cos_tilt_angle;
uint8_t DM_ENABLE_flag=0;

double doq[1][6] ;
//double T_target[4][4] ; 

//T_target[][] =   {   { -0.6625,  0.4786, -0.5756, -0.0135 },
//							{  0.2291,  0.8654,  0.4455,  0.0763 },
//							{  0.7133,  0.1481, -0.6850,  0.2464 },
//							{  0.0000,  0.0000,  0.0000,  1.0000 }  };

								
								    double T_target[4][4] = {
        { -0.6625,  0.4786, -0.5756, -0.0135 },
        {  0.2291,  0.8654,  0.4455,  0.0763 },
        {  0.7133,  0.1481, -0.6850,  0.2464 },
        {  0.0000,  0.0000,  0.0000,  1.0000 }
    };

void info_get_task(void *parm)
{
	uint32_t Signal;
	BaseType_t STAUS;
	
	
	
	while (1)
	{
		/*此函数用于获取通知值和清除通知值的指定位值*/
		STAUS = xTaskNotifyWait((uint32_t)NULL,					   // 等待前清零指定任务通知值的比特位
								(uint32_t)MODE_SWITCH_INFO_SIGNAL, // 等待成功后清零指定的任务通知值比特位
								(uint32_t *)&Signal,			   // 取出通知值
								(TickType_t)portMAX_DELAY);		   // 设置阻塞时间
		if (STAUS == pdTRUE)
		{
			if (Signal & MODE_SWITCH_INFO_SIGNAL)  //获取电机，遥控等的信息
			{
							
				taskENTER_CRITICAL();  //临界区
				Get_Motor_info();      //电机角度，速度的反馈值获取
				get_chassis_info();    //获取信息，如遥控器，键鼠等，对底盘进行控制  
				remote_ctrl_gimbal_hook();   //控制机械臂
				remote_ctrl_chassis_leg_hook();   //控制底盘的腿
				taskEXIT_CRITICAL();			
				get_global_last_info();  //获取上一次拨轮的值
		
	      vofa_debug[0]=Motor[Pitch1].Brushless.angle_ref;                              
				vofa_debug[1]=Motor[Pitch1].Brushless.angle_fdb;      
				vofa_debug[2]=Motor[Pitch1].Brushless.torque_fbd;			
				vofa_debug[3]=Motor[Pitch2].Brushless.spd_fdb;	   		
				vofa_debug[4]=Motor[Pitch2].Brushless.angle_ref + GO8010_init_angle1;		
			  vofa_debug[5]=Motor[Pitch2].Brushless.angle_fdb;
        gain_angle();					
        custom_arm_ik(T_target,doq) ;

				

 							
				if (global_mode == RELEASE_CTRL)  //给所有电机电流发0，起到断电的作用  写一下程序的启动逻辑：当电池一上电，infoget任务其实是一直在跑的，而只有遥控器右遥打到中间，它才会通知电机和底盘任务开始运作
				{
					memset(&CAN1_current, 0, sizeof(CAN1_current));
					memset(&CAN2_current, 0, sizeof(CAN2_current));
					pitch1_grav_torque=0;
					DM_ENABLE_flag = 1;

					xTaskGenericNotify((TaskHandle_t)can_msg_send_Task_Handle,
									   (uint32_t)MODE_SWITCH_MSG_SIGNAL,
									   (eNotifyAction)eSetBits,
									   (uint32_t *)NULL);

					xTaskGenericNotify((TaskHandle_t)comm_8010_task_Handle,
									   (uint32_t)MODE_SWITCH_MSG_SIGNAL,
									   (eNotifyAction)eSetBits,
									   (uint32_t *)NULL);
				}
				else                      
				{ // 通知数据处理函数运行   通知底盘及电机任务，进行初始化，电机角度赋值等

					xTaskGenericNotify((TaskHandle_t)chassis_Task_Handle,
									   (uint32_t)INFO_GET_CHASSIS_SIGNAL,
									   (eNotifyAction)eSetBits,
									   (uint32_t *)NULL);

					xTaskGenericNotify((TaskHandle_t)motor_Task_Handle,
									   (uint32_t)INFO_GET_MOTOR_SIGNAL,
									   (eNotifyAction)eSetBits,
									   (uint32_t *)NULL);
				}

		//		info_stack_surplus = uxTaskGetStackHighWaterMark(NULL);
			}
		}
	}
}

static void get_chassis_info(void)
{
	keyboard_chassis_hook();     //键鼠对底盘的控制
	remote_ctrl_chassis_hook();  //遥控器对底盘的控制
	get_structure_param();       //底盘轮子参数等的一些赋值
}


//@breif 这段代码的主要功能是 读取上位机（PC）发送的机器人结构参数，并进行有效性检查，最后更新到全局控制参数中。
//@breif 简单来说，就是允许用户在电脑端修改机器人的尺寸（如轮距、轮子周长）和云台位置，而不需要重新烧录代码，同时包含了防止参数设置错误的保护机制

//具体的与上位机的交互数据被我删了，详细请查看工程老代码
static void get_structure_param(void)
{
	
		glb_struct.chassis_config = DEFAULT_CONFIG;
		glb_struct.wheel_perimeter = PERIMETER;
		glb_struct.wheel_base = WHEELBASE;
		glb_struct.wheel_track = WHEELTRACK;
		glb_struct.gimbal_config = DEFAULT_CONFIG;
		glb_struct.gimbal_x_offset = GIMBAL_X_OFFSET;
		glb_struct.gimbal_y_offset = GIMBAL_Y_OFFSET;
	
}
static void CAN_ecd_to_angle(moto_measure_t *ptr, motor_t *Motor)
{
	switch (Motor->MOTOR_TYPE)
	{
	case M6020:
		ptr->total_angle = ptr->total_ecd * (ENCODER_ANGLE_RATIO); // 6020电机1比1直接转
		break;
	case M3508:
		ptr->total_angle = ptr->total_ecd * (ENCODER_ANGLE_RATIO / DECELE_RATIO_3508); // 3508电机
		break;
	case M2006:
		ptr->total_angle = ptr->total_ecd * (ENCODER_ANGLE_RATIO / DECELE_RATIO_2006); // 2006电机
		break;
	case BM_1010B:
	  ptr->total_angle = ptr->total_ecd * (BM_ENCODER_ANGLE_RATIO); 
    break;
	  default:
		break;
	}
}

static void Get_Motor_info(void)
{
	uint8_t ID;
	for (ID = 0; ID < Motor_count; ID++)
	{
		if (Whether_Brushless_Motor(Motor[ID]))
		{
			uint8_t esc_ID,master_ID;
			esc_ID = Motor[ID].Brushless.ESC_ID - 1;
			master_ID = Motor[ID].Brushless.MASTER_ID - 1;

			  switch (Motor[ID].Brushless.CAN_ID)
			{
			case 1:
			{
					if(Motor->MOTOR_TYPE==M6020 || Motor->MOTOR_TYPE==M3508 || Motor->MOTOR_TYPE==M2006)
			  {			
					CAN_ecd_to_angle(&Motor_CAN1_data[esc_ID], &Motor[ID]);
					Motor[ID].Brushless.ecd_fdb = Motor_CAN1_data[esc_ID].ecd;
					Motor[ID].Brushless.spd_fdb = Motor_CAN1_data[esc_ID].speed_rpm;
					Motor[ID].Brushless.angle_fdb = Motor_CAN1_data[esc_ID].total_angle;
					Motor[ID].Brushless.current_read = Motor_CAN1_data[esc_ID].given_current;
				}
							  if(Whether_DM_Motor(Motor[ID]))
			    {
											 
						switch (Motor[ID].MOTOR_TYPE)
						{
						case DM_10010L:
						{
									Motor[ID].Brushless.angle_fdb  = uint_to_float(DM_Motor_CAN1_data[master_ID].ecd,-12.5f,12.5f,16);
									Motor[ID].Brushless.spd_fdb    = uint_to_float(DM_Motor_CAN1_data[master_ID].speed_rpm,-25.0,25.0,12);
									Motor[ID].Brushless.torque_fbd = uint_to_float(DM_Motor_CAN1_data[master_ID].torque,-200.0,200.0,12);						
						}
						break;
						case DM_4310:
             {						 
                 	Motor[ID].Brushless.angle_fdb  = uint_to_float(DM_Motor_CAN1_data[master_ID].ecd,-12.5,12.5,16);
									Motor[ID].Brushless.spd_fdb    = uint_to_float(DM_Motor_CAN1_data[master_ID].speed_rpm,-25.0,25.0,12);
									Motor[ID].Brushless.torque_fbd = uint_to_float(DM_Motor_CAN1_data[master_ID].torque,-10.0,10.0,12);	
						 }
						break;

						case DM_4340:
             {
	                Motor[ID].Brushless.angle_fdb  = uint_to_float(DM_Motor_CAN1_data[master_ID].ecd,-12.5,12.5,16);
									Motor[ID].Brushless.spd_fdb    = uint_to_float(DM_Motor_CAN1_data[master_ID].speed_rpm,-25.0,25.0,12);
									Motor[ID].Brushless.torque_fbd = uint_to_float(DM_Motor_CAN1_data[master_ID].torque,-28.0,28.0,12);				
						 }
						break;
						default:
							break;
						}
											 
				}			
			}
			break;
			case 2:
			{
				  if(Motor->MOTOR_TYPE==M6020 || Motor->MOTOR_TYPE==M3508 || Motor->MOTOR_TYPE==M2006)
				{
					CAN_ecd_to_angle(&DJ_Motor_CAN2_data[esc_ID], &Motor[ID]);
					Motor[ID].Brushless.ecd_fdb = DJ_Motor_CAN2_data[esc_ID].ecd;
					Motor[ID].Brushless.spd_fdb = DJ_Motor_CAN2_data[esc_ID].speed_rpm;
					Motor[ID].Brushless.angle_fdb = DJ_Motor_CAN2_data[esc_ID].total_angle;
					Motor[ID].Brushless.current_read = DJ_Motor_CAN2_data[esc_ID].given_current;
					Motor[ID].Brushless.torque_fbd = DJ_Motor_CAN2_data[esc_ID].torque;	
				}
						
			  	   if(Whether_DM_Motor(Motor[ID]))
			  {
            switch (Motor[ID].MOTOR_TYPE)
						{
						case DM_10010L:
						{
									Motor[ID].Brushless.angle_fdb  = uint_to_float(DM_Motor_CAN2_data[master_ID].ecd,-12.5f,12.5f,16);
									Motor[ID].Brushless.spd_fdb    = uint_to_float(DM_Motor_CAN2_data[master_ID].speed_rpm,-25.0,25.0,12);
									Motor[ID].Brushless.torque_fbd = uint_to_float(DM_Motor_CAN2_data[master_ID].torque,-200.0,200.0,12);						
						}
						break;
						case DM_4310:
             {						 
                 	Motor[ID].Brushless.angle_fdb  = uint_to_float(DM_Motor_CAN2_data[master_ID].ecd,-12.5,12.5,16);
									Motor[ID].Brushless.spd_fdb    = uint_to_float(DM_Motor_CAN2_data[master_ID].speed_rpm,-25.0,25.0,12);
									Motor[ID].Brushless.torque_fbd = uint_to_float(DM_Motor_CAN2_data[master_ID].torque,-10.0,10.0,12);	
						 
						 }
						break;

						case DM_4340:
             {
	                Motor[ID].Brushless.angle_fdb  = uint_to_float(DM_Motor_CAN2_data[master_ID].ecd,-12.5,12.5,16);
									Motor[ID].Brushless.spd_fdb    = uint_to_float(DM_Motor_CAN2_data[master_ID].speed_rpm,-25.0,25.0,12);
									Motor[ID].Brushless.torque_fbd = uint_to_float(DM_Motor_CAN2_data[master_ID].torque,-28.0,28.0,12);				
						 }
						break;
						default:
							break;
						}
			  }		
			}
			break;
			}
			if (Motor[ID].MOTOR_TYPE == go_8010)
			{
				go8010_receive();
				switch (Motor[ID].Brushless.GO_ID)
				{
				  case GO_8010_1:
				  {
					    if (motor_recevie.id == GO_8010_1)
					  {
								Motor[ID].Brushless.torque_fbd = motor_recevie.T;
								Motor[ID].Brushless.spd_fdb = motor_recevie.W;
								Motor[ID].Brushless.l_angle_fdb=Motor[ID].Brushless.angle_fdb;   
								Motor[ID].Brushless.angle_fdb = motor_recevie.Pos; 
								if(poweren_flag==0){
									GO8010_init_angle1=motor_recevie.Pos;      //这行代码一定不能注释掉，不然电机会飞起来
									poweren_flag=1;
						     }					
					  }
				  }
				break;
				    case GO_8010_2:
				  {
					     if (motor_recevie.id == GO_8010_2)
					  {
								Motor[ID].Brushless.torque_fbd = motor_recevie.T;
								Motor[ID].Brushless.spd_fdb = motor_recevie.W;
								Motor[ID].Brushless.l_angle_fdb=Motor[ID].Brushless.angle_fdb;
								Motor[ID].Brushless.angle_fdb = motor_recevie.Pos;

					  }
				  }
				break;
				}
			}
		}
	}
	for (ID = 0; ID <= CHASSIS_BR; ID++)
	{ // 
		if (chassis_mode != CHASSIS_EXCHANGE_MODE )
		{
			Motor_CAN1_data[ID].round_cnt = 0;
			Motor_CAN1_data[ID].total_ecd = Motor_CAN1_data[ID].round_cnt * 8192 + Motor_CAN1_data[ID].ecd - Motor_CAN1_data[ID].offset_ecd;
		}
		chassis.wheel_angle_fdb[ID] = Motor_CAN1_data[ID].total_angle;
		chassis.wheel_spd_fdb[ID] = Motor_CAN1_data[ID].speed_rpm;
		chassis.current[ID] = Motor_CAN1_data[ID].given_current;
	}
	  chassis.joint_leg_angle_fdb[right_joint]  = Motor_CAN1_data[4].total_angle;     //0xc1
	  chassis.joint_leg_spd_fdb[right_joint]    = Motor_CAN1_data[4].speed_rpm;
	  chassis.joint_leg_angle_fdb[left_joint]   = Motor_CAN1_data[5].total_angle;     //0xc2
	  chassis.joint_leg_spd_fdb[left_joint]     = Motor_CAN1_data[5].speed_rpm;	
}

/*
			函数：目的为获取相对角度，让电机转劣弧 
			static int16_t get_relative_pos(int16_t raw_ecd, int16_t center_offset) // 得到相对位置
			具体函数去老代码查找，新代码已删
                                       */


uint8_t rc_change_state = 1;
uint8_t rc_middle_change_sw1;
uint8_t rc_middle_change_sw2;
/*此函数用于获取波轮上一次的位置*/
static void get_global_last_info(void)
{
	if (rc_change_state)
	{
		rc_middle_change_sw1 = rc.sw1;
		glb_sw.last_sw1 = rc_middle_change_sw1;
		glb_sw.last_last_sw1 = glb_sw.last_sw1;

		rc_middle_change_sw2 = rc.sw2;
		glb_sw.last_sw2 = rc_middle_change_sw2;

		rc_change_state = 0;
	}
	if (rc_middle_change_sw1 != rc.sw1)
	{
		glb_sw.last_last_sw1 = glb_sw.last_sw1;
		glb_sw.last_sw1 = rc_middle_change_sw1;
		rc_middle_change_sw1 = rc.sw1;
	}
	if (rc_middle_change_sw2 != rc.sw2)
	{
		glb_sw.last_sw2 = rc_middle_change_sw2;
		rc_middle_change_sw2 = rc.sw2;
	}

	glb_sw.last_iw = rc.iw;
}
