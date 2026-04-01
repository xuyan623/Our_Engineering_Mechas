#include "modeswitch_task.h"
#include "STM32_TIM_BASE.h"
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "remote_ctrl.h"
#include "keyboard.h"
#include "comm_task.h"
#include "chassis_task.h"
#include "controller.h"
#include "pc_tx_data.h"
#include "judge_tx_data.h"
#include "bsp_can.h"
#include "motor_8010.h"
/**********************************************************************************
*******本源代码控制逻辑于20.12.19张某秀修改（若有建议及疑问请联系QQ：3155460945）******
***********************************************************************************
//1.首先获取全局状态，然后经操作选定底盘模式
//2.底盘模式包含其他工程模式的功能：补给、夹取、障碍块搬运、救援、兑换金币模式
//3.所以判断进入工程模式的哪种模式可通过查看底盘的模式
*********************21.1.31开始全部修改了夹取模式的控制逻辑**************************

***********************************************************************************/

UBaseType_t mode_switch_stack_surplus;

extern TaskHandle_t info_get_Task_Handle;

global_status global_mode; 
global_status last_global_mode = RELEASE_CTRL;

chassis_status chassis_mode;
chassis_status last_chassis_mode = CHASSIS_RELEASE;

rescue_status rescue_mode;
rescue_status last_rescue_mode = RESCUE_INIT;

clamp_status clamp_mode;
clamp_status last_clamp_mode = CLAMP_INIT;

barrier_carry_status barrier_carry_mode;
barrier_carry_status last_barrier_carry_mode = BARRIER_CARRY_INIT;

supply_status supply_mode;
supply_status last_supply_mode;

gimbal_status gimbal_mode;

/*小/大资源岛模式具体的操作模式不需要处理上次的模式*/
small_island_mode_t small_island_mode;
big_island_mode_t   big_island_mode;


//获取遥控信息，进行机器人模式的切换，并通知info_get_task进行信息的汇总，进而对电机进行命令等
void mode_switch_task(void *parm)
{
	uint32_t mode_switch_wake_time = osKernelSysTick();
	while(1)
	{
		
	//	go8010_task();
		get_last_mode();           //获取所有任务上次的模式
		get_main_mode();           //主要模式就是通过右摇杆的上中下进行切换
		get_chassis_mode();        //在主要模式下的进一步细分，用以打出动作的切换
		remote_ctrl_clamp_hook();  //遥控器控制动作下一步的进行
  	keyboard_clamp_hook();     //键鼠控制函数
		
//    /*发送遥控数据*/
//    send_rc_data1();
//    send_rc_data2();
//    send_rc_data3();
    
    xTaskGenericNotify( (TaskHandle_t) info_get_Task_Handle, 
                        (uint32_t) MODE_SWITCH_INFO_SIGNAL, 
                        (eNotifyAction) eSetBits, 
                        (uint32_t *)NULL );
    
 //   mode_switch_stack_surplus = uxTaskGetStackHighWaterMark(NULL);
    
    vTaskDelayUntil(&mode_switch_wake_time, 6);//绝对延时函数
  }
}
/*************************************************************
*****1.首先判断状态有没有变化
*****2.状态发送变化时再赋值上次状态
*****3.上次状态和当前状态保持不一样，即当前状态切换时再更新上次状态

********该思路只在工程代码更改，其他或可能是沿用新框架的思路
***************************************************************/
uint8_t	global_change_state = 1;
	/*中间变化模式缓冲*/
uint8_t	global_middle_change_mode;
uint8_t chassis_middle_change_mode;
uint8_t rescue_middle_change_mode;
uint8_t clamp_middle_change_mode ;
uint8_t barrier_carry_middle_change_mode;
uint8_t supply_middle_change_mode;


void get_last_mode(void)
{	
	if(global_change_state)//只运行一次，初始化就好
	{		
		global_middle_change_mode        = global_mode;
		last_global_mode                 = (global_status)global_middle_change_mode;
		
		chassis_middle_change_mode       = chassis_mode;
		last_chassis_mode                = (chassis_status)chassis_middle_change_mode;
		
		rescue_middle_change_mode        = rescue_mode;
		last_rescue_mode                 = (rescue_status)rescue_middle_change_mode;
		
		clamp_middle_change_mode         = clamp_mode;
		last_clamp_mode                  = (clamp_status)clamp_middle_change_mode;
		
    barrier_carry_middle_change_mode = barrier_carry_mode;
		last_barrier_carry_mode          = (barrier_carry_status)barrier_carry_middle_change_mode;
		
		supply_middle_change_mode        = supply_mode;
		last_supply_mode                 = (supply_status)supply_middle_change_mode;
		
		global_change_state = 0;
	}
	if((global_status)global_middle_change_mode != global_mode )                      //获取上一次全局状态
	{
		last_global_mode                = (global_status)global_middle_change_mode;
		global_middle_change_mode       = global_mode;
	}
	if((chassis_status)chassis_middle_change_mode !=chassis_mode )                    //获取上一次底盘状态
	{
		last_chassis_mode               = (chassis_status)chassis_middle_change_mode;
		chassis_middle_change_mode      = chassis_mode;
		
		/*控制抬升 发生模式变化时 抬升的变动都需用单环控制好升降速度*/
//		upraise_angle_ctrl_state[0] = 0;
//		upraise_angle_ctrl_state[1] = 0;
//		upraise.updown_flag        = RAISE; //抬升标志
	}
	if(rescue_middle_change_mode != rescue_mode)                                      //获取上一次救援状态
	{
		last_rescue_mode                 = (rescue_status)rescue_middle_change_mode;
		rescue_middle_change_mode        = rescue_mode;
	}
	if((clamp_status)clamp_middle_change_mode != clamp_mode)                          //获取上一次夹取状态
	{
		last_clamp_mode                  = (clamp_status)clamp_middle_change_mode;
		clamp_middle_change_mode         = clamp_mode;
	}
	if((barrier_carry_status)barrier_carry_middle_change_mode != barrier_carry_mode)   //获取上一次搬运障碍块状态
	{
		last_barrier_carry_mode          = (barrier_carry_status)barrier_carry_middle_change_mode;
		barrier_carry_middle_change_mode = barrier_carry_mode;
	}
	if((supply_status)supply_middle_change_mode != supply_mode)                        //获取上一次补给状态
	{
		last_supply_mode                 = (supply_status)supply_middle_change_mode;
		supply_middle_change_mode        = supply_mode;
	}
}

void get_main_mode(void)
{
  switch(rc.sw2)
  {
    case RC_UP:
    {
    	global_mode = MANUAL_CTRL;   			//正常模式
    }
    break;
    case RC_MI:
    {
    	global_mode = ENGINEER_CTRL;           //工程模式（夹取兑换等）				
    }break;
    case RC_DN:
    {
    	global_mode = RELEASE_CTRL;						//断电状态
    }
    break;
    default:
    {
		  global_mode = RELEASE_CTRL;
    }break;
  }
}



void get_chassis_mode(void)
{
  switch(global_mode)
  {
    case RELEASE_CTRL:
		{
    	chassis_mode = CHASSIS_RELEASE;
			PUMP_OFF
		}
    break;  
		
		/*因特殊情况在此添加防守模式*/
    case MANUAL_CTRL:
    {
			if(RC_IW_UP &&  rc.sw1 == RC_UP)
			{  
				chassis_mode = PITCH3_TORQUE_COLLECTION_MODE;
			}
			else if((glb_sw.last_iw > IW_UP) && (rc.iw <= IW_UP) &&  rc.sw1 == RC_DN)
			{
				chassis_mode = CHASSIS_LEG_UP_MODE;
			}
		  else if(RC_IW_UP_NOMAL && rc.sw1 == RC_MI)        
      {
        chassis_mode = CHASSIS_SECONDDARY_ORE_MODE;		    
			}	
			else if(chassis_mode != PITCH3_TORQUE_COLLECTION_MODE && chassis_mode != CHASSIS_SECONDDARY_ORE_MODE && chassis_mode != CHASSIS_URGENT_MEASURE &&chassis_mode  != CHASSIS_LEG_UP_MODE)
			{
        chassis_mode = CHASSIS_NORMAL_MODE;
			}
			
			if((glb_sw.last_iw < IW_DN) && (rc.iw >= IW_DN) && chassis_mode == PITCH3_TORQUE_COLLECTION_MODE)   //左摇杆向下滑
			{
				chassis_mode = CHASSIS_NORMAL_MODE;
			}
		  else if((glb_sw.last_iw < IW_DN) && (rc.iw >= IW_DN) && chassis_mode == CHASSIS_LEG_UP_MODE)
			{
				chassis_mode = CHASSIS_NORMAL_MODE;
			}
			else if((glb_sw.last_iw < IW_DN) && (rc.iw >= IW_DN) && chassis_mode == CHASSIS_SECONDDARY_ORE_MODE)
			{
				chassis_mode = CHASSIS_NORMAL_MODE;				
			}
    }break;
    
	/***********************************************************
	***************global_mode = ENGINEER_CTRL******************
	******************该模式下切换其他模式说明*********************
	**** @1.rc.sw1 = RC_UP && glb_sw.last_iw向上 切换到笔直大资源岛模式
	**** @1.rc.sw1 = RC_UP && glb_sw.last_iw向下 切换到倾斜大资源岛模式

	**** @4.rc.sw1 = RC_MI && glb_sw.last_iw向上 切换到兑换模式
	**** @5.rc.sw1 = RC_MI && glb_sw.last_iw向下 切换到检录模式

	**** @6.rc.sw1 = RC_DN && glb_sw.last_iw向上 切换到侧臂小资源岛夹取模式
	**** @7.rc.sw1 = RC_DN && glb_sw.last_iw向上 切换到夹地面模式
	****************************************************************/
    case ENGINEER_CTRL:
    {
		/*************工程模式——遥控切换模式**************/
		if(RC_SW2_MI_IW_UP && rc.sw1 == RC_UP)
    {          
       chassis_mode = CHASSIS_GET_ENERGY_UNIT_MODE;    //左滑轮往上滑  获取能量单元
		}
		else if(RC_SW2_MI_IW_DN && rc.sw1 == RC_UP)
		{
			chassis_mode = CHASSIS_GET_ENERGY_UNIT1_MODE;    
    }
		else if(RC_SW2_MI_IW_UP && rc.sw1 == RC_MI)
		{
			chassis_mode = CHASSIS_EXCHANGE_MODE;     				  //兑换模式	
		}
		else if(RC_SW2_MI_IW_DN && rc.sw1 == RC_MI)
		{
			chassis_mode = CHASSIS_GET_ENERGY_UNIT2_MODE;     
		}

		else if(RC_SW2_MI_IW_UP && rc.sw1 == RC_DN)      
		{
			chassis_mode = CLASSIS_PRIMARY_MODE;    		  //侧臂小资源岛夹取模式
			
		}

		else if(RC_SW2_MI_IW_DN && rc.sw1 == RC_DN)
    {   
   		chassis_mode = CHASSIS_LEG_DOWN_MODE;			
    }				
    }break; 
	
    default:
    {
		chassis_mode = CHASSIS_RELEASE;
    }break;
  }
}

