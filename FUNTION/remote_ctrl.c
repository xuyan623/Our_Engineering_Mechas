#include "remote_ctrl.h"
#include "stdlib.h"
#include "string.h"
#include "sys_config.h"
#include "STM32_TIM_BASE.h"
#include "keyboard.h"
#include "modeswitch_task.h"
#include "clamp.h"

rc_ctrl_t rm;
sw_record_t glb_sw;
clamp_action_status clamp_action;
exchange_action_status exchange_action;

//存矿与兑换使能用
uint8_t	change_mode_flag=4;
uint8_t clamp_action_ReadyToChange_state=0;
uint8_t exchange_action_ReadyToChange_state=1;
u8 primary_turn_ore_flag = 0;

/*dbus数组解析*/
void remote_ctrl(rc_info_t *rc,uint8_t *dbus_buf)
{
	rc->ch1 = (dbus_buf[0] | dbus_buf[1] << 8) & 0x07FF;
  rc->ch1 -= 1024;
  rc->ch2 = (dbus_buf[1] >> 3 | dbus_buf[2] << 5) & 0x07FF;
  rc->ch2 -= 1024;
  rc->ch3 = (dbus_buf[2] >> 6 | dbus_buf[3] << 2 | dbus_buf[4] << 10) & 0x07FF;
  rc->ch3 -= 1024;
  rc->ch4 = (dbus_buf[4] >> 1 | dbus_buf[5] << 7) & 0x07FF;
  rc->ch4 -= 1024;
  
  /* prevent remote control zero deviation */
  if(rc->ch1 <= 5 && rc->ch1 >= -5)
    rc->ch1 = 0;
  if(rc->ch2 <= 5 && rc->ch2 >= -5)
    rc->ch2 = 0;
  if(rc->ch3 <= 5 && rc->ch3 >= -5)
    rc->ch3 = 0;
  if(rc->ch4 <= 5 && rc->ch4 >= -5)
    rc->ch4 = 0;
  
  rc->sw1 = ((dbus_buf[5] >> 4) & 0x000C) >> 2;
  rc->sw2 = (dbus_buf[5] >> 4) & 0x0003;
  rc->iw = (dbus_buf[16] | dbus_buf[17] << 8) & 0x07FF;
	
  if ((abs(rc->ch1) > 660) || \
      (abs(rc->ch2) > 660) || \
      (abs(rc->ch3) > 660) || \
      (abs(rc->ch4) > 660))
  {
    memset(rc, 0, sizeof(rc_info_t));
    return ;
  }

  rc->mouse.x = dbus_buf[6] | (dbus_buf[7] << 8); // x axis
  rc->mouse.y = dbus_buf[8] | (dbus_buf[9] << 8);
  rc->mouse.z = dbus_buf[10] | (dbus_buf[11] << 8);

  rc->mouse.l = dbus_buf[12];
  rc->mouse.r = dbus_buf[13];

  rc->kb.key_code = ((dbus_buf[14] | dbus_buf[15] << 8)); // key borad code| (dbus_buf[16] | dbus_buf[17] << 8) << 8
}

/*
* @ RC_RESOLUTION :摇杆最大值 660 
* @ CHASSIS_RC_MAX_SPEED_X
    CHASSIS_RC_MAX_SPEED_Y
    CHASSIS_RC_MAX_SPEED_R ：平移和旋转的速度最大值
  @ CHASSIS_RC_MOVE_RATIO_X
    CHASSIS_RC_MOVE_RATIO_Y
    CHASSIS_RC_MOVE_RATIO_R : 数值方向
*/
/*底盘赋值*/
int rocker_maximum_continuous_times;//如控制旋转时，摇杆持续打在最大档位将放开限制
static void chassis_operation_func(int16_t forward_back, int16_t left_right, int16_t rotate)
{
  rm.vx =  forward_back / RC_RESOLUTION * CHASSIS_RC_MAX_SPEED_X * CHASSIS_RC_MOVE_RATIO_X;
  rm.vy = -left_right / RC_RESOLUTION * CHASSIS_RC_MAX_SPEED_Y * CHASSIS_RC_MOVE_RATIO_Y;
	
	/*因为在模拟小陀螺时限制了旋转速度会出现达不到想要的转速*/
	if(!(rotate == RC_RESOLUTION || rotate == - RC_RESOLUTION))
	{
		rocker_maximum_continuous_times = HAL_GetTick();
	}
	if(HAL_GetTick() - rocker_maximum_continuous_times > 2000)//放开限制
	{
		rm.vw = rotate / RC_RESOLUTION * CHASSIS_RC_MAX_SPEED_R * CHASSIS_RC_MOVE_RATIO_R;
	}
	else
	{
    rm.vw = rotate / RC_RESOLUTION * CHASSIS_RC_MAX_SPEED_R * CHASSIS_RC_MOVE_RATIO_R*0.5f;//乘以0.5是因为操作时感觉转弯过于灵敏
	}
}


static void gimbal_operation_func(int16_t pit1_ctrl, int16_t yaw_ctrl,int16_t pit2_ctrl)
{
	/* gimbal coordinate system is right hand coordinate system */
	rm.pit1_v += -pit1_ctrl * 0.0000030f * GIMBAL_RC_MOVE_RATIO_PIT;
	rm.pit2_v += -pit2_ctrl * 0.000045f * GIMBAL_RC_MOVE_RATIO_PIT;	
	rm.yaw_v +=  yaw_ctrl *   0.0000070f * GIMBAL_RC_MOVE_RATIO_YAW; 
}

static void chassis_joint_leg_func(int16_t leg_ctrl)
{
   rm.pit_leg += leg_ctrl*0.0001;
}

void remote_ctrl_chassis_leg_hook(void)
{
    chassis_joint_leg_func(rc.ch4);
}



void remote_ctrl_chassis_hook(void)
{
    chassis_operation_func(rc.ch1, rc.ch2, rc.ch3);
}

void remote_ctrl_gimbal_hook(void)
{
    gimbal_operation_func(rc.ch4, rc.ch1, rc.ch2);

}

static void kb_enable_hook(void)
{
	km.kb_enable = 1;
}


//24赛季动作使能判断
void rc_clamp_store_cmd()
{
	uint8_t action_max_number=2;
	if(chassis_mode == CHASSIS_GET_ENERGY_UNIT_MODE 
	  ||chassis_mode == CHASSIS_GET_ENERGY_UNIT1_MODE 
		||chassis_mode == CHASSIS_GET_ENERGY_UNIT2_MODE
	  ||chassis_mode == CLASSIS_PRIMARY_MODE
		||chassis_mode == CHASSIS_SECONDDARY_ORE_MODE
	  ||chassis_mode == CHASSIS_LEG_UP_MODE
	  ||chassis_mode == CHASSIS_LEG_DOWN_MODE)
	{
		if(rc.sw1==RC_MI)
			clamp_action_ReadyToChange_state=1;
		if(clamp_action_ReadyToChange_state)
		{
			if(RC_SW1_DN)
			{
				
				if(clamp_action<=action_max_number)clamp_action++;
				clamp_action_ReadyToChange_state=0;
			}
			if(RC_SW1_UP)
			{
				if(clamp_action>=1)clamp_action--;
				clamp_action_ReadyToChange_state=0;
				if(have_box_number>=1)have_box_number--;
			}
		}
	}
}

void rc_exchange_action_cmd()
{
	if(chassis_mode == CHASSIS_EXCHANGE_MODE )
	{
		if(rc.sw1==RC_MI)
		{
			exchange_action_ReadyToChange_state=1;
			exchange_action=EXCHANGE_UN_CMD;
		}
		if(exchange_action_ReadyToChange_state)
		{
			if(RC_SW1_DN)//放
			{
				exchange_action=PICK_ACTION1;
				exchange_action_ReadyToChange_state=0;
			}
			if(RC_SW1_UP)//
			{
				if(exchange_action_ReadyToChange_state)
					exchange_action=PICK_ACTION2;
				exchange_action_ReadyToChange_state=0;
			}
		}
	}
}

void rc_primary_action_cmd()
{

  if(chassis_mode == CLASSIS_PRIMARY_MODE)
	{
	    if(RC_IW_DN)
		{ 
			  primary_turn_ore_flag = 1;	
		}
	}
	else 
	{
		 primary_turn_ore_flag = 0;	
	}
}


void remote_ctrl_clamp_hook(void)
{
	rc_clamp_store_cmd(); //超级无敌之一个更比5个强（替代原有的5个函数）  具体作用就是进行动作的下一步
	rc_exchange_action_cmd();   //对兑换模式下动作进行下一步的细分，具体就是（左摇杆上打：放气，左摇杆下打：掏矿）
  rc_primary_action_cmd();
	/*使能键盘鼠标*/
  kb_enable_hook();
}



