#include "keyboard.h"
#include "remote_ctrl.h"
#include "STM32_TIM_BASE.h"
#include "sys_config.h"
#include "ramp.h"
#include "chassis_task.h"
#include "modeswitch_task.h"
#include "pid.h"
#include "bsp_can.h"
#include "comm_task.h"
#include "delay.h"
#include "clamp.h"

kb_ctrl_t km;

/*****************************************************
 *                     悲 慈 佛 我                     *
 *                                                     *
 *  敢                   _oo0oo_                 脚    *
 *                      o8888888o                      *
 *  想                  88" . "88                踏    *
 *                      (| -_- |)                      *
 *  敢                  0\  =  /0                实    *
 *                    ___/`--- \___                    *
 *  干              .' \\|     |// '.            地    *
 *                 / \\|||  :  |||// \                 *
 *  醒            / _||||| -:- |||||- \          永    *
 *               |   | \\\  -  /// |   |               *
 *  狮           | \_|  ''\---/''  |_/ |         不    *
 *               \  .-\__  '-'  ___/-. /               *
 *  风        ___'. .'  /--.--\  `. .'___        言    *
 *         ."" '<  `.___\_<|>_/___.' >' "".            *
 *  范     | | :  `- \`.;`\ _ /`;.`/ - ` : | |   弃    *
 *         \  \ `_.   \_ __\ /__ _/   .-` /  /         *
 *    =====`-.____`.___ \_____/___.-`___.-'=====       *
 *                       `=---='                       *
 *                                                     *
 *     ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~     *
 *                                                     *
 *               佛祖保佑         永无BUG               *
 ******************************************************/

/*21赛季工程对于按键操作的控制逻辑 121.2.3*/

/*获取上次所在位置及当前位置 通过矿石编号差异确定移动位置*/

/*保存按键前的编号*/
/****************************************
*** @通过对比按键前的编号和按键后的编号
*** @若相同，则使能夹取
*** @若不同，则使能底盘移动
****************************************/
uint8_t button_press_state = 0; // 按键按下置1

/*矿石编号位置控制底盘移动*/
uint8_t big_island_chassis_move_done = 1; // 底盘移动到指定位置标志位 ，为1时，证明已经移动到指定位置
int big_island_chassis_move = 0;		  // 不为0时，启动底盘运动
int big_island_chassis_move_middle_change = 0;
// static void clamp_big_island_chassis_ctrl()
//{
//	if(button_press_state && big_island_ore_numbering_before_key != big_island_ore_numbering)//按键按下及按键前的编号和按键后的编号不一致
//	{
//		big_island_chassis_move_done = 0;
//
//		button_press_state = 0;//对按键处理了，将其置0
//
//		big_island_chassis_move = (int)(big_island_ore_numbering - big_island_ore_last_numbering);//获取移动距离
//	}
// }

//}
/*模式选择 -- 操作 进入/退出 视觉模式*/
/************************************************************
************前提条件：工程已处于操作手判断矿石掉落位置*********
*****@1.操作手可通过特定按键使机器人自动运动至指定位置
*****@2.仅当机器人已完成移动过程，可通过 短按 SHIFT按 进入 视觉辅助夹取模式
*****@2.仅当机器人已完成移动过程，可通过 长按 SHIFT按 退出 视觉辅助夹取模式
*/
uint8_t KB_BIG_ISLAND_AUTO_SINGLE_CLAMP_STATE = 0; // 优先级
// uint16_t big_island_shift_button_press_time    = 0;//当为视觉辅助夹取时，可长按退出到普通夹取模式
// static void clamp_big_island_mode_ctrl()//2021.2.27 该函数编辑调试耗时5小时
//{
//	if(chassis_mode == CHASSIS_CLAMP_BIG_ISLAND_STRAIGHT_MODE && big_island_chassis_move_done)/*/底盘移动完毕*/
//	{
//		if(KB_BIG_ISLAND_AUTO_SINGLE_CLAMP_CMD)
//		{
//			big_island_shift_button_press_time++;//按下时间
//
//			if(big_island_shift_button_press_time >= 300)//长按SHIFT 进入普通夹取模式
//			{
//				big_island_mode = BIG_ISLAND_ORDINARY_MODE;
//
//				KB_BIG_ISLAND_AUTO_SINGLE_CLAMP_STATE = 0;
//			}
//		}
//		else
//		{
//			if(big_island_shift_button_press_time < 300 && big_island_shift_button_press_time > 10)//短按
//			{
//				big_island_mode = BIG_ISLAND_AUTOMATIC_CLAMP_ONE_MODE;
//
//				KB_BIG_ISLAND_AUTO_SINGLE_CLAMP_STATE = 1;                          //加该标志位是为了，即使遥控操作是进入普通夹取，也能经键盘操作进入视觉辅助夹取一箱（即键盘操作比遥控操作优先级更高）
//			}
//
//			big_island_shift_button_press_time = 0;
//		}
//	}
//	else
//	{
//		big_island_mode = BIG_ISLAND_ORDINARY_MODE;//退出后重新进来默认是普通模式
//	}
//
// }
uint8_t INIT_STATE = 0; // 初始化所有电机标志位
int32_t INIT_ACTION_TIME = 0;
int32_t debug_time = 0;

void keyboard_clamp_hook(void)
{

	keyboard_exchange_hook();  //键鼠进行box_number的加减 
	keyboard_init_hook();      //键鼠reset
	kb_adrust_angle_ctrl();    //键鼠控制电机
	// keyboard_ore_adjust_hook();
}

/*兑换的箱数人为更改*/
uint8_t kb_pick_box_state = 1;
;
void keyboard_exchange_hook(void)
{
	if (km.kb_enable)
	{
		if (KB_HAVE_BOX_INC)
		{
			if (kb_pick_box_state)
			{
				have_box_number++;
				kb_pick_box_state = 0;
			}
		}

		else if (KB_HAVE_BOX_DEC)
		{
			if (kb_pick_box_state)
			{
				have_box_number--;
				kb_pick_box_state = 0;
			}
		}
		else
			kb_pick_box_state = 1;
	}
}

int32_t kb_slide_delay_time;
int32_t kb_uprise_delay_time;

int32_t kb_manipulator_delay_times[7];
int16_t kb_slide_angle[2] = {0};
int16_t kb_upraise_angle[2] = {0};

int16_t kb_adrust_angle[6] = {0};

uint8_t kb_turn_state;
/************************ 2023赛季新增 **********************/
// 正负要先debug看一下
int16_t kb_manipulator_adjust_angle[7] = {0}; // 机械臂角度

/*******************24赛季************************************/
kb_adrust_t kb_adrust[11];
kb_adrust_t kb_slide_clamp_del;
void kb_kb_adrust_angle_change(kb_adrust_t *kb_adrust, float flag, float delay_times)
{
	if ((HAL_GetTick() - kb_adrust->delay_times) > delay_times)
	{
		kb_adrust->angle += flag;
		kb_adrust->delay_times = HAL_GetTick();
	}
}
// void kb_kb_adrust_angle_change_test(kb_adrust_t *kb_adrust,int8_t flag,uint16_t delay_times)
//{
//	if((HAL_GetTick() - kb_adrust->delay_times) >delay_times )
//	{
//		kb_adrust->angle+=flag;
//		kb_adrust->delay_times=HAL_GetTick();
//	}
// }
void kb_adrust_angle_ctrl(void) // 机械臂角度控制
{
	if (km.kb_enable && (
             chassis_mode == CHASSIS_EXCHANGE_MODE || chassis_mode == CLASSIS_PRIMARY_MODE || chassis_mode == CHASSIS_SECONDDARY_ORE_MODE ))
	{
		if (PITCH1_UP)
		{
			kb_kb_adrust_angle_change(&kb_adrust[PITCH1], 0.0035, 1);
		}
		if (PITCH1_DN)
		{
			kb_kb_adrust_angle_change(&kb_adrust[PITCH1], -0.0035, 1);
		}
		if (PITCH2_UP)
		{
			kb_kb_adrust_angle_change(&kb_adrust[PITCH2], -0.0035*6.33, 1);
		}
		if (PITCH2_DN)
		{
			kb_kb_adrust_angle_change(&kb_adrust[PITCH2], 0.0035*6.33, 1);
		} 
		if (PITCH3_UP)
		{
			kb_kb_adrust_angle_change(&kb_adrust[PITCH3], 0.0035, 1);
		}
		if (PITCH3_DN)
		{
			kb_kb_adrust_angle_change(&kb_adrust[PITCH3], -0.0035, 1);
		}
		if (YAW_LEFT)
		{
			kb_kb_adrust_angle_change(&kb_adrust[YAW], 0.0035, 1);
		}
		if (YAW_RIGHT)
		{
			kb_kb_adrust_angle_change(&kb_adrust[YAW], -0.0035, 1);
		}
		if (ROLL2_LEFT)
		{
					kb_kb_adrust_angle_change(&kb_adrust[ROLL2], -0.0035, 1);
		}
		if (ROLL2_RIGHT)
		{
					kb_kb_adrust_angle_change(&kb_adrust[ROLL2], 0.0035, 1);
		}
		if (ROLL3_LEFT)
		{
					kb_kb_adrust_angle_change(&kb_adrust[ROLL3], -0.20063, 1);
		}
		if (ROLL3_RIGHT)
		{
					kb_kb_adrust_angle_change(&kb_adrust[ROLL3], 0.20063, 1);
		}

		
		
		
		
		
	}
	else if (chassis_mode == CHASSIS_NORMAL_MODE)
	{
		memset(kb_adrust, 0, sizeof(kb_adrust));
	}
}

void Stm32_SoftReset(void)
{
	__set_FAULTMASK(1);
	delay_ms(10);
	NVIC_SystemReset();
}

void keyboard_init_hook()
{
	if (km.kb_enable && chassis_mode == CHASSIS_RELEASE)
	{
		if (KB_RESET_INIT)
			Stm32_SoftReset();
	}
}

// void keyboard_ore_adjust_hook()
// {
// 	if(KB_ORE_ADJUST)
// 	{
// 		ore_adjust_state = 1;
// 	}
// }
static void chassis_direction_ctrl(uint8_t forward, uint8_t back, uint8_t left, uint8_t right)
{
	if (back)
	{
		km.vx = 1;
		rm.vx = 0;
		rm.vy = 0;
		rm.vw = 0;
	}
	else if (forward)
	{
		km.vx = -1;
		rm.vx = 0;
		rm.vy = 0;
		rm.vw = 0;
	}
	else
	{
		km.vx = 0;
		rm.vx = 0;
		rm.vy = 0;
		rm.vw = 0;
	}

	if (right)
	{
		km.vy = 1;
		rm.vx = 0;
		rm.vy = 0;
		rm.vw = 0;
	}
	else if (left)
	{
		km.vy = -1;
		rm.vx = 0;
		rm.vy = 0;
		rm.vw = 0;
	}
	else
	{
		km.vy = 0;
		rm.vx = 0;
		rm.vy = 0;
		rm.vw = 0;
	}
}
static void chassis_kb_operation_func(rc_info_t rc);
void keyboard_chassis_hook(void)
{
	if (km.kb_enable)
	{
		chassis_direction_ctrl(FORWARD, BACK, LEFT, RIGHT);
		chassis_kb_operation_func(rc);
	}

	else
	{
		km.vx = 0;
		km.vy = 0;
	}
}

static void chassis_kb_operation_func(rc_info_t rc) // 底盘运动控制函数
{
	if (FORWARD)
	{
		km.vy = -rc.kb.bit.W * CHASSIS_KB_MAX_SPEED_Y * CHASSIS_KB_MOVE_RATIO_Y;
	}
	if (BACK)
	{
		km.vy = rc.kb.bit.S * CHASSIS_KB_MAX_SPEED_Y * CHASSIS_KB_MOVE_RATIO_Y;
	}
	if (LEFT)
	{
		km.vx = -rc.kb.bit.A * CHASSIS_KB_MAX_SPEED_X * CHASSIS_KB_MOVE_RATIO_X;
	}
	if (RIGHT)
	{
		km.vx = rc.kb.bit.D * CHASSIS_KB_MAX_SPEED_X * CHASSIS_KB_MOVE_RATIO_X;
	}
	km.vw = rc.mouse.x / RC_RESOLUTION * CHASSIS_KB_MAX_SPEED_R * CHASSIS_KB_MOVE_RATIO_R * 0.2f; // 乘以0.5是因为操作时感觉转弯过于灵敏
}
