#include "detect_task.h"
#include "STM32_TIM_BASE.h"

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

#include "modeswitch_task.h"
#include "bsp_can.h"
#include "stdio.h"
#include "stdlib.h"
#include "string.h"

RCC_ClocksTypeDef RCC_Clocks;

UBaseType_t detect_stack_surplus;

global_err_t global_err;

State_t detect_state,detect_last_state;

/*******
******三维数组举例：17个学校，每个学校有两个班，每个班有12个同学
******txt三维数组解说：17个不同模块，每个模块有两种状态，每个状态有12个字符进行说明
*******/
char txt[17][2][14] = {
                    {"",""},                             // 0
                    {"底盘右上正常","底盘右上异常"},       // 1
                    {"底盘左上正常","底盘左上异常"},       // 2
                    {"底盘左下正常","底盘左下异常"},       // 3
                    {"底盘右下正常","底盘右下异常"},       // 4
                    {"遥控正常"    ,"遥控异常"    },       // 5
                    {"夹取正常"    ,"夹取异常"   },        // 6
                    {"夹紧前正常"  ,"夹紧前异常" },        // 7
                    {"夹紧后正常"  ,"夹紧后异常" },        //8
                    {"抬升正常"    ,"抬升异常"   },        //9                   
                    {"救援左正常"  ,"救援左异常" },         //10
                    {"救援右正常"  ,"救援右异常" },         //11
                    {"复活正常"    ,"复活异常"   },         //12
                    {"搬运右正常"  ,"搬运右异常"   },       //13
										{"搬运左正常"  ,"搬运左异常"   },      //14
                    {"裁判系统正常","裁判系统异常"},       //15
                    {"小电脑正常"  ,"小电脑异常"  }};      //16 
uint32_t temp1,temp2;
void detect_task(void *parm)
{
 
	uint32_t detect_wake_time = osKernelSysTick();//微秒级计时，线程管理函数
  while(1)
  {

  }
}


void detect_param_init(void)
{
  for(uint8_t id = CHASSIS_M1_OFFLINE; id <= JUDGE_SYS_OFFLINE; id++)//
  {
    global_err.list[id].param.set_timeout = 500;
    global_err.list[id].param.last_times  = 0;
    global_err.list[id].param.delta_times = 0;
    global_err.list[id].err_exist         = 0;
    global_err.err_now_id[id]             = BOTTOM_DEVICE;
  }
  global_err.list[PC_SYS_OFFLINE].param.set_timeout = 2000;
  global_err.list[PC_SYS_OFFLINE].param.last_times  = 0;
  global_err.list[PC_SYS_OFFLINE].param.delta_times = 0;
  global_err.list[PC_SYS_OFFLINE].err_exist         = 0;
  global_err.err_now_id[PC_SYS_OFFLINE]             = BOTTOM_DEVICE;
}

void err_detector_hook(int err_id)
{
  global_err.list[err_id].param.last_times = HAL_GetTick();
}

void module_offline_detect(void)
{
  for (uint8_t id = CHASSIS_M1_OFFLINE; id <= PC_SYS_OFFLINE; id++)
  {
    global_err.list[id].param.delta_times = HAL_GetTick() - global_err.list[id].param.last_times;
		
    if(global_err.list[id].param.delta_times > global_err.list[id].param.set_timeout)
    {
      global_err.err_now_id[id]     = (err_id)id;
      global_err.list[id].err_exist = 1;
      Set_bit(global_err.offline,id);//标志异常位--置位
    }
    else
    {
      global_err.err_now_id[id]     = BOTTOM_DEVICE;
      global_err.list[id].err_exist = 0;
      Reset_bit(global_err.offline,id);
    }
  }
}




