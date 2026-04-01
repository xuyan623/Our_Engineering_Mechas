/*********************************************************
*                    悲 慈 佛 我                       *
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
*                                                     *
*     ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~     *
*                                                     *
*               佛祖保佑         永无BUG               *
******************************************************/

#include "start_task.h"

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"

#include "imu_task.h"
#include "comm_task.h"
#include "modeswitch_task.h"
#include "info_get_task.h"
#include "detect_task.h"

//#include "gimbal_task.h"
#include "chassis_task.h"

#include "motor_task.h"

#include "judge_task.h"
#include "pc_task.h"
#include "comm_8010_task.h"

#define START_TASK_SIZE 128  // 128 * 32 Bit = 128 * 4 Byte（堆栈）
#define START_TASK_PRIO 2    // 优先级

#define CHASSIS_TASK_SIZE 128
#define CHASSIS_TASK_PRIO 5

#define GIMBAL_TASK_SIZE 128 
#define GIMBAL_TASK_PRIO 5

#define CLAMP_TASK_SIZE 128
#define CLAMP_TASK_PRIO 5

#define UPRAISE_TASK_SIZE 128
#define UPRAISE_TASK_PRIO 5

#define BARRIER_CARRY_TASK_SIZE 128 //搬运障碍块
#define BARRIER_CARRY_TASK_PRIO 5

#define SUPPLY_TASK_SIZE 128
#define SUPPLY_TASK_PRIO 5

#define CAN_MSG_SEND_TASK_SIZE 256
#define CAN_MSG_SEND_TASK_PRIO 6

#define MODE_SWITCH_TASK_SIZE 256
#define MODE_SWITCH_TASK_PRIO 5

#define INFO_GET_TASK_SIZE 256
#define INFO_GET_TASK_PRIO 5

#define DETECT_TASK_SIZE 128
#define DETECT_TASK_PRIO 4

#define IMU_TASK_SIZE 128
#define IMU_TASK_PRIO 5

#define JUDEG_TX_TASK_SIZE 320
#define JUDEG_TX_TASK_PRIO 5

#define JUDEG_RX_TASK_SIZE 320
#define JUDEG_RX_TASK_PRIO 5

#define PC_TX_TASK_SIZE 256
#define PC_TX_TASK_PRIO 4

#define PC_RX_TASK_SIZE 256
#define PC_RX_TASK_PRIO 4


#define MOTOR_TASK_SIZE 320
#define MOTOR_TASK_PRIO 5

#define COMM_8010_TASK_SIZE 256
#define COMM_8010_TASK_PRIO 5

TaskHandle_t start_Task_Handle;

TaskHandle_t gimbal_Task_Handle;
TaskHandle_t chassis_Task_Handle;

TaskHandle_t clamp_Task_Handle;
TaskHandle_t upraise_Task_Handle;
TaskHandle_t barrier_carry_Task_Handle;
TaskHandle_t supply_Task_Handle;
TaskHandle_t can_msg_send_Task_Handle;
TaskHandle_t mode_switch_Task_Handle;
TaskHandle_t info_get_Task_Handle;
TaskHandle_t detect_Task_Handle;
TaskHandle_t imu_Task_Handle;
TaskHandle_t slide_Task_Handle;

TaskHandle_t judge_tx_Task_Handle;
TaskHandle_t judge_rx_Task_Handle;
TaskHandle_t pc_tx_Task_Handle;
TaskHandle_t pc_rx_Task_Handle;

TaskHandle_t motor_Task_Handle;
TaskHandle_t comm_8010_task_Handle;

void start_task(void *parm)
{
	taskENTER_CRITICAL();
	
	{
		/*创建任务 -- 自动分配一个任务控制块*/                     
    	xTaskCreate((TaskFunction_t)chassis_task,			//任务函数
					(const char *)"chassis_task",  			//任务名称
					(uint16_t)CHASSIS_TASK_SIZE,   			//堆栈大小（动态内存申请）
					(void * )NULL,                 			//传送任务函数的参数
					(UBaseType_t)CHASSIS_TASK_PRIO,			//优先级
					(TaskHandle_t *) &chassis_Task_Handle );//任务句柄
							
							
		xTaskCreate((TaskFunction_t)Motor_task,
					(const char *)"motor_task",
					(uint16_t)MOTOR_TASK_SIZE,
					(void * )NULL,
					(UBaseType_t)MOTOR_TASK_PRIO,
					(TaskHandle_t *) &motor_Task_Handle );
					
		xTaskCreate((TaskFunction_t)coom_8010_task,
					(const char *)"coom_8010_task",
					(uint16_t)COMM_8010_TASK_SIZE,
					(void * )NULL,
					(UBaseType_t)COMM_8010_TASK_PRIO,
					(TaskHandle_t *) &comm_8010_task_Handle );

  	}        

  	{
		xTaskCreate((TaskFunction_t)can_msg_send_task,
					(const char *)"can_msg_send_task",
					(uint16_t)CAN_MSG_SEND_TASK_SIZE,
					(void * )NULL,
					(UBaseType_t)CAN_MSG_SEND_TASK_PRIO,
					(TaskHandle_t *) &can_msg_send_Task_Handle );
	}
	
	{						
		xTaskCreate((TaskFunction_t)mode_switch_task,
					(const char *)"mode_switch_task",
					(uint16_t)MODE_SWITCH_TASK_SIZE,
					(void * )NULL,
					(UBaseType_t)MODE_SWITCH_TASK_PRIO,
					(TaskHandle_t *) &mode_switch_Task_Handle );
							
		xTaskCreate((TaskFunction_t)info_get_task,
					(const char *)"info_get_task",
					(uint16_t)INFO_GET_TASK_SIZE,
					(void * )NULL,
					(UBaseType_t)INFO_GET_TASK_PRIO,
					(TaskHandle_t *) &info_get_Task_Handle );
								
		xTaskCreate((TaskFunction_t)detect_task,
					(const char *)"detect_task",
					(uint16_t)DETECT_TASK_SIZE,
					(void * )NULL,
					(UBaseType_t)DETECT_TASK_PRIO,
					(TaskHandle_t *) &detect_Task_Handle );
								
		xTaskCreate((TaskFunction_t)imu_task,
					(const char *)"imu_task",
					(uint16_t)IMU_TASK_SIZE,
					(void * )NULL,
					(UBaseType_t)IMU_TASK_PRIO,
					(TaskHandle_t *) &imu_Task_Handle );
	}						
	
  	{
    	xTaskCreate((TaskFunction_t)judge_tx_task,
					(const char *)"judge_tx_task",
					(uint16_t)JUDEG_TX_TASK_SIZE,
					(void * )NULL,
					(UBaseType_t)JUDEG_TX_TASK_PRIO,
					(TaskHandle_t *) &judge_tx_Task_Handle );
	
    	xTaskCreate((TaskFunction_t)judge_rx_task,
					(const char *)"judge_rx_task",
					(uint16_t)JUDEG_RX_TASK_SIZE,
					(void * )NULL,
					(UBaseType_t)JUDEG_RX_TASK_PRIO,
					(TaskHandle_t *) &judge_rx_Task_Handle );
	
//    	xTaskCreate((TaskFunction_t)pc_tx_task,
//					(const char *)"pc_tx_task",
//					(uint16_t)PC_TX_TASK_SIZE,
//					(void * )NULL,
//					(UBaseType_t)PC_TX_TASK_PRIO,
//					(TaskHandle_t *) &pc_tx_Task_Handle );
//	
//    	xTaskCreate((TaskFunction_t)pc_rx_task,
//					(const char *)"pc_rx_task",
//					(uint16_t)PC_RX_TASK_SIZE,
//					(void * )NULL,
//					(UBaseType_t)PC_RX_TASK_PRIO,
//					(TaskHandle_t *) &pc_rx_Task_Handle );
	
  	}
  
	vTaskDelete(start_Task_Handle);//删除开始任务（利用任务句柄完成删除操作 删除自身任务也可使用NULL）
	
	taskEXIT_CRITICAL();					 //退出临界区	
}

void TASK_START(void)
{
	xTaskCreate((TaskFunction_t)start_task,
				(const char *)"start_task",
				(uint16_t)START_TASK_SIZE,
				(void * )NULL,
				(UBaseType_t)START_TASK_PRIO,
				(TaskHandle_t *) &start_Task_Handle );
}

