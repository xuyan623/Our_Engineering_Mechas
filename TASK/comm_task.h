#ifndef _comm_task_H
#define _comm_task_H


#include "stm32f4xx.h"
#include "User.h"

#define JUDGE_UART_TX_SIGNAL            ( 1 << 0 )
#define JUDGE_UART_IDLE_SIGNAL          ( 1 << 1 )

#define PC_UART_TX_SIGNAL               ( 1 << 2 )
#define PC_UART_IDLE_SIGNAL             ( 1 << 3 )

#define CHASSIS_MOTOR_MSG_SIGNAL        ( 1 << 4 )
#define MODE_SWITCH_MSG_SIGNAL          ( 1 << 9 )

#define INFO_GET_CHASSIS_SIGNAL         ( 1 << 13 )

#define MODE_SWITCH_INFO_SIGNAL         ( 1 << 21 )

#define INFO_GET_MOTOR_SIGNAL          ( 1 << 22 )
#define INFO_SEND_MOTOR_SIGNAL          ( 1 << 23 )

#define COMM_8010_TASK_SIGNAL           ( 1 << 24 )
#define GO_8010_INIT_SIGNAL             ( 1 << 25 )

//typedef struct
//{
//  /* 底盘电机电流 */
//  int16_t chassis_cur[4];
//	/* 抬升电机电流 */
//	int16_t upraise_cur[2];
//	/* 障碍块电机电流 */
//	int16_t barrier_carry_cur[2];
//	/* 救援电机电流 */
//	int16_t rescue_cur[2];
//	/* 1.夹取翻转电机电流*/
//	/* 1.夹取旋转电机电流 */
//	/* 2.兑换挡板电机电流 */
//	int16_t clamp_cur[3];
//	/* 姿态调制电机电流 */
//	int16_t clamp_attitude_adjustment_cur[1];
//	int16_t manipulator[6];
//	
//} motor_current_t;



extern int16_t CAN1_current[9];
extern int16_t CAN2_current[9];

void can_msg_send_task(void *parm);


#endif 

