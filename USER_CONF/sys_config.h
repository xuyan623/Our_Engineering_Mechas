#ifndef _sys_config_H
#define _sys_config_H

#include "stm32f4xx.h"

#define DEBUG

#define INFANTRY_1  1
#define INFANTRY_2  2
#define INFANTRY_3  3
#define INFANTRY_4  4
#define INFANTRY_5  5

/*闪躲模式 小陀螺或者小猫步*/
#define LG 1
#define TWIST 2

#define  INFANTRY_NUM  INFANTRY_1
#define  DODGE_MODE LG //TWIST
/**********************遥控 设置配置***************************/
/* 摇杆最大值 */
#define RC_RESOLUTION     660.0f
/*************************底盘 配置*******************************/
/* 遥控模式  底盘 速度 限制 */
/* 前后 速度 (mm/s) */
#define CHASSIS_RC_MAX_SPEED_X  3300.0f
#define CHASSIS_RC_MOVE_RATIO_X 1.0f
/* 左右 速度 (mm/s) */
#define CHASSIS_RC_MAX_SPEED_Y  3300.0f
#define CHASSIS_RC_MOVE_RATIO_Y 1.0f
/* 底盘 旋转 速度 (deg/s) */
#define CHASSIS_RC_MAX_SPEED_R  480.0f
#define CHASSIS_RC_MOVE_RATIO_R 1.0f

/* 键盘模式  底盘 速度 限制 */
/* 左右 速度 (mm/s) */
#define CHASSIS_KB_MAX_SPEED_X  1000.0f 
#define CHASSIS_KB_MOVE_RATIO_X 1.0f
/* ?? ?? (mm/s) */
#define CHASSIS_KB_MAX_SPEED_Y  1000.0f
#define CHASSIS_KB_MOVE_RATIO_Y 0.6f
/* ?? ?? ?? (deg/s) */
#define CHASSIS_KB_MAX_SPEED_R  600.0f
#define CHASSIS_KB_MOVE_RATIO_R 1.0f

/**************************陀螺仪 配置*******************************/
/* 置1为上电校准，通常一块板子只需要校准一次，然后将BMI088里的offset参数填入BMI088driver.h里然后重新置0，即可上电直接读取参数不用建准*/
#define CALIBRATION 0

/**************************云台 配置*******************************/
/* 遥控模式 云台速度控制 */
/* pitch 轴 速度 */
#define GIMBAL_RC_MOVE_RATIO_PIT 1.0f
/* yaw 轴 速度 */
#define GIMBAL_RC_MOVE_RATIO_YAW 1.0f

/* 遥控模式 云台速度控制 */
/* pitch 轴 速度 */
#define GIMBAL_PC_MOVE_RATIO_PIT 0.7f
/* yaw 轴 速度 */
#define GIMBAL_PC_MOVE_RATIO_YAW 0.7f

/**************************射击 配置********************************/
/* 射速 */
#define DEFAULT_FRIC_WHEEL_SPEED 2000 //maximum value is 2500
/* 拨盘电机单发 */
#define TRIGGER_MOTOR_SPEED      -2000 //1500
/* 拨盘电机连发 */
#define C_TRIGGER_MOTOR_SPEED    -3000	//新步 /************************ 底盘硬件 配置 ****************************/
/* 麦轮半径(mm) */
#define RADIUS                 76
/* 麦轮周长(mm) */
#define PERIMETER              478

/*轮距（左右）*/
#define WHEELTRACK             375//415
/*轴距（前后）*/
#define WHEELBASE              365//406

/*云台偏移底盘中心X轴（前后）距离*/
#define GIMBAL_X_OFFSET        -9//130
/*云台偏移底盘中心Y轴（左右）距离*/
#define GIMBAL_Y_OFFSET        0//0

/* 底盘电机 3508 */
/* 底盘电机的减速比 */
#define CHASSIS_DECELE_RATIO (1.0f/19.0f)
/* 单一 3508 电机的 最大 转速, unit is rpm */
#define MAX_WHEEL_RPM        8500  //8347rpm = 3500mm/s
/* 底盘 最大 平移速度 , unit is mm/s */
#define MAX_CHASSIS_VX_SPEED 3300  //8000rpm
#define MAX_CHASSIS_VY_SPEED 3300
/* 底盘 最大 旋转速度 , unit is degree/s */
#define MAX_CHASSIS_VR_SPEED 480   //5000rpm
/*小陀螺转速*/
#define LG_SPEED	300
/*小陀螺或小猫步下最大VW*/
#define MAX_DODGE_SPEED	400

/************************** 云台硬件 配置 *****************************/
/* 电机编码值 和 角度（度） 的比率 */
#define ENCODER_ANGLE_RATIO    (360.0f/8192.0f)
/* pitch轴 电机 的 减速比 */
#define PIT_DECELE_RATIO       1.0f
/* yaw轴 电机 的 减速比 */
#define YAW_DECELE_RATIO       1.0f
/* pitch轴 正方向参数 */
#define PIT_MOTO_POSITIVE_DIR  -1.0f
/* yaw轴 正方向参数 */
#define YAW_MOTO_POSITIVE_DIR  1.0f
/* 拨盘 电机正方向参数 */
#define TRI_MOTO_POSITIVE_DIR  1.0f

/******************本末电机编码值和角度比率*********************/

#define BM_ENCODER_ANGLE_RATIO (360.0f/32768.0f)



/*********************** 系统 交互接口 配置 ****************************/

/* CAN 相关 */
#define CHASSIS_CAN       CAN1
#define RESCUE_CAN        CAN1
#define UPRAISE_CAN       CAN2
#define POWER_CAN         CAN1
#define SUPER_CAP_CAN     CAN1


#define CLAMP_EXCHANGE_CAN         	CAN2
#define DETECT_CAN        					CAN2
/* UART 相关 */
/**
  * @attention
  * close usart DMA receive interrupt, so need add 
  * uart_receive_handler() before HAL_UART_IROHandler() in uart interrupt function
**/
#define DBUS_HUART         huart1 //遥控接收器
#define JUDGE_HUART        huart3 //裁判系统接口
#define COMPUTER_HUART     huart6 //MINI电脑接口

/* 云台 相关 */
//#define PIT_ANGLE_MAX      15//18//15
//#define PIT_ANGLE_MIN      -25//20//-25
#define YAW_ANGLE_MAX      50
#define YAW_ANGLE_MIN      -50

/* 监测任务 相关 */
#define DEFAULT_TUNE       0	//音调

/*缓冲功率*/
#define POWER_BUFFER			 60

/*自瞄参数*/
#define	SYSTEM_DELAY	0.3

/* IMU 温度控制 */
#define DEFAULT_IMU_TEMP   50

/* 计算 相关 */
/* 弧度 系数 */
#define RADIAN_COEF        57.3f
/* 圆周率 */
#define PI                 3.142f

//判断最大最小值
#define VAL_LIMIT(val, min, max) \
do {\
if((val) <= (min))\
{\
  (val) = (min);\
}\
else if((val) >= (max))\
{\
  (val) = (max);\
}\
} while(0)\

#endif


