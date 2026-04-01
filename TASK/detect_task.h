#ifndef _detect_task_H
#define _detect_task_H


#include "stm32f4xx.h"

#define Set_bit(data,bit) (data|=(1<<bit))
#define Reset_bit(data,bit) (data&=(~(1<<bit)))

typedef enum
{
  BOTTOM_DEVICE        = 0,
/*底盘*/
  CHASSIS_M1_OFFLINE   = 1,//离线设备ID表
  CHASSIS_M2_OFFLINE   = 2,
  CHASSIS_M3_OFFLINE   = 3,
  CHASSIS_M4_OFFLINE   = 4,
/*遥控*/	
  REMOTE_CTRL_OFFLINE  = 5,
/*夹取*/
  CLAMP_M1_OFFLINE     = 6,
	CLAMP_M2_OFFLINE     = 7,
/*姿态调整*/
	CLAMP_ATTITUDE_ADJUSTMENT_M1_OFFLINE = 8,
/*矿石夹紧*/          
//	CLAMP_CLAMPING_ORE_M1_OFFLINE   = 7,
//	CLAMP_CLAMPING_ORE_M2_OFFLINE   = 8,
/*抬升*/
  UPRAISE_M1_OFFLINE   = 9,
	UPRAISE_M2_OFFLINE   = 10,
/*救援*/
  RESCUE_M1_OFFLINE    = 11,
  RESCUE_M2_OFFLINE    = 12,
/*兑换挡板*/
	EXCHANGE_OFFLINE		 = 13,

  JUDGE_SYS_OFFLINE    = 14,
/*小电脑*/
  PC_SYS_OFFLINE       = 15, 
/*机械臂*/
	MANIPULATO_M0_OFFLINE= 16,
	MANIPULATO_M1_OFFLINE= 17,
	MANIPULATO_M2_OFFLINE= 18,
	MANIPULATO_M3_OFFLINE= 19,
	MANIPULATO_M4_OFFLINE= 20,
	MANIPULATO_M5_OFFLINE= 21,
	/*陀螺仪*/
	IMU_OFFLINE					 = 22,
	/*总个数*/
  ERROR_LIST_LENGTH    = 23,
}err_id;
typedef union
{
	uint32_t offline;
	struct//关键字struct能定义各种类型的变量集合，称为结构(structure)，并把它们视为一个单元。（结构体）
	{
    uint32_t m0 : 1; //结构体中的冒号表示位域。此行证明m0长度为1
    uint32_t m1 : 1;
    uint32_t m2 : 1;
    uint32_t m3 : 1;
    uint32_t m4 : 1;
		uint32_t m5 : 1;
		uint32_t m6 : 1;
		uint32_t m7 : 1;
		uint32_t m8 : 1;
		uint32_t m9 : 1;
    uint32_t m10 : 1;
    uint32_t m11 : 1;
    uint32_t m12 : 1;
		uint32_t m13 : 1;
		uint32_t m14 : 1;
		uint32_t m15 : 1;
		uint32_t m16 : 1;
		uint32_t m17 : 1;
    uint32_t reserve : 14;
	}bit;
}State_t;
typedef struct
{
  uint32_t last_times;
  uint32_t delta_times;
  uint16_t set_timeout;
  
}detect_param_t;

typedef struct
{
  uint8_t err_exist;
  detect_param_t param; 
  
}err_status;

typedef struct
{
  uint8_t    err_now_id[ERROR_LIST_LENGTH];
  err_status list[ERROR_LIST_LENGTH];//列表
  uint32_t   offline;
}global_err_t;

extern global_err_t global_err;
extern State_t state;

void detect_task(void *parm);
void detect_param_init(void);
void err_detector_hook(int err_id);
void module_offline_detect(void);



#endif

