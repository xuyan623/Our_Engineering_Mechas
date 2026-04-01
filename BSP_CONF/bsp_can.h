#ifndef _bsp_can_H
#define _bsp_can_H

#include "stm32f4xx.h"
#include "User.h"
#include "motor_task.h"


#define FILTER_BUF 5

#define Speed_Data 0x01
#define BusCurrent 0x02
#define QValue 0x03
#define RotorPosition 0x04
#define FaultInfo 0x05
#define WarningInfo 0x06
#define MOSTemperature 0x07
#define MotorWindingTemperature 0x08
#define CurrentMode 0x09
#define SystemVoltage 10
#define RotationCount 11
#define SystemState 12
#define AbsolutePosition 13
#define MaxPhaseCurrent 14

#define Mode_Current 0
#define	Mode_Speed 1
#define Mode_Position 2


#define DM10010L_P_MIN -12.5
#define DM10010L_P_MAX  12.5
#define DM10010L_V_MIN -25.0
#define DM10010L_V_MAX  25.0
#define DM10010L_T_MIN -200.0   
#define DM10010L_T_MAX  200.0   //200.0

#define DM4310_P_MIN -12.5
#define DM4310_P_MAX  12.5
#define DM4310_V_MIN -25.0
#define DM4310_V_MAX  25.0
#define DM4310_T_MIN -10.0
#define DM4310_T_MAX  10.0

#define DM4340_P_MIN -12.5
#define DM4340_P_MAX  12.5
#define DM4340_V_MIN -25.0
#define DM4340_V_MAX  25.0
#define DM4340_T_MIN -28.0
#define DM4340_T_MAX  28.0

#define DM_Kp_MAX 500
#define DM_Kp_MIN 0
#define DM_Kd_MAX 5
#define DM_Kd_MIN 0

/*3508的减速比 */
#define DECELE_RATIO_3508 (19.0f/1.0f)
/*大力3508  */
#define DECELE_RATIO_3508_2 (51.0f/1.0f)
/*2006的减速比*/
#define DECELE_RATIO_2006 (36.0f/1.0f)

/* CAN send and receive ID */
typedef enum
{
	CAN_3508_M1_ID              = 0x201,//底盘 3508
	CAN_3508_M2_ID              = 0x202,
	CAN_3508_M3_ID              = 0x203,
	CAN_3508_M4_ID              = 0x204,

  CAN_CHASSIS_ALL_ID          = 0x200,
	
	CAN_UPRAISE_ALL_ID   				= 0x1ff,
  
}
can1_msg_id_e;

typedef enum
{

	DJ_Gear_CAN_L_ID							 			= 0x200,
  DJ_Gear_CAN_H_ID             				= 0x1ff, 
	
	DJ_6020_CAN_L_ID                   =  0x1FE,
	DJ_6020_CAN_H_ID                   =  0x2FE,
	
	
	CAN_DM_4310_ID              = 0x3FE
	
}can2_msg_id_e;

typedef struct
{
  uint16_t ecd;                  //编码位
  uint16_t last_ecd;
  
  float  speed_rpm;            //转速
  int16_t  given_current;

  int32_t  round_cnt;
  int32_t  total_ecd;
  float  total_angle;          //角度
  float    DM_angle;
	
  uint16_t offset_ecd;
  uint32_t msg_cnt;					
  
  int32_t  ecd_raw_rate;
  int32_t  rate_buf[FILTER_BUF];
  uint8_t  buf_cut;
  int32_t  filter_rate;
	uint8_t  err_id;
	float  torque;
} moto_measure_t;


extern moto_measure_t moto_chassis[4];           
extern moto_measure_t Motor_CAN1_data[7];
extern moto_measure_t DJ_Motor_CAN2_data[7];
extern moto_measure_t DM_Motor_CAN1_data[7];
extern moto_measure_t DM_Motor_CAN2_data[7];


void get_moto_offset(moto_measure_t* ptr, CanRxMsg *message);
void get_BM_Moto_offset(moto_measure_t* ptr, CanRxMsg *message);
void DJ_encoder_data_handler(moto_measure_t* ptr, CanRxMsg *message);
void BM1010b_encoder_data_handler(moto_measure_t* ptr,CanRxMsg *message);
void DM_encoder_data_handler(moto_measure_t* ptr,CanRxMsg *message);


void send_can1_low_cur(int16_t iq1, int16_t iq2, int16_t iq3, int16_t iq4);
void send_can1_high_cur(int16_t iq1,int16_t iq2,int16_t iq3);

void send_gear_can2_low_cur(int16_t iq1,int16_t iq2,int16_t iq3,int16_t iq4);
void send_gear_can2_high_cur(int16_t iq1,int16_t iq2,int16_t iq3);
void send_6020_can1_high_cur(int16_t iq1,int16_t iq2,int16_t iq3);
void send_6020_can2_high_cur(int16_t iq1,int16_t iq2,int16_t iq3);

void Motor10010B_Current(int16_t Current1 ,int16_t Current2,int16_t Current3,int16_t Current4);
void Motor1010B_FeedBackmode(uint8_t motor_ID,uint8_t Interval,uint8_t Data0,uint8_t Data1,uint8_t Data2,uint8_t Data3);

void Motor10010B_Enable(void);
void DM_Enable_Send(uint8_t Stdid);
void DM_Enable(void);
void DM_Dis_Enable_Send(uint8_t Stdid);
void DM_Dis_Enable(void);
void DM_CAN1_Enable_Send(uint8_t Stdid);
void DM_CAN1_Dis_Enable_Send(uint8_t Stdid);

void DM_MIT(uint32_t Stdid,int angle_raw,int V_raw,int Kp_raw,int Kd_raw,int tff_raw);
void DM_CAN1_MIT(uint32_t Stdid,int angle_raw,int V_raw,int Kp_raw,int Kd_raw,int tff_raw);

void DM_MIT_send(motor_t* motor);
void DM_MIT_send_zer0(void);
void Motor10010L_Position(motor_t* motor);

void send_rc_data1(void);
void send_rc_data2(void);
void send_rc_data3(void);
void send_detect_state(void);

float uint_to_float (int x_int, float x_min, float x_max, int bits);
int float_to_uint(float x, float x_min, float x_max, int bits);

#endif 

