#ifndef _controller_H
#define _controller_H

#include "judge_rx_data.h"

#define CONTROLLER_DATA_LENGTH 12
#define JUDGE_FORMAT_LENGTH 9
#define MODEL_MODE 0
#define GYROSOPE_MODE 1

#define AS5600_NUMBER 6

#define CONTROLLER_ALL_DATA_LENGTH CONTROLLER_DATA_LENGTH+JUDGE_FORMAT_LENGTH

/**************比率换算************************/
#define MODEL_X_MAX 771
#define MODEL_Y_MAX 518
#define MODEL_Z_MAX 766

#define EXCHANGE_X_MAX 550
#define EXCHANGE_Y_MAX 550
#define EXCHANGE_Z_MAX 5500

#define SMALL_MODEL_X_RATIO ((2*3.14)/360)*6.33
#define SMALL_MODEL_Y_RATIO ((2*3.14)/360)*6.33
#define SMALL_MODEL_Z_RATIO ((2*3.14)/360)*6.33
#define SMALL_MODEL_PITCH_RATIO (25.5/1)

#define JOINT1_RATIO ((2*3.14)/360)*6.33
#define JOINT2_RATIO ((2*3.14)/360)*6.33
#define JOINT_YAW_RATIO  764/258
#define UPRAIES_RATIO    2640/800
#define PITCH_ROLL_RATIO     301/164
/**************比率换算************************/

typedef struct
{
	uint8_t cmd_id;
	
	int16_t pit_gyro_angle;
  int16_t roll_gyro_angle;
  int16_t yaw_gyro_angle;
	
	int16_t x_displacement;
	int16_t y_displacement;
	int16_t z_displacement;
}gyrosope_data_t;

typedef struct
{
	uint8_t cmd_id;
	uint8_t main_data[30];
}usart_data_t;

extern gyrosope_data_t gyrosope_data,gyrosope_past_data;
extern float small_model_data[6];
extern float  model_out_angle[AS5600_NUMBER];
void controller_task(void);
typedef enum
{
	// 锟斤拷锟斤拷为MOTOR_NAME 锟金级碉拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷
	
	UPRAISe,
	joInt_3_yaw,
	joInt_1,
	joInt_2,
	exchange_Pitch,
	exchange_Roll,

} motor_name_status2;


// 编码器多圈角度管理结构体
typedef struct {
    float current_angle;      // 当前单圈角度（范围：0°~360°或 -180°~180°）
    int32_t total_turns;      // 累计圈数（正为顺时针，负为逆时针）
    float total_angle;        // 多圈总角度 = total_turns * 360 + current_angle
    float last_angle;         // 上一次单圈角度（用于检测溢出）
	motor_name_status2 MOTOR_NAME;
} EncoderMultiTurn;

float UpdateMultiTurnAngle(EncoderMultiTurn* encoder, float* new_angle,motor_name_status2 MOTOR_NAME);



#endif
