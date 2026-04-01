#include "stm32f4xx.h"
#include "motor_8010.h"
#include "PID.h"
#include "dma.h"

#define PI 3.14159265358979f
#define KD 0.06
uint8_t Temp_buffer[16]; // 这个是缓存DMA的数据
// GO_Motorfield motor_recevie;//这个是将DMA的数据解算

float pos_target;
uint16_t Serial_RxData; 
uint8_t Serial_RxFlag;	

GO_Motorfield motor_recevie;

extern uint8_t motor_8010_TXbuf[TX_MAX_BUFLEN];
extern uint8_t motor_8010_RXbuf[RX_MAX_BUFLEN];

typedef struct
{
	float target_ecd;
	float target_speed;
	float kp_p;
	float kd_sp;
	float target_T_power;
	uint8_t safe;
} motor_measure; // 电机信息的结构体

motor_measure test; // 发送数据，在DUBGE里面改

/*****************电机信息的解析************************/
#define get_Go8010_motor_measure(ptr, data)                                                                                               \
	{                                                                                                                                       \
		int32_t pp;                                                                                                                           \
		int16_t vv,tt ;                                                                                                                       \
		(ptr)->id = data[2] & 0x0f;                                                                                                           \
		(ptr)->mode = data[2] >> 4;                                                                                                           \
		tt = (int16_t)((data[3]) | (data[4] << 8)) ;                                                                                          \
		vv = (int16_t)((data[5]) | (data[6]) << 8) ;                                                                                          \
			(ptr)->T = tt/256.0f;  	                                                                                                            \
			(ptr)->W =  vv * 2.0f * PI/256.f-0.122718468f;   		                                                                                      \
		pp = (int32_t)((data[7] | (uint32_t)data[8] << 8 | (uint32_t)data[9] << 16 | (uint32_t)data[10] << 24));                              \
			(ptr)->Pos = pp * 2.0f * PI / 32768.0f;                                                                                                \
	} 
/*****************************************************/

/*********************限幅函数********************/
#define LIMIT_MIN_MAX(x, min, max) (x) = (((x) <= (min)) ? (min) : (((x) >= (max)) ? (max) : (x)))
/*************************************************/

/***********************发送函数********************/

uint8_t sdata[17];
// id，前馈力矩，角度位置，角速度，Kp，Kd	
void GO_M8010_send_data(int id, float T, float Pos, float W, float K_P, float K_W)
{
	int16_t tff, v, kp, kd;
	int32_t pos = 0;
	uint16_t joint_mode = 1; // 1是控制模式

	LIMIT_MIN_MAX(T, T_MIN, T_MAX);
	LIMIT_MIN_MAX(W, V_MIN, V_MAX);
	LIMIT_MIN_MAX(Pos, (fp32)P_MIN, (fp32)P_MAX);

	tff = 256 * T;
	v = W * 256 / 6.2831f;
	pos = Pos * 32768 / 6.2831f;
	kp = K_P * 1280;
	kd = K_W * 1280;

	sdata[0] = 0xFE;
	sdata[1] = 0xEE;
	sdata[2] = (joint_mode & 0x0f) << 4 | id;
	sdata[3] = tff & 0x00ff;
	sdata[4] = tff >> 8;
	sdata[5] = v & 0xff;
	sdata[6] = v >> 8;
	sdata[7] = pos & 0xff;
	sdata[8] = pos >> 8;
	sdata[9] = pos >> 16;
	sdata[10] = pos >> 24;
	sdata[11] = kp & 0xff;
	sdata[12] = kp >> 8;
	sdata[13] = kd & 0xff;
	sdata[14] = kd >> 8;
	sdata[15] = crc_ccitt(&sdata[0], 15) & 0xff;
	sdata[16] = crc_ccitt(&sdata[0], 15) >> 8;

	for (int i = 0; i <= 16; i++)
	{
		motor_8010_TXbuf[i] = sdata[i];
	}
}
/***************************************************/

/***********************任务函数********************/

void go8010_receive()
{
	get_Go8010_motor_measure(&motor_recevie, motor_8010_RXbuf);
}

/***************************************************/
