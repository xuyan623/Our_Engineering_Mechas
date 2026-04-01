#ifndef  _DEBUG_WAVE_H 
#define  _DEBUG_WAVE_H 

#include "stm32f4xx.h"


#define TIM_Fre  333     //采样频率(rtos的时间调度: 3ms)
#define Period   5 			 //周期
#define T_ms		 3
typedef enum
{
	Sine = 0,
	Square
}Sweep_Type;

typedef struct{

float sweep_freq;       //模拟频率
uint8_t A ;             //频率振幅

}debug_wave_t;


extern debug_wave_t debug_wave;
extern uint8_t sweep_vofa_stop;

float Sine_wave(debug_wave_t *Debug_wave);
float Sweep(void);
float Sawtooth_wave(debug_wave_t *Debug_wave);

#endif 
