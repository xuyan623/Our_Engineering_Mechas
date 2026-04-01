#ifndef __fuzzy_pid_H__
#define __fuzzy_pid_H__

#include "pid.h"
#include "math.h"
#include "stm32f4xx.h"

#define NN 7
#define NB -3
#define NM -2
#define NS -1
#define ZO 0
#define PS 1
#define PM 2
#define PB 3


typedef struct
{
	float e;					//误差
	float de;					//误差变化率
	float e_max;			//误差的最大值
	float de_max;			//误差变化率的最大值
	float Kp_max;			//pid的最大值
	float Ki_max;
	float Kd_max;
	float Ke;					//量化论域
	float Kde;
	float Ku_p;
	float Ku_i;
	float Ku_d;
	float percent;

}fuzzy_pid;

float trimf(float x, float a, float b, float c);
void fuzzy_count(void);
void fuzzy_calc(pid_t *pid);

#endif
