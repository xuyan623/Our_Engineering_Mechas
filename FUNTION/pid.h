#ifndef _pid_H
#define _pid_H

#include "stm32f4xx.h"
#include "User.h"
typedef enum
{
	INIT,
	DONE,
}INIT_STATUS;

enum
{
	NOW_ERR = 0,
	LAST_ERR,
	LLAST_ERR,
};

typedef struct pid
{
	double set;
	double get;
	float error[3];
  
	float kp;
	float ki;
	float kd;
	
	float pout;
	float iout;
	float dout;
	float out; 
	
	float maxout;
	float integral_limit;
  	float output_deadband;
	
	void (*f_pid_init)( struct pid *pid_t,
						float p,
						float i,
						float d,
						float max_out,
						float integral_limit);
	void (*f_pid_reset)(struct pid *pid_t,
						float p,
						float i,
						float d);
										
}pid_t;

typedef struct
{
	float p;
	float i;
	float d;
	float max_out;
	float integral_limit;
}pid_parameter_t;

typedef struct 
{
	float Kp;
  float Kd;
	float tff;
}mit_parameter_t;




float pid_calc(pid_t *pid,float get,float set);
float fuzzy_pid_calc(pid_t *pid, float get, float set);

void PID_Struct_Init(pid_t *pid,
					 float p,
					 float i,
					 float d,
					 float max_out,
					 float integral_limit,
					 INIT_STATUS init_status);

extern pid_t pid_spd[4];
extern pid_t pid_chassis_angle;
extern pid_t pid_chassis_pit_angle;
extern pid_t pid_imu_tmp;
extern pid_t pid_chassis_joint_leg_angle[2];
extern pid_t pid_chassis_joint_leg_spd[2];
#endif
