#ifndef _comm_8010_task_H
#define _comm_8010_task_H

#include "stm32f4xx.h"
#include "User.h"
#include "motor_task.h"

void coom_8010_task(void *parm);
void go_8010_test_tesk1(int id,float T,float Pos,float W,float K_P,float K_W);
void go_8010_init(int id,float torque);
int go_8010_stop(int id);
float tamp_task(float input,float step,float ref_angle);
void ref_8010_angle(float get_angle[]);
void get_8010_angle(float receive_angle[]);
void go_8010_init_task(int GO_ID,float speed);
//float pitch2_torque_calcuate (float pitch2_init_angle,float pitch2_angle_fdb,float pitch1_angle_fdb);
float pitch2_polynomial_fitting(float pitch2_init_angle,float pitch2_angle_fdb,float pitch1_angle_fdb);
float pitch2_friction_fitting(float pitch2_speed_fdb);
float pitch2_grav_torque_calculate(void);

/*前馈控制参数*/
typedef struct
{
	//前馈补偿参数
	float a;
	float b;
	//缓存参数
	float rin;
	float lastRin;
	float perrRin;
}FFC;

#define Single_mode 1
#define Both_mode   0
#endif 
