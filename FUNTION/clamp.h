#ifndef _clamp
#define _clamp

#include "stdint.h"

#define UPRISE_DISPLACEMENT_TO_ANGLE -35


#define ACTIONING 0
#define ACTION_DONE 1
#define SECOND_ACTION_DONE 2
#define MAX_BOX_NUMBER 2
#define SLIDE_MAX_CHANGE 765

#define pi 3.1415926535


void clamp_angle_handle(void);

void normal_clmap_handler(void);
void pitch3_torque_collection_handler(void);
void get_energy_unit_handler(void);
void get_energy_unit1_handler(void);
void get_energy_unit2_handler(void);
uint8_t store_energy_unit_handler(void);
uint8_t store_energy_unit1_handler(void);

uint8_t chassis_leg_up_handler(void) ;
uint8_t store_handler(void);
uint8_t pick_handler(void);
uint8_t pick_handler1(void);
void exchange_handler(void);
uint8_t primary_handler(void);
uint8_t secondray_ore_handler(void);
uint8_t chassis_leg_down_handler(void) ;
void Motor_change_mode_angle(float* Mode_now,uint8_t low_ID,uint8_t hight_ID);

void Motor_uprise_angle_change(int16_t change_displacement);
void Motor_slide_angle_change(int16_t change_displacement);
void Motor_uprise_clamp_angle_change(float change_displacement);
void motor_all_angle_change(int motor_type,float ref_angle,float ratio);

extern uint8_t have_box_number;



void Polynomial_Trajectory_Planning(float start_angle,float final_angle,float t);   //默认起始的速度和最终的速度为0          
void Polynomial_Trajectory_running(int motor_type,uint32_t t,double a0,double a1,double a2,double a3);
void Change_Position_to_Motor_Angle (uint8_t Position_mode);
float normalize_angle(float theta);

#endif

