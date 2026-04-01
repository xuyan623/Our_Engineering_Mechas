#ifndef _modeswitch_task_H
#define _modeswitch_task_H


#include "stm32f4xx.h"
#include "string.h"

#define MEMSET(flag,type) (memset((type*)flag,0,sizeof(type)))

typedef enum
{
  INIT_NEVER = 0,
  INIT_DONE,
}global_state;

typedef enum
{
  CLAMP_VIEW = 0, 
  ODJIUST_VIEW,   
}view_switch_t;

typedef enum
{
  RELEASE_CTRL,
  MANUAL_CTRL,
//  SEMI_AUTOMATIC_CTRL,     
  ENGINEER_CTRL,           
}global_status;
typedef enum
{
  GIMBAL_RELEASE = 0,
  GIMBAL_NORMAL_MODE,
  GIMBAL_ENGINEER_MODE,
}gimbal_status;

typedef enum
{
  CHASSIS_RELEASE,
  CHASSIS_NORMAL_MODE,
  CHASSIS_RESCUE_MODE,
	CHASSIS_SUPPLY_MODE,        
	PITCH3_TORQUE_COLLECTION_MODE,  
	CHASSIS_URGENT_MEASURE,
	CHASSIS_EXCHANGE_MODE,          
	CLASSIS_PRIMARY_MODE,
	CHASSIS_GET_ENERGY_UNIT_MODE,
	CHASSIS_GET_ENERGY_UNIT1_MODE,
  CHASSIS_GET_ENERGY_UNIT2_MODE,
	CHASSIS_CLAMP_CATCH_MODE,
	CHASSIS_SECONDDARY_ORE_MODE,            
  CHASSIS_STOP_MODE,  
	CHASSIS_LEG_DOWN_MODE,
	CHASSIS_DEFEND_MODE,          
	CHASSIS_LEG_UP_MODE,						
}chassis_status;

typedef enum
{
  RESCUE_INIT = 0,
  RESCUE_ENABLE, 
}rescue_status;

typedef enum
{
  EXCHANGE_INIT = 0,
  EXCHANGE_ENABLE_MODE, 
	
}exchange_status;

typedef enum
{
  CLAMP_INIT = 0,	
  SMALL_ISLAND,           
  BIG_ISLAND_STRAIGHT,             
	BIG_ISLAND_SLANTED,             
	GROUND_MODE,            
	CATCH_MODE,								
	EXCHANGE_MODE,           
	TEST_MODE,	
	DEFEND_MODE,	
}clamp_status;

typedef enum
{
	SMALL_ISLAND_ORDINARY_MODE,            
	SMALL_ISLAND_AUTOMATIC_CLAMP_ONE_MODE, 
	SMALL_ISLAND_AUTOMATIC_MODE,           
}small_island_mode_t;

typedef enum
{
  BIG_ISLAND_ORDINARY_MODE,            
	BIG_ISLAND_AUTOMATIC_CLAMP_ONE_MODE, 	
}big_island_mode_t;

typedef enum
{
  BARRIER_CARRY_INIT = 0,
  BARRIER_CARRY_ENABLE, 
}barrier_carry_status;

/*补给控制模式*/
typedef enum
{
  SUPPLY_INIT = 0,
  SUPPLY_TO_HERO,  
}supply_status;

extern global_status global_mode; 
extern global_status last_global_mode;

extern chassis_status chassis_mode;
extern chassis_status last_chassis_mode;

extern rescue_status rescue_mode;
extern rescue_status last_rescue_mode;

extern clamp_status clamp_mode;
extern clamp_status last_clamp_mode;

extern barrier_carry_status barrier_carry_mode;
extern barrier_carry_status last_barrier_carry_mode;

extern supply_status supply_mode;
extern supply_status last_supply_mode;


extern small_island_mode_t small_island_mode;
extern big_island_mode_t   big_island_mode;

extern gimbal_status gimbal_mode;

extern view_switch_t view_switch;


void mode_switch_task(void *parm);
void get_last_mode(void);
void get_main_mode(void);
void get_chassis_mode(void);

//void get_rescue_mode(void);
void get_clamp_mode(void);
void get_barrier_carry_mode(void);
void get_supply_mode(void);
void get_gimbal_mode(void);
//void get_shoot_mode(void);

void get_monitor_display_mode(void);
void get_client_layer_mode(void);
void get_view_switch(void);

#endif






