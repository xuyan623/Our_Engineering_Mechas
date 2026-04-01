/*****************************************************

*****************************************************/
#include "STM32_TIM_BASE.h"

#include "clamp.h"
#include "motor_task.h"
#include "modeswitch_task.h"
#include "remote_ctrl.h"
#include "controller.h"
#include "keyboard.h"
#include "delay.h"
#include "math.h"
#include "General_function.h"
#include "chassis_task.h"
Motor_angle_t Mode;

uint32_t clamp_record_time;
uint8_t action_one_state = ACTIONING;
uint8_t action_two_state = ACTIONING;
uint8_t action_three_state = ACTIONING;
uint8_t have_box_number = 0;
uint8_t have_box_number_samll = 0;

uint8_t current_limit_intit_flag = 0;
float last_angle[2]={0};

extern float vofa_debug[6];
extern float GO8010_init_angle1;
extern float pitch3_cur;
extern chassis_t chassis;

//这里有一个电流限幅的结构体，具体看老车代码

extern PoSition Robotic_arm_Position[7]; 
float theta1;
float theta2;

u8 chassis_leg_up_handler_time_flag = 0;

void clamp_angle_handle()
{
	uint8_t ID;

//这里有一个关于电流的限制函数，具体看老车代码，这里先删了

	switch (chassis_mode)//获取此时的车的模式
{
	case CHASSIS_NORMAL_MODE:  
	{
		normal_clmap_handler();
		clamp_action = CLAMP_UN_CMD;
	}
	break;
		case PITCH3_TORQUE_COLLECTION_MODE:
	{
		pitch3_torque_collection_handler();
	}
	break;	
	case CLASSIS_PRIMARY_MODE:
	{
		primary_handler();
	}
	break;	
		case CHASSIS_SECONDDARY_ORE_MODE:
	{
		secondray_ore_handler();
	}
	break;

	case CHASSIS_GET_ENERGY_UNIT_MODE:
	{
		get_energy_unit_handler();
	}
	break;
	case CHASSIS_GET_ENERGY_UNIT1_MODE:
	{
		get_energy_unit1_handler();
	}
	break;
		case CHASSIS_GET_ENERGY_UNIT2_MODE:
	{
		get_energy_unit2_handler();
	}
	break;
	case CHASSIS_EXCHANGE_MODE:
	{	
		 exchange_handler();
	}
	break;
	case CHASSIS_LEG_UP_MODE:
	{
		chassis_leg_up_handler();
	}break;
	case CHASSIS_LEG_DOWN_MODE:
	{
		chassis_leg_down_handler();
	}break;
	default:
	{
		clamp_action = CLAMP_UN_CMD;
		PUMP_OFF;
	}
	break;
	
}

//        if(global_mode == MANUAL_CTRL)
//	    {  
//			  Motor[Grip].Angle.mode_angle = 0;
//			}				
//	      else if(global_mode == ENGINEER_CTRL)
//			{
//				Motor[Grip].Angle.mode_angle = 2.2f;
//			}

//这里就是最终的角度计算
	for (ID = 0; ID < Motor_count; ID++) 
	{
		if (Motor[ID].MOTOR_NAME > CHASSIS_BR && Whether_Brushless_Motor(Motor[ID])) 
		{
			if (Motor[ID].MOTOR_TYPE != go_8010){
			     Motor[ID].Brushless.angle_ref = Motor[ID].Angle.normal_angle + Motor[ID].Angle.mode_angle + Motor[ID].Angle.offset_angle;
			}			
			else{	
				   Motor[ID].Brushless.angle_ref = Motor[ID].Angle.normal_angle + Motor[ID].Angle.offset_angle + Motor[ID].Angle.mode_angle;		
			}
			
		}
		else if (Whether_Brushless_Motor(Motor[ID]) == 0)
		{
			Motor[ID].Servo.angle_ref = Motor[ID].Angle.normal_angle + Motor[ID].Angle.mode_angle;
		}	
	}
	
	      Motor[Big_Yaw].Brushless.angle_ref += kb_adrust[YAW].angle;
	      Motor[Pitch1].Brushless.angle_ref  += kb_adrust[PITCH1].angle;
	      Motor[Pitch2].Brushless.angle_ref  += kb_adrust[PITCH2].angle;
	      Motor[Pitch3].Brushless.angle_ref  += kb_adrust[PITCH3].angle;
		    Motor[Roll2].Brushless.angle_ref  += kb_adrust[ROLL2].angle;
	      Motor[Roll3].Brushless.angle_ref  += kb_adrust[ROLL3].angle;

	
	
	  
		    Motor[Pitch2].Brushless.angle_ref+=rm.pit2_v;
	      Motor[Pitch1].Brushless.angle_ref+=rm.pit1_v;
	      Motor[Big_Yaw].Brushless.angle_ref+=rm.yaw_v;
	
	
	      rm.pit1_v = constrain(rm.pit1_v,-2.62-Motor[Pitch1].Angle.normal_angle - Motor[Pitch1].Angle.mode_angle,-Motor[Pitch1].Angle.normal_angle - Motor[Pitch1].Angle.mode_angle);
	      Motor[Pitch1].Brushless.angle_ref = constrain(Motor[Pitch1].Brushless.angle_ref,-2.62,0);                   //这个函数得注意好上下限，注意好正负号

	      rm.pit2_v = constrain(rm.pit2_v,-3.14*6.33-Motor[Pitch2].Angle.normal_angle-Motor[Pitch2].Angle.mode_angle,-Motor[Pitch2].Angle.normal_angle - Motor[Pitch2].Angle.mode_angle);
	 //     Motor[Pitch2].Brushless.angle_ref = constrain(Motor[Pitch2].Brushless.angle_ref,-3.14*6.33,0);     //切记，8010电机的堵转角度没写在angle_ref里面，后面在改一下吧            
	
	//在这里加上键鼠与自定义控制器的角度
//	Motor[UPRAISE].Brushless.angle_ref +=  model_out_angle[0] +kb_adrust[UPRISE].angle;
//	Motor[joint_1].Brushless.angle_ref += (model_out_angle[1] - kb_adrust[JOINT_1].angle);
//	Motor[joint_2].Brushless.angle_ref += (model_out_angle[2] + kb_adrust[JOINT_2].angle);
//	Motor[joint_3_yaw].Brushless.angle_ref += (model_out_angle[3] + kb_adrust[ADRUST_YAW].angle);


//	Motor[exchang_pitch_and_roll_l].Brushless.angle_ref += 2 * (-(kb_adrust[ADRUST_ROLL].angle + model_out_angle[5]) + (kb_adrust[ADRUST_PITCH].angle + model_out_angle[4]));
//	Motor[exchang_pitch_and_roll_r].Brushless.angle_ref += 2 * (-(kb_adrust[ADRUST_ROLL].angle + model_out_angle[5]) - (kb_adrust[ADRUST_PITCH].angle + model_out_angle[4]));

//	Motor[UPRAISE_CLAMP].Brushless.angle_ref += kb_adrust[ADRUST_Z].angle;
//	Motor[P_T_Pitch].Servo.angle_ref += kb_adrust[P_T_PITCH].angle;
//	Motor[P_T_Yaw].Servo.angle_ref += kb_adrust[P_T_YAW].angle;

	
//	if(Motor[UPRAISE].Brushless.angle_ref<Motor[UPRAISE].Angle.offset_angle)
//	{
//		kb_adrust[UPRISE].angle=last_angle[0];		
//	}
//	else
//	{
//		last_angle[0]=kb_adrust[UPRISE].angle;
//	}		
//		if(Motor[UPRAISE_CLAMP].Brushless.angle_ref<Motor[UPRAISE_CLAMP].Angle.offset_angle)
//	{
//		kb_adrust[UPRAISE_CLAMP].angle=last_angle[1];		
//	}
//	else
//	{
//		last_angle[1]=kb_adrust[UPRAISE_CLAMP].angle;
	//}		
	/******************�������*******************************/
	//	if(Motor[SLIDE_L].Brushless.angle_ref<Motor[SLIDE_L].Angle.offset_angle-SLIDE_MAX_CHANGE)
	//		Motor[SLIDE_L].Brushless.angle_ref=Motor[SLIDE_L].Angle.offset_angle-SLIDE_MAX_CHANGE;
	//	if(Motor[SLIDE_R].Brushless.angle_ref>Motor[SLIDE_R].Angle.offset_angle+SLIDE_MAX_CHANGE)
	//		Motor[SLIDE_R].Brushless.angle_ref=Motor[SLIDE_R].Angle.offset_angle+SLIDE_MAX_CHANGE;
}
 u8 chassis_leg_flag=0.0;

void normal_clmap_handler(void)
{
	uint8_t ID;

	 chassis_leg_up_handler_time_flag = 0;			
	for (ID = MOTOR_MIN_ID; ID <= MOTOR_MAX_ID; ID++){
		
		Motor[ID].Angle.mode_angle = 0;
	}
	     chassis.up_angle = 0.0f;

	PUMP_ON
	Exchange_ON
	CLAMP_ON
      chassis_leg_flag=0;

		if (have_box_number_samll > 0 || have_box_number > 0)
			PUMP_ON else PUMP_OFF
}

void get_energy_unit_handler(void)
{    
	  switch (clamp_action)
  {
	    case CLAMP_UN_CMD:
	  {
		  clamp_record_time = HAL_GetTick();
			Motor_change_mode_angle(&Mode.get_energy_angle, Big_Yaw, MOTOR_MAX_ID); 
			action_one_state =   ACTIONING;
			action_two_state =   ACTIONING;
			action_three_state = ACTIONING;
	
		}break;
	  
			case ACTION_ONE:
	 {
		 if(action_one_state == ACTIONING)
		{			
			 motor_all_angle_change(Grip,0,1);	
			 clamp_record_time = HAL_GetTick();
		}	   
	 }
	break;
			case ACTION_TWO:
		{
				if(action_two_state == ACTIONING)
		  {
				 motor_all_angle_change(Pitch3,0.43,1);	
				 motor_all_angle_change(Pitch2,-0.43+1.19447,-6.33);			
				 clamp_record_time = HAL_GetTick();
			}
		}break; 	 
			case ACTION_THREE:
		{
			if (action_three_state == ACTIONING)
			action_three_state = store_energy_unit_handler();
		else
			clamp_action = CLAMP_UN_CMD;
	
		}break;
	
	}
}

void get_energy_unit1_handler(void)
{
 switch (clamp_action)
  {
	    case CLAMP_UN_CMD:
	  {
		  clamp_record_time = HAL_GetTick();
			Motor_change_mode_angle(&Mode.get_energy_angle1, Big_Yaw, MOTOR_MAX_ID); 
			action_one_state =   ACTIONING;
			action_two_state =   ACTIONING;
			action_three_state = ACTIONING;
	
		}break;	  
			case ACTION_ONE:
	 {
		 if(action_one_state == ACTIONING)
		{	
			 motor_all_angle_change(Grip,0,1);	
			 clamp_record_time = HAL_GetTick();
		}			   
	 }
	break;
			case ACTION_TWO:
		{
				if(action_two_state == ACTIONING)
		  {
				 clamp_record_time = HAL_GetTick();
			}
		}break; 	 
			case ACTION_THREE:
		{
//			if (action_three_state == ACTIONING)				
//		else
//			clamp_action = CLAMP_UN_CMD;
	
		}break;
	
	}
}

void get_energy_unit2_handler(void)
{
 switch (clamp_action)
  {
	    case CLAMP_UN_CMD:
	  {
		  clamp_record_time = HAL_GetTick();
			Motor_change_mode_angle(&Mode.get_energy_angle2, Big_Yaw, MOTOR_MAX_ID); 
			action_one_state =   ACTIONING;
			action_two_state =   ACTIONING;
			action_three_state = ACTIONING;
	
		}break;	  
			case ACTION_ONE:
	 {
		 if(action_one_state == ACTIONING)
		{	
			 motor_all_angle_change(Grip,0,1);	
			 clamp_record_time = HAL_GetTick();
		}			   
	 }
	break;
			case ACTION_TWO:
		{
				if(action_two_state == ACTIONING)
		  {  
				 store_energy_unit1_handler();
			}
		}break; 	 
			case ACTION_THREE:
		{
//			if (action_three_state == ACTIONING)
//				
//		else
//			clamp_action = CLAMP_UN_CMD;
	
		}break;
	
	}
}

uint8_t store_energy_unit_handler(void)
{    
	  u8 store_flag = 0;
  	uint32_t store_action_times;
	  store_action_times = HAL_GetTick() - clamp_record_time;
	
  	Motor_change_mode_angle(&Mode.store_energy_angle, Big_Yaw, MOTOR_MAX_ID); 
	
		if (store_action_times > 800 && store_action_times < 1150)
	{ 
		 motor_all_angle_change(Grip,-1.8,1);
	}	

	return store_flag;
}

uint8_t store_energy_unit1_handler(void) 
{    
	  u8 store_flag = 0;
  	uint32_t store_action_times;
	  store_action_times = HAL_GetTick() - clamp_record_time;
	
  	Motor_change_mode_angle(&Mode.store_energy_angle1, Big_Yaw, MOTOR_MAX_ID); 
	
		if (store_action_times > 1150 && store_action_times < 1400)
	{ 
		 motor_all_angle_change(Grip,-1.8,1);
	}	

	return store_flag;
}

void pitch3_torque_collection_handler(void)
{
  static	u8 flag = 0;
	if((Motor[Pitch3].Brushless.angle_fdb - Motor[Pitch3].Angle.offset_angle )<  -180 )
	{
	   flag=1;
  }
	 Motor[Pitch3].Brushless.spd_ref = -70;  

	if(flag == 1){
	 Motor[Pitch3].Brushless.spd_ref = 200;   	 
	Motor[Pitch3].Brushless.controller.pid_ctrl.speed_pid.kp =  7; 
	}
	 float q_6,q_3,q_2;
	   float q_int2,q_int3,q6_int;
   //,q6,q3   
   q6_int = Motor[Pitch3].Angle.offset_angle*3.14/180;

	 float cos_q4;
	 
	 q_int2 = 0.226893; q_int3 = -0.3665;	
	 q_2 = q_int2 - Motor[Pitch1].Brushless.angle_fdb;  //Motor[Pitch1].Brushless.angle_fdb          
	 q_3 =(Motor[Pitch2].Brushless.angle_fdb-GO8010_init_angle1)/6.33 + q_int3;   //q3也是
   q_6 = -Motor[Pitch3].Brushless.angle_fdb*3.14/180 + q6_int + 1.57 ;     //这里的q6_int的含义与q3_int，q2_int的含义有区别，这里主要是加了个1.57，所以电机角度取负

   vofa_debug[0] =q_2+q_3+q_6 ;   //要用的时候要把inforget的任务注释掉
 //pitch3_cur =  vofa_debug[0];
 //Motor[Pitch3].Brushless.spd_ref = 200;   
}
//uint32_t urgent_action_times;
//void urgent_measure_handler (void)
//{   

//	switch (clamp_action)
//{
//	case CLAMP_UN_CMD:
//				
//	{ 	   
//		
//	     urgent_action_times=HAL_GetTick();		
//	     PUMP_ON
//	//		 motor_all_angle_change(joint_1,-9.7683897+0.52*6.33,1);
//	//	   motor_all_angle_change(joint_2,7.55945778+0.52*6.33,1);
////    motor_all_angle_change(joint_1,-1.5458588,1);
////		 motor_all_angle_change(joint_3_yaw,-331/3,3);
////		 motor_all_angle_change(joint_2,3.5838575,1);
////		 motor_all_angle_change(UPRAISE_CLAMP,9.2,-35);
////	  motor_all_angle_change(P_T_Pitch,61,1);
////	  motor_all_angle_change(P_T_Yaw,-133,1);
//		

//	     action_one_state = ACTIONING;
//	}break;
//	case ACTION_ONE:
//	{
//		if (action_one_state == ACTIONING)
//		{
//			if ((HAL_GetTick() - urgent_action_times) > 20 && (HAL_GetTick() - urgent_action_times) < 100)
//			{ 
//						     Motor_change_mode_angle(&Mode.store_angle3, P_T_Yaw, MOTOR_MAX_ID); 

//			
//			}	
//			
//		
//		  else if ((HAL_GetTick() - urgent_action_times) > 500 && (HAL_GetTick() - urgent_action_times) < 550)
//		   {
//				Exchange_OFF
//				have_box_number_samll++;
// 
//				 
//			}
//		}
//	}
//	break;
//		
//		}
//}


//int16_t test_final = 0;
//void big_island_stright_clamp_handler(void)
//{
//	switch (clamp_action)
//	{
//	case CLAMP_UN_CMD:
//	{
//		
//		Motor_change_mode_angle(&Mode.bigisland_straight_angle, P_T_Yaw, MOTOR_MAX_ID); 
//		clamp_record_time = HAL_GetTick();
//		action_one_state = ACTIONING;
//		action_two_state = ACTIONING;
//		PUMP_ON
//		CLAMP_ON
//	}
//	break;
//	case ACTION_ONE:
//	{
//		if (action_one_state == ACTIONING)
//		{
//			if ((HAL_GetTick() - clamp_record_time) > 200 && (HAL_GetTick() - clamp_record_time) < 250)
//			{

//				motor_all_angle_change(UPRAISE,381,1);

//			}
//			else if ((HAL_GetTick() - clamp_record_time) > 1500)
//			{
//				if (have_box_number < MAX_BOX_NUMBER)
//					have_box_number++;
//				action_one_state = ACTION_DONE;
//			}
//		}
//		else
//			clamp_record_time = HAL_GetTick();
//	}
//	break;
//	case ACTION_TWO:
//	{
//		if (action_two_state == ACTIONING)
//			action_two_state = store_handler();
//		else
//			clamp_action = CLAMP_UN_CMD;
//	}
//	break;
//	}
//}

//uint8_t store_big_island_slanted_second_box = 0;
//void big_island_slanted_clamp_handler(void)
//{
//	switch (clamp_action)
//	{
//	case CLAMP_UN_CMD:
//	{
//		
//		Motor_change_mode_angle(&Mode.bigisland_slanted_angle, P_T_Yaw, MOTOR_MAX_ID);
//		clamp_record_time = HAL_GetTick();
//		action_one_state = ACTIONING;
//		action_two_state = ACTIONING;
//		PUMP_ON
//		CLAMP_ON
//	}
//	break;
//	case ACTION_ONE:
//	{ 
//		 uint32_t big_island_slanted_action_times;
//		 big_island_slanted_action_times = HAL_GetTick() - clamp_record_time;
//		if (action_one_state == ACTIONING)
//		{
//      if (big_island_slanted_action_times > 0 && big_island_slanted_action_times < 200)		
//			{   
//				
//				Polynomial_Trajectory_running(joint_1,big_island_slanted_action_times,0,0,0.000341,-0.00000113750);
//				Polynomial_Trajectory_running(joint_2,big_island_slanted_action_times,0,0,0.000341,-0.00000113750);
//				
//			}
//			

//			
//			else if ((HAL_GetTick() - clamp_record_time) > 1000)
//			{
//				if (have_box_number < MAX_BOX_NUMBER)
//					have_box_number++;
//				action_one_state = ACTION_DONE;
//			}
//		}
//		else
//		{
//			clamp_record_time = HAL_GetTick();
//		}
//	}
//	break;
//	case ACTION_TWO:
//	{
//		if (action_two_state == ACTIONING)
//			action_two_state = store_handler();
//		else
//			clamp_action = CLAMP_UN_CMD;
//	}
//	break;
//	}	
//}
//uint32_t time_test = 0;
//void small_island_clamp_handler(void)
//{

//	switch (clamp_action)
//	{
//	case CLAMP_UN_CMD:
//	{
//		
//		Motor_change_mode_angle(&Mode.smallisland_angle, P_T_Yaw, MOTOR_MAX_ID);
//		clamp_record_time = HAL_GetTick();
//		action_one_state = ACTIONING;
//		action_two_state = ACTIONING;
//		action_three_state = ACTIONING;
//		PUMP_ON
//		CLAMP_ON
//		Exchange_ON;
//	}
//	break;
//	case ACTION_ONE:
//	{
//		       
//		if (action_one_state == ACTIONING){

//			if ((HAL_GetTick() - clamp_record_time) > 200 && (HAL_GetTick() - clamp_record_time < 250))
//			{
//				motor_all_angle_change(UPRAISE_CLAMP,0,UPRISE_DISPLACEMENT_TO_ANGLE);
//			}
//			else if ((HAL_GetTick() - clamp_record_time) > 1000 && (HAL_GetTick() - clamp_record_time < 1100))
//			{
//				motor_all_angle_change(UPRAISE_CLAMP,23,UPRISE_DISPLACEMENT_TO_ANGLE);
//			}
//			else if ((HAL_GetTick() - clamp_record_time) > 1600 && (HAL_GetTick() - clamp_record_time < 1650))
//			{
//				
//				motor_all_angle_change(clamp_pitch_and_roll_l,0,2.3);
//				motor_all_angle_change(clamp_pitch_and_roll_r,0,2.3);
//				motor_all_angle_change(CLAMP_SILD,0,45);
//							
//			}
//		     
//			 if(have_box_number_samll==0 ){
//		   if ((HAL_GetTick() - clamp_record_time) > 2100 && (HAL_GetTick() - clamp_record_time < 4800))
//			{
//		    
//	        if (action_one_state == ACTIONING)
//					{
//			        action_one_state = store_handler();
//						    if(action_one_state == 1){
//						      	clamp_action = CLAMP_UN_CMD;
//						        have_box_number_samll++;					
//					}
//		    }						
//		  }		 
//		}
//				 else if(have_box_number_samll>0 && (HAL_GetTick() - clamp_record_time > 1750))
//				 {				 
//				    action_one_state = 1;				 
//				 } 
//				 
//				 
//				 
//				 
//	}  
//		 else{
//	 		   	clamp_record_time = HAL_GetTick();
//	       }	 
//	}		
//	break;
//			
//	case ACTION_TWO:
//{
//		  if (action_two_state == ACTIONING){
//						
//			if ((HAL_GetTick() - clamp_record_time) > 20 && (HAL_GetTick() - clamp_record_time < 70))
//		{
//			Motor_change_mode_angle(&Mode.store_angle, clamp_pitch_and_roll_l, MOTOR_MAX_ID);
//		}

//		else if ((HAL_GetTick() - clamp_record_time) > 700 && (HAL_GetTick() - clamp_record_time < 750))
//		{
//			CLAMP_OFF
//		}
//		else if ((HAL_GetTick() - clamp_record_time) > 1000 && (HAL_GetTick() - clamp_record_time < 1050))
//		{
//		   motor_all_angle_change(joint_2,1,6.33);
//			
//		}

//			else if ((HAL_GetTick() - clamp_record_time) > 1400 && (HAL_GetTick() - clamp_record_time < 1450))

//		{
//				clamp_action = CLAMP_UN_CMD;
//		}		

//	}
//}
//	break;
//}
//}	
//void clamp_ground_handler(void)
//{
//	switch (clamp_action)
//	{
//	case CLAMP_UN_CMD:
//	{
//		
//		clamp_record_time = HAL_GetTick();
//		action_one_state = ACTIONING;
//		action_two_state = ACTIONING;
//		PUMP_ON
//		motor_all_angle_change(UPRAISE,2305,1);

//	}
//	break;
//	case ACTION_ONE:
//	{
//		if (action_one_state == ACTIONING)
//		{  
//			action_one_state=pick_handler();
//		}	
//		else
//		{
//			action_one_state = ACTION_DONE;		
//			clamp_record_time = HAL_GetTick();
//		}
//		   
//	}
//	break;
////	case ACTION_TWO:
////	{
////		if (action_two_state == ACTIONING)
////			action_two_state = store_handler();
////		else
////			clamp_action = CLAMP_UN_CMD;
////	}
////	break;
//	}
//}

//uint8_t ready_to_del_box_number_flag = 1;
void exchange_handler(void)
{

	switch (exchange_action)
	{
	
	case EXCHANGE_UN_CMD:
	{
		Motor_change_mode_angle(&Mode.exchange_angle, YAW, MOTOR_MAX_ID);
		clamp_record_time = HAL_GetTick();
		action_two_state = ACTIONING;
	}break;
	case PICK_ACTION1:
	{
		if (action_one_state == ACTIONING)
			action_one_state = pick_handler();
		else
		{
			exchange_action = EXCHANGE_UN_CMD;
			clamp_record_time = HAL_GetTick();
		}
	}
	break;
	case PICK_ACTION2:
	{
		if (action_two_state == ACTIONING)
				action_two_state = pick_handler1();
			else
			{
				exchange_action = EXCHANGE_UN_CMD;
				clamp_record_time = HAL_GetTick();
			}
	}
	break;
	}
}

//uint8_t store_handler(void)
//{

//	uint8_t finish_flag = 0;
//	uint32_t store_action_times;
//	store_action_times = HAL_GetTick() - clamp_record_time;

//	if (CHASSIS_STORE_ENERGY_INIT_MODE || chassis_mode == CHASSIS_CLAMP_BIG_ISLAND_STRAIGHT_MODE)
//	{
//		if (store_action_times > 10 && store_action_times < 100)
//		{
//		   motor_all_angle_change(UPRAISE,2505,1);
//		   motor_all_angle_change(UPRAISE_CLAMP,0,-35);
//		}
//		else if (store_action_times > 800 && store_action_times < 870)
//		{		
//			Motor_change_mode_angle(&Mode.store_angle2, P_T_Yaw, MOTOR_MAX_ID);	
//		}
//		else if(store_action_times > 1800 && store_action_times < 1860)
//		{
//		    motor_all_angle_change(exchang_pitch_and_roll_l,-110,2);
//			motor_all_angle_change(exchang_pitch_and_roll_r,-110,-2);

//		}
//		else if (store_action_times > 2200 && store_action_times < 2260)
//		{
//			motor_all_angle_change(UPRAISE,1700,1);
//			
//			
//		}
//		else if (store_action_times > 2300 && store_action_times < 2350)
//		{
//			Exchange_OFF
//		}
//		
//		else if (store_action_times > 2600 && store_action_times < 2650)
//		{
//			
//           motor_all_angle_change(UPRAISE,2400,1);	
//		}
//		
//	   else	if (store_action_times >= 3200 )
//		{
//			finish_flag = 0;
//		}
//	}
//	else if (chassis_mode == CLASSIS_PRIMARY_MODE)
//	{
//    	if (store_action_times > 2150 && store_action_times < 2190)
//		{
//			Motor_change_mode_angle(&Mode.store_angle, clamp_pitch_and_roll_l, MOTOR_MAX_ID);
//		}

//		else if (store_action_times > 2870 && store_action_times < 2910)
//		{
//			CLAMP_OFF
//		}
//		else if (store_action_times >= 3400 && store_action_times < 3450)
//		{
//		motor_all_angle_change(joint_2,1,6.33);
//			
//			//Motor[joint_1].Angle.mode_angle = 0 * 6.33;
//		}

//		else if (store_action_times >= 3490)

//		{
//			finish_flag = 1;
//		}
//	}


//	return finish_flag;
//}


uint8_t pick_handler(void)
{
	uint8_t finish_flag = 0;
	uint32_t pick_action_times;
	pick_action_times = HAL_GetTick() - clamp_record_time;
	
	if (chassis_mode == CHASSIS_EXCHANGE_MODE )
 {
		if (pick_action_times > 100 && pick_action_times < 500) 
		{
			Motor_change_mode_angle(&Mode.store_energy_angle, YAW, MOTOR_MAX_ID);	
		}
		else if (pick_action_times > 1200 && pick_action_times < 1260)
		{
			Motor_change_mode_angle(&Mode.exchange_pick_angle, YAW, MOTOR_MAX_ID);	
		}
		else if (pick_action_times > 1400 && pick_action_times < 1450)
		{
				motor_all_angle_change(Pitch2,1.04067,-6.33);
				motor_all_angle_change(Roll2,0.05,1);	
		}
		else if(pick_action_times > 1550 && pick_action_times < 1600)
		{
				motor_all_angle_change(Pitch2,0.82,-6.33);   //上——正
				motor_all_angle_change(Pitch3,-1.0,1);
				motor_all_angle_change(Roll2,-0.04512,1);  // 逆时针——负
				motor_all_angle_change(Roll3,-49.3427,1);
		}
			else if(pick_action_times > 1800 && pick_action_times < 1850)
		{
			  motor_all_angle_change(Grip,0,1);	
		}
		else if (pick_action_times >= 2000 && pick_action_times < 2050)
		{
				motor_all_angle_change(Pitch2,1.3,-6.33);	
		}
		else if (pick_action_times >= 2350)
		{
				finish_flag = 1;		
		}
  }	
    return finish_flag;
	  
}

uint8_t pick_handler1(void)
{
	uint8_t finish_flag = 0;
	uint32_t pick_action_times;
	pick_action_times = HAL_GetTick() - clamp_record_time;
	
	if (chassis_mode == CHASSIS_EXCHANGE_MODE )
 {
		if (pick_action_times > 100 && pick_action_times < 500) 
		{
			Motor_change_mode_angle(&Mode.store_energy_angle1, YAW, MOTOR_MAX_ID);	
		}
		else if (pick_action_times > 1200 && pick_action_times < 1260)
		{
			Motor_change_mode_angle(&Mode.exchange_pick_angle1, YAW, MOTOR_MAX_ID);		
		}
		else if (pick_action_times > 1300 && pick_action_times < 1350)
		{ 		
				motor_all_angle_change(Pitch2,0.87,-6.33);
				motor_all_angle_change(Pitch3,-1.19,1);             
		}
				else if (pick_action_times > 1460 && pick_action_times < 1500)
		{   
		  	motor_all_angle_change(Big_Yaw,-2.00189481,1);
				motor_all_angle_change(Pitch2,0.810012383,-6.33);
				motor_all_angle_change(Pitch3,-1.11,1);
    		motor_all_angle_change(Pitch1,1.20,-1);
		}	
		else if(pick_action_times > 2170 && pick_action_times < 2220)
		{
				motor_all_angle_change(Grip,0,1);
		}
			else if(pick_action_times > 2390 && pick_action_times < 2430)
		{
				motor_all_angle_change(Pitch2,1.6,-6.33);
				motor_all_angle_change(Pitch3,-1.45,1);
		}
		else if (pick_action_times >= 2820)
		{
				finish_flag = 1;
		}	
 }	
    return finish_flag;  
}


extern u8 primary_turn_ore_flag;    //这个是负责调换车头的模式
uint8_t primary_handler(void)
{    
	  switch (clamp_action)
  {			
	    case CLAMP_UN_CMD:
	  {
			
		  motor_all_angle_change(Pitch2,1.0,-6.33);	
			motor_all_angle_change(Pitch1,0.6,-1);	
		  clamp_record_time = HAL_GetTick();
			
			action_one_state =   ACTIONING;
			action_two_state =   ACTIONING;
			action_three_state = ACTIONING;
	
		}break;
	  
			case ACTION_ONE:
	 {
		 if(action_one_state == ACTIONING)
		{			
		
        	Motor_change_mode_angle(&Mode.primary_angle , Big_Yaw, MOTOR_MAX_ID);
		
		}	   
	 }
	break;
			case ACTION_TWO:
		{
				if(action_two_state == ACTIONING)
		  { 
				if(	primary_turn_ore_flag == 1)
				{ 
					motor_all_angle_change(Roll3,180,1);
				}
				else
				motor_all_angle_change(Roll3,0,1);
					
			
			
			}
		}break; 	
	
			case ACTION_THREE:
		{
        motor_all_angle_change(Grip,-1.8,1);
					motor_all_angle_change(Pitch2,2.5,-6.33);	
				
			
		}break;
	
	}
}

uint8_t secondray_ore_handler(void)   
{    
	  switch (clamp_action)
  {
	    case CLAMP_UN_CMD:
	  {
		  clamp_record_time = HAL_GetTick();			
			action_one_state =   ACTIONING;
			action_two_state =   ACTIONING;
			action_three_state = ACTIONING;
			motor_all_angle_change(Pitch1,1.38691,-1);
			motor_all_angle_change(Pitch2,1,-6.33);
		}break;
	  
			case ACTION_ONE:
	 {
		 if(action_one_state == ACTIONING)
		{			
			if((HAL_GetTick()-clamp_record_time) >=50 && (HAL_GetTick()-clamp_record_time) <=100 )	
		    Motor_change_mode_angle(&Mode.secondray_ore_angle , Big_Yaw, MOTOR_MAX_ID);
			
			else if((HAL_GetTick()-clamp_record_time) >=400 && (HAL_GetTick()-clamp_record_time) <=450 )	
				motor_all_angle_change(Pitch3,-1.6,1);

		}	   
	 }
	break;
			case ACTION_TWO:
		{
				if(action_two_state == ACTIONING)
		  {
				
			}
		}break; 	


		
			case ACTION_THREE:
		{


			
		}break;
	
	}
}
uint8_t chassis_leg_up_handler(void)   
{    
	  switch (clamp_action)
  {
	    case CLAMP_UN_CMD:
	  {
			action_one_state =   ACTIONING;
			action_two_state =   ACTIONING;
			action_three_state = ACTIONING;
			
			  if(chassis_leg_up_handler_time_flag == 0)
		  {
			  clamp_record_time = HAL_GetTick();	
        chassis_leg_up_handler_time_flag = 1;			
			}

    u32 chassis_leg_up_handler_time = HAL_GetTick() - clamp_record_time;

			if(chassis_leg_up_handler_time > 10 && chassis_leg_up_handler_time < 100)
			{		
				chassis.up_angle = 35.0f;
			}
			else if(chassis_leg_up_handler_time > 850 && chassis_leg_up_handler_time < 900)
			{
				  	Motor_change_mode_angle(&Mode.chassis_leg_up_angle,Big_Yaw,MOTOR_MAX_ID);
			}
		
		}break;
	  
			case ACTION_ONE:
	 {
		 if(action_one_state == ACTIONING)
		{		
			
		}	   
	 }
	break;
			case ACTION_TWO:
		{
				if(action_two_state == ACTIONING)
		  {

			}
		}break; 	


		
			case ACTION_THREE:
		{


			
		}break;
	
	}
}

uint8_t chassis_leg_down_handler(void)   
{    
	  switch (clamp_action)
  {
	    case CLAMP_UN_CMD:
	  {

			action_one_state =   ACTIONING;
			action_two_state =   ACTIONING;
			action_three_state = ACTIONING;
		  clamp_record_time = HAL_GetTick();			


		}break;
	  
			case ACTION_ONE:
	 {
		 if(action_one_state == ACTIONING)
		{			
				 // chassis.up_angle = tamp_task(chassis.joint_leg_angle_fdb[right_joint]-4.4-rm.pit_leg,0.3,-20);

		}	   
	 }
	break;
			case ACTION_TWO:
		{
				if(action_two_state == ACTIONING)
		  {

			}
		}break; 	


		
			case ACTION_THREE:
		{


			
		}break;
	
	}
}




//void Urgent_exchange_handler(void)
//{

////	#define Air_Pump_Exchange_ON Air_Pump_Clamp_ON ;
////	#define Air_Pump_Exchange_OFF Air_Pump_Clamp_OFF ;
////	#define Exchange_ON CLAMP_ON ;
////	#define Exchange_OFF CLAMP_OFF ;
////	
//}
//void check_handler(void)
//{
//	
//	Motor_change_mode_angle(&Mode.check_angle, UPRAISE, MOTOR_MAX_ID);
//}

////轨迹规划 ---- 给出起始的位置和末尾的位置还有预期运动的时间，即可得出运动方程的参数a                                  
//void Polynomial_Trajectory_Planning(float start_angle,float final_angle,float t)   //默认起始的速度和最终的速度为0          
//{

//     double a0,a1,a2,a3;             //   theta = a0+a1*t+a2*t2+a3*t3;                  
//	                                 //    dtheta = a1+2*a2*t+3*a3*t2;       
//     float t2,t3;                                      
//	    t2 = t*t;             
//	    t3 = t*t*t; 
//	
//	   float eq1,eq2;
//	                                 
// 
//	    a0 = start_angle;
//			a1 = 0;                                  
//			
//      eq1 = final_angle-a0; //a2*t2+a3*t3
//	    eq2 = 0;              //2*a2*t2+3*a3*t3
//	
//	    a2 = (eq1*3 - eq2*t)/t2;
//	    a3 = (eq2*t - 2*eq1)/t3;
//	
//}
// 
//void Polynomial_Trajectory_running(int motor_type,uint32_t t,double a0,double a1,double a2,double a3)
//{
//   uint32_t t2,t3;
//   t2 = t*t;
//	 t3 = t*t*t;
//	Motor[motor_type].Angle.mode_angle = a0+a1*t+a2*t2+a3*t3;
////  Motor[motor_type].Brushless.spd_ref += a1+2*a2*t+3*a3*t2;


//}

void Motor_change_mode_angle(float *Mode_now, uint8_t low_ID, uint8_t hight_ID)
{
	uint8_t ID;
	uint32_t mode_angle_adress;
	uint32_t now_angle_adress;
	uint32_t target_adress;
	mode_angle_adress = (uint32_t)&Mode.mode_angle;
	now_angle_adress = (uint32_t)Mode_now;
	for (ID = low_ID; ID <= hight_ID; ID++)
	{
		target_adress = ((uint32_t)&Motor[ID].Angle.mode_angle + now_angle_adress - mode_angle_adress);
		Motor[ID].Angle.mode_angle = *(float *)target_adress;
	}
}

//void Motor_uprise_clamp_angle_change(float change_displacement)
//{
//	float change_angle;
//	change_angle = change_displacement * UPRISE_DISPLACEMENT_TO_ANGLE;
//	Motor[UPRAISE_CLAMP].Angle.mode_angle = change_angle;
//	// Motor[UPRISE_R].Angle.mode_angle+=change_angle;
//}

//void Motor_slide_angle_change(int16_t change_displacement)
//{
//	int16_t change_angle;
//	change_angle = change_displacement ;

//}


void motor_all_angle_change(int motor_type,float ref_angle,float ratio)
{
	Motor[motor_type].Angle.mode_angle=ref_angle*ratio;
}



//void Change_Position_to_Motor_Angle (float x,float y,float z)
//{
//      float a2,d4,    
//     a2=0.275; d4 = 0.3;	   d6=0.057;;


//                                                     
//     x=Robotic_arm_Position[normal_mode].x;
//     y=Robotic_arm_Position[normal_mode].y;
//     z=Robotic_arm_Position[normal_mode].z;
//	   Kx=x;     //先默认theta0 为0度  //Kx=(x-d2*sin(theta0_1))/cos(theta0_1);
//	
//     A=-2*a2*z-2*d3*Kx;
//     B=-2*a2*Kx+2*d3*z;
//     C=a1*a1-(z*z+Kx*Kx+a2*a2+d3*d3);
//     fi=atan2(B,A);
//     u1=pi-asin(C/sqrt(A*A+B*B))-fi;
//     u2=asin(C/sqrt(A*A+B*B))-fi;
////   theta1=atan2(z-a2*sin(u1)+d3*cos(u1),Kx-d3*sin(u1)-a2*cos(u1));     //这俩个公式得出来的是肘关节在上的，或者肘关节在左的
////   theta2=u1-theta1;

//     theta1=atan2(z-a2*sin(u2)+d3*cos(u2),Kx-d3*sin(u2)-a2*cos(u2));     //这俩个公式是肘关节在下的,或者得出的是肘关节在右的  
//     theta2=u2-theta1;		 	 
//		 theta1=-normalize_angle(theta1)+0.2793f;   
//		 theta2=normalize_angle(theta2)+1.85f;      
//		 theta1 = constrain(theta1,-2.5133,0);
//     theta2 = constrain(theta2,-2,0);			 
//		Motor[Pitch1].Brushless.angle_ref=theta1;	 
//		Motor[Pitch2].Brushless.angle_ref=theta2*6.33f;	 		 
//}



float normalize_angle(float theta) {
    // 步骤1：取 2π 的模，将角度归一化到 [0, 2π)
    float two_pi = 2 * pi;
    theta = fmod(theta, two_pi);  // fmod 处理正负浮点数的取模
    
    // 步骤2：将 [π, 2π) 转换为 [-π, 0)
    if (theta > pi) {
        theta -= two_pi;
    }
    
    return theta;
}
