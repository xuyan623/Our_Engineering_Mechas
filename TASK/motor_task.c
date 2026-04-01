#include "motor_task.h"
#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "bsp_can.h"
#include "clamp.h"
#include "comm_8010_task.h"
#include "comm_task.h"
#include "modeswitch_task.h"
#include "motor_8010.h"
#include "stm32f4xx.h"
#include "task.h"
#include "General_function.h"

UBaseType_t Motor_stack_surplus;
extern TaskHandle_t can_msg_send_Task_Handle;
extern TaskHandle_t comm_8010_task_Handle;

extern float GO8010_init_angle1;  

float pitch3_cur=0;

motor_t Motor[Motor_count] = {0};
Machine_Angle_T Machine_angle[Machine_NUMBER] = {0};
PoSition Robotic_arm_Position[_MODE_MAX] = {0}; 

uint8_t Motor_init_state = INIT_NEVER;
uint8_t All_Offset_Angle_init_state = 0;

extern float big_yaw_grav_torque;
extern float pitch1_grav_torque;
extern float pitch2_grav_torque;
extern float pitch3_grav_torque;
extern float roll1_grav_torque;
extern float roll2_grav_torque;
extern float roll3_grav_torque;

float temperory_angle[6];


motor_angle_debug motor_debug[3] =
{
        {0, 0, 0, 0},
        {0, 0, 0, 0},
        {0, 0, 0, 0}
}; 
		
void Motor_Init(void)
{
    Motor10010B_Enable();
    Motor1010B_FeedBackmode(0x01,1, AbsolutePosition, Speed_Data, QValue, FaultInfo); 
    Motor1010B_FeedBackmode(0x02,1, AbsolutePosition, Speed_Data, QValue, FaultInfo);
}

// 对每个电机进行详细的参数配置
void Motor_base_init()
{
    motor_t Motor_Struct;
    // 复制粘贴后只用改一个名字就能配置相应电机
    { // CHASSIS INIT
        Motor_Struct.MOTOR_NAME = CHASSIS_FR;
        Motor_Struct.MOTOR_TYPE = M3508;
        Motor_Struct.Brushless.CAN_ID = 1;
        Motor_Struct.Brushless.ESC_ID = 1;
        Motor_Struct.Brushless.speed = 0;
        Motor_Struct.Brushless.state = 0;
        Motor_Struct.Brushless.offset_angle_init_flag = 0;
        Motor_Struct.Brushless.offset_angle_init_speed = 0;

        memcpy(&Motor[Motor_Struct.MOTOR_NAME], &Motor_Struct, sizeof(Motor_Struct));
        memset(&Motor_Struct, 0, sizeof(Motor_Struct));
    }

    {
        Motor_Struct.MOTOR_NAME = Joint_leg_L;   //0x02    
        Motor_Struct.MOTOR_TYPE = BM_1010B;
        Motor_Struct.Brushless.CAN_ID = 1;
        Motor_Struct.Brushless.ESC_ID = 6;                  // 这里的ESC_ID是发送的数组id，是接收的数组（id-1）
        Motor_Struct.Brushless.Speed_or_Angle_flag = 0;     // 用角度环还是速度环
        Motor_Struct.Brushless.Angle_to_Speed_mode = 0;     // 开切换
        Motor_Struct.Brushless.speed = 0;                   // 大角度差下的速度
        Motor_Struct.Brushless.offset_angle_init_flag = 0;  // 堵转初始化，1为开启堵转
        Motor_Struct.Brushless.offset_angle_init_speed = 0; // 堵转的速度
        Motor_Struct.Brushless.state = 0;

        memcpy(&Motor[Motor_Struct.MOTOR_NAME], &Motor_Struct, sizeof(Motor_Struct));
        memset(&Motor_Struct, 0, sizeof(Motor_Struct));
    }

    {
			  Motor_Struct.MOTOR_NAME = Joint_leg_R;   //0x01     
        Motor_Struct.MOTOR_TYPE = BM_1010B;
        Motor_Struct.Brushless.CAN_ID = 1;
			  Motor_Struct.Brushless.ESC_ID = 5;                  //这个值是多少无所谓，只要不要干扰到其他电机就行
        Motor_Struct.Brushless.Speed_or_Angle_flag = 0;
        Motor_Struct.Brushless.Angle_to_Speed_mode = 0;
        Motor_Struct.Brushless.speed = 0;
        Motor_Struct.Brushless.state = 0;
        Motor_Struct.Brushless.offset_angle_init_flag = 0;
        Motor_Struct.Brushless.offset_angle_init_speed = 0;

        memcpy(&Motor[Motor_Struct.MOTOR_NAME], &Motor_Struct, sizeof(Motor_Struct));
        memset(&Motor_Struct, 0, sizeof(Motor_Struct));
    }

    {
        Motor_Struct.MOTOR_NAME = Big_Yaw;
        Motor_Struct.MOTOR_TYPE = DM_4340;
        Motor_Struct.Brushless.CAN_ID = 2;
        Motor_Struct.Brushless.MASTER_ID = 1;
        Motor_Struct.Brushless.Speed_or_Angle_flag = 0;
        Motor_Struct.Brushless.Angle_to_Speed_mode = 0;
        Motor_Struct.Brushless.speed = 0;
        Motor_Struct.Brushless.state = 0;
        Motor_Struct.Brushless.offset_angle_init_flag = 0;
        Motor_Struct.Brushless.offset_angle_init_speed = 0;
        Motor_Struct.Brushless.control_mode = MOTOR_MODE_MIT;
        memcpy(&Motor[Motor_Struct.MOTOR_NAME], &Motor_Struct, sizeof(Motor_Struct));
        memset(&Motor_Struct, 0, sizeof(Motor_Struct));
    }

    {
        Motor_Struct.MOTOR_NAME = Pitch1;
        Motor_Struct.MOTOR_TYPE = DM_10010L;
        Motor_Struct.Brushless.CAN_ID = 2;
        Motor_Struct.Brushless.MASTER_ID = 2;    //0x01
        Motor_Struct.Brushless.Speed_or_Angle_flag = 0;
        Motor_Struct.Brushless.Angle_to_Speed_mode = 0;
        Motor_Struct.Brushless.spd_ref = 0.25;
        Motor_Struct.Brushless.state = 0;
        Motor_Struct.Brushless.offset_angle_init_flag = 0;
        Motor_Struct.Brushless.offset_angle_init_speed = 0;
        Motor_Struct.Brushless.control_mode = MOTOR_MODE_MIT;

        memcpy(&Motor[Motor_Struct.MOTOR_NAME], &Motor_Struct, sizeof(Motor_Struct));
        memset(&Motor_Struct, 0, sizeof(Motor_Struct));
    }

    {
        Motor_Struct.MOTOR_NAME = Roll1;
        Motor_Struct.MOTOR_TYPE = DM_4340;
        Motor_Struct.Brushless.CAN_ID = 2;
        Motor_Struct.Brushless.MASTER_ID = 3;
        Motor_Struct.Brushless.Speed_or_Angle_flag = 0;
        Motor_Struct.Brushless.speed = 0;
        Motor_Struct.Brushless.state = 0;
        Motor_Struct.Brushless.offset_angle_init_flag = 0;
        Motor_Struct.Brushless.offset_angle_init_speed = 0;
        Motor_Struct.Brushless.control_mode = MOTOR_MODE_MIT;

        memcpy(&Motor[Motor_Struct.MOTOR_NAME], &Motor_Struct, sizeof(Motor_Struct));
        memset(&Motor_Struct, 0, sizeof(Motor_Struct));
    }
    {
        Motor_Struct.MOTOR_NAME = Pitch2;
        Motor_Struct.MOTOR_TYPE = go_8010;
        Motor_Struct.Brushless.GO_ID = GO_8010_1;
        Motor_Struct.Brushless.speed = 0;
        Motor_Struct.Brushless.state = 0;
        Motor_Struct.Brushless.offset_angle_init_flag = 0;
        Motor_Struct.Brushless.offset_angle_init_speed = 0;
        Motor_Struct.Brushless.control_mode = MOTOR_MODE_MIT;

        memcpy(&Motor[Motor_Struct.MOTOR_NAME], &Motor_Struct, sizeof(Motor_Struct));
        memset(&Motor_Struct, 0, sizeof(Motor_Struct));
    }

    {
        Motor_Struct.MOTOR_NAME = Roll2;
        Motor_Struct.MOTOR_TYPE = DM_4310;
        Motor_Struct.Brushless.CAN_ID = 1;
        Motor_Struct.Brushless.MASTER_ID = 4;             //0x04
        Motor_Struct.Brushless.Speed_or_Angle_flag = 0;
        Motor_Struct.Brushless.speed = 0;
        Motor_Struct.Brushless.state = 0;
        Motor_Struct.Brushless.offset_angle_init_flag = 0;
        Motor_Struct.Brushless.offset_angle_init_speed = 0;
        Motor_Struct.Brushless.control_mode = MOTOR_MODE_MIT;

        memcpy(&Motor[Motor_Struct.MOTOR_NAME], &Motor_Struct, sizeof(Motor_Struct));
        memset(&Motor_Struct, 0, sizeof(Motor_Struct));
    }

    { // exchang_pitch_and_roll_l INIT
        Motor_Struct.MOTOR_NAME = Pitch3;
        Motor_Struct.MOTOR_TYPE = DM_4310;
        Motor_Struct.Brushless.CAN_ID = 2;
        Motor_Struct.Brushless.MASTER_ID = 6;
        Motor_Struct.Brushless.Speed_or_Angle_flag = 0;
        Motor_Struct.Brushless.speed = 0;
        Motor_Struct.Brushless.state = 0;	
        Motor_Struct.Brushless.offset_angle_init_flag = 0;
        Motor_Struct.Brushless.offset_angle_init_speed = 800;
        Motor_Struct.Brushless.control_mode = MOTOR_MODE_MIT;

        memcpy(&Motor[Motor_Struct.MOTOR_NAME], &Motor_Struct, sizeof(Motor_Struct));
        memset(&Motor_Struct, 0, sizeof(Motor_Struct));
    }

    {
        Motor_Struct.MOTOR_NAME = Roll3;
        Motor_Struct.MOTOR_TYPE = M6020;
        Motor_Struct.Brushless.CAN_ID = 1;
			  Motor_Struct.Brushless.ESC_ID = 7;       
        Motor_Struct.Brushless.Speed_or_Angle_flag = 0;
        Motor_Struct.Brushless.speed = 0;
        Motor_Struct.Brushless.state = 0;
        Motor_Struct.Brushless.offset_angle_init_flag = 0;
        Motor_Struct.Brushless.offset_angle_init_speed = 0;
        Motor_Struct.Brushless.control_mode = MOTOR_MODE_PID;

        memcpy(&Motor[Motor_Struct.MOTOR_NAME], &Motor_Struct, sizeof(Motor_Struct));
        memset(&Motor_Struct, 0, sizeof(Motor_Struct));
    }

    {
        Motor_Struct.MOTOR_NAME = Grip;
        Motor_Struct.MOTOR_TYPE = DM_4310;
        Motor_Struct.Brushless.CAN_ID = 2;
        Motor_Struct.Brushless.MASTER_ID = 5;
        Motor_Struct.Brushless.Speed_or_Angle_flag = 0;
        Motor_Struct.Brushless.speed = 0;
        Motor_Struct.Brushless.state = 0;
        Motor_Struct.Brushless.offset_angle_init_flag = 0;
        Motor_Struct.Brushless.offset_angle_init_speed = 0;
        Motor_Struct.Brushless.control_mode = MOTOR_MODE_MIT;

        memcpy(&Motor[Motor_Struct.MOTOR_NAME], &Motor_Struct, sizeof(Motor_Struct));
        memset(&Motor_Struct, 0, sizeof(Motor_Struct));
    }

    Motor_pid_init(DONE);
    Motor_base_init_copy(CHASSIS_FR, CHASSIS_BR); // 通过复制实现对四个轮子的同时赋值
    Machine_angle_init();
    Motor_angle_init_test();
}

// pid赋值
void Motor_pid_init(INIT_STATUS init_status)
{
    pid_motor_parameter_t PID_Motor_parameter_Struct;
    mit_motor_parameter_t MIT_Motor_parameter_Struct;

    {
        MIT_Motor_parameter_Struct.MOTOR_NAME = Big_Yaw;
        MIT_Motor_parameter_Struct.Kp = 30;         //22.228
        MIT_Motor_parameter_Struct.Kd = 0.01;
        Motor_MIT_Struct_Init(&Motor[Big_Yaw], MIT_Motor_parameter_Struct);
    }

    {
        MIT_Motor_parameter_Struct.MOTOR_NAME = Pitch1;
        MIT_Motor_parameter_Struct.Kp = 20;      //50
        MIT_Motor_parameter_Struct.Kd = 0.01;   //0.05
        Motor_MIT_Struct_Init(&Motor[Pitch1], MIT_Motor_parameter_Struct);
    }

    {
        MIT_Motor_parameter_Struct.MOTOR_NAME = Roll1;
        MIT_Motor_parameter_Struct.Kp = 0;
        MIT_Motor_parameter_Struct.Kd = 0;
        Motor_MIT_Struct_Init(&Motor[Roll1], MIT_Motor_parameter_Struct);
    }

    {
        MIT_Motor_parameter_Struct.MOTOR_NAME = Pitch2;
        MIT_Motor_parameter_Struct.Kp = 0;
        MIT_Motor_parameter_Struct.Kd = 0;
        Motor_MIT_Struct_Init(&Motor[Pitch2], MIT_Motor_parameter_Struct);
    }

    {
        MIT_Motor_parameter_Struct.MOTOR_NAME = Roll2;
        MIT_Motor_parameter_Struct.Kp = 7;           //7
        MIT_Motor_parameter_Struct.Kd = 0.01;       //0.01
        Motor_MIT_Struct_Init(&Motor[Roll2], MIT_Motor_parameter_Struct);
    }

    {
        MIT_Motor_parameter_Struct.MOTOR_NAME = Pitch3;
        MIT_Motor_parameter_Struct.Kp = 10;        //20
        MIT_Motor_parameter_Struct.Kd = 0.02;        //0.01
        Motor_MIT_Struct_Init(&Motor[Pitch3], MIT_Motor_parameter_Struct);
    }

    {
        PID_Motor_parameter_Struct.MOTOR_NAME = Roll3;
        PID_Motor_parameter_Struct.angle.p = 5;        //4
        PID_Motor_parameter_Struct.angle.i = 0;
        PID_Motor_parameter_Struct.angle.d = 0;
			  PID_Motor_parameter_Struct.angle.max_out = 100;        // 指令不超过1s 1.6圈
        PID_Motor_parameter_Struct.angle.integral_limit = 10;

        PID_Motor_parameter_Struct.speed.p = 40;    //30
        PID_Motor_parameter_Struct.speed.i = 0;
        PID_Motor_parameter_Struct.speed.d = 0;
        PID_Motor_parameter_Struct.speed.max_out = 15000; // 16384
        PID_Motor_parameter_Struct.speed.integral_limit = 500;

        Motor_PID_Struct_Init(&Motor[Roll3], PID_Motor_parameter_Struct, init_status);
    }

    {
        MIT_Motor_parameter_Struct.MOTOR_NAME = Grip;
        MIT_Motor_parameter_Struct.Kp = 18 ;
        MIT_Motor_parameter_Struct.Kd = 0.1;
        Motor_MIT_Struct_Init(&Motor[Grip], MIT_Motor_parameter_Struct);
    }
}
// 对每个动作下电机角度的依次赋值
void Machine_angle_init()
{
    {
        Machine_angle[Big_Yaw_machine].Machine_angle_l.normal_angle = 0;
        Machine_angle[Pitch1_machine].Machine_angle_l.normal_angle = 0.0;
        Machine_angle[Roll1_machine].Machine_angle_l.normal_angle = 0;
			  Machine_angle[Pitch2_machine].Machine_angle_l.normal_angle = 0;   //弧度制 注意下面的机构角度和电机角度的比例，除了减速比还加了个负号，所以这里不用负号,0.85
        Machine_angle[Roll2_machine].Machine_angle_l.normal_angle = 0;    // 记得这是弧度值
        Machine_angle[Pitch3_machine].Machine_angle_l.normal_angle = 0.15;  
        Machine_angle[Roll3_machine].Machine_angle_l.normal_angle = 91+180;
        Machine_angle[Grip_machine].Machine_angle_l.normal_angle = 1.8;

        Robotic_arm_Position[normal_mode].x = -57.48;
        Robotic_arm_Position[normal_mode].y = -78;
        Robotic_arm_Position[normal_mode].z = 164.6;
    }
		
		{
		    Machine_angle[Big_Yaw_machine].Machine_angle_l.get_energy_angle = 0;
		    Machine_angle[Pitch1_machine].Machine_angle_l.get_energy_angle = 1.24218;
		    Machine_angle[Roll1_machine].Machine_angle_l.get_energy_angle = 0;
		    Machine_angle[Pitch2_machine].Machine_angle_l.get_energy_angle = 1.19447;   //
		    Machine_angle[Roll2_machine].Machine_angle_l.get_energy_angle = 0;
		    Machine_angle[Pitch3_machine].Machine_angle_l.get_energy_angle = 0;
		    Machine_angle[Roll3_machine].Machine_angle_l.get_energy_angle = 0;
		    Machine_angle[Grip_machine].Machine_angle_l.get_energy_angle = -1.8;
		}
		
		{
		    Machine_angle[Big_Yaw_machine].Machine_angle_l.get_energy_angle1 =-0.00667;
		    Machine_angle[Pitch1_machine].Machine_angle_l.get_energy_angle1 = 1.035;
		    Machine_angle[Roll1_machine].Machine_angle_l.get_energy_angle1 = 0;
		    Machine_angle[Pitch2_machine].Machine_angle_l.get_energy_angle1 = 5.53/6.33+0.34+0.1;   
		    Machine_angle[Roll2_machine].Machine_angle_l.get_energy_angle1 = 0.6178;
		    Machine_angle[Pitch3_machine].Machine_angle_l.get_energy_angle1 = -0.194;
		    Machine_angle[Roll3_machine].Machine_angle_l.get_energy_angle1 = -90.39;
		    Machine_angle[Grip_machine].Machine_angle_l.get_energy_angle1 = -1.8;
		}
		
		{
		    Machine_angle[Big_Yaw_machine].Machine_angle_l.get_energy_angle2 = 0.148584366;
		    Machine_angle[Pitch1_machine].Machine_angle_l.get_energy_angle2 = 0.99088;
		    Machine_angle[Roll1_machine].Machine_angle_l.get_energy_angle2 = 0;
		    Machine_angle[Pitch2_machine].Machine_angle_l.get_energy_angle2 = 1.04010+0.1;   
		    Machine_angle[Roll2_machine].Machine_angle_l.get_energy_angle2 = 0.093270302;
		    Machine_angle[Pitch3_machine].Machine_angle_l.get_energy_angle2 = 0.07834;
		    Machine_angle[Roll3_machine].Machine_angle_l.get_energy_angle2 = -54.396;
		    Machine_angle[Grip_machine].Machine_angle_l.get_energy_angle2 = -1.8;
		}	
		
		{
		    Machine_angle[Big_Yaw_machine].Machine_angle_l.store_energy_angle  = 1.041 ;
		    Machine_angle[Pitch1_machine].Machine_angle_l.store_energy_angle  =  1.1042;
		    Machine_angle[Roll1_machine].Machine_angle_l.store_energy_angle  = 0;
		    Machine_angle[Pitch2_machine].Machine_angle_l.store_energy_angle  = 1.18567;   //记得除6.33
		    Machine_angle[Roll2_machine].Machine_angle_l.store_energy_angle  = 0.016975;
		    Machine_angle[Pitch3_machine].Machine_angle_l.store_energy_angle = -1.55555;
		    Machine_angle[Roll3_machine].Machine_angle_l.store_energy_angle  = -50.519;
		    Machine_angle[Grip_machine].Machine_angle_l.store_energy_angle  = 0;
		}
		
		{
		    Machine_angle[Big_Yaw_machine].Machine_angle_l.store_energy_angle1  = -1.9018459;   
		    Machine_angle[Pitch1_machine].Machine_angle_l.store_energy_angle1  =  1.00080109;
		    Machine_angle[Roll1_machine].Machine_angle_l.store_energy_angle1  = 0;
		    Machine_angle[Pitch2_machine].Machine_angle_l.store_energy_angle1  = 1.008974;   //记得除6.33
		    Machine_angle[Roll2_machine].Machine_angle_l.store_energy_angle1  = 0.09136295;
		    Machine_angle[Pitch3_machine].Machine_angle_l.store_energy_angle1 = -1.222925131;
		    Machine_angle[Roll3_machine].Machine_angle_l.store_energy_angle1  = 20.4541016;
		    Machine_angle[Grip_machine].Machine_angle_l.store_energy_angle1  = 0;
		}	
		
	
    {
		    Machine_angle[Big_Yaw_machine].Machine_angle_l.exchange_pick_angle  = 1.041;
		    Machine_angle[Pitch1_machine].Machine_angle_l.exchange_pick_angle  =  1.2042;
		    Machine_angle[Roll1_machine].Machine_angle_l.exchange_pick_angle  = 0;
		    Machine_angle[Pitch2_machine].Machine_angle_l.exchange_pick_angle  = 1.08567;   
		    Machine_angle[Roll2_machine].Machine_angle_l.exchange_pick_angle  = 0.016975;
		    Machine_angle[Pitch3_machine].Machine_angle_l.exchange_pick_angle = -1.55555;
		    Machine_angle[Roll3_machine].Machine_angle_l.exchange_pick_angle  = -50.519;
		    Machine_angle[Grip_machine].Machine_angle_l.exchange_pick_angle  = -1.8;
		}
		
		{		
		    Machine_angle[Big_Yaw_machine].Machine_angle_l.exchange_pick_angle1  = -1.90189481;
		    Machine_angle[Pitch1_machine].Machine_angle_l.exchange_pick_angle1  =  1.1;
		    Machine_angle[Roll1_machine].Machine_angle_l.exchange_pick_angle1  = 0;
		    Machine_angle[Pitch2_machine].Machine_angle_l.exchange_pick_angle1  = 1.00948341;    
		    Machine_angle[Roll2_machine].Machine_angle_l.exchange_pick_angle1  = 0.092153325;
		    Machine_angle[Pitch3_machine].Machine_angle_l.exchange_pick_angle1 = -1.363282;
		    Machine_angle[Roll3_machine].Machine_angle_l.exchange_pick_angle1  = 15.583;
		    Machine_angle[Grip_machine].Machine_angle_l.exchange_pick_angle1  = -1.8;
		}
	  {
		    Machine_angle[Big_Yaw_machine].Machine_angle_l.exchange_angle  = 0;
		    Machine_angle[Pitch1_machine].Machine_angle_l.exchange_angle  =  0.64218;
		    Machine_angle[Roll1_machine].Machine_angle_l.exchange_angle  = 0;
		    Machine_angle[Pitch2_machine].Machine_angle_l.exchange_angle  = 1.0447;   
		    Machine_angle[Roll2_machine].Machine_angle_l.exchange_angle  = 0;
		    Machine_angle[Pitch3_machine].Machine_angle_l.exchange_angle = 0;
		    Machine_angle[Roll3_machine].Machine_angle_l.exchange_angle  = 0;
		    Machine_angle[Grip_machine].Machine_angle_l.exchange_angle  =  0;
		}
		
		{
		    Machine_angle[Big_Yaw_machine].Machine_angle_l.primary_angle   = 0.0;
		    Machine_angle[Pitch1_machine].Machine_angle_l.primary_angle   =  1.46691;
		    Machine_angle[Roll1_machine].Machine_angle_l.primary_angle   = 0;
		    Machine_angle[Pitch2_machine].Machine_angle_l.primary_angle   = 2.0053;   
		    Machine_angle[Roll2_machine].Machine_angle_l.primary_angle   = 0.1192;
		    Machine_angle[Pitch3_machine].Machine_angle_l.primary_angle  = -1.6;
		    Machine_angle[Roll3_machine].Machine_angle_l.primary_angle   =180;
		    Machine_angle[Grip_machine].Machine_angle_l.primary_angle   =  0;
		}
		
		{
		    Machine_angle[Big_Yaw_machine].Machine_angle_l.secondray_ore_angle   = 0;
		    Machine_angle[Pitch1_machine].Machine_angle_l.secondray_ore_angle   =  1.48691;
		    Machine_angle[Roll1_machine].Machine_angle_l.secondray_ore_angle   = 0;
		    Machine_angle[Pitch2_machine].Machine_angle_l.secondray_ore_angle   = 0.85;   
		    Machine_angle[Roll2_machine].Machine_angle_l.secondray_ore_angle   =  -1.57;
		    Machine_angle[Pitch3_machine].Machine_angle_l.secondray_ore_angle  = 0;
		    Machine_angle[Roll3_machine].Machine_angle_l.secondray_ore_angle   =  0;
		    Machine_angle[Grip_machine].Machine_angle_l.secondray_ore_angle   =  0;		
		}
		
				Machine_angle[Big_Yaw_machine].Machine_angle_l.chassis_leg_up_angle    = 0;
		    Machine_angle[Pitch1_machine].Machine_angle_l.chassis_leg_up_angle  = 1.22;
        Machine_angle[Roll1_machine].Machine_angle_l.chassis_leg_up_angle  = 0;
			  Machine_angle[Pitch2_machine].Machine_angle_l.chassis_leg_up_angle  = 1.22;   //弧度制 注意下面的机构角度和电机角度的比例，除了减速比还加了个负号，所以这里不用负号,0.85
        Machine_angle[Roll2_machine].Machine_angle_l.chassis_leg_up_angle  = 0;    // 记得这是弧度值
        Machine_angle[Pitch3_machine].Machine_angle_l.chassis_leg_up_angle  = 0.0;  
        Machine_angle[Roll3_machine].Machine_angle_l.chassis_leg_up_angle  =  0;
        Machine_angle[Grip_machine].Machine_angle_l.chassis_leg_up_angle  = 0.0;   
}

// 电机实际角度与转子角度的比例
void Motor_angle_init_test()
{
    config_full_mapping_one(Big_Yaw_machine, Big_Yaw, 1);
    config_full_mapping_one(Pitch1_machine, Pitch1, -1);
    config_full_mapping_one(Roll1_machine, Roll1, 1);
    config_full_mapping_one(Pitch2_machine, Pitch2, -6.33);
    config_full_mapping_one(Roll2_machine, Roll2, 1);
    config_full_mapping_one(Pitch3_machine, Pitch3, 1);
    config_full_mapping_one(Roll3_machine, Roll3, 1);
    config_full_mapping_one(Grip_machine, Grip, 1);
}

void Motor_task(void *parm)
{
    uint32_t Signal;
    BaseType_t STAUS;
    uint8_t ID;

    while (1)
    {

        STAUS = xTaskNotifyWait((uint32_t)NULL, // 接受info_get任务的通知
                                (uint32_t)INFO_GET_MOTOR_SIGNAL,
                                (uint32_t *)&Signal,
                                (TickType_t)portMAX_DELAY);

        if (STAUS == pdTRUE)
        {

            if (Motor_init_state == INIT_NEVER) // 未初始化前，进行电机参数的赋值，还有舵机与气泵初始化（用于调试）
            {
                Motor_base_init();
                Air_Pump_Init();
							  Motor_Init();
                Motor_init_state = INIT_DONE;
            }
            for (ID = 0; ID < Motor_count; ID++)
            {
                if (Whether_Brushless_Motor(Motor[ID]) == 0)
                {
                    // Motor_Servo_handler(ID);
                }
            }
            debug_angle_calucate(&motor_debug[0]); // 这是方便写动作的一个内部的计算器
            debug_angle_calucate(&motor_debug[1]);
            debug_angle_calucate(&motor_debug[2]);

            if ((Signal & INFO_GET_MOTOR_SIGNAL) && chassis_mode != CHASSIS_RELEASE)  //底盘在右拨杆打到上面前，都是release模式，所以要打到最上面才能跑这里，所以当右摇杆从上面打到中间依旧能跑这里，只有打到最下面才结束下面代码
            {
                /******************ʼ*****************************/
                if (All_Offset_Angle_init_state == 0)
                {
                  // All_Offset_Angle_init_state = Motor_offset_angle_init();   //初始化，也就是堵转
                     All_Offset_Angle_init_state = 1;

                    // 通知8010，其实也是跟这一页文件干的事差不多，但8010与can电机的通信方式不同，是串口，就没在这运用
                    xTaskGenericNotify((TaskHandle_t)comm_8010_task_Handle,
                                       (uint32_t)GO_8010_INIT_SIGNAL,
                                       (eNotifyAction)eSetBits,
                                       (uint32_t *)NULL);
                }
                else
                {
                    clamp_angle_handle(); // 具体动作

                    for (ID = MOTOR_MIN_ID; ID < Motor_count; ID++)
                    {
                        if ((Whether_Brushless_Motor(Motor[ID])) && Motor[ID].Brushless.control_mode == MOTOR_MODE_PID)
                        {
                            /*******************************************/
													  Motor_angle_to_speed(ID);   // 角度差过大转速度 以及变更在特定模式下，令Pitch3为速度环，进行力矩采集
                            Motor_pid_clac(ID);         // pid计算
											//		Motor[Pitch3].Brushless.current_send-=Pitch3_Compensation(Motor[Pitch3].Brushless.spd_fdb,Motor[Pitch3].Brushless.controller.pid_ctrl.speed_pid.out);  
                            Motor_current_into_CAN(ID); // 将最后pid的输出值打包成电流	
                        }
                        xTaskGenericNotify((TaskHandle_t)comm_8010_task_Handle,
                                           (uint32_t)INFO_SEND_MOTOR_SIGNAL,
                                           (eNotifyAction)eSetBits,
                                           (uint32_t *)NULL);
                        
												Motor_mit_tff_caculation();											
                    }//遍历电机
                }//电机初始化完成后，正式运行
            }//任务主内容

            xTaskGenericNotify((TaskHandle_t)can_msg_send_Task_Handle,
                               (uint32_t)INFO_SEND_MOTOR_SIGNAL,
                               (eNotifyAction)eSetBits,
                               (uint32_t *)NULL);

            //		Motor_stack_surplus = uxTaskGetStackHighWaterMark(NULL);
        } 
    } 
} 
void gain_angle(void)
{
   temperory_angle[0]=Motor[Big_Yaw].Brushless.angle_fdb - Motor[Big_Yaw].Angle.normal_angle ;
   temperory_angle[1]=Motor[Pitch1].Brushless.angle_fdb - Motor[Pitch1].Angle.normal_angle ;
   temperory_angle[2]=Motor[Pitch2].Brushless.angle_fdb - Motor[Pitch2].Angle.normal_angle - GO8010_init_angle1;
   temperory_angle[3]=Motor[Roll2].Brushless.angle_fdb - Motor[Roll2].Angle.normal_angle;
   temperory_angle[4]=Motor[Pitch3].Brushless.angle_fdb - Motor[Pitch3].Angle.normal_angle;
   temperory_angle[5]=Motor[Roll3].Brushless.angle_fdb - Motor[Roll3].Angle.normal_angle;

}

void Motor_base_init_copy(uint8_t low, uint8_t hight)
{
    uint8_t ID;
    uint8_t count = 0;
    for (ID = low + 1; ID <= hight; ID++)
    {
        memcpy(&Motor[ID], &Motor[low], sizeof(Motor[low]));
        if (Whether_Brushless_Motor(Motor[ID]))
        {
            count++;
            Motor[ID].MOTOR_NAME = (motor_name_status)ID;
            Motor[ID].Brushless.ESC_ID = Motor[low].Brushless.ESC_ID + count;
        }
    }
}
void Motor_base_init_reversal(uint8_t ID)
{
    uint32_t first_mode_angle_adress;
    int16_t reversal_angle;
    uint8_t i;
    first_mode_angle_adress = (uint32_t)&Motor[ID].Angle.normal_angle;
    Motor[ID].Brushless.offset_angle_init_speed = -Motor[ID].Brushless.offset_angle_init_speed;
    for (i = 0; i < MODE_ANGLE_NUMBER; i++)
    {
        reversal_angle = *(int16_t *)first_mode_angle_adress;
        *(int16_t *)first_mode_angle_adress = -reversal_angle;
        first_mode_angle_adress++;
        first_mode_angle_adress++;
    }
}             
void Motor_PID_Struct_Init(motor_t *Motor_recieve, pid_motor_parameter_t parameter_Struct, INIT_STATUS init_status)
{
    PID_Struct_Init(&Motor_recieve->Brushless.controller.pid_ctrl.angle_pid, parameter_Struct.angle.p, parameter_Struct.angle.i, parameter_Struct.angle.d,
                    parameter_Struct.angle.max_out, parameter_Struct.angle.integral_limit, init_status);
    PID_Struct_Init(&Motor_recieve->Brushless.controller.pid_ctrl.speed_pid, parameter_Struct.speed.p, parameter_Struct.speed.i, parameter_Struct.speed.d,
                    parameter_Struct.speed.max_out, parameter_Struct.speed.integral_limit, init_status);
}

void Motor_MIT_Struct_Init(motor_t *Motor, mit_motor_parameter_t parameter_Struct)
{
    Motor->Brushless.controller.mit_params.Kp = parameter_Struct.Kp;
    Motor->Brushless.controller.mit_params.Kd = parameter_Struct.Kd;
}
uint8_t Motor_offset_angle_init(void)
{
    uint8_t ID;
    uint8_t all_init_state;
    all_init_state = 1;

    for (ID = Pitch3; ID < Motor_count; ID++)
    {
        
            if(Motor[ID].Brushless.offset_angle_init_flag == 0)
           {
                if(Motor[ID].MOTOR_TYPE == M3508 || Motor[ID].MOTOR_TYPE == M2006)
                {
                    Motor[ID].Brushless.spd_ref = Motor[ID].Brushless.offset_angle_init_speed;              
                    if ((fabs(Motor[ID].Brushless.spd_ref) - fabs(Motor[ID].Brushless.spd_fdb)) > 0.6 * fabs(Motor[ID].Brushless.spd_ref))
                    {
                        Motor[ID].Brushless.err_count++;
                        if (Motor[ID].Brushless.err_count > 500)
                        {
                            Motor[ID].Brushless.offset_angle_init_flag = 1;
                            Motor[ID].Angle.offset_angle = Motor[ID].Brushless.angle_fdb;
                            Motor[ID].Brushless.spd_ref = 0;
                            Motor[ID].Brushless.err_count = 0;
                        }
                    }
								}
            }
            
            Motor_pid_clac(ID);
            Motor_current_into_CAN(ID);
						
            if (Motor[Pitch3].Brushless.offset_angle_init_flag == 0)  //这里其实的意义是实现多个电机堵转的时候要全部完成堵转才判断堵转完成，因为我们只有一个关节需要堵转，所以我们直接对Ptich3进行设置
                all_init_state = 0; 
        
    }
    return all_init_state;
}
uint32_t TIM_out;
void Motor_Servo_handler(uint8_t ID)
{

    TIM_out = ((2000 / Motor[ID].Servo.Rotation_range) * Motor[ID].Servo.angle_ref) + 500;
    switch (Motor[ID].Servo.Compare)
    {
    case 1: {
        TIM_SetCompare1(Motor[ID].Servo.TIM, TIM_out);
    }
    break;
    case 2: {
        TIM_SetCompare2(Motor[ID].Servo.TIM, TIM_out);
    }
    break;
    case 3: {
        TIM_SetCompare3(Motor[ID].Servo.TIM, TIM_out);
    }
    break;
    case 4: {
        TIM_SetCompare4(Motor[ID].Servo.TIM, TIM_out);
    }
    break;
    }
}

void Motor_angle_to_speed(uint8_t ID)
{
    Motor[ID].Brushless.Speed_or_Angle_flag = ANGLE_MODE;

    if (Motor[ID].Brushless.Angle_to_Speed_mode && All_Offset_Angle_init_state )
    {
        Motor[ID].Brushless.Speed_or_Angle_flag = SPEED_MODE;

        if (Motor[ID].Brushless.angle_ref - Motor[ID].Brushless.angle_fdb > 100)
        {
            Motor[ID].Brushless.spd_ref = Motor[ID].Brushless.speed;
        }
        else if (Motor[ID].Brushless.angle_ref - Motor[ID].Brushless.angle_fdb < -100)
        {
            Motor[ID].Brushless.spd_ref = -Motor[ID].Brushless.speed;
        }
        else
            Motor[ID].Brushless.Speed_or_Angle_flag = ANGLE_MODE;
    }
		
		if(chassis_mode == PITCH3_TORQUE_COLLECTION_MODE)
		{
		    Motor[Pitch3].Brushless.Speed_or_Angle_flag = SPEED_MODE;
		
		}
}

void Motor_pid_clac(uint8_t ID)
{
    if (Motor[ID].Brushless.Speed_or_Angle_flag == ANGLE_MODE)
    {
        pid_calc(&Motor[ID].Brushless.controller.pid_ctrl.angle_pid, Motor[ID].Brushless.angle_fdb, Motor[ID].Brushless.angle_ref);
        Motor[ID].Brushless.spd_ref = Motor[ID].Brushless.controller.pid_ctrl.angle_pid.out;
        pid_calc(&Motor[ID].Brushless.controller.pid_ctrl.speed_pid, Motor[ID].Brushless.spd_fdb, Motor[ID].Brushless.spd_ref);
        Motor[ID].Brushless.current_send = Motor[ID].Brushless.controller.pid_ctrl.speed_pid.out;
    }
    else if (Motor[ID].Brushless.Speed_or_Angle_flag == SPEED_MODE)
    {
        Motor[ID].Brushless.current_send = pid_calc(&Motor[ID].Brushless.controller.pid_ctrl.speed_pid, Motor[ID].Brushless.spd_fdb, Motor[ID].Brushless.spd_ref);
    }
}
void Motor_current_into_CAN(uint8_t ID)
{
    switch (Motor[ID].Brushless.CAN_ID)
    {
    case 1: {
        memcpy(&CAN1_current[Motor[ID].Brushless.ESC_ID], &Motor[ID].Brushless.current_send, sizeof(Motor[ID].Brushless.current_send));
    }
    break;
    case 2: {
        memcpy(&CAN2_current[Motor[ID].Brushless.ESC_ID-1], &Motor[ID].Brushless.current_send, sizeof(Motor[ID].Brushless.current_send));
    }
    break;
    }
}
uint8_t Whether_Brushless_Motor(motor_t Motor)
{
    switch (Motor.MOTOR_TYPE)
    {
    case M6020:
    case M3508:
    case M2006:
    case go_8010:
    case BM_1010B:
    case DM_10010L:
    case DM_4310:
		case DM_4340:
        return 1;
    default:
        return 0;
    }
}


uint8_t Whether_DM_Motor(motor_t Motor)
{
    switch (Motor.MOTOR_TYPE)
    {
    case DM_10010L:
    case DM_4310:
		case DM_4340:
        return 1;
    default:
        return 0;
    }
}

// 获取角度字段指针
float *get_angle_field(float *mode_angle, int16_t count)
{
    return (float *)((uint32_t)mode_angle + (sizeof(Motor_angle_t) / MODE_ANGLE_NUMBER) * count); // 根据count获取对应的模式角度
}

// 双角度映射
void config_full_mapping_couple(int16_t Machine_ID_l, int16_t Machine_ID, int16_t Motor_ID, float Machine_l_ratio, float Machine_ratio, int16_t total_ratio)
{
    float *Machine_l_scr = &Machine_angle[Machine_ID_l].Machine_angle_l.normal_angle;
    float *Machine_scr = &Machine_angle[Machine_ID].Machine_angle_l.normal_angle;
    float *Motor_scr = &Motor[Motor_ID].Angle.normal_angle;
    // 这个减2的原因是让init_angle，offset_angle等电机特有值不被赋值
    for (int i = 0; i < MODE_ANGLE_NUMBER - 2; i++)
    {
        float *Machine_l_dest = get_angle_field(Machine_l_scr, i);
        float *Machine_dest = get_angle_field(Machine_scr, i);
        float *Motor_dest = get_angle_field(Motor_scr, i);

        *Motor_dest = (*Machine_dest) * Machine_ratio + (*Machine_l_dest) * Machine_l_ratio;
    }
}

// 单角度映射
void config_full_mapping_one(int16_t Machine_ID, int16_t Motor_ID, float total_ratio)
{
    float *Machine_scr = &Machine_angle[Machine_ID].Machine_angle_l.normal_angle;
    float *Motor_scr = &Motor[Motor_ID].Angle.normal_angle;
    for (int i = 0; i < MODE_ANGLE_NUMBER - 2; i++)
    {
        float *Machine_dest = get_angle_field(Machine_scr, i);
        float *Motor_dest = get_angle_field(Motor_scr, i);
        *Motor_dest = *Machine_dest * total_ratio;
    }
}

void Air_Pump_Init(void)
{
    PUMP_OFF
}


void debug_angle_calucate(motor_angle_debug *angle_debug)
{
    angle_debug->mode_angle = angle_debug->targrt_angle - angle_debug->offset_angle - angle_debug->normal_angle;
}

void Motor_mit_tff_caculation(void)
{	
	{
     Motor[Big_Yaw].Brushless.controller.mit_params.tff = 0;
	}
	
  {
//     pitch1_grav_torque = pitch1_grav_torque_calcuate();  
//		 pitch1_grav_torque=constrain(pitch1_grav_torque,-20,20);		  
//     Motor[Pitch1].Brushless.controller.mit_params.tff = pitch1_grav_torque;
	}
  
	{
	   Motor[Roll1].Brushless.controller.mit_params.tff = 0;
	}
	
	{
		 pitch3_grav_torque = pitch3_grav_torque_calcuate();
		 Motor[Pitch3].Brushless.controller.mit_params.tff = pitch3_grav_torque;
	}
	
	{
	   roll2_grav_torque = roll2_grav_torque_calculate();
		 Motor[Roll2].Brushless.controller.mit_params.tff = roll2_grav_torque_calculate();
	}
}

float pitch3_grav_torque_calcuate(void)
{
	 float q6_int =0;
	 float toq,m7,m6,rz7,ry6,d7;	
	 float q_4 = 0;
	 float q_6,q_3,q_2;
	 m7=0.631; rz7=0.062165 ; m6=0.37708; ry6=0.05038; d7=0.057;      	
   float q_int2,q_int3,q_int6;     	 
	 q_int2 = 0.20944; q_int3 = 0.079799; q_int6 = 0.15;   //这个q_int6 应该是 0.15的，但是偏移在其他地方计算了，所以这里设0	
	 q_2 = q_int2 + Motor[Pitch1].Brushless.angle_fdb;  //Motor[Pitch1].Brushless.angle_fdb          	
	 q_3 =(-Motor[Pitch2].Brushless.angle_fdb+GO8010_init_angle1)/6.33 - q_int3;   //q3也是	 
	 q_4 =-Motor[Roll2].Brushless.angle_fdb;	 
   q_6 = Motor[Pitch3].Brushless.angle_fdb - q_int6 ;     //这里的q6_int的含义与q3_int，q2_int的含义有区别，这里主要是加了个1.57，所以电机角度取负 
   
	 toq = 9.8*0.1525*(cos(q_6)*cos(q_2+q_3)*cos(q_4)-sin(q_6)*sin(q_2+q_3));       //0.0941904054     m7*d7+m7*rz7+m6*ry6
 //cur = 9.8*(d7*m7+rz7*m7+ry6*m6)*(cos(q_6)*cos(q_2+q_3)         -sin(q_6)*sin(q_2+q_3))/0.16888f*16384.0f/20.0f;  
  
   return toq;    //电流方向的讲究我主要是直接读，然后在看要不要取反的
}


float roll2_grav_torque_calculate(void)
{
     float q_int2,q_int3,q_int6;    
     float m7,m6,rz7,ry6,d7;
     float q_6,q_3,q_2,q_4;
     float toq=0;    
	   float lm_56 = 0;
	   lm_56 = 0.13;
	   m7=0.631; rz7=0.062165; m6=0.377; ry6=0.05038; d7=0.057;
	   q_int2 = 0.366519; q_int3 = -0.353786;	
	
	   q_2 = q_int2 - Motor[Pitch1].Brushless.angle_fdb;  //Motor[Pitch1].Brushless.angle_fdb
		 q_3 =(Motor[Pitch2].Brushless.angle_fdb-GO8010_init_angle1)/6.33 + q_int3;	 
	   q_4 =-Motor[Roll2].Brushless.angle_fdb;
     q_6 = Motor[Pitch3].Brushless.angle_fdb + 0.15;
	
	
	   toq = 9.8*cos(q_2+q_3)*(sin(q_4)*( lm_56*sin(q_6)));
	
     return toq;
}
float lm2 =0;
float pitch1_grav_torque_calcuate(void)    //之前pitch1用的是七转六的公式，现在直接用六转六的试试
{
     float q_int2,q_int3,q_int5;    
     float m7,m6,m5,m4,m3,m2,rx2,ry2,a2,rx3,rx4,ry4,d4;
     float q_2,q_3,q_4,q_5,q_6;   
     float toq=0; 
	   float l_m2,lm_56,lm_34;    
                        
                                                     	//	   lm_56 = 0.16 ;  //m6*d6+m6*rz6+m5*ry5;  
	                                                     //      lm_34 = 0.30228;                  //m3*ry3+m4*(rz4+d4)  rz4+d4 = roll的质心到pitch2关节的距离
     lm_56 = 0.13;
	   lm_34 = 0.30228;   
	
     m6=0.89679; m5=0.5359; m4=0.6769; m3=0.80; m2=1.5215;     //     m6=0.631; m5=0.37708; m4=0.6769; m3=0.80; m2=1.5215;     
     rx2=0.21929; ry2=-0.000476;rx3=0.004006;rx4=0.002649;ry4=-0.003381;
     a2=0.275; d4 = 0.3;
	   q_int2 = 0.366519;     //21
	   q_int3 = -0.353786;       //5
	   q_int5 = 0.15;
	   l_m2 = 1.1028;       //0.92854807    (a2*(m3+m4+m6+m7)+m2*rx2)
	
	   q_2 = q_int2 - Motor[Pitch1].Brushless.angle_fdb;
	   q_3 = q_int3 + (Motor[Pitch2].Brushless.angle_fdb-GO8010_init_angle1)/6.33;
		 q_5 = q_int5 + Motor[Pitch3].Brushless.angle_fdb;
		 q_4 =-Motor[Roll2].Brushless.angle_fdb;

//     toq =(l_m2*cos(q_2)-m2*ry2*sin(q_2))*9.8 + pitch2_grav_torque*6.33 ; 
		   toq =(l_m2*cos(q_2)-m2*ry2*sin(q_2))*9.8 + ( -lm_34*cos(q_2+q_3) - lm_56*(cos(q_2+q_3)*cos(q_5) + sin(q_2+q_3)*sin(q_5)*cos(q_4)) + m4*ry4*sin(q_4)*sin(q_2+q_3) - (m5+m6)*d4*cos(q_2+q_3) )*9.8;
    


  //lm2 = ( -lm_34*cos(q_2+q_3)- lm_56*(cos(q_2+q_3)*cos(q_5) + sin(q_2+q_3)*sin(q_5)*cos(q_4)) );//- m3*rx3*sin(q_2+q_3)//+ m4*(-rx4*cos(q_4)+ry4*sin(q_4))*sin(q_2+q_3) - m5*d4*cos(q_2+q_3) - m6*d4*cos(q_2+q_3) )*9.8;
	
	   return -toq;
}

float Pitch3_Compensation(float w_ref,float w)
{

  float uq_positive = 200;
  float uq_negative = -200; 
  float uq_comp = 0;
  float width = 1.0f;
  float b = (uq_positive + uq_negative)/2;
  float k = (uq_positive - b)/width;
  if(w>=0)
  {
     if(w<width)
        uq_comp = k*w +b;
		 else
        uq_comp = uq_positive;
	}
  else
  {
     if(w>width)
        uq_comp = k*w + b;
     else
        uq_comp = -uq_negative;
	}
	if(w_ref >=0)
	   return uq_comp;
	else if(w_ref<0)
	   return -uq_comp;
	else
     return 0;

}








