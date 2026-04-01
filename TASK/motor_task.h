#ifndef _moto
#define _moto

#include "pid.h"
#include "fuzzy_pid.h"
#include "string.h"

#include "User.h"
/**************电机数目定义************/
#define MOTOR_NUMBER 15
#define MODE_ANGLE_NUMBER 15
#define Machine_NUMBER 13   
#define MOTOR_MAX_ID Grip
#define MOTOR_MIN_ID Big_Yaw  
/*************************************/



#ifdef BACK_DRIVE
#define NORMAL_LOOK 255
#define MODE_LOOK -193
#else
#define NORMAL_LOOK 62
#define MODE_LOOK 0
#endif

#define GO_8010_1 1     //1
#define GO_8010_2 0

/**************电机名定义************/
typedef enum
{
	//以下为MOTOR_NAME 逐级递增方便增改
	CHASSIS_FR = 0,
	CHASSIS_FL,
	CHASSIS_BL,
	CHASSIS_BR,

	Joint_leg_L,           
	Joint_leg_R,   //包括当前和在这之前的电机，都为底盘电机，增减电机要以当前的电机为分界线增加
  Big_Yaw,     /*4340*/
	Pitch1,      /*10010L*/
	Roll1,       /*4340*/
	Pitch2,      /*8010*/
	Roll2,       /*4310*/
  Pitch3,  
 	Roll3,
	Grip,
	Motor_count
}motor_name_status;

typedef enum
{
	M6020 = 0,
	M3508,
	M2006,
	go_8010,
	BM_1010B,
	DM_10010L,
	DM_4310,
	DM_4340,
	SERVO,
}motor_type_status;

typedef enum {
    MOTOR_MODE_PID = 0,
    MOTOR_MODE_MIT,
} MotorControlMode;


typedef struct
{
	uint8_t state;					            //初始化完成 
	uint8_t offset_angle_init_flag;	    //初始化标志
	int16_t offset_angle_init_speed;    //堵转初始化速度
	uint16_t err_count;                 //堵转时间的记录

	uint8_t CAN_ID;                     //can1或2
	uint8_t ESC_ID;                     //电调ID
	uint8_t GO_ID;                      //8010ID
	uint8_t MASTER_ID;                  //达妙ID          

	uint8_t Angle_to_Speed_mode;        //是否在角度差过大时切换到速度环
	uint8_t Speed_or_Angle_flag;        //选择速度或角度环
	int16_t speed;				              //角度差过大直接给多少速度

	int32_t ecd_fdb;                    //电机编码反馈值
	float torque_fbd;                   //力矩反馈
	float torque_ref;                   //力矩参考
	float angle_fdb;                    //角度环get
	float l_angle_fdb;                  //上一刻的角度反馈
	float angle_ref;	                  //角度环目标
	float spd_fdb;		                  //速度环get
	float spd_ref;		                  //速度环目标
	float current_read;                 //电流反馈
	int16_t current_send;               //电流发送

  MotorControlMode control_mode;

    union 
    {
        struct 
        {
            pid_t angle_pid;              // 角度环
            pid_t speed_pid;              // 速度环
        } pid_ctrl;
				
        mit_parameter_t mit_params;       // 结构体小很多，节省内存
    } controller;                         // 联合体变量名

}Brushless_motor_t;

typedef struct
{
	TIM_TypeDef *TIM;
	uint8_t Compare;
	int16_t angle_ref;

	uint16_t Rotation_range;            //旋转范围
} Servo_motor_t;

typedef struct
{
	float normal_angle;
	float exchange_angle;
	float exchange_pick_angle;
	float exchange_pick_angle1;
	float primary_angle;
	float secondray_ore_angle;
	float get_energy_angle;
	float get_energy_angle1;
	float get_energy_angle2;	
	float store_energy_angle;
	float store_energy_angle1;
  float chassis_leg_up_angle;

	float mode_angle;

	float init_angle;	            //初始化得到的基准
	float offset_angle;           //进模式角度相对变化用基准
}Motor_angle_t;

typedef struct
{
	motor_name_status MOTOR_NAME; //等于其宏定义，debug看名字
	motor_type_status MOTOR_TYPE; //电机类型
	Brushless_motor_t Brushless;
	Servo_motor_t Servo;
	Motor_angle_t Angle;

}motor_t;

typedef struct
{
	motor_name_status MOTOR_NAME;
	pid_parameter_t speed;
	pid_parameter_t angle;
}pid_motor_parameter_t;

typedef struct
{
	motor_name_status MOTOR_NAME;
  float Kp;
  float Kd;
	float tff;
}mit_motor_parameter_t;

typedef struct 
{
	float offset_angle;
	float targrt_angle;
	float normal_angle;
	float mode_angle;
}motor_angle_debug;

typedef enum
{
	Big_Yaw_machine = 0,
	Pitch1_machine,
	Roll1_machine,
	Pitch2_machine,
  Roll2_machine,
	Pitch3_machine,
	Roll3_machine,
	Grip_machine
}Machine_Angle_N;

typedef struct
{
	Motor_angle_t Machine_angle_l;
} Machine_Angle_T;



typedef struct
{
	float x;
	float y;
	float z;	
}PoSition;

typedef enum
{  	
	 normal_mode = 0,
	 exchange_mode,
	 store_mode,
	 store_mode2,
	 store_mode3,
	 bigisland_straight_mode,
	 smallisland_mode,
	 ground_mode,
   _MODE_MAX  
	
}Position_Machine;

extern motor_t Motor[Motor_count];

void Motor_task(void *parm);
void Motor_base_init(void);
void Motor_base_init_copy(uint8_t low, uint8_t hight);
void Motor_base_init_reversal(uint8_t ID);
void Motor_pid_init(INIT_STATUS init_status);
void Motor_angle_init(void);
void Motor_PID_Struct_Init(motor_t *Motor_recieve, pid_motor_parameter_t parameter_Struct, INIT_STATUS init_status);
void Motor_MIT_Struct_Init(motor_t *Motor,mit_motor_parameter_t parameter_Struct);

uint8_t Motor_offset_angle_init(void);
void Motor_Servo_handler(uint8_t ID);
void Motor_pid_clac(uint8_t ID);
void Motor_current_into_CAN(uint8_t ID);
uint8_t Whether_Brushless_Motor(motor_t Motor);
uint8_t Whether_DM_Motor(motor_t Motor);
uint8_t motor_8010_speed_get_limit(uint8_t ID);
void config_full_mapping_couple(int16_t Machine_ID_l, int16_t Machine_ID, int16_t Motor_ID, float Machine_l_ratio, float Machine_ratio, int16_t total_ratio);
void config_full_mapping_one(int16_t Machine_ID, int16_t Motor_ID, float total_ratio);
void Motor_angle_init_test(void);
void Machine_angle_init(void);
float *get_angle_field(float *mode_angle, int16_t count);
void debug_angle_calucate(motor_angle_debug* angle_debug);
void Air_Pump_Init(void);
void Motor_Init(void);
void Motor_angle_to_speed(uint8_t ID);
void Motor_mit_tff_caculation(void);
void gain_angle(void);

float Yaw_FFC_OUT(float x_n);
float Yaw_Compensation(float w_ref,float w);
float torque_Trajectory_Planning(float start_torque,float final_torque,float t,uint32_t tq);   //默认起始的速度和最终的速度为0          

float pitch1_grav_torque_calcuate(void);
float pitch3_grav_torque_calcuate(void);
float roll2_grav_torque_calculate(void);
float Pitch3_Compensation(float w_ref,float w);


#endif
