#ifndef _pc_tx_data_H
#define _pc_tx_data_H



#include "stm32f4xx.h"
#include "data_packet.h"

#define PC_TX_FIFO_BUFLEN 500

typedef enum
{
	TRACK_AMOR_MODE			=0,
	BIG_BUFF_MODE				=1,
	NORMAL_CTRL_MODE		 =2,
	SMALL_BUFF_MODE			=3,
}mode_t;

typedef enum
{
	red  = 1,
	blue = 2,
}enemy_color_t;

/** 
  * @brief  gimbal information
  */
typedef __packed struct
{
  uint8_t ctrl_mode;          /* gimbal control mode */
  float   pit_relative_angle; /* pitch angle(degree) relative to the gimbal center */
  float   yaw_relative_angle; /* yaw angle(degree) relative to the gimbal center */
  float   pit_absolute_angle; /* pitch angle(degree) relative to ground */
  float   yaw_absolute_angle; /* yaw angle(degree) relative to ground */
  float   pit_palstance;      /* pitch axis palstance(degree/s) */
  float   yaw_palstance;      /* yaw axis palstance(degree/s) */
} gimbal_info_t;

/** 
  * @brief  shoot information
  */
typedef __packed struct
{
  int16_t remain_bullets;  /* the member of remain bullets */
  int16_t shoot_bullets;    /* the member of bullets that have been shot */
  uint8_t fric_wheel_run;  /* friction run or not */
} shoot_info_t;

/** 
  * @brief  gimbal calibrate command
  */
typedef __packed struct
{
	mode_t	main_mode;					//模式：0装甲板 1神符 2空
	enemy_color_t	enemy_color;	//敌方颜色:1red 2blue
	float	bullet_spd;					  //子弹速度
	float pit;
	float	yaw;
	
}pc_info_t;

typedef enum
{
	display_clamp_vision = 0,    //显示夹取的摄像头视野
	display_rescue_vision ,      //显示救援模式的视野
	display_blank_screen,        //模拟黑屏
	
}monitor_display_mode_t;//显示器切屏控制

typedef __packed struct
{
	monitor_display_mode_t	display;
	
}engineer_info_t;
typedef struct
{
  /* data send */
//  gimbal_info_t     gimbal_information;
//  shoot_info_t      shoot_task_information;
//	pc_info_t					pc_need_information;
	
	engineer_info_t    pc_need_information;
	
} send_pc_t;

extern fifo_s_t  pc_txdata_fifo;

extern send_pc_t    pc_send_mesg;

void pc_tx_param_init(void);
void pc_send_data_packet_pack(void);
void get_upload_data(void);
void get_infantry_info(void);

#endif
