#ifndef _judge_rx_data_H
#define _judge_rx_data_H


#include "stm32f4xx.h"
#include "data_packet.h"

#define JUDGE_RX_FIFO_BUFLEN 256

/**代码相对于21赛季裁判系统通信缺少模块如下***

*0x0004 -- 飞镖发射状态，飞镖发射时发送     已添加
*0x0005 -- 人工智能挑战赛加成与惩罚区状态   已添加

*0x0103 -- 请求补给站补弹数据（由参赛队发送 上限10HZ RM对抗赛尚未开发）未添加
*0x0104 -- 裁判警告数据，警告发生后发送     已添加
*0x0105 -- 飞镖发射口倒计时                已添加

*0x0208 -- 弹丸剩余发射数，仅空中机器人、哨兵机器人以及ICRA机器人发送 已添加
*0x0209 -- 机器人RFID状态                  已添加
*0x020A -- 飞镖机器人客户端指令数据         已添加
**/
typedef enum
{
	GAME_STATE_ID                      = 0x0001,  //比赛状态数据
	GAME_RESULT_ID 	                   = 0x0002,  //比赛结果数据
	GAME_ROBOT_HP_ID                   = 0x0003,	//比赛机器人血量数据            
	DART_STATE_ID                      = 0x0004,  //飞镖发射状态                 
	ICRA_BUFF_DEBUFF_ZONE_STATE_ID     = 0x0005,  //人工智能挑战赛加成与惩罚区状态
	EVENT_DATA_ID 				             = 0x0101,	//场地事件数据
	SUPPLY_PROJECTILE_ACTION_ID        = 0x0102,	//场地补给站动作标识数据，动作发生后发送
  SUPPLY_PROJECTILE_BOOK_ID          = 0x0103,	// 请求补给站补弹数据 （还未开发！！）          
	REFEREE_WARNING_ID                 = 0x0104,  // 裁判系统警告信息
	DART_REMAINING_TINME_ID            = 0x0105,  //飞镖发射口倒计时
	GAME_ROBOT_STATE_ID                = 0x0201,	//机器人状态数据                  
	POWER_HEAT_DATA_ID                 = 0x0202,	//实时功率热量数据
	GAME_ROBOT_POS_ID                  = 0x0203,	//机器人位置数据
	BUFF_ID                            = 0x0204,	//机器人增益数据
	AERIAL_ROBOT_ENERGY_ID             = 0x0205,	//空中机器人能量状态数据
	ROBOT_HURT_ID                      = 0x0206,	//伤害状态数据，伤害发生后发送
	SHOOT_DATA_ID                      = 0x0207,	//实时射击数据，子弹发射后发送
	BULLET_REMAINING_ID                = 0x0208,  //子弹剩余数量：空中机器人，哨兵机器人以及ICRA机器人
  RFID_STATE_ID                      = 0x0209,  //机器人RFID状态
	DAT_CLIENT_CMD_ID                  = 0x020A,  //飞镖机器人客户端指令数据
	STUDENT_INTERACTIVE_HEADER_DATA_ID = 0x0301,	//机器人间交互数据 和 客户端通信（发送方触发发送）

  CUSTOM_CONTROLLER_INTERACTION_DATA_ID = 0x0302,//自定义控制器交互数据接口，通过客户端触发发送，上限 30HZ	
	CLIENT_MINIMAP_INTERACTIVE_DATDA_ID   = 0x0303,//客户端小地图交互数据
	KEYBOARD_AND_MOUSE_INFORMATION_ID     = 0x0304,//键盘、鼠标信息，通过图传串口发送
} judge_data_id_e;

/*裁判系统数据结构体*/
/*0001*/
typedef __packed struct
{
 uint8_t game_type : 4;
 uint8_t game_progress : 4;
 uint16_t stage_remain_time;
 uint64_t SyncTimeStamp;
} ext_game_status_t;
/*0002*/
typedef __packed struct
{
 uint8_t winner;
} ext_game_result_t;
/*0003*/
typedef __packed struct
{
 uint16_t red_1_robot_HP;
 uint16_t red_2_robot_HP; 
 uint16_t red_3_robot_HP; 
 uint16_t red_4_robot_HP; 
 uint16_t red_5_robot_HP; 
 uint16_t red_7_robot_HP; 
 uint16_t red_outpost_HP;
 uint16_t red_base_HP; 
 uint16_t blue_1_robot_HP; 
 uint16_t blue_2_robot_HP; 
 uint16_t blue_3_robot_HP; 
 uint16_t blue_4_robot_HP; 
 uint16_t blue_5_robot_HP; 
 uint16_t blue_7_robot_HP; 
 uint16_t blue_outpost_HP;
 uint16_t blue_base_HP;
} ext_game_robot_HP_t;
/*0004*/
typedef __packed struct
{
 uint8_t dart_belong; 
 uint16_t stage_remaining_time; 
} ext_dart_status_t;

/*0005*/
typedef __packed struct
{
 uint8_t F1_zone_status:1;
 uint8_t F1_zone_buff_debuff_status:3; 
 uint8_t F2_zone_status:1;
 uint8_t F2_zone_buff_debuff_status:3; 
 uint8_t F3_zone_status:1;
 uint8_t F3_zone_buff_debuff_status:3; 
 uint8_t F4_zone_status:1;
 uint8_t F4_zone_buff_debuff_status:3; 
 uint8_t F5_zone_status:1;
 uint8_t F5_zone_buff_debuff_status:3; 
 uint8_t F6_zone_status:1;
 uint8_t F6_zone_buff_debuff_status:3;
 uint16_t red1_bullet_left;
 uint16_t red2_bullet_left;
uint16_t blue1_bullet_left;
uint16_t blue2_bullet_left;
} ext_ICRA_buff_debuff_zone_status_t;
/*0101*/
typedef __packed struct
{
 uint32_t event_type;
} ext_event_data_t;
/*0102*/
typedef __packed struct
{
 uint8_t supply_projectile_id; 
 uint8_t supply_robot_id; 
 uint8_t supply_projectile_step; 
uint8_t supply_projectile_num;
} ext_supply_projectile_action_t;
/*0103*/


/*0104*/
typedef __packed struct
{
 uint8_t level;
 uint8_t foul_robot_id; 
} ext_referee_warning_t;
/*0105*/
typedef __packed struct
{
 uint8_t dart_remaining_time;
} ext_dart_remaining_time_t;

/*0201*/
typedef __packed struct
{
 uint8_t robot_id;
 uint8_t robot_level;
 uint16_t remain_HP;
 uint16_t max_HP;
 uint16_t shooter_id1_17mm_cooling_rate;
 uint16_t shooter_id1_17mm_cooling_limit;
 uint16_t shooter_id1_17mm_speed_limit;
 uint16_t shooter_id2_17mm_cooling_rate;
 uint16_t shooter_id2_17mm_cooling_limit;
 uint16_t shooter_id2_17mm_speed_limit;
 uint16_t shooter_id1_42mm_cooling_rate;
 uint16_t shooter_id1_42mm_cooling_limit;
 uint16_t shooter_id1_42mm_speed_limit;
 uint16_t chassis_power_limit;
 uint8_t mains_power_gimbal_output : 1;
 uint8_t mains_power_chassis_output : 1;
 uint8_t mains_power_shooter_output : 1;
} ext_game_robot_status_t;
/*0202*/
typedef __packed struct
{
 uint16_t chassis_volt; 
 uint16_t chassis_current; 
 float chassis_power; 
 uint16_t chassis_power_buffer; 
 uint16_t shooter_id1_17mm_cooling_heat;
 uint16_t shooter_id2_17mm_cooling_heat;
 uint16_t shooter_id1_42mm_cooling_heat;
} ext_power_heat_data_t;
/*0203*/
typedef __packed struct
{
 float x;
 float y;
 float z;
 float yaw;
} ext_game_robot_pos_t;
/*0204*/
typedef __packed struct
{
 uint8_t power_rune_buff;
}ext_buff_t;
/*0205*/
typedef __packed struct
{
 uint8_t attack_time;
} aerial_robot_energy_t;
/*0206*/
typedef __packed struct
{
 uint8_t armor_id : 4;
 uint8_t hurt_type : 4;
} ext_robot_hurt_t;
/*0207*/
typedef __packed struct
{
 uint8_t bullet_type;
 uint8_t shooter_id;
 uint8_t bullet_freq;
 float bullet_speed;
} ext_shoot_data_t;
/*0208*/
typedef __packed struct
{
 uint16_t bullet_remaining_num_17mm;
 uint16_t bullet_remaining_num_42mm;
 uint16_t coin_remaining_num;
} ext_bullet_remaining_t;
/*0209*/
typedef __packed struct
{
  uint32_t rfid_status;
} ext_rfid_status_t;
/*020A*/
typedef __packed struct
{
 uint8_t dart_launch_opening_status;
 uint8_t dart_attack_target;
 uint16_t target_change_time;
 uint8_t first_dart_speed;
 uint8_t second_dart_speed;
 uint8_t third_dart_speed;
 uint8_t fourth_dart_speed;
 uint16_t last_dart_launch_time;
 uint16_t operate_launch_cmd_time;
} ext_dart_client_cmd_t;
/*0302*/
typedef __packed struct
{
  uint8_t data[50];
}robot_interactive_data_t;
/*0303*/
typedef __packed struct
{
	float target_position_x;
	float target_position_y;
	float target_position_z;
	uint8_t commd_keyboard;
	uint16_t target_robot_ID;
}ext_robot_command1_t;
/*0304*/
typedef __packed struct
{
	int16_t mouse_x;
	int16_t mouse_y;
	int16_t mouse_z;
	int8_t left_button_down;
	int8_t right_button_down;
	uint16_t keyboard_value;
	uint16_t reserved;
}ext_robot_command2_t;

typedef struct
{
  ext_game_status_t                      	    game_state;               	 	   //0x0001
  ext_game_result_t                      	    game_result;               	     //0x0002
  ext_game_robot_HP_t             			 	    game_robot_HP;     		           //0x0003
	ext_dart_status_t                           dart_state;                      //0x0004
	ext_ICRA_buff_debuff_zone_status_t          ICRA_buff_debuff_zone_state;     //0x0005
  ext_event_data_t  										      event_data;                	     //0x0101
  ext_supply_projectile_action_t     	        supply_projectile_action;  	     //0x0102
 // ext_supply_projectile_booking_t     	      supply_projectile_booking; 	   //0x0103
	ext_referee_warning_t                       referee_warning;              
	ext_dart_remaining_time_t                   dart_remaining_time;
  ext_game_robot_status_t        		          game_robot_state;      		       //0x0201
  ext_power_heat_data_t   				            power_heat_data;     		 	 
  ext_game_robot_pos_t   				              game_robot_pos;				           //0x0203
  ext_buff_t						                      buff;						                 //0x0204
  aerial_robot_energy_t					              aerial_robot_energy;			       //0x0205
  ext_robot_hurt_t						                robot_hurt;						           //0x0206
  ext_shoot_data_t						                shoot_data;			           	     //0x0207
	ext_bullet_remaining_t                      bullet_remaining;
	ext_rfid_status_t                           rfid_state;
	ext_dart_client_cmd_t                       dart_client_cmd;
  //ext_student_interactive_header_data_t   	  student_interactive_header_data; //0x0301
 
  robot_interactive_data_t                    robot_interactive_data;  //0x0302
	ext_robot_command1_t                        minimap_interactive_data;//0x0303
	ext_robot_command2_t                        mouse_keyboard_informationt;
} judge_rxdata_t;

extern judge_rxdata_t judge_recv_mesg;//读取回来保存在该变量
extern uart_dma_rxdata_t judge_rx_obj;
extern unpack_data_t judge_unpack_obj;

extern judge_rxdata_t control_recv_mesg;
extern uart_dma_rxdata_t control_rx_obj;
extern unpack_data_t control_unpack_obj;

void judgement_rx_param_init(void);
void judgement_data_handler(uint8_t *p_frame);
void controller_data_handler(uint8_t *p_frame);
#endif

