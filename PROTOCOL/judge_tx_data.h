#ifndef _judge_tx_data_H
#define _judge_tx_data_H


#include "stm32f4xx.h"
#include "data_packet.h"

#define JUDGE_TX_FIFO_BUFLEN 500                                       //用于给发送邮箱的初始化
#define STUDENT_DATA_LENGTH   sizeof(ext_student_interactive_data_t_t) //交互数据的长度 

//机器人间交互数据包
typedef __packed struct
{
	uint16_t data_cmd_id;//内容ID、发送者ID、接受者ID和交互数据
	uint16_t sender_ID;
	uint16_t receiver_ID;
	
	uint8_t data[113];//最大113
	
}ext_student_interactive_data_t_t;

//学生交互id
typedef enum
{
	//学生交互机器人id
	STUDENT_RED_HERO_ID       			= 1,
	STUDENT_RED_ENGINEER_ID	  			= 2,
	STUDENT_RED_INFANTRY3_ID  			= 3,
	STUDENT_RED_INFANTRY4_ID  			= 4,
	STUDENT_RED_INFANTRY5_ID  			= 5,
	STUDENT_RED_AERIAL_ID	    			= 6,
	STUDENT_RED_SENTRY_ID	    			= 7,
	STUDENT_RED_RadarStation_ID	    = 9,
	STUDENT_RED_Outpost_ID	    		= 10,//前哨站(前哨和基地的ID用于小地图交互数据)
	STUDENT_RED_Base_ID	    				= 11,//基地
	STUDENT_BLUE_HERO_ID	    			= 101,
	STUDENT_BLUE_ENGINEER_ID  			= 102,
	STUDENT_BLUE_INFANTRY3_ID 			= 103,
	STUDENT_BLUE_INFANTRY4_ID 			= 104,
	STUDENT_BLUE_INFANTRY5_ID 			= 105,
	STUDENT_BLUE_AERIAL_ID    			= 106,
	STUDENT_BLUE_SENTRY_ID	  			= 107,
	STUDENT_BLUE_RadarStation_ID	  = 109,
	STUDENT_BLUE_Outpost_ID	    		= 110,
	STUDENT_BLUE_Base_ID	    			= 111,
	
	//机器人对应客户端id
	RED_HERO_CLIENT_ID	      = 0x0101,
	RED_ENGINEER_CLIENT_ID    = 0x0102,
	RED_INFANTRY3_CLIENT_ID	  = 0x0103,
	RED_INFANTRY4_CLIENT_ID	  = 0x0104,
	RED_INFANTRY5_CLIENT_ID   = 0x0105,
	RED_AERIAL_CLIENT_ID      = 0x0106,
	BLUE_HERO_CLIENT_ID	      = 0x0165,
	BLUE_ENGINEER_CLIENT_ID   = 0x0166,
	BLUE_INFANTRY3_CLIENT_ID  = 0x0167,
	BLUE_INFANTRY4_CLIENT_ID  = 0x0168,
	BLUE_INFANTRY5_CLIENT_ID  = 0x0169,
	BLUE_AERIAL_CLIENT_ID	    = 0x016A,	
}interactive_id_e;
typedef enum
{
	CONSTANT = 0,
	CHASSIS_MODE,
	PICK_BOX,
	UPRAISE_HEIGHT,
	MODE_STATE,
	AUXILIARY_LINE,
}text_twist_t;
//内容ID和命令ID
typedef enum
{
	/*stm32是小端模式，比如一个32位无符号数0x12345678,
	从低地址到高地址依次存储78、56、34、12
	
	32bit宽的数0x12345678在小端模式及大端模式CPU中存放的方式（假设地址从0x4000开始存放）为：
	内存地址		小端模式存放内容		大端模式存放内容
	0x4000			0x78								0x12
	0x4001			0x56								0x34
	0x4002			0x34								0x56
	0x4003			0x12								0x78							*/
	//内容ID
	RobotCommunication														= 0x0200,//可以在0x0200~0x02FF中自行添加一个或多个作为交互ID
	Client_Delete_Graph														= 0x0100,
	Client_Draw_One_Graph													= 0x0101,
	Client_Draw_Two_Graph													= 0x0102,
	Client_Draw_Five_Graph												= 0x0103,
	Client_Draw_Seven_Graph												= 0x0104,
	Client_Draw_Character_Graph										= 0x0110,
	//命令ID
	Robot_communicative_data											= 0x0301,
	Custom_control_interactive_port								= 0x0302,
	Client_map_intercactive_data									= 0x0303,
	Keyboard_mouse_data														= 0x0304,
	
}Content_ID;

//客户端删除图形
typedef __packed struct
{
	uint16_t data_cmd_id;//内容ID、发送者ID、接受者ID和图形设置
	uint16_t sender_ID;
	uint16_t receiver_ID;
	uint8_t operate_tpye;
	uint8_t layer;
}ext_client_custom_graphic_delete_t;

//图形数据配置，具体查看裁判系统串口协议附录
typedef __packed struct
{
	uint8_t graphic_name[3];
	uint32_t operate_tpye:3;// 类型说明符 位域名：位域长度,后面的位域长度表示该位域占了多少个位
	uint32_t graphic_tpye:3;
	uint32_t layer:4;
	uint32_t color:4;
	uint32_t start_angle:9;
	uint32_t end_angle:9;
	uint32_t width:10;
	uint32_t start_x:11;
	uint32_t start_y:11; 
	uint32_t radius:10;
	uint32_t end_x:11;
	uint32_t end_y:11;
}graphic_data_struct_t;

//客户端绘制一个图形
typedef __packed struct
{
	uint16_t data_cmd_id;//内容ID、发送者ID、接受者ID和图形设置
	uint16_t sender_ID;
	uint16_t receiver_ID;
	graphic_data_struct_t grapic_data_struct;
}ext_client_custom_graphic_single_t;

//客户端绘制二个图形
typedef __packed struct
{
	uint16_t data_cmd_id;
	uint16_t sender_ID;
	uint16_t receiver_ID;
	graphic_data_struct_t grapic_data_struct[2];
}ext_client_custom_graphic_double_t;


//客户端绘制五个图形
typedef __packed struct
{
	uint16_t data_cmd_id;
	uint16_t sender_ID;
	uint16_t receiver_ID;
	graphic_data_struct_t grapic_data_struct[5];
	
}ext_client_custom_graphic_five_t;

//客户端绘制七个图形
typedef __packed struct
{
	uint16_t data_cmd_id;
	uint16_t sender_ID;
	uint16_t receiver_ID;
	graphic_data_struct_t grapic_data_struct[7];
}ext_client_custom_graphic_seven_t;

//客户端绘制字符
typedef __packed struct
{
	uint16_t data_cmd_id;
	uint16_t sender_ID;
	uint16_t receiver_ID;
	graphic_data_struct_t grapic_data_struct;
	uint8_t data[30];//需要发送的字符内容
}ext_client_custom_character_t;


// 0x0101 图形数据配置，具体查看裁判系统串口协议附录 2025
typedef __packed struct
{
	uint8_t figure_name[3];
	uint32_t operate_tpye : 3; // 类型说明符 位域名：位域长度,后面的位域长度表示该位域占了多少个位
	uint32_t figure_tpye : 3;
	uint32_t layer : 4;
	uint32_t color : 4;

	uint32_t details_a : 9;
	uint32_t details_b : 9;
	uint32_t width : 10;
	uint32_t start_x : 11;
	uint32_t start_y : 11;
	uint32_t details_c : 10;
	uint32_t details_d : 11;
	uint32_t details_e : 11;
} interaction_figure_t; // interaction_figure_t

 void grapic_line_fun(interaction_figure_t *figure, uint8_t figure_name, uint32_t operate_tpye, uint32_t layer,
							uint32_t color, uint32_t width, uint32_t start_x, uint32_t start_y, uint32_t end_x, uint32_t end_y);






//图形设置
typedef enum
{
	//图形命名，可自行添加
	Text1											=1,
	Text2											=2,
	Text3											=3,
	Text4											=4,
	Text5											=5,
	Text6											=6,
	Text7											=7,
	Pick											=8,
	Pick_Box							  	=9,
	Chassis										=10,
	line_1c										=11,
	line_2c										=12,
	line_3c										=13,
	line_4c										=14,
	line_5c										=15,
	Mode_State								=16,
	HEIGHT										=17,
	//图形操作
	Null_operate								=0,//空操作
	Delete_graph								=1,//删除图层
	Delete_all									=2,//删除所有
	
	Add													=1,//增加
	Change											=2,//修改
	Delete											=3,//删除
	//图形类型
	Straight_line								=0,//直线
	Rectangle										=1,//矩形
	Circle											=2,//整圆
	ellipse											=3,//椭圆
	Circular_arc								=4,//圆弧
	Floatnumber									=5,//浮点数
	Intnumber										=6,//整形数
	Character										=7,//字符
	//图层数
	layer0											=0,
	layer1											=1,
	layer2											=2,
	layer3											=3,
	layer4											=4,
	layer5											=5,
	layer6											=6,
	layer7											=7,
	layer8											=8,
	layer9											=9,
	//颜色
	Redblue											=0,//红蓝主色
	Yellow											=1,//黄色
	Green												=2,//绿色
	Orange											=3,//橙色
	Amaranth										=4,//紫红色
	Pink												=5,//粉色
	Cyan												=6,//青色
	Black												=7,//黑色
	White												=8,//白色
}client_graphic_draw_operate_data_e;

/** 
  * @brief  the data structure receive from judgement
  */
typedef struct
{
	//0x0200~0x02FF发送的内容
  ext_student_interactive_data_t_t	  					ext_student_interactive_data;
	ext_client_custom_graphic_five_t							ext_client_custom_graphic_long_line;
	ext_client_custom_character_t									ext_client_custom_character_text;
	//0x0100~0x0110发送的内容
	ext_client_custom_graphic_delete_t						ext_client_custom_graphic_delete;
	ext_client_custom_graphic_single_t						ext_client_custom_graphic_single;
	ext_client_custom_graphic_double_t						ext_client_custom_graphic_double;
	ext_client_custom_graphic_five_t							ext_client_custom_graphic_five;
	ext_client_custom_graphic_seven_t							ext_client_custom_graphic_seven;
	ext_client_custom_character_t									ext_client_custom_character;
	
	/*字符绘制*/
	ext_client_custom_character_t									ext_client_custom_character_text1;
	ext_client_custom_character_t									ext_client_custom_character_text2;
	ext_client_custom_character_t									ext_client_custom_character_text3;
	ext_client_custom_character_t									ext_client_custom_character_text4;	
	ext_client_custom_character_t									ext_client_custom_character_text5;
	ext_client_custom_character_t									ext_client_custom_character_text6;
	ext_client_custom_character_t									ext_client_custom_character_text7;
	
	/*矿石箱数*/
	ext_client_custom_character_t									ext_client_custom_character_pick;
	ext_client_custom_character_t									ext_client_custom_character_pick_box;
	/*模式*/
	ext_client_custom_character_t									ext_client_custom_character_chassis;
	/*模式状态*/
	ext_client_custom_character_t									ext_client_custom_character_mode_state;
	/*抬升高度*/
	ext_client_custom_character_t									ext_client_custom_character_upraise;
	/*绘制5个图形*/
	ext_client_custom_graphic_five_t							ext_client_custom_graphic_car_line;
	/*绘制2个图形*/
	ext_client_custom_graphic_double_t            ext_client_custom_graphic_catch_line;
} judge_txdata_t;

extern judge_txdata_t judge_send_mesg;
extern fifo_s_t  judge_txdata_fifo;

void judgement_tx_param_init(void);
void judgement_client_packet_pack(uint8_t *p_data);
void judgement_client_graphics_draw_pack(uint8_t text_twist);

//static void client_graphic_DODGE_low_voltage (void);
static void client_graphic_draw_accelerate (void);
static void client_graphic_delete_accelerate (void);
static void client_graphic_draw_cap (void);
static void client_graphic_draw_5m_line (void);
#endif 

