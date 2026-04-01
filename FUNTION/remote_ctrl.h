#ifndef _remote_ctrl_H
#define _remote_ctrl_H

#include "stm32f4xx.h"
#include "rc.h"


/*切换模式操作*/                                                          /*IW_DN 1354 IW_UP 694*/                    
/*当rc.sw1不一样时可切换成救援、补给、障碍块搬运*/

#define RC_SW2_MI_IW_DN           ((rc.sw2 == RC_MI) && (glb_sw.last_iw < IW_DN) && (rc.iw >= IW_DN)) 


#define RC_SW2_MI_IW_UP           ((rc.sw2 == RC_MI) && (glb_sw.last_iw > IW_UP) && (rc.iw <= IW_UP))

#define RC_IW_UP_NOMAL     ((rc.sw2 == RC_UP) && (glb_sw.last_iw > IW_UP) && (rc.iw <= IW_UP))


#define RC_SW1_DN						((glb_sw.last_sw1 == RC_MI) && (rc.sw1 == RC_DN))
#define RC_SW1_UP						((glb_sw.last_sw1 == RC_MI) && (rc.sw1 == RC_UP))

#define RC_IW_UP (glb_sw.last_iw > IW_UP) && (rc.iw <= IW_UP)
#define RC_IW_DN (glb_sw.last_iw < IW_DN) && (rc.iw >= IW_DN)


typedef enum
{
	CLAMP_UN_CMD = 0, //
	ACTION_ONE, //确认动作
	ACTION_TWO,//存矿动作
	ACTION_THREE,
}clamp_action_status;

typedef enum
{
	EXCHANGE_UN_CMD  = 0, //
	PICK_ACTION1, 
	PICK_ACTION2,
}exchange_action_status;

enum
{
  RC_UP = 1,
  RC_MI = 3,
  RC_DN = 2,
	IW_UP = 600,		//	min 364		middle 1024	拨轮
	IW_DN = 1260,		//	max 1684
};

typedef enum
{
	CLOCKWISE_ADJUST = 0, //顺时针调整状态
	ONTICLOCKWISE_ADJUST, //逆时针调整状态
}adjusement_t;

typedef struct
{
	/*记录拨杆和拨轮上一次状态*/
  uint8_t last_sw1;
	uint8_t last_last_sw1;//夹取时，因为按键要求过多，因此添加上上次的判断
	
  uint8_t last_sw2;
  uint16_t last_iw;
} sw_record_t;

typedef struct
{
	/*底盘方向*/
  float vx; //前后
  float vy; //左右
  float vw; //转弯
	
	float pit_leg;
	
  /*云台方向*/
  float pit1_v;   //机械臂的pitch1
	float pit2_v;   //pitch2
  float yaw_v;
	
  
	
} rc_ctrl_t;


extern rc_ctrl_t rm;
extern sw_record_t glb_sw;
extern adjusement_t adjustment_cmd;
extern clamp_action_status clamp_action;
extern exchange_action_status exchange_action;

extern int32_t adjustment_slide_v;
extern int32_t adjustment_upraise_v;
extern int32_t adjustment_pit_v;

void remote_ctrl(rc_info_t *rc,uint8_t *dbus_buf);
void remote_ctrl_chassis_hook(void);
void remote_ctrl_gimbal_hook(void);
//void remote_ctrl_shoot_hook(void);

void remote_ctrl_supply_hook(void);
void remote_ctrl_rescue_hook(void);
void remote_ctrl_clamp_hook(void);
void remote_ctrl_barrier_carry_hook(void);
void remote_ctrl_chassis_leg_hook(void);

#endif


