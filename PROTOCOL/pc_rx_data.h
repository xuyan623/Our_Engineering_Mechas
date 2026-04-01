#ifndef _pc_rx_data_H
#define _pc_rx_data_H



#include "stm32f4xx.h"
#include "data_packet.h"

#define PC_RX_FIFO_BUFLEN 500


typedef enum
{
  GLOBAL_NORMAL        = 0,
  SOFTWARE_WARNING     = 1,
  SOFTWARE_ERROR       = 2,
  SOFTWARE_FATAL_ERROR = 3,
  GIMBAL_ERROR         = 4,
  CHASSIS_ERROR        = 5,
  HARAWARE_ERROR       = 6,
} err_level_e;

typedef enum
{
  NO_CONFIG      = 0,
  DEFAULT_CONFIG = 1,
  CUSTOM_CONFIG  = 3,
} struct_config_e;

/** 
  * @brief  bottom software version information
  */
typedef __packed struct
{
  uint8_t num[4];
} version_info_t;

/********** the information from computer **********/

typedef __packed struct
{
  int16_t x_offset;   /* offset(mm) relative to the x-axis of the chassis center */
  int16_t y_offset;   /* offset(mm) relative to the y-axis of the chassis center */
  float   w_spd;      /* rotation speed(degree/s) of chassis */
} chassis_rotate_t;

/** 
  * @brief  chassis control information
  */
typedef __packed struct
{
  uint8_t          ctrl_mode; /* chassis control mode */
  int16_t          x_spd;     /* x-axis move speed(mm/s) of chassis */
  int16_t          y_spd;     /* y-axis move speed(mm/s) of chassis */
  chassis_rotate_t w_info;    /* rotation control of chassis */
	
} chassis_ctrl_t;

/** 
  * @brief  gimbal control information
  */
typedef __packed struct
{
  uint32_t time;
  float    pit_ref;      /* gimbal pitch reference angle(degree) */
  float    yaw_ref;      /* gimbal yaw reference angle(degree) */
  uint8_t  visual_valid; /* visual information valid or not */
	uint8_t	 buff_shoot;		//debug	神符 发射
	
	/**算法对矿石识别对准**/
	int16_t        y_distance;      //左为正 右为负
	uint8_t        judge;           //0表示未对准 1表示对准
	
} gimbal_ctrl_t;

/** 
  * @brief  shoot control information
  */
typedef __packed struct
{
  uint8_t  shoot_cmd;      /* single shoot command */
  uint8_t  c_shoot_cmd;    /* continuous shoot command */
  uint8_t  fric_wheel_run; /* friction run or not */
  uint16_t fric_wheel_spd; /* fricrion wheel speed */
} shoot_ctrl_t;

/** 
  * @brief  robot system error level
  */
typedef __packed struct
{
  err_level_e err_level; /* the error level is included in err_level_e enumeration */
} global_err_level_t;

/** 
  * @brief  infantry structure configuration information
  */
typedef __packed struct
{
  struct_config_e  chassis_config;  /* chassis structure config state */
  uint16_t         wheel_perimeter; /* the perimeter(mm) of wheel */
  uint16_t         wheel_track;     /* wheel track distance(mm) */
  uint16_t         wheel_base;      /* wheelbase distance(mm) */
  struct_config_e  gimbal_config;   /* gimbal structure config state */
  int16_t          gimbal_x_offset; /* gimbal offset(mm) relative to the x-axis of the chassis center */
  int16_t          gimbal_y_offset; /* gimbal offset(mm) relative to the y-axis of the chassis center */
} infantry_structure_t;

/** 
  * @brief  gimbal calibrate command
  */
typedef __packed struct
{
  uint8_t type;        /* 0x01 calibrate gimbal center, 0x02 calibrate camera */
} cali_cmd_t;

typedef struct
{
  /* data receive */
  chassis_ctrl_t       chassis_control_data;
  gimbal_ctrl_t        gimbal_control_data; 
  shoot_ctrl_t         shoot_control_data; 
  global_err_level_t   global_error_level;
  infantry_structure_t structure_data;		
  cali_cmd_t           cali_cmd_data;	
} receive_pc_t;

extern uart_dma_rxdata_t pc_rx_obj;
extern unpack_data_t pc_unpack_obj;
extern receive_pc_t pc_recv_mesg;
extern infantry_structure_t glb_struct;

void pc_rx_param_init(void);
void pc_data_handler(uint8_t *p_frame);

#endif
