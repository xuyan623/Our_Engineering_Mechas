#include "pc_tx_data.h"
#include "judge_rx_data.h"
#include "modeswitch_task.h"
//#include "gimbal_task.h"

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

/*小电脑发送*/
static SemaphoreHandle_t pc_txdata_mutex;
fifo_s_t  pc_txdata_fifo;
static uint8_t   pc_txdata_buf[PC_TX_FIFO_BUFLEN];

void pc_tx_param_init(void)
{
    /* create the judge_rxdata_mutex mutex  */  
  pc_txdata_mutex = xSemaphoreCreateMutex();
  
  /* judge data fifo init */
  fifo_s_init(&pc_txdata_fifo, pc_txdata_buf, PC_TX_FIFO_BUFLEN, pc_txdata_mutex);
}


/* data send */
send_pc_t    pc_send_mesg;

uint8_t pc_tx_packet_id = GIMBAL_DATA_ID;
int count_cali_count    = 0;
void pc_send_data_packet_pack(void)
{
//	get_upload_data();
	
	data_packet_pack(SENTRY_DATA_ID, (uint8_t *)&pc_send_mesg.pc_need_information, 
							 sizeof(send_pc_t), UP_REG_ID);
/*	
//	switch (pc_tx_packet_id)
//	{
//		case GIMBAL_DATA_ID:
//		{
//			data_packet_pack(GIMBAL_DATA_ID, (uint8_t *)&pc_send_mesg.gimbal_information, 
//									 sizeof(gimbal_info_t), UP_REG_ID);
//			pc_tx_packet_id = GIMBAL_DATA_ID;
//		}break;
//		case CHASSIS_DATA_ID:
//		{
//			data_packet_pack(CHASSIS_DATA_ID, (uint8_t *)&pc_send_mesg.chassis_information, 
//											 sizeof(chassis_info_t), UP_REG_ID);
//			pc_tx_packet_id = GIMBAL_DATA_ID;
//		}break;
//		case SHOOT_TASK_DATA_ID:
//		{
//			data_packet_pack(SHOOT_TASK_DATA_ID, (uint8_t *)&pc_send_mesg.shoot_task_information,
//											 sizeof(shoot_info_t), UP_REG_ID);
//			pc_tx_packet_id = CHASSIS_DATA_ID;
//		}break;
		
//        case INFANTRY_ERR_ID:
//        {
//          data_packet_pack(INFANTRY_ERR_ID, (uint8_t *)&pc_send_mesg.bottom_error_data,
//                           sizeof(infantry_err_t), UP_REG_ID);
//          pc_tx_packet_id = CONFIG_RESPONSE_ID;
//        }break;
			
//        case CONFIG_RESPONSE_ID:
//        {
//          data_packet_pack(CONFIG_RESPONSE_ID, (uint8_t *)&pc_send_mesg.structure_config_data,
//                           sizeof(config_response_t), UP_REG_ID);
//          pc_tx_packet_id = REMOTE_CTRL_INFO_ID;
//        }break;
			
//        case REMOTE_CTRL_INFO_ID:
//        {
//          data_packet_pack(REMOTE_CTRL_INFO_ID, (uint8_t *)&pc_send_mesg.remote_ctrl_data,
//                           sizeof(rc_info_t), UP_REG_ID);
//          pc_tx_packet_id = BOTTOM_VERSION_ID;
//        }break;
			
//        case BOTTOM_VERSION_ID:
//        {
//          data_packet_pack(BOTTOM_VERSION_ID, (uint8_t *)&pc_send_mesg.version_info_data,
//                           sizeof(version_info_t), UP_REG_ID);
//          pc_tx_packet_id = CHASSIS_DATA_ID;
//        }break;
	}
*/
}

void get_upload_data(void)
{
  taskENTER_CRITICAL();
  
  get_infantry_info();
  
  taskEXIT_CRITICAL();
}


void get_infantry_info(void)
{
//	uint8_t temp_gim_mode = gimbal_mode;

//	/* 获取敌方颜色 */
//	if(judge_recv_mesg.game_robot_state.robot_id>10)
//		pc_send_mesg.pc_need_information.enemy_color = red;	//敌方红		1
//	else
//		pc_send_mesg.pc_need_information.enemy_color = blue;//敌方蓝		2

//	/* get gimable ctrl mode */
//	switch(temp_gim_mode)
//	{
//		case GIMBAL_TRACK_ARMOR:
//		{
//			pc_send_mesg.pc_need_information.main_mode = TRACK_AMOR_MODE;		//自瞄模式
//			pc_send_mesg.pc_need_information.pit = -gimbal.sensor.pit_relative_angle;
//			pc_send_mesg.pc_need_information.yaw = gimbal.sensor.yaw_gyro_angle;//发送陀螺仪的数据
//			pc_send_mesg.pc_need_information.bullet_spd = judge_recv_mesg.shoot_data.bullet_speed;
//		}break;
		
//		default:
//		{
//			pc_send_mesg.pc_need_information.main_mode = NORMAL_CTRL_MODE;	//2
//		}break;
//	}
	
}
