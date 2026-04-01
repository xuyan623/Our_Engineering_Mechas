#include "controller.h"
#include "STM32_TIM_BASE.h"
#include "modeswitch_task.h"
#include "judge_rx_data.h"
#include "tim.h"
#include "kalman_filter.h"
#include "bsp_dwt.h"
gyrosope_data_t gyrosope_data,gyrosope_past_data;

float small_model_data[AS5600_NUMBER];
float model_out_angle[AS5600_NUMBER];
float model_out_target_angle[AS5600_NUMBER];
EncoderMultiTurn encoder[6]={0};
float last_total_angle=0;

LADRC_NUM small_modle_angle=
{
	.h=0.003,//定时时间及时间步长
	.r=10,//跟踪速度参数
};


//static int read_encoder(void)
//{
//    int encoderNum = 0;
//    encoderNum = (int)((int16_t)(TIM4->CNT)); /*CNT为uint32, 转为int16*/
//    TIM_SetCounter(TIM4, CNT_INIT);/*CNT设初值*/

//    return encoderNum;
//}
float last_small_data[AS5600_NUMBER]={0};
float now_small_data[AS5600_NUMBER]={0};

//自定义控制器的数据从图传链路传来，最后就是传送到了这里 -- model_out_angle 
void controller_task(void)
{
	usart_data_t usart_controller_data;
	gyrosope_data_t gyrosope_now_data;

	memcpy(&usart_controller_data,&control_recv_mesg.robot_interactive_data,sizeof(usart_controller_data));
	
	switch(judge_recv_mesg.robot_interactive_data.data[0])//第一位为cmd_id,表示传回来的符号的意义  0x302,0x304
	{
		case MODEL_MODE://小模型模式
		{
			memcpy(&small_model_data,&usart_controller_data.main_data,sizeof(small_model_data));
//			for(int now_num=0;now_num<AS5600_NUMBER;now_num++)
//			{
//				now_small_data[now_num]=small_model_data[now_num];
//				model_out_target_angle[now_num]=low_pass_filter(now_small_data[now_num],last_small_data[now_num],0.8);
//			}
//			for(int target_num=0;target_num<AS5600_NUMBER;target_num++)
//			{
//				//LADRC_TD(&small_modle_angle,small_model_data[target_num]);
//				model_out_target_angle[target_num]=small_modle_angle.v1;
//			}
			

		}break;
		case GYROSOPE_MODE://陀螺仪模式
		{
			memcpy(&gyrosope_now_data,&usart_controller_data.main_data,sizeof(gyrosope_data));
		}break;
	}
	if(chassis_mode != CHASSIS_EXCHANGE_MODE )
	{ 
//		memset(&model_out_angle,0,sizeof(model_out_angle));
		model_out_angle[0]=0;
		model_out_angle[1]=0;
		model_out_angle[2]=0;
		model_out_angle[3]=0;
		model_out_angle[4]=0;
//	model_out_angle[5]=0;		
		memset(&gyrosope_data,0,CONTROLLER_DATA_LENGTH);
		memcpy(&gyrosope_past_data,&gyrosope_now_data,sizeof(gyrosope_now_data));//陀螺仪兑换前的数值在其他模式下不断记录用作处理零飘
		
	//	encoder[5].last_angle = model_out_angle[5];
//		encoder[5].last_angle = small_model_data[5];
	//	encoder[5].last_angle /=360.0;0
    model_out_angle[5] = small_model_data[5];
		
	}
	else if(chassis_mode == CHASSIS_EXCHANGE_MODE)
	{ 
		  		
	//  model_out_angle[5]=UpdateMultiTurnAngle(&encoder[5],&small_model_data[5],exchange_Roll);
    model_out_angle[5] = small_model_data[5];
  //   model_out_angle[5] =0;

	  model_out_angle[0]=small_model_data[0]*UPRAIES_RATIO;
		model_out_angle[1]=-small_model_data[1]*JOINT1_RATIO;
		model_out_angle[2]=-small_model_data[2]*JOINT2_RATIO;
		model_out_angle[3]=small_model_data[3]*JOINT_YAW_RATIO;
		model_out_angle[4]=small_model_data[4]*PITCH_ROLL_RATIO;
		
		memcpy(&gyrosope_data,&gyrosope_now_data,sizeof(gyrosope_data));
		gyrosope_data.yaw_gyro_angle=gyrosope_now_data.yaw_gyro_angle-gyrosope_past_data.yaw_gyro_angle;
	}
//	counter_test=read_encoder();

}
//	if(chassis_mode == CHASSIS_EXCHANGE_MODE)//chassis_mode == CHASSIS_EXCHANGE_MODE
//	memcpy(&small_model_data,&judge_recv_mesg.robot_interactive_data,CONTROLLER_DATA_LENGTH);
//	else 
//	memset(&small_model_data,0,CONTROLLER_DATA_LENGTH);

 /**
 * @brief 更新多圈角度
 * @param encoder 编码器对象
 * @param new_angle 最新读取的单圈角度（单位：度）
 * @return 多圈总角度（累计值）

 */

float UpdateMultiTurnAngle(EncoderMultiTurn* encoder, float* new_angle,motor_name_status2 MOTOR_NAME) {

	encoder->MOTOR_NAME=MOTOR_NAME;
	
    float delta_angle = *new_angle - encoder->last_angle;
 //   delta_angle = delta_angle/2;
    if (delta_angle > 180.0f) {
        delta_angle -= 360.0f;
        encoder->total_turns--; // 圈数减1
    } 
    else if (delta_angle < -180.0f) {
        delta_angle += 360.0f;
        encoder->total_turns++; // 圈数加1
    }
    encoder->total_turns=0;
    // 更新单圈角度和总角度
    encoder->current_angle = *new_angle;
    encoder->total_angle = encoder->total_turns * 360.0f + encoder->current_angle;
 //   encoder->last_angle = *new_angle; // 保存当前角度供下次比较

    return encoder->total_angle;
}

void ResetEncoderTurns(EncoderMultiTurn* encoder) {
    
	// 将累计圈数清零
    encoder->total_turns = 0;
    // 重新计算总角度
    encoder->total_angle = encoder->current_angle;
}



