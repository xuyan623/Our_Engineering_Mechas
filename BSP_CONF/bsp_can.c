#include "bsp_can.h"
#include "detect_task.h"
#include "sys_config.h"
#include "stdlib.h"
#include "rc.h"
#include "delay.h"
#include "modeswitch_task.h"
#include "motor_task.h"
#include "User.h"
#include "bsp_dwt.h"
#include "General_function.h"
#include "bsp_dwt.h"

CanRxMsg rx1_message;
CanRxMsg rx2_message;
moto_measure_t moto_chassis[4];        
/***************删除*******************/
moto_measure_t Motor_CAN1_data[7];
moto_measure_t DM_Motor_CAN1_data[7];
moto_measure_t DJ_Motor_CAN2_data[7];
moto_measure_t DM_Motor_CAN2_data[7];

u8 maiIbox;

u32 no_maiTbox_cnt,fail_cnt;

float dt_sec;

 float uint_to_float(int x_int, float x_min, float x_max, int bits)//达妙解算
{
	 float span = x_max - x_min;
	 float offset = x_min;
	 return (float) (x_int*span)/((float)((1<<bits)-1)) + offset;
}

int float_to_uint(float x, float x_min, float x_max, int bits)
{
	float span = x_max - x_min;
	float offset = x_min;
	return (int) ((x-offset)*((float)((1<<bits)-1))/span);
}


static void STD_CAN_RxCpltCallback(CAN_TypeDef *_hcan,CanRxMsg *message)
{
	if(_hcan == CAN1)
	{
		switch(message->StdId)
		{
			case CAN_3508_M1_ID:
			case CAN_3508_M2_ID:
			case CAN_3508_M3_ID:
			case CAN_3508_M4_ID:
			{
				static uint8_t i = 0;
        //处理电机ID号
        i = message->StdId - CAN_3508_M1_ID;
        //处理电机数据宏函数
        moto_chassis[i].msg_cnt++ <= 50 ? get_moto_offset(&moto_chassis[i], message) : DJ_encoder_data_handler(&moto_chassis[i], message);
        //记录时间 -- 异常处理
        err_detector_hook(CHASSIS_M1_OFFLINE + i);
   		}break;
			default:
				break;
		}               
		static uint8_t ID=0;
		ID = message->StdId - 0x201;
		//处理电机数据宏函数
		Motor_CAN1_data[ID].msg_cnt++ <= 50 ? get_moto_offset(&Motor_CAN1_data[ID], message) : DJ_encoder_data_handler(&Motor_CAN1_data[ID], message);

	  if(message->StdId==0xc1)
		{
		  Motor_CAN1_data[4].given_current = (message-> Data[4]<<8)|(message->Data[5]);
	    Motor_CAN1_data[4].msg_cnt++ <= 50 ? get_BM_Moto_offset(&Motor_CAN1_data[4], message) : BM1010b_encoder_data_handler(&Motor_CAN1_data[4], message);     //先读取编码值偏移，然后正式开始计算
      static uint32_t last_rx_time = 0;   
       dt_sec = DWT_GetDeltaT(&last_rx_time); 
		}  
		  if(message->StdId==0xc2)   
		{
		  Motor_CAN1_data[5].given_current = (message-> Data[4]<<8)|(message->Data[5]);
	    Motor_CAN1_data[5].msg_cnt++ <= 50 ? get_BM_Moto_offset(&Motor_CAN1_data[5], message) : BM1010b_encoder_data_handler(&Motor_CAN1_data[5], message);			
    }			                                                                                                                                                        //		//记录时间 -- 异常处理 有需要再说吧，异常好像一般都不是软件问题 err_detector_hook(CHASSIS_M1_OFFLINE + i);

		if(message->StdId==0x20A)  //6020
		{
			//处理电机数据宏函数
			Motor_CAN1_data[6].msg_cnt++ <= 50 ? get_moto_offset(&Motor_CAN1_data[6], message) : DJ_encoder_data_handler(&Motor_CAN1_data[6], message);
		}	
		
				  switch(message->StdId)
		{      
				case 0x00: 
				case 0x01: 
				case 0x02: 
				case 0x03: 
				case 0x04: 
				case 0x05:	
				{
					static uint8_t ID=0;
					ID = message->StdId - 0x00;
					DM_encoder_data_handler(&DM_Motor_CAN1_data[ID],message);
				}break; 
				default:
				break;		
		}	
		
		
	}
	else
	{
			if(message->StdId>=0x200&&message->StdId<=0x208)
		{
			static uint8_t ID=0;
			ID = message->StdId - 0x201;
			//处理电机数据宏函数
			DJ_Motor_CAN2_data[ID].msg_cnt++ <= 50 ? get_moto_offset(&DJ_Motor_CAN2_data[ID], message) : DJ_encoder_data_handler(&DJ_Motor_CAN2_data[ID], message);
		}	
			
		  switch(message->StdId)
		{      
				case 0x00: 
				case 0x01: 
				case 0x02: 
				case 0x03: 
				case 0x04: 
				case 0x05:	
				{
					static uint8_t ID=0;
					ID = message->StdId - 0x00;
					DM_encoder_data_handler(&DM_Motor_CAN2_data[ID],message);
				}break; 
				default:
				break;		
		}	
	}
}

void DM_encoder_data_handler(moto_measure_t* ptr,CanRxMsg *message)
{
   ptr->ecd =  (message->Data[1]<<8)|message->Data[2]; 
   ptr->speed_rpm = (message->Data[3]<<4)|(message->Data[4]>>4);
   ptr->torque = ((message->Data[4]&0x0F)<<8)|message->Data[5];
}

void BM1010b_encoder_data_handler(moto_measure_t* ptr,CanRxMsg *message) //本末电机的解算，包括速度，角度，多圈累计等
{
    uint16_t speed=0;
	
    speed=(uint16_t)(message->Data[2]<<8 | message->Data[3]);

    if(speed>32768)
       ptr->speed_rpm = (speed-65536);
    else
       ptr->speed_rpm =speed;
		
       ptr->speed_rpm = ptr->speed_rpm*360.0f/600.0f;   //600 = 60*10 60为60秒，10为分辨率，最终单位转化为了每秒多少度
		
	     ptr->last_ecd = ptr->ecd;
			 ptr->ecd      = (uint16_t)(message->Data[0] << 8 | message->Data[1]);
			
			if (ptr->ecd - ptr->last_ecd > 16384)
		{
				 ptr->round_cnt--;
				 ptr->ecd_raw_rate = ptr->ecd - ptr->last_ecd - 32768;
		}
			else if (ptr->ecd - ptr->last_ecd < -16384)
		{
				 ptr->round_cnt++;
				 ptr->ecd_raw_rate = ptr->ecd - ptr->last_ecd + 32768;
		}
			else
		{
				 ptr->ecd_raw_rate = ptr->ecd - ptr->last_ecd;
		}
				 ptr->total_ecd = ptr->round_cnt * 32768 + ptr->ecd - ptr->offset_ecd;
}

void DJ_encoder_data_handler(moto_measure_t* ptr, CanRxMsg *message)//大疆解算
{
		ptr->last_ecd = ptr->ecd;
		ptr->ecd      = (uint16_t)(message->Data[0] << 8 | message->Data[1]);		
		if (ptr->ecd - ptr->last_ecd > 4096)
	{
			ptr->round_cnt--;
			ptr->ecd_raw_rate = ptr->ecd - ptr->last_ecd - 8192;
	}
		else if (ptr->ecd - ptr->last_ecd < -4096)
	{
			ptr->round_cnt++;
			ptr->ecd_raw_rate = ptr->ecd - ptr->last_ecd + 8192;
	}
		else
	{
			ptr->ecd_raw_rate = ptr->ecd - ptr->last_ecd;
	}
		ptr->total_ecd = ptr->round_cnt * 8192 + ptr->ecd ;   //这个 - ptr->offset_ecd 其实感觉没那么必要，所以我这里久删掉了，主要是我需要记录这样的偏移
		/* total angle, unit is degree */	
		ptr->speed_rpm     = (int16_t)(message->Data[2] << 8 | message->Data[3]);
		ptr->given_current = (int16_t)(message->Data[4] << 8 | message->Data[5]);
}
/**
  * @brief     get motor initialize offset value
  * @param     ptr: Pointer to a moto_measure_t structure
  * @retval    None
  * @attention this function should be called after system can init
  */
void get_moto_offset(moto_measure_t* ptr, CanRxMsg *message)
{
    ptr->ecd        = (uint16_t)(message->Data[0] << 8 | message->Data[1]);
    ptr->offset_ecd = ptr->ecd;
}

void get_BM_Moto_offset(moto_measure_t* ptr, CanRxMsg *message)
{
    ptr->ecd        = (uint16_t)(message->Data[0] << 8 | message->Data[1]);
    ptr->offset_ecd = ptr->ecd;
}

/**
  * @brief  send current which pid calculate to esc. message to calibrate 6025 gimbal motor esc
  * @param  current value corresponding motor(yaw/pitch/trigger)
  */
void send_can1_low_cur(int16_t iq1, int16_t iq2, int16_t iq3, int16_t iq4)
{
    CanTxMsg TxMessage;
    TxMessage.StdId = CAN_CHASSIS_ALL_ID;//标准标识符
    TxMessage.IDE = CAN_ID_STD;          // 定义标识符的类型为标准标识符
    TxMessage.RTR = CAN_RTR_DATA;        //数据帧
    TxMessage.DLC = 0x08;                //数据长度为0x08
    TxMessage.Data[0] = iq1 >> 8;       
    TxMessage.Data[1] = iq1;
    TxMessage.Data[2] = iq2 >> 8;
    TxMessage.Data[3] = iq2;
    TxMessage.Data[4] = iq3 >> 8;
    TxMessage.Data[5] = iq3;
    TxMessage.Data[6] = iq4 >> 8;
    TxMessage.Data[7] = iq4;
	
    CAN_Transmit(CAN1, &TxMessage);
}

void send_can1_high_cur(int16_t iq1,int16_t iq2,int16_t iq3)
{
    CanTxMsg TxMessage;
    TxMessage.StdId = CAN_UPRAISE_ALL_ID;
    TxMessage.IDE   = CAN_ID_STD;
    TxMessage.RTR   = CAN_RTR_DATA;
    TxMessage.DLC   = 0x08;
    TxMessage.Data[0] = iq1 >> 8;
    TxMessage.Data[1] = iq1;
    TxMessage.Data[2] = iq2 >> 8;
    TxMessage.Data[3] = iq2;
    TxMessage.Data[4] = iq3 >> 8;
    TxMessage.Data[5] = iq3;	
    CAN_Transmit(CAN1, &TxMessage);	
}


void Motor10010B_Enable(void)
{
		CanTxMsg TxMessage;
		TxMessage.DLC = 0x08;
		TxMessage.IDE = CAN_Id_Standard;
		TxMessage.RTR = CAN_RTR_Data;
		TxMessage.StdId = 0x38;//设置ID为1
		TxMessage.Data[0] = 0x02;
		TxMessage.Data[1] = 0x02;
		TxMessage.Data[2] = 0x02;
		TxMessage.Data[3] = 0x02;
		TxMessage.Data[4] = 0x02;
		TxMessage.Data[5] = 0x02;
		TxMessage.Data[6] = 0x02;
		TxMessage.Data[7] = 0x02;
		 while(CAN_Transmit(CAN1, &TxMessage)==CAN_TxStatus_NoMailBox);
		 while(CAN_TransmitStatus(CAN1,0) == CAN_TxStatus_Failed);
}
void Motor10010B_Current(int16_t Current1 ,int16_t Current2,int16_t Current3,int16_t Current4)
{
		CanTxMsg TxMessage;
		TxMessage.DLC = 0x08;
		TxMessage.IDE = CAN_Id_Standard;
		TxMessage.RTR = CAN_RTR_Data;
		TxMessage.StdId = 0X32;
		Current1=constrain(Current1,-7500,7500);
		Current2=constrain(Current2,-7500,7500);		
		TxMessage.Data[0] = (Current1&0xFF00)>>8;
		TxMessage.Data[1] = Current1&0x00FF;
		TxMessage.Data[2] = (Current2&0xFF00)>>8;
		TxMessage.Data[3] = Current2&0x00FF;
		TxMessage.Data[4] = (Current3&0xFF00)>>8;
		TxMessage.Data[5] = Current3&0x00FF;
		TxMessage.Data[6] = (Current4&0xFF00)>>8;
		TxMessage.Data[7] = Current4&0x00FF;
		CAN_Transmit(CAN1, &TxMessage);
}

void Motor10010L_MIT(uint32_t ID,float angle,float V,float Kp,float Kd,float tff)
{
	int tff_raw=float_to_uint(tff,DM10010L_T_MIN,DM10010L_T_MAX,12);
	int V_raw=float_to_uint(V,DM10010L_V_MIN,DM10010L_V_MAX,12);
	int angle_raw=float_to_uint(angle,DM10010L_P_MIN,DM10010L_P_MAX,16);
	int Kp_raw =float_to_uint(Kp,DM_Kp_MIN,DM_Kp_MAX,12);
	int Kd_raw =float_to_uint(Kd,DM_Kd_MIN,DM_Kd_MAX,12);

    CanTxMsg TxMessage;

  	TxMessage.DLC = 0x08;
		TxMessage.IDE = CAN_Id_Standard;
		TxMessage.RTR = CAN_RTR_Data;
		TxMessage.StdId = ID;
		TxMessage.Data[0] = angle_raw >> 8;
		TxMessage.Data[1] = angle_raw;
		TxMessage.Data[2] = V_raw >> 4;
		TxMessage.Data[3] = (V_raw<<4) | (Kp_raw >> 8);
		TxMessage.Data[4] = Kp_raw;
		TxMessage.Data[5] = Kd_raw >> 4;
		TxMessage.Data[6] = (Kd_raw <<4) | (tff_raw >>8);
		TxMessage.Data[7] = tff_raw;
}

void Motor10010L_Position(motor_t* motor)
{
	CanTxMsg TxMessage;	
	 uint8_t *pbuf, *vbuf;
	 static uint8_t Stdid = 0;

	 pbuf=(uint8_t*)&motor->Brushless.angle_ref;
   vbuf=(uint8_t*)&motor->Brushless.spd_ref;
   Stdid = 0x00 + motor->Brushless.MASTER_ID - 1;
	
	TxMessage.DLC = 0x08;
	TxMessage.IDE = CAN_Id_Standard;
	TxMessage.RTR = CAN_RTR_Data;
	TxMessage.StdId = Stdid + 0x100; 
	TxMessage.Data[0] = *pbuf;
	TxMessage.Data[1] = *(pbuf+1);;
	TxMessage.Data[2] = *(pbuf+2);;
	TxMessage.Data[3] = *(pbuf+3);;
	TxMessage.Data[4] = *vbuf;
	TxMessage.Data[5] = *(vbuf+1);
	TxMessage.Data[6] = *(vbuf+2);
	TxMessage.Data[7] = *(vbuf+3);
	while(CAN_Transmit(CAN2, &TxMessage)==CAN_TxStatus_NoMailBox);
	while(CAN_TransmitStatus(CAN2,0) == CAN_TxStatus_Failed);

}





void DM_MIT_send(motor_t* motor)
{
    static uint8_t Stdid = 0;
     Stdid = 0x00 + motor->Brushless.MASTER_ID - 1;

      switch (motor->MOTOR_TYPE)
			{
			case DM_10010L:
			{
				motor->Brushless.angle_ref = tamp_task(motor->Brushless.angle_fdb,0.093,motor->Brushless.angle_ref);
				int tff_raw=float_to_uint(motor->Brushless.controller.mit_params.tff,DM10010L_T_MIN,DM10010L_T_MAX,12);
				int V_raw=0;
				int angle_raw=float_to_uint(motor->Brushless.angle_ref,DM10010L_P_MIN,DM10010L_P_MAX,16);
				int Kp_raw =float_to_uint(motor->Brushless.controller.mit_params.Kp,DM_Kp_MIN,DM_Kp_MAX,12);
				int Kd_raw =float_to_uint(motor->Brushless.controller.mit_params.Kd,DM_Kd_MIN,DM_Kd_MAX,12);
        DM_MIT(Stdid,angle_raw,0,Kp_raw,Kd_raw,tff_raw);
			}	
			break;
			case DM_4340:
      {			
 				motor->Brushless.angle_ref = tamp_task(motor->Brushless.angle_fdb,0.1,motor->Brushless.angle_ref);
				int tff_raw=float_to_uint(motor->Brushless.controller.mit_params.tff,DM4340_T_MIN,DM4340_T_MAX,12);
				int V_raw=0;
				int angle_raw=float_to_uint(motor->Brushless.angle_ref,DM4340_P_MIN,DM4340_P_MAX,16);
				int Kp_raw =float_to_uint(motor->Brushless.controller.mit_params.Kp,DM_Kp_MIN,DM_Kp_MAX,12);
				int Kd_raw =float_to_uint(motor->Brushless.controller.mit_params.Kd,DM_Kd_MIN,DM_Kd_MAX,12);
        DM_MIT(Stdid,angle_raw,0,Kp_raw,Kd_raw,tff_raw);
			}
      break;
      case DM_4310:
      {
				if(motor->MOTOR_NAME==Roll2)
				motor->Brushless.angle_ref = tamp_task(motor->Brushless.angle_fdb,0.2,motor->Brushless.angle_ref);		
				
//				if(motor->MOTOR_NAME==Pitch3)
//				motor->Brushless.angle_ref = tamp_task(motor->Brushless.angle_fdb,0.12,motor->Brushless.angle_ref);		
	
				
				int tff_raw=float_to_uint(motor->Brushless.controller.mit_params.tff,DM4310_T_MIN,DM4310_T_MAX,12);
				int V_raw=0;
				int angle_raw=float_to_uint(motor->Brushless.angle_ref,DM4310_P_MIN,DM4310_P_MAX,16);
				int Kp_raw =float_to_uint(motor->Brushless.controller.mit_params.Kp,DM_Kp_MIN,DM_Kp_MAX,12);
				int Kd_raw =float_to_uint(motor->Brushless.controller.mit_params.Kd,DM_Kd_MIN,DM_Kd_MAX,12);
				
				
								if(motor->MOTOR_NAME==Roll2)
					{							
			         DM_CAN1_MIT(Stdid,angle_raw,0,Kp_raw,Kd_raw,tff_raw);
					}
				else
			  {
							 DM_MIT(Stdid,angle_raw,0,Kp_raw,Kd_raw,tff_raw);
			  }
			}
			break;
			default:
				break;
			}
}


void DM_CAN1_MIT(uint32_t Stdid,int angle_raw,int V_raw,int Kp_raw,int Kd_raw,int tff_raw)
{
    CanTxMsg TxMessage;
  	TxMessage.DLC = 0x08;
		TxMessage.IDE = CAN_Id_Standard;
		TxMessage.RTR = CAN_RTR_Data;
		TxMessage.StdId = Stdid;
		TxMessage.Data[0] = angle_raw >> 8;
		TxMessage.Data[1] = angle_raw;
		TxMessage.Data[2] = V_raw >> 4;
		TxMessage.Data[3] = (V_raw<<4) | (Kp_raw >> 8);
		TxMessage.Data[4] = Kp_raw;
		TxMessage.Data[5] = Kd_raw >> 4;
		TxMessage.Data[6] = (Kd_raw <<4) | (tff_raw >>8);
		TxMessage.Data[7] = tff_raw;

    
	 while(CAN_Transmit(CAN1, &TxMessage)==CAN_TxStatus_NoMailBox);	
//	 while(CAN_TransmitStatus(CAN2,0) == CAN_TxStatus_Failed);
}


void DM_MIT(uint32_t Stdid,int angle_raw,int V_raw,int Kp_raw,int Kd_raw,int tff_raw)
{
    CanTxMsg TxMessage;
  	TxMessage.DLC = 0x08;
		TxMessage.IDE = CAN_Id_Standard;
		TxMessage.RTR = CAN_RTR_Data;
		TxMessage.StdId = Stdid;
		TxMessage.Data[0] = angle_raw >> 8;
		TxMessage.Data[1] = angle_raw;
		TxMessage.Data[2] = V_raw >> 4;
		TxMessage.Data[3] = (V_raw<<4) | (Kp_raw >> 8);
		TxMessage.Data[4] = Kp_raw;
		TxMessage.Data[5] = Kd_raw >> 4;
		TxMessage.Data[6] = (Kd_raw <<4) | (tff_raw >>8);
		TxMessage.Data[7] = tff_raw;

	
	//  CAN_Transmit(CAN2, &TxMessage);

//			uint8_t mbx;
//			uint32_t timeout = 0xF; // 定义一个超时计数器

//			// 尝试获取邮箱
//			mbx = CAN_Transmit(CAN2, &TxMessage);
//			if (mbx == CAN_TxStatus_NoMailBox)
//			{
//					// 处理邮箱满的情况，或者直接返回错误
//					return; 
//			}

//			// 等待发送完成，增加超时判断
//			while(CAN_TransmitStatus(CAN2, mbx) == CAN_TxStatus_Failed)
//			{
//					timeout--;
//					if(timeout == 0) 
//					{
//							// 超时了，可以在这里打印错误日志或进行复位
//							// break; // 跳出死循环，防止卡死
//							return;
//					}
//			}

	    
	 while(CAN_Transmit(CAN2, &TxMessage)==CAN_TxStatus_NoMailBox);	
//	 while(CAN_TransmitStatus(CAN2,0) == CAN_TxStatus_Failed);
}

void DM_MIT_send_zer0(void)
{
	 for(int i=0;i<8;i++)
	{
		 uint8_t Stdid = 0;
		 Stdid = 0x00 + i;
		 DM_MIT(Stdid,0,0,0,0,0);
	}
  
}


void Motor1010B_FeedBackmode(uint8_t motor_ID,uint8_t Interval,uint8_t Data0,uint8_t Data1,uint8_t Data2,uint8_t Data3)
{
		CanTxMsg TxMessage;
		TxMessage.IDE = CAN_Id_Standard;
		TxMessage.DLC = 0x08;
		TxMessage.RTR = CAN_RTR_Data;
		TxMessage.StdId = 0x34;
		TxMessage.Data[0] = motor_ID;//电机ID为1
		TxMessage.Data[1] = 0x01;//主动反馈模式
		TxMessage.Data[2] = Interval;
		TxMessage.Data[3] = Data0;
		TxMessage.Data[4] = Data1;
		TxMessage.Data[5] = Data2;
		TxMessage.Data[6] = Data3;
		TxMessage.Data[7] = 0x00;
		 while(CAN_Transmit(CAN1, &TxMessage)==CAN_TxStatus_NoMailBox);
		 while(CAN_TransmitStatus(CAN1,0) == CAN_TxStatus_Failed);	
}


void DM_Enable(void)
{
	for(int i=0;i<8;i++)
	{
		 uint8_t Stdid = 0;
		 Stdid = 0x00 + i;
		 DM_Enable_Send(Stdid);
	}
}

void DM_Dis_Enable(void)
{
	for(int i=0;i<8;i++)
	{
		 uint8_t Stdid = 0;
		 Stdid = 0x00 + i;
		 DM_Dis_Enable_Send(Stdid);
	}
}

void DM_CAN1_Dis_Enable_Send(uint8_t Stdid)
{
		CanTxMsg TxMessage;
		TxMessage.DLC = 0x08;
		TxMessage.IDE = CAN_Id_Standard;
		TxMessage.RTR = CAN_RTR_Data;
		TxMessage.StdId = Stdid;
		TxMessage.Data[0] = 0xFF;
		TxMessage.Data[1] = 0xFF;
		TxMessage.Data[2] = 0xFF;
		TxMessage.Data[3] = 0xFF;
		TxMessage.Data[4] = 0xFF;
		TxMessage.Data[5] = 0xFF;
		TxMessage.Data[6] = 0xFF;
		TxMessage.Data[7] = 0xFD;  

		while(CAN_Transmit(CAN1, &TxMessage)==CAN_TxStatus_NoMailBox);
		while(CAN_TransmitStatus(CAN1,0) == CAN_TxStatus_Failed);
}

void DM_Dis_Enable_Send(uint8_t Stdid)
{
		CanTxMsg TxMessage;
		TxMessage.DLC = 0x08;
		TxMessage.IDE = CAN_Id_Standard;
		TxMessage.RTR = CAN_RTR_Data;
		TxMessage.StdId = Stdid;//设置ID为1
		TxMessage.Data[0] = 0xFF;
		TxMessage.Data[1] = 0xFF;
		TxMessage.Data[2] = 0xFF;
		TxMessage.Data[3] = 0xFF;
		TxMessage.Data[4] = 0xFF;
		TxMessage.Data[5] = 0xFF;
		TxMessage.Data[6] = 0xFF;
		TxMessage.Data[7] = 0xFD;  
	   // CAN_Transmit(CAN2, &TxMessage);	

		while(CAN_Transmit(CAN2, &TxMessage)==CAN_TxStatus_NoMailBox);
//		while(CAN_TransmitStatus(CAN2,0) == CAN_TxStatus_Failed);
}

void DM_CAN1_Enable_Send(uint8_t Stdid)
{
		CanTxMsg TxMessage;
		TxMessage.DLC = 0x08;
		TxMessage.IDE = CAN_Id_Standard;
		TxMessage.RTR = CAN_RTR_Data;
		TxMessage.StdId = Stdid;
		TxMessage.Data[0] = 0xFF;
		TxMessage.Data[1] = 0xFF;
		TxMessage.Data[2] = 0xFF;
		TxMessage.Data[3] = 0xFF;
		TxMessage.Data[4] = 0xFF;
		TxMessage.Data[5] = 0xFF;
		TxMessage.Data[6] = 0xFF;
		TxMessage.Data[7] = 0xFC;  
	
		while(CAN_Transmit(CAN1, &TxMessage)==CAN_TxStatus_NoMailBox);
//		while(CAN_TransmitStatus(CAN2,0) == CAN_TxStatus_Failed);
}

void DM_Enable_Send(uint8_t Stdid)
{
		CanTxMsg TxMessage;
		TxMessage.DLC = 0x08;
		TxMessage.IDE = CAN_Id_Standard;
		TxMessage.RTR = CAN_RTR_Data;
		TxMessage.StdId = Stdid;
		TxMessage.Data[0] = 0xFF;
		TxMessage.Data[1] = 0xFF;
		TxMessage.Data[2] = 0xFF;
		TxMessage.Data[3] = 0xFF;
		TxMessage.Data[4] = 0xFF;
		TxMessage.Data[5] = 0xFF;
		TxMessage.Data[6] = 0xFF;
		TxMessage.Data[7] = 0xFC;  
	
		while(CAN_Transmit(CAN2, &TxMessage)==CAN_TxStatus_NoMailBox);
//		while(CAN_TransmitStatus(CAN2,0) == CAN_TxStatus_Failed);
}
u32 total_request_3508_maiTbox_cnt;
u32 no_maiTbox_3508_cnt;
u32 fail_3508_cnt;
void send_gear_can2_low_cur(int16_t iq1,int16_t iq2,int16_t iq3,int16_t iq4)
{
    CanTxMsg TxMessage;
    TxMessage.StdId = DJ_Gear_CAN_L_ID;
    TxMessage.IDE   = CAN_ID_STD;
    TxMessage.RTR   = CAN_RTR_DATA;
    TxMessage.DLC   = 0x08;
    TxMessage.Data[0] = iq1 >> 8;
    TxMessage.Data[1] = iq1;
    TxMessage.Data[2] = iq2 >> 8;
    TxMessage.Data[3] = iq2;
    TxMessage.Data[4] = iq3 >> 8;
    TxMessage.Data[5] = iq3;
    TxMessage.Data[6] = iq4 >> 8;
    TxMessage.Data[7] = iq4;	
    CAN_Transmit(CAN2, &TxMessage);	
	
	   total_request_3508_maiTbox_cnt++;
		if(CAN_Transmit(CAN2, &TxMessage) == CAN_TxStatus_NoMailBox)  //返回发送失败
		{
			no_maiTbox_3508_cnt++;
		}
		if(CAN_TransmitStatus(CAN2, maiIbox) == CAN_TxStatus_Failed)
		{
			fail_3508_cnt++;
		}
	
}

void send_gear_can2_high_cur(int16_t iq1,int16_t iq2,int16_t iq3)
{
    CanTxMsg TxMessage;
    TxMessage.StdId = DJ_Gear_CAN_H_ID;
    TxMessage.IDE   = CAN_ID_STD;
    TxMessage.RTR   = CAN_RTR_DATA;
    TxMessage.DLC   = 0x08;
    TxMessage.Data[0] = iq1 >> 8;
    TxMessage.Data[1] = iq1;
    TxMessage.Data[2] = iq2 >> 8;
    TxMessage.Data[3] = iq2;
	  TxMessage.Data[4] = iq3 >> 8;
    TxMessage.Data[5] = iq3;
    CAN_Transmit(CAN2, &TxMessage);	
}

void send_6020_can1_high_cur(int16_t iq1,int16_t iq2,int16_t iq3)
{
    CanTxMsg TxMessage;
    TxMessage.StdId = DJ_6020_CAN_H_ID;
    TxMessage.IDE   = CAN_ID_STD;
    TxMessage.RTR   = CAN_RTR_DATA;
    TxMessage.DLC   = 0x08;
    TxMessage.Data[0] = iq1 >> 8;
    TxMessage.Data[1] = iq1;
    TxMessage.Data[2] = iq2 >> 8;
    TxMessage.Data[3] = iq2;
	  TxMessage.Data[4] = iq3 >> 8;
    TxMessage.Data[5] = iq3;

 	CAN_Transmit(CAN1, &TxMessage);	
}



void send_6020_can2_high_cur(int16_t iq1,int16_t iq2,int16_t iq3)
{
    CanTxMsg TxMessage;
    TxMessage.StdId = DJ_6020_CAN_H_ID;
    TxMessage.IDE   = CAN_ID_STD;
    TxMessage.RTR   = CAN_RTR_DATA;
    TxMessage.DLC   = 0x08;
    TxMessage.Data[0] = iq1 >> 8;
    TxMessage.Data[1] = iq1;
    TxMessage.Data[2] = iq2 >> 8;
    TxMessage.Data[3] = iq2;
	  TxMessage.Data[4] = iq3 >> 8;
    TxMessage.Data[5] = iq3;

 	CAN_Transmit(CAN2, &TxMessage);	
}

//can1中断
void CAN1_RX0_IRQHandler(void)
{
    if (CAN_GetITStatus(CAN1, CAN_IT_FMP0) != RESET)
    {
        CAN_ClearITPendingBit(CAN1, CAN_IT_FMP0);
			//接收CAN的信息，并将读到的信息存进rx1_message
        CAN_Receive(CAN1, CAN_FIFO0, &rx1_message);
			//处理接收数据：具体是哪个电机的数据及数据的具体意义及大小
        STD_CAN_RxCpltCallback(CAN1,&rx1_message);

			 static uint32_t last_rx_time = 0;   
       dt_sec = DWT_GetDeltaT(&last_rx_time); 
    }
}
//can2中断
void CAN2_RX0_IRQHandler(void)
{
    if (CAN_GetITStatus(CAN2, CAN_IT_FMP0) != RESET)
    {
        CAN_ClearITPendingBit(CAN2, CAN_IT_FMP0);
        CAN_Receive(CAN2, CAN_FIFO0, &rx2_message);
        STD_CAN_RxCpltCallback(CAN2,&rx2_message);
			
			
    			
    }
}


