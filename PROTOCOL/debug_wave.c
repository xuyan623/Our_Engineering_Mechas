#include "debug_wave.h"
#include "math.h"


static uint32_t n = 0;
float  CHECK;
uint8_t sweep_vofa_stop = 0;
debug_wave_t debug_wave;
Sweep_Type Sweep_type = Sine;
#define Fre_Num_Square	71
#define Fre_Num_Sine	60
float sweep_fre_sine[Fre_Num_Sine] = { 1.000000, 1.500000, 2.000000, 2.500000, 3.000000, 3.500000, 4.000000, 4.500000, 5.000000,
							    5.500000, 6.000000, 6.500000, 7.000000, 7.500000, 8.000000, 8.500000, 9.000000, 9.500000,
 							   10.000000,10.500000,11.000000,11.500000,12.000000,12.500000,13.000000,13.500000,
							   14.000000,14.500000,15.000000,15.500000,16.000000,16.500000,17.000000,17.500000,
							   18.000000,18.500000,19.000000,19.500000,20.000000,20.500000,21.000000,21.500000,
							   22.000000,24.000000,25.000000,26.000000,27.000000,28.000000,29.000000,30.000000,
							   31.000000,32.000000,33.000000,34.000000,35.000000,36.000000,37.000000,38.000000,39.000000,40.000000};

float sweep_fre_square[Fre_Num_Square] = {1.000000,1.200000,1.400000,1.600000,1.800000,2.000000,2.200000,2.400000,2.600000,2.800000,
																					3.000000,3.200000,3.400000,3.600000,3.800000,4.000000,4.200000,4.400000,4.600000,4.800000,
																					5.000000,5.200000,5.400000,5.600000,5.800000,6.000000,6.200000,6.400000,6.600000,6.800000,
																					7.000000,7.200000,7.400000,7.600000,7.800000,8.000000,8.200000,8.400000,8.600000,8.800000,
																					9.000000,9.200000,9.400000,9.600000,9.800000,10.000000,10.200000,10.400000,10.600000,10.800000,
																					11.000000,11.200000,11.400000,11.600000,11.800000,12.000000,12.200000,12.400000,12.600000,
																					12.800000,13.000000,13.200000,13.400000,13.600000,13.800000,14.000000,14.200000,
																					14.400000,14.600000,14.800000,15.000000};

/*正弦波*/
float Sine_wave(debug_wave_t *Debug_wave)
{		 
		 float w,angle_ref;//omg暂时用不上 
	   uint32_t N;
	
		//TIM_FRE 为函数执行频率,等效于采样频率 
		w = Debug_wave->sweep_freq*6.28f/TIM_Fre;// 数字角频率
	
//		omg = Debug_wave->sweep_freq*6.28f;// 模拟角频率
	
		N = (uint32_t)(2*3.14159/w);//周期点数 ,采样数量

		if(n<(uint32_t)(Period*N))
		{
			angle_ref = Debug_wave->A * sinf(w*n);//给定目标角度 
			n++;
		}
		if(n>=(uint32_t)(Period*N))	
			n = 0;
		
		return angle_ref; 
}


/*锯齿波*/
	float Sawtooth_wave(debug_wave_t *Debug_wave)
	{
		
			float w,angle_ref;//omg
		 static uint32_t N = 0;
	 		//TIM_FRE 为函数执行频率,等效于采样频率 
			w = Debug_wave->sweep_freq*6.28f/TIM_Fre;// 数字角频率
//	  omg = Debug_wave->sweep_freq*6.28f;// 模拟角频率
			N = (uint32_t)(2*3.14159/w);//周期点数 ,采样数量
			if(n<(uint32_t)(Period*N))
			{  
				// 线性增长，范围从 0 到 A				
				angle_ref =  Debug_wave->A * (float)(n % N) / N;//给定目标角度 
				if(angle_ref>Debug_wave->A)angle_ref=0;
				n++;
			}
			if(n>=(uint32_t)(Period*N))	
				n = 0;
			
			return angle_ref; 
	}



//扫频1-40 Hz 默认spd_Amplitude=5
float Sweep(void)
	{
		static u32 i_fre = 0;
		float w,speed_ref;//omg暂时用不上
		uint8_t spd_Amplitude = 5;
		uint32_t N;
			if(Sweep_type == Sine)
			{
			
				//TIM_FRE 为函数执行频率,等效于采样频率 
				w = sweep_fre_sine[i_fre]*6.28f/TIM_Fre;//模拟角频率 
				CHECK=w;
//			omg = sweep_fre_sine[i_fre]*6.28f;//数字角频率 
				N = (uint32_t)(2*3.14159/w);//周期点数 ,采样数量
	      if(i_fre<38)
				{
						if(n<(uint32_t)(Period*N))
						{
							speed_ref = spd_Amplitude*sinf(w*n);//给定目标角速度 
							n++;
						}
						if(n>=(uint32_t)(Period*N))
						{
							n = 0;
							i_fre++;//获取下一个频率 
						}
				}
				
				else  //扫完一次频之后就重置
				{
//					  i_fre = 0;
//					  n=0;
//						speed_ref = 0;
//						sweep_vofa_stop = 1;
					speed_ref=0;
				}
				return speed_ref; 
			}
	//扫方波
	else if(Sweep_type == Square)
	{
		static u32 cnt = 0;
		if(i_fre < Fre_Num_Square)
		{
			if(cnt*T_ms < 1000/(sweep_fre_square[i_fre]*2))//50%占空比
			{
				speed_ref = spd_Amplitude;
				cnt++;
			}
			else if((cnt*T_ms >= 1000/(sweep_fre_square[i_fre]*2))&&(cnt*T_ms < 1000/(sweep_fre_square[i_fre])))
			{
				speed_ref = -spd_Amplitude;
				cnt++;
			}
			if(cnt*T_ms >= 1000/(sweep_fre_square[i_fre]))
			{
				cnt = 0;
				n++;
				if(n == Period)
				{
					n = 0;
					i_fre++;
					if(i_fre == Fre_Num_Square)
					{
						speed_ref = 0;
					}
				}
			}
		}
		else//扫频结束
		{
			speed_ref = 0;
			sweep_vofa_stop = 1;
		}
			
		
	}
	else//防报错
		speed_ref = 0;
	
	return speed_ref;
	
	}

	