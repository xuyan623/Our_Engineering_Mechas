#include "stm32f4xx.h"
#include "math.h"
#include "chassis_task.h"




void Chassis_Power_Control(chassis_t *chassis_power_control)
{

	float chassis_max_power = 120; // 底盘最大给定功率
	float input_power = 0;		 // 限定功率（来自电管）
	float initial_give_power[4]; // 原先将要达到的功率
	float initial_total_power = 0;
	float chassis_power_buffer = 0.0f;
	//用于求解方程的未知量表达式
	float Pcu_2_sum = 0;
	float Pm_2_sum = 0;
	float Pmag_2_sum = 0;
	float Pa_sum = 0;
	//功率损耗系数
	static float constant = 0.8152f;
	static float k1 = 0.000002329f;
	static float k2 = 0.0000001804f;
	static float k3 = 0.0000002496f;
	
	for (uint8_t i = 0; i < 4; i++) // 计算每个轮子原本将要达到的功率
	{
		// Pin = Pm + k1w^2 + k2tao^2 + constant
		// 拟合公式:Pin = k1*w*Icmd + k2*Icmd^2 + k2*w^2 + constant
		initial_give_power[i] = k1*chassis_power_control->current[i]*chassis_power_control->wheel_spd_fdb[i]
						+ k2*chassis_power_control->current[i]*chassis_power_control->current[i]
						+ k3*chassis_power_control->wheel_spd_fdb[i]*chassis_power_control->wheel_spd_fdb[i]
						+	constant;
		
		Pm_2_sum += chassis_power_control->current[i]*chassis_power_control->wheel_spd_fdb[i];
		Pcu_2_sum += chassis_power_control->current[i]*chassis_power_control->current[i];
		Pmag_2_sum += chassis_power_control->wheel_spd_fdb[i]*chassis_power_control->wheel_spd_fdb[i];
		Pa_sum += constant;
		
		if (initial_give_power[i] < 0) // 产生反生电动势，则不加入总功率的计算
			continue;
		initial_total_power += initial_give_power[i]; // 底盘原先将要达到的功率
	}


	if (initial_total_power > chassis_max_power) // 底盘将要超功率--进行功率控制， 若没超，则继续pid运算至超功率
	{
		// 缩放系数（最终功率总和 = chassis_max_power）
		float torque_scale_1 = 0;
		float torque_scale_2 = 0;
		for (uint8_t i = 0; i < 4; i++)
		{
			if (initial_give_power[i] < 0) // 放出功率的就不参与计算
			{
				continue;
			}
			//计算力矩缩放系数
			float a = k2*Pcu_2_sum;
			float b = k1 * Pm_2_sum;
			float c = k3*Pmag_2_sum + Pa_sum - chassis_max_power;
			
			torque_scale_1 = (-b + sqrtf(b*b - 4*a*c))/(2*a);
			torque_scale_2 = (-b - sqrtf(b*b - 4*a*c))/(2*a);
			
			if(((0<= torque_scale_1)&&(torque_scale_1 <=1))&&((0<= torque_scale_2)&&(torque_scale_2 <=1)))
			{
				if(torque_scale_1 > torque_scale_2)
					chassis_power_control->current[i] = torque_scale_1*chassis_power_control->current[i];
				else
					chassis_power_control->current[i] = torque_scale_2*chassis_power_control->current[i];
			}
			else if((0<= torque_scale_1)&&(torque_scale_1 <=1))
				chassis_power_control->current[i] = torque_scale_1*chassis_power_control->current[i];
			else if((0<= torque_scale_2)&&(torque_scale_2 <=1))
				chassis_power_control->current[i] = torque_scale_2*chassis_power_control->current[i];
			else
				chassis_power_control->current[i] = 0;
			
			if(chassis_power_control->current[i] > 16000)
				chassis_power_control->current[i] = 16000;
			else if(chassis_power_control->current[i] < -16000)
				chassis_power_control->current[i] = -16000;
		}
	}
}





