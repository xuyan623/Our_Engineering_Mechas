#include "imu_task.h"
#include "bsp_imu.h"       // 底层驱动
#include "pid.h"
#include "sys_config.h"
#include "math.h"
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "bsp_can.h"
#include "comm_task.h"
#include "delay.h"
#include "bsp_dwt.h"       
#include "dma.h"
#include "detect_task.h"
#include "ahrs.h"               //算法库（2026.1.20 没用）   
#include "modeswitch_task.h"
#include "filter.h"       
#include "stm32f4xx_exti.h"
#include "stm32f4xx_syscfg.h" 
#include "usart.h"
#include "imu_task.h"

#define RAD_TO_ANGLE  57.295779513082320876798154814105f
#define GYRO_LPF_FACTOR 0.55
#define ALPHA_COMPLEMENT 0.005

UBaseType_t imu_stack_surplus;
extern TaskHandle_t imu_Task_Handle;

float imu_pid[3] = {10, 0.1, 0}; 

fp32 gyro_scale_factor[3][3] = {MPU6500_BOARD_INSTALL_SPIN_MATRIX};  
fp32 accel_scale_factor[3][3] = {MPU6500_BOARD_INSTALL_SPIN_MATRIX};

static const fp32 fliter_num[3] = {1.929454039488895f, -0.93178349823448126f, 0.002329458745586203f};

//static const fp32 fliter_num[3] = {
//    1.91119695364553f,   // 对应 y[n-1] 的系数 (即 -a1)
//   -0.91497501979657f,  // 对应 y[n-2] 的系数 (即 -a2)
//    0.00377931673984f    // 对应  x[n] 的系数 (即 b0+b1+b2)
//};


float INS_gyro[3] = {0.0f, 0.0f, 0.0f};
static fp32 INS_accel[3] = {0.0f, 0.0f, 0.0f};
static fp32 INS_mag[3] = {0.0f, 0.0f, 0.0f};
static fp32 INS_quat[4] = {0.0f, 0.0f, 0.0f, 0.0f};

fp32 INS_angle[3] = {0.0f, 0.0f, 0.0f}; 
fp32 INS_angle_final[3] = {0.0f, 0.0f, 0.0f}; // 转换单位后的角度
fp32 INS_palstance_final[3] = {0.0f, 0.0f, 0.0f}; // 角速度

static fp32 accel_fliter_1[3] = {0.0f, 0.0f, 0.0f};
static fp32 accel_fliter_2[3] = {0.0f, 0.0f, 0.0f};
static fp32 accel_fliter_3[3] = {0.0f, 0.0f, 0.0f};

static fp32 gyro_filter[3] = {0.0f, 0.0f, 0.0f};


// 外部引用你的 IMU 数据结构，假设你的定义里没有 yaw_cnt，我们需要加
extern imu_data_t imu; // 来自 bsp_imu.h
extern mpu_data_t mpu_data;


void imu_temp_keep(void)
{
    if(imu.temp > 46)
    {   
        
      GPIO_ResetBits(GPIOB, GPIO_Pin_5); 
    }
    else
    {   
       
      GPIO_SetBits(GPIOB, GPIO_Pin_5); 
    }
}

static void imu_cali_slove(fp32 gyro[3], fp32 accel[3], fp32 mag[3], mpu_data_t *bmi088)
{
    // 临时变量存储转换后的物理量
    fp32 gyro_rad[3];
    fp32 accel_m_s2[3];

    // 1. 先转换单位 (假设 mpu_data 还是 raw 值，这里重新转换)
    // 陀螺仪: 2000dps -> 16.384 LSB/dps -> /57.3 -> rad/s
    gyro_rad[0] = mpu_data.gx / 16.384f / 57.3f;
    gyro_rad[1] = mpu_data.gy / 16.384f / 57.3f;
    gyro_rad[2] = mpu_data.gz / 16.384f / 57.3f;

    // 加速度计: 假设 +/- 8g 量程 -> 4096 LSB/g -> *9.8 -> m/s^2
    // 请根据你的实际量程修改 4096 这个系数
    accel_m_s2[0] = mpu_data.ax / 4096.0f * 9.8f;
    accel_m_s2[1] = mpu_data.ay / 4096.0f * 9.8f;
    accel_m_s2[2] = mpu_data.az / 4096.0f * 9.8f;

    for (uint8_t i = 0; i < 3; i++)
    {
        // 2. 再进行安装误差矩阵旋转
        gyro[i] = gyro_rad[0] * gyro_scale_factor[i][0]
                + gyro_rad[1] * gyro_scale_factor[i][1]
                + gyro_rad[2] * gyro_scale_factor[i][2];

        accel[i] = accel_m_s2[0] * accel_scale_factor[i][0]
                 + accel_m_s2[1] * accel_scale_factor[i][1]
                 + accel_m_s2[2] * accel_scale_factor[i][2];
        
    }
}


float dt =0;
void imu_task(void const *argu)
{
	
	 
    uint16_t i_init = 0;
    float yaw_angle_last = 0;
    uint32_t imu_wake_time = osKernelSysTick();
	
    uint32_t time_last =0;
		 
	  time_last = DWT->CYCCNT;

	

    // 获取任务句柄
    imu_Task_Handle = xTaskGetHandle(pcTaskGetName(NULL));
    
    // 设置启动标志
//   mpu_get_data();
 	 AHRS_init(INS_quat, INS_accel, INS_mag);

	 accel_fliter_1[0] = accel_fliter_2[0] = accel_fliter_3[0] = INS_accel[0];
	 accel_fliter_1[1] = accel_fliter_2[1] = accel_fliter_3[1] = INS_accel[1];
	 accel_fliter_1[2] = accel_fliter_2[2] = accel_fliter_3[2] = INS_accel[2]; 

	
	 
    while (1)
    {

			
		{
		//	imu_temp_keep();
			
    //    mpu_get_data();

		imu_cali_slove(INS_gyro, INS_accel, INS_mag, &mpu_data);
			
		// --- 加速度滤波部分已移除，直接使用 INS_accel ---
			
		// 陀螺仪数据赋值与轴映射（此部分保留，用于适配板子安装方向）
		gyro_filter[A_BOARD_LENGTH] = INS_gyro[0];
		gyro_filter[A_BOARD_WIDTH]	= INS_gyro[1];
		gyro_filter[A_BOARD_HEIGHT] = INS_gyro[2]; 
			
			
		dt = DWT_GetDeltaT(&time_last);      
		
		// 姿态解算：直接传入 INS_accel (原始校准后数据)
		AHRS_update(INS_quat, dt, INS_gyro, INS_accel, INS_mag);
		
		/*获取陀螺仪的值*/
		INS_angle[A_BOARD_HEIGHT] = get_yaw(INS_quat);
		INS_angle[A_BOARD_WIDTH]	= get_pitch(INS_quat);
		INS_angle[A_BOARD_LENGTH]	= get_roll(INS_quat);			
			
			
/**********将三轴陀螺仪弧度转为角度,并且A板安装不同对应的转轴不同，只需在此处调换  PIT  YAW  ROLL  **********/
		INS_angle_final[0] = INS_angle[A_BOARD_LENGTH] * RAD_TO_ANGLE;		//A板的长为轴，灯一侧抬高为正
		INS_palstance_final[0] = gyro_filter[A_BOARD_LENGTH];

		INS_angle_final[1] = INS_angle[A_BOARD_HEIGHT] * RAD_TO_ANGLE;		//A板的高为轴，C板朝上逆时针为正
		INS_palstance_final[1] = gyro_filter[A_BOARD_HEIGHT];

		INS_angle_final[2] = -INS_angle[A_BOARD_WIDTH] * RAD_TO_ANGLE;		//A板的宽为轴，灯一侧抬高为正
		INS_palstance_final[2] = -gyro_filter[A_BOARD_WIDTH];
			
			
			
			
			
			
    //    imu_stack_surplus = uxTaskGetStackHighWaterMark(NULL);	
        err_detector_hook(IMU_OFFLINE);
        
     //   vTaskDelayUntil(&imu_wake_time, IMU_TASK_PERIOD);
    }

			
			
			
		//	imu_temp_keep();
			
//        mpu_get_data();

//			  imu_cali_slove(INS_gyro, INS_accel, INS_mag, &mpu_data);
//			
//		accel_fliter_1[0] = accel_fliter_2[0];
//		accel_fliter_2[0] = accel_fliter_3[0];

//		accel_fliter_3[0] = accel_fliter_2[0] * fliter_num[0] + accel_fliter_1[0] * fliter_num[1] + INS_accel[0] * fliter_num[2];

//		accel_fliter_1[1] = accel_fliter_2[1];
//		accel_fliter_2[1] = accel_fliter_3[1];

//		accel_fliter_3[1] = accel_fliter_2[1] * fliter_num[0] + accel_fliter_1[1] * fliter_num[1] + INS_accel[1] * fliter_num[2];

//		accel_fliter_1[2] = accel_fliter_2[2];
//		accel_fliter_2[2] = accel_fliter_3[2];

//		accel_fliter_3[2] = accel_fliter_2[2] * fliter_num[0] + accel_fliter_1[2] * fliter_num[1] + INS_accel[2] * fliter_num[2];
//  
//			
//			
//				gyro_filter[A_BOARD_LENGTH] = INS_gyro[0];
//				gyro_filter[A_BOARD_WIDTH]	= INS_gyro[1];
//				gyro_filter[A_BOARD_HEIGHT] = INS_gyro[2]; 
//			
//			
//				dt = DWT_GetDeltaT(&time_last);      
//		AHRS_update(INS_quat, dt, INS_gyro, accel_fliter_3, INS_mag);
//		/*获取陀螺仪的值*/
//		INS_angle[A_BOARD_HEIGHT] = get_yaw(INS_quat);
//		INS_angle[A_BOARD_WIDTH]	= get_pitch(INS_quat);
//		INS_angle[A_BOARD_LENGTH]	= get_roll(INS_quat);			
//			
//			
///**********将三轴陀螺仪弧度转为角度,并且A板安装不同对应的转轴不同，只需在此处调换  PIT  YAW  ROLL  **********/
//				INS_angle_final[0] = INS_angle[A_BOARD_LENGTH] * RAD_TO_ANGLE;		//A板的长为轴，灯一侧抬高为正
//		INS_palstance_final[0] = gyro_filter[A_BOARD_LENGTH];

//				INS_angle_final[1] = INS_angle[A_BOARD_HEIGHT] * RAD_TO_ANGLE;		//A板的高为轴，C板朝上逆时针为正
//		INS_palstance_final[1] = gyro_filter[A_BOARD_HEIGHT];

//				INS_angle_final[2] = -INS_angle[A_BOARD_WIDTH] * RAD_TO_ANGLE;		//A板的宽为轴，灯一侧抬高为正
//		INS_palstance_final[2] = -gyro_filter[A_BOARD_WIDTH];
//			
//			
//			
//			
//			
//			
//    //    imu_stack_surplus = uxTaskGetStackHighWaterMark(NULL);	
//        err_detector_hook(IMU_OFFLINE);
//        
//        vTaskDelayUntil(&imu_wake_time, IMU_TASK_PERIOD);
    }
}

























