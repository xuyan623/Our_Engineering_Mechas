#include "stm32f4xx.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_spi.h"
#include "mpu6500_reg.h" 
#include "ist8310_reg.h"
#include <string.h>
#include <math.h>
#include "delay.h"
#include "bsp_imu.h"
#include "imu_task.h"
#include "STM32_TIM_BASE.h"


#define SPI_TIMEOUT  0xFFFF  //超时报错

#define MPU_DELAY(x) delay_ms(x)

#define MPU_NSS_LOW()   GPIO_ResetBits(GPIOF, GPIO_Pin_6)
#define MPU_NSS_HIGH()  GPIO_SetBits(GPIOF, GPIO_Pin_6)

uint8_t mpu_buff[14];       /* buffer to save imu raw data */
uint8_t ist_buff[6];        /* buffer to save IST8310 raw data */

mpu_data_t mpu_data;
imu_data_t imu={0};



/**
 * @brief  SPI 发送接收一个字节
 */
static uint8_t SPI5_ReadWriteByte(uint8_t TxData)
{
    __IO uint32_t timeout = SPI_TIMEOUT;

    /* 1. 等待发送缓冲区为空 (TXE) */
    while (SPI_I2S_GetFlagStatus(SPI5, SPI_I2S_FLAG_TXE) == RESET)
    {
        if ((timeout--) == 0) 
        {
            // 超时错误处理：这里返回 0xFF 表示失败
            return 0xFF; 
        }
    }

    /* 2. 发送数据 */
    SPI_I2S_SendData(SPI5, TxData);

    /* 3. 重置超时计数 */
    timeout = SPI_TIMEOUT;

    /* 4. 等待接收缓冲区非空 (RXNE) */
    while (SPI_I2S_GetFlagStatus(SPI5, SPI_I2S_FLAG_RXNE) == RESET)
    {
        if ((timeout--) == 0) 
        {
            return 0xFF;
        }
    }

    /* 5. 返回接收到的数据 */
    return SPI_I2S_ReceiveData(SPI5);
}

/**
 * @brief  写单字节寄存器
 */
uint8_t mpu_write_byte(uint8_t const reg, uint8_t const data)
{
    MPU_NSS_LOW();
    SPI5_ReadWriteByte(reg & 0x7F);  //让最高位为0，进行写操作
    SPI5_ReadWriteByte(data);
    MPU_NSS_HIGH();
    return 0;
}

/**
 * @brief  读单字节寄存器
 */
uint8_t mpu_read_byte(uint8_t const reg)
{
    uint8_t rx;
    MPU_NSS_LOW();
    SPI5_ReadWriteByte(reg | 0x80);
    rx = SPI5_ReadWriteByte(0xFF); // 发送 dummy byte 读取数据
    MPU_NSS_HIGH();
    return rx;
}

/**
 * @brief  读多字节寄存器
 */
uint8_t mpu_read_bytes(uint8_t const regAddr, uint8_t* pData, uint8_t len)
{
    MPU_NSS_LOW();
    SPI5_ReadWriteByte(regAddr | 0x80);
    while (len--) {
        *pData = SPI5_ReadWriteByte(0xFF);
        pData++;
    }
    MPU_NSS_HIGH();
    return 0;
}


uint8_t mpu_set_gyro_fsr(uint8_t fsr)
{
    return mpu_write_byte(MPU6500_GYRO_CONFIG, fsr << 3);
}

uint8_t mpu_set_accel_fsr(uint8_t fsr)
{
    return mpu_write_byte(MPU6500_ACCEL_CONFIG, fsr << 3); 
}

static void ist_reg_write_by_mpu(uint8_t addr, uint8_t data)
{
    mpu_write_byte(MPU6500_I2C_SLV1_CTRL, 0x00);
    MPU_DELAY(2);
    mpu_write_byte(MPU6500_I2C_SLV1_REG, addr);
    MPU_DELAY(2);
    mpu_write_byte(MPU6500_I2C_SLV1_DO, data);
    MPU_DELAY(2);
    mpu_write_byte(MPU6500_I2C_SLV1_CTRL, 0x80 | 0x01);
    MPU_DELAY(10);
}

static uint8_t ist_reg_read_by_mpu(uint8_t addr)
{
    uint8_t retval;
    mpu_write_byte(MPU6500_I2C_SLV4_REG, addr);
    MPU_DELAY(10);
    mpu_write_byte(MPU6500_I2C_SLV4_CTRL, 0x80);
    MPU_DELAY(10);
    retval = mpu_read_byte(MPU6500_I2C_SLV4_DI);
    mpu_write_byte(MPU6500_I2C_SLV4_CTRL, 0x00);
    MPU_DELAY(10);
    return retval;
}

static void mpu_master_i2c_auto_read_config(uint8_t device_address, uint8_t reg_base_addr, uint8_t data_num)
{
    mpu_write_byte(MPU6500_I2C_SLV1_ADDR, device_address);
    MPU_DELAY(2);
    mpu_write_byte(MPU6500_I2C_SLV1_REG, IST8310_R_CONFA);
    MPU_DELAY(2);
    mpu_write_byte(MPU6500_I2C_SLV1_DO, IST8310_ODR_MODE);
    MPU_DELAY(2);

    mpu_write_byte(MPU6500_I2C_SLV0_ADDR, 0x80 | device_address);
    MPU_DELAY(2);
    mpu_write_byte(MPU6500_I2C_SLV0_REG, reg_base_addr);
    MPU_DELAY(2);

    mpu_write_byte(MPU6500_I2C_SLV4_CTRL, 0x03);
    MPU_DELAY(2);
    mpu_write_byte(MPU6500_I2C_MST_DELAY_CTRL, 0x01 | 0x02);
    MPU_DELAY(2);
    mpu_write_byte(MPU6500_I2C_SLV1_CTRL, 0x80 | 0x01);
    MPU_DELAY(6); 
    mpu_write_byte(MPU6500_I2C_SLV0_CTRL, 0x80 | data_num);
    MPU_DELAY(2);
}

uint8_t ist8310_init()
{
    mpu_write_byte(MPU6500_USER_CTRL, 0x30);
    MPU_DELAY(10);
    mpu_write_byte(MPU6500_I2C_MST_CTRL, 0x0d); 
    MPU_DELAY(10);

    mpu_write_byte(MPU6500_I2C_SLV1_ADDR, IST8310_ADDRESS);  
    MPU_DELAY(10);
    mpu_write_byte(MPU6500_I2C_SLV4_ADDR, 0x80 | IST8310_ADDRESS);
    MPU_DELAY(10);

    ist_reg_write_by_mpu(IST8310_R_CONFB, 0x01);
    MPU_DELAY(10);
    if (IST8310_DEVICE_ID_A != ist_reg_read_by_mpu(IST8310_WHO_AM_I))
        return 1;

    ist_reg_write_by_mpu(IST8310_R_CONFB, 0x01); 
    MPU_DELAY(10);

    ist_reg_write_by_mpu(IST8310_R_CONFA, 0x00); 
    if (ist_reg_read_by_mpu(IST8310_R_CONFA) != 0x00)
        return 2;
    MPU_DELAY(10);

    ist_reg_write_by_mpu(IST8310_R_CONFB, 0x00);
    if (ist_reg_read_by_mpu(IST8310_R_CONFB) != 0x00)
        return 3;
    MPU_DELAY(10);
        
    ist_reg_write_by_mpu(IST8310_AVGCNTL, 0x24); 
    if (ist_reg_read_by_mpu(IST8310_AVGCNTL) != 0x24)
        return 4;
    MPU_DELAY(10);

    ist_reg_write_by_mpu(IST8310_PDCNTL, 0xc0);
    if (ist_reg_read_by_mpu(IST8310_PDCNTL) != 0xc0)
        return 5;
    MPU_DELAY(10);

    mpu_write_byte(MPU6500_I2C_SLV1_CTRL, 0x00);
    MPU_DELAY(10);
    mpu_write_byte(MPU6500_I2C_SLV4_CTRL, 0x00);
    MPU_DELAY(10);

    mpu_master_i2c_auto_read_config(IST8310_ADDRESS, IST8310_R_XL, 0x06);
    MPU_DELAY(100);
    return 0;
}

void ist8310_get_data(uint8_t* buff)
{
    mpu_read_bytes(MPU6500_EXT_SENS_DATA_00, buff, 6); 
}


//void mpu_offset_call(void)
//{
//    int i;
//    int16_t ax_sum = 0, ay_sum = 0, az_sum = 0;
//    int16_t gx_sum = 0, gy_sum = 0, gz_sum = 0;
//    int16_t ax_var = 0, ay_var = 0, az_var = 0; // 方差，用于检测静止
//    
//    for (i=0; i<6000; i++)
//    {
//        mpu_read_bytes(MPU6500_ACCEL_XOUT_H, mpu_buff, 14);
//        
//        int16_t ax = mpu_buff[0] << 8 | mpu_buff[1];
//        int16_t ay = mpu_buff[2] << 8 | mpu_buff[3];
//        int16_t az = mpu_buff[4] << 8 | mpu_buff[5];
//        int16_t gx = mpu_buff[8]  << 8 | mpu_buff[9];
//        int16_t gy = mpu_buff[10] << 8 | mpu_buff[11];
//        int16_t gz = mpu_buff[12] << 8 | mpu_buff[13];
//        
//        ax_sum += ax; ay_sum += ay; az_sum += az;
//        gx_sum += gx; gy_sum += gy; gz_sum += gz;
//        
//      
//        
//        MPU_DELAY(5);
//    }
//    
//    // 保存零偏（平均值）
//    mpu_data.ax_offset = ax_sum / 6000;
//    mpu_data.ay_offset = ay_sum / 6000;
//    mpu_data.az_offset = az_sum / 6000;
//    mpu_data.gx_offset = gx_sum / 6000;
//    mpu_data.gy_offset = gy_sum / 6000;
//    mpu_data.gz_offset = gz_sum / 6000;
//    
//}

// 假设你的加速度计量程是 ±8g (4096 LSB/g)，如果不是请修改这个宏
// ±2g -> 16384, ±4g -> 8192, ±8g -> 4096, ±16g -> 2048
#define MPU6500_ACCEL_SENSITIVITY_8G  4096

void mpu_offset_call(void)
{
    uint16_t i;
    int32_t ax_sum = 0, ay_sum = 0, az_sum = 0;
    int32_t gx_sum = 0, gy_sum = 0, gz_sum = 0;
    
    int16_t ax_max, ax_min, ay_max, ay_min, az_max, az_min;
    int16_t gx_max, gx_min, gy_max, gy_min, gz_max, gz_min;
    
    int16_t sample_count = 6000; // 优化：采样2000次即可
    
    // 第一次读取用于初始化最大最小值
    mpu_read_bytes(MPU6500_ACCEL_XOUT_H, mpu_buff, 14);
    ax_max = ax_min = (int16_t)(mpu_buff[0] << 8 | mpu_buff[1]);
    ay_max = ay_min = (int16_t)(mpu_buff[2] << 8 | mpu_buff[3]);
    az_max = az_min = (int16_t)(mpu_buff[4] << 8 | mpu_buff[5]);
    gx_max = gx_min = (int16_t)(mpu_buff[8]  << 8 | mpu_buff[9]);
    gy_max = gy_min = (int16_t)(mpu_buff[10] << 8 | mpu_buff[11]);
    gz_max = gz_min = (int16_t)(mpu_buff[12] << 8 | mpu_buff[13]);
  
	
    for (i = 0; i < sample_count; i++)
    {
						
        mpu_read_bytes(MPU6500_ACCEL_XOUT_H, mpu_buff, 14);
        
        int16_t ax = (int16_t)(mpu_buff[0] << 8 | mpu_buff[1]);
        int16_t ay = (int16_t)(mpu_buff[2] << 8 | mpu_buff[3]);
        int16_t az = (int16_t)(mpu_buff[4] << 8 | mpu_buff[5]);
        int16_t gx = (int16_t)(mpu_buff[8]  << 8 | mpu_buff[9]);
        int16_t gy = (int16_t)(mpu_buff[10] << 8 | mpu_buff[11]);
        int16_t gz = (int16_t)(mpu_buff[12] << 8 | mpu_buff[13]);
        
        // 累加
        ax_sum += ax; ay_sum += ay; az_sum += az;
        gx_sum += gx; gy_sum += gy; gz_sum += gz;
        
        // 更新最大最小值
        if(ax > ax_max) ax_max = ax; if(ax < ax_min) ax_min = ax;
        if(ay > ay_max) ay_max = ay; if(ay < ay_min) ay_min = ay;
        if(az > az_max) az_max = az; if(az < az_min) az_min = az;
        
        if(gx > gx_max) gx_max = gx; if(gx < gx_min) gx_min = gx;
        if(gy > gy_max) gy_max = gy; if(gy < gy_min) gy_min = gy;
        if(gz > gz_max) gz_max = gz; if(gz < gz_min) gz_min = gz;
        
        // 优化：延时1ms，总耗时约2秒
        MPU_DELAY(1);
    }
    
    // 简单的静止检测：如果波动超过阈值，说明校准时在抖动，本次校准可能不准
    // 阈值经验值：加速度波动 < 50LSB，陀螺仪波动 < 20LSB
    if ((ax_max - ax_min > 50) || (ay_max - ay_min > 50) || (az_max - az_min > 50) ||
        (gx_max - gx_min > 20) || (gy_max - gy_min > 20) || (gz_max - gz_min > 20))
    {
      
    }

    // 保存陀螺仪零偏（直接取平均即可）
    mpu_data.gx_offset = (int16_t)(gx_sum / sample_count);
    mpu_data.gy_offset = (int16_t)(gy_sum / sample_count);
    mpu_data.gz_offset = (int16_t)(gz_sum / sample_count);
    
    // 保存加速度计零偏
    // X 和 Y 轴理论上为 0，直接取平均值作为零偏
    mpu_data.ax_offset = (int16_t)(ax_sum / sample_count);
    mpu_data.ay_offset = (int16_t)(ay_sum / sample_count);
    
   
    mpu_data.az_offset = (int16_t)(az_sum / sample_count) - MPU6500_ACCEL_SENSITIVITY_8G;
    
   
}
   


///**
// * @brief  模仿 BMI088 逻辑的高级 MPU6500 校准函数
// * @note   包含静止检测、比例因子校准、低通滤波零偏计算
// */
//void mpu_offset_call(void)
//{
//    static uint32_t startTime;
//    uint32_t i;
//    
//    // 临时变量：用于物理量计算（float）
//    float ax_f, ay_f, az_f, gx_f, gy_f, gz_f;
//    float gNorm, gNormTemp;
//    float gNormMax, gNormMin;
//    float gyroMax[3], gyroMin[3];
//    float gyroDiff[3], gNormDiff;

//    // 累加器
//    float gx_sum = 0, gy_sum = 0, gz_sum = 0;
//    float gNorm_sum = 0;

//    // 最终结果
//    float final_gyro_offset[3] = {0.0f};
//    float final_accel_offset[3] = {0.0f}; // 初始化为0，用于滤波

//    // 阈值定义 (参考 BMI088 逻辑)
//    // 加速度模值波动阈值
//    #define G_NORM_DIFF_THRESH  0.5f
//    // 陀螺仪波动阈值
//    #define GYRO_DIFF_THRESH    0.15f
//    // 重力模值偏差阈值
//    #define G_NORM_BIAS_THRESH  0.5f
//    // 陀螺仪零偏绝对值阈值
//    #define GYRO_OFFSET_THRESH  0.01f

//    // 校准采样次数
//    #define CALI_TIMES  2000 
//    // 加速度计零偏滤波次数
//    #define ACC_CALI_TIMES 1000

//	   startTime = HAL_GetTick();
//    // ==========================================
//    // 第一阶段：循环等待静止，并计算陀螺仪零偏 & 加速度计比例因子
//    // ==========================================
//    do
//    {
//        /* 1. 超时保护：超过 10 秒没找到静止时刻，退出使用默认值 */
//        if (HAL_GetTick() - startTime > 10000)
//        {
//            // 这里可以写默认值，或者直接 return 保留上次的值
//            // 为了简单，这里直接退出，不更新零偏
//            return;
//        }

//        // 每次重试前，先重置累加器
//        gx_sum = 0; gy_sum = 0; gz_sum = 0;
//        gNorm_sum = 0;
//        
//        // 第一次读取，用于初始化最大最小值
//        mpu_read_bytes(MPU6500_ACCEL_XOUT_H, mpu_buff, 14);
//        
//        // 转换为物理单位
//        ax_f = (int16_t)(mpu_buff[0] << 8 | mpu_buff[1]) / 4096.0f * 9.8f;
//        ay_f = (int16_t)(mpu_buff[2] << 8 | mpu_buff[3]) / 4096.0f * 9.8f;
//        az_f = (int16_t)(mpu_buff[4] << 8 | mpu_buff[5]) / 4096.0f * 9.8f;
//        gx_f = (int16_t)(mpu_buff[8]  << 8 | mpu_buff[9])  / 16.384f / 57.3f;
//        gy_f = (int16_t)(mpu_buff[10] << 8 | mpu_buff[11]) / 16.384f / 57.3f;
//        gz_f = (int16_t)(mpu_buff[12] << 8 | mpu_buff[13]) / 16.384f / 57.3f;

//        gNormTemp = sqrtf(ax_f*ax_f + ay_f*ay_f + az_f*az_f);
//        gNormMax = gNormTemp;
//        gNormMin = gNormTemp;
//        for(uint8_t j=0; j<3; j++) {
//            gyroMax[j] = (j==0 ? gx_f : (j==1 ? gy_f : gz_f));
//            gyroMin[j] = gyroMax[j];
//        }

//        /* 2. 循环采样 CALI_TIMES 次 */
//        for (i = 0; i < CALI_TIMES; i++)
//        {
//            mpu_read_bytes(MPU6500_ACCEL_XOUT_H, mpu_buff, 14);

//            // 读取并转换单位
//            ax_f = (int16_t)(mpu_buff[0] << 8 | mpu_buff[1]) / 4096.0f * 9.8f;
//            ay_f = (int16_t)(mpu_buff[2] << 8 | mpu_buff[3]) / 4096.0f * 9.8f;
//            az_f = (int16_t)(mpu_buff[4] << 8 | mpu_buff[5]) / 4096.0f * 9.8f;
//            gx_f = (int16_t)(mpu_buff[8]  << 8 | mpu_buff[9])  / 16.384f / 57.3f;
//            gy_f = (int16_t)(mpu_buff[10] << 8 | mpu_buff[11]) / 16.384f / 57.3f;
//            gz_f = (int16_t)(mpu_buff[12] << 8 | mpu_buff[13]) / 16.384f / 57.3f;

//            // 累加用于求平均
//            gNormTemp = sqrtf(ax_f*ax_f + ay_f*ay_f + az_f*az_f);
//            gNorm_sum += gNormTemp;
//            gx_sum += gx_f;
//            gy_sum += gy_f;
//            gz_sum += gz_f;

//            // 更新最大最小值
//            if (gNormTemp > gNormMax) gNormMax = gNormTemp;
//            if (gNormTemp < gNormMin) gNormMin = gNormTemp;
//            for (uint8_t j = 0; j < 3; j++) {
//                float val = (j==0 ? gx_f : (j==1 ? gy_f : gz_f));
//                if (val > gyroMax[j]) gyroMax[j] = val;
//                if (val < gyroMin[j]) gyroMin[j] = val;
//            }

//            // 实时检测剧烈抖动，如果超出阈值直接 break 本次尝试
//            gNormDiff = gNormMax - gNormMin;
//            gyroDiff[0] = gyroMax[0] - gyroMin[0];
//            gyroDiff[1] = gyroMax[1] - gyroMin[1];
//            gyroDiff[2] = gyroMax[2] - gyroMin[2];

//            if (gNormDiff > G_NORM_DIFF_THRESH ||
//                gyroDiff[0] > GYRO_DIFF_THRESH ||
//                gyroDiff[1] > GYRO_DIFF_THRESH ||
//                gyroDiff[2] > GYRO_DIFF_THRESH)
//            {
//                break;
//            }

//            MPU_DELAY(1);
//        }

//        /* 3. 计算本次尝试的统计结果 */
//        if (i < CALI_TIMES) {
//            // 如果中间 break 了，说明动了，继续 do-while 循环重试
//            continue; 
//        }

//        gNorm = gNorm_sum / (float)CALI_TIMES;
//        final_gyro_offset[0] = gx_sum / (float)CALI_TIMES;
//        final_gyro_offset[1] = gy_sum / (float)CALI_TIMES;
//        final_gyro_offset[2] = gz_sum / (float)CALI_TIMES;

//        // 重新计算最终的波动范围
//        gNormDiff = gNormMax - gNormMin;
//        gyroDiff[0] = gyroMax[0] - gyroMin[0];
//        gyroDiff[1] = gyroMax[1] - gyroMin[1];
//        gyroDiff[2] = gyroMax[2] - gyroMin[2];

//    } 
//    while (gNormDiff > G_NORM_DIFF_THRESH ||          // 加速度波动大
//           fabsf(gNorm - 9.8f) > G_NORM_BIAS_THRESH ||  // 重力模值不对
//           gyroDiff[0] > GYRO_DIFF_THRESH ||            // 陀螺仪波动大
//           gyroDiff[1] > GYRO_DIFF_THRESH ||
//           gyroDiff[2] > GYRO_DIFF_THRESH ||
//           fabsf(final_gyro_offset[0]) > GYRO_OFFSET_THRESH || // 陀螺仪零偏大
//           fabsf(final_gyro_offset[1]) > GYRO_OFFSET_THRESH ||
//           fabsf(final_gyro_offset[2]) > GYRO_OFFSET_THRESH);

//    // ==========================================
//    // 第二阶段：计算加速度计比例因子 & 零偏
//    // ==========================================
//    
//    // 1. 计算比例因子
//    // 理论 gNorm 应该是 9.81，实际是 gNorm。 Scale = 理论 / 实际
//    mpu_data.accel_scale = 9.81f / gNorm;

//    // 2. 计算加速度计零偏
//    // 再次采样，利用刚才算出的 Scale
//    for (i = 0; i < ACC_CALI_TIMES; i++)
//    {
//        mpu_read_bytes(MPU6500_ACCEL_XOUT_H, mpu_buff, 6);

//        // 先转换成原始物理量(未校准比例因子)
//        ax_f = (int16_t)(mpu_buff[0] << 8 | mpu_buff[1]) / 4096.0f * 9.8f;
//        ay_f = (int16_t)(mpu_buff[2] << 8 | mpu_buff[3]) / 4096.0f * 9.8f;
//        az_f = (int16_t)(mpu_buff[4] << 8 | mpu_buff[5]) / 4096.0f * 9.8f;

//        // 应用比例因子
//        ax_f *= mpu_data.accel_scale;
//        ay_f *= mpu_data.accel_scale;
//        az_f *= mpu_data.accel_scale;  

//        // 一阶低通滤波计算零偏
//        // New = Old * 0.8 + Input * 0.2
//        // 注意：静止时，X/Y 轴理论为 0，Z 轴理论为 g (如果没减去g) 或者 0 (如果减去g)
//        // 这里我们计算的是传感器原始读数对应的物理零偏。
//        // 也就是说，我们要存下来的是：当板子不动时，传感器输出了多少。
//        final_accel_offset[0] = final_accel_offset[0] * 0.8f + ax_f * 0.2f;
//        final_accel_offset[1] = final_accel_offset[1] * 0.8f + ay_f * 0.2f;
//        final_accel_offset[2] = final_accel_offset[2] * 0.8f + az_f * 0.2f;

//        MPU_DELAY(2);
//    }

//    // ==========================================
//    // 第三阶段：将 float 结果转回 int16 保存
//    // ==========================================

//    // 1. 陀螺仪零偏
//    // 物理单位 -> LSB. LSB = rad/s * 57.3 * 16.384
//    mpu_data.gx_offset = (int16_t)(final_gyro_offset[0] );
//    mpu_data.gy_offset = (int16_t)(final_gyro_offset[1] );
//    mpu_data.gz_offset = (int16_t)(final_gyro_offset[2] );

//    // 2. 加速度计零偏
//    // 物理单位 -> LSB. LSB = (m/s^2 / 9.8) * 4096
//    mpu_data.ax_offset = (int16_t)(final_accel_offset[0]);
//    mpu_data.ay_offset = (int16_t)(final_accel_offset[1]);
//    // Z轴的处理：通常我们会减去 1g，让 Z 轴在静止时输出 0。
//    // 但因为我们这里有比例因子，逻辑会稍微复杂点。
//    // 最简单的方法：直接保存算出来的零偏，然后应用时：
//    // Accel_Raw = (Raw_ADC - Offset_Raw) * Scale
//    // 这样静止时 Z 轴读数会自动归零。
//    mpu_data.az_offset = (int16_t)(final_accel_offset[2] );

//}


void mpu_get_data()
{
    mpu_read_bytes(MPU6500_ACCEL_XOUT_H, mpu_buff, 14);

    mpu_data.ax   = (mpu_buff[0] << 8 | mpu_buff[1])   - mpu_data.ax_offset;
    mpu_data.ay   = (mpu_buff[2] << 8 | mpu_buff[3])  - mpu_data.ay_offset;
    mpu_data.az   = (mpu_buff[4] << 8 | mpu_buff[5]) - mpu_data.az_offset;
    mpu_data.temp = mpu_buff[6] << 8 | mpu_buff[7];

    mpu_data.gx = ((mpu_buff[8]  << 8 | mpu_buff[9])  - mpu_data.gx_offset);
    mpu_data.gy = ((mpu_buff[10] << 8 | mpu_buff[11])- mpu_data.gy_offset);
    mpu_data.gz = ((mpu_buff[12] << 8 | mpu_buff[13]) - mpu_data.gz_offset);

    ist8310_get_data(ist_buff);
    memcpy(&mpu_data.mx, ist_buff, 6);

    memcpy(&imu.ax, &mpu_data.ax, 6 * sizeof(int16_t));
	
    imu.temp = 21 + mpu_data.temp / 333.87f;
	  /* 2000dps -> rad/s */
	  imu.wx   = mpu_data.gx / 16.384f / 57.3f; 
    imu.wy   = mpu_data.gy / 16.384f / 57.3f; 
    imu.wz   = mpu_data.gz / 16.384f / 57.3f;
}

// -----------------------------------------------------------
// 目标函数: mpu_device_init (标准库版)
// -----------------------------------------------------------

/**
	* @brief  initialize imu mpu6500 and magnet meter ist3810
  * @param  
	* @retval 
  */
    uint8_t id;

uint8_t mpu_device_init(void)
{
    uint8_t i = 0;
    // 配置数组：寄存器地址，配置值
    uint8_t MPU6500_Init_Data[8][2] = {
        { MPU6500_PWR_MGMT_1, 0x80 },     /* Reset Device */ 
        { MPU6500_PWR_MGMT_1, 0x03 },     /* Clock Source - Gyro-Z */ 
        { MPU6500_PWR_MGMT_2, 0x00 },     /* Enable Acc & Gyro */ 
        { MPU6500_CONFIG, 0x04 },         /* LPF 41Hz */ 
        { MPU6500_GYRO_CONFIG, 0x18 },    /* +-2000dps */ 
        { MPU6500_ACCEL_CONFIG, 0x10 },   /* +-8G */ 
        { MPU6500_ACCEL_CONFIG_2, 0x02 }, /* enable LowPassFilter  Set Acc LPF */ 
        { MPU6500_USER_CTRL, 0x20 },      /* Enable AUX */ 
    };

    MPU_DELAY(100);

    id = mpu_read_byte(MPU6500_WHO_AM_I);
    
    // 循环写入初始化配置
    for (i = 0; i < 8; i++)  
    {
        mpu_write_byte(MPU6500_Init_Data[i][0], MPU6500_Init_Data[i][1]);
        MPU_DELAY(1);
    }

    // 设置量程
    mpu_set_gyro_fsr(3);     // +/- 2000dps
    mpu_set_accel_fsr(2);    // +/- 8g

    // 初始化磁力计
   // ist8310_init();
    
    // 校准零偏
    mpu_offset_call();
    
    return 0;
}
