#include "driver/mpu6500/mpu6500.h"

#include "bsp/bsp_init.h"
#include "driver/mpu6500/mpu6500_reg.h"
#define MPU6500_WRITE_REG_NUM  (9U)

static const uint8_t write_mpu6500_reg_data[MPU6500_WRITE_REG_NUM][2] = {
    {MPU_SMPLRT_DIV, 0x09},
    {MPU_PWR_MGMT_1, 0x03},
    {MPU_PWR_MGMT_2, 0x00},
    {MPU_CONFIG, 0x04},
    {MPU_GYRO_CONFIG, 0x18},
    {MPU_ACCEL_CONFIG, 0x10},
    {MPU_ACCEL_CONFIG_2, 0x02},
    {MPU_INTBP_CFG, 0x00},
    {MPU_INT_ENABLE, MPU_RAW_RDY_EN},
};

static mpu_data_t g_mpu_data = {0};

mpu_data_t* get_mpu_data(void)
{
    return &g_mpu_data;
}

uint8_t mpu6500_init(void)
{
    uint8_t write_index = 0U;
    uint8_t readback = 0U;

    mpu6500_write_single_reg(MPU_PWR_MGMT_1, MPU_DEVICE_RESET);
    mpu6500_delay_ms(50U);

    readback = mpu6500_read_single_reg(MPU_WHO_AM_I);
    if (readback != MPU6500_ID)
    {
        return 1U;
    }

    for (write_index = 0U; write_index < MPU6500_WRITE_REG_NUM; write_index++)
    {
        mpu6500_write_single_reg(write_mpu6500_reg_data[write_index][0], write_mpu6500_reg_data[write_index][1]);
        readback = mpu6500_read_single_reg(write_mpu6500_reg_data[write_index][0]);
        if (readback != write_mpu6500_reg_data[write_index][1])
        {
            return 2U;
        }
    }

    return 0U;
}

void mpu6500_write_single_reg(uint8_t reg, uint8_t data)
{
    mpu6500_SPI_NS_L();
    SPI5_ReadWriteByte(reg);
    SPI5_ReadWriteByte(data);
    mpu6500_SPI_NS_H();
}

uint8_t mpu6500_read_single_reg(uint8_t reg)
{
    uint8_t result = 0U;

    mpu6500_SPI_NS_L();
    SPI5_ReadWriteByte((uint8_t)(reg | 0x80U));
    result = SPI5_ReadWriteByte(0xFFU);
    mpu6500_SPI_NS_H();
    return result;
}

void mpu6500_write_muli_reg(uint8_t reg, uint8_t* buf, uint8_t len)
{
    uint8_t index = 0U;

    if (buf == 0)
    {
        return;
    }

    mpu6500_SPI_NS_L();
    SPI5_ReadWriteByte(reg);
    for (index = 0U; index < len; index++)
    {
        SPI5_ReadWriteByte(buf[index]);
    }
    mpu6500_SPI_NS_H();
}

void mpu6500_read_muli_reg(uint8_t reg, uint8_t* buf, uint8_t len)
{
    uint8_t index = 0U;

    if (buf == 0)
    {
        return;
    }

    mpu6500_SPI_NS_L();
    SPI5_ReadWriteByte((uint8_t)(reg | 0x80U));
    for (index = 0U; index < len; index++)
    {
        buf[index] = SPI5_ReadWriteByte(0xFFU);
    }
    mpu6500_SPI_NS_H();
}
