#include "driver/mpu6500/ist8310.h"

#include "driver/mpu6500/ist8310_reg.h"
#include "driver/mpu6500/mpu6500.h"
#include "driver/mpu6500/mpu6500_reg.h"

static void ist_reg_write_by_mpu(uint8_t addr, uint8_t data)
{
    mpu6500_write_single_reg(MPU_I2CSLV1_CTRL, 0x00);
    mpu6500_delay_ms(2);
    mpu6500_write_single_reg(MPU_I2CSLV1_REG, addr);
    mpu6500_delay_ms(2);
    mpu6500_write_single_reg(MPU_I2CSLV1_DO, data);
    mpu6500_delay_ms(2);
    mpu6500_write_single_reg(MPU_I2CSLV1_CTRL, 0x80 | 0x01);
    mpu6500_delay_ms(10);
}

static uint8_t ist_reg_read_by_mpu(uint8_t addr)
{
    uint8_t retval = 0u;

    mpu6500_write_single_reg(MPU_I2CSLV4_REG, addr);
    mpu6500_delay_ms(10);
    mpu6500_write_single_reg(MPU_I2CSLV4_CTRL, 0x80);
    mpu6500_delay_ms(10);
    retval = mpu6500_read_single_reg(MPU_I2CSLV4_DI);
    mpu6500_write_single_reg(MPU_I2CSLV4_CTRL, 0x00);
    mpu6500_delay_ms(10);

    return retval;
}

static void mpu_master_i2c_auto_read_config(uint8_t device_address, uint8_t reg_base_addr, uint8_t data_num)
{
    mpu6500_write_single_reg(MPU_I2CSLV1_ADDR, device_address);
    mpu6500_delay_ms(2);
    mpu6500_write_single_reg(MPU_I2CSLV1_REG, IST8310_R_CONFA);
    mpu6500_delay_ms(2);
    mpu6500_write_single_reg(MPU_I2CSLV1_DO, IST8310_ODR_MODE);
    mpu6500_delay_ms(2);

    mpu6500_write_single_reg(MPU_I2CSLV0_ADDR, 0x80 | device_address);
    mpu6500_delay_ms(2);
    mpu6500_write_single_reg(MPU_I2CSLV0_REG, reg_base_addr);
    mpu6500_delay_ms(2);

    mpu6500_write_single_reg(MPU_I2CSLV4_CTRL, 0x03);
    mpu6500_delay_ms(2);
    mpu6500_write_single_reg(MPU_I2C_MST_DELAY_CTRL, 0x01 | 0x02);
    mpu6500_delay_ms(2);
    mpu6500_write_single_reg(MPU_I2CSLV1_CTRL, 0x80 | 0x01);
    mpu6500_delay_ms(6);
    mpu6500_write_single_reg(MPU_I2CSLV0_CTRL, 0x80 | data_num);
    mpu6500_delay_ms(2);
}

uint8_t ist8310_init(void)
{
    mpu6500_write_single_reg(MPU_USER_CTRL, 0x30);
    mpu6500_delay_ms(10);
    mpu6500_write_single_reg(MPU_I2C_MST_CTRL, 0x0D);
    mpu6500_delay_ms(10);

    mpu6500_write_single_reg(MPU_I2CSLV1_ADDR, IST8310_ADDRESS);
    mpu6500_delay_ms(10);
    mpu6500_write_single_reg(MPU_I2CSLV4_ADDR, 0x80 | IST8310_ADDRESS);
    mpu6500_delay_ms(10);

    ist_reg_write_by_mpu(IST8310_R_CONFB, 0x01);
    mpu6500_delay_ms(10);
    if (IST8310_DEVICE_ID_A != ist_reg_read_by_mpu(IST8310_WHO_AM_I))
    {
        return 1U;
    }

    ist_reg_write_by_mpu(IST8310_R_CONFA, 0x00);
    if (ist_reg_read_by_mpu(IST8310_R_CONFA) != 0x00)
    {
        return 2U;
    }

    ist_reg_write_by_mpu(IST8310_R_CONFB, 0x00);
    if (ist_reg_read_by_mpu(IST8310_R_CONFB) != 0x00)
    {
        return 3U;
    }

    ist_reg_write_by_mpu(IST8310_AVGCNTL, 0x24);
    if (ist_reg_read_by_mpu(IST8310_AVGCNTL) != 0x24)
    {
        return 4U;
    }

    ist_reg_write_by_mpu(IST8310_PDCNTL, 0xC0);
    if (ist_reg_read_by_mpu(IST8310_PDCNTL) != 0xC0)
    {
        return 5U;
    }

    mpu_master_i2c_auto_read_config(IST8310_ADDRESS, IST8310_R_XL, 0x06);
    mpu6500_delay_ms(100);
    return 0U;
}

void ist8310_get_data(int16_t* mx, int16_t* my, int16_t* mz)
{
    mpu_data_t* mpu_data = get_mpu_data();

    if (mpu_data == 0)
    {
        return;
    }

    if (mx != 0)
    {
        *mx = mpu_data->mx;
    }
    if (my != 0)
    {
        *my = mpu_data->my;
    }
    if (mz != 0)
    {
        *mz = mpu_data->mz;
    }
}
