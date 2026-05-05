#ifndef NEW_ROBOT_MPU6500_REG_H
#define NEW_ROBOT_MPU6500_REG_H

#define MPU6500_ID               (0x70)

#define MPU_SMPLRT_DIV           (0x19)
#define MPU_CONFIG               (0x1A)
#define MPU_GYRO_CONFIG          (0x1B)
#define MPU_ACCEL_CONFIG         (0x1C)
#define MPU_ACCEL_CONFIG_2       (0x1D)

#define MPU_I2C_MST_CTRL         (0x24)
#define MPU_I2CSLV0_ADDR         (0x25)
#define MPU_I2CSLV0_REG          (0x26)
#define MPU_I2CSLV0_CTRL         (0x27)
#define MPU_I2CSLV1_ADDR         (0x28)
#define MPU_I2CSLV1_REG          (0x29)
#define MPU_I2CSLV1_CTRL         (0x2A)
#define MPU_I2CSLV4_ADDR         (0x31)
#define MPU_I2CSLV4_REG          (0x32)
#define MPU_I2CSLV4_CTRL         (0x34)
#define MPU_I2CSLV4_DI           (0x35)
#define MPU_I2CSLV1_DO           (0x64)
#define MPU_I2C_MST_DELAY_CTRL   (0x67)

#define MPU_ACCEL_XOUT_H         (0x3B)
#define MPU_EXT_SENS_DATA_00     (0x49)

#define MPU_USER_CTRL            (0x6A)
#define MPU_PWR_MGMT_1           (0x6B)
#define MPU_PWR_MGMT_2           (0x6C)
#define MPU_WHO_AM_I             (0x75)

#define MPU_INTBP_CFG            (0x37)
#define MPU_INT_ENABLE           (0x38)
#define MPU_INT_STATUS           (0x3A)

#define MPU_DEVICE_RESET         (0x80)
#define MPU_RAW_RDY_EN           (0x01)

#endif
