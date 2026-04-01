#ifndef _spi_H
#define _spi_H


#include "stm32f4xx.h"

void MySPI_W_SS(uint8_t BitValue);
void MySPI_W_SCK(uint8_t BitValue);
uint8_t MySPI_R_MISO_ACCEL(void);
uint8_t MySPI_R_MISO_GYRO(void);
void SPI_DEVICE(void);
void MySPI_Start(void);
void MySPI_Stop(void);
uint8_t MySPI_SwapByte_GYRO(uint8_t ByteSend);
uint8_t MySPI_SwapByte_ACCEL(uint8_t ByteSend);
void MySPI_W_MOSI_GYRO(uint8_t BitValue);
void MySPI_W_MOSI_ACCEL(uint8_t BitValue);
void BMI088_read_muli_reg_ACCLE(uint8_t reg, uint8_t *buf, uint8_t len);
void BMI088_read_muli_reg_GORY(uint8_t reg, uint8_t *buf, uint8_t len);
#endif

