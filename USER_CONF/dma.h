#ifndef _dma_H
#define _dma_H

#include "stm32f4xx.h"

#define DBUS_MAX_LEN 36
#define DBUS_BUFLEN 18

#define USART2_MAX_LEN 50
#define USART2_BUFLEN  18

#define JUDGE_MAX_LEN 39

#define PC_MAX_LEN 512
#define USART8_MAX_LEN 50
#define USART8_BUFLEN  18

#define SPI5_BUFLEN    23

extern uint8_t dbus_buf[2][DBUS_MAX_LEN];
extern uint8_t  judge_rxbuf[2][JUDGE_MAX_LEN];
extern uint8_t  pc_rxbuf[2][PC_MAX_LEN];
extern uint8_t judge_control_rxbuf[2][JUDGE_MAX_LEN];

#define TX_MAX_BUFLEN  17
#define RX_MAX_BUFLEN  16

extern uint8_t  motor_8010_TXbuf[TX_MAX_BUFLEN];
extern uint8_t  motor_8010_RXbuf[RX_MAX_BUFLEN];

void USART1_DMA(void);
void USART3_DMA(void);
void USART6_DMA(void);
void SPI5_DMA(void);
void UART8_DMA(void);



#endif
