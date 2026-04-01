#ifndef _rc_H
#define _rc_H


#include "stm32f4xx.h"


typedef __packed struct
{
  /* Ò¡¸Ë */
  int16_t ch1;
  int16_t ch2;
  int16_t ch3;
  int16_t ch4;
  /* ²¦¸Ë */
  uint8_t sw1;
  uint8_t sw2;
	/* ²¦ÂÖ */
	uint16_t iw;
  /* Êó±ê */
  __packed struct
  {
    int16_t x;
    int16_t y;
    int16_t z;
  
    uint8_t l;
    uint8_t r;
  } mouse;
  /* ¼üÅÌ */
  __packed union
  {
    uint16_t key_code;
    __packed struct 
    {
      uint16_t W:1;
      uint16_t S:1;
      uint16_t A:1;
      uint16_t D:1;
      uint16_t SHIFT:1;
      uint16_t CTRL:1;
      uint16_t Q:1;
      uint16_t E:1;
      uint16_t R:1;
      uint16_t F:1;
      uint16_t G:1;
      uint16_t Z:1;
      uint16_t X:1;
      uint16_t C:1;
      uint16_t V:1;
      uint16_t B:1;
	  uint16_t T:1; 
		uint16_t Y:1;
		uint16_t U:1;
		uint16_t I:1;

    } bit;
  } kb;
} rc_info_t;

extern rc_info_t   rc;

void DMA_ReStart(void);



#endif

