#ifndef _can_H
#define _can_H

#include "stm32f4xx.h"

void CAN1_DEVICE(uint8_t mode,uint8_t sjw,uint8_t bs1,uint8_t bs2,uint16_t psc);
void CAN2_DEVICE(uint8_t mode,uint8_t sjw,uint8_t bs1,uint8_t bs2,uint16_t psc);



#endif 

