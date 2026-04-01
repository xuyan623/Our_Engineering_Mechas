/**
  ******************************************************************************
  * @file	 bsp_dwt.h
  * @author  Wang Hongxi
  * @version V1.0.0
  * @date    2020/2/14
  * @brief
  ******************************************************************************
  * @attention
  *
  ******************************************************************************
  */
#ifndef _BSP_DWT_H
#define _BSP_DWT_H

#include "stdint.h"

void DWT_Init(uint32_t CPU_Freq_mHz);
float DWT_GetDeltaT(uint32_t *cnt_last);
double DWT_GetDeltaT64(uint32_t *cnt_last);
float DWT_GetTimeline(void);
void DWT_Delay(float Delay);

#endif /* BSP_DWT_H_ */
