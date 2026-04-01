/**************************************************************************************** 
  * @file       filters.h
  * @brief      some filter algorithms
  ***************************************************************************************
  * Version     Date           Author        Modification
  * V1.0.0      2021/2/6      CHEN Shu       完成
  ***************************************************************************************
  */
#ifndef _FILTERS_H
#define _FILTERS_H
#include "stm32f4xx.h"


/*************** structure definitions ***************/
/* Low Pass Filter */
typedef struct {
    float lpfArray[3][3];  /* Row    0~2 is n-2, n-1, now
                              Column 0~2 is x, y, z */
    uint8_t lpfCount;
} s_LPF_DATA_t;

/* Slip Window Filter */
typedef struct {
    float   slipArray[8][3];  /* Row    0~8 is n-8, n-7..., now
                                 Column 0~8 is x, y, z */
    float   slipOut[3];
    float   slipSum[3];
    uint8_t slipCount;
} s_SLIP_FILTER_t;

/***************** interface functions ****************/
void LpfAlgorithm(float *dataGet, s_LPF_DATA_t *lpfData);
void SlipFilter(float *dataGet, s_SLIP_FILTER_t *slipData);

#endif //_FILTERS_H
