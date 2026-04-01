/**************************************************************************************** 
  * @file       filters.c
  * @brief      some filter algorithms
  ***************************************************************************************
  * Version     Date           Author        Modification
  * V1.0.0      2021/2/6      CHEN Shu       完成
  ***************************************************************************************
  */

#include "filters.h"

/***************** interface functions ****************/
/**
  * @brief The function to applying the low pass filter
  * @param dataGet the array of the newest fetched data
  * @param lpfData the LPF data structure
  */
void LpfAlgorithm(float *dataGet, s_LPF_DATA_t *lpfData)
{
    static const float filterNum[3] = {1.929454039488895f, -0.93178349823448126f, 0.002329458745586203f};

    if (lpfData->lpfCount == 0) {  //if it is the first time
        lpfData->lpfArray[0][0] = lpfData->lpfArray[1][0] = lpfData->lpfArray[2][0] = (float)dataGet[0]; //x axis
        lpfData->lpfArray[0][1] = lpfData->lpfArray[1][1] = lpfData->lpfArray[2][1] = (float)dataGet[1]; //y axis
        lpfData->lpfArray[0][2] = lpfData->lpfArray[1][2] = lpfData->lpfArray[2][2] = (float)dataGet[2]; //z axis
        lpfData->lpfCount = 1;
    } else {
        //x axis -----------------------------------
        lpfData->lpfArray[0][0] = lpfData->lpfArray[1][0];
        lpfData->lpfArray[1][0] = lpfData->lpfArray[2][0];
        lpfData->lpfArray[2][0] = lpfData->lpfArray[1][0] * filterNum[0] + \
                                  lpfData->lpfArray[0][0] * filterNum[1] + \
                                  (float)dataGet[0] * filterNum[2];
        //y axis -----------------------------------
        lpfData->lpfArray[0][1] = lpfData->lpfArray[1][1];
        lpfData->lpfArray[1][1] = lpfData->lpfArray[2][1];
        lpfData->lpfArray[2][1] = lpfData->lpfArray[1][1] * filterNum[0] + \
                                  lpfData->lpfArray[0][1] * filterNum[1] + \
                                  (float)dataGet[1] * filterNum[2];
        //z axis -----------------------------------
        lpfData->lpfArray[0][2] = lpfData->lpfArray[1][2];
        lpfData->lpfArray[1][2] = lpfData->lpfArray[2][2];
        lpfData->lpfArray[2][2] = lpfData->lpfArray[1][2] * filterNum[0] + \
                                  lpfData->lpfArray[0][2] * filterNum[1] + \
                                  (float)dataGet[2] * filterNum[2];
    }
}

/**
  * @brief the application of the slip window filter algorithm
  * @param dataGet the newest fetched data
  * @param slipData the slip window filter data structure
  */
void SlipFilter(float *dataGet, s_SLIP_FILTER_t *slipData)
{
    uint8_t funCount1, funCount2;
    const uint8_t slipDepth = 8;
    if (slipData->slipCount == 0) {
        //here is 3 axis:
        for (funCount1 = 0; funCount1 < 3; funCount1++) {
            for (funCount2 = 0; funCount2 < slipDepth; funCount2++) {
                slipData->slipArray[funCount2][funCount1] = dataGet[funCount1];
            }
            slipData->slipSum[funCount1] = dataGet[funCount1] * (float)slipDepth;
            slipData->slipOut[funCount1] = dataGet[funCount1];
        }
        slipData->slipCount = 1;
    } else {
        for (funCount1 = 0; funCount1 < 3; funCount1++) {
            slipData->slipSum[funCount1] -= slipData->slipArray[0][funCount1];
            for (funCount2 = 1; funCount2 < slipDepth; funCount2++) {
                slipData->slipArray[funCount2 - 1][funCount1] = slipData->slipArray[funCount2][funCount1];
                if (funCount2 == slipDepth - 1) {
                    slipData->slipArray[funCount2][funCount1] = dataGet[funCount1];
                }
            }
            slipData->slipSum[funCount1] += dataGet[funCount1];
            slipData->slipOut[funCount1] = slipData->slipSum[funCount1] / (float)slipDepth;
        }
    }
}
