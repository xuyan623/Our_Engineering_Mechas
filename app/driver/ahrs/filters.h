#ifndef NEW_ROBOT_FILTERS_H
#define NEW_ROBOT_FILTERS_H

#include <stdint.h>

#ifndef FILTERS_PI
#define FILTERS_PI 3.14159265358979323846f
#endif

typedef struct
{
    float lpfArray[3][3];
    uint8_t lpfCount;
} s_LPF_DATA_t;

typedef struct
{
    float alpha;
    float last[3];
    uint8_t init_flag;
} s_FIRST_ORDER_LPF_t;

typedef struct
{
    float slipArray[8][3];
    float slipOut[3];
    float slipSum[3];
    uint8_t slipCount;
} s_SLIP_FILTER_t;

void LpfAlgorithm(float* dataGet, s_LPF_DATA_t* lpfData);
void FirstOrderLPF_Init(s_FIRST_ORDER_LPF_t* lpf, float cutoff_freq, float sample_freq);
void FirstOrderLPF_Update(float* data, s_FIRST_ORDER_LPF_t* lpf);
void SlipFilter(float* dataGet, s_SLIP_FILTER_t* slipData);

#endif
