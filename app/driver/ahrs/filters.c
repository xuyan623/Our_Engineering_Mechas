#include "driver/ahrs/filters.h"

void LpfAlgorithm(float* dataGet, s_LPF_DATA_t* lpfData)
{
    static const float filterNum[3] = {1.929454039488895f, -0.93178349823448126f, 0.002329458745586203f};
    uint8_t axis = 0U;

    if (dataGet == 0 || lpfData == 0)
    {
        return;
    }

    if (lpfData->lpfCount == 0U)
    {
        for (axis = 0U; axis < 3U; axis++)
        {
            lpfData->lpfArray[0][axis] = dataGet[axis];
            lpfData->lpfArray[1][axis] = dataGet[axis];
            lpfData->lpfArray[2][axis] = dataGet[axis];
        }
        lpfData->lpfCount = 1U;
    }
    else
    {
        for (axis = 0U; axis < 3U; axis++)
        {
            lpfData->lpfArray[0][axis] = lpfData->lpfArray[1][axis];
            lpfData->lpfArray[1][axis] = lpfData->lpfArray[2][axis];
            lpfData->lpfArray[2][axis] = lpfData->lpfArray[1][axis] * filterNum[0] +
                                         lpfData->lpfArray[0][axis] * filterNum[1] +
                                         dataGet[axis] * filterNum[2];
            dataGet[axis] = lpfData->lpfArray[2][axis];
        }
    }
}

void FirstOrderLPF_Init(s_FIRST_ORDER_LPF_t* lpf, float cutoff_freq, float sample_freq)
{
    uint8_t axis = 0U;
    float wc = 0.0f;
    float sample_period = 0.0f;

    if (lpf == 0 || cutoff_freq <= 0.0f || sample_freq <= 0.0f)
    {
        return;
    }

    wc = 2.0f * FILTERS_PI * cutoff_freq;
    sample_period = 1.0f / sample_freq;
    lpf->alpha = wc * sample_period / (wc * sample_period + 2.0f);

    for (axis = 0U; axis < 3U; axis++)
    {
        lpf->last[axis] = 0.0f;
    }
    lpf->init_flag = 0U;
}

void FirstOrderLPF_Update(float* data, s_FIRST_ORDER_LPF_t* lpf)
{
    uint8_t axis = 0U;

    if (data == 0 || lpf == 0)
    {
        return;
    }

    if (lpf->init_flag == 0U)
    {
        for (axis = 0U; axis < 3U; axis++)
        {
            lpf->last[axis] = data[axis];
        }
        lpf->init_flag = 1U;
        return;
    }

    for (axis = 0U; axis < 3U; axis++)
    {
        lpf->last[axis] = lpf->alpha * data[axis] + (1.0f - lpf->alpha) * lpf->last[axis];
        data[axis] = lpf->last[axis];
    }
}

void SlipFilter(float* dataGet, s_SLIP_FILTER_t* slipData)
{
    const uint8_t slipDepth = 8U;
    uint8_t axis = 0U;
    uint8_t index = 0U;

    if (dataGet == 0 || slipData == 0)
    {
        return;
    }

    if (slipData->slipCount == 0U)
    {
        for (axis = 0U; axis < 3U; axis++)
        {
            for (index = 0U; index < slipDepth; index++)
            {
                slipData->slipArray[index][axis] = dataGet[axis];
            }
            slipData->slipSum[axis] = dataGet[axis] * (float)slipDepth;
            slipData->slipOut[axis] = dataGet[axis];
        }
        slipData->slipCount = 1U;
        return;
    }

    for (axis = 0U; axis < 3U; axis++)
    {
        slipData->slipSum[axis] -= slipData->slipArray[0][axis];
        for (index = 1U; index < slipDepth; index++)
        {
            slipData->slipArray[index - 1U][axis] = slipData->slipArray[index][axis];
        }
        slipData->slipArray[slipDepth - 1U][axis] = dataGet[axis];
        slipData->slipSum[axis] += dataGet[axis];
        slipData->slipOut[axis] = slipData->slipSum[axis] / (float)slipDepth;
        dataGet[axis] = slipData->slipOut[axis];
    }
}
