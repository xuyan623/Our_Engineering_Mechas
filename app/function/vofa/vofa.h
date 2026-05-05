#ifndef NEW_ROBOT_VOFA_H
#define NEW_ROBOT_VOFA_H

#include "drivers/model/device.h"
#include <stdint.h>

typedef union
{
    float fdata;
    unsigned long ldata;
} FloatLongType;

void vofa_justfloat_send(Device* serial_dev, const float* fdata, uint16_t fdata_num);

#endif
