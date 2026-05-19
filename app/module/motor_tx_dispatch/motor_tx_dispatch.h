#ifndef NEW_ROBOT_MOTOR_TX_DISPATCH_H
#define NEW_ROBOT_MOTOR_TX_DISPATCH_H

#include "core/om_def.h"
#include <stdint.h>

typedef enum
{
    MOTOR_TX_SOURCE_ARM = 0u,
    MOTOR_TX_SOURCE_CHASSIS = 1u,
    MOTOR_TX_SOURCE_OTHER_CONTROL = 2u,
    MOTOR_TX_SOURCE_COUNT
} MotorTxRequestSource;

void motor_tx_dispatch_init(void);
OmRet motor_tx_dispatch_submit(MotorTxRequestSource source);
uint32_t motor_tx_dispatch_drain_sources_mask(void);
OmBool motor_tx_dispatch_take_overflow_flag(void);

#endif
