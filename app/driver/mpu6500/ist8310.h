#ifndef NEW_ROBOT_IST8310_H
#define NEW_ROBOT_IST8310_H

#include <stdint.h>

uint8_t ist8310_init(void);
void ist8310_get_data(int16_t* mx, int16_t* my, int16_t* mz);

#endif
