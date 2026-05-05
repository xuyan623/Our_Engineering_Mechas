#ifndef NEW_ROBOT_BOARD_LED_H
#define NEW_ROBOT_BOARD_LED_H

#include "core/om_def.h"
#include <stdint.h>

OmRet board_led_init(void);
void board_led_set_green(OmBool on);
void board_led_set_red(OmBool on);
void board_led_set_user8(uint8_t index, OmBool on);
void board_led_set_user8_mask(uint8_t on_mask);
void board_led_set_user8_all(OmBool on);
void board_led_set_running(void);
void board_led_set_fault(void);

#endif
