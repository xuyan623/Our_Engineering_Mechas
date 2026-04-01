#ifndef __CRC_H
#define __CRC_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>
#include "struct_typedef.h"

extern uint16_t const crc_ccitt_table[256];
extern uint16_t const crc_ccitt_false_table[256];
extern uint16_t crc_ccitt(uint8_t /*const*/ *buffer, size_t len);
extern uint16_t crc_ccitt_false(uint16_t crc, const uint8_t* buffer, uint32_t len);

static uint16_t crc_ccitt_byte(uint16_t crc, const uint8_t c)
{
	return (crc >> 8) ^ crc_ccitt_table[(crc ^ c) & 0xff];
}

static inline uint16_t crc_ccitt_false_byte(uint16_t crc, const uint8_t c)
{
	return (crc << 8) ^ crc_ccitt_false_table[(crc >> 8) ^ c];
}

#ifdef __cplusplus
}
#endif
#endif /*__CRC_H */


