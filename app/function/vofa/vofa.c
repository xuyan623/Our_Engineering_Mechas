#include "function/vofa/vofa.h"

static const uint8_t g_vofa_tail[4] = {0x00u, 0x00u, 0x80u, 0x7Fu};
#define VOFA_JUSTFLOAT_MAX_FLOATS (32u)
#define VOFA_JUSTFLOAT_MAX_BYTES  (VOFA_JUSTFLOAT_MAX_FLOATS * 4u + sizeof(g_vofa_tail))
static uint8_t g_vofa_send_frame[VOFA_JUSTFLOAT_MAX_BYTES] = {0u};

static void vofa_float_turn_u8(float fdata, uint8_t bytes[4])
{
    uint8_t index = 0U;
    FloatLongType data;

    data.fdata = fdata;
    for (index = 0U; index < 4U; index++)
    {
        bytes[index] = (uint8_t)(data.ldata >> (index * 8U));
    }
}

void vofa_justfloat_send(Device* serial_dev, const float* fdata, uint16_t fdata_num)
{
    uint16_t index = 0U;
    uint8_t bytes[4] = {0u};
    uint16_t frame_size = 0u;
    uint16_t offset = 0u;

    if (serial_dev == 0 || fdata == 0 || fdata_num == 0U)
    {
        return;
    }

    if (fdata_num > VOFA_JUSTFLOAT_MAX_FLOATS)
    {
        return;
    }

    frame_size = (uint16_t)(fdata_num * 4u + (uint16_t)sizeof(g_vofa_tail));
    for (index = 0U; index < fdata_num; index++)
    {
        vofa_float_turn_u8(fdata[index], bytes);
        g_vofa_send_frame[offset++] = bytes[0];
        g_vofa_send_frame[offset++] = bytes[1];
        g_vofa_send_frame[offset++] = bytes[2];
        g_vofa_send_frame[offset++] = bytes[3];
    }

    for (index = 0U; index < (uint16_t)sizeof(g_vofa_tail); index++)
    {
        g_vofa_send_frame[offset++] = g_vofa_tail[index];
    }

    (void)device_write(serial_dev, 0, g_vofa_send_frame, frame_size);
}
