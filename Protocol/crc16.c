#include "crc16.h"

// Bitwise, no lookup table. Pure function with no shared state, so any
// task can call it without locking. 58 bytes per packet is ~470 loop
// iterations - a few microseconds at 180 MHz.
uint16_t crc16_ccitt(const uint8_t *data, uint32_t length)
{
    uint16_t crc = 0xFFFF;

    for (uint32_t i = 0; i < length; i++)
    {
        crc ^= (uint16_t)data[i] << 8;

        for (uint8_t bit = 0; bit < 8; bit++)
        {
            crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021)
                                 : (uint16_t)(crc << 1);
        }
    }

    return crc;
}
