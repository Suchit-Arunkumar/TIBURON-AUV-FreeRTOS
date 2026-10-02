#ifndef CRC16_H
#define CRC16_H

#include <stdint.h>

// CRC-16-CCITT: poly 0x1021, init 0xFFFF, no reflection, no final XOR.
// Same CRC as the Pico firmware and the Pi, so frames are wire-compatible.
// Python: crcmod.mkCrcFun(0x11021, initCrc=0xFFFF, rev=False)
uint16_t crc16_ccitt(const uint8_t *data, uint32_t length);

#endif
