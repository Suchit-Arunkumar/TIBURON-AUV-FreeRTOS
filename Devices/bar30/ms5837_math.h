#ifndef MS5837_MATH_H
#define MS5837_MATH_H

#include <stdint.h>

/*
 * MS5837-30BA calculations, kept apart from the I2C code so they can be
 * checked on a PC against the datasheet's own worked example
 * (tests/host/test_bar30.c). Page numbers are from the TE datasheet,
 * REV C2 12/2019.
 */

/* Number of 16-bit PROM words used: word 0 (CRC + factory) and C1..C6. */
#define MS5837_PROM_WORDS   7

/*
 * CRC-4 over the PROM (page 10). Returns 1 if the 4-bit CRC stored in the
 * top of word 0 matches the other words.
 */
int ms5837_prom_crc_ok(const uint16_t prom[MS5837_PROM_WORDS]);

/*
 * First and second order compensation (pages 11 and 12).
 *
 *   d1, d2       raw 24-bit pressure and temperature conversions
 *   pressure     out, in units of 0.1 mbar (= 10 Pa)
 *   temperature  out, in units of 0.01 degC
 *
 * second_order = 0 gives the first-order result only, which is what the
 * datasheet's example column shows.
 */
void ms5837_compensate(const uint16_t prom[MS5837_PROM_WORDS],
                       uint32_t d1, uint32_t d2, int second_order,
                       int32_t *pressure, int32_t *temperature);

#endif
