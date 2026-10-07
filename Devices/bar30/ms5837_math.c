#include "ms5837_math.h"

int ms5837_prom_crc_ok(const uint16_t prom[MS5837_PROM_WORDS])
{
    /* The datasheet's routine works on 8 words, with the CRC nibble of
     * word 0 and the unused word 7 both zeroed first. Work on a copy so
     * the caller's PROM is left alone. */
    uint16_t n[8];
    uint16_t rem = 0U;

    for (int i = 0; i < MS5837_PROM_WORDS; i++)
    {
        n[i] = prom[i];
    }
    n[0] &= 0x0FFFU;
    n[7]  = 0U;

    for (int cnt = 0; cnt < 16; cnt++)
    {
        if (cnt % 2 == 1)
        {
            rem ^= (uint16_t)(n[cnt >> 1] & 0x00FFU);
        }
        else
        {
            rem ^= (uint16_t)(n[cnt >> 1] >> 8);
        }

        for (int bit = 8; bit > 0; bit--)
        {
            if (rem & 0x8000U)
            {
                rem = (uint16_t)((rem << 1) ^ 0x3000U);
            }
            else
            {
                rem = (uint16_t)(rem << 1);
            }
        }
    }

    uint16_t crc = (uint16_t)((rem >> 12) & 0x000FU);

    return (crc == (uint16_t)(prom[0] >> 12)) ? 1 : 0;
}

void ms5837_compensate(const uint16_t prom[MS5837_PROM_WORDS],
                       uint32_t d1, uint32_t d2, int second_order,
                       int32_t *pressure, int32_t *temperature)
{
    const int64_t c1 = prom[1], c2 = prom[2], c3 = prom[3];
    const int64_t c4 = prom[4], c5 = prom[5], c6 = prom[6];

    /*
     * First order (page 11). 64-bit throughout: OFF and SENS need 41 bits.
     *
     * The datasheet's / 2^n are right shifts, which round down. C's / rounds
     * towards zero, which differs for negative values: with the page 11
     * example, dT * C6 / 2^23 is -18.58, and the datasheet's TEMP of 1981
     * needs -19, not -18. GCC's >> on a negative signed value is an
     * arithmetic shift, so it rounds down as the datasheet expects.
     */
    int64_t dt   = (int64_t)d2 - (c5 << 8);
    int64_t temp = 2000 + ((dt * c6) >> 23);
    int64_t off  = (c2 << 16) + ((c4 * dt) >> 7);
    int64_t sens = (c1 << 15) + ((c3 * dt) >> 8);

    int64_t ti = 0, offi = 0, sensi = 0;

    if (second_order)
    {
        /* Second order (page 12). */
        int64_t t20 = temp - 2000;

        if (temp < 2000)
        {
            ti    = (3 * dt * dt) >> 33;
            offi  = (3 * t20 * t20) >> 1;
            sensi = (5 * t20 * t20) >> 3;

            if (temp < -1500)
            {
                int64_t t15 = temp + 1500;

                offi  += 7 * t15 * t15;
                sensi += 4 * t15 * t15;
            }
        }
        else
        {
            ti    = (2 * dt * dt) >> 37;
            offi  = (t20 * t20) >> 4;
            sensi = 0;
        }
    }

    off  -= offi;
    sens -= sensi;

    *pressure    = (int32_t)(((((int64_t)d1 * sens) >> 21) - off) >> 13);
    *temperature = (int32_t)(temp - ti);
}
