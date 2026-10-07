/*
 * Host test for the MS5837 maths (Devices/bar30/ms5837_math.c), against
 * the "Example / Typical" column of the TE datasheet (REV C2 12/2019,
 * page 11) and the CRC-4 routine on page 10.
 */
#include "test.h"
#include "ms5837_math.h"

/* Reference CRC-4: the datasheet's C code, copied as printed. */
static unsigned char crc4_datasheet(unsigned int n_prom[])
{
    int cnt;
    unsigned int n_rem = 0;
    unsigned char n_bit;

    n_prom[0] = ((n_prom[0]) & 0x0FFF);
    n_prom[7] = 0;
    for (cnt = 0; cnt < 16; cnt++)
    {
        if (cnt % 2 == 1) n_rem ^= (unsigned short)((n_prom[cnt >> 1]) & 0x00FF);
        else              n_rem ^= (unsigned short)(n_prom[cnt >> 1] >> 8);
        for (n_bit = 8; n_bit > 0; n_bit--)
        {
            if (n_rem & (0x8000)) n_rem = (n_rem << 1) ^ 0x3000;
            else                  n_rem = (n_rem << 1);
        }
    }
    n_rem = ((n_rem >> 12) & 0x000F);
    return (unsigned char)(n_rem ^ 0x00);
}

int main(void)
{
    /* Word 0's low 12 bits: product type 0x1A at bits [11:5] (page 9);
     * C1..C6 from the example column on page 11. */
    uint16_t prom[7] = { 0x1A << 5, 34982, 36352, 20328, 22354, 26646, 26146 };

    /* Put the datasheet's own CRC in the top nibble of word 0. */
    unsigned int n[8] = { prom[0], prom[1], prom[2], prom[3], prom[4], prom[5], prom[6], 0 };
    prom[0] |= (uint16_t)(crc4_datasheet(n) << 12);

    CHECK(ms5837_prom_crc_ok(prom) == 1);

    /* Any single-bit change in the calibration words is caught. */
    for (int w = 0; w < 7; w++)
    {
        for (int b = 0; b < 16; b++)
        {
            if (w == 0 && b >= 12)
            {
                continue;       /* flipping the stored CRC itself */
            }
            uint16_t bad[7];
            for (int i = 0; i < 7; i++) bad[i] = prom[i];
            bad[w] ^= (uint16_t)(1u << b);
            CHECK(ms5837_prom_crc_ok(bad) == 0);
        }
    }

    /* First-order example: D1 = 4958179, D2 = 6815414 gives
     * TEMP = 1981 (19.81 degC) and P = 39998 (3999.8 mbar). */
    int32_t p, t;
    ms5837_compensate(prom, 4958179, 6815414, 0, &p, &t);
    CHECK(t == 1981);
    CHECK(p == 39998);

    /* Second order at 19.81 degC (below 20 degC) must lower the result
     * slightly, never by more than a few mbar. */
    int32_t p2, t2;
    ms5837_compensate(prom, 4958179, 6815414, 1, &p2, &t2);
    CHECK(t2 <= t && t2 > t - 50);
    CHECK(p2 <= p && p2 > p - 50);

    return test_summary("bar30");
}
