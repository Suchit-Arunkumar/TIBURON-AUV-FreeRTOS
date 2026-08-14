#include "bar30.h"
#include "i2c.h"

#define BAR30_ADDR     0x76
#define BAR30_RESET    0x1E
#define BAR30_PROM     0xA0
#define BAR30_CONV_D1  0x48
#define BAR30_CONV_D2  0x58
#define BAR30_ADC_READ 0x00

static uint16_t prom[8];

static void delay_ms_simple(uint32_t ms)
{
    for (uint32_t i = 0; i < ms * 8000; i++)
        __NOP();
}

void bar30_init(void)
{
    uint8_t cmd;
    uint8_t buf[2];

    cmd = BAR30_RESET;
    i2c_write(BAR30_ADDR , &cmd, 1);

    delay_ms_simple(10);

    for(int i = 0; i < 8; i++){
        cmd = BAR30_PROM + (i*2);
        i2c_write(BAR30_ADDR , &cmd, 1);
        i2c_read(BAR30_ADDR , buf, 2);
        prom[i] = (buf[0] << 8) | buf[1];
    }
}

float bar30_read(void)
{
    uint8_t cmd;
    uint8_t buf[3];
    uint32_t D1, D2;
    int32_t dT, TEMP;
    int64_t OFF, SENS, P;

    cmd = BAR30_CONV_D1;
    i2c_write(BAR30_ADDR, &cmd, 1);
    delay_ms_simple(10);

    cmd = BAR30_ADC_READ;
    i2c_write(BAR30_ADDR, &cmd, 1);
    i2c_read(BAR30_ADDR, buf, 3);
    D1 = (buf[0] << 16) | (buf[1] << 8) | buf[2];

    cmd = BAR30_CONV_D2;
    i2c_write(BAR30_ADDR, &cmd, 1);
    delay_ms_simple(10);

    cmd = BAR30_ADC_READ;
    i2c_write(BAR30_ADDR, &cmd, 1);
    i2c_read(BAR30_ADDR, buf, 3);
    D2 = (buf[0] << 16) | (buf[1] << 8) | buf[2];

    dT   = (int32_t)D2 - ((int32_t)prom[5] << 8);
    TEMP = 2000 + ((int64_t)dT * prom[6]) / (1 << 23);
    OFF  = ((int64_t)prom[2] << 16) + ((int64_t)prom[4] * dT) / (1 << 7);
    SENS = ((int64_t)prom[1] << 15) + ((int64_t)prom[3] * dT) / (1 << 8);
    P    = ((int64_t)D1 * SENS / (1 << 21) - OFF) / (1 << 13);

    return (P - 101300.0f) / (1025.0f * 9.80665f);
}
