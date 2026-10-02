#include "bar30.h"
#include "i2c.h"

#include "FreeRTOS.h"
#include "task.h"

#define BAR30_ADDR     0x76
#define BAR30_RESET    0x1E
#define BAR30_PROM     0xA0
#define BAR30_CONV_D1  0x48
#define BAR30_CONV_D2  0x58
#define BAR30_ADC_READ 0x00

static uint16_t prom[8];

/* Only called from bar30_task, so a real RTOS delay is safe here. The old
 * NOP loop gave ~2 ms instead of 10 at 180 MHz - shorter than the 9 ms
 * OSR-4096 conversion, so the ADC read returned 0. */
static void delay_ms_simple(uint32_t ms)
{
    vTaskDelay(pdMS_TO_TICKS(ms));
}

I2C_Status bar30_init(void)
{
    uint8_t cmd;
    uint8_t buf[2];
    I2C_Status st;

    cmd = BAR30_RESET;
    st = i2c_write(BAR30_ADDR , &cmd, 1);

    /* Nothing on the bus: fail out now rather than reading 8 PROM words
     * of garbage and computing depth from it. */
    if (st != I2C_OK)
    {
        return st;
    }

    delay_ms_simple(10);

    for(int i = 0; i < 8; i++){
        cmd = BAR30_PROM + (i*2);

        st = i2c_write(BAR30_ADDR , &cmd, 1);
        if (st != I2C_OK) { return st; }

        st = i2c_read(BAR30_ADDR , buf, 2);
        if (st != I2C_OK) { return st; }

        prom[i] = (buf[0] << 8) | buf[1];
    }

    return I2C_OK;
}

I2C_Status bar30_read(float *out_depth_m)
{
    uint8_t cmd;
    I2C_Status st;
    uint8_t buf[3];
    uint32_t D1, D2;
    int32_t dT, TEMP;
    int64_t OFF, SENS, P;

    cmd = BAR30_CONV_D1;
    st = i2c_write(BAR30_ADDR, &cmd, 1);
    if (st != I2C_OK) { return st; }
    delay_ms_simple(10);

    cmd = BAR30_ADC_READ;
    st = i2c_write(BAR30_ADDR, &cmd, 1);
    if (st != I2C_OK) { return st; }
    st = i2c_read(BAR30_ADDR, buf, 3);
    if (st != I2C_OK) { return st; }
    D1 = (buf[0] << 16) | (buf[1] << 8) | buf[2];

    cmd = BAR30_CONV_D2;
    st = i2c_write(BAR30_ADDR, &cmd, 1);
    if (st != I2C_OK) { return st; }
    delay_ms_simple(10);

    cmd = BAR30_ADC_READ;
    st = i2c_write(BAR30_ADDR, &cmd, 1);
    if (st != I2C_OK) { return st; }
    st = i2c_read(BAR30_ADDR, buf, 3);
    if (st != I2C_OK) { return st; }
    D2 = (buf[0] << 16) | (buf[1] << 8) | buf[2];

    dT   = (int32_t)D2 - ((int32_t)prom[5] << 8);
    TEMP = 2000 + ((int64_t)dT * prom[6]) / (1 << 23);
    OFF  = ((int64_t)prom[2] << 16) + ((int64_t)prom[4] * dT) / (1 << 7);
    SENS = ((int64_t)prom[1] << 15) + ((int64_t)prom[3] * dT) / (1 << 8);
    P    = ((int64_t)D1 * SENS / (1 << 21) - OFF) / (1 << 13);

    /*
     * TEMP is part of the MS5837 first-order compensation sequence and is
     * kept so this reads as the datasheet does, but this driver returns
     * depth only. It would be needed for a temperature reading or for the
     * second-order compensation, neither of which is implemented.
     */
    (void)TEMP;

    /* Only now, with every transfer confirmed, publish a value.
     * MS5837-30BA reports P in 0.1 mbar (10 Pa) units, hence the x10. */
    *out_depth_m = ((float)P * 10.0f - 101300.0f) / (1025.0f * 9.80665f);

    return I2C_OK;
}
