#include "bar30.h"
#include "ms5837_math.h"
#include "i2c.h"

#include "FreeRTOS.h"
#include "task.h"

/* Commands, datasheet page 7. */
#define BAR30_ADDR      0x76U
#define BAR30_RESET     0x1EU
#define BAR30_PROM      0xA0U       /* + 2 x word index, words 0..6 */
#define BAR30_CONV_D1   0x48U       /* pressure,    OSR 4096 */
#define BAR30_CONV_D2   0x58U       /* temperature, OSR 4096 */
#define BAR30_ADC_READ  0x00U

/*
 * Conversion wait. OSR 4096 takes up to 9.04 ms (datasheet page 2, and
 * note 4 there: use the maximum). vTaskDelay(n) can return after only
 * n - 1 full ticks, because the first tick may be almost over when it is
 * called, so 10 could be as little as 9 ms: too short, and an early ADC
 * read returns 0 (page 8), which shows up as a negative depth spike.
 * 12 guarantees at least 11 ms.
 */
#define BAR30_CONV_MS   12U

/*
 * Surface pressure is taken once at init. A reading outside this band is
 * not plausible air pressure - most likely the board rebooted underwater -
 * so the standard atmosphere is used instead of reading 0 m at depth.
 */
#define SURFACE_MIN_PA      95000.0f
#define SURFACE_MAX_PA     105000.0f
#define SURFACE_DEFAULT_PA 101325.0f

#define GRAVITY_M_S2        9.80665f

static uint16_t   prom[MS5837_PROM_WORDS];
static int        initialised = 0;
static float      surface_pa  = SURFACE_DEFAULT_PA;
static I2C_Status last_i2c    = I2C_OK;

const char *bar30_status_str(Bar30Status s)
{
    switch (s)
    {
        case BAR30_OK:           return "ok";
        case BAR30_ERR_I2C:      return i2c_status_str(last_i2c);
        case BAR30_ERR_PROM_CRC: return "PROM CRC mismatch";
        case BAR30_ERR_NOT_INIT: return "not initialised";
        default:                 return "?";
    }
}

I2C_Status bar30_last_i2c(void)
{
    return last_i2c;
}

float bar30_surface_pa(void)
{
    return surface_pa;
}

/* Send one command byte. */
static int command(uint8_t cmd)
{
    last_i2c = i2c_write(BAR30_ADDR, &cmd, 1U);
    return last_i2c == I2C_OK;
}

/* Start a conversion, wait it out, read the 24-bit result. */
static int convert(uint8_t conv_cmd, uint32_t *out)
{
    uint8_t buf[3];

    if (!command(conv_cmd))
    {
        return 0;
    }

    vTaskDelay(pdMS_TO_TICKS(BAR30_CONV_MS));

    if (!command(BAR30_ADC_READ))
    {
        return 0;
    }

    last_i2c = i2c_read(BAR30_ADDR, buf, 3U);
    if (last_i2c != I2C_OK)
    {
        return 0;
    }

    *out = ((uint32_t)buf[0] << 16) | ((uint32_t)buf[1] << 8) | buf[2];
    return 1;
}

static Bar30Status read_raw(float *pressure_pa, float *temperature_c)
{
    uint32_t d1, d2;

    if (!convert(BAR30_CONV_D1, &d1) || !convert(BAR30_CONV_D2, &d2))
    {
        return BAR30_ERR_I2C;
    }

    int32_t p, t;

    ms5837_compensate(prom, d1, d2, 1, &p, &t);

    *pressure_pa   = (float)p * 10.0f;          /* 0.1 mbar = 10 Pa */
    *temperature_c = (float)t / 100.0f;
    return BAR30_OK;
}

Bar30Status bar30_init(void)
{
    initialised = 0;

    if (!command(BAR30_RESET))
    {
        return BAR30_ERR_I2C;
    }

    /* Reset reloads the PROM into the sensor's registers. This datasheet
     * gives no time for it; 10 ms is a generous allowance. */
    vTaskDelay(pdMS_TO_TICKS(10));

    for (int i = 0; i < MS5837_PROM_WORDS; i++)
    {
        uint8_t buf[2];

        if (!command((uint8_t)(BAR30_PROM + 2 * i)))
        {
            return BAR30_ERR_I2C;
        }

        last_i2c = i2c_read(BAR30_ADDR, buf, 2U);
        if (last_i2c != I2C_OK)
        {
            return BAR30_ERR_I2C;
        }

        prom[i] = (uint16_t)((buf[0] << 8) | buf[1]);
    }

    /* Without this, a bit flipped on the wire becomes a permanent scale
     * error in every depth reading. */
    if (!ms5837_prom_crc_ok(prom))
    {
        return BAR30_ERR_PROM_CRC;
    }

    float p, t;
    Bar30Status st = read_raw(&p, &t);

    if (st != BAR30_OK)
    {
        return st;
    }

    surface_pa  = ((p >= SURFACE_MIN_PA) && (p <= SURFACE_MAX_PA)) ? p : SURFACE_DEFAULT_PA;
    initialised = 1;

    return BAR30_OK;
}

Bar30Status bar30_read(Bar30Data *out)
{
    if (!initialised)
    {
        return BAR30_ERR_NOT_INIT;
    }

    float p, t;
    Bar30Status st = read_raw(&p, &t);

    if (st != BAR30_OK)
    {
        return st;
    }

    out->pressure_pa   = p;
    out->temperature_c = t;
    out->depth_m       = (p - surface_pa) / (BAR30_WATER_DENSITY_KG_M3 * GRAVITY_M_S2);

    return BAR30_OK;
}
