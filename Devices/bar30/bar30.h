#ifndef BAR30_H
#define BAR30_H

#include <stdint.h>

#include "i2c.h"

/*
 * Blue Robotics Bar30: an MS5837-30BA on I2C address 0x76.
 *
 * Only called from depth_task. It waits for each
 * conversion with vTaskDelay, so the I2C bus is free for the BNO085 in
 * between.
 */

/* Fresh water. 1025 for sea water. */
#define BAR30_WATER_DENSITY_KG_M3   997.0f

typedef struct
{
    float pressure_pa;      /* absolute */
    float temperature_c;
    float depth_m;          /* below the surface pressure taken at init */
} Bar30Data;

typedef enum
{
    BAR30_OK = 0,
    BAR30_ERR_I2C,          /* see bar30_last_i2c() for which stage */
    BAR30_ERR_PROM_CRC,     /* calibration words read back corrupted */
    BAR30_ERR_NOT_INIT      /* bar30_read() before a successful init */
} Bar30Status;

const char *bar30_status_str(Bar30Status s);

/* The I2C error behind the last BAR30_ERR_I2C. */
I2C_Status bar30_last_i2c(void);

/*
 * Reset, read and CRC-check the calibration PROM, then take one reading
 * as the surface pressure. Takes about 35 ms. Can be called again at any
 * time, so a sensor plugged in after boot is picked up.
 */
Bar30Status bar30_init(void);

/* One pressure + temperature conversion, about 25 ms. *out is written
 * only on BAR30_OK, so a failed read never publishes a made-up value. */
Bar30Status bar30_read(Bar30Data *out);

/* Surface pressure in Pa that depth is measured from. */
float bar30_surface_pa(void);

#endif
