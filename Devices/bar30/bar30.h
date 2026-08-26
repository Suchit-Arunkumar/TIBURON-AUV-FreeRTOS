#ifndef BAR30_H
#define BAR30_H

#include <stm32f446xx.h>
#include <stdint.h>

#include "i2c.h"

/*
 * MS5837-30BA depth sensor.
 *
 * Both entry points now report I2C status rather than returning a value
 * that silently means "0 m" when nothing is on the bus. With no sensor
 * attached - the normal bench state - bar30_init() returns
 * I2C_ERR_ADDR_NACK immediately instead of the driver hanging forever in
 * an unbounded flag poll.
 */

/* Reads the factory PROM calibration. Must succeed before bar30_read()
 * produces anything meaningful. */
I2C_Status bar30_init(void);

/*
 * One pressure + temperature conversion, converted to depth in metres.
 * Writes *out_depth_m only on I2C_OK; leaves it untouched otherwise, so
 * a failed read cannot inject a bogus 0 m into the filter.
 */
I2C_Status bar30_read(float *out_depth_m);

#endif
