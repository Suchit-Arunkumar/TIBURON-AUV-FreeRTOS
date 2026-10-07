#ifndef ADC_DEPTH_H
#define ADC_DEPTH_H

#include <stdint.h>
#include <stdbool.h>

/*
 * Analog depth: DFRobot Gravity water pressure sensor (SEN0257) on PA4,
 * ADC1 channel 4, through a 2:1 resistor divider.
 *
 * From the DFRobot product page (docs/datasheets):
 *   range 0-1 MPa, output 0.5-4.5 V linear, 5 V supply
 *   -> 1 MPa / 4 V = 250 kPa per volt, zero at 0.5 V (nominal)
 *   accuracy 0.5-1 % of full scale = 5-10 kPa, about 0.5-1 m of water
 *
 * That makes it a backup for the Bar30, not a replacement. (The Pico test
 * sketch multiplied by 400 kPa/V; for this 1 MPa part that reads 1.6x too
 * high.)
 *
 * The 4.5 V maximum must never reach the pin directly: through the 2:1
 * divider it is 2.25 V.
 *
 * Only depth_task uses ADC1, so there is no locking.
 *
 * UNTESTED on hardware; the conversion maths is checked on the PC
 * (tests/host/test_adc_depth.c).
 */

#define ADC_DEPTH_DIVIDER          2.0f      /* sensor volts per pin volt  */
#define ADC_DEPTH_KPA_PER_V        250.0f
#define ADC_DEPTH_ZERO_V           0.5f      /* output at 0 kPa, nominal   */
#define ADC_DEPTH_WATER_KG_M3      997.0f    /* fresh water                */

typedef struct
{
    float sensor_v;         /* at the sensor, divider undone */
    float pressure_pa;      /* gauge: 0 at the surface        */
    float depth_m;          /* filtered                       */
} AdcDepthData;

/* Gauge pressure in Pa from the sensor's output voltage. */
static inline float adc_depth_pressure_pa(float sensor_v, float zero_v)
{
    return (sensor_v - zero_v) * ADC_DEPTH_KPA_PER_V * 1000.0f;
}

/* Depth in m from gauge pressure in Pa. */
static inline float adc_depth_from_pa(float pressure_pa)
{
    return pressure_pa / (ADC_DEPTH_WATER_KG_M3 * 9.80665f);
}

/*
 * Configure PA4 and ADC1, then measure the zero. The zero is only taken
 * from the sensor if it reads within 0.45-0.55 V; anything else means
 * the board started under water (or the sensor is odd) and the nominal
 * 0.5 V is used instead.
 */
void adc_depth_init(void);

/*
 * One filtered reading. Returns false if the sensor looks disconnected:
 * its output never drops below 0.5 V when powered, so a reading under
 * 0.3 V means nothing is driving the pin.
 */
bool adc_depth_read(AdcDepthData *out);

/* The zero voltage in use. */
float adc_depth_zero_v(void);

#endif
