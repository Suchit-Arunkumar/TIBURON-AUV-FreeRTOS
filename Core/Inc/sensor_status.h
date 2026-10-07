#ifndef SENSOR_STATUS_H
#define SENSOR_STATUS_H

#include <stdint.h>

/*
 * Sensor state for the console. A missing sensor doesn't stop its task:
 * it keeps running, publishes nothing, and reports ABSENT (never answered:
 * check the wiring) or FAULTED (answered, then stopped: check the sensor).
 */
typedef enum
{
    SENSOR_INIT    = 0,   /* not yet determined                        */
    SENSOR_OK      = 1,   /* producing fresh data                      */
    SENSOR_ABSENT  = 2,   /* never responded since boot                */
    SENSOR_FAULTED = 3    /* responded once, then stopped or errored   */
} SensorState;

const char *sensor_state_str(SensorState s);

// Written by each sensor task, read by the health report. Single words,
// so no lock needed.
extern volatile SensorState g_vn200_state;
extern volatile SensorState g_bno085_state;
extern volatile SensorState g_dvl_state;
extern volatile SensorState g_bar30_state;
extern volatile SensorState g_adc_depth_state;

#endif /* SENSOR_STATUS_H */
