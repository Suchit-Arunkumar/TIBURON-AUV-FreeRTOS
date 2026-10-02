#ifndef SENSOR_STATUS_H
#define SENSOR_STATUS_H

#include <stdint.h>

/*
 * Sensor presence.
 *
 * None of the three sensors is attached to the bench rig - the VN-200,
 * the Wayfinder DVL and the Bar30 all live on the vehicle. Before this,
 * that produced two different silent failures:
 *
 *   - vn200_task and dvl_task blocked on ulTaskNotifyTake(portMAX_DELAY),
 *     so with no UART traffic they simply never ran again. Harmless in
 *     itself, but indistinguishable from a task that had crashed.
 *   - bar30_task called into an I2C driver whose every wait was
 *     unbounded, so with no device on the bus it spun forever at
 *     priority 4, permanently starving logging, the SPI owner and the
 *     console.
 *
 * Each task now degrades to SENSOR_ABSENT: it keeps running, publishes
 * nothing, and says so once on the console.
 *
 * The distinction that matters for the bench is PRESENT vs ABSENT vs
 * FAULTED: absent means nothing ever answered, faulted means something
 * answered and then stopped. They call for different things - check the
 * wiring, versus check the sensor.
 */
typedef enum
{
    SENSOR_INIT    = 0,   /* not yet determined                        */
    SENSOR_OK      = 1,   /* producing fresh data                      */
    SENSOR_ABSENT  = 2,   /* never responded since boot                */
    SENSOR_FAULTED = 3    /* responded once, then stopped or errored   */
} SensorState;

const char *sensor_state_str(SensorState s);

/*
 * Published by each sensor task for the console health report. Plain
 * enums written by one task and read by another: a torn read is not
 * possible for a single aligned word on Cortex-M, and a stale read is
 * harmless for a status display.
 */
extern volatile SensorState g_vn200_state;
extern volatile SensorState g_dvl_state;
extern volatile SensorState g_bar30_state;

#endif /* SENSOR_STATUS_H */
