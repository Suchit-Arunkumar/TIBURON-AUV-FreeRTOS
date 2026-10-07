#ifndef BNO085_H
#define BNO085_H

#include <stdint.h>
#include <stdbool.h>

/*
 * BNO085 on I2C1, no INT or RST pins, so it's polled. CEVA's SH-2 library
 * (Middlewares/Third_Party/sh2) does the protocol; this file gives it
 * open/read/write/time and turns its events into one reading.
 *
 * Reports at 50 Hz: game rotation vector (no magnetometer, which is
 * unreliable near thruster motors; yaw starts at an arbitrary zero),
 * calibrated gyro (rad/s), accelerometer (m/s^2, with gravity).
 *
 * Axis signs not yet checked against the VN-200. UNTESTED on hardware.
 */

typedef struct
{
    float yaw_deg;
    float pitch_deg;
    float roll_deg;

    float gyro_x_rad_s;
    float gyro_y_rad_s;
    float gyro_z_rad_s;

    float accel_x_m_s2;
    float accel_y_m_s2;
    float accel_z_m_s2;

    uint8_t accuracy;       /* 0 unreliable .. 3 high, from the report status */
} Bno085Data;

/* Find the sensor (0x4A or 0x4B), reset it, enable the reports. Blocks
 * for about half a second when a sensor answers. False if it failed;
 * safe to call again. */
bool bno085_open(void);

/* Poll the sensor. True when a new orientation arrived (with the latest
 * gyro and accel in *out). Re-enables the reports if the sensor reset. */
bool bno085_service(Bno085Data *out);

/* True while a session is open. */
bool bno085_is_open(void);

/* Close the session, e.g. after the sensor stopped answering. */
void bno085_close(void);

/* Times the sensor reported a reset, and events that failed to decode. */
uint32_t bno085_resets(void);
uint32_t bno085_decode_errors(void);

#endif
