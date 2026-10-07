#ifndef BNO085_H
#define BNO085_H

#include <stdint.h>
#include <stdbool.h>

/*
 * BNO085 on I2C1 (shared with the Bar30), no INT or RST pins.
 *
 * The protocol work (SHTP framing, SH-2 commands) is CEVA's library in
 * Middlewares/Third_Party/sh2. This file supplies the four functions that
 * library needs to reach the hardware (open, read, write, time) and turns
 * its sensor events into one reading.
 *
 * Without the INT pin the sensor is polled: bno085_service() reads the
 * 4-byte SHTP header, and a whole packet only if the header says one is
 * waiting.
 *
 * Reports enabled, all at 50 Hz (same as the Pico setup minus the extras):
 *   game rotation vector   accel + gyro fusion, no magnetometer, which is
 *                          the right choice near thruster motors; yaw
 *                          starts at an arbitrary zero and drifts slowly
 *   calibrated gyroscope   rad/s
 *   accelerometer          m/s^2, includes gravity
 *
 * Axis convention: not yet compared against the VN-200 (which reports NED).
 * Do the tilt test from the hardware checklist before relying on signs
 * when switching between the two. UNTESTED on hardware.
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

/*
 * Find the sensor (0x4A, then 0x4B), soft-reset it, open the SH-2
 * session and enable the reports. Blocks for roughly half a second, so
 * call it from the IMU task, never from the control path. Returns false
 * if no sensor answered or any step failed; safe to call again later.
 */
bool bno085_open(void);

/*
 * Poll the sensor and handle whatever it sent. Returns true when a new
 * orientation arrived; *out then has it, together with the latest gyro
 * and accel. If the sensor reset itself, the reports are re-enabled here.
 */
bool bno085_service(Bno085Data *out);

/* True while a session is open. */
bool bno085_is_open(void);

/* Close the session, e.g. after the sensor stopped answering. */
void bno085_close(void);

/* Times the sensor reported a reset, and events that failed to decode. */
uint32_t bno085_resets(void);
uint32_t bno085_decode_errors(void);

#endif
