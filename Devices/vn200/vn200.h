#ifndef VN200_H
#define VN200_H

#include <stdint.h>
#include <stdbool.h>

/*
 * VN-200 binary output parser.
 *
 * The sensor is set up once from a PC (VectorNav Control Center, or any
 * serial terminal) and the settings saved to its flash, so this firmware
 * only listens. The commands, from the VN-200 user manual (UM004):
 *
 *   $VNWRG,06,0*XX                 ASCII async output off (section 6.2.7)
 *   $VNWRG,75,1,8,01,0128*XX       binary output 1 on serial port 1,
 *                                  800 Hz / 8 = 100 Hz, group 1 (Common),
 *                                  fields YawPitchRoll + AngularRate + Accel
 *   $VNWNV*57                      save to flash (section 6.1.3)
 *
 * Use port 2 in the second command if the VN-200's serial port 2 is the
 * one wired to the STM32. *XX tells the sensor to skip the checksum, so
 * the lines can be typed by hand.
 *
 * Resulting packet, 42 bytes (manual section 5.3):
 *
 *   0       sync      0xFA
 *   1       group     0x01            Common group only
 *   2-3     fields    0x0128          bit 3 YawPitchRoll, bit 5 AngularRate,
 *                                     bit 8 Accel (little-endian u16)
 *   4-15    yaw, pitch, roll          float, degrees, NED      (5.4.4)
 *   16-27   angular rate x, y, z      float, rad/s, body frame (5.4.6)
 *   28-39   accel x, y, z             float, m/s^2, body frame, includes
 *                                     gravity                  (5.4.9)
 *   40-41   CRC-16
 *
 * About 36% of 115200 baud at 100 Hz.
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
} VN200Data;

/*
 * Feed one received byte. Returns true when this byte completed a valid
 * packet; the reading is then in *out. All parser state is private to the
 * calling task (vn200/imu task), so there is no locking.
 */
bool vn200_feed_byte(uint8_t byte, VN200Data *out);

/* Packets accepted, and packets rejected for a bad CRC or header. */
uint32_t vn200_packets_ok(void);
uint32_t vn200_packets_bad(void);

#endif
