#ifndef DVL_H
#define DVL_H

#include <stdint.h>
#include <stdbool.h>

/*
 * Teledyne Wayfinder DVL, binary data output message.
 *
 * Layout from Teledyne's "Wayfinder Binary Interface Packet Protocol"
 * page (field sizes summed into offsets), little-endian, 116 bytes:
 *
 *   0-5     SOP            AA 10 01 74 00 10
 *   6-14    data ID        05 6D 00 AA 11 69 00 00 00
 *   15      system type    0x4C (76) for a Wayfinder
 *   16-20   sub-type, firmware version
 *   21-28   RTC: year, month, day, hour, minute, second (u8), ms (u16)
 *   29      coordinate system
 *   30-45   bottom-track velocity x, y, z, error   float m/s
 *   46-61   range to bottom, beams 1-4              float m
 *   62      mean range to bottom                    float m
 *   66      speed of sound                          float m/s
 *   70      BT status (u16), 72 BIT flags (u16)
 *   74-85   input voltage, transmit voltage, transmit current (float)
 *   86-111  serial number (6), reserved (20)
 *   112     checksum - data (u16), 114 checksum (u16)
 *
 * Velocities and ranges are NaN when there is no bottom lock (Wayfinder
 * DVL Guide, "Data Screening"). Those frames are still published, with
 * velocity_valid = false, so the Pi can tell "DVL alive, no bottom" from
 * "no DVL".
 */
#define DVL_PACKET_LENGTH 116U

typedef struct
{
    uint32_t timestamp_ms;      /* STM32 tick when the frame arrived */

    bool  velocity_valid;       /* false when any of vx, vy, vz is NaN */
    uint8_t coordinate_system;

    float vx_m_s;
    float vy_m_s;
    float vz_m_s;
    float verr_m_s;

    float range_beam_m[4];
    float mean_range_m;
    float speed_of_sound_m_s;

    uint16_t status;
    uint16_t bit;

    float input_voltage_v;
} DVLData;

/*
 * Feed one received byte. Returns true when this byte completed a frame
 * that passed the checks; the frame is then in *out. All parser state is
 * private to dvl_task, so there is no locking.
 */
bool dvl_feed_byte(uint8_t byte, DVLData *out);

/* Frames accepted, and frames rejected (bad header or checksum). */
uint32_t dvl_frames_ok(void);
uint32_t dvl_frames_bad(void);

/*
 * Which checksum reading matched, see dvl.c: final checksum over bytes
 * 0-113 (includes the data checksum) or over 0-111 (excludes it).
 */
uint32_t dvl_sum_incl_count(void);
uint32_t dvl_sum_excl_count(void);

#endif
