#include "vn200.h"

#include <string.h>

#define VN200_SYNC          0xFAU
#define VN200_GROUPS        0x01U       /* Common group only           */
#define VN200_FIELDS        0x0128U     /* YawPitchRoll, AngularRate, Accel */

#define VN200_HEADER_LEN    4U          /* sync, group, 2-byte field mask */
#define VN200_PACKET_LEN    42U         /* header + 36 payload + 2 CRC   */

static uint8_t  packet[VN200_PACKET_LEN];
static uint16_t packet_len = 0U;

static uint32_t count_ok  = 0U;
static uint32_t count_bad = 0U;

uint32_t vn200_packets_ok(void)  { return count_ok; }
uint32_t vn200_packets_bad(void) { return count_bad; }

/*
 * CRC-16 exactly as printed in the manual (section 4.4.3).
 *
 * The check (section 5.3.5): run it over everything after the sync byte,
 * including the two CRC bytes. A good packet gives 0. That is easier than
 * extracting the CRC field, and it does not depend on which byte order
 * the CRC is sent in.
 */
static uint16_t vn200_crc16(const uint8_t *data, uint16_t length)
{
    uint16_t crc = 0U;

    for (uint16_t i = 0U; i < length; i++)
    {
        crc  = (uint16_t)((uint8_t)(crc >> 8) | (uint16_t)(crc << 8));
        crc ^= data[i];
        crc ^= (uint8_t)(crc & 0xFFU) >> 4;
        crc ^= (uint16_t)(crc << 12);
        crc ^= (uint16_t)((crc & 0x00FFU) << 5);
    }

    return crc;
}

/* Payload values are little-endian, like the Cortex-M4. */
static float read_float(const uint8_t *p)
{
    float value;

    memcpy(&value, p, sizeof(value));
    return value;
}

/*
 * Drop the first byte of what has been collected and slide forward to the
 * next 0xFA, if there is one. Throwing the whole buffer away instead would
 * skip over a real packet that started somewhere inside a bad one.
 */
static void resync(void)
{
    uint16_t next = 1U;

    while ((next < packet_len) && (packet[next] != VN200_SYNC))
    {
        next++;
    }

    packet_len = (uint16_t)(packet_len - next);
    memmove(packet, &packet[next], packet_len);
}

/* The header is fixed for this configuration, so a mismatch shows up as
 * soon as the first four bytes are in, not 38 bytes later. */
static bool header_ok(void)
{
    uint16_t fields = (uint16_t)(packet[2] | ((uint16_t)packet[3] << 8));

    return (packet[1] == VN200_GROUPS) && (fields == VN200_FIELDS);
}

bool vn200_feed_byte(uint8_t byte, VN200Data *out)
{
    if ((packet_len == 0U) && (byte != VN200_SYNC))
    {
        return false;
    }

    packet[packet_len++] = byte;

    /*
     * Re-check after every resync: the bytes that slid to the front may
     * already be a bad header, or already a complete packet.
     */
    for (;;)
    {
        if ((packet_len >= VN200_HEADER_LEN) && !header_ok())
        {
            count_bad++;
            resync();
            continue;
        }

        if (packet_len < VN200_PACKET_LEN)
        {
            return false;
        }

        if (vn200_crc16(&packet[1], VN200_PACKET_LEN - 1U) != 0U)
        {
            count_bad++;
            resync();
            continue;
        }

        break;
    }

    const uint8_t *p = &packet[VN200_HEADER_LEN];

    out->yaw_deg      = read_float(&p[0]);
    out->pitch_deg    = read_float(&p[4]);
    out->roll_deg     = read_float(&p[8]);

    out->gyro_x_rad_s = read_float(&p[12]);
    out->gyro_y_rad_s = read_float(&p[16]);
    out->gyro_z_rad_s = read_float(&p[20]);

    out->accel_x_m_s2 = read_float(&p[24]);
    out->accel_y_m_s2 = read_float(&p[28]);
    out->accel_z_m_s2 = read_float(&p[32]);

    count_ok++;
    packet_len = 0U;

    return true;
}
