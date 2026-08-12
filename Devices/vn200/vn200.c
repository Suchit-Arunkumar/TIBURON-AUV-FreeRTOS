#include "vn200.h"
#include <string.h>

#define VN200_SYNC              0xFA

#define VN200_GROUP_IMU         0x04
#define VN200_GROUP_ATTITUDE    0x10

#define VN200_GROUPS            0x14

#define VN200_IMU_FIELDS        0x0018
#define VN200_ATT_FIELDS        0x0002

#define VN200_PACKET_LENGTH     44

static uint8_t vn200_packet[VN200_PACKET_LENGTH];
static uint16_t vn200_index = 0;
static VN200Data vn200_latest;

//===========================================================================================================================
static bool vn200_validate_header(const uint8_t *packet)
{
    if (packet[0] != VN200_SYNC)
    {
        return false;
    }

    if (packet[1] != VN200_GROUPS)
    {
        return false;
    }

    uint16_t imu_fields =
        ((uint16_t)packet[2]) |
        ((uint16_t)packet[3] << 8);

    if (imu_fields != VN200_IMU_FIELDS)
    {
        return false;
    }

    uint16_t attitude_fields =
        ((uint16_t)packet[4]) |
        ((uint16_t)packet[5] << 8);

    if (attitude_fields != VN200_ATT_FIELDS)
    {
        return false;
    }

    return true;
}

//===========================================================================================================================
static uint16_t vn200_read_u16(const uint8_t *p)
{
    return ((uint16_t)p[0]) |
           ((uint16_t)p[1] << 8);
}

//===========================================================================================================================
static float vn200_read_float(const uint8_t *p)
{
    uint32_t raw =
        ((uint32_t)p[0]) |
        ((uint32_t)p[1] << 8) |
        ((uint32_t)p[2] << 16) |
        ((uint32_t)p[3] << 24);

    float value;

    memcpy(&value, &raw, sizeof(value));

    return value;
}

//===========================================================================================================================
static uint16_t vn200_crc16(const uint8_t *data, uint16_t length)
{
    uint16_t crc = 0;

    for (uint16_t i = 0; i < length; i++)
    {
        crc = (uint8_t)(crc >> 8) | (crc << 8);
        crc ^= data[i];
        crc ^= (uint8_t)(crc & 0xFF) >> 4;
        crc ^= crc << 12;
        crc ^= (crc & 0x00FF) << 5;
    }

    return crc;
}

//===========================================================================================================================
bool vn200_parse(const uint8_t *packet,
                 uint16_t length,
                 VN200Data *out)
{
    if (packet == NULL || out == NULL)
    {
        return false;
    }

    if (length != VN200_PACKET_LENGTH)
    {
        return false;
    }

    if (!vn200_validate_header(packet))
    {
        return false;
    }

    uint16_t calculated_crc =
        vn200_crc16(&packet[1], length - 3U);

    uint16_t received_crc =
        vn200_read_u16(&packet[length - 2U]);

    if (calculated_crc != received_crc)
    {
        return false;
    }

    out->accel_x = vn200_read_float(&packet[6]);
    out->accel_y = vn200_read_float(&packet[10]);
    out->accel_z = vn200_read_float(&packet[14]);

    out->gyro_x = vn200_read_float(&packet[18]);
    out->gyro_y = vn200_read_float(&packet[22]);
    out->gyro_z = vn200_read_float(&packet[26]);

    out->yaw = vn200_read_float(&packet[30]);
    out->pitch = vn200_read_float(&packet[34]);
    out->roll = vn200_read_float(&packet[38]);

    out->valid = true;

    return true;
}

//===========================================================================================================================
void vn200_feed_byte(uint8_t byte)
{
    if (vn200_index == 0)
    {
        if (byte != VN200_SYNC)
        {
            return;
        }
    }

    vn200_packet[vn200_index++] = byte;

    if (vn200_index >= VN200_PACKET_LENGTH)
    {
        if (vn200_parse(vn200_packet,
                        VN200_PACKET_LENGTH,
                        &vn200_latest))
        {
            /* Packet successfully parsed */
        }

        vn200_index = 0;
    }
}

//===========================================================================================================================
bool vn200_get_data(VN200Data *out)
{
    if (out == NULL)
    {
        return false;
    }

    if (!vn200_latest.valid)
    {
        return false;
    }

    *out = vn200_latest;

    return true;
}
