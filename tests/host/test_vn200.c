/*
 * Host test for the VN-200 parser (Devices/vn200/vn200.c).
 *
 * Builds packets the way the VN-200 user manual describes them (section
 * 5.3: sync, group, field mask, payload, CRC that makes the running CRC
 * over everything after the sync byte come out to zero) and checks that
 * the parser accepts good ones, rejects corrupted ones, and finds its way
 * back into sync after garbage.
 */
#include "test.h"
#include "vn200.h"

#include <string.h>

/* Same algorithm as the manual (section 4.4.3), written independently. */
static uint16_t crc_manual(const uint8_t *d, int n)
{
    uint16_t crc = 0;

    for (int i = 0; i < n; i++)
    {
        crc  = (uint16_t)((uint8_t)(crc >> 8) | (crc << 8));
        crc ^= d[i];
        crc ^= (uint8_t)(crc & 0xff) >> 4;
        crc ^= (uint16_t)(crc << 12);
        crc ^= (uint16_t)((crc & 0x00ff) << 5);
    }

    return crc;
}

static void put_float(uint8_t *p, float f)
{
    memcpy(p, &f, 4);
}

/* 42-byte packet: group 0x01, fields 0x0128, nine floats, CRC. */
static void make_packet(uint8_t *pkt, const float v[9])
{
    pkt[0] = 0xFA;
    pkt[1] = 0x01;
    pkt[2] = 0x28;
    pkt[3] = 0x01;

    for (int i = 0; i < 9; i++)
    {
        put_float(&pkt[4 + 4 * i], v[i]);
    }

    /* The CRC goes out high byte first, which is what makes the check
     * over the whole packet return zero. */
    uint16_t crc = crc_manual(&pkt[1], 39);
    pkt[40] = (uint8_t)(crc >> 8);
    pkt[41] = (uint8_t)(crc & 0xFF);
}

static int feed(const uint8_t *bytes, int n, VN200Data *out)
{
    int got = 0;

    for (int i = 0; i < n; i++)
    {
        if (vn200_feed_byte(bytes[i], out))
        {
            got++;
        }
    }

    return got;
}

int main(void)
{
    const float v[9] = { 123.5f, -4.25f, 2.0f, 0.1f, -0.2f, 0.3f, 0.5f, -0.5f, -9.81f };
    uint8_t pkt[42];
    VN200Data d;

    make_packet(pkt, v);

    /* The CRC rule from the manual holds for the packet we built. */
    CHECK(crc_manual(&pkt[1], 41) == 0);

    /* A good packet is accepted and every field lands in the right place. */
    memset(&d, 0, sizeof(d));
    CHECK(feed(pkt, 42, &d) == 1);
    CHECK_NEAR(d.yaw_deg,      123.5f, 1e-6);
    CHECK_NEAR(d.pitch_deg,    -4.25f, 1e-6);
    CHECK_NEAR(d.roll_deg,      2.0f,  1e-6);
    CHECK_NEAR(d.gyro_x_rad_s,  0.1f,  1e-6);
    CHECK_NEAR(d.gyro_z_rad_s,  0.3f,  1e-6);
    CHECK_NEAR(d.accel_z_m_s2, -9.81f, 1e-6);

    /* One flipped bit anywhere after the sync byte is rejected. */
    uint32_t bad_before = vn200_packets_bad();
    for (int i = 1; i < 42; i++)
    {
        uint8_t c[42];
        memcpy(c, pkt, 42);
        c[i] ^= 0x10;
        CHECK(feed(c, 42, &d) == 0);
    }
    CHECK(vn200_packets_bad() > bad_before);

    /* After all that garbage, a good packet is found again. */
    CHECK(feed(pkt, 42, &d) == 1);

    /* Junk containing 0xFA before a packet does not hide it. */
    uint8_t stream[200];
    int n = 0;
    const uint8_t junk[] = { 0x00, 0xFA, 0x13, 0xFA, 0xFA, 0x01, 0x28, 0x99, 0x55 };
    memcpy(&stream[n], junk, sizeof(junk)); n += (int)sizeof(junk);
    memcpy(&stream[n], pkt, 42);            n += 42;
    memcpy(&stream[n], pkt, 42);            n += 42;
    CHECK(feed(stream, n, &d) == 2);

    /* The old field mask (0x0018 in the IMU group) is not accepted: that
     * mask means uncompensated gyro + temperature, not accel + gyro. */
    uint8_t old[42];
    memcpy(old, pkt, 42);
    old[1] = 0x14; old[2] = 0x18; old[3] = 0x00;
    CHECK(feed(old, 42, &d) == 0);

    return test_summary("vn200");
}
