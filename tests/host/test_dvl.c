/*
 * Host test for the Wayfinder parser (Devices/dvl/dvl.c).
 *
 * Frames are built from the field list on Teledyne's "Wayfinder Binary
 * Interface Packet Protocol" page. This checks the parser against that
 * layout, not against a real DVL: offsets are only confirmed once a
 * real frame has gone through it.
 */
#include "test.h"
#include "dvl.h"
#include "FreeRTOS.h"

#include <string.h>

TickType_t host_tick = 0;

static void put_f(uint8_t *f, int off, float v) { memcpy(&f[off], &v, 4); }

/* include_data_sum: which checksum reading to build the frame with. */
static void make_frame(uint8_t *f, float vx, float vy, float vz, int include_data_sum)
{
    static const uint8_t head[15] = { 0xAA, 0x10, 0x01, 0x74, 0x00, 0x10,
                                      0x05, 0x6D, 0x00, 0xAA, 0x11, 0x69, 0x00, 0x00, 0x00 };
    memset(f, 0, 116);
    memcpy(f, head, 15);
    f[15] = 0x4C;                       /* system type: Wayfinder */
    f[29] = 2;                          /* coordinate system      */
    put_f(f, 30, vx);
    put_f(f, 34, vy);
    put_f(f, 38, vz);
    put_f(f, 42, 0.01f);                /* error velocity         */
    for (int b = 0; b < 4; b++) put_f(f, 46 + 4 * b, 2.0f + (float)b);
    put_f(f, 62, 3.5f);                 /* mean range             */
    put_f(f, 66, 1500.0f);              /* speed of sound         */
    f[70] = 0x34; f[71] = 0x12;         /* BT status 0x1234       */
    put_f(f, 74, 24.0f);                /* input voltage          */

    uint16_t data_sum = 0;
    for (int i = 15; i < 112; i++) data_sum = (uint16_t)(data_sum + f[i]);
    f[112] = (uint8_t)data_sum; f[113] = (uint8_t)(data_sum >> 8);

    uint16_t sum = 0;
    int end = include_data_sum ? 114 : 112;
    for (int i = 0; i < end; i++) sum = (uint16_t)(sum + f[i]);
    f[114] = (uint8_t)sum; f[115] = (uint8_t)(sum >> 8);
}

static int feed(const uint8_t *b, int n, DVLData *out)
{
    int got = 0;
    for (int i = 0; i < n; i++) if (dvl_feed_byte(b[i], out)) got++;
    return got;
}

int main(void)
{
    uint8_t f[116];
    DVLData d;

    /* Good frame, bottom lock. */
    make_frame(f, 0.25f, -0.5f, 0.05f, 1);
    CHECK(feed(f, 116, &d) == 1);
    CHECK(d.velocity_valid);
    CHECK_NEAR(d.vx_m_s, 0.25f, 1e-6);
    CHECK_NEAR(d.vy_m_s, -0.5f, 1e-6);
    CHECK_NEAR(d.vz_m_s, 0.05f, 1e-6);
    CHECK_NEAR(d.range_beam_m[3], 5.0f, 1e-6);
    CHECK_NEAR(d.mean_range_m, 3.5f, 1e-6);
    CHECK_NEAR(d.speed_of_sound_m_s, 1500.0f, 1e-3);
    CHECK(d.status == 0x1234);
    CHECK(d.coordinate_system == 2);
    CHECK_NEAR(d.input_voltage_v, 24.0f, 1e-6);

    /* No bottom lock: NaN velocity is published, flagged invalid. */
    make_frame(f, NAN, NAN, NAN, 1);
    CHECK(feed(f, 116, &d) == 1);
    CHECK(!d.velocity_valid);

    /* Either checksum reading is accepted, and counted separately. */
    uint32_t excl = dvl_sum_excl_count();
    make_frame(f, 0.1f, 0.1f, 0.1f, 0);
    CHECK(feed(f, 116, &d) == 1);
    CHECK(dvl_sum_excl_count() == excl + 1);

    /* A corrupted byte is rejected. */
    make_frame(f, 0.1f, 0.2f, 0.3f, 1);
    f[50] ^= 0x01;
    CHECK(feed(f, 116, &d) == 0);

    /* Junk with 0xAA in it before two good frames: both are found. */
    uint8_t s[400];
    int n = 0;
    const uint8_t junk[] = { 0xAA, 0x10, 0x01, 0x00, 0xAA, 0x55, 0x12 };
    memcpy(&s[n], junk, sizeof(junk)); n += (int)sizeof(junk);
    make_frame(&s[n], 1.0f, 0.0f, 0.0f, 1); n += 116;
    make_frame(&s[n], 2.0f, 0.0f, 0.0f, 1); n += 116;
    CHECK(feed(s, n, &d) == 2);
    CHECK_NEAR(d.vx_m_s, 2.0f, 1e-6);

    return test_summary("dvl");
}
