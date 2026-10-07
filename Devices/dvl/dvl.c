#include "dvl.h"

#include <string.h>
#include <math.h>

#include "FreeRTOS.h"
#include "task.h"

#define DVL_SOP_LENGTH       6U
#define DVL_DATA_ID_LENGTH   9U
#define DVL_SYSTEM_TYPE      0x4CU

/* Offsets: see the table in dvl.h. */
#define OFF_SYSTEM_TYPE      15U
#define OFF_COORD_SYSTEM     29U
#define OFF_VEL_X            30U
#define OFF_VEL_Y            34U
#define OFF_VEL_Z            38U
#define OFF_VEL_ERR          42U
#define OFF_RANGE1           46U
#define OFF_MEAN_RANGE       62U
#define OFF_SPEED_OF_SOUND   66U
#define OFF_STATUS           70U
#define OFF_BIT              72U
#define OFF_INPUT_VOLTAGE    74U
#define OFF_CHECKSUM_DATA    112U
#define OFF_CHECKSUM         114U

static const uint8_t sop[DVL_SOP_LENGTH] =
    { 0xAA, 0x10, 0x01, 0x74, 0x00, 0x10 };

static const uint8_t data_id[DVL_DATA_ID_LENGTH] =
    { 0x05, 0x6D, 0x00, 0xAA, 0x11, 0x69, 0x00, 0x00, 0x00 };

static uint8_t  frame[DVL_PACKET_LENGTH];
static uint16_t frame_len = 0U;

static uint32_t count_ok   = 0U;
static uint32_t count_bad  = 0U;
static uint32_t count_incl = 0U;
static uint32_t count_excl = 0U;

uint32_t dvl_frames_ok(void)      { return count_ok; }
uint32_t dvl_frames_bad(void)     { return count_bad; }
uint32_t dvl_sum_incl_count(void) { return count_incl; }
uint32_t dvl_sum_excl_count(void) { return count_excl; }

static uint16_t read_u16(const uint8_t *p)
{
    return (uint16_t)(p[0] | ((uint16_t)p[1] << 8));
}

static float read_float(const uint8_t *p)
{
    float v;

    memcpy(&v, p, sizeof(v));
    return v;
}

/* Sum of bytes, ignoring rollover. */
static uint16_t byte_sum(const uint8_t *p, uint16_t n)
{
    uint16_t sum = 0U;

    for (uint16_t i = 0U; i < n; i++)
    {
        sum = (uint16_t)(sum + p[i]);
    }

    return sum;
}

/*
 * The final checksum is "the sum of all the non-checksum bytes in the
 * message, ignoring rollover". Teledyne's page does not say whether the
 * data checksum at 112-113 counts as a "checksum byte", so both readings
 * are accepted and counted. The first bench run with a real DVL shows
 * which one the firmware uses; then the other can be removed.
 * UNTESTED: no real Wayfinder frame has been through this parser yet.
 */
static bool checksum_ok(void)
{
    uint16_t received = read_u16(&frame[OFF_CHECKSUM]);

    if (byte_sum(frame, OFF_CHECKSUM) == received)
    {
        count_incl++;
        return true;
    }

    if (byte_sum(frame, OFF_CHECKSUM_DATA) == received)
    {
        count_excl++;
        return true;
    }

    return false;
}

/* Bytes collected so far still match the fixed header. */
static bool header_prefix_ok(void)
{
    for (uint16_t i = 0U; (i < frame_len) && (i < DVL_SOP_LENGTH + DVL_DATA_ID_LENGTH); i++)
    {
        uint8_t want = (i < DVL_SOP_LENGTH) ? sop[i] : data_id[i - DVL_SOP_LENGTH];

        if (frame[i] != want)
        {
            return false;
        }
    }

    return (frame_len <= OFF_SYSTEM_TYPE) || (frame[OFF_SYSTEM_TYPE] == DVL_SYSTEM_TYPE);
}

/* Drop the first byte and slide forward to the next 0xAA, so a real frame
 * that started inside a bad one is not skipped. */
static void resync(void)
{
    uint16_t next = 1U;

    while ((next < frame_len) && (frame[next] != sop[0]))
    {
        next++;
    }

    frame_len = (uint16_t)(frame_len - next);
    memmove(frame, &frame[next], frame_len);
}

bool dvl_feed_byte(uint8_t byte, DVLData *out)
{
    if ((frame_len == 0U) && (byte != sop[0]))
    {
        return false;
    }

    frame[frame_len++] = byte;

    for (;;)
    {
        if (!header_prefix_ok())
        {
            resync();
            continue;
        }

        if (frame_len < DVL_PACKET_LENGTH)
        {
            return false;
        }

        if (!checksum_ok())
        {
            count_bad++;
            resync();
            continue;
        }

        break;
    }

    out->timestamp_ms      = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
    out->coordinate_system = frame[OFF_COORD_SYSTEM];

    out->vx_m_s   = read_float(&frame[OFF_VEL_X]);
    out->vy_m_s   = read_float(&frame[OFF_VEL_Y]);
    out->vz_m_s   = read_float(&frame[OFF_VEL_Z]);
    out->verr_m_s = read_float(&frame[OFF_VEL_ERR]);

    for (int b = 0; b < 4; b++)
    {
        out->range_beam_m[b] = read_float(&frame[OFF_RANGE1 + 4U * (uint16_t)b]);
    }

    out->mean_range_m       = read_float(&frame[OFF_MEAN_RANGE]);
    out->speed_of_sound_m_s = read_float(&frame[OFF_SPEED_OF_SOUND]);
    out->status             = read_u16(&frame[OFF_STATUS]);
    out->bit                = read_u16(&frame[OFF_BIT]);
    out->input_voltage_v    = read_float(&frame[OFF_INPUT_VOLTAGE]);

    out->velocity_valid = isfinite(out->vx_m_s) &&
                          isfinite(out->vy_m_s) &&
                          isfinite(out->vz_m_s);

    count_ok++;
    frame_len = 0U;

    return true;
}
