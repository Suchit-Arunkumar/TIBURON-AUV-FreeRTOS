#include "bno085.h"

#include <math.h>
#include <string.h>

#include "sh2.h"
#include "sh2_hal.h"
#include "sh2_err.h"
#include "sh2_SensorValue.h"

#include "i2c.h"
#include "timer_timebase.h"

#include "FreeRTOS.h"
#include "task.h"

#define REPORT_INTERVAL_US   20000U     /* 50 Hz */

#define RAD_TO_DEG           57.29577951f

static uint8_t  bno_addr    = 0x4AU;
static bool     session     = false;
static bool     reset_seen  = false;
static uint32_t reset_count = 0U;
static uint32_t decode_errs = 0U;

/* Latest value of each report, filled by the sensor callback. */
static struct
{
    float   qw, qx, qy, qz;
    uint8_t q_accuracy;
    bool    q_new;
    float   gx, gy, gz;
    float   ax, ay, az;
} cache;

/* ------------------------------------------------------------------------
 * The sh2_Hal_t functions: how the SH-2 library reaches the sensor.
 * ------------------------------------------------------------------------ */

/*
 * SHTP over I2C: a write of {length lo, length hi, channel, sequence,
 * command}. This one is a reset on the executable channel, the same packet
 * the Adafruit driver sends. It doubles as the probe for which address
 * the sensor answers on.
 *
 * One try per address. With no sensor fitted each try is a quick address
 * NACK, so probing costs well under a millisecond; the IMU task simply
 * tries again a couple of seconds later (Adafruit retries five times with
 * 30 ms pauses, which would hold the task for 300 ms every time).
 */
static int hal_open(sh2_Hal_t *self)
{
    (void)self;

    static const uint8_t soft_reset[] = { 5, 0, 1, 0, 1 };
    static const uint8_t addrs[]      = { 0x4A, 0x4B };

    for (unsigned a = 0; a < sizeof(addrs); a++)
    {
        if (i2c_write(addrs[a], soft_reset, sizeof(soft_reset)) == I2C_OK)
        {
            bno_addr = addrs[a];

            /* Time for the hub to reboot before it is talked to. */
            vTaskDelay(pdMS_TO_TICKS(300));
            return 0;
        }
    }

    return -1;
}

static void hal_close(sh2_Hal_t *self)
{
    (void)self;
}

/*
 * Every SHTP packet starts with a 4-byte header whose first two bytes are
 * the packet length (bit 15 is a "continued" flag). Read just the header
 * first; if the length is zero there is nothing waiting. Otherwise read
 * the whole packet in one transfer: the sensor sends the header again,
 * then the payload, which is exactly what the library expects in pBuffer.
 */
static int hal_read(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len, uint32_t *t_us)
{
    (void)self;

    uint8_t header[4];

    if (i2c_read(bno_addr, header, sizeof(header)) != I2C_OK)
    {
        return 0;
    }

    uint16_t packet_len = (uint16_t)(header[0] | ((uint16_t)header[1] << 8));
    packet_len &= 0x7FFFU;

    if ((packet_len == 0U) || (packet_len > len))
    {
        return 0;
    }

    if (i2c_read(bno_addr, pBuffer, packet_len) != I2C_OK)
    {
        return 0;
    }

    *t_us = micros();
    return (int)packet_len;
}

static int hal_write(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len)
{
    (void)self;

    if (i2c_write(bno_addr, pBuffer, (uint16_t)len) != I2C_OK)
    {
        return 0;
    }

    return (int)len;
}

static uint32_t hal_time_us(sh2_Hal_t *self)
{
    (void)self;
    return micros();
}

static sh2_Hal_t hal =
{
    .open      = hal_open,
    .close     = hal_close,
    .read      = hal_read,
    .write     = hal_write,
    .getTimeUs = hal_time_us,
};

/* ------------------------------------------------------------------------
 * Callbacks. Both run inside sh2_service(), in the IMU task.
 * ------------------------------------------------------------------------ */

static void on_async_event(void *cookie, sh2_AsyncEvent_t *event)
{
    (void)cookie;

    /* The hub rebooted (brown-out, ESD...). Its report settings are gone. */
    if (event->eventId == SH2_RESET)
    {
        reset_seen = true;
    }
}

/*
 * The library calls this once per event, and one SHTP packet can carry
 * several. Each report type is cached separately, the same fix as in the
 * Pico test sketch: a single shared slot keeps only the last event and
 * silently loses the rest.
 */
static void on_sensor_event(void *cookie, sh2_SensorEvent_t *event)
{
    (void)cookie;

    sh2_SensorValue_t v;

    if (sh2_decodeSensorEvent(&v, event) != SH2_OK)
    {
        decode_errs++;
        return;
    }

    switch (v.sensorId)
    {
        case SH2_GAME_ROTATION_VECTOR:
            cache.qw         = v.un.gameRotationVector.real;
            cache.qx         = v.un.gameRotationVector.i;
            cache.qy         = v.un.gameRotationVector.j;
            cache.qz         = v.un.gameRotationVector.k;
            cache.q_accuracy = (uint8_t)(v.status & 0x03U);
            cache.q_new      = true;
            break;

        case SH2_GYROSCOPE_CALIBRATED:
            cache.gx = v.un.gyroscope.x;
            cache.gy = v.un.gyroscope.y;
            cache.gz = v.un.gyroscope.z;
            break;

        case SH2_ACCELEROMETER:
            cache.ax = v.un.accelerometer.x;
            cache.ay = v.un.accelerometer.y;
            cache.az = v.un.accelerometer.z;
            break;

        default:
            break;
    }
}

/* ------------------------------------------------------------------------ */

static bool enable_reports(void)
{
    static const sh2_SensorId_t ids[] =
    {
        SH2_GAME_ROTATION_VECTOR,
        SH2_GYROSCOPE_CALIBRATED,
        SH2_ACCELEROMETER,
    };

    sh2_SensorConfig_t config;

    memset(&config, 0, sizeof(config));
    config.reportInterval_us = REPORT_INTERVAL_US;

    for (unsigned i = 0; i < sizeof(ids) / sizeof(ids[0]); i++)
    {
        if (sh2_setSensorConfig(ids[i], &config) != SH2_OK)
        {
            return false;
        }
    }

    return true;
}

/* Quaternion to yaw/pitch/roll, ZYX order, degrees. Same as the Pico
 * sketch. Pitch is clamped at +/-90 degrees, where yaw and roll couple. */
static void quat_to_euler(float w, float x, float y, float z, Bno085Data *out)
{
    float sinp = 2.0f * (w * y - z * x);

    if (sinp >  1.0f) sinp =  1.0f;
    if (sinp < -1.0f) sinp = -1.0f;

    out->roll_deg  = atan2f(2.0f * (w * x + y * z), 1.0f - 2.0f * (x * x + y * y)) * RAD_TO_DEG;
    out->pitch_deg = asinf(sinp) * RAD_TO_DEG;
    out->yaw_deg   = atan2f(2.0f * (w * z + x * y), 1.0f - 2.0f * (y * y + z * z)) * RAD_TO_DEG;
}

bool bno085_open(void)
{
    /* The library has a single session. Opening again on top of a live
     * one fails every time, so always close first (the Pico sketch found
     * the same). */
    bno085_close();

    memset(&cache, 0, sizeof(cache));
    reset_seen = false;

    if (sh2_open(&hal, on_async_event, NULL) != SH2_OK)
    {
        return false;
    }

    session = true;

    if ((sh2_setSensorCallback(on_sensor_event, NULL) != SH2_OK) || !enable_reports())
    {
        bno085_close();
        return false;
    }

    /* The reset we just caused is reported once at start-up; it is not a
     * new reset. */
    reset_seen = false;
    return true;
}

void bno085_close(void)
{
    if (session)
    {
        sh2_close();
        session = false;
    }
}

bool bno085_is_open(void)
{
    return session;
}

bool bno085_service(Bno085Data *out)
{
    if (!session)
    {
        return false;
    }

    sh2_service();

    if (reset_seen)
    {
        reset_seen = false;
        reset_count++;
        (void)enable_reports();
    }

    if (!cache.q_new)
    {
        return false;
    }

    cache.q_new = false;

    quat_to_euler(cache.qw, cache.qx, cache.qy, cache.qz, out);

    out->gyro_x_rad_s = cache.gx;
    out->gyro_y_rad_s = cache.gy;
    out->gyro_z_rad_s = cache.gz;
    out->accel_x_m_s2 = cache.ax;
    out->accel_y_m_s2 = cache.ay;
    out->accel_z_m_s2 = cache.az;
    out->accuracy     = cache.q_accuracy;

    return true;
}

uint32_t bno085_resets(void)
{
    return reset_count;
}

uint32_t bno085_decode_errors(void)
{
    return decode_errs;
}
