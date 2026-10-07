#include "imu_task.h"

#include "uart3.h"
#include "vn200.h"
#include "bno085.h"
#include "source_select.h"
#include "sensor_status.h"
#include "console.h"

#include <stdint.h>
#include <stdbool.h>

/*
 * One task, two IMUs.
 *
 * Wake-ups come from two places:
 *   - the USART3 interrupt, whenever VN-200 bytes arrive (~100 Hz);
 *   - a 5 ms timeout, which is when the BNO085 gets polled. It has no
 *     INT pin wired, so polling is the only way to know it has data.
 *
 * Each pass: drain the VN-200 bytes, poll the BNO085, decide which one
 * is active, publish its newest reading to imuQueue if there is one.
 *
 * A sensor is OK while it produces good readings, ABSENT if it never has,
 * FAULTED if it did and then went silent for IMU_SILENT_MS.
 */
#define IMU_POLL_MS          5U
#define IMU_SILENT_MS        3000U
#define BNO_RETRY_MS         2000U     /* try to (re)open the BNO085 */
#define BNO_LOST_MS          1000U     /* no orientation for this long: reopen */

QueueHandle_t imuQueue      = NULL;
TaskHandle_t  imuTaskHandle = NULL;

static uint32_t now_ms(void)
{
    return (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
}

/* OK / ABSENT / FAULTED bookkeeping for one sensor; prints each change. */
static void update_state(volatile SensorState *state, const char *name,
                         bool seen, uint32_t last_ms, uint32_t now)
{
    SensorState next;

    if (seen && ((now - last_ms) <= IMU_SILENT_MS))
    {
        next = SENSOR_OK;
    }
    else if (now < IMU_SILENT_MS)
    {
        return;                     /* still starting up */
    }
    else
    {
        next = (*state == SENSOR_OK || *state == SENSOR_FAULTED)
             ? SENSOR_FAULTED : SENSOR_ABSENT;
    }

    if (*state != next)
    {
        *state = next;
        console_printf("%s: %s", name, sensor_state_str(next));
    }
}

static void from_vn200(const VN200Data *v, uint32_t t, ImuSample *s)
{
    s->timestamp_ms = t;
    s->source       = SRC_VN200;
    s->accuracy     = 3U;
    s->yaw_deg      = v->yaw_deg;
    s->pitch_deg    = v->pitch_deg;
    s->roll_deg     = v->roll_deg;
    s->gyro_x_rad_s = v->gyro_x_rad_s;
    s->gyro_y_rad_s = v->gyro_y_rad_s;
    s->gyro_z_rad_s = v->gyro_z_rad_s;
    s->accel_x_m_s2 = v->accel_x_m_s2;
    s->accel_y_m_s2 = v->accel_y_m_s2;
    s->accel_z_m_s2 = v->accel_z_m_s2;
}

static void from_bno085(const Bno085Data *b, uint32_t t, ImuSample *s)
{
    s->timestamp_ms = t;
    s->source       = SRC_BNO085;
    s->accuracy     = b->accuracy;
    s->yaw_deg      = b->yaw_deg;
    s->pitch_deg    = b->pitch_deg;
    s->roll_deg     = b->roll_deg;
    s->gyro_x_rad_s = b->gyro_x_rad_s;
    s->gyro_y_rad_s = b->gyro_y_rad_s;
    s->gyro_z_rad_s = b->gyro_z_rad_s;
    s->accel_x_m_s2 = b->accel_x_m_s2;
    s->accel_y_m_s2 = b->accel_y_m_s2;
    s->accel_z_m_s2 = b->accel_z_m_s2;
}

void imu_task(void *argument)
{
    (void)argument;

    uint8_t rx_data[64];

    /* Enable USART3's interrupt now that a task context exists. */
    uart3_irq_enable();

    SourceSelect select;
    source_select_init(&select, SRC_VN200, SRC_BNO085);

    ImuSample vn_sample  = { 0 };
    ImuSample bno_sample = { 0 };
    bool      vn_seen  = false, bno_seen = false;
    bool      vn_new   = false, bno_new  = false;
    uint32_t  bno_last_try = 0U;
    uint8_t   active   = SRC_NONE;

    for (;;)
    {
        (void)ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(IMU_POLL_MS));

        uint32_t now = now_ms();

        /* --- VN-200: drain everything buffered, 64 bytes at a time. --- */
        uint16_t received;

        while ((received = uart3_read(rx_data, sizeof(rx_data))) > 0U)
        {
            for (uint16_t i = 0U; i < received; i++)
            {
                VN200Data v;

                if (vn200_feed_byte(rx_data[i], &v))
                {
                    from_vn200(&v, now_ms(), &vn_sample);
                    vn_seen = true;
                    vn_new  = true;
                }
            }
        }

        /* --- BNO085: poll, and (re)open it when needed. --- */
        if (bno085_is_open())
        {
            Bno085Data b;

            if (bno085_service(&b))
            {
                from_bno085(&b, now_ms(), &bno_sample);
                bno_seen = true;
                bno_new  = true;
            }
            else if (!bno_seen ? (now - bno_last_try > BNO_LOST_MS)
                               : (now - bno_sample.timestamp_ms > BNO_LOST_MS))
            {
                /* Open, but nothing for a second: start again. */
                bno085_close();
                bno_last_try = now;
            }
        }
        else if ((now - bno_last_try) >= BNO_RETRY_MS)
        {
            bno_last_try = now;

            if (bno085_open())
            {
                console_printf("BNO085: open");
            }
        }

        now = now_ms();

        update_state(&g_vn200_state,  "VN200",  vn_seen,  vn_sample.timestamp_ms,  now);
        update_state(&g_bno085_state, "BNO085", bno_seen, bno_sample.timestamp_ms, now);

        /* --- Pick the source and publish. --- */
        uint8_t next = source_select_update(&select, now,
                                            vn_seen,  vn_sample.timestamp_ms,
                                            bno_seen, bno_sample.timestamp_ms);

        if (next != active)
        {
            console_printf("IMU source: %s -> %s",
                           sensor_source_str(active), sensor_source_str(next));
            active = next;
        }

        if (imuQueue != NULL)
        {
            if ((active == SRC_VN200) && vn_new)
            {
                xQueueOverwrite(imuQueue, &vn_sample);
            }
            else if ((active == SRC_BNO085) && bno_new)
            {
                xQueueOverwrite(imuQueue, &bno_sample);
            }
        }

        vn_new  = false;
        bno_new = false;
    }
}
