#include "depth_task.h"
#include "bar30.h"
#include "adc_depth.h"
#include "i2c.h"
#include "source_select.h"
#include "sensor_status.h"
#include "console.h"

#include "FreeRTOS.h"
#include "task.h"
#include "bench.h"

/*
 * Runs every 20 ms. The analog sensor is read every pass; the Bar30 has
 * its own next-attempt time, so its back-off when failing doesn't slow the
 * analog backup down. A Bar30 reading takes ~25 ms (two conversions).
 */
#define DEPTH_PERIOD_MS           20U

// Retry slower after a failure (100 ms), and once a second after 5 in a
// row, which still notices it being plugged in.
#define BAR30_SLOW_RETRY_MS      100U
#define BAR30_ABSENT_RETRY_MS   1000U
#define BAR30_ABSENT_FAILURES      5U

// in case a slave is holding SDA low
#define BAR30_RECOVER_AFTER       20U

#define ADC_ABSENT_READS          50U   /* ~1 s of failed reads */

QueueHandle_t depthQueue = NULL;

static uint32_t now_ms(void)
{
    return (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
}

typedef struct
{
    uint8_t     initialised;
    uint32_t    consecutive_fail;
    uint32_t    next_try_ms;
    bool        seen;
    DepthSample sample;
} Bar30State;

static void bar30_failed(Bar30State *b, Bar30Status st, uint32_t now)
{
    if (b->consecutive_fail < 0xFFFFFFFFUL)
    {
        b->consecutive_fail++;
    }

    if (b->consecutive_fail == BAR30_ABSENT_FAILURES)
    {
        SensorState next = (g_bar30_state == SENSOR_OK) ? SENSOR_FAULTED : SENSOR_ABSENT;

        g_bar30_state = next;
        console_printf("Bar30: %s (%s)", sensor_state_str(next), bar30_status_str(st));
    }

    if ((b->consecutive_fail % BAR30_RECOVER_AFTER) == 0U)
    {
        i2c1_bus_recover();
    }

    // re-initialise when it comes back, in case it was swapped
    if (b->consecutive_fail >= BAR30_ABSENT_FAILURES)
    {
        b->initialised = 0U;
    }

    b->next_try_ms = now + ((b->consecutive_fail < BAR30_ABSENT_FAILURES)
                            ? BAR30_SLOW_RETRY_MS : BAR30_ABSENT_RETRY_MS);
}

// Init if needed, else read. True when b->sample has a new reading.
static bool bar30_step(Bar30State *b)
{
    Bar30Status st;

    if (!b->initialised)
    {
        st = bar30_init();

        if (st == BAR30_OK)
        {
            b->initialised      = 1U;
            b->consecutive_fail = 0U;
            console_printf("Bar30: ok, surface %ld Pa", (long)bar30_surface_pa());
            return false;
        }
    }
    else
    {
        Bar30Data d;

        st = bar30_read(&d);

        if (st == BAR30_OK)
        {
            b->consecutive_fail = 0U;
            g_bar30_state       = SENSOR_OK;

            b->seen                 = true;
            b->sample.timestamp_ms  = now_ms();
            b->sample.source        = SRC_BAR30;
            b->sample.depth_m       = d.depth_m;
            b->sample.pressure_pa   = d.pressure_pa;
            b->sample.temperature_c = d.temperature_c;
            return true;
        }
    }

    bar30_failed(b, st, now_ms());
    return false;
}

void depth_task(void *argument)
{
    (void)argument;

    TickType_t last_wake = xTaskGetTickCount();

    Bar30State bar = { 0 };

    DepthSample adc_sample  = { 0 };
    bool        adc_seen    = false;
    uint32_t    adc_fails   = 0U;

    adc_depth_init();
    console_printf("ADC depth: zero %ld mV", (long)(adc_depth_zero_v() * 1000.0f));

    SourceSelect select;
    source_select_init(&select, SRC_BAR30, SRC_ADC);
    uint8_t active = SRC_NONE;

    for (;;)
    {
        uint32_t now = now_ms();

        /* --- Bar30, when its next attempt is due. --- */
        bool bar_new = false;

        if ((int32_t)(now - bar.next_try_ms) >= 0)
        {
            bar_new = bar30_step(&bar);
        }

        /* --- Analog sensor, every pass. --- */
        bool adc_new = false;
        AdcDepthData a;

        if (adc_depth_read(&a))
        {
            adc_fails = 0U;
            adc_seen  = true;
            adc_new   = true;

            adc_sample.timestamp_ms  = now_ms();
            adc_sample.source        = SRC_ADC;
            adc_sample.depth_m       = a.depth_m;
            adc_sample.pressure_pa   = a.pressure_pa;
            adc_sample.temperature_c = 0.0f;

            if (g_adc_depth_state != SENSOR_OK)
            {
                g_adc_depth_state = SENSOR_OK;
                console_printf("ADC depth: ok");
            }
        }
        else if (++adc_fails == ADC_ABSENT_READS)
        {
            g_adc_depth_state = (g_adc_depth_state == SENSOR_OK) ? SENSOR_FAULTED : SENSOR_ABSENT;
            console_printf("ADC depth: %s", sensor_state_str(g_adc_depth_state));
        }

        /* --- Pick the source and publish. --- */
        now = now_ms();

        uint8_t next = source_select_update(&select, now,
                                            bar.seen, bar.sample.timestamp_ms,
                                            adc_seen, adc_sample.timestamp_ms);

        if (next != active)
        {
            console_printf("Depth source: %s -> %s",
                           sensor_source_str(active), sensor_source_str(next));
            active = next;
        }

        if (depthQueue != NULL)
        {
            if ((active == SRC_BAR30) && bar_new)
            {
                xQueueOverwrite(depthQueue, &bar.sample);
            }
            else if ((active == SRC_ADC) && adc_new)
            {
                xQueueOverwrite(depthQueue, &adc_sample);
            }
        }

        // bench build only: MPU-6050 I2C test
        bench_i2c_service();

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(DEPTH_PERIOD_MS));
    }
}
