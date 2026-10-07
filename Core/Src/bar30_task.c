#include "bar30_task.h"
#include "bar30.h"
#include "i2c.h"
#include "sensor_status.h"
#include "console.h"

#include "FreeRTOS.h"
#include "task.h"
#include "bench.h"

/*
 * A reading is two conversions of ~12 ms each, so the loop runs at about
 * 35 Hz with nothing else in it; this period only matters once readings
 * fail quickly.
 */
#define BAR30_TASK_PERIOD_MS      20U

/*
 * Back off when the sensor is not answering.
 *
 * Every failed attempt still costs bounded I2C timeouts, so retrying at
 * the full rate would burn CPU doing nothing but timing out. Once the
 * sensor is declared absent the period stretches to 1 s, which is still
 * fast enough to notice it being plugged in.
 */
#define BAR30_ABSENT_PERIOD_MS   1000U
#define BAR30_SLOW_PERIOD_MS      100U
#define BAR30_ABSENT_FAILURES       5U

/* Attempt a bus recovery after this many consecutive failures, in case a
 * slave is holding SDA low rather than simply being missing. */
#define BAR30_RECOVER_AFTER        20U

QueueHandle_t bar30Queue = NULL;

/* After a run of failures, mark the sensor and log it once. */
static void note_failure(uint32_t consecutive_fail, Bar30Status st)
{
    if (consecutive_fail == BAR30_ABSENT_FAILURES)
    {
        /*
         * Distinguish "never worked" from "worked and stopped" - the
         * first says check the wiring, the second says check the sensor.
         */
        SensorState next = (g_bar30_state == SENSOR_OK)
                         ? SENSOR_FAULTED : SENSOR_ABSENT;

        g_bar30_state = next;

        console_printf("Bar30: %s (%s)", sensor_state_str(next), bar30_status_str(st));
    }

    if ((consecutive_fail % BAR30_RECOVER_AFTER) == 0U)
    {
        /* A slave reset mid-transfer can hold SDA low forever; clocking
         * the bus frees it. Harmless if nothing is attached. */
        i2c1_bus_recover();
    }
}

void bar30_task(void *argument)
{
    (void)argument;

    TickType_t last_wake = xTaskGetTickCount();

    uint32_t consecutive_fail = 0;
    uint8_t  initialised      = 0;

    while (1)
    {
        Bar30Status st;

        if (!initialised)
        {
            /*
             * Reset, PROM read and CRC check. Retried every pass until it
             * works, so a sensor connected after boot is picked up without
             * a reset and always gets its own calibration words. (Before,
             * init ran once at boot, and a sensor plugged in later was
             * read with an all-zero PROM.)
             */
            st = bar30_init();

            if (st == BAR30_OK)
            {
                initialised      = 1U;
                consecutive_fail = 0U;
                console_printf("Bar30: ok, surface %ld Pa", (long)bar30_surface_pa());
            }
        }
        else
        {
            Bar30Data data;

            st = bar30_read(&data);

            if (st == BAR30_OK)
            {
                consecutive_fail = 0U;
                g_bar30_state    = SENSOR_OK;

                /* Length-1 queue: keep only the newest depth. */
                if (bar30Queue != NULL)
                {
                    xQueueOverwrite(bar30Queue, &data.depth_m);
                }
            }
        }

        if (st != BAR30_OK)
        {
            if (consecutive_fail < 0xFFFFFFFFUL)
            {
                consecutive_fail++;
            }

            note_failure(consecutive_fail, st);

            /*
             * A sensor that keeps failing is re-initialised once it
             * answers again, in case it was swapped or power-cycled.
             * Nothing is published while failing: bar30Queue keeps its
             * last value, so telemetry never reports a made-up depth.
             */
            if (consecutive_fail >= BAR30_ABSENT_FAILURES)
            {
                initialised = 0U;
            }
        }

        /*
         * Back off on consecutive FAILURES, not merely on a confirmed
         * absent sensor.
         *
         * The dangerous case is not a cleanly missing device - that is
         * declared absent after 5 tries and drops to a 1 s period. It is
         * a device that NACKs intermittently, which on jumper wires to a
         * breakout is the common failure. One failure is enough to slow
         * to 100 ms.
         */
        TickType_t period;

        if (consecutive_fail == 0U)
        {
            period = pdMS_TO_TICKS(BAR30_TASK_PERIOD_MS);
        }
        else if (consecutive_fail < BAR30_ABSENT_FAILURES)
        {
            period = pdMS_TO_TICKS(BAR30_SLOW_PERIOD_MS);
        }
        else
        {
            period = pdMS_TO_TICKS(BAR30_ABSENT_PERIOD_MS);
        }

        /* BENCH_HIL: queued I2C bench work (MPU-6050 stress) runs here,
         * between Bar30 transfers. */
        bench_i2c_service();

        vTaskDelayUntil(&last_wake, period);
    }
}
