#include "bar30_task.h"
#include "bar30.h"
#include "i2c.h"
#include "sensor_status.h"
#include "console.h"

#include "FreeRTOS.h"
#include "task.h"
#include "bench.h"

#define BAR30_TASK_PERIOD_MS      20U

/*
 * Back off when the sensor is not answering.
 *
 * Every failed read still costs two bounded I2C timeouts, so retrying at
 * the full 50 Hz would burn CPU at priority 4 doing nothing but timing
 * out. Once the sensor is declared absent the period stretches to 1 s,
 * which is still fast enough to notice it being plugged in.
 */
#define BAR30_ABSENT_PERIOD_MS   1000U
#define BAR30_SLOW_PERIOD_MS      100U
#define BAR30_ABSENT_FAILURES       5U

/* Attempt a bus recovery after this many consecutive failures, in case a
 * slave is holding SDA low rather than simply being missing. */
#define BAR30_RECOVER_AFTER        20U

QueueHandle_t bar30Queue = NULL;

void bar30_task(void *argument)
{
    (void)argument;

    TickType_t last_wake = xTaskGetTickCount();

    uint32_t consecutive_fail = 0;
    uint8_t  announced_ok     = 0;

    /*
     * PROM calibration. If this fails the sensor is not on the bus, but
     * the task still runs - it retries, so a sensor connected after boot
     * is picked up without a reset.
     */
    I2C_Status st = bar30_init();

    if (st != I2C_OK)
    {
        g_bar30_state = SENSOR_ABSENT;
        console_printf("Bar30: ABSENT (%s)", i2c_status_str(st));
    }

    while (1)
    {
        float depth = 0.0f;

        st = bar30_read(&depth);

        if (st == I2C_OK)
        {
            consecutive_fail = 0;

            if (g_bar30_state != SENSOR_OK)
            {
                g_bar30_state = SENSOR_OK;

                if (announced_ok == 0U)
                {
                    announced_ok = 1U;
                    console_printf("Bar30: ok");
                }
            }

            if (bar30Queue != NULL)
            {
                xQueueOverwrite(bar30Queue, &depth);
            }
        }
        else
        {
            if (consecutive_fail < 0xFFFFFFFFUL)
            {
                consecutive_fail++;
            }

            if (consecutive_fail == BAR30_ABSENT_FAILURES)
            {
                /*
                 * Distinguish "never worked" from "worked and stopped" -
                 * the first says check the wiring, the second says check
                 * the sensor.
                 */
                SensorState next = (g_bar30_state == SENSOR_OK)
                                 ? SENSOR_FAULTED : SENSOR_ABSENT;

                g_bar30_state = next;

                console_printf("Bar30: %s (%s)",
                               sensor_state_str(next),
                               i2c_status_str(st));
            }

            if ((consecutive_fail % BAR30_RECOVER_AFTER) == 0U)
            {
                /* A slave reset mid-transfer can hold SDA low forever;
                 * clocking the bus frees it. Harmless if nothing is
                 * attached. */
                i2c1_bus_recover();
            }

            /*
             * Nothing is published on failure. bar30Queue keeps its last
             * value, so telemetry never reports a fabricated depth.
             */
        }

        /*
         * Back off on consecutive FAILURES, not merely on a confirmed
         * absent sensor.
         *
         * The dangerous case is not a cleanly missing device - that is
         * declared absent after 5 tries and drops to a 1 s period. It is
         * a device that NACKs intermittently, which on jumper wires to a
         * breakout is the common failure. Keying the period on
         * g_bar30_state alone left such a device at the full 20 ms
         * period, retrying at 50 Hz and paying an I2C timeout each time.
         *
         * One failure is enough to slow to 100 ms. Combined with the
         * yielding wait in i2c.c, worst-case duty at priority 4 drops
         * from roughly 65% to under 2%.
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
         * in the bus owner's own context, between Bar30 transactions. */
        bench_i2c_service();

        vTaskDelayUntil(&last_wake, period);
    }
}
