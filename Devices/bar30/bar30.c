#include "bar30_task.h"

#include "bar30.h"

#include "FreeRTOS.h"
#include "task.h"


/*
 * Bar30 conversion already contains delays internally.
 *
 * The task therefore simply performs a measurement and
 * publishes the newest result.
 */
#define BAR30_TASK_PERIOD_MS      20U


QueueHandle_t bar30Queue = NULL;


void bar30_task(void *argument)
{
    (void)argument;


    TickType_t last_wake =
        xTaskGetTickCount();


    while (1)
    {
        /*
         * Read current depth.
         */
        float depth =
            bar30_read();


        /*
         * Keep only the newest measurement.
         *
         * The filter checks whether xQueueReceive()
         * actually obtains this value, which becomes
         * the "new data" indication.
         */
        if (bar30Queue != NULL)
        {
            xQueueOverwrite(
                bar30Queue,
                &depth
            );
        }


        /*
         * Schedule next measurement.
         */
        vTaskDelayUntil(
            &last_wake,
            pdMS_TO_TICKS(BAR30_TASK_PERIOD_MS)
        );
    }
}
