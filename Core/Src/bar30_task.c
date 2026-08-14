#include "bar30_task.h"
#include "bar30.h"
#include "FreeRTOS.h"
#include "task.h"

#define BAR30_TASK_PERIOD_MS      20U

QueueHandle_t bar30Queue = NULL;

void bar30_task(void *argument)
{
    (void)argument;

    TickType_t last_wake = xTaskGetTickCount();

    while (1)
    {
        float depth = bar30_read();

        if (bar30Queue != NULL)
        {
            xQueueOverwrite(bar30Queue, &depth);
        }

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(BAR30_TASK_PERIOD_MS));
    }
}
