#include "dvl_task.h"
#include "uart4.h"
#include "dvl.h"

#include <stdint.h>

#include "FreeRTOS.h"
#include "task.h"

#include "sensor_status.h"
#include "console.h"

/*
 * Health is judged on good frames, not on bytes, the same as imu_task.
 * A frame with no bottom lock still counts as good: the DVL is alive and
 * talking, it just cannot see the bottom (velocity_valid is false).
 *
 * The Wayfinder pings every ~55 ms at 5 m altitude and slower further up
 * (Wayfinder DVL Guide, "Ping Timing"), so 3 s of silence means it has
 * stopped, not that it is between pings.
 */
#define DVL_WAIT_MS          500U
#define DVL_SILENT_MS        3000U

QueueHandle_t dvlQueue = NULL;

void dvl_task(void *argument)
{
    (void)argument;

    uint8_t rx_data[128];

    /* Enable UART4's interrupt now that a task context exists. */
    uart4_irq_enable();

    TickType_t last_good = xTaskGetTickCount();
    uint8_t    announced = 0U;

    while (1)
    {
        (void)ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(DVL_WAIT_MS));

        /*
         * Drain everything UART4 has buffered, 128 bytes at a time, and
         * feed every byte to the DVL parser. Reading only one chunk per
         * wake-up left the rest waiting for the next interrupt.
         */
        uint16_t received;

        while ((received = uart4_read(rx_data, sizeof(rx_data))) > 0U)
        {
            for (uint16_t i = 0U; i < received; i++)
            {
                DVLData data;

                if (!dvl_feed_byte(rx_data[i], &data))
                {
                    continue;
                }

                last_good = xTaskGetTickCount();

                if (g_dvl_state != SENSOR_OK)
                {
                    g_dvl_state = SENSOR_OK;

                    if (announced == 0U)
                    {
                        announced = 1U;
                        console_printf("DVL: ok");
                    }
                }

                /* Length-1 queue: keep only the newest frame. */
                if (dvlQueue != NULL)
                {
                    xQueueOverwrite(dvlQueue, &data);
                }
            }
        }

        if ((xTaskGetTickCount() - last_good) > pdMS_TO_TICKS(DVL_SILENT_MS))
        {
            SensorState next = (g_dvl_state == SENSOR_OK)
                             ? SENSOR_FAULTED : SENSOR_ABSENT;

            if ((g_dvl_state != next) && (g_dvl_state != SENSOR_FAULTED))
            {
                g_dvl_state = next;
                console_printf("DVL: %s", sensor_state_str(next));
            }
        }
    }
}
