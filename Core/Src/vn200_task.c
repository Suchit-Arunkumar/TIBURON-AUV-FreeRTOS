#include "vn200_task.h"

#include "uart3.h"
#include "vn200.h"

#include <stdint.h>

#include "FreeRTOS.h"
#include "task.h"

#include "sensor_status.h"
#include "console.h"

/*
 * Health is judged on good packets, not on bytes.
 *
 * The task wakes on every UART interrupt, but bytes alone prove nothing:
 * a VN-200 configured for a different output, or a baud mismatch, sends
 * plenty of bytes and never a valid packet. So the sensor is OK only
 * while CRC-valid packets keep arriving.
 *
 * VN200_WAIT_MS bounds the wait so the task still runs when nothing at
 * all arrives (no sensor wired); a portMAX_DELAY wait would park it
 * forever, indistinguishable from a crash.
 *
 * ABSENT: no good packet since boot for VN200_SILENT_MS.
 * FAULTED: good packets were arriving, then none for VN200_SILENT_MS.
 */
#define VN200_WAIT_MS          500U
#define VN200_SILENT_MS        3000U

QueueHandle_t vn200Queue = NULL;

void vn200_task(void *argument)
{
    (void)argument;

    uint8_t rx_data[64];

    /* Enable USART3's interrupt now that a task context exists. */
    uart3_irq_enable();

    TickType_t last_good = xTaskGetTickCount();
    uint8_t    announced = 0U;

    while (1)
    {
        (void)ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(VN200_WAIT_MS));

        /*
         * Drain everything UART3 has buffered, 64 bytes at a time. Reading
         * only one chunk per wake-up left the rest waiting for the next
         * interrupt, so a backlog could build until the ring overflowed.
         */
        uint16_t received;

        while ((received = uart3_read(rx_data, sizeof(rx_data))) > 0U)
        {
            for (uint16_t i = 0U; i < received; i++)
            {
                VN200Data data;

                if (!vn200_feed_byte(rx_data[i], &data))
                {
                    continue;
                }

                last_good = xTaskGetTickCount();

                if (g_vn200_state != SENSOR_OK)
                {
                    g_vn200_state = SENSOR_OK;

                    if (announced == 0U)
                    {
                        announced = 1U;
                        console_printf("VN200: ok");
                    }
                }

                /* Length-1 queue: keep only the newest reading. */
                if (vn200Queue != NULL)
                {
                    xQueueOverwrite(vn200Queue, &data);
                }
            }
        }

        if ((xTaskGetTickCount() - last_good) > pdMS_TO_TICKS(VN200_SILENT_MS))
        {
            SensorState next = (g_vn200_state == SENSOR_OK)
                             ? SENSOR_FAULTED : SENSOR_ABSENT;

            if ((g_vn200_state != next) && (g_vn200_state != SENSOR_FAULTED))
            {
                g_vn200_state = next;

                /* Announce each transition once, not every cycle. */
                console_printf("VN200: %s", sensor_state_str(next));
            }
        }
    }
}
