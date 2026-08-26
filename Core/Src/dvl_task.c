#include "dvl_task.h"
#include "uart4.h"
#include "dvl.h"

#include <stdint.h>

#include "FreeRTOS.h"
#include "task.h"

#include "sensor_status.h"
#include "console.h"
/*
 * Absence detection.
 *
 * The ISR notifies this task on every UART IDLE. With no sensor wired
 * there is no traffic and no notification ever arrives, so a
 * portMAX_DELAY wait would park the task forever - indistinguishable
 * from a crash. A bounded wait lets it notice, publish SENSOR_ABSENT and
 * carry on.
 *
 * ABSENT after this many consecutive timeouts from boot; FAULTED if data
 * had been flowing and then stopped. filter_task gates on freshness, so
 * neither state feeds stale values into the estimate.
 */
#define DVL_WAIT_MS          500U
#define DVL_ABSENT_TIMEOUTS  6U     /* ~3 s of silence */



QueueHandle_t dvlQueue = NULL;


//===========================================================================================================================
void dvl_task(void *argument)
{
    (void)argument;

    uint8_t rx_data[128];

    uint32_t silent_cycles = 0;
    uint8_t  announced     = 0;

    while (1)
    {
        /*
         * Sleep until the UART4 IDLE ISR wakes us, or the wait expires.
         * Bounded, not portMAX_DELAY - see the note above.
         */
        if (ulTaskNotifyTake(
                pdTRUE,
                pdMS_TO_TICKS(DVL_WAIT_MS)
            ) == 0U)
        {
            if (silent_cycles < DVL_ABSENT_TIMEOUTS)
            {
                silent_cycles++;
            }

            if (silent_cycles >= DVL_ABSENT_TIMEOUTS)
            {
                SensorState next = (g_dvl_state == SENSOR_OK)
                                 ? SENSOR_FAULTED : SENSOR_ABSENT;

                if (g_dvl_state != next)
                {
                    g_dvl_state = next;
                    console_printf("DVL: %s", sensor_state_str(next));
                }
            }

            continue;
        }

        silent_cycles = 0;


        /*
         * Drain all bytes currently stored in the
         * UART4 software ring buffer.
         */
        uint16_t received =
            uart4_read(
                rx_data,
                sizeof(rx_data)
            );


        /*
         * Feed every byte into the DVL parser.
         */
        for (uint16_t i = 0U; i < received; i++)
        {
            if (dvl_feed_byte(rx_data[i]))
            {
                DVLData data;

                if (dvl_get_data(&data))
                {
                    if (g_dvl_state != SENSOR_OK)
                    {
                        g_dvl_state = SENSOR_OK;

                        if (announced == 0U)
                        {
                            announced = 1U;
                            console_printf("DVL: ok");
                        }
                    }

                    /*
                     * Queue length = 1.
                     *
                     * Keep newest measurement.
                     */
                    if (dvlQueue != NULL)
                    {
                        xQueueOverwrite(
                            dvlQueue,
                            &data
                        );
                    }
                }
            }
        }
    }
}
