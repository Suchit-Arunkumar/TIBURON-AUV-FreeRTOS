#include "dvl_task.h"
#include "uart4.h"
#include "dvl.h"

#include <stdint.h>

#include "FreeRTOS.h"
#include "task.h"


QueueHandle_t dvlQueue = NULL;


//===========================================================================================================================
void dvl_task(void *argument)
{
    (void)argument;

    uint8_t rx_data[128];

    while (1)
    {
        /*
         * Sleep until UART4 IDLE ISR wakes us.
         */
        ulTaskNotifyTake(
            pdTRUE,
            portMAX_DELAY
        );


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
