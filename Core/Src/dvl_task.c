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

    uint8_t rx_data[64];

    while (1)
    {
        /*
         * Sleep until UART4 IDLE ISR notifies us.
         */
        ulTaskNotifyTake(
            pdTRUE,
            portMAX_DELAY
        );

        /*
         * Drain bytes accumulated by UART4.
         */
        uint16_t received =
            uart4_read(
                rx_data,
                sizeof(rx_data)
            );

        /*
         * Feed bytes into DVL parser.
         */
        for (uint16_t i = 0; i < received; i++)
        {
            dvl_feed_byte(rx_data[i]);
        }
    }
}
