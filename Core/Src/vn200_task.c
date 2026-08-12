#include "vn200_task.h"
#include "uart3.h"
#include "vn200.h"

#include <stdint.h>

#include "FreeRTOS.h"
#include "task.h"

//===========================================================================================================================
void vn200_task(void *argument)
{
    (void)argument;

    uint8_t rx_data[64];

    while (1)
    {
        /*
         * Sleep until USART3 IDLE ISR notifies us.
         */
        ulTaskNotifyTake(
            pdTRUE,
            portMAX_DELAY
        );

        /*
         * Drain all bytes accumulated by UART3.
         */
        uint16_t received =
            uart3_read(
                rx_data,
                sizeof(rx_data)
            );

        /*
         * Feed bytes into VN-200 packet assembler.
         */
        for (uint16_t i = 0; i < received; i++)
        {
            vn200_feed_byte(rx_data[i]);
        }
    }
}
