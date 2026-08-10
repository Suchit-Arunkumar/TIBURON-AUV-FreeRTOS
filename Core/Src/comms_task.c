#include "comms_task.h"
#include "packet.h"
#include "uart_packet.h"
#include "control_loop.h"

TaskHandle_t commsTaskHandle = NULL;
QueueHandle_t commandQueue = NULL;

TelemetryPayload telemetry;
uint8_t tx_buf[PACKET_SIZE];

void comms_task(void *argument)
{
    (void)argument;

    CommandPayload cmd;

    for (;;)
    {
        /*
         * Wait for either:
         * notification 0 = incoming UART command
         * notification 1 = telemetry transmission request
         */
        uint32_t rx_notify =
            ulTaskNotifyTakeIndexed(0, pdTRUE, 0);

        uint32_t tx_notify =
            ulTaskNotifyTakeIndexed(1, pdTRUE, 0);

        if (rx_notify == 0 && tx_notify == 0)
        {
            /*
             * Nothing pending.
             * Sleep briefly rather than busy-looping.
             */
            vTaskDelay(pdMS_TO_TICKS(1));
            continue;
        }

        /* ---------------- RX COMMANDS ---------------- */

        if (rx_notify > 0)
        {
            if (packet_parse_cmd(&cmd))
            {
                xQueueSend(
                    commandQueue,
                    &cmd,
                    pdMS_TO_TICKS(1)
                );
            }
        }

        /* ---------------- TELEMETRY TX ---------------- */

        if (tx_notify > 0)
        {
        	if (tx_notify > 0)
        	{
        	    memset(&telemetry, 0, sizeof(telemetry));

        	    telemetry.armed = control_loop_get_armed();
        	    telemetry.link_ok = control_loop_get_link();

        	    control_loop_get_pwm(telemetry.esc_pwm, 8);

        	    packet_build_telemetry(&telemetry, tx_buf);

        	    uart1_write_buf(tx_buf, PACKET_SIZE);
        	}
        }
    }
}
