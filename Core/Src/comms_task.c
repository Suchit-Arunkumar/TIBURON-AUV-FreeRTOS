#include "comms_task.h"
#include "packet.h"
#include "uart_packet.h"
#include "control_loop.h"
#include <string.h>

TaskHandle_t commsTaskHandle = NULL;
QueueHandle_t commandQueue = NULL;

void comms_task(void *argument)
{
    (void)argument;

    for (;;)
    {
        uint32_t notify_value;

        /*
         * Block until either:
         * bit 0 = RX command event
         * bit 1 = telemetry TX event
         */
        notify_value = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        /* RX: command packet arrived */
        if (notify_value & (1UL << 0))
        {
            CommandPayload cmd;

            if (packet_parse_cmd(&cmd))
            {
                xQueueSend(
                    commandQueue,
                    &cmd,
                    pdMS_TO_TICKS(1)
                );
            }
        }

        /* TX: telemetry requested */
        if (notify_value & (1UL << 1))
        {
            TelemetryPayload telemetry;
            uint8_t tx_buf[PACKET_SIZE];

            memset(&telemetry, 0, sizeof(telemetry));

            telemetry.armed = control_loop_get_armed();
            telemetry.link_ok = control_loop_get_link();

            control_loop_get_pwm(
                telemetry.esc_pwm,
                8
            );

            packet_build_telemetry(
                &telemetry,
                tx_buf
            );

            uart1_write_buf(
                tx_buf,
                PACKET_SIZE
            );
        }
    }
}
