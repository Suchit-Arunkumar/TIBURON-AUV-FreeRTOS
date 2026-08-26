#include "comms_task.h"
#include "uart_packet.h"
#include "packet.h"
#include "uart_packet.h"
#include "control_loop.h"
#include <string.h>

TaskHandle_t commsTaskHandle = NULL;
QueueHandle_t commandQueue = NULL;

/*
 * Commands parsed successfully but not delivered to control_task.
 *
 * This was the one silent-loss path left in the Phase 11 queue analysis:
 * every other drop site had a counter, so a full commandQueue was the
 * only way to lose data with no diagnostic. Non-zero here means
 * control_task is not draining - it should consume one per 20 ms tick,
 * so a depth-4 queue only fills if the control loop has stalled.
 *
 * Written by comms_task, read by dummy_task's health report.
 */
static volatile uint32_t cmd_drop_count = 0;

/* CRC-valid packets received since boot. Monotonic; the recovery streak
 * itself is owned entirely by control_task. */
static volatile uint32_t cmd_valid_count = 0;

uint32_t comms_cmd_drops(void)
{
    return cmd_drop_count;
}

uint32_t comms_cmd_valid(void)
{
    return cmd_valid_count;
}

void comms_task(void *argument)
{
    (void)argument;

    /*
     * Enable USART1's interrupt here, not in main. The scheduler is
     * running and this handle is populated, so the ISR's
     * xTaskNotifyFromISR / portYIELD_FROM_ISR pair is safe. See the note
     * in uart_packet.c.
     */
    uart1_irq_enable();

    for (;;)
    {
    	uint32_t notify_value;

    	xTaskNotifyWait(
    	    0,
    	    0xFFFFFFFFUL,
    	    &notify_value,
    	    portMAX_DELAY
    	);

        /* RX: command packet arrived */
        if (notify_value & (1UL << 0))
        {
            CommandPayload cmd;

            if (packet_parse_cmd(&cmd))
            {
                cmd_valid_count++;

                /*
                 * Short bounded wait, then drop and count. Blocking
                 * longer would back up the RX path behind a stalled
                 * control loop, and the newest command is always the one
                 * worth having.
                 */
                if (xQueueSend(
                        commandQueue,
                        &cmd,
                        pdMS_TO_TICKS(1)
                    ) != pdPASS)
                {
                    cmd_drop_count++;
                }
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

            /*
             * TelemetryPayload is __packed__, so esc_pwm is not
             * guaranteed to be 2-byte aligned and passing its address to
             * a uint16_t* parameter is undefined behaviour on a target
             * that faults on unaligned access. Fill an aligned local and
             * copy the bytes in.
             */
            uint16_t pwm_aligned[8];

            control_loop_get_pwm(pwm_aligned, 8);

            memcpy(
                telemetry.esc_pwm,
                pwm_aligned,
                sizeof(pwm_aligned)
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
