#include "comms_task.h"
#include "uart_packet.h"
#include "packet.h"
#include "uart_packet.h"
#include "control_loop.h"
#include "bar30_task.h"
#include <string.h>
#include "bench.h"

TaskHandle_t commsTaskHandle = NULL;
QueueHandle_t commandQueue = NULL;

/*
 * Commands parsed successfully but not delivered to control_task.
 *
 * This was the one silent-loss path left in the Phase 11 queue analysis:
 * every other drop site had a counter, so a full commandQueue was the
 * only way to lose data with no diagnostic. Non-zero here means
 * control_task is not draining - it empties the queue every 20 ms tick,
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

            /*
             * Drain every complete frame, not just the first. One IDLE
             * event can deliver more than one frame (two CMDs back to
             * back, or a CMD behind looped-back telemetry); with a single
             * parse per notification the second one waited for the next
             * burst to arrive, a full tick late or, if the Pi then went
             * quiet, never.
             */
            while (packet_parse_cmd(&cmd))
            {
                cmd_valid_count++;
                bench_cmd_parsed(&cmd);

                /*
                 * Short bounded wait, then drop and count. The queue is
                 * only full if control_task has stopped draining it, and
                 * blocking longer would just back up the RX path behind
                 * a stalled control loop.
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
        if ((notify_value & (1UL << 1)) && !bench_comms_skip_telemetry())
        {
            TelemetryPayload telemetry;
            uint8_t tx_buf[PACKET_SIZE];

            memset(&telemetry, 0, sizeof(telemetry));

            telemetry.armed   = control_loop_get_armed();
            telemetry.link_ok = control_loop_get_link();

            /*
             * Same split as the Pico firmware:
             *   depth_m     - the Pi's fused depth, what the PID acts on
             *   raw_depth_m - the onboard Bar30, telemetry only
             *
             * xQueuePeek, not Receive: bar30Queue is depth-1 overwrite and
             * this only reads the latest value. Stays 0 with no sensor.
             */
            float pose_now[N_DOF];
            float u_now[N_DOF];

            control_loop_get_pose(pose_now);
            control_loop_get_u(u_now);

            telemetry.depth_m   = pose_now[2];

            float raw_depth = 0.0f;
            if ((bar30Queue != NULL) &&
                (xQueuePeek(bar30Queue, &raw_depth, 0) == pdPASS))
            {
                telemetry.raw_depth_m = raw_depth;
            }
            telemetry.sat_flags = control_loop_get_sat_flags();

            for (int i = 0; i < N_DOF; i++)
            {
                telemetry.pid_u[i] = u_now[i];
            }

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

        /* BENCH_HIL: loopback injector writes after telemetry, same task, so
         * the two never interleave on the wire. Nothing otherwise. */
        if (notify_value & (1UL << 1))
        {
            bench_comms_after_tx();
        }
    }
}
