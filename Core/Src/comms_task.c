#include "comms_task.h"
#include "uart_packet.h"
#include "packet.h"
#include "control_loop.h"
#include "depth_task.h"
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

/*
 * Outgoing frames.
 *
 * A small ring of whole frames that the TX DMA sends in order. Only
 * comms_task touches it: it builds a frame in the slot at head, and when
 * the DMA reports a frame done (COMMS_NOTIFY_TX_DONE) it frees the slot
 * at tail and starts the next one. The ISR never reads or writes the
 * ring, so the ring needs no lock.
 *
 * A frame takes 5.4 ms on the wire and comms_task queues at most a few
 * per 20 ms tick, so four slots never fill on a healthy link. If they
 * ever do, the new frame is dropped and counted.
 */
#define TX_SLOTS  4U

static uint8_t tx_ring[TX_SLOTS][PACKET_SIZE];
static uint8_t tx_head      = 0;
static uint8_t tx_tail      = 0;
static uint8_t tx_count     = 0;
static bool    tx_in_flight = false;

/* Written by comms_task, read by dummy_task's health report. */
static volatile uint32_t tx_drop_count = 0;

uint32_t comms_tx_drops(void)
{
    return tx_drop_count;
}

/* Slot to build the next frame in, or NULL (counted) if the ring is full. */
static uint8_t *tx_reserve(void)
{
    if (tx_count >= TX_SLOTS)
    {
        tx_drop_count++;
        return NULL;
    }

    return tx_ring[tx_head];
}

/* The frame in the reserved slot is complete: queue it. */
static void tx_commit(void)
{
    tx_head = (uint8_t)((tx_head + 1U) % TX_SLOTS);
    tx_count++;
}

/* Start the DMA on the oldest queued frame, if it is idle. */
static void tx_kick(void)
{
    if (!tx_in_flight && (tx_count > 0U))
    {
        if (uart1_tx_dma_start(tx_ring[tx_tail], PACKET_SIZE))
        {
            tx_in_flight = true;
        }
    }
}

/* The DMA finished the frame at tail: free its slot. */
static void tx_done(void)
{
    if (tx_in_flight)
    {
        tx_in_flight = false;
        tx_tail = (uint8_t)((tx_tail + 1U) % TX_SLOTS);
        tx_count--;
    }
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

        if (notify_value & COMMS_NOTIFY_TX_DONE)
        {
            tx_done();
        }

        /* RX: command packet arrived */
        if (notify_value & COMMS_NOTIFY_RX)
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
        uint8_t *slot = NULL;

        if ((notify_value & COMMS_NOTIFY_TELEMETRY) &&
            !bench_comms_skip_telemetry() &&
            ((slot = tx_reserve()) != NULL))
        {
            TelemetryPayload telemetry;
            ControlSnapshot  ctl;

            memset(&telemetry, 0, sizeof(telemetry));
            control_loop_snapshot(&ctl);

            telemetry.armed     = ctl.armed;
            telemetry.link_ok   = ctl.link_ok;
            telemetry.sat_flags = ctl.sat_flags;

            /*
             * Same split as the Pico firmware:
             *   depth_m     - the Pi's fused depth, what the PID acts on
             *   raw_depth_m - the onboard depth sensor (Bar30, or the
             *                 analog backup), telemetry only
             *
             * xQueuePeek, not Receive: depthQueue holds one sample, and
             * other readers want it too. Stays 0 with no sensor.
             */
            telemetry.depth_m = ctl.pose[2];

            DepthSample depth;
            if ((depthQueue != NULL) &&
                (xQueuePeek(depthQueue, &depth, 0) == pdPASS))
            {
                telemetry.raw_depth_m = depth.depth_m;
            }

            for (int i = 0; i < N_DOF; i++)
            {
                telemetry.pid_u[i] = ctl.u[i];
            }

            /*
             * TelemetryPayload is __packed__, so esc_pwm is not
             * guaranteed to be 2-byte aligned. memcpy rather than
             * element assignment through a uint16_t pointer.
             */
            memcpy(telemetry.esc_pwm, ctl.pwm_us, sizeof(telemetry.esc_pwm));

            packet_build_telemetry(&telemetry, slot);
            tx_commit();
        }

        tx_kick();

        /* BENCH_HIL: loopback injector writes after telemetry. Its blocking
         * write waits for the DMA frame to finish first, so the two never
         * interleave on the wire. Nothing otherwise. */
        if (notify_value & COMMS_NOTIFY_TELEMETRY)
        {
            bench_comms_after_tx();
        }
    }
}
