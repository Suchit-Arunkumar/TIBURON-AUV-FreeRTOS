#include "comms_task.h"
#include "uart_packet.h"
#include "packet.h"
#include "control_loop.h"
#include "depth_task.h"
#include "imu_task.h"
#include "dvl_task.h"
#include "dvl.h"
#include <string.h>
#include "bench.h"

TaskHandle_t commsTaskHandle = NULL;
QueueHandle_t commandQueue = NULL;

// Valid commands that didn't fit in commandQueue. Control empties it every
// tick, so non-zero means the control loop has stalled.
static volatile uint32_t cmd_drop_count = 0;

// CRC-valid command packets since boot.
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
 * Outgoing frames, sent in order by the TX DMA. Only this task touches
 * the ring (the interrupt just signals TX_DONE), so no lock. Per 20 ms
 * tick there are at most 3 frames of 5.4 ms each, so 4 slots is enough.
 */
#define TX_SLOTS  4U

static uint8_t tx_ring[TX_SLOTS][PACKET_SIZE];
static uint8_t tx_head      = 0;
static uint8_t tx_tail      = 0;
static uint8_t tx_count     = 0;
static bool    tx_in_flight = false;

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

/* Milliseconds since a reading taken at t, saturated to fit a u16. */
static uint16_t age_ms(uint32_t now, uint32_t t)
{
    uint32_t age = now - t;
    return (uint16_t)((age > 0xFFFFU) ? 0xFFFFU : age);
}

static uint32_t now_ms(void)
{
    return (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
}

// SENSORS frame (0x03), sent with every telemetry frame. Peek, not
// receive: other tasks read the same samples.
static void queue_sensors_frame(void)
{
    uint8_t *slot = tx_reserve();

    if (slot == NULL)
    {
        return;
    }

    SensorsPayload p;
    ImuSample      imu;
    DepthSample    depth;
    uint32_t       now = now_ms();

    memset(&p, 0, sizeof(p));       /* reserved bytes go into the CRC too */
    p.stamp_ms = now;

    if ((imuQueue != NULL) && (xQueuePeek(imuQueue, &imu, 0) == pdPASS))
    {
        p.imu_age_ms   = age_ms(now, imu.timestamp_ms);
        p.imu_source   = imu.source;
        p.imu_accuracy = imu.accuracy;
        p.yaw_deg      = imu.yaw_deg;
        p.pitch_deg    = imu.pitch_deg;
        p.roll_deg     = imu.roll_deg;
        p.gyro_x_rad_s = imu.gyro_x_rad_s;
        p.gyro_y_rad_s = imu.gyro_y_rad_s;
        p.gyro_z_rad_s = imu.gyro_z_rad_s;
        p.accel_x_m_s2 = imu.accel_x_m_s2;
        p.accel_y_m_s2 = imu.accel_y_m_s2;
        p.accel_z_m_s2 = imu.accel_z_m_s2;
    }
    else
    {
        p.imu_age_ms = 0xFFFFU;
    }

    if ((depthQueue != NULL) && (xQueuePeek(depthQueue, &depth, 0) == pdPASS))
    {
        p.depth_age_ms = age_ms(now, depth.timestamp_ms);
        p.depth_source = depth.source;
        p.depth_m      = depth.depth_m;
        p.water_temp_c = depth.temperature_c;
    }
    else
    {
        p.depth_age_ms = 0xFFFFU;
    }

    packet_build(TYPE_SENSORS, &p, slot);
    tx_commit();
}

// DVL frame (0x04), only when dvl_task has a frame we haven't sent yet.
static void queue_dvl_frame_if_new(void)
{
    static uint32_t last_sent_ms = 0U;
    static bool     sent_any     = false;

    DVLData d;

    if ((dvlQueue == NULL) || (xQueuePeek(dvlQueue, &d, 0) != pdPASS))
    {
        return;
    }

    if (sent_any && (d.timestamp_ms == last_sent_ms))
    {
        return;
    }

    uint8_t *slot = tx_reserve();

    if (slot == NULL)
    {
        return;
    }

    DvlPayload p;
    uint32_t   now = now_ms();

    memset(&p, 0, sizeof(p));
    p.stamp_ms           = now;
    p.age_ms             = age_ms(now, d.timestamp_ms);
    p.velocity_valid     = d.velocity_valid ? 1U : 0U;
    p.coordinate_system  = d.coordinate_system;
    p.vx_m_s             = d.vx_m_s;
    p.vy_m_s             = d.vy_m_s;
    p.vz_m_s             = d.vz_m_s;
    p.verr_m_s           = d.verr_m_s;
    p.mean_range_m       = d.mean_range_m;
    p.status             = d.status;
    p.bit                = d.bit;
    p.speed_of_sound_m_s = d.speed_of_sound_m_s;
    p.input_voltage_v    = d.input_voltage_v;

    for (int b = 0; b < 4; b++)
    {
        p.range_beam_m[b] = d.range_beam_m[b];
    }

    packet_build(TYPE_DVL, &p, slot);
    tx_commit();

    last_sent_ms = d.timestamp_ms;
    sent_any     = true;
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

    // here, not in main: the scheduler and this task's handle now exist
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

            // One interrupt can bring more than one frame, so parse
            // until there are none left.
            while (packet_parse_cmd(&cmd))
            {
                cmd_valid_count++;
                bench_cmd_parsed(&cmd);

                // only full if control has stalled: wait 1 ms, then drop
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

            // depth_m: the Pi's fused depth (what the PID uses).
            // raw_depth_m: this board's own depth sensor, 0 if none.
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

            // packed struct: esc_pwm may be unaligned, so memcpy
            memcpy(telemetry.esc_pwm, ctl.pwm_us, sizeof(telemetry.esc_pwm));

            packet_build_telemetry(&telemetry, slot);
            tx_commit();

            queue_sensors_frame();
            queue_dvl_frame_if_new();
        }

        tx_kick();

        // bench build only: loopback test frames go out after telemetry
        if (notify_value & COMMS_NOTIFY_TELEMETRY)
        {
            bench_comms_after_tx();
        }
    }
}
