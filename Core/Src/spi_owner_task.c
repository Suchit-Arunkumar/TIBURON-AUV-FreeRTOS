#include "spi_owner_task.h"

#include "spi.h"
#include "ili9341.h"
#include "sd_card.h"
#include "control_loop.h"
#include "console.h"
#include "imu_task.h"
#include "depth_task.h"
#include "dvl_task.h"
#include "dvl.h"
#include "sensor_status.h"

#include "FreeRTOS.h"
#include "task.h"

#include <stdio.h>
#include <string.h>

// When no request arrives for this long, redraw the display.
#define DISPLAY_REFRESH_MS   500U

// Older than this and a reading is shown as missing.
#define STALE_MS             500U

QueueHandle_t spiRequestQueue = NULL;

static volatile uint32_t sd_write_count = 0;
static volatile uint32_t sd_error_count = 0;

uint32_t spi_owner_sd_writes(void) { return sd_write_count; }
uint32_t spi_owner_sd_errors(void) { return sd_error_count; }

// Each handler sets the clock its device needs before using the bus.

static void handle_sd_block(const SpiRequest *req)
{
    spi_set_baud(SPI_BR_SD_DATA);

    SD_Status st = sd_write_block(req->block_addr, req->block);

    if (st == SD_OK)
    {
        sd_write_count++;
    }
    else
    {
        sd_error_count++;

        // only the first few, so a pulled card doesn't flood the console
        if (sd_error_count <= 3U)
        {
            console_printf("SD write err @blk %lu: %s",
                           (unsigned long)req->block_addr,
                           sd_status_str(st));
        }
    }
}

/*
 * Status screen. What's on screen is remembered per character, and only
 * characters that changed are redrawn: a full redraw of the 320x240 panel
 * at 5.6 MHz takes ~220 ms, which would hold up SD writes.
 */
static char shown[TFT_ROWS][TFT_COLS];

static void put_line(uint8_t row, const char *text, uint16_t fg)
{
    char padded[TFT_COLS];

    memset(padded, ' ', sizeof(padded));
    memcpy(padded, text, strnlen(text, TFT_COLS));

    for (uint8_t col = 0; col < TFT_COLS; col++)
    {
        if (padded[col] != shown[row][col])
        {
            char one[2] = { padded[col], '\0' };

            ili9341_draw_text(col, row, one, fg, TFT_BLACK);
            shown[row][col] = padded[col];
        }
    }
}

static long rounded(float x)
{
    return (long)(x + ((x >= 0.0f) ? 0.5f : -0.5f));
}

static char state_letter(SensorState s)
{
    switch (s)
    {
        case SENSOR_OK:      return 'o';
        case SENSOR_ABSENT:  return '-';
        case SENSOR_FAULTED: return 'F';
        default:             return '?';
    }
}

static void handle_display(void)
{
    char line[TFT_COLS + 8];
    uint32_t now = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);

    spi_set_baud(SPI_BR_TFT);

    ControlSnapshot ctl;
    control_loop_snapshot(&ctl);

    put_line(0, "TIBURON AUV", TFT_CYAN);

    snprintf(line, sizeof(line), "ARM %d  LINK %d  FS %d",
             ctl.armed ? 1 : 0, ctl.link_ok ? 1 : 0,
             control_loop_in_failsafe() ? 1 : 0);
    put_line(1, line, ctl.armed ? TFT_RED : TFT_WHITE);

    ImuSample imu;
    if ((imuQueue != NULL) && (xQueuePeek(imuQueue, &imu, 0) == pdPASS) &&
        ((now - imu.timestamp_ms) <= STALE_MS))
    {
        snprintf(line, sizeof(line), "IMU %s acc %u",
                 sensor_source_str(imu.source), (unsigned)imu.accuracy);
        put_line(2, line, TFT_GREEN);

        snprintf(line, sizeof(line), "Y %4ld P %4ld R %4ld",
                 rounded(imu.yaw_deg), rounded(imu.pitch_deg), rounded(imu.roll_deg));
        put_line(3, line, TFT_WHITE);
    }
    else
    {
        put_line(2, "IMU none", TFT_YELLOW);
        put_line(3, "", TFT_WHITE);
    }

    DepthSample depth;
    if ((depthQueue != NULL) && (xQueuePeek(depthQueue, &depth, 0) == pdPASS) &&
        ((now - depth.timestamp_ms) <= STALE_MS))
    {
        snprintf(line, sizeof(line), "DEPTH %s %5ld cm",
                 sensor_source_str(depth.source), rounded(depth.depth_m * 100.0f));
        put_line(4, line, TFT_GREEN);
    }
    else
    {
        put_line(4, "DEPTH none", TFT_YELLOW);
    }

    DVLData dvl;
    if ((dvlQueue != NULL) && (xQueuePeek(dvlQueue, &dvl, 0) == pdPASS) &&
        ((now - dvl.timestamp_ms) <= 2000U))
    {
        if (dvl.velocity_valid)
        {
            snprintf(line, sizeof(line), "DVL vx %4ld vy %4ld cm/s",
                     rounded(dvl.vx_m_s * 100.0f), rounded(dvl.vy_m_s * 100.0f));
            put_line(5, line, TFT_GREEN);
        }
        else
        {
            put_line(5, "DVL no bottom lock", TFT_YELLOW);
        }
    }
    else
    {
        put_line(5, "DVL none", TFT_YELLOW);
    }

    snprintf(line, sizeof(line), "VN:%c BN:%c DV:%c B3:%c AD:%c",
             state_letter(g_vn200_state), state_letter(g_bno085_state),
             state_letter(g_dvl_state), state_letter(g_bar30_state),
             state_letter(g_adc_depth_state));
    put_line(6, line, TFT_WHITE);

    snprintf(line, sizeof(line), "SD %lu blk %lu err",
             (unsigned long)sd_write_count, (unsigned long)sd_error_count);
    put_line(7, line, (sd_error_count != 0U) ? TFT_YELLOW : TFT_WHITE);

    snprintf(line, sizeof(line), "PWM %4u %4u %4u %4u",
             ctl.pwm_us[0], ctl.pwm_us[1], ctl.pwm_us[2], ctl.pwm_us[3]);
    put_line(8, line, TFT_WHITE);

    snprintf(line, sizeof(line), "    %4u %4u %4u %4u",
             ctl.pwm_us[4], ctl.pwm_us[5], ctl.pwm_us[6], ctl.pwm_us[7]);
    put_line(9, line, TFT_WHITE);
}

void spi_owner_task(void *argument)
{
    (void)argument;

    // static: 520 bytes, too big for this task's stack. Only this task
    // uses it.
    static SpiRequest req;

    // force a full first draw
    memset(shown, 0, sizeof(shown));

    while (1)
    {
        if (xQueueReceive(
                spiRequestQueue,
                &req,
                pdMS_TO_TICKS(DISPLAY_REFRESH_MS)
            ) == pdPASS)
        {
            switch (req.type)
            {
                case SPI_REQ_SD_BLOCK:
                    handle_sd_block(&req);
                    break;

                case SPI_REQ_DISPLAY:
                    handle_display();
                    break;

                default:
                    break;
            }
        }
        else
        {
            // the receive timeout doubles as the display refresh tick
            handle_display();
        }
    }
}
