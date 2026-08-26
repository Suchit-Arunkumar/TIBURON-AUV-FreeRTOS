#include "sd_card.h"
#include "spi.h"

#include "FreeRTOS.h"
#include "task.h"

#include <stddef.h>

/*
 * Post-write busy timeout.
 *
 * The SD Physical Layer Simplified Specification sets the host timeout
 * for a single-block write at 250 ms (for SDSC it is derived from
 * TAAC/NSAC/R2W_FACTOR in the CSD but capped at the same figure). Typical
 * commit on a generic Class 10 card is 2-3 ms; the worst case is reached
 * when the card runs internal wear-levelling mid-write.
 *
 * TIMED AGAINST THE SCHEDULER TICK, not a loop count.
 *
 * Two earlier versions of this were loop counts. The first was a bare
 * 100000 iterations - about 110 ms, less than half the allowance, so a
 * merely-slow card was reported as failed. The second derived the count
 * from an estimated 1.822 us per poll, of which 0.40 us was a COMPUTED
 * driver overhead: if the true figure is 0.25 or 0.60 us, that "250 ms"
 * is really 229 ms or 275 ms, and 229 ms fails a card the specification
 * says must be tolerated. A loop count is also at the mercy of what the
 * optimiser does to the loop body.
 *
 * xTaskGetTickCount() is ground truth and immune to both. The only
 * caller of the block operations is spi_owner_task, which runs
 * post-scheduler, so the tick is always available there.
 */
#define SD_WRITE_TIMEOUT_MS        250U
#define SD_READ_TIMEOUT_MS         100U

/*
 * Yield between polls instead of spinning.
 *
 * Spinning held the CPU at priority 3 for the entire wait. Control
 * (7) and the sensor tasks (4-6) preempt regardless, so the 50 Hz
 * deadline was never at risk - but everything BELOW the bus owner was
 * starved for up to 250 ms: consoleQueue stopped draining and the LD2
 * heartbeat froze, silencing diagnostics at precisely the moment
 * something worth diagnosing was happening.
 *
 * A 1 ms delay costs no useful latency. The card is not ready; polling it
 * 137000 times instead of 250 does not make it ready sooner, and the
 * typical 2-3 ms commit still completes in 2-3 polls.
 *
 * The chip select stays asserted across the delay, which is correct - SD
 * signals busy on DO with CS low. Nothing else can touch the bus meanwhile
 * because this task owns it structurally.
 */
#define SD_BUSY_POLL_INTERVAL_MS   1U

/*
 * Pre-scheduler fallback bound.
 *
 * Only reachable if a block operation is ever called from main before the
 * scheduler starts. It is not today - sd_init() does not use the busy
 * wait - so this exists to fail safe rather than hang, and its duration
 * is deliberately approximate.
 */
#define SD_PRESCHED_SPIN_LIMIT     200000UL

static int (*sd_card_detect)(void) = NULL;

void sd_set_card_detect(int (*present_fn)(void))
{
    sd_card_detect = present_fn;
}

const char *sd_status_str(SD_Status s)
{
    switch (s)
    {
        case SD_OK:                 return "OK";
        case SD_ERR_NO_CARD:        return "NO CARD (CMD0 no response)";
        case SD_ERR_CMD8:           return "CMD8 check pattern mismatch";
        case SD_ERR_ACMD41_TIMEOUT: return "ACMD41 timeout (card never left idle)";
        case SD_ERR_OCR_VOLTAGE:    return "OCR: 3.3V window unsupported";
        case SD_ERR_IO:             return "block I/O error";
        case SD_ERR_CD_ABSENT:      return "card-detect: slot empty";
        default:                    return "unknown";
    }
}

/*
 * Wait for the card to release DO (return 0xFF). Returns SD_OK if it did
 * so within timeout_ms, SD_ERR_IO otherwise.
 */
static SD_Status sd_wait_ready(uint32_t timeout_ms)
{
    if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING)
    {
        TickType_t start  = xTaskGetTickCount();
        TickType_t budget = pdMS_TO_TICKS(timeout_ms);

        for (;;)
        {
            if (spi_receive() == 0xFF)
            {
                return SD_OK;
            }

            if ((xTaskGetTickCount() - start) >= budget)
            {
                return SD_ERR_IO;
            }

            vTaskDelay(pdMS_TO_TICKS(SD_BUSY_POLL_INTERVAL_MS));
        }
    }

    /* No tick source yet - bounded spin, see the comment above. */
    for (uint32_t i = 0; i < SD_PRESCHED_SPIN_LIMIT; i++)
    {
        if (spi_receive() == 0xFF)
        {
            return SD_OK;
        }
    }

    return SD_ERR_IO;
}

/* Wait for a specific token rather than for not-busy. Same tick-based
 * budget and the same yield rationale. */
static SD_Status sd_wait_token(uint8_t token, uint32_t timeout_ms)
{
    if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING)
    {
        TickType_t start  = xTaskGetTickCount();
        TickType_t budget = pdMS_TO_TICKS(timeout_ms);

        for (;;)
        {
            if (spi_receive() == token)
            {
                return SD_OK;
            }

            if ((xTaskGetTickCount() - start) >= budget)
            {
                return SD_ERR_IO;
            }

            vTaskDelay(pdMS_TO_TICKS(SD_BUSY_POLL_INTERVAL_MS));
        }
    }

    for (uint32_t i = 0; i < SD_PRESCHED_SPIN_LIMIT; i++)
    {
        if (spi_receive() == token)
        {
            return SD_OK;
        }
    }

    return SD_ERR_IO;
}

static void sd_delay(void)
{
    for (volatile uint32_t i = 0; i < 8000; i++)
    {
    }
}

static uint8_t sd_send_cmd(uint8_t cmd, uint32_t arg, uint8_t crc)
{
    // 1. transmit command byte: 0x40 | cmd
    spi_transmit(0x40 | cmd);

    // 2. transmit arg bytes MSB first
    spi_transmit((arg >> 24) & 0xFF);
    spi_transmit((arg >> 16) & 0xFF);
    spi_transmit((arg >>  8) & 0xFF);
    spi_transmit((arg >>  0) & 0xFF);

    // 3. transmit crc with stop bit
    spi_transmit(crc | 0x01);

    // 4. poll up to 8 times for valid R1 (bit7 == 0)
    uint8_t resp = 0xFF;
    for (uint8_t i = 0; i < 8; i++) {
        resp = spi_receive();
        if (!(resp & 0x80)) break;
    }
    return resp;
}

SD_Status sd_init(void)
{
    /* Optional mechanical card detect, if one was ever registered. */
    if ((sd_card_detect != NULL) && (sd_card_detect() == 0))
    {
        return SD_ERR_CD_ABSENT;
    }

    /*
     * B7: the whole identification sequence must run inside the SD
     * specification's 100-400 kHz window. SPI_BR_SD_INIT is 351.6 kHz.
     * Previously this all ran at full speed, which some cards tolerate
     * and others silently refuse.
     */
    spi_set_baud(SPI_BR_SD_INIT);

    // 1. power stabilisation delay
    sd_delay();

    // 2. 80 dummy clocks with CS high to enter SPI mode
    spi_deselect_sd();
    for (uint8_t i = 0; i < 10; i++) spi_transmit(0xFF);

    // 3. CMD0 — software reset, enter SPI mode
    spi_select_sd();
    uint8_t r1 = sd_send_cmd(0, 0x00000000, 0x95);
    spi_deselect_sd();
    spi_transmit(0xFF);

    if (r1 != 0x01) return SD_ERR_NO_CARD;

    // 4. CMD8 — interface condition, check for v2 card
    spi_select_sd();
    r1 = sd_send_cmd(8, 0x000001AA, 0x87);
    uint8_t r7[4];
    for (uint8_t i = 0; i < 4; i++) r7[i] = spi_receive();
    spi_deselect_sd();
    spi_transmit(0xFF);

    /*
     * R7 must echo the voltage nibble and the 0xAA check pattern in its
     * last two bytes. These four bytes have to be clocked out either way
     * to keep the bus in sync; now the answer is actually inspected.
     */
    if ((r1 & 0x04) == 0)
    {
        /* Card understood CMD8, so it is v2.0+. Validate the echo. */
        if (((r7[2] & 0x0F) != 0x01) || (r7[3] != 0xAA))
        {
            return SD_ERR_CMD8;
        }
    }

    // 5. ACMD41 loop — wait for card to finish init
    uint32_t timeout = 1000;
    do {
        // CMD55 — APP_CMD prefix
        spi_select_sd();
        sd_send_cmd(55, 0x00000000, 0x65);
        spi_deselect_sd();
        spi_transmit(0xFF);

        // CMD41 — send operating condition, HCS bit set for SDHC
        spi_select_sd();
        r1 = sd_send_cmd(41, 0x40000000, 0x77);
        spi_deselect_sd();
        spi_transmit(0xFF);

        sd_delay();
    } while ((r1 & 0x01) && --timeout);

    if (timeout == 0) return SD_ERR_ACMD41_TIMEOUT;

    // 6. CMD58 — read OCR, verify 3.3V support
    spi_select_sd();
    r1 = sd_send_cmd(58, 0x00000000, 0xFD);
    uint8_t ocr[4];
    for (uint8_t i = 0; i < 4; i++) ocr[i] = spi_receive();
    spi_deselect_sd();
    spi_transmit(0xFF);

    if (r1 != 0x00) return SD_ERR_IO;
    if (!(ocr[1] & 0x30)) return SD_ERR_OCR_VOLTAGE;

    /*
     * Identification is done. Move to the data rate for everything that
     * follows. The bus owner re-asserts this per request type, but
     * leaving it correct here keeps sd_init self-contained.
     */
    spi_set_baud(SPI_BR_SD_DATA);

    return SD_OK;
}

SD_Status sd_write_block(uint32_t block_addr, const uint8_t *data)
{
    uint8_t r1;

    // SDSC cards need byte addressing
    // block_addr *= 512;

    spi_select_sd();

    // CMD24 = WRITE_BLOCK
    r1 = sd_send_cmd(24, block_addr, 0x01);

    if (r1 != 0x00)
    {
        spi_deselect_sd();
        spi_transmit(0xFF);
        return SD_ERR_IO;
    }

    // One byte gap before data token
    spi_transmit(0xFF);

    // Start block token
    spi_transmit(0xFE);

    // Send 512-byte sector
    for (uint16_t i = 0; i < 512; i++)
    {
        spi_transmit(data[i]);
    }

    // Dummy CRC (ignored unless CRC enabled)
    spi_transmit(0xFF);
    spi_transmit(0xFF);

    // Data response token
    uint8_t response = spi_receive();

    if ((response & 0x1F) != 0x05)
    {
        spi_deselect_sd();
        spi_transmit(0xFF);
        return SD_ERR_IO;
    }

    /*
     * Wait out the card's internal program cycle: tick-timed, yielding
     * between polls. See SD_WRITE_TIMEOUT_MS above.
     */
    SD_Status busy = sd_wait_ready(SD_WRITE_TIMEOUT_MS);

    spi_deselect_sd();
    spi_transmit(0xFF);

    return busy;
}

SD_Status sd_read_block(uint32_t block_addr, uint8_t *buf)
{
    uint8_t r1;

    // SDSC cards require byte addressing
    // block_addr *= 512;

    // 1. CS low, send CMD17
    spi_select_sd();

    r1 = sd_send_cmd(17, block_addr, 0x01);

    if (r1 != 0x00)
    {
        spi_deselect_sd();
        spi_transmit(0xFF);
        return SD_ERR_IO;
    }

    // 2. Wait for start token 0xFE
    if (sd_wait_token(0xFE, SD_READ_TIMEOUT_MS) != SD_OK)
    {
        spi_deselect_sd();
        spi_transmit(0xFF);
        return SD_ERR_IO;
    }

    // 3. Read 512-byte sector
    for (uint16_t i = 0; i < 512; i++)
    {
        buf[i] = spi_receive();
    }

    // 4. Discard CRC16
    spi_receive();
    spi_receive();

    spi_deselect_sd();
    spi_transmit(0xFF);

    return SD_OK;
}
