// SD card over SPI2 (SPI mode), single-block reads and writes.
// After the scheduler starts only spi_owner_task calls this.

#include "sd_card.h"
#include "spi.h"

#include "FreeRTOS.h"
#include "task.h"
#include "timer_timebase.h"

#include <stddef.h>
#include <stdbool.h>

// The SD spec allows a single-block write up to 250 ms (wear levelling);
// a typical one takes 2-3 ms.
#define SD_WRITE_TIMEOUT_MS        250U
#define SD_READ_TIMEOUT_MS         100U

// While the card is busy, sleep 1 ms between polls instead of spinning,
// so the tasks below this one keep running during a long write.
#define SD_BUSY_POLL_INTERVAL_MS   1U

#define SD_US_PER_MS               1000UL

static int (*sd_card_detect)(void) = NULL;

// SDSC cards (2 GB and under) take a byte address, SDHC/SDXC a block
// number. Set from the OCR's CCS bit in sd_init().
static bool byte_addressing = false;

void sd_set_card_detect(int (*present_fn)(void))
{
    sd_card_detect = present_fn;
}

bool sd_is_sdhc(void)
{
    return !byte_addressing;
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

static uint32_t to_card_address(uint32_t block)
{
    return byte_addressing ? (block * 512U) : block;
}

// Before the scheduler there is no tick to time out on, so these waits use
// the TIM2 microsecond counter instead.
static SD_Status sd_wait_presched(uint8_t token, uint32_t timeout_ms)
{
    uint32_t start   = micros();
    uint32_t budget  = timeout_ms * SD_US_PER_MS;

    for (;;)
    {
        if (spi_receive() == token)
        {
            return SD_OK;
        }

        if ((micros() - start) >= budget)
        {
            return SD_ERR_IO;
        }
    }
}

// Wait for the card to stop signalling busy (it returns 0xFF when ready).
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

    return sd_wait_presched(0xFF, timeout_ms);
}

// Wait for a specific token, e.g. the 0xFE that starts a data block.
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

    return sd_wait_presched(token, timeout_ms);
}

static void sd_delay(void)
{
    for (volatile uint32_t i = 0; i < 8000; i++)
    {
    }
}

static uint8_t sd_send_cmd(uint8_t cmd, uint32_t arg, uint8_t crc)
{
    spi_transmit(0x40 | cmd);

    spi_transmit((arg >> 24) & 0xFF);
    spi_transmit((arg >> 16) & 0xFF);
    spi_transmit((arg >>  8) & 0xFF);
    spi_transmit((arg >>  0) & 0xFF);

    spi_transmit(crc | 0x01);   // CRC + stop bit

    // R1 arrives within 8 bytes; bit 7 clear marks it
    uint8_t resp = 0xFF;
    for (uint8_t i = 0; i < 8; i++) {
        resp = spi_receive();
        if (!(resp & 0x80)) break;
    }
    return resp;
}

SD_Status sd_init(void)
{
    if ((sd_card_detect != NULL) && (sd_card_detect() == 0))
    {
        return SD_ERR_CD_ABSENT;
    }

    // identification must run at 100-400 kHz
    spi_set_baud(SPI_BR_SD_INIT);

    sd_delay();

    // 80 clocks with CS high puts the card in SPI mode
    spi_deselect_sd();
    for (uint8_t i = 0; i < 10; i++) spi_transmit(0xFF);

    // CMD0: reset
    spi_select_sd();
    uint8_t r1 = sd_send_cmd(0, 0x00000000, 0x95);
    spi_deselect_sd();
    spi_transmit(0xFF);

    if (r1 != 0x01) return SD_ERR_NO_CARD;

    // CMD8: voltage check. A v2+ card echoes the 0xAA pattern; a v1 card
    // rejects the command (illegal-command bit set).
    spi_select_sd();
    r1 = sd_send_cmd(8, 0x000001AA, 0x87);
    uint8_t r7[4];
    for (uint8_t i = 0; i < 4; i++) r7[i] = spi_receive();
    spi_deselect_sd();
    spi_transmit(0xFF);

    if ((r1 & 0x04) == 0)
    {
        if (((r7[2] & 0x0F) != 0x01) || (r7[3] != 0xAA))
        {
            return SD_ERR_CMD8;
        }
    }

    // ACMD41 (CMD55 + CMD41) until the card leaves idle. HCS set: we
    // accept high-capacity cards.
    uint32_t timeout = 1000;
    do {
        spi_select_sd();
        sd_send_cmd(55, 0x00000000, 0x65);
        spi_deselect_sd();
        spi_transmit(0xFF);

        spi_select_sd();
        r1 = sd_send_cmd(41, 0x40000000, 0x77);
        spi_deselect_sd();
        spi_transmit(0xFF);

        sd_delay();
    } while ((r1 & 0x01) && --timeout);

    if (timeout == 0) return SD_ERR_ACMD41_TIMEOUT;

    // CMD58: read OCR. Bits 20-21 (ocr[1] & 0x30) = 3.2-3.4 V supported.
    // Bit 30 (ocr[0] & 0x40) = CCS: block addressing (SDHC/SDXC).
    spi_select_sd();
    r1 = sd_send_cmd(58, 0x00000000, 0xFD);
    uint8_t ocr[4];
    for (uint8_t i = 0; i < 4; i++) ocr[i] = spi_receive();
    spi_deselect_sd();
    spi_transmit(0xFF);

    if (r1 != 0x00) return SD_ERR_IO;
    if (!(ocr[1] & 0x30)) return SD_ERR_OCR_VOLTAGE;

    byte_addressing = ((ocr[0] & 0x40) == 0);

    if (byte_addressing)
    {
        // CMD16: SDSC cards need the block length set to 512 bytes
        spi_select_sd();
        r1 = sd_send_cmd(16, 512U, 0x01);
        spi_deselect_sd();
        spi_transmit(0xFF);

        if (r1 != 0x00) return SD_ERR_IO;
    }

    spi_set_baud(SPI_BR_SD_DATA);

    return SD_OK;
}

SD_Status sd_write_block(uint32_t block_addr, const uint8_t *data)
{
    uint8_t r1;

    spi_select_sd();

    r1 = sd_send_cmd(24, to_card_address(block_addr), 0x01);   // WRITE_BLOCK

    if (r1 != 0x00)
    {
        spi_deselect_sd();
        spi_transmit(0xFF);
        return SD_ERR_IO;
    }

    spi_transmit(0xFF);     // one byte gap
    spi_transmit(0xFE);     // start-of-data token

    for (uint16_t i = 0; i < 512; i++)
    {
        spi_transmit(data[i]);
    }

    // dummy CRC (CRC checking is off in SPI mode)
    spi_transmit(0xFF);
    spi_transmit(0xFF);

    // data response: xxx0 0101 = accepted
    uint8_t response = spi_receive();

    if ((response & 0x1F) != 0x05)
    {
        spi_deselect_sd();
        spi_transmit(0xFF);
        return SD_ERR_IO;
    }

    // the card holds MISO low while it programs the block
    SD_Status busy = sd_wait_ready(SD_WRITE_TIMEOUT_MS);

    spi_deselect_sd();
    spi_transmit(0xFF);

    return busy;
}

SD_Status sd_read_block(uint32_t block_addr, uint8_t *buf)
{
    uint8_t r1;

    spi_select_sd();

    r1 = sd_send_cmd(17, to_card_address(block_addr), 0x01);   // READ_SINGLE_BLOCK

    if (r1 != 0x00)
    {
        spi_deselect_sd();
        spi_transmit(0xFF);
        return SD_ERR_IO;
    }

    if (sd_wait_token(0xFE, SD_READ_TIMEOUT_MS) != SD_OK)
    {
        spi_deselect_sd();
        spi_transmit(0xFF);
        return SD_ERR_IO;
    }

    for (uint16_t i = 0; i < 512; i++)
    {
        buf[i] = spi_receive();
    }

    // discard the CRC
    spi_receive();
    spi_receive();

    spi_deselect_sd();
    spi_transmit(0xFF);

    return SD_OK;
}
