#include "sd_card.h"
#include "spi.h"

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
 * The previous code capped the busy poll at 100000 loop iterations. At
 * SPI_BR_SD_DATA that is roughly 110 ms of real time - LESS THAN HALF the
 * spec allowance - so a card that was merely slow got reported as failed.
 *
 * This is expressed in polls derived from the byte time so the meaning
 * survives a change of SPI clock, rather than as a bare loop count.
 */
#define SD_WRITE_TIMEOUT_MS        250U
#define SD_READ_TIMEOUT_MS         100U

/*
 * One busy poll clocks one byte: 8/f_SCK plus ~0.4 us of polled driver
 * overhead. At SPI_BR_SD_DATA (5.625 MHz) that is 1.422 + 0.40 = 1.822 us.
 *
 * Expressed in ns and divided out, so the timeout means the wall-clock
 * duration it claims to. Using a rounded 1 us instead would stretch a
 * 250 ms budget to 455 ms of real bus hold on a dead card - which is the
 * exact number the Phase 8 worst-case analysis quotes, so it has to be
 * right rather than merely conservative.
 *
 * If SPI_BR_SD_DATA is raised to SPI_BR_DIV4, update this to 1111.
 */
#define SD_POLL_NS                 1822U
#define SD_TIMEOUT_POLLS(ms)       (((ms) * 1000000UL) / SD_POLL_NS)

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
    uint32_t timeout;

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
     * Wait out the card's internal program cycle. Now sized to the
     * specification's 250 ms allowance instead of the previous ~110 ms,
     * which failed slow-but-healthy cards.
     */
    timeout = SD_TIMEOUT_POLLS(SD_WRITE_TIMEOUT_MS);

    while (--timeout)
    {
        if (spi_receive() == 0xFF)
            break;
    }

    spi_deselect_sd();
    spi_transmit(0xFF);

    if (timeout == 0)
        return SD_ERR_IO;

    return SD_OK;
}

SD_Status sd_read_block(uint32_t block_addr, uint8_t *buf)
{
    uint8_t r1;
    uint32_t timeout;

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
    timeout = SD_TIMEOUT_POLLS(SD_READ_TIMEOUT_MS);

    while (--timeout)
    {
        if (spi_receive() == 0xFE)
            break;
    }

    if (timeout == 0)
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
