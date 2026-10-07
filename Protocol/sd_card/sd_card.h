#ifndef SD_CARD_H
#define SD_CARD_H

#include <stdint.h>
#include <stdbool.h>

/*
 * Distinct failure codes.
 *
 * sd_init() used to return a bare SD_FAIL for every failure, which made
 * "no card in the slot" indistinguishable from "card present but the
 * voltage window is wrong". These separate the cases that need different
 * responses from the bench.
 *
 * There is no card-detect GPIO wired: typical micro-SD breakouts expose
 * only CS/MOSI/MISO/SCK/VCC/GND. SD_ERR_NO_CARD infers absence from CMD0
 * never producing a valid R1 - nothing is driving the bus. If a breakout
 * with a mechanical CD switch is fitted later, sd_set_card_detect() is
 * the single place to hook it; nothing else in the state machine changes.
 */
typedef enum
{
    SD_OK                  = 0,

    /* CMD0 never returned 0x01. Either the slot is empty, or wiring/power
     * is wrong. Nothing is responding on the bus at all. */
    SD_ERR_NO_CARD         = 1,

    /* CMD8 returned an R7 that does not echo the 0xAA check pattern.
     * A v1.x card, or a signalling problem. */
    SD_ERR_CMD8            = 2,

    /* ACMD41 never cleared the idle bit within the retry budget. The card
     * is present and talking but never finished initialising. */
    SD_ERR_ACMD41_TIMEOUT  = 3,

    /* CMD58's OCR does not report support for the 3.2-3.4 V window. */
    SD_ERR_OCR_VOLTAGE     = 4,

    /* A block operation failed: bad command response, missing data token,
     * or the card stayed busy past the write timeout. */
    SD_ERR_IO              = 5,

    /* A card-detect switch, if one is ever wired, says the slot is
     * empty. Never returned today. */
    SD_ERR_CD_ABSENT       = 6
} SD_Status;

/* Human-readable form, for the console. */
const char *sd_status_str(SD_Status s);

/*
 * Runs CMD0/CMD8/ACMD41/CMD58 at SPI_BR_SD_INIT (351.6 kHz), then leaves
 * the bus at SPI_BR_SD_DATA for subsequent block operations. Fixes the
 * B7 defect where the whole init sequence ran at full speed, outside the
 * 100-400 kHz window the SD specification mandates.
 */
SD_Status sd_init(void);

SD_Status sd_read_block(uint32_t block_addr, uint8_t *buf);

SD_Status sd_write_block(uint32_t block_addr, const uint8_t *data);

/* True for SDHC/SDXC (block addressing). Block numbers passed to the
 * read/write functions are converted for older SDSC cards. */
bool sd_is_sdhc(void);

/*
 * Optional card-detect hook. Pass a function returning 1 when a card is
 * present, or NULL (the default) to disable the check. Provided so a
 * breakout with a CD switch can be adopted in one line rather than by
 * restructuring sd_init().
 */
void sd_set_card_detect(int (*present_fn)(void));

#endif
