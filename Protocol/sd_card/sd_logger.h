// sd_logger.h

#ifndef SD_LOGGER_H
#define SD_LOGGER_H

#include <stdint.h>
#include "sd_card.h"


typedef struct __attribute__((packed))
{
    uint32_t timestamp_ms;   // FreeRTOS tick at record time

    float depth_m;

    float roll_deg;
    float pitch_deg;
    float yaw_deg;

    uint16_t pwm[8];         // 8 thrusters

    uint8_t armed;           // 0 = disarmed, 1 = armed
    uint8_t link_ok;         // 0 = link lost, 1 = link healthy

    uint16_t crc16;          // truncated hardware CRC32, low 16 bits

} LogRecord;

/*
 * First block used for log data. Blocks below this are left alone so a
 * partition table or boot sector, if the card has one, is not stomped.
 */
#define SD_LOG_FIRST_BLOCK   100UL

// Reset the block cursor.
void sd_logger_init(void);

/*
 * Block address for a given block sequence number. logging_task assembles
 * whole 512-byte blocks itself, so the logger's job is now just the
 * address mapping - the per-record write path is gone.
 */
uint32_t sd_logger_block_for_seq(uint32_t seq);

#endif
