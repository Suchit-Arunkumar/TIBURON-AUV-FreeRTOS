#ifndef SD_LOGGER_H
#define SD_LOGGER_H

#include <stdint.h>
#include "sd_card.h"

/*
 * Log layout on the card: raw 512-byte blocks from SD_LOG_FIRST_BLOCK on,
 * no filesystem. Each block is a 16-byte header plus as many whole records
 * as fit. A new run starts after the last block of the previous one, with
 * seq starting again at 0, so runs can be told apart offline.
 */

/* Blocks below this are left alone (partition table, boot sector). */
#define SD_LOG_FIRST_BLOCK   100UL

/* How far the start-up search looks: 2^22 blocks = 2 GB. */
#define SD_LOG_MAX_BLOCKS    (1UL << 22)

#define LOG_BLOCK_MAGIC      0x54424C31UL   /* "TBL1" */

typedef struct __attribute__((packed))
{
    uint32_t magic;
    uint32_t seq;              /* block number within this run, from 0 */
    uint16_t record_count;     /* valid records in this block          */
    uint16_t record_size;      /* sizeof(LogRecord), for offline parse */
    uint32_t first_timestamp;  /* tick of the first record             */
} LogBlockHeader;

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

    // The board's own sensors (0 / source 0 when missing or stale)
    uint8_t imu_source;      // SensorSource: 1 VN-200, 2 BNO085
    uint8_t depth_source;    // 3 Bar30, 4 analog
    uint8_t dvl_valid;       // 1 = bottom lock
    uint8_t reserved;

    float imu_yaw_deg;
    float imu_pitch_deg;
    float imu_roll_deg;
    float sensor_depth_m;
    float dvl_vx_m_s;
    float dvl_vy_m_s;

    uint16_t crc16;          // CRC-16-CCITT over the fields above

} LogRecord;

#define LOG_RECORDS_PER_BLOCK \
    ((512U - sizeof(LogBlockHeader)) / sizeof(LogRecord))

/*
 * Find where this run should start writing: the first block after
 * SD_LOG_FIRST_BLOCK that doesn't hold a log block. Binary search, so
 * about 22 block reads. Assumes earlier runs were written as one unbroken
 * run of blocks (true unless a write failed). Call once, after sd_init()
 * and before the scheduler starts.
 */
uint32_t sd_logger_find_start(void);

/* Set the block that seq 0 of this run goes to. */
void sd_logger_init(uint32_t start_block);

uint32_t sd_logger_block_for_seq(uint32_t seq);

#endif
