#include "sd_logger.h"

/*
 * The old sd_logger_write() wrote one 512-byte block per 40-byte record:
 * 12.8x write amplification and one program cycle per record. Records are
 * now batched into full blocks by logging_task, so all that remains here
 * is the sequence-to-address mapping.
 */

static uint32_t base_block = SD_LOG_FIRST_BLOCK;

void sd_logger_init(void)
{
    base_block = SD_LOG_FIRST_BLOCK;
}

uint32_t sd_logger_block_for_seq(uint32_t seq)
{
    return base_block + seq;
}
