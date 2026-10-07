#include "sd_logger.h"

static uint32_t base_block = SD_LOG_FIRST_BLOCK;

// Used only at start-up, before the scheduler.
static uint8_t probe[512];

static int block_in_use(uint32_t block)
{
    if (sd_read_block(block, probe) != SD_OK)
    {
        return 0;   // unreadable (e.g. past the end of the card): free
    }

    uint32_t magic = (uint32_t)probe[0] | ((uint32_t)probe[1] << 8) |
                     ((uint32_t)probe[2] << 16) | ((uint32_t)probe[3] << 24);

    return magic == LOG_BLOCK_MAGIC;
}

uint32_t sd_logger_find_start(void)
{
    // Blocks [first, end) are used and the rest are free, so binary search
    // for the boundary instead of reading every block.
    uint32_t lo = SD_LOG_FIRST_BLOCK;
    uint32_t hi = SD_LOG_FIRST_BLOCK + SD_LOG_MAX_BLOCKS;

    while (lo < hi)
    {
        uint32_t mid = lo + (hi - lo) / 2U;

        if (block_in_use(mid))
        {
            lo = mid + 1U;
        }
        else
        {
            hi = mid;
        }
    }

    return lo;
}

void sd_logger_init(uint32_t start_block)
{
    base_block = start_block;
}

uint32_t sd_logger_block_for_seq(uint32_t seq)
{
    return base_block + seq;
}
