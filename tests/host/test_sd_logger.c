/*
 * Host test for the start-of-log search (Protocol/sd_card/sd_logger.c),
 * against a fake card where the first `used` log blocks hold the magic.
 */
#include "test.h"
#include "sd_logger.h"

#include <string.h>

static uint32_t used;           /* log blocks already written      */
static uint32_t card_blocks;    /* reads past this fail, like a real card */
static int      reads;

SD_Status sd_read_block(uint32_t block, uint8_t *buf)
{
    reads++;
    memset(buf, 0, 512);

    if (block >= card_blocks)
    {
        return SD_ERR_IO;
    }

    if ((block >= SD_LOG_FIRST_BLOCK) && (block < SD_LOG_FIRST_BLOCK + used))
    {
        uint32_t m = LOG_BLOCK_MAGIC;
        memcpy(buf, &m, 4);
    }

    return SD_OK;
}

static uint32_t search(uint32_t n_used, uint32_t n_card)
{
    used = n_used;
    card_blocks = n_card;
    reads = 0;
    return sd_logger_find_start();
}

int main(void)
{
    const uint32_t big = 16u * 1024u * 1024u;   /* 8 GB card */

    /* Blank card: start at the first log block. */
    CHECK(search(0, big) == SD_LOG_FIRST_BLOCK);

    /* After 1, 2, and a few thousand blocks of earlier runs. */
    CHECK(search(1, big) == SD_LOG_FIRST_BLOCK + 1);
    CHECK(search(2, big) == SD_LOG_FIRST_BLOCK + 2);
    CHECK(search(4321, big) == SD_LOG_FIRST_BLOCK + 4321);

    /* Fast: about log2(2^22) reads, not one per block. */
    CHECK(reads <= 23);

    /* A small card (reads past its end fail) still works. */
    CHECK(search(500, 1000) == SD_LOG_FIRST_BLOCK + 500);

    /* Card full up to the search limit. */
    CHECK(search(SD_LOG_MAX_BLOCKS, big) == SD_LOG_FIRST_BLOCK + SD_LOG_MAX_BLOCKS);

    /* The block mapping uses the start found. */
    sd_logger_init(SD_LOG_FIRST_BLOCK + 4321);
    CHECK(sd_logger_block_for_seq(0) == SD_LOG_FIRST_BLOCK + 4321);
    CHECK(sd_logger_block_for_seq(7) == SD_LOG_FIRST_BLOCK + 4328);

    return test_summary("sd_logger");
}
