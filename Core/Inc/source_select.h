#ifndef SOURCE_SELECT_H
#define SOURCE_SELECT_H

#include <stdint.h>
#include <stdbool.h>

/*
 * Choose between a preferred sensor and a backup for the same quantity
 * (VN-200 / BNO085 for attitude, Bar30 / ADC for depth).
 *
 *   - A sensor is fresh if its last reading is at most SOURCE_STALE_MS old.
 *   - Use the preferred sensor whenever it is fresh.
 *   - If it goes stale, switch to the backup (if that is fresh).
 *   - Switch back only after the preferred sensor has been fresh for
 *     SOURCE_RETURN_MS without a break, so a flaky connection does not
 *     make the output flip back and forth every few readings.
 *   - Neither fresh: SRC_NONE.
 *
 * Pure logic, no RTOS, so it is tested on the PC
 * (tests/host/test_source_select.c).
 */

#define SOURCE_STALE_MS    200U
#define SOURCE_RETURN_MS  1000U

typedef struct
{
    uint8_t  preferred;
    uint8_t  backup;
    uint8_t  active;
    bool     preferred_was_fresh;
    uint32_t preferred_fresh_since;
} SourceSelect;

void source_select_init(SourceSelect *s, uint8_t preferred, uint8_t backup);

/*
 * now_ms: current time. *_seen: a reading has ever arrived. *_last_ms:
 * when the latest reading arrived. Returns the source to use.
 */
uint8_t source_select_update(SourceSelect *s, uint32_t now_ms,
                             bool preferred_seen, uint32_t preferred_last_ms,
                             bool backup_seen,    uint32_t backup_last_ms);

#endif
