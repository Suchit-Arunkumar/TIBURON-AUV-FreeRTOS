#ifndef DVL_H
#define DVL_H

#include <stdint.h>
#include <stdbool.h>

typedef struct
{
    float vx;
    float vy;
    float vz;

    uint16_t status;
    uint16_t bit;

    uint32_t timestamp_ms;

    bool valid;

} DVLData;


/*
 * Feed one byte into DVL parser.
 */
void dvl_feed_byte(uint8_t byte);


/*
 * Get latest valid DVL measurement.
 */
bool dvl_get_data(DVLData *out);

#endif
