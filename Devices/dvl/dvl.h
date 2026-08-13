#ifndef DVL_H
#define DVL_H

#include <stdint.h>
#include <stdbool.h>

#define DVL_PACKET_LENGTH 116U

typedef struct
{
    /* Wayfinder instrument coordinate system */
    uint8_t coordinate_system;

    /* Bottom-track velocity [m/s] */
    float vx;
    float vy;
    float vz;
    float velocity_error;

    /* Range to bottom [m] */
    float range_beam1;
    float range_beam2;
    float range_beam3;
    float range_beam4;
    float mean_range;

    /* Speed of sound used by DVL [m/s] */
    float speed_of_sound;

    /* Wayfinder status */
    uint16_t status;

    /* Built-in-test status */
    uint16_t bit;

    /* Electrical measurements */
    float input_voltage;
    float transmit_voltage;
    float transmit_current;

    /* Wayfinder RTC timestamp */
    uint8_t year;
    uint8_t month;
    uint8_t day;
    uint8_t hour;
    uint8_t minute;
    uint8_t second;
    uint16_t milliseconds;

    /* STM32 reception timestamp */
    uint32_t timestamp_ms;

    /*
     * True only when the packet and velocity data
     * have passed validation.
     */
    bool valid;

} DVLData;


/*
 * Feed one byte into the Wayfinder parser.
 *
 * Returns true when a complete valid measurement
 * has been decoded.
 */
bool dvl_feed_byte(uint8_t byte);


/*
 * Copy the latest valid DVL measurement.
 */
bool dvl_get_data(DVLData *out);

#endif
