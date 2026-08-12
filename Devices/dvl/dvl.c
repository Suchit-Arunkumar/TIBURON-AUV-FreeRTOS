#include "dvl.h"

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"


/*
 * ===========================================================================================================================
 * WAYFINDER DVL PROTOCOL
 *
 * Model: TBD
 *
 * IMPORTANT:
 * These definitions are provisional.
 * Verify against the exact Wayfinder model/interface manual.
 * ===========================================================================================================================
 */

#define DVL_START_BYTE          0xAAU

#define DVL_HEADER_SIZE         4U
#define DVL_CHECKSUM_SIZE       2U

#define DVL_LENGTH_OFFSET       3U

#define DVL_MAX_PACKET_SIZE     512U


typedef enum
{
    DVL_WAIT_START = 0,
    DVL_READ_HEADER,
    DVL_COLLECT_PACKET

} DVLParserState;


static uint8_t dvl_packet[DVL_MAX_PACKET_SIZE];

static uint16_t dvl_index = 0;
static uint16_t dvl_expected_length = 0;

static DVLParserState dvl_state =
    DVL_WAIT_START;

static DVLData dvl_latest;


//===========================================================================================================================
static void dvl_parser_reset(void)
{
    dvl_index = 0;
    dvl_expected_length = 0;

    dvl_state = DVL_WAIT_START;
}


//===========================================================================================================================
static uint16_t dvl_checksum(
    const uint8_t *data,
    uint16_t length
)
{
    /*
     * TODO:
     * Replace with exact Wayfinder checksum algorithm.
     */
    uint32_t sum = 0;

    for (uint16_t i = 0; i < length; i++)
    {
        sum += data[i];
    }

    return (uint16_t)sum;
}


//===========================================================================================================================
static bool dvl_validate_packet(void)
{
    if (dvl_index < DVL_CHECKSUM_SIZE)
    {
        return false;
    }

    /*
     * TODO:
     * Verify exact checksum byte order and coverage.
     */
    uint16_t received_checksum =
        ((uint16_t)dvl_packet[dvl_index - 2U]) |
        ((uint16_t)dvl_packet[dvl_index - 1U] << 8U);

    uint16_t calculated_checksum =
        dvl_checksum(
            dvl_packet,
            dvl_index - DVL_CHECKSUM_SIZE
        );

    return calculated_checksum ==
           received_checksum;
}


//===========================================================================================================================
static bool dvl_parse_packet(void)
{
    /*
     * Model-specific velocity fields are TBD.
     *
     * DO NOT mark dvl_latest.valid = true until the exact
     * Wayfinder payload has been verified.
     */

    float vx = 0.0f;
    float vy = 0.0f;
    float vz = 0.0f;

    uint16_t status = 0;
    uint16_t bit = 0;

    (void)vx;
    (void)vy;
    (void)vz;
    (void)status;
    (void)bit;

    return false;
}


//===========================================================================================================================
void dvl_feed_byte(uint8_t byte)
{
    switch (dvl_state)
    {
        case DVL_WAIT_START:

            if (byte == DVL_START_BYTE)
            {
                dvl_packet[0] = byte;

                dvl_index = 1U;

                dvl_state = DVL_READ_HEADER;
            }

            break;


        case DVL_READ_HEADER:

            if (dvl_index >= DVL_MAX_PACKET_SIZE)
            {
                dvl_parser_reset();
                break;
            }

            dvl_packet[dvl_index++] = byte;

            if (dvl_index >= DVL_HEADER_SIZE)
            {
                dvl_expected_length =
                    dvl_packet[DVL_LENGTH_OFFSET];

                if (dvl_expected_length <
                    DVL_HEADER_SIZE)
                {
                    dvl_parser_reset();
                    break;
                }

                if (dvl_expected_length >
                    DVL_MAX_PACKET_SIZE)
                {
                    dvl_parser_reset();
                    break;
                }

                dvl_state =
                    DVL_COLLECT_PACKET;
            }

            break;


        case DVL_COLLECT_PACKET:

            if (dvl_index >= DVL_MAX_PACKET_SIZE)
            {
                dvl_parser_reset();
                break;
            }

            dvl_packet[dvl_index++] = byte;

            if (dvl_index >= dvl_expected_length)
            {
                if (dvl_validate_packet())
                {
                    (void)dvl_parse_packet();
                }

                dvl_parser_reset();
            }

            break;


        default:

            dvl_parser_reset();

            break;
    }
}


//===========================================================================================================================
bool dvl_get_data(DVLData *out)
{
    if (out == NULL)
    {
        return false;
    }

    bool valid;

    taskENTER_CRITICAL();

    valid = dvl_latest.valid;

    if (valid)
    {
        *out = dvl_latest;
    }

    taskEXIT_CRITICAL();

    return valid;
}
