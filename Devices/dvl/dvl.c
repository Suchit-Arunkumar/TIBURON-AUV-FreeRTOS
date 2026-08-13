#include "dvl.h"

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>

#include "FreeRTOS.h"
#include "task.h"


/*
 * ===========================================================================================================================
 * TELEDYNE RDI WAYFINDER
 *
 * Binary Data Output
 *
 * Official packet:
 *
 *   SOP ID   = AA 10 01 74 00 10
 *   Data ID  = 05 6D 00 AA 11 69 00 00 00
 *   Length   = 116 bytes total
 *
 * All multi-byte values are little-endian.
 * ===========================================================================================================================
 */

#define DVL_PACKET_LENGTH           116U

#define DVL_SOP_LENGTH              6U
#define DVL_DATA_ID_LENGTH          9U


/*
 * Fixed Wayfinder SOP.
 */
static const uint8_t dvl_sop[DVL_SOP_LENGTH] =
{
    0xAAU,
    0x10U,
    0x01U,
    0x74U,
    0x00U,
    0x10U
};


/*
 * Fixed Wayfinder Binary Data Output ID.
 */
static const uint8_t dvl_data_id[DVL_DATA_ID_LENGTH] =
{
    0x05U,
    0x6DU,
    0x00U,
    0xAAU,
    0x11U,
    0x69U,
    0x00U,
    0x00U,
    0x00U
};


/*
 * Wayfinder system type.
 */
#define DVL_SYSTEM_TYPE             0x4CU


/*===========================================================================================================================
 * Packet offsets
 * ===========================================================================================================================*/

#define DVL_OFFSET_SYSTEM_TYPE          15U
#define DVL_OFFSET_SYSTEM_SUBTYPE       16U

#define DVL_OFFSET_FW_MAJOR             17U
#define DVL_OFFSET_FW_MINOR             18U
#define DVL_OFFSET_FW_PATCH             19U
#define DVL_OFFSET_FW_BUILD             20U

#define DVL_OFFSET_YEAR                 21U
#define DVL_OFFSET_MONTH                22U
#define DVL_OFFSET_DAY                  23U
#define DVL_OFFSET_HOUR                 24U
#define DVL_OFFSET_MINUTE               25U
#define DVL_OFFSET_SECOND               26U

#define DVL_OFFSET_MILLISECONDS         27U

#define DVL_OFFSET_COORDINATE_SYSTEM    29U

#define DVL_OFFSET_VEL_X                30U
#define DVL_OFFSET_VEL_Y                34U
#define DVL_OFFSET_VEL_Z                38U
#define DVL_OFFSET_VEL_ERROR            42U

#define DVL_OFFSET_RANGE1               46U
#define DVL_OFFSET_RANGE2               50U
#define DVL_OFFSET_RANGE3               54U
#define DVL_OFFSET_RANGE4               58U

#define DVL_OFFSET_MEAN_RANGE           62U
#define DVL_OFFSET_SPEED_OF_SOUND       66U

#define DVL_OFFSET_STATUS               70U
#define DVL_OFFSET_BIT                  72U

#define DVL_OFFSET_INPUT_VOLTAGE        74U
#define DVL_OFFSET_TRANSMIT_VOLTAGE     78U
#define DVL_OFFSET_TRANSMIT_CURRENT     82U

#define DVL_OFFSET_SERIAL               86U

#define DVL_OFFSET_RESERVED             92U

#define DVL_OFFSET_CHECKSUM_DATA        112U
#define DVL_OFFSET_CHECKSUM             114U


/*===========================================================================================================================
 * Parser state
 * ===========================================================================================================================*/

typedef enum
{
    DVL_WAIT_SOP = 0,
    DVL_COLLECT_PACKET

} DVLParserState;


static uint8_t dvl_packet[DVL_PACKET_LENGTH];

static uint16_t dvl_index = 0U;

static uint8_t dvl_sop_index = 0U;

static DVLParserState dvl_state =
    DVL_WAIT_SOP;

static DVLData dvl_latest;


/*===========================================================================================================================
 * Read little-endian uint16
 * ===========================================================================================================================*/
static uint16_t dvl_read_u16(const uint8_t *p)
{
    return ((uint16_t)p[0]) |
           ((uint16_t)p[1] << 8U);
}


/*===========================================================================================================================
 * Read little-endian float32
 * ===========================================================================================================================*/
static float dvl_read_float(const uint8_t *p)
{
    uint32_t raw =
        ((uint32_t)p[0]) |
        ((uint32_t)p[1] << 8U) |
        ((uint32_t)p[2] << 16U) |
        ((uint32_t)p[3] << 24U);

    float value;

    memcpy(&value, &raw, sizeof(value));

    return value;
}


/*===========================================================================================================================
 * Wayfinder checksum
 *
 * Sum of all non-checksum bytes.
 * Rollover is ignored.
 *
 * The final checksum occupies bytes 114-115.
 * ===========================================================================================================================*/
static uint16_t dvl_checksum(
    const uint8_t *data,
    uint16_t length
)
{
    uint32_t sum = 0U;

    for (uint16_t i = 0U; i < length; i++)
    {
        sum += data[i];
    }

    return (uint16_t)sum;
}


/*===========================================================================================================================
 * Reset parser
 * ===========================================================================================================================*/
static void dvl_parser_reset(void)
{
    dvl_index = 0U;
    dvl_sop_index = 0U;

    dvl_state = DVL_WAIT_SOP;
}


/*===========================================================================================================================
 * Validate complete packet
 * ===========================================================================================================================*/
static bool dvl_validate_packet(void)
{
    /*
     * Validate SOP.
     */
    for (uint16_t i = 0U; i < DVL_SOP_LENGTH; i++)
    {
        if (dvl_packet[i] != dvl_sop[i])
        {
            return false;
        }
    }


    /*
     * Validate Data ID.
     */
    for (uint16_t i = 0U; i < DVL_DATA_ID_LENGTH; i++)
    {
        if (dvl_packet[DVL_SOP_LENGTH + i] !=
            dvl_data_id[i])
        {
            return false;
        }
    }


    /*
     * Validate Wayfinder system type.
     */
    if (dvl_packet[DVL_OFFSET_SYSTEM_TYPE] !=
        DVL_SYSTEM_TYPE)
    {
        return false;
    }


    /*
     * Received final checksum.
     */
    uint16_t received_checksum =
        dvl_read_u16(
            &dvl_packet[DVL_OFFSET_CHECKSUM]
        );


    /*
     * Calculate checksum over all bytes before
     * the final checksum.
     */
    uint16_t calculated_checksum =
        dvl_checksum(
            dvl_packet,
            DVL_OFFSET_CHECKSUM
        );


    return calculated_checksum ==
           received_checksum;
}


/*===========================================================================================================================
 * Parse validated packet
 * ===========================================================================================================================*/
static bool dvl_parse_packet(void)
{
    if (!dvl_validate_packet())
    {
        return false;
    }


    DVLData data;

    memset(&data, 0, sizeof(data));


    /*
     * Coordinate system.
     */
    data.coordinate_system =
        dvl_packet[DVL_OFFSET_COORDINATE_SYSTEM];


    /*
     * Wayfinder RTC.
     */
    data.year =
        dvl_packet[DVL_OFFSET_YEAR];

    data.month =
        dvl_packet[DVL_OFFSET_MONTH];

    data.day =
        dvl_packet[DVL_OFFSET_DAY];

    data.hour =
        dvl_packet[DVL_OFFSET_HOUR];

    data.minute =
        dvl_packet[DVL_OFFSET_MINUTE];

    data.second =
        dvl_packet[DVL_OFFSET_SECOND];

    data.milliseconds =
        dvl_read_u16(
            &dvl_packet[DVL_OFFSET_MILLISECONDS]
        );


    /*
     * Bottom-track velocity.
     */
    data.vx =
        dvl_read_float(
            &dvl_packet[DVL_OFFSET_VEL_X]
        );

    data.vy =
        dvl_read_float(
            &dvl_packet[DVL_OFFSET_VEL_Y]
        );

    data.vz =
        dvl_read_float(
            &dvl_packet[DVL_OFFSET_VEL_Z]
        );

    data.velocity_error =
        dvl_read_float(
            &dvl_packet[DVL_OFFSET_VEL_ERROR]
        );


    /*
     * Bottom ranges.
     */
    data.range_beam1 =
        dvl_read_float(
            &dvl_packet[DVL_OFFSET_RANGE1]
        );

    data.range_beam2 =
        dvl_read_float(
            &dvl_packet[DVL_OFFSET_RANGE2]
        );

    data.range_beam3 =
        dvl_read_float(
            &dvl_packet[DVL_OFFSET_RANGE3]
        );

    data.range_beam4 =
        dvl_read_float(
            &dvl_packet[DVL_OFFSET_RANGE4]
        );

    data.mean_range =
        dvl_read_float(
            &dvl_packet[DVL_OFFSET_MEAN_RANGE]
        );


    /*
     * Speed of sound.
     */
    data.speed_of_sound =
        dvl_read_float(
            &dvl_packet[DVL_OFFSET_SPEED_OF_SOUND]
        );


    /*
     * Status.
     */
    data.status =
        dvl_read_u16(
            &dvl_packet[DVL_OFFSET_STATUS]
        );


    /*
     * Built-in-test.
     */
    data.bit =
        dvl_read_u16(
            &dvl_packet[DVL_OFFSET_BIT]
        );


    /*
     * Electrical measurements.
     */
    data.input_voltage =
        dvl_read_float(
            &dvl_packet[DVL_OFFSET_INPUT_VOLTAGE]
        );

    data.transmit_voltage =
        dvl_read_float(
            &dvl_packet[DVL_OFFSET_TRANSMIT_VOLTAGE]
        );

    data.transmit_current =
        dvl_read_float(
            &dvl_packet[DVL_OFFSET_TRANSMIT_CURRENT]
        );


    /*
     * Reception timestamp.
     */
    data.timestamp_ms =
        (uint32_t)(
            xTaskGetTickCount() *
            portTICK_PERIOD_MS
        );


    /*
     * Wayfinder uses NaN for bad velocity values.
     *
     * Do not publish a measurement containing invalid
     * velocity components.
     */
    if (!isfinite(data.vx) ||
        !isfinite(data.vy) ||
        !isfinite(data.vz))
    {
        return false;
    }


    data.valid = true;


    /*
     * Publish atomically.
     */
    taskENTER_CRITICAL();

    dvl_latest = data;

    taskEXIT_CRITICAL();


    return true;
}


/*===========================================================================================================================
 * Feed one byte
 * ===========================================================================================================================*/
bool dvl_feed_byte(uint8_t byte)
{
    switch (dvl_state)
    {
        /*---------------------------------------------------------------------------------------------------------------*/
        case DVL_WAIT_SOP:

            /*
             * Search for the complete six-byte SOP.
             *
             * This prevents starting a packet from an arbitrary
             * 0xAA occurring inside unrelated data.
             */
            if (byte == dvl_sop[dvl_sop_index])
            {
                dvl_packet[dvl_sop_index] = byte;

                dvl_sop_index++;

                if (dvl_sop_index >= DVL_SOP_LENGTH)
                {
                    dvl_index = DVL_SOP_LENGTH;

                    dvl_state =
                        DVL_COLLECT_PACKET;

                    dvl_sop_index = 0U;
                }
            }
            else
            {
                /*
                 * If this byte itself could be the beginning
                 * of a new SOP, restart at SOP byte 1.
                 */
                if (byte == dvl_sop[0])
                {
                    dvl_packet[0] = byte;

                    dvl_sop_index = 1U;
                }
                else
                {
                    dvl_sop_index = 0U;
                }
            }

            break;


        /*---------------------------------------------------------------------------------------------------------------*/
        case DVL_COLLECT_PACKET:

            /*
             * Store packet byte.
             */
            if (dvl_index >= DVL_PACKET_LENGTH)
            {
                dvl_parser_reset();

                return false;
            }

            dvl_packet[dvl_index++] = byte;


            /*
             * Complete 116-byte packet.
             */
            if (dvl_index >= DVL_PACKET_LENGTH)
            {
                bool valid =
                    dvl_parse_packet();

                dvl_parser_reset();

                return valid;
            }

            break;


        /*---------------------------------------------------------------------------------------------------------------*/
        default:

            dvl_parser_reset();

            break;
    }

    return false;
}


/*===========================================================================================================================
 * Get latest valid DVL data
 * ===========================================================================================================================*/
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
