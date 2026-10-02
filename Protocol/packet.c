#include "packet.h"
#include "ring_buffer.h"
#include "crc16.h"
#include <string.h>

void packet_build_telemetry(const TelemetryPayload *tp, uint8_t *out_buf){

	// include headers
	out_buf[0] = STX1;
	out_buf[1] = STX2;

	//payload len
	out_buf[2] = PAYLOAD_LEN;

	//MSG ID
	out_buf[3] = TYPE_TELEMETRY;

	memcpy(&out_buf[4], tp, sizeof(TelemetryPayload));

	// CRC-16 over LEN + TYPE + PAYLOAD, sent high byte first
	uint16_t crc = crc16_ccitt(&out_buf[2], 2 + PAYLOAD_LEN);

	out_buf[PACKET_SIZE - 2]= (uint8_t)(crc >> 8);
	out_buf[PACKET_SIZE - 1]= (uint8_t)(crc);

}

uint8_t packet_parse_cmd(CommandPayload *out_cmd)
{
    while (rx_avail() >= PACKET_SIZE)
    {
        // Check STX
        if (rx_peek(0) != STX1 ||
            rx_peek(1) != STX2)
        {
            rx_eat(1);
            continue;
        }
        // Check length
        if (rx_peek(2) != PAYLOAD_LEN)
        {
            rx_eat(1);
            continue;
        }
        // Check type
        if (rx_peek(3) != TYPE_CMD)
        {
            rx_eat(1);
            continue;
        }
        // Copy packet into temp buffer
        uint8_t pkt[PACKET_SIZE];
        for (int i = 0; i < PACKET_SIZE; i++)
        {
            pkt[i] = rx_peek(i);
        }
        // Compute CRC over LEN + TYPE + PAYLOAD
        uint16_t calc_crc =
            crc16_ccitt(&pkt[2], 2 + PAYLOAD_LEN);
        // Reconstruct received CRC (high byte first)
        uint16_t recv_crc =
            (uint16_t)(((uint16_t)pkt[PACKET_SIZE - 2] << 8) |
                       pkt[PACKET_SIZE - 1]);
        if (calc_crc != recv_crc)
        {
            rx_eat(1);
            continue;
        }
        // Copy payload into struct
        memcpy(out_cmd,
               &pkt[4],
               sizeof(CommandPayload));
        // Remove packet from buffer
        rx_eat(PACKET_SIZE);
        return 1;
    }
    return 0;
}
