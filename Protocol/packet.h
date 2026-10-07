// packet.h

#ifndef PACKET_H
#define PACKET_H

#include <stdint.h>
#include "struct.h"   // TelemetryPayload, CommandPayload



#define STX1         0xAA
#define STX2         0x55
// PAYLOAD_LEN (56) comes from struct.h
// Same frame as the Pico firmware, so the existing Pi code works unchanged:
// [0xAA][0x55][LEN][TYPE][56-byte payload][CRC16 hi][CRC16 lo]
#define PACKET_SIZE  62   // 4 header + 56 payload + 2 CRC16

#define TYPE_TELEMETRY  0x01
#define TYPE_CMD        0x02
#define TYPE_SENSORS    0x03
#define TYPE_DVL        0x04

// Fills out_buf[PACKET_SIZE] with a complete frame of the given type:
// header, the 56-byte payload, CRC-16. payload must be PAYLOAD_LEN bytes.
void packet_build(uint8_t type, const void *payload, uint8_t *out_buf);

// Same, for a telemetry frame.
void packet_build_telemetry(const TelemetryPayload *tp, uint8_t *out_buf);

// Scans ring buffer for a valid CMD packet
// Returns 1 if found and CRC passes, 0 otherwise
// On success, copies parsed payload into *out_cmd
uint8_t packet_parse_cmd(CommandPayload *out_cmd);

#endif
