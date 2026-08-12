#ifndef VN200_H
#define VN200_H

#include <stdint.h>
#include <stdbool.h>

typedef struct
{
    float accel_x;
    float accel_y;
    float accel_z;

    float gyro_x;
    float gyro_y;
    float gyro_z;

    float yaw;
    float pitch;
    float roll;

    bool valid;

} VN200Data;

bool vn200_parse(const uint8_t *packet,
                 uint16_t length,
                 VN200Data *out);

void vn200_feed_byte(uint8_t byte);

bool vn200_get_data(VN200Data *out);

#endif
