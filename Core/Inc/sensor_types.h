#ifndef SENSOR_TYPES_H
#define SENSOR_TYPES_H

#include <stdint.h>

/*
 * What imu_task and depth_task publish, whichever sensor it came from.
 * Readers (comms, logging, display) never need to know which chip is
 * fitted; `source` says, for telemetry and the log.
 */

typedef enum
{
    SRC_NONE   = 0,
    SRC_VN200  = 1,
    SRC_BNO085 = 2,
    SRC_BAR30  = 3,
    SRC_ADC    = 4
} SensorSource;

const char *sensor_source_str(uint8_t source);

typedef struct
{
    uint32_t timestamp_ms;      /* STM32 tick when the reading arrived */
    uint8_t  source;            /* SRC_VN200 or SRC_BNO085             */
    uint8_t  accuracy;          /* BNO085 0..3; 3 for the VN-200       */

    float yaw_deg;              /* attitude                            */
    float pitch_deg;
    float roll_deg;

    float gyro_x_rad_s;         /* body-frame angular rate             */
    float gyro_y_rad_s;
    float gyro_z_rad_s;

    float accel_x_m_s2;         /* body-frame acceleration, with gravity */
    float accel_y_m_s2;
    float accel_z_m_s2;
} ImuSample;

typedef struct
{
    uint32_t timestamp_ms;
    uint8_t  source;            /* SRC_BAR30 or SRC_ADC */
    float    depth_m;           /* positive down        */
    float    pressure_pa;       /* Bar30: absolute; ADC: gauge */
    float    temperature_c;     /* Bar30 only, 0 for the ADC sensor */
} DepthSample;

#endif
