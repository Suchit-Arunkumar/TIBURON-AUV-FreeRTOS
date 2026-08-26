#include "sensor_status.h"

volatile SensorState g_vn200_state = SENSOR_INIT;
volatile SensorState g_dvl_state   = SENSOR_INIT;
volatile SensorState g_bar30_state = SENSOR_INIT;

const char *sensor_state_str(SensorState s)
{
    switch (s)
    {
        case SENSOR_INIT:    return "init";
        case SENSOR_OK:      return "ok";
        case SENSOR_ABSENT:  return "ABSENT";
        case SENSOR_FAULTED: return "FAULTED";
        default:             return "?";
    }
}
