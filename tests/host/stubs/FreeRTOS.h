/* Host-test stand-in for the few FreeRTOS names the drivers use. */
#ifndef FREERTOS_H
#define FREERTOS_H

#include <stdint.h>

typedef uint32_t TickType_t;
#define portTICK_PERIOD_MS  1U

extern TickType_t host_tick;

#endif
