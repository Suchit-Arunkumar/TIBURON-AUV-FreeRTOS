#ifndef STRUCT_H
#define STRUCT_H

#include <stdint.h>
#include "stm32f446xx.h"

#define PAYLOAD_LEN 56

// ── TYPE 0x01 : TELEMETRY payload (Pico → Pi, 56 bytes) ─────────────────────
//
// Centralised debug/monitoring snapshot — everything the Pi needs in one place.
//
// Layout (__packed__, 56 bytes):
//   float    depth_m        —  4 : control depth (current_z from fused CMD state)
//   float    raw_depth_m    —  4 : raw pressure-sensor depth (telemetry/debug only)
//   float    pid_u[6]       — 24 : PID output per DOF (U[]) for surge…yaw
//   uint16_t esc_pwm[8]     — 16 : actual ESC PWM values T1–T8 (µs)
//   uint8_t  armed          —  1 : armed state (0/1)
//   uint8_t  sat_flags      —  1 : bit0=sat_vert, bit1=sat_horiz, bit2=sat_yaw
//   uint8_t  link_ok        —  1 : 1 = link healthy, 0 = lost/timeout
//   uint8_t  reserved[5]    —  5 : zero-padding
//                              ──
//   Total                      56 bytes ✓
typedef struct __attribute__((packed)) {
    float    depth_m;
    float    raw_depth_m;
    float    pid_u[6];
    uint16_t esc_pwm[8];
    uint8_t  armed;
    uint8_t  sat_flags;
    uint8_t  link_ok;
    uint8_t  reserved[5];
} TelemetryPayload;
_Static_assert(sizeof(TelemetryPayload) == PAYLOAD_LEN,"TelemetryPayload size mismatch");




// ── TYPE 0x02 : CMD payload (Pi → Pico, 56 bytes) ───────────────────────────
// Python format string: '<12f3B5s'
typedef struct __attribute__((packed)) {
    float    current_x;
    float    current_y;
    float    current_z;
    float    current_roll;
    float    current_pitch;
    float    current_yaw;

    float    target_x;
    float    target_y;
    float    target_z;
    float    target_roll;
    float    target_pitch;
    float    target_yaw;

    uint8_t  armed;
    uint8_t  seq;

    uint8_t  reserved[6];
} CommandPayload;
_Static_assert(sizeof(CommandPayload) == PAYLOAD_LEN,"CommandPayload size mismatch");



// ── TYPE 0x03 : SENSORS payload (STM32 → Pi, 56 bytes, 50 Hz) ───────────────
//
// The active IMU and depth readings, whichever sensor they come from. Sent
// with every telemetry frame. *_age_ms is how old the reading was when the
// frame was built: the Pi should ignore a reading older than ~200 ms (the
// source would read "none" by then anyway).
//
// Python format string: '<IHHBBBB3f3f3fff'
typedef struct __attribute__((packed)) {
    uint32_t stamp_ms;        // STM32 time the frame was built
    uint16_t imu_age_ms;
    uint16_t depth_age_ms;
    uint8_t  imu_source;      // 0 none, 1 VN-200, 2 BNO085
    uint8_t  imu_accuracy;    // BNO085 0..3, VN-200 always 3
    uint8_t  depth_source;    // 0 none, 3 Bar30, 4 analog
    uint8_t  reserved;
    float    yaw_deg, pitch_deg, roll_deg;
    float    gyro_x_rad_s, gyro_y_rad_s, gyro_z_rad_s;
    float    accel_x_m_s2, accel_y_m_s2, accel_z_m_s2;   // includes gravity
    float    depth_m;
    float    water_temp_c;    // Bar30 only, else 0
} SensorsPayload;
_Static_assert(sizeof(SensorsPayload) == PAYLOAD_LEN, "SensorsPayload size mismatch");



// ── TYPE 0x04 : DVL payload (STM32 → Pi, 56 bytes, on each new DVL frame) ───
//
// One Wayfinder frame. velocity_valid = 0 means no bottom lock: the
// velocities are NaN and must not be used (do not treat them as 0).
//
// Python format string: '<IHBB4f4ffHHff'
typedef struct __attribute__((packed)) {
    uint32_t stamp_ms;        // STM32 time the frame was built
    uint16_t age_ms;          // how old the DVL reading was at that time
    uint8_t  velocity_valid;
    uint8_t  coordinate_system;
    float    vx_m_s, vy_m_s, vz_m_s, verr_m_s;
    float    range_beam_m[4];
    float    mean_range_m;
    uint16_t status;
    uint16_t bit;
    float    speed_of_sound_m_s;
    float    input_voltage_v;
} DvlPayload;
_Static_assert(sizeof(DvlPayload) == PAYLOAD_LEN, "DvlPayload size mismatch");



#endif
