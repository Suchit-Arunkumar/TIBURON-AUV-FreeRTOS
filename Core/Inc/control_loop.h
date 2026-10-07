
#ifndef __CONTROL_LOOP_H
#define __CONTROL_LOOP_H

#include "main.h"
#include <stdint.h>
#include <stdbool.h>
#include "timer_pwm.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#define N_DOF           6
#define N_THR           8

#define PWM_MIN         1100
#define PWM_MAX         1900
#define PWM_NEUTRAL     1500
// Slew limit while ramping up after a disarm: 50 us per tick, so neutral
// to full takes 8 ticks (160 ms).
#define PWM_RAMP_STEP   50      /* us per 50 Hz tick -> 2500 us/s slew rate */
#define CMD_TIMEOUT_MS  500u    /* ms without a valid CMD before failsafe     */

// Valid packets in a row needed to leave failsafe: one packet after a
// dropout doesn't prove the link is back.
#define CMD_RECOVERY_PACKETS  3u

extern volatile bool link_ok;

/* Log records dropped because logQueue was full. */
uint32_t control_log_drops(void);

void control_loop_get_pwm(uint16_t *out, uint8_t len);
void control_loop_get_pose(float out[N_DOF]);

/* Per-DOF PID output, surge..yaw, in the same units as U_MAX. */
void control_loop_get_u(float out[N_DOF]);

/* Allocation saturation: bit0 vertical, bit1 horizontal, bit2 yaw. */
uint8_t control_loop_get_sat_flags(void);

// What telemetry reports, copied in one go so a packet never mixes two
// control ticks.
typedef struct
{
    float    pose[N_DOF];
    float    u[N_DOF];
    uint16_t pwm_us[N_THR];
    uint8_t  sat_flags;
    bool     armed;
    bool     link_ok;
} ControlSnapshot;

void control_loop_snapshot(ControlSnapshot *out);
bool control_loop_get_armed(void);
bool control_loop_get_link(void);

void control_loop_init(void);

void control_loop_tick(void);

/* Pose (x, y, z, roll, pitch, yaw) from the Pi's fused nav state. */
void state_update(const float new_pose[N_DOF]);

// New setpoint from a valid command. In failsafe, packets only count
// towards recovery until CMD_RECOVERY_PACKETS have arrived.
void target_update(const float new_target[N_DOF], bool arm_flag);

/* True while the failsafe latch is engaged and recovery is incomplete. */
bool control_loop_in_failsafe(void);

// True while the slew limiter is still on after a disarm.
bool control_loop_ramping(void);

// Recovery progress, for the health report.
uint8_t control_loop_recovery_count(void);

void enterFailsafe(void);

void checkCommandTimeout(void);

void computePID(float dt);

void computeAllocation(void);

void applyPWM(void);

extern TaskHandle_t controlTaskHandle;
void control_task(void *argument);

#endif /* __CONTROL_LOOP_H */
