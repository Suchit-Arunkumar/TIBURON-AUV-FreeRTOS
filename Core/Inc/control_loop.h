
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
/*
 * Slew limit, applied ONLY while ramping out of failsafe.
 *
 * 50 us per 20 ms tick is 2500 us/s. Against the 800 us band that is
 * 6.25% of full range per tick: a full-scale reversal takes 16 ticks
 * (320 ms) and neutral to full takes 8 ticks (160 ms).
 *
 * That is the right behaviour when recovering from a comms loss - the
 * vehicle should walk up from neutral, not step - and the WRONG
 * behaviour in normal closed-loop operation, where it silently caps
 * control authority and looks exactly like badly tuned gains. It used to
 * be applied on every armed tick; see control_loop_ramping().
 */
#define PWM_RAMP_STEP   50      /* us per 50 Hz tick -> 2500 us/s slew rate */
#define CMD_TIMEOUT_MS  500u    /* ms without a valid CMD before failsafe     */

/*
 * Recovery after a comms loss requires this many CONSECUTIVE CRC-valid
 * packets. One good packet after a dropout is not evidence of a restored
 * link - it may be the only one that got through, or a stale frame that
 * was buffered somewhere along the way. Three in a row at the Pi's send
 * rate is real.
 */
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

/*
 * Everything telemetry reports about the controller, copied in one go.
 * Read through control_loop_snapshot() from other tasks, so a packet never
 * mixes values from two different control ticks.
 */
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

/*
 * Accept a new setpoint from a CRC-valid command packet.
 *
 * While in failsafe, the first CMD_RECOVERY_PACKETS-1 calls only count
 * towards recovery and do NOT move the setpoint or re-arm. The packet
 * that completes the streak becomes the active setpoint.
 *
 * A setpoint from before the dropout is never restored: enterFailsafe()
 * zeroes the target, and the ramp in applyPWM() then walks the thrusters
 * up from neutral rather than stepping to whatever the vehicle was doing
 * when the link died.
 */
void target_update(const float new_target[N_DOF], bool arm_flag);

/* True while the failsafe latch is engaged and recovery is incomplete. */
bool control_loop_in_failsafe(void);

/*
 * True while the post-recovery slew limiter is still active. Clears on
 * the first tick where every output reached its commanded value without
 * being clamped, after which the PID has full authority.
 */
bool control_loop_ramping(void);

/* Consecutive CRC-valid packets seen since the last loss, saturating at
 * CMD_RECOVERY_PACKETS. For the console health report. */
uint8_t control_loop_recovery_count(void);

void enterFailsafe(void);

void checkCommandTimeout(void);

void computePID(float dt);

void computeAllocation(void);

void applyPWM(void);

extern TaskHandle_t controlTaskHandle;
void control_task(void *argument);

#endif /* __CONTROL_LOOP_H */
