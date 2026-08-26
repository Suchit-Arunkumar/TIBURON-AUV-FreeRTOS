// =============================================================================
// control_loop.c — ROV 6-DOF Control Loop  (STM32 port)
//
// Ported from: ROV_6DOF_TUNING.ino  (RP2350 / arduino-pico)
// Target:      STM32  (TIM7 fires at 50 Hz → dt = 0.02 s, always)
//
// External dependencies expected from the BSP:
//   void pwm_set_us(uint8_t ch, uint16_t us)  — set ESC PWM channel (0-7)
//   bool link_ok                              — set/cleared here; read by comms
//
// Timing comes from FreeRTOS (xTaskGetTickCount), not from a SysTick
// counter — the kernel owns SysTick.
// =============================================================================

#include "control_loop.h"
#include <math.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include "FreeRTOS.h"
#include "task.h"
#include "packet.h"
#include "comms_task.h"
#include "logging_task.h"
#include "sd_logger.h"
#include "crc_hw.h"
#include "timer_basic.h"
#include "iwdg.h"


// =============================================================================
// SECTION 1 — CONSTANTS
// =============================================================================
#define N_DOF           6
#define N_THR           8

#define PWM_MIN         1100
#define PWM_MAX         1900
#define PWM_NEUTRAL     1500
#define PWM_RAMP_STEP   50          // us/tick at 50 Hz -> 2500 us/s, RECOVERY ONLY

#define CMD_TIMEOUT_MS  500u        // ms before failsafe triggers

#define THRUST_DEADZONE 0.02f


// FreeRTOS handle used by TIM7 ISR to notify the control task
TaskHandle_t controlTaskHandle = NULL;

// =============================================================================
// SECTION 2 — ALLOCATION MATRICES
//
// FORWARD  B  (6×8)   tau = B · T
// INVERSE  B⁺ (8×6)   T  = B⁺ · U
// =============================================================================
__attribute__((unused))
static const float B_forward[N_DOF][N_THR] = {
    { 0.0f,     0.0f,     0.0f,     0.0f,    0.7070f, -0.7070f,  0.7070f,  0.7070f},
    { 0.0f,     0.0f,     0.0f,     0.0f,   -0.7070f,  0.7070f,  0.7070f, -0.7070f},
    { 1.0f,     1.0f,     1.0f,     1.0f,    0.0f,     0.0f,     0.0f,     0.0f   },
    { 0.1700f, -0.1200f,  0.1200f, -0.1200f,-0.0140f,  0.0140f, -0.0140f,  0.0140f},
    {-0.1200f, -0.1200f,  0.1500f,  0.1500f,-0.0140f, -0.0140f, -0.0140f, -0.0140f},
    { 0.0f,     0.0f,     0.0f,     0.0f,    0.0346f,  0.0346f, -0.3250f,  0.3250f}
};

static const float B_pinv[N_THR][N_DOF] = {
    {-0.0349f, -0.0776f,  0.2493f,  2.0482f, -1.6573f, -0.2618f},
    {-0.0448f, -0.0089f,  0.3060f, -2.0430f, -2.0306f,  0.0586f},
    { 0.0439f,  0.0148f,  0.1989f,  1.6903f,  1.9984f, -0.0310f},
    { 0.0358f,  0.0717f,  0.2459f, -1.6955f,  1.6895f,  0.2341f},
    { 0.0765f, -0.5493f,  0.0036f, -0.0622f, -0.1885f, -1.0178f},
    { 0.0617f,  0.8347f,  0.0029f, -0.0503f, -0.1523f,  1.9589f},
    { 0.7072f,  0.7072f,  0.0000f,  0.0000f,  0.0000f,  0.0000f},
    { 0.6925f,  0.6768f, -0.0007f,  0.0120f,  0.0363f,  2.9767f}
};

// =============================================================================
// SECTION 3 — PID GAINS AND LIMITS
//                          Surge   Sway    Heave   Roll    Pitch   Yaw
// =============================================================================
static const float kp[N_DOF]        = { 0.00f,  0.00f,  0.00f,  0.00f,  0.00f,  0.00f };
static const float ki[N_DOF]        = { 0.00f,  0.00f,  0.00f,  0.00f,  0.00f,  0.00f };
static const float kd[N_DOF]        = { 0.00f,  0.00f,  0.00f,  0.00f,  0.00f,  0.00f };
static const float kff[N_DOF]       = { 0.00f,  0.00f,  0.00f,  0.00f,  0.00f,  0.00f };
static const float FF_OFFSET[N_DOF] = { 0.0f,   0.0f,   0.0f,   0.0f,   0.0f,   0.0f  };
static const float I_CLAMP[N_DOF]   = { 20.0f,  20.0f,  20.0f,  5.0f,   5.0f,   5.0f  };
static const float U_MAX[N_DOF]     = { 80.0f,  80.0f,  80.0f,  20.0f,  20.0f,  20.0f };

// =============================================================================
// SECTION 4 — STATIC STATE VARIABLES
// =============================================================================
static float pose[N_DOF]     = {0};
static float target[N_DOF]   = {0};
static float errState[N_DOF] = {0};
static float errPrev[N_DOF]  = {0};
static float errInt[N_DOF]   = {0};
static float U[N_DOF]        = {0};
static float T_out[N_THR]    = {0};


/*
 * VOLATILE POLICY (Phase 12 audit).
 *
 * Anything written in one execution context and read in another is
 * volatile. Note honestly what that does and does not buy here: every
 * cross-context read below goes through a non-inlined accessor in a
 * DIFFERENT translation unit, and LTO is off, so the compiler already
 * cannot cache these across the call. No live miscompilation was found.
 *
 * volatile is added because the guarantee should come from the
 * declaration rather than from the accident of where the function lives -
 * enabling -flto, or moving an accessor into a header as static inline,
 * would silently remove the protection otherwise.
 */

/* Read by comms_task (telemetry) and spi_owner_task (OLED). */
static volatile bool       g_armed       = false;

/* control_task only: written by target_update, read by
 * checkCommandTimeout, both in the same call chain. Not cross-context. */
static TickType_t          last_cmd_tick = 0;

static int                 g_pwm_current[N_THR];

/*
 * Failsafe latch and recovery counter.
 *
 * g_in_failsafe is sticky: once the link is declared lost it stays set
 * until CMD_RECOVERY_PACKETS consecutive CRC-valid packets have arrived.
 * Without the latch, a single packet arriving inside the timeout window
 * would silently re-arm the vehicle from one frame.
 */
/*
 * Both written only by control_task (target_update / enterFailsafe) and
 * by control_loop_init pre-scheduler; read by dummy_task's health report.
 *
 * The 3-packet streak deliberately lives HERE and not in comms_task.
 * comms_task parses packets and posts to commandQueue; it never touches
 * the streak. So there is no read-modify-write split across two
 * priorities - the increment, the compare and the reset all happen in
 * control_task.
 */
static volatile bool    g_in_failsafe    = true;
static volatile uint8_t g_recovery_count = 0;

/*
 * Slew limiter, active only while ramping out of neutral.
 *
 * Set whenever the outputs are forced to neutral - boot, or any failsafe
 * trip - and cleared on the first tick where no output needed clamping,
 * i.e. the moment the thrusters have caught up with what the PID is
 * asking for. From then on the controller has full authority and a step
 * command is a step.
 */
static volatile bool g_ramping = true;

/* Written by control_task; read by comms_task and spi_owner_task. */
volatile bool link_ok = false;

// =============================================================================
// SECTION 5 — INTERNAL HELPERS
// =============================================================================
static inline float wrapAngle180(float angle)
{
    while (angle >  180.0f) angle -= 360.0f;
    while (angle < -180.0f) angle += 360.0f;
    return angle;
}

static int thrust_to_pwm(float u)
{
    if (u >  1.0f) u =  1.0f;
    if (u < -1.0f) u = -1.0f;
    if (fabsf(u) < THRUST_DEADZONE) return PWM_NEUTRAL;
    return (u >= 0.0f)
        ? PWM_NEUTRAL + (int)(u * (float)(PWM_MAX - PWM_NEUTRAL))
        : PWM_NEUTRAL + (int)(u * (float)(PWM_NEUTRAL - PWM_MIN));
}

// =============================================================================
// SECTION 6 — PUBLIC API
// =============================================================================
bool control_loop_in_failsafe(void)
{
    return g_in_failsafe;
}

bool control_loop_ramping(void)
{
    return g_ramping;
}

uint8_t control_loop_recovery_count(void)
{
    return g_recovery_count;
}

void control_loop_init(void)
{
    for (int i = 0; i < N_THR; i++) {
        g_pwm_current[i] = PWM_NEUTRAL;
        pwm_set_us((uint8_t)i, (uint16_t)PWM_NEUTRAL);
    }

    g_in_failsafe    = true;
    g_recovery_count = 0;
    g_ramping        = true;

    memset(errInt,   0, sizeof(errInt));
    memset(errPrev,  0, sizeof(errPrev));
    memset(errState, 0, sizeof(errState));
    memset(U,        0, sizeof(U));
    memset(T_out,    0, sizeof(T_out));
    g_armed     = false;
    link_ok     = false;
    last_cmd_tick = 0;
}

void control_loop_tick(void)
{
    const float dt = 0.02f;

    checkCommandTimeout();

    if (g_armed) {
        computePID(dt);
        computeAllocation();
    }

    applyPWM();
}

void state_update(const StateEstimate *state)
{
    if (state == NULL || !state->attitude_valid) return;

    pose[0] = state->x;
    pose[1] = state->y;
    pose[2] = state->z;
    pose[3] = state->roll;
    pose[4] = state->pitch;
    pose[5] = state->yaw;
}

void target_update(const float new_target[N_DOF], bool arm_flag)
{
    /* Every CRC-valid packet refreshes the watchdog, whether or not it
     * is allowed to move the setpoint yet. */
    last_cmd_tick = xTaskGetTickCount();

    if (g_in_failsafe)
    {
        if (g_recovery_count < CMD_RECOVERY_PACKETS)
        {
            g_recovery_count++;
        }

        if (g_recovery_count < CMD_RECOVERY_PACKETS)
        {
            /*
             * Streak incomplete. Count it and nothing else - no setpoint,
             * no arming. Thrusters stay at neutral.
             */
            return;
        }

        /* Streak complete: the link is real. Leave failsafe and take
         * this packet's setpoint - never a cached pre-loss one. */
        g_in_failsafe = false;
    }

    memcpy(target, new_target, N_DOF * sizeof(float));
    g_armed       = arm_flag;
    link_ok       = true;
}

// =============================================================================
// SECTION 7 — computePID()
// =============================================================================
void computePID(float dt)
{
    if (dt <= 0.0f) return;

    for (int i = 0; i < N_DOF; i++) {
        float raw_err = target[i] - pose[i];
        if (i == 5) raw_err = wrapAngle180(raw_err);

        errState[i] = raw_err;

        errInt[i] += errState[i] * dt;
        if      (errInt[i] >  I_CLAMP[i]) errInt[i] =  I_CLAMP[i];
        else if (errInt[i] < -I_CLAMP[i]) errInt[i] = -I_CLAMP[i];

        float eDot  = (errState[i] - errPrev[i]) / dt;
        errPrev[i]  = errState[i];

        float ff  = kff[i] * target[i] + FF_OFFSET[i];
        float raw = kp[i] * errState[i]
                  + ki[i] * errInt[i]
                  + kd[i] * eDot
                  + ff;

        if      (raw >  U_MAX[i]) raw =  U_MAX[i];
        else if (raw < -U_MAX[i]) raw = -U_MAX[i];
        U[i] = raw;
    }
}

// =============================================================================
// SECTION 8 — computeAllocation()
// =============================================================================
void computeAllocation(void)
{
    float T_trans[N_THR] = {0};
    float T_yaw[N_THR]   = {0};
    float T_vert[N_THR]  = {0};

    for (int i = 0; i < N_THR; i++) {
        T_trans[i] = B_pinv[i][0]*U[0] + B_pinv[i][1]*U[1];
        T_yaw[i]   = B_pinv[i][5]*U[5];
        T_vert[i]  = B_pinv[i][2]*U[2] + B_pinv[i][3]*U[3] + B_pinv[i][4]*U[4];
    }

    float maxV = 0.0f;
    for (int i = 0; i < 4; i++)
        if (fabsf(T_vert[i]) > maxV) maxV = fabsf(T_vert[i]);
    if (maxV > 1.0f)
        for (int i = 0; i < 4; i++) T_vert[i] /= maxV;

    float maxTn = 0.0f;
    for (int i = 4; i < N_THR; i++)
        if (fabsf(T_trans[i]) > maxTn) maxTn = fabsf(T_trans[i]);
    if (maxTn > 1.0f)
        for (int i = 4; i < N_THR; i++) T_trans[i] /= maxTn;

    float maxY = 0.0f;
    for (int i = 4; i < N_THR; i++)
        if (fabsf(T_yaw[i]) > maxY) maxY = fabsf(T_yaw[i]);
    if (maxY > 1.0f)
        for (int i = 4; i < N_THR; i++) T_yaw[i] /= maxY;

    for (int i = 0; i < N_THR; i++) {
        T_out[i] = (i < 4) ? T_vert[i] : (T_trans[i] + T_yaw[i]);
        if      (T_out[i] >  1.0f) T_out[i] =  1.0f;
        else if (T_out[i] < -1.0f) T_out[i] = -1.0f;
    }
}

// =============================================================================
// SECTION 9 — applyPWM()
// =============================================================================
void applyPWM(void)
{
    /* Set if ANY channel had to be clamped this tick. */
    bool clamped_any = false;

    for (int i = 0; i < N_THR; i++) {
        if (!g_armed) {
            g_pwm_current[i] = PWM_NEUTRAL;
        } else {
            float thrust   = (i == 0) ? -T_out[i] : T_out[i];
            int target_pwm = thrust_to_pwm(thrust);

            if (g_ramping) {
                /*
                 * Recovery ramp ONLY. Walk towards the commanded value at
                 * PWM_RAMP_STEP per tick, so the thrusters come up from
                 * neutral smoothly after a failsafe trip or at boot.
                 */
                int delta = target_pwm - g_pwm_current[i];

                if (delta > PWM_RAMP_STEP) {
                    delta = PWM_RAMP_STEP;
                    clamped_any = true;
                } else if (delta < -PWM_RAMP_STEP) {
                    delta = -PWM_RAMP_STEP;
                    clamped_any = true;
                }

                g_pwm_current[i] += delta;
            } else {
                /*
                 * Normal operation: the PID gets full authority.
                 *
                 * This limiter used to run on EVERY armed tick, capping
                 * the loop at 6.25% of full range per tick - a full-scale
                 * reversal took 320 ms. That is invisible from the
                 * outside and presents as badly tuned gains.
                 */
                g_pwm_current[i] = target_pwm;
            }

            if      (g_pwm_current[i] > PWM_MAX) g_pwm_current[i] = PWM_MAX;
            else if (g_pwm_current[i] < PWM_MIN) g_pwm_current[i] = PWM_MIN;
        }

        pwm_set_us((uint8_t)i, (uint16_t)g_pwm_current[i]);
    }

    /*
     * The ramp is complete once a whole pass needed no clamping - every
     * thruster has caught up with its commanded value. Only meaningful
     * while armed; a disarmed vehicle sits at neutral and stays ramping,
     * which is what we want for the next arming.
     */
    if (g_ramping && g_armed && !clamped_any) {
        g_ramping = false;
    }
}

// =============================================================================
// SECTION 10 — enterFailsafe()
// =============================================================================
void enterFailsafe(void)
{
    g_armed = false;
    link_ok = false;

    g_in_failsafe    = true;
    g_recovery_count = 0;

    /* Outputs are about to be forced to neutral, so the next arming must
     * walk them back up rather than step. */
    g_ramping = true;

    for (int i = 0; i < N_DOF; i++)
        errInt[i] = errPrev[i] = errState[i] = U[i] = 0.0f;

    /*
     * Discard the setpoint as well as the integrator state.
     *
     * This is the rule that matters: a stale packet arriving after a long
     * dropout must not re-apply the throttle the vehicle was carrying
     * when the link died. With target zeroed, recovery starts from a
     * stationary command, and applyPWM's PWM_RAMP_STEP slew then walks
     * the outputs up from PWM_NEUTRAL at 2500 us/s rather than stepping.
     */
    for (int i = 0; i < N_DOF; i++)
        target[i] = 0.0f;
}

// =============================================================================
// SECTION 11 — checkCommandTimeout()
// =============================================================================
void checkCommandTimeout(void)
{
    /*
     * Evaluated from control_task every 20 ms, so it cannot be starved by
     * a hung comms_task: the check lives on the deadline task, not on the
     * task that parses packets.
     *
     * The guard on last_cmd_tick keeps a board that has never received a
     * packet out of a permanent failsafe-trip loop; g_in_failsafe already
     * starts true, so the vehicle is disarmed either way.
     */
    if (last_cmd_tick != 0 &&
        (xTaskGetTickCount() - last_cmd_tick) > pdMS_TO_TICKS(CMD_TIMEOUT_MS))
    {
        if (!g_in_failsafe)
        {
            enterFailsafe();
        }
    }
}


void control_loop_get_pose(float out[N_DOF])
{
    if (out == NULL)
    {
        return;
    }

    memcpy(out, pose, N_DOF * sizeof(float));
}

void control_loop_get_pwm(uint16_t *out, uint8_t len)
{
    if (out == NULL)
    {
        return;
    }

    if (len > N_THR)
    {
        len = N_THR;
    }

    for (uint8_t i = 0; i < len; i++)
    {
        out[i] = (uint16_t)g_pwm_current[i];
    }
}

bool control_loop_get_armed(void)
{
    return g_armed;
}

bool control_loop_get_link(void)
{
    return link_ok;
}

// =============================================================================
// FreeRTOS CONTROL TASK
// =============================================================================
/*
 * P8: log every LOG_DECIMATION-th control tick.
 *
 * Control runs at 50 Hz; 50/10 = 5 Hz of records. Records are batched
 * into 512-byte blocks by logging_task, so this rate no longer implies
 * one SD block program per record.
 */
#define LOG_DECIMATION   10U

/*
 * Records dropped because logQueue was full.
 *
 * Non-zero means the logging pipeline could not keep up - almost
 * certainly the bus owner stuck in a long SD program cycle. Reported
 * over the console rather than inferred from gaps in the data.
 */
static volatile uint32_t log_drop_count = 0;

uint32_t control_log_drops(void)
{
    return log_drop_count;
}

void control_task(void *argument)
{
    // This task doesn't use its argument.
	(void)argument;

	static uint32_t log_tick_count = 0;

	/*
	 * B6: start the 50 Hz tick here, not in main.
	 *
	 * TIM7's ISR notifies this task and calls portYIELD_FROM_ISR. Both
	 * require the scheduler to be running and this handle to be
	 * populated. Starting the timer from main left a window in which
	 * neither was true. By the time this line executes, the scheduler has
	 * started and controlTaskHandle is set, so the window is closed by
	 * construction rather than by timing luck.
	 */
	tim7_init();

    // Run forever because FreeRTOS tasks are persistent execution contexts.
	for (;;)
	{
	    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

	    StateEstimate state;

	    if (xQueueReceive(stateQueue, &state, 0) == pdPASS)
	    {
	        state_update(&state);
	    }

	    CommandPayload cmd;

	    if (xQueueReceive(commandQueue, &cmd, 0) == pdPASS)
	    {
	        float new_target[6] = {
	            cmd.target_x,
	            cmd.target_y,
	            cmd.target_z,
	            cmd.target_roll,
	            cmd.target_pitch,
	            cmd.target_yaw
	        };

	        target_update(new_target, cmd.armed);
	    }

	    control_loop_tick();

	    /*
	     * Refresh the watchdog only here, and only after a completed
	     * tick. If this loop stops running, the board resets and the
	     * ESCs lose their signal — which is what we want — rather than
	     * staying alive with the last PWM values latched.
	     *
	     * Compiles to nothing unless ENABLE_IWDG is defined in iwdg.h.
	     */
	    iwdg_kick();

	    /* Tell Comms Task that a telemetry update is ready */
	    xTaskNotify(
	        commsTaskHandle,
	        (1UL << 1),
	        eSetBits
	    );

	    /*
	     * Flush the staging block on the armed->disarmed edge.
	     *
	     * enterFailsafe() clears g_armed, so a comms timeout produces
	     * this edge too and both cases are covered by one check. The
	     * records straddling a failsafe trip are the ones most worth
	     * having on the card, and without this they would sit in RAM
	     * until the block happened to fill - up to 2.4 s later, or
	     * never, if the board is then power-cycled.
	     */
	    {
	        static uint8_t prev_armed = 0;
	        uint8_t now_armed = control_loop_get_armed() ? 1U : 0U;

	        if ((prev_armed == 1U) && (now_armed == 0U) && (logQueue != NULL))
	        {
	            LogQueueItem flush_item;

	            flush_item.kind = LOG_ITEM_FLUSH;
	            memset(&flush_item.record, 0, sizeof(flush_item.record));

	            /* Zero block time here too - see below. */
	            if (xQueueSend(logQueue, &flush_item, 0) != pdPASS)
	            {
	                log_drop_count++;
	            }
	        }

	        prev_armed = now_armed;
	    }

	    /*
	     * P8: forward a decimated log record to logging_task.
	     *
	     * xQueueSend with a ZERO block time, always. control_task must
	     * never wait on the logging pipeline: a full logQueue means the
	     * bus owner is mid SD program cycle, which can legitimately last
	     * 250 ms, and blocking here would miss twelve 50 Hz deadlines.
	     * Dropping the record is the correct trade - and it is counted,
	     * not silent.
	     */
	    log_tick_count++;

	    if (log_tick_count >= LOG_DECIMATION)
	    {
	        log_tick_count = 0;

	        if (logQueue != NULL)
	        {
	            LogQueueItem item;
	            LogRecord *record = &item.record;
	            float pose_now[N_DOF];
	            uint16_t pwm_now[N_THR];

	            item.kind = LOG_ITEM_RECORD;

	            control_loop_get_pose(pose_now);
	            control_loop_get_pwm(pwm_now, N_THR);
	            memcpy(record->pwm, pwm_now, sizeof(pwm_now));

	            record->timestamp_ms = (uint32_t)xTaskGetTickCount();
	            record->depth_m      = pose_now[2];
	            record->roll_deg     = pose_now[3];
	            record->pitch_deg    = pose_now[4];
	            record->yaw_deg      = pose_now[5];
	            record->armed        = control_loop_get_armed() ? 1 : 0;
	            record->link_ok      = control_loop_get_link()  ? 1 : 0;
	            record->crc16        = 0;

	            /*
	             * Hardware CRC32 truncated to its low 16 bits - this is a
	             * truncated CRC32, not a CRC16, and the field name is
	             * historical. Computed over the record excluding the
	             * field itself, which is why crc16 is last in the struct.
	             */
	            record->crc16 = (uint16_t)crc_compute(
	                (const uint8_t *)record,
	                sizeof(LogRecord) - sizeof(record->crc16)
	            );

	            if (xQueueSend(logQueue, &item, 0) != pdPASS)
	            {
	                log_drop_count++;
	            }
	        }
	    }

	}
}
