// 6-DOF control loop: PID per axis, thrust allocation, PWM, command
// timeout. Ported from the Pico firmware (ROV_6DOF_TUNING.ino). Runs in
// control_task, woken by TIM7 every 20 ms.

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
#include "crc16.h"
#include "timer_basic.h"
#include "iwdg.h"
#include "bench.h"


#define N_DOF           6
#define N_THR           8

#define PWM_MIN         1100
#define PWM_MAX         1900
#define PWM_NEUTRAL     1500
#define PWM_RAMP_STEP   50          // us per 20 ms tick = 2500 us/s, only while ramping

#define CMD_TIMEOUT_MS  500u

#define THRUST_DEADZONE 0.02f


TaskHandle_t controlTaskHandle = NULL;

// Allocation: tau = B * T (6 forces/torques from 8 thrusts), and its
// pseudo-inverse T = B+ * U, computed offline.
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

// Gains are zero until the vehicle is tuned in the water.
//                                       Surge   Sway    Heave   Roll    Pitch   Yaw
static const float kp[N_DOF]        = { 0.00f,  0.00f,  0.00f,  0.00f,  0.00f,  0.00f };
static const float ki[N_DOF]        = { 0.00f,  0.00f,  0.00f,  0.00f,  0.00f,  0.00f };
static const float kd[N_DOF]        = { 0.00f,  0.00f,  0.00f,  0.00f,  0.00f,  0.00f };
static const float kff[N_DOF]       = { 0.00f,  0.00f,  0.00f,  0.00f,  0.00f,  0.00f };
static const float FF_OFFSET[N_DOF] = { 0.0f,   0.0f,   0.0f,   0.0f,   0.0f,   0.0f  };
static const float I_CLAMP[N_DOF]   = { 20.0f,  20.0f,  20.0f,  5.0f,   5.0f,   5.0f  };
static const float U_MAX[N_DOF]     = { 80.0f,  80.0f,  80.0f,  20.0f,  20.0f,  20.0f };

static float pose[N_DOF]     = {0};
static float target[N_DOF]   = {0};
static float errState[N_DOF] = {0};
static float errPrev[N_DOF]  = {0};
static float errInt[N_DOF]   = {0};
static float U[N_DOF]        = {0};
static float T_out[N_THR]    = {0};


// volatile: written here, read by other tasks (telemetry, display, health
// report).
static volatile bool       g_armed       = false;

static TickType_t          last_cmd_tick = 0;

static int                 g_pwm_current[N_THR];

// Failsafe latch. Stays set until CMD_RECOVERY_PACKETS valid packets have
// arrived, none more than CMD_TIMEOUT_MS apart, so one stray packet can't
// re-arm the vehicle.
static volatile bool    g_in_failsafe    = true;
static volatile uint8_t g_recovery_count = 0;

// Slew limiter, on only while ramping up from neutral (boot, any disarm).
// Turns off once every output has caught up with its command.
static volatile bool g_ramping = true;

// bit0 vertical, bit1 horizontal, bit2 yaw: set when that group had to be
// scaled down. Lets the Pi tell a wrong PID from a vehicle that is simply
// out of thrust.
static volatile uint8_t g_sat_flags = 0;

volatile bool link_ok = false;

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

// Clear the integral, last error and output, and turn the ramp back on.
// Runs on every disarm (Pi or failsafe) so re-arming never starts from a
// stale integral or jumps straight to full thrust.
static void reset_controller(void)
{
    for (int i = 0; i < N_DOF; i++)
        errInt[i] = errPrev[i] = errState[i] = U[i] = 0.0f;

    g_ramping = true;
}

void control_loop_tick(void)
{
    const float dt = 0.02f;
    static bool was_armed = false;

    checkCommandTimeout();

    if (was_armed && !g_armed) {
        reset_controller();
    }
    was_armed = g_armed;

    if (g_armed) {
        computePID(dt);
        computeAllocation();
    }

    applyPWM();
}

// The pose comes from the Pi's fused state in each command packet.
void state_update(const float new_pose[N_DOF])
{
    if (new_pose == NULL) return;

    memcpy(pose, new_pose, N_DOF * sizeof(float));
}

void target_update(const float new_target[N_DOF], bool arm_flag)
{
    // Any valid packet resets the timeout, even during recovery.
    last_cmd_tick = xTaskGetTickCount();

    if (g_in_failsafe)
    {
        if (g_recovery_count < CMD_RECOVERY_PACKETS)
        {
            g_recovery_count++;
        }

        if (g_recovery_count < CMD_RECOVERY_PACKETS)
        {
            return;     // still recovering: thrusters stay at neutral
        }

        g_in_failsafe = false;
    }

    memcpy(target, new_target, N_DOF * sizeof(float));
    g_armed       = arm_flag;
    link_ok       = true;
}

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

// Thrusters 0-3 are vertical, 4-7 horizontal. Each group is scaled down on
// its own if any thruster in it would exceed full thrust.
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

    uint8_t sat = 0U;

    float maxV = 0.0f;
    for (int i = 0; i < 4; i++)
        if (fabsf(T_vert[i]) > maxV) maxV = fabsf(T_vert[i]);
    if (maxV > 1.0f) {
        for (int i = 0; i < 4; i++) T_vert[i] /= maxV;
        sat |= (1U << 0);
    }

    float maxTn = 0.0f;
    for (int i = 4; i < N_THR; i++)
        if (fabsf(T_trans[i]) > maxTn) maxTn = fabsf(T_trans[i]);
    if (maxTn > 1.0f) {
        for (int i = 4; i < N_THR; i++) T_trans[i] /= maxTn;
        sat |= (1U << 1);
    }

    float maxY = 0.0f;
    for (int i = 4; i < N_THR; i++)
        if (fabsf(T_yaw[i]) > maxY) maxY = fabsf(T_yaw[i]);
    if (maxY > 1.0f) {
        for (int i = 4; i < N_THR; i++) T_yaw[i] /= maxY;
        sat |= (1U << 2);
    }

    g_sat_flags = sat;

    for (int i = 0; i < N_THR; i++) {
        T_out[i] = (i < 4) ? T_vert[i] : (T_trans[i] + T_yaw[i]);
        if      (T_out[i] >  1.0f) T_out[i] =  1.0f;
        else if (T_out[i] < -1.0f) T_out[i] = -1.0f;
    }
}

void applyPWM(void)
{
    bool clamped_any = false;

    for (int i = 0; i < N_THR; i++) {
        if (!g_armed) {
            g_pwm_current[i] = PWM_NEUTRAL;
        } else {
            float thrust   = (i == 0) ? -T_out[i] : T_out[i];   // thruster 1 inverted, as in the Pico firmware
            int target_pwm = thrust_to_pwm(thrust);

            if (g_ramping) {
                // Walk towards the command at PWM_RAMP_STEP per tick.
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
                // Full authority. (An always-on limiter here used to cap
                // the loop at 6% of range per tick, which looks exactly
                // like badly tuned gains.)
                g_pwm_current[i] = target_pwm;
            }

            if      (g_pwm_current[i] > PWM_MAX) g_pwm_current[i] = PWM_MAX;
            else if (g_pwm_current[i] < PWM_MIN) g_pwm_current[i] = PWM_MIN;
        }

        pwm_set_us((uint8_t)i, (uint16_t)g_pwm_current[i]);
    }

    // Ramp done once a whole tick needed no limiting.
    if (g_ramping && g_armed && !clamped_any) {
        g_ramping = false;
    }
}

void enterFailsafe(void)
{
    g_armed = false;
    link_ok = false;

    g_in_failsafe    = true;
    g_recovery_count = 0;

    reset_controller();

    // Clear the setpoint too, so a stale packet after the dropout can't
    // bring back the thrust the vehicle had when the link died.
    for (int i = 0; i < N_DOF; i++)
        target[i] = 0.0f;
}

// Runs in control_task, not comms_task, so it still fires if comms_task
// hangs. last_cmd_tick == 0 means no packet has ever arrived; the vehicle
// starts in failsafe anyway.
void checkCommandTimeout(void)
{
    TickType_t since = xTaskGetTickCount() - last_cmd_tick;

    if (last_cmd_tick != 0 && since > pdMS_TO_TICKS(CMD_TIMEOUT_MS))
    {
        if (!g_in_failsafe)
        {
            enterFailsafe();
            bench_failsafe_entered((uint32_t)since * portTICK_PERIOD_MS);
        }
        else if (g_recovery_count != 0U)
        {
            // A gap during recovery restarts the count: the 3 packets
            // must be consecutive, not just 3 since the dropout.
            g_recovery_count = 0U;
        }
    }
}


uint8_t control_loop_get_sat_flags(void)
{
    return g_sat_flags;
}

void control_loop_get_u(float out[N_DOF])
{
    if (out == NULL)
    {
        return;
    }

    memcpy(out, U, N_DOF * sizeof(float));
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

// Copy everything telemetry needs in one critical section. Calling the
// getters one by one from comms_task could be interrupted by the control
// task between them, mixing values from two ticks.
void control_loop_snapshot(ControlSnapshot *out)
{
    if (out == NULL)
    {
        return;
    }

    taskENTER_CRITICAL();

    memcpy(out->pose, pose, sizeof(out->pose));
    memcpy(out->u,    U,    sizeof(out->u));

    for (int i = 0; i < N_THR; i++)
    {
        out->pwm_us[i] = (uint16_t)g_pwm_current[i];
    }

    out->sat_flags = g_sat_flags;
    out->armed     = g_armed;
    out->link_ok   = link_ok;

    taskEXIT_CRITICAL();
}

bool control_loop_get_link(void)
{
    return link_ok;
}

#define LOG_DECIMATION   10U    // 50 Hz / 10 = 5 log records per second

static volatile uint32_t log_drop_count = 0;

uint32_t control_log_drops(void)
{
    return log_drop_count;
}

void control_task(void *argument)
{
	(void)argument;

	static uint32_t log_tick_count = 0;

	// Start the 50 Hz timer from here, once this task's handle exists for
	// the TIM7 interrupt to notify.
	tim7_init();

	for (;;)
	{
	    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
	    bench_control_wake();

	    CommandPayload cmd;

	    // Empty the queue so this tick uses the newest command. Every
	    // packet still counts towards failsafe recovery.
	    while (xQueueReceive(commandQueue, &cmd, 0) == pdPASS)
	    {
	        float new_pose[6] = {
	            cmd.current_x,
	            cmd.current_y,
	            cmd.current_z,
	            cmd.current_roll,
	            cmd.current_pitch,
	            cmd.current_yaw
	        };

	        state_update(new_pose);

	        float new_target[6] = {
	            cmd.target_x,
	            cmd.target_y,
	            cmd.target_z,
	            cmd.target_roll,
	            cmd.target_pitch,
	            cmd.target_yaw
	        };

	        target_update(new_target, cmd.armed);
	        bench_cmd_consumed(&cmd);
	    }

	    control_loop_tick();
	    bench_after_control_tick();

	    // Only this task refreshes the watchdog: if the control loop stops,
	    // the board resets and the ESCs lose their signal.
	    iwdg_kick();

	    xTaskNotify(
	        commsTaskHandle,
	        COMMS_NOTIFY_TELEMETRY,
	        eSetBits
	    );

	    // On a disarm (including a failsafe), flush the log block so the
	    // records around it reach the card.
	    {
	        static uint8_t prev_armed = 0;
	        uint8_t now_armed = control_loop_get_armed() ? 1U : 0U;

	        if ((prev_armed == 1U) && (now_armed == 0U) && (logQueue != NULL))
	        {
	            LogQueueItem flush_item;

	            flush_item.kind = LOG_ITEM_FLUSH;
	            memset(&flush_item.record, 0, sizeof(flush_item.record));

	            if (xQueueSend(logQueue, &flush_item, 0) != pdPASS)
	            {
	                log_drop_count++;
	            }
	        }

	        prev_armed = now_armed;
	    }

	    // Zero timeout: an SD card busy for 250 ms costs log records, never
	    // a control tick.
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

	            // CRC over everything before the CRC field (it is last).
	            record->crc16 = crc16_ccitt(
	                (const uint8_t *)record,
	                sizeof(LogRecord) - sizeof(record->crc16)
	            );

	            if (xQueueSend(logQueue, &item, 0) != pdPASS)
	            {
	                log_drop_count++;
	            }
	        }
	    }

	    bench_control_done();
	}
}
