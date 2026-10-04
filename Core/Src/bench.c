/*
 * bench.c - hardware-in-the-loop instrumentation for a bare NUCLEO-F446RE.
 *
 * Compiled only when BENCH_HIL is 1 (bench_config.h). Everything here is
 * passive until a console key asks for it; the laptop side is
 * tools/hil/hil_rtos.py, which sends the keys and parses the "B:" lines.
 *
 * Wiring it can use (all optional, each test reports SKIP without it):
 *   PA9  -> PA10   USART1 loopback: the vehicle's Pi-link UART talks to itself
 *   PWMx -> PA15   one thruster pin into TIM2_CH1 input capture
 *   MPU-6050       on I2C1, PB8 = SCL, PB9 = SDA, 3V3, GND
 *
 * Timing is measured with the DWT cycle counter (PM0214), which runs at
 * HCLK, so 180 cycles = 1 us. The laptop converts.
 *
 * Keys (single character on the ST-LINK VCP; see bench_print_help):
 *   b  report            z  zero statistics
 *   a  inject armed CMDs at 50 Hz     d  inject disarmed CMDs at 50 Hz
 *   1  inject one armed CMD           x  stop injecting
 *   e  inject bad-CRC CMDs            j  inject CMDs wrapped in junk, split
 *   w  256-byte DMA wrap test (with the HT/TC fix)
 *   v  same, HT/TC events ignored: the pre-fix behaviour
 *   p  PWM capture report on PA15     P  toggle PWM signature outputs
 *   i  MPU-6050 stress, i2c_read()    I  same, i2c_read_rm() (RM0390 sequences)
 *   K  suspend comms_task             k  resume it
 *   F  trip configASSERT (fault latch test)
 *   W  hang control_task (watchdog test; needs ENABLE_IWDG)
 */
#include "bench.h"

#if BENCH_HIL

#include <stdio.h>
#include <string.h>
#include "stm32f446xx.h"
#include "console.h"
#include "control_loop.h"
#include "comms_task.h"
#include "uart_packet.h"
#include "packet.h"
#include "crc16.h"
#include "i2c.h"
#include "timer_pwm.h"
#include "timer_timebase.h"
#include "fault_latch.h"

#define CYC_PER_US   180UL

static inline uint32_t cyc(void) { return DWT->CYCCNT; }

/* ===========================================================================
 * Min/max/sum accumulator. Each one has exactly one writer context; the
 * reader (dummy_task) snapshots it inside a critical section.
 * ======================================================================== */
typedef struct
{
    uint32_t n;
    uint32_t min;
    uint32_t max;
    uint64_t sum;
} Stat;

static void stat_reset(volatile Stat *s)
{
    s->n = 0; s->min = 0xFFFFFFFFUL; s->max = 0; s->sum = 0;
}

static inline void stat_add(volatile Stat *s, uint32_t v)
{
    s->n++;
    if (v < s->min) s->min = v;
    if (v > s->max) s->max = v;
    s->sum += v;
}

static uint32_t stat_avg(const Stat *s)
{
    return (s->n != 0U) ? (uint32_t)(s->sum / s->n) : 0U;
}

/* ===========================================================================
 * TIM7 -> control_task timing
 * ======================================================================== */
static volatile uint32_t t7_last_cyc;      /* ISR entry, most recent      */
static volatile uint32_t t7_prev_cyc;
static volatile uint8_t  t7_have_prev;
static volatile Stat     st_t7_period;     /* ISR writer                   */
static volatile Stat     st_wake;          /* control_task writer          */
static volatile Stat     st_exec;          /* control_task writer          */
static volatile Stat     st_cmdlat;        /* control_task writer          */
static volatile uint32_t wake_cyc;
static volatile uint8_t  reset_req_ctl;    /* dummy -> control             */

static volatile Stat     st_fs_ms;         /* control_task writer          */

void bench_tim7_isr(void)
{
    uint32_t now = cyc();
    if (t7_have_prev)
    {
        stat_add(&st_t7_period, now - t7_prev_cyc);
    }
    t7_prev_cyc  = now;
    t7_last_cyc  = now;
    t7_have_prev = 1U;
}

void bench_control_wake(void)
{
    uint32_t now = cyc();
    wake_cyc = now;

    if (reset_req_ctl)
    {
        taskENTER_CRITICAL();              /* masks TIM7 (priority 5)      */
        stat_reset(&st_t7_period);
        t7_have_prev = 0U;
        taskEXIT_CRITICAL();
        stat_reset(&st_wake);
        stat_reset(&st_exec);
        stat_reset(&st_cmdlat);
        stat_reset(&st_fs_ms);
        reset_req_ctl = 0U;
        return;                            /* this tick's ISR stamp may predate the reset */
    }

    stat_add(&st_wake, now - t7_last_cyc);
}

void bench_control_done(void)
{
    stat_add(&st_exec, cyc() - wake_cyc);
}

void bench_failsafe_entered(uint32_t ms_since_last_cmd)
{
    stat_add(&st_fs_ms, ms_since_last_cmd);
}

/* ---- command latency: parse (comms) -> dequeue (control) ---------------- */
void bench_cmd_parsed(CommandPayload *cmd)
{
    uint32_t now = cyc();
    /* reserved[] is zero on the wire and unused by control_task; carry the
     * parse timestamp in the local copy so each command is timed exactly. */
    memcpy(&cmd->reserved[0], &now, sizeof(now));
    cmd->reserved[4] = 0xB5;
}

void bench_cmd_consumed(const CommandPayload *cmd)
{
    if (cmd->reserved[4] != 0xB5)
    {
        return;
    }
    uint32_t t;
    memcpy(&t, &cmd->reserved[0], sizeof(t));
    stat_add(&st_cmdlat, cyc() - t);
}

/* ===========================================================================
 * Control-task actions: PWM signature, hang
 * ======================================================================== */
static volatile uint8_t pwm_signature;
static volatile uint8_t hang_req;

void bench_after_control_tick(void)
{
    if (pwm_signature)
    {
        /* Channel k at 1100 + 100k us: a jumper on any pin identifies the
         * channel by its pulse width, which checks the pin routing as well
         * as the timer setup. Written after applyPWM(), every tick. */
        for (uint8_t ch = 0; ch < 8U; ch++)
        {
            pwm_set_us(ch, (uint16_t)(1100U + 100U * ch));
        }
    }

    if (hang_req)
    {
        /* Stop refreshing the watchdog the way a hung control loop would:
         * interrupts stay on, every other task keeps its priority, only
         * control_task stops. */
        for (;;)
        {
        }
    }
}

/* ===========================================================================
 * USART1 loopback injector (comms_task context)
 * ======================================================================== */
typedef enum
{
    INJ_OFF = 0,
    INJ_VALID,
    INJ_BADCRC,
    INJ_JUNK,
    INJ_SINGLE
} InjMode;

static volatile InjMode inj_mode;
static volatile uint8_t inj_armed;
static volatile uint8_t inj_single_pending;

static uint32_t inj_seq;
static uint32_t inj_sent;
static uint32_t rng_state = 0x2545F491UL;

#define RT_RING 32U
static float    rt_ring[RT_RING];
static uint8_t  rt_head;
static uint32_t tlm_ok, tlm_bad, rt_ok, rt_bad, rt_poison;
static volatile uint8_t reset_req_comms;

#define POISON_Z  (-999.0f)

static uint32_t rnd(void)
{
    rng_state = rng_state * 1664525UL + 1013904223UL;
    return rng_state >> 8;
}

static void build_cmd_frame(uint8_t *f, float z, uint8_t armed, uint8_t corrupt)
{
    CommandPayload c;
    memset(&c, 0, sizeof(c));
    c.current_z = z;
    c.armed     = armed;
    c.seq       = (uint8_t)inj_seq;

    f[0] = STX1;
    f[1] = STX2;
    f[2] = PAYLOAD_LEN;
    f[3] = TYPE_CMD;
    memcpy(&f[4], &c, sizeof(c));
    uint16_t crc = crc16_ccitt(&f[2], 2U + PAYLOAD_LEN);
    if (corrupt)
    {
        crc ^= 0x0001U;
    }
    f[PACKET_SIZE - 2] = (uint8_t)(crc >> 8);
    f[PACKET_SIZE - 1] = (uint8_t)crc;
}

static float next_z(void)
{
    inj_seq++;
    float z = 1000.0f + (float)(inj_seq % 1000000UL);   /* exact in float32 */
    rt_ring[rt_head] = z;
    rt_head = (uint8_t)((rt_head + 1U) % RT_RING);
    return z;
}

/* ---- 256-byte DMA wrap test --------------------------------------------- */
typedef enum { WRAP_IDLE = 0, WRAP_ARMED, WRAP_SETTLE } WrapState;

static volatile WrapState wrap_state;
static volatile uint8_t   wrap_legacy_req;
static volatile uint8_t   dma_ignore;        /* read by the DMA ISR          */
static volatile uint32_t  dma_bytes;         /* bytes moved DMA buf -> ring  */
static volatile uint32_t  dma_events;        /* HT/TC events processed       */
static volatile uint32_t  idle_events;
static uint32_t           wrap_snapshot;
static uint8_t            wrap_settle_ticks;
static uint32_t           wrap_last_got;
static uint8_t            wrap_last_legacy;
static uint8_t            wrap_have_result;

void bench_uart1_rx_bytes(uint16_t n, bool from_dma_event)
{
    dma_bytes += n;
    if (from_dma_event) dma_events++; else idle_events++;
}

bool bench_uart1_dma_events_ignored(void)
{
    return dma_ignore != 0U;
}

bool bench_comms_skip_telemetry(void)
{
    if (wrap_state == WRAP_ARMED)
    {
        static uint8_t burst[256];
        for (uint16_t i = 0; i < 256U; i++)
        {
            burst[i] = (uint8_t)i;           /* no 0xAA 0x55 pair anywhere  */
        }
        dma_ignore    = wrap_legacy_req;
        wrap_snapshot = dma_bytes;

        /* Exactly 256 bytes with no idle gap. The scheduler is suspended so
         * control_task cannot open a gap by preempting the TX loop; the
         * USART1 and DMA interrupts still run, which is the point. */
        vTaskSuspendAll();
        uart1_write_buf(burst, 256U);
        (void)xTaskResumeAll();

        wrap_settle_ticks = 3U;              /* last byte + IDLE detection */
        wrap_state = WRAP_SETTLE;
        return true;
    }

    if (wrap_state == WRAP_SETTLE)
    {
        if (--wrap_settle_ticks == 0U)
        {
            wrap_last_got    = dma_bytes - wrap_snapshot;
            wrap_last_legacy = wrap_legacy_req;
            wrap_have_result = 1U;
            dma_ignore       = 0U;
            wrap_state       = WRAP_IDLE;
            console_printf("B:wrap sent=256 got=%lu mode=%s",
                           (unsigned long)wrap_last_got,
                           wrap_last_legacy ? "legacy" : "fix");
            return false;
        }
        return true;                         /* keep the line quiet         */
    }

    return false;
}

void bench_comms_after_tx(void)
{
    if (reset_req_comms)
    {
        tlm_ok = tlm_bad = rt_ok = rt_bad = rt_poison = 0;
        inj_sent = 0;
        reset_req_comms = 0U;
    }

    if (wrap_state != WRAP_IDLE)
    {
        return;
    }

    uint8_t f[PACKET_SIZE];
    InjMode mode = inj_mode;

    if (inj_single_pending)
    {
        inj_single_pending = 0U;
        build_cmd_frame(f, next_z(), 1U, 0U);
        uart1_write_buf(f, PACKET_SIZE);
        inj_sent++;
        return;
    }

    switch (mode)
    {
        case INJ_VALID:
            build_cmd_frame(f, next_z(), inj_armed, 0U);
            uart1_write_buf(f, PACKET_SIZE);
            inj_sent++;
            break;

        case INJ_BADCRC:
            build_cmd_frame(f, POISON_Z, 1U, 1U);
            uart1_write_buf(f, PACKET_SIZE);
            inj_sent++;
            break;

        case INJ_JUNK:
        {
            uint8_t junk[32];
            uint8_t n = (uint8_t)(1U + rnd() % 11U);
            for (uint8_t i = 0; i < n; i++)
            {
                junk[i] = (uint8_t)rnd();
            }
            if ((rnd() % 10U) < 3U)          /* decoy header, truncated     */
            {
                junk[n++] = STX1; junk[n++] = STX2;
                junk[n++] = PAYLOAD_LEN; junk[n++] = TYPE_CMD;
                for (uint8_t i = 0; i < 12U; i++) junk[n++] = 0U;
            }
            uart1_write_buf(junk, n);

            build_cmd_frame(f, next_z(), inj_armed, 0U);
            if ((rnd() & 1U) != 0U)
            {
                /* Split across two IDLE events: the line goes quiet for 2 ms
                 * mid-frame, so the parser sees a partial frame first. */
                uint16_t cut = (uint16_t)(1U + rnd() % (PACKET_SIZE - 1U));
                uart1_write_buf(f, cut);
                vTaskDelay(pdMS_TO_TICKS(2));
                uart1_write_buf(&f[cut], (uint16_t)(PACKET_SIZE - cut));
            }
            else
            {
                uart1_write_buf(f, PACKET_SIZE);
            }
            inj_sent++;
            break;
        }

        default:
            break;
    }
}

void bench_telemetry_looped(const uint8_t *payload)
{
    if (payload == NULL)
    {
        tlm_bad++;                           /* sync+LEN+TYPE ok, CRC not   */
        return;
    }

    TelemetryPayload t;
    memcpy(&t, payload, sizeof(t));
    tlm_ok++;

    if (t.depth_m == POISON_Z)
    {
        rt_poison++;
        return;
    }
    if (t.depth_m == 0.0f)
    {
        return;                              /* no command consumed yet     */
    }
    for (uint8_t i = 0; i < RT_RING; i++)
    {
        if (rt_ring[i] == t.depth_m)
        {
            rt_ok++;
            return;
        }
    }
    rt_bad++;
}

/* ===========================================================================
 * PWM measured on PA15: TIM2_CH1 input capture, both edges.
 *
 * TIM2 is already the 1 MHz free-running micros() counter; capture only
 * latches CNT into CCR1 and does not disturb it. The PWM timers count the
 * same 1 us ticks from the same clock tree, so this checks PSC/ARR/CCR and
 * the pin routing exactly, and SYSCLK only relative to itself: the clock's
 * absolute rate is checked against the laptop in the 'b' report instead.
 * ======================================================================== */
static volatile uint32_t cap_rise;
static volatile uint8_t  cap_have_rise;
static volatile Stat     st_cap_hi;
static volatile Stat     st_cap_per;

/* Reader-side reset: TIM2 (priority 6) is masked by the critical section,
 * so this cannot interleave with the handler, and it works even when no
 * edges are arriving. */
static void cap_reset(void)
{
    taskENTER_CRITICAL();
    stat_reset(&st_cap_hi);
    stat_reset(&st_cap_per);
    cap_have_rise = 0U;
    taskEXIT_CRITICAL();
}

static void pa15_capture_init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;

    /* PA15 resets as JTDI: AF0 with a pull-up. Clear both before taking it. */
    GPIOA->MODER  &= ~(3U << (2 * 15));
    GPIOA->MODER  |=  (2U << (2 * 15));
    GPIOA->PUPDR  &= ~(3U << (2 * 15));
    GPIOA->PUPDR  |=  (2U << (2 * 15));            /* pull-down: open = low  */
    GPIOA->AFR[1] &= ~(0xFU << ((15 - 8) * 4));
    GPIOA->AFR[1] |=  (1U   << ((15 - 8) * 4));    /* AF1 = TIM2_CH1         */

    TIM2->CCER  &= ~TIM_CCER_CC1E;
    TIM2->CCMR1 &= ~(TIM_CCMR1_CC1S | TIM_CCMR1_IC1F | TIM_CCMR1_IC1PSC);
    TIM2->CCMR1 |=  (1U << TIM_CCMR1_CC1S_Pos)     /* CC1 input, TI1         */
                 |  (3U << TIM_CCMR1_IC1F_Pos);    /* 8 samples at 90 MHz    */
    TIM2->CCER  |=  TIM_CCER_CC1P | TIM_CCER_CC1NP /* both edges             */
                 |  TIM_CCER_CC1E;
    TIM2->SR     = ~(TIM_SR_CC1IF | TIM_SR_CC1OF);
    TIM2->DIER  |=  TIM_DIER_CC1IE;

    /* 6 = below the kernel ceiling; the handler calls no FreeRTOS API. */
    NVIC_SetPriority(TIM2_IRQn, 6);
    NVIC_EnableIRQ(TIM2_IRQn);
}

void TIM2_IRQHandler(void)
{
    uint32_t sr = TIM2->SR;
    if (sr & TIM_SR_CC1IF)
    {
        uint32_t c    = TIM2->CCR1;                 /* clears CC1IF          */
        uint32_t high = GPIOA->IDR & (1U << 15);

        if (high)
        {
            if (cap_have_rise)
            {
                stat_add(&st_cap_per, c - cap_rise);
            }
            cap_rise      = c;
            cap_have_rise = 1U;
        }
        else if (cap_have_rise)
        {
            stat_add(&st_cap_hi, c - cap_rise);
        }
    }
    if (sr & TIM_SR_CC1OF)
    {
        TIM2->SR = ~TIM_SR_CC1OF;
    }
}

/* ===========================================================================
 * CPU load from FreeRTOS run-time stats (TIM2 counter, 1 us).
 * ======================================================================== */
#define MAX_TASKS 10U
typedef struct
{
    const char  *name;
    TaskHandle_t h;
    uint32_t     base;
} TaskEntry;

static TaskEntry tasks[MAX_TASKS];
static uint8_t   n_tasks;
static uint32_t  cpu_base_total;

void bench_register_task(const char *short_name, TaskHandle_t handle)
{
    if ((n_tasks < MAX_TASKS) && (handle != NULL))
    {
        tasks[n_tasks].name = short_name;
        tasks[n_tasks].h    = handle;
        n_tasks++;
    }
}

static void cpu_window_reset(void)
{
    cpu_base_total = TIM2->CNT;
    for (uint8_t i = 0; i < n_tasks; i++)
    {
        tasks[i].base = ulTaskGetRunTimeCounter(tasks[i].h);
    }
}

/* ===========================================================================
 * MSP high-water mark. Handlers run on the MSP; with every interrupt in this
 * design masked, nothing is on it, so the whole reserve can be painted.
 * ======================================================================== */
extern uint32_t _estack;
extern uint32_t _Min_Stack_Size;              /* linker absolute symbol      */
#define MSP_PAINT 0xA5A5A5A5UL

static uint32_t *msp_base;
static uint32_t  msp_size;

void bench_dummy_first_run(void)
{
    uint32_t top  = (uint32_t)&_estack;
    msp_size      = (uint32_t)&_Min_Stack_Size;
    msp_base      = (uint32_t *)(top - msp_size);

    taskENTER_CRITICAL();
    for (uint32_t *p = msp_base; p < (uint32_t *)top; p++)
    {
        *p = MSP_PAINT;
    }
    taskEXIT_CRITICAL();

    /* The idle task's handle only exists once the scheduler is running. */
    bench_register_task("Idle", xTaskGetIdleTaskHandle());
    cpu_window_reset();
}

static uint32_t msp_used(void)
{
    if (msp_base == NULL)
    {
        return 0;
    }
    uint32_t *p   = msp_base;
    uint32_t *top = (uint32_t *)((uint32_t)msp_base + msp_size);
    while ((p < top) && (*p == MSP_PAINT))
    {
        p++;
    }
    return (uint32_t)top - (uint32_t)p;
}

/* ===========================================================================
 * MPU-6050 I2C stress (bar30_task context: it owns I2C1)
 * ======================================================================== */
#define MPU_ADDR        0x68U
#define MPU_WHO_AM_I    0x75U
#define MPU_PWR_MGMT_1  0x6BU
#define MPU_ACCEL_XOUT  0x3BU
#define MPU_TEMP_OUT    0x41U
#define MPU_CYCLES      500U

static volatile uint8_t mpu_req;             /* 0 none, 1 legacy, 2 RM0390 */

typedef I2C_Status (*ReadFn)(uint8_t, uint8_t *, uint8_t);

typedef struct
{
    uint32_t i2c_err, stale, e_who, e_tmp, e_acc, e_14;
} MpuErr;

static I2C_Status mpu_read_reg(ReadFn rd, uint8_t reg, uint8_t *buf, uint8_t n,
                               MpuErr *e)
{
    I2C_Status st = i2c_write(MPU_ADDR, &reg, 1U);
    if (st != I2C_OK)
    {
        return st;
    }
    /* A byte left in DR by the previous transfer: the signature of a read
     * whose ACK/STOP timing was overrun (RM0390 master receiver, N <= 2 and
     * the last three bytes of N > 2). Count it before it corrupts buf[0]. */
    if (I2C1->SR1 & I2C_SR1_RXNE)
    {
        e->stale++;
    }
    return rd(MPU_ADDR, buf, n);
}

static int16_t be16(const uint8_t *b) { return (int16_t)(((uint16_t)b[0] << 8) | b[1]); }

static bool accel_plausible(const uint8_t *b)
{
    int32_t x = be16(&b[0]), y = be16(&b[2]), z = be16(&b[4]);
    int64_t m2 = (int64_t)x * x + (int64_t)y * y + (int64_t)z * z;
    const int64_t lo = (int64_t)(0.6 * 16384) * (int64_t)(0.6 * 16384);
    const int64_t hi = (int64_t)(1.4 * 16384) * (int64_t)(1.4 * 16384);
    return (m2 >= lo) && (m2 <= hi);                  /* +/-2 g range, at rest */
}

static bool temp_plausible(const uint8_t *b)
{
    int32_t raw = be16(b);                            /* T = raw/340 + 36.53  */
    return (raw > (-10 - 37) * 340) && (raw < (70 - 37) * 340);
}

static void mpu_stress(uint8_t which)
{
    ReadFn rd = (which == 2U) ? i2c_read_rm : i2c_read;
    const char *name = (which == 2U) ? "rm0390" : "legacy";
    MpuErr e;
    memset(&e, 0, sizeof(e));

    uint8_t wake[2] = { MPU_PWR_MGMT_1, 0x00U };
    uint8_t who0 = 0;
    I2C_Status st = i2c_write(MPU_ADDR, wake, 2U);
    if (st == I2C_OK)
    {
        vTaskDelay(pdMS_TO_TICKS(50));               /* clock settles        */
        st = mpu_read_reg(rd, MPU_WHO_AM_I, &who0, 1U, &e);
    }
    if (st != I2C_OK)
    {
        console_printf("B:mpu absent (%s)", i2c_status_str(st));
        return;
    }

    uint8_t pwr = 0xFF;                 /* PWR_MGMT_1: 0x40 = still asleep */
    (void)mpu_read_reg(rd, MPU_PWR_MGMT_1, &pwr, 1U, &e);

    uint32_t t14_min = 0xFFFFFFFFUL, t14_max = 0;
    uint8_t  b[14];

    for (uint32_t k = 0; k < MPU_CYCLES; k++)
    {
        uint8_t who = 0;
        if (mpu_read_reg(rd, MPU_WHO_AM_I, &who, 1U, &e) != I2C_OK) e.i2c_err++;
        else if (who != who0)                                         e.e_who++;

        if (mpu_read_reg(rd, MPU_TEMP_OUT, b, 2U, &e) != I2C_OK)      e.i2c_err++;
        else if (!temp_plausible(b))                                  e.e_tmp++;

        if (mpu_read_reg(rd, MPU_ACCEL_XOUT, b, 6U, &e) != I2C_OK)    e.i2c_err++;
        else if (!accel_plausible(b))                                 e.e_acc++;

        uint32_t t0 = micros();
        I2C_Status s14 = mpu_read_reg(rd, MPU_ACCEL_XOUT, b, 14U, &e);
        uint32_t dt = micros() - t0;
        if (s14 != I2C_OK)                                            e.i2c_err++;
        else
        {
            if (!accel_plausible(b) || !temp_plausible(&b[6]))        e.e_14++;
            if (dt < t14_min) t14_min = dt;
            if (dt > t14_max) t14_max = dt;
        }
    }

    console_printf("B:mpu m=%s n=%lu who=0x%02X ewho=%lu etmp=%lu eacc=%lu e14=%lu",
                   name, (unsigned long)MPU_CYCLES, who0,
                   (unsigned long)e.e_who, (unsigned long)e.e_tmp,
                   (unsigned long)e.e_acc, (unsigned long)e.e_14);
    console_printf("B:mpu2 m=%s i2cerr=%lu stale=%lu t14=%lu-%lu pwr=0x%02X",
                   name, (unsigned long)e.i2c_err, (unsigned long)e.stale,
                   (unsigned long)t14_min, (unsigned long)t14_max, pwr);
}

void bench_i2c_service(void)
{
    uint8_t which = mpu_req;
    if (which != 0U)
    {
        mpu_req = 0U;
        mpu_stress(which);
    }
}

/* ===========================================================================
 * Init
 * ======================================================================== */
void bench_init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;

    stat_reset(&st_t7_period);
    stat_reset(&st_wake);
    stat_reset(&st_exec);
    stat_reset(&st_cmdlat);
    stat_reset(&st_fs_ms);
    stat_reset(&st_cap_hi);
    stat_reset(&st_cap_per);

    pa15_capture_init();
}

/* ===========================================================================
 * Console (dummy_task context)
 * ======================================================================== */
static void snap(const volatile Stat *src, Stat *dst)
{
    taskENTER_CRITICAL();
    dst->n = src->n; dst->min = src->min; dst->max = src->max; dst->sum = src->sum;
    taskEXIT_CRITICAL();
    if (dst->n == 0U)
    {
        dst->min = 0;
    }
}

static void print_stat(const char *tag, const volatile Stat *s)
{
    Stat c;
    snap(s, &c);
    console_printf("B:%s n=%lu min=%lu max=%lu avg=%lu", tag,
                   (unsigned long)c.n, (unsigned long)c.min,
                   (unsigned long)c.max, (unsigned long)stat_avg(&c));
}

static void print_cpu(void)
{
    uint32_t total = TIM2->CNT - cpu_base_total;
    char line[CONSOLE_LINE_LEN];
    uint32_t pos = 0;
    uint32_t idle_pm = 0;

    if (total == 0U)
    {
        return;
    }

    TaskHandle_t idle = xTaskGetIdleTaskHandle();
    pos = (uint32_t)snprintf(line, sizeof(line), "B:cpu win_us=%lu", (unsigned long)total);
    for (uint8_t i = 0; i < n_tasks; i++)
    {
        uint32_t d  = ulTaskGetRunTimeCounter(tasks[i].h) - tasks[i].base;
        uint32_t pm = (uint32_t)(((uint64_t)d * 1000U) / total);
        if (tasks[i].h == idle)
        {
            idle_pm = pm;
            continue;
        }
        if (pos > (sizeof(line) - 12U))
        {
            console_printf("%s", line);
            pos = (uint32_t)snprintf(line, sizeof(line), "B:cpu");
        }
        pos += (uint32_t)snprintf(&line[pos], sizeof(line) - pos, " %s=%lu",
                                  tasks[i].name, (unsigned long)pm);
    }
    console_printf("%s", line);
    console_printf("B:cpu idle=%lu (permille of window)", (unsigned long)idle_pm);
}

static void print_report(void)
{
    uint32_t tick, us, cy;
    taskENTER_CRITICAL();
    tick = (uint32_t)xTaskGetTickCount();
    us   = TIM2->CNT;
    cy   = DWT->CYCCNT;
    taskEXIT_CRITICAL();

    console_printf("B:clk tick=%lu us=%lu cyc=%lu",
                   (unsigned long)tick, (unsigned long)us, (unsigned long)cy);
    print_stat("t7", &st_t7_period);
    print_stat("wake", &st_wake);
    print_stat("exec", &st_exec);
    print_stat("cmdlat", &st_cmdlat);
    print_stat("fs_ms", &st_fs_ms);
    console_printf("B:inj sent=%lu tlm=%lu tbad=%lu rtok=%lu rtbad=%lu poison=%lu",
                   (unsigned long)inj_sent, (unsigned long)tlm_ok, (unsigned long)tlm_bad,
                   (unsigned long)rt_ok, (unsigned long)rt_bad,
                   (unsigned long)rt_poison);
    console_printf("B:rx dma=%lu ringdrop=%lu dmaev=%lu idleev=%lu",
                   (unsigned long)dma_bytes, (unsigned long)rx_dropped_count(),
                   (unsigned long)dma_events, (unsigned long)idle_events);
    console_printf("B:ctl fs=%d armed=%d rec=%u valid=%lu qdrop=%lu",
                   control_loop_in_failsafe() ? 1 : 0,
                   control_loop_get_armed() ? 1 : 0,
                   (unsigned)control_loop_recovery_count(),
                   (unsigned long)comms_cmd_valid(),
                   (unsigned long)comms_cmd_drops());
    print_cpu();
    console_printf("B:msp used=%lu size=%lu heap_free=%u heap_min=%u",
                   (unsigned long)msp_used(), (unsigned long)msp_size,
                   (unsigned)xPortGetFreeHeapSize(),
                   (unsigned)xPortGetMinimumEverFreeHeapSize());
    console_printf("B:end");
}

static void print_pwm(void)
{
    Stat hi, per;
    snap(&st_cap_hi, &hi);
    snap(&st_cap_per, &per);
    console_printf("B:pwm n=%lu hi=%lu-%lu per=%lu-%lu sig=%u",
                   (unsigned long)hi.n, (unsigned long)hi.min, (unsigned long)hi.max,
                   (unsigned long)per.min, (unsigned long)per.max,
                   (unsigned)pwm_signature);
    cap_reset();
}

void bench_print_help(void)
{
    console_printf("bench: b=report z=zero p=pwm P=pwm-signature");
    console_printf("bench: a/d=inject armed/disarmed 1=one x=stop e=badcrc j=junk");
    console_printf("bench: w/v=dma wrap fix/legacy i/I=mpu legacy/rm0390");
    console_printf("bench: K/k=suspend/resume comms F=fault W=hang control");
}

bool bench_console_key(char c)
{
    switch (c)
    {
        case 'b': print_report();                                       return true;
        case 'z':
            reset_req_ctl   = 1U;
            reset_req_comms = 1U;
            cap_reset();
            cpu_window_reset();
            console_printf("B:zeroed");
            return true;

        case 'a': inj_armed = 1U; inj_mode = INJ_VALID;  console_printf("B:inj armed");    return true;
        case 'd': inj_armed = 0U; inj_mode = INJ_VALID;  console_printf("B:inj disarmed"); return true;
        case 'x': inj_mode = INJ_OFF;                    console_printf("B:inj off");      return true;
        case '1': inj_single_pending = 1U;               console_printf("B:inj one");      return true;
        case 'e': inj_mode = INJ_BADCRC;                 console_printf("B:inj badcrc");   return true;
        case 'j': inj_armed = 0U; inj_mode = INJ_JUNK;   console_printf("B:inj junk");     return true;

        case 'w':
        case 'v':
            if (wrap_state == WRAP_IDLE)
            {
                inj_mode        = INJ_OFF;
                wrap_legacy_req = (c == 'v') ? 1U : 0U;
                wrap_state      = WRAP_ARMED;
            }
            return true;

        case 'p': print_pwm();                                           return true;
        case 'P':
            pwm_signature ^= 1U;
            console_printf("B:pwm signature=%u", (unsigned)pwm_signature);
            return true;

        case 'i': mpu_req = 1U; console_printf("B:mpu queued legacy"); return true;
        case 'I': mpu_req = 2U; console_printf("B:mpu queued rm0390"); return true;

        case 'K': vTaskSuspend(commsTaskHandle); console_printf("B:comms suspended"); return true;
        case 'k': vTaskResume(commsTaskHandle);  console_printf("B:comms resumed");   return true;

        case 'F':
            console_printf("B:fault tripping configASSERT");
            vTaskDelay(pdMS_TO_TICKS(100));        /* let the line drain */
            configASSERT(c == 0);
            return true;

        case 'W':
            console_printf("B:hang control_task");
            hang_req = 1U;
            return true;

        default:
            return false;
    }
}

#endif /* BENCH_HIL */
