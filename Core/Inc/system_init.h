#ifndef SYSTEM_INIT_H
#define SYSTEM_INIT_H

#include "stm32f4xx.h"
#include <stdint.h>

/*
 * Nucleo-F446RE clock tree — target 180 MHz.
 *
 *   HSE      8 MHz   (ST-LINK MCO, NO crystal fitted -> HSEBYP required)
 *   PLL      M=8  -> VCO in  1 MHz
 *            N=360 -> VCO out 360 MHz
 *            P=2  -> SYSCLK  180 MHz
 *            Q=7  -> 51.4 MHz (USB unused; kept legal)
 *   AHB      DIV1  -> HCLK  180 MHz
 *   APB1     DIV4  -> PCLK1  45 MHz, APB1 timer clock  90 MHz
 *   APB2     DIV2  -> PCLK2  90 MHz, APB2 timer clock 180 MHz
 *
 * These three constants are the single source of truth for every BRR,
 * CCR, TRISE and PSC in the drivers. If the clock tree changes, every
 * value derived from these must be rechecked.
 */
#define SYSCLK_HZ   180000000U
#define APB1CLK_HZ   45000000U
#define APB2CLK_HZ   90000000U

/* APBx timer clocks are 2x PCLKx whenever the APBx prescaler is not 1. */
#define APB1_TIMCLK_HZ   (2U * APB1CLK_HZ)   /*  90 MHz — TIM2..7  */
#define APB2_TIMCLK_HZ   (2U * APB2CLK_HZ)   /* 180 MHz — TIM1/8/9..11 */

typedef enum
{
    /* Nominal: 180 MHz from the 8 MHz HSE bypass input. */
    CLOCK_OK_HSE                  = 0,

    /* HSE never reported ready. Still 180 MHz, but from the HSI via a
     * re-tuned PLL (M=16), so accuracy is HSI-grade (+-1%) rather than
     * crystal-grade. UART framing survives this; long-horizon timing
     * does not. */
    CLOCK_DEGRADED_HSI_PLL        = 1,

    /* PLL never locked from either source. Running raw HSI at 16 MHz.
     * Every driver constant above is now wrong by 11.25x — the board
     * boots only so it can report this. */
    CLOCK_FAULT_PLL_TIMEOUT       = 2,

    /* Over-drive handshake timed out. */
    CLOCK_FAULT_OVERDRIVE_TIMEOUT = 3,

    /* SYSCLK never switched to the PLL. */
    CLOCK_FAULT_SWITCH_TIMEOUT    = 4
} ClockStatus;

/* Result of the last system_clock_init(). Readable before any UART is
 * up, so a fault can be reported the moment stdio exists. */
extern volatile ClockStatus g_clock_status;

ClockStatus  system_clock_init(void);
const char  *system_clock_status_str(ClockStatus s);

#endif /* SYSTEM_INIT_H */
