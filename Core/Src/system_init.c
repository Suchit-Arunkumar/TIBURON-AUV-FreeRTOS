#include "system_init.h"

/*
 * Composed register values.
 *
 * Both registers are written with '=', not '|='. PLLCFGR resets to
 * 0x24003010 (PLLM=16, PLLN=192, PLLQ=4, PLLR=2) and the CFGR prescaler
 * fields are live on a warm reset, so OR-ing onto them produces a
 * silently wrong clock rather than a compile error. That was defect B1
 * and B2 in the Phase A audit.
 */

/* PLLM=8, PLLN=360, PLLP=2(00), PLLSRC=HSE, PLLQ=7, PLLR=2.
 * PLLR must stay in 2..7 even though it is unused here — 0 is reserved. */
#define PLLCFGR_HSE_180MHZ                       \
    ( (8UL   << RCC_PLLCFGR_PLLM_Pos)   |        \
      (360UL << RCC_PLLCFGR_PLLN_Pos)   |        \
      (0UL   << RCC_PLLCFGR_PLLP_Pos)   |        \
      RCC_PLLCFGR_PLLSRC_HSE            |        \
      (7UL   << RCC_PLLCFGR_PLLQ_Pos)   |        \
      (2UL   << RCC_PLLCFGR_PLLR_Pos) )

/* Same VCO, driven from the 16 MHz HSI: M=16 keeps VCO-in at 1 MHz, so
 * N/P/Q are unchanged and SYSCLK is still 180 MHz. That is what makes
 * the HSE fallback worth having — every BRR/CCR/PSC in the drivers
 * stays valid, and only the accuracy degrades. */
#define PLLCFGR_HSI_180MHZ                       \
    ( (16UL  << RCC_PLLCFGR_PLLM_Pos)   |        \
      (360UL << RCC_PLLCFGR_PLLN_Pos)   |        \
      (0UL   << RCC_PLLCFGR_PLLP_Pos)   |        \
      RCC_PLLCFGR_PLLSRC_HSI            |        \
      (7UL   << RCC_PLLCFGR_PLLQ_Pos)   |        \
      (2UL   << RCC_PLLCFGR_PLLR_Pos) )

/* HPRE=DIV1, PPRE1=DIV4, PPRE2=DIV2, SW left at 0 (HSI) for now. */
#define CFGR_PRESCALERS                          \
    ( RCC_CFGR_HPRE_DIV1  |                      \
      RCC_CFGR_PPRE1_DIV4 |                      \
      RCC_CFGR_PPRE2_DIV2 )

/*
 * Bounded-wait budget.
 *
 * Every spin below runs before the PLL is switched in, i.e. on the
 * 16 MHz HSI. The loop body is a load, a test and a branch, so roughly
 * 4 cycles; 0x00080000 iterations is about 130 ms. HSE bypass off the
 * ST-LINK MCO settles in well under 1 ms and PLL lock takes ~200 us, so
 * this is three orders of magnitude of margin. It exists to make a dead
 * input diagnosable, not to race a slow one.
 */
#define CLOCK_WAIT_ITERATIONS   0x00080000UL

volatile ClockStatus g_clock_status = CLOCK_FAULT_PLL_TIMEOUT;

/* Returns 1 if the masked bits in *reg reached the wanted state before
 * the budget expired, 0 on timeout. */
static uint32_t wait_for_bits(volatile uint32_t *reg,
                              uint32_t mask,
                              uint32_t want_set)
{
    for (uint32_t i = 0; i < CLOCK_WAIT_ITERATIONS; i++)
    {
        uint32_t is_set = ((*reg & mask) != 0UL) ? 1UL : 0UL;

        if (is_set == want_set)
        {
            return 1UL;
        }
    }

    return 0UL;
}

/*
 * Steps 4-6 of the 180 MHz sequence: over-drive on, over-drive switch,
 * then flash latency. Split out only because the HSE and HSI paths need
 * it verbatim.
 *
 * Skipping this leaves the core at 180 MHz on a regulator provisioned
 * for less, reading flash with too few wait states. The failure is not
 * graceful.
 */
static uint32_t enable_overdrive_and_flash(void)
{
    /* 4. Over-drive enable. */
    PWR->CR |= PWR_CR_ODEN;

    if (!wait_for_bits(&PWR->CSR, PWR_CSR_ODRDY, 1UL))
    {
        return 0UL;
    }

    /* 5. Over-drive switch. */
    PWR->CR |= PWR_CR_ODSWEN;

    if (!wait_for_bits(&PWR->CSR, PWR_CSR_ODSWRDY, 1UL))
    {
        return 0UL;
    }

    /*
     * 6. Flash: 5 wait states for 180 MHz at 2.7-3.6 V, plus prefetch
     *    and both caches. Composed write — LATENCY is a 4-bit field and
     *    OR-ing onto it cannot lower a stale value.
     */
    FLASH->ACR = FLASH_ACR_LATENCY_5WS |
                 FLASH_ACR_PRFTEN      |
                 FLASH_ACR_ICEN        |
                 FLASH_ACR_DCEN;

    /* Readback: the latency field must actually have taken. */
    if ((FLASH->ACR & FLASH_ACR_LATENCY) != FLASH_ACR_LATENCY_5WS)
    {
        return 0UL;
    }

    return 1UL;
}

/* Last resort: leave the chip on the raw 16 MHz HSI so it can boot far
 * enough to say why. Prescalers go to DIV1 and flash to 0 WS, because
 * nothing downstream is valid at this point anyway. */
static void fall_back_to_raw_hsi(void)
{
    RCC->CR |= RCC_CR_HSION;
    (void)wait_for_bits(&RCC->CR, RCC_CR_HSIRDY, 1UL);

    FLASH->ACR = FLASH_ACR_PRFTEN | FLASH_ACR_ICEN | FLASH_ACR_DCEN;

    RCC->CFGR = 0UL;   /* HPRE/PPRE1/PPRE2 = DIV1, SW = HSI */

    RCC->CR &= ~RCC_CR_PLLON;
    (void)wait_for_bits(&RCC->CR, RCC_CR_PLLRDY, 0UL);

    SystemCoreClockUpdate();
}

ClockStatus system_clock_init(void)
{
    ClockStatus status = CLOCK_OK_HSE;

    /*
     * 1. Power interface clock. The PWR registers touched below are
     *    unwritable until this is on.
     */
    RCC->APB1ENR |= RCC_APB1ENR_PWREN;
    (void)RCC->APB1ENR;   /* force the write to land before PWR is touched */

    /*
     * 2. Regulator voltage Scale 1. Required for any SYSCLK above
     *    168 MHz, and a precondition for the over-drive handshake.
     */
    PWR->CR = (PWR->CR & ~PWR_CR_VOS) | PWR_CR_VOS;   /* VOS = 0b11 */

    /*
     * 3a. HSE in bypass. The Nucleo-F446RE has no crystal fitted; the
     *     8 MHz arrives as a digital clock from the ST-LINK MCO, so
     *     HSEBYP must be set. HSEBYP is only writable while HSEON is
     *     clear, hence the explicit clear first — this matters on a warm
     *     reset where HSE may already be running.
     */
    RCC->CR &= ~(RCC_CR_HSEON | RCC_CR_HSEBYP);
    (void)wait_for_bits(&RCC->CR, RCC_CR_HSERDY, 0UL);

    RCC->CR |= RCC_CR_HSEBYP;
    RCC->CR |= RCC_CR_HSEON;

    uint32_t hse_ok = wait_for_bits(&RCC->CR, RCC_CR_HSERDY, 1UL);

    /*
     * 3b. PLL. PLLCFGR is write-protected while PLLON is set, so the
     *     PLL is stopped before it is reconfigured.
     */
    RCC->CR &= ~RCC_CR_PLLON;
    (void)wait_for_bits(&RCC->CR, RCC_CR_PLLRDY, 0UL);

    if (hse_ok)
    {
        RCC->PLLCFGR = PLLCFGR_HSE_180MHZ;
    }
    else
    {
        /* Keep 180 MHz, lose crystal accuracy. See the enum comment. */
        status = CLOCK_DEGRADED_HSI_PLL;
        RCC->CR &= ~RCC_CR_HSEON;
        RCC->PLLCFGR = PLLCFGR_HSI_180MHZ;
    }

    RCC->CR |= RCC_CR_PLLON;

    if (!wait_for_bits(&RCC->CR, RCC_CR_PLLRDY, 1UL))
    {
        fall_back_to_raw_hsi();
        g_clock_status = CLOCK_FAULT_PLL_TIMEOUT;
        return g_clock_status;
    }

    /* 4, 5, 6. Over-drive then flash latency — before any clock rises. */
    if (!enable_overdrive_and_flash())
    {
        fall_back_to_raw_hsi();
        g_clock_status = CLOCK_FAULT_OVERDRIVE_TIMEOUT;
        return g_clock_status;
    }

    /*
     * 7. Bus prescalers, in a single write, while SYSCLK is still the
     *    16 MHz HSI. Setting these before the switch means APB1 and APB2
     *    are never momentarily overclocked.
     */
    RCC->CFGR = CFGR_PRESCALERS;

    /*
     * 8. Switch SYSCLK to the PLL.
     */
    RCC->CFGR = CFGR_PRESCALERS | RCC_CFGR_SW_PLL;

    for (uint32_t i = 0; i < CLOCK_WAIT_ITERATIONS; i++)
    {
        if ((RCC->CFGR & RCC_CFGR_SWS) == RCC_CFGR_SWS_PLL)
        {
            /*
             * Set the CMSIS variable from ground truth rather than asking
             * SystemCoreClockUpdate() to re-derive it from HSE_VALUE and
             * PLLCFGR. Both PLL configurations above are built to land on
             * exactly SYSCLK_HZ, so this is the authoritative value and
             * the derivation is just another chance to be wrong.
             *
             * FreeRTOS does not read this - configCPU_CLOCK_HZ is a
             * literal - but anything else that consults SystemCoreClock
             * now gets the right answer.
             */
            SystemCoreClock = SYSCLK_HZ;

            g_clock_status = status;
            return status;
        }
    }

    fall_back_to_raw_hsi();
    g_clock_status = CLOCK_FAULT_SWITCH_TIMEOUT;
    return g_clock_status;
}

/*
 * LD2 fault blink - the only diagnostic that survives a broken clock.
 *
 * On any fatal clock status the core is on the raw 16 MHz HSI. Every UART
 * divisor in the tree assumes a 45 MHz APB1, so USART2 would run at
 * 16e6/391 = 40.9 kBaud against a host expecting 115200 and the console
 * would be unreadable garbage. The LED is all that is left.
 *
 * Pattern: the status code as a count of short pulses, then a long gap,
 * forever. Count the blinks to identify the fault without a debugger:
 *
 *   2 blinks -> CLOCK_FAULT_PLL_TIMEOUT
 *   3 blinks -> CLOCK_FAULT_OVERDRIVE_TIMEOUT
 *   4 blinks -> CLOCK_FAULT_SWITCH_TIMEOUT
 *
 * Timing is cycle-counted against 16 MHz, not tick-based - there is no
 * scheduler and SysTick is not running.
 */

#define HSI_FALLBACK_HZ         16000000UL
#define BLINK_LOOP_CYCLES       4UL     /* volatile inc + cmp + branch */

static void blink_delay_ms(uint32_t ms)
{
    uint32_t iterations = (HSI_FALLBACK_HZ / 1000UL / BLINK_LOOP_CYCLES) * ms;

    for (volatile uint32_t i = 0; i < iterations; i++)
    {
    }
}

void clock_fault_blink_forever(ClockStatus s)
{
    /* Configure PA5 here rather than assuming gpio_init has run - this is
     * reachable before any driver is up. */
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    (void)RCC->AHB1ENR;

    GPIOA->MODER &= ~(3UL << (2 * 5));
    GPIOA->MODER |=  (1UL << (2 * 5));

    uint32_t pulses = (uint32_t)s;

    if (pulses == 0UL)
    {
        pulses = 1UL;   /* never render a silent pattern */
    }

    for (;;)
    {
        for (uint32_t i = 0; i < pulses; i++)
        {
            GPIOA->BSRR = (1UL << 5);
            blink_delay_ms(150UL);

            GPIOA->BSRR = (1UL << (5 + 16));
            blink_delay_ms(150UL);
        }

        blink_delay_ms(1200UL);
    }
}

int clock_status_is_fatal(ClockStatus s)
{
    /*
     * OK and DEGRADED both leave the core at 180 MHz, so every derived
     * BRR, CCR and PSC stays valid and the scheduler may start. Every
     * other status means the board fell back to the raw 16 MHz HSI, where
     * the UART baud divisors are off by 11.25x and nothing downstream is
     * trustworthy - including the console that would have reported it.
     */
    return (s != CLOCK_OK_HSE) && (s != CLOCK_DEGRADED_HSI_PLL);
}

const char *system_clock_status_str(ClockStatus s)
{
    switch (s)
    {
        case CLOCK_OK_HSE:
            return "OK (HSE bypass, 180 MHz)";
        case CLOCK_DEGRADED_HSI_PLL:
            return "DEGRADED (no HSE; HSI PLL, 180 MHz, +/-1%)";
        case CLOCK_FAULT_PLL_TIMEOUT:
            return "FAULT (PLL never locked; raw HSI 16 MHz)";
        case CLOCK_FAULT_OVERDRIVE_TIMEOUT:
            return "FAULT (over-drive timeout; raw HSI 16 MHz)";
        case CLOCK_FAULT_SWITCH_TIMEOUT:
            return "FAULT (SYSCLK switch timeout; raw HSI 16 MHz)";
        default:
            return "FAULT (unknown)";
    }
}
