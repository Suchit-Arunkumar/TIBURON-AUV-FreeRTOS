#include "system_init.h"

// 180 MHz clock setup. Both registers are written whole ('=', not '|='):
// PLLCFGR doesn't reset to zero, so OR-ing onto it gives a wrong clock.

// 8 MHz / 8 x 360 / 2 = 180 MHz. PLLR unused but must be 2..7.
#define PLLCFGR_HSE_180MHZ                       \
    ( (8UL   << RCC_PLLCFGR_PLLM_Pos)   |        \
      (360UL << RCC_PLLCFGR_PLLN_Pos)   |        \
      (0UL   << RCC_PLLCFGR_PLLP_Pos)   |        \
      RCC_PLLCFGR_PLLSRC_HSE            |        \
      (7UL   << RCC_PLLCFGR_PLLQ_Pos)   |        \
      (2UL   << RCC_PLLCFGR_PLLR_Pos) )

// Fallback without HSE: 16 MHz HSI / 16, same multipliers, still
// 180 MHz, so every baud rate and timer setting stays valid (just less
// accurate).
#define PLLCFGR_HSI_180MHZ                       \
    ( (16UL  << RCC_PLLCFGR_PLLM_Pos)   |        \
      (360UL << RCC_PLLCFGR_PLLN_Pos)   |        \
      (0UL   << RCC_PLLCFGR_PLLP_Pos)   |        \
      RCC_PLLCFGR_PLLSRC_HSI            |        \
      (7UL   << RCC_PLLCFGR_PLLQ_Pos)   |        \
      (2UL   << RCC_PLLCFGR_PLLR_Pos) )

// AHB 180 MHz, APB1 45 MHz, APB2 90 MHz
#define CFGR_PRESCALERS                          \
    ( RCC_CFGR_HPRE_DIV1  |                      \
      RCC_CFGR_PPRE1_DIV4 |                      \
      RCC_CFGR_PPRE2_DIV2 )

// Every wait is bounded (~130 ms at 16 MHz), so a dead clock source gives
// an error code instead of a hang. The real waits take under 1 ms.
#define CLOCK_WAIT_ITERATIONS   0x00080000UL

volatile ClockStatus g_clock_status = CLOCK_FAULT_PLL_TIMEOUT;

// 1 if the bits reached the wanted state in time, 0 on timeout.
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

// Above 168 MHz the regulator needs over-drive, and flash needs 5 wait
// states. Both paths (HSE and HSI) use this.
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

    // 5 wait states for 180 MHz at 3.3 V, prefetch, caches
    FLASH->ACR = FLASH_ACR_LATENCY_5WS |
                 FLASH_ACR_PRFTEN      |
                 FLASH_ACR_ICEN        |
                 FLASH_ACR_DCEN;

    // check it actually took
    if ((FLASH->ACR & FLASH_ACR_LATENCY) != FLASH_ACR_LATENCY_5WS)
    {
        return 0UL;
    }

    return 1UL;
}

// Last resort: back to the raw 16 MHz HSI, just enough to blink an error.
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

    // PWR registers can't be written until its clock is on
    RCC->APB1ENR |= RCC_APB1ENR_PWREN;
    (void)RCC->APB1ENR;   /* force the write to land before PWR is touched */

    // regulator scale 1, needed above 168 MHz and for over-drive
    PWR->CR = (PWR->CR & ~PWR_CR_VOS) | PWR_CR_VOS;   /* VOS = 0b11 */

    // HSE bypass: the Nucleo has no crystal, the ST-LINK supplies 8 MHz.
    // HSEBYP can only change with HSE off, so turn it off first.
    RCC->CR &= ~(RCC_CR_HSEON | RCC_CR_HSEBYP);
    (void)wait_for_bits(&RCC->CR, RCC_CR_HSERDY, 0UL);

    RCC->CR |= RCC_CR_HSEBYP;
    RCC->CR |= RCC_CR_HSEON;

    uint32_t hse_ok = wait_for_bits(&RCC->CR, RCC_CR_HSERDY, 1UL);

    // PLLCFGR can't be written while the PLL runs
    RCC->CR &= ~RCC_CR_PLLON;
    (void)wait_for_bits(&RCC->CR, RCC_CR_PLLRDY, 0UL);

    if (hse_ok)
    {
        RCC->PLLCFGR = PLLCFGR_HSE_180MHZ;
    }
    else
    {
        /* no HSE: run the PLL from the HSI instead */
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

    /* over-drive and flash wait states before the clock goes up */
    if (!enable_overdrive_and_flash())
    {
        fall_back_to_raw_hsi();
        g_clock_status = CLOCK_FAULT_OVERDRIVE_TIMEOUT;
        return g_clock_status;
    }

    // bus dividers first, so APB1/APB2 are never briefly overclocked
    RCC->CFGR = CFGR_PRESCALERS;

    // switch SYSCLK to the PLL
    RCC->CFGR = CFGR_PRESCALERS | RCC_CFGR_SW_PLL;

    for (uint32_t i = 0; i < CLOCK_WAIT_ITERATIONS; i++)
    {
        if ((RCC->CFGR & RCC_CFGR_SWS) == RCC_CFGR_SWS_PLL)
        {
            // both PLL settings give exactly SYSCLK_HZ
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
 * With the clock broken the console baud rate is wrong, so the LED is the
 * only way to report it: 2 blinks = PLL never locked, 3 = over-drive
 * failed, 4 = clock switch failed. Timed by counting cycles at 16 MHz.
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
    // OK and DEGRADED both run at 180 MHz; anything else is on 16 MHz
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
