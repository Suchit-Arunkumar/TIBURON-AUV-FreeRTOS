#include "timer_pwm.h"
#include "system_init.h"

/*
 * Eight ESC outputs across two timers.
 *
 * Previously pwm_set_us() ignored its channel argument and wrote CCR1
 * unconditionally, so all eight thrusters collapsed onto one output and
 * the last write of each control tick won. Audit finding B4.
 *
 *   ch 0..3  TIM3 CH1..CH4   PB4  PB5  PB0  PB1   AF2   (APB1)
 *   ch 4..7  TIM8 CH1..CH4   PC6  PC7  PC8  PC9   AF3   (APB2)
 *
 * The two prescalers are deliberately different. TIM3 hangs off APB1
 * whose timer clock is 90 MHz; TIM8 hangs off APB2 whose timer clock is
 * 180 MHz. Both must end up counting at 1 MHz, so:
 *
 *   TIM3  PSC = (90e6  / 1e6) - 1 =  89
 *   TIM8  PSC = (180e6 / 1e6) - 1 = 179
 *
 * Making them equal would run thrusters 5-8 at twice the intended frame
 * rate and half the intended pulse width.
 *
 * ARR = 19999 on both: 20000 counts at 1 MHz = 20 ms = 50 Hz, with 1 us
 * of resolution across the 1100-1900 us BlueRobotics Basic ESC range.
 */

/* Derived from the clock tree rather than hardcoded, so a clock change
 * is a compile-time consequence instead of a silent timing bug. */
#define PWM_TICK_HZ        1000000U
#define TIM3_PSC_VALUE     ((APB1_TIMCLK_HZ / PWM_TICK_HZ) - 1U)   /*  89 */
#define TIM8_PSC_VALUE     ((APB2_TIMCLK_HZ / PWM_TICK_HZ) - 1U)   /* 179 */
#define PWM_ARR_VALUE      19999U                                  /* 20 ms */

#define PWM_US_MIN         1100U
#define PWM_US_MAX         1900U
#define PWM_US_NEUTRAL     1500U

#define PWM_CHANNELS       8U

/* PWM mode 1, preload enabled: OCxM = 110, OCxPE = 1. */
#define OC_PWM1_LOW        ((6U << 4) | (1U << 3))    /* CH1 / CH3 half */
#define OC_PWM1_HIGH       ((6U << 12) | (1U << 11))  /* CH2 / CH4 half */

static void tim3_gpio_init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;

    /*
     * PB4 is NJTRST. It comes out of reset already in an alternate
     * function state for the JTAG tap, so its MODER and AFR bits must be
     * cleared explicitly rather than OR-ed onto — otherwise thruster 1
     * never moves and nothing in the code looks wrong.
     *
     * SWD (PA13/PA14) is untouched, so debugging still works.
     */
    GPIOB->MODER &= ~((3U << (2 * 4)) | (3U << (2 * 5)) |
                      (3U << (2 * 0)) | (3U << (2 * 1)));
    GPIOB->MODER |=  ((2U << (2 * 4)) | (2U << (2 * 5)) |
                      (2U << (2 * 0)) | (2U << (2 * 1)));

    /* AF2 = TIM3 on all four. PB0/PB1/PB4/PB5 are all in AFR[0]. */
    GPIOB->AFR[0] &= ~((0xFU << (4 * 4)) | (0xFU << (5 * 4)) |
                       (0xFU << (0 * 4)) | (0xFU << (1 * 4)));
    GPIOB->AFR[0] |=  ((2U << (4 * 4)) | (2U << (5 * 4)) |
                       (2U << (0 * 4)) | (2U << (1 * 4)));

    GPIOB->OSPEEDR |= (1U << (2 * 4)) | (1U << (2 * 5)) |
                      (1U << (2 * 0)) | (1U << (2 * 1));
}

static void tim8_gpio_init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;

    /* PC6..PC9, AF3 = TIM8. PC6/PC7 are in AFR[0], PC8/PC9 in AFR[1]. */
    GPIOC->MODER &= ~((3U << (2 * 6)) | (3U << (2 * 7)) |
                      (3U << (2 * 8)) | (3U << (2 * 9)));
    GPIOC->MODER |=  ((2U << (2 * 6)) | (2U << (2 * 7)) |
                      (2U << (2 * 8)) | (2U << (2 * 9)));

    GPIOC->AFR[0] &= ~((0xFU << (6 * 4)) | (0xFU << (7 * 4)));
    GPIOC->AFR[0] |=  ((3U << (6 * 4)) | (3U << (7 * 4)));

    GPIOC->AFR[1] &= ~((0xFU << ((8 - 8) * 4)) | (0xFU << ((9 - 8) * 4)));
    GPIOC->AFR[1] |=  ((3U << ((8 - 8) * 4)) | (3U << ((9 - 8) * 4)));

    GPIOC->OSPEEDR |= (1U << (2 * 6)) | (1U << (2 * 7)) |
                      (1U << (2 * 8)) | (1U << (2 * 9));
}

void pwm_init(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
    RCC->APB2ENR |= RCC_APB2ENR_TIM8EN;

    tim3_gpio_init();
    tim8_gpio_init();

    /* --- Time base ------------------------------------------------- */
    TIM3->PSC = TIM3_PSC_VALUE;
    TIM3->ARR = PWM_ARR_VALUE;

    TIM8->PSC = TIM8_PSC_VALUE;
    TIM8->ARR = PWM_ARR_VALUE;

    /* --- Output compare: PWM mode 1 with preload on all 8 ----------- */
    TIM3->CCMR1 = OC_PWM1_LOW | OC_PWM1_HIGH;   /* CH1, CH2 */
    TIM3->CCMR2 = OC_PWM1_LOW | OC_PWM1_HIGH;   /* CH3, CH4 */

    TIM8->CCMR1 = OC_PWM1_LOW | OC_PWM1_HIGH;
    TIM8->CCMR2 = OC_PWM1_LOW | OC_PWM1_HIGH;

    /*
     * --- Neutral BEFORE any output stage is enabled -----------------
     *
     * CCRx resets to 0. Enabling CCxE with a CCR still at 0 presents a
     * 0 us pulse to the ESC, which is not a valid frame and which some
     * ESCs latch as a fault. Write 1500 first, always.
     */
    TIM3->CCR1 = PWM_US_NEUTRAL;
    TIM3->CCR2 = PWM_US_NEUTRAL;
    TIM3->CCR3 = PWM_US_NEUTRAL;
    TIM3->CCR4 = PWM_US_NEUTRAL;

    TIM8->CCR1 = PWM_US_NEUTRAL;
    TIM8->CCR2 = PWM_US_NEUTRAL;
    TIM8->CCR3 = PWM_US_NEUTRAL;
    TIM8->CCR4 = PWM_US_NEUTRAL;

    /* --- Enable the capture/compare outputs ------------------------- */
    TIM3->CCER = TIM_CCER_CC1E | TIM_CCER_CC2E |
                 TIM_CCER_CC3E | TIM_CCER_CC4E;

    TIM8->CCER = TIM_CCER_CC1E | TIM_CCER_CC2E |
                 TIM_CCER_CC3E | TIM_CCER_CC4E;

    /*
     * TIM8 is an advanced-control timer. Its outputs stay electrically
     * disconnected until the main output enable is set, regardless of
     * CCER. TIM3 is a general-purpose timer and has no equivalent bit.
     * Forgetting this leaves thrusters 5-8 silently dead.
     */
    TIM8->BDTR |= TIM_BDTR_MOE;

    /* --- Auto-reload preload, then start counting ------------------- */
    TIM3->CR1 |= TIM_CR1_ARPE;
    TIM8->CR1 |= TIM_CR1_ARPE;

    /* Force the preloaded CCR and ARR values into their shadow
     * registers so the very first frame is already 1500 us. */
    TIM3->EGR = TIM_EGR_UG;
    TIM8->EGR = TIM_EGR_UG;

    TIM3->CR1 |= TIM_CR1_CEN;
    TIM8->CR1 |= TIM_CR1_CEN;
}

void pwm_set_us(uint8_t channel, uint16_t us)
{
    /* Out-of-range channel is a caller bug, not something to paper over
     * by writing a real output. Drop it. */
    if (channel >= PWM_CHANNELS)
    {
        return;
    }

    /* Clamp into the ESC's usable band. A T200 will not see a command
     * outside 1100-1900 us from this firmware under any code path. */
    if (us < PWM_US_MIN)
    {
        us = PWM_US_MIN;
    }
    else if (us > PWM_US_MAX)
    {
        us = PWM_US_MAX;
    }

    switch (channel)
    {
        case 0: TIM3->CCR1 = us; break;
        case 1: TIM3->CCR2 = us; break;
        case 2: TIM3->CCR3 = us; break;
        case 3: TIM3->CCR4 = us; break;
        case 4: TIM8->CCR1 = us; break;
        case 5: TIM8->CCR2 = us; break;
        case 6: TIM8->CCR3 = us; break;
        case 7: TIM8->CCR4 = us; break;
        default: break;
    }
}

void pwm_all_neutral(void)
{
    for (uint8_t i = 0U; i < PWM_CHANNELS; i++)
    {
        pwm_set_us(i, (uint16_t)PWM_US_NEUTRAL);
    }
}
