#include "timer_pwm.h"
#include "system_init.h"

/*
 * Eight ESC outputs:
 *   ch 0..3  TIM3 CH1..CH4   PB4  PB5  PB0  PB1   AF2
 *   ch 4..7  TIM8 CH1..CH4   PC6  PC7  PC8  PC9   AF3
 * Both count at 1 MHz (1 us per count) with a 20 ms period. TIM3's clock
 * is 90 MHz and TIM8's is 180 MHz, so the prescalers differ.
 */

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

    // PB4 boots as a JTAG pin (NJTRST): clear its bits, don't just OR,
    // or thruster 1 never moves.
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

    TIM3->PSC = TIM3_PSC_VALUE;
    TIM3->ARR = PWM_ARR_VALUE;

    TIM8->PSC = TIM8_PSC_VALUE;
    TIM8->ARR = PWM_ARR_VALUE;

    TIM3->CCMR1 = OC_PWM1_LOW | OC_PWM1_HIGH;   /* CH1, CH2 */
    TIM3->CCMR2 = OC_PWM1_LOW | OC_PWM1_HIGH;   /* CH3, CH4 */

    TIM8->CCMR1 = OC_PWM1_LOW | OC_PWM1_HIGH;
    TIM8->CCMR2 = OC_PWM1_LOW | OC_PWM1_HIGH;

    // 1500 us before the outputs are enabled: the compare registers reset
    // to 0, and a 0 us pulse isn't a valid ESC frame.
    TIM3->CCR1 = PWM_US_NEUTRAL;
    TIM3->CCR2 = PWM_US_NEUTRAL;
    TIM3->CCR3 = PWM_US_NEUTRAL;
    TIM3->CCR4 = PWM_US_NEUTRAL;

    TIM8->CCR1 = PWM_US_NEUTRAL;
    TIM8->CCR2 = PWM_US_NEUTRAL;
    TIM8->CCR3 = PWM_US_NEUTRAL;
    TIM8->CCR4 = PWM_US_NEUTRAL;

    TIM3->CCER = TIM_CCER_CC1E | TIM_CCER_CC2E |
                 TIM_CCER_CC3E | TIM_CCER_CC4E;

    TIM8->CCER = TIM_CCER_CC1E | TIM_CCER_CC2E |
                 TIM_CCER_CC3E | TIM_CCER_CC4E;

    // TIM8 (advanced timer) outputs stay off until MOE is set.
    TIM8->BDTR |= TIM_BDTR_MOE;

    TIM3->CR1 |= TIM_CR1_ARPE;
    TIM8->CR1 |= TIM_CR1_ARPE;

    // load the preloaded values now, so the first frame is already 1500 us
    TIM3->EGR = TIM_EGR_UG;
    TIM8->EGR = TIM_EGR_UG;

    TIM3->CR1 |= TIM_CR1_CEN;
    TIM8->CR1 |= TIM_CR1_CEN;
}

void pwm_set_us(uint8_t channel, uint16_t us)
{
    // bad channel: drop it rather than drive some other output
    if (channel >= PWM_CHANNELS)
    {
        return;
    }

    // nothing outside 1100-1900 us ever reaches an ESC
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

void pwm_fault_neutral(void)
{
    // No update event to apply it at once: restarting the counter
    // mid-pulse would stretch that pulse into a one-frame command. It
    // takes effect at the next frame.
    TIM3->CCR1 = PWM_US_NEUTRAL;
    TIM3->CCR2 = PWM_US_NEUTRAL;
    TIM3->CCR3 = PWM_US_NEUTRAL;
    TIM3->CCR4 = PWM_US_NEUTRAL;

    TIM8->CCR1 = PWM_US_NEUTRAL;
    TIM8->CCR2 = PWM_US_NEUTRAL;
    TIM8->CCR3 = PWM_US_NEUTRAL;
    TIM8->CCR4 = PWM_US_NEUTRAL;
}
