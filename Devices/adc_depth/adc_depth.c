#include "adc_depth.h"

#include "stm32f446xx.h"
#include "timer_timebase.h"

#define ADC_CH_DEPTH      4U        /* PA4 */
#define ADC_CH_VREFINT    17U

#define SAMPLES           16U       /* averaged per reading */

/*
 * VREFINT calibration: the raw ADC reading of the internal reference
 * taken at the factory with VDDA = 3.3 V, stored in system memory
 * (STM32F446 datasheet, "internal reference voltage calibration values").
 * Measuring VREFINT now and comparing gives the real VDDA, so a supply
 * that is a little off does not scale every depth reading with it.
 * UNTESTED: confirm the console shows VDDA close to 3.3 V on the board.
 */
#define VREFINT_CAL_ADDR  ((const uint16_t *)0x1FFF7A2AUL)
#define VDDA_CAL_V        3.3f

/* Exponential filter weight of each new reading. At the ~30 Hz depth_task
 * rate this averages over roughly the last 5 readings (~0.15 s). */
#define FILTER_ALPHA      0.2f

#define ZERO_MIN_V        0.45f
#define ZERO_MAX_V        0.55f
#define DISCONNECTED_V    0.3f

static float zero_v      = ADC_DEPTH_ZERO_V;
static float vdda_v      = VDDA_CAL_V;
static float depth_filt  = 0.0f;
static bool  have_filter = false;

float adc_depth_zero_v(void)
{
    return zero_v;
}

/* One conversion on one channel. Returns 0xFFFF if it never finishes. */
static uint16_t convert(uint32_t channel)
{
    ADC1->SQR3 = channel;
    ADC1->SR   = 0U;
    ADC1->CR2 |= ADC_CR2_SWSTART;

    /* 480-cycle sample + 12 conversion cycles at 22.5 MHz is ~22 us. */
    uint32_t start = micros();

    while (!(ADC1->SR & ADC_SR_EOC))
    {
        if ((micros() - start) > 200U)
        {
            return 0xFFFFU;
        }
    }

    return (uint16_t)ADC1->DR;
}

/* Average of SAMPLES conversions, as a raw count. */
static float average(uint32_t channel)
{
    uint32_t sum = 0U;

    for (uint32_t i = 0U; i < SAMPLES; i++)
    {
        sum += convert(channel);
    }

    return (float)sum / (float)SAMPLES;
}

/* Re-measure VDDA from VREFINT; keep the nominal 3.3 V if it is not
 * plausible. */
static void update_vdda(void)
{
    float raw = average(ADC_CH_VREFINT);

    if (raw > 0.0f)
    {
        float v = VDDA_CAL_V * (float)(*VREFINT_CAL_ADDR) / raw;

        if ((v > 3.0f) && (v < 3.6f))
        {
            vdda_v = v;
        }
    }
}

static float sensor_volts(void)
{
    float pin_v = average(ADC_CH_DEPTH) * vdda_v / 4095.0f;

    return pin_v * ADC_DEPTH_DIVIDER;
}

void adc_depth_init(void)
{
    /* PA4 analog (MODER = 11), no pull. */
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    GPIOA->MODER |=  (3U << (2U * 4U));
    GPIOA->PUPDR &= ~(3U << (2U * 4U));

    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
    (void)RCC->APB2ENR;

    /* ADC clock = PCLK2 (90 MHz) / 4 = 22.5 MHz, under the 36 MHz limit.
     * Enable the VREFINT channel. */
    ADC->CCR = (ADC->CCR & ~ADC_CCR_ADCPRE) | ADC_CCR_ADCPRE_0 | ADC_CCR_TSVREFE;

    /* 12-bit, right-aligned, single conversion, one channel. */
    ADC1->CR1  = 0U;
    ADC1->CR2  = 0U;
    ADC1->SQR1 = 0U;

    /* Longest sample time (480 cycles) on both channels. The sensor sits
     * behind a divider (a high source impedance), and VREFINT needs at
     * least 10 us of sampling. */
    ADC1->SMPR2 |= (7U << ADC_SMPR2_SMP4_Pos);
    ADC1->SMPR1 |= (7U << ADC_SMPR1_SMP17_Pos);

    ADC1->CR2 |= ADC_CR2_ADON;

    /* Let the ADC and the internal reference settle (a few us each). */
    uint32_t start = micros();
    while ((micros() - start) < 100U)
    {
    }

    update_vdda();

    float v = sensor_volts();

    zero_v = ((v >= ZERO_MIN_V) && (v <= ZERO_MAX_V)) ? v : ADC_DEPTH_ZERO_V;
    have_filter = false;
}

bool adc_depth_read(AdcDepthData *out)
{
    update_vdda();

    float v = sensor_volts();

    if (v < DISCONNECTED_V)
    {
        have_filter = false;
        return false;
    }

    float pa    = adc_depth_pressure_pa(v, zero_v);
    float depth = adc_depth_from_pa(pa);

    if (!have_filter)
    {
        depth_filt  = depth;
        have_filter = true;
    }
    else
    {
        depth_filt += FILTER_ALPHA * (depth - depth_filt);
    }

    out->sensor_v    = v;
    out->pressure_pa = pa;
    out->depth_m     = depth_filt;

    return true;
}
