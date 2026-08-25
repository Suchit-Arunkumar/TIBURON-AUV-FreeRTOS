#include "timer_basic.h"



/*
 * 50 Hz control-loop tick.
 *
 * TIM7 is on APB1, whose timer clock is 90 MHz:
 *   90 MHz / (1799 + 1) = 50 kHz
 *   50 kHz / (999  + 1) = 50 Hz   -> 20 ms, exactly
 *
 * B6: this is called from inside control_task on its first iteration,
 * NOT from main. Starting the timer before vTaskStartScheduler() means
 * the ISR could fire with controlTaskHandle still NULL and could call
 * portYIELD_FROM_ISR() before the scheduler exists.
 */
void tim7_init(void)
{
    // 1. Enable TIM7 clock in RCC
    RCC->APB1ENR |= RCC_APB1ENR_TIM7EN;

    // 2. PSC = 1799 -> 50 kHz counter tick
    TIM7->PSC = 1799;

    // 3. ARR = 999 -> 50 Hz update event
    TIM7->ARR = 999;

    // 4. Load PSC/ARR into their shadow registers, then clear the update
    //    flag the UG event itself sets, so the first real tick is a full
    //    20 ms rather than immediate.
    TIM7->EGR = TIM_EGR_UG;
    TIM7->SR  = ~TIM_SR_UIF;

    // 5. Enable the update interrupt
    TIM7->DIER |= TIM_DIER_UIE;

    /*
     * 6. NVIC priority 5.
     *
     * This is exactly configMAX_SYSCALL_INTERRUPT_PRIORITY
     * (configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY = 5, raw 0x50), the
     * highest priority from which a FromISR call is legal. One step
     * higher (4) would be numerically below the ceiling and the kernel
     * assert would catch it.
     */
    NVIC_SetPriority(TIM7_IRQn, 5);

    // 7. Enable TIM7_IRQn in NVIC
    NVIC_EnableIRQ(TIM7_IRQn);

    // 8. Start the counter
    TIM7->CR1 |= TIM_CR1_CEN;
}
