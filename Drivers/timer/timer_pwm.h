#ifndef TIMER_PWM_H
#define TIMER_PWM_H

#include <stdint.h>
#include "stm32f446xx.h"

/*
 * Eight ESC channels: TIM3 CH1-4 (thrusters 1-4) and TIM8 CH1-4
 * (thrusters 5-8). 50 Hz frame, 1 us resolution, 1100-1900 us band with
 * 1500 us neutral - BlueRobotics Basic ESC / T200.
 *
 * pwm_init() leaves every channel at 1500 us with the counters running,
 * so the ESCs arm during the rest of boot.
 */
void pwm_init(void);

/* channel 0-7. Out-of-range channels are ignored; us is clamped into
 * [1100, 1900]. */
void pwm_set_us(uint8_t channel, uint16_t us);

/* All eight to 1500 us. Used by the failsafe path. */
void pwm_all_neutral(void);

/*
 * Fault path version of pwm_all_neutral(): plain register writes, no
 * calls, no RTOS. Safe from HardFault and the fault latch. The new value
 * reaches the pins at the start of the next 20 ms frame.
 */
void pwm_fault_neutral(void);

#endif
