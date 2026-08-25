#ifndef IWDG_H
#define IWDG_H

#include "stm32f446xx.h"
#include <stdint.h>

/* ======================================================================
 * ENABLE_IWDG - the one line to toggle.
 *
 * Commented out by default so bench work, breakpoints and single-stepping
 * do not reset the board. Uncomment for anything that goes in the water.
 *
 * When enabled, ONLY control_task refreshes the watchdog. That is
 * deliberate: if the control loop stops running, the board resets and the
 * thrusters stop, rather than the board staying alive with the last PWM
 * values latched into the ESCs. A watchdog kicked from a low-priority
 * task or a timer would defeat that entirely.
 * ====================================================================== */
/* #define ENABLE_IWDG */

/*
 * ~1 s timeout. LSI is nominally 32 kHz; PR=4 divides by 64, giving a
 * 500 Hz count, and RLR=500 is therefore 1.0 s.
 *
 * LSI is an RC oscillator specified across 17-47 kHz over the full
 * temperature range, so the real timeout spans roughly 0.53-1.47 s. The
 * control loop runs at 50 Hz (20 ms), so even the fast end leaves ~26
 * missed ticks of margin before a reset.
 */
#define IWDG_PR_DIV64     4U
#define IWDG_RELOAD_1S    500U

void iwdg_init(void);
void iwdg_kick(void);

#endif
