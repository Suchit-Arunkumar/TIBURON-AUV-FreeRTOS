#ifndef IWDG_H
#define IWDG_H

#include "stm32f446xx.h"
#include <stdint.h>

/* ======================================================================
 * ENABLE_IWDG - set by the build configuration.
 *
 *   Release (no DEBUG symbol): always on. This is the build for the
 *                              vehicle, and it cannot be built without
 *                              a watchdog by accident.
 *   Debug:                     off, so breakpoints and single-stepping do
 *                              not reset the board. Add -DENABLE_IWDG to
 *                              the Debug defines to test the watchdog.
 *
 * When enabled, ONLY control_task refreshes the watchdog. That is
 * deliberate: if the control loop stops running, the board resets and the
 * thrusters stop, rather than the board staying alive with the last PWM
 * values latched into the ESCs. A watchdog kicked from a low-priority
 * task or a timer would defeat that entirely.
 * ====================================================================== */
#if !defined(DEBUG) && !defined(ENABLE_IWDG)
#define ENABLE_IWDG
#endif

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

/*
 * Freeze the IWDG counter while the core is halted by a debugger.
 * Unconditional and independent of ENABLE_IWDG - see the comment in
 * iwdg.c. Call once, early in main.
 */
void iwdg_freeze_on_halt(void);

void iwdg_init(void);
void iwdg_kick(void);

/*
 * 1 when this build has the watchdog compiled in.
 *
 * The default is OFF, which is right for bench work and wrong for
 * anything in the water. A build with no watchdog must never be
 * mistakable for one with it: main prints a banner line and dummy_task
 * uses a distinct LD2 pattern, so the state is visible both on the
 * console and across the room.
 */
static inline int iwdg_is_enabled(void)
{
#ifdef ENABLE_IWDG
    return 1;
#else
    return 0;
#endif
}

#endif
