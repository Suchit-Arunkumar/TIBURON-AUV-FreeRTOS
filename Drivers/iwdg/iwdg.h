#ifndef IWDG_H
#define IWDG_H

#include "stm32f446xx.h"
#include <stdint.h>

/*
 * Watchdog: always on in Release (the vehicle build), off in Debug so a
 * breakpoint doesn't reset the board. -DENABLE_IWDG turns it on in Debug.
 *
 * Only control_task refreshes it, so if the control loop stops, the board
 * resets and the ESCs lose their signal.
 */
#if !defined(DEBUG) && !defined(ENABLE_IWDG)
#define ENABLE_IWDG
#endif

/*
 * ~1 s: LSI 32 kHz / 64 = 500 Hz, 500 counts. The LSI can be anywhere
 * from 17 to 47 kHz, so really 0.53-1.47 s; still ~26 control ticks.
 */
#define IWDG_PR_DIV64     4U
#define IWDG_RELOAD_1S    500U

/* Stop the watchdog counting while a debugger has the core halted.
 * Harmless without one, so always called. */
void iwdg_freeze_on_halt(void);

void iwdg_init(void);
void iwdg_kick(void);

/* 1 if this build has the watchdog. Shown on the boot banner and by the
 * LED pattern, so the two builds can't be confused. */
static inline int iwdg_is_enabled(void)
{
#ifdef ENABLE_IWDG
    return 1;
#else
    return 0;
#endif
}

#endif
