#include "iwdg.h"

/*
 * Both functions compile to nothing unless ENABLE_IWDG is defined, so
 * call sites do not need their own #ifdef and the control loop reads the
 * same either way.
 */

void iwdg_init(void)
{
#ifdef ENABLE_IWDG
    /* 1. Unlock PR and RLR. */
    IWDG->KR = 0x5555U;

    /* 2. Prescaler: LSI / 64 -> ~500 Hz. */
    while (IWDG->SR & IWDG_SR_PVU)
    {
    }
    IWDG->PR = IWDG_PR_DIV64;

    /* 3. Reload value: 500 counts at ~500 Hz -> ~1 s. */
    while (IWDG->SR & IWDG_SR_RVU)
    {
    }
    IWDG->RLR = IWDG_RELOAD_1S;

    /* 4. Load the counter, then start. Once started the IWDG cannot be
     *    stopped by software - only a reset clears it. */
    IWDG->KR = 0xAAAAU;
    IWDG->KR = 0xCCCCU;
#endif
}

void iwdg_kick(void)
{
#ifdef ENABLE_IWDG
    IWDG->KR = 0xAAAAU;
#endif
}
