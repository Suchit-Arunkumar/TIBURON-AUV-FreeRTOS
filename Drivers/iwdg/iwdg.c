#include "iwdg.h"

/*
 * Both functions compile to nothing unless ENABLE_IWDG is defined, so
 * call sites do not need their own #ifdef and the control loop reads the
 * same either way.
 */

void iwdg_freeze_on_halt(void)
{
    /*
     * Unconditional - NOT behind ENABLE_IWDG.
     *
     * The IWDG runs from the LSI and cannot be stopped once started, and
     * it does not halt when the core halts. Without this, every
     * breakpoint and every single-step becomes a reset a second later,
     * which makes the debugger useless precisely when the watchdog is
     * enabled and you most want it.
     *
     * DBG_IWDG_STOP freezes the counter only while the core is halted by
     * the debugger. On a free-running board, and on a board with no
     * debugger attached, it has no effect at all - so there is no reason
     * to make it conditional and one good reason not to: a conditional
     * version would be absent from exactly the build being debugged.
     */
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;   /* DBGMCU sits behind this */
    (void)RCC->APB2ENR;

    DBGMCU->APB1FZ |= DBGMCU_APB1_FZ_DBG_IWDG_STOP;
}

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
