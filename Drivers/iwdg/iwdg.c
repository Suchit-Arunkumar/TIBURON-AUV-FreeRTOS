#include "iwdg.h"

// iwdg_init and iwdg_kick compile to nothing without ENABLE_IWDG, so
// callers need no #ifdefs.

void iwdg_freeze_on_halt(void)
{
    // Once started the IWDG can't be stopped and keeps counting while a
    // debugger has the core halted, so every breakpoint would end in a
    // reset. This freezes it only while halted; no effect otherwise.
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;   /* DBGMCU sits behind this */
    (void)RCC->APB2ENR;

    DBGMCU->APB1FZ |= DBGMCU_APB1_FZ_DBG_IWDG_STOP;
}

void iwdg_init(void)
{
#ifdef ENABLE_IWDG
    /* unlock PR and RLR */
    IWDG->KR = 0x5555U;

    /* LSI / 64 -> ~500 Hz */
    while (IWDG->SR & IWDG_SR_PVU)
    {
    }
    IWDG->PR = IWDG_PR_DIV64;

    /* 500 counts -> ~1 s */
    while (IWDG->SR & IWDG_SR_RVU)
    {
    }
    IWDG->RLR = IWDG_RELOAD_1S;

    /* reload, then start; only a reset can stop it now */
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
