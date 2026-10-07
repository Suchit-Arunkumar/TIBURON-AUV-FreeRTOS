#include "fault_latch.h"

#include "stm32f446xx.h"
#include "cmsis_gcc.h"
#include "timer_pwm.h"

#include <stdio.h>

// In .noinit so a reset doesn't clear it. No initialiser on purpose:
// that would put it in .data, which is reloaded on every reset.
FaultLatch g_fault_latch __attribute__((section(".noinit")));

static void copy_detail(char *dst, const char *src)
{
    uint32_t i = 0;

    if (src != 0)
    {
        while ((i < (FAULT_DETAIL_LEN - 1U)) && (src[i] != '\0'))
        {
            dst[i] = src[i];
            i++;
        }
    }

    dst[i] = '\0';
}

// __FILE__ is the full build path; print just the file name.
static const char *basename_of(const char *path)
{
    const char *out = path;

    if (path == 0)
    {
        return "?";
    }

    for (const char *p = path; *p != '\0'; p++)
    {
        if ((*p == '/') || (*p == '\\'))
        {
            out = p + 1;
        }
    }

    return out;
}

static const char *kind_str(uint32_t kind)
{
    switch (kind)
    {
        case FAULT_ASSERT:         return "configASSERT";
        case FAULT_STACK_OVERFLOW: return "STACK OVERFLOW";
        case FAULT_MALLOC_FAILED:  return "MALLOC FAILED";
        case FAULT_INIT_FAILED:    return "INIT FAILED";
        case FAULT_HARDFAULT:      return "HARDFAULT";
        default:                   return "UNKNOWN";
    }
}

void fault_latch_fail(FaultKind kind,
                      const char *file,
                      uint32_t line,
                      uint32_t pc,
                      const char *detail)
{
    __disable_irq();

    // a halted board must not leave the thrusters running
    pwm_fault_neutral();

    // First fault wins. main clears the previous run's latch at boot, so a
    // valid one here means a second fault while handling the first.
    if (!fault_latch_valid())
    {
        g_fault_latch.kind = (uint32_t)kind;
        g_fault_latch.file = file;
        g_fault_latch.line = line;
        g_fault_latch.pc   = pc;
        copy_detail(g_fault_latch.detail, detail);

        // magic last, so a half-written record never looks valid
        g_fault_latch.magic = FAULT_LATCH_MAGIC;
    }

    __DSB();

    // Stop in the debugger if one is attached. Without one, BKPT would
    // escalate to a HardFault.
    if (CoreDebug->DHCSR & CoreDebug_DHCSR_C_DEBUGEN_Msk)
    {
        __BKPT(0);
    }

    for (;;)
    {
    }
}

void fault_assert_failed(const char *file, uint32_t line)
{
    // an address inside the function whose configASSERT failed
    uint32_t pc = (uint32_t)__builtin_return_address(0);

    fault_latch_fail(FAULT_ASSERT, file, line, pc, "");
}

int fault_latch_valid(void)
{
    return (g_fault_latch.magic == FAULT_LATCH_MAGIC) ? 1 : 0;
}

void fault_latch_clear(void)
{
    g_fault_latch.magic = 0UL;
    g_fault_latch.kind  = (uint32_t)FAULT_NONE;
    g_fault_latch.file  = 0;
    g_fault_latch.line  = 0UL;
    g_fault_latch.pc    = 0UL;
    g_fault_latch.detail[0] = '\0';
}

void fault_latch_report(void)
{
    if (!fault_latch_valid())
    {
        return;
    }

    printf("\r\n*** LATCHED FAULT FROM PREVIOUS RUN ***\r\n");
    printf("  kind : %s\r\n", kind_str(g_fault_latch.kind));
    printf("  at   : %s:%lu\r\n",
           basename_of(g_fault_latch.file),
           (unsigned long)g_fault_latch.line);
    printf("  pc   : 0x%08lX\r\n", (unsigned long)g_fault_latch.pc);

    if (g_fault_latch.detail[0] != '\0')
    {
        printf("  info : %s\r\n", g_fault_latch.detail);
    }

    printf("***************************************\r\n\r\n");

    fault_latch_clear();
}
