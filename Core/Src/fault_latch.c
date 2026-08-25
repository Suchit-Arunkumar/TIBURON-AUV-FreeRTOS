#include "fault_latch.h"

#include "stm32f446xx.h"
#include "cmsis_gcc.h"

#include <stdio.h>

/*
 * .noinit — see the header. Deliberately not initialised: giving this an
 * initialiser would move it to .data and it would be overwritten from
 * flash on every reset, defeating the entire point.
 */
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

/* __FILE__ carries the full build path. Print only the last component —
 * the rest is noise on a 20-column terminal and identical for every
 * call site in the same tree. */
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
        default:                   return "UNKNOWN";
    }
}

void fault_latch_fail(FaultKind kind,
                      const char *file,
                      uint32_t line,
                      uint32_t pc,
                      const char *detail)
{
    /*
     * Interrupts off first. Everything below writes the latch, and a
     * preempting ISR that faults too would otherwise overwrite the
     * first — and the first is the one that matters.
     */
    __disable_irq();

    g_fault_latch.kind = (uint32_t)kind;
    g_fault_latch.file = file;
    g_fault_latch.line = line;
    g_fault_latch.pc   = pc;
    copy_detail(g_fault_latch.detail, detail);

    /* Magic written last, so a reset landing mid-update cannot leave a
     * half-filled record that reads as valid. */
    g_fault_latch.magic = FAULT_LATCH_MAGIC;

    __DSB();

    /*
     * Break to the debugger if one is attached. With no debugger this
     * executes as a no-op on Cortex-M4 rather than escalating, so the
     * spin below is what actually holds the board.
     */
    __BKPT(0);

    for (;;)
    {
    }
}

void fault_assert_failed(const char *file, uint32_t line)
{
    /* The caller's return address — i.e. an address inside whichever
     * function evaluated the failing configASSERT. */
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
