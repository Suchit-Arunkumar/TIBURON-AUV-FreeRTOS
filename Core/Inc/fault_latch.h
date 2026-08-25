#ifndef FAULT_LATCH_H
#define FAULT_LATCH_H

#include <stdint.h>

/*
 * Reset-surviving fault latch.
 *
 * The problem this solves: configASSERT, the stack-overflow hook and the
 * malloc-failed hook all used to end in a bare spin loop. When one fired
 * you got a dead board and no way to find out which check tripped or
 * where. This records that before halting, in a RAM struct the startup
 * code does not clear, so the answer survives the warm reset you
 * inevitably reach for next.
 *
 * The struct lives in .noinit, placed after _ebss in the linker script
 * so startup_stm32f446retx.s never zeroes it. It is therefore garbage on
 * a cold power-up, which is why every read is gated on the magic word.
 * A random cold-boot word matching FAULT_LATCH_MAGIC is a 1-in-2^32
 * event; the cost of it happening is one spurious boot message.
 */

#define FAULT_LATCH_MAGIC     0x54464B31UL   /* "TFK1" */
#define FAULT_DETAIL_LEN      24

typedef enum
{
    FAULT_NONE            = 0,
    FAULT_ASSERT          = 1,   /* configASSERT(x) evaluated false      */
    FAULT_STACK_OVERFLOW  = 2,   /* vApplicationStackOverflowHook        */
    FAULT_MALLOC_FAILED   = 3,   /* vApplicationMallocFailedHook         */
    FAULT_INIT_FAILED     = 4    /* a queue or task failed to be created */
} FaultKind;

typedef struct
{
    uint32_t magic;
    uint32_t kind;                       /* FaultKind                    */
    const char *file;                    /* points into flash .rodata    */
    uint32_t line;
    uint32_t pc;                         /* return address of the caller */
    char     detail[FAULT_DETAIL_LEN];   /* task name, or a short tag    */
} FaultLatch;

extern FaultLatch g_fault_latch;

/* Record and halt. Never returns. */
void fault_latch_fail(FaultKind kind,
                      const char *file,
                      uint32_t line,
                      uint32_t pc,
                      const char *detail) __attribute__((noreturn));

/*
 * configASSERT lands here. Captures its own return address as the PC, so
 * the reported address is inside the function that asserted rather than
 * inside the latch code.
 */
void fault_assert_failed(const char *file, uint32_t line) __attribute__((noreturn));

/* 1 if a latched fault is present from a previous run. */
int  fault_latch_valid(void);

/*
 * Print any latched fault over stdio and clear it. Must be called early
 * in main, after the VCP is up and before anything that could re-fault.
 * Safe to call when nothing is latched — it prints nothing.
 */
void fault_latch_report(void);

void fault_latch_clear(void);

#endif /* FAULT_LATCH_H */
