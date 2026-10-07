#ifndef FAULT_LATCH_H
#define FAULT_LATCH_H

#include <stdint.h>

/*
 * Fault latch: on an assert, stack overflow, failed allocation or CPU
 * fault, save what happened and where in RAM that survives a reset
 * (.noinit, not zeroed at start-up), then halt. The next boot prints it.
 * After a power cycle the RAM is random, so it only counts if the magic
 * word matches.
 */

#define FAULT_LATCH_MAGIC     0x54464B31UL   /* "TFK1" */
#define FAULT_DETAIL_LEN      24

typedef enum
{
    FAULT_NONE            = 0,
    FAULT_ASSERT          = 1,   /* configASSERT(x) evaluated false      */
    FAULT_STACK_OVERFLOW  = 2,   /* vApplicationStackOverflowHook        */
    FAULT_MALLOC_FAILED   = 3,   /* vApplicationMallocFailedHook         */
    FAULT_INIT_FAILED     = 4,   /* a queue or task failed to be created */
    FAULT_HARDFAULT       = 5    /* CPU fault; pc is the faulting address */
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

/*
 * Thrusters to neutral, record, halt. Never returns.
 *
 * The first fault of a run wins: if something is already latched (a
 * second fault while handling the first), it is not overwritten.
 */
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
