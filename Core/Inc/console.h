#ifndef CONSOLE_H
#define CONSOLE_H

#include <stdint.h>

#include "FreeRTOS.h"
#include "queue.h"

/*
 * Single-owner console.
 *
 * configUSE_NEWLIB_REENTRANT is 0, which means every task shares one
 * struct _reent and, with it, one stdout FILE (__sf). Two tasks calling
 * printf concurrently corrupt that shared buffer. The usual fix is to set
 * the flag to 1 and pay ~96 bytes of _reent per TCB across nine tasks.
 *
 * This is the cheaper fix: any task may call console_printf(), which
 * formats into its OWN stack buffer and posts the finished bytes to a
 * queue. Exactly one task drains that queue and performs the output, and
 * it does so with uart2_write_buf() rather than printf - so after the
 * scheduler starts, stdout is never touched at all.
 *
 * THE SINGLE STDIO OWNER IS dummy_task (Core/Src/main.c, priority 1).
 * It blocks on consoleQueue and uses the receive timeout as its heartbeat
 * tick, so it needs no second wake source.
 *
 * Before the scheduler starts there is only one context, so main calls
 * printf directly; that remains safe and is the only place stdout is
 * used.
 *
 * Caveat, stated rather than glossed: vsnprintf still reads shared
 * _impure_ptr state for locale. With nano.specs and integer-only formats
 * that access is read-only, so it is safe here - but it is a reason to
 * keep formats simple and to avoid %f.
 */

#define CONSOLE_LINE_LEN    80
#define CONSOLE_QUEUE_DEPTH 8

typedef struct
{
    char text[CONSOLE_LINE_LEN];
} ConsoleLine;

extern QueueHandle_t consoleQueue;

/*
 * Format and post one line. Safe from any task.
 *
 * Never blocks: if the queue is full the line is dropped and the drop
 * counter increments. Diagnostics must not be able to stall the caller,
 * least of all control_task.
 *
 * Not safe from an ISR - use the FromISR variants of the queue API
 * directly if that is ever needed.
 */
void console_printf(const char *fmt, ...) __attribute__((format(printf, 1, 2)));

/* Lines dropped because consoleQueue was full, since boot. */
uint32_t console_dropped(void);

#endif /* CONSOLE_H */
