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

/* ======================================================================
 * NO FLOATING-POINT CONVERSIONS. %f, %e and %g DO NOT WORK HERE.
 *
 * The build links --specs=nano.specs without -u _printf_float, so the
 * float formatting code is not present. A %f does not error, warn, or
 * print a wrong number - it prints NOTHING, and the rest of the line
 * silently goes with it. Verified absent from the ELF: no _printf_float
 * symbol is linked.
 *
 * DECISION: fixed-point milli-units, not -u _printf_float. Reasons:
 *
 *   1. ~6 KB of flash for cosmetics on a diagnostic path.
 *   2. Varargs promote float to double unconditionally - that promotion
 *      is mandated by the language, so -Wdouble-promotion cannot flag
 *      it. Every %f call site would quietly reintroduce soft-float
 *      double conversion into a codebase that is otherwise strictly
 *      single-precision on a single-precision FPU.
 *   3. Float formatting costs another ~100 bytes of stack per call, on
 *      the CALLER's stack, which is already the least-measured part of
 *      the stack budget.
 *   4. Milli-units suit the quantities: depth in mm, angles in
 *      millidegrees, PID output in milli-units. Fixed width, and
 *      trivially parsed from a captured log.
 *
 * Use console_fmt_milli() for any float, and print it with %s.
 * ====================================================================== */

#define CONSOLE_LINE_LEN    80
#define CONSOLE_QUEUE_DEPTH 8

/* Widest output is "-32768.999" plus NUL. */
#define CONSOLE_MILLI_LEN   16

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

/*
 * Render a float as a signed fixed-point string with three decimals,
 * into a caller-supplied buffer of at least CONSOLE_MILLI_LEN bytes.
 * Returns buf, so it drops straight into a %s argument:
 *
 *     char d[CONSOLE_MILLI_LEN];
 *     console_printf("depth %s m", console_fmt_milli(d, sizeof(d), z));
 *
 * Handles the -0.5 case correctly, where the integer part is 0 but the
 * value is negative and a naive split loses the sign.
 *
 * The buffer is CALLER-SUPPLIED on purpose. A static or single shared
 * buffer would make this:
 *
 *     console_printf("d=%s u=%s", fmt(depth), fmt(u));   // BROKEN
 *
 * print the same value twice - argument evaluation order is unspecified,
 * so both calls would write the one buffer before printf read either.
 * That failure reads as a sensor fault, not a formatting fault, which is
 * the worst kind to debug. Two values need two buffers.
 */
const char *console_fmt_milli(char *buf, uint32_t buflen, float v);

/*
 * On-demand command interface.
 *
 * console_rx_isr_char() is called from USART2_IRQHandler with whatever
 * the operator typed. It stores one byte - latest wins, no ring buffer,
 * because these are single-key commands typed by a human and a dropped
 * repeat is harmless.
 *
 * console_take_command() is called by the stdio owner, which polls it on
 * its existing 50 ms wake. Returns 0 when nothing is pending. Worst-case
 * latency from keypress to output is therefore ~50 ms, which is
 * imperceptible for this purpose and avoids giving the ISR any
 * scheduler interaction at all.
 */
void console_rx_isr_char(char c);
char console_take_command(void);

/* Call once from the stdio owner task. Its own console_printf() lines are
 * then written directly instead of through consoleQueue. */
void console_set_owner(void);

#endif /* CONSOLE_H */
