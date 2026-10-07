#ifndef CONSOLE_H
#define CONSOLE_H

#include <stdint.h>

#include "FreeRTOS.h"
#include "queue.h"

/*
 * Console with a single owner. printf isn't reentrant in this build
 * (configUSE_NEWLIB_REENTRANT 0), so tasks call console_printf(), which
 * formats on the caller's stack and queues the line. Only dummy_task
 * writes to the UART. Before the scheduler starts, main uses printf
 * directly.
 *
 * No %f: nano.specs has no float formatting, and %f silently prints
 * nothing (and floats in varargs are promoted to double anyway). Use
 * console_fmt_milli() and %s.
 */

#define CONSOLE_LINE_LEN    80
#define CONSOLE_QUEUE_DEPTH 8

/* Widest output is "-32768.999" plus NUL. */
#define CONSOLE_MILLI_LEN   16

typedef struct
{
    char text[CONSOLE_LINE_LEN];
} ConsoleLine;

extern QueueHandle_t consoleQueue;

/* Format and queue one line. Never blocks: a full queue drops the line
 * and counts it. Not for use in an ISR. */
void console_printf(const char *fmt, ...) __attribute__((format(printf, 1, 2)));

/* Lines dropped because consoleQueue was full, since boot. */
uint32_t console_dropped(void);

/*
 * A float as text with three decimals, e.g.
 *     char d[CONSOLE_MILLI_LEN];
 *     console_printf("depth %s m", console_fmt_milli(d, sizeof(d), z));
 * The caller passes the buffer so two values in one printf don't share
 * one (argument order is unspecified, so both would print the same).
 */
const char *console_fmt_milli(char *buf, uint32_t buflen, float v);

/* Single-key commands: the USART2 interrupt stores the last key typed,
 * dummy_task picks it up on its next 50 ms wake (0 = nothing). */
void console_rx_isr_char(char c);
char console_take_command(void);

/* Call once from the console task; its own lines are then written
 * directly instead of through the queue. */
void console_set_owner(void);

#endif /* CONSOLE_H */
