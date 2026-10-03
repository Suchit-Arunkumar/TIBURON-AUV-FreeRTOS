#include "console.h"

#include <stdarg.h>
#include <stdio.h>
#include "task.h"
#include "uart.h"

QueueHandle_t consoleQueue = NULL;

static volatile uint32_t console_drop_count = 0;

/* The stdio owner (dummy_task), registered from its first iteration. */
static TaskHandle_t console_owner = NULL;

void console_set_owner(void)
{
    console_owner = xTaskGetCurrentTaskHandle();
}

/* Written by USART2_IRQHandler, read and cleared by the stdio owner. */
static volatile char console_pending_cmd = 0;

void console_rx_isr_char(char c)
{
    console_pending_cmd = c;
}

char console_take_command(void)
{
    char c = console_pending_cmd;

    if (c != 0)
    {
        console_pending_cmd = 0;
    }

    return c;
}

void console_printf(const char *fmt, ...)
{
    /*
     * The formatted line lives on the CALLER's stack, so every task pays
     * CONSOLE_LINE_LEN bytes of stack for the duration of this call. That
     * is accounted for in the Phase 9 stack audit.
     */
    ConsoleLine line;
    va_list args;

    va_start(args, fmt);
    (void)vsnprintf(line.text, sizeof(line.text), fmt, args);
    va_end(args);

    /*
     * The owner writes its own lines directly. It is the only task that
     * drains consoleQueue, so a line it posted there could not be printed
     * until it returned to its loop, and a report longer than the queue
     * (CONSOLE_QUEUE_DEPTH = 8) lost its tail: the 13-line health and stack
     * reports were printing 8 lines and counting the rest as drops.
     * Writing directly is still single-owner output; the only cost is
     * that lines other tasks queued meanwhile print after the report.
     */
    if ((console_owner != NULL) && (xTaskGetCurrentTaskHandle() == console_owner))
    {
        uart2_write_str(line.text);
        uart2_write_str("\r\n");
        return;
    }

    if (consoleQueue == NULL)
    {
        /* Called before the queue exists - drop rather than fault. */
        console_drop_count++;
        return;
    }

    /*
     * Zero block time, always. A full console queue means the owner is
     * behind; the correct response is to lose the message, not to delay
     * whichever task produced it.
     */
    if (xQueueSend(consoleQueue, &line, 0) != pdPASS)
    {
        console_drop_count++;
    }
}

uint32_t console_dropped(void)
{
    return console_drop_count;
}

const char *console_fmt_milli(char *buf, uint32_t buflen, float v)
{
    if ((buf == NULL) || (buflen == 0U))
    {
        return "";
    }

    /* Round half away from zero, then split. Working in integer
     * milli-units from here on keeps the rest of this integer-only. */
    float scaled = v * 1000.0f;
    int32_t milli = (int32_t)(scaled + ((v >= 0.0f) ? 0.5f : -0.5f));

    int32_t whole = milli / 1000;
    int32_t frac  = milli % 1000;

    if (frac < 0)
    {
        frac = -frac;
    }

    /*
     * When |v| < 1 and v is negative, whole is 0 and carries no sign, so
     * "%ld.%03ld" would render -0.5 as "0.500". Prefix the sign
     * explicitly in exactly that case.
     */
    const char *sign = ((milli < 0) && (whole == 0)) ? "-" : "";

    (void)snprintf(buf, (size_t)buflen, "%s%ld.%03ld",
                   sign, (long)whole, (long)frac);

    return buf;
}
