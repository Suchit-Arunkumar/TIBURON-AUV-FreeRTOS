#include "console.h"

#include <stdarg.h>
#include <stdio.h>

QueueHandle_t consoleQueue = NULL;

static uint32_t console_drop_count = 0;

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
