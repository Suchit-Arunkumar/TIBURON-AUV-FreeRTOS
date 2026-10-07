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
    // on the caller's stack: every task that prints needs ~80 B for this
    ConsoleLine line;
    va_list args;

    va_start(args, fmt);
    (void)vsnprintf(line.text, sizeof(line.text), fmt, args);
    va_end(args);

    // The owner writes its own lines directly: queued, a report longer
    // than the queue lost its tail, since nobody drains the queue while
    // the owner is busy filling it.
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

    // never wait: losing a debug line is better than delaying the caller
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
