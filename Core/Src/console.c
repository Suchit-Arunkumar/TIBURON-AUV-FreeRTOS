#include "console.h"

#include <stdarg.h>
#include <stdio.h>

QueueHandle_t consoleQueue = NULL;

static uint32_t console_drop_count = 0;

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
