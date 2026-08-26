#include <stdio.h>
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"

#include "system_init.h"
#include "gpio.h"
#include "uart.h"
#include "uart_packet.h"
#include "uart3.h"
#include "uart4.h"

#include "spi.h"
#include "oled.h"
#include "sd_card.h"

#include "timer_basic.h"
#include "timer_pwm.h"
#include "timer_timebase.h"

#include "control_loop.h"


#include "struct.h"
#include "packet.h"
#include "sd_logger.h"

#include "crc_hw.h"

#include "i2c.h"
#include "bar30.h"

#include "iwdg.h"

#include "fault_latch.h"

#include "comms_task.h"
#include "vn200_task.h"
#include "vn200.h"
#include "dvl_task.h"
#include "dvl.h"
#include "filter_task.h"
#include "bar30_task.h"
#include "logging_task.h"
#include "spi_owner_task.h"
#include "console.h"
#include "sensor_status.h"


//===========================================================================================================================
// Dummy task
//===========================================================================================================================

/*
 * P9 — task handles needed to call uxTaskGetStackHighWaterMark() on every
 * task. control/comms/vn200/dvl already had handles for other reasons;
 * these are new, added specifically for the stack audit below.
 */
static TaskHandle_t bar30TaskHandle    = NULL;
static TaskHandle_t filterTaskHandle   = NULL;
static TaskHandle_t spiOwnerTaskHandle = NULL;
static TaskHandle_t loggingTaskHandle  = NULL;
static TaskHandle_t dummyTaskHandle    = NULL;

/*
 * P9 stack audit.
 *
 * uxTaskGetStackHighWaterMark(handle) returns the SMALLEST amount of free
 * stack a task has ever had since it started, in words - not the current
 * free amount. It is a watermark, not a live gauge: a task that touched
 * 90% of its stack once, briefly, during startup reports that number
 * forever after.
 *
 * ---------------------------------------------------------------------
 * KNOWN BLIND SPOT - the numbers this prints are a LOWER BOUND on usage,
 * not a complete picture.
 *
 * A high-water mark only records paths that actually executed. Several
 * of the deepest paths in this firmware do not run in a quiet bench
 * session, so their stack cost will not appear here:
 *
 *   - console_printf() formats on the CALLER's stack: an 80-byte
 *     ConsoleLine plus vsnprintf's own frame, on top of whatever that
 *     task was already using. Most tasks only call it on an error path.
 *     spi_owner_task's SD-error branch is the clearest example - it
 *     never fires unless a write actually fails.
 *   - control_task's failsafe branch and the flush-on-disarm path need
 *     a link loss or a disarm to be exercised.
 *   - comms_task's TX branch needs a packet to send.
 *   - The sensor tasks' parse paths need real VN-200/DVL traffic, which
 *     needs hardware that is not attached.
 *
 * Read the marks as "at least this much was used", and do not trim a
 * stack on the strength of a run that never entered these branches. See
 * the P9 notes in the README for what would justify trimming.
 * ---------------------------------------------------------------------
 */
static void print_stack_audit(void)
{
    typedef struct
    {
        const char    *name;
        TaskHandle_t   handle;
        uint16_t       allocated_words;
    } AuditEntry;

    AuditEntry tasks[] =
    {
        { "Control", controlTaskHandle,  256 },
        { "Filter",  filterTaskHandle,   256 },
        { "Comms",   commsTaskHandle,    256 },
        { "VN200",   vn200TaskHandle,    256 },
        { "DVL",     dvlTaskHandle,      256 },
        { "Bar30",   bar30TaskHandle,    256 },
        { "SPIOwner",spiOwnerTaskHandle, 256 },
        { "Logging", loggingTaskHandle,  256 },
        { "Dummy",   dummyTaskHandle,    256 },
    };

    console_printf("--- STACK HWM (free words = min ever seen) ---");

    for (uint8_t i = 0; i < sizeof(tasks) / sizeof(tasks[0]); i++)
    {
        if (tasks[i].handle == NULL)
        {
            console_printf("%-8s NULL HANDLE - not tracked", tasks[i].name);
            continue;
        }

        UBaseType_t hwm_words = uxTaskGetStackHighWaterMark(tasks[i].handle);

        console_printf(
            "%-8s alloc=%4u w  free=%4lu w (%4lu B)  used<=%4lu B",
            tasks[i].name,
            tasks[i].allocated_words,
            (unsigned long)hwm_words,
            (unsigned long)(hwm_words * sizeof(StackType_t)),
            (unsigned long)((tasks[i].allocated_words - hwm_words)
                            * sizeof(StackType_t))
        );
    }

    console_printf("NOTE: lower bound - untaken branches not counted");
    console_printf("NOTE: static predictions include an ESTIMATED newlib");
    console_printf("      vsnprintf frame (~120B); treat as unverified");
    console_printf("--- END ---");
}

static void print_health(void)
{
    console_printf("--- HEALTH ---");
    console_printf("clk       : %s", system_clock_status_str(g_clock_status));
    console_printf("heap free : %u B of %u B",
                   (unsigned)xPortGetFreeHeapSize(),
                   (unsigned)configTOTAL_HEAP_SIZE);
    console_printf("heap min  : %u B ever free",
                   (unsigned)xPortGetMinimumEverFreeHeapSize());
    console_printf("log drops : %lu records",
                   (unsigned long)control_log_drops());
    console_printf("con drops : %lu lines",
                   (unsigned long)console_dropped());
    console_printf("sensors   : VN200=%s DVL=%s Bar30=%s",
                   sensor_state_str(g_vn200_state),
                   sensor_state_str(g_dvl_state),
                   sensor_state_str(g_bar30_state));
    console_printf("link      : %s  armed=%d  recovery=%u/%u",
                   control_loop_in_failsafe() ? "FAILSAFE" : "ok",
                   control_loop_get_armed() ? 1 : 0,
                   (unsigned)control_loop_recovery_count(),
                   (unsigned)CMD_RECOVERY_PACKETS);
    console_printf("sd blocks : %lu ok, %lu err",
                   (unsigned long)spi_owner_sd_writes(),
                   (unsigned long)spi_owner_sd_errors());
    console_printf("log stage : %lu recs, %lu blks, %lu post drops",
                   (unsigned long)logging_records_staged(),
                   (unsigned long)logging_blocks_emitted(),
                   (unsigned long)logging_spi_post_drops());
    console_printf("--- END ---");
}

static void print_help(void)
{
    console_printf("commands: s=stack hwm  h=health  ?=help");
}

static void dummy_task(void *argument)
{
    (void)argument;

    /*
     * THE SINGLE STDIO OWNER.
     *
     * configUSE_NEWLIB_REENTRANT is 0, so all nine tasks share one
     * struct _reent and one stdout FILE. Rather than pay ~96 bytes of
     * _reent per TCB to make printf reentrant, every other task calls
     * console_printf(), which formats into its own stack and posts the
     * bytes to consoleQueue. This task is the only one that performs
     * output, and it does so with uart2_write_str() - so after the
     * scheduler starts, stdout is never touched at all.
     *
     * The queue receive timeout doubles as the heartbeat tick, so no
     * second wake source is needed. Heartbeat phase is tracked against
     * xTaskGetTickCount rather than the timeout firing, so console
     * traffic cannot skew the blink rate.
     */
    ConsoleLine line;

    TickType_t last_blink = xTaskGetTickCount();
    uint8_t    blink_phase = 0;

    /* Let every task run through a few normal cycles before reading HWM. */
    TickType_t audit_at = xTaskGetTickCount() + pdMS_TO_TICKS(5000);
    uint8_t    audit_done = 0;

    for (;;)
    {
        if (xQueueReceive(consoleQueue, &line, pdMS_TO_TICKS(50)) == pdPASS)
        {
            uart2_write_str(line.text);
            uart2_write_str("\r\n");
        }

        /*
         * On-demand reports. The operator types a key on the VCP;
         * USART2_IRQHandler latches it and this poll picks it up on the
         * next 50 ms wake.
         */
        switch (console_take_command())
        {
            case 's':
            case 'S':
                print_stack_audit();
                break;

            case 'h':
            case 'H':
                print_health();
                break;

            case '?':
                print_help();
                break;

            default:
                break;
        }

        TickType_t now = xTaskGetTickCount();

        /*
         * One automatic report a few seconds after boot, so the numbers
         * exist even if nobody is watching the console. After that it is
         * on request only - see the blind-spot note on print_stack_audit.
         */
        if ((audit_done == 0U) && ((int32_t)(now - audit_at) >= 0))
        {
            audit_done = 1U;
            print_help();
            print_stack_audit();
            print_health();
        }

        /*
         * Heartbeat.
         *
         * A healthy 180 MHz board from the HSE gives a steady 1 Hz blink.
         * When the HSE was dead and the clock fell back to the HSI PLL,
         * the console may be unreadable - HSI is only about 1% accurate
         * at room temperature and worse across range, which is enough to
         * corrupt 115200 framing. So the fallback gets a visually
         * distinct double-pulse instead: blink-blink-pause, once a
         * second, readable across the room with no UART at all.
         */
        if (g_clock_status == CLOCK_DEGRADED_HSI_PLL)
        {
            /* 80 ms, 80 ms, 80 ms, then rest to fill 1000 ms. */
            static const TickType_t pattern[4] = { 80, 80, 80, 760 };

            if ((now - last_blink) >= pdMS_TO_TICKS(pattern[blink_phase]))
            {
                last_blink = now;
                blink_phase = (uint8_t)((blink_phase + 1U) & 0x3U);

                if (blink_phase == 3U)
                {
                    GPIOA->BSRR = (1U << (5 + 16));   /* off during rest */
                }
                else
                {
                    GPIOA->ODR ^= (1U << 5);
                }
            }
        }
        else
        {
            if ((now - last_blink) >= pdMS_TO_TICKS(500))
            {
                last_blink = now;
                GPIOA->ODR ^= (1U << 5);
            }
        }
    }
}


//===========================================================================================================================
// FreeRTOS stack overflow hook
//===========================================================================================================================
/*
 * Both hooks now latch into .noinit and halt, instead of blinking a LED
 * that told you nothing about which task died or why. The task name goes
 * into the latch detail field, so a warm reset prints it.
 */
void vApplicationStackOverflowHook(
    TaskHandle_t xTask,
    char *pcTaskName
)
{
    (void)xTask;

    fault_latch_fail(
        FAULT_STACK_OVERFLOW,
        __FILE__,
        __LINE__,
        (uint32_t)__builtin_return_address(0),
        (const char *)pcTaskName
    );
}


//===========================================================================================================================
// FreeRTOS malloc failure hook
//===========================================================================================================================
void vApplicationMallocFailedHook(void)
{
    fault_latch_fail(
        FAULT_MALLOC_FAILED,
        __FILE__,
        __LINE__,
        (uint32_t)__builtin_return_address(0),
        "pvPortMalloc returned NULL"
    );
}


//===========================================================================================================================
// Checked task creation
//===========================================================================================================================
/*
 * xTaskCreate returns errCOULD_NOT_ALLOCATE_REQUIRED_MEMORY when the
 * heap is exhausted. Every call used to discard that, so a heap that
 * ran out midway through init produced a board that started the
 * scheduler with tasks silently missing — the control loop simply
 * never running, with nothing to indicate why.
 *
 * Latch the task name and stop. This runs before the scheduler, so a
 * halt here is safe.
 */
static void create_task_checked(
    TaskFunction_t fn,
    const char *name,
    uint16_t stack_words,
    UBaseType_t priority,
    TaskHandle_t *handle
)
{
    BaseType_t ok = xTaskCreate(
        fn,
        name,
        stack_words,
        NULL,
        priority,
        handle
    );

    if (ok != pdPASS)
    {
        printf("TASK CREATE FAIL: %s\r\n", name);

        fault_latch_fail(
            FAULT_INIT_FAILED,
            __FILE__,
            __LINE__,
            (uint32_t)__builtin_return_address(0),
            name
        );
    }
}


//===========================================================================================================================
// MAIN
//===========================================================================================================================
int main(void)
{
    /*
     * 1. Configure system clock.
     *    180 MHz PLL.
     */
    system_clock_init();

    /*
     * A fatal clock status means the core is on the raw 16 MHz HSI. Every
     * UART divisor assumes a 45 MHz APB1, so the console would emit
     * garbage, and configCPU_CLOCK_HZ is a 180 MHz literal, so SysTick
     * would be programmed 11.25x fast. Neither the console nor the
     * scheduler can be trusted; blink the status code on LD2 forever and
     * go no further. This is also the invariant that lets
     * configCPU_CLOCK_HZ be a literal at all.
     */
    if (clock_status_is_fatal(g_clock_status))
    {
        clock_fault_blink_forever(g_clock_status);
    }


    /*
     * 2. NVIC priority grouping: 4 bits of preemption, 0 of subpriority.
     *
     * Before ANY NVIC_SetPriority call, so every priority written later
     * is interpreted the way it was meant. PRIGROUP does not alter stored
     * IPR bytes, only how the core splits them, but setting it first
     * removes the question entirely. All-preemption is what FreeRTOS
     * expects.
     */
    SCB->AIRCR =
        (0x5FAUL << SCB_AIRCR_VECTKEY_Pos) |
        (3UL << SCB_AIRCR_PRIGROUP_Pos);

    /*
     * Freeze the IWDG counter whenever the debugger halts the core.
     * Unconditional, deliberately NOT behind ENABLE_IWDG: the IWDG cannot
     * be stopped once started and does not halt with the core, so without
     * this every breakpoint becomes a reset a second later. No effect on
     * a free-running board, and a conditional version would be missing
     * from exactly the build being debugged.
     */
    iwdg_freeze_on_halt();


    /*
     * 3. Initialize the LD2 heartbeat LED on PA5.
     *
     * PA5 is genuinely a GPIO again — the SPI bus moved to SPI2 on
     * PB13/14/15, so nothing steals this pin later in init any more.
     */
    gpio_init(
        GPIOA,
        5
    );


    /*
     * 4. Initialize UART2 for debug output.
     */
    uart2_init();

    printf("BOOT OK\r\n");
    printf("CLK: %s\r\n", system_clock_status_str(g_clock_status));


    /*
     * 5. Report and clear any fault latched by the previous run.
     *
     * Deliberately placed here: the VCP is up so this is printable, and
     * nothing that could fault again has run yet. Prints nothing when no
     * fault is latched.
     */
    fault_latch_report();


    /*
     * 6. Initialize hardware CRC.
     */
    crc_init();


    /*
     * 7. Initialize SPI2 (OLED + SD card).
     *
     * ADC and DAC init used to sit here. Both were dead code — nothing
     * ever called adc_read() or dac_write() — and both claimed pins the
     * final map assigns elsewhere: PA0 is now UART4_TX, and PA4 is
     * reserved for a future ADC depth input and deliberately left
     * unconfigured. Drivers deleted.
     */
    spi2_init();


    /*
     * 8. Microsecond timebase.
     *
     * Placed before the SD card: sd_card.c bounds its PRE-SCHEDULER waits
     * with micros(), because the tick-based path is only valid once the
     * scheduler is running. Before that, xTickCount is 0 and never
     * advances, so a tick deadline could never expire, and vTaskDelay()
     * has no scheduler to return from.
     */
    timer2_timebase_init();


    /*
     * 9. Initialize SD card.
     */
    SD_Status sd_status =
        sd_init();

    printf("SD: %s\r\n", sd_status_str(sd_status));


    /*
     * 10. Initialize I2C1.
     */
    i2c1_init();


    /*
     * 11. Initialize OLED.
     */
    oled_init();

    oled_draw_string(
        0,
        0,
        "ROV OK"
    );

    oled_update();


    /*
     * 12. Initialize all eight thruster PWM outputs.
     *
     * TIM3 CH1-4 + TIM8 CH1-4, every channel written to 1500 us before
     * any output stage is enabled, counters running. The ESCs arm during
     * the remainder of boot.
     */
    pwm_init();


    /*
     * 13. Initialize control-loop state.
     */
    control_loop_init();


    /*
     * 14. Start the independent watchdog.
     *
     * Compiles to nothing unless ENABLE_IWDG is defined in iwdg.h, which
     * it is not by default. Started last among the peripherals so the
     * slower init steps above cannot trip it, and refreshed only by
     * control_task.
     */
    iwdg_init();


    /*
     * B6: tim7_init() used to sit here.
     *
     * It has moved into control_task's first iteration. TIM7's ISR
     * notifies controlTaskHandle and calls portYIELD_FROM_ISR, and
     * neither is safe until the scheduler is running and that handle is
     * populated — starting the timer here left a window where the 50 Hz
     * interrupt could fire before either was true.
     */


    /*
     * 15. Initialize queues.
     */

    commandQueue =
        xQueueCreate(
            4,
            sizeof(CommandPayload)
        );

    dvlQueue =
        xQueueCreate(
            1,
            sizeof(DVLData)
        );

    /*
     * VN-200 latest measurement queue.
     */
    vn200Queue =
        xQueueCreate(
            1,
            sizeof(VN200Data)
        );


    /*
     * DVL queue already exists.
     */


    /*
     * Bar30 latest depth queue.
     */
    bar30Queue =
        xQueueCreate(
            1,
            sizeof(float)
        );

    /*
     * Latest state-estimate queue (filter_task -> control_task).
     */
    stateQueue =
        xQueueCreate(
            1,
            sizeof(StateEstimate)
        );

    /*
     * Phase 8 — control_task -> logging_task.
     *
     * Depth 8 of LogQueueItem (44 B) = 352 B of storage. Records arrive
     * at 5 Hz and logging_task drains them into its staging block almost
     * instantly, only stalling when it posts a full block to the bus
     * owner. Depth 8 covers 1.6 s of records, comfortably longer than the
     * 250 ms worst-case SD program cycle that could hold it up.
     */
    logQueue =
        xQueueCreate(
            8,
            sizeof(LogQueueItem)
        );

    /*
     * Phase 8 — logging_task -> spi_owner_task.
     *
     * SpiRequest carries a whole 512-byte block by value, so each slot is
     * ~520 B. Depth 2 = ~1.1 KB of heap. Blocks now leave only every
     * ~2.4 s, so depth 2 is 4.8 s of absorption against a 250 ms
     * worst-case write. Passing by value rather than by pointer costs one
     * 512-byte memcpy per block - negligible at 0.4 Hz - and avoids any
     * buffer-ownership handoff between the two tasks.
     */
    spiRequestQueue =
        xQueueCreate(
            2,
            sizeof(SpiRequest)
        );

    /*
     * Phase 8 — every task -> dummy_task, the single stdio owner.
     */
    consoleQueue =
        xQueueCreate(
            CONSOLE_QUEUE_DEPTH,
            sizeof(ConsoleLine)
        );

    /*
     * Verify queue creation.
     *
     * A NULL queue here would otherwise surface much later as a
     * configASSERT deep inside the kernel on the first send. Latch the
     * real cause and stop before the scheduler ever starts.
     */
    if (commandQueue == NULL ||
        dvlQueue == NULL ||
        vn200Queue == NULL ||
        bar30Queue == NULL ||
        stateQueue == NULL ||
        logQueue == NULL ||
        spiRequestQueue == NULL ||
        consoleQueue == NULL)
    {
        printf("QUEUE CREATE FAIL\r\n");

        fault_latch_fail(
            FAULT_INIT_FAILED,
            __FILE__,
            __LINE__,
            (uint32_t)__builtin_return_address(0),
            "xQueueCreate returned NULL"
        );
    }


    /*
     * 16. Create the application tasks.
     *
     * Priority scheme (configMAX_PRIORITIES = 8, so 7 is the top):
     *
     *   7  Control   50 Hz deadline; nothing may delay it
     *   6  Filter    must have a fresh estimate ready before Control wakes
     *   5  Comms     command ingest and telemetry egress
     *   4  VN200 / DVL / Bar30   sensor drivers, equal and interchangeable
     *   3  SPIOwner  sole owner of SPI2 (OLED + SD)
     *   2  Logging   batches records; posts blocks to the bus owner
     *   1  Dummy     heartbeat, stack audit, single stdio owner
     *   0  IDLE      kernel
     *
     * Priority 2 is no longer shared with anything: configUSE_TIMERS is
     * now 0, so the timer service daemon that used to sit there is gone.
     *
     * Every creation is checked — see create_task_checked above.
     */
    create_task_checked(control_task, "Control Task", 256, 7, &controlTaskHandle);
    create_task_checked(filter_task,  "Filter Task",  256, 6, &filterTaskHandle);
    create_task_checked(comms_task,   "Comms Task",   256, 5, &commsTaskHandle);
    create_task_checked(vn200_task,   "VN200 Task",   256, 4, &vn200TaskHandle);
    create_task_checked(dvl_task,     "DVL Task",     256, 4, &dvlTaskHandle);
    create_task_checked(bar30_task,   "Bar30 Task",   256, 4, &bar30TaskHandle);
    create_task_checked(spi_owner_task, "SPI Owner",   256, 3, &spiOwnerTaskHandle);
    create_task_checked(logging_task, "Logging Task", 256, 2, &loggingTaskHandle);
    create_task_checked(dummy_task,   "Dummy",        256, 1, &dummyTaskHandle);


    /*
     * LAST BEFORE THE SCHEDULER: CONFIGURE the interrupt-driven UARTs.
     *
     * Configure only. Each NVIC line is enabled by its CONSUMING TASK, on
     * that task's first iteration:
     *
     *     USART1 -> commsTaskHandle  -> uart1_irq_enable() in comms_task
     *     USART3 -> vn200TaskHandle  -> uart3_irq_enable() in vn200_task
     *     UART4  -> dvlTaskHandle    -> uart4_irq_enable() in dvl_task
     *
     * Enabling the lines here would fix the NULL-handle problem and leave
     * the harder half. Between the last xTaskCreate and
     * vTaskStartScheduler(), pxCurrentTCB is populated but PSP is still
     * ZERO - vPortSVCHandler sets it, and that only runs inside
     * vPortStartFirstTask. An ISR reaching portYIELD_FROM_ISR in that
     * window pends PendSV, whose context save does 'mrs r0, PSP' then
     * 'stmdb r0!, {...}': a write through a null stack pointer. Same
     * defect class as B6.
     *
     * Deferring each enable to its task closes that window completely.
     * From vTaskStartScheduler() onward the kernel closes it too -
     * tasks.c calls portDISABLE_INTERRUPTS() (BASEPRI = 0x50) before
     * xPortStartScheduler(), masking every interrupt in this design
     * (0x50 and 0x60) until vPortSVCHandler clears BASEPRI with a task
     * genuinely running.
     */
    uart1_init();   /* Pi link      */
    uart3_init();   /* VN-200       */
    uart4_init();   /* Wayfinder DVL */

    printf("UARTS CONFIGURED (IRQs enabled by their tasks)\r\n");


    /*
     * 17. Start FreeRTOS scheduler.
     */
    vTaskStartScheduler();


    /*
     * Scheduler should never return.
     */
    while (1)
    {
    }
}
