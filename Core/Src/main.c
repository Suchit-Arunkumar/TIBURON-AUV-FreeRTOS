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
#include "display_task.h"


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
static TaskHandle_t displayTaskHandle  = NULL;
static TaskHandle_t loggingTaskHandle  = NULL;
static TaskHandle_t dummyTaskHandle    = NULL;

/*
 * P9 stack audit.
 *
 * uxTaskGetStackHighWaterMark(handle) returns the SMALLEST amount of free
 * stack a task has ever had since it started, in words — not the current
 * free amount. It's a watermark, not a live gauge. A task that used 90% of
 * its stack once, even briefly during startup, will report that 90% number
 * forever after, even if it's back to using 10% right now.
 *
 * This runs ONCE, several seconds after boot, so every task has gone
 * through at least a few iterations of its normal worst-case code path
 * (e.g. control_task's failsafe branch, comms_task's TX branch) before the
 * numbers are read. Numbers are printed as: allocated words, HWM free
 * words remaining, and free bytes remaining (HWM * sizeof(StackType_t)).
 *
 * This does NOT trim anything automatically. You read the printed output,
 * decide per-task whether the allocated size in xTaskCreate is wastefully
 * large, and edit those numbers yourself — that's the actual P9 work.
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
        { "Control", controlTaskHandle, 256 },
        { "Comms",   commsTaskHandle,   256 },
        { "VN200",   vn200TaskHandle,   256 },
        { "DVL",     dvlTaskHandle,     256 },
        { "Bar30",   bar30TaskHandle,   256 },
        { "Filter",  filterTaskHandle,  256 },
        { "Display", displayTaskHandle, 256 },
        { "Logging", loggingTaskHandle, 256 },
        { "Dummy",   dummyTaskHandle,   128 },
    };

    printf("\r\n=== P9 STACK AUDIT (words free = min-ever-seen, not current) ===\r\n");

    for (uint8_t i = 0; i < sizeof(tasks) / sizeof(tasks[0]); i++)
    {
        if (tasks[i].handle == NULL)
        {
            printf("%-8s NULL HANDLE - not tracked\r\n", tasks[i].name);
            continue;
        }

        UBaseType_t hwm_words = uxTaskGetStackHighWaterMark(tasks[i].handle);

        printf(
            "%-8s alloc=%4u words  hwm_free=%4lu words  (%4lu bytes)\r\n",
            tasks[i].name,
            tasks[i].allocated_words,
            (unsigned long)hwm_words,
            (unsigned long)(hwm_words * sizeof(StackType_t))
        );
    }

    printf("=== END AUDIT ===\r\n\r\n");
}

static void dummy_task(void *argument)
{
    (void)argument;

    /* Let every task run through a few normal cycles before reading HWM. */
    vTaskDelay(pdMS_TO_TICKS(5000));
    print_stack_audit();

    while (1)
    {
        GPIOA->ODR ^= (1U << 5);

        vTaskDelay(
            pdMS_TO_TICKS(500)
        );
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
     * 2. Initialize the LD2 heartbeat LED on PA5.
     *
     * PA5 is genuinely a GPIO again — the SPI bus moved to SPI2 on
     * PB13/14/15, so nothing steals this pin later in init any more.
     */
    gpio_init(
        GPIOA,
        5
    );


    /*
     * 3. Initialize UART2 for debug output.
     */
    uart2_init();

    printf("BOOT OK\r\n");
    printf("CLK: %s\r\n", system_clock_status_str(g_clock_status));


    /*
     * 3b. Report and clear any fault latched by the previous run.
     *
     * Deliberately placed here: the VCP is up so this is printable, and
     * nothing that could fault again has run yet. Prints nothing when no
     * fault is latched.
     */
    fault_latch_report();


    /*
     * 4. Initialize hardware CRC.
     */
    crc_init();


    /*
     * 5. Initialize SPI2 (OLED + SD card).
     *
     * ADC and DAC init used to sit here. Both were dead code — nothing
     * ever called adc_read() or dac_write() — and both claimed pins the
     * final map assigns elsewhere: PA0 is now UART4_TX, and PA4 is
     * reserved for a future ADC depth input and deliberately left
     * unconfigured. Drivers deleted.
     */
    spi2_init();


    /*
     * 8. Initialize SD card.
     */
    SD_Status sd_status =
        sd_init();

    if (sd_status == SD_OK)
    {
        printf("SD OK\r\n");
    }
    else
    {
        printf("SD FAIL\r\n");
    }


    /*
     * 9. Initialize I2C1.
     */
    i2c1_init();


    /*
     * 10. Initialize Bar30.
     */
    bar30_init();


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
     * 12. Initialize USART1.
     *
     * USART1:
     *     DMA RX
     *     IDLE interrupt
     *     Raspberry Pi communications
     */
    uart1_init();

    uart2_write_str(
        "UART1 initialized\r\n"
    );


    /*
     * 13. Initialize USART3.
     *
     * USART3:
     *     DMA RX
     *     IDLE interrupt
     *     VN-200
     */
    uart3_init();


    /*
     * 14. Initialize UART4.
     *
     * UART4:
     *     DMA RX
     *     IDLE interrupt
     *     Wayfinder DVL
     */
    uart4_init();


    /*
     * 15. Initialize all eight thruster PWM outputs.
     *
     * TIM3 CH1-4 + TIM8 CH1-4, every channel written to 1500 us before
     * any output stage is enabled, counters running. The ESCs arm during
     * the remainder of boot.
     */
    pwm_init();


    /*
     * 16. Initialize microsecond timebase.
     */
    timer2_timebase_init();


    /*
     * 17. Configure NVIC priority grouping.
     */
    SCB->AIRCR =
        (0x5FAUL << SCB_AIRCR_VECTKEY_Pos) |
        (3UL << SCB_AIRCR_PRIGROUP_Pos);


    /*
     * 18. Initialize control-loop state.
     */
    control_loop_init();


    /*
     * 19. Start the independent watchdog.
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
     * 20. Initialize queues.
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
     * Depth 4: absorbs one slow SD write cycle without blocking
     * control_task's non-blocking send.
     */
    logQueue =
        xQueueCreate(
            4,
            sizeof(LogRecord)
        );

    /*
     * Phase 8 — logging_task -> display_task (SPI bus owner).
     */
    spiRequestQueue =
        xQueueCreate(
            4,
            sizeof(SpiRequest)
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
        spiRequestQueue == NULL)
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
     * 21. Create the application tasks.
     *
     * Priority scheme (configMAX_PRIORITIES = 8, so 7 is the top):
     *
     *   7  Control   50 Hz deadline; nothing may delay it
     *   6  Filter    must have a fresh estimate ready before Control wakes
     *   5  Comms     command ingest and telemetry egress
     *   4  VN200 / DVL / Bar30   sensor drivers, equal and interchangeable
     *   3  Display   owns SPI2
     *   2  Logging   feeds Display; never touches SPI itself
     *   1  Dummy     heartbeat and the one-shot stack audit
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
    create_task_checked(display_task, "Display Task", 256, 3, &displayTaskHandle);
    create_task_checked(logging_task, "Logging Task", 256, 2, &loggingTaskHandle);
    create_task_checked(dummy_task,   "Dummy",        128, 1, &dummyTaskHandle);


    /*
     * 26. Start FreeRTOS scheduler.
     */
    vTaskStartScheduler();


    /*
     * Scheduler should never return.
     */
    while (1)
    {
    }
}
