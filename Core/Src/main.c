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


#include "i2c.h"
#include "bar30.h"

#include "iwdg.h"

#include "fault_latch.h"

#include "comms_task.h"
#include "imu_task.h"
#include "vn200.h"
#include "dvl_task.h"
#include "dvl.h"
#include "depth_task.h"
#include "logging_task.h"
#include "spi_owner_task.h"
#include "console.h"
#include "sensor_status.h"
#include "bench.h"


static TaskHandle_t depthTaskHandle    = NULL;
static TaskHandle_t spiOwnerTaskHandle = NULL;
static TaskHandle_t loggingTaskHandle  = NULL;
static TaskHandle_t dummyTaskHandle    = NULL;

/*
 * Stack high-water marks: the least free stack each task has ever had.
 * Only paths that actually ran are counted, so treat these as a lower
 * bound until the error paths have been exercised (the bench script does).
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
        { "Comms",   commsTaskHandle,    256 },
        { "IMU",     imuTaskHandle,      384 },
        { "DVL",     dvlTaskHandle,      256 },
        { "Depth",   depthTaskHandle,    384 },
        { "SPIOwner",spiOwnerTaskHandle, 256 },
        { "Logging", loggingTaskHandle,  256 },
        { "Dummy",   dummyTaskHandle,    384 },
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
    console_printf("sensors   : VN200=%s BNO085=%s DVL=%s Bar30=%s ADC=%s",
                   sensor_state_str(g_vn200_state),
                   sensor_state_str(g_bno085_state),
                   sensor_state_str(g_dvl_state),
                   sensor_state_str(g_bar30_state),
                   sensor_state_str(g_adc_depth_state));
    console_printf("link      : %s  armed=%d  recovery=%u/%u",
                   control_loop_in_failsafe() ? "FAILSAFE" : "ok",
                   control_loop_get_armed() ? 1 : 0,
                   (unsigned)control_loop_recovery_count(),
                   (unsigned)CMD_RECOVERY_PACKETS);
    console_printf("cmd pkts  : %lu valid, %lu dropped (queue full)",
                   (unsigned long)comms_cmd_valid(),
                   (unsigned long)comms_cmd_drops());
    console_printf("tx frames : %lu dropped (ring full)",
                   (unsigned long)comms_tx_drops());
    console_printf("rx drops  : VN200=%lu DVL=%lu bytes",
                   (unsigned long)uart3_rx_dropped(),
                   (unsigned long)uart4_rx_dropped());
    console_printf("vn200 pkt : %lu ok, %lu bad",
                   (unsigned long)vn200_packets_ok(),
                   (unsigned long)vn200_packets_bad());
    console_printf("dvl frame : %lu ok, %lu bad (sum incl=%lu excl=%lu)",
                   (unsigned long)dvl_frames_ok(),
                   (unsigned long)dvl_frames_bad(),
                   (unsigned long)dvl_sum_incl_count(),
                   (unsigned long)dvl_sum_excl_count());
    console_printf("wdg/ramp  : IWDG=%s  slew_limit=%s",
                   iwdg_is_enabled() ? "ARMED" : "DISABLED",
                   control_loop_ramping() ? "ramping" : "off (full authority)");
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
    bench_print_help();
}


/* Why the last reset happened (RCC_CSR), then clear the flags so the next
 * boot reports only its own cause. */
static void reset_cause_report(void)
{
    uint32_t csr = RCC->CSR;

    printf("RESET:%s%s%s%s%s%s%s\r\n",
           (csr & RCC_CSR_LPWRRSTF) ? " LPWR" : "",
           (csr & RCC_CSR_WWDGRSTF) ? " WWDG" : "",
           (csr & RCC_CSR_IWDGRSTF) ? " IWDG" : "",
           (csr & RCC_CSR_SFTRSTF)  ? " SOFT" : "",
           (csr & RCC_CSR_PORRSTF)  ? " POR"  : "",
           (csr & RCC_CSR_PINRSTF)  ? " PIN"  : "",
           (csr & RCC_CSR_BORRSTF)  ? " BOR"  : "");

    RCC->CSR |= RCC_CSR_RMVF;
}

/*
 * The only task that prints. Others call console_printf(), which formats
 * on their own stack and queues the line here, so printf never needs to be
 * reentrant. The 50 ms receive timeout doubles as the heartbeat tick.
 */
static void dummy_task(void *argument)
{
    (void)argument;

    ConsoleLine line;

    console_set_owner();
    bench_dummy_first_run();

    TickType_t last_blink = xTaskGetTickCount();
    uint8_t    blink_phase = 0;

    /* Let every task run a few cycles before the first report. */
    TickType_t audit_at = xTaskGetTickCount() + pdMS_TO_TICKS(5000);
    uint8_t    audit_done = 0;

    for (;;)
    {
        if (xQueueReceive(consoleQueue, &line, pdMS_TO_TICKS(50)) == pdPASS)
        {
            uart2_write_str(line.text);
            uart2_write_str("\r\n");
        }

        /* Key typed on the console, latched by USART2_IRQHandler. */
        char c = console_take_command();

        switch (c)
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
                (void)bench_console_key(c);
                break;
        }

        TickType_t now = xTaskGetTickCount();

        if ((audit_done == 0U) && ((int32_t)(now - audit_at) >= 0))
        {
            audit_done = 1U;
            print_help();
            print_stack_audit();
            print_health();
        }

        /*
         * Heartbeat, readable without a console:
         *   steady 1 Hz   all good
         *   double pulse  running on the HSI fallback clock (UART timing
         *                 may be off, so the console may be garbled)
         *   short blip    no watchdog in this build
         */
        {
            static const TickType_t pat_degraded[4] = {  80,  80,  80, 760 };
            static const TickType_t pat_no_wdg[2]   = {  60, 1940 };

            const TickType_t *pattern;
            uint8_t phase_count;
            uint8_t off_phase;

            if (g_clock_status == CLOCK_DEGRADED_HSI_PLL)
            {
                pattern     = pat_degraded;
                phase_count = 4U;
                off_phase   = 3U;
            }
            else if (!iwdg_is_enabled())
            {
                pattern     = pat_no_wdg;
                phase_count = 2U;
                off_phase   = 1U;
            }
            else
            {
                pattern     = NULL;
                phase_count = 0U;
                off_phase   = 0U;
            }

            if (pattern == NULL)
            {
                if ((now - last_blink) >= pdMS_TO_TICKS(500))
                {
                    last_blink = now;
                    GPIOA->ODR ^= (1U << 5);
                }
            }
            else
            {
                if ((now - last_blink) >= pdMS_TO_TICKS(pattern[blink_phase]))
                {
                    last_blink  = now;
                    blink_phase = (uint8_t)((blink_phase + 1U) % phase_count);

                    if (blink_phase == off_phase)
                    {
                        GPIOA->BSRR = (1U << (5 + 16));   /* dark for the rest */
                    }
                    else
                    {
                        GPIOA->ODR ^= (1U << 5);
                    }
                }
            }
        }
    }
}


/* Both FreeRTOS hooks save the cause in the fault latch and halt; the
 * next boot prints it. */
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


/* A task that fails to create (heap exhausted) stops the boot with its
 * name saved, instead of starting the scheduler without it. */
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


int main(void)
{
    /* 180 MHz from the PLL. */
    system_clock_init();

    /*
     * Fatal means the PLL never came up and the core is on the raw 16 MHz
     * HSI. Every UART divisor and the SysTick setting assume 180 MHz, so
     * nothing can be trusted: blink the error code and stop.
     */
    if (clock_status_is_fatal(g_clock_status))
    {
        clock_fault_blink_forever(g_clock_status);
    }

    /* All 4 priority bits for preemption, none for sub-priority, as
     * FreeRTOS expects. Set before any NVIC_SetPriority. */
    SCB->AIRCR =
        (0x5FAUL << SCB_AIRCR_VECTKEY_Pos) |
        (3UL << SCB_AIRCR_PRIGROUP_Pos);

    /* The IWDG keeps counting when a debugger halts the core; without
     * this every breakpoint would reset the board. */
    iwdg_freeze_on_halt();

    gpio_init(GPIOA, 5);    /* LD2 heartbeat */

    uart2_init();           /* console */

    printf("BOOT OK\r\n");
    printf("CLK: %s\r\n", system_clock_status_str(g_clock_status));
    reset_cause_report();

    /* Print and clear any fault saved by the previous run. */
    fault_latch_report();

    spi2_init();

    /* Microsecond timer. Before the SD card, which uses it for timeouts
     * while the scheduler (and so the tick) isn't running yet. */
    timer2_timebase_init();

    bench_init();

    SD_Status sd_status = sd_init();

    printf("SD: %s\r\n", sd_status_str(sd_status));

    i2c1_init();
    int i2c_lock_ok = i2c1_lock_create();

    oled_init();
    oled_draw_string(0, 0, "ROV OK");
    oled_update();

    /* All eight outputs at 1500 us before they are enabled; the ESCs arm
     * during the rest of boot. */
    pwm_init();

    control_loop_init();

    /* Release builds only (iwdg.h). Started after the slow init steps. */
    iwdg_init();

    if (iwdg_is_enabled())
    {
        printf("IWDG: ENABLED (~1 s, kicked by control_task only)\r\n");
    }
    else
    {
        printf("**********************************************\r\n");
        printf("*  IWDG DISABLED - BENCH BUILD               *\r\n");
        printf("*  No watchdog. A hung control loop will NOT  *\r\n");
        printf("*  reset the board or stop the thrusters.     *\r\n");
        printf("*  Release builds always have it.            *\r\n");
        printf("**********************************************\r\n");
    }

    /*
     * Queues. Everything is allocated here, before the scheduler starts,
     * so the heap can't run out later. Sizes and policies: README section 5.
     */
    commandQueue    = xQueueCreate(4, sizeof(CommandPayload));
    dvlQueue        = xQueueCreate(1, sizeof(DVLData));
    imuQueue        = xQueueCreate(1, sizeof(ImuSample));
    depthQueue      = xQueueCreate(1, sizeof(DepthSample));
    logQueue        = xQueueCreate(8, sizeof(LogQueueItem));
    spiRequestQueue = xQueueCreate(2, sizeof(SpiRequest));
    consoleQueue    = xQueueCreate(CONSOLE_QUEUE_DEPTH, sizeof(ConsoleLine));

    if (commandQueue == NULL ||
        dvlQueue == NULL ||
        imuQueue == NULL ||
        depthQueue == NULL ||
        logQueue == NULL ||
        spiRequestQueue == NULL ||
        consoleQueue == NULL ||
        !i2c_lock_ok)
    {
        printf("QUEUE OR MUTEX CREATE FAIL\r\n");

        fault_latch_fail(
            FAULT_INIT_FAILED,
            __FILE__,
            __LINE__,
            (uint32_t)__builtin_return_address(0),
            "queue/mutex create NULL"
        );
    }

    /* Priorities: README section 4. Stack sizes are in words (4 bytes). */
    create_task_checked(control_task, "Control Task", 256, 7, &controlTaskHandle);
    create_task_checked(comms_task,   "Comms Task",   256, 5, &commsTaskHandle);
    create_task_checked(imu_task,     "IMU Task",     384, 4, &imuTaskHandle);
    create_task_checked(dvl_task,     "DVL Task",     256, 4, &dvlTaskHandle);
    create_task_checked(depth_task,   "Depth Task",   384, 4, &depthTaskHandle);
    create_task_checked(spi_owner_task, "SPI Owner",   256, 3, &spiOwnerTaskHandle);
    create_task_checked(logging_task, "Logging Task", 256, 2, &loggingTaskHandle);
    /* 1536 B: the bench measured 928 of 1024 B used, mostly vsnprintf. */
    create_task_checked(dummy_task,   "Dummy",        384, 1, &dummyTaskHandle);

    bench_register_task("Ctl",  controlTaskHandle);
    bench_register_task("Com",  commsTaskHandle);
    bench_register_task("IMU",  imuTaskHandle);
    bench_register_task("DVL",  dvlTaskHandle);
    bench_register_task("Dep",  depthTaskHandle);
    bench_register_task("SPI",  spiOwnerTaskHandle);
    bench_register_task("Log",  loggingTaskHandle);
    bench_register_task("Dum",  dummyTaskHandle);

    /*
     * Configure the UARTs, but leave their interrupts off: each task turns
     * on its own when it first runs. An interrupt that woke a task before
     * the scheduler started would write through a null stack pointer (PSP
     * is still 0 until the first task starts).
     */
    uart1_init();   /* Pi link       */
    uart3_init();   /* VN-200        */
    uart4_init();   /* Wayfinder DVL */

    printf("UARTS CONFIGURED (IRQs enabled by their tasks)\r\n");

#if BENCH_HIL
    printf("**********************************************\r\n");
    printf("*  BENCH_HIL BUILD - laptop test hooks in    *\r\n");
    printf("*  Console keys can drive ESC outputs off    *\r\n");
    printf("*  neutral. Never flash this to the vehicle. *\r\n");
    printf("*  Use the Release build for the vehicle.    *\r\n");
    printf("**********************************************\r\n");
#endif

    vTaskStartScheduler();

    /* Only reached if the scheduler could not start. */
    while (1)
    {
    }
}
