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

#include "adc.h"
#include "dac.h"

#include "struct.h"
#include "packet.h"
#include "sd_logger.h"

#include "crc_hw.h"

#include "i2c.h"
#include "bar30.h"

#include "iwdg.h"

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
void vApplicationStackOverflowHook(
    TaskHandle_t xTask,
    char *pcTaskName
)
{
    (void)xTask;
    (void)pcTaskName;

    __disable_irq();

    while (1)
    {
        GPIOA->ODR ^= (1U << 5);

        for (volatile uint32_t i = 0;
             i < 500000;
             i++)
        {
        }
    }
}


//===========================================================================================================================
// FreeRTOS malloc failure hook
//===========================================================================================================================
void vApplicationMallocFailedHook(void)
{
    __disable_irq();

    while (1)
    {
        GPIOA->ODR ^= (1U << 5);

        for (volatile uint32_t i = 0;
             i < 500000;
             i++)
        {
        }
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
     * 2. Initialize GPIO.
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


    /*
     * 4. Initialize hardware CRC.
     */
    crc_init();


    /*
     * 5. Initialize ADC.
     */
    adc_init();


    /*
     * 6. Initialize DAC.
     */
    dac_init();


    /*
     * 7. Initialize SPI1.
     */
    spi1_init();


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
     * 15. Initialize thruster PWM outputs.
     *
     * Outputs start at neutral.
     */
    timer3_pwm_init();


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
     * 19. Start 50 Hz control-loop timer.
     */
    tim7_init();


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
     */
    if (commandQueue == NULL ||
        dvlQueue == NULL ||
        vn200Queue == NULL ||
        bar30Queue == NULL ||
        stateQueue == NULL ||
        logQueue == NULL ||
        spiRequestQueue == NULL)
    {
        printf(
            "QUEUE CREATE FAIL\r\n"
        );

        __disable_irq();

        while (1)
        {
        }
    }


    /*
     * 21. Create Control task.
     *
     * Priority = 7
     */
    xTaskCreate(
        control_task,
        "Control Task",
        256,
        NULL,
        7,
        &controlTaskHandle
    );


    /*
     * 22. Create Communications task.
     *
     * Priority = 5
     */
    xTaskCreate(
        comms_task,
        "Comms Task",
        256,
        NULL,
        5,
        &commsTaskHandle
    );


    /*
     * 23. Create VN-200 task.
     *
     * Priority = 4
     */
    xTaskCreate(
        vn200_task,
        "VN200 Task",
        256,
        NULL,
        4,
        &vn200TaskHandle
    );


    /*
     * 24. Create DVL task.
     *
     * Priority = 4
     */
    xTaskCreate(
        dvl_task,
        "DVL Task",
        256,
        NULL,
        4,
        &dvlTaskHandle
    );

    /*
     * Create Bar30 task.
     *
     * Priority = 4
     */
    xTaskCreate(
        bar30_task,
        "Bar30 Task",
        256,
        NULL,
        4,
        &bar30TaskHandle
    );


    /*
     * Create Phase 6 filter task.
     *
     * Priority = 6
     *
     * Control  = 7
     * Filter   = 6
     * Comms    = 5
     * Sensors  = 4
     * Display  = 3
     * Logging  = 2
     * Dummy    = 1
     */
    xTaskCreate(
        filter_task,
        "Filter Task",
        256,
        NULL,
        6,
        &filterTaskHandle
    );

    /*
     * Phase 8 — Create Display task.
     *
     * Sole owner of SPI1 (OLED + SD). Priority = 3.
     */
    xTaskCreate(
        display_task,
        "Display Task",
        256,
        NULL,
        3,
        &displayTaskHandle
    );

    /*
     * Phase 8 — Create Logging task.
     *
     * Lowest of the "real" tasks — never touches SPI directly.
     * Priority = 2.
     */
    xTaskCreate(
        logging_task,
        "Logging Task",
        256,
        NULL,
        2,
        &loggingTaskHandle
    );

    /*
     * 25. Create dummy task.
     *
     * Priority = 1
     */
    xTaskCreate(
        dummy_task,
        "Dummy",
        128,
        NULL,
        1,
        &dummyTaskHandle
    );


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
