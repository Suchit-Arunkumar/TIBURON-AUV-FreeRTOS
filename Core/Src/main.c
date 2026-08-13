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


//===========================================================================================================================
// Dummy task
//===========================================================================================================================
static void dummy_task(void *argument)
{
    (void)argument;

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
     * Verify queue creation.
     */
    if (commandQueue == NULL ||
        dvlQueue == NULL ||
        vn200Queue == NULL ||
        bar30Queue == NULL)
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
        NULL
    );


    /*
     * Create Phase 6 filter task.
     *
     * Priority = 6
     *
     * Control = 7
     * Filter  = 6
     * Comms   = 5
     * Sensors = 4
     */
    xTaskCreate(
        filter_task,
        "Filter Task",
        256,
        NULL,
        6,
        NULL
    );/*
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
        NULL
    );


    /*
     * Create Phase 6 filter task.
     *
     * Priority = 6
     *
     * Control = 7
     * Filter  = 6
     * Comms   = 5
     * Sensors = 4
     */
    xTaskCreate(
        filter_task,
        "Filter Task",
        256,
        NULL,
        6,
        NULL
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
        NULL
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
