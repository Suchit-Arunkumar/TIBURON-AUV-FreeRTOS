#include <stdio.h>
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>
#include "system_init.h"
#include "gpio.h"
#include "uart.h"
#include "uart_packet.h"
#include "spi.h"
#include "oled.h"
#include "sd_card.h"
#include "timer_basic.h"
#include "control_loop.h"
#include "adc.h"
#include "dac.h"
#include "struct.h"
#include "packet.h"
#include "sd_logger.h"
#include "crc_hw.h"
#include "i2c.h"
#include "bar30.h"
#include "timer_pwm.h"
#include "timer_timebase.h"
#include "iwdg.h"


static void dummy_task(void *argument)
{
    (void)argument;

    while (1)
    {
        GPIOA->ODR ^= (1U << 5);

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

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

        for (volatile uint32_t i = 0; i < 500000; i++)
        {
        }
    }
}


void vApplicationMallocFailedHook(void)
{
    __disable_irq();

    while (1)
    {
        GPIOA->ODR ^= (1U << 5);

        for (volatile uint32_t i = 0; i < 500000; i++)
        {
        }
    }
}

int main(void)
{
    // 1. Configure system clocks (180 MHz PLL)
    system_clock_init();

    // 3. Initialize GPIO
    gpio_init(GPIOA, 5);

    // 4. Initialize UART2 for debug prints
    uart2_init();
    printf("BOOT OK\r\n");

    // 5. Initialize CRC peripheral
    crc_init();

    // 6. Initialize ADC
    adc_init();

    // 7. Initialize DAC
    dac_init();

    // 8. Initialize SPI
    spi1_init();

    // 9. Initialize SD card (slow startup)
    SD_Status sd_status = sd_init();
    if (sd_status == SD_OK)
    {
        printf("SD OK\r\n");
    }
    else
    {
        printf("SD FAIL\r\n");
    }

    // 10. Initialize I2C
    i2c1_init();

    // 11. Initialize Bar30 pressure sensor
    bar30_init();

    // 12. Initialize OLED display
    oled_init();
    oled_draw_string(0, 0, "ROV OK");
    oled_update();

    // 13. initialize USART1 with DMA RX and IDLE interrupt
    uart1_init();

    // print confirmation message over UART2 that UART1 is up
    uart2_write_str("UART1 initialized\r\n");

    // 14. Initialize thruster PWM outputs (neutral)
    timer3_pwm_init();

    // 15. Initialize microsecond timebase
    timer2_timebase_init();

    // 18. Arm watchdog LAST
   // iwdg_init();

    SCB->AIRCR =
        (0x5FAUL << SCB_AIRCR_VECTKEY_Pos) |
        (3UL << SCB_AIRCR_PRIGROUP_Pos);

    // 16. Initialize control loop state
    control_loop_init();

    xTaskCreate(
        control_task,
        "Control Task",
        256,
        NULL,
        6,
        &controlTaskHandle
    );

    // 17. Start 50 Hz control loop timer ISR
    tim7_init();



    xTaskCreate(
        dummy_task,
        "Dummy",
        128,
        NULL,
        1,
        NULL
    );

    vTaskStartScheduler();

    while (1)
    {
        /* Should never reach here */
    }
}

