#include "uart4.h"
#include "stm32f446xx.h"

#define APB1CLK 45000000U
#define UART4_BR 115200U

static uint8_t dma4_rx_buf[UART4_DMA_BUF_SIZE];

static uint16_t last_dma_pos = 0U;

static uint8_t uart4_rx_buf[UART4_DMA_BUF_SIZE];

static volatile uint16_t uart4_rx_head = 0U;
static volatile uint16_t uart4_rx_tail = 0U;

/* Bytes lost because the ring was full. ISR writes, any task reads. */
static volatile uint32_t uart4_rx_dropped_count = 0U;

TaskHandle_t dvlTaskHandle = NULL;


//===========================================================================================================================
static void uart4_rx_store(
    const uint8_t *data,
    uint16_t length
)
{
    for (uint16_t i = 0U; i < length; i++)
    {
        uint16_t next_head =
            (uint16_t)(
                (uart4_rx_head + 1U) %
                UART4_DMA_BUF_SIZE
            );


        /*
         * Ring buffer full.
         *
         * Drop incoming data rather than overwrite
         * unread bytes.
         */
        if (next_head == uart4_rx_tail)
        {
            uart4_rx_dropped_count += (uint32_t)(length - i);
            return;
        }


        uart4_rx_buf[uart4_rx_head] =
            data[i];

        uart4_rx_head =
            next_head;
    }
}


//===========================================================================================================================
void uart4_init(void)
{
    /*
     * GPIOA clock.
     *
     * DVL moved from PC10/PC11 to PA0/PA1 so USART3 can take PC10/PC11
     * for the VN-200. PA0 previously carried the unused ADC input, which
     * has been removed along with the DAC.
     */
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;


    /*
     * PA0 = UART4_TX (AF8).
     */
    GPIOA->MODER &= ~(3U << (2U * 0U));
    GPIOA->MODER |=  (2U << (2U * 0U));


    /*
     * PA1 = UART4_RX (AF8).
     */
    GPIOA->MODER &= ~(3U << (2U * 1U));
    GPIOA->MODER |=  (2U << (2U * 1U));

    /* PA1 pull-up: keeps an unconnected RX at idle-high, not floating */
    GPIOA->PUPDR &= ~(3U << (2U * 1U));
    GPIOA->PUPDR |=  (1U << (2U * 1U));


    /*
     * PA0 = AF8.
     */
    GPIOA->AFR[0] &= ~(0xFU << 0U);
    GPIOA->AFR[0] |=  (8U << 0U);


    /*
     * PA1 = AF8.
     */
    GPIOA->AFR[0] &= ~(0xFU << 4U);
    GPIOA->AFR[0] |=  (8U << 4U);


    /*
     * UART4 clock.
     */
    RCC->APB1ENR |= RCC_APB1ENR_UART4EN;


    /*
     * 115200 baud from 45 MHz APB1.
     */
    UART4->BRR =
        ((APB1CLK + UART4_BR / 2U) /
         UART4_BR);


    /*
     * Receiver enabled.
     */
    UART4->CR1 |= USART_CR1_RE;


    /*
     * IDLE interrupt.
     */
    UART4->CR1 |= USART_CR1_IDLEIE;


    /*
     * DMA RX request.
     */
    UART4->CR3 |= USART_CR3_DMAR;


    /*
     * UART4 enabled.
     */
    UART4->CR1 |= USART_CR1_UE;


    /*
     * DMA1 clock.
     */
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;


    /*
     * Disable DMA1 Stream2.
     */
    DMA1_Stream2->CR &= ~DMA_SxCR_EN;

    while (DMA1_Stream2->CR & DMA_SxCR_EN)
    {
    }


    /*
     * Clear DMA configuration.
     */
    DMA1_Stream2->CR = 0U;


    /*
     * Peripheral address.
     */
    DMA1_Stream2->PAR =
        (uint32_t)&UART4->DR;


    /*
     * Memory address.
     */
    DMA1_Stream2->M0AR =
        (uint32_t)dma4_rx_buf;


    /*
     * Circular buffer length.
     */
    DMA1_Stream2->NDTR =
        UART4_DMA_BUF_SIZE;


    /*
     * DMA Channel 4.
     */
    DMA1_Stream2->CR |=
        (4U << DMA_SxCR_CHSEL_Pos);


    /*
     * Peripheral-to-memory.
     */
    DMA1_Stream2->CR &= ~DMA_SxCR_DIR;


    /*
     * Memory increment.
     */
    DMA1_Stream2->CR |= DMA_SxCR_MINC;


    /*
     * Circular mode.
     */
    DMA1_Stream2->CR |= DMA_SxCR_CIRC;


    /*
     * 8-bit peripheral.
     */
    DMA1_Stream2->CR &= ~DMA_SxCR_PSIZE;


    /*
     * 8-bit memory.
     */
    DMA1_Stream2->CR &= ~DMA_SxCR_MSIZE;


    /*
     * Half-transfer and transfer-complete interrupts: see
     * uart4_dma_drain().
     */
    DMA1_Stream2->CR |= DMA_SxCR_HTIE | DMA_SxCR_TCIE;


    /*
     * Enable DMA.
     */
    DMA1_Stream2->CR |= DMA_SxCR_EN;


    /*
     * UART4 interrupt.
     */
    NVIC_SetPriority(
        UART4_IRQn,
        6
    );

    /* Same priority as UART4 on purpose: both handlers update
     * last_dma_pos, and equal priorities cannot preempt each other. */
    NVIC_SetPriority(DMA1_Stream2_IRQn, 6);

    /* NVIC_EnableIRQ deferred to uart4_irq_enable() - see below. */
}


//===========================================================================================================================
/*
 * Move whatever the DMA has written since the last call into the ring
 * buffer, then wake the DVL task.
 *
 * The DMA position is SIZE - NDTR, which says where the DMA is, not how
 * far it has gone. With the IDLE interrupt as the only trigger, a burst of
 * exactly 256 bytes with no gap brings the position back to where it
 * was and the whole burst looks like no data at all, and a stream with no
 * gaps never raises IDLE. The half-transfer and transfer-complete
 * interrupts fire every half buffer, so between two calls the DMA can
 * never move a full lap. USART1 already had this fix (uart_packet.c).
 */
static void uart4_dma_drain(void)
{
    uint16_t current_pos =
        (uint16_t)(UART4_DMA_BUF_SIZE - DMA1_Stream2->NDTR);

    if (current_pos >= UART4_DMA_BUF_SIZE)
    {
        current_pos = 0U;
    }

    uint16_t moved = 0U;

    if (current_pos > last_dma_pos)
    {
        moved = (uint16_t)(current_pos - last_dma_pos);
        uart4_rx_store(&dma4_rx_buf[last_dma_pos], moved);
    }
    else if (current_pos < last_dma_pos)
    {
        uart4_rx_store(&dma4_rx_buf[last_dma_pos],
                     (uint16_t)(UART4_DMA_BUF_SIZE - last_dma_pos));
        uart4_rx_store(&dma4_rx_buf[0], current_pos);
        moved = (uint16_t)(UART4_DMA_BUF_SIZE - last_dma_pos + current_pos);
    }

    last_dma_pos = current_pos;

    if ((moved != 0U) && (dvlTaskHandle != NULL))
    {
        BaseType_t woken = pdFALSE;

        xTaskNotifyFromISR(dvlTaskHandle, (1UL << 0), eSetBits, &woken);
        portYIELD_FROM_ISR(woken);
    }
}


void UART4_IRQHandler(void)
{
    if (UART4->SR & USART_SR_IDLE)
    {
        volatile uint32_t dummy;

        /* Clear IDLE: read SR, then DR. */
        dummy = UART4->SR;
        dummy = UART4->DR;
        (void)dummy;

        uart4_dma_drain();
    }
}


void DMA1_Stream2_IRQHandler(void)
{
    const uint32_t mask = DMA_LISR_HTIF2 | DMA_LISR_TCIF2;

    if ((DMA1->LISR & mask) == 0U)
    {
        return;
    }

    DMA1->LIFCR = DMA_LIFCR_CHTIF2 | DMA_LIFCR_CTCIF2;

    uart4_dma_drain();
}


uint32_t uart4_rx_dropped(void)
{
    return uart4_rx_dropped_count;
}


//===========================================================================================================================
uint16_t uart4_read(
    uint8_t *out,
    uint16_t max_len
)
{
    uint16_t count = 0U;


    if (out == NULL || max_len == 0U)
    {
        return 0U;
    }


    while ((uart4_rx_tail != uart4_rx_head) &&
           (count < max_len))
    {
        out[count++] =
            uart4_rx_buf[uart4_rx_tail];


        uart4_rx_tail =
            (uint16_t)(
                (uart4_rx_tail + 1U) %
                UART4_DMA_BUF_SIZE
            );
    }


    return count;
}

/*
 * B6-class hazard, second half.
 *
 * Configuring the peripheral and ENABLING its NVIC line are now separate.
 * Between the last xTaskCreate and vTaskStartScheduler(), pxCurrentTCB is
 * populated but PSP is still zero - vPortSVCHandler sets it, and that
 * only runs inside vPortStartFirstTask. An interrupt in that window that
 * reached portYIELD_FROM_ISR would pend PendSV, and PendSV's context save
 * does `mrs r0, PSP` then `stmdb r0!, {...}` - writing through a null
 * stack pointer.
 *
 * From vTaskStartScheduler() onward the window is closed by the kernel:
 * tasks.c does portDISABLE_INTERRUPTS() (BASEPRI = 0x50) before
 * xPortStartScheduler(), which masks every interrupt in this design
 * (0x50 and 0x60) until vPortSVCHandler clears BASEPRI with a task
 * running. So enabling the line from inside the consuming task's first
 * iteration means no kernel API is reachable from any ISR until a task
 * context genuinely exists.
 */
void uart4_irq_enable(void)
{
    NVIC_EnableIRQ(UART4_IRQn);
    NVIC_EnableIRQ(DMA1_Stream2_IRQn);
}
