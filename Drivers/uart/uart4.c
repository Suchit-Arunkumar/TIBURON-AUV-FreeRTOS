// UART4: Wayfinder DVL (through an RS-232 converter), 115200 baud, RX only.
// Same pattern as uart3.c: circular DMA, drained into a ring on IDLE and
// half/full-transfer interrupts, read by dvl_task.

#include "uart4.h"
#include "stm32f446xx.h"

#define APB1CLK 45000000U
#define UART4_BR 115200U

static uint8_t dma4_rx_buf[UART4_DMA_BUF_SIZE];

static uint16_t last_dma_pos = 0U;

static uint8_t uart4_rx_buf[UART4_DMA_BUF_SIZE];

static volatile uint16_t uart4_rx_head = 0U;
static volatile uint16_t uart4_rx_tail = 0U;

static volatile uint32_t uart4_rx_dropped_count = 0U;

TaskHandle_t dvlTaskHandle = NULL;


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

        // full: drop new bytes rather than overwrite unread ones
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


void uart4_init(void)
{
    // PA0 TX, PA1 RX, AF8
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;

    GPIOA->MODER &= ~(3U << (2U * 0U));
    GPIOA->MODER |=  (2U << (2U * 0U));

    GPIOA->MODER &= ~(3U << (2U * 1U));
    GPIOA->MODER |=  (2U << (2U * 1U));

    // pull-up so an unconnected RX idles high instead of floating
    GPIOA->PUPDR &= ~(3U << (2U * 1U));
    GPIOA->PUPDR |=  (1U << (2U * 1U));

    GPIOA->AFR[0] &= ~(0xFU << 0U);
    GPIOA->AFR[0] |=  (8U << 0U);

    GPIOA->AFR[0] &= ~(0xFU << 4U);
    GPIOA->AFR[0] |=  (8U << 4U);

    RCC->APB1ENR |= RCC_APB1ENR_UART4EN;

    UART4->BRR =
        ((APB1CLK + UART4_BR / 2U) /
         UART4_BR);

    UART4->CR1 |= USART_CR1_RE;
    UART4->CR1 |= USART_CR1_IDLEIE;
    UART4->CR3 |= USART_CR3_DMAR;
    UART4->CR1 |= USART_CR1_UE;

    // DMA1 Stream2 channel 4 = UART4_RX, circular, bytes
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;

    DMA1_Stream2->CR &= ~DMA_SxCR_EN;

    while (DMA1_Stream2->CR & DMA_SxCR_EN)
    {
    }

    DMA1_Stream2->CR = 0U;

    DMA1_Stream2->PAR =
        (uint32_t)&UART4->DR;

    DMA1_Stream2->M0AR =
        (uint32_t)dma4_rx_buf;

    DMA1_Stream2->NDTR =
        UART4_DMA_BUF_SIZE;

    DMA1_Stream2->CR |=
        (4U << DMA_SxCR_CHSEL_Pos);

    DMA1_Stream2->CR &= ~DMA_SxCR_DIR;

    DMA1_Stream2->CR |= DMA_SxCR_MINC;

    DMA1_Stream2->CR |= DMA_SxCR_CIRC;

    DMA1_Stream2->CR &= ~DMA_SxCR_PSIZE;

    DMA1_Stream2->CR &= ~DMA_SxCR_MSIZE;

    DMA1_Stream2->CR |= DMA_SxCR_HTIE | DMA_SxCR_TCIE;

    DMA1_Stream2->CR |= DMA_SxCR_EN;

    // Same priority for both: they share last_dma_pos. Enabled by
    // uart4_irq_enable().
    NVIC_SetPriority(
        UART4_IRQn,
        6
    );

    NVIC_SetPriority(DMA1_Stream2_IRQn, 6);
}


// Copy what the DMA wrote since last time into the ring and wake the task.
// See uart3.c for why the half/full-transfer events are needed too.
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

        // reading SR then DR clears IDLE
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

// Called by dvl_task when it first runs, never from main (see uart3.c).
void uart4_irq_enable(void)
{
    NVIC_EnableIRQ(UART4_IRQn);
    NVIC_EnableIRQ(DMA1_Stream2_IRQn);
}
