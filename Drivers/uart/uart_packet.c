// USART1: Raspberry Pi link, 115200 baud. RX: circular DMA (DMA2 Stream2)
// into the packet ring buffer. TX: one frame at a time on DMA2 Stream7.

#include "uart_packet.h"
#include "stm32f446xx.h"
#include "ring_buffer.h"
#include "comms_task.h"
#include "bench.h"
#include <stdbool.h>

#define APB2CLK 90000000U
#define UART1_BR 115200U

static uint8_t dma_rx_buf[DMA_BUF_SIZE];

static uint16_t last_dma_pos = 0;

void uart1_init(void)
{
	// PA9 TX, PA10 RX, AF7
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;

	GPIOA->MODER &= ~(3U << (2*9));
	GPIOA->MODER |=  (2U << (2*9));

	GPIOA->MODER &= ~(3U << (2*10));
	GPIOA->MODER |=  (2U << (2*10));

	// pull-up so an unconnected RX idles high instead of floating
	GPIOA->PUPDR &= ~(3U << (2*10));
	GPIOA->PUPDR |=  (1U << (2*10));

	GPIOA->AFR[1] &= ~((0xFU << 4) | (0xFU << 8));
	GPIOA->AFR[1] |=  ((7U   << 4) | (7U   << 8));

	RCC->APB2ENR |= RCC_APB2ENR_USART1EN;

	USART1->BRR = ((APB2CLK + UART1_BR/2)/UART1_BR);

	USART1->CR1 |= USART_CR1_RE | USART_CR1_TE;
	USART1->CR1 |= USART_CR1_IDLEIE;
	USART1->CR3 |= USART_CR3_DMAR;
	USART1->CR1 |= USART_CR1_UE;

	RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;

	// RX: DMA2 Stream2 channel 4, circular
	    DMA2_Stream2->CR &= ~DMA_SxCR_EN;
	    while(DMA2_Stream2->CR & DMA_SxCR_EN);

	    DMA2_Stream2->PAR = (uint32_t)&USART1->DR;
	    DMA2_Stream2->M0AR = (uint32_t)dma_rx_buf;
	    DMA2_Stream2->NDTR = DMA_BUF_SIZE;

	    DMA2_Stream2->CR &= ~DMA_SxCR_CHSEL;
	    DMA2_Stream2->CR |= (4U << DMA_SxCR_CHSEL_Pos);

	    DMA2_Stream2->CR |= DMA_SxCR_CIRC;
	    DMA2_Stream2->CR |= DMA_SxCR_HTIE | DMA_SxCR_TCIE;
	    DMA2_Stream2->CR |= DMA_SxCR_MINC;
	    DMA2_Stream2->CR &= ~DMA_SxCR_DIR;

	    DMA2_Stream2->CR |= DMA_SxCR_EN;


	// TX: DMA2 Stream7 channel 4. Address and length are set per frame
	// in uart1_tx_dma_start().
	    USART1->CR3 |= USART_CR3_DMAT;

	    DMA2_Stream7->CR &= ~DMA_SxCR_EN;
	    while (DMA2_Stream7->CR & DMA_SxCR_EN);

	    DMA2_Stream7->PAR = (uint32_t)&USART1->DR;
	    DMA2_Stream7->CR  = (4U << DMA_SxCR_CHSEL_Pos)   // channel 4
	                      | DMA_SxCR_DIR_0               // memory -> peripheral
	                      | DMA_SxCR_MINC
	                      | DMA_SxCR_TCIE                // frame finished
	                      | DMA_SxCR_TEIE;               // transfer error

	    NVIC_SetPriority(DMA2_Stream7_IRQn, 6);

	 // USART1 and its RX DMA at the same priority: they share
	 // last_dma_pos. Enabled later by uart1_irq_enable().
	 NVIC_SetPriority(USART1_IRQn, 5);
	 NVIC_SetPriority(DMA2_Stream2_IRQn, 5);

}



/*
 * Copy what the DMA wrote since last time into the ring and wake
 * comms_task. The position (SIZE - NDTR) can't tell "no data" from "a
 * full 256-byte lap", so as well as IDLE this runs on the half and full
 * transfer events, 128 bytes apart. The bench reproduces the lost lap
 * with the HT/TC events ignored ('v' key).
 */
static void uart1_dma_drain(bool from_dma_event)
{
    uint16_t current_pos = (uint16_t)(DMA_BUF_SIZE - DMA2_Stream2->NDTR);

    if (current_pos >= DMA_BUF_SIZE)
    {
        current_pos = 0;
    }

    uint16_t moved = 0;

    if (current_pos > last_dma_pos)
    {
        moved = (uint16_t)(current_pos - last_dma_pos);
        rx_write(&dma_rx_buf[last_dma_pos], moved);
    }
    else if (current_pos < last_dma_pos)
    {
        rx_write(&dma_rx_buf[last_dma_pos], (uint16_t)(DMA_BUF_SIZE - last_dma_pos));
        rx_write(&dma_rx_buf[0], current_pos);
        moved = (uint16_t)(DMA_BUF_SIZE - last_dma_pos + current_pos);
    }

    last_dma_pos = current_pos;

    bench_uart1_rx_bytes(moved, from_dma_event);

    if ((moved != 0U) && (commsTaskHandle != NULL))
    {
        BaseType_t xHigherPriorityTaskWasWoken = pdFALSE;

        xTaskNotifyFromISR(
            commsTaskHandle,
            COMMS_NOTIFY_RX,
            eSetBits,
            &xHigherPriorityTaskWasWoken
        );

        portYIELD_FROM_ISR(xHigherPriorityTaskWasWoken);
    }
}

void USART1_IRQHandler(void)
{
    if(USART1->SR & USART_SR_IDLE)
    {
        volatile uint32_t dummy;

        /* Clear IDLE flag: SR read then DR read */
        dummy = USART1->SR;
        dummy = USART1->DR;
        (void)dummy;

        uart1_dma_drain(false);
    }
}

void DMA2_Stream2_IRQHandler(void)
{
    const uint32_t mask = DMA_LISR_HTIF2 | DMA_LISR_TCIF2;

    if ((DMA2->LISR & mask) == 0U)
    {
        return;
    }

    DMA2->LIFCR = DMA_LIFCR_CHTIF2 | DMA_LIFCR_CTCIF2;

    /* BENCH_HIL 'v': behave as the firmware did before this handler
     * existed, to reproduce the wrap on demand. Always false otherwise. */
    if (bench_uart1_dma_events_ignored())
    {
        return;
    }

    uart1_dma_drain(true);
}


/*
 * Send a frame with DMA and return straight away; the transfer-complete
 * interrupt tells comms_task when it's gone. Returns false if a frame is
 * still going out. buf must not change until then. (Writing bytes by hand
 * cost 5.4 ms of busy-waiting per frame.)
 * UNTESTED on hardware.
 */
bool uart1_tx_dma_start(const uint8_t *buf, uint16_t len)
{
    if (DMA2_Stream7->CR & DMA_SxCR_EN)
    {
        return false;
    }

    // leftover flags from the last transfer stop the stream starting
    DMA2->HIFCR = DMA_HIFCR_CTCIF7 | DMA_HIFCR_CHTIF7 | DMA_HIFCR_CTEIF7
                | DMA_HIFCR_CDMEIF7 | DMA_HIFCR_CFEIF7;

    DMA2_Stream7->M0AR = (uint32_t)buf;
    DMA2_Stream7->NDTR = len;
    DMA2_Stream7->CR  |= DMA_SxCR_EN;

    return true;
}

void DMA2_Stream7_IRQHandler(void)
{
    uint32_t done = DMA2->HISR & (DMA_HISR_TCIF7 | DMA_HISR_TEIF7);

    if (done == 0U)
    {
        return;
    }

    DMA2->HIFCR = DMA_HIFCR_CTCIF7 | DMA_HIFCR_CTEIF7;

    // On a transfer error the frame is lost, but the stream has stopped
    // and the buffer is free either way.
    if (commsTaskHandle != NULL)
    {
        BaseType_t woken = pdFALSE;

        xTaskNotifyFromISR(commsTaskHandle, COMMS_NOTIFY_TX_DONE, eSetBits, &woken);
        portYIELD_FROM_ISR(woken);
    }
}

// Blocking write, used only by the bench injector. Waits for any DMA
// frame first so the two don't interleave.

void uart1_write_buf(uint8_t *buf, uint16_t len)
{
    while (DMA2_Stream7->CR & DMA_SxCR_EN)
    {
    }

    for(uint16_t i = 0; i < len; i++)
    {
        while(!(USART1->SR & USART_SR_TXE))
        {
        }

        USART1->DR = buf[i];
    }

    while (!(USART1->SR & USART_SR_TC)) { }
}

void uart1_write_byte(uint8_t b)
{
    while(!(USART1->SR & USART_SR_TXE))
    {
    }

    USART1->DR = b;
}

// Called by comms_task when it first runs, never from main (see uart3.c).
void uart1_irq_enable(void)
{
    NVIC_EnableIRQ(USART1_IRQn);
    NVIC_EnableIRQ(DMA2_Stream2_IRQn);
    NVIC_EnableIRQ(DMA2_Stream7_IRQn);
}
