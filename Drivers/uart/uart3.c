// USART3: VN-200, 115200 baud, RX only. Circular DMA into dma3_rx_buf,
// moved into a software ring by the interrupts below, read by imu_task.

#include "uart3.h"
#include "stm32f446xx.h"

#define APB1CLK 45000000U
#define UART3_BR 115200U

static uint8_t dma3_rx_buf[UART3_DMA_BUF_SIZE];

static uint16_t last_dma_pos = 0;

// Bigger than the DMA buffer: opening the BNO085 can hold imu_task for
// ~0.5 s, and this holds about a second of VN-200 data.
static uint8_t uart3_rx_buf[UART3_RX_RING_SIZE];

static volatile uint16_t uart3_rx_head = 0;
static volatile uint16_t uart3_rx_tail = 0;

static volatile uint32_t uart3_rx_dropped_count = 0;



static void uart3_rx_store(const uint8_t *data, uint16_t length)
{
	for(uint16_t i = 0; i < length; i++)
	{
		uint16_t next_head =
			(uint16_t)((uart3_rx_head + 1U) % UART3_RX_RING_SIZE);

		// full: drop new bytes rather than overwrite unread ones
		if(next_head == uart3_rx_tail)
		{
			uart3_rx_dropped_count += (uint32_t)(length - i);
			return;
		}

		uart3_rx_buf[uart3_rx_head] = data[i];

		uart3_rx_head = next_head;
	}
}


void uart3_init(void)
{
	// PC10 TX, PC11 RX, AF7
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;

	GPIOC->MODER &= ~(3U << (2U * 10U));
	GPIOC->MODER |=  (2U << (2U * 10U));

	GPIOC->MODER &= ~(3U << (2U * 11U));
	GPIOC->MODER |=  (2U << (2U * 11U));

	// pull-up so an unconnected RX idles high instead of floating
	GPIOC->PUPDR &= ~(3U << (2U * 11U));
	GPIOC->PUPDR |=  (1U << (2U * 11U));

	GPIOC->AFR[1] &= ~(0xFU << 8U);
	GPIOC->AFR[1] |=  (7U << 8U);

	GPIOC->AFR[1] &= ~(0xFU << 12U);
	GPIOC->AFR[1] |=  (7U << 12U);

	RCC->APB1ENR |= RCC_APB1ENR_USART3EN;

	USART3->BRR =
		((APB1CLK + UART3_BR / 2U) / UART3_BR);

	USART3->CR1 |= USART_CR1_RE;
	USART3->CR1 |= USART_CR1_IDLEIE;
	USART3->CR3 |= USART_CR3_DMAR;
	USART3->CR1 |= USART_CR1_UE;

	// DMA1 Stream1 channel 4 = USART3_RX, circular, bytes
	RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;

	DMA1_Stream1->CR &= ~DMA_SxCR_EN;

	while(DMA1_Stream1->CR & DMA_SxCR_EN)
	{
	}

	DMA1_Stream1->CR = 0;

	DMA1_Stream1->PAR =
		(uint32_t)&USART3->DR;

	DMA1_Stream1->M0AR =
		(uint32_t)dma3_rx_buf;

	DMA1_Stream1->NDTR =
		UART3_DMA_BUF_SIZE;

	DMA1_Stream1->CR &= ~DMA_SxCR_CHSEL;

	DMA1_Stream1->CR |=
		(4U << DMA_SxCR_CHSEL_Pos);

	DMA1_Stream1->CR &= ~DMA_SxCR_DIR;

	DMA1_Stream1->CR |= DMA_SxCR_MINC;

	DMA1_Stream1->CR |= DMA_SxCR_CIRC;

	DMA1_Stream1->CR &= ~DMA_SxCR_PSIZE;
	DMA1_Stream1->CR &= ~DMA_SxCR_MSIZE;

	DMA1_Stream1->CR |= DMA_SxCR_HTIE | DMA_SxCR_TCIE;

	DMA1_Stream1->CR |= DMA_SxCR_EN;

	// Same priority for both: they share last_dma_pos, and equal
	// priorities can't interrupt each other. Enabled by uart3_irq_enable().
	NVIC_SetPriority(USART3_IRQn, 6);
	NVIC_SetPriority(DMA1_Stream1_IRQn, 6);
}


/*
 * Copy what the DMA wrote since last time into the ring and wake the task.
 * Called on IDLE and on the half/full-transfer events. IDLE alone misses a
 * burst of exactly one buffer length (the position comes back to the same
 * place), and a gap-free stream never raises IDLE at all.
 */
static void uart3_dma_drain(void)
{
    uint16_t current_pos =
        (uint16_t)(UART3_DMA_BUF_SIZE - DMA1_Stream1->NDTR);

    if (current_pos >= UART3_DMA_BUF_SIZE)
    {
        current_pos = 0U;
    }

    uint16_t moved = 0U;

    if (current_pos > last_dma_pos)
    {
        moved = (uint16_t)(current_pos - last_dma_pos);
        uart3_rx_store(&dma3_rx_buf[last_dma_pos], moved);
    }
    else if (current_pos < last_dma_pos)
    {
        uart3_rx_store(&dma3_rx_buf[last_dma_pos],
                     (uint16_t)(UART3_DMA_BUF_SIZE - last_dma_pos));
        uart3_rx_store(&dma3_rx_buf[0], current_pos);
        moved = (uint16_t)(UART3_DMA_BUF_SIZE - last_dma_pos + current_pos);
    }

    last_dma_pos = current_pos;

    if ((moved != 0U) && (imuTaskHandle != NULL))
    {
        BaseType_t woken = pdFALSE;

        xTaskNotifyFromISR(imuTaskHandle, (1UL << 0), eSetBits, &woken);
        portYIELD_FROM_ISR(woken);
    }
}


void USART3_IRQHandler(void)
{
    if (USART3->SR & USART_SR_IDLE)
    {
        volatile uint32_t dummy;

        // reading SR then DR clears IDLE
        dummy = USART3->SR;
        dummy = USART3->DR;
        (void)dummy;

        uart3_dma_drain();
    }
}


void DMA1_Stream1_IRQHandler(void)
{
    const uint32_t mask = DMA_LISR_HTIF1 | DMA_LISR_TCIF1;

    if ((DMA1->LISR & mask) == 0U)
    {
        return;
    }

    DMA1->LIFCR = DMA_LIFCR_CHTIF1 | DMA_LIFCR_CTCIF1;

    uart3_dma_drain();
}


uint32_t uart3_rx_dropped(void)
{
    return uart3_rx_dropped_count;
}


uint16_t uart3_read(uint8_t *out, uint16_t max_len)
{
	uint16_t count = 0;

	if(out == 0 || max_len == 0)
	{
		return 0;
	}

	while((uart3_rx_tail != uart3_rx_head) &&
		  (count < max_len))
	{
		out[count++] =
			uart3_rx_buf[uart3_rx_tail];

		uart3_rx_tail =
			(uint16_t)((uart3_rx_tail + 1U) %
					   UART3_RX_RING_SIZE);
	}

	return count;
}

// Called by imu_task when it first runs, never from main: an interrupt
// that woke a task before the scheduler started would write through a
// null stack pointer.
void uart3_irq_enable(void)
{
    NVIC_EnableIRQ(USART3_IRQn);
    NVIC_EnableIRQ(DMA1_Stream1_IRQn);
}
