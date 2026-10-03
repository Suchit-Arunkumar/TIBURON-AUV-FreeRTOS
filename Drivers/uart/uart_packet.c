#include "uart_packet.h"
#include "stm32f446xx.h"
#include "ring_buffer.h"
#include "comms_task.h"
#include "bench.h"
#include <stdbool.h>

#define APB2CLK 90000000U
#define UART1_BR 115200U

// DMA receive buffer — DMA writes directly into this
static uint8_t dma_rx_buf[DMA_BUF_SIZE];

// track last DMA position to detect new bytes
static uint16_t last_dma_pos = 0;

void uart1_init(void)
{
    // 1. enable GPIOA clock in RCC AHB1ENR
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;

    // 2. configure PA9 as alternate function mode (MODER = 10)
	GPIOA->MODER &= ~(3U << (2*9));
	GPIOA->MODER |=  (2U << (2*9));

    // 3. configure PA10 as alternate function mode (MODER = 10)
	GPIOA->MODER &= ~(3U << (2*10));
	GPIOA->MODER |=  (2U << (2*10));

	// PA10 pull-up: UART idles high, so an unconnected RX reads idle, not noise
	GPIOA->PUPDR &= ~(3U << (2*10));
	GPIOA->PUPDR |=  (1U << (2*10));

    // 4./5. PA9 -> AF7 (USART1 TX), PA10 -> AF7 (USART1 RX), AFRH.
    //    Cleared before set. A bare |= is correct only from reset, which is
    //    the read-modify-write bug class that broke PA6 in the bare-metal
    //    tree; every other driver here already clears first.
	GPIOA->AFR[1] &= ~((0xFU << 4) | (0xFU << 8));
	GPIOA->AFR[1] |=  ((7U   << 4) | (7U   << 8));

    // 6. enable USART1 clock in RCC APB2ENR
	RCC->APB2ENR |= RCC_APB2ENR_USART1EN;

    // 7. set baud rate in USART1 BRR (APB2 = 90MHz, target = 115200)
	USART1->BRR = ((APB2CLK + UART1_BR/2)/UART1_BR);

    // 8. enable receiver (RE) and transmitter (TE) in USART1 CR1
    //    TE is needed for telemetry - without it TXE never sets and
    //    uart1_write_buf() spins forever
	USART1->CR1 |= USART_CR1_RE | USART_CR1_TE;

    // 9. enable IDLE line interrupt (IDLEIE bit) in USART1 CR1
	USART1->CR1 |= USART_CR1_IDLEIE;

    // 10. enable DMA RX request (DMAR bit) in USART1 CR3
	USART1->CR3 |= USART_CR3_DMAR;

    // 11. enable USART1 (UE bit) in USART1 CR1
	USART1->CR1 |= USART_CR1_UE;

	// 12. enable DMA2 clock
	RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;

	// 13. configure DMA2 Stream2 for USART1 RX

	    // disable stream first before configuring
	    DMA2_Stream2->CR &= ~DMA_SxCR_EN;

	    // wait until stream is disabled
	    while(DMA2_Stream2->CR & DMA_SxCR_EN);

	    // set peripheral address — where DMA reads from
	    DMA2_Stream2->PAR = (uint32_t)&USART1->DR;

	    // set memory address — where DMA writes to
	    DMA2_Stream2->M0AR = (uint32_t)dma_rx_buf;

	    // set number of data items
	    DMA2_Stream2->NDTR = DMA_BUF_SIZE;

	    // set channel 4 (CHSEL bits 27:25 = 100)
	    DMA2_Stream2->CR &= ~DMA_SxCR_CHSEL;
	    DMA2_Stream2->CR |= (4U << DMA_SxCR_CHSEL_Pos);

	    // enable circular mode
	    DMA2_Stream2->CR |= DMA_SxCR_CIRC;

	    // half-transfer and transfer-complete interrupts: see
	    // uart1_dma_drain() for why IDLE alone is not enough
	    DMA2_Stream2->CR |= DMA_SxCR_HTIE | DMA_SxCR_TCIE;

	    // enable memory increment
	    DMA2_Stream2->CR |= DMA_SxCR_MINC;

	    // set transfer direction — peripheral to memory
	    DMA2_Stream2->CR &= ~DMA_SxCR_DIR;

	    // enable the stream
	    DMA2_Stream2->CR |= DMA_SxCR_EN;


	 // 14. enable USART1 interrupt in NVIC
	 NVIC_SetPriority(USART1_IRQn, 5);

	 // Same priority as USART1 on purpose: the two handlers share
	 // last_dma_pos, and equal priorities cannot preempt each other.
	 NVIC_SetPriority(DMA2_Stream2_IRQn, 5);
	 /* NVIC_EnableIRQ deferred to uart1_irq_enable() - see below. */

}



/*
 * Move everything the DMA has written since the last call into the ring
 * buffer, then wake comms_task.
 *
 * The position is SIZE - NDTR, so it can only tell "where the DMA is", not
 * "how far it has gone". If exactly DMA_BUF_SIZE bytes arrive between two
 * calls, the position comes back to where it was and the whole buffer is
 * indistinguishable from no data at all. With IDLE as the only trigger that
 * happens whenever a burst is exactly 256 bytes long with no gap; the bench
 * reproduces it on demand (tools/hil/hil_rtos.py, "DMA wrap").
 *
 * The half-transfer and transfer-complete interrupts fire at fixed points
 * in the buffer, 128 bytes apart, so between two calls the DMA can never
 * move a full lap. IDLE still catches the tail of a burst that ends between
 * those points. Both handlers run at priority 5, at the FreeRTOS ceiling,
 * and cannot preempt each other.
 */
static void uart1_dma_drain(bool from_dma_event)
{
    uint16_t current_pos = (uint16_t)(DMA_BUF_SIZE - DMA2_Stream2->NDTR);

    /* Defensive: NDTR is 1..SIZE while the stream runs, so this is 0..SIZE-1;
     * fold an out-of-range read to 0 rather than index past the buffer. */
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

    /*
     * B5: USART1 is enabled before comms_task is created, so a byte
     * arriving in that window would reach a NULL handle and trip
     * configASSERT inside the kernel. USART3 and UART4 already
     * guarded this; USART1 did not.
     */
    if ((moved != 0U) && (commsTaskHandle != NULL))
    {
        BaseType_t xHigherPriorityTaskWasWoken = pdFALSE;

        xTaskNotifyFromISR(
            commsTaskHandle,
            (1UL << 0),
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


void uart1_write_buf(uint8_t *buf, uint16_t len)
{
    for(uint16_t i = 0; i < len; i++)
    {
        while(!(USART1->SR & USART_SR_TXE))
        {
        }

        USART1->DR = buf[i];
    }
}

void uart1_write_byte(uint8_t b)
{
    while(!(USART1->SR & USART_SR_TXE))
    {
    }

    USART1->DR = b;
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
void uart1_irq_enable(void)
{
    NVIC_EnableIRQ(USART1_IRQn);
    NVIC_EnableIRQ(DMA2_Stream2_IRQn);
}
