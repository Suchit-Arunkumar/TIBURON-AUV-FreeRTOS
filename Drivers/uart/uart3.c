	#include "uart3.h"
	#include "stm32f446xx.h"

	#define APB1CLK 45000000U
	#define UART3_BR 115200U

	static uint8_t dma3_rx_buf[UART3_DMA_BUF_SIZE];

	static uint16_t last_dma_pos = 0;

	static uint8_t uart3_rx_buf[UART3_DMA_BUF_SIZE];

	static volatile uint16_t uart3_rx_head = 0;
	static volatile uint16_t uart3_rx_tail = 0;

	TaskHandle_t vn200TaskHandle = NULL;


	//===========================================================================================================================
	static void uart3_rx_store(const uint8_t *data, uint16_t length)
	{
		for(uint16_t i = 0; i < length; i++)
		{
			uint16_t next_head =
				(uint16_t)((uart3_rx_head + 1U) % UART3_DMA_BUF_SIZE);

			/*
			 * Buffer full.
			 * Drop the incoming byte rather than overwrite unread data.
			 */
			if(next_head == uart3_rx_tail)
			{
				return;
			}

			uart3_rx_buf[uart3_rx_head] = data[i];

			uart3_rx_head = next_head;
		}
	}


	//===========================================================================================================================
	void uart3_init(void)
	{
		/*
		 * 1. Enable GPIOC clock
		 *
		 * VN-200 moved from PB10/PB11 to PC10/PC11: PB10/PB11 are now
		 * free, and PC10/PC11 were released when UART4 moved to PA0/PA1.
		 */
		RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;


		/*
		 * 2. Configure PC10 as alternate-function mode
		 *    PC10 = USART3_TX (AF7)
		 */
		GPIOC->MODER &= ~(3U << (2U * 10U));
		GPIOC->MODER |=  (2U << (2U * 10U));


		/*
		 * 3. Configure PC11 as alternate-function mode
		 *    PC11 = USART3_RX (AF7)
		 */
		GPIOC->MODER &= ~(3U << (2U * 11U));
		GPIOC->MODER |=  (2U << (2U * 11U));


		/*
		 * 4. Set PC10 alternate function to AF7
		 */
		GPIOC->AFR[1] &= ~(0xFU << 8U);
		GPIOC->AFR[1] |=  (7U << 8U);


		/*
		 * 5. Set PC11 alternate function to AF7
		 */
		GPIOC->AFR[1] &= ~(0xFU << 12U);
		GPIOC->AFR[1] |=  (7U << 12U);


		/*
		 * 6. Enable USART3 clock
		 */
		RCC->APB1ENR |= RCC_APB1ENR_USART3EN;


		/*
		 * 7. Configure baud rate
		 *    APB1 = 45 MHz
		 *    Baud = 115200
		 */
		USART3->BRR =
			((APB1CLK + UART3_BR / 2U) / UART3_BR);


		/*
		 * 8. Enable receiver
		 */
		USART3->CR1 |= USART_CR1_RE;


		/*
		 * 9. Enable IDLE line interrupt
		 */
		USART3->CR1 |= USART_CR1_IDLEIE;


		/*
		 * 10. Enable DMA RX request
		 */
		USART3->CR3 |= USART_CR3_DMAR;


		/*
		 * 11. Enable USART3
		 */
		USART3->CR1 |= USART_CR1_UE;


		/*
		 * 12. Enable DMA1 clock
		 */
		RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;


		/*
		 * 13. Disable DMA1 Stream1 before configuration
		 */
		DMA1_Stream1->CR &= ~DMA_SxCR_EN;

		while(DMA1_Stream1->CR & DMA_SxCR_EN)
		{
		}


		/*
		 * 14. Clear DMA configuration
		 */
		DMA1_Stream1->CR = 0;


		/*
		 * 15. Peripheral address
		 *
		 * DMA reads bytes from USART3->DR
		 */
		DMA1_Stream1->PAR =
			(uint32_t)&USART3->DR;


		/*
		 * 16. Memory address
		 *
		 * DMA writes bytes into dma3_rx_buf
		 */
		DMA1_Stream1->M0AR =
			(uint32_t)dma3_rx_buf;


		/*
		 * 17. Number of bytes in DMA buffer
		 */
		DMA1_Stream1->NDTR =
			UART3_DMA_BUF_SIZE;


		/*
		 * 18. Select Channel 4
		 */
		DMA1_Stream1->CR &= ~DMA_SxCR_CHSEL;

		DMA1_Stream1->CR |=
			(4U << DMA_SxCR_CHSEL_Pos);


		/*
		 * 19. Peripheral-to-memory
		 */
		DMA1_Stream1->CR &= ~DMA_SxCR_DIR;


		/*
		 * 20. Enable memory increment
		 */
		DMA1_Stream1->CR |= DMA_SxCR_MINC;


		/*
		 * 21. Enable circular mode
		 */
		DMA1_Stream1->CR |= DMA_SxCR_CIRC;


		/*
		 * 22. Configure byte transfers
		 *
		 * PSIZE = 00 → 8-bit peripheral
		 * MSIZE = 00 → 8-bit memory
		 */
		DMA1_Stream1->CR &= ~DMA_SxCR_PSIZE;
		DMA1_Stream1->CR &= ~DMA_SxCR_MSIZE;


		/*
		 * 23. Enable DMA stream
		 */
		DMA1_Stream1->CR |= DMA_SxCR_EN;


		/*
		 * 24. Enable USART3 interrupt
		 */
		NVIC_SetPriority(USART3_IRQn, 6);
		/* NVIC_EnableIRQ deferred to uart3_irq_enable() - see below. */
	}


	//===========================================================================================================================
	void USART3_IRQHandler(void)
	{
		if(USART3->SR & USART_SR_IDLE)
		{
			volatile uint32_t dummy;

			/*
			 * Clear USART IDLE flag
			 *
			 * Read SR followed by DR.
			 */
			dummy = USART3->SR;
			dummy = USART3->DR;

			/*
			 * Current DMA write position
			 */
			uint16_t current_pos =
				UART3_DMA_BUF_SIZE -
				DMA1_Stream1->NDTR;


			/*
			 * DMA has not wrapped around.
			 */
			if(current_pos > last_dma_pos)
			{
				uart3_rx_store(
					&dma3_rx_buf[last_dma_pos],
					current_pos - last_dma_pos
				);
			}


			/*
			 * DMA wrapped around.
			 */
			else if(current_pos < last_dma_pos)
			{
				uart3_rx_store(
					&dma3_rx_buf[last_dma_pos],
					UART3_DMA_BUF_SIZE - last_dma_pos
				);

				uart3_rx_store(
					&dma3_rx_buf[0],
					current_pos
				);
			}


			/*
			 * Remember current DMA position.
			 */
			last_dma_pos = current_pos;

			if (vn200TaskHandle != NULL)
			{
			    BaseType_t xHigherPriorityTaskWasWoken = pdFALSE;

			    xTaskNotifyFromISR(
			        vn200TaskHandle,
			        (1UL << 0),
			        eSetBits,
			        &xHigherPriorityTaskWasWoken
			    );

			    portYIELD_FROM_ISR(xHigherPriorityTaskWasWoken);
			}

			(void)dummy;
		}
	}


	//===========================================================================================================================
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
						   UART3_DMA_BUF_SIZE);
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
void uart3_irq_enable(void)
{
    NVIC_EnableIRQ(USART3_IRQn);
}
