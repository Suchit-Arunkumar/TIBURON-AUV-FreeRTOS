#include "uart.h"
#include "stm32f446xx.h"
#include "console.h"
#include <stdio.h>

#define APB1CLK 45000000U
#define UART_BR 115200U

void uart2_init(void)
{
    // 1. enable GPIOA clock in RCC AHB1ENR
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;

    // 2. configure PA2 as alternate function mode (MODER = 10)
	// PA2 alternate function mode
	GPIOA->MODER &= ~(3U << (2 * 2));  // clear bits 5:4
	GPIOA->MODER |=  (2U << (2 * 2));  // set 10 = AF mode

    // 3. configure PA3 as alternate function mode (MODER = 10)
	// PA3 alternate function mode
	GPIOA->MODER &= ~(3U << (2 * 3));  // clear bits 5:4
	GPIOA->MODER |=  (2U << (2 * 3));  // set 10 = AF mode

    // 4. set PA2 alternate function to AF7 (USART2 TX) in AFRL
	GPIOA->AFR[0] &= ~(0xF << 8);
	GPIOA->AFR[0] |= (7 << 8);

    // 5. set PA3 alternate function to AF7 (USART2 RX) in AFRL
	GPIOA->AFR[0] &= ~(0xF << 12);
	GPIOA->AFR[0] |= (7 << 12);

    // 6. enable USART2 clock in RCC APB1ENR
	RCC->APB1ENR |= RCC_APB1ENR_USART2EN;

    // 7. set baud rate in USART2 BRR (APB1 = 45MHz, target = 115200)
	USART2->BRR = ((APB1CLK + UART_BR/2)/UART_BR);

    // 8. enable transmitter (TE bit) in USART2 CR1
	USART2->CR1 |= USART_CR1_TE;

    /*
     * 9. Enable the receiver and its interrupt.
     *
     * P9: the stack high-water report is on demand rather than one-shot,
     * so a keypress on the VCP has to reach the firmware. RX was
     * configured on PA3 all along but never enabled.
     */
	USART2->CR1 |= USART_CR1_RE;
	USART2->CR1 |= USART_CR1_RXNEIE;

    /*
     * 10. NVIC priority 6.
     *
     * Numerically above configMAX_SYSCALL_INTERRUPT_PRIORITY (5, raw
     * 0x50), so this ISR is masked inside kernel critical sections and
     * may legally call the FromISR API. It does not call it today - it
     * only writes one volatile byte - but the priority has to be right
     * regardless, or the kernel assert would fire the moment it did.
     */
	NVIC_SetPriority(USART2_IRQn, 6);
	NVIC_EnableIRQ(USART2_IRQn);

    // 11. enable USART2 (UE bit) in USART2 CR1
	USART2->CR1 |= USART_CR1_UE;
}


//===========================================================================================================================
void USART2_IRQHandler(void)
{
	if (USART2->SR & USART_SR_RXNE)
	{
		/* Reading DR clears RXNE. */
		uint8_t ch = (uint8_t)(USART2->DR & 0xFFU);

		/*
		 * Hand the byte to the console command latch. Deliberately does
		 * not touch any FreeRTOS API: the stdio owner already wakes every
		 * 50 ms, so a single volatile byte is enough and this ISR stays
		 * free of scheduler interaction entirely.
		 */
		console_rx_isr_char((char)ch);
	}
}


void uart2_write_byte(uint8_t b)
{
    // 1. wait until TXE flag is set in USART2 SR (transmit register empty)
	while(!(USART2->SR & USART_SR_TXE)){

	}
    // 2. write byte to USART2 DR
	USART2->DR = b;
}

void uart2_write_buf(uint8_t *buf, uint16_t len)
{
    //loop through each byte in buf for len times
    //call uart2_write_byte for each byte

	for(int i = 0; i < len; i++)
		{
			uart2_write_byte(buf[i]);
		}
}

void uart2_write_str(const char *s)
{
    // 1. loop while current character is not null terminator '\0'

    // 2. call uart2_write_byte for current character

    // 3. move to next character

	while(*s != '\0'){
		uart2_write_byte(*s);
		s++;
	}
}

int _write(int fd, char *buf, int len)
{
    // 1. call uart2_write_buf passing buf and len
	uart2_write_buf((uint8_t *)buf,len);
    // 2. return len
	return(len);
}


