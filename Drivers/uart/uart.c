// USART2: debug console on the ST-LINK virtual COM port, 115200 baud.
// Polled TX; RX interrupt stores one key for the console commands.

#include "uart.h"
#include "stm32f446xx.h"
#include "console.h"
#include <stdio.h>

#define APB1CLK 45000000U
#define UART_BR 115200U

void uart2_init(void)
{
	// PA2 TX, PA3 RX, AF7
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;

	GPIOA->MODER &= ~(3U << (2 * 2));
	GPIOA->MODER |=  (2U << (2 * 2));

	GPIOA->MODER &= ~(3U << (2 * 3));
	GPIOA->MODER |=  (2U << (2 * 3));

	GPIOA->AFR[0] &= ~(0xF << 8);
	GPIOA->AFR[0] |= (7 << 8);

	GPIOA->AFR[0] &= ~(0xF << 12);
	GPIOA->AFR[0] |= (7 << 12);

	RCC->APB1ENR |= RCC_APB1ENR_USART2EN;

	USART2->BRR = ((APB1CLK + UART_BR/2)/UART_BR);

	USART2->CR1 |= USART_CR1_TE;

	// RX interrupt for the single-key console commands
	USART2->CR1 |= USART_CR1_RE;
	USART2->CR1 |= USART_CR1_RXNEIE;

	// 6 is inside the FreeRTOS-safe range, though this handler makes no
	// kernel calls. Enabled here since it never wakes a task.
	NVIC_SetPriority(USART2_IRQn, 6);
	NVIC_EnableIRQ(USART2_IRQn);

	USART2->CR1 |= USART_CR1_UE;
}


void USART2_IRQHandler(void)
{
	if (USART2->SR & USART_SR_RXNE)
	{
		// reading DR clears RXNE
		uint8_t ch = (uint8_t)(USART2->DR & 0xFFU);

		// just store it; the console task polls every 50 ms
		console_rx_isr_char((char)ch);
	}
}


void uart2_write_byte(uint8_t b)
{
	while(!(USART2->SR & USART_SR_TXE)){

	}
	USART2->DR = b;
}

void uart2_write_buf(uint8_t *buf, uint16_t len)
{
	for(int i = 0; i < len; i++)
		{
			uart2_write_byte(buf[i]);
		}
}

void uart2_write_str(const char *s)
{
	while(*s != '\0'){
		uart2_write_byte(*s);
		s++;
	}
}

// printf goes here (newlib); only used before the scheduler starts
int _write(int fd, char *buf, int len)
{
	uart2_write_buf((uint8_t *)buf,len);
	return(len);
}
