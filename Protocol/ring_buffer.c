#include "ring_buffer.h"
#include "stm32f4xx.h"


// internal buffer
static uint8_t  rx_buf[RX_BUF_SIZE];

/*
 * The genuine ISR-to-task case, and the one the rest of the tree already
 * got right: uart3.c and uart4.c both declare their head/tail volatile,
 * and this file did not.
 *
 * rx_write() runs in USART1_IRQHandler; rx_avail(), rx_peek() and
 * rx_eat() run in comms_task. rx_count in particular is read in
 * packet_parse_cmd's framing loop, which is exactly the shape a compiler
 * is entitled to hoist.
 */

// head — where new data is written (ISR)
static volatile uint16_t rx_head = 0;

// tail — where data is read from (task)
static volatile uint16_t rx_tail = 0;

// number of bytes available (ISR increments, task decrements)
static volatile uint16_t rx_count = 0;


void rx_write(uint8_t *data, uint16_t len)
{
    	// 1. loop through len bytes

        // 2. write data[i] into rx_buf at rx_head position

        // 3. increment rx_head, wrap around using modulo RX_BUF_SIZE

        // 4. increment rx_count

	/*
	 * No critical section here. This runs in USART1_IRQHandler, and the
	 * only other writer of rx_count is rx_eat() in task context, which
	 * this ISR preempts. Nothing at a higher interrupt priority touches
	 * the ring.
	 *
	 * The previous unconditional __enable_irq() was a latent bug: called
	 * from an ISR it would re-enable interrupts regardless of the
	 * caller's state, breaking any critical section in effect further up.
	 */
	for (uint16_t i = 0; i < len; i++)
	{
	    rx_buf[rx_head] = data[i];

	    rx_head++;
	    rx_head %= RX_BUF_SIZE;

	    rx_count++;
	}
}

uint16_t rx_avail(void)
{
    // 1. return rx_count
	return(rx_count);
}

uint8_t rx_peek(uint16_t offset)
{
    // 1. return byte at position (rx_tail + offset) % RX_BUF_SIZE
	return rx_buf[(rx_tail + offset) % RX_BUF_SIZE];
}

void rx_eat(uint16_t len)
{
    // 1. move rx_tail forward by len, wrap using modulo RX_BUF_SIZE

    // 2. subtract len from rx_count


	/* rx_tail is task-private - the ISR never touches it. */
	rx_tail += len;
	rx_tail = rx_tail % RX_BUF_SIZE;

	/*
	 * rx_count is shared with the ISR and this is a read-modify-write, so
	 * it does need protection. Save and restore PRIMASK rather than
	 * unconditionally re-enabling, so this is correct even if a caller
	 * already had interrupts masked.
	 */
	uint32_t primask = __get_PRIMASK();
	__disable_irq();

	rx_count -= len;

	__set_PRIMASK(primask);
}
