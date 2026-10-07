// Receive ring for the Pi link. rx_write() runs in the USART1/DMA
// interrupt; rx_avail/peek/eat run in comms_task. Shared variables are
// volatile so the parser loop re-reads rx_count every time.

#include "ring_buffer.h"
#include "stm32f4xx.h"


static uint8_t  rx_buf[RX_BUF_SIZE];

static volatile uint16_t rx_head = 0;     // written by the interrupt
static volatile uint16_t rx_tail = 0;     // written by the task
static volatile uint16_t rx_count = 0;    // both
static volatile uint32_t rx_dropped = 0;  // bytes lost to a full ring


// No lock needed here: the only other writer of rx_count is rx_eat(),
// in a task this interrupt can preempt but not the other way round.
void rx_write(uint8_t *data, uint16_t len)
{
	for (uint16_t i = 0; i < len; i++)
	{
	    // full: drop the rest rather than overwrite unread bytes
	    if (rx_count >= RX_BUF_SIZE)
	    {
	        rx_dropped += (uint32_t)(len - i);
	        return;
	    }

	    rx_buf[rx_head] = data[i];

	    rx_head++;
	    rx_head %= RX_BUF_SIZE;

	    rx_count++;
	}
}

uint16_t rx_avail(void)
{
	return(rx_count);
}

uint8_t rx_peek(uint16_t offset)
{
	return rx_buf[(rx_tail + offset) % RX_BUF_SIZE];
}

void rx_eat(uint16_t len)
{
	rx_tail += len;
	rx_tail = rx_tail % RX_BUF_SIZE;

	// rx_count is shared with the interrupt and this is read-modify-write,
	// so mask interrupts around it (restoring the previous state).
	uint32_t primask = __get_PRIMASK();
	__disable_irq();

	rx_count -= len;

	__set_PRIMASK(primask);
}

uint32_t rx_dropped_count(void)
{
	return rx_dropped;
}
