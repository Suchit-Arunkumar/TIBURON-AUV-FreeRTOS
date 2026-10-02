#include "crc_hw.h"
#include "FreeRTOS.h"
#include "task.h"

void crc_init(void)
{
    // 1. Enable CRC clock in RCC->AHB1ENR
	RCC->AHB1ENR |= RCC_AHB1ENR_CRCEN;
}

uint32_t crc_compute(const uint8_t *data, uint32_t length)
{
	// One CRC unit, shared by control_task (log records) and comms_task
	// (packets). Without this, control preempting comms mid-calculation
	// resets the unit and corrupts the comms result.
	taskENTER_CRITICAL();

    // 1. Reset the CRC unit using CRC->CR bit 0
	CRC->CR |= (1 << 0);

    // 2. Feed data into CRC->DR word by word (4 bytes at a time)
    //    handle remaining bytes (if length not multiple of 4)

	for(uint32_t i = 0; i < (length & ~3U); i+= 4){

		uint32_t word = ((uint32_t)data[i]     << 24) |
		                ((uint32_t)data[i+1]   << 16) |
		                ((uint32_t)data[i+2]   << 8)  |
		                ((uint32_t)data[i+3]);

		CRC->DR = (word << 0);
	}

	uint32_t remaining = length % 4;

	if(remaining > 0)
	{
	    uint32_t word = 0;
	    for(uint32_t j = 0; j < remaining; j++)
	    {
	        word |= ((uint32_t)data[length - remaining + j] << (24 - j*8));
	    }
	    CRC->DR = word;
	}


    // 3. Return CRC->DR
	uint32_t crc = CRC->DR;

	taskEXIT_CRITICAL();

	return crc;
}
