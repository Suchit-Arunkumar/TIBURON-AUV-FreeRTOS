#include <stdio.h>
#include "stm32f446xx.h"
#include "i2c.h"

#define APB1CLK_MHZ  45U

/*
 * Wait budget for every flag poll.
 *
 * One I2C byte at 100 kHz is ~90 us, and the slowest single wait here is
 * an address phase at ~90 us. At 180 MHz a poll iteration is a load, a
 * test and a branch over an APB1 bridge - call it ~20 core cycles, so
 * ~0.11 us. 20000 iterations is therefore roughly 2.2 ms: about 24 byte
 * times, generous for a healthy bus and immediate on a dead one.
 *
 * Deliberately a loop count and not a tick count, unlike the SD busy
 * wait: i2c1_init() and bar30_init() both run before the scheduler
 * exists, so there is no tick to read. The consequence of the estimate
 * being off is a timeout somewhere between 1 ms and 5 ms, which changes
 * nothing about the outcome - the device is either there or it is not.
 */
#define I2C_WAIT_ITERATIONS   20000UL

const char *i2c_status_str(I2C_Status s)
{
    switch (s)
    {
        case I2C_OK:            return "OK";
        case I2C_ERR_START:     return "START timeout (bus held?)";
        case I2C_ERR_ADDR_NACK: return "address NACK (no device)";
        case I2C_ERR_TXE:       return "TXE timeout";
        case I2C_ERR_RXNE:      return "RXNE timeout";
        case I2C_ERR_BUSY:      return "bus busy";
        default:                return "unknown";
    }
}

/* Returns 1 if the flag appeared within budget, 0 on timeout. Also aborts
 * early on AF (acknowledge failure), which is how a missing device
 * announces itself rather than by simply never responding. */
static uint32_t wait_flag(volatile uint32_t *reg, uint32_t flag)
{
    for (uint32_t i = 0; i < I2C_WAIT_ITERATIONS; i++)
    {
        if (*reg & flag)
        {
            return 1UL;
        }

        if (I2C1->SR1 & I2C_SR1_AF)
        {
            /* Clear AF so the next transfer starts clean. */
            I2C1->SR1 &= ~I2C_SR1_AF;
            return 0UL;
        }
    }

    return 0UL;
}

void i2c1_init(void)
{
    // 1. enable GPIOB clock in RCC AHB1ENR
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;

    // 2. configure PB8 as alternate function mode (MODER = 10)
	GPIOB->MODER &= ~(3 << (2*8));
	GPIOB->MODER |=  (2 << (2*8));

    // 3. configure PB9 as alternate function mode (MODER = 10)
	GPIOB->MODER &= ~(3 << (2*9));
	GPIOB->MODER |=  (2 << (2*9));

    // 4. set PB8 and PB9 to open-drain in OTYPER
	GPIOB->OTYPER |= GPIO_OTYPER_OT8;
	GPIOB->OTYPER |= GPIO_OTYPER_OT9;

    // 5. set PB8 AF4 in AFRH (AFR[1])
	// 6. set PB9 AF4 in AFRH (AFR[1])
	GPIOB->AFR[1] &= ~((0xFU << 0) | (0xFU << 4));
	GPIOB->AFR[1] |= (4 << 0);
	GPIOB->AFR[1] |= (4 << 4);

    // 7. enable I2C1 clock in RCC APB1ENR
	RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;

    // 8. reset I2C1 via CR1 SWRST bit, then clear it
	I2C1->CR1 |= I2C_CR1_SWRST;
	I2C1->CR1 &= ~I2C_CR1_SWRST;

    // 9. set CR2 with APB1 frequency in MHz (45)
	I2C1->CR2 = APB1CLK_MHZ;

    // 10. set CCR = 225 for 100kHz standard mode
	I2C1->CCR = 225 ;

    // 11. set TRISE = 46
	I2C1->TRISE = 46;

    // 12. enable I2C1 via PE bit in CR1
	I2C1->CR1 |= I2C_CR1_PE;
}

void i2c1_bus_recover(void)
{
    /*
     * A slave that was reset mid-transfer can hold SDA low and leave the
     * bus permanently busy. The standard escape is to drive SCL manually
     * for nine cycles so the slave finishes clocking out its byte and
     * releases SDA, then issue a STOP.
     */
    I2C1->CR1 &= ~I2C_CR1_PE;

    /* PB8/PB9 to open-drain GPIO outputs. */
    GPIOB->MODER &= ~((3U << (2 * 8)) | (3U << (2 * 9)));
    GPIOB->MODER |=  ((1U << (2 * 8)) | (1U << (2 * 9)));
    GPIOB->OTYPER |= GPIO_OTYPER_OT8 | GPIO_OTYPER_OT9;

    GPIOB->BSRR = (1U << 8) | (1U << 9);   /* both released high */

    for (uint8_t i = 0; i < 9U; i++)
    {
        GPIOB->BSRR = (1U << (8 + 16));    /* SCL low  */
        for (volatile uint32_t d = 0; d < 500; d++) { }
        GPIOB->BSRR = (1U << 8);           /* SCL high */
        for (volatile uint32_t d = 0; d < 500; d++) { }
    }

    /* STOP: SDA low->high while SCL is high. */
    GPIOB->BSRR = (1U << (9 + 16));
    for (volatile uint32_t d = 0; d < 500; d++) { }
    GPIOB->BSRR = (1U << 9);

    /* Back to the peripheral. */
    i2c1_init();
}

I2C_Status i2c_write(uint8_t addr, uint8_t *data, uint8_t len)
{
    // 1. generate START condition
	I2C1->CR1 |= I2C_CR1_START;

    // 2. wait for SB flag in SR1
	if (!wait_flag(&I2C1->SR1, I2C_SR1_SB))
	{
		I2C1->CR1 |= I2C_CR1_STOP;
		return I2C_ERR_START;
	}

	// 3. send slave address with write bit (addr << 1 | 0)
	I2C1->DR = (addr << 1) | 0;

	// 4. wait for ADDR flag in SR1
	if (!wait_flag(&I2C1->SR1, I2C_SR1_ADDR))
	{
		I2C1->CR1 |= I2C_CR1_STOP;
		return I2C_ERR_ADDR_NACK;
	}

	// 5. clear ADDR flag by reading SR1 then SR2
	(void)I2C1->SR1;
	(void)I2C1->SR2;

	// 6. loop len times:
	//    a. wait for TXE flag
	//    b. write byte to DR

	for(int i = 0; i < len; i++){

		if (!wait_flag(&I2C1->SR1, I2C_SR1_TXE))
		{
			I2C1->CR1 |= I2C_CR1_STOP;
			return I2C_ERR_TXE;
		}
		I2C1->DR = data[i] ;

	}

	// 7. STOP
	I2C1->CR1 |= I2C_CR1_STOP;

	return I2C_OK;
}

I2C_Status i2c_read(uint8_t addr, uint8_t *buf, uint8_t len)
{
    // 1. generate START condition
	I2C1->CR1 |= I2C_CR1_ACK;
	I2C1->CR1 |= I2C_CR1_START;

    // 2. wait for SB flag in SR1
	if (!wait_flag(&I2C1->SR1, I2C_SR1_SB))
	{
		I2C1->CR1 |= I2C_CR1_STOP;
		return I2C_ERR_START;
	}

    // 3. send slave address with read bit (addr << 1 | 1)
    //    also enable ACK in CR1 before sending address
	I2C1->DR = (addr << 1) | 1;

    // 4. wait for ADDR flag in SR1
	if (!wait_flag(&I2C1->SR1, I2C_SR1_ADDR))
	{
		I2C1->CR1 |= I2C_CR1_STOP;
		return I2C_ERR_ADDR_NACK;
	}

    // 5. clear ADDR flag by reading SR1 then SR2
	(void)I2C1->SR1;
	(void)I2C1->SR2;

    // 6. loop len times:
    //    a. if this is the last byte, disable ACK and generate STOP
    //    b. wait for RXNE flag in SR1
    //    c. read byte from DR into buf[i]
	for(int i = 0; i < len; i++){
	    if(i == (len - 1)){
	        I2C1->CR1 &= ~I2C_CR1_ACK;
	        I2C1->CR1 |= I2C_CR1_STOP;
	    }
	    if (!wait_flag(&I2C1->SR1, I2C_SR1_RXNE))
	    {
	        I2C1->CR1 |= I2C_CR1_STOP;
	        return I2C_ERR_RXNE;
	    }
	    buf[i] = I2C1->DR;
	}

	return I2C_OK;
}
