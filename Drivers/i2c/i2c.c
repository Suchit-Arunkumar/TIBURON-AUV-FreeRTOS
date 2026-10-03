#include <stdio.h>
#include "stm32f446xx.h"
#include "i2c.h"
#include "timer_timebase.h"

#include "FreeRTOS.h"
#include "task.h"

#define APB1CLK_MHZ  45U

/*
 * Wait strategy: spin briefly, then yield.
 *
 * A single I2C phase at 100 kHz takes ~90 us. Spinning for that is
 * cheaper than a context switch, so the FAST PATH is a bounded spin of
 * I2C_SPIN_US - a healthy transfer never leaves it and pays nothing.
 *
 * Beyond that the bus is misbehaving, and the SLOW PATH yields 1 ms
 * between polls. This is the fix for the Phase 11 residual risk: every
 * wait used to spin for its whole budget, so one failed read cost ~6 x
 * 2.2 ms = ~13 ms of solid CPU at priority 4. A sensor that is cleanly
 * absent was survivable because bar30_task backs off to 1 s, but a
 * sensor that NACKs INTERMITTENTLY - far more likely on jumper wires to
 * a breakout - kept the 20 ms period and would have held priority 4 at
 * roughly 65% duty, starving logging, the SPI owner and the console.
 *
 * Timing comes from TIM2's free-running microsecond counter, so the
 * budget means what it says regardless of optimisation level, and the
 * unsigned subtraction is correct across TIM2's ~71.6-minute rollover.
 *
 * The scheduler-state split matters because i2c1_init() may run before
 * the scheduler; vTaskDelay() there would have no scheduler to return
 * from.
 */
#define I2C_SPIN_US       250UL    /* fast path: ~2.7 byte times   */
#define I2C_TIMEOUT_US   2500UL    /* total budget: ~28 byte times */

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

/* One poll. 1 = flag set, 0 = keep waiting, -1 = NACKed, give up now. */
static int32_t poll_once(volatile uint32_t *reg, uint32_t flag)
{
    if (*reg & flag)
    {
        return 1;
    }

    if (I2C1->SR1 & I2C_SR1_AF)
    {
        /* A missing device announces itself with AF rather than by
         * simply never responding. Clear it so the next transfer starts
         * clean. */
        I2C1->SR1 &= ~I2C_SR1_AF;
        return -1;
    }

    return 0;
}

/* Returns 1 if the flag appeared within budget, 0 on timeout or NACK. */
static uint32_t wait_flag(volatile uint32_t *reg, uint32_t flag)
{
    uint32_t start = micros();

    /* --- fast path: spin. A healthy bus finishes here. --- */
    do
    {
        int32_t r = poll_once(reg, flag);

        if (r == 1)  { return 1UL; }
        if (r == -1) { return 0UL; }

    } while ((micros() - start) < I2C_SPIN_US);

    /* --- slow path: the bus is not behaving. Stop hogging the CPU. --- */
    uint32_t can_yield =
        (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING) ? 1UL : 0UL;

    while ((micros() - start) < I2C_TIMEOUT_US)
    {
        int32_t r = poll_once(reg, flag);

        if (r == 1)  { return 1UL; }
        if (r == -1) { return 0UL; }

        if (can_yield)
        {
            vTaskDelay(pdMS_TO_TICKS(1));
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


/*
 * i2c_read_rm() - master receiver per RM0390 section 24.3.3 ("Closing the
 * communication"), polling method.
 *
 * Why i2c_read() above is not enough under an RTOS. It sets ACK before
 * START and clears it, with STOP, only when it reaches the last byte. The
 * peripheral decides ACK/NACK for a byte at that byte's ninth clock, so the
 * clear has to land before the last byte finishes shifting in: one byte time,
 * about 90 us at 100 kHz. bar30_task runs at priority 4, and comms_task
 * (priority 5) busy-waits ~5.4 ms on every telemetry frame. If the task is
 * preempted while waiting for byte N-2, byte N-1 is ACKed, the slave sends
 * an extra byte, and it is left in DR with RXNE set, where the next read
 * returns it as its first byte. Nothing reports an error.
 *
 * The sequences below never depend on software winning that race. The
 * peripheral stretches SCL while DR and the shift register are both full
 * (BTF), so the ACK/STOP decisions are made while the bus is frozen, and
 * the few instructions that must be back to back run with the kernel's
 * interrupts masked (BASEPRI; safe before and after the scheduler starts).
 */
static I2C_Status rm_fail(I2C_Status st)
{
    I2C1->CR1 |= I2C_CR1_STOP;
    I2C1->CR1 &= ~I2C_CR1_POS;
    return st;
}

I2C_Status i2c_read_rm(uint8_t addr, uint8_t *buf, uint8_t len)
{
    UBaseType_t m;

    if (len == 0U)
    {
        return I2C_OK;
    }

    I2C1->CR1 &= ~I2C_CR1_POS;
    I2C1->CR1 |=  I2C_CR1_ACK;
    if (len == 2U)
    {
        /* POS: the ACK bit now applies to the byte AFTER the one being
         * received, so clearing it right after ADDR NACKs byte 2. */
        I2C1->CR1 |= I2C_CR1_POS;
    }

    I2C1->CR1 |= I2C_CR1_START;
    if (!wait_flag(&I2C1->SR1, I2C_SR1_SB))
    {
        return rm_fail(I2C_ERR_START);
    }

    I2C1->DR = (uint8_t)((addr << 1) | 1U);
    if (!wait_flag(&I2C1->SR1, I2C_SR1_ADDR))
    {
        return rm_fail(I2C_ERR_ADDR_NACK);
    }

    if (len == 1U)
    {
        /* NACK must be armed before ADDR is cleared, and STOP set straight
         * after, or the single byte gets ACKed and a second one follows. */
        I2C1->CR1 &= ~I2C_CR1_ACK;
        m = taskENTER_CRITICAL_FROM_ISR();
        (void)I2C1->SR1;
        (void)I2C1->SR2;
        I2C1->CR1 |= I2C_CR1_STOP;
        taskEXIT_CRITICAL_FROM_ISR(m);

        if (!wait_flag(&I2C1->SR1, I2C_SR1_RXNE))
        {
            return rm_fail(I2C_ERR_RXNE);
        }
        buf[0] = (uint8_t)I2C1->DR;
        return I2C_OK;
    }

    if (len == 2U)
    {
        m = taskENTER_CRITICAL_FROM_ISR();
        (void)I2C1->SR1;
        (void)I2C1->SR2;
        I2C1->CR1 &= ~I2C_CR1_ACK;
        taskEXIT_CRITICAL_FROM_ISR(m);

        /* BTF: byte 1 in DR, byte 2 in the shift register, SCL held. */
        if (!wait_flag(&I2C1->SR1, I2C_SR1_BTF))
        {
            return rm_fail(I2C_ERR_RXNE);
        }
        m = taskENTER_CRITICAL_FROM_ISR();
        I2C1->CR1 |= I2C_CR1_STOP;
        buf[0] = (uint8_t)I2C1->DR;
        taskEXIT_CRITICAL_FROM_ISR(m);
        buf[1] = (uint8_t)I2C1->DR;

        I2C1->CR1 &= ~I2C_CR1_POS;
        return I2C_OK;
    }

    /* len > 2 */
    (void)I2C1->SR1;
    (void)I2C1->SR2;

    uint8_t i = 0;
    while ((uint8_t)(len - i) > 3U)
    {
        if (!wait_flag(&I2C1->SR1, I2C_SR1_RXNE))
        {
            return rm_fail(I2C_ERR_RXNE);
        }
        buf[i++] = (uint8_t)I2C1->DR;
    }

    /* Three left. BTF: byte N-2 in DR, N-1 in the shift register, SCL held,
     * so the NACK for byte N can be armed with no deadline. */
    if (!wait_flag(&I2C1->SR1, I2C_SR1_BTF))
    {
        return rm_fail(I2C_ERR_RXNE);
    }
    I2C1->CR1 &= ~I2C_CR1_ACK;

    /* Reading N-2 releases SCL and byte N starts; STOP must be set before
     * it finishes, so these three accesses are not interruptible. */
    m = taskENTER_CRITICAL_FROM_ISR();
    buf[i++] = (uint8_t)I2C1->DR;           /* N-2 */
    I2C1->CR1 |= I2C_CR1_STOP;
    buf[i++] = (uint8_t)I2C1->DR;           /* N-1 */
    taskEXIT_CRITICAL_FROM_ISR(m);

    if (!wait_flag(&I2C1->SR1, I2C_SR1_RXNE))
    {
        return rm_fail(I2C_ERR_RXNE);
    }
    buf[i] = (uint8_t)I2C1->DR;             /* N */

    return I2C_OK;
}
