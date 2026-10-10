#include <stdio.h>
#include "stm32f446xx.h"
#include "i2c.h"
#include "timer_timebase.h"

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#define APB1CLK_MHZ  45U

/*
 * Every wait is bounded (a slave can hold the bus forever), and timed with
 * the TIM2 microsecond counter. A healthy byte takes ~90 us at 100 kHz, so
 * spin for a bit first; after that the bus is misbehaving and the task
 * sleeps 1 ms between polls instead of hogging the CPU. No sleeping before
 * the scheduler starts.
 */
#define I2C_SPIN_US       250UL    /* fast path: ~3 byte times     */
#define I2C_TIMEOUT_US   2500UL    /* budget per wait: ~27 byte times */

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
        /* NACK: no device. Clear it for the next transfer. */
        I2C1->SR1 &= ~I2C_SR1_AF;
        return -1;
    }

    return 0;
}

/* Returns 1 if the flag appeared within budget, 0 on timeout or NACK. */
static uint32_t wait_flag(volatile uint32_t *reg, uint32_t flag)
{
    uint32_t start = micros();

    do
    {
        int32_t r = poll_once(reg, flag);

        if (r == 1)  { return 1UL; }
        if (r == -1) { return 0UL; }

    } while ((micros() - start) < I2C_SPIN_US);

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

    /*
     * One last poll after the deadline. A task preempted for longer than
     * the budget would otherwise report a timeout on a transfer that had
     * finished, then abort it mid-way (found on the bench).
     */
    int32_t r = poll_once(reg, flag);
    return (r == 1) ? 1UL : 0UL;
}

/*
 * Abort a transfer. A STOP at the wrong moment can leave the F4's I2C
 * stuck BUSY with both lines high (seen on the bench), so if BUSY is still
 * set afterwards, reset the peripheral.
 */
static I2C_Status i2c_fail(I2C_Status st)
{
    I2C1->CR1 |= I2C_CR1_STOP;
    I2C1->CR1 &= ~I2C_CR1_POS;

    uint32_t start = micros();
    while ((I2C1->CR1 & I2C_CR1_STOP) && ((micros() - start) < 1000UL))
    {
    }

    if (I2C1->SR2 & I2C_SR2_BUSY)
    {
        i2c1_init();                 /* SWRST + full reconfigure */
    }
    return st;
}

void i2c1_init(void)
{
	// PB8 SCL, PB9 SDA: AF4, open-drain
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;

	GPIOB->MODER &= ~(3 << (2*8));
	GPIOB->MODER |=  (2 << (2*8));

	GPIOB->MODER &= ~(3 << (2*9));
	GPIOB->MODER |=  (2 << (2*9));

	GPIOB->OTYPER |= GPIO_OTYPER_OT8;
	GPIOB->OTYPER |= GPIO_OTYPER_OT9;

	GPIOB->AFR[1] &= ~((0xFU << 0) | (0xFU << 4));
	GPIOB->AFR[1] |= (4 << 0);
	GPIOB->AFR[1] |= (4 << 4);

	RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;

	I2C1->CR1 |= I2C_CR1_SWRST;
	I2C1->CR1 &= ~I2C_CR1_SWRST;

	I2C1->CR2 = APB1CLK_MHZ;

	// 100 kHz standard mode: SCL high = low = CCR cycles of 45 MHz,
	// 45 MHz / (2 x 225) = 100 kHz. 400 kHz was unreliable on bench wiring.
	I2C1->CCR = 225U;

	// max rise time 1000 ns x 45 MHz + 1
	I2C1->TRISE = 46U;

	I2C1->CR1 |= I2C_CR1_PE;
}

static void bus_recover_unlocked(void)
{
    /*
     * A slave reset mid-transfer can hold SDA low forever. Clock SCL by
     * hand nine times so it finishes its byte and lets go, then send STOP.
     */
    I2C1->CR1 &= ~I2C_CR1_PE;

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

    i2c1_init();
}

static I2C_Status write_unlocked(uint8_t addr, const uint8_t *data, uint16_t len)
{
	I2C1->CR1 |= I2C_CR1_START;

	if (!wait_flag(&I2C1->SR1, I2C_SR1_SB))
	{
		return i2c_fail(I2C_ERR_START);
	}

	I2C1->DR = (addr << 1) | 0;

	if (!wait_flag(&I2C1->SR1, I2C_SR1_ADDR))
	{
		return i2c_fail(I2C_ERR_ADDR_NACK);
	}

	// reading SR1 then SR2 clears ADDR
	(void)I2C1->SR1;
	(void)I2C1->SR2;

	for(uint16_t i = 0; i < len; i++){

		if (!wait_flag(&I2C1->SR1, I2C_SR1_TXE))
		{
			return i2c_fail(I2C_ERR_TXE);
		}
		I2C1->DR = data[i] ;

	}

	// Wait for BTF (last byte fully sent) before STOP, or the last byte
	// is dropped (RM0390 EV8_2). Found when the MPU-6050 wouldn't wake.
	if (!wait_flag(&I2C1->SR1, I2C_SR1_BTF))
	{
		return i2c_fail(I2C_ERR_TXE);
	}

	I2C1->CR1 |= I2C_CR1_STOP;

	return I2C_OK;
}

static I2C_Status read_legacy_unlocked(uint8_t addr, uint8_t *buf, uint16_t len)
{
	I2C1->CR1 |= I2C_CR1_ACK;
	I2C1->CR1 |= I2C_CR1_START;

	if (!wait_flag(&I2C1->SR1, I2C_SR1_SB))
	{
		return i2c_fail(I2C_ERR_START);
	}

	I2C1->DR = (addr << 1) | 1;

	if (!wait_flag(&I2C1->SR1, I2C_SR1_ADDR))
	{
		return i2c_fail(I2C_ERR_ADDR_NACK);
	}

	(void)I2C1->SR1;
	(void)I2C1->SR2;

	// NACK + STOP set in software just before the last byte: too late if
	// the task is preempted at that moment
	for(uint16_t i = 0; i < len; i++){
	    if(i == (len - 1U)){
	        I2C1->CR1 &= ~I2C_CR1_ACK;
	        I2C1->CR1 |= I2C_CR1_STOP;
	    }
	    if (!wait_flag(&I2C1->SR1, I2C_SR1_RXNE))
	    {
	        return i2c_fail(I2C_ERR_RXNE);
	    }
	    buf[i] = I2C1->DR;
	}

	return I2C_OK;
}


/*
 * Master receive using the reference manual's sequences (RM0390 24.3.3).
 *
 * The legacy version clears ACK in software just before the last byte,
 * a ~23 us window. If the task is preempted then, one byte too many is
 * ACKed and the extra byte turns up at the start of the next read, with
 * no error reported. Here the ACK/STOP decisions are made while the
 * peripheral holds the clock (BTF), and the few accesses that must be
 * back to back run with interrupts masked.
 */
static I2C_Status rm_fail(I2C_Status st)
{
	return i2c_fail(st);
}

static I2C_Status read_unlocked(uint8_t addr, uint8_t *buf, uint16_t len)
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
        /* POS: ACK applies to the next byte, so clearing it right after
         * ADDR NACKs byte 2. */
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
        /* NACK before clearing ADDR, STOP straight after. */
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

    uint16_t i = 0;
    while ((uint16_t)(len - i) > 3U)
    {
        if (!wait_flag(&I2C1->SR1, I2C_SR1_RXNE))
        {
            return rm_fail(I2C_ERR_RXNE);
        }
        buf[i++] = (uint8_t)I2C1->DR;
    }

    /* Three left. At BTF the clock is held, so NACK can be set safely. */
    if (!wait_flag(&I2C1->SR1, I2C_SR1_BTF))
    {
        return rm_fail(I2C_ERR_RXNE);
    }
    I2C1->CR1 &= ~I2C_CR1_ACK;

    /* Reading N-2 releases the clock; STOP must be set before byte N ends. */
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


/*
 * The Bar30 (depth_task) and BNO085 (imu_task) share I2C1. Each public
 * function holds this mutex for one transfer only, so the BNO085 can use
 * the bus while the Bar30 waits for a conversion. A mutex rather than a
 * semaphore for priority inheritance. Skipped before the scheduler runs.
 */
#define I2C_LOCK_TIMEOUT_MS   50U

static SemaphoreHandle_t i2c_mutex = NULL;

int i2c1_lock_create(void)
{
    if (i2c_mutex == NULL)
    {
        i2c_mutex = xSemaphoreCreateMutex();
    }

    return (i2c_mutex != NULL) ? 1 : 0;
}

static int bus_take(void)
{
    if ((i2c_mutex == NULL) ||
        (xTaskGetSchedulerState() != taskSCHEDULER_RUNNING))
    {
        return 1;
    }

    return (xSemaphoreTake(i2c_mutex, pdMS_TO_TICKS(I2C_LOCK_TIMEOUT_MS)) == pdTRUE) ? 1 : 0;
}

static void bus_give(void)
{
    if ((i2c_mutex != NULL) &&
        (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING))
    {
        (void)xSemaphoreGive(i2c_mutex);
    }
}

I2C_Status i2c_write(uint8_t addr, const uint8_t *data, uint16_t len)
{
    if (!bus_take())
    {
        return I2C_ERR_BUSY;
    }

    I2C_Status st = write_unlocked(addr, data, len);

    bus_give();
    return st;
}

I2C_Status i2c_read(uint8_t addr, uint8_t *buf, uint16_t len)
{
    if (!bus_take())
    {
        return I2C_ERR_BUSY;
    }

    I2C_Status st = read_unlocked(addr, buf, len);

    bus_give();
    return st;
}

I2C_Status i2c_read_legacy(uint8_t addr, uint8_t *buf, uint16_t len)
{
    if (!bus_take())
    {
        return I2C_ERR_BUSY;
    }

    I2C_Status st = read_legacy_unlocked(addr, buf, len);

    bus_give();
    return st;
}

void i2c1_bus_recover(void)
{
    if (!bus_take())
    {
        return;
    }

    bus_recover_unlocked();

    bus_give();
}
