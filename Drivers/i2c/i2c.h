
#ifndef I2C_h
#define I2C_h

#include <stdint.h>
#include "stm32f446xx.h"

/*
 * Bounded I2C.
 *
 * Every wait in this driver used to be an unbounded `while (!(SR1 &
 * flag));`. With no device on the bus - which is the normal state of the
 * bench rig, since the Bar30 is not attached - bar30_task would spin
 * forever at priority 4 on the very first START. Control (7), filter (6)
 * and comms (5) would keep running, so the board would look alive, while
 * logging (2), the SPI owner (3) and the console (1) were starved
 * permanently. No log blocks, no OLED, no heartbeat, no diagnostics.
 *
 * Every wait is now bounded and reports which stage timed out, so a
 * missing sensor degrades to a flagged SENSOR_ABSENT state instead.
 */
typedef enum
{
    I2C_OK             = 0,
    I2C_ERR_START      = 1,   /* SB never set - bus stuck or SCL held low */
    I2C_ERR_ADDR_NACK  = 2,   /* no device ACKed the address              */
    I2C_ERR_TXE        = 3,   /* transmit register never emptied          */
    I2C_ERR_RXNE       = 4,   /* receive register never filled            */
    I2C_ERR_BUSY       = 5    /* bus still busy from a previous transfer  */
} I2C_Status;

const char *i2c_status_str(I2C_Status s);

/*
 * Create the bus mutex. Call once from main before the scheduler starts;
 * returns 0 if the heap is exhausted. Every function below takes the
 * mutex for the length of one transfer once the scheduler is running.
 */
int        i2c1_lock_create(void);

/* 400 kHz fast mode. Every device on I2C1 supports it: the Bar30
 * (MS5837 datasheet, SCL max 400 kHz), the BNO085 and the bench MPU-6050. */
void       i2c1_init(void);

/* Lengths are 16-bit: a BNO085 SHTP packet can be a few hundred bytes. */
I2C_Status i2c_write(uint8_t addr, const uint8_t *data, uint16_t len);

/* Master receive using RM0390's per-length sequences (N = 1, N = 2 with
 * POS, N > 2 with BTF on the last three bytes). Safe if the task is
 * preempted mid-transfer. Use this one. */
I2C_Status i2c_read(uint8_t addr, uint8_t *buf, uint16_t len);

/* The original receive routine. It can ACK one byte too many if the task
 * is preempted at the wrong moment, and the extra byte then corrupts the
 * next read. Kept only so the bench test can show the difference. */
I2C_Status i2c_read_legacy(uint8_t addr, uint8_t *buf, uint16_t len);

/*
 * Recover a bus left stuck by a device mid-transfer. Clears BUSY by
 * clocking SCL manually, then re-initialises the peripheral. Called by
 * bar30_task after a timeout rather than on every error.
 */
void       i2c1_bus_recover(void);

#endif
