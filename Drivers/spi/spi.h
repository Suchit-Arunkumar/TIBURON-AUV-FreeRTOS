#ifndef SPI_H
#define SPI_H

#include <stdint.h>

/*
 * SPI2 clock dividers. SPI2 runs from APB1 = 45 MHz:
 *   /4   11.25 MHz
 *   /8    5.625 MHz   SD data and the TFT (ILI9341 writes up to 10 MHz)
 *   /128  352 kHz     SD start-up (must be 100-400 kHz)
 * SD data could go to /4 once the card is reliable on the real wiring.
 */
#define SPI_BR_DIV2     (0U << 3)
#define SPI_BR_DIV4     (1U << 3)
#define SPI_BR_DIV8     (2U << 3)
#define SPI_BR_DIV16    (3U << 3)
#define SPI_BR_DIV32    (4U << 3)
#define SPI_BR_DIV64    (5U << 3)
#define SPI_BR_DIV128   (6U << 3)
#define SPI_BR_DIV256   (7U << 3)

#define SPI_BR_SD_INIT      SPI_BR_DIV128
#define SPI_BR_SD_DATA      SPI_BR_DIV8
#define SPI_BR_TFT          SPI_BR_DIV8

void    spi2_init(void);

/* Change the clock. Only with both chip selects high (the bus owner
 * calls it before each device's transfers). */
void    spi_set_baud(uint32_t br_bits);

void    spi_transmit(uint8_t data);
uint8_t spi_receive(void);
uint8_t spi_transfer(uint8_t data);
void    spi_select_tft(void);
void    spi_deselect_tft(void);
void    spi_select_sd(void);
void    spi_deselect_sd(void);

#endif
