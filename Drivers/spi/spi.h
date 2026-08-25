#ifndef SPI_H
#define SPI_H

#include <stdint.h>

/*
 * SPI2 baud divisors.
 *
 * SPI2 is on APB1, so BR divides PCLK1 = 45 MHz:
 *
 *   BR  div    f_SCK        use
 *   000    2   22.500 MHz   too fast for both devices
 *   001    4   11.250 MHz   SD full speed (cards are rated >= 25 MHz)
 *   010    8    5.625 MHz   OLED — inside the SSD1306 ~10 MHz ceiling
 *   011   16    2.813 MHz
 *   100   32    1.406 MHz
 *   101   64  703.125 kHz
 *   110  128  351.563 kHz   SD init — inside the 100-400 kHz window
 *   111  256  175.781 kHz
 *
 * div 128 is the only divisor that lands in the SD card's mandatory
 * 100-400 kHz initialisation window; div 64 at 703 kHz is already out.
 * See Phase 8 for the arithmetic behind the full-speed choices.
 */
#define SPI_BR_DIV2     (0U << 3)
#define SPI_BR_DIV4     (1U << 3)
#define SPI_BR_DIV8     (2U << 3)
#define SPI_BR_DIV16    (3U << 3)
#define SPI_BR_DIV32    (4U << 3)
#define SPI_BR_DIV64    (5U << 3)
#define SPI_BR_DIV128   (6U << 3)
#define SPI_BR_DIV256   (7U << 3)

/* Named rates, so call sites say what they mean. */
#define SPI_BR_SD_INIT      SPI_BR_DIV128   /* 351.6 kHz */
#define SPI_BR_SD_FAST      SPI_BR_DIV4     /*  11.25 MHz */
#define SPI_BR_OLED         SPI_BR_DIV8     /*   5.625 MHz */

void    spi2_init(void);

/*
 * Change SCK rate. Only legal with both chip selects deasserted and the
 * bus idle — enforced by convention, since only the bus owner task calls
 * this.
 */
void    spi_set_baud(uint32_t br_bits);

void    spi_transmit(uint8_t data);
uint8_t spi_receive(void);
uint8_t spi_transfer(uint8_t data);
void    spi_select_oled(void);
void    spi_deselect_oled(void);
void    spi_select_sd(void);
void    spi_deselect_sd(void);

#endif
