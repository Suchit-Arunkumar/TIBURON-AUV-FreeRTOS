#include "spi.h"
#include "stm32f446xx.h"

/*
 * SPI2 — shared bus for the SSD1306 OLED and the SD card.
 *
 * Moved off SPI1 (PA5/PA6/PA7) because that mapping collided with LD2 on
 * PA5 and with TIM3_CH1 on PA6; audit finding B3. SPI2 on PB13/14/15 is
 * clear of every other function in the final pin map.
 *
 *   PB13  SPI2_SCK   AF5
 *   PB14  SPI2_MISO  AF5
 *   PB15  SPI2_MOSI  AF5
 *   PC4   SD_CS      GPIO out, idle high
 *   PC5   OLED_CS    GPIO out, idle high
 *
 * SPI2 sits on APB1 = 45 MHz, so the BR field divides 45 MHz. See
 * spi.h for the divisor table and why each rate was chosen.
 */

void spi2_init(void)
{
    /* 1. Peripheral clocks: GPIOB (bus pins), GPIOC (both chip selects). */
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
    RCC->APB1ENR |= RCC_APB1ENR_SPI2EN;

    /* 2. PB13/14/15 to alternate function mode. Mask-then-set, never a
     *    bare OR — a stale MODER field is how B3 happened. */
    GPIOB->MODER &= ~((3U << (2 * 13)) | (3U << (2 * 14)) | (3U << (2 * 15)));
    GPIOB->MODER |=  ((2U << (2 * 13)) | (2U << (2 * 14)) | (2U << (2 * 15)));

    /* MISO pull-up: the SD spec wants DO pulled up, and with no card fitted
     * a floating MISO could be mistaken for a valid R1 response. */
    GPIOB->PUPDR &= ~(3U << (2 * 14));
    GPIOB->PUPDR |=  (1U << (2 * 14));

    /* 3. High speed on all three — the bus runs at up to 11.25 MHz. */
    GPIOB->OSPEEDR |= (3U << (2 * 13)) | (3U << (2 * 14)) | (3U << (2 * 15));

    /* 4. AF5 = SPI2 for all three. Pins 13-15 live in AFR[1], at
     *    nibble (pin - 8). */
    GPIOB->AFR[1] &= ~((0xFU << ((13 - 8) * 4)) |
                       (0xFU << ((14 - 8) * 4)) |
                       (0xFU << ((15 - 8) * 4)));
    GPIOB->AFR[1] |=  ((5U << ((13 - 8) * 4)) |
                       (5U << ((14 - 8) * 4)) |
                       (5U << ((15 - 8) * 4)));

    /* 5. Chip selects as push-pull outputs. Driven high BEFORE they are
     *    switched to outputs, so neither device sees a spurious select
     *    during the mode change. */
    GPIOC->BSRR = (1U << 4) | (1U << 5);

    GPIOC->MODER &= ~((3U << (2 * 4)) | (3U << (2 * 5)));
    GPIOC->MODER |=  ((1U << (2 * 4)) | (1U << (2 * 5)));

    GPIOC->OSPEEDR |= (3U << (2 * 4)) | (3U << (2 * 5));

    /* Re-assert idle-high now that they are genuinely outputs. */
    GPIOC->BSRR = (1U << 4) | (1U << 5);

    /* 6. CR1, composed in one write.
     *    SSM+SSI  software slave management, internal NSS high (no MODF)
     *    MSTR     master
     *    BR       start slow; the bus owner sets the real rate per device
     *    CPOL=0, CPHA=0  SPI mode 0 — what both the SSD1306 and SD
     *                    cards in SPI mode require. */
    SPI2->CR1 = SPI_CR1_SSM | SPI_CR1_SSI | SPI_CR1_MSTR | SPI_BR_DIV256;

    SPI2->CR2 = 0U;

    /* 7. Enable. */
    SPI2->CR1 |= SPI_CR1_SPE;
}

void spi_set_baud(uint32_t br_bits)
{
    /*
     * BR[2:0] may only be changed while the peripheral is disabled, and
     * only while the bus is idle. Both chip selects are expected to be
     * high when this is called — that is the bus owner's contract.
     */
    while (SPI2->SR & SPI_SR_BSY)
    {
    }

    SPI2->CR1 &= ~SPI_CR1_SPE;
    SPI2->CR1 = (SPI2->CR1 & ~SPI_CR1_BR) | (br_bits & SPI_CR1_BR);
    SPI2->CR1 |= SPI_CR1_SPE;
}

void spi_transmit(uint8_t data)
{
    /* 1. Wait for room in the TX buffer. */
    while (!(SPI2->SR & SPI_SR_TXE))
    {
    }

    /* 2. 8-bit access to DR. A 16-bit write would clock out two frames. */
    *((__IO uint8_t *)&SPI2->DR) = data;

    /* 3. Wait for the byte to reach the shift register, then for the
     *    shift register to empty. */
    while (!(SPI2->SR & SPI_SR_TXE))
    {
    }

    while (SPI2->SR & SPI_SR_BSY)
    {
    }

    /* 4. Drain RXNE. Full duplex clocks a byte in for every byte out;
     *    leaving it sets OVR, which then corrupts the next read. */
    (void)(*((__IO uint8_t *)&SPI2->DR));
}

uint8_t spi_transfer(uint8_t data)
{
    while (!(SPI2->SR & SPI_SR_TXE))
    {
    }

    *((__IO uint8_t *)&SPI2->DR) = data;

    while (!(SPI2->SR & SPI_SR_RXNE))
    {
    }

    return (uint8_t)(*((__IO uint8_t *)&SPI2->DR));
}

uint8_t spi_receive(void)
{
    /* 0xFF, not 0x00: SD cards read MOSI during a receive and a stream
     * of zeroes can be mistaken for a command byte. */
    return spi_transfer(0xFFU);
}

void spi_select_oled(void)
{
    GPIOC->BSRR = (1U << (5 + 16));
}

void spi_deselect_oled(void)
{
    GPIOC->BSRR = (1U << 5);
}

void spi_select_sd(void)
{
    GPIOC->BSRR = (1U << (4 + 16));
}

void spi_deselect_sd(void)
{
    GPIOC->BSRR = (1U << 4);
}
