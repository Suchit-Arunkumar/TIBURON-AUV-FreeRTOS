// SPI2, shared by the SD card and the ILI9341 TFT. Only spi_owner_task
// uses it once the scheduler runs.
//
//   PB13 SCK, PB14 MISO, PB15 MOSI (AF5)
//   PC4  SD card CS, PC5 TFT CS (GPIO, idle high)

#include "spi.h"
#include "stm32f446xx.h"

void spi2_init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
    RCC->APB1ENR |= RCC_APB1ENR_SPI2EN;

    GPIOB->MODER &= ~((3U << (2 * 13)) | (3U << (2 * 14)) | (3U << (2 * 15)));
    GPIOB->MODER |=  ((2U << (2 * 13)) | (2U << (2 * 14)) | (2U << (2 * 15)));

    // MISO pull-up: SD cards expect it, and a floating MISO with no card
    // could read as a valid response
    GPIOB->PUPDR &= ~(3U << (2 * 14));
    GPIOB->PUPDR |=  (1U << (2 * 14));

    GPIOB->OSPEEDR |= (3U << (2 * 13)) | (3U << (2 * 14)) | (3U << (2 * 15));

    GPIOB->AFR[1] &= ~((0xFU << ((13 - 8) * 4)) |
                       (0xFU << ((14 - 8) * 4)) |
                       (0xFU << ((15 - 8) * 4)));
    GPIOB->AFR[1] |=  ((5U << ((13 - 8) * 4)) |
                       (5U << ((14 - 8) * 4)) |
                       (5U << ((15 - 8) * 4)));

    // chip selects high before they become outputs, so neither device
    // sees a glitch
    GPIOC->BSRR = (1U << 4) | (1U << 5);

    GPIOC->MODER &= ~((3U << (2 * 4)) | (3U << (2 * 5)));
    GPIOC->MODER |=  ((1U << (2 * 4)) | (1U << (2 * 5)));

    GPIOC->OSPEEDR |= (3U << (2 * 4)) | (3U << (2 * 5));

    GPIOC->BSRR = (1U << 4) | (1U << 5);

    // master, software NSS, mode 0, slowest clock until a device is chosen
    SPI2->CR1 = SPI_CR1_SSM | SPI_CR1_SSI | SPI_CR1_MSTR | SPI_BR_DIV256;

    SPI2->CR2 = 0U;

    SPI2->CR1 |= SPI_CR1_SPE;
}

void spi_set_baud(uint32_t br_bits)
{
    // BR can only change while SPI is disabled, and disabling it mid-byte
    // corrupts that byte, so wait for the bus to go idle first
    while (SPI2->SR & SPI_SR_BSY)
    {
    }

    SPI2->CR1 &= ~SPI_CR1_SPE;
    SPI2->CR1 = (SPI2->CR1 & ~SPI_CR1_BR) | (br_bits & SPI_CR1_BR);
    SPI2->CR1 |= SPI_CR1_SPE;
}

void spi_transmit(uint8_t data)
{
    while (!(SPI2->SR & SPI_SR_TXE))
    {
    }

    // 8-bit access: a 16-bit write would send two bytes
    *((__IO uint8_t *)&SPI2->DR) = data;

    // wait for the byte to actually finish before CS can go high
    while (!(SPI2->SR & SPI_SR_TXE))
    {
    }

    while (SPI2->SR & SPI_SR_BSY)
    {
    }

    // read the byte clocked in, or the next one sets the overrun flag
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

// Clock out 0xFF while reading: SD cards watch MOSI, and zeros could
// look like the start of a command.
uint8_t spi_receive(void)
{
    return spi_transfer(0xFFU);
}

void spi_select_tft(void)
{
    GPIOC->BSRR = (1U << (5 + 16));
}

void spi_deselect_tft(void)
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
