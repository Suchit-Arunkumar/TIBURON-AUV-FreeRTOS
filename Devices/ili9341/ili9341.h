#ifndef ILI9341_H
#define ILI9341_H

#include <stdint.h>

/*
 * ILI9341 240x320 TFT on SPI2 (shared with the SD card), used in landscape
 * (320 x 240). Pins: CS PC5, DC PC0 (low = command, high = data), RST PC1.
 * Only spi_owner_task may call these once the scheduler runs.
 *
 * Text only: characters are the 5x7 font drawn at 2x, so each takes a
 * 12 x 16 pixel cell and the screen holds 26 x 15 of them.
 *
 * The start-up commands follow Adafruit's ILI9341 library (the datasheet
 * isn't in docs/datasheets yet). UNTESTED on hardware.
 */

#define ILI9341_WIDTH    320
#define ILI9341_HEIGHT   240

#define TFT_COLS          26
#define TFT_ROWS          15

/* RGB565 */
#define TFT_BLACK    0x0000U
#define TFT_WHITE    0xFFFFU
#define TFT_RED      0xF800U
#define TFT_GREEN    0x07E0U
#define TFT_YELLOW   0xFFE0U
#define TFT_CYAN     0x07FFU

/* Reset and configure the panel, then clear it. ~300 ms. */
void ili9341_init(void);

void ili9341_fill_rect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color);

/* Text at a character cell (col 0..25, row 0..14). */
void ili9341_draw_text(uint8_t col, uint8_t row, const char *s,
                       uint16_t fg, uint16_t bg);

#endif
