#include "ili9341.h"
#include "font5x7.h"
#include "spi.h"
#include "timer_timebase.h"
#include "stm32f446xx.h"

#include "FreeRTOS.h"
#include "task.h"

// ILI9341 command codes
#define CMD_SWRESET   0x01U
#define CMD_SLPOUT    0x11U
#define CMD_DISPON    0x29U
#define CMD_CASET     0x2AU     // column range
#define CMD_PASET     0x2BU     // row (page) range
#define CMD_RAMWR     0x2CU     // pixel data follows
#define CMD_MADCTL    0x36U     // scan direction / orientation
#define CMD_COLMOD    0x3AU     // pixel format

#define MADCTL_LANDSCAPE   0x28U    // row/column exchange + BGR order
#define COLMOD_16BIT       0x55U    // RGB565

#define SCALE   2U                  // font drawn at 2x
#define CELL_W  (6U * SCALE)        // 5 columns + 1 spacing
#define CELL_H  (8U * SCALE)

#define DC_PIN    0U    // PC0
#define RST_PIN   1U    // PC1

static void dc_command(void) { GPIOC->BSRR = (1U << (DC_PIN + 16U)); }
static void dc_data(void)    { GPIOC->BSRR = (1U << DC_PIN); }

// Works before the scheduler starts (main calls init) and after.
static void wait_ms(uint32_t ms)
{
    if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING)
    {
        vTaskDelay(pdMS_TO_TICKS(ms));
        return;
    }

    uint32_t start = micros();
    while ((micros() - start) < ms * 1000U)
    {
    }
}

static void write_cmd(uint8_t cmd)
{
    spi_select_tft();
    dc_command();
    spi_transmit(cmd);
    spi_deselect_tft();
}

static void write_cmd_data(uint8_t cmd, const uint8_t *data, uint8_t len)
{
    spi_select_tft();
    dc_command();
    spi_transmit(cmd);
    dc_data();
    for (uint8_t i = 0; i < len; i++)
    {
        spi_transmit(data[i]);
    }
    spi_deselect_tft();
}

// Set the drawing window and start a pixel write. CS stays low; the
// caller streams w*h pixels and then deselects.
static void begin_window(uint16_t x, uint16_t y, uint16_t w, uint16_t h)
{
    uint16_t x1 = (uint16_t)(x + w - 1U);
    uint16_t y1 = (uint16_t)(y + h - 1U);

    const uint8_t cols[4] = { (uint8_t)(x >> 8), (uint8_t)x, (uint8_t)(x1 >> 8), (uint8_t)x1 };
    const uint8_t rows[4] = { (uint8_t)(y >> 8), (uint8_t)y, (uint8_t)(y1 >> 8), (uint8_t)y1 };

    write_cmd_data(CMD_CASET, cols, 4);
    write_cmd_data(CMD_PASET, rows, 4);

    spi_select_tft();
    dc_command();
    spi_transmit(CMD_RAMWR);
    dc_data();
}

static void pixel(uint16_t color)
{
    spi_transmit((uint8_t)(color >> 8));
    spi_transmit((uint8_t)color);
}

void ili9341_init(void)
{
    // DC and RST as outputs, both high
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
    GPIOC->BSRR   = (1U << DC_PIN) | (1U << RST_PIN);
    GPIOC->MODER &= ~((3U << (2U * DC_PIN)) | (3U << (2U * RST_PIN)));
    GPIOC->MODER |=  ((1U << (2U * DC_PIN)) | (1U << (2U * RST_PIN)));

    spi_set_baud(SPI_BR_TFT);

    // hardware reset
    GPIOC->BSRR = (1U << (RST_PIN + 16U));
    wait_ms(10);
    GPIOC->BSRR = (1U << RST_PIN);
    wait_ms(150);

    write_cmd(CMD_SWRESET);
    wait_ms(150);

    write_cmd(CMD_SLPOUT);
    wait_ms(150);

    const uint8_t colmod = COLMOD_16BIT;
    write_cmd_data(CMD_COLMOD, &colmod, 1);

    const uint8_t madctl = MADCTL_LANDSCAPE;
    write_cmd_data(CMD_MADCTL, &madctl, 1);

    write_cmd(CMD_DISPON);
    wait_ms(20);

    ili9341_fill_rect(0, 0, ILI9341_WIDTH, ILI9341_HEIGHT, TFT_BLACK);
}

void ili9341_fill_rect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color)
{
    if ((w == 0U) || (h == 0U))
    {
        return;
    }

    begin_window(x, y, w, h);

    for (uint32_t n = (uint32_t)w * h; n > 0U; n--)
    {
        pixel(color);
    }

    spi_deselect_tft();
}

// One character cell: 5x7 glyph scaled up, with a blank column and row.
static void draw_char(uint16_t x, uint16_t y, char c, uint16_t fg, uint16_t bg)
{
    if ((c < 0x20) || (c > 0x7E))
    {
        c = ' ';
    }

    const uint8_t *glyph = font5x7[(uint8_t)(c - 0x20)];

    begin_window(x, y, CELL_W, CELL_H);

    for (uint8_t row = 0; row < CELL_H; row++)
    {
        uint8_t font_row = (uint8_t)(row / SCALE);

        for (uint8_t col = 0; col < CELL_W; col++)
        {
            uint8_t font_col = (uint8_t)(col / SCALE);
            uint8_t on = (font_col < 5U) && (font_row < 7U) &&
                         ((glyph[font_col] >> font_row) & 1U);

            pixel(on ? fg : bg);
        }
    }

    spi_deselect_tft();
}

void ili9341_draw_text(uint8_t col, uint8_t row, const char *s,
                       uint16_t fg, uint16_t bg)
{
    while ((*s != '\0') && (col < TFT_COLS) && (row < TFT_ROWS))
    {
        draw_char((uint16_t)(col * CELL_W), (uint16_t)(row * CELL_H), *s, fg, bg);
        s++;
        col++;
    }
}
