#include "uart4.h"
#include "stm32f446xx.h"

#include "FreeRTOS.h"
#include "task.h"

#define APB1CLK 45000000U
#define UART4_BR 115200U

static uint8_t dma4_rx_buf[UART4_DMA_BUF_SIZE];

static uint16_t last_dma_pos = 0;

static uint8_t uart4_rx_buf[UART4_DMA_BUF_SIZE];

static volatile uint16_t uart4_rx_head = 0;
static volatile uint16_t uart4_rx_tail = 0;

TaskHandle_t dvlTaskHandle = NULL;


//===========================================================================================================================
static void uart4_rx_store(const uint8_t *data, uint16_t length)
{
    for (uint16_t i = 0; i < length; i++)
    {
        uint16_t next_head =
            (uint16_t)((uart4_rx_head + 1U) % UART4_DMA_BUF_SIZE);

        /*
         * Buffer full.
         * Drop incoming byte rather than overwrite unread data.
         */
        if (next_head == uart4_rx_tail)
        {
            return;
        }

        uart4_rx_buf[uart4_rx_head] = data[i];

        uart4_rx_head = next_head;
    }
}


//===========================================================================================================================
void uart4_init(void)
{
    /*
     * 1. Enable GPIOC clock
     */
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;


    /*
     * 2. Configure PC10 as alternate-function mode
     *    PC10 = UART4_TX
     */
    GPIOC->MODER &= ~(3U << (2U * 10U));
    GPIOC->MODER |=  (2U << (2U * 10U));


    /*
     * 3. Configure PC11 as alternate-function mode
     *    PC11 = UART4_RX
     */
    GPIOC->MODER &= ~(3U << (2U * 11U));
    GPIOC->MODER |=  (2U << (2U * 11U));


    /*
     * 4. Set PC10 alternate function to AF8
     */
    GPIOC->AFR[1] &= ~(0xFU << 8U);
    GPIOC->AFR[1] |=  (8U << 8U);


    /*
     * 5. Set PC11 alternate function to AF8
     */
    GPIOC->AFR[1] &= ~(0xFU << 12U);
    GPIOC->AFR[1] |=  (8U << 12U);


    /*
     * 6. Enable UART4 clock
     */
    RCC->APB1ENR |= RCC_APB1ENR_UART4EN;


    /*
     * 7. Configure baud rate
     *    APB1 = 45 MHz
     *    Baud = 115200
     */
    UART4->BRR =
        ((APB1CLK + UART4_BR / 2U) / UART4_BR);


    /*
     * 8. Enable receiver
     */
    UART4->CR1 |= USART_CR1_RE;


    /*
     * 9. Enable IDLE line interrupt
     */
    UART4->CR1 |= USART_CR1_IDLEIE;


    /*
     * 10. Enable DMA RX request
     */
    UART4->CR3 |= USART_CR3_DMAR;


    /*
     * 11. Enable UART4
     */
    UART4->CR1 |= USART_CR1_UE;


    /*
     * 12. Enable DMA1 clock
     */
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;


    /*
     * 13. Disable DMA1 Stream2 before configuration
     */
    DMA1_Stream2->CR &= ~DMA_SxCR_EN;

    while (DMA1_Stream2->CR & DMA_SxCR_EN)
    {
    }


    /*
     * 14. Clear DMA configuration
     */
    DMA1_Stream2->CR = 0;


    /*
     * 15. Peripheral address
     */
    DMA1_Stream2->PAR =
        (uint32_t)&UART4->DR;


    /*
     * 16. Memory address
     */
    DMA1_Stream2->M0AR =
        (uint32_t)dma4_rx_buf;


    /*
     * 17. Number of bytes
     */
    DMA1_Stream2->NDTR =
        UART4_DMA_BUF_SIZE;


    /*
     * 18. Select Channel 4
     */
    DMA1_Stream2->CR &= ~DMA_SxCR_CHSEL;

    DMA1_Stream2->CR |=
        (4U << DMA_SxCR_CHSEL_Pos);


    /*
     * 19. Peripheral-to-memory
     */
    DMA1_Stream2->CR &= ~DMA_SxCR_DIR;


    /*
     * 20. Enable memory increment
     */
    DMA1_Stream2->CR |= DMA_SxCR_MINC;


    /*
     * 21. Enable circular mode
     */
    DMA1_Stream2->CR |= DMA_SxCR_CIRC;


    /*
     * 22. Configure byte transfers
     */
    DMA1_Stream2->CR &= ~DMA_SxCR_PSIZE;
    DMA1_Stream2->CR &= ~DMA_SxCR_MSIZE;


    /*
     * 23. Enable DMA stream
     */
    DMA1_Stream2->CR |= DMA_SxCR_EN;


    /*
     * 24. Enable UART4 interrupt
     */
    NVIC_SetPriority(UART4_IRQn, 6);
    NVIC_EnableIRQ(UART4_IRQn);
}


//===========================================================================================================================
void UART4_IRQHandler(void)
{
    if (UART4->SR & USART_SR_IDLE)
    {
        volatile uint32_t dummy;

        /*
         * Clear UART IDLE flag.
         *
         * Read SR followed by DR.
         */
        dummy = UART4->SR;
        dummy = UART4->DR;


        /*
         * Current DMA write position
         */
        uint16_t current_pos =
            UART4_DMA_BUF_SIZE -
            DMA1_Stream2->NDTR;


        /*
         * DMA has not wrapped around.
         */
        if (current_pos > last_dma_pos)
        {
            uart4_rx_store(
                &dma4_rx_buf[last_dma_pos],
                current_pos - last_dma_pos
            );
        }


        /*
         * DMA wrapped around.
         */
        else if (current_pos < last_dma_pos)
        {
            uart4_rx_store(
                &dma4_rx_buf[last_dma_pos],
                UART4_DMA_BUF_SIZE - last_dma_pos
            );

            uart4_rx_store(
                &dma4_rx_buf[0],
                current_pos
            );
        }


        /*
         * Remember current DMA position.
         */
        last_dma_pos = current_pos;


        /*
         * Wake DVL task.
         */
        if (dvlTaskHandle != NULL)
        {
            BaseType_t xHigherPriorityTaskWasWoken = pdFALSE;

            xTaskNotifyFromISR(
                dvlTaskHandle,
                (1UL << 0),
                eSetBits,
                &xHigherPriorityTaskWasWoken
            );

            portYIELD_FROM_ISR(xHigherPriorityTaskWasWoken);
        }

        (void)dummy;
    }
}


//===========================================================================================================================
uint16_t uart4_read(uint8_t *out, uint16_t max_len)
{
    uint16_t count = 0;

    if (out == NULL || max_len == 0)
    {
        return 0;
    }

    while ((uart4_rx_tail != uart4_rx_head) &&
           (count < max_len))
    {
        out[count++] =
            uart4_rx_buf[uart4_rx_tail];

        uart4_rx_tail =
            (uint16_t)((uart4_rx_tail + 1U) %
                       UART4_DMA_BUF_SIZE);
    }

    return count;
}
