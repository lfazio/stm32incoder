#include "board/trace.h"
#include "board/board.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

/* USART2_TX -> DMA1 Stream 6, channel 4 (RM0390 Rev 9 Table 28). */
#define TRACE_DMA_STREAM   DMA1_Stream6
#define TRACE_DMA_CHANNEL  4u
#define TRACE_DMA_IRQn     DMA1_Stream6_IRQn
#define TRACE_DMA_CLEAR    (DMA_HIFCR_CTCIF6 | DMA_HIFCR_CHTIF6 | DMA_HIFCR_CTEIF6 | \
                            DMA_HIFCR_CDMEIF6 | DMA_HIFCR_CFEIF6)

#define RING_SIZE   4096u   /* power of two */
#define RING_MASK   (RING_SIZE - 1u)

static UART_HandleTypeDef s_uart;
static volatile uint8_t   s_ring[RING_SIZE];
static volatile uint32_t  s_head;      /* producer */
static volatile uint32_t  s_tail;      /* consumer */
static volatile uint32_t  s_busy_len;  /* bytes currently owned by the DMA */
static volatile uint32_t  s_dropped;

static void trace_kick(void)
{
    if (s_busy_len != 0u) {
        return;                         /* a transfer is already running */
    }

    uint32_t head = s_head;
    uint32_t tail = s_tail;
    if (head == tail) {
        return;                         /* nothing to send */
    }

    /* Send up to the end of the ring; the wrap is picked up next kick. */
    uint32_t len = (head > tail) ? (head - tail) : (RING_SIZE - tail);
    s_busy_len = len;

    TRACE_DMA_STREAM->CR &= ~DMA_SxCR_EN;
    while (TRACE_DMA_STREAM->CR & DMA_SxCR_EN) { }
    DMA1->HIFCR = TRACE_DMA_CLEAR;

    TRACE_DMA_STREAM->M0AR = (uint32_t)(uintptr_t)&s_ring[tail];
    TRACE_DMA_STREAM->NDTR = len;

    __HAL_UART_CLEAR_FLAG(&s_uart, UART_FLAG_TC);
    TRACE_DMA_STREAM->CR |= DMA_SxCR_EN;
    TRACE_UART->CR3 |= USART_CR3_DMAT;
}

void trace_init(void)
{
    GPIO_InitTypeDef io = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_USART2_CLK_ENABLE();
    __HAL_RCC_DMA1_CLK_ENABLE();

    io.Pin       = TRACE_UART_TX_PIN | TRACE_UART_RX_PIN;
    io.Mode      = GPIO_MODE_AF_PP;
    io.Pull      = GPIO_PULLUP;
    io.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    io.Alternate = TRACE_UART_AF;
    HAL_GPIO_Init(TRACE_UART_GPIO, &io);

    s_uart.Instance          = TRACE_UART;
    s_uart.Init.BaudRate     = TRACE_UART_BAUD;
    s_uart.Init.WordLength   = UART_WORDLENGTH_8B;
    s_uart.Init.StopBits     = UART_STOPBITS_1;
    s_uart.Init.Parity       = UART_PARITY_NONE;
    s_uart.Init.Mode         = UART_MODE_TX_RX;
    s_uart.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    s_uart.Init.OverSampling = UART_OVERSAMPLING_16;
    HAL_UART_Init(&s_uart);

    TRACE_DMA_STREAM->CR = 0;
    while (TRACE_DMA_STREAM->CR & DMA_SxCR_EN) { }
    DMA1->HIFCR = TRACE_DMA_CLEAR;
    TRACE_DMA_STREAM->PAR = (uint32_t)(uintptr_t)&TRACE_UART->DR;
    TRACE_DMA_STREAM->CR  = (TRACE_DMA_CHANNEL << DMA_SxCR_CHSEL_Pos)
                          | DMA_SxCR_DIR_0      /* memory -> peripheral */
                          | DMA_SxCR_MINC
                          | DMA_SxCR_TCIE;
    TRACE_DMA_STREAM->FCR = 0;

    /* Below the SSI interrupts: tracing must never delay a frame. */
    HAL_NVIC_SetPriority(TRACE_DMA_IRQn, 3, 0);
    HAL_NVIC_EnableIRQ(TRACE_DMA_IRQn);
}

void trace_write(const char *data, uint32_t len)
{
    uint32_t head  = s_head;
    uint32_t tail  = s_tail;
    uint32_t used  = (head - tail) & RING_MASK;
    uint32_t space = RING_MASK - used;

    if (len > space) {
        s_dropped++;
        return;
    }

    for (uint32_t i = 0; i < len; i++) {
        s_ring[(head + i) & RING_MASK] = (uint8_t)data[i];
    }
    s_head = (head + len) & RING_MASK;

    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    trace_kick();
    __set_PRIMASK(primask);
}

void trace_printf(const char *fmt, ...)
{
    char    buf[192];
    va_list ap;

    va_start(ap, fmt);
    int n = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);

    if (n > 0) {
        trace_write(buf, (uint32_t)((n < (int)sizeof(buf)) ? n : (int)sizeof(buf) - 1));
    }
}

uint32_t trace_dropped(void)
{
    return s_dropped;
}

void trace_flush(void)
{
    while (s_busy_len != 0u || s_head != s_tail) { }
    while (!(TRACE_UART->SR & USART_SR_TC)) { }
}

void DMA1_Stream6_IRQHandler(void)
{
    if (DMA1->HISR & DMA_HISR_TCIF6) {
        DMA1->HIFCR = TRACE_DMA_CLEAR;
        TRACE_UART->CR3 &= ~USART_CR3_DMAT;

        s_tail     = (s_tail + s_busy_len) & RING_MASK;
        s_busy_len = 0;
        trace_kick();
    } else {
        DMA1->HIFCR = TRACE_DMA_CLEAR;
    }
}
