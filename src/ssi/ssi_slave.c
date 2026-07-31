#include "ssi/ssi_slave.h"
#include "board/board.h"

/* DMA assignment, RM0390 Rev 9 Table 29 (DMA2 request mapping):
 *   SPI1_RX -> DMA2 Stream 2, channel 3   (also available on stream 0)
 *   SPI1_TX -> DMA2 Stream 3, channel 3   (also available on stream 5)
 * Stream 0 is left free for ADC1 (channel 0).
 */
#define SSI_RX_STREAM   DMA2_Stream2
#define SSI_TX_STREAM   DMA2_Stream3
#define SSI_DMA_CHANNEL 3u

#define SSI_RX_IRQn     DMA2_Stream2_IRQn

/* Flags for streams 2 and 3 live in the low interrupt registers. */
#define SSI_RX_CLEAR_FLAGS (DMA_LIFCR_CTCIF2 | DMA_LIFCR_CHTIF2 | DMA_LIFCR_CTEIF2 | \
                            DMA_LIFCR_CDMEIF2 | DMA_LIFCR_CFEIF2)
#define SSI_TX_CLEAR_FLAGS (DMA_LIFCR_CTCIF3 | DMA_LIFCR_CHTIF3 | DMA_LIFCR_CTEIF3 | \
                            DMA_LIFCR_CDMEIF3 | DMA_LIFCR_CFEIF3)

/* Tmu gap timer. TIM6 is a basic timer on APB1; with the APB1 prescaler != 1
 * its clock is 2 x PCLK1 = 90 MHz, so a prescaler of 90 gives a 1 us tick.
 *
 * The gap does not start exactly at the reference the specification uses. Tmu
 * is measured "from last falling edge of clock", but end-of-message is detected
 * by the receive DMA, which completes on the last *rising* edge -- half a clock
 * period later -- and the interrupt then takes time to reach gap_timer_start().
 *
 * A logic-analyser capture at 2 MHz measured 21.01 us mean from the last
 * falling edge, against the specified 20 us +/- 1 us. Subtracting one tick
 * centres it. The residual term is the half clock period, which the emulator
 * cannot know: it is 0.25 us at 2 MHz but 5 us at the 100 kHz lower limit, so
 * Tmu drifts long as the master's clock slows. See README "Measured against
 * the specification".
 */
#define GAP_TIM         TIM6
#define GAP_TIM_IRQn    TIM6_DAC_IRQn
#define GAP_TICK_PER_US 1u
#define GAP_OVERHEAD_US 1u

static ssi_slave_config_t    s_cfg;
static ssi_slave_provider_t  s_provider;
static void                 *s_ctx;
static uint8_t               s_nbytes;

static volatile uint8_t      s_tx[4];
static volatile uint8_t      s_rx[4];
static volatile bool         s_armed;
static volatile uint16_t     s_last_ndtr;
static volatile uint32_t     s_frames;
static volatile uint32_t     s_resyncs;
static volatile bool         s_gap_level;
static volatile uint32_t     s_moder_af;

/* --- DATA line ownership --------------------------------------------------
 * PB4 alternates between the SPI (which shifts message bits out of it) and
 * plain GPIO (which holds the idle/error-flag level between messages). The
 * SSI idle state is defined as DATA HIGH, and the SPI cannot guarantee that
 * level while no clock is running, so the pin is taken back after each frame.
 */
#define DATA_MODER_MASK  (3u << (SSI_SLAVE_DATA_PIN_NUM * 2u))
#define DATA_MODER_OUT   (1u << (SSI_SLAVE_DATA_PIN_NUM * 2u))
#define DATA_MODER_AF    (2u << (SSI_SLAVE_DATA_PIN_NUM * 2u))

static inline void data_drive(bool level)
{
    SSI_SLAVE_GPIO->BSRR = level ? SSI_SLAVE_DATA_PIN
                                 : ((uint32_t)SSI_SLAVE_DATA_PIN << 16u);
}

static inline void data_take_gpio(void)
{
    uint32_t m = SSI_SLAVE_GPIO->MODER;
    SSI_SLAVE_GPIO->MODER = (m & ~DATA_MODER_MASK) | DATA_MODER_OUT;
}

static inline void data_give_spi(void)
{
    uint32_t m = SSI_SLAVE_GPIO->MODER;
    SSI_SLAVE_GPIO->MODER = (m & ~DATA_MODER_MASK) | DATA_MODER_AF;
}

/* --- frame staging -------------------------------------------------------- */

static void stage_frame(void)
{
    ssi_slave_frame_t f = { .payload = 0, .error_flag = false };

    if (s_provider != NULL) {
        s_provider(s_ctx, &f);
    }

    /* MSB first: byte 0 carries D(n-1)..D(n-8). */
    for (uint8_t i = 0; i < s_nbytes; i++) {
        uint8_t shift = (uint8_t)(s_cfg.n_bits - 8u * (i + 1u));
        s_tx[i] = (uint8_t)((f.payload >> shift) & 0xFFu);
    }

    /* Note 3 of section 5.4.1: after the last rising edge the data line is set
     * by the Error Flag. PV is the inverse of the ERROR FLAG, so a valid
     * measurement drives DATA low during the gap. */
    s_gap_level = f.error_flag;
}

/* --- SPI/DMA arming ------------------------------------------------------- */

static void ssi_arm(void)
{
    SSI_SLAVE_SPI->CR1 &= ~SPI_CR1_SPE;

    SSI_TX_STREAM->CR &= ~DMA_SxCR_EN;
    SSI_RX_STREAM->CR &= ~DMA_SxCR_EN;
    while ((SSI_TX_STREAM->CR & DMA_SxCR_EN) || (SSI_RX_STREAM->CR & DMA_SxCR_EN)) {
    }

    DMA2->LIFCR = SSI_RX_CLEAR_FLAGS | SSI_TX_CLEAR_FLAGS;

    /* Drain anything the shift register captured from an aborted cycle. */
    (void)SSI_SLAVE_SPI->DR;
    (void)SSI_SLAVE_SPI->SR;

    SSI_TX_STREAM->M0AR = (uint32_t)(uintptr_t)s_tx;
    SSI_TX_STREAM->NDTR = s_nbytes;
    SSI_RX_STREAM->M0AR = (uint32_t)(uintptr_t)s_rx;
    SSI_RX_STREAM->NDTR = s_nbytes;
    s_last_ndtr = s_nbytes;

    SSI_RX_STREAM->CR |= DMA_SxCR_EN;
    SSI_TX_STREAM->CR |= DMA_SxCR_EN;

    SSI_SLAVE_SPI->CR1 |= SPI_CR1_SPE;

    /* The SPI drives MISO LOW whenever it holds the pin and no clock is
     * running (measured: PB4 reads 0 with the pin in AF mode and the slave
     * armed). SSI requires DATA to idle HIGH -- that level is what tells the
     * controller a new Read Cycle may start -- so the pin stays under GPIO
     * control until the master's first falling edge, and EXTI3 hands it to the
     * SPI at that edge. See EXTI3_IRQHandler. */
    data_drive(true);
    data_take_gpio();
    s_moder_af = (SSI_SLAVE_GPIO->MODER & ~DATA_MODER_MASK) | DATA_MODER_AF;

    EXTI->PR   = SSI_SLAVE_SCK_PIN;      /* discard any stale edge */
    EXTI->IMR |= SSI_SLAVE_SCK_PIN;
    s_armed = true;
}

/* --- init ----------------------------------------------------------------- */

static void gpio_init(void)
{
    GPIO_InitTypeDef io = {0};

    __HAL_RCC_GPIOB_CLK_ENABLE();

    /* CLOCK input. Idle state of the SSI clock is HIGH and CPOL=1 expects the
     * same, so pull up to keep a disconnected input in a defined idle state. */
    io.Pin       = SSI_SLAVE_SCK_PIN;
    io.Mode      = GPIO_MODE_AF_PP;
    io.Pull      = GPIO_PULLUP;
    io.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    io.Alternate = SSI_SLAVE_AF;
    HAL_GPIO_Init(SSI_SLAVE_GPIO, &io);

    /* MOSI is unused by SSI. It is configured so the receive shift register --
     * which is what counts clock edges for us -- sees a defined level. */
    io.Pin  = SSI_SLAVE_MOSI_PIN;
    io.Pull = GPIO_PULLDOWN;
    HAL_GPIO_Init(SSI_SLAVE_GPIO, &io);

    /* DATA output.
     *
     * The alternate function selector must be programmed while the pin is in
     * an AF mode: HAL_GPIO_Init() only writes GPIOx->AFR[] when Mode is one of
     * the AF modes. Initialising this pin directly as GPIO_MODE_OUTPUT_PP
     * would leave AFR at its reset value of 0, and AF0 on PB4 is NJTRST, not
     * SPI1_MISO -- so the MODER flip in data_give_spi() would hand the line to
     * the JTAG block and no data would ever come out.
     */
    io.Pin       = SSI_SLAVE_DATA_PIN;
    io.Mode      = GPIO_MODE_AF_PP;
    io.Pull      = GPIO_NOPULL;
    io.Alternate = SSI_SLAVE_AF;
    HAL_GPIO_Init(SSI_SLAVE_GPIO, &io);

    /* Now take it back to GPIO at the idle HIGH level. From here on only MODER
     * is touched, so AFR keeps pointing at SPI1_MISO. */
    data_drive(true);
    data_take_gpio();

    /* EXTI on the clock input, falling edge only. EXTI observes the pad, so it
     * works even though PB3 is in alternate-function mode for SPI1_SCK. It is
     * unmasked per Read Cycle by ssi_arm() and masked again by the handler
     * after the first edge, so it costs one interrupt per message. */
    __HAL_RCC_SYSCFG_CLK_ENABLE();
    SYSCFG->EXTICR[0] = (SYSCFG->EXTICR[0] & ~SYSCFG_EXTICR1_EXTI3)
                      | SYSCFG_EXTICR1_EXTI3_PB;
    EXTI->FTSR |=  SSI_SLAVE_SCK_PIN;
    EXTI->RTSR &= ~SSI_SLAVE_SCK_PIN;
    EXTI->IMR  &= ~SSI_SLAVE_SCK_PIN;

    /* Highest priority in the system: this handler must complete within half a
     * clock period (250 ns at the 2 MHz maximum) so that DATA is valid before
     * the master samples it on the following rising edge. */
    HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(EXTI3_IRQn);
}

static void spi_init(void)
{
    __HAL_RCC_SPI1_CLK_ENABLE();

    SSI_SLAVE_SPI->CR1 = 0;
    /* Slave (MSTR=0), CPOL=1 idle-high clock, CPHA=1 sample on the second
     * (rising) edge so DATA changes on the falling edge, 8-bit, MSB first.
     * SSM=1 with SSI=0 keeps the slave permanently selected: SSI has no chip
     * select line. */
    SSI_SLAVE_SPI->CR1 = SPI_CR1_CPOL | SPI_CR1_CPHA | SPI_CR1_SSM;
    SSI_SLAVE_SPI->CR2 = SPI_CR2_TXDMAEN | SPI_CR2_RXDMAEN;
}

static void dma_init(void)
{
    __HAL_RCC_DMA2_CLK_ENABLE();

    SSI_TX_STREAM->CR = 0;
    while (SSI_TX_STREAM->CR & DMA_SxCR_EN) { }
    SSI_TX_STREAM->PAR = (uint32_t)(uintptr_t)&SSI_SLAVE_SPI->DR;
    SSI_TX_STREAM->CR  = (SSI_DMA_CHANNEL << DMA_SxCR_CHSEL_Pos)
                       | DMA_SxCR_DIR_0        /* memory -> peripheral */
                       | DMA_SxCR_MINC
                       | DMA_SxCR_PL_1;        /* high priority */
    SSI_TX_STREAM->FCR = 0;

    SSI_RX_STREAM->CR = 0;
    while (SSI_RX_STREAM->CR & DMA_SxCR_EN) { }
    SSI_RX_STREAM->PAR = (uint32_t)(uintptr_t)&SSI_SLAVE_SPI->DR;
    SSI_RX_STREAM->CR  = (SSI_DMA_CHANNEL << DMA_SxCR_CHSEL_Pos)
                       | DMA_SxCR_MINC
                       | DMA_SxCR_PL_0 | DMA_SxCR_PL_1  /* very high */
                       | DMA_SxCR_TCIE;
    SSI_RX_STREAM->FCR = 0;

    HAL_NVIC_SetPriority(SSI_RX_IRQn, 0, 1);
    HAL_NVIC_EnableIRQ(SSI_RX_IRQn);
}

static void gap_timer_init(void)
{
    __HAL_RCC_TIM6_CLK_ENABLE();

    GAP_TIM->CR1 = 0;
    GAP_TIM->PSC = (APB1_TIMCLK_HZ / 1000000u) - 1u;   /* 1 us per tick */
    GAP_TIM->ARR = ((s_cfg.tmu_us - GAP_OVERHEAD_US) * GAP_TICK_PER_US) - 1u;
    GAP_TIM->EGR = TIM_EGR_UG;      /* load PSC/ARR */
    GAP_TIM->SR  = 0;               /* UG set UIF; drop it */
    GAP_TIM->CR1 = TIM_CR1_OPM;     /* one pulse: stops itself at update */
    GAP_TIM->DIER = TIM_DIER_UIE;

    HAL_NVIC_SetPriority(GAP_TIM_IRQn, 0, 2);
    HAL_NVIC_EnableIRQ(GAP_TIM_IRQn);
}

static inline void gap_timer_start(void)
{
    GAP_TIM->CNT = 0;
    GAP_TIM->SR  = 0;
    GAP_TIM->CR1 |= TIM_CR1_CEN;
}

bool ssi_slave_init(const ssi_slave_config_t *cfg,
                    ssi_slave_provider_t provider, void *ctx)
{
    if (cfg == NULL || cfg->n_bits == 0u || cfg->n_bits > 32u ||
        (cfg->n_bits % 8u) != 0u || cfg->tmu_us == 0u) {
        return false;
    }

    s_cfg      = *cfg;
    s_provider = provider;
    s_ctx      = ctx;
    s_nbytes   = (uint8_t)(cfg->n_bits / 8u);
    s_frames   = 0;
    s_resyncs  = 0;
    s_armed    = false;

    gpio_init();
    spi_init();
    dma_init();
    gap_timer_init();
    return true;
}

void ssi_slave_start(void)
{
    data_drive(true);
    data_take_gpio();
    stage_frame();
    ssi_arm();
}

void ssi_slave_poll(void)
{
    if (!s_armed) {
        return;
    }

    /* A healthy link either sits at NDTR == n (no cycle in progress) or races
     * through a message in at most n x 10 us. Anything else means the master
     * stopped clocking mid-message, which would leave the byte framing skewed
     * for every later cycle -- so drop it and re-arm. */
    uint16_t ndtr = (uint16_t)SSI_RX_STREAM->NDTR;

    if (ndtr != s_nbytes && ndtr == s_last_ndtr) {
        s_resyncs++;
        data_drive(true);
        data_take_gpio();
        stage_frame();
        ssi_arm();
    }
    s_last_ndtr = ndtr;
}

void ssi_slave_get_stats(ssi_slave_stats_t *out)
{
    out->frames  = s_frames;
    out->resyncs = s_resyncs;
}

/* --- interrupts ----------------------------------------------------------- */

/* First falling edge of a Read Cycle: hand the DATA line from GPIO (which was
 * holding the idle HIGH level) to the SPI, which has already shifted D(n-1)
 * onto its output. This must complete before the master samples on the
 * following rising edge -- half a clock period, 250 ns at 2 MHz -- so the
 * handler is three stores and nothing else.
 *
 * The interrupt is masked here and re-armed by ssi_arm(), so it costs exactly
 * one interrupt per message rather than one per clock. */
void EXTI3_IRQHandler(void)
{
    /* Read-modify-write only this pin's two MODER bits. Storing a whole
     * precomputed MODER word would be a couple of cycles faster, but it
     * republishes the mode of every other pin in the port as it was when the
     * frame was armed -- which silently reverted the test harness's clock pin
     * from GPIO back to alternate function on the first edge, killing the
     * clock mid-burst. Never write a shared register wholesale from an ISR. */
    SSI_SLAVE_GPIO->MODER =
        (SSI_SLAVE_GPIO->MODER & ~DATA_MODER_MASK) | DATA_MODER_AF;
    EXTI->IMR &= ~SSI_SLAVE_SCK_PIN;
    EXTI->PR   = SSI_SLAVE_SCK_PIN;
}

/* End of message: the receive DMA has counted all n clocks. */
void DMA2_Stream2_IRQHandler(void)
{
    if (DMA2->LISR & DMA_LISR_TCIF2) {
        DMA2->LIFCR = SSI_RX_CLEAR_FLAGS;

        /* Take the line back and present the Error Flag for the gap. */
        data_drive(s_gap_level);
        data_take_gpio();

        SSI_SLAVE_SPI->CR1 &= ~SPI_CR1_SPE;
        SSI_TX_STREAM->CR  &= ~DMA_SxCR_EN;
        SSI_RX_STREAM->CR  &= ~DMA_SxCR_EN;

        s_armed = false;
        s_frames++;

        gap_timer_start();
    } else {
        DMA2->LIFCR = SSI_RX_CLEAR_FLAGS;
    }
}

/* Tmu elapsed: latest position data is now available, DATA returns HIGH and a
 * new Read Cycle may start. */
void TIM6_DAC_IRQHandler(void)
{
    if (GAP_TIM->SR & TIM_SR_UIF) {
        GAP_TIM->SR = 0;

        data_drive(true);
        stage_frame();
        ssi_arm();
    }
}
