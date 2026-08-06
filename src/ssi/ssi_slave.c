#include "ssi/ssi_slave.h"
#include "board/board.h"

/* Run the timing-critical handlers from SRAM rather than flash.
 *
 * The STM32F446 is a Cortex-M4 and has no TCM -- tightly-coupled memory is a
 * Cortex-M7 feature. The equivalent here is to place the code in SRAM: the
 * linker script already gathers .RamFunc inside the .data output section, so
 * the startup file's existing _sdata.._edata copy relocates it at reset with
 * no extra plumbing.
 *
 * Whether this actually helps is an empirical question, not an obvious win.
 * The ART accelerator gives zero-wait-state flash execution when its
 * instruction cache hits (RM0390 3.4.2), and a handler that runs every Read
 * Cycle is likely resident. Against that, SRAM instruction fetches use the
 * Cortex-M4 system bus, which also carries data and competes with the DMA
 * traffic this design leans on. Measure both ways before believing either.
 */
#if defined(SIMENC_RAMFUNC)
#define SSI_RAMFUNC __attribute__((section(".RamFunc"), noinline, long_call))
#else
#define SSI_RAMFUNC
#endif

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

/* End of message: counted clocks, or counted received bytes.
 *
 * By default the receive DMA's transfer-complete event marks end of message --
 * it counts clock edges for us, since every capture edge moves a bit. That is
 * free but it ties the frame length to whole bytes, and it ties end-of-message
 * detection to the SPI's own capture edge. Both are limits worth removing:
 *
 *   - SSI7 (n=30) and SSI8 (n=18) are not byte aligned, so the byte count
 *     cannot express them at all.
 *   - The clock-edge fix (see the README's KNOWN DEVIATION) needs SPE asserted
 *     only after the first falling edge, which costs the receive path one
 *     capture edge -- n-1 instead of n -- and that count is exactly what this
 *     detection uses.
 *
 * SIMENC_CLOCK_COUNTER moves the job to TIM3, counting clock edges on ETR with
 * ARR = n-1 so its update event lands on the nth. That counts the wire, not the
 * SPI, so it is indifferent to both.
 *
 * It fires on the last *rising* edge, where the receive DMA fires today, not
 * the last falling edge -- even though the falling edge is the reference Tmu is
 * specified from and ending there would delete the half-period correction
 * entirely. In the shipping CPOL=1 configuration the last data bit D0 is only
 * driven at that last falling edge, so reclaiming the line there would truncate
 * it. Same instant, same correction, different detector: that is the whole
 * intended change. Moving the reference is a separate step, and it only becomes
 * correct once DATA moves to the rising edge.
 *
 * NOT VALIDATED ON HARDWARE -- it needs a wire that the bench does not have
 * yet (see SSI_ETR_PIN), and without it the counter never counts and the link
 * stops dead. Hence opt-in, and hence the default is unchanged.
 */
#if defined(SIMENC_CLOCK_COUNTER)
#define CLK_TIM         SSI_ETR_TIM
#define CLK_TIM_IRQn    SSI_ETR_TIM_IRQn
#endif

/* Tmu gap timer, TIM6, a basic timer on APB1 (90 MHz with the APB1 prescaler
 * != 1). The gap it times is corrected for two things, because it does not
 * start at the reference the specification uses -- see the half-period note by
 * s_period_ns below, and GAP_ISR_OVERHEAD_NS.
 */
#define GAP_TIM         TIM6
#define GAP_TIM_IRQn    TIM6_DAC_IRQn

/* 0.1 us tick, so the half-period correction below can be applied with better
 * than microsecond resolution. */
#define GAP_TICK_NS     100u
#define GAP_TIM_HZ      (1000000000u / GAP_TICK_NS)

/* Fixed part of the correction: interrupt latency from the last rising edge to
 * gap_timer_start(), plus the latency from the gap timer's update to DATA
 * actually going high. Measured at ~1.6 us, from two captures that agree --
 * 20.89 us at 2 MHz (half period 0.25 us) and 20.85 us at 175.8 kHz (half
 * period 2.89 us), each against a 20 us target. */
#define GAP_ISR_OVERHEAD_NS  1600u

/* Lower bound, so a pathologically slow master cannot drive the gap to zero. */
#define GAP_MIN_NS           1000u

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

/* Dynamic half-period correction.
 *
 * Tmu is specified "from last falling edge of clock", but end of message is
 * detected by the receive DMA, which completes on the last *rising* edge --
 * half a clock period later. That term is 0.25 us at the 2 MHz maximum but
 * 5 us at the 100 kHz minimum, so a fixed compensation only holds near the top
 * of the range: below roughly 400 kHz a fixed value pushes Tmu outside its
 * 20 us +/- 1 us window.
 *
 * T is measured without touching EXTI3_IRQHandler, which has only ~26 ns of
 * margin. The receive DMA's half-transfer event fires exactly n_bits/2 clocks
 * before transfer-complete, and both run in non-critical handlers, so the two
 * cycle counts give T for free:  T = (t_TC - t_HT) / (n_bits/2).
 */
#if defined(SIMENC_CLOCK_COUNTER)
/* Sticky: the counter has actually reached n and ended a message. Set from the
 * update handler, which is the only thing that proves it -- an earlier version
 * polled CNT != 0 instead and was wrong in both directions. It read "counting"
 * off 11 counts of noise on a floating input that never reached n, and once the
 * wire was on it read "no edges", because every re-arm resets CNT to 0 and the
 * poll almost never lands mid-message. The wire this needs is easy to leave
 * off, and without it the link is simply dead with nothing pointing at the
 * cause, so `stat` has to answer the question rather than approximate it. */
static volatile bool         s_clk_counted;
#endif

static volatile uint32_t     s_t_half;         /* DWT cycles at half transfer */
static volatile uint32_t     s_ht_bits;        /* bits already clocked in at that point */
static volatile uint32_t     s_period_ns;      /* measured clock period T */
static volatile bool         s_period_valid;

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

static inline void spi_configure(void);
static inline void spi_reset(void);
#if defined(SIMENC_CLOCK_COUNTER)
static inline void clock_counter_arm(void);
#endif

static void ssi_arm(void)
{
    SSI_SLAVE_SPI->CR1 &= ~SPI_CR1_SPE;

    SSI_TX_STREAM->CR &= ~DMA_SxCR_EN;
    SSI_RX_STREAM->CR &= ~DMA_SxCR_EN;
    while ((SSI_TX_STREAM->CR & DMA_SxCR_EN) || (SSI_RX_STREAM->CR & DMA_SxCR_EN)) {
    }

    DMA2->LIFCR = SSI_RX_CLEAR_FLAGS | SSI_TX_CLEAR_FLAGS;

    /* Reset the peripheral rather than just disabling it, so no byte survives
     * from an aborted or edge-corrupted cycle. See spi_reset(). */
    spi_reset();

    SSI_TX_STREAM->M0AR = (uint32_t)(uintptr_t)s_tx;
    SSI_TX_STREAM->NDTR = s_nbytes;
    SSI_RX_STREAM->M0AR = (uint32_t)(uintptr_t)s_rx;
    SSI_RX_STREAM->NDTR = s_nbytes;
    s_last_ndtr = s_nbytes;

    SSI_RX_STREAM->CR |= DMA_SxCR_EN;
    SSI_TX_STREAM->CR |= DMA_SxCR_EN;

#if defined(SIMENC_CLOCK_COUNTER)
    clock_counter_arm();
#endif

#if !defined(SIMENC_RISING_EDGE)
    SSI_SLAVE_SPI->CR1 |= SPI_CR1_SPE;
#else
    /* Deferred to the first falling edge, in EXTI3_IRQHandler.
     *
     * With CPOL=0 the SPI's idle clock level is LOW, but the SSI clock idles
     * HIGH between messages. Enabling SPE here would arm the state machine
     * against a level it reads as already mid-bit, and the measured symptom was
     * every second Read Cycle coming out empty -- 32 clocks either way, but the
     * payload alternating with C0000000. Waiting until the clock has gone low
     * arms it against the level it expects.
     *
     * The reason this is affordable now: it costs the receive path one capture
     * edge, n-1 instead of n, and that count used to be what detected end of
     * message. With SIMENC_CLOCK_COUNTER counting the wire instead, nothing
     * depends on it. */
#endif

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

/* Slave (MSTR=0), 8-bit, MSB first. SSM=1 with SSI=0 keeps the slave
 * permanently selected: SSI has no chip select line.
 *
 * CPHA=1 always: the leading clock edge shifts a bit out, the trailing edge
 * samples. CPOL chooses which edge is which, and that is the whole difference
 * between the two builds:
 *
 *   CPOL=1  leading = falling.  DATA changes on the falling edge. Half a period
 *           early against the specification -- the known deviation.
 *   CPOL=0  leading = rising.   DATA changes on the rising edge, which is what
 *           5.4.1 note 2, POSITAL and RLS all describe.
 *
 * CPOL=0 also makes the SPI's idle level LOW, while the SSI clock idles HIGH.
 * That mismatch is why SPE cannot be asserted in the gap here -- see ssi_arm(). */
static inline void spi_configure(void)
{
#if defined(SIMENC_RISING_EDGE)
    SSI_SLAVE_SPI->CR1 = SPI_CR1_CPHA | SPI_CR1_SSM;
#else
    SSI_SLAVE_SPI->CR1 = SPI_CR1_CPOL | SPI_CR1_CPHA | SPI_CR1_SSM;
#endif
    SSI_SLAVE_SPI->CR2 = SPI_CR2_TXDMAEN | SPI_CR2_RXDMAEN;
}

/* Full peripheral reset through RCC->APB2RSTR.
 *
 * Clearing SPE is not enough to get a clean slate: it does not empty the
 * transmit buffer. A byte left there is shifted out ahead of the next frame,
 * so every following Read Cycle arrives one byte late -- and because each
 * re-arm simply queues four more bytes behind the stale one, the offset
 * persists until the peripheral is actually reset. That is what the loopback
 * continuity test used to trigger: it toggles the clock pin, the armed SPI
 * counts those edges, and a partial byte is stranded. */
static inline void spi_reset(void)
{
    __HAL_RCC_SPI1_FORCE_RESET();
    __HAL_RCC_SPI1_RELEASE_RESET();
    spi_configure();
}

static void spi_init(void)
{
    __HAL_RCC_SPI1_CLK_ENABLE();

    SSI_SLAVE_SPI->CR1 = 0;
    spi_configure();
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
#if !defined(SIMENC_CLOCK_COUNTER)
                       | DMA_SxCR_TCIE        /* end of message */
#endif
                       | DMA_SxCR_HTIE;       /* always: measures T */
    SSI_RX_STREAM->FCR = 0;

    HAL_NVIC_SetPriority(SSI_RX_IRQn, 0, 1);
    HAL_NVIC_EnableIRQ(SSI_RX_IRQn);
}

#if defined(SIMENC_CLOCK_COUNTER)
/* TIM3 counting SSI clock edges on ETR, so that end of message is the nth
 * edge on the wire rather than the nth byte through the SPI. */
static void clock_counter_init(void)
{
    GPIO_InitTypeDef io = {0};

    __HAL_RCC_GPIOD_CLK_ENABLE();

    /* Pull up to match the SSI idle level, so a missing wire idles high rather
     * than floating and counting noise. It still will not count -- that is the
     * point of the check in ssi_slave_clock_counter_ok(). */
    io.Pin       = SSI_ETR_PIN;
    io.Mode      = GPIO_MODE_AF_PP;
    io.Pull      = GPIO_PULLUP;
    io.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    io.Alternate = SSI_ETR_AF;
    HAL_GPIO_Init(SSI_ETR_GPIO, &io);

    __HAL_RCC_TIM3_CLK_ENABLE();

    CLK_TIM->CR1 = 0;
    CLK_TIM->PSC = 0;                      /* count every edge, not every Nth */
    CLK_TIM->ARR = (uint32_t)s_cfg.n_bits - 1u;

    /* External clock mode 2 (RM0390 17.3.12): ECE=1 clocks the counter from
     * ETRF. ETP=0 counts rising edges, which is where the receive DMA completes
     * today. No ETR prescaler and no filter: the filter samples ETR over
     * several clocks and would delay the update event, and the gap this event
     * starts is what has to land inside 20 us +/- 1 us. */
    CLK_TIM->SMCR = TIM_SMCR_ECE;

    CLK_TIM->EGR = TIM_EGR_UG;             /* load PSC/ARR */
    CLK_TIM->SR  = 0;                      /* UG set UIF; drop it */
    CLK_TIM->DIER = TIM_DIER_UIE;

    /* Same priority as the receive DMA's end-of-message interrupt it replaces:
     * above everything except the EXTI3 handover, which it must never preempt
     * -- equal preemption priority, higher subpriority. */
    HAL_NVIC_SetPriority(CLK_TIM_IRQn, 0, 1);
    HAL_NVIC_EnableIRQ(CLK_TIM_IRQn);
}

/* Restarts the count for one message. The counter must be zeroed rather than
 * left to free-run: anything that toggles the clock pin between messages --
 * the loopback continuity test, above all -- would otherwise carry a count
 * into the next frame and end it early. */
static inline void clock_counter_arm(void)
{
    CLK_TIM->CR1 &= ~TIM_CR1_CEN;
    CLK_TIM->CNT  = 0;
    CLK_TIM->SR   = 0;
    CLK_TIM->CR1 |= TIM_CR1_CEN;
}
#endif /* SIMENC_CLOCK_COUNTER */

static void gap_timer_init(void)
{
    __HAL_RCC_TIM6_CLK_ENABLE();

    GAP_TIM->CR1 = 0;
    GAP_TIM->PSC = (APB1_TIMCLK_HZ / GAP_TIM_HZ) - 1u;   /* 0.1 us per tick */
    GAP_TIM->ARR = (s_cfg.tmu_us * 1000u) / GAP_TICK_NS; /* replaced per frame */
    GAP_TIM->EGR = TIM_EGR_UG;      /* load PSC/ARR */
    GAP_TIM->SR  = 0;               /* UG set UIF; drop it */
    GAP_TIM->CR1 = TIM_CR1_OPM;     /* one pulse: stops itself at update */
    GAP_TIM->DIER = TIM_DIER_UIE;

    HAL_NVIC_SetPriority(GAP_TIM_IRQn, 0, 2);
    HAL_NVIC_EnableIRQ(GAP_TIM_IRQn);
}

/* Programs the gap so that DATA returns HIGH one Tmu after the last *falling*
 * edge, given the clock period measured during this frame. */
SSI_RAMFUNC static void gap_timer_start(void)
{
    uint32_t want = s_cfg.tmu_us * 1000u;
    uint32_t sub  = GAP_ISR_OVERHEAD_NS;

    if (s_period_valid) {
        sub += s_period_ns / 2u;          /* the half clock period */
    } else {
        sub += 250u;                      /* first frame: assume 2 MHz */
    }

    uint32_t ns = (want > sub + GAP_MIN_NS) ? (want - sub) : GAP_MIN_NS;

    GAP_TIM->ARR = (ns / GAP_TICK_NS) - 1u;
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

    /* Cycle counter, used to measure the master's clock period. */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    s_period_valid = false;

    gpio_init();
    spi_init();
    dma_init();
#if defined(SIMENC_CLOCK_COUNTER)
    clock_counter_init();
#endif
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

bool ssi_slave_set_frame_bits(uint8_t n_bits)
{
    if (n_bits == 0u || n_bits > 32u || (n_bits % 8u) != 0u) {
        return false;
    }

    /* Stop the gap timer first: its handler re-arms, and it must not fire
     * between the config change and the fresh arm below. */
    GAP_TIM->CR1 &= ~TIM_CR1_CEN;
    GAP_TIM->SR   = 0;
    EXTI->IMR    &= ~SSI_SLAVE_SCK_PIN;

    s_cfg.n_bits = n_bits;
    s_nbytes     = (uint8_t)(n_bits / 8u);
#if defined(SIMENC_CLOCK_COUNTER)
    CLK_TIM->CR1 &= ~TIM_CR1_CEN;
    CLK_TIM->ARR  = (uint32_t)n_bits - 1u;
    CLK_TIM->EGR  = TIM_EGR_UG;
    CLK_TIM->SR   = 0;
#endif

    ssi_slave_start();
    return true;
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
    out->frames    = s_frames;
    out->resyncs   = s_resyncs;
    out->period_ns = s_period_valid ? s_period_ns : 0u;
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
SSI_RAMFUNC void EXTI3_IRQHandler(void)
{
    /* Read-modify-write only this pin's two MODER bits. Storing a whole
     * precomputed MODER word would be a couple of cycles faster, but it
     * republishes the mode of every other pin in the port as it was when the
     * frame was armed -- which silently reverted the test harness's clock pin
     * from GPIO back to alternate function on the first edge, killing the
     * clock mid-burst. Never write a shared register wholesale from an ISR. */
    SSI_SLAVE_GPIO->MODER =
        (SSI_SLAVE_GPIO->MODER & ~DATA_MODER_MASK) | DATA_MODER_AF;
#if defined(SIMENC_RISING_EDGE)
    /* The clock is low now, which is CPOL=0's idle level, so this is the first
     * moment the state machine can be armed correctly. It has to happen before
     * the rising edge that follows -- half a period, 250 ns at 2 MHz -- because
     * that edge is when D(n-1) is due on the pin. This is the extra work the
     * rising-edge build puts on the tightest path in the design; measure the
     * handover before trusting it at 2 MHz. */
    SSI_SLAVE_SPI->CR1 |= SPI_CR1_SPE;
#endif
    EXTI->IMR &= ~SSI_SLAVE_SCK_PIN;
    EXTI->PR   = SSI_SLAVE_SCK_PIN;
}

/* End of message, whichever detector saw it: all n clocks have been counted.
 * Both callers run at the same instant -- the last rising edge -- so the gap
 * correction below is the same either way. */
static void end_of_message(void)
{
    /* Read the cycle counter before anything else, in particular before the
     * APB write that clears the DMA flags: that write can stall, and folding
     * the stall into the measured interval inflates T, which shortens the gap
     * by half the error. */
    uint32_t elapsed = DWT->CYCCNT - s_t_half;

    DMA2->LIFCR = SSI_RX_CLEAR_FLAGS;

    /* The interval spans however many clocks remained after the half-transfer
     * event, which s_ht_bits records exactly. Guard against a wildly
     * out-of-range value from an aborted cycle before trusting it. */
    uint32_t span = (s_ht_bits > 0u && s_ht_bits < s_cfg.n_bits)
                  ? (uint32_t)(s_cfg.n_bits - s_ht_bits)
                  : (uint32_t)(s_cfg.n_bits / 2u);
    /* ns per cycle is 1000/180 = 5.56, so scale before dividing -- doing it
     * the other way truncates to 5 and reports every period 10% short.
     * Worst case 16 periods at 100 kHz is 288000 cycles, so x1000 still
     * fits comfortably in 32 bits. */
    uint32_t per_ns = (elapsed * 1000u) / (span * (SYSCLK_HZ / 1000000u));
    if (per_ns >= 400u && per_ns <= 12000u) {   /* 2 MHz .. ~83 kHz */
        s_period_ns    = per_ns;
        s_period_valid = true;
    }

    /* Take the line back and present the Error Flag for the gap. */
    data_drive(s_gap_level);
    data_take_gpio();

    SSI_SLAVE_SPI->CR1 &= ~SPI_CR1_SPE;
    SSI_TX_STREAM->CR  &= ~DMA_SxCR_EN;
    SSI_RX_STREAM->CR  &= ~DMA_SxCR_EN;

    s_armed = false;
    s_frames++;

    gap_timer_start();
}

/* The receive DMA. It always provides the half-transfer timestamp that gives
 * the measured clock period T; it marks end of message only when the clock
 * counter is not built in, in which case its TC interrupt is not even enabled
 * and TIM3 owns that job. */
void DMA2_Stream2_IRQHandler(void)
{
    /* Half transfer: the rest of the clocks are still to come. Timestamp only. */
    if (DMA2->LISR & DMA_LISR_HTIF2) {
        DMA2->LIFCR = DMA_LIFCR_CHTIF2;
        s_t_half = DWT->CYCCNT;
        /* Read how far the transfer actually got rather than assuming half.
         * For an odd byte count -- n=24 is three bytes -- the half-transfer
         * event does not land on n_bits/2, and assuming it does under-measures
         * T and leaves the gap long. */
        s_ht_bits = (uint32_t)(s_nbytes - (uint8_t)SSI_RX_STREAM->NDTR) * 8u;
    }

    if (DMA2->LISR & DMA_LISR_TCIF2) {
#if defined(SIMENC_CLOCK_COUNTER)
        DMA2->LIFCR = SSI_RX_CLEAR_FLAGS;   /* TIM3 ends the message, not this */
#else
        end_of_message();                   /* clears the flags itself */
#endif
    } else {
        DMA2->LIFCR = SSI_RX_CLEAR_FLAGS;
    }
}

#if defined(SIMENC_CLOCK_COUNTER)
/* End of message: TIM3 has counted the nth clock edge on the wire. */
void TIM3_IRQHandler(void)
{
    if (CLK_TIM->SR & TIM_SR_UIF) {
        CLK_TIM->SR   = 0;
        CLK_TIM->CR1 &= ~TIM_CR1_CEN;
        s_clk_counted = true;
        end_of_message();
    }
}

bool ssi_slave_clock_counter_ok(void)
{
    return s_clk_counted;
}
#endif

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
