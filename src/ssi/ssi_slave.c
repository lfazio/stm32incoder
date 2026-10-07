#include "ssi/ssi_slave.h"
#include "board/board.h"

/* DMA2 stream 3: flags live in the LOW interrupt registers. */
#define SSI_DATA_CLEAR_FLAGS (DMA_LIFCR_CTCIF3 | DMA_LIFCR_CHTIF3 | \
                              DMA_LIFCR_CTEIF3 | DMA_LIFCR_CDMEIF3 | \
                              DMA_LIFCR_CFEIF3)

#define CLK_TIM         SSI_CLKIN_TIM      /* TIM8: APB2, so DMA2 reaches GPIO */

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

/* One BSRR word per message bit. 32 is the longest frame the variants use. */
static volatile uint32_t     s_bsrr[32];
/* One BSRR word per falling edge, n+1 of them: n no-ops then the Error Flag. */
static volatile uint32_t     s_ef[33];
static volatile bool         s_armed;
static volatile uint16_t     s_last_ndtr;
static volatile uint32_t     s_frames;
static volatile uint32_t     s_resyncs;
static volatile bool         s_gap_level;

/* Dynamic half-period correction.
 *
 * Tmu is specified "from last falling edge of clock", but the message ends on
 * the last *rising* edge -- half a clock period later. That term is 0.25 us at
 * the 2 MHz maximum but 5 us at the 100 kHz minimum, so a fixed compensation
 * only holds near the top of the range: below roughly 400 kHz a fixed value
 * pushes Tmu outside its 20 us +/- 1 us window.
 *
 * T comes free from the DATA DMA, which moves exactly one word per rising
 * edge. Its half-transfer event therefore fires a known number of clock
 * periods before transfer-complete, and both run in the same non-critical
 * handler:  T = (t_TC - t_HT) / (periods between them).
 */
/* Sticky: the counter has actually reached n and ended a message. Set from the
 * update handler, which is the only thing that proves it -- an earlier version
 * polled CNT != 0 instead and was wrong in both directions. It read "counting"
 * off 11 counts of noise on a floating input that never reached n, and once the
 * wire was on it read "no edges", because every re-arm resets CNT to 0 and the
 * poll almost never lands mid-message. The wire this needs is easy to leave
 * off, and without it the link is simply dead with nothing pointing at the
 * cause, so `stat` has to answer the question rather than approximate it. */
static volatile bool         s_clk_counted;

static volatile uint32_t     s_t_half;         /* DWT cycles at half transfer */
static volatile uint32_t     s_ht_bits;        /* bits already clocked in at that point */
static volatile uint32_t     s_period_ns;      /* measured clock period T */
static volatile bool         s_period_valid;

/* --- DATA line -------------------------------------------------------------
 * PB4 is a GPIO output throughout. Only the *writer* changes: software between
 * messages, DMA during one. There is no pin handover, so nothing has to happen
 * at the first falling edge and the first bit period is untouched. */
static inline void data_drive(bool level)
{
    SSI_SLAVE_GPIO->BSRR = level ? SSI_SLAVE_DATA_PIN
                                 : ((uint32_t)SSI_SLAVE_DATA_PIN << 16u);
}

/* --- frame staging -------------------------------------------------------- */

static void stage_frame(void)
{
    ssi_slave_frame_t f = { .payload = 0, .error_flag = false };

    if (s_provider != NULL) {
        s_provider(s_ctx, &f);
    }

    /* One BSRR word per bit, MSB first. Writing BSRR sets or resets the pin in
     * a single store with no read-modify-write, so the DMA needs no knowledge
     * of any other pin in the port -- which is the register-sharing trap that
     * bit us in EXTI3_IRQHandler, avoided here by construction. */
    for (uint8_t i = 0; i < s_cfg.n_bits; i++) {
        uint32_t bit = (f.payload >> (s_cfg.n_bits - 1u - i)) & 1u;
        s_bsrr[i] = bit ? (uint32_t)SSI_SLAVE_DATA_PIN
                        : ((uint32_t)SSI_SLAVE_DATA_PIN << 16u);
        s_ef[i] = 0u;              /* no-op: BSRR ignores a zero word */
    }
    /* The (n+1)'th falling edge ends the Read Cycle, and by then the controller
     * has sampled D0. Hand the line to the Error Flag there, from hardware, so
     * it does not wait for an interrupt. */
    s_ef[s_cfg.n_bits] = f.error_flag ? (uint32_t)SSI_SLAVE_DATA_PIN
                                      : ((uint32_t)SSI_SLAVE_DATA_PIN << 16u);

    /* Note 3 of section 5.4.1: after the last rising edge the data line is set
     * by the Error Flag. PV is the inverse of the ERROR FLAG, so a valid
     * measurement drives DATA low during the gap. */
    s_gap_level = f.error_flag;
}

/* --- arming --------------------------------------------------------------- */

static inline void clock_counter_arm(void);

/* Arm one message: the DATA DMA reloads, then the edge counter starts.
 *
 * There is no pin handover and no shift register here. PB4 stays a GPIO output
 * throughout; the only thing that changes is who writes it -- software during
 * the gap, the DMA during the message. So nothing has to happen at the first
 * falling edge, which is why this build has no EXTI3 handler and none of its
 * 250 ns deadline. The first rising edge writes D(n-1) by itself. */
static void ssi_arm(void)
{
    /* Silence the request source before touching the stream.
     *
     * TIM3 free-runs with ARR=0, so every counted edge is an update and every
     * update is a DMA request. Reloading the stream while that source is still
     * live leaves a request able to appear across the re-arm -- and a request
     * raised while the stream is disabled is simply lost, after which the
     * stream sits at its reload value and never moves again. That is the
     * observed failure: one good message, then NDTR parked at n with the
     * watchdog seeing an idle count and nothing to resync. Mask UDE first, and
     * unmask only once the stream is loaded and enabled. */
    CLK_TIM->CR1  &= ~TIM_CR1_CEN;
    CLK_TIM->DIER &= ~(TIM_DIER_CC2DE | TIM_DIER_CC4DE);
    CLK_TIM->SR    = 0;

    SSI_DATA_DMA_STREAM->CR &= ~DMA_SxCR_EN;
    while (SSI_DATA_DMA_STREAM->CR & DMA_SxCR_EN) { }
    DMA2->LIFCR = SSI_DATA_CLEAR_FLAGS;

    SSI_DATA_DMA_STREAM->M0AR = (uint32_t)(uintptr_t)s_bsrr;
    SSI_DATA_DMA_STREAM->NDTR = s_cfg.n_bits;
    SSI_DATA_DMA_STREAM->CR  |= DMA_SxCR_EN;

    SSI_EF_DMA_STREAM->CR   &= ~DMA_SxCR_EN;
    while (SSI_EF_DMA_STREAM->CR & DMA_SxCR_EN) { }
    DMA2->HIFCR = DMA_HIFCR_CTCIF7 | DMA_HIFCR_CHTIF7 | DMA_HIFCR_CTEIF7 |
                  DMA_HIFCR_CDMEIF7 | DMA_HIFCR_CFEIF7;
    SSI_EF_DMA_STREAM->M0AR = (uint32_t)(uintptr_t)s_ef;
    SSI_EF_DMA_STREAM->NDTR = (uint32_t)s_cfg.n_bits + 1u;
    SSI_EF_DMA_STREAM->CR  |= DMA_SxCR_EN;

    /* The gap level is already on the pin; leave it there until the first
     * rising edge overwrites it with D(n-1). */
    CLK_TIM->CNT   = 0;
    CLK_TIM->SR    = 0;
    CLK_TIM->DIER |= TIM_DIER_CC2DE | TIM_DIER_CC4DE;
    CLK_TIM->CR1  |= TIM_CR1_CEN;
    s_armed = true;
}

/* --- init ----------------------------------------------------------------- */

static void gpio_init(void)
{
    GPIO_InitTypeDef io = {0};

    __HAL_RCC_GPIOB_CLK_ENABLE();

    /* DATA output, and that is the whole of it.
     *
     * The pin is a plain GPIO output for the entire Read Cycle: the DMA writes
     * it through BSRR and nothing ever hands it to a peripheral, so there is no
     * alternate function to program and no mode to flip mid-frame. The clock is
     * not read here at all -- it clocks TIM8 through PC6, set up in
     * clock_input_init(). */
    io.Pin   = SSI_SLAVE_DATA_PIN;
    io.Mode  = GPIO_MODE_OUTPUT_PP;
    io.Pull  = GPIO_NOPULL;
    io.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(SSI_SLAVE_GPIO, &io);

    /* SSI idles DATA HIGH; that level is what tells the controller a new Read
     * Cycle may start. */
    data_drive(true);
}



/* TIM8 clocked by the SSI clock on TI1F_ED: one count per *edge*, ARR = 1, so
 * a channel-2 compare lands on every rising edge to drive the DATA DMA and a
 * channel-4 compare on every falling edge to place the Error Flag. */
static void clock_counter_init(void)
{
    GPIO_InitTypeDef io = {0};

    __HAL_RCC_GPIOC_CLK_ENABLE();
    io.Pin       = SSI_CLKIN_PIN;
    io.Alternate = SSI_CLKIN_AF;
    /* Pull up to match the SSI idle level, so a missing wire idles high rather
     * than floating and counting noise. It still will not count to n -- that is
     * the point of the check in ssi_slave_clock_counter_ok(). */
    io.Mode      = GPIO_MODE_AF_PP;
    io.Pull      = GPIO_PULLUP;
    io.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(SSI_CLKIN_GPIO, &io);

    __HAL_RCC_TIM8_CLK_ENABLE();

    CLK_TIM->CR1 = 0;
    CLK_TIM->PSC = 0;                      /* count every edge, not every Nth */
    /* Two edges per bit, so an update every second edge is one per bit -- and
     * because the clock idles HIGH the edges pair up (F1,R1), (F2,R2)..., which
     * puts every update on a rising edge.
     *
     * Never 0. RM0390: "The counter is blocked while the auto-reload value is
     * null" -- with ARR=0 the timer does not count at all, raises no events and
     * requests no DMA, while every register still reads back correct. */
    CLK_TIM->ARR = 1u;

    /* External clock mode 1 (SMS=111) with TS=100 = TI1F_ED, the channel-1
     * edge detector: it pulses on *both* edges of TI1, so the counter advances
     * at twice the bit rate. CC1S=01 maps the pin to IC1 so the detector sees
     * it; the capture value itself is never read, only the edges matter. */
    /* Channel 1 is the input the counter is clocked from; channel 2 is an
     * output compare left frozen (OC2M=000), used only for the event it raises.
     * CCR2=0 matches when the counter wraps 1 -> 0, which is the rising edge --
     * the same instant as the update event, but a compare event repeats where
     * the update event was measured to fire exactly once. */
    CLK_TIM->CCMR1 = TIM_CCMR1_CC1S_0;
    CLK_TIM->CCR2  = 0u;    /* counter at 0: a rising edge  -> next data bit */
    CLK_TIM->CCR4  = 1u;    /* counter at 1: a falling edge -> Error Flag slot */
    CLK_TIM->SMCR  = TIM_SMCR_SMS_0 | TIM_SMCR_SMS_1 | TIM_SMCR_SMS_2
                   | TIM_SMCR_TS_2;

    CLK_TIM->EGR = TIM_EGR_UG;             /* load PSC/ARR */
    CLK_TIM->SR  = 0;                      /* UG set UIF; drop it */
    CLK_TIM->DIER = TIM_DIER_CC2DE         /* one request per rising edge  */
                  | TIM_DIER_CC4DE;        /* one request per falling edge */

    /* Same priority as the receive DMA's end-of-message interrupt it replaces:
     * above everything except the EXTI3 handover, which it must never preempt
     * -- equal preemption priority, higher subpriority. */
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

static void gap_timer_start(void);

/* DMA1 Stream 2 Channel 5 = TIM3_UP [RM0390 Rev 9 Table 28], memory to
 * peripheral, one 32-bit word per clock edge into GPIOB->BSRR. */
static void data_dma_init(void)
{
    __HAL_RCC_DMA2_CLK_ENABLE();

    SSI_DATA_DMA_STREAM->CR = 0;
    while (SSI_DATA_DMA_STREAM->CR & DMA_SxCR_EN) { }
    SSI_DATA_DMA_STREAM->PAR = (uint32_t)(uintptr_t)SSI_DATA_BSRR;
    SSI_DATA_DMA_STREAM->CR  = (SSI_DATA_DMA_CHANNEL << DMA_SxCR_CHSEL_Pos)
                             | DMA_SxCR_DIR_0            /* memory -> periph */
                             | DMA_SxCR_MINC
                             | DMA_SxCR_PSIZE_1          /* 32-bit */
                             | DMA_SxCR_MSIZE_1
                             | DMA_SxCR_PL_0 | DMA_SxCR_PL_1  /* very high */
                             | DMA_SxCR_TCIE             /* end of message */
                             | DMA_SxCR_HTIE;           /* measures T */
    SSI_DATA_DMA_STREAM->FCR = 0;

    /* Error Flag stream: same shape, no interrupt -- nothing needs to know. */
    SSI_EF_DMA_STREAM->CR = 0;
    while (SSI_EF_DMA_STREAM->CR & DMA_SxCR_EN) { }
    SSI_EF_DMA_STREAM->PAR = (uint32_t)(uintptr_t)SSI_DATA_BSRR;
    SSI_EF_DMA_STREAM->CR  = (SSI_EF_DMA_CHANNEL << DMA_SxCR_CHSEL_Pos)
                           | DMA_SxCR_DIR_0
                           | DMA_SxCR_MINC
                           | DMA_SxCR_PSIZE_1 | DMA_SxCR_MSIZE_1
                           | DMA_SxCR_PL_0 | DMA_SxCR_PL_1;
    SSI_EF_DMA_STREAM->FCR = 0;

    HAL_NVIC_SetPriority(SSI_DATA_DMA_IRQn, 0, 1);
    HAL_NVIC_EnableIRQ(SSI_DATA_DMA_IRQn);
}

/* End of message: the DMA has written all n bits, so the last rising edge has
 * just happened. 5.4.1 note 3 puts the Error Flag here. */
void DMA2_Stream3_IRQHandler(void)
{
    /* Half transfer: timestamp only. One word per rising edge, so the interval
     * from here to transfer-complete is a whole number of clock periods. */
    if (DMA2->LISR & DMA_LISR_HTIF3) {
        DMA2->LIFCR = DMA_LIFCR_CHTIF3;
        s_t_half  = DWT->CYCCNT;
        /* How far the transfer actually got, rather than assuming half: an odd
         * n would not land on n/2. */
        s_ht_bits = (uint32_t)s_cfg.n_bits - (uint32_t)SSI_DATA_DMA_STREAM->NDTR;
    }

    if (DMA2->LISR & DMA_LISR_TCIF3) {
        uint32_t elapsed = DWT->CYCCNT - s_t_half;

        /* Stop the request source in the same breath as ending the message, so
         * nothing can be raised during the gap that survives into the re-arm. */
        CLK_TIM->CR1  &= ~TIM_CR1_CEN;
        CLK_TIM->DIER &= ~(TIM_DIER_CC2DE | TIM_DIER_CC4DE);

        /* The Error Flag is already on the line: the CC4 DMA put it there at
         * the last falling edge. Driving it here is exactly the interrupt
         * latency this replaced, and it is what made the gap open with a
         * spurious HIGH pulse whenever D0 was 1. */
        SSI_DATA_DMA_STREAM->CR &= ~DMA_SxCR_EN;
        SSI_EF_DMA_STREAM->CR   &= ~DMA_SxCR_EN;
        DMA2->LIFCR = SSI_DATA_CLEAR_FLAGS;
        /* The interval spans however many rising edges remained after the
         * half-transfer event, which s_ht_bits records exactly. */
        uint32_t span = (s_ht_bits > 0u && s_ht_bits < s_cfg.n_bits)
                      ? (uint32_t)(s_cfg.n_bits - s_ht_bits)
                      : (uint32_t)(s_cfg.n_bits / 2u);
        /* ns per cycle is 1000/180 = 5.56, so scale before dividing -- the
         * other order truncates to 5 and reports every period 10% short. */
        uint32_t per_ns = (elapsed * 1000u) / (span * (SYSCLK_HZ / 1000000u));
        if (per_ns >= 400u && per_ns <= 12000u) {   /* 2 MHz .. ~83 kHz */
            s_period_ns    = per_ns;
            s_period_valid = true;
        }

        s_armed = false;
        s_frames++;
        s_clk_counted = true;
        gap_timer_start();
    } else {
        DMA2->LIFCR = SSI_DATA_CLEAR_FLAGS;
    }
}

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
static void gap_timer_start(void)
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
    data_dma_init();
    clock_counter_init();
    gap_timer_init();
    return true;
}

void ssi_slave_stop(void)
{
    /* Park the transport so something else can borrow the DATA pin.
     *
     * The loopback continuity test drives PB4 and toggles the clock pin, and
     * the clock reaches PC6 -- so without this, TIM8 counts those edges and the
     * DATA DMA writes PB4 from under the test, which reports the wire OPEN when
     * it is fine. The old SPI path had the same hazard through EXTI3 and masked
     * it the same way. */
    CLK_TIM->CR1  &= ~TIM_CR1_CEN;
    CLK_TIM->DIER  = 0;
    CLK_TIM->SR    = 0;

    SSI_DATA_DMA_STREAM->CR &= ~DMA_SxCR_EN;
    SSI_EF_DMA_STREAM->CR   &= ~DMA_SxCR_EN;
    while ((SSI_DATA_DMA_STREAM->CR & DMA_SxCR_EN) ||
           (SSI_EF_DMA_STREAM->CR & DMA_SxCR_EN)) {
    }

    GAP_TIM->CR1 &= ~TIM_CR1_CEN;
    GAP_TIM->SR   = 0;

    s_armed = false;
}

void ssi_slave_start(void)
{
    data_drive(true);
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

    s_cfg.n_bits = n_bits;
    s_nbytes     = (uint8_t)(n_bits / 8u);

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
    uint16_t ndtr = (uint16_t)SSI_DATA_DMA_STREAM->NDTR;
    const uint16_t idle_ndtr = s_cfg.n_bits;

    if (ndtr != idle_ndtr && ndtr == s_last_ndtr) {
        s_resyncs++;
        data_drive(true);
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

/* End of message, whichever detector saw it: all n clocks have been counted.
 * Both callers run at the same instant -- the last rising edge -- so the gap
 * correction below is the same either way. */


/* Lives outside the handler guard above: with SIMENC_TIMER_DATA the DMA ends
 * the message and TIM3 raises no interrupt at all, but the question this
 * answers -- did the ETR wire ever carry a full message -- is the same. */
bool ssi_slave_clock_counter_ok(void)
{
    return s_clk_counted;
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
