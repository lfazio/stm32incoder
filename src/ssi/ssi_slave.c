#include "ssi/ssi_slave.h"
#include "board/board.h"

/* DMA2 stream 3: flags live in the LOW interrupt registers. */
#define SSI_DATA_CLEAR_FLAGS (DMA_LIFCR_CTCIF3 | DMA_LIFCR_CHTIF3 | \
                              DMA_LIFCR_CTEIF3 | DMA_LIFCR_CDMEIF3 | \
                              DMA_LIFCR_CFEIF3)
#define SSI_DATA_IRQ_ENABLES (DMA_SxCR_TCIE | DMA_SxCR_HTIE)

/* DMA2 stream 7: flags live in the HIGH interrupt registers. */
#define SSI_TMU_CLEAR_FLAGS  (DMA_HIFCR_CTCIF7 | DMA_HIFCR_CHTIF7 | \
                              DMA_HIFCR_CTEIF7 | DMA_HIFCR_CDMEIF7 | \
                              DMA_HIFCR_CFEIF7)

#define CLK_TIM         SSI_CLKIN_TIM      /* TIM8: APB2, so DMA2 reaches GPIO */

/* Tmu timer, TIM6, a basic timer on APB1 (90 MHz with the APB1 prescaler
 * != 1). Every falling edge of the SSI clock resets it in hardware -- see
 * tmu_init() -- so its overflow means exactly what 5.4.1 defines Tmu as: the
 * time since the *last falling edge*, however many clocks the master sent. */
#define TMU_TIM         TIM6
#define TMU_TIM_IRQn    TIM6_DAC_IRQn

#define TMU_TICK_NS     100u
#define TMU_TIM_HZ      (1000000000u / TMU_TICK_NS)

/* From the Tmu overflow to DATA actually going HIGH: interrupt entry, staging
 * the next frame and re-arming, all of which happen *before* DATA goes HIGH so
 * that a master starting the moment it sees HIGH finds the slave ready.
 *
 * It is not a constant -- staging SSI6 runs a CRC that SSI4 does not -- so the
 * handler reads it off the Tmu counter on every Read Cycle and takes it out of
 * the next one. This is only the value used until the first measurement:
 * `stat` showed tmu_arm = 4800 ns in 9 samples of 10 and 5300 ns in the tenth,
 * SSI4 against a 562.5 kHz master. */
#define TMU_ARM_SEED_NS 4800u

static ssi_slave_config_t    s_cfg;
static ssi_slave_provider_t  s_provider;
static void                 *s_ctx;

/* One BSRR word per message bit, then one for the Error Flag. 32 is the
 * longest frame the variants use. */
#define SSI_MAX_BITS    32u
static volatile uint32_t     s_bsrr[SSI_MAX_BITS + 1u];

/* What the falling-edge DMA writes into TIM6->EGR. In SRAM, not flash, so the
 * DMA's read of it does not wait on flash wait states. */
static uint32_t              s_tmu_ug = TIM_EGR_UG;
static volatile uint16_t     s_tmu_arr;        /* Tmu less the arming time, in ticks, minus one */

static volatile uint32_t     s_frames;         /* n bits and the Error Flag clocked */
static volatile uint32_t     s_short;          /* exactly n clocks */
static volatile uint32_t     s_aborted;        /* fewer than n */
static volatile uint32_t     s_tmu_arm_ticks;  /* last overflow -> DATA HIGH */

/* Idle-frame refresh, see ssi_slave_refresh(). s_arm_gen moves on every
 * ssi_arm(), so a refresh built against one arm is never copied into the
 * buffer of the next. s_running is false while the transport is parked. */
static volatile uint32_t     s_arm_gen;
static volatile bool         s_running;
static volatile uint32_t     s_refreshed;      /* idle frame rebuilt from a new sample */
static volatile uint32_t     s_refresh_skipped;/* Read Cycle already under way */

/* Sticky: the counter has actually reached n and ended a message. Set from the
 * end-of-Read-Cycle handler, which is the only thing that proves it -- an
 * earlier version polled CNT != 0 instead and was wrong in both directions. It
 * read "counting" off 11 counts of noise on a floating input that never reached
 * n, and once the wire was on it read "no edges", because every re-arm resets
 * CNT to 0 and the poll almost never lands mid-message. The wire this needs is
 * easy to leave off, and without it the link is simply dead with nothing
 * pointing at the cause, so `stat` has to answer the question rather than
 * approximate it. */
static volatile bool         s_clk_counted;

/* Clock period, for `stat` only -- nothing in the timing depends on it any
 * more, since Tmu is now timed from the falling edge itself.
 *
 * T comes free from the DATA DMA, which moves exactly one word per rising
 * edge. Its half-transfer event therefore fires a known number of clock
 * periods before transfer-complete:  T = (t_TC - t_HT) / (periods between).
 * Only a master that clocks all n+1 bits reaches TC, so a master that stops at
 * n leaves this at its last value. */
static volatile uint32_t     s_t_half;         /* DWT cycles at half transfer */
static volatile uint32_t     s_ht_bits;        /* words already moved at that point */
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

static inline uint32_t bsrr_word(bool level)
{
    return level ? (uint32_t)SSI_SLAVE_DATA_PIN
                 : ((uint32_t)SSI_SLAVE_DATA_PIN << 16u);
}

/* --- frame staging -------------------------------------------------------- */

/* Fetches a frame from the provider and expands it into n+1 BSRR words. */
static void build_frame(volatile uint32_t *buf, uint8_t n_bits)
{
    ssi_slave_frame_t f = { .payload = 0, .error_flag = false };

    if (s_provider != NULL) {
        s_provider(s_ctx, &f);
    }

    /* One BSRR word per bit, MSB first. Writing BSRR sets or resets the pin in
     * a single store with no read-modify-write, so the DMA needs no knowledge
     * of any other pin in the port -- which is the register-sharing trap that
     * bit us in EXTI3_IRQHandler, avoided here by construction. */
    for (uint8_t i = 0; i < n_bits; i++) {
        buf[i] = bsrr_word(((f.payload >> (n_bits - 1u - i)) & 1u) != 0u);
    }

    /* The Error Flag is simply bit n+1.
     *
     * 5.4.1 note 3 sets the data line to the Error Flag "after the last rising
     * edge of the clock sequence", and the Read Cycle runs Trc = n*T + 0.5*T
     * from the first falling edge -- n+1 clock periods, so the last rising edge
     * is R(n+1). Each rising edge transmits the next bit (note 2), so the same
     * DMA that shifted D0 out on R(n) puts the flag out on R(n+1), +56 ns like
     * every other bit, with no interrupt. D0 stays on the line through F(n+1),
     * where the master samples it.
     *
     * PV is the inverse of the ERROR FLAG, so a valid measurement drives DATA
     * LOW for the rest of Tmu. */
    buf[n_bits] = bsrr_word(f.error_flag);
}

/* Only ever called while the DATA DMA is idle: from the Tmu handler before
 * ssi_arm(), and from ssi_slave_start(). */
static void stage_frame(void)
{
    build_frame(s_bsrr, s_cfg.n_bits);
}

/* --- arming --------------------------------------------------------------- */

/* Stop the DATA DMA without it reporting an end of message.
 *
 * Clearing EN on a live memory-to-peripheral stream sets TCIF [RM0390 Rev 9
 * 9.3.14 and 9.3.15, p.216]. With TCIE on, every re-arm therefore raised a
 * transfer-complete interrupt that the handler took for the end of a real
 * message. TCIE and HTIE are not protected bits [RM0390 Rev 9 DMA_SxCR,
 * p.226], so they go off in the same write as EN, and anything already pended
 * goes with them. */
static void data_dma_halt(void)
{
    SSI_DATA_DMA_STREAM->CR &= ~(DMA_SxCR_EN | SSI_DATA_IRQ_ENABLES);
    while (SSI_DATA_DMA_STREAM->CR & DMA_SxCR_EN) { }
    DMA2->LIFCR = SSI_DATA_CLEAR_FLAGS;
    NVIC_ClearPendingIRQ(SSI_DATA_DMA_IRQn);
}

/* Arm one message: the DATA DMA reloads, then the edge counter starts.
 *
 * There is no pin handover and no shift register here. PB4 stays a GPIO output
 * throughout; the only thing that changes is who writes it -- software between
 * Read Cycles, the DMA during one. The first rising edge writes D(n-1) by
 * itself.
 *
 * The data request is masked before the stream is touched: a request raised
 * while the stream is disabled is lost, after which the stream sits at its
 * reload value and never moves again. Unmask only once the stream is loaded
 * and enabled. The falling-edge request (CC4) is left alone -- it only ever
 * resets the Tmu timer. */
static void ssi_arm(void)
{
    CLK_TIM->CR1  &= ~TIM_CR1_CEN;
    CLK_TIM->DIER &= ~TIM_DIER_CC2DE;
    CLK_TIM->SR    = 0;

    data_dma_halt();
    s_ht_bits = 0;      /* no half-transfer seen in this message yet */

    SSI_DATA_DMA_STREAM->M0AR = (uint32_t)(uintptr_t)s_bsrr;
    SSI_DATA_DMA_STREAM->NDTR = (uint32_t)s_cfg.n_bits + 1u;
    SSI_DATA_DMA_STREAM->CR  |= SSI_DATA_IRQ_ENABLES | DMA_SxCR_EN;

    CLK_TIM->CNT   = 0;
    CLK_TIM->SR    = 0;
    CLK_TIM->DIER |= TIM_DIER_CC2DE | TIM_DIER_CC4DE;
    CLK_TIM->CR1  |= TIM_CR1_CEN;

    s_arm_gen++;
}

/* Park the Tmu timer between Read Cycles.
 *
 * TIM6 runs continuously and every falling edge resets it, so between Read
 * Cycles -- no edges at all -- it would overflow every Tmu for nothing. Parking
 * puts a 0xFFFF period in the shadow register with UG, then leaves Tmu in the
 * preload. The first falling edge of the next Read Cycle writes UG again, which
 * resets the counter *and* loads that Tmu [RM0390 Rev 9 19.4.1 ARPE and 19.4.5
 * UG, p.624-627], so the cycle is timed exactly while an idle line costs one
 * interrupt every 6.5 ms. URS=1 keeps both UGs from raising an interrupt. */
static void tmu_park(void)
{
    TMU_TIM->ARR = 0xFFFFu;
    TMU_TIM->EGR = TIM_EGR_UG;
    TMU_TIM->ARR = s_tmu_arr;
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
     * clock_counter_init(). */
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
 * channel-4 compare on every falling edge to reset the Tmu timer. */
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
    /* Channels 2 and 4 are output compares left frozen (OCxM=000), used only
     * for the events they raise -- a compare event repeats where the update
     * event was measured to fire exactly once. CCR2=0 matches when the counter
     * wraps 1 -> 0, the rising edge; CCR4=1 matches when it steps 0 -> 1, the
     * falling edge. */
    CLK_TIM->CCMR1 = TIM_CCMR1_CC1S_0;
    CLK_TIM->CCMR2 = 0u;
    CLK_TIM->CCR2  = 0u;    /* counter at 0: a rising edge  -> next data bit */
    CLK_TIM->CCR4  = 1u;    /* counter at 1: a falling edge -> Tmu restarts */
    CLK_TIM->SMCR  = TIM_SMCR_SMS_0 | TIM_SMCR_SMS_1 | TIM_SMCR_SMS_2
                   | TIM_SMCR_TS_2;

    CLK_TIM->EGR = TIM_EGR_UG;             /* load PSC/ARR */
    CLK_TIM->SR  = 0;                      /* UG set UIF; drop it */
    CLK_TIM->DIER = 0;                     /* ssi_arm() enables the requests */
}

/* DMA2 Stream 3 Channel 7 = TIM8_CH2 [RM0390 Rev 9 Table 29], memory to
 * peripheral, one 32-bit word per rising edge into GPIOB->BSRR. */
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

    HAL_NVIC_SetPriority(SSI_DATA_DMA_IRQn, 0, 1);
    HAL_NVIC_EnableIRQ(SSI_DATA_DMA_IRQn);
}

/* Tmu, timed from the last falling edge by hardware.
 *
 * The previous gap timer started from the end of the message and corrected for
 * where it thought the last falling edge was. That broke as soon as a master
 * sent n+1 clocks -- which 5.4.1 requires -- and would break further for any
 * that clocks a whole number of bytes past it, as an SPI master does.
 *
 * Here the falling edges themselves restart Tmu: TIM8's channel-4 compare
 * raises a DMA request on every falling edge, and a circular one-word stream
 * writes UG into TIM6->EGR, which resets its counter [RM0390 Rev 9 19.4.5].
 * TIM6 is on APB1, which the DMA2 peripheral bus reaches [RM0390 Rev 9 2.1.5,
 * p.55]. When no falling edge has come for Tmu, TIM6 overflows and its handler
 * ends the Read Cycle. No interrupt runs per edge.
 *
 * TIM8_CH4 -> DMA2 Stream 7, Channel 7 [RM0390 Rev 9 Table 29]. */
static void tmu_init(void)
{
    __HAL_RCC_TIM6_CLK_ENABLE();

    uint32_t ticks = ((uint32_t)s_cfg.tmu_us * 1000u - TMU_ARM_SEED_NS) / TMU_TICK_NS;
    s_tmu_arr = (uint16_t)(ticks - 1u);

    TMU_TIM->CR1  = 0;
    TMU_TIM->PSC  = (APB1_TIMCLK_HZ / TMU_TIM_HZ) - 1u;   /* 0.1 us per tick */
    TMU_TIM->CR1  = TIM_CR1_ARPE | TIM_CR1_URS;
    tmu_park();                     /* the UG in it loads PSC as well */
    TMU_TIM->SR   = 0;
    TMU_TIM->DIER = TIM_DIER_UIE;

    SSI_TMU_DMA_STREAM->CR = 0;
    while (SSI_TMU_DMA_STREAM->CR & DMA_SxCR_EN) { }
    DMA2->HIFCR = SSI_TMU_CLEAR_FLAGS;
    SSI_TMU_DMA_STREAM->PAR  = (uint32_t)(uintptr_t)&TMU_TIM->EGR;
    SSI_TMU_DMA_STREAM->M0AR = (uint32_t)(uintptr_t)&s_tmu_ug;
    SSI_TMU_DMA_STREAM->NDTR = 1u;
    SSI_TMU_DMA_STREAM->FCR  = 0;
    SSI_TMU_DMA_STREAM->CR   = (SSI_TMU_DMA_CHANNEL << DMA_SxCR_CHSEL_Pos)
                             | DMA_SxCR_DIR_0            /* memory -> periph */
                             | DMA_SxCR_CIRC             /* every falling edge */
                             | DMA_SxCR_PSIZE_1          /* 32-bit */
                             | DMA_SxCR_MSIZE_1
                             | DMA_SxCR_PL_1;            /* high: below DATA */
    SSI_TMU_DMA_STREAM->CR  |= DMA_SxCR_EN;

    HAL_NVIC_SetPriority(TMU_TIM_IRQn, 0, 2);
    HAL_NVIC_EnableIRQ(TMU_TIM_IRQn);
}

/* DATA has carried all n bits and then the Error Flag: R(n+1) has passed. Only
 * the clock period is measured here; the Read Cycle ends when Tmu says so. */
void DMA2_Stream3_IRQHandler(void)
{
    /* Half transfer: timestamp only. One word per rising edge, so the interval
     * from here to transfer-complete is a whole number of clock periods. */
    if (DMA2->LISR & DMA_LISR_HTIF3) {
        DMA2->LIFCR = DMA_LIFCR_CHTIF3;
        s_t_half  = DWT->CYCCNT;
        /* How far the transfer actually got, rather than assuming half: an odd
         * count would not land on the middle. */
        s_ht_bits = (uint32_t)s_cfg.n_bits + 1u - (uint32_t)SSI_DATA_DMA_STREAM->NDTR;
    }

    if (DMA2->LISR & DMA_LISR_TCIF3) {
        uint32_t elapsed = DWT->CYCCNT - s_t_half;
        uint32_t words   = (uint32_t)s_cfg.n_bits + 1u;

        /* Stop the data requests. TIM8 keeps counting, so any further clocks
         * still restart Tmu through channel 4. */
        CLK_TIM->DIER &= ~TIM_DIER_CC2DE;
        DMA2->LIFCR = SSI_DATA_CLEAR_FLAGS;

        /* ssi_arm() zeroes s_ht_bits, so a message whose half-transfer was
         * never seen -- or was served together with TC -- keeps the last good
         * T rather than dividing a stale timestamp. */
        if (s_ht_bits > 0u && s_ht_bits < words) {
            uint32_t span = words - s_ht_bits;
            /* ns per cycle is 1000/180 = 5.56, so scale before dividing -- the
             * other order truncates to 5 and reports every period 10% short. */
            uint32_t per_ns = (elapsed * 1000u) / (span * (SYSCLK_HZ / 1000000u));
            if (per_ns >= 400u && per_ns <= 12000u) {   /* 2 MHz .. ~83 kHz */
                s_period_ns    = per_ns;
                s_period_valid = true;
            }
        }
    } else {
        DMA2->LIFCR = SSI_DATA_CLEAR_FLAGS;
    }
}

/* Tmu has elapsed since the last falling edge: the Read Cycle is over, however
 * many clocks it had. Latest position data becomes available and DATA returns
 * HIGH (5.4.1 note 4). */
void TIM6_DAC_IRQHandler(void)
{
    if ((TMU_TIM->SR & TIM_SR_UIF) == 0u) {
        return;
    }
    TMU_TIM->SR = 0;

    const uint32_t words = (uint32_t)s_cfg.n_bits + 1u;
    const uint32_t left  = SSI_DATA_DMA_STREAM->NDTR;

    if (left == words && CLK_TIM->CNT == 0u) {
        /* Not a single edge since the last arm: the parked overflow. */
        tmu_park();
        return;
    }

    /* Classify by how far the DATA DMA got. A master that sends only n clocks
     * gets D0 out but never clocks the Error Flag, and its first sample -- at
     * F1 or right at R1 -- was the idle level, not D(n-1). It is worth
     * counting separately because it decodes as a frame shifted by one bit. */
    if (left == 0u) {
        s_frames++;
    } else if (left == 1u) {
        s_short++;
    } else {
        s_aborted++;
    }
    if (left <= 1u) {
        s_clk_counted = true;
    }

    /* Ready before DATA says so: a master may start the next Read Cycle the
     * moment it sees HIGH. */
    stage_frame();
    ssi_arm();

    /* The counter has been running since the overflow, so it now holds what
     * the two calls above cost. Take that out of the next Read Cycle's Tmu.
     * Bounded, so one freak reading cannot collapse the gap. */
    const uint32_t arm = TMU_TIM->CNT;
    const uint32_t tmu = ((uint32_t)s_cfg.tmu_us * 1000u) / TMU_TICK_NS;
    s_tmu_arm_ticks = arm;
    if (arm < tmu / 2u) {
        s_tmu_arr = (uint16_t)(tmu - arm - 1u);
    }

    tmu_park();
    data_drive(true);
}

bool ssi_slave_init(const ssi_slave_config_t *cfg,
                    ssi_slave_provider_t provider, void *ctx)
{
    if (cfg == NULL || cfg->n_bits == 0u || cfg->n_bits > SSI_MAX_BITS ||
        (cfg->n_bits % 8u) != 0u || cfg->tmu_us == 0u) {
        return false;
    }

    s_cfg      = *cfg;
    s_provider = provider;
    s_ctx      = ctx;
    s_frames   = 0;
    s_short    = 0;
    s_aborted  = 0;
    s_refreshed       = 0;
    s_refresh_skipped = 0;

    /* Cycle counter, used to measure the master's clock period. */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    s_period_valid = false;

    gpio_init();
    data_dma_init();
    clock_counter_init();
    tmu_init();
    return true;
}

void ssi_slave_stop(void)
{
    /* Park the transport so something else can borrow the DATA pin.
     *
     * The loopback continuity test drives PB4 and toggles the clock pin, and
     * the clock reaches PC6 -- so without this, TIM8 counts those edges and the
     * DATA DMA writes PB4 from under the test, which reports the wire OPEN when
     * it is fine. */
    s_running = false;      /* first: no refresh may touch the buffer from here */
    CLK_TIM->CR1  &= ~TIM_CR1_CEN;
    CLK_TIM->DIER  = 0;
    CLK_TIM->SR    = 0;

    data_dma_halt();

    TMU_TIM->CR1 &= ~TIM_CR1_CEN;
    TMU_TIM->SR   = 0;
    NVIC_ClearPendingIRQ(TMU_TIM_IRQn);
}

void ssi_slave_start(void)
{
    stage_frame();
    ssi_arm();
    tmu_park();
    TMU_TIM->SR   = 0;
    TMU_TIM->CR1 |= TIM_CR1_CEN;
    data_drive(true);
    s_running = true;
}

/* Rebuild the armed frame from the latest sample, if no Read Cycle has begun.
 *
 * Without this a new measurement reached the line only at the end of the
 * *next* Read Cycle's Tmu, so the frame a master read carried the sample that
 * was current when its *previous* read ended: as old as the master's idle
 * time, without bound. The first read after reset reported PV=0 however long
 * the board had been up, and `burst` needed warm-up cycles to see a `fixed`
 * value. The Tmu handler still stages at the end of every Read Cycle (5.4.1
 * note 4); this keeps the idle frame current in between.
 *
 * The frame is built into a shadow outside the critical section, so that the
 * provider's cost -- a CRC for SSI6 -- never runs with interrupts masked. Only
 * the test and the copy are atomic: about 7 cycles a word, 1.3 us for SSI4.
 *
 * Why the copy cannot be torn by a master that starts right after the test:
 *  - TIM8 CC4IF is the "a Read Cycle has begun" flag. ssi_arm() clears SR, and
 *    the only thing that sets CC4IF afterwards is the counter stepping 0 -> 1,
 *    which is F1. CNT cannot serve: it returns to 0 on every rising edge.
 *  - If F1 lands just after the test, the DMA reads word k on R(k+1), at least
 *    T/2 + k*T after F1 -- 250 + 500k ns at 2 MHz, the fastest clock 5.4.1
 *    allows. The copy writes word k about 40k ns after the test, word 0
 *    first, with interrupts masked so nothing can stall it. It stays ahead.
 *  - A Tmu overflow that ends a real Read Cycle cannot fall inside the
 *    critical section, because the copy only happens when no edge has come.
 *    Only the parked overflow can be delayed, and it does nothing.
 * The generation and n_bits checks reject a shadow built before a re-arm or a
 * variant change. */
void ssi_slave_refresh(void)
{
    const uint32_t gen = s_arm_gen;
    const uint8_t  n   = s_cfg.n_bits;
    uint32_t       shadow[SSI_MAX_BITS + 1u];

    if (!s_running) {
        return;
    }

    build_frame(shadow, n);

    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    if (s_running && gen == s_arm_gen && n == s_cfg.n_bits &&
        (CLK_TIM->SR & TIM_SR_CC4IF) == 0u) {
        for (uint32_t i = 0; i <= n; i++) {
            s_bsrr[i] = shadow[i];
        }
        s_refreshed++;
    } else {
        s_refresh_skipped++;
    }
    __set_PRIMASK(primask);
}

bool ssi_slave_set_frame_bits(uint8_t n_bits)
{
    if (n_bits == 0u || n_bits > SSI_MAX_BITS || (n_bits % 8u) != 0u) {
        return false;
    }

    /* Stop first: the Tmu handler re-arms, and it must not run between the
     * config change and the fresh arm below. */
    ssi_slave_stop();
    s_cfg.n_bits = n_bits;
    ssi_slave_start();
    return true;
}

void ssi_slave_get_stats(ssi_slave_stats_t *out)
{
    out->frames      = s_frames;
    out->short_reads = s_short;
    out->aborted     = s_aborted;
    out->period_ns   = s_period_valid ? s_period_ns : 0u;
    out->tmu_arm_ns  = s_tmu_arm_ticks * TMU_TICK_NS;
    out->refreshed       = s_refreshed;
    out->refresh_skipped = s_refresh_skipped;
}

bool ssi_slave_clock_counter_ok(void)
{
    return s_clk_counted;
}
