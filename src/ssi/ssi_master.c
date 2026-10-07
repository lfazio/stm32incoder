#include "ssi/ssi_master.h"
#include "board/board.h"

/* SPI2 lives on APB1, so its clock is PCLK1 = 45 MHz. The SSI clock window is
 * 100 kHz .. 2 MHz (section 5.4.1); with the SPI's power-of-two prescalers the
 * usable divisors are therefore 32 (1.406 MHz) through 256 (176 kHz). */
#define SSI_MASTER_MIN_HZ  100000u
#define SSI_MASTER_MAX_HZ  2000000u

static uint32_t s_clock_hz;
static uint8_t  s_br;   /* CR1.BR field value */

static void dwt_init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

void ssi_master_delay_us(uint32_t us)
{
    uint32_t start  = DWT->CYCCNT;
    uint32_t cycles = us * (SYSCLK_HZ / 1000000u);
    while ((DWT->CYCCNT - start) < cycles) { }
}

/* Which edge this test master samples on has to follow the edge the slave
 * under test drives, or every frame reads back shifted by one bit.
 *
 *   slave drives on   master samples on   builds
 *   rising edge       falling edge        default, and SIMENC_RISING_EDGE
 *   falling edge      rising edge         SPI fallback (TIMER_DATA=OFF)
 *
 * A real SSI controller always samples on the falling edge; the rising-edge
 * case exists only because the SPI fallback drives DATA half a period early,
 * which is the deviation documented in the README. */
#if defined(SIMENC_TIMER_DATA) || defined(SIMENC_RISING_EDGE)
#define MASTER_SAMPLE_ON_FALLING 1
#else
#define MASTER_SAMPLE_ON_FALLING 0
#endif

static void spi_apply(void)
{
    SSI_MASTER_SPI->CR1 &= ~SPI_CR1_SPE;
    /* Master, CPOL=1 so the clock idles HIGH as SSI requires, 8-bit, MSB
     * first. CPOL=1 makes the leading edge of each bit period the falling one,
     * so CPHA=0 samples on the fall and CPHA=1 on the rise -- see
     * MASTER_SAMPLE_ON_FALLING.
     *
     * SSM=1 with SSI=1 holds the internal NSS high, which a master needs to
     * avoid a mode fault. */
    SSI_MASTER_SPI->CR1 = SPI_CR1_MSTR | SPI_CR1_CPOL
#if !MASTER_SAMPLE_ON_FALLING
                        | SPI_CR1_CPHA
#endif
                        | SPI_CR1_SSM | SPI_CR1_SSI
                        | ((uint32_t)s_br << SPI_CR1_BR_Pos);
    SSI_MASTER_SPI->CR2 = 0;
    SSI_MASTER_SPI->CR1 |= SPI_CR1_SPE;
}

void ssi_master_init(void)
{
    GPIO_InitTypeDef io = {0};

    dwt_init();

    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_SPI2_CLK_ENABLE();

    io.Pin       = SSI_MASTER_SCK_PIN;
    io.Mode      = GPIO_MODE_AF_PP;
    io.Pull      = GPIO_PULLUP;         /* SSI clock idles HIGH */
    io.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    io.Alternate = SSI_MASTER_AF;
    HAL_GPIO_Init(SSI_MASTER_SCK_GPIO, &io);

    io.Pin  = SSI_MASTER_MISO_PIN;
    io.Pull = GPIO_PULLUP;              /* DATA idles HIGH */
    HAL_GPIO_Init(SSI_MASTER_MISO_GPIO, &io);

    s_br       = 4;                      /* /32 */
    s_clock_hz = PCLK1_HZ >> (s_br + 1u);
    spi_apply();
}

uint32_t ssi_master_set_clock(uint32_t requested_hz)
{
    for (uint8_t br = 0; br < 8u; br++) {
        uint32_t hz = PCLK1_HZ >> (br + 1u);
        if (hz > SSI_MASTER_MAX_HZ || hz < SSI_MASTER_MIN_HZ) {
            continue;
        }
        if (hz <= requested_hz || s_clock_hz == 0u) {
            s_br       = br;
            s_clock_hz = hz;
            spi_apply();
            return s_clock_hz;
        }
    }
    /* Nothing at or below the request is legal: fall back to the slowest. */
    s_br       = 7;
    s_clock_hz = PCLK1_HZ >> 8u;
    spi_apply();
    return s_clock_hz;
}

uint32_t ssi_master_get_clock(void)
{
    return s_clock_hz;
}

bool ssi_master_data_idle_high(void)
{
    return (SSI_MASTER_MISO_GPIO->IDR & SSI_MASTER_MISO_PIN) != 0u;
}

/* --- exact-rate engine: TIM1 compare events + DMA to GPIO ------------------
 *
 * TIM1 is on APB2, whose timer clock is 180 MHz, so ARR = 89 gives a 90-count
 * period = 500 ns = 2.000 MHz exactly. Three DMA streams hang off three
 * compare channels (RM0390 Rev 9 Table 29, DMA2 channel 6):
 *
 *   CH1 -> stream 1 : writes BSRR to pull the clock LOW   (start of period)
 *   CH3 -> stream 6 : writes BSRR to drive the clock HIGH (mid period)
 *   CH4 -> stream 4 : reads GPIOB->IDR into the sample buffer
 *
 * The pulse count is exactly the DMA transfer count, so the burst stops itself
 * after n edges and leaves the line HIGH -- no software has to chase the timer.
 * Streams 0/2/3 of DMA2 are already taken by ADC1 and SPI1, hence 1/4/6.
 */
#define X_TIM            TIM1
#define X_DMA_CHANNEL    6u

/* The clock is TIM1's period, so any rate of the form 180 MHz / N is reachable
 * rather than only the SPI baud generator's powers of two. N is bounded by the
 * SSI window itself: N = 90 is 2.000 MHz and N = 1800 is 100.0 kHz.
 *
 * Resolution is set by N being an integer, so it is coarsest at the top of the
 * range -- 1.1 % per step at 2 MHz, 0.06 % at 100 kHz. */
#define X_N_MIN          90u     /* 180 MHz / 90   = 2.000 MHz */
#define X_N_MAX          1800u   /* 180 MHz / 1800 = 100.0 kHz */

/* Power-on rate: 500 kHz, mid-range rather than at the 2 MHz limit, so the
 * default is a rate a real controller would plausibly use. Raise it with
 * `clk 2000000` when deliberately exercising the timing corner. */
#define X_N_DEFAULT      360u    /* 180 MHz / 360  = 500.0 kHz */

static uint32_t s_x_n = X_N_DEFAULT;  /* timer period in APB2 timer ticks */

#define X_LOW_STREAM     DMA2_Stream1
#define X_HIGH_STREAM    DMA2_Stream6
#define X_SAMPLE_STREAM  DMA2_Stream4

#define X_LOW_CLEAR   (DMA_LIFCR_CTCIF1 | DMA_LIFCR_CHTIF1 | DMA_LIFCR_CTEIF1 | \
                       DMA_LIFCR_CDMEIF1 | DMA_LIFCR_CFEIF1)
#define X_HIGH_CLEAR  (DMA_HIFCR_CTCIF6 | DMA_HIFCR_CHTIF6 | DMA_HIFCR_CTEIF6 | \
                       DMA_HIFCR_CDMEIF6 | DMA_HIFCR_CFEIF6)
#define X_SMPL_CLEAR  (DMA_HIFCR_CTCIF4 | DMA_HIFCR_CHTIF4 | DMA_HIFCR_CTEIF4 | \
                       DMA_HIFCR_CDMEIF4 | DMA_HIFCR_CFEIF4)

static const uint32_t s_clk_low  = (uint32_t)SSI_MASTER_SCK_PIN << 16u;
static const uint32_t s_clk_high = (uint32_t)SSI_MASTER_SCK_PIN;
static volatile uint32_t s_idr[32];

static uint32_t s_dbg_ndtr_lo, s_dbg_ndtr_hi, s_dbg_ndtr_smp;
static uint32_t s_dbg_tim_cnt, s_dbg_tim_sr, s_dbg_lisr, s_dbg_hisr;

void ssi_master_timer_debug(uint32_t *out7)
{
    out7[0] = s_dbg_ndtr_lo;
    out7[1] = s_dbg_ndtr_hi;
    out7[2] = s_dbg_ndtr_smp;
    out7[3] = s_dbg_tim_cnt;
    out7[4] = s_dbg_tim_sr;
    out7[5] = s_dbg_lisr;
    out7[6] = s_dbg_hisr;
}

uint32_t ssi_master_exact_hz(void)
{
    return APB2_TIMCLK_HZ / s_x_n;
}

/* Selects the timer-generated clock rate. Rounds to the nearest achievable
 * 180 MHz / N and clamps to the specified 100 kHz .. 2 MHz window; returns the
 * rate actually programmed. */
uint32_t ssi_master_set_exact_clock(uint32_t requested_hz)
{
    uint32_t n;

    if (requested_hz == 0u) {
        n = X_N_MAX;
    } else {
        /* round to nearest N */
        n = (APB2_TIMCLK_HZ + (requested_hz / 2u)) / requested_hz;
    }
    if (n < X_N_MIN) { n = X_N_MIN; }
    if (n > X_N_MAX) { n = X_N_MAX; }

    s_x_n = n;
    return ssi_master_exact_hz();
}

static void x_stream_setup(DMA_Stream_TypeDef *st, uint32_t par, uint32_t m0ar,
                           uint32_t cr_extra, uint16_t ndtr)
{
    st->CR &= ~DMA_SxCR_EN;
    while (st->CR & DMA_SxCR_EN) { }

    st->PAR  = par;
    st->M0AR = m0ar;
    st->NDTR = ndtr;
    st->FCR  = 0;
    st->CR   = (X_DMA_CHANNEL << DMA_SxCR_CHSEL_Pos)
             | DMA_SxCR_PSIZE_1 | DMA_SxCR_MSIZE_1   /* 32-bit both sides */
             | DMA_SxCR_PL_0 | DMA_SxCR_PL_1         /* very high */
             | cr_extra;
}

bool ssi_master_read_timer(uint8_t n_bits, uint32_t *raw)
{
    if (n_bits == 0u || n_bits > 32u) {
        return false;
    }
    if (!ssi_master_data_idle_high()) {
        return false;
    }

    __HAL_RCC_TIM1_CLK_ENABLE();
    __HAL_RCC_DMA2_CLK_ENABLE();

    /* Take the clock pin away from SPI2 and hold it at the idle HIGH level
     * before anything else, so the slave sees no spurious edge. */
    GPIOB->BSRR = SSI_MASTER_SCK_PIN;
    uint32_t moder_save = GPIOB->MODER;
    GPIOB->MODER = (moder_save & ~(3u << (10u * 2u))) | (1u << (10u * 2u));

    X_TIM->CR1  = 0;
    X_TIM->PSC  = 0;
    X_TIM->ARR  = s_x_n - 1u;
    X_TIM->CCR1 = 1u;                /* falling edge, start of period */
    X_TIM->CCR3 = (s_x_n / 2u) + 1u; /* rising edge, half a period later */
#if MASTER_SAMPLE_ON_FALLING
    /* Late in the high phase, just before the next falling edge, which is
     * where an SSI controller reads: the bit was set on the rising edge and
     * stays valid across the fall.
     *
     * Sampling one tick *after* the rising edge -- which this used to do --
     * reads whatever was on the line before that edge, i.e. the previous bit.
     * The slave puts each bit up 56 ns after the rising edge (DMA latency),
     * which is 10 timer ticks at 500 kHz and 11 at 2 MHz, so a sample one tick
     * later always lost the race. The symptom was the whole frame shifted one
     * bit, making `fixed` and `ramp` read back (value >> 1) with ZPD shifted
     * into the top: 0x5A5A5 came out as 447186. */
    X_TIM->CCR4 = s_x_n - 2u;
#else
    /* The SPI fallback drives DATA on the falling edge, so the bit is already
     * up by the rising edge and this samples in its stable window. */
    X_TIM->CCR4 = (s_x_n / 2u) + 2u;
#endif
    X_TIM->CCMR1 = 0;          /* channels as output compare, frozen: the
                                * compare flags still fire, and no pin is
                                * driven by the timer itself */
    X_TIM->CCMR2 = 0;
    X_TIM->CNT  = 0;
    X_TIM->SR   = 0;

    DMA2->LIFCR = X_LOW_CLEAR;
    DMA2->HIFCR = X_HIGH_CLEAR | X_SMPL_CLEAR;

    x_stream_setup(X_LOW_STREAM,  (uint32_t)(uintptr_t)&GPIOB->BSRR,
                   (uint32_t)(uintptr_t)&s_clk_low,  DMA_SxCR_DIR_0, n_bits);
    x_stream_setup(X_HIGH_STREAM, (uint32_t)(uintptr_t)&GPIOB->BSRR,
                   (uint32_t)(uintptr_t)&s_clk_high, DMA_SxCR_DIR_0, n_bits);
    x_stream_setup(X_SAMPLE_STREAM, (uint32_t)(uintptr_t)&GPIOB->IDR,
                   (uint32_t)(uintptr_t)s_idr, DMA_SxCR_MINC, n_bits);

    X_LOW_STREAM->CR    |= DMA_SxCR_EN;
    X_HIGH_STREAM->CR   |= DMA_SxCR_EN;
    X_SAMPLE_STREAM->CR |= DMA_SxCR_EN;

    X_TIM->DIER = TIM_DIER_CC1DE | TIM_DIER_CC3DE | TIM_DIER_CC4DE;
    X_TIM->CR1  = TIM_CR1_CEN;

    /* n_bits x 500 ns is 16 us for a 32-bit frame; bail out far beyond that. */
    uint32_t guard = 0;
    while ((DMA2->HISR & DMA_HISR_TCIF4) == 0u) {
        if (++guard > 200000u) {
            break;
        }
    }

    s_dbg_ndtr_lo  = X_LOW_STREAM->NDTR;
    s_dbg_ndtr_hi  = X_HIGH_STREAM->NDTR;
    s_dbg_ndtr_smp = X_SAMPLE_STREAM->NDTR;
    s_dbg_tim_cnt  = X_TIM->CNT;
    s_dbg_tim_sr   = X_TIM->SR;
    s_dbg_lisr     = DMA2->LISR;
    s_dbg_hisr     = DMA2->HISR;

    X_TIM->CR1  = 0;
    X_TIM->DIER = 0;
    X_LOW_STREAM->CR    &= ~DMA_SxCR_EN;
    X_HIGH_STREAM->CR   &= ~DMA_SxCR_EN;
    X_SAMPLE_STREAM->CR &= ~DMA_SxCR_EN;

    bool complete = (guard <= 200000u);

    /* Leave the line HIGH and give the pin back to SPI2. */
    GPIOB->BSRR  = SSI_MASTER_SCK_PIN;
    GPIOB->MODER = moder_save;

    if (!complete) {
        return false;
    }

    uint32_t v = 0;
    for (uint8_t i = 0; i < n_bits; i++) {
        v <<= 1;
        if (s_idr[i] & SSI_MASTER_MISO_PIN) {
            v |= 1u;
        }
    }
    *raw = v;
    return true;
}

/* Time PB10 -> pin, in DWT cycles, as the median of several tries. Each try
 * drives the clock pin low, settles, then drives it high and spins on IDR. */
static uint32_t edge_delay_cycles(volatile uint32_t *idr, uint32_t mask)
{
    uint32_t best[9];

    for (uint32_t k = 0; k < 9u; k++) {
        GPIOB->BSRR = (uint32_t)SSI_MASTER_SCK_PIN << 16u;
        ssi_master_delay_us(20);

        uint32_t t0 = DWT->CYCCNT;
        GPIOB->BSRR = SSI_MASTER_SCK_PIN;
        uint32_t guard = 0;
        while (((*idr) & mask) == 0u && guard < 4000u) {
            guard++;
        }
        best[k] = DWT->CYCCNT - t0;
    }
    /* median of 9, by selection -- no sorting library and none needed */
    for (uint32_t i = 0; i < 5u; i++) {
        uint32_t m = i;
        for (uint32_t j = i + 1u; j < 9u; j++) {
            if (best[j] < best[m]) { m = j; }
        }
        uint32_t t = best[i]; best[i] = best[m]; best[m] = t;
    }
    return best[4];
}

void ssi_clock_skew(uint32_t *pb3_ns, uint32_t *pc6_ns)
{
    EXTI->IMR &= ~SSI_SLAVE_SCK_PIN;
    EXTI->PR   = SSI_SLAVE_SCK_PIN;

    uint32_t moder_save = GPIOB->MODER;
    uint32_t odr_save   = GPIOB->ODR;

    uint32_t m = moder_save & ~(3u << (10u * 2u));
    m |= (1u << (10u * 2u));                 /* PB10 push-pull output */
    GPIOB->MODER = m;

    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    uint32_t c_pb3 = edge_delay_cycles(&GPIOB->IDR, SSI_SLAVE_SCK_PIN);
    uint32_t c_pc6 = edge_delay_cycles(&GPIOC->IDR, GPIO_PIN_6);

    GPIOB->ODR   = odr_save;
    GPIOB->MODER = moder_save;

    /* Scale before dividing: 1000/(180e6/1e6) truncates to 5 instead of 5.56. */
    *pb3_ns = (c_pb3 * 1000u) / (SYSCLK_HZ / 1000000u);
    *pc6_ns = (c_pc6 * 1000u) / (SYSCLK_HZ / 1000000u);
}

bool ssi_loopback_check(bool *clock_ok, bool *data_ok)
{
    const uint32_t drive = SSI_MASTER_SCK_PIN | SSI_SLAVE_DATA_PIN;

    /* Silence the slave's first-edge handover first. Driving the clock pin low
     * here looks exactly like the start of a Read Cycle, and EXTI3_IRQHandler
     * would rewrite GPIOB->MODER underneath this test -- putting PB10 back into
     * alternate-function mode, so the pin would stop following ODR and every
     * wire would be misreported as OPEN. ssi_slave_start() re-arms it. */
    EXTI->IMR &= ~SSI_SLAVE_SCK_PIN;
    EXTI->PR   = SSI_SLAVE_SCK_PIN;

    uint32_t moder_save = GPIOB->MODER;
    uint32_t odr_save   = GPIOB->ODR;

    /* PB10 and PB4 as push-pull outputs; PB3/PB14 stay as they are and are
     * simply sampled through IDR, which reads the pad in any mode. */
    uint32_t m = moder_save;
    m &= ~((3u << (10u * 2u)) | (3u << (4u * 2u)));
    m |=  ((1u << (10u * 2u)) | (1u << (4u * 2u)));
    GPIOB->MODER = m;

    bool ck = true;
    bool dt = true;

    for (uint32_t lvl = 0; lvl <= 1u; lvl++) {
        GPIOB->BSRR = lvl ? drive : (drive << 16u);
        ssi_master_delay_us(50);

        uint32_t idr  = GPIOB->IDR;
        bool     want = (lvl != 0u);

        if (((idr & SSI_SLAVE_SCK_PIN) != 0u) != want) {
            ck = false;
        }
        if (((idr & SSI_MASTER_MISO_PIN) != 0u) != want) {
            dt = false;
        }
    }

    GPIOB->ODR   = odr_save;
    GPIOB->MODER = moder_save;

    *clock_ok = ck;
    *data_ok  = dt;
    return ck && dt;
}

bool ssi_master_read(uint8_t n_bits, uint32_t *raw)
{
    if ((n_bits % 8u) != 0u || n_bits > 32u) {
        return false;
    }
    if (!ssi_master_data_idle_high()) {
        return false;
    }

    uint8_t  nbytes = (uint8_t)(n_bits / 8u);
    uint32_t value  = 0;

    /* Writing a byte is what produces 8 clock pulses; the byte value itself is
     * irrelevant because the slave ignores MOSI. */
    for (uint8_t i = 0; i < nbytes; i++) {
        while (!(SSI_MASTER_SPI->SR & SPI_SR_TXE)) { }
        *(volatile uint8_t *)&SSI_MASTER_SPI->DR = 0xFFu;
        while (!(SSI_MASTER_SPI->SR & SPI_SR_RXNE)) { }
        value = (value << 8u) | *(volatile uint8_t *)&SSI_MASTER_SPI->DR;
    }
    while (SSI_MASTER_SPI->SR & SPI_SR_BSY) { }

    *raw = value;
    return true;
}
