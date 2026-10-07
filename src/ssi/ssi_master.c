#include "ssi/ssi_master.h"
#include "board/board.h"

/* The SSI clock window, section 5.4.1. */
#define SSI_MASTER_MIN_HZ  100000u
#define SSI_MASTER_MAX_HZ  2000000u

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

void ssi_master_init(void)
{
    GPIO_InitTypeDef io = {0};

    dwt_init();

    __HAL_RCC_GPIOB_CLK_ENABLE();

    /* The clock pin is a plain GPIO output: TIM1 compare events drive it
     * through DMA writes to BSRR. Hold it at the SSI idle level before the
     * first burst so the slave sees no spurious edge. */
    SSI_MASTER_SCK_GPIO->BSRR = SSI_MASTER_SCK_PIN;
    io.Pin   = SSI_MASTER_SCK_PIN;
    io.Mode  = GPIO_MODE_OUTPUT_PP;
    io.Pull  = GPIO_PULLUP;             /* SSI clock idles HIGH */
    io.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(SSI_MASTER_SCK_GPIO, &io);

    io.Pin  = SSI_MASTER_MISO_PIN;
    io.Mode = GPIO_MODE_INPUT;
    io.Pull = GPIO_PULLUP;              /* DATA idles HIGH */
    HAL_GPIO_Init(SSI_MASTER_MISO_GPIO, &io);
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
 * Streams 0/3/7 of DMA2 are taken by ADC1 and the slave's DATA path, hence
 * 1/4/6.
 */
#define X_TIM            TIM1
#define X_DMA_CHANNEL    6u

/* The clock is TIM1's period, so any rate of the form 180 MHz / N is reachable
 * N is bounded by the
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

    /* Hold the clock at its idle HIGH level before anything else, so the
     * slave sees no spurious edge. The pin is already a GPIO output. */
    GPIOB->BSRR = SSI_MASTER_SCK_PIN;

    X_TIM->CR1  = 0;
    X_TIM->PSC  = 0;
    X_TIM->ARR  = s_x_n - 1u;
    X_TIM->CCR1 = 1u;                /* falling edge, start of period */
    X_TIM->CCR3 = (s_x_n / 2u) + 1u; /* rising edge, half a period later */
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

    /* Leave the line at the SSI idle level. */
    GPIOB->BSRR  = SSI_MASTER_SCK_PIN;

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

bool ssi_loopback_check(bool *clock_ok, bool *data_ok)
{
    const uint32_t drive = SSI_MASTER_SCK_PIN | SSI_SLAVE_DATA_PIN;

    uint32_t moder_save = GPIOB->MODER;
    uint32_t odr_save   = GPIOB->ODR;

    /* PB10 and PB4 as push-pull outputs; PC6/PB14 stay as they are and are
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

        /* The clock is checked at PC6, which is the pin that matters: it is
         * what clocks TIM8 and therefore what shifts DATA out. */
        if (((SSI_CLKIN_GPIO->IDR & SSI_CLKIN_PIN) != 0u) != want) {
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
