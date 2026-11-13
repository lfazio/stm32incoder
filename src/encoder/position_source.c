#include "encoder/position_source.h"
#include "board/board.h"
#include "encoder/circular_mean.h"
#include "ssi/ssi_variant.h"

/* The IncOder specifies an "Internal Position Update Period" of < 0.1 ms for
 * all digital comms options (Product Guide 4.12). We run the acquisition at
 * 10 kHz (100 us), the fastest rate that still satisfies "< 0.1 ms". */
#define UPDATE_RATE_HZ   10000u

/* Oversampling.
 *
 * The ADC free-runs and DMA writes into this circular buffer; each position
 * update averages the whole buffer. Sized so the buffer spans one update
 * period, so an update averages the conversions taken *during* that period.
 *
 * It used to be four samples taken at the update rate itself, which meant each
 * update was a moving average over the previous 400 us -- four update periods.
 * The Time Stamp claims to say when the position was measured, and a 400 us
 * smear makes that claim false.
 *
 * fADC = PCLK2/8 = 11.25 MHz and a 12-bit conversion takes 84 sampling + 12
 * conversion cycles = 96, so 8.53 us each, and 12 of them span 102 us against
 * the 100 us update period. 12x oversampling is worth about 1.8 bits of noise
 * averaging (sqrt 12).
 *
 * Pushing this further means a faster ADC clock and shorter sampling, which
 * costs bus bandwidth, which the SSI data path shares. Do not raise it without
 * re-checking the bit timing on an analyser. */
#define ADC_AVG_SAMPLES  12u

/* The conversion timing above, as the code configures it in adc_init(). Keep
 * these in step with ADC_CLOCK_SYNC_PCLK_DIV8 and ADC_SAMPLETIME_84CYCLES. */
#define ADC_CLOCK_DIV       8u
#define ADC_SAMPLE_CYCLES   84u
#define ADC_CONVERT_CYCLES  12u   /* 12-bit successive approximation */
#define ADC_CONV_NS  ((uint32_t)(((uint64_t)(ADC_SAMPLE_CYCLES + ADC_CONVERT_CYCLES) \
                                  * ADC_CLOCK_DIV * 1000000000ull) / PCLK2_HZ))  /* 8533 */
#define ADC_HOLD_NS  ((uint32_t)(((uint64_t)ADC_CONVERT_CYCLES \
                                  * ADC_CLOCK_DIV * 1000000000ull) / PCLK2_HZ))  /* 1067 */

/* Where the averaged value sits in time, measured back from the update tick.
 *
 * The sum covers the ADC_AVG_SAMPLES most recent *completed* conversions. Each
 * one holds the input at the end of its sampling phase, ADC_HOLD_NS before it
 * completes. The last one completed some time d before the tick, d uniform over
 * one conversion because the ADC free-runs against the tick. The centroid of
 * the window is therefore
 *
 *     d + ADC_HOLD_NS + (ADC_AVG_SAMPLES - 1) / 2 * ADC_CONV_NS
 *
 * and with d at its mean of ADC_CONV_NS / 2 that is
 * ADC_AVG_SAMPLES / 2 * ADC_CONV_NS + ADC_HOLD_NS = 51.2 + 1.07 = 52.3 us. The
 * spread of d is +/-4.3 us around it: under SSI4's 10 us TS resolution, but
 * not under SSI9's 1 us. */
#define ADC_WINDOW_DELAY_NS  ((ADC_AVG_SAMPLES * ADC_CONV_NS) / 2u + ADC_HOLD_NS)

/* Pole pairs of the motor whose electrical angle the analog input carries.
 * Not a hardware value from docs/: it is the user's motor, which has 12. The
 * console changes it at runtime (`pp <n>`).
 *
 * The ceiling keeps the scaling in 32 bits: the mechanical sum is under
 * POLE_PAIRS_MAX * ADC_TURN_SUM = 64 * 49152 < 2^22, and it is shifted left by
 * at most 22 - 12 = 10. */
#define POLE_PAIRS_DEFAULT  12u
#define POLE_PAIRS_MAX      64u

/* One electrical turn in units of the averaging-window sum: VREF+ is code
 * CIRCULAR_TURN, and the sum adds ADC_AVG_SAMPLES of them. */
#define ADC_TURN_SUM  (CIRCULAR_TURN * ADC_AVG_SAMPLES)   /* 49152 */

/* ADC1 -> DMA2 Stream 0, channel 0 (RM0390 Rev 9 Table 29). Stream 4 carries
 * the same request; streams 3/7 are taken by the slave's DATA path. */
#define ADC_DMA_STREAM   DMA2_Stream0
#define ADC_DMA_CHANNEL  0u

/* The ADC is 12-bit; the position field is 19 or 22 bits depending on variant,
 * so full scale is reached by shifting left. Either way the analog resolution
 * stays 12-bit -- the extra field width does not create information. */
static uint8_t  s_pos_bits;
static uint32_t s_pos_mask;

void position_source_set_width(uint8_t bits)
{
    if (bits < 12u || bits > 22u) {
        return;
    }
    s_pos_bits = bits;
    s_pos_mask = (1u << bits) - 1u;
}

static ADC_HandleTypeDef s_adc;
static TIM_HandleTypeDef s_tim;
static DMA_HandleTypeDef s_adc_dma;

static volatile uint16_t s_samples[ADC_AVG_SAMPLES];
static volatile bool     s_adc_running;
/* The averaging window has been filled since the ADC (re)started. A flag, not
 * a count: a conversion counter wraps, and one that was compared against
 * ADC_AVG_SAMPLES dropped PV for one window every ~10 hours. */
static volatile bool     s_adc_filled;
static volatile uint32_t s_adc_overruns;

static position_source_t s_src = POS_SRC_ADC;
static uint32_t          s_fixed;
static int32_t           s_ramp_step = 64;
static uint32_t          s_ramp;

/* Electrical-turn tracking. The update tick is the only writer, except
 * position_source_set_pole_pairs(), which writes with interrupts masked. */
static uint32_t s_pole_pairs = POLE_PAIRS_DEFAULT;
static uint32_t s_elec_turn;      /* k, 0 .. s_pole_pairs - 1 */
static uint32_t s_prev_sum;       /* last window sum, 0 .. ADC_TURN_SUM - 1 */
static bool     s_have_prev;      /* s_prev_sum belongs to a valid window */

static void tim_init(void)
{
    __HAL_RCC_TIM2_CLK_ENABLE();

    /* TIM2 sits on APB1; with the APB1 prescaler != 1 its clock is 90 MHz.
     * It does not trigger the ADC -- that free-runs -- it only paces the
     * position update at the rate 4.12 specifies, through its update
     * interrupt. The ADC and this tick are not in step. */
    s_tim.Instance               = TIM2;
    s_tim.Init.Prescaler         = (APB1_TIMCLK_HZ / 1000000u) - 1u;  /* 1 MHz */
    s_tim.Init.CounterMode       = TIM_COUNTERMODE_UP;
    s_tim.Init.Period            = (1000000u / UPDATE_RATE_HZ) - 1u;  /* 100 us */
    s_tim.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    s_tim.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    HAL_TIM_Base_Init(&s_tim);
}

static void adc_init(void)
{
    __HAL_RCC_DMA2_CLK_ENABLE();
    __HAL_RCC_ADC1_CLK_ENABLE();

    s_adc_dma.Instance                 = ADC_DMA_STREAM;
    s_adc_dma.Init.Channel             = DMA_CHANNEL_0;
    s_adc_dma.Init.Direction           = DMA_PERIPH_TO_MEMORY;
    s_adc_dma.Init.PeriphInc           = DMA_PINC_DISABLE;
    s_adc_dma.Init.MemInc              = DMA_MINC_ENABLE;
    s_adc_dma.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    s_adc_dma.Init.MemDataAlignment    = DMA_MDATAALIGN_HALFWORD;
    s_adc_dma.Init.Mode                = DMA_CIRCULAR;
    s_adc_dma.Init.Priority            = DMA_PRIORITY_LOW;
    s_adc_dma.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
    HAL_DMA_Init(&s_adc_dma);
    __HAL_LINKDMA(&s_adc, DMA_Handle, s_adc_dma);

    /* ADC clock = PCLK2 / ADC_CLOCK_DIV = 11.25 MHz, inside the 36 MHz maximum. */
    s_adc.Instance                   = ANGLE_ADC;
    s_adc.Init.ClockPrescaler        = ADC_CLOCK_SYNC_PCLK_DIV8;
    s_adc.Init.Resolution            = ADC_RESOLUTION_12B;
    s_adc.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
    s_adc.Init.ScanConvMode          = DISABLE;
    s_adc.Init.EOCSelection          = ADC_EOC_SINGLE_CONV;
    /* Free-running rather than triggered at the update rate: the point is to
     * take many conversions inside each update period, not one. */
    s_adc.Init.ContinuousConvMode    = ENABLE;
    s_adc.Init.DiscontinuousConvMode = DISABLE;
    s_adc.Init.NbrOfConversion       = 1;
    s_adc.Init.ExternalTrigConv      = ADC_SOFTWARE_START;
    s_adc.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_NONE;
    s_adc.Init.DMAContinuousRequests = ENABLE;
    HAL_ADC_Init(&s_adc);

    ADC_ChannelConfTypeDef ch = {0};
    ch.Channel      = ANGLE_ADC_CHANNEL;
    ch.Rank         = 1;
    ch.SamplingTime = ADC_SAMPLETIME_84CYCLES;
    HAL_ADC_ConfigChannel(&s_adc, &ch);

    /* Lowest of our interrupt priorities: the SSI path must always win. */
    HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 3, 1);
    HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
}

void DMA2_Stream0_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&s_adc_dma);
}

void position_source_init(void)
{
    /* Until a variant is selected, use the default one's position width. */
    position_source_set_width(ssi_variant_position_bits(SSI_VARIANT_DEFAULT));

    tim_init();
    adc_init();
}

void position_source_start(void)
{
    if (!s_adc_running) {
        HAL_ADC_Start_DMA(&s_adc, (uint32_t *)s_samples, ADC_AVG_SAMPLES);
        HAL_TIM_Base_Start(&s_tim);
        s_adc_running = true;
    }
}

void position_source_select(position_source_t src)
{
    s_src = src;
}

position_source_t position_source_get(void)
{
    return s_src;
}

const char *position_source_name(position_source_t src)
{
    switch (src) {
    case POS_SRC_ADC:   return "adc";
    case POS_SRC_FIXED: return "fixed";
    case POS_SRC_RAMP:  return "ramp";
    default:            return "?"; 
    }
}

void position_source_set_fixed(uint32_t counts)
{
    s_fixed = counts & s_pos_mask;
}

void position_source_set_ramp_step(int32_t counts_per_update)
{
    s_ramp_step = counts_per_update;
}

bool position_source_set_pole_pairs(uint32_t pairs)
{
    if (pairs == 0u || pairs > POLE_PAIRS_MAX) {
        return false;
    }

    /* The update tick reads all three; it must never see a turn index from
     * the old pole count against the new one. */
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    s_pole_pairs = pairs;
    s_elec_turn  = 0u;
    s_have_prev  = false;
    __set_PRIMASK(primask);
    return true;
}

uint32_t position_source_pole_pairs(void)
{
    return s_pole_pairs;
}

uint32_t position_source_elec_turn(void)
{
    return s_elec_turn;
}

/* Sum of the averaging window taken on the circle (circular_mean.h): divided by
 * ADC_AVG_SAMPLES it is the window's circular mean in ADC codes, modulo one
 * turn, somewhere in 0 .. 2 turns.
 *
 * Kept un-divided on purpose. Dividing down to 12 bits first throws away
 * whatever the oversampling bought, and it is the sum that gets scaled into
 * the position field.
 *
 * A copy, because circular_sum() sorts its input and the DMA keeps writing.
 * A slot replaced during the copy only trades the oldest conversion for a
 * newer one, and the order of the samples does not matter. */
static uint32_t adc_sum(void)
{
    uint16_t v[ADC_AVG_SAMPLES];
    for (uint32_t i = 0; i < ADC_AVG_SAMPLES; i++) {
        v[i] = s_samples[i];
    }
    return circular_sum(v, ADC_AVG_SAMPLES);
}

uint32_t position_source_delay_us(void)
{
    /* Synthetic sources have no measurement window: their value is the value
     * at the tick. */
    if (s_src != POS_SRC_ADC) {
        return 0u;
    }
    return (ADC_WINDOW_DELAY_NS + 500u) / 1000u;
}

uint16_t position_source_raw_adc(void)
{
    return (uint16_t)(((adc_sum() + ADC_AVG_SAMPLES / 2u) / ADC_AVG_SAMPLES)
                      % CIRCULAR_TURN);
}

void position_source_tick(void)
{
    if (s_src == POS_SRC_RAMP) {
        s_ramp = (uint32_t)((int32_t)s_ramp + s_ramp_step) & s_pos_mask;
    }
    /* The turn tracking compares consecutive windows. One taken before the
     * source left `adc` says nothing about where the input is now. */
    if (s_src != POS_SRC_ADC) {
        s_have_prev = false;
    }
}

/* Counts the electrical turns the analog input has made, from one update to the
 * next, and returns k + sum / ADC_TURN_SUM as a sum: the mechanical angle in
 * electrical-turn units, 0 .. pole_pairs * ADC_TURN_SUM - 1.
 *
 * The step between two windows is taken on the shortest arc, so a crossing of
 * 0 V / VDDA in either direction moves k by one, and a noise spike that crosses
 * and comes back moves it twice and cancels. That only holds while the input
 * moves less than half an electrical turn per update period: under 5 kHz
 * electrical, 25 000 rpm at 12 pole pairs.
 *
 * Only the update tick may call this (through position_source_read()). */
static uint32_t track_elec_turn(uint32_t sum, uint32_t pairs)
{
    /* Two windows from either side of the ADC filling up, or of an overrun,
     * have nothing to do with each other. */
    if (!position_source_valid()) {
        s_have_prev = false;
    } else if (s_have_prev) {
        const int32_t d = (int32_t)sum - (int32_t)s_prev_sum;
        if (d < -(int32_t)(ADC_TURN_SUM / 2u)) {
            s_elec_turn = (s_elec_turn + 1u) % pairs;            /* forward through 0 */
        } else if (d > (int32_t)(ADC_TURN_SUM / 2u)) {
            s_elec_turn = (s_elec_turn + pairs - 1u) % pairs;    /* backward through 0 */
        }
        s_prev_sum = sum;
    } else {
        s_prev_sum  = sum;
        s_have_prev = true;
    }
    return s_elec_turn * ADC_TURN_SUM + sum;
}

uint32_t position_source_read(void)
{
    switch (s_src) {
    case POS_SRC_FIXED:
        return s_fixed;
    case POS_SRC_RAMP:
        return s_ramp;
    case POS_SRC_ADC:
    default: {
        /* The analog input is the motor's electrical angle: 0 V .. VREF+ is
         * one electrical turn, 1/pole_pairs of a mechanical one. The encoder
         * reports the mechanical angle, (k + electrical) / pole_pairs, with k
         * the electrical turn counted by track_elec_turn(). It runs on through
         * every electrical turn and wraps only once per pole_pairs of them, so
         * a controller that computes pole_pairs * mechanical gets the input
         * back, with no jump in speed when the input wraps. Which k the
         * emulator starts on is arbitrary, like where a real encoder is
         * clamped onto the shaft; the electrical angle does not depend on it.
         *
         * Scale the circular mean onto the position field.
         *
         * 1 LSB is VREF+/4096, and on the ideal transfer curve the
         * transitions sit at 0.5, 1.5, ... 4094.5 LSB [DS10693 Rev 11
         * Figure 45], so code k is centred on k/4096 of full scale. The
         * position field is one turn of 2^bits counts, and incoder.c works
         * modulo that. Code k is therefore k * 2^(bits - 12) counts, exactly,
         * and VREF+ is a whole turn: the same angle as 0 V.
         *
         * This was (2^bits - 1) / 4095 per code for a while, so that code
         * 4095 would land on 2^bits - 1. That is a gain of 4096/4095: +127
         * counts at the top of a 19-bit field, and a step of 1 count instead
         * of 128 from code 4095 across zero. The 127 counts above code 4095
         * are not unreachable by mistake; they are inside its bin.
         *
         * Scaling the sum rather than the average keeps the oversampling: the
         * step is 2^(bits - 12) / 12, 10.7 counts at 19 bits. That fills the
         * IncOder's numeric range, which is what a controller expects, but it
         * is not 19 bits of measurement: the source is a 12-bit ADC, and
         * averaging 12 samples is worth about 1.8 bits more (sqrt(12)) and
         * only when the input carries enough noise to dither. Reaching 19
         * honest bits would need 4^7 = 16384 samples per update, which does
         * not fit a 100 us period.
         *
         * With pole pairs, one ADC code is 2^(bits - 12) / pole_pairs counts
         * (10.7 at 19 bits and 12 pairs) and the sum's step is 1/12 of that.
         * The scaling below divides by ADC_AVG_SAMPLES * pole_pairs; with one
         * pole pair it is the one-turn mapping above, bit for bit.
         *
         * Rounded to nearest. The circular sum can sit up to two turns up, so
         * it is folded into one electrical turn before the turns are counted.
         * 32 bits are enough: the mechanical sum is under
         * POLE_PAIRS_MAX * ADC_TURN_SUM < 2^22, and shifted by at most 10. */
        const uint32_t pairs = s_pole_pairs;
        const uint32_t mech  = track_elec_turn(adc_sum() % ADC_TURN_SUM, pairs);
        const uint32_t div   = ADC_AVG_SAMPLES * pairs;
        return (((mech << (s_pos_bits - 12u)) + div / 2u) / div) & s_pos_mask;
    }
    }
}

bool position_source_valid(void)
{
    /* Synthetic sources are always valid. The analog source only becomes valid
     * once the DMA has filled the averaging window; before that PV must be 0,
     * which matches the IncOder's "Power Up Time To 1st Measurement".
     *
     * It also stops being valid on an overrun. In DMA mode an overrun blocks
     * the ADC's DMA requests [RM0390 Rev 9 13.8.1], so the window freezes on
     * its last values. Without this check the frames would go on reporting a
     * frozen position with PV=1. position_source_service() restarts it. */
    if (s_src != POS_SRC_ADC) {
        return true;
    }
    return s_adc_filled && (ANGLE_ADC->SR & ADC_SR_OVR) == 0u;
}

void position_source_service(void)
{
    if (!s_adc_running || (ANGLE_ADC->SR & ADC_SR_OVR) == 0u) {
        return;
    }

    /* PV stays 0 until the restarted DMA has filled a whole window again.
     * HAL_ADC_Start_DMA() clears OVR itself. */
    s_adc_filled = false;
    HAL_ADC_Stop_DMA(&s_adc);
    HAL_ADC_Start_DMA(&s_adc, (uint32_t *)s_samples, ADC_AVG_SAMPLES);
    s_adc_overruns++;
}

uint32_t position_source_overruns(void)
{
    return s_adc_overruns;
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ANGLE_ADC) {
        s_adc_filled = true;
    }
}
