#include "encoder/position_source.h"
#include "board/board.h"
#include "ssi/ssi4.h"

/* The IncOder specifies an "Internal Position Update Period" of < 0.1 ms for
 * all digital comms options (Product Guide 4.12). We run the acquisition at
 * 10 kHz (100 us), the fastest rate that still satisfies "< 0.1 ms". */
#define UPDATE_RATE_HZ   10000u

/* ADC noise smoothing. Averaging N samples costs N x 100 us of lag; 4 keeps
 * the lag (400 us) well inside a typical SSI polling interval. */
#define ADC_AVG_SAMPLES  4u

/* ADC1 -> DMA2 Stream 0, channel 0 (RM0390 Rev 9 Table 29). Stream 4 carries
 * the same request; streams 2/3 are taken by SPI1. */
#define ADC_DMA_STREAM   DMA2_Stream0
#define ADC_DMA_CHANNEL  0u

/* The ADC is 12-bit; the position field is 19 or 22 bits depending on variant,
 * so full scale is reached by shifting left. Either way the analog resolution
 * stays 12-bit -- the extra field width does not create information. */
static uint8_t  s_pos_bits = SSI4_POSITION_BITS;
static uint32_t s_pos_mask = SSI4_POSITION_MAX;

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
static volatile uint32_t s_adc_conversions;

static position_source_t s_src = POS_SRC_ADC;
static uint32_t          s_fixed;
static int32_t           s_ramp_step = 64;
static uint32_t          s_ramp;

static void tim_init(void)
{
    __HAL_RCC_TIM2_CLK_ENABLE();

    /* TIM2 sits on APB1; with the APB1 prescaler != 1 its clock is 90 MHz. */
    s_tim.Instance               = TIM2;
    s_tim.Init.Prescaler         = (APB1_TIMCLK_HZ / 1000000u) - 1u;  /* 1 MHz */
    s_tim.Init.CounterMode       = TIM_COUNTERMODE_UP;
    s_tim.Init.Period            = (1000000u / UPDATE_RATE_HZ) - 1u;  /* 100 us */
    s_tim.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    s_tim.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    HAL_TIM_Base_Init(&s_tim);

    /* Update event drives TRGO, which is what starts each ADC conversion. */
    TIM_MasterConfigTypeDef mc = {0};
    mc.MasterOutputTrigger = TIM_TRGO_UPDATE;
    mc.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
    HAL_TIMEx_MasterConfigSynchronization(&s_tim, &mc);
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

    /* ADC clock = PCLK2 / 8 = 11.25 MHz, inside the 36 MHz maximum. */
    s_adc.Instance                   = ANGLE_ADC;
    s_adc.Init.ClockPrescaler        = ADC_CLOCK_SYNC_PCLK_DIV8;
    s_adc.Init.Resolution            = ADC_RESOLUTION_12B;
    s_adc.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
    s_adc.Init.ScanConvMode          = DISABLE;
    s_adc.Init.EOCSelection          = ADC_EOC_SINGLE_CONV;
    s_adc.Init.ContinuousConvMode    = DISABLE;
    s_adc.Init.DiscontinuousConvMode = DISABLE;
    s_adc.Init.NbrOfConversion       = 1;
    s_adc.Init.ExternalTrigConv      = ADC_EXTERNALTRIGCONV_T2_TRGO;
    s_adc.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_RISING;
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

uint16_t position_source_raw_adc(void)
{
    uint32_t sum = 0;
    for (uint32_t i = 0; i < ADC_AVG_SAMPLES; i++) {
        sum += s_samples[i];
    }
    return (uint16_t)(sum / ADC_AVG_SAMPLES);
}

void position_source_tick(void)
{
    if (s_src == POS_SRC_RAMP) {
        s_ramp = (uint32_t)((int32_t)s_ramp + s_ramp_step) & s_pos_mask;
    }
}

uint32_t position_source_read(void)
{
    switch (s_src) {
    case POS_SRC_FIXED:
        return s_fixed;
    case POS_SRC_RAMP:
        return s_ramp;
    case POS_SRC_ADC:
    default:
        return ((uint32_t)position_source_raw_adc() << (s_pos_bits - 12u))
               & s_pos_mask;
    }
}

bool position_source_valid(void)
{
    /* Synthetic sources are always valid. The analog source only becomes valid
     * once the DMA has filled the averaging window; before that PV must be 0,
     * which matches the IncOder's "Power Up Time To 1st Measurement". */
    if (s_src != POS_SRC_ADC) {
        return true;
    }
    return s_adc_conversions >= ADC_AVG_SAMPLES;
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ANGLE_ADC) {
        s_adc_conversions += ADC_AVG_SAMPLES;
    }
}
