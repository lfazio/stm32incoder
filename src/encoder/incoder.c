#include "encoder/incoder.h"
#include "encoder/position_source.h"
#include "board/board.h"

/* Time Stamp counter: "a continuously incrementing counter in the range
 * 0.00 ms to 20.47 ms (at which point it restarts at 0.00 ms). It has a
 * resolution of 10 us" (5.4.2).
 *
 * TIM7 is a basic timer on APB1, clocked at 90 MHz. A prescaler of 900 gives a
 * 100 kHz count (10 us per tick) and ARR = 2047 wraps after 2048 ticks, i.e.
 * 20.48 ms, so the counter spans 0.00 .. 20.47 ms exactly as specified.
 * Nothing reads it but the update tick, and it free-runs entirely in hardware.
 */
#define TS_TIM        TIM7

static volatile uint32_t s_position;
static volatile uint16_t s_timestamp;
static volatile bool     s_valid;
static volatile uint32_t s_updates;
static ssi_variant_t     s_variant = SSI_VARIANT_DEFAULT;
static uint32_t          s_zero_offset;
static bool              s_zero_default = true;
static volatile bool     s_force_error;

static void timestamp_timer_init(void)
{
    uint16_t tick_us = ssi_variant_ts_tick_us(s_variant);

    if (tick_us == 0u) {
        tick_us = SSI_TIMESTAMP_TICK_US;   /* variant carries no TS; keep it running */
    }

    __HAL_RCC_TIM7_CLK_ENABLE();

    TS_TIM->CR1  = 0;
    TS_TIM->PSC  = (uint16_t)((APB1_TIMCLK_HZ / 1000000u) * tick_us - 1u);
    TS_TIM->ARR  = SSI_TIMESTAMP_MAX;                   /* 2047 */
    TS_TIM->EGR  = TIM_EGR_UG;
    TS_TIM->SR   = 0;
    TS_TIM->DIER = 0;
    TS_TIM->CR1  = TIM_CR1_CEN;
}

bool incoder_set_variant(ssi_variant_t v)
{
    if (v >= SSI_VARIANT_COUNT) {
        return false;
    }
    s_variant = v;

    /* The position field width and the Time Stamp resolution both come from
     * the variant, so retune both before the transport starts moving the new
     * frame length. */
    position_source_set_width(ssi_variant_position_bits(v));
    timestamp_timer_init();

    return ssi_slave_set_frame_bits(ssi_variant_frame_bits(v));
}

ssi_variant_t incoder_variant(void)
{
    return s_variant;
}

void incoder_init(void)
{
    timestamp_timer_init();
    s_position     = 0;
    s_timestamp    = 0;
    s_valid        = false;
    s_updates      = 0;
    s_zero_offset  = 0;
    s_zero_default = true;
}

uint16_t incoder_timestamp_now(void)
{
    return (uint16_t)(TS_TIM->CNT & SSI_TIMESTAMP_MAX);
}

void incoder_update(void)
{
    position_source_tick();

    uint32_t mask = (1u << ssi_variant_position_bits(s_variant)) - 1u;
    uint32_t raw  = position_source_read();
    uint32_t pos  = (raw - s_zero_offset) & mask;

    /* Latch position and its timestamp together: TS reports when the position
     * was measured, not when it is transmitted. */
    s_timestamp = incoder_timestamp_now();
    s_position  = pos;
    s_valid     = position_source_valid() && !s_force_error;
    s_updates++;
}

void incoder_get_state(incoder_state_t *out)
{
    out->position     = s_position;
    out->timestamp    = s_timestamp;
    out->valid        = s_valid;
    out->zero_default = s_zero_default;
    out->updates      = s_updates;
}

void incoder_zero_set(void)
{
    /* Make the current raw position read as zero. */
    s_zero_offset  = position_source_read();
    s_zero_default = false;
}

void incoder_zero_reset(void)
{
    s_zero_offset  = 0;
    s_zero_default = true;
}

uint32_t incoder_zero_offset(void)
{
    return s_zero_offset;
}

void incoder_force_error(bool on)
{
    s_force_error = on;
}

bool incoder_error_forced(void)
{
    return s_force_error;
}

void incoder_ssi_provider(void *ctx, ssi_slave_frame_t *out)
{
    (void)ctx;

    ssi_sample_t sample = {
        .pv  = s_valid,
        .zpd = s_zero_default,
        .pd  = s_position,
        .ts  = s_timestamp,
    };

    out->payload = ssi_variant_pack(s_variant, &sample);
    /* PV is the inverse of the ERROR FLAG (5.4.2), and the ERROR FLAG is what
     * the data line carries during the gap (5.4.1 note 3). */
    out->error_flag = !sample.pv;
}
