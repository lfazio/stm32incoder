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

/* The latched measurement, double-buffered.
 *
 * The update tick writes it at priority 2; the Tmu handler reads it through
 * the provider at priority 0. Separate variables let the reader preempt the
 * writer between two stores and send a new TS with the previous update's PD
 * and PV. Here the tick fills the slot nobody is reading and publishes it with
 * one store to s_cur. The reader preempts the writer, never the reverse, so a
 * slot it reads is never being written. */
static ssi_sample_t      s_sample[2];
static volatile uint8_t  s_cur;
static volatile uint32_t s_updates;
static volatile ssi_variant_t s_variant = SSI_VARIANT_DEFAULT;
static volatile uint32_t s_zero_offset;
static volatile bool     s_zero_default = true;
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

    /* Park the transport before anything changes. Otherwise a frame staged in
     * between -- by the Tmu handler or by the update tick's refresh -- packs
     * the new variant into the old frame length. ssi_slave_set_frame_bits()
     * re-arms it below. */
    ssi_slave_stop();
    s_variant = v;

    /* The position field width and the Time Stamp resolution both come from
     * the variant, so retune both before the transport starts moving the new
     * frame length. */
    position_source_set_width(ssi_variant_position_bits(v));
    timestamp_timer_init();

    if (!ssi_slave_set_frame_bits(ssi_variant_frame_bits(v))) {
        ssi_slave_start();      /* never leave the link parked */
        return false;
    }
    return true;
}

ssi_variant_t incoder_variant(void)
{
    return s_variant;
}

void incoder_init(void)
{
    timestamp_timer_init();
    s_sample[0]    = (ssi_sample_t){ .pv = false, .zpd = true, .pd = 0u, .ts = 0u };
    s_sample[1]    = s_sample[0];
    s_cur          = 0;
    s_updates      = 0;
    s_zero_offset  = 0;
    s_zero_default = true;
}

uint16_t incoder_timestamp_now(void)
{
    return (uint16_t)(TS_TIM->CNT & SSI_TIMESTAMP_MAX);
}

/* TS ticks to take off the latched counter so that TS marks the middle of the
 * source's measurement window rather than its end (5.4.2: "the value of the
 * Time Stamp counter when the position was measured"). Rounded to the
 * nearest tick of the active variant: 5 for SSI4's 10 us, 52 for SSI9's 1 us. */
static uint32_t timestamp_lag_ticks(ssi_variant_t v)
{
    uint32_t tick_us = ssi_variant_ts_tick_us(v);

    if (tick_us == 0u) {
        tick_us = SSI_TIMESTAMP_TICK_US;
    }
    return (position_source_delay_us() + tick_us / 2u) / tick_us;
}

void incoder_update(void)
{
    const ssi_variant_t v = s_variant;

    position_source_tick();

    /* Read the counter first, right as the source is read: the 64-bit scaling
     * in position_source_read() takes about a microsecond. */
    const uint16_t now  = incoder_timestamp_now();
    const uint32_t raw  = position_source_read();
    const uint32_t mask = (1u << ssi_variant_position_bits(v)) - 1u;

    /* The zero commands write offset and ZPD with interrupts masked, so this
     * tick can never see one without the other. */
    const uint32_t offset = s_zero_offset;
    const bool     zpd    = s_zero_default;

    /* Latch position and its timestamp together: TS reports when the position
     * was measured, not when it is transmitted. */
    const uint8_t next = (uint8_t)(s_cur ^ 1u);
    ssi_sample_t *slot = &s_sample[next];
    slot->pd  = (raw - offset) & mask;
    slot->ts  = (uint16_t)((now - timestamp_lag_ticks(v)) & SSI_TIMESTAMP_MAX);
    slot->pv  = position_source_valid() && !s_force_error;
    slot->zpd = zpd;
    __DMB();
    s_cur = next;
    s_updates++;

    /* Put it on the line now, not at the end of the next Read Cycle. */
    ssi_slave_refresh();
}

void incoder_get_state(incoder_state_t *out)
{
    const ssi_sample_t s = s_sample[s_cur];

    out->position     = s.pd;
    out->timestamp    = s.ts;
    out->valid        = s.pv;
    out->zero_default = s.zpd;
    out->updates      = s_updates;
}

static void zero_write(uint32_t offset, bool is_default)
{
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    s_zero_offset  = offset;
    s_zero_default = is_default;
    __set_PRIMASK(primask);
}

void incoder_zero_set(void)
{
    /* Make the current raw position read as zero. */
    zero_write(position_source_read(), false);
}

void incoder_zero_reset(void)
{
    zero_write(0u, true);
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

    /* One read of the published slot: PV, ZPD, PD and TS from one update. */
    const ssi_sample_t sample = s_sample[s_cur];

    out->payload = ssi_variant_pack(s_variant, &sample);
    /* PV is the inverse of the ERROR FLAG (5.4.2), and the ERROR FLAG is what
     * the data line carries during the gap (5.4.1 note 3). */
    out->error_flag = !sample.pv;
}
