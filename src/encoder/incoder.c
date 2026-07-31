#include "encoder/incoder.h"
#include "encoder/position_source.h"
#include "ssi/ssi4.h"
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
#define TS_TICK_HZ    (1000000u / SSI4_TIMESTAMP_TICK_US)   /* 100 kHz */

static volatile uint32_t s_position;
static volatile uint16_t s_timestamp;
static volatile bool     s_valid;
static volatile uint32_t s_updates;
static uint32_t          s_zero_offset;
static bool              s_zero_default = true;

static void timestamp_timer_init(void)
{
    __HAL_RCC_TIM7_CLK_ENABLE();

    TS_TIM->CR1  = 0;
    TS_TIM->PSC  = (APB1_TIMCLK_HZ / TS_TICK_HZ) - 1u;   /* 899 */
    TS_TIM->ARR  = SSI4_TIMESTAMP_MAX;                   /* 2047 */
    TS_TIM->EGR  = TIM_EGR_UG;
    TS_TIM->SR   = 0;
    TS_TIM->DIER = 0;
    TS_TIM->CR1  = TIM_CR1_CEN;
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
    return (uint16_t)(TS_TIM->CNT & SSI4_TIMESTAMP_MAX);
}

void incoder_update(void)
{
    position_source_tick();

    uint32_t raw = position_source_read();
    uint32_t pos = (raw - s_zero_offset) & SSI4_POSITION_MAX;

    /* Latch position and its timestamp together: TS reports when the position
     * was measured, not when it is transmitted. */
    s_timestamp = incoder_timestamp_now();
    s_position  = pos;
    s_valid     = position_source_valid();
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

void incoder_ssi_provider(void *ctx, ssi_slave_frame_t *out)
{
    (void)ctx;

    ssi4_frame_t f = {
        .pv  = s_valid,
        .zpd = s_zero_default,
        .pd  = s_position,
        .ts  = s_timestamp,
    };

    out->payload = ssi4_pack(&f);
    /* PV is the inverse of the ERROR FLAG (5.4.2), and the ERROR FLAG is what
     * the data line carries during the gap (5.4.1 note 3). */
    out->error_flag = !f.pv;
}
