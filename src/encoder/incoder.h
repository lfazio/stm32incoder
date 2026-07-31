/* Emulated Zettlex IncOder, SSI4 variant.
 *
 * Sits on top of the SSI4 codec and the generic SSI slave transport: this
 * module owns the sensor behaviour (position, zero point, timestamp, validity)
 * and knows nothing about SPI or DMA.
 *
 * Timing behaviour taken from the Product Guide:
 *   - Internal Position Update Period < 0.1 ms (4.12) -- we latch a new
 *     position every 100 us.
 *   - The Time Stamp field records the counter value at the moment the
 *     position was measured (5.4.2), so position and timestamp are latched
 *     together by the update tick, not when the frame is transmitted.
 *   - Zero Point: as supplied the zero point is the factory setting, which is
 *     what ZPD reports (5.2).
 */
#ifndef SIMENC_INCODER_H
#define SIMENC_INCODER_H

#include <stdbool.h>
#include <stdint.h>

#include "ssi/ssi_slave.h"
#include "ssi/ssi_variant.h"

typedef struct {
    uint32_t position;    /* 19-bit, zero-point corrected */
    uint16_t timestamp;   /* 11-bit, 10 us ticks */
    bool     valid;       /* drives PV */
    bool     zero_default;/* drives ZPD */
    uint32_t updates;     /* internal position updates since boot */
} incoder_state_t;

void incoder_init(void);

/* Selects the SSI payload variant. Reconfigures the transport's frame length,
 * the position field width and the Time Stamp tick, then re-arms. */
bool          incoder_set_variant(ssi_variant_t v);
ssi_variant_t incoder_variant(void);

/* Latches a new position together with its timestamp. Called from the 100 us
 * update tick. */
void incoder_update(void);

void incoder_get_state(incoder_state_t *out);

/* Zero Point handling (5.2). Setting a non-factory zero point clears ZPD;
 * resetting restores the factory zero point and sets ZPD. */
void incoder_zero_set(void);
void incoder_zero_reset(void);
uint32_t incoder_zero_offset(void);

/* Raw free-running Time Stamp counter, 0..2047. */
uint16_t incoder_timestamp_now(void);

/* Forces PV to 0, i.e. reports the error condition. Besides exercising the
 * Error Flag path, this is what makes D31 a meaningful test bit: PV is
 * normally 1, and the DATA line also idles HIGH, so a late first-bit handover
 * would be invisible. With PV forced to 0, a late handover shows up as D31
 * reading 1 instead of 0. */
void incoder_force_error(bool on);
bool incoder_error_forced(void);

/* Frame provider handed to the SSI transport. */
void incoder_ssi_provider(void *ctx, ssi_slave_frame_t *out);

#endif /* SIMENC_INCODER_H */
