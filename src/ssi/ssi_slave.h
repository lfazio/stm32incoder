/* Generic SSI slave transport (the sensor side of the link).
 *
 * Protocol, from IncOder Product Guide Rev 4.11.8 section 5.4.1:
 *
 *   - RS-422 hardware standard: differential DATA output, differential CLOCK
 *     input. Neither is terminated with load resistors.
 *   - Clock period T with 1/T between 100 kHz and 2 MHz.
 *   - In idle state CLOCK and DATA are both HIGH.
 *   - The first falling edge after Tmu starts the Read Cycle and the transfer.
 *   - Each rising edge of CLOCK transmits the next data bit, starting Dn-1.
 *   - After the last rising edge the data line is set by the Error Flag for
 *     the period Tmu - 0.5xT.
 *   - Tmu (message update time) = 20 us +/- 1 us, measured from the last
 *     falling edge. After Tmu the DATA line is HIGH, indicating that a new
 *     Read Cycle can be started.
 *   - Timg (intermessage gap) must be > Tmu.
 *   - n is the number of bits in the message, not including the Error Flag.
 *
 * This layer is payload agnostic: it moves n bits and knows nothing about
 * what they mean. SSI4 (and later SSI1/2/6/9) sit on top of it.
 *
 * Implementation: no peripheral shifts the bits. The incoming SSI clock drives
 * TIM8 through its channel-1 edge detector, counting both edges, and two
 * compare channels raise DMA requests that write GPIOB->BSRR -- one per rising
 * edge for the next data bit, one per falling edge for the Error Flag slot.
 * DATA is therefore set on the rising edge, as 5.4.1 note 2 requires, and the
 * DMA's transfer-complete marks the end of the n-bit message. No interrupt runs
 * per bit, and the data pin never changes owner mid-frame.
 */
#ifndef SIMENC_SSI_SLAVE_H
#define SIMENC_SSI_SLAVE_H

#include <stdbool.h>
#include <stdint.h>

/* Payload handed to the transport for the next Read Cycle. */
typedef struct {
    uint32_t payload;     /* right-aligned in cfg.n_bits, sent MSB first */
    bool     error_flag;  /* level driven on DATA during the gap after the frame */
} ssi_slave_frame_t;

/* Called to fetch the frame that the next Read Cycle will transmit.
 * Runs in interrupt context at end-of-frame: keep it short. */
typedef void (*ssi_slave_provider_t)(void *ctx, ssi_slave_frame_t *out);

typedef struct {
    uint8_t  n_bits;  /* message length excluding the Error Flag; multiple of 8, <= 32 */
    uint16_t tmu_us;  /* message update time, 20 for the IncOder */
} ssi_slave_config_t;

typedef struct {
    uint32_t frames;     /* completed Read Cycles */
    uint32_t resyncs;    /* frames abandoned mid-message and re-armed */
    uint32_t period_ns;  /* master clock period measured during the last frame */
} ssi_slave_stats_t;

bool ssi_slave_init(const ssi_slave_config_t *cfg,
                    ssi_slave_provider_t provider, void *ctx);

/* Arms the transport: DATA goes HIGH and the first falling edge will clock a
 * message out. */
void ssi_slave_start(void);

/* Parks the transport: stops the clock counter and both DATA DMA streams so
 * something else can drive the DATA pin. Call ssi_slave_start() to resume.
 * Needed by the loopback continuity test, which would otherwise race the DMA
 * for the pin it is trying to measure. */
void ssi_slave_stop(void);

/* Changes the message length, for switching SSI payload variant. Disarms,
 * reconfigures and re-arms; n_bits must still be a multiple of 8 and <= 32. */
bool ssi_slave_set_frame_bits(uint8_t n_bits);

/* Recovers from a master that abandoned a Read Cycle part-way through. Call
 * periodically from the main loop; it is a no-op when the link is healthy. */
void ssi_slave_poll(void);

void ssi_slave_get_stats(ssi_slave_stats_t *out);

/* True once the clock counter has actually reached n and ended a message.
 *
 * The clock arrives on PC6 (TIM8_CH1, morpho CN10-4) and is what shifts DATA
 * out; without that wire the counter never reaches n, no message ever ends and
 * the link is simply dead -- a symptom with nothing pointing at its cause, so
 * `stat` reports it. Reaching n is the property worth reporting: a floating
 * input picks up enough noise to make "saw an edge" true while the link stays
 * dead. */
bool ssi_slave_clock_counter_ok(void);

#endif /* SIMENC_SSI_SLAVE_H */
