/* SSI master -- loopback test harness only, not part of the emulated encoder.
 *
 * Drives a Read Cycle exactly as an SSI controller would: n clock pulses at a
 * rate inside the specified 100 kHz .. 2 MHz window, then an intermessage gap
 * longer than Tmu. Used to validate the slave first by jumper loopback at TTL
 * level on this board, and afterwards through a pair of MAX490 RS-422
 * transceivers.
 *
 * Deliberately simple (blocking, polled): it is a bring-up instrument, and the
 * timing-critical hardware path is the slave.
 */
#ifndef SIMENC_SSI_MASTER_H
#define SIMENC_SSI_MASTER_H

#include <stdbool.h>
#include <stdint.h>

void ssi_master_init(void);

/* True when the DATA line is idle HIGH, i.e. the slave says a new Read Cycle
 * may be started (section 5.4.1). */
bool ssi_master_data_idle_high(void);

void ssi_master_delay_us(uint32_t us);

/* Runs one Read Cycle at the clock rate set by ssi_master_set_exact_clock
 * (500 kHz at power-on).
 *
 * The clock pin is a plain GPIO driven by DMA writes to GPIOB->BSRR on TIM1
 * compare events; TIM1 runs at 180 MHz, so any rate of the form 180 MHz / N is
 * exact, including 2.000 MHz at the top of the SSI window. DATA is captured by
 * a third DMA reading GPIOB->IDR, late in the high phase -- where an SSI
 * controller samples, since the encoder sets each bit on the rising edge and it
 * stays valid across the fall. */
bool     ssi_master_read_timer(uint8_t n_bits, uint32_t *raw);
uint32_t ssi_master_exact_hz(void);

/* Sets the clock rate, anywhere in 100 kHz .. 2 MHz in steps of 180 MHz / N.
 * Returns the rate actually programmed. */
uint32_t ssi_master_set_exact_clock(uint32_t requested_hz);

/* Post-mortem of the last timer-engine burst: NDTR of the low/high/sample streams,
 * then TIM1 CNT, TIM1 SR, DMA2 LISR, DMA2 HISR. */
void ssi_master_timer_debug(uint32_t *out7);

/* Continuity self-test for the loopback jumpers.
 *
 * Drives the two source pins (PB10 clock out, PB4 data out) and checks that
 * the destinations follow both levels: PC6 for the clock -- the pin that
 * clocks TIM8, and so the one that matters -- and PB14 for DATA. Answers "is
 * the wire actually there" without a meter. The SSI slave must be re-armed
 * with ssi_slave_start() afterwards. */
bool ssi_loopback_check(bool *clock_ok, bool *data_ok);



#endif /* SIMENC_SSI_MASTER_H */
