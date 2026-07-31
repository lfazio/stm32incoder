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

/* Runs one Read Cycle of n_bits and returns the raw message, MSB first.
 * Returns false if the line was not idle beforehand. */
bool ssi_master_read(uint8_t n_bits, uint32_t *raw);

/* Selects the clock rate. Only the prescaler values that land inside the
 * SSI window are accepted; returns the rate actually programmed, in Hz. */
uint32_t ssi_master_set_clock(uint32_t requested_hz);
uint32_t ssi_master_get_clock(void);

void ssi_master_delay_us(uint32_t us);

/* Runs one Read Cycle at exactly 2.000 MHz, the SSI maximum.
 *
 * The SPI baud generator cannot produce it: SPI2 is clocked from PCLK1 =
 * 45 MHz and divides by powers of two, so the closest legal rates are
 * 1.40625 MHz and 2.8125 MHz (the latter 40 % over the limit). This engine
 * instead drives the clock pin as a plain GPIO, writing GPIOB->BSRR from DMA
 * on TIM1 compare events -- TIM1 runs at 180 MHz, and 180/90 = 2.000 MHz
 * exactly. DATA is captured by a third DMA reading GPIOB->IDR.
 *
 * The pin assignment and wiring are unchanged; only the pin's mode differs
 * while this engine runs.
 *
 * This is the worst case for the slave: its EXTI3 handover has just half a
 * clock period, 250 ns, to put DATA on the line before the master samples. */
bool     ssi_master_read_2mhz(uint8_t n_bits, uint32_t *raw);
uint32_t ssi_master_exact_hz(void);

/* Post-mortem of the last 2 MHz burst: NDTR of the low/high/sample streams,
 * then TIM1 CNT, TIM1 SR, DMA2 LISR, DMA2 HISR. */
void ssi_master_2mhz_debug(uint32_t *out7);

/* Continuity self-test for the loopback jumpers.
 *
 * Drives the two source pins (PB10 clock out, PB4 data out) as plain GPIO and
 * checks that the destination pins (PB3, PB14) follow both levels. Answers
 * "is the wire actually there" without a meter. The SSI slave must be
 * re-armed with ssi_slave_start() afterwards. */
bool ssi_loopback_check(bool *clock_ok, bool *data_ok);

#endif /* SIMENC_SSI_MASTER_H */
