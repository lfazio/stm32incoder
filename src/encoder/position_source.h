/* Where the emulated shaft angle comes from.
 *
 * The analog input is the normal source, but loopback bring-up needs to run
 * without any signal wired to A0, so synthetic sources can replace it at
 * runtime from the console.
 *
 * Mapping of the analog input to position is a choice of this emulator, not a
 * value from the IncOder specification. The analog input is the *electrical*
 * angle of a motor with pole_pairs pole pairs (12 by default): the full ADC
 * span (0 V .. VDDA) is one electrical turn, and VDDA is the same angle as
 * 0 V. One electrical turn is 1/pole_pairs of a mechanical turn, and the
 * encoder reports the mechanical angle, so the emulator counts the electrical
 * turns and reports (k + electrical) / pole_pairs. One ADC code is
 * 2^19 / 4096 / pole_pairs counts of the 19-bit SSI4 field, 10.7 at 12 pairs;
 * with one pole pair it is 128, and VDDA = 2^19 counts = 360 deg.
 *
 * Because 0 V and VDDA are one angle, the average is taken on the circle
 * (circular_mean.h): an input ramp that wraps from one end of the ADC span to
 * the other reads as continuous rotation.
 */
#ifndef SIMENC_POSITION_SOURCE_H
#define SIMENC_POSITION_SOURCE_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    POS_SRC_ADC = 0,  /* PA0 / A0, free-running, DMA fed, 12x circular mean */
    POS_SRC_FIXED,    /* constant value, set from the console */
    POS_SRC_RAMP,     /* free-running sweep, for loopback tests */
} position_source_t;

void position_source_init(void);

/* Sets the position field width, which the selected SSI variant decides
 * (19 bits for SSI4/SSI9, 22 for SSI1/SSI2/SSI6). Rescales the analog mapping
 * so full scale still means a full revolution. */
void position_source_set_width(uint8_t bits);

/* Starts the ADC and the update-tick timer. Safe to call even when another
 * source is selected; the conversions simply go unused. */
void position_source_start(void);

void              position_source_select(position_source_t src);
position_source_t position_source_get(void);
const char       *position_source_name(position_source_t src);

/* Constant value used by POS_SRC_FIXED, and ramp step used by POS_SRC_RAMP
 * (counts added per internal update period). */
void position_source_set_fixed(uint32_t counts);
void position_source_set_ramp_step(int32_t counts_per_update);

/* Circular mean of the ADC averaging window in codes, 0..4095, for tracing. */
uint16_t position_source_raw_adc(void);

/* Pole pairs of the motor whose electrical angle PA0 carries, 1..64; 12 at
 * power-on. Setting it restarts the electrical turn count at 0. Returns false,
 * changing nothing, outside 1..64. Synthetic sources are not affected: their
 * values are already mechanical counts. */
bool     position_source_set_pole_pairs(uint32_t pairs);
uint32_t position_source_pole_pairs(void);

/* Electrical turn the analog input is in, 0 .. pole_pairs - 1, for tracing. */
uint32_t position_source_elec_turn(void);

/* Current mechanical position, already clamped to the variant's field width.
 *
 * Stateful for the analog source: it counts the electrical turns between one
 * call and the next. Call it only from the update tick (incoder_update()); a
 * call from the main loop could be preempted by the tick halfway through and
 * then write back a stale turn count. */
uint32_t position_source_read(void);

/* True when the selected source has produced at least one usable value and
 * is still producing them (no ADC overrun). This drives the PV (Position
 * Valid) flag. */
bool position_source_valid(void);

/* How long before the update tick the value returned by
 * position_source_read() was measured, in microseconds: the centroid of the
 * ADC averaging window (~52 us), 0 for the synthetic sources. The Time Stamp is
 * set back by this much. */
uint32_t position_source_delay_us(void);

/* Restarts the ADC after an overrun. Call from the main loop. */
void     position_source_service(void);
uint32_t position_source_overruns(void);

/* Advances time-dependent sources. Called from the position update tick. */
void position_source_tick(void);

#endif /* SIMENC_POSITION_SOURCE_H */
