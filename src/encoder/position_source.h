/* Where the emulated shaft angle comes from.
 *
 * The analog input is the normal source, but loopback bring-up needs to run
 * without any signal wired to A0, so synthetic sources can replace it at
 * runtime from the console.
 *
 * Mapping of the analog input to position is a choice of this emulator, not a
 * value from the IncOder specification: the full ADC span (0 V .. VDDA) maps
 * linearly onto one turn, i.e. 0 V = 0 counts = 0 deg and VDDA = 2^19 counts
 * = 360 deg, the same angle as 0 V. One ADC code is 128 counts of the 19-bit
 * SSI4 field; the 12-sample average refines that to 10.7.
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

/* Current 19-bit position, already clamped to the SSI4 field width. */
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
