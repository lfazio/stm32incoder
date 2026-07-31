/* Where the emulated shaft angle comes from.
 *
 * The analog input is the normal source, but loopback bring-up needs to run
 * without any signal wired to A0, so synthetic sources can replace it at
 * runtime from the console.
 *
 * Mapping of the analog input to position is a choice of this emulator, not a
 * value from the IncOder specification: the full ADC span (0 V .. VDDA) maps
 * linearly onto the full 19-bit position range, i.e. 0 V = 0 counts = 0 deg
 * and VDDA = 524287 counts ~ 360 deg. The ADC is 12-bit, so an analog-driven
 * position moves in steps of 128 counts even though the SSI4 field is 19-bit.
 */
#ifndef SIMENC_POSITION_SOURCE_H
#define SIMENC_POSITION_SOURCE_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    POS_SRC_ADC = 0,  /* PA0 / A0, timer-triggered, DMA fed */
    POS_SRC_FIXED,    /* constant value, set from the console */
    POS_SRC_RAMP,     /* free-running sweep, for loopback tests */
} position_source_t;

void position_source_init(void);

/* Sets the position field width, which the selected SSI variant decides
 * (19 bits for SSI4/SSI9, 22 for SSI1/SSI2/SSI6). Rescales the analog mapping
 * so full scale still means a full revolution. */
void position_source_set_width(uint8_t bits);

/* Starts the ADC + trigger timer. Safe to call even when another source is
 * selected; the conversions simply go unused. */
void position_source_start(void);

void              position_source_select(position_source_t src);
position_source_t position_source_get(void);
const char       *position_source_name(position_source_t src);

/* Constant value used by POS_SRC_FIXED, and ramp step used by POS_SRC_RAMP
 * (counts added per internal update period). */
void position_source_set_fixed(uint32_t counts);
void position_source_set_ramp_step(int32_t counts_per_update);

/* Raw last ADC sample, for tracing. */
uint16_t position_source_raw_adc(void);

/* Current 19-bit position, already clamped to the SSI4 field width. */
uint32_t position_source_read(void);

/* True when the selected source has produced at least one usable value. This
 * drives the PV (Position Valid) flag. */
bool position_source_valid(void);

/* Advances time-dependent sources. Called from the position update tick. */
void position_source_tick(void);

#endif /* SIMENC_POSITION_SOURCE_H */
