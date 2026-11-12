/* Mean of ADC codes read as angles.
 *
 * The analog input is an angle: 0 V and VDDA are the same shaft position
 * (position_source.h). An arithmetic mean breaks as soon as a window straddles
 * that point -- 4000 and 100 average to 2050, half a turn from either. On a
 * 40 ms capture of a 6.1 kHz sawtooth on PA0, 62% of the 12-sample windows
 * straddled it, and their arithmetic mean only ranged 1180..2904 for an input
 * spanning 0..4079.
 *
 * Pure logic, no hardware: host-testable.
 */
#ifndef SIMENC_CIRCULAR_MEAN_H
#define SIMENC_CIRCULAR_MEAN_H

#include <stdint.h>

/* ADC codes per turn. 1 LSB is VREF+/4096 [DS10693 Rev 11 Figure 45], so
 * VREF+ itself -- the same angle as 0 V -- is code 4096, not 4095. */
#define CIRCULAR_TURN  4096u

/* Sum of the n codes in v[] (each 0..CIRCULAR_TURN-1), unwrapped onto the
 * shortest arc of the circle that holds them all. sum / n is their circular
 * mean in codes, modulo CIRCULAR_TURN, and lies in [0, 2 * CIRCULAR_TURN).
 *
 * Exact while the window leaves an empty stretch of the circle wider than the
 * widest gap between its samples: for a steady rotation, under
 * CIRCULAR_TURN / n codes per sample (341 for n = 12).
 *
 * The order of v[] does not matter, and v[] is sorted in place. 1 <= n. */
uint32_t circular_sum(uint16_t *v, uint32_t n);

#endif /* SIMENC_CIRCULAR_MEAN_H */
