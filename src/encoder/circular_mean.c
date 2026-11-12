#include "encoder/circular_mean.h"

/* Seen on the circle, a window of samples occupies an arc and leaves the rest
 * empty. The widest gap between neighbouring samples is that empty stretch, so
 * the arc starts just after it. Lifting the samples that come before the cut,
 * in ascending order, by one turn makes the arc contiguous, and a plain sum
 * finishes the job.
 *
 * Two simpler unwraps fail on the signal that prompted this:
 *
 * - Folding every sample to within half a turn of one reference sample only
 *   works while the whole window spans less than half a turn. The 6.1 kHz
 *   sawtooth moves 211 codes per conversion, 2321 across 12: a sample at one
 *   end of the window lands on the wrong side of a reference at the other.
 *
 * - Unwrapping each sample against its predecessor in time needs the DMA
 *   buffer's chronological order, which races the DMA, and it breaks on the
 *   generator's retrace: one sample caught halfway down splits the jump into
 *   two steps of less than half a turn, both read as forward motion, and every
 *   later sample in the window ends up a whole turn off.
 *
 * A retrace sample still pulls the windows that contain it here, but only by
 * its own share of the sum, not by a turn. A real encoder has no retrace.
 *
 * Twelve samples at 10 kHz: an insertion sort is the cheapest sort that fits,
 * and it is already sorted when the input stands still. */
uint32_t circular_sum(uint16_t *v, uint32_t n)
{
    for (uint32_t i = 1u; i < n; i++) {
        const uint16_t x = v[i];
        uint32_t j = i;
        while (j > 0u && v[j - 1u] > x) {
            v[j] = v[j - 1u];
            j--;
        }
        v[j] = x;
    }

    /* The gap that runs from the largest sample round through zero to the
     * smallest wins ties, so a window that does not straddle zero is never
     * lifted. */
    uint32_t cut    = 0u;
    uint32_t widest = (uint32_t)v[0] + CIRCULAR_TURN - v[n - 1u];
    for (uint32_t i = 1u; i < n; i++) {
        const uint32_t gap = (uint32_t)v[i] - v[i - 1u];
        if (gap > widest) {
            widest = gap;
            cut    = i;
        }
    }

    uint32_t sum = 0u;
    for (uint32_t i = 0u; i < n; i++) {
        sum += v[i];
    }
    /* v[0] .. v[cut - 1] lie past zero: one turn each. */
    return sum + cut * CIRCULAR_TURN;
}
