/* SSI payload variants -- IncOder Product Guide Rev 4.11.8, section 5.4.2.
 *
 * The transport moves n bits and knows nothing of their meaning; these are the
 * codecs that give those bits meaning. All are pure logic, no hardware, so the
 * frame layouts can be checked on a host.
 *
 * Only the byte-aligned variants are here, because ssi_slave transfers whole
 * bytes:
 *
 *   SSI1  n=24  D23 PV, D22 ZPD, D21-D0 PD[21:0]
 *   SSI2  n=24  D23-D2 PD[21:0], D1 parity, D0 alarm
 *   SSI4  n=32  D31 PV, D30 ZPD, D29-D11 PD[18:0], D10-D0 TS[10:0] @ 10 us
 *   SSI6  n=32  D31-D24 CRC-8, D23 PV, D22 ZPD, D21-D0 PD[21:0]
 *   SSI9  n=32  as SSI4 but the Time Stamp counts in 1 us steps
 *
 * SSI7 (n=30) and SSI8 (n=18) are deliberately absent: they are not byte
 * aligned, so they need bit-level padding in the transport first.
 *
 * Note the guide's own caps on resolution: SSI4 and SSI9 limit measurement to
 * 19 bits, the others carry 22.
 */
#ifndef SIMENC_SSI_VARIANT_H
#define SIMENC_SSI_VARIANT_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    SSI_VARIANT_1 = 0,
    SSI_VARIANT_2,
    SSI_VARIANT_4,
    SSI_VARIANT_6,
    SSI_VARIANT_9,
    SSI_VARIANT_COUNT
} ssi_variant_t;

#define SSI_VARIANT_DEFAULT  SSI_VARIANT_4

/* What the sensor knows, before any variant decides how to lay it out. */
typedef struct {
    bool     pv;   /* position valid; the ERROR FLAG is its inverse */
    bool     zpd;  /* zero point is the factory one */
    uint32_t pd;   /* position, right-aligned in the variant's position width */
    uint16_t ts;   /* time stamp counter, for the variants that carry one */
} ssi_sample_t;

const char *ssi_variant_name(ssi_variant_t v);

/* Message length n, excluding the Error Flag. Always a multiple of 8 here. */
uint8_t ssi_variant_frame_bits(ssi_variant_t v);

/* Width of the position field, which also caps the usable resolution. */
uint8_t ssi_variant_position_bits(ssi_variant_t v);

/* Time Stamp resolution in microseconds, or 0 when the variant has no TS.
 * The counter always spans 2048 steps, so this also sets its wrap period. */
uint16_t ssi_variant_ts_tick_us(ssi_variant_t v);

uint32_t ssi_variant_pack(ssi_variant_t v, const ssi_sample_t *s);
void     ssi_variant_unpack(ssi_variant_t v, uint32_t raw, ssi_sample_t *s);

/* True when the frame carries its own integrity check and it verifies.
 * Variants without one always return true. */
bool ssi_variant_check(ssi_variant_t v, uint32_t raw);

/* Accepts "1", "2", "4", "6", "9" and the "ssi4" spellings. */
bool ssi_variant_from_name(const char *name, ssi_variant_t *out);

#endif /* SIMENC_SSI_VARIANT_H */
