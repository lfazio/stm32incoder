/* SSI4 frame codec -- IncOder Product Guide Rev 4.11.8, section 5.4.2.
 *
 * SSI4 is one of the SSI payload variants (n = 32 bits):
 *
 *   D31      PV        Position Valid flag. 1 when position data is valid,
 *                      otherwise 0 (the inverse of the ERROR FLAG).
 *   D30      ZPD       Zero Point Default. 1 when the Zero Point is at the
 *                      factory default, otherwise 0.
 *   D29-D11  PD[18:0]  Binary position data. If the resolution of the device
 *                      is less than 19 bits the MSBs of this field are 0.
 *                      The LSB of the field is in D11. When PV is 0 the value
 *                      is not defined.
 *   D10-D0   TS[10:0]  Time stamp: the value of the Time Stamp counter when
 *                      the position was measured. Always valid. Continuously
 *                      incrementing, 0.00 ms .. 20.47 ms, 10 us resolution.
 *
 * "Note: the use of SSI4 limits the measurement resolution to a maximum of
 * 19 bits."
 *
 * This module is pure logic -- no hardware, no state -- so the frame layout can
 * be unit tested on a host and reused by other SSI payload variants later.
 */
#ifndef SIMENC_SSI4_H
#define SIMENC_SSI4_H

#include <stdbool.h>
#include <stdint.h>

#define SSI4_FRAME_BITS      32u
#define SSI4_POSITION_BITS   19u
#define SSI4_POSITION_MAX    ((1u << SSI4_POSITION_BITS) - 1u)  /* 524287 */
#define SSI4_TIMESTAMP_BITS  11u
#define SSI4_TIMESTAMP_MAX   ((1u << SSI4_TIMESTAMP_BITS) - 1u) /* 2047 */

/* Time Stamp counter resolution, from the SSI4 field description. */
#define SSI4_TIMESTAMP_TICK_US   10u
#define SSI4_TIMESTAMP_PERIOD    (SSI4_TIMESTAMP_MAX + 1u)      /* 20.48 ms */

typedef struct {
    bool     pv;   /* D31 */
    bool     zpd;  /* D30 */
    uint32_t pd;   /* D29..D11, 19 bits */
    uint16_t ts;   /* D10..D0,  11 bits */
} ssi4_frame_t;

uint32_t ssi4_pack(const ssi4_frame_t *f);
void     ssi4_unpack(uint32_t raw, ssi4_frame_t *f);

#endif /* SIMENC_SSI4_H */
