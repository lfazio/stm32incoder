#include "ssi/ssi4.h"

#define SSI4_PV_SHIFT   31u
#define SSI4_ZPD_SHIFT  30u
#define SSI4_PD_SHIFT   11u   /* LSB of PD is in D11 */
#define SSI4_TS_SHIFT   0u

uint32_t ssi4_pack(const ssi4_frame_t *f)
{
    return ((uint32_t)(f->pv  ? 1u : 0u) << SSI4_PV_SHIFT)
         | ((uint32_t)(f->zpd ? 1u : 0u) << SSI4_ZPD_SHIFT)
         | ((f->pd & SSI4_POSITION_MAX)  << SSI4_PD_SHIFT)
         | ((uint32_t)(f->ts & SSI4_TIMESTAMP_MAX) << SSI4_TS_SHIFT);
}

void ssi4_unpack(uint32_t raw, ssi4_frame_t *f)
{
    f->pv  = (raw >> SSI4_PV_SHIFT)  & 1u;
    f->zpd = (raw >> SSI4_ZPD_SHIFT) & 1u;
    f->pd  = (raw >> SSI4_PD_SHIFT)  & SSI4_POSITION_MAX;
    f->ts  = (uint16_t)((raw >> SSI4_TS_SHIFT) & SSI4_TIMESTAMP_MAX);
}
