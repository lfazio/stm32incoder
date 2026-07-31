#include "ssi/ssi_variant.h"

#include <string.h>

/* CRC-8 for SSI6, exactly as the guide specifies it: polynomial 0x97, initial
 * value 0x00, MSB first (not reversed), no final XOR. */
static uint8_t crc8(uint32_t data24)
{
    uint8_t bytes[3] = {
        (uint8_t)((data24 >> 16) & 0xFFu),
        (uint8_t)((data24 >> 8) & 0xFFu),
        (uint8_t)(data24 & 0xFFu),
    };
    uint8_t crc = 0x00u;

    for (uint32_t i = 0; i < 3u; i++) {
        crc ^= bytes[i];
        for (uint32_t b = 0; b < 8u; b++) {
            crc = (crc & 0x80u) ? (uint8_t)((crc << 1) ^ 0x97u)
                                : (uint8_t)(crc << 1);
        }
    }
    return crc;
}

/* Parity over the position field for SSI2: 0 for an even number of 1s in the
 * data (D23-D2), 1 for an odd number. */
static uint32_t parity22(uint32_t pd)
{
    uint32_t v = pd & 0x3FFFFFu;
    v ^= v >> 16;
    v ^= v >> 8;
    v ^= v >> 4;
    v ^= v >> 2;
    v ^= v >> 1;
    return v & 1u;
}

typedef struct {
    const char *name;
    uint8_t     frame_bits;
    uint8_t     position_bits;
    uint16_t    ts_tick_us;
} variant_info_t;

static const variant_info_t s_info[SSI_VARIANT_COUNT] = {
    [SSI_VARIANT_1] = { "ssi1", 24u, 22u, 0u  },
    [SSI_VARIANT_2] = { "ssi2", 24u, 22u, 0u  },
    [SSI_VARIANT_4] = { "ssi4", 32u, 19u, 10u },
    [SSI_VARIANT_6] = { "ssi6", 32u, 22u, 0u  },
    [SSI_VARIANT_9] = { "ssi9", 32u, 19u, 1u  },
};

const char *ssi_variant_name(ssi_variant_t v)
{
    return (v < SSI_VARIANT_COUNT) ? s_info[v].name : "?";
}

uint8_t ssi_variant_frame_bits(ssi_variant_t v)
{
    return (v < SSI_VARIANT_COUNT) ? s_info[v].frame_bits : 0u;
}

uint8_t ssi_variant_position_bits(ssi_variant_t v)
{
    return (v < SSI_VARIANT_COUNT) ? s_info[v].position_bits : 0u;
}

uint16_t ssi_variant_ts_tick_us(ssi_variant_t v)
{
    return (v < SSI_VARIANT_COUNT) ? s_info[v].ts_tick_us : 0u;
}

uint32_t ssi_variant_pack(ssi_variant_t v, const ssi_sample_t *s)
{
    uint32_t pd22 = s->pd & 0x3FFFFFu;

    switch (v) {
    case SSI_VARIANT_1:
        return ((uint32_t)(s->pv  ? 1u : 0u) << 23)
             | ((uint32_t)(s->zpd ? 1u : 0u) << 22)
             | pd22;

    case SSI_VARIANT_2:
        /* Alarm is the inverse sense of PV: 1 indicates an error condition. */
        return (pd22 << 2)
             | (parity22(pd22) << 1)
             | (s->pv ? 0u : 1u);

    case SSI_VARIANT_6: {
        uint32_t body = ((uint32_t)(s->pv  ? 1u : 0u) << 23)
                      | ((uint32_t)(s->zpd ? 1u : 0u) << 22)
                      | pd22;
        return ((uint32_t)crc8(body) << 24) | body;
    }

    case SSI_VARIANT_4:
    case SSI_VARIANT_9:
    default:
        /* Identical layout; only the Time Stamp resolution differs, and that
         * lives in the counter feeding it, not in the encoding.
         * D31 PV, D30 ZPD, D29-D11 PD[18:0], D10-D0 TS[10:0]. */
        return ((uint32_t)(s->pv  ? 1u : 0u) << 31)
             | ((uint32_t)(s->zpd ? 1u : 0u) << 30)
             | ((s->pd & 0x7FFFFu) << 11)
             | ((uint32_t)s->ts & SSI_TIMESTAMP_MAX);
    }
}

void ssi_variant_unpack(ssi_variant_t v, uint32_t raw, ssi_sample_t *s)
{
    switch (v) {
    case SSI_VARIANT_1:
        s->pv  = (raw >> 23) & 1u;
        s->zpd = (raw >> 22) & 1u;
        s->pd  = raw & 0x3FFFFFu;
        s->ts  = 0;
        break;

    case SSI_VARIANT_2:
        s->pd  = (raw >> 2) & 0x3FFFFFu;
        s->pv  = ((raw & 1u) == 0u);       /* alarm clear means data valid */
        s->zpd = true;                     /* SSI2 does not carry ZPD */
        s->ts  = 0;
        break;

    case SSI_VARIANT_6:
        s->pv  = (raw >> 23) & 1u;
        s->zpd = (raw >> 22) & 1u;
        s->pd  = raw & 0x3FFFFFu;
        s->ts  = 0;
        break;

    case SSI_VARIANT_4:
    case SSI_VARIANT_9:
    default:
        s->pv  = (raw >> 31) & 1u;
        s->zpd = (raw >> 30) & 1u;
        s->pd  = (raw >> 11) & 0x7FFFFu;
        s->ts  = (uint16_t)(raw & SSI_TIMESTAMP_MAX);
        break;
    }
}

bool ssi_variant_check(ssi_variant_t v, uint32_t raw)
{
    switch (v) {
    case SSI_VARIANT_2:
        return parity22((raw >> 2) & 0x3FFFFFu) == ((raw >> 1) & 1u);
    case SSI_VARIANT_6:
        return crc8(raw & 0x00FFFFFFu) == (uint8_t)((raw >> 24) & 0xFFu);
    default:
        return true;
    }
}

bool ssi_variant_from_name(const char *name, ssi_variant_t *out)
{
    if (name == NULL) {
        return false;
    }
    /* Accept both "4" and "ssi4". */
    if (strncmp(name, "ssi", 3) == 0) {
        name += 3;
    }
    for (uint32_t i = 0; i < (uint32_t)SSI_VARIANT_COUNT; i++) {
        if (strcmp(name, s_info[i].name + 3) == 0) {
            *out = (ssi_variant_t)i;
            return true;
        }
    }
    return false;
}
