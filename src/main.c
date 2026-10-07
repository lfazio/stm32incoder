/* simenc -- Zettlex IncOder emulator (SSI4) on NUCLEO-F446RE.
 *
 * Layering, bottom up:
 *   ssi_slave  generic SSI slave transport (TIM8 + DMA, Tmu gap, idle-high)
 *   ssi4       SSI4 payload codec (PV / ZPD / PD[18:0] / TS[10:0])
 *   incoder    sensor behaviour: position, zero point, timestamp, validity
 *
 * ssi_master is a bring-up instrument, not part of the emulator.
 */
#include "board/board.h"
#include "board/console.h"
#include "board/trace.h"
#include "encoder/incoder.h"
#include "encoder/position_source.h"
#include "ssi/ssi_variant.h"
#include "ssi/ssi_master.h"
#include "ssi/ssi_slave.h"

/* Internal position update tick: TIM2 update, 100 us. The same event triggers
 * the ADC in hardware, so the conversion and this latch stay in step. */
void TIM2_IRQHandler(void)
{
    if (TIM2->SR & TIM_SR_UIF) {
        TIM2->SR = (uint16_t)~TIM_SR_UIF;
        incoder_update();
    }
}

int main(void)
{
    board_init();
    trace_init();
    console_init();

    incoder_init();
    position_source_init();
    ssi_master_init();

    const ssi_slave_config_t ssi_cfg = {
        .n_bits = ssi_variant_frame_bits(SSI_VARIANT_DEFAULT),
        .tmu_us = 20u,               /* Tmu = 20 us +/- 1 us (5.4.1) */
    };
    if (!ssi_slave_init(&ssi_cfg, incoder_ssi_provider, NULL)) {
        trace_printf("FATAL: ssi_slave_init rejected the configuration\r\n");
        trace_flush();
        while (1) { }
    }

    /* Tick priority below the SSI interrupts so a frame is never delayed. */
    HAL_NVIC_SetPriority(TIM2_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(TIM2_IRQn);
    TIM2->DIER |= TIM_DIER_UIE;

    position_source_start();
    ssi_slave_start();

    console_banner();

    uint32_t last_blink = 0;
    uint32_t last_stat  = 0;

    for (;;) {
        console_poll();
        ssi_slave_poll();

        uint32_t now = HAL_GetTick();

        if ((now - last_blink) >= 500u) {
            last_blink = now;
            board_led_toggle();
        }

        /* Holding B1 streams state, which is handy while probing with a scope. */
        if (board_button_pressed() && (now - last_stat) >= 250u) {
            last_stat = now;
            incoder_state_t st;
            incoder_get_state(&st);
            trace_printf("pos=%lu ts=%u pv=%u adc=%u\r\n",
                         (unsigned long)st.position, (unsigned)st.timestamp,
                         (unsigned)st.valid, (unsigned)position_source_raw_adc());
        }
    }
}
