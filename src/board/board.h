/* Board definitions for NUCLEO-F446RE (MB1136).
 *
 * Every pin/AF/DMA value below is taken from:
 *   [DS]  DS10693 Rev 11, STM32F446xC/E datasheet, Table 10 (pin descriptions,
 *         incl. the I/O structure column) and Table 11 (alternate function).
 *   [UM]  UM1724 Rev 17, Table 19 (ARDUINO connectors on NUCLEO-F446RE) and
 *         Table 29 (ST morpho connector on NUCLEO-F401RE/F411RE/F446RE).
 *   [RM]  RM0390 Rev 9, Table 28 (DMA1 request mapping) and Table 29 (DMA2).
 */
#ifndef SIMENC_BOARD_H
#define SIMENC_BOARD_H

#include "stm32f4xx_hal.h"
#include <stdbool.h>
#include <stdint.h>

/* ---------------------------------------------------------------------------
 * Pin map
 *
 * SSI slave (the emulated IncOder). No SPI is involved: the incoming clock
 * drives a timer, and DMA shifts each bit onto a plain GPIO.
 *
 *   PC6  TIM8_CH1  AF3  [DS Table 11 p.60]  CN10-4   FT, 5 V tolerant
 *        <- SSI CLOCK in   (from MAX490 receiver output RO)
 *   PB4  GPIO out        [DS Table 11 p.59]  CN9-6 (D5) / CN10-27
 *        -> SSI DATA out  (to MAX490 driver input DI), written by DMA via BSRR
 *
 * SSI master (loopback test harness only, TIM1 compare events + DMA):
 *   PB10 GPIO out        CN9-7 (D6) / CN10-25   clock out
 *   PB14 GPIO in         CN10-28                DATA in, sampled from IDR
 *
 * Analog angle input:
 *   PA0  ADC1_IN0       [DS Table 10 p.47]  CN8-1  (A0)  / CN7-28
 *        NOTE PA0 is FT, but when used as an analog input it must stay within
 *        0..VDDA (3.3 V). Do not drive it from 5 V logic.
 *
 * Trace / console: USART2 PA2/PA3 -> ST-LINK virtual COM port [UM 6.8],
 * default solder bridge configuration (SB13/SB14 ON).
 *
 * Status: LD2 on PA5 [UM 7.6], user button B1 on PC13 [UM 7.7].
 * ------------------------------------------------------------------------- */

#define SSI_SLAVE_GPIO           GPIOB
#define SSI_SLAVE_DATA_PIN       GPIO_PIN_4
#define SSI_SLAVE_DATA_PIN_NUM   4u

/* DATA shifted out by the incoming clock itself.
 *
 * TIM8 is clocked by TI1F_ED, the channel-1 edge detector, which counts *both*
 * clock edges. With ARR=1 the counter therefore wraps on every second edge,
 * and because the SSI clock idles HIGH the edges pair up as (F1,R1), (F2,R2)...
 * so the wrap always lands on a rising edge. A frozen channel-2 compare with
 * CCR2=0 matches at that wrap and raises one DMA request, which writes the next
 * bit to GPIOB->BSRR -- one bit shifted out per rising edge, which is what
 * 5.4.1 note 2 requires. Nothing writes DATA between F1 and R1, so the first
 * bit period stays untouched.
 *
 * Three things were measured on hardware and do not work, recorded so they are
 * not retried:
 *   - ARR=0 for one update per edge blocks the counter outright. RM0390: "The
 *     counter is blocked while the auto-reload value is null."
 *   - The trigger event with TDE fires exactly once; nothing clears TIF.
 *   - DMA1 cannot reach GPIO. It is on AHB1, and driving GPIOB->BSRR from DMA1
 *     raises a transfer error on the first word and the hardware disables the
 *     stream (TEIF set, one transfer, stream off). Only DMA2 can -- which is
 *     why the TIM1 test master always worked. Hence TIM8, the free APB2 timer.
 *
 * TIM8_CH1 = PC6 on AF3 [DS Table 11 p.60], morpho CN10 pin 4.
 * TIM8_CH2 -> DMA2 Stream 3, Channel 7 [RM0390 Rev 9 Table 29]; ADC1 keeps
 * stream 0.
 */
#define SSI_CLKIN_GPIO           GPIOC
#define SSI_CLKIN_PIN            GPIO_PIN_6
#define SSI_CLKIN_AF             GPIO_AF3_TIM8
#define SSI_CLKIN_TIM            TIM8

#define SSI_DATA_DMA_STREAM      DMA2_Stream3
#define SSI_DATA_DMA_CHANNEL     7u
#define SSI_DATA_DMA_IRQn        DMA2_Stream3_IRQn

/* Error Flag, driven by hardware at the last falling edge.
 *
 * 5.4.1 note 3 hands the data line to the Error Flag after the last rising
 * edge, but D0 must still be readable by a controller sampling on the falling
 * edge that follows it. The last falling edge is therefore the only instant
 * that satisfies both, and it is an edge the counter already sees: channel 4
 * compares against CCR4=1, which matches when the both-edge counter is at 1 --
 * every falling edge.
 *
 * There are n+1 falling edges in a Read Cycle (F1 starts it, F(n+1) ends it),
 * so the buffer is n+1 words: n zeros and then the flag. Writing 0 to BSRR
 * sets and resets nothing, so the first n are deliberate no-ops and only the
 * last one moves the line. No interrupt is involved, which is the whole point.
 *
 * TIM8_CH4 -> DMA2 Stream 7, Channel 7 [RM0390 Rev 9 Table 29]. Stream 4 also
 * carries a TIM8 request but belongs to the test master's IDR sampler.
 */
#define SSI_EF_DMA_STREAM        DMA2_Stream7
#define SSI_EF_DMA_CHANNEL       7u
#define SSI_DATA_BSRR            (&GPIOB->BSRR)

#define SSI_MASTER_SCK_GPIO      GPIOB
#define SSI_MASTER_SCK_PIN       GPIO_PIN_10
#define SSI_MASTER_MISO_GPIO     GPIOB
#define SSI_MASTER_MISO_PIN      GPIO_PIN_14

#define ANGLE_ADC                ADC1
#define ANGLE_ADC_GPIO           GPIOA
#define ANGLE_ADC_PIN            GPIO_PIN_0
#define ANGLE_ADC_CHANNEL        ADC_CHANNEL_0

#define TRACE_UART               USART2
#define TRACE_UART_GPIO          GPIOA
#define TRACE_UART_TX_PIN        GPIO_PIN_2
#define TRACE_UART_RX_PIN        GPIO_PIN_3
#define TRACE_UART_AF            GPIO_AF7_USART2
#define TRACE_UART_BAUD          921600u

#define LED_GPIO                 GPIOA
#define LED_PIN                  GPIO_PIN_5
#define BUTTON_GPIO              GPIOC
#define BUTTON_PIN               GPIO_PIN_13

/* Clock tree produced by board_init(); see board.c for the derivation.
 * APB1 = 45 MHz and APB2 = 90 MHz are the documented maxima [DS 3.6]. */
#define SYSCLK_HZ                180000000u
#define PCLK1_HZ                 45000000u
#define PCLK2_HZ                 90000000u
#define APB1_TIMCLK_HZ           90000000u   /* APB1 prescaler != 1 -> x2 */
#define APB2_TIMCLK_HZ           180000000u  /* APB2 prescaler != 1 -> x2 */

/* Which oscillator the PLL actually locked to. This determines whether the
 * Time Stamp field can meet its "better than 1%" accuracy: measured +0.02% on
 * HSE and +1.40% on HSI. */
typedef enum { CLK_SRC_HSE = 0, CLK_SRC_HSI } clock_source_t;

clock_source_t board_clock_source(void);
const char    *board_clock_source_name(void);
bool           board_timestamp_in_spec(void);

void board_init(void);
void board_led_set(bool on);
void board_led_toggle(void);
bool board_button_pressed(void);

#endif /* SIMENC_BOARD_H */
