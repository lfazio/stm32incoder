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
 * SSI slave (the emulated IncOder). SPI1 is remapped onto port B so that the
 * clock input does not land on PA5, which carries the LD2 LED [UM 7.6] and is
 * TTa (3.3 V only) [DS Table 10 p.41] -- unsuitable for a 5 V MAX490 output.
 *
 *   PB3  SPI1_SCK  AF5  [DS Table 11 p.59]  CN9-4  (D3)   FT, 5 V tolerant
 *        <- SSI CLOCK in   (from MAX490 receiver output RO)
 *   PB4  SPI1_MISO AF5  [DS Table 11 p.59]  CN9-6  (D5)
 *        -> SSI DATA out   (to MAX490 driver input DI)
 *   PB5  SPI1_MOSI AF5  [DS Table 11 p.59]  CN9-5  (D4)
 *        unused; configured with a pull-down purely so the slave receive shift
 *        register clocks deterministically -- that is what counts SSI clocks.
 *
 * SSI master (loopback test harness only, SPI2):
 *   PB10 SPI2_SCK  AF5  [DS Table 11 p.59]  CN9-7  (D6)  / CN10-25
 *   PB14 SPI2_MISO AF5  [DS Table 11 p.59]  CN10-28
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

#define SSI_SLAVE_SPI            SPI1
#define SSI_SLAVE_GPIO           GPIOB
#define SSI_SLAVE_SCK_PIN        GPIO_PIN_3
#define SSI_SLAVE_DATA_PIN       GPIO_PIN_4
#define SSI_SLAVE_MOSI_PIN       GPIO_PIN_5
#define SSI_SLAVE_AF             GPIO_AF5_SPI1
/* Bit position of PB4 inside GPIOB->MODER (2 bits per pin). */
#define SSI_SLAVE_DATA_PIN_NUM   4u

/* End-of-message clock counter -- only built when SIMENC_CLOCK_COUNTER is on.
 *
 * TIM3 in external clock mode 2 counts SSI clock edges on its ETR input and
 * raises an update event on the nth, which is end of message for any n,
 * byte aligned or not, and independent of what the SPI is doing with the bits.
 *
 * ETR has to be a second pad: PB3 already carries SPI1_SCK on AF5, and a pad
 * holds one alternate function at a time (its AF1 is TIM2_CH2, no use here).
 * PD2 is TIM3_ETR on AF2 [DS Table 11] and comes out on morpho CN7 pin 4
 * [UM Table 29], so the clock net needs a second wire to it.
 */
#define SSI_ETR_TIM              TIM3
#define SSI_ETR_TIM_IRQn         TIM3_IRQn
#define SSI_ETR_GPIO             GPIOD
#define SSI_ETR_PIN              GPIO_PIN_2
#define SSI_ETR_AF               GPIO_AF2_TIM3

/* DATA shifted out by DMA, clocked by the SSI clock itself.
 *
 * TIM3 counts clock rising edges on ETR with ARR=0, so it raises an update
 * event on every one, and TIM3_UP drives a DMA that writes the next bit to
 * GPIOB->BSRR. Each rising edge therefore shifts one bit onto the bus, which
 * is what 5.4.1 note 2 describes, and the bit is then valid across the falling
 * edge where the controller samples it.
 *
 * Getting one DMA request per rising edge takes counting at twice the bit
 * rate. TIM3 is clocked by TI1F_ED, the channel-1 edge detector, which counts
 * *both* clock edges; with ARR=1 the update event then falls on every second
 * edge. The clock idles HIGH, so the edges pair up as (F1,R1), (F2,R2)... and
 * every update lands exactly on a rising edge -- one bit shifted out per rising
 * edge, which is what 5.4.1 note 2 requires.
 *
 * Nothing writes DATA between F1 and R1, so the first half period stays
 * untouched, which the SPI handover could never manage.
 *
 * Two other routes were tried on hardware and do not work: ARR=0 for an update
 * per edge blocks the counter outright (RM0390: "The counter is blocked while
 * the auto-reload value is null"), and the trigger event with TDE fires exactly
 * once because nothing clears TIF. The update event is the one pattern that
 * repeats -- the same one the test master's clock generator uses.
 *
 * The request comes from a compare event, not the update event: measured on
 * hardware, the update event with UDE moves exactly one word and then stops,
 * while compare events repeat -- which is what the test master's own clock
 * generator has always relied on. Channel 1 is occupied being the clock input,
 * so channel 2 does the compare, with CCR2=0 so it matches when the counter
 * wraps to 0, which is the rising edge.
 *
 * It must be **TIM8 and DMA2**, not TIM3 and DMA1. GPIO lives on AHB1 and DMA1
 * cannot reach it: driving GPIOB->BSRR from DMA1 raises a transfer error on the
 * first word and the hardware disables the stream (measured: TEIF set, one
 * transfer, stream off). DMA2 is the one that can, which is why the test master
 * -- TIM1 on APB2, so DMA2 -- has always worked. TIM8 is the free APB2 timer.
 *
 * TIM8_CH2 -> DMA2 Stream 3, Channel 7 [RM0390 Rev 9 Table 29]. In this build
 * there is no SPI1, so streams 2 and 3 are free; ADC1 keeps stream 0.
 *
 * TI1F_ED is a channel-1 input, so the clock reaches TIM8_CH1 = PC6 on **AF3**
 * [DS Table 11], morpho CN10 pin 4. The same pad also carries TIM3_CH1 on AF2,
 * so this is a change of alternate function only -- the wire does not move.
 */
#define SSI_CLKIN_GPIO           GPIOC
#define SSI_CLKIN_PIN            GPIO_PIN_6
#define SSI_CLKIN_AF             GPIO_AF3_TIM8
#define SSI_CLKIN_TIM            TIM8

#define SSI_DATA_DMA_STREAM      DMA2_Stream3
#define SSI_DATA_DMA_CHANNEL     7u
#define SSI_DATA_DMA_IRQn        DMA2_Stream3_IRQn
#define SSI_DATA_BSRR            (&GPIOB->BSRR)

#define SSI_MASTER_SPI           SPI2
#define SSI_MASTER_SCK_GPIO      GPIOB
#define SSI_MASTER_SCK_PIN       GPIO_PIN_10
#define SSI_MASTER_MISO_GPIO     GPIOB
#define SSI_MASTER_MISO_PIN      GPIO_PIN_14
#define SSI_MASTER_AF            GPIO_AF5_SPI2

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
