# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

Firmware that emulates a Zettlex/Celera Motion **IncOder** inductive angle
encoder in its **SSI4** protocol variant, running on a **NUCLEO-F446RE**.
See `README.md` for the pin map, loopback wiring and architecture diagram.

## Ground rule: never invent a hardware value

Every pin, alternate function, DMA stream/channel, clock and protocol timing
value must come from a document in `docs/` and be cited in a comment next to
the code that uses it. The PDFs are the authority:

- `docs/sensors/IncOder_Product_Guide_MIDI_ULTRA_Rev_4.11.8.pdf` — §5.4.1 SSI
  protocol/timing, §5.4.2 SSI4 frame layout, §4.12 update rate, §5.2 zero point.
- `docs/stm32/stm32f446mc.pdf` (DS10693 Rev 11) — **Table 11** alternate
  function mapping (AF numbers), **Table 10** pin descriptions incl. the I/O
  structure column (`FT` = 5 V tolerant, `TTa` = 3.3 V, ADC-connected).
- `docs/stm32/rm0390-*.pdf` (RM0390 Rev 9) — **Tables 28/29** DMA request
  mapping, bus maximum frequencies.
- `docs/stm32/um1724-*.pdf` (UM1724 Rev 17) — **Table 19** Arduino connectors,
  **Table 29** ST morpho connector, LD2/VCP/HSE solder-bridge configuration.

`pdftotext -layout` mangles the wide AF and pin tables — read those pages as
images with the Read tool instead of trusting the text extraction.

Two traps already hit, both caught by checking the source documents:

- ST's own `UART_Printf` example for this board configures **APB2 = HCLK/1**
  (180 MHz), twice the 90 MHz maximum in DS10693 §3.6. This project uses `/2`.
- SPI1's *default* pins are wrong here: **PA5 is `TTa`** (3.3 V only) and
  carries LD2, so SPI1 is remapped to PB3/PB4/PB5 for a 5 V-safe clock input.

## Build, flash, talk to it

```sh
cmake -B build -G Ninja -DCMAKE_TOOLCHAIN_FILE=cmake/arm-none-eabi.cmake \
      -DCMAKE_BUILD_TYPE=Release
cmake --build build
cmake --build build --target flash        # openocd -f board/st_nucleo_f4.cfg
stty -F /dev/ttyACM0 921600 raw -echo && cat /dev/ttyACM0
```

Send console commands with `printf 'stat\r' > /dev/ttyACM0` while a reader
holds the port open. `-DSIMENC_CLOCK_SOURCE=HSE` selects the ST-LINK MCO.

## Toolchain constraint: picolibc, not newlib

Debian's `gcc-arm-none-eabi` ships **picolibc**; there is no `nano.specs`.
`cmake/arm-none-eabi.cmake` detects which C library exists and links
`--specs=picolibc.specs --crt0=none` (ST's startup file already provides
`Reset_Handler`, so picolibc's crt0 must not be linked).

**Consequence — do not use libc functions with hidden thread-local state.**
Because `--crt0=none` skips `_init_tls()`, anything living in `.tbss` reads and
writes a garbage address. `strtok` (whose `_strtok_last` is in `.tbss`) and
`errno`-setting functions such as `strtoul`/`strtol` are therefore banned;
`src/board/console.c` parses by hand instead. After touching console or string
code, re-check that no TLS crept back in:

```sh
arm-none-eabi-nm build/simenc.elf | grep -iE 'tbss|errno|__aeabi_read_tp'
```

That must print nothing.

## Layering (keep it)

```
encoder/incoder.c   sensor behaviour: position, zero point, timestamp, PV/ZPD
      ↑
ssi/ssi4.c          SSI4 frame codec — pure logic, no hardware, host-testable
      ↑
ssi/ssi_slave.c     generic SSI slave transport (SPI1 + DMA + Tmu gap)
```

`ssi_slave` is payload-agnostic on purpose: it moves *n* bits so the other SSI
variants can be added as sibling codecs next to `ssi4.c`. Do not leak SSI4
field knowledge into it, and do not leak SPI/DMA knowledge upward.
`ssi/ssi_master.c` is a bring-up instrument, not part of the emulated device.

## Real-time constraints in the SSI path

- The SSI clock may run to 2 MHz, so DATA must be valid within 250 ns of a
  falling edge. Bit shifting is therefore left entirely to the SPI hardware —
  never add per-bit software work.
- End of message is detected by the **receive** DMA's transfer-complete event
  (it counts clocks); the transmit DMA cannot do this because it completes
  early, when the last byte is handed to the shift register.
- NVIC priorities are ordered deliberately: SSI RX DMA (0,0) and the Tmu timer
  (0,1) outrank the update tick (2,0), trace DMA (3,0) and ADC DMA (3,1).
  Keep tracing off the SSI path — `trace_printf` only fills a ring buffer.
- Do not add an RTOS to this path. The timing budget is sub-microsecond and is
  met today by DMA plus two short ISRs.

## Two GPIO traps already hit on this board

Both were found by reading registers off the running target, not by reasoning:

- `HAL_GPIO_Init()` writes `GPIOx->AFR[]` **only** when `Mode` is an AF mode.
  PB4 was initialised as `GPIO_MODE_OUTPUT_PP`, so its AFR nibble stayed at
  reset 0 — and AF0 on PB4 is NJTRST, not SPI1_MISO. The later MODER flip to AF
  handed DATA to the JTAG block. Always program the AF selector once in AF mode
  first, then change only MODER.
- **The SPI drives MISO LOW while it holds the pin with no clock running**
  (measured: `IDR` bit 4 = 0 when armed). SSI requires DATA to idle HIGH, so the
  pin stays GPIO-high until EXTI3 hands it over on the first falling edge.

- **Never write a shared register wholesale from an ISR.** `EXTI3_IRQHandler`
  stored a precomputed `GPIOB->MODER` word for speed, which republished *every*
  pin's mode as it was when the frame was armed. That silently reverted the test
  harness's clock pin from GPIO back to alternate function on the first edge,
  killing the clock mid-burst. Read-modify-write only the bits you own; the few
  extra cycles still fit the 250 ns budget.

A third trap, cheap to re-check: enabling an IRQ in the NVIC without defining
its handler silently binds the startup file's `Default_Handler`, which is an
infinite loop. The board then hangs with no output the moment that interrupt
first fires — here, as soon as a clock edge finally reached PB3. After adding
any `HAL_NVIC_EnableIRQ`, confirm the handler is actually linked:

```sh
for h in EXTI3_IRQHandler DMA2_Stream2_IRQHandler TIM6_DAC_IRQHandler \
         DMA2_Stream0_IRQHandler DMA1_Stream6_IRQHandler TIM2_IRQHandler; do
  arm-none-eabi-nm build/simenc.elf | grep -q " T $h" || echo "MISSING: $h"
done
```

To diagnose a silent board, halt it and read `xPSR`: a non-zero IPSR means it is
stuck in an exception, and `IPSR - 16` is the IRQ number.

Useful register addresses for this board (GPIOB): MODER `0x40020400`,
IDR `0x40020410`, AFRL `0x40020420`, AFRH `0x40020424`; SPI1 `0x40013000`,
SPI2 `0x40003800`; DMA2 stream 2 `0x40026440`, stream 3 `0x40026458`.

## Verification status

Verified on hardware: 180 MHz clock, DMA trace/console, ADC acquisition, the
10 kHz update tick, the timestamp counter wrapping across 0..2047, and the
GPIO/SPI/DMA register configuration of both SSI ends.

**The SSI4 frame is verified on the wire** by TTL loopback: `fixed` values of
0x12345, 0x2AAAA, 0x5A5A5 and 0 all decoded back with the correct `pd`, `pv=1`
and `zpd=1`, at 1.40625 MHz, 351.6 kHz and 175.8 kHz, with `resyncs=0`. The
timestamp advanced exactly 20 ticks (200 µs) between reads spaced 200 µs apart.

**The 2 MHz corner is verified**: `read2m` clocks at exactly 2.000 MHz from TIM1
compare events driving DMA writes to `GPIOB->BSRR`. With `err on` forcing PV=0 —
which is what makes D31 a sensitive bit, since PV is normally 1 and the line also
idles HIGH — 25 of 25 frames decoded correctly with `resyncs=0`. That proves the
EXTI3 handover meets its 250 ns deadline.

Still to do: the same run through two MAX490 modules at RS-422.

Run `wire` before trusting any `read`; `FFFFFFFF` with `frames=0` means a jumper
is missing. Note the first `read` immediately after `wire` can return a bad frame
because the test borrows the DATA pin and re-arms the slave — discard it.
