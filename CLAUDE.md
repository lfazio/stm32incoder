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
- Pin I/O structure matters for a 5 V MAX490 output: **PA5 is `TTa`** (3.3 V
  only) and carries LD2. The SSI clock lands on PC6, which is `FT`.

## Build, flash, talk to it

```sh
cmake -B build -G Ninja -DCMAKE_TOOLCHAIN_FILE=cmake/arm-none-eabi.cmake \
      -DCMAKE_BUILD_TYPE=Release
cmake --build build
cmake --build build --target flash        # openocd -f board/st_nucleo_f4.cfg
stty -F /dev/ttyACM0 921600 raw -echo && cat /dev/ttyACM0
```

Send console commands with `printf 'stat\r' > /dev/ttyACM0` while a reader
holds the port open; `help` lists them. `-DSIMENC_CLOCK_SOURCE` is AUTO by
default (HSE, falling back to HSI).

**There is no SPI anywhere.** The incoming SSI clock drives TIM8 through its
channel-1 edge detector (TI1F_ED, both edges, ARR=1) on **PC6** (TIM8_CH1 AF3,
morpho CN10-4), and two compare channels raise DMA requests:

- **CC2, every rising edge** -> DMA2 S3 writes the next word to `GPIOB->BSRR`.
  The buffer is **n+1 words**: D(n-1)..D0, then the Error Flag.
- **CC4, every falling edge** -> DMA2 S7 writes UG into `TIM6->EGR`, restarting
  the Tmu timer. TIM6 overflowing therefore means "Tmu since the *last* falling
  edge", whatever the number of clocks, and its handler ends the Read Cycle.

**A Read Cycle is n+1 clock periods, not n.** 5.4.1 gives Trc = n x T + 0.5 x T
from the first falling edge to the last rising edge, which only fits n+1
falling and n+1 rising edges: F1 starts the cycle and carries no data, R1..R(n)
put out D(n-1)..D0, the master samples them on F2..F(n+1), and R(n+1) -- "the
last rising edge" of note 3 -- puts out the Error Flag. This project assumed n
for a long time because its own test master sent n, and several conclusions
below were wrong for that reason alone.

Consequences: DATA changes on the rising edge (+56 ns, DMA latency), nothing
touches the line between F1 and R1, there is no pin handover and no 250 ns
deadline, and n need not be a multiple of 8. PB4 is a GPIO output for the whole
cycle -- only the *writer* changes. **PC6 is the only clock input**; without that
wire the link is dead and `stat` says `clock in PC6: NEVER REACHED n`.

PB3 and PB5 (the old SPI1 SCK/MOSI) and PD2 (the old TIM3_ETR counter) are no
longer used at all. SWO trace on PB3 is free again.

The test master is TIM1 compare events + DMA. It sends **n+1 clocks** and
samples **late in the high phase, just before the next falling edge** -- where
an SSI controller reads -- so its samples are D(n-1)..D0 and then the Error
Flag, which `read` prints as `ef` and `burst` checks against PV. It
used to sample one tick after the rising edge, which was right when the slave
drove on the falling edge; after the slave moved, every frame read back shifted
one bit and `fixed 0x5A5A5` came out as 447186 = `(pd >> 1) | (ZPD << 18)`.

2 MHz is not a corner case: with no handover there is no deadline, and it
measures like any other rate (197/197, 0 glitches).

Three hardware facts, each of which cost a probe cycle:
- `ARR=0` does not give an update per edge, it blocks the counter (RM0390).
- A trigger event with TDE fires exactly once; nothing clears TIF.
- **DMA1 cannot reach GPIO** -- it is on AHB1. Driving `GPIOB->BSRR` from DMA1
  raises a transfer error and the hardware disables the stream. Only DMA2 can,
  which is why the TIM1 test master always worked. Hence TIM8, on APB2.

The emulator serves Read Cycles from reset; no command starts it. At power-on
the position source is `adc`, the zero point is factory (ZPD=1), PV=1, and the
**test master clock is 500 kHz** — raise it with `clk 2000000` to exercise the
timing corner. Commands that only exist for loopback bring-up are `wire`, `clk`, `read`, `burst`; the rest drive the emulated device.

The test master clock is generated by TIM1 compare events driving DMA to
`GPIOB->BSRR`, so any rate of the form 180 MHz / N is reachable across the whole
100 kHz..2 MHz window (N = 90..1800).

`trace_printf` truncates at 192 bytes (its stack buffer). Print long listings as
several calls; a single long `cmd_help` was silently cut mid-line.

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
ssi/ssi_variant.c   SSI payload codecs — pure logic, no hardware, host-testable
      ↑
ssi/ssi_slave.c     generic SSI slave transport (TIM8 + DMA + Tmu gap)
```

`ssi_slave` is payload-agnostic on purpose: it moves *n* bits, and the payload
variants live as sibling codecs in `ssi/ssi_variant.c` -- SSI1, SSI2, SSI4
(default), SSI6 and SSI9, selected at runtime with `ssi <n>`. SSI7 and SSI8 are
absent because they are not byte aligned. Adding a variant means adding a
pack/unpack there and a row in the descriptor table, nothing else.

SSI6's CRC-8 (polynomial 0x97) cannot use the CRC peripheral: RM0390 4.1/4.2
give it "a fixed generator polynomial", CRC-32 0x4C11DB7, with no CRC_POL or
POLYSIZE on this family. It stays in software, in `stage_frame()`, which runs
in the Tmu handler before DATA returns HIGH: off the bit path, but inside the
self-measured `tmu_arm`. Do not leak SSI4
field knowledge into it, and do not leak timer/DMA knowledge upward.
`ssi/ssi_master.c` is a bring-up instrument, not part of the emulated device.

## Real-time constraints in the SSI path

- Bit shifting is done entirely by the clock and DMA — never add per-bit
  software work. There is no per-frame deadline left, because there is no pin
  handover.
- The Read Cycle ends when TIM6 overflows: Tmu with no falling edge. Nothing
  else ends it -- not the DATA DMA's transfer-complete, which now only measures
  the clock period for `stat`, and not a polled watchdog, which is gone. A
  master that stops early is recovered by the same overflow.
- The Tmu handler stages the next frame and re-arms **before** it drives DATA
  HIGH, so a master that starts the moment it sees HIGH finds the slave ready.
  That time (3.9 us SSI1, 4.8 us SSI4, 5.0 us SSI6, read off TIM6 itself) is
  measured every cycle and taken out of the next one's Tmu. `stat` shows it as
  `tmu_arm`. Anything added to `stage_frame()` or the provider lands there.
- The update tick also rebuilds the armed frame after every latch
  (`incoder_update()` -> `ssi_slave_refresh()`), but only while TIM8 CC4IF is
  clear, i.e. before F1. It builds into a shadow first, then tests and copies
  under PRIMASK (1.3 us for 33 words). The copy is safe against an F1 right
  after the test because it outruns the DMA. Without the refresh a frame was as
  old as the master's idle time: the first read after reset said PV=0 and
  `burst` needed warm-up cycles. Do not use TIM8 CNT as the idle test -- it
  returns to 0 on every rising edge. The provider now runs from both handlers,
  so keep it reentrant.
- The latched sample is double-buffered (`s_sample[2]` + `s_cur` in
  incoder.c). The Tmu handler preempts the tick, so separate variables let it
  pack a new TS with the previous update's PD/PV. Never go back to writing the
  fields one by one.
- NVIC priorities are ordered deliberately: the DATA DMA (0,1) and the Tmu timer
  (0,2) outrank the update tick (2,0), trace DMA (3,0) and ADC DMA (3,1).
  Keep tracing off the SSI path — `trace_printf` only fills a ring buffer.
- Do not add an RTOS to this path. The timing budget is sub-microsecond and is
  met today by DMA plus two short ISRs.

## Two GPIO traps already hit on this board

Both were found by reading registers off the running target, not by reasoning,
and both still apply to any pin this project hands between owners:

- `HAL_GPIO_Init()` writes `GPIOx->AFR[]` **only** when `Mode` is an AF mode.
  Initialising a pin as `GPIO_MODE_OUTPUT_PP` leaves its AFR nibble at reset 0,
  and AF0 on PB4 is NJTRST. If a pin is ever given back to a peripheral by
  flipping MODER, program the AF selector once in an AF mode first. (The data
  path no longer does this — PB4 is a GPIO output throughout — but the trap is
  general.)
- **Never write a shared register wholesale from an ISR.** A handler stored a
  precomputed `GPIOB->MODER` word for speed, which republished *every* pin's
  mode as it was when the frame was armed. That silently reverted the test
  harness's clock pin from GPIO back to alternate function, killing the clock
  mid-burst. Read-modify-write only the bits you own.

A third trap, cheap to re-check: enabling an IRQ in the NVIC without defining
its handler silently binds the startup file's `Default_Handler`, which is an
infinite loop. The board then hangs with no output the moment that interrupt
first fires — here, as soon as a clock edge finally reached PB3. After adding
any `HAL_NVIC_EnableIRQ`, confirm the handler is actually linked:

```sh
for h in DMA2_Stream3_IRQHandler TIM6_DAC_IRQHandler \
         DMA2_Stream0_IRQHandler DMA1_Stream6_IRQHandler TIM2_IRQHandler; do
  arm-none-eabi-nm build/simenc.elf | grep -q " T $h" || echo "MISSING: $h"
done
```

To diagnose a silent board, halt it and read `xPSR`: a non-zero IPSR means it is
stuck in an exception, and `IPSR - 16` is the IRQ number.

Useful register addresses for this board (GPIOB): MODER `0x40020400`,
IDR `0x40020410`, AFRL `0x40020420`, AFRH `0x40020424`; GPIOC IDR `0x40020810`;
TIM8 `0x40010400`; DMA2 stream 3 `0x40026458`, stream 7 `0x400264b8`, LISR
`0x40026400`, HISR `0x40026404`.

## Verification status

Verified on hardware: 180 MHz clock, DMA trace/console, ADC acquisition, the
10 kHz update tick, the timestamp counter wrapping across 0..2047, and the whole
100 kHz..2 MHz clock window swept exactly (180 MHz / N, each rate confirmed by
the slave's own independent period measurement).

**Verified with a logic analyser** (Saleae Logic Pro 16 via the `logic2` MCP,
125 MS/s) on the timer data path, 500 kHz, 198 cycles:

  32 clocks on every cycle
  DATA +56 ns after its rising edge, 3720/3720 on the correct side
  DATA moves between F1 and R1          0/198
  gaps opening with a spurious HIGH     0/197
  payload sampled at the falling edge   198/198
  Tmu 20.43 us

2 MHz is the same picture: +56 ns, 3740/3740, 197/197, Tmu 19.66 us.

Done at RS-422 through two MAX490 modules: the whole sweep with `resyncs=0` and
all five variants clean at 500 kHz.

Time Stamp accuracy was measured by regressing the decoded TS field against
capture time over a 0.4 s span: HSI is **+1.401%** (fails the "better than 1%"
specification) and HSE is **+0.021%**. `SIMENC_CLOCK_SOURCE` therefore defaults
to `AUTO` -- try HSE, fall back to HSI -- and the banner reports which locked,
flagging `[timestamp OUT OF SPEC]` on the fallback. Measure that over a long
span: 13 ms gave a spurious +3.16% for HSI because the Read Cycle period beats
against the 100 us update tick that latches TS.

Two rates that are easy to conflate, both fixed by the guide: the **position
latch is 10 kHz** (4.12 says "< 0.1 ms", and 5.5.1 pins it by calling ASI2's
10 kHz "the same rate as Internal Position Update Period"), while the **Time
Stamp counter ticks at 100 kHz**, 10 us resolution (5.4.2). 100 kHz is the
counter, not the latch -- do not "fix" the update rate to 100 kHz.

TS is set back to the **centroid of the ADC averaging window**:
`position_source_delay_us()` = 12/2 x 8.533 + 1.067 = 52 us, which is 5 ticks
for SSI4 and 52 for SSI9. The tick comes at the *end* of the window, and
stamping that dated every position ~5 ticks late. `fixed`/`ramp` have no window
and return 0. If the ADC timing or ADC_AVG_SAMPLES changes, the macros in
position_source.c recompute it.

PV for the ADC source is `s_adc_filled && !ADC_SR_OVR`. A conversion counter
compared against 12 wrapped every ~10.2 h (2^32/12 x 102.4 us) and dropped PV
for one window. An overrun freezes the DMA window on its last values, and
nothing caught it until `position_source_service()` (main loop) restarted the
ADC.

Tmu needs no clock-period correction any more: it is timed from the falling
edge itself, so the half-period term that used to range from 0.25 us at 2 MHz
to 5 us at 100 kHz is simply not there. Do not reintroduce a timer started from
the end of the message -- that is what tied Tmu to a guess about where the last
falling edge was, and it came out one whole period short for every master that
sends the n+1 clocks the guide asks for. T is still measured (half-transfer to
transfer-complete of the DATA DMA) but only for `stat`, and only on a Read
Cycle that reaches n+1 clocks.

Watch integer truncation in that arithmetic: `1000/(SYSCLK_HZ/1000000)` is 5,
not 5.56, which reported every period 10% short until it was caught against the
analyser. Scale before dividing.

**Re-verified on hardware with a logic analyser after the SPI removal** (a
different board; ST-LINK serial 066DFF555670655067161435, one adapter attached
so no `adapter serial` pin is needed). 197 cycles at each rate:

                              500 kHz          2 MHz
  clocks per cycle            32 on every one  32 on every one
  DATA after its rising edge  +56 ns, 3724/3724  +56 ns, 3734/3734
  DATA moves F1..R1           0                0
  payload                     197/197          197/197
  Tmu                         20.10..20.19 us  20.14..20.24 us

Loopback: `fixed 0x5A5A5` reads back 370085 at 100k/250k/500k/1M/1.5M/2M,
198/198 each, all five variants 98/98 at 500 kHz including ssi2 and ssi6 --
which is the useful signal, since they are the only variants with an integrity
check and a one-bit shift breaks parity and CRC where it merely relabels a
position field.

**The Error Flag is the (n+1)th bit, and that is all it ever was.** It rides
the DATA DMA as word n and lands on R(n+1), +56 ns like any bit. The history is
worth keeping because each wrong turn looked reasonable:

- Driven from the end-of-message interrupt it was 416 ns late and, at 500 kHz,
  destroyed D0 before the master sampled it.
- Moved to a gap timer at 0.75 T after "the last rising edge" it stopped
  destroying D0 but took the line ~3 us late.
- A falling-edge DMA with the flag at buffer index n "could not work" because
  a Read Cycle "has exactly n falling edges" -- true of this project's test
  master, false of the guide. Trc = n x T + 0.5 x T is n+1 clocks.

The check that catches a flag that is not being driven is still the same:
**compare the gap level between PV=1 and PV=0** (`err on`). A glitch count
cannot see it, because a stale last data bit is nearly constant for a `fixed`
payload.

## What an external master taught that the loopback could not

The first real controller on the link (an SPI master, 562.5 kHz, polling at
10 kHz) broke three things the loopback had passed, all for one reason: `read`
and `burst` block the main loop and send n clocks, so nothing that depends on
either was ever exercised.

- **Anything polled from the main loop runs inside a bit period.** The loop
  takes a few hundred ns. A watchdog that compared NDTR on consecutive passes
  re-armed after nearly every bit, and the frame restarted from D(n-1) every
  three to five bits. Signature on a capture: DATA writes on *falling* edges
  (re-armed while the clock was LOW) and a TS field that never advances.
- **Clearing EN on a live memory-to-peripheral DMA stream sets TCIF** (RM0390
  9.3.14/9.3.15). With TCIE on, every re-arm looked like an end of message.
  Mask TCIE/HTIE in the same write as EN -- `data_dma_halt()`.
- **A master that sends n clocks reads every frame one bit late.** Its first
  sample, at F1 or right at R1, is the idle HIGH; what it decodes is
  `(1 << (n-1)) | (frame >> 1)`, so SSI4 at position 0 reads back PD = 0x40000
  (ZPD in the top of the field) and TS counts by 5. That is the master's bug,
  and a real IncOder would show it too -- do not "fix" it in the emulator.
  `stat` counts such reads as `short` and says so.

An SPI master on an 8-bit frame cannot send 33 clocks; it sends 40, and that is
fine: sample on the edge the *slave* sees falling, discard the first bit, take
the next n, and the one after is the Error Flag. Clocks past n+1 only restart
Tmu.

Verified on hardware against that master (`stat`, no analyser yet):

- SSI4, master sending 32 = n clocks: every read `short`, at exactly the
  10 kHz polling rate, `aborted` not moving. A Tmu restart that did not work
  would leave TIM6 parked and count ~153/s instead.
- SSI1, same master, 32 clocks >= n+1 = 25: every read `full`, `measured_T`
  1776 ns against a 1778 ns clock. That exercises the Error Flag word and seven
  clocks past it.

**Not yet captured on an analyser:** the Error Flag landing on R(n+1), and Tmu
measured from the last falling edge. The Tmu and Error Flag figures above this
section were taken on the old two-stage gap with an n-clock test master and say
nothing about the current path. The loopback jumpers were off for this work, so
`read`/`burst` with the n+1 test master are untested too.

**Not yet run on hardware at all:** the idle-frame refresh, the double-buffered
sample, the TS centroid offset and the ADC overrun recovery. Their checks are:
- `read 1` 5 s after reset must say `pv=1`.
- `fixed X` then `read 1` must give X on Read Cycle 0.
- With `src ramp`, every pair of consecutive frames on a capture must satisfy
  `dPD == step x dTS/10`.
- Clearing EN on DMA2 S0 (`0x40026410`) must show `pv=0`, then `adc_ovr=1`,
  then `pv=1` again.
