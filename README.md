# simenc — Zettlex IncOder emulator (SSI4) on NUCLEO-F446RE

Emulates a Zettlex/Celera Motion IncOder inductive angle encoder speaking the
**SSI4** protocol variant, on a NUCLEO-F446RE. The shaft angle comes from one
analog input; the SSI link is driven by hardware (SPI shift register + DMA +
timers) so no interrupt runs per bit.

Every register, pin and timing value in this project is taken from a document
in `docs/`, cited in the source. Nothing is guessed.

| Document | Used for |
|---|---|
| `docs/sensors/IncOder_Product_Guide_MIDI_ULTRA_Rev_4.11.8.pdf` | SSI protocol (5.4.1), SSI4 frame (5.4.2), update rate (4.12), zero point (5.2) |
| `docs/stm32/stm32f446mc.pdf` (DS10693 Rev 11) | pin alternate functions (Table 11), 5 V tolerance (Table 10) |
| `docs/stm32/rm0390-…​.pdf` (RM0390 Rev 9) | DMA request mapping (Tables 28/29), bus maximum frequencies |
| `docs/stm32/um1724-…​.pdf` (UM1724 Rev 17) | board connectors (Table 19/29), LD2, VCP, HSE options |

## Build and flash

```sh
git submodule update --init vendor/STM32CubeF4
git -C vendor/STM32CubeF4 submodule update --init --depth 1 \
    Drivers/CMSIS/Device/ST/STM32F4xx Drivers/STM32F4xx_HAL_Driver

cmake -B build -G Ninja -DCMAKE_TOOLCHAIN_FILE=cmake/arm-none-eabi.cmake \
      -DCMAKE_BUILD_TYPE=Release
cmake --build build
cmake --build build --target flash          # OpenOCD, board/st_nucleo_f4.cfg
```

Console/trace on the ST-LINK virtual COM port, **921600 8N1**:

```sh
stty -F /dev/ttyACM0 921600 raw -echo && cat /dev/ttyACM0
```

`SIMENC_CLOCK_SOURCE` defaults to **AUTO**: lock the 8 MHz ST-LINK MCO (HSE) if
the board's solder bridges provide it (UM1724 7.9.1, the factory setting), and
fall back to the internal 16 MHz RC otherwise. HSE is preferred because HSI
misses the Time Stamp accuracy specification — see
[Measured against the specification](#measured-against-the-specification). The
banner reports which oscillator locked. Force one with
`-DSIMENC_CLOCK_SOURCE=HSE` (hangs if absent) or `=HSI`.

## Pin map

All alternate functions verified in DS10693 Table 11; 5 V tolerance ("FT") in
Table 10; connector positions in UM1724 Tables 19 and 29.

| Signal | Pin | AF | Arduino | Morpho | I/O structure |
|---|---|---|---|---|---|
| SSI **CLOCK in** (slave) | PB3 | AF5 SPI1_SCK | CN9-4 (D3) | CN10-31 | FT (5 V tolerant) |
| SSI **DATA out** (slave) | PB4 | AF5 SPI1_MISO | CN9-6 (D5) | CN10-27 | — |
| SPI1_MOSI (unused) | PB5 | AF5 | CN9-5 (D4) | CN10-29 | leave open |
| Master CLOCK out (test) | PB10 | AF5 SPI2_SCK | CN9-7 (D6) | CN10-25 | FT |
| Master DATA in (test) | PB14 | AF5 SPI2_MISO | — | CN10-28 | FT |
| Angle analog in | PA0 | ADC1_IN0 | CN8-1 (A0) | CN7-28 | **0–3.3 V only** |
| Trace/console | PA2/PA3 | AF7 USART2 | — | — | to ST-LINK VCP |
| Status LED (LD2) | PA5 | GPIO | CN5-6 (D13) | — | — |

SPI1 is deliberately **not** on its default PA5/PA6/PA7: PA5 carries the LD2 LED
and is `TTa` (3.3 V only), which would both load the clock line and be unsafe
against a 5 V MAX490 output.

> PB3 is JTDO/TRACESWO. Using it as SPI1_SCK costs SWO trace; SWD debugging
> (which is what the on-board ST-LINK uses) is unaffected.

## Loopback testing

### Stage 1 — TTL loopback on the board (2 jumper wires)

Both wires land on the ST morpho connector **CN10**, and the DATA wire is
simply between two facing pins:

| Wire | From | To |
|---|---|---|
| CLOCK | **CN10-25** (PB10, master out) | **CN10-31** (PB3, slave in) |
| DATA | **CN10-27** (PB4, slave out) | **CN10-28** (PB14, master in) |

Equivalently, the clock wire can use the Arduino header: **CN9-7 (D6) → CN9-4 (D3)**.

The DATA pair **CN10-27 / CN10-28 are directly opposite each other** across the
two rows of CN10, so a plain 2-pin jumper cap works there — no wire needed.
The CLOCK pair is easiest on the Arduino header, where both ends are
silkscreened: **D6 → D3**.

Check the wiring before anything else — the firmware can test it itself:

```
wire
```

It drives PB10 and PB4 as GPIO and verifies that PB3 and PB14 follow both
levels, printing `OK` or `OPEN` per wire. `OPEN` means that pin pair is not
connected, whatever the wire looks like.

Then:

```
src fixed
fixed 0x12345      # bypasses the ADC entirely
read 4             # runs 4 Read Cycles and decodes each frame
stat
```

`read` should report `pd=74565` (0x12345) with `pv=1`, `zpd=1`, and a `ts` that
advances between cycles. With no jumpers fitted you get `raw=FFFFFFFF` instead —
that is the master's idle pull-up, and `frames=0` in `stat` confirms it.

### Test clock rates

`clk <hz>` selects the fastest SPI prescaler at or below the request, and never
leaves the 100 kHz…2 MHz SSI window. SPI2 is clocked from PCLK1 = 45 MHz and its
prescalers are powers of two, so only four rates are reachable:

| Divider | Rate | Note |
|---|---|---|
| /16 | 2.8125 MHz | **rejected** — 40 % above the 2 MHz SSI maximum |
| /32 | 1.40625 MHz | fastest legal rate; the default |
| /64 | 703.125 kHz | |
| /128 | 351.5625 kHz | |
| /256 | 175.78125 kHz | slowest reachable (spec minimum is 100 kHz) |

The SPI baud generator cannot reach 2 MHz, so `clk 2000000` gives 1.40625 MHz.

### `read2m` — the 2 MHz corner

`read2m [n]` runs Read Cycles at **exactly 2.000 MHz** using a different engine:
the clock pin becomes a plain GPIO and TIM1 compare events drive DMA writes to
`GPIOB->BSRR`, with a third DMA capturing `GPIOB->IDR`. TIM1 runs at 180 MHz and
180/90 = 2.000 MHz exactly. The pulse count is just the DMA transfer count, so
the burst stops itself after n edges. **The wiring is unchanged** — only the
pin's mode differs while the burst runs.

This is the emulator's worst case: the EXTI3 handover then has half a clock
period, 250 ns, to get DATA onto the line before the master samples it.

Test it with `err on`, which forces PV=0. That matters: PV is normally 1 *and*
the DATA line idles HIGH, so a late handover would leave D31 reading 1 and look
correct. With PV forced to 0, a late handover shows up immediately as D31 = 1.

```
fixed 0x5A5A5
err on
read2m 25          # every frame must decode pv=0 zpd=1 pd=370085
err off
```

### One-cycle data latency is intentional

A value changed between Read Cycles appears in the *next* frame, not the current
one: the payload is staged at the end of Tmu, matching "after Tmu, the latest
position data is now available for transmission in the next Read Cycle"
(5.4.1 note 4). A real encoder latches its data the same way.

### Stage 2 — RS-422 through two MAX490

A MAX490 has one driver and one receiver, which is exactly the SSI topology:
DATA is driven differentially by the encoder, CLOCK is received differentially.
Use two modules, U1 on the encoder side and U2 on the controller side.

```
DATA  (encoder -> controller)
  PB4  ──▶ U1.DI     U1.Y/Z ══twisted pair══▶ U2.A/B     U2.RO ──▶ PB14

CLOCK (controller -> encoder)
  PB10 ──▶ U2.DI     U2.Y/Z ══twisted pair══▶ U1.A/B     U1.RO ──▶ PB3
```

- **Supply:** MAX490 is a 5 V part — feed it from CN6-5 (+5V). Its RO outputs
  swing to ~5 V, which is safe because PB3 and PB14 are both `FT`. In the other
  direction the STM32's 3.3 V output clears the MAX490's ~2 V input threshold.
- **Ground:** tie the two modules' grounds together and to the Nucleo GND.
- **Termination:** the Product Guide states that DATA outputs and CLOCK inputs
  are *not* terminated with load resistors. Add 120 Ω at the receiving end only
  if you run a long pair.

## Measured against the specification

Captured with a Saleae Logic Pro 16 at 125 MS/s (8 ns resolution), 3.3 V
threshold, falling-edge trigger on the clock: `burst2m 200 50` with
`fixed 0x5A5A5`, run once with PV=1 and once with `err on`.

**These numbers are measured against commit `be94f25`** (HSE clock, critical path
in SRAM, dynamic Tmu correction). Re-capture after touching the SSI path — an
earlier revision of this table was silently stale for exactly that reason.

![SSI4 Read Cycle captured at 2 MHz](docs/img/ssi4-2mhz-capture.svg)

| Property | Specified | Measured (200 Read Cycles) |
|---|---|---|
| Clocks per message | n = 32 | **32 on every cycle** |
| Clock rate | 100 kHz … 2 MHz | 1.9997 MHz (period 500.07 ns mean) |
| Tmu, last falling edge → DATA HIGH | 20 µs ± 1 µs | **19.89 µs mean, 20.06 µs max** |
| Idle state | CLOCK and DATA HIGH | HIGH |
| Gap level = Error Flag (inverse of PV) | LOW when PV=1 | LOW ×199; **HIGH ×199 under `err on`** |
| Payload | PD = 370085 (0x5A5A5) | **199/199 decode correctly** |
| First falling edge → DATA valid | < 0.5·T = 250 ns | **200–224 ns, 209 ns mean** |

Both runs show one non-matching first frame. That is the documented one-cycle
staging latency — the frame was staged before the preceding `err` command took
effect — not a decode failure; every subsequent cycle is correct.

What measuring actually changed:

- **Tmu was out of spec** at 21.01 µs, because end-of-message is detected on the
  last *rising* edge — half a clock period after the falling edge the
  specification measures from — and the interrupt adds latency on top.

  The half-period term is clock-rate dependent (0.25 µs at 2 MHz, 5 µs at
  100 kHz), so it is now **measured per frame rather than assumed**: the receive
  DMA's half-transfer event fires exactly `n_bits/2` clocks before
  transfer-complete, and both run in non-critical handlers, so `T` comes for
  free without touching `EXTI3_IRQHandler` and its 26 ns of margin. `stat`
  reports the measured `T`. Verified at both ends of the range:

  | Master clock | Measured T | Tmu (spec 20 µs ± 1) |
  |---|---|---|
  | 2.000 MHz | 498 ns | **19.69 µs** mean, 20.06 max |
  | 175.8 kHz | 5772 ns | **20.04 µs** mean, 19.96–20.12 |

  With the previous fixed correction the slow case would have sat near 22.6 µs,
  outside the window.
- **The Error Flag appears ~0.7 µs after the last rising edge**, not immediately:
  the end-of-frame interrupt has to run first, so the line still shows D0 for
  that long. It is 3.5 % of the Tmu window, and SSI4 carries validity in PV
  inside the frame rather than in the trailing flag, but a controller that
  samples the Error Flag very early would read the last data bit instead.
- **The handover margin is thin.** 224 ns worst case against a 250 ns budget is
  about 10 %, after moving the handler to SRAM (it was 232 ns / 7 % in flash).
  It passes, but it is not the comfortable margin estimated before measuring.
- **HSI cannot meet the Time Stamp accuracy, so HSE is now the default.**
  The field is specified accurate to "better than 1 % (based on the system
  oscillator)". Measured directly against the analyser's crystal by regressing
  the decoded TS field against capture time over a 0.4 s span:

  | Clock source | Clock period | Time Stamp tick | Accuracy | Spec (< 1 %) |
  |---|---|---|---|---|
  | HSI, 16 MHz RC | 493.02 ns | 9.8619 µs | **+1.401 %** | ✗ FAIL |
  | HSE, 8 MHz ST-LINK MCO | 500.03 ns | 9.9979 µs | **+0.021 %** | ✓ PASS |

  `SIMENC_CLOCK_SOURCE` now defaults to `AUTO`: try HSE, fall back to HSI if the
  board's solder bridges do not route the MCO. The banner reports which one
  locked, and flags `[timestamp OUT OF SPEC]` when it had to fall back.

  Measure this over a long span. A first attempt across only 13 ms gave a
  spurious +3.16 % for HSI, because the ~90 µs Read Cycle period beats against
  the 100 µs position-update tick that latches TS.

## Critical path in SRAM

There is **no TCM on this part** — tightly-coupled memory is a Cortex-M7 feature
and the STM32F446 is a Cortex-M4. The equivalent is to execute from SRAM, which
`SIMENC_RAMFUNC` (default **ON**) does for `EXTI3_IRQHandler`. No linker or
startup work is needed: the linker script already gathers `.RamFunc` inside the
`.data` output section, so the startup file's `_sdata.._edata` copy relocates it
at reset.

Measured on the handover at 2.000 MHz, `err on`, 1000 Read Cycles each:

| `EXTI3_IRQHandler` in | min | mean | max | jitter | worst-case margin |
|---|---|---|---|---|---|
| Flash (`0x080019d4`) | 184 ns | 205 ns | 232 ns | 48 ns | 18 ns (7.2 %) |
| **SRAM (`0x20000014`)** | 200 ns | 209 ns | **224 ns** | **24 ns** | **26 ns (10.4 %)** |

SRAM is *not* uniformly faster — its mean is 4 ns worse and its best case 16 ns
worse. What it does is halve the jitter and cut the tail, which is what a hard
deadline actually cares about: worst case improves 232 → 224 ns, lifting the
margin from 7.2 % to 10.4 %. That fits the ART accelerator's behaviour — flash
is quick when its instruction cache hits and occasionally slow when it does not,
while SRAM is uniform.

Two caveats worth keeping in mind. The 8 ns worst-case gain is exactly one
sample period at 125 MS/s, so the robust result here is the halved jitter, not
the 8 ns; both figures reproduced identically across 200- and 1000-cycle runs.
And 26 ns is still not a comfortable margin — this is an incremental
improvement, not a fix that makes 2 MHz safe by a wide margin.

Build the comparison yourself with `-DSIMENC_RAMFUNC=OFF`.

## Architecture

Layered so the SSI transport can be reused for the other SSI payload variants:

```
encoder/incoder.c        sensor behaviour: position, zero point, timestamp, PV/ZPD
        ↑
ssi/ssi4.c               SSI4 frame codec — pure logic, no hardware
        ↑
ssi/ssi_slave.c          generic SSI slave transport (SPI1 + DMA + Tmu gap)
```

`ssi_slave` moves *n* bits and knows nothing of their meaning, so SSI1/2/6/9
(all byte-aligned) can be added as further codecs beside `ssi4.c`.
`ssi/ssi_master.c` is a bring-up instrument, not part of the emulator.

### How a Read Cycle is served

1. Between messages PB4 is a plain GPIO holding the SSI idle-HIGH level.
2. On arming, PB4 is handed to SPI1 (slave, CPOL=1/CPHA=1 — the hardware then
   changes DATA on the falling edge and the master samples on the rising edge,
   exactly the SSI relationship). TX DMA feeds the 4 payload bytes.
3. The **receive** DMA is what counts clocks: its transfer-complete event fires
   precisely when 32 clocks have been seen, marking end of message.
4. That ISR takes PB4 back, drives the Error Flag level, and starts TIM6 as a
   one-shot for Tmu = 20 µs.
5. TIM6's update returns DATA to HIGH, stages the next frame and re-arms.

Position and timestamp are latched **together** by the 100 µs update tick, so
`TS` reports when the position was measured rather than when it was sent.

### Peripheral allocation

| Peripheral | Role | DMA (RM0390 Tables 28/29) |
|---|---|---|
| SPI1 slave | SSI DATA shift-out, clock counting | TX DMA2 S3 C3, RX DMA2 S2 C3 |
| TIM6 | Tmu one-shot gap | — |
| TIM7 | Time Stamp counter, 10 µs tick, wraps 2048 | — |
| TIM2 | 10 kHz update tick + ADC trigger (TRGO) | — |
| ADC1 IN0 | angle acquisition | DMA2 S0 C0, circular |
| USART2 | trace/console | TX DMA1 S6 C4 |
| SPI2 master | loopback harness | polled |

## Design decisions that are *not* from the specification

- **The analog input is 3.3 V only — there is no 5 V-capable ADC pin, and no
  way to make one.** DS10693 gives the ADC conversion range as `VAIN` =
  0…**VREF+**. The `FT` marking describes the *digital* I/O structure; in analog
  mode the pad switches straight onto the ADC sampling capacitor, so PA0 being
  `FT` does **not** make it a 5 V analog input.

  Neither supply trick helps. On the LQFP64 the datasheet pinout labels pin 13
  **`VDDA/VREF+`** and pin 12 **`VSSA/VREF-`** — VREF+ is internally bonded to
  VDDA, so raising VREF+ *is* raising VDDA. And DS10693 Table 13 gives the
  absolute maximum for "External main supply voltage (including VDDA, VDD,
  VDDUSB and VBAT)" as **4.0 V**, with an operating range of 1.7–3.6 V. Feeding
  5 V to VDDA/VREF+ — with or without SB57 removed — exceeds the absolute
  maximum and would damage the part, and VDDA also feeds the RCs and PLL, not
  just the ADC. Removing SB57 is only useful for supplying a *cleaner* reference
  from CN5 pin 8 within 1.7–3.6 V (keeping VDDA − VREF+ < 1.2 V).

  Scale a 5 V source with a divider instead, keeping the source impedance under
  the 50 kΩ `RAIN` limit: 5.1 kΩ / 10 kΩ gives 5 V → 3.31 V at ~3.4 kΩ.
- **Analog → angle mapping.** 0 V…VDDA maps linearly onto 0…524287 counts
  (0…360°). The ADC is 12-bit, so an analog-driven position moves in steps of
  128 counts even though the SSI4 field is 19-bit.
- **19-bit resolution.** SSI4 caps measurement resolution at 19 bits, so the
  emulator presents the maximum the variant allows.
- **Update rate 10 kHz.** The guide specifies "< 0.1 ms"; 100 µs is the fastest
  value satisfying it.
- **Error-flag hold time.** The guide holds the Error Flag for `Tmu − 0.5·T`;
  the emulator holds it for the full `Tmu`, since it does not measure `T`. The
  difference is at most 5 µs (at the 100 kHz clock limit).

## Known limitations

- `ssi_slave` supports byte-aligned frame lengths only (n = 8/16/24/32), which
  covers SSI1, SSI2, SSI4, SSI6 and SSI9. SSI7 (n=30) and SSI8 (n=18) would need
  bit-level padding.
- Single-turn only; the multi-turn variants (SSI31/32) are not implemented.
- The EXTI3 handover has a hard deadline: it must set DATA to the SPI output
  within half a clock period of the first falling edge (250 ns at the 2 MHz
  maximum, 356 ns at the default 1.4 MHz). It is the highest-priority interrupt
  in the system for that reason, and it is verified at 2.000 MHz with PV forced
  to 0 (see `read2m`). If D31 is ever seen wrong, this is where to look — drop
  the clock rate to confirm.
- **The handover margin is ~10 % and that is close to inherent.** See
  "Critical path in SRAM" below for the measurement. Roughly 200 ns of the
  budget is interrupt entry and EXTI propagation rather than the handler body,
  which is three register writes, so further code tuning has little headroom.
- The first `read`/`read2m` after any state change returns the previously staged
  frame — the one-cycle latency described above, not an error. Read twice.
