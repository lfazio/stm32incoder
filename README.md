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
| `docs/stm32/rm0390-….pdf` (RM0390 Rev 9) | DMA request mapping (Tables 28/29), bus maximum frequencies |
| `docs/stm32/um1724-….pdf` (UM1724 Rev 17) | board connectors (Table 19/29), LD2, VCP, HSE options |

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
| Master CLOCK out (test) | PB10 | GPIO, or AF5 SPI2_SCK | CN9-7 (D6) | CN10-25 | FT |
| Master DATA in (test) | PB14 | read via IDR, or AF5 SPI2_MISO | — | CN10-28 | FT |
| Angle analog in | PA0 | ADC1_IN0 | CN8-1 (A0) | CN7-28 | **0–3.3 V only** |
| Trace/console | PA2/PA3 | AF7 USART2 | — | — | to ST-LINK VCP |
| Status LED (LD2) | PA5 | GPIO | CN5-6 (D13) | — | — |

The test-master pins carry two alternatives: by default TIM1+DMA drives PB10 as
plain GPIO and samples PB14 through `GPIOB->IDR`, while `readspi`/`burstspi` put
both into AF5 for SPI2. Nothing about the wiring changes between the two.

SPI1 is deliberately **not** on its default PA5/PA6/PA7: PA5 carries the LD2 LED
and is `TTa` (3.3 V only), which would both load the clock line and be unsafe
against a 5 V MAX490 output.

> PB3 is JTDO/TRACESWO. Using it as SPI1_SCK costs SWO trace; SWD debugging
> (which is what the on-board ST-LINK uses) is unaffected.

## Console

USART2 → ST-LINK virtual COM port, **921600 8N1**. Commands are newline
terminated; `printf 'stat\r' > /dev/ttyACM0` works while a reader holds the port.

| Command | Effect |
|---|---|
| `help` | list the commands |
| `stat` | encoder and link state (see below) |
| `src adc\|fixed\|ramp` | select the position source |
| `fixed <counts>` | fixed position, 0…524287; also selects `fixed` |
| `ramp <step>` | counts added per 100 µs update; also selects `ramp` |
| `zero set\|reset` | set the zero point here, or restore the factory one (ZPD=1) |
| `err on\|off` | force PV=0, reporting the error condition |
| `ssi 1\|2\|4\|6\|9` | select the payload variant; **SSI4 is the default** |
| `wire` | loopback jumper continuity self-test |
| `clk <hz>` | test-master clock, any rate 100 kHz…2 MHz |
| `read [n]` | run *n* Read Cycles and decode each |
| `burst [n] [gapus]` | *n* cycles back to back, summary only — use for scope capture |
| `readspi [n]`, `burstspi [n] [gapus]` | same via the SPI baud generator, as a cross-check |

`read` also prints a diagnostic line after the burst — the three DMA `NDTR`
counters plus TIM1 and DMA2 status — which is what pinned down a clock that was
dying mid-burst. Ignore it unless something is wrong.

`stat` reports the position source, position, timestamp, PV/ZPD, the internal
update count, the raw ADC value, the zero offset, completed frames, resyncs,
dropped trace bytes, whether DATA is idle high, and the clock period the slave
measured during the last frame.

### State at power-on

| | Default |
|---|---|
| System clock | HSE if the ST-LINK MCO is strapped, else HSI — the banner says which, and flags `[timestamp OUT OF SPEC]` on the HSI fallback |
| Position source | `adc` — PA0, 0…VDDA mapped to 0…524287 counts |
| Zero point | factory, so ZPD = 1 |
| PV | 1 (`err off`) |
| SSI payload variant | **SSI4** — n = 32 bits, 19-bit position, 10 µs Time Stamp |
| SSI slave | armed, Tmu = 20 µs |
| Test-master clock | **500 kHz** (timer engine) |
| LD2 | blinks at 1 Hz; hold B1 to stream state |

The emulator is serving Read Cycles from reset — no command is needed to start
it. The console only changes what it reports and drives the loopback test
master, which is not part of the emulated device.

## SSI payload variants

The transport moves *n* bits and knows nothing of their meaning, so the payload
variants are sibling codecs in `ssi/ssi_variant.c`. `ssi <n>` switches between
them at runtime; it reconfigures the frame length, the position field width and
the Time Stamp tick, then re-arms.

| Variant | n | Layout (Product Guide 5.4.2) | Position | Time Stamp |
|---|---|---|---|---|
| SSI1 | 24 | D23 PV, D22 ZPD, D21-D0 PD | 22 bit | — |
| SSI2 | 24 | D23-D2 PD, D1 parity, D0 alarm | 22 bit | — |
| **SSI4** | 32 | D31 PV, D30 ZPD, D29-D11 PD, D10-D0 TS | 19 bit | 10 µs |
| SSI6 | 32 | D31-D24 CRC-8, D23 PV, D22 ZPD, D21-D0 PD | 22 bit | — |
| SSI9 | 32 | as SSI4 | 19 bit | 1 µs |

Verified on the wire with `fixed 0x12345`, each decoding back to 74565 and each
running `burst 100 50` at `ok=98 bad=0`:

```
ssi1  C12345      PV|ZPD|0x12345
ssi2  048D16      0x12345<<2, parity 1 (odd), alarm 0
ssi4  C91A2AA2    PV|ZPD|0x12345<<11|ts
ssi6  DFC12345    CRC-8 0xDF over the 24-bit body
ssi9  C91A2B92    as ssi4, TS counting in 1 µs steps
```

SSI7 (n=30) and SSI8 (n=18) are **not** implemented: they are not byte aligned,
and `ssi_slave` transfers whole bytes.

**The SSI6 CRC is computed in software, and has to be.** The STM32F446 has a CRC
peripheral, but RM0390 4.1 describes "a *fixed* generator polynomial" and 4.2
names it: "CRC-32 (Ethernet) polynomial: 0x4C11DB7". There is no `CRC_POL` or
`POLYSIZE` register on this family — a programmable polynomial and width arrived
with F0/F3/L4/H7. SSI6 needs CRC-8 with polynomial 0x97, so the hardware unit
cannot produce it. The software version is 24 shift/xor steps inside
`stage_frame()`, which runs in the Tmu handler *after* DATA has already been
driven high, so it is off both the 250 ns handover path and the Tmu measurement.

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

`wire` is safe to run at any time: the slave resets SPI1 through
`RCC->APB2RSTR` on every arm, so the clock edges the test injects cannot leave
the byte framing skewed. It used to, and needed a board reset afterwards — see
"Arming resets the peripheral" below.

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

`clk <hz>` sets the master clock. It is generated by TIM1 compare events driving
DMA writes to `GPIOB->BSRR`, so **any rate of the form 180 MHz / N is reachable**
across the whole specified window — not just the four rates the SPI baud
generator can produce. N is bounded by the window itself: N = 90 is 2.000 MHz,
N = 1800 is 100.0 kHz. Resolution is coarsest at the top (1.1 % per step at
2 MHz, 0.06 % at 100 kHz).

Verified against the slave's own independent period measurement:

| Requested | Programmed | Slave measured T |
|---|---|---|
| 2 MHz | 2 000 000 Hz | 495 ns |
| 1 MHz | 1 000 000 Hz | 995 ns |
| 500 kHz | 500 000 Hz | 1997 ns |
| 250 kHz | 250 000 Hz | 3999 ns |
| 100 kHz | 100 000 Hz | 10000 ns |

Every rate programmed exactly (+0.0 %), and `burst 40` decoded `ok=38 bad=0` at
each — `burst` skips two warm-up cycles, so it reports `n-2`. The power-on
default is **500 kHz**.

`readspi` / `burstspi` run the same traffic through the SPI baud generator
instead. That path only reaches 1.40625 MHz, 703.125 kHz, 351.5625 kHz and
175.78125 kHz, but it samples DATA with a hardware shift register rather than a
phase-programmed DMA read, so it is a useful independent cross-check.

### The 2 MHz corner

`clk 2000000` puts the link at the SSI maximum, which is the emulator's worst
case: the EXTI3 handover then has half a clock period, 250 ns, to get DATA onto
the line before the master samples it.

Test it with `err on`, which forces PV=0. That matters: PV is normally 1 *and*
the DATA line idles HIGH, so a late handover would leave D31 reading 1 and look
correct. With PV forced to 0, a late handover shows up immediately as D31 = 1.

```
fixed 0x5A5A5
clk 2000000
err on
burst 200 50       # every frame must decode pd=370085 with pv=0
err off
```

### Arming resets the peripheral

`ssi_arm()` resets SPI1 through `RCC->APB2RSTR` rather than merely clearing
`SPE`, because **clearing `SPE` does not empty the transmit buffer**. A byte
left there is shifted out ahead of the next frame, so every following Read Cycle
arrives one byte late — and since each re-arm just queues four more bytes behind
the stale one, the offset never clears itself.

That was reachable in practice: `wire` toggles the clock pin, the armed SPI
counted those edges, and a partial byte was stranded. At 500 kHz, `burst 200 50`
went from `ok=198 bad=0` to `ok=7 bad=191` if `wire` had run first. With the
reset it is `ok=198 bad=0` either way, and the first Read Cycle after `wire` —
which used to be reliably wrong — now decodes correctly too.

### One-cycle data latency is intentional

A value changed between Read Cycles appears in the *next* frame, not the current
one: the payload is staged at the end of Tmu, matching "after Tmu, the latest
position data is now available for transmission in the next Read Cycle"
(5.4.1 note 4). A real encoder latches its data the same way.

Measured, changing `fixed` and reading immediately each time:

| after | Read Cycle 0 | Read Cycles 1+ |
|---|---|---|
| `fixed 0x12345` | 370085 *(previous)* | **74565** |
| `fixed 0x5A5A5` | 74565 *(previous)* | **370085** |
| `fixed 0x2AAAA` | 370085 *(previous)* | **174762** |

Exactly one cycle, and every stale frame is well formed — `pv=1`, `zpd=1`,
correct structure. `burst` skips two warm-up cycles rather than one: only one is
needed for the staging itself, the second is margin against a console command
landing mid-cycle.

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
threshold, falling-edge trigger on the clock: `clk 2000000` then `burst 200 50` with `fixed 0x5A5A5`, run once with PV=1 and once with `err on`.

**These numbers were measured with HSE locked, the critical path in SRAM
(`SIMENC_RAMFUNC=ON`) and the dynamic Tmu correction in place.** Re-capture with
`tools/logic/` after touching the SSI path — an earlier revision of this table
was silently stale for exactly that reason. (This used to cite a commit hash;
history has since been rewritten twice, which made the hash dangle. The build
configuration is the durable reference.)

![SSI4 Read Cycle captured at 2 MHz](docs/img/ssi4-2mhz-capture.svg)

| Property | Specified | Measured (200 Read Cycles) |
|---|---|---|
| Clocks per message | n = 32 | **32 on every cycle** |
| Clock rate | 100 kHz … 2 MHz | 2.0008 MHz (period 499.79 ns mean) |
| Tmu, last falling edge → DATA HIGH | 20 µs ± 1 µs | **19.95 µs mean, 19.90–20.06** |
| Idle state | CLOCK and DATA HIGH | HIGH |
| Gap level = Error Flag (inverse of PV) | LOW when PV=1 | LOW ×200; **HIGH ×299 under `err on`** |
| Payload | PD = 370085 (0x5A5A5) | **200/200 decode correctly** |
| First falling edge → DATA valid | < 0.5·T = 250 ns | **192–224 ns, 209 ns mean** (n=299) |

The PV=1 run is clean on all 200 cycles, including the first. It was 199/200
before `ssi_arm()` began resetting the peripheral — that stale opening frame was
the transmit-buffer bug, not an inherent artefact. The `err on` run still shows
one non-matching frame, which is the genuine one-cycle staging latency: it was
staged before the `err` command took effect.

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
  | 2.000 MHz | 498 ns | **19.95 µs** mean, 19.90–20.06 |
  | 100.0 kHz | 10000 ns | **20.04 µs** mean, 19.96–20.08 |

  Both ends of the specified range — a 20× span in clock period — land within
  0.05 µs of the 20 µs target, which is the point of measuring `T` per frame
  rather than assuming it.

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
| Flash (`-DSIMENC_RAMFUNC=OFF`) | 184 ns | 205 ns | 232 ns | 48 ns | 18 ns (7.2 %) |
| **SRAM (default)** | 200 ns | 209 ns | **224 ns** | **24 ns** | **26 ns (10.4 %)** |

SRAM is *not* uniformly faster — its mean is 4 ns worse and its best case 16 ns
worse. What it does is halve the jitter and cut the tail, which is what a hard
deadline actually cares about: worst case improves 232 → 224 ns, lifting the
margin from 7.2 % to 10.4 %. That fits the ART accelerator's behaviour — flash
is quick when its instruction cache hits and occasionally slow when it does not,
while SRAM is uniform.

Re-measured after `ssi_arm()` gained the peripheral reset: 192–224 ns over 299
cycles, so the worst case is unchanged at 224 ns — the reset costs the handover
nothing.

Two caveats worth keeping in mind. The 8 ns worst-case gain is exactly one
sample period at 125 MS/s, so the robust result here is the halved jitter, not
the 8 ns; both figures reproduced identically across 200- and 1000-cycle runs.
And 26 ns is still not a comfortable margin — this is an incremental
improvement, not a fix that makes 2 MHz safe by a wide margin.

Build the comparison yourself with `-DSIMENC_RAMFUNC=OFF`.

**Only `EXTI3_IRQHandler` belongs in SRAM.** Moving the rest of the SSI path
there too — the end-of-frame DMA handler, the Tmu timer handler, `stage_frame`
and `ssi_arm` — was measured and is *worse*:

| | Error Flag latency | Tmu | RAM |
|---|---|---|---|
| EXTI3 only (shipping) | **0.64 µs** mean, 0.71 max | 19.89 µs | 6472 B |
| Whole path in SRAM | 0.71 µs mean, 0.75 max | 20.03 µs | 7040 B |

(Both rows were measured before `ssi_arm()` gained the peripheral reset, so
their absolute Tmu reads 19.89 µs rather than today's 19.95 µs. They were taken
under identical conditions, so the comparison between them still stands.)

That is the same effect seen above, pointing the other way: SRAM trades mean
speed for determinism. The handover needs the tail bounded because it has a hard
250 ns deadline, so it wins there. The end-of-frame path has no hard deadline —
its latency is compensated, and its jitter is ~±0.1 µs against a ±1 µs Tmu
window — so the mean is what matters and flash is quicker. It would also cost
568 bytes of RAM and force `GAP_ISR_OVERHEAD_NS` to be re-tuned.

The test master needs nothing: its clock comes from TIM1 compare events driving
DMA, with software out of the loop entirely.

## Architecture

Layered so the SSI transport can be reused for the other SSI payload variants:

```
encoder/incoder.c        sensor behaviour: position, zero point, timestamp, PV/ZPD
        ↑
ssi/ssi4.c               SSI4 frame codec — pure logic, no hardware
        ↑
ssi/ssi_slave.c          generic SSI slave transport (SPI1 + DMA + Tmu gap)
```

"Generic" here means **payload-agnostic, not hardware-agnostic**: `ssi_slave`
moves *n* bits and knows nothing of their meaning, so SSI1/2/6/9 (all
byte-aligned) can be added as further codecs beside `ssi4.c`. It is still
specifically the SPI1-based transport — porting to another peripheral means
editing it, not swapping a back end.

`ssi/ssi_master.c` is a bring-up instrument, not part of the emulated device.

### How a Read Cycle is served

1. Between messages PB4 is a plain GPIO holding the SSI idle-HIGH level.
2. On arming, SPI1 is enabled (slave, CPOL=1/CPHA=1 — the hardware then changes
   DATA on the falling edge and the master samples on the rising edge, exactly
   the SSI relationship) and TX DMA is loaded with the 4 payload bytes. **PB4
   stays GPIO**, because the SPI drives MISO LOW while it holds the pin with no
   clock running, which would break the idle-HIGH requirement.
3. `EXTI3` fires on the master's **first falling edge** and hands PB4 to SPI1.
   This is the hard deadline: it must complete within half a clock period.
4. The **receive** DMA is what counts clocks: its half-transfer event times the
   clock period, and its transfer-complete event fires precisely when 32 clocks
   have been seen, marking end of message.
5. That ISR takes PB4 back, drives the Error Flag level, and starts TIM6 as a
   one-shot for Tmu less the measured half period and the fixed ISR overhead.
6. TIM6's update returns DATA to HIGH, stages the next frame and re-arms.

Position and timestamp are latched **together** by the 100 µs update tick, so
`TS` reports when the position was measured rather than when it was sent.

### Peripheral allocation

| Peripheral | Role | DMA (RM0390 Tables 28/29) |
|---|---|---|
| SPI1 slave | SSI DATA shift-out, clock counting | TX DMA2 S3 C3, RX DMA2 S2 C3 |
| EXTI3 | first-falling-edge handover of the DATA pin | — |
| TIM6 | Tmu one-shot gap, 0.1 µs tick | — |
| TIM7 | Time Stamp counter, 10 µs tick, wraps 2048 | — |
| TIM2 | 10 kHz update tick + ADC trigger (TRGO) | — |
| ADC1 IN0 | angle acquisition | DMA2 S0 C0, circular |
| USART2 | trace/console | TX DMA1 S6 C4 |
| **TIM1** | **test-master clock, any 180 MHz / N** | **CH1→DMA2 S1 C6 (clock low), CH3→S6 C6 (clock high), CH4→S4 C6 (sample IDR)** |
| SPI2 master | `readspi`/`burstspi` cross-check only, polled | — |

The two masters are alternatives, not layers: TIM1 drives the clock pin as plain
GPIO via DMA writes to `BSRR`, while SPI2 drives it as AF5. Only one owns PB10
at a time, and the timer engine restores the pin afterwards.

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
- **Error-flag hold time.** The guide holds the Error Flag for `Tmu − 0.5·T`.
  The emulator does measure `T` — from the receive DMA's half-transfer to
  transfer-complete interval — and sets the gap to `Tmu − 0.5·T` less a fixed
  interrupt overhead, so Tmu lands inside spec across the whole clock range
  (verified 19.95 µs at 2 MHz and 20.04 µs at 100 kHz). The residual deviation
  is at the *start* of the gap, not its length: the Error Flag appears ~0.7 µs
  late because the end-of-frame interrupt has to run first.

## Known limitations

- `ssi_slave` supports byte-aligned frame lengths only (n = 8/16/24/32). SSI1,
  SSI2, SSI4, SSI6 and SSI9 are implemented; SSI7 (n=30) and SSI8 (n=18) would
  need bit-level padding in the transport first.
- The SSI6 CRC is checked against our own implementation of the guide's
  parameters, which proves round-trip consistency rather than conformance to an
  external reference vector.
- Single-turn only; the multi-turn variants (SSI31/32) are not implemented.
- The EXTI3 handover has a hard deadline: it must set DATA to the SPI output
  within half a clock period of the first falling edge — 250 ns at the 2 MHz
  maximum, 1 µs at the 500 kHz power-on default. It is the highest-priority
  interrupt in the system for that reason, and is verified at 2.000 MHz with PV
  forced to 0 (`clk 2000000` then `err on`). If D31 is ever seen wrong, this is
  where to look — drop the clock rate to confirm.
- **The handover margin is ~10 % and that is close to inherent.** See
  "Critical path in SRAM" above for the measurement. Roughly 200 ns of the
  budget is interrupt entry and EXTI propagation rather than the handler body,
  which is three register writes, so further code tuning has little headroom.
