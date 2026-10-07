# stm32incoder — Zettlex IncOder emulator (SSI4) on NUCLEO-F446RE

Emulates a Zettlex/Celera Motion IncOder inductive angle encoder speaking the
**SSI4** protocol variant, on a NUCLEO-F446RE. The shaft angle comes from one
analog input; the SSI link is driven entirely by hardware — the incoming clock
itself shifts each bit out through DMA — so no interrupt runs per bit, and none
runs per frame on the data path at all.

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
git submodule update --init          # cmsis_core, cmsis_device_f4, hal_driver

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

### Build options

| Option | Default | Effect |
|---|---|---|
| `SIMENC_CLOCK_SOURCE` | `AUTO` | `HSE`, `HSI`, or try HSE and fall back |

The SSI clock must reach **PC6** (`TIM8_CH1`, morpho **CN10-4**). That is the
only clock input: it clocks TIM8, which is what shifts DATA out. Without it no
message ever completes and the link is simply dead — `stat` reports
`etr=NEVER REACHED n` rather than leaving that a puzzle.

How the bits get onto the wire is described in
[How each bit reaches the wire](#how-each-bit-reaches-the-wire).

## Pin map

All alternate functions verified in DS10693 Table 11; 5 V tolerance ("FT") in
Table 10; connector positions in UM1724 Tables 19 and 29.

| Signal | Pin | AF | Arduino | Morpho | I/O structure |
|---|---|---|---|---|---|
| SSI **CLOCK in** (slave) | **PC6** | **AF3 TIM8_CH1** | — | **CN10-4** | FT (5 V tolerant) |
| SSI **DATA out** (slave) | PB4 | GPIO out, written by DMA | CN9-6 (D5) | CN10-27 | — |
| Master CLOCK out (test) | PB10 | GPIO, driven by DMA | CN9-7 (D6) | CN10-25 | FT |
| Master DATA in (test) | PB14 | GPIO in, read via IDR | — | CN10-28 | FT |
| Angle analog in | PA0 | ADC1_IN0 | CN8-1 (A0) | CN7-28 | **0–3.3 V only** |
| Trace/console | PA2/PA3 | AF7 USART2 | — | — | to ST-LINK VCP |
| Status LED (LD2) | PA5 | GPIO | CN5-6 (D13) | — | — |

**There is one clock input.** PC6 clocks TIM8, which is what shifts each data
bit out, so that is the only pin the SSI clock has to reach. It is otherwise
unused on this board and is 5 V tolerant, so it takes the MAX490 receiver
output directly.

PB4 is a plain GPIO output throughout — the DMA writes it through `BSRR`. It
never belongs to a peripheral, so there is no alternate function to program and
no mode to flip mid-frame.

Both test-master pins are plain GPIO too: TIM1 compare events drive PB10 through
DMA writes to `BSRR`, and a third DMA samples PB14 through `GPIOB->IDR`.

> PB3 and PB5 carried the SPI1 clock and MOSI and are no longer used at all.
> PB3 is JTDO/TRACESWO, so SWO trace is free again.

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

Checked on the analyser at 500 kHz, 100 Read Cycles each, with the payload
decoded independently in `tools/logic/variants.py` — parity and CRC recomputed
there rather than trusted from the firmware:

| Variant | clocks/cycle | Tmu | frames | integrity |
|---|---|---|---|---|
| SSI1 | 24 × 100 | 20.13 µs | pass | — |
| SSI2 | 24 × 100 | 20.34 µs | pass | parity ✓ |
| SSI4 | 32 × 100 | 20.03 µs | pass | — |
| SSI6 | 32 × 100 | 20.04 µs | pass | CRC-8 ✓ |
| SSI9 | 32 × 100 | 20.02 µs | pass | — |

The independent CRC-8 of the SSI6 body `0xC12345` is `0xDF`, which is what the
firmware emitted — so that field is now confirmed against a second
implementation of the guide's parameters, not just round-tripped.

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
| CLOCK | **CN10-25** (PB10, master out) | **CN10-4** (PC6, slave in) |
| DATA | **CN10-27** (PB4, slave out) | **CN10-28** (PB14, master in) |

The DATA pair **CN10-27 / CN10-28 are directly opposite each other** across the
two rows of CN10, so a plain 2-pin jumper cap works there — no wire needed.

Check the wiring before anything else — the firmware can test it itself:

```
wire
```

It drives PB10 and PB4 as GPIO and verifies that PB3 and PB14 follow both
levels, printing `OK` or `OPEN` per wire. It does **not** check the PC6 tap —
`stat` does, reporting `etr=counted n` once a full message has been clocked, or
`etr=NEVER REACHED n` if that wire is missing. Do not test that tap by asking
whether the counter has seen an edge: a floating pin picks up enough noise to
answer yes while the link stays dead. `OPEN` means that pin pair is not
connected, whatever the wire looks like.

`wire` is safe to run at any time: every arm reloads the DMA transfer counts
from scratch, so the clock edges the test injects cannot leave the framing
skewed.

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
across the whole specified window. N is bounded by the window itself: N = 90 is 2.000 MHz,
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


### Oversampling and the update rate

Two rates, easily conflated, and the guide fixes both:

- **Position latch: 10 kHz.** §4.12 gives "Internal Position Update Period
  < 0.1 millisecond", and §5.5.1 pins the value by describing ASI2 frames as
  "transmitted at a rate of 10kHz nominal (*same rate as Internal Position
  Update Period*)". So 100 µs is the specified rate, not a floor to beat.
- **Time Stamp tick: 10 µs**, i.e. a 100 kHz counter spanning 0.00…20.47 ms
  (§5.4.2). That 100 kHz is the *counter*, not the latch.

The emulator has always latched at 10 kHz and ticked TS at 100 kHz, which is
correct. What was wrong is what the latch averaged: four ADC samples taken *at*
the update rate, so every position was a moving average over the previous
400 µs — four update periods. TS claims to record when the position was
measured, and a 400 µs smear makes that claim false.

The ADC now free-runs and each update averages the 12 conversions taken during
that period (12 × 8.53 µs = 102 µs against the 100 µs period), which is both
genuinely per-period and worth about 1.8 bits of noise averaging.

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

**Verified on the RS-422 link**, with both pairs running through the two
modules, `fixed 0x5A5A5`, `burst 200 50` at each rate:

| Master clock | Result | Payload |
|---|---|---|
| 100 kHz | 198/198 | correct |
| 250 kHz | 198/198 | correct |
| 500 kHz | 198/198 | correct |
| 1.0 MHz | 198/198 | correct |
| 1.5 MHz | 194–197 / 200 | correct |
| 2.0 MHz | 196–198 / 200 | shifted one bit — see below |

`resyncs=0` throughout, and all five variants are clean at 500 kHz (98/98 each
for SSI1, SSI2, SSI4, SSI6, SSI9). The two top rates scatter by half to two per
cent run to run, but they scatter by the same amount over the plain TTL jumpers
measured the same afternoon, so that is the link's own margin at those rates
rather than anything the transceivers introduce. (Those figures predate the
timer data path.)

The one-bit shift at 2 MHz — the test master decodes PD as 447186 =
`(0x5A5A5 >> 1) | 0x40000` instead of 370085, consistently, which is why the
count still reads 198/198 — is the on-board master sampling, not the wire. The
logic analyser decodes the same frames correctly at 2 MHz. It is the KNOWN
DEVIATION above seen from the receiving end: master and slave are both a half
period out, in the same direction, which cancels below ~1 MHz and stops
cancelling as the period approaches the sampling delay.

## Conformance: clock edges and the Error Flag

Everything below is measured on the analyser, not inferred.

### How each bit reaches the wire

SSI puts each data bit on the CLOCK's **rising** edge; the controller then reads
it in the phase that follows, sampling on the **falling** edge. The first
falling edge only starts the Read Cycle and latches the position — no data moves
on it.

The sources agree:

- IncOder §5.4.1 note 2: "Each rising edge of the CLOCK transmits the next data
  bit of the message, starting with Dn-1."
- POSITAL: "With the following rising edge transition of the clock signal the
  transmission begins with the most significant bit (MSB). With each following
  rising edge transition of the clock signal, the next bit is set on the output
  of the data line."
- RLS: "The MSB then appears on the DATA output at the next rising edge… At each
  subsequent rising edge of the CLOCK the next bit is transmitted."

**No peripheral shifts the bits.** The incoming clock does it directly. TIM8 is
clocked by `TI1F_ED`, the channel-1 edge detector, which counts **both** clock
edges; with `ARR=1` the counter wraps on every second edge, and because the
clock idles HIGH the edges pair as (F1,R1), (F2,R2)… so the wrap always lands on
a rising edge. A frozen channel-2 compare with `CCR2=0` matches at that wrap and
raises one DMA request, which writes the next bit to `GPIOB->BSRR`.

A second compare, channel 4 with `CCR4=1`, matches when the counter is at 1 —
every falling edge — and drives a second DMA from a buffer of n+1 words: n zeros
and then the Error Flag. Writing 0 to `BSRR` sets and resets nothing, so the
first n are deliberate no-ops and only the (n+1)'th falling edge moves the line.
That edge ends the Read Cycle, by which point the controller has sampled D0, so
it is the only instant satisfying both §5.4.1 note 3 and the controller.

What this buys, structurally rather than by tuning:

- **DATA changes on the rising edge**, +56 ns, which is DMA latency.
- **Nothing touches the line between F1 and R1** — no compare event exists
  there, so the first bit period cannot be disturbed.
- **No pin handover.** PB4 is a GPIO output for the whole cycle; only the
  *writer* changes. There is no 250 ns deadline and no interrupt in the bit path.
- **The Error Flag is placed by hardware**, not by an interrupt, so the gap
  cannot open with a spurious pulse.
- **n need not be a multiple of 8**, which is what kept SSI7 and SSI8 out.

![One Read Cycle with DATA shifted by the clock](docs/img/ssi4-500khz-timer.svg)

Measured on the analyser at 500 kHz, 198 cycles:

| | result |
|---|---|
| Clocks per cycle | 32 on every cycle |
| **DATA after its rising edge** | **+56 ns**, 3720/3720 |
| **DATA moves between F1 and R1** | **0 / 198** |
| **Gaps opening with a HIGH pulse** | **0 / 197** |
| Payload sampled at the falling edge | **198 / 198 correct** |
| Tmu | 20.43 µs |

The +56 ns is 2.8 % of a bit period at 500 kHz and 22 % at 2 MHz, so the bit is
settled long before the controller samples it on the following falling edge.

**Both the clock taps and the probe must come off the same point.** For a while
DATA appeared to be written 72 ns *before* its rising edge. It was the wiring: a
second clock tap had been taken upstream of the MAX490 pair while the receiver
fed the other, so the timer was triggered by an early copy of the clock. With
one receiver feeding one pin this cannot arise; it was a bench artefact.

### What the loopback can and cannot prove here

The test master samples where the slave puts the data, and the two have to move
together. It samples late in the high phase, just before the next falling edge,
which is where an SSI controller reads.

Getting this wrong reads every frame shifted by one bit, and it did: the master
used to sample one timer tick after the rising edge, which was right while the
slave drove on the falling edge and became wrong when it moved to the rising
one. The slave puts each bit up 56 ns after that edge, so a sample 5.6 ns after
it captured the *previous* bit, and `fixed 0x5A5A5` read back as 447186 —
`(pd >> 1) | (ZPD << 18)` exactly.


### The Error Flag, and why it is hardware-placed

§5.4.1 note 3 hands the data line to the Error Flag after the last rising edge.
Driving it from software does not work well enough: an earlier build set it in
the end-of-message interrupt and it arrived **0.83 µs late** at 500 kHz — 41 % of
a bit period. Worse than "late", it was *wrong*: when the last data bit D0
happened to be 1 the line was already HIGH when the message ended and stayed
HIGH until the interrupt pulled it down, so the gap **opened with a HIGH pulse**
while PV=1 says it must be LOW throughout. A controller reading the Error Flag
early sees a spurious error. D0 is the Time Stamp LSB, so it hit about half of
all cycles — 96 of 197 measured.

| | gaps opening HIGH | pulse width |
|---|---|---|
| software, from the end-of-message ISR | 96 / 197 | 1800 ns |
| **hardware, CC4 DMA at the last falling edge** | **0 / 197** | — |

Measuring only the settled gap level hides this completely — that is what "gap
level LOW ×200" reports — and it is why it went unnoticed. It was caught by eye
on a capture, not by the analysis scripts, which now check for it.


## Measured against the specification

Captured with a Saleae Logic Pro 16 at 125 MS/s (8 ns resolution), 3.3 V
threshold, falling-edge trigger on the clock: `clk 2000000` then `burst 200 50` with `fixed 0x5A5A5`, run once with PV=1 and once with `err on`.

**These numbers were measured with HSE locked and the dynamic Tmu correction in
place.** Re-capture with
`tools/logic/` after touching the SSI path — an earlier revision of this table
was silently stale for exactly that reason. (This used to cite a commit hash;
history has since been rewritten twice, which made the hash dangle. The build
configuration is the durable reference.)

### The ordinary case, at 500 kHz

At 500 kHz one bit is 2 µs, so a whole Read Cycle fits on the page at a scale
where the frame layout is legible.

![One SSI4 Read Cycle captured at 500 kHz](docs/img/ssi4-500khz-capture.svg)

One cycle, all 32 clocks, with the SSI4 fields banded over the bit cells: PV,
ZPD, PD[18:0], TS[10:0]. The payload decoded straight off the capture is
`PD=370085` — the 0x5A5A5 that was set — with PV=1 and ZPD=1, and the whole run
is **200/200 correct**, 32 clocks on every cycle, `Tmu = 20.16 µs` mean
(20.10–20.21), gap LOW throughout. The Error Flag follows the last rising edge
by 0.83 µs.

The faint vertical rules mark every **clock rising edge** — the edges the
specification says each data bit should be set on. DATA transitions land
between them, on the falling edges — this capture predates the timer data path
and is kept only to show what the deviation looked like. The green rule is the
first falling edge, which starts the Read Cycle.

### The same thing at 2 MHz

2 MHz is the top of the specified range, and on the default data path it is not
a special case — there is no handover, no deadline and nothing to tighten, so it
measures like any other rate.

![SSI4 Read Cycle captured at 2 MHz](docs/img/ssi4-2mhz-capture.svg)

| | 500 kHz | 2 MHz |
|---|---|---|
| Clocks per cycle | 32 on every one | 32 on every one |
| Payload | 197/197 | 197/197 |
| DATA after its rising edge | +56 ns, 3720/3720 | +56 ns, 3740/3740 |
| — as a fraction of the half period | 2.8 % | 22 % |
| DATA moves between F1 and R1 | 0 | 0 |
| Gaps opening with a HIGH pulse | 0 | 0 |
| Tmu (spec 20 µs ± 1) | 20.43 µs | 19.66 µs |

Both land inside the Tmu window from the same dynamic correction, which is the
point of measuring `T` per frame rather than assuming it: the half-period term
it removes is 1 µs at 500 kHz and 0.25 µs at 2 MHz.


## Architecture

Layered so the SSI transport can be reused for the other SSI payload variants:

```
encoder/incoder.c        sensor behaviour: position, zero point, timestamp, PV/ZPD
        ↑
ssi/ssi_variant.c        SSI payload codecs — pure logic, no hardware
        ↑
ssi/ssi_slave.c          generic SSI slave transport (TIM8 + DMA + Tmu gap)
```

"Generic" here means **payload-agnostic, not hardware-agnostic**: `ssi_slave`
moves *n* bits and knows nothing of their meaning, so SSI1/2/6/9 (all
byte-aligned) live as further codecs in `ssi_variant.c`. It is still
specifically the TIM8+DMA transport — porting to another part means editing it,
not swapping a back end.

`ssi/ssi_master.c` is a bring-up instrument, not part of the emulated device.

### How a Read Cycle is served

1. Between messages PB4 is a GPIO output holding the SSI idle-HIGH level. It
   stays a GPIO output for the whole cycle; only the *writer* changes.
2. On arming, the next frame is expanded into two `BSRR` buffers — one word per
   bit, and one word per falling edge holding n no-ops then the Error Flag —
   and both DMA streams are loaded. TIM8's counter is zeroed and started.
3. The master's **first falling edge** is counted but writes only a no-op, so
   nothing happens on the line. There is no handover and no deadline.
4. Each **rising** edge wraps the counter, matching the CH2 compare, and the DMA
   writes the next bit. The half-transfer event times the clock period on the
   way past.
5. After n bits the DMA's transfer-complete marks end of message and starts TIM6
   as a one-shot for Tmu less the measured half period and the ISR overhead.
   The Error Flag is *not* set here — the CH4 DMA already placed it on the last
   falling edge.
6. TIM6's update returns DATA to HIGH, stages the next frame and re-arms.

Position and timestamp are latched **together** by the 100 µs update tick, so
`TS` reports when the position was measured rather than when it was sent.

### Peripheral allocation

| Peripheral | Role | DMA (RM0390 Tables 28/29) |
|---|---|---|
| **TIM8** | **clocked by the SSI clock on PC6 (TI1F_ED, both edges, ARR=1); CH2 compare shifts each data bit, CH4 compare places the Error Flag** | **CH2→DMA2 S3 C7, CH4→DMA2 S7 C7, both → `GPIOB->BSRR`** |
| TIM6 | Tmu one-shot gap, 0.1 µs tick | — |
| TIM7 | Time Stamp counter, 10 µs tick, wraps 2048 | — |
| TIM2 | 10 kHz update tick + ADC trigger (TRGO) | — |
| ADC1 IN0 | angle acquisition | DMA2 S0 C0, circular |
| USART2 | trace/console | TX DMA1 S6 C4 |
| **TIM1** | **test-master clock, any 180 MHz / N** | **CH1→DMA2 S1 C6 (clock low), CH3→S6 C6 (clock high), CH4→S4 C6 (sample IDR)** |

Both DATA streams must be on DMA2: GPIO is on AHB1 and DMA1 cannot reach it.
That is also why the timer is TIM8 — it is the free APB2 timer, and APB2 timer
requests land on DMA2. No SPI is involved anywhere, at either end of the link.

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
  (verified 20.07 µs at 2 MHz and 20.14 µs at 100 kHz). The residual deviation
  is at the *start* of the gap, not its length: the Error Flag appears ~0.7 µs
  late because the end-of-frame interrupt has to run first.

## Known limitations

- **The SSI clock must reach PC6** (CN10-4). It is the only clock input, and
  without it nothing works at all — `stat` says `etr=NEVER REACHED n`.
- `ssi_slave_set_frame_bits()` still rejects frame lengths that are not a
  multiple of 8, so SSI7 (n=30) and SSI8 (n=18) remain unimplemented. Nothing in
  the data path requires that any more — end of message comes from a DMA
  transfer count of n, whatever n is — so the restriction is now only the guard
  and the variant table.
- The SSI6 CRC is checked against our own implementation of the guide's
  parameters, which proves round-trip consistency rather than conformance to an
  external reference vector.
- Single-turn only; the multi-turn variants (SSI31/32) are not implemented.
- There is **no per-frame deadline left**. The old SPI path had to hand the
  DATA pin over within half a clock period of the first falling edge — 250 ns at
  2 MHz, with about 10 % margin. That handover no longer exists, so neither does
  the deadline or the interrupt priority built around it.
