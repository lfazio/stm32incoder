# Logic-analyser verification

Drives a Saleae Logic Pro 16 through the `logic2` MCP server's JSON-RPC endpoint
(default `http://127.0.0.1:10530`) and checks the captured waveform against the
IncOder Product Guide. Channel 0 is CLOCK, channel 1 is DATA.

Wire the loopback first (README "Stage 1") and confirm with `wire` on the
console. Set the device id in `cap2.sh` to match `get_devices`.

```sh
# one Read Cycle burst at 2 MHz, PV valid
./cap2.sh out_pv1 "clk 2000000; burst 200 50" 0.025 125000000
python3 analyse.py out_pv1/digital.csv "PV=1" 370085 1

# same with PV forced to 0 -- required to see the first-bit handover, because
# PV is normally 1 and the line also idles HIGH
printf 'err on\r' > /dev/ttyACM0
./cap2.sh out_pv0 "burst 200 50" 0.025 125000000
python3 analyse.py out_pv0/digital.csv "PV=0" 370085 0
```

- `analyse.py` — clocks per cycle, clock period, Tmu, gap level, Error Flag
  latency, first-bit handover, and payload decode.
- `hand.py` — handover distribution and margin against the half-period budget.
  Use it to compare `-DSIMENC_RAMFUNC=ON/OFF`.
- `ts_acc.py` — Time Stamp accuracy, by regressing the decoded TS field against
  capture time. **Use a long span** (`burst 400 1000` at 25 MS/s): over 13 ms
  the Read Cycle period beats against the 100 us update tick and the answer is
  biased by a factor of two.
- `variants.py` — variant-aware check: clocks per cycle, Tmu, gap level, and a
  payload decode written straight from Product Guide 5.4.2, independent of the
  firmware. It recomputes SSI2's parity and SSI6's CRC-8 itself, so those are
  verified against a second implementation rather than round-tripped.

  ```sh
  ./cap2.sh cap_ssi1 "burst 100 50" 0.025 125000000
  python3 variants.py cap_ssi1/digital.csv ssi1 24 74565
  ```
- `svg2.py` — renders the two-panel 2 MHz figure used in the top-level README.
  Takes the PV=1 capture, the `err on` capture, and the output path.
- `svg500.py` — renders the single-panel 500 kHz figure, with the SSI4 fields
  banded over the bit cells. Takes one capture and the output path, and decodes
  the frame itself, so the caption's numbers come from the capture rather than
  being copied in by hand.

Both draw a faint rule at every clock rising edge. That is deliberate: the
specification puts each data bit on the rising edge, so the grid is what makes
the known half-period deviation visible instead of merely asserted.

`cap2.sh` does not set the position: send `fixed <value>` yourself before
calling it, and pass the same value to the analysis. It used to force
`0x5A5A5`, which silently contradicted whatever the caller had just set.

The measured figures in the top-level README name the commit they were taken
against. Re-run these after changing anything in the SSI path.
