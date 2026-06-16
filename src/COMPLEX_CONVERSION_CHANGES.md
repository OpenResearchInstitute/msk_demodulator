# Complex-Baseband Conversion — Change Manifest

*"Cast TRUE SIGHT, then SCATTER the ghosts." — pluto_msk goes complex throughout.*

Open Research Institute · Haifuraiya / pluto_msk · MDT receive chain
Assembled for the end-to-end loopback bring-up. Everything here is receive-side
+ stimulus; the modulator RTL does not change (only its freq-word *values*).

---

## 0. One-paragraph why

The demod core was correctly converted to complex (4-mult mixer, both Costas
loops fed I and Q). But three things downstream were left in a half-state: the
lock detector's scaling was refactored by guesswork (overflows on strong
signals, and it never grew the hysteresis it needs), `msk_top` still fed the
demod through the old real-only `rx_samples` port (Q dead-ended at a debug
capture), and the testbenches still placed the tones at an **IF** (the old
real-projection workaround) instead of at complex baseband. This manifest fixes
all three. After this, a fresh run is complex end-to-end with no real-mode
ghosts in the datapath, the register defaults, or the stimulus.

---

## 1. Files changed

| # | File | Repo / path | Change | Suggested spell |
|---|------|-------------|--------|-----------------|
| 1 | `costas_lock_detect.vhd` | `msk_demodulator/src/` | Overflow-proof wide accumulator (no `>>3` band-aid); normalized delta; **hysteresis + K-of-N dwell**; lock decision once per window | Cast STEADFAST |
| 2 | `costas_loop.vhd` | `msk_demodulator/src/` | Threshold feed `symbol_lock_threshold & x"0000"` → `x"0000" & symbol_lock_threshold` (zero-extend, not <<16) | Cast TRUE STRIKE |
| 3 | `msk_top.vhd` | `pluto_msk/src/` | Wire the Q path: `rx_samples_q_mux` + `rx_samples_q_dec`; demod `rx_samples =>` → `rx_i_samples =>` + `rx_q_samples =>`; `SAMPLE_GATED_NCO => False` (Pluto: clk == fs) | Cast TRUE SIGHT |
| 4 | `tb_msk_modem_134byte.vhd` | `pluto_msk/sim/` | RX/TX F1/F2 freq words IF → ±baud/4 about DC | Cast BANISH IF |
| 5 | `msk_test.py` | `pluto_msk/sim/` | `freq_if_mult` 32→0, `rx_offset` 100→0 (cocotb path) | Cast BANISH IF |

Already proven by ghdl sim last session: the fixed detector holds lock on a
strong signal (no overflow) and rides through near-threshold dips that make the
old bare-compare chatter. See `sim/run.sh`.

---

## 2. The freq-word numbers (the IF ghost)

Convention (from the register doc): `FW = Fn * 2^32 / Fs`, `Fs = 61.44e6`.
Complex baseband: tones at ±baud/4 about DC, baud = 54200, so ±13550 Hz.

- f1 = −baud/4 = −13550 Hz → `0xFFF18B8E`  (NCO-wrapped negative)
- f2 = +baud/4 = +13550 Hz → `0x000E7472`

TX and RX both move to these (loopback through a perfect channel, so they
match). f1 < f2 ordering preserved, which keeps the dclk/cclk discriminant
signs the same as the validated config.

**Verify the NCO accepts the negative (wrapped) word for f1** — this is the open
risk #2 from the demod plan. The phase accumulator should wrap correctly
(0xFFF18B8E ≈ −0.000221 of full circle per sample = −13550 Hz), but confirm in
the waveform that the f1 loop tunes downward, not to ~+61 MHz garbage.

---

## 3. Symbol-lock threshold — calibrate, don't guess

The TB currently writes `SYMBOL_LOCK_CTRL = 0x001F9410` (threshold=2021,
count=16). That 2021 was meaningful against the OLD `<<16` detector; with the
fixed detector + zero-extend its scale is different and depends on the loopback
signal level, which we can only know by measuring.

**Bring-up order (de-risks the first run):**
1. First run: set threshold LOW so lock is guaranteed and you can validate the
   datapath + frame decode. Write `SYMBOL_LOCK_CTRL = 0x00040010`
   (threshold=256, count=16).
2. With lock asserted and steady, read `cst_acc_iq_delta_f1` (the ILA / debug
   port now carries the *normalized* delta) in the waveform.
3. Set the final threshold to ~70% of that observed locked delta, write it back
   into `SYMBOL_LOCK_CTRL` bits [25:10], rerun.

If `cst_acc_iq_delta_f1` reads tiny (< ~1000), lower `NORM_SH` in
`costas_lock_detect` (coarse knob, default 16; try 13). If it pegs the 16-bit
threshold field, raise `NORM_SH`. `HYST` (default 500) and `DWELL` (default 2)
are the chatter-immunity knobs — widen `HYST` if lock still flickers, raise
`DWELL` to require more consecutive good/bad windows.

---

## 4. Diagnostic ladder for the run (if it doesn't decode)

1. **Do the carrier loops lock at all?** Watch `cst_lock_f1` / `cst_lock_f2`.
   - Never assert → freq words wrong (wrong place) OR threshold too high. Drop
     threshold to 0x100 and recheck; confirm f1/f2 tune toward ∓13.55 kHz.
   - Assert then chatter → threshold/hysteresis; calibrate per §3.
2. **Carrier locked, frame sync never fires?** This is the §8.1 polarity story
   from the demod plan: the soft path may need `RX_INVERT`. The TB has
   `rx_invert`; flip it and watch `corr_peak`.
3. **Frame sync locks, frames wrong?** Soft-bit scaling — the demod's
   `data_out` shift changed (`>>5` → `>>1`); confirm soft bits still fit
   `signed(15:0)` and aren't saturating into `opv-decode`.
4. **Won't elaborate?** The complex `msk_demodulator` submodule has no
   `rx_samples` port — make sure the submodule pointer is on `complex-baseband-rx`
   and `msk_top.vhd` is the patched copy (#3).

---

## 5. The "complex throughout" decisions (architecture, not just bring-up)

- **Complex is a superset of real.** The 4-mult mixer with Q tied to 0 is
  bit-exact to the old real mixer, so committing to complex-only loses no
  capability. Pluto's AD9363, the channelizer, and the modulator all produce
  I/Q; there is no real-only source in the system. The real-input path was a
  scar from the old mixer, not a feature.
- **Submodule API:** the `msk_demodulator` submodule has already committed
  (dropped `rx_samples`). **Before tagging it complex-only, check every other
  consumer** (libreSDR, etc.) — the demod plan §1 deliberately left them
  untouched. If a legacy consumer truly has only a real ADC, give it a thin edge
  wrapper that ties Q=0, not a mode switch inside the core.
- **Sequencing for Friedrichshafen:** the architecture commitment is now, but
  time the *deletion* of the real path to AFTER the complex path is
  silicon-green (Phase 6). Keep the fallback on a git tag, not a live
  `if real_mode` branch.
- **Register defaults:** RX/TX F1/F2 FreqWord reset to `0x0` (set by software),
  so no IF ghost lives in the register *defaults* — but make sure the
  production driver/software writes the ±baud/4 words, not the old IF words.

---

## 6. Verify-then-trust checklist before committing

- [ ] `msk_top.vhd` elaborates against the complex submodule (no `rx_samples`
      port error).
- [ ] f1 loop tunes to −13.55 kHz, f2 to +13.55 kHz (negative-word NCO wrap OK).
- [ ] First run at threshold=256 → `cst_lock_f1/f2` assert and hold; 10 frames
      decode and match input.
- [ ] Read `cst_acc_iq_delta_f1`; set threshold to ~70%; rerun, lock still holds
      (no chatter through the 10-frame burst).
- [ ] Soft bits within `signed(15:0)`, no saturation.
- [ ] Bug Hunt Trophy Case updated; Version Stack pinned.

---

## 7. Trophy Case (new entries)

### 7.1 The strong-signal unlock (accumulator overflow)
- Symptom: lock dropped on the *strongest* signals, held on moderate ones.
- Root cause: lock-detector `acc_i`/`acc_q` were 32-bit; the `>>3` "overflow
  guard" was sized by guess and still overflowed at full scale over the
  integration window. `acc_i` wrapped negative → `acc_i − acc_q` went negative →
  unlock. Louder = overflows sooner = the counterintuitive signature.
- Proof: ghdl TB, I=30000 → branch `cst_lock=0`, delta = −1,573,309,592.
- Fix: wide `SUM_W` accumulator with a real headroom budget; `>>3` removed.
- Lesson: a magic shift to "prevent overflow" without a fixed-point budget is a
  guess wearing a fix's clothes. Write the budget down.

### 7.2 The missing hysteresis
- Symptom: `cst_lock` chatters on real signals near threshold; frame sync can't
  hold.
- Root cause: lock was a bare per-clock threshold compare — a smoke alarm with
  no minimum dwell, re-deciding every cycle.
- Fix: decide once per integration window; hysteresis (lock-high / unlock-low)
  + K-of-N dwell.

### 7.3 The half-converted msk_top
- Symptom: complex demod, but Q never reaches it; against the complex submodule,
  `msk_top` won't even elaborate.
- Root cause: conversion done at modulator + TB + demod core, but `msk_top`
  still fed the old real `rx_samples` port; `rx_samples_Q` dead-ended at a debug
  capture. The new lock/dbg outputs *were* wired — the cosmetic half was done,
  the load-bearing half wasn't.
- Fix: parallel Q mux + decimation, demod rewired to `rx_i_samples`/`rx_q_samples`.

### 7.4 The IF ghost in the stimulus
- Symptom: would never lock at complex baseband even with the RTL fixed.
- Root cause: TB hardcoded RX/TX freq words at ~420/447 kHz (IF = baud/4 × 32),
  the old real-projection workaround; cocotb had `freq_if_mult=32`, `rx_offset=100`.
- Fix: tones to ±baud/4 about DC (TB hex + cocotb knobs).
- Lesson: "complex throughout" means the stimulus and the register intent too,
  not just the datapath.
