# Complex-Baseband MSK Demodulation — Plan of Attack

*"Cast TRUE SIGHT" — the demod stops squinting through one eye.*

Open Research Institute · Haifuraiya · MDT receive chain
Re-entry anchor. Start each session here. Updated end of every session.

---

## 0. The one-paragraph why

The channelizer already hands every channel out at **complex baseband**. The
current demod throws Q on the floor, keeps only I (`Re{}`), and then has to
manufacture an IF offset so the two MSK tones don't fold onto each other — and
that IF is exactly what shoved our signal into the channel wall and cost us the
~7.5% edge abrasion. Taking `Re{}` also folds the negative-frequency **noise**
onto the signal band: the classic real-projection penalty, ~3 dB of link
margin we are currently throwing away. A complex-baseband demod eats the
channelizer's I/Q directly: signal sits dead-center at true baseband (no
abrasion, no IF, no post-NCO), and we get the ~3 dB back. On a transponder,
3 dB is enormous. This is the architecturally-coherent receiver, not a polish.

**This is receive-side only. The modulator does not change.**

---

## 1. Starting point & config discipline

- **Fork point:** `msk_demodulator` branch `tx_sample_scale` (most recent active
  dev) **+ the `SAMPLE_GATED_NCO` edits** already on disk. Confirmed: your
  running copy = `tx_sample_scale` + gate generic/wiring only; lock detector is
  stock `tx_sample_scale`.
- **New branch:** `complex-baseband-rx` off that state. Keeps libreSDR and every
  other real-input consumer of `msk_demodulator` untouched.
- **Pin it:** record the `tx_sample_scale` base SHA in the Version Stack (§9)
  before the first edit, so the diff is always legible.
- Safe-commit pattern as always: explicit `git add` paths → `git diff --cached
  --stat` → `git commit -F /tmp/msg.txt` heredoc. No `-A`, no `--`.

---

## 2. Domain model (write the math down before touching VHDL)

### 2.1 The mixer — the ONE thing that changes

Current `costas_loop` front-end, real input `x_r`, carrier NCO `(car_cos, car_sin)`:

```
cos_samples (I) = x_r * car_cos        -- 2 real multiplies
sin_samples (Q) = x_r * car_sin
```

Complex input `x = x_i + j*x_q`, downconvert by `e^{-jθ} = car_cos - j*car_sin`:

```
x * e^{-jθ} = (x_i + j x_q)(car_cos - j car_sin)
  cos_samples (I) = x_i*car_cos + x_q*car_sin    -- 4 real multiplies + 2 adds
  sin_samples (Q) = x_q*car_cos - x_i*car_sin
```

That's the whole structural change: **2 multiplies become a 4-multiply complex
mixer, and the input gains a Q port.** Everything downstream of `cos_samples` /
`sin_samples` is unchanged.

### 2.2 Tone placement

At complex baseband, centered, the two MSK tones sit at **±baud/4 = ±13.55 kHz**
about DC (not IF±13.55). So:
- `freq_word_f1` → +13.55 kHz
- `freq_word_f2` → −13.55 kHz  (signed / NCO-wrap: `2^NCO_W − inc(13.55k)`)

Confirm the NCO accepts a "negative" (wrapped) freq word for f2 — likely yes
(phase accumulator wraps), but it's an explicit check, not an assumption.

### 2.3 Discriminants — the thing to VALIDATE, not assume

The symbol/carrier discriminants are cross-products between the two loops:

```
dclk = cos_f1*sin_f2 - cos_f2*sin_f1     (symbol clock)
cclk = cos_f1*cos_f2 + sin_f1*sin_f2     (carrier clock)
```

These were designed living **with** the real-input image. Complex input removes
the image, which should make them *cleaner*, but the signs/scaling may shift
(especially with f2 now at a negative frequency). The formulas likely carry
over; **whether they carry over is a Phase-3 simulation result, not a claim.**

---

## 3. Inventory — KEEP vs CHANGE (we are not building from scratch)

### KEEP (reuse verbatim)
- Two `costas_loop` instances (the dual-tone structure).
- Carrier NCO per loop (`PHASE_INIT`, freq-word mechanism, `SAMPLE_GATED_NCO`).
- Loop filter — proportional-integral (`lpf_p/i_gain/shift`, `alpha`) and every
  gain we tuned (LPF_I_SHIFT=29, LPF_P_SHIFT=20, gains 0x7FFFFF).
- `costas_lock_detect` (improve it in Phase 4, but reuse the engine).
- `dclk`/`cclk` cross-product structure.
- Soft-decision output path (`rx_data_soft`, `rx_dvalid`).
- `frame_sync_detector_soft` downstream — **untouched**.

### CHANGE
- `costas_loop`: add `q_samples` input port; real mixer → 4-mult complex mixer.
- `msk_demodulator`: `rx_samples` → `rx_i_samples` + `rx_q_samples`; wire both
  to both loops; freq words → ±baud/4 about DC.
- `haifuraiya_rx_top`: feed `chan_i` **and** `chan_q` to the demod; delete the
  planned post-NCO/IF entirely; place the signal centered at DC.
- `opv_chan_stim_gen.py`: centroid → 0 Hz (centered in channel 0) instead of 40 kHz.

### CHANGE (opportunistic — fixes the lock chatter we already diagnosed)
- `costas_lock_detect`: add hysteresis (lock-high / unlock-low) or a K-of-N
  dwell so `cst_lock` stops toggling at the threshold. Frame sync needs a lock
  bit that holds.

---

## 4. Phased plan — 10 days (10 Jun → ~20 Jun, Friedrichshafen)

> Gate philosophy: **simulate before synthesize.** No XSA build until the
> isolated demod decodes in sim (Phase 3). Hard go/no-go at Phase 3 protects the
> demo (see §6).

| Phase | Days | "Spell" | Deliverable |
|---|---|---|---|
| 0 — Domain model & branch | 0–1 | Cast COMMUNE | Math in §2 confirmed on paper; `complex-baseband-rx` branched; centered DC complex MSK stimulus generated |
| 1 — Complex `costas_loop` | 2–3 | Cast TRUE SIGHT | Q port + 4-mult mixer; analyzes clean (ghdl/nvc) |
| 2 — Complex `msk_demodulator` top | 3–4 | Cast TRUE SIGHT | i/q ports, both loops fed, freq words ±baud/4 |
| 3 — **Isolated demod sim (THE GATE)** | 4–6 | Cast IDENTIFY | Complex baseband MSK in → lock + soft bits → `opv-decode -3` clean, discriminants/signs validated |
| 4 — Lock hysteresis | 6–7 | Cast STEADFAST | `cst_lock` holds through data; no chatter |
| 5 — Integrate rx_top + channelizer TB | 7–8 | Cast MEND | `tb_haifuraiya_channelizer_axi` end-to-end, centered DC stim, `opv-decode -3` clean through channelizer |
| 6 — Hardware bring-up | 8–9 | Cast MANIFEST | XSA build, timing closed (re-verify WNS for wider mixer), ZCU102, ILA shows lock holding on silicon |
| Buffer / contingency | 9–10 | — | Margin, docs, trophy case, version stack pinned |

---

## 5. Validation strategy (measurement is authority)

- **Phase 3 gate (isolated):** drive the complex demod with centered complex
  baseband MSK (from `opv_chan_stim_gen.py` at fc=0, or the modulator output).
  Success = soft bits that `opv-decode -3` accepts, with the carrier loops
  locked and steady (watch the I/Q energy ratio — want `acc_i >> acc_q`, not the
  2:1 we saw in the real-input run).
- **Phase 5 gate (end-to-end):** same, but through the channelizer testbench,
  channel 0, no twiddle needed (DC bin). Envelope should be the prototype floor
  (~0.5%), not 7.5% — centering removed the abrasion.
- **Phase 6 (silicon):** ILA on `cst_lock_f1/f2`, `frame_sync_locked`,
  `frames_received`. Lock asserts and *stays*; frames increment.
- Strict estimate-vs-measurement separation. Bit-exact unit checks where
  possible (we have ghdl in the loop now — proven on the channelizer fix).

---

## 6. Demo safety / Plan B (Friedrichshafen is real)

The complex demod is the *right* architecture, but the booth must not depend on
a validate-before-trust change landing.

- **Go/no-go at end of Phase 3 (~Day 6):** if the isolated complex demod
  decodes, full speed to integration. If it's fighting the discriminants,
  **fall back** to the proven path for the demo and continue complex as the
  post-demo upgrade.
- **Plan B (demo-safe):** real-input demod (unchanged) + signal **DC-centered in
  channel 0** + a single small post-NCO in rx_top to lift to ~25 kHz. Gets a
  clean centered decode with zero demod risk. (Or the even cheaper interim: trim
  the in-channel IF 40→25 kHz, stimulus-only, ripple 7.5%→~2–3%.)
- Keep Plan B's pieces alive (don't delete the rx_top real-input path) until the
  complex path clears Phase 5.

---

## 7. Open questions / risks (resolve, don't hand-wave)

1. Do `dclk`/`cclk` signs survive the complex/negative-f2 change? → Phase 3 sim.
2. NCO negative freq word for f2 (−13.55 kHz) — confirm wrap behavior.
3. Timing: +2 multiplies per mixer per loop (~4 extra DSP per demod). Trivial on
   resource (channelizer uses 1536 of 2520 DSP); re-verify **WNS** after the
   wider mixer, don't assume.
4. Lock-detector overflow guard (`>>3`) was sized for the real-input scale;
   recheck `acc_i` headroom at the new (full-energy) signal level.
5. Soft-metric scaling into `opv-decode -3` may shift with the ~3 dB more energy
   — confirm the soft-bit dynamic range still fits `signed(15:0)`.

---

## 8. Bug Hunt Trophy Case

*(fill as we go — bug, root cause, fix, how caught)*

- _(none yet — branch not cut)_

---

## 9. Version Stack (pin everything)

| Component | Version / SHA | Notes |
|---|---|---|
| `msk_demodulator` base | `tx_sample_scale` @ `<SHA TBD>` | fork point for complex-baseband-rx |
| local delta on base | SAMPLE_GATED_NCO edits | carry onto the branch |
| Vivado / PetaLinux | 2022.2 | unchanged |
| channelizer | backward-commutator fix ("Cast TRUE SEEING") | bin 0 correct; non-zero bins need twiddle (separate) |
| stimulus gen | `opv_chan_stim_gen.py` @ fc=0 (centered) | regenerate for complex path |

---

## 10. Re-entry checklist (start of each session)

1. `git status` / `git log --oneline -5` on `complex-baseband-rx`.
2. Which phase are we in (§4)? What was the last green sim?
3. Re-read §2 math before editing the mixer.
4. Anything new in the Trophy Case (§8)?
5. Has the Phase-3 go/no-go been hit? If past Day 6 and not green, re-read §6.
