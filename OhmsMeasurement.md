# Ohms Measurement Methodology

This document describes the resistance measurement method used in
OpenSourceDiffVM-RevB. The method is **ratiometric, 4-wire-Kelvin, with
polarity-reversed EMF cancellation** against a *manually-wired* reference
resistor. It is not a standard 2-wire / 4-wire ohmmeter and deserves its own
write-up.

For wiring and the API contract, see [`OhmsMeas.h`](OhmsMeas.h). For the
firmware command surface, see the "Ohms Mode — Manual R_ref" section in
[`CLAUDE.md`](CLAUDE.md).

## Topology

```
     V_exc (±1 V or ±2.5 V)
          │
       [SW1] ◄── polarity reversal (TMUX7234, controlled by Teensy)
          │
   ┌──── Vx2 (R_ref top sense, Kelvin)
   │
[R_ref]   ◄── manually wired into a socket; one of r1..r8 (see table below)
   │
   ├──── Vx3 (DUT high sense, Kelvin / R_ref bottom sense)
   │
[R_dut]   ◄── unknown, 4-terminal connection
   │
   ├──── Vx4 (DUT low sense, Kelvin)
   │
        GND (return)
```

All three sense points feed the high-precision front end via the MUX36D08
input mux. The current flowing through the series chain is shared between
R_ref and R_dut, so:

```
                Vx3 − Vx4         V_dut
   R_dut  =  R_ref × ─────────  =  R_ref × ─────
                Vx2 − Vx3         V_ref
```

V_exc, the polarity-switch on-resistance, and any series wiring drops all
cancel out of the ratio — only the calibrated R_ref value and the four
sense voltages enter the result. This is what makes the method robust:
the entire excitation chain can drift and you still get a correct answer.

### EMF cancellation

Thermal EMFs at the Kelvin junctions are cancelled by measuring each sense
voltage at **both excitation polarities** and forming `(V+ − V−)/2`. The
firmware handles this automatically; pass `--no-emf-cancel` to skip it for
speed.

### Signal-chain zero offset

Each sense voltage has the auto-zero offset subtracted
(`OhmsMeasApi::getSignalChainOffset`). The auto-zero tracks DAC zero-code
drift (ε₀) plus the preamp chain residual via a GND-channel measurement at
the DAC null code. Removing ε₀ from each sense before forming the ratio
matters because: even though gain drift (ε₁) already cancels in
V_dut/V_ref, offset drift does not — it creates a differential error
proportional to (V_ref − V_dut)/(V_dut × V_ref). With EMF cancellation
(default) a constant offset cancels anyway; the correction mainly matters
for `--no-emf-cancel` and for keeping the ± polarity readings symmetric for
the mirror-seed bracket math. Run `zero` before precision ohms work so the
offset is current.

### Excitation magnitude

The firmware auto-selects 1 V excitation for R_ref < 500 Ω (just r2 in
the current jig) and 2.5 V for everything else. This keeps R_ref
dissipation reasonable at the low-Ω end without giving up
signal-to-noise on the high-Ω end.

## Reference resistors

Five resistors are physically wired one-at-a-time into the breakout
socket. Each is identified by color band and by short alias. The
firmware tracks which one is currently installed via the `rref` CLI
command (selection is not persisted across reboots).

| Alias | Color  | R_ref (Ω)   | V_exc  |
|-------|--------|-------------|--------|
| r2    | black  | 200.13237   | 1.0 V  |
| r3    | yellow | 2002.03982  | 2.5 V  |
| r8    | green  | 5000.15707  | 2.5 V  |
| r4    | white  | 20000.07073 | 2.5 V  |
| r5    | blue   | 50009.54528 | 2.5 V  |

Values are firmware constants in `RREF_ALIASES[]` (see the .ino source).

**2026-06-22: Full re-anchor** of all five rrefs via Keithley 2002 direct measurement
using Krasimir 1.0620233K cert and Keithley-traceable 5450 decade standards with
temperature correction applied. Formula: `R_ref_new = R_ref_old × (R_true / R_measured)`;
multiple DUT measurements averaged per rref. DUTs: 5450 190Ω/1k/1.9k/10k/19k/100k,
SR1010 1k/100k (all assigned values K2002-traceable).

**Hardware jig revisions (2026-06-05):** the original r1 (20 Ω), r6
(1.9 MΩ), and r7 (11 MΩ) were physically removed — at the
low-Ω/high-current end r1 caused TMUX charge-injection issues, and at
the high-Ω end (r5 = 200 kΩ, r6, r7) BinSrch could not converge,
likely due to AD8428 oscillation at high source impedance. The new r5
is a 50 kΩ blue resistor (was 200 kΩ blue), kept under the practical
threshold where AAF + preamp behave linearly. r8 was re-color-coded
from yellow2 to green.

The r8 5 kΩ value deliberately bridges the decade gap between r3
(2 kΩ) and r4 (20 kΩ), so the 1 kΩ–10 kΩ range is well-served at the
optimum ratio (see below) rather than at a 5× mismatch. The r5 50 kΩ
similarly bridges between r4 (20 kΩ) and the absent 200 kΩ slot.

## Choosing R_ref for a given DUT

### Principle

Let **ρ = R_dut / R_ref**. With per-sense measurement noise σ_v and a fixed
V_exc, the fractional uncertainty in R_dut propagates as:

```
   σ_R       σ_v
   ──── = ────── × √[ (1 + ρ)² · (1 + 1/ρ²) ]
   R_dut   V_exc
```

That bracket has a minimum at **ρ = 1** (where it equals 8) and grows
quadratically as ρ moves away from 1:

| ρ        | bracket | σ_R penalty vs ρ=1 |
|----------|---------|--------------------|
| 1.0      | 8.0     | 1.00×              |
| 0.5 / 2  | 11.25   | 1.19×              |
| 0.1 / 10 | 122     | 3.91×              |
| 0.01/100 | ~10 000 | ~35×               |

The takeaway: **pick R_ref so that R_dut is within ~10× of it.** Within a
factor of 3, you are at near-optimal precision. Outside a factor of 100,
the noise advantage of ratiometric measurement collapses.

### DUT-range guide

| Alias | R_ref      | Sweet R_dut (ρ≈1) | Good range (0.1 ≤ ρ ≤ 10) | Sane usable (0.01 ≤ ρ ≤ 100) |
|-------|------------|-------------------|----------------------------|-------------------------------|
| r2    | 200 Ω      | ~200 Ω            | 20 Ω – 2 kΩ                | 2 Ω – 20 kΩ                   |
| r3    | 2 kΩ       | ~2 kΩ             | 200 Ω – 20 kΩ              | 20 Ω – 200 kΩ                 |
| r8    | 5 kΩ       | ~5 kΩ             | 500 Ω – 50 kΩ              | 50 Ω – 500 kΩ                 |
| r4    | 20 kΩ      | ~20 kΩ            | 2 kΩ – 200 kΩ              | 200 Ω – 2 MΩ                  |
| r5    | 50 kΩ      | ~50 kΩ            | 5 kΩ – 50 kΩ (Z ceiling; see below) | 500 Ω – 50 kΩ         |

The "good" ranges overlap by design. For a 10 kΩ DUT you could pick r3
(ρ=5, ~3× penalty), r8 (ρ=2, ~1.2× penalty), r4 (ρ=0.5, ~1.2× penalty),
or r5 (ρ=0.2, ~1.5× penalty). Pick the one nearest ρ=1 when you have
the choice. **Practical coverage with the current jig is roughly 50 Ω
to 200 kΩ at sub-ppm precision.** Above ~50 kΩ use r4, not r5 — the
front-end source-impedance ceiling caps r5 at DUTs ≤ ~50 kΩ (see
below); r4 reaches MΩ-scale DUTs at reduced precision.

**High source impedance ceiling: Z_src = R_ref ∥ R_dut ≤ ~25 kΩ
(characterized 2026-06-10).** The noise math says r5 should be good
through 500 kΩ DUTs, but the front end has a hard stability ceiling
that the noise math knows nothing about. The AD8428 bank's input
current varies with its differential input voltage; through a high
source impedance at the sense node that variation feeds back into the
input. Loop gain scales with Z_src: harmless below ~20 kΩ, near unity
at ~30 kΩ. Consequences, measured with `dacsweep` at r5 + 100 kΩ
(Z_src ≈ 33 kΩ):

- The static chop-demod transfer compresses ~16–27×: full +FS → −FS
  transition in 1–2 DAC codes instead of the nominal ~15, even with
  100 ms of settle per half-cycle. The both-phase acceptance window is
  essentially empty.
- At normal chop cadence the kicked node locks into a chop-synchronous
  **period-2 limit cycle** (~±1.2 mV at the input, with a ~6-pair beat),
  so readings flip between soft rails of either sign. No search
  strategy converges on that, and an integration through it would be
  meaningless even if one did.
- Control: the identical sweep on a low-Z sense (Vx2) is textbook —
  16-code window, slope exactly gain 2000.

This is the same project-level constraint as voltage mode ("low source
impedance only"), now with a number attached. It is also, in hindsight,
what retired the original 200 kΩ r5 / 1.9 MΩ r6 / 11 MΩ r7 — those
could never have worked.

**Design rule:** keep **R_ref ∥ R_dut ≤ 25 kΩ** at the Vx3 node.
Since R_ref ∥ R_dut < R_ref, any R_ref ≤ 20 kΩ (r2/r3/r8/r4) satisfies
the rule for *every* DUT — for big DUTs, use r4 and accept the ρ
penalty (ρ = 50 ≈ 18× noise vs ρ = 1; still usable). r5 (50 kΩ) is
safe only for DUTs ≤ ~50 kΩ (ρ ≤ 1, Z_src ≤ 25 kΩ), where it gives the
best precision in its band. The firmware warns when
`--nominal` implies Z_src over the ceiling, and explains the ceiling
when a Vx3 search fails at R_ref ≥ 30 kΩ.

**Search improvements that came out of this investigation** (kept —
they benefit all R_refs): `meas r` seeds every BinSrch from physics
(Vx4 ≈ 0 V, Vx2 ≈ ±V_exc, Vx3 from the divider given `--nominal`, the
opposite-polarity code by mirror), bisects inside a small bracket with
flush chop pairs at normal cadence, and resets the AAF through the
preamp's linear region (input mux → GND + autozero-null DAC code) on
cold entry. Cold convergence on the low-Z senses dropped from ~17
probes to 3.

**Diagnostics** (see `help`): `dacsweep <ch> <a> <b> [step] [pairs]
[settleUs]` plots the chop window per DAC code (a == b gives a
time-series at one code); `dacset <code>` parks the DAC for scope work
with `hold`; `ohms pol +|-` / `ohms exc 1|2.5` drive the excitation
manually.

## Practical caveats outside the math

The clean ratiometric formula above hides several real-world effects that
become dominant near the ends of the range.

- **Below ~10 Ω.** Contact resistance and thermal EMF at the Kelvin posts
  start to dominate even with 4-wire sensing. r1 will measure sub-Ω DUTs
  in principle, but expect a µΩ-level noise floor and very tight settling
  requirements. Always EMF-cancel.
- **Above ~10 MΩ.** AD8428 input bias current (~50 nA typ) drops a real
  voltage across both R_ref and R_dut, biasing the result. At 100 MΩ that
  is ~5 mV → ~50 ppm error before any other consideration. Use r7 only
  when you have to, and treat results above 100 MΩ as approximate.
- **Self-heating in R_ref.** Excitation current `I = V_exc / (R_ref + R_dut)`.
  For r1 (20 Ω) at 1 V into a short-DUT condition, P_Rref = 50 mW —
  enough for the R_ref's tempco to walk the value during long
  integrations. Use shorter `--cycles` runs or let the resistor cool
  between measurements when integrating for tens of seconds.
- **Johnson noise in high-R R_ref.** `√(4 k T R · BW)` at R = 10 MΩ and
  BW = 1 Hz is ~13 µV/√Hz. The longer you integrate, the lower the
  effective bandwidth and the lower the noise — but the floor at very
  high R_ref is set by R_ref itself, not by the front end.
- **Preamp input range.** Each sense point (Vx2, Vx3, Vx4) must lie
  within ±5 V of ground for the BinSrch DAC tracker to acquire it. With
  V_exc = ±2.5 V and proper grounding this is automatic; check polarity
  wiring if BinSrch fails on one polarity.

## Verification status

First-light bringup of the OHMS-1 breakout completed **2026-05-30**. The
chain is proven to the following levels:

- **Repeatability (precision)**: sub-ppm within a fixed R_ref. Two
  consecutive measurements at cycles=50 and cycles=100 against a 9.4 kΩ
  DUT with r4 produced ratios identical to 8 decimal places
  (0.46999346). Excitation rail drift of ~1.3 ppm between the runs was
  fully cancelled by the ratiometric measurement; ratio held to ≤ 0.1
  ppm.
- **Cross-reference consistency**: 18 ppm spread between r3 (2 kΩ) and r4
  (20 kΩ) when measuring the same 9.4 kΩ DUT. Attributed to independent
  Keithley-transfer calibration uncertainty on each R_ref value, not to
  any instrument systematic. Each individual measurement was self-consistent
  to sub-ppm.
- **Absolute accuracy**: −20 ppm worst-case error vs a 9400 Ω ±0.005 %
  precision DUT (well inside the resistor's ±50 ppm tolerance window).
- **EMF cancellation**: ~60 µV of thermal asymmetry observed at the DUT
  Kelvin posts (Vx3+/Vx3− midpoint offset) absorbed cleanly by the
  polarity-reversal averaging.
- **Per-sense routing**: Vx2, Vx3, Vx4 confirmed reading the correct
  Kelvin nodes (V_R_dut + V_R_ref closes KVL against V_exc at Vx2 to
  within a few µV; Vx4 sits at ground despite current flow, confirming
  4-wire Kelvin connection integrity).

Practical implication: trust **sub-ppm relative measurements** for
trending (RTS, drift, repeat checks of a single DUT). Treat **absolute
accuracy** as bounded by the R_ref calibration — ~20 ppm with the
current Keithley transfer cal. To tighten absolute accuracy, re-cal the
R_ref bank against a single primary standard and patch `RREF_ALIASES`.

Bringup also surfaced two hardware items, both folded into
[`hardware-errata.md`](hardware-errata.md): the connector ribbon seating
issue (off-by-one row → no damage) and **OHMS-2** (TMUX7234 NO/NC
reversed vs schematic; firmware inverts `PIN_OHMS_POL` sense).

## Rule of thumb

> Pick R_ref ≈ R_dut. Within a factor of 3 you are at near-optimal
> precision; within a factor of 10 you are still excellent. Don't
> overthink it — but keep R_ref ∥ R_dut ≤ 25 kΩ: for DUTs above
> ~50 kΩ that means r4, not r5.

If you genuinely don't know R_dut yet, start with r4 (20 kΩ) at 2.5 V and
look at the V_dut / V_ref ratio in the output. The ratio tells you ρ,
which tells you which R_ref to swap in for the precision measurement.
