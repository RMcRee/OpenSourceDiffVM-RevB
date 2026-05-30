# DiffVM Rev B — Hardware Errata

This document records deviations between the physical assembled Rev B board
and the KiCad schematic / PCB files in `DiffVM/` (`DiffVM_4L.kicad_sch`,
`DiffVM_4L.kicad_pcb`, and per-sheet `FrontEnd.kicad_sch`, `DAC.kicad_sch`,
`VRef.kicad_sch`). Each entry should be folded back into the schematic
before the next board revision is fabricated.

## Index

| ID | Subsystem | Summary |
|----|-----------|---------|
| [DAC-1](#dac-1) | AD5760 output Sallen-Key filter | Resistor values reduced; filter now underdamped |
| [PREAMP-1](#preamp-1) | AD8428 input RFI filter | 33 nF differential + 1 µF Vcm caps added |
| [ADC-1](#adc-1) | ADS127L11 anti-alias filter (C19) | 22 nF → 47 nF |
| [ADC-2](#adc-2) | ADS127L11 VREFP series resistor | 50 Ω added in series |
| [REF-1](#ref-1) | VrefRaw routing (J3 → MUX S3) | Unbuffered output sags under preamp loading; buffer required |
| [IC-1](#ic-1) | U3 op-amp substitution | OPA2140AID → ADA4522-2 |
| [MISC-1](#misc-1) | Other capacitor value changes | C16, C18, C54 |
| [OHMS-1](#ohms-1) | Ohms-measurement breakout (external for now, onboard for next rev) | New external breakout with ±2.5 V excitation, 2× MUX36S08 R_ref bank, TMUX7234 polarity switch, 4-wire Kelvin DUT header |
| [OHMS-2](#ohms-2) | TMUX7234 polarity switch NO/NC swap | As-built SW1 has NO/NC reversed vs schematic; firmware inverts PIN_OHMS_POL sense |

---

## Errata

### DAC-1 — AD5760 output filter resistor values  <a id="dac-1"></a>

**Location:** Sallen-Key low-pass filter on the AD5760 DAC output, upstream of
the TMUX7234 input mux (see `DAC.kicad_sch`).

| | Schematic | Actual on board |
|---|---|---|
| R (series) | 9.01 kΩ | **1.68 kΩ** |
| R (Sallen-Key feedback) | 18.0 kΩ | **3.17 kΩ** |

**Status:** The reduced resistor values changed the filter Q. Empirical
step-response measurement (augmented `postTestReferences()`, 0 → +FS step,
30 s probe window) shows the filter is now **underdamped** with ringing at
roughly 10 s period; 10 µV settling against rail-to-rail step takes ~30 s
rather than the ~75 ms expected from the original ~25 ms τ.

**Firmware impact:** `binarySearchDAC()` no longer relies on a hardcoded
RC settle time — it chops while searching so the slow ringing averages out.
`DAC_SETTLE_MIN_MS = 80` / `DAC_SETTLE_MAX_MS = 300` in `DacDriver::
calculateSettleTime()` are wildly under-budget for true 1 µV settling but
are tolerated because measurements are themselves chopped and integrated.

**Recommended fix:** Re-design the filter Q at the next board revision —
the present values are believed to be an unintended rework artifact rather
than a deliberate design choice. Until then, leave the firmware
chop-based mitigation in place.

---

### PREAMP-1 — AD8428 input RFI filter additions  <a id="preamp-1"></a>

**Location:** Differential input pair of the AD8428 instrumentation amplifier,
on the preamp side of the existing 22 Ω series resistors at each input leg
(see `FrontEnd.kicad_sch`).

**Not present on schematic. Added on the board:**

| Component | Value | Function |
|---|---|---|
| Differential cap (no designator on schematic) | **33 nF** | RC differential-mode filter with the 22 Ω series resistors |
| C15 (common-mode Vcm cap) | **1 µF** | Common-mode RC filter to ground |

**Rationale:** RFI insurance. A previously-investigated 12 ms preamp swell
on the input traced to an external source (Fluke calibrator), but the
differential and Vcm caps were retained as belt-and-braces against future
RF coupling on the inputs.

**Firmware impact:** The added capacitance contributes to the post-chop-edge
settle window. `CHOP_SETTLE_US = 10000` (10 ms) in
`OpenSourceDiffVM-RevB.ino` accommodates this — see the inline comment
"`post-chop-edge settle: preamp + TMUX + AAF (τ≈56ms after 47nF cap, was 21nF)`".

---

### ADC-1 — ADS127L11 anti-alias filter cap (C19)  <a id="adc-1"></a>

**Location:** Differential anti-alias filter at the ADS127L11 input pair
(see `DAC.kicad_sch`).

| | Schematic | Actual on board |
|---|---|---|
| C19 | 22 nF | **47 nF** |

**Firmware impact:** Increases the AAF time constant. Tracked by the
comment near `CHOP_SETTLE_US` in the firmware: `τ ≈ 56 ms after 47 nF cap,
was 21 nF`. The 10 ms `CHOP_SETTLE_US` allows the AAF to settle within
each chop half-cycle.

**Status:** This is the current ADEV baseline cap value (see project memory
`project_adev_baseline_47nf.md` — 46 nV @ τ=15 s, OSR=12800, samples=14
achieved with C19 = 47 nF).

---

### ADC-2 — ADS127L11 VREFP series resistor  <a id="adc-2"></a>

**Location:** Between the 2.5 V reference output and the ADS127L11 VREFP pin
(see `DAC.kicad_sch`).

| | Schematic | Actual on board |
|---|---|---|
| Series R on VREFP | direct connection | **50 Ω** |

**Rationale:** The ADS127L11 internal reference buffer (REFP_BUF, CONFIG1
bit 3, enabled in firmware via `ADC_REFBUF_ENABLE = true`) sinks dynamic
current spikes when sampling. The series resistor decouples those spikes
from the upstream reference network so they don't perturb the filtered
ADR1001 average that drives the DAC reference.

**Firmware impact:** None directly — the REFP_BUF bit must remain enabled
to compensate for the source impedance the resistor adds.

---

### REF-1 — VrefRaw is not directly usable as a reference  <a id="ref-1"></a>

**Location:** J3 (VrefRaw, the unbuffered three-ADR1001 average upstream of
the Sallen-Key reference filter) is routed to the input multiplexer S3
channel for runtime reference-drift monitoring (see `VRef.kicad_sch` and
`FrontEnd.kicad_sch`).

**Issue:** The MUX S3 path loads VrefRaw with the preamp/mux input
impedance during measurement. The loading is enough to sag VrefRaw out of
spec — making the runtime drift measurement unreliable.

**Workaround in firmware:** `postTestReferences()` is currently bypassed in
POST. The DAC reference drift compensation system still works for the
internal path (the DAC sees TP1, the filter *output*, not VrefRaw), but
the live runtime drift measurement is disabled until a hardware fix
is in place.

**Hardware fix required:** Add a buffer (e.g. another OPA828 in voltage-
follower configuration) between J3 and the MUX S3 input. After the buffer
is installed, re-enable `postTestReferences()` and the runtime
`measureFilterError()` machinery.

---

### IC-1 — U3 op-amp substitution  <a id="ic-1"></a>

**Location:** U3 on `FrontEnd.kicad_sch` (SOIC-8 dual op-amp footprint).

| | Schematic | Actual on board |
|---|---|---|
| U3 part number | OPA2140AID | **ADA4522-2** |

**Rationale:** The ADA4522-2 is a zero-drift (chopper-stabilized) dual
precision op-amp with substantially lower offset and 1/f noise than the
OPA2140AID JFET op-amp originally specified:

| Parameter | OPA2140AID (schematic) | ADA4522-2 (actual) |
|---|---|---|
| Input offset voltage (typ) | 120 µV | **0.4 µV** |
| Offset drift (typ) | 0.35 µV/°C | **0.022 µV/°C** |
| Input voltage noise (0.1–10 Hz, p-p) | 0.25 µV | **0.117 µV** |
| Input bias current (typ) | 0.5 pA | 50 pA |
| GBW | 11 MHz | 2.7 MHz |

The substitution trades bandwidth and input bias current for ~300× better
input offset and ~16× better drift. Pin-compatible SOIC-8 dual; same
supply rails. The bias current is higher (CMOS chopper inputs vs JFET),
so the substitution is most appropriate where source impedance is moderate
and DC accuracy dominates over BW/Iib budget.

**Firmware impact:** None directly.

---

### MISC-1 — Other capacitor value changes  <a id="misc-1"></a>

These capacitor values were changed during preamp-chain tuning and
anti-alias adjustments. Their subsystem context should be confirmed
against the schematic before merging.

| Designator | Schematic | Actual on board | Subsystem (TBD — confirm) |
|---|---|---|---|
| C16 | 22 nF | **55 nF** | — |
| C18 | 22 nF | **55 nF** | — |
| C54 | 22 nF | **97 nF** | — |

C16 and C18 were changed together (same value, paired change). C54 was
changed independently.

---

### OHMS-1 — Ohms-measurement breakout (external; integrate on next rev)  <a id="ohms-1"></a>

**Location:** New off-board breakout PCB; connects to DiffVM via a
10-conductor ribbon. See `docs/ohms-breakout.svg` for the full
schematic.

**Function:** Adds resistance measurement to the instrument using the
existing chop loop, preamp, DAC, and the unused Vx3 / Vx4 / Vx5 input-
MUX channels. Topology is 3-Kelvin-sense ratiometric with ±2.5 V
polarity-reversed excitation:

```
R_dut = R_ref × (Vx3 − Vx4) / (Vx5 − Vx3)
```

V_exc and the R_on of the polarity switch and Force MUX cancel out of
the ratio — the only cal'd quantities are the R_ref bank values.

**Breakout content (this revision, external):**

| Component | Role |
|---|---|
| ADA4522-2 dual op-amp | Half A = +2.5 V follower; Half B = G=−1 inverter for −2.5 V |
| Vishay VHP100 10 kΩ matched pair | Inverter gain-set, ≤2 ppm tracking |
| TMUX7234 SPDT | Polarity switch (same part as the input-chop TMUXes — already qualified) |
| 2× MUX36S08 | Force MUX + Sense MUX, both driven by the same 3 address-bit lines |
| 7× precision R_ref (20 Ω, 200 Ω, 2 kΩ, 20 kΩ, 200 kΩ, 2 MΩ, 20 MΩ) | Vishay Z-foil / Caddock USR; matches Keithley 2002 ranges for cross-validation |
| 4-terminal Kelvin binding posts | DUT under test |

**Firmware impact:** Adds new module `OhmsMeas.{h,cpp}`. New Teensy
GPIOs:

| Function | Teensy pin |
|---|---|
| Excitation polarity (SW1.IN) | 5 |
| R_ref MUX A0 (U2 & U3) | 6 |
| R_ref MUX A1 (U2 & U3) | 7 |
| R_ref MUX A2 (U2 & U3) | 8 |

New CLI: `meas r <bank_idx>`, `cal r rref`, `cal r show`.
CalibrationStore gains an `OhmsCalBlock` (version-bumped).

**Required for next PCB revision:**

1. Move the ±2.5 V generator onboard, using the spare half of the
   board's U3 (ADA4522-2 per [IC-1](#ic-1) substitution) as the
   G=−1 inverter. Add a matched-R pair (Vishay VHP100 footprint).
2. Onboard 2× MUX36S08 R_ref bank (or pull the bank to onboard with
   a smaller MUX36D04 if only 4 ranges are wanted).
3. Onboard TMUX7234 SPDT for polarity selection.
4. Dedicated 4-terminal Kelvin DUT header with separate force/sense
   pads — keep sense traces away from the force-path copper.
5. Provide Vx5 routing from the MUX36D08 S8 channel to the Sense MUX
   output.
6. Reserve Teensy pins 5/6/7/8 (or equivalent free GPIO) for the
   ohms control bus.

Until the next rev: the external breakout is documented here as the
authoritative build reference. Schematic source: `docs/ohms-breakout.svg`.

**Verification status (2026-05-30):**

First-light bringup completed. All five phases (excitation rails, polarity
switch, V_exc rail select, end-to-end against a known DUT, quantitative
accuracy) passed.

| Phase | Result |
|-------|--------|
| 1 Rails | 1.0 V and 2.5 V rails within spec; unloaded |
| 2 Polarity | TMUX flips cleanly; NO/NC swap discovered (see [OHMS-2](#ohms-2)) |
| 3 V_exc select | 1.0 V ↔ 2.5 V rail switch works |
| 4 / 5 Measurement | 9400 Ω ±0.005 % DUT measured against r4 = −20 ppm; against r3 = −1.8 ppm. 18 ppm R_ref-to-R_ref spread attributed to independent transfer-cal uncertainty against the Keithley reference. Sub-ppm repeatability within a fixed R_ref. EMF cancellation absorbs ~60 µV of thermal asymmetry on DUT Kelvin posts. Ratiometric rejection holds the ratio to 0.1 ppm against ~1.3 ppm V_exc drift between runs. |

Caveats discovered during bringup:
- Cable seating: an off-by-one-row Teensy↔breakout ribbon misalignment shorted
  every signal to GND. POST + adev after re-seating showed no chip damage.
- TMUX polarity: see [OHMS-2](#ohms-2).

---

### OHMS-2 — TMUX7234 polarity switch NO/NC reversed  <a id="ohms-2"></a>

**Location:** SW1 (TMUX7234) on the OHMS-1 breakout, controlled by
`PIN_OHMS_POL` (Teensy pin 5). See `docs/ohms-breakout.svg` and
[OHMS-1](#ohms-1).

**Symptom (discovered during phase-1 bringup, 2026-05-30):**
With `PIN_OHMS_POL = HIGH` the SW1 common pin sources −V_exc rather than
the schematic-intended +V_exc; with `PIN_OHMS_POL = LOW` it sources
+V_exc. The NO and NC pins of the as-built SW1 are reversed relative to
the schematic.

**Status:** Accommodated in firmware rather than reworked on the
breakout. Convention:

| `PIN_OHMS_POL` | SW1 common output |
|----------------|-------------------|
| **LOW**        | **+V_exc**        |
| HIGH           | −V_exc            |

Inverted in:
- `PIN_OHMS_POL` pin comment in `OpenSourceDiffVM-RevB.ino`
- `ohmsAdapter_setPolarity()` (writes `LOW` for `pos=true`)
- Setup default (`digitalWrite(PIN_OHMS_POL, LOW)` boots to +V_exc)
- `ohms pol +|−` CLI in `cmdOhms()`

EMF cancellation in `OhmsMeas::cmdMeasR` is sign-agnostic, so the
measurement math is unaffected — only the human-facing labels needed
correcting.

**Fix for next rev:** correct the NO/NC tie on the breakout silkscreen
and re-pin, then revert the firmware inversions.

---

## Status notes

- The schematic in `DiffVM/` represents Rev B as **originally designed**.
  Applying the changes above brings the schematic into agreement with the
  current physical board.
- Items DAC-1 and REF-1 are *known design issues* in addition to being
  documentation deltas — they should be redesigned, not merely transcribed,
  at the next board revision.
- Firmware compensations (chop-based settling for DAC-1, `postTestReferences`
  bypass for REF-1) should be removed in lockstep with the hardware fixes.
