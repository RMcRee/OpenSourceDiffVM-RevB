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
| [MISC-1](#misc-1) | Other capacitor value changes | C16, C18, C54 |

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

## Status notes

- The schematic in `DiffVM/` represents Rev B as **originally designed**.
  Applying the changes above brings the schematic into agreement with the
  current physical board.
- Items DAC-1 and REF-1 are *known design issues* in addition to being
  documentation deltas — they should be redesigned, not merely transcribed,
  at the next board revision.
- Firmware compensations (chop-based settling for DAC-1, `postTestReferences`
  bypass for REF-1) should be removed in lockstep with the hardware fixes.
