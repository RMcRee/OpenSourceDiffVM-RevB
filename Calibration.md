# DiffVM Rev B — Calibration Checklist

## Measurement Equation

Every voltage reading is computed as:

```
Vx = [V_dac(code) + preamp_delta] × divider_ratio × refCorrection − autoZeroOffset
```

Where:
```
V_dac(code)    = dacCalTable[code]  (calibrated) or code × DAC_LSB_V  (nominal)
preamp_delta   = ADC_mean × ADC_LSB_V / PREAMP_GAIN
ADC_LSB_V      = (2 × ADC_VREF) / 2^24  ≈ 298.0 nV/count
refCorrection  = 1 + filterErr / NOMINAL_REF_V   (reference drift compensation)
```

**Key insight:** `V_dac` is the dominant term (nearly equals Vx); `preamp_delta` is a small
residual correction (≤ ~1 DAC LSB = 152.6 µV in steady state). Calibration requirements
are therefore very different for these two terms.

---

## Sensitivity Analysis

Errors in parameters that scale `V_dac` propagate at **full signal amplitude**.
Errors in parameters that scale only `preamp_delta` propagate at **reduced amplitude**
(≤ 152.6 µV / Vx of the full-range signal).

| Parameter | Affects | Sensitivity (nV per 1 ppm error) | Expected uncertainty | Est. error at Vx = 1 V |
|---|---|---|---|---|
| ADR1001 reference (5 V) | `V_dac` absolute scale | 1000 nV/ppm | ±200 ppm initial; ±0.05 ppm/°C | **±200 µV abs** / ±50 pV/°C |
| DAC INL (uncalibrated) | `V_dac` nonlinearity | up to ±10 LSB = ±1.526 mV | ±10 LSB typ (AD5760) | **±1.526 mV** |
| DAC INL (with cal table) | `V_dac` residual | noise-limited | few nV averaged | ±few nV |
| HV divider ratio (÷10, 10 V in) | `Vx` output scale | 10,000 nV/ppm | component tolerances, need measurement | TBD |
| HV divider ratio (÷108.76, 100 V in) | `Vx` output scale | 100,000 nV/ppm | nominal from Caddock specs, unverified | TBD |
| HV divider ratio (÷984.65, 1 kV in) | `Vx` output scale | 1,000,000 nV/ppm | nominal, unverified | TBD |
| Zero offset (no auto-zero) | additive offset | N/A (additive) | ±10 – 100 µV from preamp + mux offsets | **±10 – 100 µV** |
| Zero offset (with auto-zero) | additive offset residual | N/A (additive) | ±few nV (noise floor of GND measurement) | ±few nV |
| PREAMP_GAIN (2000) | `preamp_delta` scale | 0.15 nV/ppm (at 1 DAC LSB delta) | ±3000 ppm (AD8428 ±0.3%) | ±450 pV |
| ADC_VREF (2.500 V) | `preamp_delta` scale | 0.15 nV/ppm (at 1 DAC LSB delta) | depends on reference source | similar to gain |
| Ref drift compensation | `V_dac` correction residual | proportional to drift rate | 10% prediction error typical | 12.5 nV at 1 ppm/min drift |
| Ref filter time constant (1.05 s) | drift prediction accuracy | proportional to drift rate | ±20% estimated | 25 nV at 1 ppm/min drift |
| DAC filter settling | transient accuracy | step-size dependent | verified to 10 ppm per comments | ±10 ppm during transitions |

**Summary ranking by impact at 1 V full-range:**

1. **ADR1001 reference absolute accuracy** — sets ppm-level absolute floor
2. **DAC INL (must calibrate)** — ±1.5 mV without the table, ±few nV with it
3. **HV divider ratios** — dominate at HV ranges; unchecked constants in firmware
4. **Zero offset** — handled by auto-zero; verify regularly
5. **Preamp gain** — 300–450 pV at 1 DAC LSB delta; matters at sub-nV goals
6. **ADC_VREF** — similar magnitude to gain error
7. **Drift compensation** — matters during temperature transients

---

## Calibration Items (Priority Order)

### CAL-01 — ADR1001 Voltage Reference (Absolute Value)  ★★★ CRITICAL

**What:** The AD5760 DAC output is ratiometric to `VREFP` (the filtered ADR1001 average at TP1).
The DAC calibration table stores voltages measured during calibration — those measurements are
only as accurate as the reference at calibration time.

**Impact:** 1 ppm reference error → 1 ppm error in all Vx readings. At 200 ppm initial tolerance,
the reference is off by up to 1 mV at 5 V — far outside the goal.

**How to calibrate:**
1. Measure TP1 (filtered reference output, the DAC VREFP) with a calibrated DC voltmeter
   traceable to a national standard (e.g., a 10 V Josephson reference or calibrated Fluke 8508A).
2. Record actual voltage, e.g. `V_ref_actual = 4.999987 V`.
3. After building the DAC calibration table (CAL-02), all table entries were measured *relative to
   this reference* and are self-consistent. No firmware constant needs to change if the table
   was built using the same reference value.
4. For absolute traceability, scale all Vx readings by `V_ref_actual / 5.000000` — or rebuild
   the DAC calibration table after the reference has been measured and stabilized.

**Firmware constant:** None directly; accuracy is embedded in the calibration table values.

**Recommended interval:** Before each calibration table build; annually thereafter for absolute work.
Note: ADR1001 is specified 0.05 ppm/°C, so 10 °C ambient change causes 500 pV at 1 V — acceptable
for most work but measure the reference when highest absolute accuracy is needed.

---

### CAL-02 — DAC INL Calibration Table  ★★★ CRITICAL

**What:** The 16,384-point table `dacCalTable` maps each 4th DAC code (14-bit index) to an actual
measured output voltage, correcting AD5760 integral nonlinearity (INL typ ±3 LSB, max ±10 LSB).
Without the table the firmware uses `code × DAC_LSB_V` (nominal), which can be off by ±1.5 mV.

**Impact:** The DAC sets the coarse measurement value. ±1.5 mV uncalibrated is catastrophic
for a nV-level instrument. With the table, the error is limited by ADC noise during calibration
(achieves ±few nV with sufficient averaging).

**Current state:** The table is initialized to nominal values at boot (`dacCalTable.isValid()`
returns `false`). If a previous `cal save` was run with a valid table, it is automatically
restored from LittleFS flash on boot.

**Build commands (implemented):**

`cal build dac` — automatic anchor calibration (no external source needed):
- Switches to the GND input; binary-searches DAC to zero-crossing; sweeps ±2 table entries
  around the convergence code (5 entries total); measures actual V_dac at each code.
- Switches to VrefRaw (~5 V); binary-searches DAC; sweeps ±2 entries around that code.
- Marks the table valid and saves to flash automatically.
- Coverage: ~5 points near 0 V + ~5 points near 5 V. Corrects DAC zero offset and full-scale
  gain. Duration: < 5 seconds.

`cal point <voltage>` — capture a single entry from a known external voltage applied to Vx:
- User applies an accurate known voltage to the Vx input terminal.
- Firmware binary-searches DAC to lock onto the applied voltage.
- Accumulates 20 chopped measurements; stores `V_dac = V_known − ADC_residual/PREAMP_GAIN`
  at the current DAC code.
- Repeat at different voltages across ±5 V to fill in the full table.
- Call `cal save` when finished to persist the updated table.

**Typical workflow:**
```
cal build dac              # Quick auto-cal at 0 V and 5 V anchors; saves automatically
# (optional: apply external source at multiple voltages for full-range INL correction)
cal point 1.000000         # Apply 1.000000 V to Vx, then run this
cal point 2.000000         # Apply 2.000000 V to Vx, then run this
# ... repeat at -5, -4, -3, -2, -1, 0, 1, 2, 3, 4, 5 V etc.
cal save                   # Persist all point entries to flash
```

**Measurement window constraint:** The preamp clips when |Vx − V_dac| > ~1 mV. Each `cal point`
call can only store one table entry (the code the DAC locked to). To cover the full ±5 V range,
~16,380 separate measurements would be needed — in practice, a `diffvm.py` script stepping an
external calibrated source handles this automatically.

**Sparse calibration: interpolating between cardinal points.** If you do not have an automated
sweep and instead measured ~30 representative ("cardinal") points across the range, you can
piecewise-linearly fill in the rest:

```
cal cardinals      # list every operator-set point with idx, code, voltage, residual µV
cal clear <code>   # remove a bad cardinal (e.g. 'cal clear 19664' if its residual is way out)
cal interpolate    # linearly interpolate between adjacent cardinals; extrapolates outside
                   # the cardinal range using the slope of the nearest pair
cal save           # persist the now-fully-populated table to flash
```

`cal cardinals` is non-destructive and can be re-run any time. After `cal interpolate`, every
slot is non-nominal so the cardinal set effectively becomes "all slots" — to refine, run
`cal load` (reverts to the saved state) before re-interpolating.

**Choosing where to put cardinal points:** the DAC INL is roughly cubic, so place a cardinal
near each end of travel (±5 V), one near zero, and the rest spaced more densely where the INL
gradient is largest (typically near the rails). 30 well-chosen points typically give residual
INL well below ±10 µV across the range; the residual scales with the *missing curvature* of
the actual INL curve at the gaps between cardinals.

**Firmware constants used (nominal table initialization):**
```cpp
static constexpr double DAC_LSB_V = (2.0 * DAC_VREF) / DAC_FSR_CODES;  // ~152.59 µV
static constexpr double DAC_VREF  = 5.0;  // Update if reference measurement shows different value
```

**Recommended interval:** After initial build, rebuild if reference is serviced, or annually.

---

### CAL-02a — Keithley-validated DAC table refinement (`cal verify` / `cal smooth`)  ★★★ CRITICAL

**What:** After the bulk DAC table is built (CAL-02), `cal verify` walks through a default
list of 25 target voltages (±5, ±4, ±3, ±2.5, ±2, ±1, ±0.5, ±0.1, ±0.05, ±0.01, ±0.005,
±0.001, 0 V) and uses a Keithley 2002 (or any traceable DMM) as a second reference to spot-check
the calibration. Setup: a precision source (Fluke calibrator) drives the DiffVM `Vx2` input;
the Keithley measures that same input. The firmware sets a *deterministic* 14-bit-aligned DAC
code at each target (no binary search — so the same target always lands on the same code
between runs), runs a full-length chop measurement, computes DiffVM's reported `Vx`, and asks
you to type the Keithley reading. If DiffVM disagrees with Keithley by more than `--thresh`
(default 10 µV), the cal table entry at that code is overwritten with the value that would
make DiffVM match Keithley:
```
vDac_new = V_keithley − (adcMean × ADC_LSB_V / PREAMP_GAIN)
```
Within-threshold targets are kept as-is — most points should pass, and the cal table is only
touched where it's genuinely wrong.

**Why a separate command from `cal point`:** `cal point` uses `binarySearchDAC()` to lock onto
a known voltage, then stores the measured DAC residual. That's fine for *building* a table but
non-deterministic for re-verification — the code it converges to depends on noise and starting
state. `cal verify` instead picks the DAC code by deterministic rounding, accepts the operator's
Keithley reading as the trusted absolute value, and only updates the table when the existing
entry is wrong by more than threshold.

**Workflow per target:**
1. Firmware prints `[i/25] Target +0.05 V, DAC code 328 (V_nom = +0.050033 V). Apply ~0.05 V; enter Keithley reading (V) or s/a/q:`.
2. You dial the Fluke near 0.05 V, read the Keithley (e.g. `0.049902`), type that into the firmware.
3. Firmware runs ~285 chop cycles (≈15 s, configurable via `--cycles N`) at the deterministic DAC code.
4. It prints DiffVM, Keithley, and diff in µV. Either reports "OK: within ±10 µV — no update" or "Updated cardinal[code]: was X, now Y (Δ = Z µV)".
5. **Smoothing between targets:** every time a target is confirmed (Updated *or* Within-threshold) the firmware linearly interpolates the slots between the previous confirmed target and the current one. This keeps the table self-consistent across the verified range and prevents the discontinuities that `interpolateGaps()` can't fix once the table is dense (`MAX_CARDINALS=256` cap is bypassed by `interpolateBetweenCodes`).
6. Single-letter commands at the prompt: `s` skip this target, `a` abort (no save), `q` quit + smooth + suggest save.

**Commands:**
```
cal verify                                    # interactive, default 25 targets, 10 µV thresh, ~15 s/target
cal verify --thresh 5                         # tighter threshold for a follow-up refinement pass
cal verify --cycles 50                        # quick pass (~3 s/target) before a careful one
cal verify 0.05 0.1 0.5                       # override target list
cal save                                      # persist the updated table to flash
```

**`cal smooth` — post-hoc fix without the Keithley:** if a verify session was saved before the
inter-target smoothing was implemented (or if the session was interrupted partway), the saved
table can have correct cardinal values at the verify codes surrounded by stale cal-build-dac
interpolation on either side. Symptom: `cal cardinals` shows residual spikes at the verify
codes (30–130 µV jumps relative to neighbours). Fix with no measurement:
```
cal smooth        # blends slots between consecutive default verify-target codes
cal smooth 0.05 0.1   # blend just one pair (e.g. if only two cardinals are isolated)
cal save          # persist
```
This reuses the existing cardinal values at each target as fixed endpoints and linearly blends
everything between them. Safe to run with the Keithley turned off.

**Implementation notes:**
- The verify workflow lives in its own module: `CalVerify.h` / `CalVerify.cpp`, decoupled from
  the .ino through a `CalVerifyApi` function-pointer struct (no direct dependency on
  `DacCalibrationTable` or other internal types).
- `DacCalibrationTable::interpolateBetweenCodes(lo, hi)` is the linear-blend primitive used by
  both `cal verify` (mid-session) and `cal smooth` (post-hoc). Operates on the exact code range
  specified, so it's unaffected by `MAX_CARDINALS=256`.
- The default target list `kCalVerifyDefaultTargets[]` is exported from `CalVerify.h` so
  `cal smooth` and `cal verify` use the exact same codes.
- The calibration input is `Vx2` (separate from the normal Vx1 measurement port) so the
  Keithley/Fluke setup can stay wired during normal use.

**Recommended interval:** Run `cal verify` after every `cal build dac` to validate the dense
table against an independent reference; re-run annually or after the Keithley is recertified.
Run `cal smooth` only as a one-shot recovery for an interrupted verify session.

---

### CAL-03 — Zero Offset (Auto-Zero)  ★★★ CRITICAL

**What:** `performAutoZero()` measures the GND input channel, extracts the full-chain offset
(preamp DC offset, MUX charge injection residuals, PCB thermal EMFs, DAC zero error), and stores
it in `scanner.autoZeroOffset`. All subsequent Vx readings subtract this value.

**Impact:** Without auto-zero: the preamp and mux offset contributions can be 10–100 µV.
Chopping eliminates most preamp offset and 1/f noise, but thermal EMFs and DAC zero offsets
persist. Even 100 nV of uncorrected offset is 100 nV absolute error.

**Current state:** Implemented and functional. Auto-zero runs:
- Manually: `zero` command
- Automatically: every `autoZeroInterval` scan sweeps (default 10, configurable with `autozero interval N`)
- Enable/disable: `autozero on|off`

**Not persisted:** `autoZeroOffset` is stored in RAM and lost on reboot. A first `zero` run
after boot is mandatory before trusting any readings.

**Verification:**
1. After auto-zero, switch to GND channel (`scan add GND`) and verify reading ≈ 0 ± noise floor.
2. Typical residual should be < ±10 nV (noise limited at 20 chop cycles, ~1 µV/√(20)
   divided by the √N improvement).

**Recommended interval:** Every measurement session start; automatically every few minutes during
long runs (thermal EMFs change with environment temperature).

---

### CAL-04 — HV Divider Ratios  ★★ HIGH (HV ranges only)

**What:** Firmware uses runtime-adjustable constants for the three HV divider ratios
(initialized to calculated defaults at boot; overridden from flash if `cal save` was run):
```cpp
static double DIVIDER_RATIO_10   = 10.0;      // ±50V range
static double DIVIDER_RATIO_100  = 108.76;    // ±500V range (calculated from Caddock specs)
static double DIVIDER_RATIO_1000 = 984.65;    // ±5kV range (calculated from Caddock specs)
```

`DIVIDER_RATIO_100` and `DIVIDER_RATIO_1000` are calculated from nominal Caddock 1776-C4815
resistor segment values. Actual values depend on:
- Caddock resistor ratios (specified ±0.01% matching, but absolute resistance has tolerance)
- MUX36D04 on-resistance (~250 Ω modeled in firmware, varies with process)
- PCB trace resistance at the junction points

**Impact:** A 100 ppm ratio error at the ÷108.76 range (measuring 100 V) = 10 µV at 1 V referred
to input — dominates all other error sources for HV operation.

**How to calibrate:**
1. Apply a known accurate voltage in the ÷10 range (e.g., from a calibrated 10 V source at TP-HV).
2. Compare the DiffVM reading to the known voltage; compute the actual ratio.
3. For ÷100 and ÷1000, apply voltages at 10× and 100× respectively with known accuracy and
   compute actual ratios.

Alternatively, cross-calibrate using the ±5V range: apply exactly 10 V (or 100 V, 1000 V) to the
HV input and compare the HV-range reading to a direct ±5V measurement of a /10 (or /100, /1000)
attenuated version using a calibrated resistive divider.

**Firmware update:** Use serial commands — no recompile needed:
```
cal set div10  <measured_ratio>
cal set div100 <measured_ratio>
cal set div1000 <measured_ratio>
cal save
```
Values persist in LittleFS flash and reload automatically on boot.

**Recommended interval:** Before first HV measurements; annually or after component replacement.

---

### CAL-04a — OPA828 Buffer Offset  ★★ HIGH (HV ranges only)

**What:** The HV divider output is buffered by an OPA828 voltage follower before the MUX36D08
input multiplexer. The OPA828 input offset voltage (VOS) adds a fixed DC error to every HVDiv
measurement. Typical VOS is a few to tens of µV; the production maximum is 150 µV at 25°C.

**Why chopping does not help:** The TMUX7234 chopper switches sit *after* the OPA828 in the signal
path. Chopping toggles signal polarity at the TMUX, so only offsets that appear after that point
(i.e., in the AD8428 preamp chain) are cancelled. The OPA828 offset passes through the chopper
unchanged, appearing as a fixed DC error in `vxAtPreamp`.

**Signal path:**
```
HV Input → Caddock divider → [OPA828 + VOS] → TPD4E1B06 → MUX36D08 S6 → TMUX (chop) → AD8428 → ADC
                                  ↑ error here — not cancelled by chopping
```

**Impact on measurement accuracy:**

The offset is at the OPA828 output and is multiplied by the active divider ratio in the final result:

| Range | Ratio | 16 µV offset → input-referred error |
|-------|-------|--------------------------------------|
| ±50V (÷10) | 10.0 | **160 µV** |
| ±500V (÷108.76) | 108.76 | **1.74 mV** |
| ±5kV (÷984.65) | 984.65 | **15.8 mV** |

The ÷10 range is most affected. At ÷100 and ÷1000 the input-referred impact is negligible
relative to full scale, but the OPA828 offset itself should still be corrected for completeness.

**How the correction is applied in firmware:**

In `computeInputVoltage()`, when channel == `HVDiv`, `OPA828_OFFSET` is subtracted from
`vxAtPreamp` before multiplying by the divider ratio:

```
Vx = (vxAtPreamp − OPA828_OFFSET) × dividerRatio
```

**Measurement procedure:**

1. Short the HV divider input to ground (or connect a stable, known-zero source).
2. Select the ÷10 range (`divider 10`).
3. Allow the measurement to stabilize (15–30 minutes of thermal warm-up recommended).
4. Record the displayed voltage; divide by the divider ratio to get the raw offset:
   ```
   OPA828_OFFSET = displayed_voltage / DIVIDER_RATIO_10
   ```
   Example: display reads 160 µV → OPA828_OFFSET = 160 µV / 10 = **16 µV = 0.000016 V**.
5. Apply and save:
   ```
   cal set opa828 0.000016
   cal save
   ```
6. Verify: re-measure with HV input grounded — reading should be near zero (noise floor).

**Temperature drift:** The OPA828 typical offset TC is 1.8 µV/°C. A 10°C warm-up from cold start
changes the offset by ~18 µV. At ÷10 this is 180 µV referred to input. For best results, measure
the offset after the board has thermally stabilised (typically 15–30 minutes after power-on), and
re-calibrate if the ambient temperature changes by more than a few degrees.

**Firmware update:** No recompile needed:
```
cal set opa828 <value_in_volts>    (e.g. 0.000016 for 16 µV)
cal save
```

Persisted in `/cal_consts.bin` (v2 format). On first load of a v1 file, defaults to 0.0 V.

**Recommended interval:** At initial commissioning after thermal stabilisation; re-measure if the
OPA828 is replaced or if ambient temperature changes significantly.

---

### CAL-05 — Preamp Gain (PREAMP_GAIN)  ★ MEDIUM

**What:** `PREAMP_GAIN = 2000.0` scales the ADC residual back to an input voltage:
```
preamp_delta = ADC_mean × ADC_LSB_V / PREAMP_GAIN
```
This term is the small correction between the DAC coarse voltage and the true Vx.

**Impact:** With the DAC binary-search tracking Vx to within ≤1 DAC LSB (152.6 µV), the
worst-case preamp_delta is ±152.6 µV. The AD8428 gain accuracy is ±0.3% (3000 ppm), giving:

```
Error = 3000 ppm × 152.6 µV = 458 pV
```

For 8.5-digit operation (≈3 nV at 1 V), this is well within budget.
For 9.5-digit operation (≈0.3 nV at 1 V), it contributes ~1.5× the noise floor but is still
acceptable since preamp_delta is typically much less than 1 DAC LSB when Vx is stable.

**How to measure:** Apply a known accurate voltage Vx (e.g., 1.000 000 0 V from a calibrated source
or Josephson reference). With the DAC calibration table active and a good auto-zero, the residual
`preamp_delta` term can be extracted by comparing readings with PREAMP_GAIN set to 2000 vs.
an externally measured gain value.

Alternatively, directly measure the closed-loop gain of the AD8428 chain by applying a known
differential input and measuring the output with a calibrated oscilloscope or ADC.

**Firmware update:** Use serial commands — no recompile needed:
```
cal set gain <measured_gain>
cal save
```
Value persists in LittleFS flash and reloads automatically on boot.

**Recommended interval:** At initial commissioning. Re-measure if AD8428s are replaced.

---

### CAL-06 — ADC Reference Voltage (ADC_VREF)  ★ MEDIUM

**What:** `ADC_VREF = 2.500` V sets the ADS127L11 full-scale range for the ADC:
```
ADC_LSB_V = (2 × ADC_VREF) / 2^24 ≈ 298.023 nV/count
```
This affects only `preamp_delta` (same as PREAMP_GAIN above).

**Note on reference source:** Confirm which component drives ADS127L11 VREF pin (check schematic
`DAC.kicad_sch`). If it is derived from the same ADR1001 chain as the DAC reference, then:
- ADC_VREF and DAC_VREF drift together, and their ratio is stable
- Reference absolute drift is handled by the drift compensation system
- Residual error from ADC_VREF inaccuracy is small

If ADC VREF comes from a *separate* component, measure it directly and update the constant.

**Impact:** Same order as PREAMP_GAIN error: at 1 DAC LSB preamp_delta and 1000 ppm ADC_VREF
error → 152.6 µV × 1000 ppm = 152.6 pV. Acceptable for most purposes.

**Firmware update:**
```cpp
static constexpr double ADC_VREF = 2.500;  // Replace with measured value if separate reference
```
Note: changing `ADC_VREF` also changes `ADC_LSB_V` (derived constant) automatically.

**Recommended interval:** At initial commissioning. Rarely needs updating if sharing ADR1001 chain.

---

### CAL-07 — Reference Filter Time Constant (REF_FILTER_TAU_EFF)  ★ MEDIUM

**What:** `REF_FILTER_TAU_EFF = 1.05` seconds is the effective time constant of the Sallen-Key
low-pass filter (fc = 0.3 Hz, Q = 0.566) that smooths the ADR1001 average before it reaches
the DAC reference input. It is used to predict how much the filter output lags the true reference
during temperature transients.

An incorrect τ causes over- or under-prediction of the filter error, leaving a residual that
scales with the reference drift rate.

**Impact:** At 1 ppm/min drift rate, the filter lag is ~25 ppb (125 nV at 5 V). A 20% error
in τ leaves 25 nV of uncorrected residual. For slower drift rates (normal lab conditions),
this contribution is proportionally smaller.

**How to measure:**
1. Apply a step change to the reference (or simulate in software by forcing `filterHistory` data).
2. Monitor the drift compensation output over several seconds.
3. Fit an exponential to the step response to extract the actual τ.

Alternatively, perform a frequency sweep of the J3 input signal and measure the filter −3 dB
corner frequency from the VrefRaw measurement; back-calculate τ from the corner frequency.

**Firmware update:**
```cpp
static constexpr double REF_FILTER_TAU_EFF = 1.05;  // Replace with measured value
```

Also verify the `REF_STEP_RESPONSE[]` lookup table matches the actual measured filter step
response. The table is currently theoretical (from filter transfer function computation).

**Recommended interval:** At initial commissioning after PCB assembly and component selection.

---

### CAL-08 — Reference Filter Step Response Table  ★ MEDIUM

**What:** The 81-point array `REF_STEP_RESPONSE[]` contains the theoretical step response of the
fc = 0.3 Hz, Q = 0.566 Sallen-Key filter, sampled at 0.1 s intervals. It is used by
`predictFilterError()` to estimate current filter error between measurement samples.

**Impact:** Errors in the table cause prediction overshoot or undershoot during step disturbances.
Normal lab temperature drift is slow enough that the table matters mainly during rapid thermal
transients (e.g., when the instrument is first powered on or moved).

**How to calibrate:**
1. Step the DAC reference (inject a step at J3 test point if accessible, or change the lab
   temperature rapidly).
2. Log the `filterErr` output from firmware at high time resolution.
3. Fit the observed step response and regenerate the lookup table entries.

**Firmware update:** Replace the `REF_STEP_RESPONSE[]` array values in the firmware source.

**Recommended interval:** At initial commissioning; re-verify if filter capacitors or resistors
are changed.

---

### CAL-09 — DAC Filter Settling Constants  ★ LOW–MEDIUM

**What:** After a DAC code change, the firmware waits for the second-order Bessel output filter
to settle before taking measurements. Constants:
```cpp
static constexpr uint32_t DAC_SETTLE_MIN_MS = 80;   // Small steps
static constexpr uint32_t DAC_SETTLE_MAX_MS = 300;  // Full-scale step
```
Formula: `settle_ms = 80 + 220 × √(step_size / 32768)`

The code comments say these values were derived from a measured step response ("overshoot peak
~0.45% at t≈107ms, 10 ppm settling at t≈250ms"). Verify on the assembled PCB since actual
filter capacitor and resistor tolerances affect settling time.

**Impact:** If settling time is underestimated, readings immediately after a DAC code change
contain a systematic bias. Normal measurement cycles (DAC stationary) are unaffected.

**How to verify:** After a full-scale DAC step, log the ADC output (with Vx = const) over time
and fit the settling curve. Confirm 10 ppm settling occurs before `DAC_SETTLE_MAX_MS`.

**Firmware update:** Adjust `DAC_SETTLE_MIN_MS` and/or `DAC_SETTLE_MAX_MS` if needed.

**Recommended interval:** At initial commissioning and after output filter component changes.

---

### CAL-10 — Chop Symmetry  ★ LOW (verified via POST)

**What:** The chopping scheme demodulates as `(Phase1_sum − Phase2_sum) / 2`. Perfect symmetry
requires that the SETTLE_US guard time (300 µs) and DISCARD_SAMPLES (1) are sufficient for
the mux and preamp to settle after each switch transition. Asymmetry creates a residual offset
that is NOT removed by auto-zero (since auto-zero itself uses the same chopping scheme).

**Current values:**
```cpp
static constexpr uint32_t SETTLE_US      = 300;  // Guard time after mux edge
static constexpr uint8_t  DISCARD_SAMPLES = 1;   // Samples dropped after switch
static constexpr uint8_t  GOOD_SAMPLES   = 3;    // Samples averaged per half-cycle
```

**POST test:** `postTestChopSymmetry()` verifies residual < 10% of signal magnitude.

**Impact:** Only affects measurements if the TMUX7234 or AD8428 have asymmetric settling.
Typically dominated by other error sources.

**How to verify:** Compare Phase1 and Phase2 readings (P1 and P2 from POST output) at several
signal levels. Sum = P1 + P2 should approach zero. If |sum| > 1% of |P1|, increase SETTLE_US
or DISCARD_SAMPLES.

**Recommended interval:** At initial commissioning; re-verify if TMUX components are changed.

---

### CAL-11 — ADC Clock Frequency  ★ LOW (verified via POST)

**What:** The external 25 MHz clock to the ADS127L11 determines the sample rate
(`EST_FSPS ≈ 3906.25 SPS`) and hence all timing-dependent calibration constants. Verified
by `postTestDrdyTiming()` (checks DRDY rate ±10%).

**Impact:** A frequency error shifts the DRDY timing but does not affect the voltage accuracy
of individual samples (since the ADC is a ratiometric measurement, not clock-frequency-dependent
for DC accuracy). It does affect the `tau0` calculation in Allan deviation reporting.

**How to verify:** Read the DRDY rate from POST output. The POST passes if within ±10% of
3906 Hz; for full accuracy the actual rate should be within 0.1% (i.e., the 25 MHz oscillator
specification).

**Recommended interval:** Verified at every boot via POST.

---

## Calibration State Persistence

| Calibration Item | Persisted? | Location | Notes |
|---|---|---|---|
| DAC calibration table | **Yes** | LittleFS flash (512 KB) | Persisted by `cal save`; restored on boot. Build with `cal build dac` (auto) or `cal point <v>` (external source). |
| Preamp gain (PREAMP_GAIN) | **Yes** | LittleFS flash | Persisted by `cal save`; adjust via `cal set gain <v>`. |
| HV divider ratios | **Yes** | LittleFS flash | Persisted by `cal save`; adjust via `cal set div10/div100/div1000 <v>`. |
| OPA828 buffer offset | **Yes** | LittleFS flash | Persisted by `cal save` (v2 format); adjust via `cal set opa828 <v>`. |
| Auto-zero offset | **No** | RAM only | Lost on reboot; run `zero` after every boot. |
| Reference drift tracking (filter history) | **No** | RAM only | Reinitializes after reboot; takes ~8 s to stabilize. |
| Scan configuration, divider ratio, auto-start | Yes | EEPROM | Persisted by `config save`; restored on boot. |

**Flash persistence status:** DAC calibration table, preamp gain, and divider ratios are
persisted to LittleFS (512 KB partition on Teensy 4.1 QSPI flash) via `cal save`.
The CAL-02 build procedure is implemented: `cal build dac` provides automatic anchor
calibration; `cal point <v>` enables full-range INL correction with an external source.
The full calibration round-trip is now complete.

---

## Calibration Schedule

| Trigger | Actions Required |
|---|---|
| First power-on after assembly | CAL-01, CAL-02, CAL-04, CAL-04a, CAL-05, CAL-06, CAL-07, CAL-08, CAL-09, CAL-10; verify POST passes |
| Every boot | Run `zero` (CAL-03); DAC cal table auto-loads from flash if `cal save` was run |
| Every measurement session | Run `zero` (CAL-03); verify `autozero on` is enabled |
| Temperature change > 5 °C | Run `zero` (CAL-03); allow drift compensation to stabilize (≥10 s) |
| Annual (absolute accuracy work) | CAL-01 (reference voltage); CAL-02 (rebuild table vs. fresh reference measurement) |
| After component replacement | Repeat all items relevant to the replaced component |

---

## Firmware Constants Summary

All calibration-related constants in `OpenSourceDiffVM-RevB.ino` that may need updating
based on measurements:

```cpp
// --- Core precision constants (lines ~154–168) ---
static constexpr double ADC_VREF    = 2.5;      // CAL-06: measure ADS127L11 VREF
static constexpr double DAC_VREF    = 5.0;      // CAL-01: embedded in cal table; used for nominal init only
// PREAMP_GAIN and divider ratios are now runtime-mutable; use 'cal set' + 'cal save':
static double PREAMP_GAIN           = 2000.0;   // CAL-05: 'cal set gain <v>'

// --- HV divider ratios and buffer offset ---
static double DIVIDER_RATIO_10   = 10.0;    // CAL-04: 'cal set div10 <v>'
static double DIVIDER_RATIO_100  = 108.76;  // CAL-04: 'cal set div100 <v>'
static double DIVIDER_RATIO_1000 = 984.65;  // CAL-04: 'cal set div1000 <v>'
static double OPA828_OFFSET      = 0.0;     // CAL-04a: 'cal set opa828 <v>' (volts; typ. ~16 µV)

// --- Reference drift compensation (lines ~174–197) ---
static constexpr double NOMINAL_REF_V     = 5.0;   // Low priority; use ADR1001 measured value
static constexpr double REF_FILTER_TAU_EFF = 1.05; // CAL-07: measure from step response
static const float REF_STEP_RESPONSE[]    = {...};  // CAL-08: regenerate from measured step response

// --- DAC filter settling (lines ~107–108) ---
static constexpr uint32_t DAC_SETTLE_MIN_MS = 80;   // CAL-09: verify on assembled board
static constexpr uint32_t DAC_SETTLE_MAX_MS = 300;  // CAL-09: verify on assembled board

// --- Chop timing (lines ~95–98) ---
static constexpr uint32_t SETTLE_US      = 300;  // CAL-10: increase if chop symmetry fails
static constexpr uint8_t  DISCARD_SAMPLES = 1;   // CAL-10: increase if chop symmetry fails
```
