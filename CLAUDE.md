# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is a **high-resolution differential voltmeter** (8.5-9.5 digit) for nanovolt-level sensitivity, based on the Analog Devices article on low-noise instrumentation amplifiers. It uses a Teensy 4.x microcontroller as an integral part of a chopped measurement system.

Reference: https://www.analog.com/en/resources/analog-dialogue/articles/low-noise-inamp-nanovolt-sensitivity.html

## Build Commands

Build and upload using Arduino IDE or PlatformIO with Teensy 4.x board support:
```bash
# Arduino CLI (path on this system)
"/c/Program Files/Arduino IDE/resources/app/lib/backend/resources/arduino-cli.exe" compile --fqbn teensy:avr:teensy41 OpenSourceDiffVM-RevB.ino
"/c/Program Files/Arduino IDE/resources/app/lib/backend/resources/arduino-cli.exe" upload --fqbn teensy:avr:teensy41 -p COM_PORT OpenSourceDiffVM-RevB.ino
```

## Signal Chain

```
Unknown Vx ──┐
             ├──[TMUX SPDT]──► AD8428 x4 ──► ADS127L11 ──► Teensy 4.x
DAC Output ──┘    (×2000 gain)              (24-bit ADC)
```

## Operating Principle

**Hardware signal routing (AD8428: pin 4 = V+, pin 1 = V-):**

| Phase | Switch State | DAC→ | Vx→ | ADC Reads | Signal |
|-------|--------------|------|-----|-----------|--------|
| 1 | TMUXSEL=LOW | pin 1 (V-) | pin 4 (V+) | x1 | (Vx - DAC) × 2000 |
| 2 | TMUXSEL=HIGH | pin 4 (V+) | pin 1 (V-) | x2 | (DAC - Vx) × 2000 |

**Demodulated output:** `(x1 - x2) / 2` cancels preamp offset and 1/f noise

**Final measurement:** `Vx = DAC_calibrated + ADC_average`

The DAC output must be within a few millivolts of the unknown input to avoid overdriving the preamp. DAC calibration uses 2^14 points from -5V to +5V to minimize INL errors.

## Key Components

| Component | Function |
|-----------|----------|
| 4× AD8428 | Ultra-low-noise instrumentation amplifier (1.5 nV/√Hz), gain = 2000 total |
| TMUX7234 | Low-charge-injection SPDT switches for input chopping |
| ADS127L11 | 24-bit delta-sigma ADC, ~3906 SPS |
| AD5760 | 16-bit precision DAC, provides reference point within mV of Vx |
| 3× ADR1001 | Ultra-stable voltage reference (0.05 ppm/°C), 1000+ hr burn-in |

## KiCad Project

The hardware design files are in the `DiffVM/` folder:

| File | Description |
|------|-------------|
| `DiffVM_4L.kicad_sch` | Root schematic (4-layer PCB version) |
| `FrontEnd.kicad_sch` | Front-end: TMUX switches, AD8428 preamp chain, AD5760 DAC |
| `DAC.kicad_sch` | ADS127L11 ADC and Teensy interface |
| `VRef.kicad_sch` | 3× ADR1001 voltage references |
| `DiffVM_4L.kicad_pcb` | PCB layout |

### Key Test Points

The schematic includes ~44 test points for debugging and self-test:
- **FrontEnd**: TP1-TP7, TP24-TP40, TP44 (mux outputs, filter nodes, DAC output)
- **DAC/ADC**: TP8-TP23 (ADC signals, digital interfaces)
- **VRef**: TP41-TP43 (reference outputs)

## Hardware Pin Mapping

| Function | Teensy Pin |
|----------|------------|
| ADC CS | 41 |
| DAC CS | 10 |
| ADC DRDY | 15 |
| ADC Clock (25 MHz) | 9 |
| TMUX Enable | 4 |
| TMUX Select | 2 |
| CI TMUX Select | 3 |
| DAC Reset | 25 |
| DAC Clear | 24 |
| Input Mux A0 | 35 |
| Input Mux A1 | 34 |
| Input Mux A2 | 33 |
| Input Mux EN | Hardwired to VDD |
| Divider Mux A0 | 23 |
| Divider Mux A1 | 22 |

## Input Multiplexer (MUX36D08)

An 8:1 input multiplexer allows selecting between the unknown input, calibration references, and HV divider:

| Channel | Address | Signal | Purpose |
|---------|---------|--------|---------|
| S1 | 0 | Vx1 | Unknown input 1 (normal measurement, ±5V) |
| S2 | 1 | GND | Zero calibration |
| S3 | 2 | VrefRaw | Pre-filter ADR1001 average (J3) for drift compensation |
| S4 | 3 | Vx2 | Unknown input 2 |
| S5 | 4 | Vx3 | Unknown input 3 |
| S6 | 5 | HVDiv | HV divider output (ratio selected by MUX36D04) |
| S7 | 6 | Vx4 | Unknown input 4 |
| S8 | 7 | Vx5 | Unknown input 5 |

### Key Functions

```cpp
selectInputChannel(InputChannel::GND);     // Switch to ground
selectInputChannel(InputChannel::VrefRaw); // Switch to raw reference average
selectInputChannel(InputChannel::Vx1);     // Switch back to unknown
InputChannel ch = getInputChannel();       // Get current channel
const char* name = getInputChannelName(ch); // Get channel name
```

### Self-Calibration Sequence

```cpp
// Zero calibration
selectInputChannel(InputChannel::GND);
setDacCode(0);
// ... measure offset ...

// Reference drift compensation runs automatically in main loop
// Every ~1 second, firmware measures VrefRaw to track filter error

// Resume normal operation
selectInputChannel(InputChannel::Vx1);
```

## HV Divider System (Caddock 1776-C4815 + MUX36D04 + OPA828)

Extended voltage ranges are implemented using four Caddock 1776-C4815 precision decade voltage dividers in a serial-parallel arrangement. The 1:10 divider is **hardwired always in circuit** for safety—this prevents HV transients from reaching the input when switching ratios. A MUX36D04 dual 4:1 low-leakage analog mux switches in parallel resistance to achieve higher division ratios. An OPA828 voltage follower buffers the divider output, followed by TPD4E1B06 ESD protection before the MUX36D08 input.

### Divider Topology

```
HV Input ──[FUSE]──► AB1 ──R── AB2 ──R── AB3 ──R── AB4 ──R── AB5 ──R── AB6 ──R── AB7
                              │         │         │         │                    ▲
                              │        [S1]      [S2]      [S3]                  │
                              │ MUX36D04 │         │         │                   │
                              │         │         │         │                    │
                     CD1 ──R── CD2 ──R── CD3 ──R── CD4 ──R── CD5 ──R── CD6 ──R── CD7
                      ▲       │                                                  │
                      │       ▼                                                  ▼
                      └── (from AB2)  OUTPUT ──► MUX S6                        GND
```

- **Pair AB**: Dividers A and B wired in parallel (pins tied together)
- **Pair CD**: Dividers C and D wired in parallel (pins tied together)
- **Series**: AB2 connects to CD1; CD2 is output and feeds back to AB7

### Divider Ratios (MUX36D04 Address Selection)

Calculated from Caddock 1776-C4815 segments (10M, 1.1111M, 101.01K, 10.01K) with 4 dividers as 2 parallel pairs in series. MUX path = Rseg/2 + Rseg/2 + 250Ω. Base 1.1111M arm always in circuit.

| Ratio | Address | A1 | A0 | Calculation | Range |
|-------|---------|----|----|-------------|-------|
| ÷10 | 0 | 0 | 0 | Mux bypassed (hardwired) | ±50V |
| ÷108.76 | 1 | 0 | 1 | 1.1111M ∥ 101.26K | ±500V |
| ÷984.65 | 2 | 1 | 0 | 1.1111M ∥ 10.26K | ±5kV |
| GND | 3 | 1 | 1 | Ground (zero cal) | — |

MUX36D04 EN hardwired to VDD (active high, always enabled).
Control: Teensy Pin 23 → A0, Teensy Pin 22 → A1.

### Key Functions

```cpp
// Select HV divider ratio (also switches MUX to HVDiv channel)
selectDividerRatio(DividerRatio::Div10);   // ±50V range (hardwired, mux bypassed)
selectDividerRatio(DividerRatio::Div100);  // ±500V range
selectDividerRatio(DividerRatio::Div1000); // ±5000V range
selectDividerRatio(DividerRatio::GND);     // Ground for zero calibration

// Query current state
DividerRatio ratio = getDividerRatio();
const char* name = getDividerRatioName(ratio);
double divRatio = getInputDividerRatio(InputChannel::HVDiv);
```

### Protection Components

- **Fuse**: 0ADAC0200-BE (200mA, 1000VDC) on HV input
- **ESD**: TPD4E1B06 (<50pA leakage) on divider output before MUX

### Safety Considerations

1. **Hardwired 1:10**: The base divider is always in circuit—mux switching cannot expose the input to full HV transients
2. **Input protection**: Fuse limits fault current; consider GDT for transients
3. **Spacing**: Maintain adequate creepage (>1mm per 100V) for HV traces
4. **Warning labels**: Clearly mark ±5000V maximum input
5. **Power dissipation**: Monitor Caddock temperature at high voltages


## Precision Voltage Computation (64-bit double)

The firmware computes the measured voltage using 64-bit double precision arithmetic (Teensy 4.x supports hardware double).

### Measurement Equation

```
Vx = (DAC_voltage + preamp_delta) × divider_ratio
```

Where:
- `DAC_voltage` = DAC output voltage (with optional INL calibration lookup)
- `preamp_delta` = (ADC_mean × ADC_LSB) / PREAMP_GAIN
- `divider_ratio` = input voltage divider ratio (1.0 for ±5V, 4.0 for ±20V, etc.)

### System Constants

| Parameter | Value | Notes |
|-----------|-------|-------|
| ADC_VREF | 2.500 V | ADS127L11 reference |
| ADC_LSB | ~298 nV | 5V / 2^24 |
| DAC_LSB | ~152.6 µV | 10V / 2^16 |
| PREAMP_GAIN | 2000 | 4× AD8428 cascade |
| Effective resolution | ~0.149 nV/count | ADC_LSB / PREAMP_GAIN |

### Key Functions

```cpp
// Compute measured voltage from statistics
double vx = computeInputVoltage(chopStats.mean(), currentDacCode, currentInputChannel);

// Compute measurement uncertainty
double uncertainty = computeInputUncertainty(chopStats.standardDeviation(), currentInputChannel);

// Convert DAC code to voltage (with optional calibration)
double dacV = dacCodeToVoltage(dacCode);  // Uses calibration table if valid

// Format voltage with SI prefix
char buf[32];
formatVoltage(vx, buf, sizeof(buf), 9);  // 9 significant digits
```

### DAC Calibration Table

A 16384-point calibration table corrects for DAC INL errors:
- Index: 14-bit (every 4 DAC codes)
- Value: measured voltage at that code
- Interpolation: linear between adjacent points
- Memory: ~131 KB (16384 × 8 bytes)

```cpp
// Store calibration point (during calibration procedure)
setDacCalPoint(dacCode, measuredVoltage);

// Mark table as valid after full calibration
markDacCalValid();

// Initialize with nominal (uncalibrated) values
initDacCalTable();
```

### Output Format

```
Vx = 1.23456789 mV +/- 1.23 nV (n=1000, DAC=12345, range=Vx (±5V))
  raw: mean=12345.678, sd=8.234, min=12300.123, max=12390.456
  drift: filterErr=25.3 nV, driftRate=0.167 nV/s, correction=5.06 ppb
```

## Reference Drift Compensation

The DAC reference comes from a filtered average of three ADR1001 voltage references. A Sallen-Key low-pass filter (fc=0.3Hz, Q=0.566) reduces noise but introduces lag during temperature transients. The firmware implements real-time predictive compensation to minimize this error.

### Reference Signal Path

```
ADR1001 A ──┐
ADR1001 B ──┼──► Avg ──┬──► LPF (fc=0.3Hz) ──► TP1 ──► DAC Reference
ADR1001 C ──┘          │
                      J3 (VrefRaw)
                       │
                   MUX (S3)
```

### Filter Characteristics

| Metric | Value |
|--------|-------|
| Cutoff frequency | 0.3 Hz |
| Quality factor | 0.566 (nearly critically damped) |
| 50% response | ~0.9 s |
| 90% response | ~1.85 s |
| Peak overshoot | 0.27% |
| Effective time constant | ~1.05 s |

### Lag Error Without Compensation

For a constant temperature drift rate, the filter output lags behind the true reference:

| Drift Rate | Lag Error | At 5V Reference |
|------------|-----------|-----------------|
| 1 ppm/10 min | ~2.5 ppb | 12.5 nV |
| 1 ppm/min | ~25 ppb | 125 nV |

### How Compensation Works

The key insight: measuring VrefRaw (J3) against the DAC gives us the filter error directly, because the DAC is referenced to TP1 (filter output).

1. **Periodic measurement**: Every ~385 chop cycles (~1 second), measure VrefRaw
2. **Error calculation**: ADC reading = (J3 - TP1) × preamp_gain = filter error
3. **Drift estimation**: Track error history to estimate reference drift rate
4. **Prediction**: Use step response lookup table to predict current error between measurements
5. **Correction**: Apply multiplicative correction to all Vx readings

### Correction Equation

```
Vx_corrected = Vx_measured × (1 + filterError / 5V)
```

Where `filterError` is the predicted (J3 - TP1) in volts.

### Key Functions

```cpp
// Initialize drift tracking state
initRefTracking();

// Measure current filter error (called every ~1 second)
measureFilterError();

// Predict filter error at current time
double err = predictFilterError();

// Get correction factor for Vx readings
double correction = getRefCorrectionFactor();
```

### Step Response Lookup Table

An 81-point table stores the filter step response at 0.1s intervals from 0-8 seconds. This enables accurate prediction of filter behavior between measurements using linear interpolation.

### Configuration Constants

| Constant | Value | Purpose |
|----------|-------|---------|
| `REF_SAMPLE_INTERVAL` | 385 | Chop cycles between reference measurements (~1 second) |
| `REF_MEASURE_ITERATIONS` | 10 | Chop cycles to average per reference measurement |
| `REF_MEASURE_DAC_CODE` | 32764 | DAC code for ~5V during reference measurement |
| `REF_FILTER_TAU_EFF` | 1.05 s | Effective filter time constant |
| `FILTER_HISTORY_SIZE` | 8 | Number of historical measurements to retain |

## Timing

Timing is ADC-driven (no fixed timer):
- `SETTLE_US`: 300 µs guard time after mux switching
- `DISCARD_SAMPLES`: 1 sample discarded after switching
- `GOOD_SAMPLES`: 3 samples averaged per half-cycle
- ADC sample rate: ~3906 SPS (sinc4+sinc1 filter, OSR 3200, fMOD 12.5 MHz)
- Effective chop rate: ~385 Hz (driven by ADC timing)

### DAC Filter Settling

A second-order Bessel filter on the DAC output reduces noise but requires settling time after DAC changes. Settling time scales with step size to maintain constant absolute error (~1 µV):

```
settle_ms = 80 + 220 × √(step_size / 32768)
```

| Step Size | Voltage | Settling Time |
|-----------|---------|---------------|
| 100 codes | 15 mV | ~92 ms |
| 1000 codes | 153 mV | ~118 ms |
| 10000 codes | 1.5 V | ~201 ms |
| 32768 codes | 5 V (full scale) | ~300 ms |

The settling delay is built into `setDacCode()` automatically via `calculateDacSettleTime()`.

## DAC Auto-Adjustment

The DAC must track Vx within ~2.5 mV to avoid preamp overload. The firmware implements automatic DAC adjustment:

### Overflow Detection
- Threshold: 90% of ADC full scale (±7,549,747 counts)
- Checked on first sample of each half-cycle

### Binary Search Algorithm
When overflow is detected:
1. Binary search across 16-bit DAC range (-32768 to +32767)
2. Set DAC to midpoint, read settled ADC sample
3. If ADC overflows positive: Vx > DAC → increase DAC (search upper half)
4. If ADC overflows negative: Vx < DAC → decrease DAC (search lower half)
5. Converges in ≤18 iterations
6. Statistics cleared after DAC change (measurement baseline changed)

### Startup Sequence
1. Initialize DAC in 2x mode (two's complement, ±5V range)
2. Run binary search to find initial DAC setting
3. Begin chopped measurement loop

### Key Functions
- `setDacCode(int16_t)`: Set DAC and track current code
- `binarySearchDAC()`: Full binary search, returns success/failure
- `isOverflow(int32_t)`: Check if sample exceeds threshold

## Error Sources

| Error Source | Mechanism | Mitigation in Design |
|--------------|-----------|---------------------|
| Preamp offset | DC offset in AD8428 | Eliminated by chopping: (x1-x2)/2 |
| 1/f noise | Low-frequency flicker noise | Chopping at 400 Hz moves signal above 1/f corner |
| Thermal EMF | Thermocouple effects at junctions | Requires careful PCB layout, isothermal design |
| Switch charge injection | Charge dump during TMUX switching | CI_TMUXSEL pulse sequence minimizes |
| DAC INL | Non-linearity in DAC output | 16384-point calibration table (-5V to +5V) |
| ADC settling | Transients after mux switch | 300 µs guard time + 1 sample discarded |
| Reference drift | ADR1001 aging/temperature | 1000+ hr burn-in, real-time drift compensation |
| Preamp overload | \|DAC - Vx\| too large | Auto-adjustment via binary search |
| Source impedance | Johnson noise, loading errors | Design limited to low-Z sources |
| Quantization noise | ADC resolution limit | 24-bit ADC, averaging improves by √N |

## Chopping Sequence

The `chop()` function minimizes charge injection:
1. Pulse CI_TMUXSEL HIGH
2. Toggle TMUXSEL state
3. Pulse CI_TMUXSEL LOW

## POST (Power-On Self-Test)

The firmware includes a POST that runs at startup to verify hardware functionality.

### Configuration

```cpp
static constexpr bool POST_HALT_ON_FAIL = true;  // Set false to continue despite failures
```

### Tests

| Test | Function | Pass Criteria |
|------|----------|---------------|
| ADC Comm | `postTestAdcComm()` | CONFIG1 register write/readback matches |
| DAC Comm | `postTestDacComm()` | Control register readback is valid (not 0xFFFFFF) |
| DRDY Timing | `postTestDrdyTiming()` | DRDY rate = 3906 Hz ±10% |
| Signal Chain | `postTestSignalChain()` | ADC reading < 50% full scale with DAC=0 |
| Polarity | `postTestPolarity()` | Increasing DAC decreases ADC (correct gain sign) |
| Chop Symmetry | `postTestChopSymmetry()` | Phase1 + Phase2 residual < 10% of magnitude |
| Input Mux | `postTestInputMux()` | GND input reads near zero |
| References | `postTestReferences()` | VrefRaw reading near zero when DAC at 5V |
| Zero Cal | `postTestZero()` | Chopped GND measurement within ±1 µV (configurable) |

### Example Output

```
========== POST (Power-On Self-Test) ==========
POST: ADC Comm... PASS (DEV_ID=0x0)
POST: DAC Comm... PASS (ctrl=0x400002)
POST: DRDY Timing... PASS (3906.2 Hz)
POST: Signal Chain... PASS (ADC=12345)
POST: Polarity... PASS (delta=-1234567)
POST: Chop Symmetry... PASS (P1=500000, P2=-500100, sum=-100)
POST: Input Mux... PASS (GND reading=1234)
POST: Reference... PASS (VrefRaw=50000)
POST: Zero Cal... PASS (zero=12.3 nV, threshold=±1000 nV)
================================================
POST: ALL TESTS PASSED
================================================
```

## Constraints

- **Input must be stable DC** - chopping assumes unchanging input during cycle
- **Low source impedance only** - not suitable for Kelvin-Varley dividers or high-Z sources
- **DAC tracking automatic** - binary search adjusts DAC on overflow detection

## References

- AD8428 datasheet: https://www.analog.com/media/en/technical-documentation/data-sheets/AD8428.PDF
- TMUX7234 datasheet: https://www.ti.com/lit/ds/symlink/tmux7234.pdf
- ADS127L11 datasheet: https://www.ti.com/lit/ds/symlink/ads127l11.pdf
- AD5760 datasheet: https://www.analog.com/media/en/technical-documentation/data-sheets/AD5760.pdf
- Low-noise protection: https://www.analog.com/media/en/technical-documentation/technical-articles/55798267SEN5247e.pdf
- DAC as reference: https://www.ti.com/lit/ta/sszt187/sszt187.pdf
- Kalman filter intro: https://www.cs.unc.edu/~welch/media/pdf/kalman_intro.pdf
- AN-1157 Kalman: https://www.analog.com/media/en/technical-documentation/application-notes/AN-1157.pdf
