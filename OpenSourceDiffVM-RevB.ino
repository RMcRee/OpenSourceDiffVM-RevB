/*
 * SPDX-License-Identifier: CERN-OHL-P-2.0
 * SPDX-FileCopyrightText: Copyright 2025, 2026 Randall K McRee
 */

/*
  Teensy 4.x + ADS127L11 + DAC + TMUX chopped measurement

  Pin mapping:
   - Teensy SPI pins: SCK=13, MOSI=11, MISO=12 (default SPI)
   - ADC CS = Teensy pin 41
   - DAC CS = Teensy pin 10
   - TMUX_EN = Teensy pin 4
   - ADC DRDY = Teensy pin 15
   - External ADC clock = 25.0 MHz (fMOD = fCLK/2)
   - VREF = 2.500 V
   - Mux cycle around 400 Hz => 1.25 ms high, 1.25 ms low
*/

#include <Arduino.h>
#include <SPI.h>
#include <EEPROM.h>
#include <LittleFS.h>
#include "AD5760.h"
#include "CalVerify.h"

// ---------------- Forward Declarations ----------------
// Types must be declared before Arduino auto-generates function prototypes
// (prototypes are inserted after the last #include, which is <limits> below)
struct HalfCycleResult;  // defined after LowerMoments
class LowerMoments;
class ScopedInstrumentState;
class InputMuxDriver;
class DividerMuxDriver;
class ChopperDriver;
class DacDriver;
class AdcDriver;
class DacCalibrationTable;
class CalibrationStore;
class IOutputFormatter;
struct MeasurementData;
struct SavedConfig;
enum class InputChannel : uint8_t;
enum class DividerRatio : uint8_t;

// ---------------- Output Verbosity Flags ----------------
// Set to true to include the drift and Allan deviation lines in human-readable output.
static constexpr bool SHOW_DRIFT = false;
static constexpr bool SHOW_ADEV  = false;

// ---------------- POST Configuration ----------------
// Set to true to halt on POST failure, false to continue with warning
static constexpr bool POST_HALT_ON_FAIL = false;

// Zero calibration test threshold (in nanovolts at input)
// With preamp gain of 2000 and ADC LSB of ~298nV, this translates to ADC counts.
// Loose bound: chain has natural ~250 µV RTI residual offset (chopping doesn't fully cancel
// every error term; observed ~248 µV on a healthy unit). Auto-zero compensates this in normal
// operation. POST threshold catches "totally broken" not "needs auto-zero".
static constexpr double POST_ZERO_THRESHOLD_NV = 500000.0;  // nV (500 µV)

// POST result structure
struct PostResult {
  bool adc_comm;
  bool dac_comm;
  bool drdy_timing;
  bool pin_id;
  bool signal_chain;
  bool polarity;
  bool chop_symmetry;
  bool input_mux;
  bool references;
  bool zero_cal;

  bool allPassed() const {
    return adc_comm && dac_comm && drdy_timing && pin_id &&
           signal_chain && polarity && chop_symmetry &&
           input_mux && references && zero_cal;
  }
};

// ---------------- Pins ----------------
static constexpr uint8_t PIN_CS_ADC     = 41;  // ADC chip select
static constexpr uint8_t PIN_CS_DAC     = 10;  // DAC chip select
static constexpr uint8_t PIN_DAC_RESET  = 25;  // DAC reset
static constexpr uint8_t PIN_DAC_CLEAR  = 24;  // DAC clear
static constexpr uint8_t PIN_ADC_DRDY   = 15;  // ADC DRDY (updated)
static constexpr uint8_t PIN_ADC_START  = 17;  // ADC START (LOW = idle)
static constexpr uint8_t PIN_ADC_RESET  = 40;  // ADC RESET (active low; HIGH = running)
static constexpr uint8_t PIN_CLK_ADC    = 9;   // ADC clock
static constexpr uint8_t PIN_TMUX_EN    = 4;   // TMUX enable
static constexpr uint8_t PIN_TMUXSEL    = 2;   // Controls the main DAC/Vx switches (Switches A and B)
static constexpr uint8_t PIN_CI_TMUXSEL = 3;   // Controls the common-mode voltage switches (Switches C and D)

// Input multiplexer (MUX36S08) - selects between Vx, GND, VrefRaw, ranges
static constexpr uint8_t PIN_INMUX_A0   = 35;  // Input mux address bit 0
static constexpr uint8_t PIN_INMUX_A1   = 34;  // Input mux address bit 1
static constexpr uint8_t PIN_INMUX_A2   = 33;  // Input mux address bit 2
// MUX36D08 EN hardwired to VDD (active high, always enabled) - no GPIO needed

// HV Divider tap select (MUX36D04 dual 4:1 mux, EN hardwired ON)
static constexpr uint8_t PIN_DIVMUX_A0  = 23;  // MUX36D04 A0: divider tap address bit 0
static constexpr uint8_t PIN_DIVMUX_A1  = 22;  // MUX36D04 A1: divider tap address bit 1

// ---------------- Timing ----------------
// Nominal at F_CPU=600 MHz; updated at runtime by calibrateAdcClock() from a
// measured DRDY period. Required because analogWriteFrequency() on PIN_CLK_ADC
// derives from a FlexPWM peripheral clock that scales with F_CPU and snaps to
// integer dividers — e.g. a request for 25 MHz lands on ~17.6 MHz at F_CPU=528.
static float              ADC_FCLK_HZ       = 25'000'000.0f;   // external clock (nominal)
static float              ADC_FMOD_HZ       = 12'500'000.0f;   // fMOD = fCLK/2 (nominal)
static constexpr uint32_t SETTLE_US         = 600;             // generic post-event guard (mux changes, POST tests)
static constexpr uint32_t CHOP_SETTLE_US    = 10000;           // post-chop-edge settle: preamp + TMUX + AAF (τ≈56ms after 47nF cap, was 21nF)
static constexpr uint8_t  DISCARD_SAMPLES   = 0;               // sinc4+sinc1 settles in one DRDY (sinc1 stage primes immediately)
static constexpr uint8_t  MAX_SAMPLES       = 20;              // max ADC readings per each chop phase
static constexpr int      CI_PRE_EDGE_US    = 50;              // CI cancel switches HIGH before TMUXSEL flip
static constexpr int      CI_POST_EDGE_US   = 100;             // CI cancel switches HIGH after TMUXSEL flip
uint8_t                   g_good_samples    = 14;              // samples to average per half-cycle; runtime-tunable via `samples N`

// ---------------- Demodulation Outlier Rejection (Pooled-MAD) ----------------
// Pair-diffs from the last DEMOD_POOL_N cycles are kept in a ring buffer.
// Each demodulate() call: push this cycle's diffs, recompute pooled median +
// MAD, threshold = max(k·1.4826·MAD, FLOOR). Pooling stabilises the threshold
// across cycles so the rejection bias is a constant offset (cancels in (A-B)/2)
// instead of a wandering one (which injects flicker at long ADEV τ).
// Runtime-tunable via 'drj <k>' command. k=10000 effectively disables rejection.
static int32_t           g_demodRejKx10    = 80;    // k = 8.0  (×10 for integer math)
static constexpr int32_t DEMOD_REJ_FLOOR  = 32;     // ADC counts, ~10× per-pair RMS noise at OSR=12800
static constexpr int     DEMOD_POOL_N      = 256;   // pair-diff ring buffer depth
static constexpr int     DEMOD_POOL_MIN    = 64;    // skip rejection until this many entries collected
static int32_t           g_diffPool[DEMOD_POOL_N];
static int               g_diffPoolHead    = 0;
static int               g_diffPoolFilled  = 0;

// Flush pool history. MUST be called whenever the signal level steps abruptly
// (DAC change, channel switch, range change) — otherwise the lagging pool
// median biases the next ~DEMOD_POOL_N pair-diffs toward the previous level.
static inline void demodPoolReset() {
  g_diffPoolHead = 0;
  g_diffPoolFilled = 0;
}
static uint64_t          g_demodRejTotal   = 0;
static uint64_t          g_demodPairsTotal = 0;
static uint32_t          g_demodRejSinceOutput   = 0;
static uint32_t          g_demodPairsSinceOutput = 0;

// ---------------- DAC Filter Settling ----------------
// Second-order Bessel filter on DAC output requires settling after step changes.
// From measured step response:
//   - Overshoot peak: ~0.45% at t≈107ms
//   - Undershoot trough: ~0.002% at t≈220ms
//   - 10 ppm settling: t≈250ms for full-scale step
// Settling time scales with sqrt(step_size) for constant absolute error.
static constexpr uint32_t DAC_SETTLE_MIN_MS = 80;   // Minimum settling (small steps, ~1% relative)
static constexpr uint32_t DAC_SETTLE_MAX_MS = 300;  // Maximum settling (full-scale, ~10 ppm relative)

// ---------------- Input Mux Configuration ----------------
// MUX36S08 settling time: ~100ns switching + signal path settling
// Use conservative settling for precision measurements
static constexpr uint32_t INMUX_SETTLE_US = 100;  // Input mux settling time (µs)

// Input channel enumeration (directly maps to MUX36S08 address lines)
enum class InputChannel : uint8_t {
  Vx1     = 0,  // S1: Unknown input 1 (normal measurement, ±5V)
  GND     = 1,  // S2: Ground reference (zero calibration)
  VrefRaw = 2,  // S3: Raw (pre-filter) ADR1001 average (J3) for drift compensation
  Vx2     = 3,  // S4: Unknown input 2
  Vx3     = 4,  // S5: Unknown input 3
  HVDiv   = 5,  // S6: HV divider output (ratio selected by MUX36D04)
  Vx4     = 6,  // S7: Unknown input 4
  Vx5     = 7,  // S8: Unknown input 5
};

// ---------------- HV Divider Ratio Selection (MUX36D04) ----------------
// Caddock 1776-C4815 serial-parallel arrangement with MUX36D04 tap selection
// The 1:10 divider is hardwired always in circuit for safety (no HV transients).
// MUX switches parallel resistances to change ratio; address 3 connects to GND.
enum class DividerRatio : uint8_t {
  Div10   = 0,  // Mux bypassed: 1:10 ratio only (hardwired), ±50V range
  Div100  = 1,  // CD4-AB4 bridged: 100k arm || 1M, ±500V range
  Div1000 = 2,  // CD5-AB5 bridged: 10k arm || 1M, ±5000V range
  GND     = 3,  // Ground connection for zero calibration
};

// ---------------- Voltage Divider Ratios ----------------
// Calculated ratios for Caddock 1776-C4815 serial-parallel divider
// Segments: 10M, 1.1111M, 101.01K, 10.01K (4 dividers as 2 parallel pairs in series)
// MUX path = Rseg/2 + Rseg/2 + 250Ω; base 1M arm = 1.1111M (always in circuit)
// Runtime-mutable for in-situ calibration; defaults match calculated values.
// Use 'cal set div10/div100/div1000 <v>' to adjust; 'cal factory' to reset.
static constexpr double DIVIDER_RATIO_10_DEFAULT   = 10.0;
static constexpr double DIVIDER_RATIO_100_DEFAULT  = 108.76;
static constexpr double DIVIDER_RATIO_1000_DEFAULT = 984.65;
static double DIVIDER_RATIO_10   = DIVIDER_RATIO_10_DEFAULT;    // ±50V: mux bypassed
static double DIVIDER_RATIO_100  = DIVIDER_RATIO_100_DEFAULT;   // ±500V: 1.1111M || 101.26K
static double DIVIDER_RATIO_1000 = DIVIDER_RATIO_1000_DEFAULT;  // ±5kV: 1.1111M || 10.26K

// ---------------- Precision Voltage Constants (64-bit double) ----------------
// ADC: ADS127L11, 24-bit, differential, VREF = 2.500V nominal.
// The VREF pin is driven through a 100Ω series resistor (added to handle capacitive load),
// which introduces a small voltage drop proportional to reference input current.
// Measure the actual voltage at the ADS127L11 VREF pin with a precision meter and
// set it with 'cal set adcvref <V>' to remove this scale error.
static constexpr double ADC_VREF_DEFAULT = 2.500;
static double ADC_VREF     = ADC_VREF_DEFAULT;       // runtime-calibratable; use 'cal set adcvref'
static constexpr double ADC_FSR_CODES   = 16777216.0; // 2^24 full scale codes
static double ADC_LSB_V    = (2.0 * ADC_VREF_DEFAULT) / ADC_FSR_CODES;  // updated when ADC_VREF changes

// ---------------- Overflow Detection ----------------
// 24-bit ADC full scale: ±8,388,607. Use 95% threshold for overflow detection.
static constexpr int32_t ADC_OVERFLOW_POS   =  0.95 * ADC_FSR_CODES/2;      // ~95% of 0x7FFFFF
static constexpr int32_t ADC_OVERFLOW_NEG   = -1.00 * ADC_OVERFLOW_POS;     // ~95% of -0x800000

// DAC: AD5760, 16-bit, two's complement in 2x mode, VREFP=5V, VREFN=0V
// Output range: ±5V (with 2x gain), codes 0x8000 (-32768) to 0x7FFF (+32767)
static constexpr double DAC_VREF        = 5.0;                           // DAC full-scale voltage (V)
static constexpr double DAC_FSR_CODES   = 65536.0;                       // 2^16 codes
static constexpr double DAC_LSB_V       = (2.0 * DAC_VREF) / DAC_FSR_CODES;  // ~152.6 µV/LSB

// Preamp: 4× AD8428 (or INA848) in parallel, total gain = 2000
// Runtime-mutable for in-situ calibration. Use 'cal set gain <v>' to adjust.
static constexpr double PREAMP_GAIN_DEFAULT = 2000.0;
static double PREAMP_GAIN = PREAMP_GAIN_DEFAULT;

// Cache for 'cal gain' — updated by outputMeasurement each cycle.
// g_calLastVoltage is ewmaV when EWMA is initialized, else vxCorrected — the same
// value the user sees as the primary reading.
static double       g_calLastVoltage = NAN;
static int16_t      g_calLastDacCode = 0;
static InputChannel g_calLastChannel = InputChannel::Vx1;

// OPA828 buffer offset at the HV divider output node (volts, measured at buffer output).
// Chopping does NOT cancel this offset — it must be calibrated out explicitly.
// Apply with 'cal set opa828 <v>' after measuring with DAC tracking at GND divider output.
static constexpr double OPA828_OFFSET_DEFAULT = 0.0;
static double OPA828_OFFSET = OPA828_OFFSET_DEFAULT;

// ---------------- Reference Drift Compensation ----------------
// The DAC reference comes from a filtered average of three ADR1001 references.
// Filter: Sallen-Key 2nd order, fc=0.3Hz, Q=0.566
// The raw (pre-filter) average is available at J3 for drift prediction.
// By measuring J3 vs the DAC reference (TP1), we can predict and correct
// for temperature-induced reference drift in real-time.

static constexpr double NOMINAL_REF_V = 5.0;           // Nominal reference voltage
static constexpr int16_t REF_MEASURE_DAC_CODE = 32764; // DAC code for ~5V (14-bit aligned)
static constexpr uint32_t REF_SAMPLE_INTERVAL = 385;   // Sample ref every ~385 chop cycles (~1 second)

// Effective time constant of reference filter (from step response 63% point)
static constexpr double REF_FILTER_TAU_EFF = 1.05;     // seconds

// Step response lookup table for fc=0.3Hz, Q=0.566 filter
// Sampled at 0.1s intervals from 0 to 8 seconds (81 points)
// Used to predict filter output evolution between measurements
static const float REF_STEP_RESPONSE[] = {
  0.000f, 0.019f, 0.066f, 0.132f, 0.208f, 0.289f, 0.371f, 0.450f,  // 0.0-0.7s
  0.523f, 0.592f, 0.654f, 0.711f, 0.762f, 0.808f, 0.848f, 0.884f,  // 0.8-1.5s
  0.914f, 0.940f, 0.962f, 0.979f, 0.992f, 1.001f, 1.008f, 1.013f,  // 1.6-2.3s
  1.016f, 1.018f, 1.019f, 1.019f, 1.018f, 1.016f, 1.014f, 1.012f,  // 2.4-3.1s
  1.010f, 1.008f, 1.006f, 1.004f, 1.003f, 1.002f, 1.001f, 1.000f,  // 3.2-3.9s
  1.000f, 1.000f, 1.000f, 1.000f, 1.000f, 1.000f, 1.000f, 1.000f,  // 4.0-4.7s
  1.000f, 1.000f, 1.000f, 1.000f, 1.000f, 1.000f, 1.000f, 1.000f,  // 4.8-5.5s
  1.000f, 1.000f, 1.000f, 1.000f, 1.000f, 1.000f, 1.000f, 1.000f,  // 5.6-6.3s
  1.000f, 1.000f, 1.000f, 1.000f, 1.000f, 1.000f, 1.000f, 1.000f,  // 6.4-7.1s
  1.000f, 1.000f, 1.000f, 1.000f, 1.000f, 1.000f, 1.000f, 1.000f,  // 7.2-7.9s
  1.000f                                                           // 8.0s
};
static constexpr float REF_STEP_DT = 0.1f;             // Step response sample interval (seconds)
static constexpr int REF_STEP_LEN = 81;                // Number of step response samples

// Filter error history for drift rate estimation
struct FilterSample {
  double error;        // Measured filter error (J3 - TP1) in volts
  uint32_t timestamp;  // Measurement time (millis())
};

static constexpr int FILTER_HISTORY_SIZE = 8;          // History buffer size (~8 seconds)
static constexpr int REF_MEASURE_ITERATIONS = 10;      // Chop cycles to average for each ref measurement
static FilterSample filterHistory[FILTER_HISTORY_SIZE];
static int filterHistoryIdx = 0;
static double estimatedDriftRate = 0.0;                // Reference drift rate (V/s)
static uint32_t refSampleCounter = 0;                  // Counter for periodic sampling
static bool refTrackingInitialized = false;            // True after first reference measurement

// ---------------- DAC Calibration Build Constants ----------------
static constexpr int CAL_BUILD_CYCLES  = 5;   // Chop cycles per table entry during auto build
static constexpr int CAL_POINT_CYCLES  = 20;  // Chop cycles per cal point capture
static constexpr int CAL_BUILD_WINDOW  = 2;   // Table entries swept each side of anchor (±2 = 5 total)
static constexpr int AUTOZERO_CYCLES   = 20;  // Chop cycles per auto-zero measurement

// ---------------- Debug Flags ----------------
// Toggle with 'debug on|off' command. Off by default — keeps normal operation quiet.
// When on, binary search prints per-iteration ADC/DAC traces (helpful when
// chasing settling or convergence problems).
static bool g_debugDac = false;

// ---------------- DAC Calibration Table ----------------
// Encapsulates the 16384-point calibration table for DAC INL correction.
// Index: (dacCode + 32768) >> 2 gives 14-bit index (0..16383)
// Value: actual voltage at that code (measured against ADR1001 references)
class DacCalibrationTable {
public:
  static constexpr size_t TABLE_SIZE = 16384;

  DacCalibrationTable() { initNominal(); }

  void initNominal() {
    for (size_t i = 0; i < TABLE_SIZE; i++) {
      int16_t code = (int16_t)((i << 2) - 32768);
      table_[i] = (double)code * DAC_LSB_V;
    }
    maxCodeV_ = 32767.0 * DAC_LSB_V;   // nominal V at absolute-max code 32767
    valid_ = false;
  }

  double codeToVoltage(int16_t code) const {
    // Absolute-max code 32767 has its own cal slot — used to read accurately near +5 V.
    if (code == 32767) {
      return valid_ ? maxCodeV_ : (32767.0 * DAC_LSB_V);
    }
    if (!valid_) {
      return (double)code * DAC_LSB_V;
    }
    uint16_t u = (uint16_t)(code + 32768);
    uint16_t idx = u >> 2;
    if (idx >= TABLE_SIZE - 1) {
      // Codes 32764..32766 sit between the last 14-bit-aligned slot (32764) and the
      // separately calibrated maxCodeV_ (code 32767); interpolate over the 3-LSB gap.
      double v0 = table_[TABLE_SIZE - 1];
      double v1 = maxCodeV_;
      int32_t step = (int32_t)code - 32764;   // 0..3
      return v0 + (v1 - v0) * (double)step / 3.0;
    }
    uint8_t frac = u & 0x03;
    double v0 = table_[idx];
    double v1 = table_[idx + 1];
    return v0 + (v1 - v0) * (double)frac / 4.0;
  }

  void setPoint(int16_t code, double measuredVoltage) {
    if (code == 32767) {
      maxCodeV_ = measuredVoltage;
      return;
    }
    uint16_t idx = ((uint16_t)(code + 32768)) >> 2;
    if (idx < TABLE_SIZE) {
      table_[idx] = measuredVoltage;
    }
  }

  // Restore a slot to its nominal value so it is no longer a cardinal.
  // Returns true if the slot had been overwritten from nominal.
  bool clearCardinalPoint(int16_t code) {
    if (code == 32767) {
      double nominal = 32767.0 * DAC_LSB_V;
      bool was = (maxCodeV_ != nominal);
      maxCodeV_ = nominal;
      return was;
    }
    uint16_t idx = ((uint16_t)(code + 32768)) >> 2;
    if (idx >= TABLE_SIZE) return false;
    double nominal = (double)code * DAC_LSB_V;
    bool was = (table_[idx] != nominal);
    table_[idx] = nominal;
    return was;
  }

  void markValid() { valid_ = true; }
  bool isValid() const { return valid_; }

  double maxCodeVoltage() const { return maxCodeV_; }
  void setMaxCodeVoltage(double v) { maxCodeV_ = v; }

  // Slot is "cardinal" (operator-calibrated) if it deviates from nominal.
  // Used for piecewise-linear interpolation of uncalibrated slots.
  // Detection-by-deviation is robust as long as setPoint() was called with a
  // measured voltage (essentially never bit-exact equal to k·DAC_LSB_V).
  bool isCardinalAt(size_t idx) const {
    if (idx >= TABLE_SIZE) return false;
    int16_t code = (int16_t)((idx << 2) - 32768);
    double nominal = (double)code * DAC_LSB_V;
    return table_[idx] != nominal;
  }

  // Linearly interpolate non-cardinal slots between adjacent cardinals.
  // Outside the cardinal range, extrapolate using the slope of the nearest pair.
  // Returns the number of cardinal points found. Idempotent only in the trivial
  // sense — after running, every slot is non-nominal so re-running is a no-op
  // (but harmless: the existing values are preserved).
  size_t interpolateGaps() {
    static constexpr size_t MAX_CARDINALS = 256;
    uint16_t card[MAX_CARDINALS];
    size_t n = 0;
    for (size_t i = 0; i < TABLE_SIZE; i++) {
      if (isCardinalAt(i)) {
        if (n < MAX_CARDINALS) card[n] = (uint16_t)i;
        n++;
      }
    }
    if (n < 2 || n > MAX_CARDINALS) return n;

    size_t kPos = 0;
    for (size_t i = 0; i < TABLE_SIZE; i++) {
      while (kPos + 1 < n && card[kPos + 1] <= i) kPos++;
      if (kPos < n && card[kPos] == i) continue;  // cardinal slot, leave it

      size_t lo, hi;
      if (i < card[0])           { lo = card[0];     hi = card[1];     }   // left extrap
      else if (kPos == n - 1)    { lo = card[n - 2]; hi = card[n - 1]; }   // right extrap
      else                       { lo = card[kPos];  hi = card[kPos + 1]; } // interior

      double vLo = table_[lo];
      double vHi = table_[hi];
      double t = (double)((int32_t)i - (int32_t)lo) / (double)((int32_t)hi - (int32_t)lo);
      table_[i] = vLo + (vHi - vLo) * t;
    }
    return n;
  }

  // Linearly interpolate the slots strictly between two 14-bit-aligned cardinal
  // codes, leaving the endpoints unchanged. Used by the verify workflow to
  // smooth the table after each new Keithley-validated cardinal, and by the
  // `cal smooth` command to retroactively blend a saved table whose
  // verify-updated cardinals are stranded between stale cal-build-dac values.
  // Unlike interpolateGaps(), this does not depend on the cardinal-by-deviation
  // heuristic and is unaffected by MAX_CARDINALS — it operates on the exact
  // range you specify. Caller passes the codes in either order; this normalises.
  // Returns the number of slots overwritten (0 if codes are equal or adjacent).
  size_t interpolateBetweenCodes(int16_t codeLo, int16_t codeHi) {
    if (codeLo == codeHi) return 0;
    if (codeLo > codeHi) { int16_t t = codeLo; codeLo = codeHi; codeHi = t; }
    // Skip the +32767 absolute-max slot — it lives outside the 14-bit grid and
    // its value is stored in maxCodeV_, not in table_[].
    if (codeLo == 32767 || codeHi == 32767) return 0;
    size_t idxLo = ((uint16_t)(codeLo + 32768)) >> 2;
    size_t idxHi = ((uint16_t)(codeHi + 32768)) >> 2;
    if (idxLo >= TABLE_SIZE || idxHi >= TABLE_SIZE) return 0;
    if (idxHi - idxLo < 2) return 0;  // nothing strictly between
    double vLo = table_[idxLo];
    double vHi = table_[idxHi];
    double denom = (double)((int32_t)idxHi - (int32_t)idxLo);
    size_t n = 0;
    for (size_t i = idxLo + 1; i < idxHi; i++) {
      double t = (double)((int32_t)i - (int32_t)idxLo) / denom;
      table_[i] = vLo + (vHi - vLo) * t;
      n++;
    }
    return n;
  }

  // Print the cardinal points (and their residual vs nominal) for inspection.
  void printCardinals() const {
    size_t n = 0;
    Serial.println("    idx     code        voltage_V       resid_uV");
    for (size_t i = 0; i < TABLE_SIZE; i++) {
      if (isCardinalAt(i)) {
        int16_t code = (int16_t)((i << 2) - 32768);
        double nominal = (double)code * DAC_LSB_V;
        double residual_uV = (table_[i] - nominal) * 1e6;
        char buf[80];
        snprintf(buf, sizeof(buf), "  %6u  %7d   %14.7f   %+9.2f",
                 (unsigned)i, (int)code, table_[i], residual_uV);
        Serial.println(buf);
        n++;
      }
    }
    Serial.print("  total cardinals: "); Serial.println((unsigned long)n);
  }

  // Raw table access for CalibrationStore flash I/O
  double* rawTable() { return table_; }
  const double* rawTable() const { return table_; }
  friend class CalibrationStore;

private:
  double table_[TABLE_SIZE];
  double maxCodeV_;       // calibration for absolute-max code 32767 (outside the 14-bit-aligned grid)
  bool valid_ = false;
};

static DacCalibrationTable dacCalTable;

// Defined in "Voltage Divider Scale Factors" section; used by computeInputVoltage/Uncertainty below.
double getInputDividerRatio(InputChannel channel);

// ---------------- Precision Voltage Computation ----------------
/**
 * Compute the measured input voltage using 64-bit double precision.
 *
 * The measurement equation is:
 *   Vx = (DAC_voltage + preamp_delta) × divider_ratio
 *
 * Where:
 *   DAC_voltage  = DAC output (with optional INL calibration lookup)
 *   preamp_delta = (ADC_average × ADC_LSB) / PREAMP_GAIN
 *   divider_ratio = input voltage divider ratio (1.0 for ±5V range)
 *
 * @param adcMean     Mean ADC reading from LowerMoments (in counts)
 * @param dacCode     Current DAC code
 * @param channel     Current input channel (for divider ratio)
 * @return            Measured voltage at the input terminal (in volts)
 */
double computeInputVoltage(double adcMean, int16_t dacCode, InputChannel channel) {
  // Step 1: Get DAC voltage (with optional calibration lookup)
  double dacVoltage = dacCalTable.codeToVoltage(dacCode);

  // Step 2: Convert ADC counts to voltage at preamp input
  // ADC reads (Vx - DAC) × PREAMP_GAIN, so:
  // preamp_delta = (ADC_counts × ADC_LSB_V) / PREAMP_GAIN
  double preampDelta = (adcMean * ADC_LSB_V) / PREAMP_GAIN;

  // Step 3: Combine DAC and preamp contributions
  // Vx_at_preamp = DAC + preamp_delta
  double vxAtPreamp = dacVoltage + preampDelta;

  // Step 4: Remove OPA828 buffer offset (HVDiv channel only).
  // The OPA828 sits before the TMUX switches so chopping does not cancel it.
  if (channel == InputChannel::HVDiv) {
    vxAtPreamp -= OPA828_OFFSET;
  }

  // Step 5: Scale by voltage divider ratio (if using extended range)
  double dividerRatio = getInputDividerRatio(channel);
  double vxInput = vxAtPreamp * dividerRatio;

  return vxInput;
}

/**
 * Compute the measurement uncertainty (standard deviation) in volts.
 *
 * @param adcStdDev   Standard deviation of ADC readings (in counts)
 * @param channel     Current input channel (for divider ratio)
 * @return            Uncertainty in volts at the input terminal
 */
double computeInputUncertainty(double adcStdDev, InputChannel channel) {
  // Convert ADC standard deviation to voltage at preamp input
  double uncertaintyAtPreamp = (adcStdDev * ADC_LSB_V) / PREAMP_GAIN;

  // Scale by voltage divider ratio
  double dividerRatio = getInputDividerRatio(channel);
  return uncertaintyAtPreamp * dividerRatio;
}

/**
 * Format voltage with appropriate SI prefix for readability.
 * Uses 64-bit precision for computation, formats to specified digits.
 *
 * @param voltage     Voltage value in volts
 * @param buf         Output buffer
 * @param bufLen      Buffer length
 * @param sigDigits   Significant digits to display (default 9 for 8.5-digit meter)
 * @return            Number of characters written
 */
int formatVoltage(double voltage, char* buf, size_t bufLen, int sigDigits = 9) {
  if (!buf || bufLen == 0) return 0;

  double absV = fabs(voltage);

  // Choose appropriate unit prefix
  const char* unit;
  double scale;

  if (absV >= 1.0) {
    unit = "V";
    scale = 1.0;
  } else if (absV >= 1e-3) {
    unit = "mV";
    scale = 1e3;
  } else if (absV >= 1e-6) {
    unit = "µV";
    scale = 1e6;
  } else if (absV >= 1e-9) {
    unit = "nV";
    scale = 1e9;
  } else {
    unit = "pV";
    scale = 1e12;
  }

  // Format with specified precision
  // For 9 significant digits, we need variable decimal places based on magnitude
  double scaledV = voltage * scale;
  int intDigits = (absV * scale >= 1.0) ? (int)log10(absV * scale) + 1 : 1;
  int decimals = sigDigits - intDigits;
  if (decimals < 0) decimals = 0;

  return snprintf(buf, bufLen, "%.*f %s", decimals, scaledV, unit);
}

// Effective OSR observed at runtime. Nominal-comment-claimed value for
// CONFIG3=0x1A was 12800, but empirically the ADS127L11 yields 14080 at
// fCLK=25 MHz and 12800 at fCLK=22 MHz with the same register — the filter
// code apparently maps to different total OSR depending on the input clock.
// calibrateAdcClock() derives the true OSR from PWM-register-implied fMOD and
// the measured DRDY rate, so any downstream display of "OSR=..." reflects
// what the ADC is actually doing.
static float              OSR_TOTAL         = 14080.0f;
// Nominal sample timing; updated at runtime by calibrateAdcClock(). See note
// on ADC_FCLK_HZ above for why these can't be constexpr in this firmware.
static float              EST_FSPS          = 976.5625f;   // = 12.5 MHz / 12800 (nominal)
static float              EST_TS_US         = 1024.0f;     // = 1e6 / EST_FSPS (nominal)

// ---------------- SPI ----------------
static constexpr uint32_t SPI_HZ = 5'000'000;
static constexpr uint8_t  ADC_SPI_MODE = SPI_MODE1; // ADS127L11: CPOL=0 CPHA=1

SPISettings adcSpiSettings(SPI_HZ, MSBFIRST, ADC_SPI_MODE);
// DAC uses AD5760 class (AD5760.h) in bit-bang shared mode: pins 11/12/13 switch
// between GPIO (during DAC transfers) and LPSPI4 ALT3 (for ADC use) per-transfer.

// ---------------- ADS127L11 register map ----------------
static constexpr uint8_t REG_DEV_ID  = 0x00;  // Device ID register
static constexpr uint8_t REG_REV     = 0x01;  // Revision
static constexpr uint8_t REG_STATUS  = 0x02;  // Status register
static constexpr uint8_t REG_CONTROL = 0x03;  // Control register
static constexpr uint8_t REG_MUX     = 0x04;  // Multiplexer register  MUX[1:0]
static constexpr uint8_t REG_CONFIG1 = 0x05;  // xx, ref_rng, inp_rng, vcm, refp_buf, xx, ainp_buf, ainn_buf
static constexpr uint8_t REG_CONFIG2 = 0x06;  // ext_rng, xx, sdo_mode, start_mode[1:0], speed_mode, stby_mode, pwdn
static constexpr uint8_t REG_CONFIG3 = 0x07;  // delay[2:0], filter[4:0]
static constexpr uint8_t REG_CONFIG4 = 0x08;  // clk_sel, clk_div, out_drv, xx, data, spi_crc, reg_crc, status

// Expected ADS127L11 device ID (from datasheet)
static constexpr uint8_t ADS127L11_DEV_ID = 0x00;  // DEV_ID[7:2]=0, verify communication works

// CONFIG1 bit 3: REFP_BUF — enable the internal reference input buffer.
// Recommended when a series resistor is placed between the reference source and the VREF pin
// (e.g., to handle capacitive loading on the ADC board). The buffer presents high impedance
// to the reference source, eliminating the voltage drop across the series resistor.
static bool ADC_REFBUF_ENABLE = true;

// ---------------- ADS127L11 SPI opcodes ----------------
static constexpr uint8_t OPCODE_RREG = 0x40;
static constexpr uint8_t OPCODE_WREG = 0x80;

static constexpr uint8_t CMD_START   = 0x08;
static constexpr uint8_t CMD_STOP    = 0x0A;

// ---------------- Helpers ----------------
static inline void csLow(uint8_t pin)  { digitalWriteFast(pin, LOW); }
static inline void csHigh(uint8_t pin) { digitalWriteFast(pin, HIGH); }

// ================== Hardware Driver Classes ==================
// Each driver owns its GPIO pins and hardware state.
// Method bodies are defined out-of-line where the free functions used to be.

// -------- InputMuxDriver (MUX36S08) --------
class InputMuxDriver {
public:
  void begin();
  void select(InputChannel ch);
  InputChannel current() const { return current_; }
private:
  InputChannel current_ = InputChannel::Vx1;
};
static InputMuxDriver inputMux;

// -------- DividerMuxDriver (MUX36D04) --------
class DividerMuxDriver {
public:
  void begin();
  void select(DividerRatio ratio);
  DividerRatio current() const { return current_; }
private:
  DividerRatio current_ = DividerRatio::Div10;
};
static DividerMuxDriver divMux;

// -------- ChopperDriver (TMUX7234) --------
class ChopperDriver {
public:
  void begin();
  void toggle();
  bool state() const { return state_; }
  void setState(bool s) { state_ = s; digitalWriteFast(PIN_TMUXSEL, s ? HIGH : LOW); }
private:
  volatile bool state_ = false;
};
static ChopperDriver chopper;

// -------- DacDriver — wraps AD5760, adds filter-settling logic --------
class DacDriver {
public:
  void begin();
  void setCode(int16_t code);
  void setCodeFast(int16_t code);  // No filter settling — for binary search only
  int16_t currentCode() const { return currentCode_; }
  uint32_t readControlRaw();  // POST: read control register via SDO

private:
  AD5760 _dac{PIN_CS_DAC, PIN_DAC_CLEAR, PIN_DAC_RESET, SPI};
  int16_t currentCode_ = 0;
  static uint32_t calculateSettleTime(int32_t stepSize);
};
static DacDriver dac;

// -------- AdcDriver (ADS127L11) --------
class AdcDriver {
public:
  void begin();
  void initAndConfigure();
  void writeReg(uint8_t addr, uint8_t val);
  uint8_t readReg(uint8_t addr);
  bool waitDrdy(uint32_t timeout_us = 5000);  // default safe up to OSR ~30000 (Ts ~2.4ms)
  bool readSample24(int32_t &sample);
  void start(); // start/stop control mode, per section 8.4.6.2
  void stop();
private:
  uint8_t xfer(uint8_t v);
};
static AdcDriver adc;

// ================== Per-channel user labels ==================
static constexpr int CHANNEL_LABEL_MAX = 15;
// One label per channel (indexed by InputChannel uint8_t value, 0-7).
// Empty string means "use default channel name".
static char channelLabels[8][CHANNEL_LABEL_MAX + 1] = {};

// ================== DividerMuxDriver method implementations ==================

void DividerMuxDriver::begin() {
  pinMode(PIN_DIVMUX_A0, OUTPUT);
  pinMode(PIN_DIVMUX_A1, OUTPUT);
  digitalWrite(PIN_DIVMUX_A0, LOW);
  digitalWrite(PIN_DIVMUX_A1, LOW);
  current_ = DividerRatio::Div10;
}

void DividerMuxDriver::select(DividerRatio ratio) {
  uint8_t addr = static_cast<uint8_t>(ratio);
  digitalWriteFast(PIN_DIVMUX_A0, (addr & 0x01) ? HIGH : LOW);
  digitalWriteFast(PIN_DIVMUX_A1, (addr & 0x02) ? HIGH : LOW);
  current_ = ratio;
  inputMux.select(InputChannel::HVDiv);
  delayMicroseconds(INMUX_SETTLE_US);
}

// ---------------- Voltage Divider Scale Factors ----------------
double getInputDividerRatio(InputChannel channel) {
  if (channel == InputChannel::HVDiv) {
    switch (divMux.current()) {
      case DividerRatio::Div10:   return DIVIDER_RATIO_10;
      case DividerRatio::Div100:  return DIVIDER_RATIO_100;
      case DividerRatio::Div1000: return DIVIDER_RATIO_1000;
      case DividerRatio::GND:     return DIVIDER_RATIO_10;
      default:                    return DIVIDER_RATIO_10;
    }
  }
  return 1.0;
}

const char* getDividerRatioName(DividerRatio ratio) {
  switch (ratio) {
    case DividerRatio::Div10:   return "÷10 (±50V)";
    case DividerRatio::Div100:  return "÷100 (±500V)";
    case DividerRatio::Div1000: return "÷1000 (±5kV)";
    case DividerRatio::GND:     return "GND (cal)";
    default:                    return "Unknown";
  }
}

//
// LowerMoments
// Statistical moments, mean, count, std deviation, aka Welford tracker.
// drop-in class for Teensy/Arduino (64-bit double supported)

#include <cmath>
#include <cfloat>
#include <limits>

class LowerMoments {
public:
  LowerMoments() { 	 
	clear();
  }

  // Add a sample and update mean/variance incrementally
  void accumulate(double x) {
    if (std::isnan(x) || std::isinf(x)) return;

    if (x < min_) min_ = x;
    if (x > max_) max_ = x;

    double n  = moments_[0];
    double n1 = n + 1.0;

    // Same math as the Java version:
    double delta = (moments_[1] - x) / n1;
    double d2    = delta * delta;
    double r1    = n / n1;

    moments_[2] += (1.0 + n) * d2;
    moments_[2] *= r1;

    moments_[1] -= delta;   // updates mean
    moments_[0]  = n1;      // updates count
  }

  double mean() const { return moments_[1]; }
  double count() const { return moments_[0]; }

  double variance() const {
    if (moments_[0] < 2.0) return std::numeric_limits<double>::quiet_NaN();
    return moments_[2] * moments_[0] / (moments_[0] - 1.0);
  }

  double standardDeviation() const {
    return std::sqrt(variance());
  }

  double getMin() const { return min_; }
  double getMax() const { return max_; }

  // Clear statistics
  void clear() {
    moments_[0] = moments_[1] = moments_[2] = 0.0;
    min_ = std::numeric_limits<double>::infinity();
    max_ = -std::numeric_limits<double>::infinity();
  }

  // Optional: format into a user-provided buffer (avoids heap/String)
  // Returns number of chars written (like snprintf)
  int toString(char* buf, size_t buflen) const {
    if (!buf || buflen == 0) return 0;
    return snprintf(buf, buflen, "%.12g sd=%.12g", mean(), standardDeviation());
  }

private:
  // moments_[0]=count (n), moments_[1]=mean, moments_[2]=2nd central moment accumulator
  double moments_[3];
  double min_;
  double max_;
};

// Adjusted EWMA — bias-corrected exponential moving average, equivalent to an N-point SMA
// during warm-up. Uses double precision numerator/denominator to avoid startup bias.
// The window parameter sets the equivalent simple-moving-average length.
class AdjustedEWMA {
public:
  AdjustedEWMA() { setWindow(10.0f); }
  explicit AdjustedEWMA(float windowEquivalent) { setWindow(windowEquivalent); }

  void setWindow(float windowEquivalent) {
    beta_ = 1.0f - 2.0f / (windowEquivalent + 1.0f);
    reset();
  }

  void reset() { numer_ = 0.0; denom_ = 0.0; initialized_ = false; }

  double update(double x) {
    if (!initialized_) {
      numer_ = x; denom_ = 1.0; initialized_ = true;
    } else {
      numer_ = x + (double)beta_ * numer_;
      denom_ = 1.0 + (double)beta_ * denom_;
    }
    return numer_ / denom_;
  }

  double value() const { return initialized_ ? numer_ / denom_ : 0.0; }
  bool isInitialized() const { return initialized_; }
  float window() const { return 2.0f / (1.0f - beta_) - 1.0f; }   // inverse of setWindow()

private:
  float  beta_;
  double numer_, denom_;
  bool   initialized_;
};

// HalfCycleResult. Signals overflow when gathering ADC samples
struct HalfCycleResult {
  bool overflow = false;
  int32_t overflowSample = 0;  // value of the sample that triggered overflow (only valid when overflow)
};

// ================== Allan Deviation (OADEV) Tracker ==================
//
// Real-time overlapping Allan deviation computation for voltage stability
// analysis. Stores a circular buffer of corrected voltage readings and
// computes OADEV at octave-spaced averaging times on demand.
//
// OADEV^2(m) = 1/(2 * pairs) * SUM_{i=0..N-2m} (y_bar_{i+m} - y_bar_i)^2
// where y_bar_i = (1/m) * SUM_{k=0}^{m-1} y_{i+k}, pairs = N - 2m + 1

class AllanDeviation {
public:
  static constexpr int MAX_READINGS = 4096;  // ~17 min at tau0=0.26s (32 KB)
  static constexpr int MAX_TAUS = 12;        // octave taus: m=1,2,4,...,2048

  AllanDeviation() { clear(); }

  void clear() {
    head_ = 0;
    count_ = 0;
  }

  void addReading(double voltage, float tempC) {
    readings_[head_] = voltage;
    temps_[head_] = tempC;
    head_ = (head_ + 1) % MAX_READINGS;
    if (count_ < MAX_READINGS) count_++;
  }

  int readingCount() const { return count_; }

  // Get OADEV for octave index k (tau = 2^k * tau0)
  // Returns NaN if insufficient data
  double getAdev(int octave) const {
    int m = 1 << octave;
    return computeForM(m);
  }

  double getTau(int octave, double tau0) const {
    return (double)(1 << octave) * tau0;
  }

  int numOctaves() const {
    int maxOctaves = 0;
    int m = 1;
    while (m <= count_ / 3 && maxOctaves < MAX_TAUS) {
      maxOctaves++;
      m *= 2;
    }
    return maxOctaves;
  }

  void printResults(double tau0) const {
    if (count_ < 3) {
      Serial.println("Need at least 3 readings for Allan deviation.");
      return;
    }
    Serial.print("Allan Deviation (");
    Serial.print(count_);
    Serial.print(" readings, tau0=");
    Serial.print(tau0, 4);
    Serial.print("s, OSR=");
    Serial.print((int)OSR_TOTAL);
    Serial.print(", samples=");
    Serial.print(g_good_samples);
    Serial.print(", drj k=");
    Serial.print(g_demodRejKx10 / 10.0, 1);
    Serial.println(")");
    Serial.println("  tau (s)      OADEV          pairs");
    Serial.println("  ----------   ------------   -----");

    int nOct = numOctaves();
    for (int k = 0; k < nOct; k++) {
      int m = 1 << k;
      double ad = computeForM(m);
      if (std::isnan(ad)) break;

      double tau = (double)m * tau0;
      int pairs = count_ - 2 * m + 1;

      char adBuf[24];
      formatVoltage(ad, adBuf, sizeof(adBuf), 3);

      Serial.print("  ");
      if (tau < 10.0) Serial.print(' ');
      if (tau < 100.0) Serial.print(' ');
      Serial.print(tau, 3);
      Serial.print("      ");
      Serial.print(adBuf);
      // Pad to align pairs column
      int adLen = strlen(adBuf);
      for (int p = adLen; p < 14; p++) Serial.print(' ');
      Serial.println(pairs);
    }

    // Teensy internal temperature stats over the same window
    if (count_ < 2) return;
    float tFirst = tempAt(0);
    float tLast  = tempAt(count_ - 1);
    float tMin = tFirst, tMax = tFirst;
    double tSum = 0.0, vSum = 0.0;
    for (int i = 0; i < count_; i++) {
      float t = tempAt(i);
      if (t < tMin) tMin = t;
      if (t > tMax) tMax = t;
      tSum += t;
      vSum += reading(i);
    }
    double tMean = tSum / count_;
    double vMean = vSum / count_;
    double sxy = 0.0, sxx = 0.0, syy = 0.0;
    for (int i = 0; i < count_; i++) {
      double dt = (double)tempAt(i) - tMean;
      double dv = reading(i) - vMean;
      sxy += dt * dv;
      sxx += dt * dt;
      syy += dv * dv;
    }
    double r = (sxx > 0.0 && syy > 0.0) ? (sxy / std::sqrt(sxx * syy)) : 0.0;
    double slope_nVperC = (sxx > 0.0) ? (sxy / sxx) * 1e9 : 0.0;

    Serial.print("  Teensy temp: mean=");
    Serial.print(tMean, 2);
    Serial.print("C  range=[");
    Serial.print(tMin, 2);
    Serial.print(", ");
    Serial.print(tMax, 2);
    Serial.print("]C  drift(last-first)=");
    Serial.print(tLast - tFirst, 2);
    Serial.print("C  r(V,T)=");
    Serial.print(r, 3);
    Serial.print("  slope=");
    Serial.print(slope_nVperC, 1);
    Serial.println(" nV/C");
  }

private:
  double readings_[MAX_READINGS];
  float  temps_[MAX_READINGS];
  int head_;      // next write position
  int count_;     // total readings stored (up to MAX_READINGS)

  // Linearize circular buffer index: oldest reading is at index 0
  double reading(int i) const {
    // oldest = head_ - count_, wrapped
    int idx = (head_ - count_ + i + MAX_READINGS) % MAX_READINGS;
    return readings_[idx];
  }

  float tempAt(int i) const {
    int idx = (head_ - count_ + i + MAX_READINGS) % MAX_READINGS;
    return temps_[idx];
  }

  double computeForM(int m) const {
    if (m < 1 || count_ < 2 * m + 1) {
      return std::numeric_limits<double>::quiet_NaN();
    }

    // Sliding-window block sums to avoid an O(N) prefix-sum array.
    // pairs = number of valid (i, i+m) block-pair start indices for i in [0, N-2m].
    int pairs = count_ - 2 * m + 1;

    // Compute initial two block sums: block1 = sum[0..m-1], block2 = sum[m..2m-1]
    double block1 = 0.0;
    double block2 = 0.0;
    for (int k = 0; k < m; k++) {
      block1 += reading(k);
      block2 += reading(m + k);
    }

    double sumSq = 0.0;
    double diff = (block2 - block1) / m;
    sumSq += diff * diff;

    // Slide both blocks forward
    for (int i = 1; i < pairs; i++) {
      // Remove oldest element from each block, add new element
      block1 += reading(i + m - 1) - reading(i - 1);
      block2 += reading(i + 2 * m - 1) - reading(i + m - 1);
      diff = (block2 - block1) / m;
      sumSq += diff * diff;
    }

    double avar = sumSq / (2.0 * pairs);
    return std::sqrt(avar);
  }
};
//
// Utility--dump uin32_t in binary to serial monitor
//
void printBinary32(uint32_t val) {
  for (int i = 31; i >= 0; i--) {
    Serial.print(bitRead(val, i)); // Read bit from 31 down to 0
    // Optional: Print a space every 8 bits for readability
    if (i % 8 == 0 && i != 0) Serial.print(" ");
  }
  Serial.println();
}

// -------- RAII State Guard --------
// Saves and restores input channel + DAC code on scope exit.
// Guarantees restore even on early returns or overflow aborts.
class ScopedInstrumentState {
public:
  ScopedInstrumentState()
    : savedChannel_(inputMux.current()),
      savedDac_(dac.currentCode()) {}

  ~ScopedInstrumentState() {
    inputMux.select(savedChannel_);
    dac.setCode(savedDac_);
  }

  ScopedInstrumentState(const ScopedInstrumentState&) = delete;
  ScopedInstrumentState& operator=(const ScopedInstrumentState&) = delete;

private:
  InputChannel savedChannel_;
  int16_t savedDac_;
};

// ================== DacDriver method implementations ==================

// Direct LPSPI4 byte transfer, bypassing Teensyduino SPI.transfer().
//
// Root cause of prior hang: beginTransaction() writes TCR while the module is
// *disabled* (CR=0 → configure → CR=MEN).  Writes to LPSPI TCR while MEN=0
// are silently dropped per NXP RT1060 behaviour, so the TX command FIFO has no
// command entry when the first TDR write arrives, and LPSPI never generates SCLK.
//
// Fix: push the command word (TCR) ourselves while the module is *enabled*,
// immediately before the data word (TDR).  This matches what Teensyduino's
// transfer16() does internally.  Poll RSR RXEMPTY (same as transfer16) instead
// of SR RDF which uses a watermark threshold.
static uint8_t lpspi4_xfer(uint8_t data) {
  // Write TDR only — do NOT push a TCR command entry to the FIFO here.
  //
  // Root cause of prior hang: writing TCR to the FIFO while MEN=1 pushes a
  // command entry that the 600 MHz ARM-vs-LPSPI race can consume before TDR
  // arrives.  When that happens, TDR sits alone in the FIFO (fsr_post=0x1,
  // no preceding command), and the LPSPI stalls indefinitely.
  //
  // Teensyduino's own SPI.transfer() uses TDR-only writes, relying on the
  // shadow TCR left by beginTransaction() (FRAMESZ=7, CPHA=1 = MODE1 8-bit).
  // We do the same here.  beginTransaction() in setup() sets the shadow TCR;
  // a one-time FIFO flush after that call clears the TCR entry it pushes.
  uint32_t fsr_pre = IMXRT_LPSPI4_S.FSR;
  IMXRT_LPSPI4_S.TDR = data;
  uint32_t fsr_post = IMXRT_LPSPI4_S.FSR;
  uint32_t deadline = ARM_DWT_CYCCNT + 600000UL;  // 1 ms timeout at 600 MHz
  while (IMXRT_LPSPI4_S.RSR & LPSPI_RSR_RXEMPTY) {
    if ((int32_t)(ARM_DWT_CYCCNT - deadline) >= 0) {
      // Timed out: dump full LPSPI state
      Serial.print("lpspi4_xfer TIMEOUT data=0x"); Serial.print(data, HEX);
      Serial.print(" fsr_pre=0x");  Serial.print(fsr_pre,  HEX);
      Serial.print(" fsr_post=0x"); Serial.print(fsr_post, HEX);
      Serial.print(" CR=0x");   Serial.print(IMXRT_LPSPI4_S.CR,   HEX);
      Serial.print(" SR=0x");   Serial.print(IMXRT_LPSPI4_S.SR,   HEX);
      Serial.print(" FSR=0x");  Serial.print(IMXRT_LPSPI4_S.FSR,  HEX);
      Serial.print(" RSR=0x");  Serial.print(IMXRT_LPSPI4_S.RSR,  HEX);
      Serial.print(" TCR=0x");  Serial.print(IMXRT_LPSPI4_S.TCR,  HEX);
      Serial.print(" CCR=0x");  Serial.print(IMXRT_LPSPI4_S.CCR,  HEX);
      Serial.print(" FCR=0x");  Serial.println(IMXRT_LPSPI4_S.FCR, HEX);
      while (true) {}  // halt so output is visible
    }
  }
  return IMXRT_LPSPI4_S.RDR;
}

uint32_t DacDriver::readControlRaw() {
  return _dac.readControlReg();
}

void DacDriver::begin() {
  _dac.begin(1000000, 5.0f, 0.0f);  // sets up CS/CLR/RESET pins and calls SPI.begin()
  // Use bit-bang with per-transfer pin switching so LPSPI4 stays available for ADC
  _dac.enableBitbangShared(11, 12, 13);  // MOSI=11, MISO=12, SCK=13
  _dac.hardwareReset();
  _dac.softwareReset();
  delayMicroseconds(10);
  _dac.configureGain2TwosComplement(false);
  _dac.writeCodeAndUpdate(0);
  currentCode_ = 0;
}

// ================== Input Mux Control (MUX36S08) ==================

/**
 * Initialize input mux GPIO pins.
 * Call this in setup() before using inputMux.select().
 */
// ================== InputMuxDriver method implementations ==================

void InputMuxDriver::begin() {
  pinMode(PIN_INMUX_A0, OUTPUT);
  pinMode(PIN_INMUX_A1, OUTPUT);
  pinMode(PIN_INMUX_A2, OUTPUT);
  digitalWrite(PIN_INMUX_A0, LOW);
  digitalWrite(PIN_INMUX_A1, LOW);
  digitalWrite(PIN_INMUX_A2, LOW);
  current_ = InputChannel::Vx1;
}

void InputMuxDriver::select(InputChannel channel) {
  if (channel == current_) return;

  uint8_t addr = static_cast<uint8_t>(channel);
  digitalWriteFast(PIN_INMUX_A0, (addr & 0x01) ? HIGH : LOW);
  digitalWriteFast(PIN_INMUX_A1, (addr & 0x02) ? HIGH : LOW);
  digitalWriteFast(PIN_INMUX_A2, (addr & 0x04) ? HIGH : LOW);
  current_ = channel;
  demodPoolReset();
  delayMicroseconds(INMUX_SETTLE_US);
}

const char* getInputChannelName(InputChannel channel) {
  uint8_t idx = static_cast<uint8_t>(channel);
  if (idx < 8 && channelLabels[idx][0] != '\0') {
    // Return a composite string from a small static buffer.
    // Use %.15s to bound the label length (CHANNEL_LABEL_MAX=15) and suppress
    // -Wformat-truncation: label<=15 chars + longest prefix "VrefRaw " = 8 chars
    // → 23 bytes max, well within nameBuf[48].
    static char nameBuf[48];
    switch (channel) {
      case InputChannel::Vx1:    snprintf(nameBuf, sizeof(nameBuf), "Vx1 %.15s (\xc2\xb1" "5V)", channelLabels[idx]); break;
      case InputChannel::GND:    snprintf(nameBuf, sizeof(nameBuf), "GND %.15s",                  channelLabels[idx]); break;
      case InputChannel::VrefRaw:snprintf(nameBuf, sizeof(nameBuf), "VrefRaw %.15s",              channelLabels[idx]); break;
      case InputChannel::Vx2:    snprintf(nameBuf, sizeof(nameBuf), "Vx2 %.15s",                  channelLabels[idx]); break;
      case InputChannel::Vx3:    snprintf(nameBuf, sizeof(nameBuf), "Vx3 %.15s",                  channelLabels[idx]); break;
      case InputChannel::HVDiv:  snprintf(nameBuf, sizeof(nameBuf), "HVDiv %.15s",                channelLabels[idx]); break;
      case InputChannel::Vx4:    snprintf(nameBuf, sizeof(nameBuf), "Vx4 %.15s",                  channelLabels[idx]); break;
      case InputChannel::Vx5:    snprintf(nameBuf, sizeof(nameBuf), "Vx5 %.15s",                  channelLabels[idx]); break;
      default:                   snprintf(nameBuf, sizeof(nameBuf), "Unknown %.15s",               channelLabels[idx]); break;
    }
    return nameBuf;
  }
  switch (channel) {
    case InputChannel::Vx1:     return "Vx1 (\xc2\xb1" "5V)";  // \xb1 + "5" avoids greedy hex parse
    case InputChannel::GND:     return "GND";
    case InputChannel::VrefRaw: return "VrefRaw";
    case InputChannel::Vx2:     return "Vx2";
    case InputChannel::Vx3:     return "Vx3";
    case InputChannel::HVDiv:   return "HVDiv";
    case InputChannel::Vx4:     return "Vx4";
    case InputChannel::Vx5:     return "Vx5";
    default:                    return "Unknown";
  }
}

// ================== DacDriver continued ==================

uint32_t DacDriver::calculateSettleTime(int32_t stepSize) {
  if (stepSize <= 0) return 0;
  if (stepSize > 32768) stepSize = 32768;
  const float scale = (float)(DAC_SETTLE_MAX_MS - DAC_SETTLE_MIN_MS) / sqrtf(32768.0f);
  uint32_t additionalMs = (uint32_t)(scale * sqrtf((float)stepSize));
  return DAC_SETTLE_MIN_MS + additionalMs;
}

void DacDriver::setCode(int16_t code) {
  if (code == currentCode_) return;

  int32_t stepSize = abs((int32_t)code - (int32_t)currentCode_);
  currentCode_ = code;
  _dac.writeCodeAndUpdate(code);
  demodPoolReset();

  uint32_t settleMs = calculateSettleTime(stepSize);
  if (settleMs > 0) {
    delay(settleMs);
  }
}

void DacDriver::setCodeFast(int16_t code) {
  currentCode_ = code;
  _dac.writeCodeAndUpdate(code);
  demodPoolReset();
  delay(5);  // ~5 ms: enough for ADC to see coarse signal, not full filter settle
}

// Check if sample indicates overflow
inline bool isOverflowPositive(int32_t sample) {
  return sample > ADC_OVERFLOW_POS;
}

inline bool isOverflowNegative(int32_t sample) {
  return sample < ADC_OVERFLOW_NEG;
}

inline bool isOverflow(int32_t sample) {
  return isOverflowPositive(sample) || isOverflowNegative(sample);
}

// Read a single ADC sample for DAC adjustment (with settling).
// Resets the ADC filter via stop/start so filter history from the previous
// DAC code does not produce a spurious in-range reading on the first sample.
// Returns true on success, false on ADC timeout.
// Settle time after re-enabling TMUX in binary search. The preamp has been
// disconnected for up to RC_SETTLE_MS; allow it to recover before the ADC filter
// sees any signal. Must be long enough to avoid false overflow on the discard sample.
static constexpr uint32_t TMUX_RECONNECT_US = 2000;

bool readSettledSample(int32_t &sample) {
  adc.stop();
  digitalWriteFast(PIN_TMUX_EN, LOW);   // enable TMUX (active-low) early so preamp starts recovering
  delayMicroseconds(2*SETTLE_US + 2*TMUX_RECONNECT_US);  // guard + preamp reconnect settle
  adc.start();
  int32_t discard;
  for (uint8_t i = 0; i < DISCARD_SAMPLES+1; i++) {
    if (!adc.readSample24(discard)) {
      digitalWriteFast(PIN_TMUX_EN, HIGH);  // disable TMUX on error
      return false;
    }
    if (isOverflow(discard)) {
      // Overflow confirmed on first sample — disable TMUX immediately to protect diodes.
      sample = discard;
      digitalWriteFast(PIN_TMUX_EN, HIGH);
      return true;
    }
  }
  bool ok = adc.readSample24(sample);
  if (!ok || isOverflow(sample)) {
    digitalWriteFast(PIN_TMUX_EN, HIGH);
  }
  // Non-overflow: leave TMUX enabled so normal measurement can follow without re-enabling.
  return ok;
}

/**
 * Binary search to find DAC code that brings ADC into range.
 * Returns true if successful, false if unable to find valid code.
 *
 * Uses CHOPPED measurement at each iteration: Phase A (chopper LOW) and Phase B
 * (chopper HIGH), 14 samples each, ~14 ms apart. Chop demodulation cancels both
 * the preamp's DC offset AND the underdamped DAC RC filter's slow ringing
 * (period ~10 s, see project_dac_filter.md). Same rejection mechanism that
 * makes ZeroCal robust; immune to the filter ringing that defeated the
 * single-sample + settling-detector approach.
 *
 * Hardware signal routing (from schematic):
 *   Chopper LOW  (Phase A): DAC→V−, Vx→V+ → ADC reads  (Vx − Vdac) × G + offset
 *   Chopper HIGH (Phase B): DAC→V+, Vx→V− → ADC reads −(Vx − Vdac) × G + offset
 *   Demod = (sumA − sumB) is proportional to (Vx − Vdac); offset cancels.
 *
 * Decision logic per iteration:
 *   Phase A overflows +rail → Vx ≫ Vdac → bisect up (low = mid+1)
 *   Phase A overflows −rail → Vx ≪ Vdac → bisect down (high = mid-1)
 *   Phase B overflows +rail → Vdac ≫ Vx → bisect down
 *   Phase B overflows −rail → Vdac ≪ Vx → bisect up
 *   Both in-range          → sign(sumA − sumB) decides bisection
 *
 * Side-effect cleanliness: BinSrch calls acquireHalfCycle() but NOT demodulate(),
 * so the g_demod* clip-rate counters and the g_diffPool pooled-MAD ring are
 * untouched. The pool gets reset on every dac.setCode() as usual.
 *
 * Predictive jump (secant overlay): when both phases land in-range, |demod|
 * encodes (Vx − Vdac) directly. Convert to DAC codes via
 *   step = demod · ADC_LSB_V / (2 · N · gain · DAC_LSB_V)
 * and override the next bisection midpoint with mid+step, clamped to the
 * current [low, high] bracket. The clamp keeps a bad prediction (gain/LSB
 * miscal, noise, preamp nonlinearity) from straying outside the safe range —
 * bisection then mops up. Collapses the near-edge case (e.g. zero-cal, Vx≈0)
 * from ~16 rail-bisection iterations to ~3.
 *
 * Full 16-bit search range. INL table holds cardinals at 14-bit-aligned codes
 * (+ the 32767 slot); voltages at unaligned codes are linear interpolations.
 */
bool binarySearchDAC() {
  if (g_debugDac) Serial.println("DAC: Starting binary search (chopped, full 16-bit)...");

  // Defensive LPSPI4 full reset before BinSrch does its many SPI reads.
  // Symptom: first lpspi4_xfer in the 2nd BinSrch (post-POST) times out with
  // SR=0x1000700 (MBF=1, stale TCF), FSR=0x1, RSR=RXEMPTY — module thinks it's
  // mid-transfer but produces no clocks. Hypothesis: the DAC bit-bang sequence
  // (which leaves MEN=1 and toggles SCK/MOSI/MISO pins through GPIO with SION)
  // advances the LPSPI FSM into a stuck state. Full module reset is the only
  // way to get out.
  IMXRT_LPSPI4_S.CR = 0;                      // MEN=0: disable module
  IMXRT_LPSPI4_S.CR = LPSPI_CR_RST;           // reset all registers/FSM
  IMXRT_LPSPI4_S.CR = 0;                      // clear reset
  IMXRT_LPSPI4_S.CR = LPSPI_CR_RTF | LPSPI_CR_RRF;  // flush FIFOs (idempotent post-reset)
  IMXRT_LPSPI4_S.CFGR1 = LPSPI_CFGR1_MASTER;  // master mode (post-reset default loses this)
  IMXRT_LPSPI4_S.CCR = 0x001D1D3A;            // restore clock divider seen pre-wedge
  IMXRT_LPSPI4_S.TCR = LPSPI_TCR_FRAMESZ(7) | LPSPI_TCR_CPHA;  // 8-bit, MODE1
  IMXRT_LPSPI4_S.CR = LPSPI_CR_MEN;           // re-enable

  // Enable TMUX for chopped reads (preamp needs input connected).
  digitalWriteFast(PIN_TMUX_EN, LOW);

  // Force chopper LOW so Phase A reads (Vx − Vdac) × G.
  if (chopper.state()) {
    chopper.toggle();
  }

  int32_t low  = -32768;
  int32_t high =  32767;
  int32_t bestMid = 0;
  uint64_t bestAbsDemod = UINT64_MAX;
  bool foundInRange = false;

  int32_t phaseA[MAX_SAMPLES];
  int32_t phaseB[MAX_SAMPLES];
  const int N = g_good_samples;

  // demod-to-DAC-code conversion for the predictive jump (see header comment).
  const double demodToDacCodes =
      ADC_LSB_V / (2.0 * (double)N * PREAMP_GAIN * DAC_LSB_V);
  int32_t predMidOverride = INT32_MIN;   // sentinel: no override pending

  // Acquire a half-cycle with DRDY-timeout retry. acquireHalfCycle() sets
  // overflow=true and leaves overflowSample at its default 0 when the ADC
  // fails to deliver a sample within timeout — that's distinct from a real
  // rail saturation (where overflowSample is the saturated value ±FS).
  // Without this distinction, the search treats a timeout as "negative rail"
  // (0 falls into the !(>0) branch) and bisects in the wrong direction,
  // squeezing the bracket through the linear window so no in-range read is
  // ever taken.
  //
  // Root cause of these timeouts: the AAF (C19 = 47 nF, τ ≈ 56 ms) holds
  // the ADC input outside its range after a series of preamp-saturated iters.
  // The AAF cap sits near the preamp supply rail (≈ 12 V) after sustained
  // rail; when DAC steps into the linear window the preamp output recovers
  // immediately but the AAF discharges with τ ≈ 56 ms, so it takes ~2τ
  // (≈ 110 ms) to bring the AAF back inside the ADC's ±2.5 V window. Until
  // then the modulator is pegged and DRDY suspends.
  //
  // Strategy: first attempt at normal CHOP_SETTLE_US (fine for non-rail
  // transitions), then two recovery attempts at 150 ms and 400 ms — those
  // cover 2.7τ and 7τ respectively, comfortably outside the rail zone.
  // Returns false only after all three time out; caller should bail the
  // search. On success the caller still checks out.overflow for real
  // rail saturation.
  auto acquireWithRetry = [&](int32_t* samples, HalfCycleResult& out) -> bool {
    adc.stop();
    delayMicroseconds(CHOP_SETTLE_US);
    adc.start();
    out = acquireHalfCycle(samples);
    if (!(out.overflow && out.overflowSample == 0)) return true;

    const uint32_t recoverSettleUs[2] = {150000, 400000};
    for (int retry = 0; retry < 2; retry++) {
      if (g_debugDac) {
        Serial.print("    DRDY timeout — retry "); Serial.print(retry + 1);
        Serial.print("/2 with "); Serial.print(recoverSettleUs[retry] / 1000);
        Serial.println(" ms settle (AAF recovery from rail)");
      }
      adc.stop();
      delayMicroseconds(recoverSettleUs[retry]);
      adc.start();
      out = acquireHalfCycle(samples);
      if (!(out.overflow && out.overflowSample == 0)) return true;
    }
    return false;
  };

  const int MAX_ITERATIONS = 17;  // 16 halvings of 65536; +1 for margin
  int iter = 0;
  for (; iter < MAX_ITERATIONS; iter++) {
    int32_t mid;
    if (predMidOverride != INT32_MIN) {
      mid = predMidOverride;
      predMidOverride = INT32_MIN;
    } else {
      mid = (low + high) / 2;
    }
    dac.setCode((int16_t)mid);

    // Phase A: chopper LOW, ADC reads (Vx − Vdac) × G + offset
    HalfCycleResult rA;
    if (!acquireWithRetry(phaseA, rA)) {
      Serial.print("DAC: BinSrch bail at iter "); Serial.print(iter);
      Serial.print(" DAC="); Serial.print(mid);
      Serial.println(" (persistent Phase A DRDY timeout — AAF/preamp not recovering within 560 ms)");
      break;
    }
    if (rA.overflow) {
      // Real rail saturation: overflowSample is the saturated value (±FS).
      if (rA.overflowSample > 0) low  = mid + 1;
      else                       high = mid - 1;
      if (g_debugDac) {
        Serial.print("  iter="); Serial.print(iter);
        Serial.print(" DAC="); Serial.print(mid);
        Serial.print(" PhaseA rail="); Serial.println(rA.overflowSample);
      }
      if (low > high) break;
      continue;
    }

    // Phase B: chopper HIGH, ADC reads −(Vx − Vdac) × G + offset
    chopper.toggle();
    HalfCycleResult rB;
    bool rbOk = acquireWithRetry(phaseB, rB);
    chopper.toggle();  // restore chopper LOW for next iter / for caller
    if (!rbOk) {
      Serial.print("DAC: BinSrch bail at iter "); Serial.print(iter);
      Serial.print(" DAC="); Serial.print(mid);
      Serial.println(" (persistent Phase B DRDY timeout — AAF/preamp not recovering within 560 ms)");
      break;
    }
    if (rB.overflow) {
      // Phase B sign is inverted: sample > 0 ⇒ Vdac > Vx ⇒ bisect down
      if (rB.overflowSample > 0) high = mid - 1;
      else                       low  = mid + 1;
      if (g_debugDac) {
        Serial.print("  iter="); Serial.print(iter);
        Serial.print(" DAC="); Serial.print(mid);
        Serial.print(" PhaseB rail="); Serial.println(rB.overflowSample);
      }
      if (low > high) break;
      continue;
    }

    // Both phases in-range: compute chop demod sign (offset and slow DAC
    // filter drift cancel out — only Vx − Vdac survives).
    int64_t sumA = 0, sumB = 0;
    for (int i = 0; i < N; i++) { sumA += phaseA[i]; sumB += phaseB[i]; }
    int64_t demod = sumA - sumB;             // ∝ (Vx − Vdac); offset cancels
    uint64_t absD = (uint64_t)(demod < 0 ? -demod : demod);

    if (g_debugDac) {
      Serial.print("  iter="); Serial.print(iter);
      Serial.print(" DAC="); Serial.print(mid);
      Serial.print(" demod="); Serial.println((long)demod);
    }

    // Track the in-range code with the smallest |demod|. Bisection may step
    // away from this in subsequent iters (due to ring-induced false decisions
    // at intermediate codes); we restore bestMid before returning.
    if (absD < bestAbsDemod) {
      bestAbsDemod = absD;
      bestMid      = mid;
      foundInRange = true;
    }

    if (demod > 0) low  = mid + 1;            // Vx > Vdac
    else           high = mid - 1;            // Vx < Vdac (treat exactly 0 same as <0)
    if (low > high) break;

    // Predictive jump: estimate target code from demod magnitude. Clamp to
    // current bracket so a bad prediction can never escape the bisection safety
    // net. mid+step is the code where demod would be 0 if the system were
    // perfectly linear and calibrated.
    double predD = (double)mid + (double)demod * demodToDacCodes;
    int32_t pred = (int32_t)(predD >= 0 ? predD + 0.5 : predD - 0.5);
    if (pred < low)  pred = low;
    if (pred > high) pred = high;
    predMidOverride = pred;
    if (g_debugDac) {
      Serial.print("    pred next DAC="); Serial.println(pred);
    }
  }

  if (foundInRange) {
    dac.setCode((int16_t)bestMid);
    Serial.print("DAC: Converged at code ");
    Serial.print(bestMid);
    Serial.print(" (chop |demod|=");
    Serial.print((unsigned long)bestAbsDemod);
    Serial.println(")");
    return true;
  }

  Serial.println("DAC: Binary search failed to converge!");
  return false;
}

// ================== AdcDriver method implementations ==================

uint8_t AdcDriver::xfer(uint8_t v) { return lpspi4_xfer(v); }

//
// sec 8.5.6.3
//   write register in one 16 bit frame
//
void AdcDriver::writeReg(uint8_t addr, uint8_t value) {
  csLow(PIN_CS_ADC);
  xfer(OPCODE_WREG | (addr & 0xF));  // opcode with register address embedded
  xfer(value);                       // register value
  csHigh(PIN_CS_ADC);
  delayMicroseconds(2);
}

//
// sec 8.5.6.2 two frames
//    frame1: two bytes first with reg addr
//    frame2: two bytes with reg data in first byte, second is 0x0.
//
uint8_t AdcDriver::readReg(uint8_t addr) {
  csLow(PIN_CS_ADC);
  xfer(OPCODE_RREG | (addr & 0xF));  // opcode with register address embedded
  xfer(0x00);                        // ignored
  csHigh(PIN_CS_ADC);

  delayMicroseconds(2);

  csLow(PIN_CS_ADC);
  uint8_t val = xfer(0x00);
  xfer(0x00);      // padding, discard
  csHigh(PIN_CS_ADC);
  return val;
}

bool AdcDriver::waitDrdy(uint32_t timeout_us) {
  // Edge-triggered: wait for HIGH first to discard any stale assertion left
  // over from before adc.stop() (the data register would otherwise hold a
  // pre-stop reading — saturated if the preamp was railed, e.g. binary search
  // with TMUX disabled), then wait for the genuine falling edge of the next
  // post-start conversion. ADS127L11 default DRDY mode is pulsed: LOW for
  // ~1/2 conversion period, plenty wide for digitalReadFast polling to catch.
  uint32_t t0 = micros();
  while (digitalReadFast(PIN_ADC_DRDY) == LOW) {
    if ((micros() - t0) > timeout_us) return false;
  }
  while (digitalReadFast(PIN_ADC_DRDY) == HIGH) {
    if ((micros() - t0) > timeout_us) return false;
  }
  return true;
}

//
// section 8.5.7 conversion data read, short format
//
bool AdcDriver::readSample24(int32_t &sample) {
  // Per-sample DRDY timeout sized to cover the worst filter-pipeline latency
  // we use: sinc3 OSR=26667 has 3*Ts latency before first DRDY after start().
  // 4x Ts + slack handles all filter modes without false negatives.
  const uint32_t timeoutUs = (uint32_t)(EST_TS_US * 4.0f) + 200;
  if (!waitDrdy(timeoutUs)) {
    sample = 0;
    return false;
  }

  csLow(PIN_CS_ADC);

  uint8_t b2 = xfer(0x00); // MSB
  uint8_t b1 = xfer(0x00); // MID
  uint8_t b0 = xfer(0x00); // LSB

  csHigh(PIN_CS_ADC);

  sample = (int32_t)((uint32_t)b2 << 16 | (uint32_t)b1 << 8 | (uint32_t)b0);
  if (sample & 0x00800000) sample |= 0xFF000000;  // sign extend
  return true;
}

void AdcDriver::begin() {
  pinMode(PIN_ADC_START, OUTPUT);
  stop();   // START=LOW: continuous conversion idle until SPI start

  pinMode(PIN_ADC_RESET, OUTPUT);
  digitalWriteFast(PIN_ADC_RESET, HIGH);  // RESET=HIGH: device running (active-low reset)

  // Set up the PWM channel on pin 9 (FlexPWM2_SM2_B) via analogWriteFrequency
  // first — this configures pin mux, prescaler, and the channel-B output —
  // then override VAL1/VAL3 directly to pick the highest-frequency divisor
  // that stays at or below ADS127L11's 25.6 MHz fCLK spec. The library's
  // round-to-nearest picks divisors that can land far from the requested
  // value (e.g. 25 MHz → 17.6 MHz at F_CPU=528 because F_BUS_ACTUAL ÷ 6 was
  // closer than ÷ 5). Manual VAL1 gives us deterministic control.
  analogWriteFrequency(PIN_CLK_ADC, 25000000);
  analogWrite(PIN_CLK_ADC, 128);

  // Pick the integer divisor that lands the PWM as high as possible without
  // exceeding the ADS127L11 fCLK_MAX of 25.6 MHz. Use ceil(F_BUS / target) so
  // the resulting freq = F_BUS / divisor is guaranteed ≤ target. Possible
  // outcomes for plausible F_BUS_ACTUAL values:
  //   F_BUS = 150 MHz (F_CPU=600 or 450) → divisor 6 → 25.0 MHz exact   ← current (F_CPU=450)
  //   F_BUS = 132 MHz (F_CPU=528)        → divisor 6 → 22.0 MHz   (5→26.4 is over spec)
  //   F_BUS = 120 MHz (F_CPU=480)        → divisor 5 → 24.0 MHz exact
  //   F_BUS = 105.6 MHz                  → divisor 5 → 21.12 MHz  (4→26.4 is over spec)
  //   F_BUS =  88 MHz                    → divisor 4 → 22.0 MHz   (3→29.3 is over spec)
  // If you want to push to 26.4 MHz at F_CPU=528 (3% over spec, often works
  // but no guarantee), bump ADC_FCLK_TARGET_HZ to 26'500'000.
  const uint32_t ADC_FCLK_TARGET_HZ = 25'600'000;
  uint32_t divisor = (F_BUS_ACTUAL + ADC_FCLK_TARGET_HZ - 1) / ADC_FCLK_TARGET_HZ;  // ceil
  if (divisor < 2) divisor = 2;     // FlexPWM minimum
  if (divisor > 65535) divisor = 65535;

  // Submodule 2 of FlexPWM2 drives pin 9. CLDOK gates the load; LDOK commits.
  FLEXPWM2_MCTRL |= FLEXPWM_MCTRL_CLDOK(1 << 2);
  FLEXPWM2_SM2VAL1 = divisor - 1;                // period top
  FLEXPWM2_SM2VAL3 = divisor / 2;                // channel B 50% duty
  FLEXPWM2_MCTRL |= FLEXPWM_MCTRL_LDOK(1 << 2);

  Serial.print("ADC PWM: F_CPU_ACTUAL=");
  Serial.print(F_CPU_ACTUAL / 1e6f, 3);
  Serial.print(" MHz, F_BUS_ACTUAL=");
  Serial.print(F_BUS_ACTUAL / 1e6f, 3);
  Serial.print(" MHz, divisor=");
  Serial.print(divisor);
  Serial.print(" -> nominal fCLK=");
  Serial.print((float)F_BUS_ACTUAL / divisor / 1e6f, 3);
  Serial.println(" MHz (verified by calibrateAdcClock)");

  // Diagnostic dump of the FlexPWM2 SM2 registers AFTER our overrides, so we
  // can compute the *actual* PWM period from hardware state rather than
  // assuming Teensyduino's analogWriteFrequency left things in a particular
  // form. PWM period (edge-aligned) = VAL1 - INIT + 1 input clocks, divided
  // by 2^PRSC. If centered/full mode, the output may toggle at a different
  // rate; this dump exposes that too.
  uint32_t ctrl = FLEXPWM2_SM2CTRL;
  uint32_t ctrl2 = FLEXPWM2_SM2CTRL2;
  int32_t  init = (int16_t)FLEXPWM2_SM2INIT;     // signed 16-bit
  int32_t  val1 = (int16_t)FLEXPWM2_SM2VAL1;
  int32_t  val2 = (int16_t)FLEXPWM2_SM2VAL2;
  int32_t  val3 = (int16_t)FLEXPWM2_SM2VAL3;
  uint32_t prsc = (ctrl >> 4) & 0x7;
  int32_t  period_cnt = val1 - init + 1;
  float    period_clks = (float)period_cnt * (float)(1u << prsc);
  float    expected_freq = (float)F_BUS_ACTUAL / period_clks;
  Serial.print("  FlexPWM2_SM2: INIT="); Serial.print(init);
  Serial.print(" VAL1=");                Serial.print(val1);
  Serial.print(" VAL2=");                Serial.print(val2);
  Serial.print(" VAL3=");                Serial.print(val3);
  Serial.print(" PRSC=");                Serial.print(prsc);
  Serial.print(" CTRL=0x");              Serial.print(ctrl, HEX);
  Serial.print(" CTRL2=0x");             Serial.print(ctrl2, HEX);
  Serial.print("  period=");             Serial.print(period_cnt);
  Serial.print(" cnts -> expected ");    Serial.print(expected_freq / 1e6f, 3);
  Serial.println(" MHz");

  // Maximize drive strength and slew rate on the ADC clock pad (iMXRT1062 pin 9 = GPIO_B0_11).
  // analogWrite sets mux and default pad config; we override here for clean edges.
  // DSE(7) = max drive (~53 Ω), SPEED(2) = high, SRE = fast slew rate.
  //IOMUXC_SW_PAD_CTL_PAD_GPIO_B0_11 = IOMUXC_PAD_DSE(7) | IOMUXC_PAD_SPEED(2) | IOMUXC_PAD_SRE;
}

void AdcDriver::start() {
	digitalWriteFast(PIN_ADC_START, HIGH);
}

void AdcDriver::stop() {
	digitalWriteFast(PIN_ADC_START, LOW);
}

void AdcDriver::initAndConfigure() {
  digitalWriteFast(PIN_ADC_RESET, LOW);
  delay(1);
  digitalWriteFast(PIN_ADC_RESET, HIGH);
  delay(5);
  

  // Read STATUS immediately after reset. Bit 7 (RESET_FLAG) should be 1 = 0x80.
  // If STATUS reads 0x00: SPI MODE wrong or MOSI not reaching ADC.
  // If STATUS reads 0xFF: MISO floating (not connected).
  uint8_t status = readReg(REG_STATUS);
  Serial.print("ADC STATUS after reset: 0x"); Serial.println(status, HEX);

  const uint8_t config4 = (1u << 7);  // CLK_SEL=1: external clock, Databit[3]=0 ==> 24 bit.
  writeReg(REG_CONFIG4, config4);

  // CONFIG1: low reference range, 1x input range, no VCM, optional reference buffer,
  //          no input precharge buffers.
  // Bit 3 (REFP_BUF): set when ADC_REFBUF_ENABLE=true — recommended with series resistor on VREF.
  const uint8_t config1 = ADC_REFBUF_ENABLE ? (1u << 3) : 0x00;
  writeReg(REG_CONFIG1, config1);

  // CONFIG2: high-speed mode (fMOD >= 12.5 MHz), data-output mode, default start
  //          (explicit — matches reset default 0x00)
  const uint8_t config2 = 0x00;
  writeReg(REG_CONFIG2, config2);

  const uint8_t delayCode  = 0b000;
  const uint8_t filterCode = 0b11010;  // sinc4 OSR=32 + sinc1 OSR=400 → OSR_total=12800
  const uint8_t config3 = (delayCode << 5) | (filterCode & 0x1F);
  writeReg(REG_CONFIG3, config3);


  uint8_t r1 = readReg(REG_CONFIG1);
  uint8_t r2 = readReg(REG_CONFIG2);
  uint8_t r3 = readReg(REG_CONFIG3);
  uint8_t r4 = readReg(REG_CONFIG4);

  Serial.print("ADC CONFIG1 set/read: 0x"); Serial.print(config1, HEX);
  Serial.print(" / 0x"); Serial.println(r1, HEX);

  Serial.print("ADC CONFIG2 set/read: 0x"); Serial.print(config2, HEX);
  Serial.print(" / 0x"); Serial.println(r2, HEX);

  Serial.print("ADC CONFIG3 set/read: 0x"); Serial.print(config3, HEX);
  Serial.print(" / 0x"); Serial.println(r3, HEX);

  Serial.print("ADC CONFIG4 set/read: 0x"); Serial.print(config4, HEX);
  Serial.print(" / 0x"); Serial.println(r4, HEX);

  start();
}

// ADC clock self-calibration is now folded into postTestDrdyTiming() —
// a single measurement is the source of truth for EST_FSPS/EST_TS_US/
// ADC_FCLK_HZ/ADC_FMOD_HZ/OSR_TOTAL. Previously a separate calibrateAdcClock()
// ran before POST and they disagreed when the polling occasionally missed
// short DRDY pulses; the second measurement would then fail POST's tolerance
// against the (already-wrong) first measurement. POST now uses scope-verified
// PWM-register-derived fCLK as ground truth and validates the derived OSR
// against a plausibility window.

// ================== ChopperDriver method implementations ==================

void ChopperDriver::begin() {
  pinMode(PIN_TMUX_EN, OUTPUT);
  pinMode(PIN_TMUXSEL, OUTPUT);
  pinMode(PIN_CI_TMUXSEL, OUTPUT);
  digitalWriteFast(PIN_TMUXSEL, LOW);
  digitalWriteFast(PIN_CI_TMUXSEL, LOW);
  digitalWriteFast(PIN_TMUX_EN, LOW);
  state_ = false;
}

void ChopperDriver::toggle() {
  digitalWriteFast(PIN_TMUX_EN, LOW); // just in case we got into unknown state due to post...or?
  digitalWriteFast(PIN_CI_TMUXSEL, HIGH);
  delayMicroseconds(CI_PRE_EDGE_US);
  state_ = !state_;
  digitalWriteFast(PIN_TMUXSEL, state_);
  delayMicroseconds(CI_POST_EDGE_US);
  digitalWriteFast(PIN_CI_TMUXSEL, LOW);
}

// ---------------- Chopped acquisition ----------------

// Collect samples for one half-cycle, detect overflow on any sample.
// write samples array for stats accumulation
HalfCycleResult acquireHalfCycle(int32_t *samples) {
  HalfCycleResult result;

  // Discard initial samples after settling
  int32_t discard;
  for (uint8_t i = 0; i < DISCARD_SAMPLES; i++) {
    if (!adc.readSample24(discard)) { result.overflow = true; return result; }
  }

  for (uint8_t i = 0; i < g_good_samples; i++) {
    int32_t s;
    if (!adc.readSample24(s)) { 
      result.overflow = true; 
      return result; 
    }
    if (isOverflow(s)) {
      result.overflow = true; 
      result.overflowSample = s;
      return result;  
    }
    samples[i] = s;
  }
  return result;
}

// Allan deviation tracker for voltage stability analysis
static AllanDeviation allanDev;

// ================== Scan & Logging Configuration ==================
//
// These structures control the channel scanning state machine in loop() and
// the serial output format. ScanConfig is persisted to EEPROM via SavedConfig.

static constexpr int MAX_SCAN_CHANNELS = 7;  // GND excluded (used only for auto-zero)

enum class OutputMode : uint8_t {
  Human,    // Default: multi-line human-readable text with SI prefixes
  CSV,      // Machine-readable CSV, one row per channel per integration period
  Plotter,  // Arduino Serial Plotter compatible: "label:value\t..." per line
};

struct ScanConfig {
  InputChannel channels[MAX_SCAN_CHANNELS];
  int count;
  int integrationCycles;
  bool autoZeroEnabled;
  int autoZeroInterval;
  OutputMode outputMode;
  float ewmaWindow;          // equivalent SMA length applied to all channel + auto-zero EWMAs
};

enum class ScanState : uint8_t { ONE_CHANNEL, SCANNING };

static constexpr int SCAN_SETTLE_CYCLES = 5;

// Encapsulates all scanning state: configuration, per-channel stats,
// DAC code cache, auto-zero, and the ONE_CHANNEL/SCANNING state machine.
class ScanController {
public:
  ScanController() { clearDacCache(); }

  // Configuration (public for EEPROM save/load and command handlers)
  ScanConfig config = {
    .channels = { InputChannel::Vx1 },
    .count = 1,
    .integrationCycles = 850,
    .autoZeroEnabled = false,
    .autoZeroInterval = 10,
    .outputMode = OutputMode::Human,
    .ewmaWindow = 10.0f,
  };

  // State queries
  ScanState state() const { return state_; }
  bool isScanning() const { return state_ == ScanState::SCANNING; }
  bool isOneChannel() const { return state_ == ScanState::ONE_CHANNEL; }

  void start() { state_ = ScanState::SCANNING; currentScanIndex_ = 0; cycleCount_ = 0; chopCycleCount_ = 0; }
  void stop()  { state_ = ScanState::ONE_CHANNEL; }

  // Per-channel statistics
  LowerMoments& channelStats(uint8_t idx) { return channelStats_[idx]; }
  // Per-channel phase buffers
  int32_t *bufferA(uint8_t idx) { return phaseA_[idx]; }
  int32_t *bufferB(uint8_t idx) { return phaseB_[idx]; }

  // Per-channel DAC code cache
  int16_t channelDacCode(uint8_t idx) const { return channelDacCode_[idx]; }
  bool    channelDacValid(uint8_t idx) const { return channelDacValid_[idx]; }
  void    setChannelDac(uint8_t idx, int16_t code) { channelDacCode_[idx] = code; channelDacValid_[idx] = true; }
  void    clearDacCache() { for (int i = 0; i < 8; i++) { channelDacCode_[i] = 0; channelDacValid_[i] = false; } }

  // Per-channel adjusted EWMA (one per InputChannel slot)
  AdjustedEWMA& channelEwma(uint8_t idx) { return channelEwma_[idx]; }
  void resetChannelEwma(uint8_t idx) { channelEwma_[idx].reset(); }
  void resetAllChannelEwma() { for (auto& e : channelEwma_) e.reset(); }
  void setEwmaWindow(float w) {
    for (auto& e : channelEwma_) e.setWindow(w);
    autoZeroEwma_.setWindow(w);
  }

  // Auto-zero
  AdjustedEWMA autoZeroEwma_;   // smooths successive auto-zero measurements
  double autoZeroOffset = 0.0;  // EWMA-filtered offset applied to all readings
  double autoZeroRaw    = 0.0;  // last single-measurement raw offset (for display)
  double autoZeroSdev   = 0.0;  // repeatability sdev of the two auto-zero samples
  bool autoZeroValid = false;

  // Scan loop state
  int currentScanIndex() const { return currentScanIndex_; }
  void setCurrentScanIndex(int i) { currentScanIndex_ = i; }
  int cycleCount() const { return cycleCount_; }
  void incCycleCount() { cycleCount_++; }
  int chopCycleCount() const { return chopCycleCount_; }
  void setChopCycleCount(int c) { chopCycleCount_ = c; }
  void incChopCycleCount() { chopCycleCount_++; }

private:
  ScanState state_ = ScanState::ONE_CHANNEL;
  int currentScanIndex_ = 0;
  int cycleCount_ = 0;
  int chopCycleCount_ = 0;
  LowerMoments channelStats_[8];
  int32_t phaseA_[8][MAX_SAMPLES];
  int32_t phaseB_[8][MAX_SAMPLES];
  AdjustedEWMA channelEwma_[8];
  int16_t channelDacCode_[8];
  bool channelDacValid_[8];
};

static ScanController scanner;

// ================== Serial Command Interface ==================

static char cmdBuffer[128];
static int cmdBufferIdx = 0;

/**
 * Parse a channel name string (case-insensitive) to InputChannel enum.
 * Inverse of getChannelShortName(). Called by cmdScanAdd(), cmdScanRemove(),
 * and cmdLabel() to convert user-typed channel names from serial commands.
 *
 * @param name  Channel name string (e.g., "Vx1", "VrefRaw", "HVDiv")
 * @param ch    Output: corresponding InputChannel value
 * @return      true on success, false if name not recognized
 */
bool parseChannelName(const char* name, InputChannel &ch) {
  if (strcasecmp(name, "Vx1") == 0)     { ch = InputChannel::Vx1; return true; }
  if (strcasecmp(name, "GND") == 0)     { ch = InputChannel::GND; return true; }
  if (strcasecmp(name, "VrefRaw") == 0) { ch = InputChannel::VrefRaw; return true; }
  if (strcasecmp(name, "Vx2") == 0)     { ch = InputChannel::Vx2; return true; }
  if (strcasecmp(name, "Vx3") == 0)     { ch = InputChannel::Vx3; return true; }
  if (strcasecmp(name, "HVDiv") == 0)   { ch = InputChannel::HVDiv; return true; }
  if (strcasecmp(name, "Vx4") == 0)     { ch = InputChannel::Vx4; return true; }
  if (strcasecmp(name, "Vx5") == 0)     { ch = InputChannel::Vx5; return true; }
  return false;
}

/**
 * Get short channel name string without range/description suffix.
 * Used by CsvFormatter and PlotterFormatter where compact names are needed.
 * If a user label is set for the channel, returns that label instead.
 * Compare with getInputChannelName() which returns descriptive names like "Vx1 (±5V)".
 *
 * @param channel  Input channel enum value
 * @return         Label if set, otherwise short name: "Vx1", "GND", "VrefRaw", etc.
 */
const char* getChannelShortName(InputChannel channel) {
  uint8_t idx = static_cast<uint8_t>(channel);
  if (idx < 8 && channelLabels[idx][0] != '\0') {
    return channelLabels[idx];
  }
  switch (channel) {
    case InputChannel::Vx1:     return "Vx1";
    case InputChannel::GND:     return "GND";
    case InputChannel::VrefRaw: return "VrefRaw";
    case InputChannel::Vx2:     return "Vx2";
    case InputChannel::Vx3:     return "Vx3";
    case InputChannel::HVDiv:   return "HVDiv";
    case InputChannel::Vx4:     return "Vx4";
    case InputChannel::Vx5:     return "Vx5";
    default:                    return "Unknown";
  }
}

// Forward declarations for command handlers and shared helpers
bool runOneChopCycle(LowerMoments &stats, int32_t* bA, int32_t* bB, bool searchOnOverflow = true);
void performAutoZero();
void printCsvHeader();
void cmdLabel(const char* arg1, const char* arg2);

void cmdScanAdd(const char* arg) {
  InputChannel ch;
  if (!parseChannelName(arg, ch)) {
    Serial.print("ERROR: Unknown channel '");
    Serial.print(arg);
    Serial.println("'. Use: Vx1, GND, VrefRaw, Vx2-Vx5, HVDiv");
    return;
  }
  // GND is reserved for auto-zero, not meaningful as a scan channel
  if (ch == InputChannel::GND) {
    Serial.println("ERROR: GND cannot be added to scan list (use 'autozero on' instead).");
    return;
  }
  // Check for duplicates
  for (int i = 0; i < scanner.config.count; i++) {
    if (scanner.config.channels[i] == ch) {
      Serial.println("Channel already in scan list.");
      return;
    }
  }
  if (scanner.config.count >= MAX_SCAN_CHANNELS) {
    Serial.println("ERROR: Scan list full (max 7 channels).");
    return;
  }
  scanner.config.channels[scanner.config.count++] = ch;
  Serial.print("Added ");
  Serial.print(getChannelShortName(ch));
  Serial.print(" to scan list (");
  Serial.print(scanner.config.count);
  Serial.println(" channels).");
}

void cmdScanRemove(const char* arg) {
  InputChannel ch;
  if (!parseChannelName(arg, ch)) {
    Serial.print("ERROR: Unknown channel '");
    Serial.print(arg);
    Serial.println("'");
    return;
  }
  for (int i = 0; i < scanner.config.count; i++) {
    if (scanner.config.channels[i] == ch) {
      // Shift remaining channels down
      for (int j = i; j < scanner.config.count - 1; j++) {
        scanner.config.channels[j] = scanner.config.channels[j + 1];
      }
      scanner.config.count--;
      Serial.print("Removed ");
      Serial.print(getChannelShortName(ch));
      Serial.println(" from scan list.");
      return;
    }
  }
  Serial.println("Channel not in scan list.");
}

void cmdScanList() {
  Serial.print("Scan list (");
  Serial.print(scanner.config.count);
  Serial.println(" channels):");
  for (int i = 0; i < scanner.config.count; i++) {
    Serial.print("  ");
    Serial.print(i + 1);
    Serial.print(". ");
    Serial.println(getInputChannelName(scanner.config.channels[i]));
  }
  if (scanner.config.count == 0) {
    Serial.println("  (empty)");
  }
}

/**
 * Begin automatic channel scanning. Called by processCommand() on "scan start",
 * and by setup() when configAutoStart is true.
 *
 * Initializes the scanning state machine that runs in loop():
 *   1. Resets scan index, cycle counters
 *   2. Switches mux to first channel and runs binarySearchDAC()
 *   3. Clears per-channel stats so integration starts fresh
 *   4. Emits CSV header if in CSV output mode
 *
 * After this, loop() enters the SCANNING branch on each iteration.
 */
void cmdScanStart() {
  if (scanner.config.count == 0) {
    Serial.println("ERROR: Scan list is empty. Add channels first.");
    return;
  }
  scanner.start();
  allanDev.clear();

  // Switch to first channel
  InputChannel firstCh = scanner.config.channels[0];
  inputMux.select(firstCh);
  uint8_t idx = (uint8_t)firstCh;
  if (scanner.channelDacValid(idx)) {
    dac.setCode(scanner.channelDacCode(idx));
  } else {
    binarySearchDAC();
    scanner.setChannelDac(idx, dac.currentCode());
  }
  scanner.channelStats(idx).clear();

  // Print CSV header if in CSV mode
  if (scanner.config.outputMode == OutputMode::CSV) {
    printCsvHeader();
  }

  Serial.println("Scanning started.");
}

void cmdScanStop() {
  scanner.stop();
  scanner.setChopCycleCount(0);
  Serial.println("Scanning stopped.");
}

void cmdLogStart() {
  scanner.config.outputMode = OutputMode::CSV;
  Serial.println("CSV logging enabled.");
  if (scanner.isScanning()) {
    printCsvHeader();
  }
}

void cmdLogStop() {
  scanner.config.outputMode = OutputMode::Human;
  Serial.println("CSV logging disabled, human-readable output restored.");
}

void cmdPlotStart() {
  scanner.config.outputMode = OutputMode::Plotter;
  Serial.println("Serial Plotter output enabled.");
}

void cmdPlotStop() {
  scanner.config.outputMode = OutputMode::Human;
  Serial.println("Serial Plotter output disabled, human-readable output restored.");
}

// Wall-clock seconds per chop cycle, modeled exactly from chopper.toggle() + the
// stop/settle/start gap + the (DISCARD + samples) readSample24 calls per half-cycle.
// Used by cmdIntegrate, cmdSetSamples, and computeTau0 so all three agree.
static double chopCycleSeconds() {
  const double fixedUs = (double)(CHOP_SETTLE_US + CI_PRE_EDGE_US + CI_POST_EDGE_US);
  const double samplesUs = (double)(DISCARD_SAMPLES + g_good_samples) * (double)EST_TS_US;
  return 2.0 * (fixedUs + samplesUs) * 1e-6;
}

// Compute measurement interval (tau0) in seconds for current integration setting
static double computeTau0() {
  return scanner.config.integrationCycles * chopCycleSeconds();
}

void cmdIntegrate(const char* arg) {
  // Argument is a target integration time in seconds. Internally translates
  // to integrationCycles using current OSR + samples so the wall-clock per
  // reading is preserved across `samples` changes.
  float secs = atof(arg);
  if (secs < 0.1f || secs > 600.0f) {
    Serial.println("ERROR: Integration time must be 0.1-600 seconds.");
    return;
  }
  const float tau0 = (float)chopCycleSeconds();
  int n = (int)roundf(secs / tau0);
  if (n < 10)    n = 10;
  if (n > 10000) n = 10000;
  scanner.config.integrationCycles = n;
  const float actualSec = n * tau0;
  Serial.print("Integration set to ");
  Serial.print(actualSec, 2);
  Serial.print(" sec per reading (");
  Serial.print(n);
  Serial.println(" chop cycles).");
}

void cmdSetSamples(const char *arg) {
  int n = atoi(arg);
  if (n < 1 || n > MAX_SAMPLES) {
    Serial.print("ERROR: Samples must be 1-"); Serial.print(MAX_SAMPLES); Serial.println(".");
    return;
  }
  // Preserve wall-clock per reading: rescale integrationCycles using the exact
  // chop-cycle model (chopCycleSeconds accounts for DISCARD + samples + fixed overhead).
  const double oldSec = scanner.config.integrationCycles * chopCycleSeconds();
  g_good_samples = n;
  const double newTau0 = chopCycleSeconds();
  int newCycles = (int)round(oldSec / newTau0);
  if (newCycles < 10)    newCycles = 10;
  if (newCycles > 10000) newCycles = 10000;
  scanner.config.integrationCycles = newCycles;

  Serial.print("Samples set to ");
  Serial.print(n);
  Serial.print(" per half cycle. Integration rescaled to ");
  Serial.print(newCycles);
  Serial.print(" chop cycles (~");
  Serial.print(newCycles * newTau0, 2);
  Serial.println(" sec per reading).");
  scanner.resetAllChannelEwma();
  allanDev.clear();
  Serial.println("EWMA + Allan deviation history cleared as a side-effect.");
}

void cmdAutoZero(const char* arg) {
  if (strcasecmp(arg, "on") == 0) {
    scanner.config.autoZeroEnabled = true;
    Serial.println("Auto-zero enabled.");
    Serial.println("WARNING: Auto-zero re-converges the DAC to near code 0 during GND measurement.");
    Serial.println("  The DAC cal table is typically uncalibrated near code 0, so the DAC's");
    Serial.println("  true near-zero offset is stored as a spurious correction applied to all");
    Serial.println("  readings. This degrades accuracy when 'cal dac' is used at measurement codes.");
    Serial.println("  Recommended: keep autozero off when using DAC calibration.");
  } else if (strcasecmp(arg, "off") == 0) {
    scanner.config.autoZeroEnabled = false;
    Serial.println("Auto-zero disabled.");
  } else {
    Serial.println("Usage: autozero on|off");
  }
}

void cmdAutoZeroInterval(const char* arg) {
  int n = atoi(arg);
  if (n < 1 || n > 1000) {
    Serial.println("ERROR: Auto-zero interval must be 1-1000 cycles.");
    return;
  }
  scanner.config.autoZeroInterval = n;
  Serial.print("Auto-zero interval set to ");
  Serial.print(n);
  Serial.println(" cycles.");
}

void cmdRange(const char* arg) {
  int r = atoi(arg);
  switch (r) {
    case 10:
      divMux.select(DividerRatio::Div10);
      Serial.println("Range set to +/-50V (div 10).");
      break;
    case 100:
      divMux.select(DividerRatio::Div100);
      Serial.println("Range set to +/-500V (div 100).");
      break;
    case 1000:
      divMux.select(DividerRatio::Div1000);
      Serial.println("Range set to +/-5kV (div 1000).");
      break;
    default:
      Serial.println("Usage: range 10|100|1000");
      break;
  }
}

void printStatus() {
  Serial.println("\n=== DiffVM Status ===");

  Serial.print("State: ");
  Serial.println(scanner.isScanning() ? "SCANNING" : "ONE_CHANNEL");

  Serial.print("Output mode: ");
  switch (scanner.config.outputMode) {
    case OutputMode::Human:  Serial.println("Human-readable"); break;
    case OutputMode::CSV:    Serial.println("CSV logging"); break;
    case OutputMode::Plotter: Serial.println("Serial Plotter"); break;
  }

  Serial.print("Integration: ");
  Serial.print(scanner.config.integrationCycles);
  Serial.print(" chop cycles (");
  Serial.print(computeTau0(), 3);
  Serial.print(" s/reading; tau0=");
  Serial.print(chopCycleSeconds() * 1000.0, 2);
  Serial.print(" ms; samples=");
  Serial.print(g_good_samples);
  Serial.print(", DISCARD=");
  Serial.print(DISCARD_SAMPLES);
  Serial.print(", OSR=");
  Serial.print((int)OSR_TOTAL);
  Serial.println(")");

  Serial.print("Auto-zero: ");
  Serial.print(scanner.config.autoZeroEnabled ? "ON" : "OFF");
  Serial.print(", interval=");
  Serial.print(scanner.config.autoZeroInterval);
  Serial.println(" cycles");

  Serial.print("EWMA window: ");
  Serial.print(scanner.channelEwma(0).window(), 1);
  Serial.println(" (equivalent SMA length)");

  if (scanner.autoZeroValid) {
    Serial.print("Auto-zero offset: ");
    Serial.print(scanner.autoZeroOffset * 1e9, 1);
    Serial.print(" nV  sdev=");
    Serial.print(scanner.autoZeroSdev * 1e9, 1);
    Serial.println(" nV");
  }

  Serial.print("Current channel: ");
  Serial.println(getInputChannelName(inputMux.current()));

  Serial.print("Current DAC code: ");
  Serial.println(dac.currentCode());

  Serial.print("Divider ratio: ");
  Serial.println(getDividerRatioName(divMux.current()));

  Serial.print("Preamp gain: ");
  Serial.println(PREAMP_GAIN, 4);
  Serial.print("Divider ratios: ÷10=");
  Serial.print(DIVIDER_RATIO_10, 4);
  Serial.print("  ÷100=");
  Serial.print(DIVIDER_RATIO_100, 4);
  Serial.print("  ÷1000=");
  Serial.println(DIVIDER_RATIO_1000, 4);
  Serial.print("OPA828 offset: ");
  Serial.print(OPA828_OFFSET * 1e6, 2); Serial.println(" uV");

  Serial.print("DAC calibration: ");
  Serial.println(dacCalTable.isValid() ? "CALIBRATED" : "nominal");

  cmdScanList();
  Serial.println("=====================\n");
}

void printHelp() {
  Serial.println("\n=== DiffVM Commands ===");
  Serial.println("scan add <ch>          Add channel (Vx1,GND,VrefRaw,Vx2-Vx5,HVDiv)");
  Serial.println("scan remove <ch>       Remove channel from scan list");
  Serial.println("scan list              Show current scan list");
  Serial.println("scan clear             Clear scan list");
  Serial.println("scan start             Begin automatic scanning");
  Serial.println("scan stop              Stop scanning");
  Serial.println("log start              Start CSV logging output");
  Serial.println("log stop               Stop CSV logging");
  Serial.println("plot start             Start Arduino Serial Plotter output");
  Serial.println("plot stop              Stop plotter output");
  Serial.println("integrate <secs>       Set integration time per reading (0.1-600 s)");
  Serial.println("samples <n>            Set samples per half chop (1-20)");
  Serial.println("drj [<k>]              Demod outlier rejection k (k=10000 disables; no arg = show)");
  Serial.println("autozero on|off        Enable/disable periodic auto-zero");
  Serial.println("autozero interval <N>  Cycles between auto-zero (readings in ONE_CHANNEL, sweeps in SCANNING) 1-1000");
  Serial.println("range <10|100|1000>    Set HV divider ratio");
  Serial.println("status                 Show current configuration");
  Serial.println("zero                   Perform immediate zero calibration");
  Serial.println("config save            Save config to EEPROM");
  Serial.println("config load            Load config from EEPROM");
  Serial.println("config factory         Reset to factory defaults");
  Serial.println("config show            Show saved vs current config");
  Serial.println("config autostart on|off  Auto-start scanning on boot");
  Serial.println("adev [clear]           Show Allan deviation (or clear history)");
  Serial.println("ewma <n>               Set EWMA window equivalent (default 10); 'ewma reset|clear' to clear");
  Serial.println("cal status             Show calibration flash status and current values");
  Serial.println("cal save               Save constants + DAC table to flash");
  Serial.println("cal load               Reload constants + DAC table from flash");
  Serial.println("cal erase              Erase calibration files from flash");
  Serial.println("cal set gain <v>       Set PREAMP_GAIN (e.g. 1998.5)");
  Serial.println("cal set adcvref <v>    Set ADC VREF voltage (measure at ADS127L11 pin, e.g. 2.4993)");
  Serial.println("cal set div10 <v>      Set DIVIDER_RATIO_10");
  Serial.println("cal set div100 <v>     Set DIVIDER_RATIO_100");
  Serial.println("cal set div1000 <v>    Set DIVIDER_RATIO_1000");
  Serial.println("cal set opa828 <v>     Set OPA828 buffer offset in volts (e.g. 0.000016)");
  Serial.println("cal factory            Reset cal constants to factory defaults (RAM)");
  Serial.println("cal build dac          Auto-calibrate DAC table at GND and VrefRaw anchors");
  Serial.println("cal point <voltage>    Capture DAC table entry at known Vx (e.g., cal point 1.23456)");
  Serial.println("cal dac <code> <v>     Store measured DAC voltage directly (e.g., cal dac 0 -0.0003515)");
  Serial.println("label list             List all channel labels");
  Serial.println("label <ch> <text>      Set label for channel (max 15 chars, no spaces)");
  Serial.println("label <ch>             Show label for channel");
  Serial.println("label reset            Clear all labels and save to flash");
  Serial.println("hold a|b               Force chopper state for scope probing (any key to exit)");
  Serial.println("debug [on|off]         Toggle binary-search trace prints (default off)");
  Serial.println("help                   Show this help");
  Serial.println("=======================\n");
}

void cmdAdev(const char* arg) {
  if (arg && strcasecmp(arg, "clear") == 0) {
    allanDev.clear();
    Serial.println("Allan deviation history cleared.");
    return;
  }
  if (allanDev.readingCount() < 3) {
    Serial.println("Need at least 3 readings for Allan deviation.");
    return;
  }
  allanDev.printResults(computeTau0());
}

// Debug: force chopper into state A or B and hold there so the AD8428 inputs
// can be probed with a scope. Uses the CI-protected toggle path (same as the
// chopped measurement loop). Leaves DAC code and input mux untouched. Prints
// the ADC sample every ~500 ms so a railed preamp is obvious. Exits on any
// incoming serial character and restores the previous chopper state.
void cmdHold(const char* arg) {
  if (!arg || (strcasecmp(arg, "a") != 0 && strcasecmp(arg, "b") != 0)) {
    Serial.println("Usage: hold a|b   (force chopper state for scope probing; any key to exit)");
    return;
  }
  bool wantB = (strcasecmp(arg, "b") == 0);
  bool savedState = chopper.state();

  digitalWriteFast(PIN_TMUX_EN, LOW);
  if (chopper.state() != wantB) chopper.toggle();

  adc.stop();
  delayMicroseconds(SETTLE_US);
  adc.start();
  delay(20);

  Serial.print("Holding chopper in state ");
  Serial.print(wantB ? 'B' : 'A');
  Serial.print(" (DAC code="); Serial.print(dac.currentCode());
  Serial.print(", MUXSEL="); Serial.print(wantB ? "HIGH" : "LOW");
  Serial.println("). Press any key to exit.");

  uint32_t lastPrint = millis();
  while (Serial.available() == 0) {
    if (millis() - lastPrint >= 500) {
      lastPrint = millis();
      int32_t s;
      if (adc.readSample24(s)) {
        Serial.print("  state="); Serial.print(wantB ? 'B' : 'A');
        Serial.print(" ADC="); Serial.println(s);
      } else {
        Serial.println("  (ADC read failed)");
      }
    }
  }
  Serial.read();  // consume wake-up character

  if (chopper.state() != savedState) chopper.toggle();
  Serial.println("Hold released.");
}

// Forward declarations for config commands (implemented in EEPROM section)
void cmdConfigSave();
void cmdConfigLoad();
void cmdConfigFactory();
void cmdConfigAutostart(const char* arg);
void cmdConfigShow();

// cmdCal() is defined after CalibrationStore (further below); forward declare here.
void cmdCal(const char* arg1, const char* arg2, const char* arg3);

/**
 * Parse and execute a complete command line from the serial buffer.
 * Called by processSerialCommands() when a newline is received.
 *
 * Tokenizes the line into command + up to 2 arguments, then dispatches
 * to the appropriate cmd*() handler function. Unrecognized commands
 * print an error message with a hint to type "help".
 *
 * @param line  Null-terminated command string (modified in place by strtok)
 */
void processCommand(char* line) {
  // Trim leading whitespace
  while (*line == ' ' || *line == '\t') line++;
  if (*line == '\0' || *line == '#') return;  // Empty or comment

  // Tokenize: first word is command
  char* cmd = strtok(line, " \t");
  char* arg1 = strtok(NULL, " \t");
  char* arg2 = strtok(NULL, " \t");
  char* arg3 = strtok(NULL, " \t");

  if (strcasecmp(cmd, "scan") == 0) {
    if (!arg1) { Serial.println("Usage: scan add|remove|list|clear|start|stop"); return; }
    if (strcasecmp(arg1, "add") == 0) {
      if (!arg2) { Serial.println("Usage: scan add <channel>"); return; }
      cmdScanAdd(arg2);
    } else if (strcasecmp(arg1, "remove") == 0) {
      if (!arg2) { Serial.println("Usage: scan remove <channel>"); return; }
      cmdScanRemove(arg2);
    } else if (strcasecmp(arg1, "list") == 0) {
      cmdScanList();
    } else if (strcasecmp(arg1, "clear") == 0) {
      scanner.config.count = 0;
      Serial.println("Scan list cleared.");
    } else if (strcasecmp(arg1, "start") == 0) {
      cmdScanStart();
    } else if (strcasecmp(arg1, "stop") == 0) {
      cmdScanStop();
    } else {
      Serial.println("Unknown scan subcommand. Use: add, remove, list, clear, start, stop");
    }
  } else if (strcasecmp(cmd, "log") == 0) {
    if (!arg1) { Serial.println("Usage: log start|stop"); return; }
    if (strcasecmp(arg1, "start") == 0) cmdLogStart();
    else if (strcasecmp(arg1, "stop") == 0) cmdLogStop();
    else Serial.println("Usage: log start|stop");
  } else if (strcasecmp(cmd, "plot") == 0) {
    if (!arg1) { Serial.println("Usage: plot start|stop"); return; }
    if (strcasecmp(arg1, "start") == 0) cmdPlotStart();
    else if (strcasecmp(arg1, "stop") == 0) cmdPlotStop();
    else Serial.println("Usage: plot start|stop");
  } else if (strcasecmp(cmd, "integrate") == 0) {
    if (!arg1) { Serial.println("Usage: integrate <0.1-600 seconds>"); return; }
    cmdIntegrate(arg1);
  } else if (strcasecmp(cmd, "autozero") == 0) {
    if (!arg1) { Serial.println("Usage: autozero on|off | autozero interval <N>"); return; }
    if (strcasecmp(arg1, "interval") == 0) {
      if (!arg2) { Serial.println("Usage: autozero interval <1-1000>"); return; }
      cmdAutoZeroInterval(arg2);
    } else {
      cmdAutoZero(arg1);
    }
  } else if (strcasecmp(cmd, "range") == 0) {
    if (!arg1) { Serial.println("Usage: range 10|100|1000"); return; }
    cmdRange(arg1);
  } else if (strcasecmp(cmd, "status") == 0) {
    printStatus();
  } else if (strcasecmp(cmd, "zero") == 0) {
    Serial.println("Performing zero calibration...");
    performAutoZero();
    if (scanner.autoZeroValid) {
      Serial.print("Zero offset: ");
      Serial.print(scanner.autoZeroOffset * 1e9, 1);
      Serial.print(" nV  sdev=");
      Serial.print(scanner.autoZeroSdev * 1e9, 1);
      Serial.println(" nV");
    }
  } else if (strcasecmp(cmd, "config") == 0) {
    if (!arg1) { Serial.println("Usage: config save|load|factory|autostart|show"); return; }
    if (strcasecmp(arg1, "save") == 0) cmdConfigSave();
    else if (strcasecmp(arg1, "load") == 0) cmdConfigLoad();
    else if (strcasecmp(arg1, "factory") == 0) cmdConfigFactory();
    else if (strcasecmp(arg1, "autostart") == 0) {
      if (!arg2) { Serial.println("Usage: config autostart on|off"); return; }
      cmdConfigAutostart(arg2);
    } else if (strcasecmp(arg1, "show") == 0) cmdConfigShow();
    else Serial.println("Usage: config save|load|factory|autostart|show");
  } else if (strcasecmp(cmd, "ewma") == 0) {
    if (!arg1 || strcasecmp(arg1, "reset") == 0 || strcasecmp(arg1, "clear") == 0) {
      scanner.resetAllChannelEwma();
      scanner.autoZeroEwma_.reset();
      Serial.println("EWMA filters reset.");
    } else {
      float w = atof(arg1);
      if (w < 1.0f || w > 10000.0f) {
        Serial.println("Usage: ewma [<window>|reset]  (window: 1..10000, default 10)");
      } else {
        scanner.setEwmaWindow(w);
        scanner.config.ewmaWindow = w;
        Serial.print("EWMA window set to "); Serial.print(w, 1); Serial.println(" (all filters reset)");
      }
    }
  } else if (strcasecmp(cmd, "adev") == 0) {
    cmdAdev(arg1);
  } else if (strcasecmp(cmd, "cal") == 0) {
    cmdCal(arg1, arg2, arg3);
  } else if (strcasecmp(cmd, "label") == 0) {
    cmdLabel(arg1, arg2);
  } else if (strcasecmp(cmd, "hold") == 0) {
    cmdHold(arg1);
  } else if (strcasecmp(cmd, "debug") == 0) {
    if (!arg1) {
      Serial.print("Debug: ");
      Serial.println(g_debugDac ? "on" : "off");
    } else if (strcasecmp(arg1, "on") == 0) {
      g_debugDac = true;
      Serial.println("Debug ON (binary-search trace).");
    } else if (strcasecmp(arg1, "off") == 0) {
      g_debugDac = false;
      Serial.println("Debug OFF.");
    } else {
      Serial.println("Usage: debug [on|off]");
    }
  } else if (strcasecmp(cmd, "help") == 0) {
    printHelp();
  } else if(strcasecmp(cmd, "samples")==0) {
    if (!arg1) { Serial.println("Usage: samples <1-20>"); return; }
    cmdSetSamples(arg1);
  } else if (strcasecmp(cmd, "drj") == 0) {
    if (!arg1) {
      Serial.print("drj k = "); Serial.print(g_demodRejKx10 / 10.0, 1);
      Serial.print("  (rej ");
      Serial.print((unsigned long)g_demodRejTotal); Serial.print('/');
      Serial.print((unsigned long)g_demodPairsTotal);
      Serial.println(" cum)  Usage: drj <k>  (k=10000 disables)");
      return;
    }
    double kf = atof(arg1);
    if (kf < 0.1) { Serial.println("k must be >= 0.1"); return; }
    g_demodRejKx10 = (int32_t)(kf * 10.0 + 0.5);
    Serial.print("Demod outlier rejection: k = ");
    Serial.println(g_demodRejKx10 / 10.0, 1);
   } else {
    Serial.print("Unknown command: '");
    Serial.print(cmd);
    Serial.println("'. Type 'help' for available commands.");
  }
}

/**
 * Read characters from USB Serial and process complete lines.
 * Called at the top of every loop() iteration. Accumulates characters into
 * cmdBuffer[] until a newline is received, then passes the complete line
 * to processCommand(). Non-blocking: returns immediately if no data available.
 *
 * Line buffer is 128 bytes; excess characters are silently dropped.
 */
void processSerialCommands() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (cmdBufferIdx > 0) {
        cmdBuffer[cmdBufferIdx] = '\0';
        processCommand(cmdBuffer);
        cmdBufferIdx = 0;
      }
    } else if (cmdBufferIdx < (int)sizeof(cmdBuffer) - 1) {
      cmdBuffer[cmdBufferIdx++] = c;
    }
  }
}

// ================== POST (Power-On Self-Test) ==================

// Decode and print the ADS127L11 STATUS register in plain English.
// Bit 7 CS_MODE, 6 ALV_FLAG, 5 POR_FLAG, 4 SPI_ERR, 3 REG_ERR,
// 2 ADC_ERR, 1 MOD_FLAG (modulator saturation), 0 DRDY.
static void printAdcStatus(uint8_t s) {
  Serial.print("  STATUS=0x");
  if (s < 0x10) Serial.print('0');
  Serial.print(s, HEX);
  Serial.print(": ");
  if (s == 0x00) { Serial.println("all clear"); return; }
  bool comma = false;
  auto flag = [&](bool set, const char* label) {
    if (!set) return;
    if (comma) Serial.print(", ");
    Serial.print(label);
    comma = true;
  };
  flag(s & 0x80, "CS_MODE");
  flag(s & 0x40, "ALV_FLAG");
  flag(s & 0x20, "POR_FLAG");
  flag(s & 0x10, "SPI_ERR");
  flag(s & 0x08, "REG_ERR");
  flag(s & 0x04, "ADC_ERR");
  flag(s & 0x02, "MOD_FLAG(modulator saturated)");
  flag(s & 0x01, "DRDY");
  Serial.println();
}

/**
 * POST Test 1: ADC Communication
 * Reads the DEV_ID register and verifies SPI communication works.
 * Also verifies CONFIG registers can be written and read back.
 */
bool postTestAdcComm() {
  Serial.print("POST: ADC Comm... ");

  // Read and decode STATUS register first — shows reset/error state before any writes
  uint8_t status = adc.readReg(REG_STATUS);
  printAdcStatus(status);

  // Read device ID register
  uint8_t devId = adc.readReg(REG_DEV_ID);

  // Write a test value to CONFIG1, read it back
  const uint8_t testVal = 0x55;
  uint8_t origVal = adc.readReg(REG_CONFIG1);
  adc.writeReg(REG_CONFIG1, testVal);
  uint8_t readBack = adc.readReg(REG_CONFIG1);
  adc.writeReg(REG_CONFIG1, origVal);  // Restore original

  bool pass = (readBack == testVal);

  if (pass) {
    Serial.print("PASS (DEV_ID=0x");
    Serial.print(devId, HEX);
    Serial.println(")");
  } else {
    Serial.print("FAIL (wrote 0x");
    Serial.print(testVal, HEX);
    Serial.print(", read 0x");
    Serial.print(readBack, HEX);
    Serial.println(")");
  }
  return pass;
}

/**
 * POST Test 2: DAC Communication
 * Writes to DAC control register and reads back via SDO.
 * AD5760 supports register readback with R/W bit = 1.
 */
bool postTestDacComm() {
  Serial.print("POST: DAC Comm... ");

  // AD5760 register readback: send read command then NOP to clock out the response.
  // readControlRaw() handles the two-frame exchange and returns the 20-bit value.
  uint32_t ctrl = dac.readControlRaw();

  // Check: not 0xFFFFF (20-bit all-ones = MISO floating / SDO not connected)
  bool pass = (ctrl != 0xFFFFF);

  if (pass) {
    Serial.print("PASS (ctrl=0x");
    Serial.print(ctrl, HEX);
    Serial.println(")");
  } else {
    Serial.print("FAIL (ctrl=0x");
    Serial.print(ctrl, HEX);
    Serial.println(")");
  }
  return pass;
}

/**
 * POST Test 3: DRDY Timing — also the canonical ADC clock self-calibration.
 *
 * Measures the DRDY pulse rate over many cycles, derives effective OSR from
 * scope-verified PWM-register-implied fMOD, and sets the runtime timing
 * constants EST_FSPS / EST_TS_US / ADC_FCLK_HZ / ADC_FMOD_HZ / OSR_TOTAL.
 *
 * Validation: the derived OSR must fall in a plausible range. If polling
 * dropped many DRDY edges (short pulse width at high fMOD), OSR reads way
 * high; if the filter is misconfigured or the ADC clock is dead, things go
 * way out of range. The plausibility window catches real faults without
 * depending on a hardcoded expected rate that's wrong whenever F_CPU changes.
 *
 * Why one measurement, not two: a separate calibrateAdcClock() used to run
 * before POST. The two measurements would sometimes disagree by 10-25% due
 * to occasional missed short DRDY pulses, then POST would fail its tolerance
 * check against the (already-wrong) calibrate output. Single source of truth
 * eliminates that.
 */
bool postTestDrdyTiming() {
  Serial.print("POST: DRDY Timing... ");
  adc.start();
  Serial.print("(DRDY pin "); Serial.print(PIN_ADC_DRDY);
  Serial.print(" = "); Serial.print(digitalReadFast(PIN_ADC_DRDY) ? "HIGH" : "LOW");
  Serial.print(") ");

  // Derive expected fCLK from FlexPWM register state. Scope-verified to match
  // hardware output at F_CPU=450. fMOD = fCLK/2 in high-speed mode.
  int32_t  pwm_init   = (int16_t)FLEXPWM2_SM2INIT;
  int32_t  pwm_val1   = (int16_t)FLEXPWM2_SM2VAL1;
  uint32_t pwm_prsc   = (FLEXPWM2_SM2CTRL >> 4) & 0x7;
  int32_t  period_cnt = pwm_val1 - pwm_init + 1;
  float    fclk_pwm = (float)F_BUS_ACTUAL / ((float)period_cnt * (float)(1u << pwm_prsc));
  float    fmod_pwm = fclk_pwm / 2.0f;

  // Per-edge timeout: 10 ms is generous for any plausible DRDY period
  // (steady-state ~1 ms; pathological slow cases ~3 ms).
  const uint32_t EDGE_TIMEOUT_US = 10000;
  elapsedMicros timeout = 0;

  // Wait for DRDY to be HIGH so we can lock onto a clean falling edge next.
  timeout = 0;
  while (digitalReadFast(PIN_ADC_DRDY) == LOW && timeout < EDGE_TIMEOUT_US) {}
  if (timeout >= EDGE_TIMEOUT_US) { Serial.println("FAIL (DRDY stuck LOW)"); return false; }

  // Warm up: discard 5 cycles to clear any sinc cascade fill latency from
  // ADC restart and any USB/IRQ startup transients.
  for (int i = 0; i < 5; i++) {
    timeout = 0;
    while (digitalReadFast(PIN_ADC_DRDY) == HIGH && timeout < EDGE_TIMEOUT_US) {}
    if (timeout >= EDGE_TIMEOUT_US) { Serial.println("FAIL (warmup fall timeout)"); return false; }
    timeout = 0;
    while (digitalReadFast(PIN_ADC_DRDY) == LOW && timeout < EDGE_TIMEOUT_US) {}
    if (timeout >= EDGE_TIMEOUT_US) { Serial.println("FAIL (warmup rise timeout)"); return false; }
  }

  // Time many cycles. 500 swamps occasional polling-missed edges over ~500 ms.
  const int N = 500;
  elapsedMicros elapsed = 0;
  for (int i = 0; i < N; i++) {
    timeout = 0;
    while (digitalReadFast(PIN_ADC_DRDY) == HIGH && timeout < EDGE_TIMEOUT_US) {}
    if (timeout >= EDGE_TIMEOUT_US) { Serial.println("FAIL (DRDY stalled high)"); return false; }
    timeout = 0;
    while (digitalReadFast(PIN_ADC_DRDY) == LOW && timeout < EDGE_TIMEOUT_US) {}
    if (timeout >= EDGE_TIMEOUT_US) { Serial.println("FAIL (DRDY stalled low)"); return false; }
  }
  uint32_t totalUs = elapsed;

  float periodUs = (float)totalUs / (float)N;
  float freqHz   = 1e6f / periodUs;
  float osr_eff  = fmod_pwm / freqHz;

  // Plausibility check. Spec window covers ADS127L11 OSR options (32..32768)
  // with margin. A wildly out-of-range OSR means either many edges missed or
  // the filter config doesn't match what we think.
  bool pass = (fclk_pwm >= 1e6f && fclk_pwm <= 26e6f &&
               osr_eff  >= 100.0f && osr_eff <= 100000.0f &&
               freqHz   >= 50.0f  && freqHz   <= 5000.0f);

  if (pass) {
    EST_FSPS    = freqHz;
    EST_TS_US   = periodUs;
    ADC_FCLK_HZ = fclk_pwm;
    ADC_FMOD_HZ = fmod_pwm;
    OSR_TOTAL   = osr_eff;

    Serial.print("PASS (fCLK="); Serial.print(fclk_pwm/1e6f, 3);
    Serial.print(" MHz, fMOD="); Serial.print(fmod_pwm/1e6f, 3);
    Serial.print(" MHz, DRDY="); Serial.print(freqHz, 1);
    Serial.print(" Hz, Ts=");    Serial.print(periodUs, 1);
    Serial.print(" us, OSR=");   Serial.print(osr_eff, 0);
    Serial.println(")");
  } else {
    Serial.print("FAIL (fCLK="); Serial.print(fclk_pwm/1e6f, 3);
    Serial.print(" MHz, DRDY="); Serial.print(freqHz, 1);
    Serial.print(" Hz, derived OSR="); Serial.print(osr_eff, 0);
    Serial.println(" — out of plausible range)");
  }

  // CONFIG2 speed_mode advisory if fMOD is below the nominal high-speed envelope.
  if (pass && fmod_pwm < 10e6f) {
    Serial.print("  NOTE: fMOD=");
    Serial.print(fmod_pwm/1e6f, 2);
    Serial.println(" MHz is below the nominal 12.5 MHz high-speed threshold;");
    Serial.println("        verify CONFIG2 speed_mode against the ADS127L11 datasheet.");
  }

  return pass;
}
//
// POST Test 3.5 Pin identification and verification
//
bool postPinIdVerify() {
  Serial.print("POST: Pin Id and verification (takes 20 seconds)... ");

  int width = 222; // mS
  digitalWriteFast(PIN_CI_TMUXSEL, LOW);
  digitalWriteFast(PIN_TMUXSEL,LOW);
  digitalWriteFast(PIN_TMUX_EN, LOW);
  // Loop ten times, each ~2 secs
  for (int i = 0; i < 10; i++) {
	delay(width);
	digitalWriteFast(PIN_TMUX_EN, HIGH);
	delay(width/2);
	digitalWriteFast(PIN_TMUXSEL,HIGH);
	digitalWriteFast(PIN_CI_TMUXSEL, HIGH);
	delay(width);
	digitalWriteFast(PIN_CI_TMUXSEL, LOW);
	delay(width);
	
	digitalWriteFast(PIN_TMUXSEL,LOW);
	
	digitalWriteFast(PIN_CI_TMUXSEL, HIGH);
	delay(width);
	digitalWriteFast(PIN_CI_TMUXSEL, LOW);
	delay(width);
	
	digitalWriteFast(PIN_CI_TMUXSEL, HIGH);
	delay(width);
	digitalWriteFast(PIN_CI_TMUXSEL, LOW);
	delay(width);
	  
    digitalWriteFast(PIN_TMUXSEL,HIGH);
    delay(width);
    digitalWriteFast(PIN_CI_TMUXSEL, LOW);
	delay(width/2);
	digitalWriteFast(PIN_TMUXSEL,LOW);
	digitalWriteFast(PIN_TMUX_EN, LOW);
  }
  Serial.println(" pid id done");
  chopper.begin(); // re-initialize
  return true;
}
// ===== Helpers shared by signal_chain / polarity / chop_symmetry tests ========
//
// Static single-phase reads at gain ×2000 saturate the ADC because the AD8428
// chain's offset (and charge stored in the Vcm filter cap) dominates a single
// phase's output. The chopper-demodulated value is the only stable observable;
// these tests therefore drive the chopper at normal cadence and read the
// demodulated mean from runOneChopCycle().
//
// Returns demod mean (in ADC LSB) on success, or NaN on overflow.
static double measureDemodMean(int cycles) {
  LowerMoments stats;
  int32_t buffA[MAX_SAMPLES];
  int32_t buffB[MAX_SAMPLES];
  for (int i = 0; i < cycles; ++i) {
    if (!runOneChopCycle(stats, buffA, buffB, false)) {
      return NAN;
    }
  }
  return stats.mean();
}

// DAC null code with mux=GND, found by postTestSignalChain via binarySearchDAC.
// postTestPolarity / postTestChopSymmetry use this null ±1 LSB so neither single
// phase rails (offset margin too small for larger steps even at gain ×2000).
// Reset to 0 if signal_chain fails — the dependent tests will then fail too,
// which is the correct cascade.
static int16_t s_postZeroDac = 0;

/**
 * POST Test 4: Signal Chain (DAC to ADC) — uses binarySearchDAC to find the null.
 * Convergence proves the entire chain (DAC → preamp → ADC) responds correctly.
 * Records the null code in s_postZeroDac for the next two tests.
 */
bool postTestSignalChain() {
  Serial.println("POST: Signal Chain...");
  ScopedInstrumentState guard;

  inputMux.select(InputChannel::GND);
  dac.setCode(0);  // 250 ms filter settling

  if (!binarySearchDAC()) {
    Serial.println("POST: Signal Chain FAIL (binarySearchDAC did not converge)");
    s_postZeroDac = 0;
    return false;
  }
  s_postZeroDac = dac.currentCode();

  // At the null, the demodulated reading should be near zero.
  const double demod = measureDemodMean(5);
  if (isnan(demod)) {
    Serial.println("POST: Signal Chain FAIL (overflow during measurement at null)");
    return false;
  }

  // Loose demod bound: chain has natural chopping-residual offset (~250 µV RTI ~= 1.7 M LSB)
  // that auto-zero handles. Threshold catches "totally broken" not "needs auto-zero".
  static constexpr double  DEMOD_THRESHOLD = 5'000'000.0;
  static constexpr int16_t ZERO_DAC_BOUND  = 200;   // expect single-digit LSB at GND
  const bool pass = (fabs(demod) < DEMOD_THRESHOLD) && (abs(s_postZeroDac) < ZERO_DAC_BOUND);

  Serial.print("POST: Signal Chain ");
  Serial.print(pass ? "PASS" : "FAIL");
  Serial.print(" (zeroDac="); Serial.print(s_postZeroDac);
  Serial.print(", demod="); Serial.print(demod, 0);
  Serial.println(")");
  return pass;
}

/**
 * POST Test 5: Polarity — chop-demodulated, ±1 LSB around the discovered null.
 * Demod = (Vx - DAC) × gain, so stepping DAC up swings demod down. Confirms the
 * sign of the chain end-to-end.
 */
bool postTestPolarity() {
  Serial.print("POST: Polarity... ");
  ScopedInstrumentState guard;
  inputMux.select(InputChannel::GND);

  const int16_t zero = s_postZeroDac;

  dac.setCode(zero - 1);
  LowerMoments prime;
  int32_t buffA[MAX_SAMPLES];
  int32_t buffB[MAX_SAMPLES];
  
  if (!runOneChopCycle(prime, buffA, buffB, false)) { Serial.println("FAIL (prime overflow @ zero-1)"); return false; }
  const double demodNeg = measureDemodMean(5);
  if (isnan(demodNeg)) { Serial.println("FAIL (overflow @ zero-1)"); return false; }

  dac.setCode(zero + 1);
  prime.clear();
  if (!runOneChopCycle(prime, buffA, buffB, false)) { Serial.println("FAIL (prime overflow @ zero+1)"); return false; }
  const double demodPos = measureDemodMean(5);
  if (isnan(demodPos)) { Serial.println("FAIL (overflow @ zero+1)"); return false; }

  const double delta = demodPos - demodNeg;
  const bool pass = (delta < -500000.0);
  Serial.print(pass ? "PASS" : "FAIL");
  Serial.print(" (zero="); Serial.print(zero);
  Serial.print(", demod@zero-1="); Serial.print(demodNeg, 0);
  Serial.print(", demod@zero+1="); Serial.print(demodPos, 0);
  Serial.print(", delta="); Serial.print(delta, 0);
  if (!pass) Serial.print(" — expected delta < -500000");
  Serial.println(")");
  return pass;
}

/**
 * POST Test 6: Chop Symmetry — magnitudes at zero±1 LSB should be equal-and-opposite.
 * Polarity catches sign errors; this catches duty-skew or charge-injection asymmetry
 * that polarity alone would miss.
 */
bool postTestChopSymmetry() {
  Serial.print("POST: Chop Symmetry... ");
  ScopedInstrumentState guard;
  inputMux.select(InputChannel::GND);

  const int16_t zero = s_postZeroDac;

  // 3-point slope linearity around the null. The chain has a natural ~262 µV RTI
  // residual offset (~-1.76M LSB demod), so symmetry-around-zero fails on a healthy
  // unit. Slopes between adjacent DAC codes are independent of that offset.
  dac.setCode(zero - 1);
  LowerMoments prime;
  int32_t buffA[MAX_SAMPLES];
  int32_t buffB[MAX_SAMPLES];
  
  if (!runOneChopCycle(prime, buffA, buffB, false)) { Serial.println("FAIL (prime overflow @ zero-1)"); return false; }
  const double demodNeg = measureDemodMean(5);
  if (isnan(demodNeg)) { Serial.println("FAIL (overflow @ zero-1)"); return false; }

  dac.setCode(zero); 
  prime.clear();
  if (!runOneChopCycle(prime, buffA, buffB, false)) { Serial.println("FAIL (prime overflow @ zero)"); return false; }
  const double demodMid = measureDemodMean(5);
  if (isnan(demodMid)) { Serial.println("FAIL (overflow @ zero)"); return false; }

  dac.setCode(zero + 1);
  prime.clear();
  if (!runOneChopCycle(prime, buffA, buffB, false)) { Serial.println("FAIL (prime overflow @ zero+1)"); return false; }
  const double demodPos = measureDemodMean(5);
  if (isnan(demodPos)) { Serial.println("FAIL (overflow @ zero+1)"); return false; }

  const double slopeLeft  = demodMid - demodNeg;   // expect ~-1.0M LSB / LSB DAC
  const double slopeRight = demodPos - demodMid;
  const double slopeAvgMag = (fabs(slopeLeft) + fabs(slopeRight)) / 2.0;
  const double asym = (slopeAvgMag > 0) ? fabs(slopeLeft - slopeRight) / slopeAvgMag : 1.0;

  const bool pass = (slopeLeft < -500000.0) && (slopeRight < -500000.0) && (asym < 0.10);
  Serial.print(pass ? "PASS" : "FAIL");
  Serial.print(" (slopeL="); Serial.print(slopeLeft, 0);
  Serial.print(", slopeR="); Serial.print(slopeRight, 0);
  Serial.print(", asym="); Serial.print(asym * 100.0, 1); Serial.print("%");
  Serial.println(")");
  return pass;
}

/**
 * POST Test 7: Input Mux
 * Verifies input multiplexer can switch channels.
 * Selects GND input and verifies ADC reads near zero.
 */
bool postTestInputMux() {
  Serial.print("POST: Input Mux... ");
  ScopedInstrumentState guard;

  dac.setCode(0);
  inputMux.select(InputChannel::GND);

  // Stop/start to flush sinc5 filter history. Chop Symmetry now waits 150ms after its
  // restore-toggle so the preamp is settled; a small discard count is sufficient here.
  static constexpr uint8_t INMUX_DISCARD = 5;
  adc.stop();
  delayMicroseconds(SETTLE_US);
  adc.start();
  int32_t s;
  Serial.print("  GND settle trace: ");
  for (uint8_t i = 0; i < INMUX_DISCARD; i++) {
    if (!adc.readSample24(s)) { Serial.println("FAIL (ADC timeout)"); return false; }
    if (i == 0 || i == 4 || i == 9 || i == INMUX_DISCARD - 1) {
      Serial.print(i); Serial.print(":"); Serial.print(s); Serial.print("  ");
    }
  }
  Serial.println();
  int64_t sum = 0;
  const int numSamples = 5;
  for (int i = 0; i < numSamples; i++) {
    if (!adc.readSample24(s)) { Serial.println("FAIL (ADC timeout)"); return false; }
    sum += s;
  }
  int32_t avg = sum / numSamples;
  adc.stop();

  // With GND and DAC=0, preamp sees ~0V differential. Allow 50% of full scale
  // for un-chopped preamp offset (AD8428 max ~200µV × 2000 gain = 400mV).
  const int32_t threshold = 4'000'000;
  bool pass = (avg > -threshold) && (avg < threshold);

  Serial.print("  ");
  Serial.print(pass ? "PASS" : "FAIL");
  Serial.print(" (GND="); Serial.print(avg); Serial.println(")");
  return pass;
}

/**
 * POST Test 8: Reference Verification + DAC Settling Characterization
 * Two things in one test, sharing the long VrefRaw filter wait:
 *  1) Measure how long the DAC's RC filter takes to bring the preamp output
 *     to within {1 mV, 100 µV, 10 µV, 1 µV} (input-referred) of its final
 *     value after a 0 → +FS step. Used to tune RC_SETTLE_MS in
 *     binarySearchDAC and the wait between DAC code changes elsewhere.
 *  2) Verify VrefRaw (pre-filter ADR1001 average at J3) is near the expected
 *     5 V, using the settled value from step (1) as the reading.
 *
 * Setup: VrefRaw provides a stable +5 V at the (-) input. With DAC parked
 * at 0 V the preamp is positively saturated; as the DAC steps to +5 V the
 * preamp comes out of saturation and the tail of the RC settle is
 * observable. Only the tail is visible (preamp linear range is ±1.25 mV at
 * the input), but that's the part that matters for chop accuracy.
 */
bool postTestReferences() {
  Serial.println("POST: Reference + DAC Settle...");
  ScopedInstrumentState guard;

  // Park DAC at 0 V and switch the (-) input to VrefRaw. VrefRaw RC filter
  // and the DAC at 0 share the long warm-up wait.
  dac.setCode(0);
  inputMux.select(InputChannel::VrefRaw);
  Serial.print("  VrefRaw filter settling (15 s)... ");
  delay(1000 * 15);
  Serial.println("done");

  // --- DAC settling characterization ---
  // Capture timestamped ADC samples after a 0 → +FS step. The DAC RC filter
  // was recently modified and τ is unknown, so we use a long window with
  // decimated storage: every ~20 ms over 30 s. Storage is function-static
  // (BSS) to avoid stack pressure (~12 KB).
  static constexpr uint32_t PROBE_WINDOW_US   = 30UL * 1000000UL;          // 30 s
  static constexpr int      PROBE_N           = 1500;
  static constexpr uint32_t STORE_INTERVAL_US = PROBE_WINDOW_US / PROBE_N; // 20 ms
  static constexpr uint32_t MID_REFLUSH_US    = 400000;                   // 400 ms
  static int32_t  probeSample[PROBE_N];
  static uint32_t probeUs[PROBE_N];

  // Flush ADC sinc memory. Match the dance used by readSettledSample (proven
  // to produce clean first samples) rather than the shorter SETTLE_US.
  adc.stop();
  delayMicroseconds(SETTLE_US + TMUX_RECONNECT_US);
  adc.start();

  // Sanity check #1: with DAC=0 and VrefRaw=+5V, the preamp should be at one
  // rail. Take 5 prestep samples — if they all agree at the rail, the ADC is
  // tracking input correctly. If they vary or disagree with expectation, the
  // ADC or input path is the suspect (not the DAC RC).
  Serial.print("  Pre-step (DAC=0, expect rail): ");
  int32_t prestepMin =  INT32_MAX, prestepMax = INT32_MIN;
  for (int i = 0; i < 5; i++) {
    int32_t s;
    if (!adc.readSample24(s)) {
      Serial.println("\n  FAIL (ADC timeout pre-step)");
      return false;
    }
    Serial.print(s); Serial.print(' ');
    if (s < prestepMin) prestepMin = s;
    if (s > prestepMax) prestepMax = s;
  }
  Serial.print("(spread="); Serial.print(prestepMax - prestepMin); Serial.println(")");

  const int16_t finalCode = 32767;  // the exception to the 14 bit alignment
  const uint32_t t0 = micros();
  dac.setCodeFast(finalCode);  // bypasses built-in settle delay; we time it ourselves

  // Sanity check #2: at MID_REFLUSH_US, do an in-flight stop/start of the ADC.
  // If post-reflush samples diverge from the values just before, the sinc
  // filter was stuck on stale modulator state. If they agree, the input is
  // genuinely still where it was, and the DAC RC really is that slow.
  int reflushIdx = -1;
  bool reflushed = false;

  // Survive transient DRDY timeouts so we get the partial trace + know where.
  // Each timeout logs the elapsed time and attempts a stop/start recovery.
  // Bail only after MAX_TIMEOUTS consecutive failures.
  static constexpr int MAX_TIMEOUTS = 4;
  int timeoutCount = 0;

  int n = 0;
  uint32_t nextStoreUs = 0;
  int32_t s = 0;
  while (n < PROBE_N) {
    if (!adc.readSample24(s)) {
      uint32_t now = micros() - t0;
      Serial.print("  [t="); Serial.print(now / 1000); Serial.print(" ms] DRDY timeout #");
      Serial.print(timeoutCount + 1); Serial.println(", recovering");
      timeoutCount++;
      if (timeoutCount >= MAX_TIMEOUTS) {
        Serial.println("  Too many timeouts, dumping partial trace");
        break;
      }
      adc.stop();
      delayMicroseconds(SETTLE_US + TMUX_RECONNECT_US);
      adc.start();
      continue;
    }
    timeoutCount = 0;  // reset on any successful read
    uint32_t now = micros() - t0;
    if (now >= nextStoreUs) {
      probeSample[n] = s;
      probeUs[n]     = now;
      n++;
      nextStoreUs += STORE_INTERVAL_US;
    }
    if (!reflushed && now >= MID_REFLUSH_US) {
      reflushIdx = n;  // first index AFTER the reflush
      adc.stop();
      delayMicroseconds(SETTLE_US + TMUX_RECONNECT_US);
      adc.start();
      reflushed = true;
    }
    if (now >= PROBE_WINDOW_US) break;
  }
  adc.stop();

  // Estimate the settled value from the last 10% of the trace.
  const int settledStart = (n * 9) / 10;
  int64_t sumSettled = 0;
  for (int i = settledStart; i < n; i++) sumSettled += probeSample[i];
  const int32_t settled = (n > settledStart) ? (int32_t)(sumSettled / (n - settledStart)) : 0;
  const uint32_t windowEndUs = (n > 0) ? probeUs[n - 1] : 0;

  Serial.print("  DAC step 0->code "); Serial.print(finalCode);
  Serial.print(" (settled="); Serial.print(settled);
  Serial.print(", N="); Serial.print(n);
  Serial.print(", window="); Serial.print(windowEndUs / 1000); Serial.println(" ms)");

  // Reflush bracket — print the sample just before the reflush and the first
  // sample after, to expose any sinc-filter stickiness.
  if (reflushIdx > 0 && reflushIdx < n) {
    Serial.print("  Reflush @ ~"); Serial.print(MID_REFLUSH_US / 1000); Serial.print(" ms: ");
    Serial.print("pre=");  Serial.print(probeSample[reflushIdx - 1]);
    Serial.print(" post="); Serial.print(probeSample[reflushIdx]);
    Serial.print(" delta="); Serial.println(probeSample[reflushIdx] - probeSample[reflushIdx - 1]);
  }

  // Sparse trace dump — sample closest to each requested time so we can see
  // the shape of the settle even before threshold analysis.
  const uint32_t traceTimesMs[] = {1, 10, 100, 500, 1000, 2000, 5000, 10000, 20000, 30000};
  Serial.println("  Trace:");
  for (uint32_t targetMs : traceTimesMs) {
    uint32_t targetUs = targetMs * 1000UL;
    int closest = -1;
    uint32_t bestDist = UINT32_MAX;
    for (int i = 0; i < n; i++) {
      uint32_t d = (probeUs[i] > targetUs) ? probeUs[i] - targetUs : targetUs - probeUs[i];
      if (d < bestDist) { bestDist = d; closest = i; }
    }
    if (closest < 0) continue;
    Serial.print("    t="); Serial.print(probeUs[closest] / 1000);
    Serial.print(" ms: ADC="); Serial.print(probeSample[closest]);
    Serial.print(" (err="); Serial.print(probeSample[closest] - settled); Serial.println(")");
  }

  // Threshold crossings (input-referred). Counts = (Vinput × PREAMP_GAIN) / ADC_LSB_V.
  // Reported time = LAST violation in the trace — i.e., the wait needed
  // before subsequent samples stay within the threshold of the settled value.
  const double countsPerVolt = PREAMP_GAIN / ADC_LSB_V;
  struct ThrSpec { const char *name; double inputV; };
  const ThrSpec thresholds[] = {
    { "1mV   ", 1.0e-3 },
    { "100uV ", 1.0e-4 },
    { "10uV  ", 1.0e-5 },
    { "1uV   ", 1.0e-6 },
  };
  for (auto &t : thresholds) {
    int32_t thrCounts = (int32_t)(t.inputV * countsPerVolt);
    uint32_t lastViolationUs = 0;
    bool everViolated = false;
    for (int i = 0; i < n; i++) {
      int32_t err = probeSample[i] - settled;
      if (err < 0) err = -err;
      if (err > thrCounts) { lastViolationUs = probeUs[i]; everViolated = true; }
    }
    Serial.print("    "); Serial.print(t.name);
    Serial.print("(\xc2\xb1"); Serial.print(thrCounts); Serial.print(" cts) -> ");
    if (!everViolated) {
      Serial.println("never violated");
    } else if (lastViolationUs >= windowEndUs) {
      Serial.print(">"); Serial.print(windowEndUs / 1000); Serial.println(" ms (window exhausted)");
    } else {
      Serial.print(lastViolationUs / 1000); Serial.print(".");
      Serial.print((lastViolationUs / 100) % 10); Serial.println(" ms");
    }
  }

  // --- Reference verification ---
  // DAC is now at +FS, mux on VrefRaw. The settled value above IS the
  // post-warm-up reference reading; reuse it rather than reading again.
  const int32_t reading = settled;
  const int32_t refThreshold = 5'000'000;  // ~60% of 8,388,607
  const bool pass = (reading > -refThreshold) && (reading < refThreshold);

  Serial.print("POST: Reference: ");
  if (pass) {
    Serial.print("PASS (VrefRaw="); Serial.print(reading); Serial.println(")");
  } else {
    Serial.print("FAIL (VrefRaw="); Serial.print(reading);
    Serial.print(", expected near 0, threshold=\xc2\xb1");
    Serial.print(refThreshold); Serial.println(")");
  }
  return pass;
}

/**
 * POST Test 9: Zero Calibration
 * Switches input mux to GND, performs chopped measurements, and verifies
 * the result is within the configured threshold of zero.
 * This tests the entire signal chain's zero offset using chopping.
 */
bool postTestZero() {
  Serial.print("POST: Zero Cal... ");
  ScopedInstrumentState guard;

  // Switch to GND input and set DAC to 0V
  inputMux.select(InputChannel::GND);
  dac.setCode(2);

  // Accumulate chopped measurements for accurate zero reading
  LowerMoments zeroStats;
  int32_t buffA[MAX_SAMPLES];
  int32_t buffB[MAX_SAMPLES];
  for (int i = 0; i < 10; i++) {
    if (!runOneChopCycle(zeroStats, buffA, buffB, false)) {
      Serial.println("FAIL (overflow during zero measurement)");
      return false;  // guard restores state
    }
  }

  // guard restores channel + DAC on scope exit

  // Convert mean to nanovolts at input
  double zeroMean = zeroStats.mean();
  double zeroNv = (zeroMean * ADC_LSB_V / PREAMP_GAIN) * 1e9;  // Convert to nV

  bool pass = (fabs(zeroNv) < POST_ZERO_THRESHOLD_NV);

  if (pass) {
    Serial.print("PASS (zero=");
    Serial.print(zeroNv, 1);
    Serial.print(" nV, threshold=±");
    Serial.print(POST_ZERO_THRESHOLD_NV, 0);
    Serial.println(" nV)");
  } else {
    Serial.print("FAIL (zero=");
    Serial.print(zeroNv, 1);
    Serial.print(" nV, threshold=±");
    Serial.print(POST_ZERO_THRESHOLD_NV, 0);
    Serial.println(" nV)");
  }
  return pass;
}

/**
 * Run all POST tests and return results.
 */
PostResult runPOST() {
  Serial.println("========== POST (Power-On Self-Test) ==========");

  PostResult result;
  result.adc_comm = postTestAdcComm();
  result.dac_comm = postTestDacComm();
  result.drdy_timing = postTestDrdyTiming();
  result.pin_id      = true; // postPinIdVerify(); // optional
  result.signal_chain = postTestSignalChain();
  result.polarity = postTestPolarity();
  result.chop_symmetry = postTestChopSymmetry();
  result.input_mux = postTestInputMux();
  // Bypassed: the VrefRaw node is unbuffered, so the preamp's input current
  // sags the reference during the measurement and the test always fails.
  // Re-enable once a hardware buffer is installed.
  result.references = true;  // postTestReferences();
  result.zero_cal = postTestZero();

  Serial.println("================================================");
  if (result.allPassed()) {
    Serial.println("POST: ALL TESTS PASSED");
  } else {
    Serial.println("POST: *** FAILURES DETECTED ***");
  }
  Serial.println("================================================");

  return result;
}

// ================== Reference Drift Compensation ==================

/**
 * Initialize reference tracking state.
 * Call this at startup before entering the main loop.
 */
void initRefTracking() {
  for (int i = 0; i < FILTER_HISTORY_SIZE; i++) {
    filterHistory[i].error = 0.0;
    filterHistory[i].timestamp = 0;
  }
  filterHistoryIdx = 0;
  estimatedDriftRate = 0.0;
  refSampleCounter = 0;
  refTrackingInitialized = false;
}

/**
 * Measure the current filter error by comparing VrefRaw (J3) against the DAC reference.
 *
 * The DAC is referenced to TP1 (filter output), so measuring J3 (filter input)
 * against the DAC gives us (J3 - TP1) directly - the filter error.
 *
 * This function temporarily switches the input mux and DAC, performs multiple
 * chopped measurements for accuracy, updates the filter history, and restores
 * the previous state.
 */
void measureFilterError() {
  ScopedInstrumentState guard;

  // Switch to raw reference input
  inputMux.select(InputChannel::VrefRaw);

  // Set DAC to ~5V to measure against the reference
  dac.setCode(REF_MEASURE_DAC_CODE);

  // Accumulate multiple chopped measurements for stability
  LowerMoments refStats;
  int32_t buffA[MAX_SAMPLES];
  int32_t buffB[MAX_SAMPLES];
  for (int i = 0; i < REF_MEASURE_ITERATIONS; i++) {
    // searchOnOverflow=false: DAC is deliberately set to ~5V; let caller handle unexpected overflow
    if (!runOneChopCycle(refStats, buffA, buffB, false)) {
      Serial.println("WARNING: Overflow during reference measurement!");
      return;  // guard restores state
    }
  }

  // Convert mean to volts: this IS the filter error (J3 - TP1) × preamp_gain
  double filterError = (refStats.mean() * ADC_LSB_V) / PREAMP_GAIN;

  // Update drift rate estimate using previous measurement
  uint32_t now = millis();
  int prevIdx = (filterHistoryIdx - 1 + FILTER_HISTORY_SIZE) % FILTER_HISTORY_SIZE;

  if (filterHistory[prevIdx].timestamp > 0) {
    double dt = (now - filterHistory[prevIdx].timestamp) / 1000.0;
    if (dt > 0.1) {  // Avoid division by very small numbers
      double prevError = filterHistory[prevIdx].error;
      // The change in filter error relates to drift rate and filter decay:
      // dError/dt ≈ driftRate - error/τ_eff
      // So: driftRate ≈ dError/dt + error/τ_eff
      double dError = filterError - prevError;
      estimatedDriftRate = dError / dt + prevError / REF_FILTER_TAU_EFF;
    }
  }

  // Store new measurement in history
  filterHistory[filterHistoryIdx].error = filterError;
  filterHistory[filterHistoryIdx].timestamp = now;
  filterHistoryIdx = (filterHistoryIdx + 1) % FILTER_HISTORY_SIZE;

  refTrackingInitialized = true;
  // guard restores channel + DAC on scope exit
}

/**
 * Predict the current filter error based on historical measurements.
 *
 * Uses the known filter step response to extrapolate from the last measurement
 * to the current time. Also accounts for continued drift creating new error.
 *
 * @return Predicted filter error (J3 - TP1) in volts
 */
double predictFilterError() {
  if (!refTrackingInitialized) {
    return 0.0;  // No data yet
  }

  // Get most recent measurement
  int lastIdx = (filterHistoryIdx - 1 + FILTER_HISTORY_SIZE) % FILTER_HISTORY_SIZE;
  double lastError = filterHistory[lastIdx].error;
  uint32_t lastTime = filterHistory[lastIdx].timestamp;

  if (lastTime == 0) {
    return 0.0;  // No valid measurement
  }

  uint32_t now = millis();
  double dt = (now - lastTime) / 1000.0;

  // The measured error decays as the filter catches up.
  // For a step change that created the error, decay follows (1 - h(t))
  // where h(t) is the step response.

  // Look up decay factor from step response table
  int idx = (int)(dt / REF_STEP_DT);
  double stepVal;
  if (idx >= REF_STEP_LEN) {
    stepVal = 1.0;  // Fully settled
  } else if (idx < 0) {
    stepVal = 0.0;
  } else {
    // Linear interpolation between table entries
    float frac = (dt / REF_STEP_DT) - idx;
    if (idx + 1 < REF_STEP_LEN) {
      stepVal = REF_STEP_RESPONSE[idx] + frac * (REF_STEP_RESPONSE[idx + 1] - REF_STEP_RESPONSE[idx]);
    } else {
      stepVal = REF_STEP_RESPONSE[idx];
    }
  }

  // Decay factor: how much of the original error remains
  double decayFactor = 1.0 - stepVal;
  double decayedError = lastError * decayFactor;

  // New error accumulates from continued drift
  // For constant drift rate r, the steady-state error is r × τ_eff
  // The new error builds up with the complement of the decay
  double newError = estimatedDriftRate * REF_FILTER_TAU_EFF * stepVal;

  return decayedError + newError;
}

/**
 * Get the correction factor to apply to voltage measurements.
 *
 * The filter error causes the DAC reference to differ from the true reference.
 * All measurements need to be scaled by (trueRef / actualRef) = (J3 / TP1).
 *
 * @return Multiplicative correction factor (typically very close to 1.0)
 */
double getRefCorrectionFactor() {
  double filterErr = predictFilterError();
  // Vx_true = Vx_measured × (J3 / TP1) = Vx_measured × (1 + filterErr / TP1)
  return 1.0 + filterErr / NOMINAL_REF_V;
}

// ================== CSV / Plotter Output ==================
// ================== Output Formatting (Strategy Pattern) ==================
//
// Three output backends, selected by OutputMode in scanner.config:
//   Human   - multi-line text with SI prefixes
//   CSV     - header + comma-separated values per measurement
//   Plotter - tab-separated label:value pairs for Arduino Serial Plotter
//
// All single-channel/ONE_CHANNEL output is routed through outputMeasurement(),
// which delegates to the active formatter via getFormatter(). Multi-channel
// Plotter mode is assembled directly in the SCANNING branch of loop() using
// beginSweep()/formatMeasurement()/endSweep().

struct MeasurementData {
  InputChannel channel;
  double voltage;
  double uncertainty;
  double adcMean;
  double adcStdDev;
  double count;
  int16_t dacCode;
  double driftPpb;
  double adcMin;
  double adcMax;
  double ewmaVoltage = 0.0;
  bool   ewmaValid   = false;
};

class IOutputFormatter {
public:
  virtual ~IOutputFormatter() = default;
  virtual void formatMeasurement(const MeasurementData& data) = 0;
  virtual void beginSweep() {}
  virtual void endSweep() {}
};

class HumanFormatter : public IOutputFormatter {
public:
  void formatMeasurement(const MeasurementData& data) override {
    char vxBuf[32], uncBuf[32];
    formatVoltage(data.voltage, vxBuf, sizeof(vxBuf), 9);
    formatVoltage(data.uncertainty, uncBuf, sizeof(uncBuf), 3);

    Serial.print(getInputChannelName(data.channel));
    Serial.print(" = ");
    Serial.print(vxBuf);
    Serial.print(" +/- ");
    Serial.print(uncBuf);
    Serial.print(" (n=");
    Serial.print(data.count, 0);
    Serial.print(", DAC=");
    Serial.print(data.dacCode);
    Serial.print(")");
    if (data.ewmaValid) {
      char ewmaBuf[32];
      formatVoltage(data.ewmaVoltage, ewmaBuf, sizeof(ewmaBuf), 9);
      Serial.print("  ewma=");
      Serial.print(ewmaBuf);
    }
    Serial.println();

    Serial.print("  raw: mean=");
    Serial.print(data.adcMean, 3);
    Serial.print(", sd=");
    Serial.print(data.adcStdDev, 3);
    Serial.print(", min=");
    Serial.print(data.adcMin, 3);
    Serial.print(", max=");
    Serial.println(data.adcMax, 3);

    if (SHOW_DRIFT) {
      double filterErr = predictFilterError();
      Serial.print("  drift: filterErr=");
      Serial.print(filterErr * 1e9, 1);
      Serial.print(" nV, driftRate=");
      Serial.print(estimatedDriftRate * 1e9, 3);
      Serial.print(" nV/s, correction=");
      Serial.print(data.driftPpb, 3);
      Serial.println(" ppb");
    }

    if (scanner.autoZeroValid) {
      Serial.print("  zero: raw=");
      Serial.print(scanner.autoZeroRaw * 1e9, 1);
      Serial.print(" nV  ewma=");
      Serial.print(scanner.autoZeroOffset * 1e9, 1);
      Serial.print(" nV  sdev=");
      Serial.print(scanner.autoZeroSdev * 1e9, 1);
      Serial.println(" nV");
    }

    if (SHOW_ADEV && allanDev.readingCount() >= 3) {
      double tau0 = computeTau0();
      Serial.print("  adev: ");
      int printed = 0;
      for (int k = 0; k < allanDev.numOctaves() && printed < 6; k++) {
        double ad = allanDev.getAdev(k);
        if (!std::isnan(ad)) {
          char adBuf[24];
          formatVoltage(ad, adBuf, sizeof(adBuf), 3);
          Serial.print("tau=");
          Serial.print(allanDev.getTau(k, tau0), 1);
          Serial.print("s:");
          Serial.print(adBuf);
          Serial.print("  ");
          printed++;
        }
      }
      Serial.println();
    }
  }
};

void printCsvHeader() {
  Serial.println("# timestamp_ms,channel,voltage_V,uncertainty_V,stddev_counts,mean_counts,n,dac_code,drift_ppb");
}

class CsvFormatter : public IOutputFormatter {
public:
  void formatMeasurement(const MeasurementData& data) override {
    Serial.print(millis());
    Serial.print(',');
    Serial.print(getChannelShortName(data.channel));
    Serial.print(',');
    Serial.print(data.voltage, 12);
    Serial.print(',');
    Serial.print(data.uncertainty, 12);
    Serial.print(',');
    Serial.print(data.adcStdDev, 3);
    Serial.print(',');
    Serial.print(data.adcMean, 3);
    Serial.print(',');
    Serial.print((int)data.count);
    Serial.print(',');
    Serial.print(data.dacCode);
    Serial.print(',');
    Serial.print(data.driftPpb, 3);
    Serial.println();
  }
};

class PlotterFormatter : public IOutputFormatter {
public:
  void formatMeasurement(const MeasurementData& data) override {
    if (lineStarted_) Serial.print('\t');
    Serial.print(getChannelShortName(data.channel));
    Serial.print(':');
    Serial.print(data.voltage, 12);
    bool singleChannel = scanner.config.count <= 1 || scanner.isOneChannel();
    if (singleChannel) {
      Serial.print('\t');
      Serial.print(getChannelShortName(data.channel));
      Serial.print("_sd:");
      Serial.print(data.uncertainty, 12);
    }
    lineStarted_ = true;
  }

  void beginSweep() override { lineStarted_ = false; }

  void endSweep() override {
    Serial.println();
    lineStarted_ = false;
  }

private:
  bool lineStarted_ = false;
};

static HumanFormatter humanFmt;
static CsvFormatter csvFmt;
static PlotterFormatter plotterFmt;

IOutputFormatter& getFormatter() {
  switch (scanner.config.outputMode) {
    case OutputMode::CSV:     return csvFmt;
    case OutputMode::Plotter: return plotterFmt;
    default:                  return humanFmt;
  }
}

// ================== Auto-Zero ==================

/**
 * Perform an auto-zero calibration by measuring the GND input channel.
 *
 * Called by:
 *   - processCommand() on "zero" command (immediate manual calibration)
 *   - loop() ONE_CHANNEL branch every scanner.config.autoZeroInterval readings
 *   - loop() SCANNING branch every scanner.config.autoZeroInterval sweeps
 *
 * Procedure:
 *   1. Save current input channel and DAC code
 *   2. Switch mux to GND, run binarySearchDAC() to find DAC null point
 *   3. Run 20 chopped measurement cycles to determine offset
 *   4. Store result in scanner.autoZeroOffset (volts); set scanner.autoZeroValid = true
 *   5. Restore original channel and DAC code
 *
 * The measured offset is subtracted from all subsequent voltage readings
 * in outputMeasurement(). This corrects for residual DC offsets in the
 * signal chain that chopping doesn't fully cancel (e.g., thermal EMFs
 * in the mux path, DAC zero error).
 *
 * Takes ~200ms (20 chop cycles + DAC settling). During this time,
 * normal measurements are paused.
 */
void performAutoZero() {
  ScopedInstrumentState guard;

  // Switch to GND
  inputMux.select(InputChannel::GND);
  dac.setCode(0);

  // Run binary search for GND (should converge quickly near 0)
  binarySearchDAC();

  // Take two independent AUTOZERO_CYCLES accumulations; their spread is the repeatability sdev.
  LowerMoments repeatStats;
  for (int rep = 0; rep < 2; rep++) {
    LowerMoments gndStats;
	int32_t buffA[MAX_SAMPLES];
	int32_t buffB[MAX_SAMPLES];
    for (int i = 0; i < AUTOZERO_CYCLES; i++) {
      if (!runOneChopCycle(gndStats, buffA, buffB, false)) {
        Serial.println("WARNING: Overflow during auto-zero!");
        return;  // guard restores state
      }
    }
    repeatStats.accumulate(computeInputVoltage(gndStats.mean(), dac.currentCode(), InputChannel::GND));
  }

  double rawAz = repeatStats.mean();
  scanner.autoZeroRaw    = rawAz;
  scanner.autoZeroSdev   = repeatStats.standardDeviation();
  scanner.autoZeroOffset = scanner.autoZeroEwma_.update(rawAz);  // EWMA-filtered applied offset
  scanner.autoZeroValid  = true;
  // guard restores channel + DAC on scope exit
}

// ================== EEPROM Configuration ==================
//
// Teensy 4.1 provides 4284 bytes of emulated EEPROM. We store ~40 bytes
// at address 0 containing the full scan configuration, divider ratio,
// and auto-start flag. A magic number and CRC16 validate the data.
//
// Written by cmdConfigSave(), read by loadConfigFromEEPROM() at startup
// and by cmdConfigLoad()/cmdConfigShow().

static constexpr uint32_t EEPROM_MAGIC = 0xD1FF0002UL;  // "DIFF" v2 — adds ewmaWindow field
static constexpr int EEPROM_BASE_ADDR = 0;

/**
 * EEPROM-persisted configuration. Written as a flat byte blob.
 * The magic number and CRC16 checksum guard against reading uninitialized
 * EEPROM (first boot) or corrupted data.
 *
 * Total size: ~40 bytes (well within the 4284-byte EEPROM).
 */
struct SavedConfig {
  uint32_t magic;           // Must equal EEPROM_MAGIC for valid config
  ScanConfig scanConfig;    // Full scan configuration (channels, integration, etc.)
  bool autoStart;           // If true, cmdScanStart() runs automatically in setup()
  DividerRatio dividerRatio;  // HV divider ratio setting to restore
  uint16_t checksum;        // CRC16 over all preceding fields
};

// Runtime auto-start flag. Set by "config autostart on/off", persisted in SavedConfig.
static bool configAutoStart = false;

/**
 * Compute CRC16 (MODBUS variant) checksum over a byte buffer.
 * Used by cmdConfigSave() and loadConfigFromEEPROM() to detect corruption.
 * Polynomial: 0xA001 (bit-reversed 0x8005).
 */
uint16_t crc16(const uint8_t* data, size_t len) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (int j = 0; j < 8; j++) {
      if (crc & 1) crc = (crc >> 1) ^ 0xA001;
      else crc >>= 1;
    }
  }
  return crc;
}

// ================== CalibrationStore ==================
// Persists DAC calibration table and preamp/divider constants to LittleFS
// flash (512 KB partition on Teensy 4.1 QSPI flash).

static constexpr uint32_t CAL_CONSTS_MAGIC = 0xCA1C0001UL;  // "CALC" v1
static constexpr uint32_t CAL_TABLE_MAGIC  = 0xDACC0002UL;  // "DACC" v2 — adds maxCodeV_ slot for code 32767
static constexpr uint32_t CAL_LFS_SIZE     = 512UL * 1024UL; // 512 KB partition

class CalibrationStore {
public:
  // Mount LittleFS_Program; format if first use. Returns true on success.
  bool begin() {
    if (!fs_.begin(CAL_LFS_SIZE)) {
      return false;
    }
    mounted_ = true;
    return true;
  }

  bool isMounted() const { return mounted_; }

  // Write preampGain + divider ratios + OPA828 offset + ADC_VREF to /cal_consts.bin
  // Format v3: [4] magic [2] version=3 [8×6=48] six doubles [2] CRC16 = 56 bytes
  bool saveConstants() {
    if (!mounted_) return false;
    // Remove before writing: FILE_WRITE appends on LittleFS, so without this
    // the new data lands after the old bytes and the next load reads stale values.
    fs_.remove("/cal_consts.bin");
    File f = fs_.open("/cal_consts.bin", FILE_WRITE);
    if (!f) return false;

    uint8_t buf[56];
    size_t pos = 0;

    uint32_t magic = CAL_CONSTS_MAGIC;
    memcpy(buf + pos, &magic, 4); pos += 4;
    uint16_t ver = 3;
    memcpy(buf + pos, &ver, 2); pos += 2;
    memcpy(buf + pos, &PREAMP_GAIN, 8); pos += 8;
    memcpy(buf + pos, &DIVIDER_RATIO_10, 8); pos += 8;
    memcpy(buf + pos, &DIVIDER_RATIO_100, 8); pos += 8;
    memcpy(buf + pos, &DIVIDER_RATIO_1000, 8); pos += 8;
    memcpy(buf + pos, &OPA828_OFFSET, 8); pos += 8;
    memcpy(buf + pos, &ADC_VREF, 8); pos += 8;
    uint16_t crc = crc16(buf, pos);
    memcpy(buf + pos, &crc, 2); pos += 2;

    f.write(buf, pos);
    f.close();
    return true;
  }

  // Read constants from /cal_consts.bin and apply them if valid.
  // Supports v1 (40 bytes, 4 doubles), v2 (48 bytes, 5 doubles), v3 (56 bytes, 6 doubles + ADC_VREF).
  bool loadConstants() {
    if (!mounted_) return false;
    File f = fs_.open("/cal_consts.bin", FILE_READ);
    if (!f) return false;

    uint8_t buf[56];
    size_t n = f.read(buf, sizeof(buf));
    f.close();
    if (n < 40) return false;

    uint32_t magic; memcpy(&magic, buf, 4);
    if (magic != CAL_CONSTS_MAGIC) return false;

    uint16_t ver; memcpy(&ver, buf + 4, 2);

    double pg, d10, d100, d1000, opa828 = 0.0, adcvref = ADC_VREF_DEFAULT;

    if (ver == 1 && n >= 40) {
      uint16_t storedCrc; memcpy(&storedCrc, buf + 38, 2);
      if (crc16(buf, 38) != storedCrc) return false;
      memcpy(&pg,    buf + 6,  8);
      memcpy(&d10,   buf + 14, 8);
      memcpy(&d100,  buf + 22, 8);
      memcpy(&d1000, buf + 30, 8);
      opa828 = OPA828_OFFSET_DEFAULT;
    } else if (ver == 2 && n >= 48) {
      uint16_t storedCrc; memcpy(&storedCrc, buf + 46, 2);
      if (crc16(buf, 46) != storedCrc) return false;
      memcpy(&pg,     buf + 6,  8);
      memcpy(&d10,    buf + 14, 8);
      memcpy(&d100,   buf + 22, 8);
      memcpy(&d1000,  buf + 30, 8);
      memcpy(&opa828, buf + 38, 8);
    } else if (ver == 3 && n >= 56) {
      uint16_t storedCrc; memcpy(&storedCrc, buf + 54, 2);
      if (crc16(buf, 54) != storedCrc) return false;
      memcpy(&pg,      buf + 6,  8);
      memcpy(&d10,     buf + 14, 8);
      memcpy(&d100,    buf + 22, 8);
      memcpy(&d1000,   buf + 30, 8);
      memcpy(&opa828,  buf + 38, 8);
      memcpy(&adcvref, buf + 46, 8);
    } else {
      return false;
    }

    // Sanity bounds
    if (pg < 100.0 || pg > 100000.0) return false;
    if (d10 < 1.0  || d10 > 100.0)   return false;
    if (d100 < 10.0 || d100 > 10000.0) return false;
    if (d1000 < 100.0 || d1000 > 100000.0) return false;
    if (opa828 < -0.001 || opa828 > 0.001) return false;
    if (adcvref < 2.0 || adcvref > 3.0) return false;

    PREAMP_GAIN        = pg;
    DIVIDER_RATIO_10   = d10;
    DIVIDER_RATIO_100  = d100;
    DIVIDER_RATIO_1000 = d1000;
    OPA828_OFFSET      = opa828;
    ADC_VREF           = adcvref;
    ADC_LSB_V          = (2.0 * ADC_VREF) / ADC_FSR_CODES;
    return true;
  }

  // Write dacCalTable to /dac_table.bin using streaming to avoid 131 KB buffer.
  // Format: [4] magic [2] version [1] valid [1] reserved [131072] table [2] CRC16
  bool saveTable() {
    if (!mounted_) return false;
    if (!dacCalTable.isValid()) {
      Serial.println("cal save: DAC table not valid; not saving table.");
      return false;
    }
    fs_.remove("/dac_table.bin");
    File f = fs_.open("/dac_table.bin", FILE_WRITE);
    if (!f) return false;

    uint16_t crc = 0xFFFF; // Running CRC16

    // Header (8 bytes)
    uint8_t hdr[8];
    uint32_t magic = CAL_TABLE_MAGIC;
    memcpy(hdr, &magic, 4);
    uint16_t ver = 1; memcpy(hdr + 4, &ver, 2);
    hdr[6] = 1;  // valid flag
    hdr[7] = 0;  // reserved
    // Update CRC over header
    for (int i = 0; i < 8; i++) {
      crc ^= hdr[i];
      for (int b = 0; b < 8; b++) crc = (crc & 1) ? (crc >> 1) ^ 0xA001 : crc >> 1;
    }
    f.write(hdr, 8);

    // Stream table in 512-byte (64 double) chunks
    static constexpr size_t CHUNK = 512;
    uint8_t chunk[CHUNK];
    const uint8_t* tableBytes = reinterpret_cast<const uint8_t*>(dacCalTable.rawTable());
    size_t totalBytes = DacCalibrationTable::TABLE_SIZE * sizeof(double);

    for (size_t offset = 0; offset < totalBytes; offset += CHUNK) {
      size_t sz = min((size_t)CHUNK, totalBytes - offset);
      memcpy(chunk, tableBytes + offset, sz);
      for (size_t i = 0; i < sz; i++) {
        crc ^= chunk[i];
        for (int b = 0; b < 8; b++) crc = (crc & 1) ? (crc >> 1) ^ 0xA001 : crc >> 1;
      }
      f.write(chunk, sz);
    }

    // v2: write maxCodeV_ (cal point for absolute-max code 32767) into CRC and file
    uint8_t maxBuf[8];
    double maxV = dacCalTable.maxCodeVoltage();
    memcpy(maxBuf, &maxV, 8);
    for (int i = 0; i < 8; i++) {
      crc ^= maxBuf[i];
      for (int b = 0; b < 8; b++) crc = (crc & 1) ? (crc >> 1) ^ 0xA001 : crc >> 1;
    }
    f.write(maxBuf, 8);

    // Write final CRC
    uint8_t crcBuf[2];
    memcpy(crcBuf, &crc, 2);
    f.write(crcBuf, 2);
    f.close();
    return true;
  }

  // Read dacCalTable from /dac_table.bin; only applies if CRC valid.
  bool loadTable() {
    if (!mounted_) return false;
    File f = fs_.open("/dac_table.bin", FILE_READ);
    if (!f) return false;

    // Read and validate header
    uint8_t hdr[8];
    if (f.read(hdr, 8) != 8) { f.close(); return false; }

    uint32_t magic; memcpy(&magic, hdr, 4);
    if (magic != CAL_TABLE_MAGIC) { f.close(); return false; }
    if (hdr[6] != 1) { f.close(); return false; } // valid flag

    uint16_t crc = 0xFFFF;
    for (int i = 0; i < 8; i++) {
      crc ^= hdr[i];
      for (int b = 0; b < 8; b++) crc = (crc & 1) ? (crc >> 1) ^ 0xA001 : crc >> 1;
    }

    // Stream table data into dacCalTable
    static constexpr size_t CHUNK = 512;
    uint8_t chunk[CHUNK];
    uint8_t* tableBytes = reinterpret_cast<uint8_t*>(dacCalTable.rawTable());
    size_t totalBytes = DacCalibrationTable::TABLE_SIZE * sizeof(double);
    size_t offset = 0;

    while (offset < totalBytes) {
      size_t sz = min((size_t)CHUNK, totalBytes - offset);
      int got = f.read(chunk, sz);
      if (got != (int)sz) { f.close(); return false; }
      memcpy(tableBytes + offset, chunk, sz);
      for (size_t i = 0; i < sz; i++) {
        crc ^= chunk[i];
        for (int b = 0; b < 8; b++) crc = (crc & 1) ? (crc >> 1) ^ 0xA001 : crc >> 1;
      }
      offset += sz;
    }

    // v2: read maxCodeV_ slot
    uint8_t maxBuf[8];
    if (f.read(maxBuf, 8) != 8) { f.close(); return false; }
    for (int i = 0; i < 8; i++) {
      crc ^= maxBuf[i];
      for (int b = 0; b < 8; b++) crc = (crc & 1) ? (crc >> 1) ^ 0xA001 : crc >> 1;
    }
    double maxV; memcpy(&maxV, maxBuf, 8);
    dacCalTable.setMaxCodeVoltage(maxV);

    // Read and check stored CRC
    uint8_t crcBuf[2];
    if (f.read(crcBuf, 2) != 2) { f.close(); return false; }
    f.close();

    uint16_t storedCrc; memcpy(&storedCrc, crcBuf, 2);
    if (crc != storedCrc) {
      dacCalTable.initNominal(); // revert to safe state
      return false;
    }

    dacCalTable.markValid();
    return true;
  }

  // Write channelLabels[8][16] to /channel_labels.bin
  // Format: [4] magic [128] labels data [2] CRC16 = 134 bytes
  bool saveLabels() {
    if (!mounted_) return false;
    fs_.remove("/channel_labels.bin");
    File f = fs_.open("/channel_labels.bin", FILE_WRITE);
    if (!f) return false;

    static constexpr uint32_t LABELS_MAGIC = 0xC4BE0001UL;
    uint8_t buf[134];
    size_t pos = 0;
    memcpy(buf + pos, &LABELS_MAGIC, 4); pos += 4;
    memcpy(buf + pos, channelLabels, 128); pos += 128;
    uint16_t crc = crc16(buf, pos);
    memcpy(buf + pos, &crc, 2); pos += 2;

    f.write(buf, pos);
    f.close();
    return true;
  }

  // Read channelLabels from /channel_labels.bin if valid.
  bool loadLabels() {
    if (!mounted_) return false;
    File f = fs_.open("/channel_labels.bin", FILE_READ);
    if (!f) return false;

    static constexpr uint32_t LABELS_MAGIC = 0xC4BE0001UL;
    uint8_t buf[134];
    size_t n = f.read(buf, sizeof(buf));
    f.close();
    if (n < 134) return false;

    uint32_t magic; memcpy(&magic, buf, 4);
    if (magic != LABELS_MAGIC) return false;

    uint16_t storedCrc; memcpy(&storedCrc, buf + 132, 2);
    if (crc16(buf, 132) != storedCrc) return false;

    memcpy(channelLabels, buf + 4, 128);
    // Ensure null termination for each label slot
    for (int i = 0; i < 8; i++) channelLabels[i][CHANNEL_LABEL_MAX] = '\0';
    return true;
  }

  // Remove calibration files from flash.
  bool eraseAll() {
    if (!mounted_) return false;
    bool ok = true;
    if (fs_.exists("/cal_consts.bin"))      ok &= fs_.remove("/cal_consts.bin");
    if (fs_.exists("/dac_table.bin"))       ok &= fs_.remove("/dac_table.bin");
    if (fs_.exists("/channel_labels.bin"))  ok &= fs_.remove("/channel_labels.bin");
    return ok;
  }

  // Load constants, table, and labels from flash. Called from setup().
  bool autoLoad() {
    bool ok = false;
    if (loadConstants()) {
      Serial.println("  Cal constants: loaded from flash.");
      ok = true;
    } else {
      Serial.println("  Cal constants: not found or invalid; using defaults.");
    }
    if (loadTable()) {
      Serial.println("  DAC cal table: loaded from flash (CALIBRATED).");
      ok = true;
    } else {
      Serial.println("  DAC cal table: not found or invalid; using nominal.");
    }
    if (loadLabels()) {
      Serial.println("  Channel labels: loaded from flash.");
      ok = true;
    }
    return ok;
  }

  // Print status of calibration flash.
  void printStatus() {
    Serial.println("--- Calibration Flash ---");
    if (!mounted_) {
      Serial.println("  Flash: NOT MOUNTED");
      return;
    }
    Serial.println("  Flash: mounted (LittleFS_Program 512 KB)");
    Serial.print("  /cal_consts.bin:      ");
    Serial.println(fs_.exists("/cal_consts.bin") ? "present" : "absent");
    Serial.print("  /dac_table.bin:       ");
    Serial.println(fs_.exists("/dac_table.bin") ? "present" : "absent");
    Serial.print("  /channel_labels.bin:  ");
    Serial.println(fs_.exists("/channel_labels.bin") ? "present" : "absent");
    Serial.print("  PREAMP_GAIN:          "); Serial.println(PREAMP_GAIN, 6);
    Serial.print("  DIVIDER_RATIO_10:     "); Serial.println(DIVIDER_RATIO_10, 6);
    Serial.print("  DIVIDER_RATIO_100:    "); Serial.println(DIVIDER_RATIO_100, 6);
    Serial.print("  DIVIDER_RATIO_1000:   "); Serial.println(DIVIDER_RATIO_1000, 6);
    Serial.print("  OPA828_OFFSET:        "); Serial.print(OPA828_OFFSET * 1e6, 3); Serial.println(" uV");
    Serial.print("  DAC cal table:        ");
    Serial.println(dacCalTable.isValid() ? "CALIBRATED" : "nominal");
  }

private:
  LittleFS_Program fs_;
  bool mounted_ = false;
};

static CalibrationStore calStore;

/**
 * Set, show, or reset user-configurable channel labels.
 *
 * Usage:
 *   label <ch> <text>   Set label for channel; saves to flash. No spaces.
 *   label <ch>          Show current label for channel.
 *   label list          Show all 8 channels with their labels.
 *   label reset         Clear all labels and save to flash.
 *
 * Labels: alphanumeric + _-./  Max 15 chars, no spaces.
 */
void cmdLabel(const char* arg1, const char* arg2) {
  if (arg1 == nullptr || arg1[0] == '\0') {
    Serial.println("Usage: label <ch> [text]  |  label list  |  label reset");
    return;
  }

  // --- label list ---
  if (strcasecmp(arg1, "list") == 0) {
    Serial.println("Channel labels:");
    const char* chNames[] = { "Vx1", "GND", "VrefRaw", "Vx2", "Vx3", "HVDiv", "Vx4", "Vx5" };
    for (int i = 0; i < 8; i++) {
      Serial.print("  "); Serial.print(chNames[i]);
      Serial.print(": ");
      Serial.println(channelLabels[i][0] ? channelLabels[i] : "(none)");
    }
    return;
  }

  // --- label reset ---
  if (strcasecmp(arg1, "reset") == 0) {
    memset(channelLabels, 0, sizeof(channelLabels));
    calStore.saveLabels();
    Serial.println("All channel labels cleared and saved.");
    return;
  }

  // --- label <ch> [text] ---
  InputChannel ch;
  if (!parseChannelName(arg1, ch)) {
    Serial.print("ERROR: Unknown channel '");
    Serial.print(arg1);
    Serial.println("'. Use: Vx1, GND, VrefRaw, Vx2-Vx5, HVDiv");
    return;
  }
  uint8_t idx = static_cast<uint8_t>(ch);

  // Show label if no text given
  if (arg2 == nullptr || arg2[0] == '\0') {
    Serial.print(arg1); Serial.print(" label: ");
    Serial.println(channelLabels[idx][0] ? channelLabels[idx] : "(none)");
    return;
  }

  // Validate label text
  size_t len = strlen(arg2);
  if (len > CHANNEL_LABEL_MAX) {
    Serial.print("ERROR: Label too long (max ");
    Serial.print(CHANNEL_LABEL_MAX);
    Serial.println(" chars).");
    return;
  }
  for (size_t i = 0; i < len; i++) {
    char c = arg2[i];
    if (!isalnum(c) && c != '_' && c != '-' && c != '.' && c != '/') {
      Serial.println("ERROR: Label may only contain: A-Z a-z 0-9 _ - . /");
      return;
    }
  }

  // Set label and save
  strncpy(channelLabels[idx], arg2, CHANNEL_LABEL_MAX);
  channelLabels[idx][CHANNEL_LABEL_MAX] = '\0';
  calStore.saveLabels();
  Serial.print(arg1); Serial.print(" label set to: ");
  Serial.println(channelLabels[idx]);
}

/**
 * Auto-calibrate the DAC INL table at GND and VrefRaw anchor points.
 *
 * Sweeps CAL_BUILD_WINDOW table entries on each side of the binary-search
 * convergence code for both anchors, measures the actual DAC output voltage
 * at each code, and stores the result in the calibration table.
 *
 * Coverage: ~5 entries near 0 V and ~5 entries near 5 V (total ~10 entries).
 * This corrects DAC zero offset and full-scale gain without any external source.
 * After both sweeps, marks the table valid and saves to flash.
 */
void cmdCalBuildDac() {
  ScopedInstrumentState guard;

  Serial.println("=== DAC Calibration Table Build ===");

  // --- Anchor 1: GND (0 V) ---
  Serial.println("Anchor 1: GND (0 V)...");
  inputMux.select(InputChannel::GND);
  dac.setCode(0);
  if (!binarySearchDAC()) {
    Serial.println("ERROR: Binary search failed on GND. Aborting.");
    return;
  }
  int16_t binSrchCode = dac.currentCode();
  Serial.print("  Converged at code "); Serial.println(binSrchCode);

  int entriesGnd = 0;
  for (int delta = -CAL_BUILD_WINDOW; delta <= CAL_BUILD_WINDOW; delta++) {
    int32_t code32 = (int32_t)binSrchCode + (int32_t)(delta * 4);
    if (code32 < -32768) code32 = -32768;
    if (code32 >  32764) code32 =  32764;
    int16_t code = (int16_t)code32;

    dac.setCode(code);

    LowerMoments stats;
	int32_t buffA[MAX_SAMPLES];
	int32_t buffB[MAX_SAMPLES];
    bool ok = true;
    for (int i = 0; i < CAL_BUILD_CYCLES; i++) {
      if (!runOneChopCycle(stats, buffA, buffB, false)) { ok = false; break; }
    }
    if (!ok) {
      Serial.print("  WARNING: Overflow at code "); Serial.println(code);
      continue;
    }

    // Vx = GND = 0 V, so: V_dac = -(demod * ADC_LSB_V) / PREAMP_GAIN
    double vDac = -(stats.mean() * ADC_LSB_V) / PREAMP_GAIN;
    dacCalTable.setPoint(code, vDac);
    entriesGnd++;

    Serial.print("  code="); Serial.print(code);
    Serial.print("  V_dac="); Serial.print(vDac * 1e6, 3); Serial.println(" uV");
  }
  Serial.print("  GND anchor: "); Serial.print(entriesGnd); Serial.println(" entries stored.");

  // --- Anchor 2: VrefRaw (~5 V) ---
  Serial.println("Anchor 2: VrefRaw (~5 V)...");
  inputMux.select(InputChannel::VrefRaw);
  dac.setCode(REF_MEASURE_DAC_CODE);
  if (!binarySearchDAC()) {
    Serial.println("ERROR: Binary search failed on VrefRaw. Aborting.");
    return;
  }
  binSrchCode = dac.currentCode();
  Serial.print("  Converged at code "); Serial.println(binSrchCode);

  int entriesVref = 0;
  for (int delta = -CAL_BUILD_WINDOW; delta <= CAL_BUILD_WINDOW; delta++) {
    int32_t code32 = (int32_t)binSrchCode + (int32_t)(delta * 4);
    if (code32 < -32768) code32 = -32768;
    if (code32 >  32764) code32 =  32764;
    int16_t code = (int16_t)code32;

    dac.setCode(code);

    LowerMoments stats;
	int32_t buffA[MAX_SAMPLES];
	int32_t buffB[MAX_SAMPLES];
    bool ok = true;
    for (int i = 0; i < CAL_BUILD_CYCLES; i++) {
      if (!runOneChopCycle(stats, buffA, buffB, false)) { 
	  ok = false; break; }
    }
    if (!ok) {
      Serial.print("  WARNING: Overflow at code "); Serial.println(code);
      continue;
    }

    // Vx = VrefRaw ~= NOMINAL_REF_V, so: V_dac = NOMINAL_REF_V - (demod * ADC_LSB_V) / PREAMP_GAIN
    double vDac = NOMINAL_REF_V - (stats.mean() * ADC_LSB_V) / PREAMP_GAIN;
    dacCalTable.setPoint(code, vDac);
    entriesVref++;

    Serial.print("  code="); Serial.print(code);
    Serial.print("  V_dac="); Serial.print(vDac, 6); Serial.println(" V");
  }
  Serial.print("  VrefRaw anchor: "); Serial.print(entriesVref); Serial.println(" entries stored.");

  // Mark valid and save to flash
  dacCalTable.markValid();
  Serial.println("DAC calibration table marked valid.");
  Serial.println("Saving to flash...");
  if (calStore.saveTable()) {
    Serial.println("Saved successfully.");
  }
  // guard restores channel + DAC on scope exit
}

/**
 * Capture a single DAC calibration table entry at a user-supplied known voltage.
 *
 * The user applies a known accurate voltage V_known to the Vx input, then
 * calls "cal point <voltage>". The firmware binary-searches the DAC to lock
 * onto V_known, accumulates CAL_POINT_CYCLES chopped measurements, and stores
 * the actual DAC output voltage at the current DAC code.
 *
 * Repeat at different applied voltages to build up the full table.
 * Call "cal save" when done to persist the table to flash.
 */
void cmdCalPoint(const char* voltageStr) {
  double vKnown = atof(voltageStr);

  if (vKnown < -5.1 || vKnown > 5.1) {
    Serial.println("Error: voltage must be within ±5.1 V.");
    return;
  }

  ScopedInstrumentState guard;

  // Select Vx1 channel (user applies the known voltage here)
  inputMux.select(InputChannel::Vx1);

  Serial.print("cal point: V_known="); Serial.print(vKnown, 8); Serial.println(" V");
  Serial.println("Binary-searching DAC to lock onto Vx...");

  if (!binarySearchDAC()) {
    Serial.println("ERROR: Binary search failed. Is the known voltage applied to Vx?");
    return;
  }

  int16_t code = dac.currentCode();
  Serial.print("  DAC code="); Serial.println(code);

  // Accumulate chopped measurements
  LowerMoments stats;
  int32_t buffA[MAX_SAMPLES];
  int32_t buffB[MAX_SAMPLES];
  for (int i = 0; i < CAL_POINT_CYCLES; i++) {
    if (!runOneChopCycle(stats, buffA, buffB, false)) {
      Serial.println("WARNING: Overflow during cal point measurement. Aborting.");
      return;
    }
  }

  // V_dac_actual = V_known - ADC_residual/gain
  double vDac = vKnown - (stats.mean() * ADC_LSB_V) / PREAMP_GAIN;
  dacCalTable.setPoint(code, vDac);

  Serial.print("  Stored: code="); Serial.print(code);
  Serial.print(" -> V_dac="); Serial.print(vDac, 8); Serial.println(" V");

  if (!dacCalTable.isValid()) {
    Serial.println("  (Table not yet marked valid. Run 'cal build dac' first,");
    Serial.println("   or mark manually; then 'cal save' to persist.)");
  } else {
    Serial.println("  (Use 'cal save' to persist updated table.)");
  }
  // guard restores channel + DAC on scope exit
}

// ---- CalVerify adapters --------------------------------------------------
// Thin shims around .ino-internal facilities (DAC driver, chop loop, cal
// table, mux) so CalVerify.cpp can interact with the firmware through only
// function pointers — see CalVerify.h.

static void calVerifyAdapter_setDac(int16_t code) {
  dac.setCode(code);
}

static double calVerifyAdapter_runChop(int nCycles, bool* overflow) {
  // measureDemodMean() returns NaN on overflow; convert to the bool* contract.
  double mean = measureDemodMean(nCycles);
  if (isnan(mean)) { if (overflow) *overflow = true; return 0.0; }
  if (overflow) *overflow = false;
  return mean;
}

static double calVerifyAdapter_codeToV(int16_t code) {
  return dacCalTable.codeToVoltage(code);
}

static void calVerifyAdapter_setPoint(int16_t code, double vDac) {
  dacCalTable.setPoint(code, vDac);
  if (!dacCalTable.isValid()) dacCalTable.markValid();
}

static int calVerifyAdapter_interp() {
  return (int)dacCalTable.interpolateGaps();
}

static void calVerifyAdapter_selectVx2() {
  inputMux.select(InputChannel::Vx2);
}

// Adapter used by both the verify workflow (mid-session smoothing) and the
// `cal smooth` command (post-hoc smoothing of a saved table).
static void calVerifyAdapter_interpBetween(int16_t lo, int16_t hi) {
  dacCalTable.interpolateBetweenCodes(lo, hi);
}

// Same snap-to-14-bit-aligned helper as in CalVerify.cpp. Duplicated here
// rather than exported because it's a one-liner and re-exposing snap math
// across the .ino/.cpp boundary isn't worth a new function pointer.
static int16_t calSnapToCardinalCode(double v_target) {
  double v_max = 32767.0 * DAC_LSB_V;
  if (v_target >= v_max - 0.5 * DAC_LSB_V) return 32767;
  long snapped = (long)lround(v_target / (4.0 * DAC_LSB_V)) * 4;
  if (snapped < -32768) snapped = -32768;
  if (snapped >  32764) snapped =  32764;
  return (int16_t)snapped;
}

/**
 * Handle 'cal' command and subcommands.
 * Defined here (after CalibrationStore) so calStore is fully typed.
 */
void cmdCal(const char* arg1, const char* arg2, const char* arg3) {
  if (!arg1) {
    Serial.println("Usage: cal status|save|load|erase|set|factory|build|point|verify|dac|cardinals|clear|interpolate|gain");
    return;
  }

  if (strcasecmp(arg1, "status") == 0) {
    calStore.printStatus();

  } else if (strcasecmp(arg1, "save") == 0) {
    Serial.println("Saving cal constants to flash...");
    if (calStore.saveConstants()) Serial.println("  Constants: saved.");
    else Serial.println("  Constants: FAILED.");
    Serial.println("Saving DAC table to flash...");
    if (calStore.saveTable()) Serial.println("  DAC table: saved.");
    // saveTable() prints reason if not valid

  } else if (strcasecmp(arg1, "load") == 0) {
    Serial.println("Loading cal from flash...");
    calStore.autoLoad();

  } else if (strcasecmp(arg1, "erase") == 0) {
    if (calStore.eraseAll()) Serial.println("Calibration files erased.");
    else Serial.println("Erase failed (flash not mounted?).");

  } else if (strcasecmp(arg1, "factory") == 0) {
    PREAMP_GAIN        = PREAMP_GAIN_DEFAULT;
    DIVIDER_RATIO_10   = DIVIDER_RATIO_10_DEFAULT;
    DIVIDER_RATIO_100  = DIVIDER_RATIO_100_DEFAULT;
    DIVIDER_RATIO_1000 = DIVIDER_RATIO_1000_DEFAULT;
    OPA828_OFFSET      = OPA828_OFFSET_DEFAULT;
    Serial.println("Cal constants reset to factory defaults (RAM only).");
    Serial.println("Use 'cal save' to persist, or reboot to revert if saved.");

  } else if (strcasecmp(arg1, "set") == 0) {
    if (!arg2 || !arg3) {
      Serial.println("Usage: cal set gain|div10|div100|div1000 <value>");
      return;
    }
    double val = atof(arg3);
    if (val == 0.0) {
      Serial.println("Error: value must be non-zero.");
      return;
    }
    if (strcasecmp(arg2, "gain") == 0) {
      if (val < 100.0 || val > 100000.0) { Serial.println("Error: gain out of range [100, 100000]."); return; }
      PREAMP_GAIN = val;
      Serial.print("PREAMP_GAIN set to "); Serial.println(PREAMP_GAIN, 6);
    } else if (strcasecmp(arg2, "div10") == 0) {
      if (val < 1.0 || val > 100.0) { Serial.println("Error: div10 out of range [1, 100]."); return; }
      DIVIDER_RATIO_10 = val;
      Serial.print("DIVIDER_RATIO_10 set to "); Serial.println(DIVIDER_RATIO_10, 6);
    } else if (strcasecmp(arg2, "div100") == 0) {
      if (val < 10.0 || val > 10000.0) { Serial.println("Error: div100 out of range [10, 10000]."); return; }
      DIVIDER_RATIO_100 = val;
      Serial.print("DIVIDER_RATIO_100 set to "); Serial.println(DIVIDER_RATIO_100, 6);
    } else if (strcasecmp(arg2, "div1000") == 0) {
      if (val < 100.0 || val > 100000.0) { Serial.println("Error: div1000 out of range [100, 100000]."); return; }
      DIVIDER_RATIO_1000 = val;
      Serial.print("DIVIDER_RATIO_1000 set to "); Serial.println(DIVIDER_RATIO_1000, 6);
    } else if (strcasecmp(arg2, "opa828") == 0) {
      if (val < -0.001 || val > 0.001) { Serial.println("Error: opa828 offset out of range [-1000, +1000] uV."); return; }
      OPA828_OFFSET = val;
      Serial.print("OPA828_OFFSET set to "); Serial.print(OPA828_OFFSET * 1e6, 3); Serial.println(" uV");
    } else if (strcasecmp(arg2, "adcvref") == 0) {
      if (val < 2.0 || val > 3.0) { Serial.println("Error: adcvref out of range [2.0, 3.0] V."); return; }
      ADC_VREF = val;
      ADC_LSB_V = (2.0 * ADC_VREF) / ADC_FSR_CODES;
      Serial.print("ADC_VREF set to "); Serial.print(ADC_VREF, 9); Serial.println(" V");
      Serial.print("ADC_LSB_V = "); Serial.print(ADC_LSB_V * 1e9, 6); Serial.println(" nV/count");
    } else {
      Serial.println("Unknown cal set parameter. Use: gain, div10, div100, div1000, opa828, adcvref");
    }
    Serial.println("(Use 'cal save' to persist to flash.)");

  } else if (strcasecmp(arg1, "build") == 0) {
    if (!arg2 || strcasecmp(arg2, "dac") != 0) {
      Serial.println("Usage: cal build dac");
      return;
    }
    cmdCalBuildDac();

  } else if (strcasecmp(arg1, "point") == 0) {
    if (!arg2) { Serial.println("Usage: cal point <voltage>"); return; }
    cmdCalPoint(arg2);

  } else if (strcasecmp(arg1, "verify") == 0) {
    // Interactive Keithley-validated calibration walkthrough. Modal — chop
    // loop pauses for the duration. See CalVerify.h/cpp for the workflow.
    ScopedInstrumentState guard;     // restores channel + DAC on scope exit
    CalVerifyApi api = {
      calVerifyAdapter_setDac,
      calVerifyAdapter_runChop,
      calVerifyAdapter_codeToV,
      calVerifyAdapter_setPoint,
      calVerifyAdapter_interp,
      calVerifyAdapter_interpBetween,
      calVerifyAdapter_selectVx2,
      ADC_LSB_V, PREAMP_GAIN, DAC_LSB_V,
      &Serial,
    };
    const char* argv[2] = { arg2, arg3 };
    int argc = (arg2 ? (arg3 ? 2 : 1) : 0);
    cmdCalVerify(api, argc, argv);

  } else if (strcasecmp(arg1, "dac") == 0) {
    // cal dac <code> <voltage>
    // Directly stores a measured DAC output voltage for a 14-bit-aligned code,
    // or for the absolute-max code 32767 (separate cal slot).
    // Example: cal dac 0 -0.0003515
    if (!arg2 || !arg3) {
      Serial.println("Usage: cal dac <code> <measured_voltage>");
      Serial.println("  code must be a multiple of 4 in [-32768, 32764], or 32767");
      return;
    }
    int32_t code = atol(arg2);
    if ((code != 32767) && (code < -32768 || code > 32764 || (code & 3) != 0)) {
      Serial.println("Error: code must be a multiple of 4 in [-32768, 32764], or 32767.");
      return;
    }
    double voltage = atof(arg3);
    if (voltage < -5.5 || voltage > 5.5) {
      Serial.println("Error: voltage out of plausible DAC range (±5.5 V).");
      return;
    }
    dacCalTable.setPoint((int16_t)code, voltage);
    if (!dacCalTable.isValid()) dacCalTable.markValid();
    Serial.print("DAC table: code="); Serial.print(code);
    Serial.print(" -> "); Serial.print(voltage, 8); Serial.println(" V  (use 'cal save' to persist)");

  } else if (strcasecmp(arg1, "clear") == 0) {
    // cal clear <code>  — restore a slot to nominal, removing it from cardinals
    if (!arg2) {
      Serial.println("Usage: cal clear <code>   (code = 14-bit-aligned, range -32768..32764, or 32767)");
      return;
    }
    int32_t code = atol(arg2);
    if ((code != 32767) && (code < -32768 || code > 32764 || (code & 3) != 0)) {
      Serial.println("Error: code must be a multiple of 4 in [-32768, 32764], or 32767.");
      return;
    }
    bool was = dacCalTable.clearCardinalPoint((int16_t)code);
    if (was) {
      Serial.print("Cleared cardinal at code="); Serial.print(code);
      Serial.println("  (use 'cal save' to persist)");
    } else {
      Serial.print("Code="); Serial.print(code);
      Serial.println(" was already at nominal — no change.");
    }

  } else if (strcasecmp(arg1, "cardinals") == 0) {
    if (!dacCalTable.isValid()) {
      Serial.println("DAC table not yet calibrated (no points set).");
      return;
    }
    dacCalTable.printCardinals();

  } else if (strcasecmp(arg1, "interpolate") == 0) {
    if (!dacCalTable.isValid()) {
      Serial.println("DAC table not yet calibrated (no points set).");
      return;
    }
    size_t n = dacCalTable.interpolateGaps();
    if (n < 2) {
      Serial.print("Not enough cardinal points to interpolate (found ");
      Serial.print((unsigned long)n);
      Serial.println("). Need at least 2.");
      return;
    }
    if (n > 256) {
      Serial.print("Found "); Serial.print((unsigned long)n);
      Serial.println(" cardinals — exceeds internal cap (256). Aborted.");
      return;
    }
    Serial.print("Interpolated using "); Serial.print((unsigned long)n);
    Serial.println(" cardinal points; all non-cardinal slots updated.");
    Serial.println("Use 'cal save' to persist, or 'cal load' to revert from flash.");

  } else if (strcasecmp(arg1, "smooth") == 0) {
    // Post-hoc fix for a verify session whose cardinals are stranded between
    // stale cal-build-dac values (interpolateGaps() was a no-op because
    // n > MAX_CARDINALS=256). Reads the existing cal-table values at each
    // verify target code and linearly blends the slots between consecutive
    // pairs. No measurement; safe to run with the Keithley turned off.
    // Targets default to the same 25-point list cal verify uses; override
    // with `cal smooth <V1> <V2> [...]` if you ran verify with a custom list.
    if (!dacCalTable.isValid()) {
      Serial.println("DAC table not yet calibrated (no points set).");
      return;
    }
    // Parse optional override targets out of arg2/arg3 (cmdCal only passes
    // three args; for a longer list, the user should re-add support — but for
    // the default 25-point case no args are needed).
    double overrideBuf[2];
    const double* targets = kCalVerifyDefaultTargets;
    int targetCount = kCalVerifyDefaultTargetCount;
    int overrideN = 0;
    if (arg2) {
      char* endp = nullptr;
      overrideBuf[overrideN] = strtod(arg2, &endp);
      if (endp == arg2) {
        Serial.println("Error: arg2 not a number."); return;
      }
      overrideN++;
    }
    if (arg3) {
      char* endp = nullptr;
      overrideBuf[overrideN] = strtod(arg3, &endp);
      if (endp == arg3) {
        Serial.println("Error: arg3 not a number."); return;
      }
      overrideN++;
    }
    if (overrideN == 1) {
      Serial.println("Error: cal smooth needs at least 2 targets to interpolate between.");
      return;
    }
    if (overrideN == 2) {
      targets = overrideBuf;
      targetCount = 2;
    }
    // Walk consecutive pairs and interpolate between them.
    Serial.print("cal smooth: blending between "); Serial.print(targetCount);
    Serial.println(" target codes (existing cardinal values preserved at endpoints)...");
    size_t totalBlended = 0;
    for (int i = 0; i + 1 < targetCount; i++) {
      int16_t codeA = calSnapToCardinalCode(targets[i]);
      int16_t codeB = calSnapToCardinalCode(targets[i + 1]);
      double  vA    = dacCalTable.codeToVoltage(codeA);
      double  vB    = dacCalTable.codeToVoltage(codeB);
      size_t  n     = dacCalTable.interpolateBetweenCodes(codeA, codeB);
      totalBlended += n;
      Serial.print("  [");  Serial.print(targets[i], 6);
      Serial.print(" V, "); Serial.print(targets[i + 1], 6);
      Serial.print(" V]  codes "); Serial.print(codeA);
      Serial.print(".."); Serial.print(codeB);
      Serial.print("  vA="); Serial.print(vA, 7);
      Serial.print(" vB="); Serial.print(vB, 7);
      Serial.print("  blended "); Serial.print((unsigned long)n);
      Serial.println(" slots");
    }
    Serial.print("cal smooth: total "); Serial.print((unsigned long)totalBlended);
    Serial.println(" slots blended. Use 'cal save' to persist.");

  } else if (strcasecmp(arg1, "gain") == 0) {
    if (!arg2 || arg2[0] == '\0') {
      Serial.println("Usage: cal gain <V_actual_V>");
      Serial.println("  Apply a known precise voltage (e.g. 2.5 V reference), let binary search settle,");
      Serial.println("  then run: cal gain 2.5");
      return;
    }
    if (isnan(g_calLastVoltage)) {
      Serial.println("No measurement cached yet -- start a scan first.");
      return;
    }
    double vActual = atof(arg2);
    if (vActual == 0.0) {
      Serial.println("V_actual cannot be zero.");
      return;
    }

    double dacV     = dacCalTable.codeToVoltage(g_calLastDacCode);
    double divRatio = getInputDividerRatio(g_calLastChannel);
    double refCorr  = getRefCorrectionFactor();

    // Invert computeInputVoltage+refCorrection to get the preamp-space delta
    // that corresponds to the displayed voltage (ewma if active, else vxCorrected).
    // vxCorrected = (dacV + preampDelta) * divRatio * refCorr  [non-HVDiv]
    // => preampDelta = vxCorrected / divRatio / refCorr - dacV
    double vxAtPreampDisplayed = g_calLastVoltage / divRatio / refCorr;
    if (g_calLastChannel == InputChannel::HVDiv) vxAtPreampDisplayed += OPA828_OFFSET;
    double deltaDisplayed = vxAtPreampDisplayed - dacV;

    // deltaActual: true preamp input delta from the reference meter reading.
    double vxAtPreampActual = vActual / divRatio;
    if (g_calLastChannel == InputChannel::HVDiv) vxAtPreampActual += OPA828_OFFSET;
    double deltaActual = vxAtPreampActual / refCorr - dacV;

    Serial.print("  Channel                  : "); Serial.println(getInputChannelName(g_calLastChannel));
    Serial.print("  DAC code / voltage       : "); Serial.print(g_calLastDacCode);
      Serial.print(" / "); Serial.print(dacV, 7); Serial.println(" V");
    Serial.print("  refCorrection            : "); Serial.println(refCorr, 9);
    Serial.print("  Displayed voltage        : "); Serial.print(g_calLastVoltage, 9); Serial.println(" V");
    Serial.print("  Preamp delta (displayed) : "); Serial.print(deltaDisplayed * 1e6, 2); Serial.println(" uV");
    Serial.print("  Preamp delta (actual)    : "); Serial.print(deltaActual * 1e6, 2); Serial.println(" uV");

    if (fabs(deltaActual) < 0.010) {
      Serial.println("WARNING: preamp delta < 10 mV -- accuracy will be poor.");
    }

    double newGain = PREAMP_GAIN * (deltaDisplayed / deltaActual);
    Serial.print("  Old PREAMP_GAIN          : "); Serial.println(PREAMP_GAIN, 4);
    Serial.print("  New PREAMP_GAIN          : "); Serial.println(newGain, 4);

    // Hard reject: catches wrong channel, wildly wrong voltage, or measurement failure.
    // AD8428 absolute gain tolerance is ±10% per stage; ±20% covers the two-stage ensemble.
    constexpr double GAIN_SANITY_TOL = 0.20;
    constexpr double GAIN_SANITY_MIN = PREAMP_GAIN_DEFAULT * (1.0 - GAIN_SANITY_TOL);  // 1600
    constexpr double GAIN_SANITY_MAX = PREAMP_GAIN_DEFAULT * (1.0 + GAIN_SANITY_TOL);  // 2400
    if (newGain < GAIN_SANITY_MIN || newGain > GAIN_SANITY_MAX) {
      Serial.print("  REJECTED: computed gain outside sanity range [");
      Serial.print(GAIN_SANITY_MIN, 0); Serial.print(", ");
      Serial.print(GAIN_SANITY_MAX, 0); Serial.println("]");
      Serial.println("  Check: correct channel selected, large preamp delta, ADC_VREF calibrated.");
      return;
    }

    // Warn if outside ±5% — likely a small-delta accuracy issue rather than a hardware problem.
    constexpr double GAIN_WARN_TOL = 0.05;
    if (fabs(newGain - PREAMP_GAIN_DEFAULT) > PREAMP_GAIN_DEFAULT * GAIN_WARN_TOL) {
      Serial.print("  WARNING: gain deviates >5% from nominal -- verify large preamp delta and correct ADC_VREF.");
      Serial.println(" Applying anyway.");
    }

    PREAMP_GAIN = newGain;
    Serial.println("  Applied. Run 'cal save' to persist.");

  } else {
    Serial.println("Unknown cal subcommand. Use: status, save, load, erase, set, factory, build, point, dac, gain");
  }
}

/**
 * Save current runtime configuration to EEPROM.
 * Called by processCommand() on "config save". Serializes scanConfig,
 * configAutoStart, and divider ratio into a SavedConfig blob
 * with magic number and CRC16, then writes byte-by-byte to EEPROM.
 */
void cmdConfigSave() {
  SavedConfig saved;
  saved.magic = EEPROM_MAGIC;
  saved.scanConfig = scanner.config;
  saved.autoStart = configAutoStart;
  saved.dividerRatio = divMux.current();

  // Compute checksum over everything except the checksum field itself
  saved.checksum = crc16((const uint8_t*)&saved, offsetof(SavedConfig, checksum));

  // Write to EEPROM
  const uint8_t* p = (const uint8_t*)&saved;
  for (size_t i = 0; i < sizeof(SavedConfig); i++) {
    EEPROM.write(EEPROM_BASE_ADDR + i, p[i]);
  }

  Serial.println("Configuration saved to EEPROM.");
}

/**
 * Read and validate configuration from EEPROM.
 * Called by setup() at boot, cmdConfigLoad(), and cmdConfigShow().
 *
 * @param saved  Output: populated with EEPROM contents if valid
 * @return       true if magic and CRC match (config is valid), false otherwise
 */
bool loadConfigFromEEPROM(SavedConfig &saved) {
  uint8_t* p = (uint8_t*)&saved;
  for (size_t i = 0; i < sizeof(SavedConfig); i++) {
    p[i] = EEPROM.read(EEPROM_BASE_ADDR + i);
  }

  // Verify magic
  if (saved.magic != EEPROM_MAGIC) {
    return false;
  }

  // Verify checksum
  uint16_t expected = crc16((const uint8_t*)&saved, offsetof(SavedConfig, checksum));
  if (saved.checksum != expected) {
    return false;
  }

  return true;
}

/**
 * Apply a loaded SavedConfig to the runtime state.
 * Called by cmdConfigLoad() and setup() after loadConfigFromEEPROM() succeeds.
 * Copies scanConfig, sets configAutoStart, and adjusts divider ratio if changed.
 */
void applyConfig(const SavedConfig &saved) {
  scanner.config = saved.scanConfig;
  configAutoStart = saved.autoStart;
  if (saved.dividerRatio != divMux.current()) {
    divMux.select(saved.dividerRatio);
  }
  scanner.setEwmaWindow(scanner.config.ewmaWindow);
}

void cmdConfigLoad() {
  SavedConfig saved;
  if (loadConfigFromEEPROM(saved)) {
    applyConfig(saved);
    Serial.println("Configuration loaded from EEPROM.");
  } else {
    Serial.println("ERROR: No valid config in EEPROM (corrupt or never saved).");
  }
}

/**
 * Reset runtime configuration to factory defaults.
 * Called by processCommand() on "config factory". Does NOT write to EEPROM;
 * user must "config save" separately to persist the reset.
 * Stops any active scan and invalidates the auto-zero offset.
 */
void cmdConfigFactory() {
  scanner.config.channels[0] = InputChannel::Vx1;
  scanner.config.count = 1;
  scanner.config.integrationCycles = 390;
  scanner.config.autoZeroEnabled = true;
  scanner.config.autoZeroInterval = 10;
  scanner.config.outputMode = OutputMode::Human;
  scanner.config.ewmaWindow = 10.0f;
  scanner.setEwmaWindow(10.0f);
  configAutoStart = false;
  scanner.autoZeroOffset = 0.0;
  scanner.autoZeroValid = false;
  scanner.clearDacCache();
  allanDev.clear();
  scanner.stop();
  Serial.println("Factory defaults restored (not saved to EEPROM).");
}

void cmdConfigAutostart(const char* arg) {
  if (strcasecmp(arg, "on") == 0) {
    configAutoStart = true;
    Serial.println("Auto-start enabled (save config to persist).");
  } else if (strcasecmp(arg, "off") == 0) {
    configAutoStart = false;
    Serial.println("Auto-start disabled (save config to persist).");
  } else {
    Serial.println("Usage: config autostart on|off");
  }
}

void cmdConfigShow() {
  Serial.println("\n=== Current Configuration ===");
  printStatus();

  Serial.println("--- Saved Configuration ---");
  SavedConfig saved;
  if (loadConfigFromEEPROM(saved)) {
    Serial.print("Auto-start: ");
    Serial.println(saved.autoStart ? "ON" : "OFF");
    Serial.print("Integration: ");
    Serial.println(saved.scanConfig.integrationCycles);
    Serial.print("Auto-zero: ");
    Serial.print(saved.scanConfig.autoZeroEnabled ? "ON" : "OFF");
    Serial.print(", interval=");
    Serial.println(saved.scanConfig.autoZeroInterval);
    Serial.print("EWMA window: ");
    Serial.println(saved.scanConfig.ewmaWindow, 1);
    Serial.print("Scan channels: ");
    Serial.println(saved.scanConfig.count);
    for (int i = 0; i < saved.scanConfig.count; i++) {
      Serial.print("  ");
      Serial.println(getInputChannelName(saved.scanConfig.channels[i]));
    }
    Serial.print("Divider: ");
    Serial.println(getDividerRatioName(saved.dividerRatio));
  } else {
    Serial.println("(No valid saved config)");
  }
  Serial.println("=============================\n");
}

// ================== Output a Measurement Result ==================

/**
 * Compute corrected voltage and output a single channel measurement result.
 * Delegates formatting to the active IOutputFormatter selected by outputMode.
 *
 * Multi-channel Plotter mode is assembled directly in loop()'s SCANNING
 * branch using beginSweep()/formatMeasurement()/endSweep().
 *
 * @param channel  Which input channel was measured
 * @param stats    LowerMoments accumulator with the measurement data
 * @param dacCode  DAC code that was active during this measurement
 */
void outputMeasurement(InputChannel channel, LowerMoments &stats, int16_t dacCode) {
  double adcMean = stats.mean();
  double adcStdDev = stats.standardDeviation();

  g_calLastDacCode = dacCode;
  g_calLastChannel = channel;

  double vxRaw = computeInputVoltage(adcMean, dacCode, channel);
  double vxUncertainty = computeInputUncertainty(adcStdDev, channel);

  double refCorrection = getRefCorrectionFactor();
  double vxCorrected = vxRaw * refCorrection;
  if (scanner.autoZeroValid) {
    vxCorrected -= scanner.autoZeroOffset;
  }

  double driftPpb = (refCorrection - 1.0) * 1e9;

  allanDev.addReading(vxCorrected, tempmonGetTemp());

  AdjustedEWMA& ewma = scanner.channelEwma((uint8_t)channel);
  double ewmaV = ewma.update(vxCorrected);
  g_calLastVoltage = ewma.isInitialized() ? ewmaV : vxCorrected;

  MeasurementData data;
  data.channel     = channel;
  data.voltage     = vxCorrected;
  data.uncertainty = vxUncertainty;
  data.adcMean     = adcMean;
  data.adcStdDev   = adcStdDev;
  data.count       = stats.count();
  data.dacCode     = dacCode;
  data.driftPpb    = driftPpb;
  data.adcMin      = stats.getMin();
  data.adcMax      = stats.getMax();
  data.ewmaVoltage = ewmaV;
  data.ewmaValid   = ewma.isInitialized();

  IOutputFormatter& fmt = getFormatter();
  fmt.beginSweep();
  fmt.formatMeasurement(data);
  fmt.endSweep();

  // Demodulation outlier-rejection report. Comment-prefixed so plotters skip it.
  // Only emits a line when a rejection actually happened in this output cycle;
  // the cumulative session totals come along when it does.
  if (g_demodRejSinceOutput > 0) {
    Serial.print("# demod-clip: ");
    Serial.print(g_demodRejSinceOutput); Serial.print('/');
    Serial.print(g_demodPairsSinceOutput);
    Serial.print(" since last output (cum ");
    Serial.print((unsigned long)g_demodRejTotal); Serial.print('/');
    Serial.print((unsigned long)g_demodPairsTotal);
    Serial.println(")");
  }
  g_demodRejSinceOutput   = 0;
  g_demodPairsSinceOutput = 0;
}

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 2000) {}

  // Configure ADC pins
  pinMode(PIN_CS_ADC,     OUTPUT);
  csHigh(PIN_CS_ADC);
  pinMode(PIN_ADC_DRDY,   INPUT_PULLUP);  // pull-up so line reads HIGH when ADC not driving

  // Configure DAC pins
  pinMode(PIN_CS_DAC,     OUTPUT);
  csHigh(PIN_CS_DAC);
  pinMode(PIN_DAC_RESET,  OUTPUT);
  pinMode(PIN_DAC_CLEAR,  OUTPUT);
  digitalWrite(PIN_DAC_RESET, HIGH);
  digitalWrite(PIN_DAC_CLEAR, HIGH);

  // Initialize hardware drivers
  chopper.begin();
  inputMux.begin();
  divMux.begin();

  // Initialize DAC calibration table with nominal values
  dacCalTable.initNominal();

  // Mount calibration flash and restore persisted values
  Serial.println("Mounting calibration flash...");
  if (calStore.begin()) {
    Serial.println("Calibration flash: mounted.");
    calStore.autoLoad();
  } else {
    Serial.println("WARNING: Calibration flash unavailable; using defaults.");
  }

  // Initialize per-channel DAC code cache
  scanner.clearDacCache();

  // Initialize reference drift tracking
  initRefTracking();

  // Set external clock for ADS127L11
  adc.begin();

  // Initialize SPI.
  // ADC uses lpspi4_xfer() (TDR-only) relying on shadow TCR set here.
  // DAC (AD5760 class) calls beginTransaction(5 MHz MODE1) per transfer — same
  // settings as ADC — so no clock conflict when they share LPSPI4.
  // Flush the command entry that beginTransaction() pushes on MEN=1.
  SPI.begin();
  SPI.beginTransaction(adcSpiSettings);  // configure clock rate and pin mux
  IMXRT_LPSPI4_S.CR |= LPSPI_CR_RTF | LPSPI_CR_RRF;  // flush any FIFO entries pushed by beginTransaction

  // Initialize DAC (calls SPI.begin() internally, which resets LPSPI4 TCR to default CPHA=0)
  dac.begin();

  // Set TCR AFTER dac.begin(): dac.begin() calls SPI.begin() which resets TCR to CPHA=0.
  // Writing TCR directly sets the shadow used when the TX FIFO command queue is empty,
  // so TDR-only transfers (lpspi4_xfer) see CPHA=1 (Mode 1) as required by ADS127L11.
  IMXRT_LPSPI4_S.CR |= LPSPI_CR_RTF | LPSPI_CR_RRF;  // flush again after dac.begin()
  IMXRT_LPSPI4_S.TCR = LPSPI_TCR_FRAMESZ(7) | LPSPI_TCR_CPHA;  // 8-bit, CPHA=1 (Mode 1)

  Serial.println("Starting ADC init...");
  adc.initAndConfigure();

  // Enable the mux
  digitalWriteFast(PIN_TMUX_EN, HIGH);

  // Wait for analog frontend to settle: DAC output op-amp and preamp supply rails need
  // ~500ms from power-on before Signal Chain and zero-cal tests give valid readings.
  Serial.println("Waiting 1s for analog frontend to settle...");
  delay(1000);

  // Note: EST_FSPS / EST_TS_US still hold their nominal values here; they'll
  // be updated to measured-from-DRDY values by postTestDrdyTiming() during POST.
  Serial.print("Nominal fSPS = ");
  Serial.print(EST_FSPS, 2);
  Serial.print(" SPS, Ts = ");
  Serial.print(EST_TS_US, 1);
  Serial.println(" us  (will be measured by POST DRDY)");

  // Run Power-On Self-Test
  PostResult postResult = runPOST();
  
  if (!postResult.allPassed()) {
    pinMode(LED_BUILTIN, OUTPUT);
    if (POST_HALT_ON_FAIL) {
      Serial.println("POST failed - system halted.");
      Serial.println("Set POST_HALT_ON_FAIL=false to continue in spite of failures.");
      // Give config information
      int i = 0;
      int code = -500;
      while (true) {
        i++;
        dac.setCode(code); code += 5;
        if (code > 100) {code = -code;}
        if (i % 2 == 0) {
          chopper.toggle();
        }
        // Blink LED to indicate failure 
        digitalWrite(LED_BUILTIN, HIGH);
        delay(2000);
        digitalWrite(LED_BUILTIN, LOW);
        delay(1000);
      }
    } else {
      Serial.println("POST failed - continuing with warnings.");
    }
  }
  adc.stop();
  delay(1);  
  adc.start();

  inputMux.select(InputChannel::Vx1);
  delay(10);

  // Initial DAC adjustment to bring ADC into range
  Serial.println("Finding initial DAC setting...");
  if (!binarySearchDAC()) {
    Serial.println("WARNING: Could not find valid DAC setting!");
  }

  // Print precision constants for reference
  Serial.println("\n=== Measurement System Constants ===");
  Serial.print("ADC LSB: ");
  Serial.print(ADC_LSB_V * 1e9, 3);
  Serial.println(" nV");
  Serial.print("DAC LSB: ");
  Serial.print(DAC_LSB_V * 1e6, 3);
  Serial.println(" uV");
  Serial.print("Preamp gain: ");
  Serial.println(PREAMP_GAIN, 0);
  Serial.print("Effective resolution: ");
  Serial.print(ADC_LSB_V / PREAMP_GAIN * 1e9, 3);
  Serial.println(" nV/count");
  Serial.print("DAC cal table: ");
  Serial.println(dacCalTable.isValid() ? "CALIBRATED" : "nominal (uncalibrated)");
  Serial.println("====================================\n");
  calStore.printStatus();

  // Load saved configuration from EEPROM (if valid)
  SavedConfig savedCfg;
  if (loadConfigFromEEPROM(savedCfg)) {
    applyConfig(savedCfg);
    Serial.println("Loaded saved configuration from EEPROM.");
  } else {
    Serial.println("No saved config found, using factory defaults.");
  }

  // Print startup configuration summary
  printStatus();

  // Auto-start scanning if configured
  if (configAutoStart && scanner.config.count > 0) {
    Serial.println("Auto-starting scan...");
    cmdScanStart();
  }

  Serial.println("Type 'help' for available commands.");
}

// Insertion sort for small arrays (N ≤ MAX_SAMPLES = 20). Faster than std::sort
// for tiny N due to lower constant overhead and no recursion.
static inline void demodSortInt32(int32_t *a, int n) {
  for (int i = 1; i < n; i++) {
    int32_t key = a[i];
    int j = i - 1;
    while (j >= 0 && a[j] > key) { a[j + 1] = a[j]; j--; }
    a[j + 1] = key;
  }
}

// Scratch buffer for pooled-MAD computation. BSS, not stack.
static int32_t s_demodScratch[DEMOD_POOL_N];

void demodulate(LowerMoments& stats, int32_t *phaseA, int32_t *phaseB) {
  const int N = g_good_samples;
  if (N <= 0) return;

  int32_t diff[MAX_SAMPLES];
  for (int i = 0; i < N; i++) diff[i] = phaseA[i] - phaseB[i];

  // Push this cycle's diffs into the ring buffer.
  for (int i = 0; i < N; i++) {
    g_diffPool[g_diffPoolHead] = diff[i];
    g_diffPoolHead = (g_diffPoolHead + 1) % DEMOD_POOL_N;
    if (g_diffPoolFilled < DEMOD_POOL_N) g_diffPoolFilled++;
  }

  // Compute pooled median + MAD if we have enough history.
  bool poolReady = (g_diffPoolFilled >= DEMOD_POOL_MIN);
  int32_t med = 0;
  int32_t threshold = INT32_MAX;
  if (poolReady) {
    const int M = g_diffPoolFilled;
    for (int i = 0; i < M; i++) s_demodScratch[i] = g_diffPool[i];
    demodSortInt32(s_demodScratch, M);
    med = s_demodScratch[M / 2];

    for (int i = 0; i < M; i++) {
      int32_t d = g_diffPool[i] - med;
      s_demodScratch[i] = d < 0 ? -d : d;
    }
    demodSortInt32(s_demodScratch, M);
    int32_t mad = s_demodScratch[M / 2];

    // Threshold = k · 1.4826 · MAD. (k_x10 / 10) · 1.4826 → mad·k_x10·1483/10000.
    threshold = (int32_t)((int64_t)mad * g_demodRejKx10 * 1483 / 10000);
    if (threshold < DEMOD_REJ_FLOOR) threshold = DEMOD_REJ_FLOOR;
  }

  for (int i = 0; i < N; i++) {
    g_demodPairsTotal++;
    g_demodPairsSinceOutput++;
    int32_t signedDev = diff[i] - med;
    int32_t absDev = signedDev < 0 ? -signedDev : signedDev;
    if (poolReady && absDev > threshold) {
      // Huber clip: cap influence at med ± threshold (symmetric, unbiased).
      int32_t clipped = (signedDev > 0) ? (med + threshold) : (med - threshold);
      stats.accumulate(((double)clipped) / 2.0);
      g_demodRejTotal++;
      g_demodRejSinceOutput++;
    } else {
      stats.accumulate(((double)diff[i]) / 2.0);
    }
  }
}

/**
 * Execute one complete chop cycle and accumulate the demodulated result.
 *
 * Called by loop() on every iteration (both ONE_CHANNEL and SCANNING modes).
 * Encapsulates the core measurement sequence that was previously inline in loop():
 *   1. acquireHalfCycle() — phase A (current TMUX state)
 *   2. chopper.toggle() — toggle TMUX with charge-injection minimization
 *   3. acquireHalfCycle() — phase B (opposite TMUX state)
 *   4. chopper.toggle() — restore original state for next cycle
 *   5. Demodulate: (sumA - sumB) / (2 × GOOD_SAMPLES) → cancels offset/1/f
 *   6. Accumulate into stats
 *
 * On overflow (preamp saturated), triggers binarySearchDAC() to re-center
 * the DAC and clears stats since the measurement baseline changed.
 *
 * @param stats   LowerMoments accumulator to add the demodulated sample to.
 *                Both ONE_CHANNEL and SCANNING modes pass scanner.channelStats()
 *                for the active channel.
 *
 * @param phaseA  Buffers to accumulate good samples, used for demodulation 
 * @param phaseB  
 * 
 * @return        true if measurement succeeded, false if overflow occurred
 *                (caller should skip output and retry on next loop iteration)
 */
// @param searchOnOverflow  If true (default), triggers binarySearchDAC() + stats.clear() on
//                          overflow — correct for continuous measurement in loop().
//                          If false, returns false immediately without touching the DAC —
//                          correct for calibration routines that manage their own DAC state.
// Consecutive BinSrch failure tracking for chop-loop overflow recovery.
// Previously, when BinSrch failed repeatedly (e.g., Vx outside the DAC ±5 V
// range or a hardware fault keeping the preamp pegged), runOneChopCycle
// would re-invoke BinSrch on every chop cycle — flooding the log and
// wasting time. The wrapper below caps auto-recovery at
// BS_FAIL_STREAK_LIMIT consecutive failures and suppresses further BinSrch
// until a chop cycle completes without overflow, which auto-resets the
// streak. User-initiated BinSrch calls (POST tests, cal commands,
// channel-switch first-visit) bypass this and call binarySearchDAC()
// directly so they still get full retry behaviour.
static int s_bsFailStreak = 0;
static constexpr int BS_FAIL_STREAK_LIMIT = 3;

static bool tryBinarySearchWithBackoff() {
  if (s_bsFailStreak >= BS_FAIL_STREAK_LIMIT) {
    Serial.print("DAC: BinSrch suppressed (");
    Serial.print(s_bsFailStreak);
    Serial.println(" consecutive failures); waiting for an in-range chop cycle to reset");
    return false;
  }
  if (binarySearchDAC()) {
    s_bsFailStreak = 0;
    return true;
  }
  s_bsFailStreak++;
  if (s_bsFailStreak == BS_FAIL_STREAK_LIMIT) {
    Serial.print("DAC: BinSrch hit failure-streak limit (");
    Serial.print(BS_FAIL_STREAK_LIMIT);
    Serial.println("); further auto-retries suppressed until an in-range cycle");
  }
  return false;
}

bool runOneChopCycle(LowerMoments &stats, int32_t* phaseA, int32_t* phaseB, bool searchOnOverflow) {
  // ensure that we are in a known polarity: Vx - DAC into INAMP...
  if (chopper.state()) {
    chopper.toggle();
  }
  adc.stop();
  delayMicroseconds(CHOP_SETTLE_US);
  adc.start();

  HalfCycleResult resultA = acquireHalfCycle(phaseA);

  if (resultA.overflow) {
    Serial.print("DAC: Phase A overflow (DAC=");
    Serial.print(dac.currentCode());
    Serial.print(" chopState="); Serial.print(chopper.state() ? 'B' : 'A');
    Serial.print(" sample="); Serial.print(resultA.overflowSample);
    Serial.println(")");
    Serial.print("  post-overflow trace:");
    for (int i = 0; i < 6; ++i) {
      int32_t s;
      if (!adc.readSample24(s)) break;
      Serial.print(' '); Serial.print(s);
    }
    Serial.println();
    if (searchOnOverflow) { tryBinarySearchWithBackoff(); stats.clear(); }
    return false;
  }

  chopper.toggle();
  adc.stop();
  delayMicroseconds(CHOP_SETTLE_US);
  if (stats.count() > 0) {
	  // Demodulate A - prevB (which is in phaseB buffer...)
	  demodulate(stats, phaseA, phaseB);
  }
  adc.start();

  HalfCycleResult resultB = acquireHalfCycle(phaseB);

  if (resultB.overflow) {
    Serial.print("DAC: Phase B overflow (DAC=");
    Serial.print(dac.currentCode());
    Serial.print(" chopState="); Serial.print(chopper.state() ? 'B' : 'A');
    Serial.print(" sample="); Serial.print(resultB.overflowSample);
    Serial.println(")");
    Serial.print("  post-overflow trace:");
    for (int i = 0; i < 6; ++i) {
      int32_t s;
      if (!adc.readSample24(s)) break;
      Serial.print(' '); Serial.print(s);
    }
    Serial.println();
    chopper.toggle();  // Return to original state
    if (searchOnOverflow) { tryBinarySearchWithBackoff(); stats.clear(); }
    return false;
  }

  chopper.toggle();
  demodulate(stats, phaseA, phaseB); // every buffer is used twice
  // Successful chop cycle → DAC is centered on Vx, system is healthy.
  // Reset the BinSrch failure streak so future overflow can re-invoke
  // recovery if Vx drifts/changes again.
  s_bsFailStreak = 0;
  return true;
}

/**
 * Main measurement loop. Runs continuously after setup().
 *
 * Structure:
 *   1. processSerialCommands() — check for user input (non-blocking)
 *   2. Reference drift tracking — measureFilterError() every ~1 second
 *   3. State machine branch:
 *
 *   ONE_CHANNEL mode:
 *     - Measures inputMux.current() continuously using scanner.channelStats()
 *     - Clears stats + resets counter automatically if the channel changes
 *     - Outputs a reading every integrationCycles via outputMeasurement()
 *
 *   SCANNING mode:
 *     - Cycles through scanner.config.channels[0..count-1]
 *     - For each channel:
 *         a. Switch mux + binarySearchDAC() (on channel change)
 *         b. Discard SCAN_SETTLE_CYCLES chop cycles for settling
 *         c. Accumulate integrationCycles into scanner.channelStats[channel]
 *         d. Output result via outputMeasurement() (or plotter assembly)
 *         e. Advance to next channel
 *     - After a full sweep: optionally run performAutoZero()
 *     - Multi-channel Plotter output is assembled here (not in outputMeasurement)
 *       because all channels must appear on one tab-separated line
 */
void loop() {
  // Process serial commands
  processSerialCommands();

  // Disabled for debugging: measureFilterError() switches mux to VrefRaw + DAC to ~5V
  // if (++refSampleCounter >= REF_SAMPLE_INTERVAL) {
  //   refSampleCounter = 0;
  //   measureFilterError();
  // }

  // ---- ONE_CHANNEL MODE: measure current channel continuously ----
  if (scanner.isOneChannel()) {
    InputChannel curCh = inputMux.current();

    // Clear stats and reset integration counter whenever the channel changes
    // (covers range switches, ScopedInstrumentState restore, first entry after scan stop)
    static InputChannel lastOneChannel = static_cast<InputChannel>(0xFF);
    static int loopCounter = 0;
    if (curCh != lastOneChannel) {
      scanner.channelStats((uint8_t)curCh).clear();
      loopCounter = 0;
      lastOneChannel = curCh;
    }

    LowerMoments &stats = scanner.channelStats((uint8_t)curCh);
	  int32_t* bA = scanner.bufferA((uint8_t)curCh);
	  int32_t* bB = scanner.bufferB((uint8_t)curCh);
    if (!runOneChopCycle(stats, bA, bB)) {
      scanner.resetChannelEwma((uint8_t)curCh);  // DAC re-centered: old EWMA history is stale
      loopCounter = 0;
      return;  // Overflow handled, restart
    }

    // Output every integrationCycles, then clear for fixed-window integration
    if (++loopCounter >= scanner.config.integrationCycles) {
      loopCounter = 0;
      outputMeasurement(curCh, stats, dac.currentCode());
      stats.clear();

      // Auto-zero: same interval logic as SCANNING mode (counts readings, not sweeps)
      scanner.incCycleCount();
      if (scanner.config.autoZeroEnabled &&
          (scanner.cycleCount() % scanner.config.autoZeroInterval) == 0) {
        performAutoZero();
        if (scanner.autoZeroValid && scanner.config.outputMode == OutputMode::Human) {
          Serial.print("  [auto-zero: offset=");
          Serial.print(scanner.autoZeroOffset * 1e9, 1);
          Serial.print(" nV  sdev=");
          Serial.print(scanner.autoZeroSdev * 1e9, 1);
          Serial.println(" nV]");
        }
      }
    }
    return;
  }

  // ---- SCANNING MODE ----
  InputChannel scanCh = scanner.config.channels[scanner.currentScanIndex()];

  // Run one chop cycle on the current channel
  LowerMoments &stats = scanner.channelStats((uint8_t)scanCh);
  int32_t* bA = scanner.bufferA((uint8_t)scanCh);
  int32_t* bB = scanner.bufferB((uint8_t)scanCh);
  if (!runOneChopCycle(stats, bA, bB)) {
    scanner.setChopCycleCount(0);  // Reset after overflow/DAC change
    scanner.setChannelDac((uint8_t)scanCh, dac.currentCode());
    scanner.resetChannelEwma((uint8_t)scanCh);  // DAC re-centered: old EWMA history is stale
    return;
  }

  scanner.incChopCycleCount();

  // Discard initial settling cycles after channel switch
  if (scanner.chopCycleCount() <= SCAN_SETTLE_CYCLES) {
    stats.clear();
    return;
  }

  // Check if integration is complete for this channel
  if (scanner.chopCycleCount() >= scanner.config.integrationCycles + SCAN_SETTLE_CYCLES) {
    // Output this channel's result
    // For multi-channel plotter mode, we buffer all channels and print one line per sweep
    if (scanner.config.outputMode == OutputMode::Plotter && scanner.config.count > 1) {
      // Multi-channel plotter: assemble one tab-separated line per sweep
      double adcMean = stats.mean();
      double adcStdDev = stats.standardDeviation();
      double vxRaw = computeInputVoltage(adcMean, dac.currentCode(), scanCh);
      double refCorrection = getRefCorrectionFactor();
      double vxCorrected = vxRaw * refCorrection;
      if (scanner.autoZeroValid) vxCorrected -= scanner.autoZeroOffset;
      double vxUncertainty = computeInputUncertainty(adcStdDev, scanCh);
      double driftPpb = (refCorrection - 1.0) * 1e9;

      AdjustedEWMA& ewma = scanner.channelEwma((uint8_t)scanCh);
      double ewmaV = ewma.update(vxCorrected);
      MeasurementData data;
      data.channel = scanCh; data.voltage = vxCorrected; data.uncertainty = vxUncertainty;
      data.adcMean = adcMean; data.adcStdDev = adcStdDev; data.count = stats.count();
      data.dacCode = dac.currentCode(); data.driftPpb = driftPpb;
      data.adcMin = stats.getMin(); data.adcMax = stats.getMax();
      data.ewmaVoltage = ewmaV; data.ewmaValid = ewma.isInitialized();

      IOutputFormatter& fmt = getFormatter();
      if (scanner.currentScanIndex() == 0) fmt.beginSweep();
      fmt.formatMeasurement(data);
      if (scanner.currentScanIndex() == scanner.config.count - 1) {
        fmt.endSweep();
        // Demod outlier-rejection report (once per sweep, comment-prefixed so plotters skip it).
        if (g_demodRejSinceOutput > 0) {
          Serial.print("# demod-clip: ");
          Serial.print(g_demodRejSinceOutput); Serial.print('/');
          Serial.print(g_demodPairsSinceOutput);
          Serial.print(" since last output (cum ");
          Serial.print((unsigned long)g_demodRejTotal); Serial.print('/');
          Serial.print((unsigned long)g_demodPairsTotal);
          Serial.println(")");
        }
        g_demodRejSinceOutput   = 0;
        g_demodPairsSinceOutput = 0;
      }
    } else {
      outputMeasurement(scanCh, stats, dac.currentCode());
    }

    // Save DAC code for this channel (for fast restore on next visit)
    scanner.setChannelDac((uint8_t)scanCh, dac.currentCode());

    // Advance to next channel
    scanner.setCurrentScanIndex(scanner.currentScanIndex() + 1);

    if (scanner.currentScanIndex() >= scanner.config.count) {
      // Completed one full sweep
      scanner.setCurrentScanIndex(0);
      scanner.incCycleCount();

      // Check if auto-zero is due
      if (scanner.config.autoZeroEnabled &&
          (scanner.cycleCount() % scanner.config.autoZeroInterval) == 0) {
        performAutoZero();
        if (scanner.autoZeroValid && scanner.config.outputMode == OutputMode::Human) {
          Serial.print("  [auto-zero: offset=");
          Serial.print(scanner.autoZeroOffset * 1e9, 1);
          Serial.print(" nV  sdev=");
          Serial.print(scanner.autoZeroSdev * 1e9, 1);
          Serial.println(" nV]");
        }
      }
    }

    // Switch to next channel
    InputChannel nextCh = scanner.config.channels[scanner.currentScanIndex()];
    if (nextCh != inputMux.current()) {
      inputMux.select(nextCh);
      uint8_t idx = (uint8_t)nextCh;
      if (scanner.channelDacValid(idx)) {
        // Restore saved DAC code (fast path: only filter settling, no search)
        dac.setCode(scanner.channelDacCode(idx));
      } else {
        // First visit to this channel: full binary search
        binarySearchDAC();
        scanner.setChannelDac(idx, dac.currentCode());
      }
    }
    scanner.channelStats((uint8_t)nextCh).clear();
    scanner.setChopCycleCount(0);
  }
}
