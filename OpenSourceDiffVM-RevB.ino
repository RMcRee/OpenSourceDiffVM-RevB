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

// ---------------- Forward Declarations ----------------
// Types must be declared before Arduino auto-generates function prototypes
// (prototypes are inserted after the last #include, which is <limits> below)
struct HalfCycleResult {
  int64_t sum;
  bool overflow;
};
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

// ---------------- POST Configuration ----------------
// Set to true to halt on POST failure, false to continue with warning
static constexpr bool POST_HALT_ON_FAIL = true;

// Zero calibration test threshold (in nanovolts at input)
// With preamp gain of 2000 and ADC LSB of ~298nV, this translates to ADC counts.
// Default 1000 nV = 1 µV allows for small offsets and noise.
static constexpr double POST_ZERO_THRESHOLD_NV = 1000.0;  // nV

// POST result structure
struct PostResult {
  bool adc_comm;
  bool dac_comm;
  bool drdy_timing;
  bool signal_chain;
  bool polarity;
  bool chop_symmetry;
  bool input_mux;
  bool references;
  bool zero_cal;

  bool allPassed() const {
    return adc_comm && dac_comm && drdy_timing &&
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
static constexpr float    ADC_FCLK_HZ       = 25'000'000.0f;   // external clock
static constexpr float    ADC_FMOD_HZ       = ADC_FCLK_HZ / 2; // fMOD = fCLK/2
static constexpr uint32_t SETTLE_US         = 300;             // guard time after each mux edge before using samples
static constexpr uint8_t  DISCARD_SAMPLES   = 1;               // toss first sample after edge
static constexpr uint8_t  GOOD_SAMPLES      = 3;               // samples to average per half-cycle
static constexpr int      PULSE_WIDTH_US    = 5;               // CI_TMUXSEL pre-transition pulse duration

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
  Vx        = 0,  // S1: Unknown input (normal measurement, ±5V)
  GND       = 1,  // S2: Ground reference (zero calibration)
  VrefRaw   = 2,  // S3: Raw (pre-filter) ADR1001 average (J3) for drift compensation
  Spare1    = 3,  // S4: Spare input
  Spare2    = 4,  // S5: Spare input
  HVDivider = 5,  // S6: HV divider output (ratio selected by MUX36D04)
  Spare3    = 6,  // S7: Spare input
  Spare4    = 7,  // S8: Spare input
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

// ---------------- Overflow Detection ----------------
// 24-bit ADC full scale: ±8,388,607. Use 90% threshold for overflow detection.
static constexpr int32_t ADC_OVERFLOW_POS   =  7'549'747;      // ~90% of 0x7FFFFF
static constexpr int32_t ADC_OVERFLOW_NEG   = -7'549'747;      // ~90% of -0x800000

// ---------------- Precision Voltage Constants (64-bit double) ----------------
// ADC: ADS127L11, 24-bit, differential, VREF = 2.500V
// Full scale range: ±VREF = ±2.5V → 2^24 codes span 5V
static constexpr double ADC_VREF        = 2.5;                           // ADC reference voltage (V)
static constexpr double ADC_FSR_CODES   = 16777216.0;                    // 2^24 full scale codes
static constexpr double ADC_LSB_V       = (2.0 * ADC_VREF) / ADC_FSR_CODES;  // ~298 nV/LSB

// DAC: AD5760, 16-bit, two's complement in 2x mode, VREFP=5V, VREFN=0V
// Output range: ±5V (with 2x gain), codes 0x8000 (-32768) to 0x7FFF (+32767)
static constexpr double DAC_VREF        = 5.0;                           // DAC full-scale voltage (V)
static constexpr double DAC_FSR_CODES   = 65536.0;                       // 2^16 codes
static constexpr double DAC_LSB_V       = (2.0 * DAC_VREF) / DAC_FSR_CODES;  // ~152.6 µV/LSB

// Preamp: 4× AD8428 in cascade, total gain = 2000
// Runtime-mutable for in-situ calibration. Use 'cal set gain <v>' to adjust.
static constexpr double PREAMP_GAIN_DEFAULT = 2000.0;
static double PREAMP_GAIN = PREAMP_GAIN_DEFAULT;

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
    valid_ = false;
  }

  double codeToVoltage(int16_t code) const {
    if (!valid_) {
      return (double)code * DAC_LSB_V;
    }
    uint16_t idx = ((uint16_t)(code + 32768)) >> 2;
    if (idx >= TABLE_SIZE - 1) {
      return table_[TABLE_SIZE - 1];
    }
    uint8_t frac = ((uint16_t)(code + 32768)) & 0x03;
    double v0 = table_[idx];
    double v1 = table_[idx + 1];
    return v0 + (v1 - v0) * (double)frac / 4.0;
  }

  void setPoint(int16_t code, double measuredVoltage) {
    uint16_t idx = ((uint16_t)(code + 32768)) >> 2;
    if (idx < TABLE_SIZE) {
      table_[idx] = measuredVoltage;
    }
  }

  void markValid() { valid_ = true; }
  bool isValid() const { return valid_; }

  // Raw table access for CalibrationStore flash I/O
  double* rawTable() { return table_; }
  const double* rawTable() const { return table_; }
  friend class CalibrationStore;

private:
  double table_[TABLE_SIZE];
  bool valid_ = false;
};

static DacCalibrationTable dacCalTable;

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

  // Step 4: Scale by voltage divider ratio (if using extended range)
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

// Estimated sample period for FILTER=11000b (OSR 32*100 = 3200)
static constexpr float    OSR_TOTAL         = 3200.0f;
static constexpr float    EST_FSPS          = ADC_FMOD_HZ / OSR_TOTAL;     // ~3906.25 SPS
static constexpr float    EST_TS_US         = 1e6f / EST_FSPS;             // ~256 us/sample

// ---------------- SPI ----------------
static constexpr uint32_t SPI_HZ = 5'000'000;
static constexpr uint8_t  ADC_SPI_MODE = SPI_MODE1; // try SPI_MODE0 if needed

SPISettings adcSpiSettings(SPI_HZ, MSBFIRST, ADC_SPI_MODE);
SPISettings dacSpiSettings(SPI_HZ, MSBFIRST, ADC_SPI_MODE);

// ---------------- ADS127L11 register map ----------------
static constexpr uint8_t REG_DEV_ID  = 0x00;  // Device ID register
static constexpr uint8_t REG_STATUS  = 0x01;  // Status register
static constexpr uint8_t REG_CONFIG1 = 0x05;
static constexpr uint8_t REG_CONFIG2 = 0x06;
static constexpr uint8_t REG_CONFIG3 = 0x07;
static constexpr uint8_t REG_CONFIG4 = 0x08;

// Expected ADS127L11 device ID (from datasheet)
static constexpr uint8_t ADS127L11_DEV_ID = 0x00;  // DEV_ID[7:2]=0, verify communication works

// ---------------- ADS127L11 SPI opcodes ----------------
static constexpr uint8_t OPCODE_RREG = 0x20;
static constexpr uint8_t OPCODE_WREG = 0x40;

static constexpr uint8_t CMD_RESET   = 0x06;
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
  InputChannel current_ = InputChannel::Vx;
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

// -------- DacDriver (AD5760) --------
class DacDriver {
public:
  void begin();
  void setCode(int16_t code);
  void setCodeFast(int16_t code);  // No filter settling — for binary search only
  int16_t currentCode() const { return currentCode_; }

  // Low-level (used by POST / calibration)
  void writeCode(uint16_t code);
  void writeControl(bool sdoDis, bool offsBin, bool tri, bool gnd, bool rbuf);

private:
  void write24(uint32_t frame);
  void writeClearCode(uint16_t code);
  void softCtrl(bool doReset, bool doClr, bool doLdac);
  static uint32_t calculateSettleTime(int32_t stepSize);
  int16_t currentCode_ = 0;
};
static DacDriver dac;

// -------- AdcDriver (ADS127L11) --------
class AdcDriver {
public:
  void begin();
  void initAndConfigure();
  void command(uint8_t cmd);
  void writeReg(uint8_t addr, uint8_t val);
  uint8_t readReg(uint8_t addr);
  bool waitDrdy(uint32_t timeout_us = 1000);
  bool readSample24(int32_t &sample);
private:
  uint8_t xfer(uint8_t v);
};
static AdcDriver adc;

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
  inputMux.select(InputChannel::HVDivider);
  delayMicroseconds(INMUX_SETTLE_US);
}

// ---------------- Voltage Divider Scale Factors ----------------
double getInputDividerRatio(InputChannel channel) {
  if (channel == InputChannel::HVDivider) {
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
//
// LowerMoments.h / drop-in class for Teensy/Arduino (64-bit double supported)

#include <cmath>
#include <cfloat>
#include <limits>

class LowerMoments {
public:
  LowerMoments() { clear(); }

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

// ================== Allan Deviation (OADEV) Tracker ==================
//
// Real-time overlapping Allan deviation computation for voltage stability
// analysis. Stores a circular buffer of corrected voltage readings and
// computes OADEV at octave-spaced averaging times on demand.
//
// OADEV^2(m) = 1/(2(N-2m)) * SUM (y_bar_{i+m} - y_bar_i)^2
// where y_bar_i = (1/m) * SUM_{k=0}^{m-1} y_{i+k}

class AllanDeviation {
public:
  static constexpr int MAX_READINGS = 4096;  // ~17 min at tau0=0.26s (32 KB)
  static constexpr int MAX_TAUS = 12;        // octave taus: m=1,2,4,...,2048

  AllanDeviation() { clear(); }

  void clear() {
    head_ = 0;
    count_ = 0;
  }

  void addReading(double voltage) {
    readings_[head_] = voltage;
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
    Serial.println("s)");
    Serial.println("  tau (s)      OADEV          pairs");
    Serial.println("  ----------   ------------   -----");

    int nOct = numOctaves();
    for (int k = 0; k < nOct; k++) {
      int m = 1 << k;
      double ad = computeForM(m);
      if (std::isnan(ad)) break;

      double tau = (double)m * tau0;
      int pairs = count_ - 2 * m;

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
  }

private:
  double readings_[MAX_READINGS];
  int head_;      // next write position
  int count_;     // total readings stored (up to MAX_READINGS)

  // Linearize circular buffer index: oldest reading is at index 0
  double reading(int i) const {
    // oldest = head_ - count_, wrapped
    int idx = (head_ - count_ + i + MAX_READINGS) % MAX_READINGS;
    return readings_[idx];
  }

  double computeForM(int m) const {
    if (m < 1 || count_ < 2 * m + 1) {
      return std::numeric_limits<double>::quiet_NaN();
    }

    // Compute prefix sums over linearized readings
    // To avoid allocating a big array, compute block sums on the fly
    // using a sliding window approach
    int pairs = count_ - 2 * m;

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

// -------- DAC (DAC) register addresses (3-bit) --------
static constexpr uint8_t DAC_ADDRESS       = 0b001;
static constexpr uint8_t DAC_ADDR_CTRL     = 0b010;
static constexpr uint8_t DAC_ADDR_CLEAR    = 0b011;
static constexpr uint8_t DAC_ADDR_SOFTCTRL = 0b100;

// ================== DacDriver method implementations ==================

void DacDriver::write24(uint32_t frame) {
  SPI.beginTransaction(dacSpiSettings);
  digitalWrite(PIN_CS_DAC, LOW);
  SPI.transfer((frame >> 16) & 0xFF);
  SPI.transfer((frame >>  8) & 0xFF);
  SPI.transfer((frame >>  0) & 0xFF);
  digitalWrite(PIN_CS_DAC, HIGH);
  SPI.endTransaction();
}

void DacDriver::writeCode(uint16_t code) {
  const uint32_t rw   = 0u;
  const uint32_t addr = DAC_ADDRESS;
  const uint32_t data = (uint32_t)code << 4;
  const uint32_t frame = (rw << 23) | (addr << 20) | (data & 0xFFFFF);
  write24(frame);
}

void DacDriver::writeClearCode(uint16_t code) {
  const uint32_t rw   = 0u;
  const uint32_t addr = DAC_ADDR_CLEAR;
  const uint32_t data = (uint32_t)code << 4;
  const uint32_t frame = (rw << 23) | (addr << 20) | (data & 0xFFFFF);
  write24(frame);
}

void DacDriver::writeControl(bool sdoDisable, bool offsetBinary,
                              bool dacTriState, bool opGndClamp, bool rbufUnityMode) {
  uint32_t data = 0;
  data |= (uint32_t)(sdoDisable   ? 1 : 0) << 5;
  data |= (uint32_t)(offsetBinary ? 1 : 0) << 4;
  data |= (uint32_t)(dacTriState  ? 1 : 0) << 3;
  data |= (uint32_t)(opGndClamp   ? 1 : 0) << 2;
  data |= (uint32_t)(rbufUnityMode? 1 : 0) << 1;

  const uint32_t rw   = 0u;
  const uint32_t addr = DAC_ADDR_CTRL;
  const uint32_t frame = (rw << 23) | (addr << 20) | (data & 0xFFFFF);
  write24(frame);
}

void DacDriver::softCtrl(bool doReset, bool doClr, bool doLdac) {
  uint32_t data = 0;
  data |= (uint32_t)(doReset ? 1 : 0) << 2;
  data |= (uint32_t)(doClr   ? 1 : 0) << 1;
  data |= (uint32_t)(doLdac  ? 1 : 0) << 0;

  const uint32_t rw   = 0u;
  const uint32_t addr = DAC_ADDR_SOFTCTRL;
  const uint32_t frame = (rw << 23) | (addr << 20) | (data & 0xFFFFF);
  write24(frame);
}

void DacDriver::begin() {
  softCtrl(true, false, false);  // Software reset
  delayMicroseconds(10);
  writeClearCode(0x0000);
  writeCode(0x0000);
  writeControl(false, false, false, false, false);  // 2x / gain-of-two config
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
  current_ = InputChannel::Vx;
}

void InputMuxDriver::select(InputChannel channel) {
  if (channel == current_) return;

  uint8_t addr = static_cast<uint8_t>(channel);
  digitalWriteFast(PIN_INMUX_A0, (addr & 0x01) ? HIGH : LOW);
  digitalWriteFast(PIN_INMUX_A1, (addr & 0x02) ? HIGH : LOW);
  digitalWriteFast(PIN_INMUX_A2, (addr & 0x04) ? HIGH : LOW);
  current_ = channel;
  delayMicroseconds(INMUX_SETTLE_US);
}

const char* getInputChannelName(InputChannel channel) {
  switch (channel) {
    case InputChannel::Vx:        return "Vx (±5V)";
    case InputChannel::GND:       return "GND";
    case InputChannel::VrefRaw:   return "VrefRaw";
    case InputChannel::Spare1:    return "Spare1";
    case InputChannel::Spare2:    return "Spare2";
    case InputChannel::HVDivider: return "HVDivider";
    case InputChannel::Spare3:    return "Spare3";
    case InputChannel::Spare4:    return "Spare4";
    default:                      return "Unknown";
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
  writeCode((uint16_t)code);

  uint32_t settleMs = calculateSettleTime(stepSize);
  if (settleMs > 0) {
    delay(settleMs);
  }
}

void DacDriver::setCodeFast(int16_t code) {
  currentCode_ = code;
  writeCode((uint16_t)code);
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
// Returns true on success, false on ADC timeout.
bool readSettledSample(int32_t &sample) {
  delayMicroseconds(SETTLE_US);
  int32_t discard;
  if (!adc.readSample24(discard)) return false;  // discard one
  return adc.readSample24(sample);
}

/**
 * Binary search to find DAC code that brings ADC into range.
 * Returns true if successful, false if unable to find valid code.
 *
 * The search uses TMUXSEL=LOW where the preamp measures (Vx - DAC) * gain:
 *   - If ADC overflows positive: Vx > DAC, need to increase DAC
 *   - If ADC overflows negative: Vx < DAC, need to decrease DAC
 *
 * Hardware signal routing (from schematic):
 *   MUXSEL=0 (LOW):  DAC→pin1(V-), Vx→pin4(V+) → output = (Vx - DAC) × gain
 *   MUXSEL=1 (HIGH): DAC→pin4(V+), Vx→pin1(V-) → output = (DAC - Vx) × gain
 *
 * DAC codes are constrained to 14-bit boundaries (multiples of 4) since
 * only these codes have calibration data. This gives 2^14 = 16384 points
 * across the ±5V range.
 */
bool binarySearchDAC() {
  Serial.println("DAC: Starting binary search (14-bit aligned)...");

  // Force TMUXSEL=LOW so ADC reads (Vx - DAC) * gain
  // This ensures consistent search direction regardless of prior mux state
  // Use chopper.toggle() to switch properly with CI_TMUXSEL sequence
  if (chopper.state()) {
    chopper.toggle();  // Toggle HIGH -> LOW using charge-injection-minimizing sequence
  }

  // Search bounds constrained to 14-bit boundaries (multiples of 4)
  // 16-bit range: -32768 to 32767, but max aligned value is 32764
  int32_t low = -32768;   // Min DAC code (aligned)
  int32_t high = 32764;   // Max DAC code (aligned: 32767 & ~3)
  int32_t mid;

  const int MAX_ITERATIONS = 16;  // 14 bits + margin

  for (int iter = 0; iter < MAX_ITERATIONS; iter++) {
    // Compute midpoint and align to 14-bit boundary (multiple of 4)
    mid = ((low + high) / 2) & ~3;
    dac.setCodeFast((int16_t)mid);  // Fast write — no full filter settle

    int32_t sample;
    if (!readSettledSample(sample)) {
      Serial.println("DAC: ADC timeout during binary search.");
      return false;
    }

    Serial.print("  iter=");
    Serial.print(iter);
    Serial.print(" DAC=");
    Serial.print(mid);
    Serial.print(" ADC=");
    Serial.println(sample);

    if (isOverflowPositive(sample)) {
      // Vx > DAC, increase DAC
      low = mid + 4;  // Next 14-bit boundary
    } else if (isOverflowNegative(sample)) {
      // Vx < DAC, decrease DAC
      high = mid - 4;  // Previous 14-bit boundary
    } else {
      // Converged — now do full settle at final code for clean integration
      dac.setCode((int16_t)mid);
      Serial.print("DAC: Converged at code ");
      Serial.print(mid);
      Serial.println(" (14-bit aligned)");
      return true;
    }

    if (low > high) {
      break;  // Search space exhausted
    }
  }

  Serial.println("DAC: Binary search failed to converge!");
  return false;
}

// ================== AdcDriver method implementations ==================

uint8_t AdcDriver::xfer(uint8_t v) { return SPI.transfer(v); }

void AdcDriver::command(uint8_t cmd) {
  SPI.beginTransaction(adcSpiSettings);
  csLow(PIN_CS_ADC);
  xfer(cmd);
  csHigh(PIN_CS_ADC);
  SPI.endTransaction();
  delayMicroseconds(2);
}

void AdcDriver::writeReg(uint8_t addr, uint8_t value) {
  SPI.beginTransaction(adcSpiSettings);
  csLow(PIN_CS_ADC);
  xfer(OPCODE_WREG | (addr & 0x1F));
  xfer(0x00);
  xfer(value);
  csHigh(PIN_CS_ADC);
  SPI.endTransaction();
  delayMicroseconds(2);
}

uint8_t AdcDriver::readReg(uint8_t addr) {
  SPI.beginTransaction(adcSpiSettings);
  csLow(PIN_CS_ADC);
  xfer(OPCODE_RREG | (addr & 0x1F));
  xfer(0x00);
  uint8_t v = xfer(0x00);
  csHigh(PIN_CS_ADC);
  SPI.endTransaction();
  delayMicroseconds(2);
  return v;
}

bool AdcDriver::waitDrdy(uint32_t timeout_us) {
  elapsedMicros t = 0;
  while (t < timeout_us) {
    if (digitalReadFast(PIN_ADC_DRDY) == LOW) return true;
  }
  return false;
}

bool AdcDriver::readSample24(int32_t &sample) {
  if (!waitDrdy(1000)) {
    sample = 0;
    return false;
  }

  SPI.beginTransaction(adcSpiSettings);
  csLow(PIN_CS_ADC);

  uint8_t b2 = xfer(0x00);
  uint8_t b1 = xfer(0x00);
  uint8_t b0 = xfer(0x00);

  csHigh(PIN_CS_ADC);
  SPI.endTransaction();

  sample = (int32_t)((uint32_t)b2 << 16 | (uint32_t)b1 << 8 | (uint32_t)b0);
  if (sample & 0x00800000) sample |= 0xFF000000;
  return true;
}

void AdcDriver::begin() {
  analogWriteFrequency(PIN_CLK_ADC, 25000000);
  analogWrite(PIN_CLK_ADC, 128);
}

void AdcDriver::initAndConfigure() {
  command(CMD_RESET);
  delay(5);

  const uint8_t delayCode  = 0b000;
  const uint8_t filterCode = 0b11000;
  const uint8_t config3 = (delayCode << 5) | (filterCode & 0x1F);
  writeReg(REG_CONFIG3, config3);

  const uint8_t config4 = (1u << 7);
  writeReg(REG_CONFIG4, config4);

  uint8_t r3 = readReg(REG_CONFIG3);
  uint8_t r4 = readReg(REG_CONFIG4);

  Serial.print("ADC CONFIG3 set/read: 0x"); Serial.print(config3, HEX);
  Serial.print(" / 0x"); Serial.println(r3, HEX);

  Serial.print("ADC CONFIG4 set/read: 0x"); Serial.print(config4, HEX);
  Serial.print(" / 0x"); Serial.println(r4, HEX);

  command(CMD_START);
}

// ================== ChopperDriver method implementations ==================

void ChopperDriver::begin() {
  pinMode(PIN_TMUX_EN, OUTPUT);
  pinMode(PIN_TMUXSEL, OUTPUT);
  pinMode(PIN_CI_TMUXSEL, OUTPUT);
  digitalWrite(PIN_TMUXSEL, LOW);
  digitalWrite(PIN_CI_TMUXSEL, LOW);
  digitalWriteFast(PIN_TMUX_EN, LOW);
  state_ = false;
}

void ChopperDriver::toggle() {
  digitalWriteFast(PIN_CI_TMUXSEL, HIGH);
  delayMicroseconds(PULSE_WIDTH_US);
  state_ = !state_;
  digitalWriteFast(PIN_TMUXSEL, state_);
  delayMicroseconds(PULSE_WIDTH_US);
  digitalWriteFast(PIN_CI_TMUXSEL, LOW);
}

// ---------------- Chopped acquisition ----------------

// Collect samples for one half-cycle, detect overflow on first sample.
// Sets result.overflow on ADC overflow OR timeout.
HalfCycleResult acquireHalfCycle() {
  HalfCycleResult result = {0, false};

  delayMicroseconds(SETTLE_US);

  // Discard initial samples after settling
  int32_t discard;
  for (uint8_t i = 0; i < DISCARD_SAMPLES; i++) {
    if (!adc.readSample24(discard)) { result.overflow = true; return result; }
  }

  // Read first good sample and check for overflow
  int32_t firstSample;
  if (!adc.readSample24(firstSample)) { result.overflow = true; return result; }
  if (isOverflow(firstSample)) {
    result.overflow = true;
    return result;
  }
  result.sum = firstSample;

  // Accumulate remaining good samples
  for (uint8_t i = 1; i < GOOD_SAMPLES; i++) {
    int32_t s;
    if (!adc.readSample24(s)) { result.overflow = true; return result; }
    result.sum += s;
  }
  return result;
}

// Statistics accumulator for chopped measurements
LowerMoments chopStats;

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
    .channels = { InputChannel::Vx },
    .count = 1,
    .integrationCycles = 100,
    .autoZeroEnabled = true,
    .autoZeroInterval = 10,
    .outputMode = OutputMode::Human,
  };

  // State queries
  ScanState state() const { return state_; }
  bool isScanning() const { return state_ == ScanState::SCANNING; }
  bool isOneChannel() const { return state_ == ScanState::ONE_CHANNEL; }

  void start() { state_ = ScanState::SCANNING; currentScanIndex_ = 0; scanCycleCount_ = 0; chopCycleCount_ = 0; }
  void stop()  { state_ = ScanState::ONE_CHANNEL; }

  // Per-channel statistics
  LowerMoments& channelStats(uint8_t idx) { return channelStats_[idx]; }

  // Per-channel DAC code cache
  int16_t channelDacCode(uint8_t idx) const { return channelDacCode_[idx]; }
  bool    channelDacValid(uint8_t idx) const { return channelDacValid_[idx]; }
  void    setChannelDac(uint8_t idx, int16_t code) { channelDacCode_[idx] = code; channelDacValid_[idx] = true; }
  void    clearDacCache() { for (int i = 0; i < 8; i++) { channelDacCode_[i] = 0; channelDacValid_[i] = false; } }

  // Auto-zero
  double autoZeroOffset = 0.0;
  bool autoZeroValid = false;

  // Scan loop state
  int currentScanIndex() const { return currentScanIndex_; }
  void setCurrentScanIndex(int i) { currentScanIndex_ = i; }
  int scanCycleCount() const { return scanCycleCount_; }
  void incScanCycleCount() { scanCycleCount_++; }
  int chopCycleCount() const { return chopCycleCount_; }
  void setChopCycleCount(int c) { chopCycleCount_ = c; }
  void incChopCycleCount() { chopCycleCount_++; }

private:
  ScanState state_ = ScanState::ONE_CHANNEL;
  int currentScanIndex_ = 0;
  int scanCycleCount_ = 0;
  int chopCycleCount_ = 0;
  LowerMoments channelStats_[8];
  int16_t channelDacCode_[8];
  bool channelDacValid_[8];
};

static ScanController scanner;

// ================== Serial Command Interface ==================

static char cmdBuffer[128];
static int cmdBufferIdx = 0;

/**
 * Parse a channel name string (case-insensitive) to InputChannel enum.
 * Inverse of getChannelShortName(). Called by cmdScanAdd() and cmdScanRemove()
 * to convert user-typed channel names from serial commands.
 *
 * @param name  Channel name string (e.g., "Vx", "VrefRaw", "HVDivider")
 * @param ch    Output: corresponding InputChannel value
 * @return      true on success, false if name not recognized
 */
bool parseChannelName(const char* name, InputChannel &ch) {
  if (strcasecmp(name, "Vx") == 0)        { ch = InputChannel::Vx; return true; }
  if (strcasecmp(name, "GND") == 0)       { ch = InputChannel::GND; return true; }
  if (strcasecmp(name, "VrefRaw") == 0)   { ch = InputChannel::VrefRaw; return true; }
  if (strcasecmp(name, "Spare1") == 0)    { ch = InputChannel::Spare1; return true; }
  if (strcasecmp(name, "Spare2") == 0)    { ch = InputChannel::Spare2; return true; }
  if (strcasecmp(name, "HVDivider") == 0) { ch = InputChannel::HVDivider; return true; }
  if (strcasecmp(name, "Spare3") == 0)    { ch = InputChannel::Spare3; return true; }
  if (strcasecmp(name, "Spare4") == 0)    { ch = InputChannel::Spare4; return true; }
  return false;
}

/**
 * Get short channel name string without range/description suffix.
 * Used by CsvFormatter and PlotterFormatter where compact names are needed.
 * Compare with getInputChannelName() which returns descriptive names like "Vx (±5V)".
 *
 * @param channel  Input channel enum value
 * @return         Static string: "Vx", "GND", "VrefRaw", etc.
 */
const char* getChannelShortName(InputChannel channel) {
  switch (channel) {
    case InputChannel::Vx:        return "Vx";
    case InputChannel::GND:       return "GND";
    case InputChannel::VrefRaw:   return "VrefRaw";
    case InputChannel::Spare1:    return "Spare1";
    case InputChannel::Spare2:    return "Spare2";
    case InputChannel::HVDivider: return "HVDivider";
    case InputChannel::Spare3:    return "Spare3";
    case InputChannel::Spare4:    return "Spare4";
    default:                      return "Unknown";
  }
}

// Forward declarations for command handlers and shared helpers
bool runOneChopCycle(LowerMoments &stats, bool searchOnOverflow = true);
void performAutoZero();
void printCsvHeader();

void cmdScanAdd(const char* arg) {
  InputChannel ch;
  if (!parseChannelName(arg, ch)) {
    Serial.print("ERROR: Unknown channel '");
    Serial.print(arg);
    Serial.println("'. Use: Vx, GND, VrefRaw, Spare1-4, HVDivider");
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

void cmdIntegrate(const char* arg) {
  int n = atoi(arg);
  if (n < 10 || n > 10000) {
    Serial.println("ERROR: Integration cycles must be 10-10000.");
    return;
  }
  scanner.config.integrationCycles = n;
  Serial.print("Integration set to ");
  Serial.print(n);
  Serial.print(" chop cycles (~");
  Serial.print((float)n / EST_FSPS * (DISCARD_SAMPLES + GOOD_SAMPLES + 1) * 2, 1);
  Serial.println(" sec per reading).");
}

void cmdAutoZero(const char* arg) {
  if (strcasecmp(arg, "on") == 0) {
    scanner.config.autoZeroEnabled = true;
    Serial.println("Auto-zero enabled.");
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
    Serial.println("ERROR: Auto-zero interval must be 1-1000 scan sweeps.");
    return;
  }
  scanner.config.autoZeroInterval = n;
  Serial.print("Auto-zero interval set to ");
  Serial.print(n);
  Serial.println(" scan sweeps.");
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
  Serial.println(" chop cycles");

  Serial.print("Auto-zero: ");
  Serial.print(scanner.config.autoZeroEnabled ? "ON" : "OFF");
  Serial.print(", interval=");
  Serial.print(scanner.config.autoZeroInterval);
  Serial.println(" sweeps");

  if (scanner.autoZeroValid) {
    Serial.print("Auto-zero offset: ");
    Serial.print(scanner.autoZeroOffset * 1e9, 1);
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

  Serial.print("DAC calibration: ");
  Serial.println(dacCalTable.isValid() ? "CALIBRATED" : "nominal");

  cmdScanList();
  Serial.println("=====================\n");
}

void printHelp() {
  Serial.println("\n=== DiffVM Commands ===");
  Serial.println("scan add <ch>       Add channel (Vx,GND,VrefRaw,Spare1-4,HVDivider)");
  Serial.println("scan remove <ch>    Remove channel from scan list");
  Serial.println("scan list           Show current scan list");
  Serial.println("scan clear          Clear scan list");
  Serial.println("scan start          Begin automatic scanning");
  Serial.println("scan stop           Stop scanning");
  Serial.println("log start           Start CSV logging output");
  Serial.println("log stop            Stop CSV logging");
  Serial.println("plot start          Start Arduino Serial Plotter output");
  Serial.println("plot stop           Stop plotter output");
  Serial.println("integrate <N>       Set chop cycles per reading (10-10000)");
  Serial.println("autozero on|off     Enable/disable periodic auto-zero");
  Serial.println("autozero interval <N>  Scan sweeps between auto-zero (1-1000)");
  Serial.println("range <10|100|1000> Set HV divider ratio");
  Serial.println("status              Show current configuration");
  Serial.println("zero                Perform immediate zero calibration");
  Serial.println("config save         Save config to EEPROM");
  Serial.println("config load         Load config from EEPROM");
  Serial.println("config factory      Reset to factory defaults");
  Serial.println("config autostart on|off  Auto-start scanning on boot");
  Serial.println("config show         Show saved vs current config");
  Serial.println("adev [clear]          Show Allan deviation (or clear history)");
  Serial.println("cal status            Show calibration flash status and current values");
  Serial.println("cal save              Save constants + DAC table to flash");
  Serial.println("cal load              Reload constants + DAC table from flash");
  Serial.println("cal erase             Erase calibration files from flash");
  Serial.println("cal set gain <v>      Set PREAMP_GAIN (e.g. 1998.5)");
  Serial.println("cal set div10 <v>     Set DIVIDER_RATIO_10");
  Serial.println("cal set div100 <v>    Set DIVIDER_RATIO_100");
  Serial.println("cal set div1000 <v>   Set DIVIDER_RATIO_1000");
  Serial.println("cal factory           Reset cal constants to factory defaults (RAM)");
  Serial.println("cal build dac         Auto-calibrate DAC table at GND and VrefRaw anchors");
  Serial.println("cal point <voltage>   Capture DAC table entry at known Vx (e.g., cal point 1.23456)");
  Serial.println("help                Show this help");
  Serial.println("=======================\n");
}

// Compute measurement interval (tau0) in seconds for current integration setting
static double computeTau0() {
  return (double)scanner.config.integrationCycles / EST_FSPS
         * (DISCARD_SAMPLES + GOOD_SAMPLES + 1) * 2;
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
    if (!arg1) { Serial.println("Usage: integrate <10-10000>"); return; }
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
  } else if (strcasecmp(cmd, "adev") == 0) {
    cmdAdev(arg1);
  } else if (strcasecmp(cmd, "cal") == 0) {
    cmdCal(arg1, arg2, arg3);
  } else if (strcasecmp(cmd, "help") == 0) {
    printHelp();
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

/**
 * POST Test 1: ADC Communication
 * Reads the DEV_ID register and verifies SPI communication works.
 * Also verifies CONFIG registers can be written and read back.
 */
bool postTestAdcComm() {
  Serial.print("POST: ADC Comm... ");

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

  // Read control register (R/W=1, ADDR=010)
  // Frame: 1_010_00000000000000000 = 0x500000
  SPI.beginTransaction(dacSpiSettings);
  digitalWrite(PIN_CS_DAC, LOW);
  SPI.transfer(0x50);  // R/W=1, ADDR=010, upper data bits
  SPI.transfer(0x00);
  SPI.transfer(0x00);
  digitalWrite(PIN_CS_DAC, HIGH);
  SPI.endTransaction();

  delayMicroseconds(5);

  // Now clock out the response
  SPI.beginTransaction(dacSpiSettings);
  digitalWrite(PIN_CS_DAC, LOW);
  uint8_t b2 = SPI.transfer(0x00);  // NOP to clock out data
  uint8_t b1 = SPI.transfer(0x00);
  uint8_t b0 = SPI.transfer(0x00);
  digitalWrite(PIN_CS_DAC, HIGH);
  SPI.endTransaction();

  // After reset, control register should have known state
  // RBUF=1 (unity mode) is default after reset, so DB1=1
  // We configured it for 2x mode (RBUF=0), so expect 0x00 in control bits
  // Check that we got a valid response (not all 1s or all 0s in frame)
  uint32_t frame = ((uint32_t)b2 << 16) | ((uint32_t)b1 << 8) | b0;

  // Simple check: frame should not be 0xFFFFFF (open circuit) or stuck
  bool pass = (frame != 0xFFFFFF) && (frame != 0x000000 || b2 != 0xFF);

  if (pass) {
    Serial.print("PASS (ctrl=0x");
    Serial.print(frame, HEX);
    Serial.println(")");
  } else {
    Serial.print("FAIL (frame=0x");
    Serial.print(frame, HEX);
    Serial.println(")");
  }
  return pass;
}

/**
 * POST Test 3: DRDY Timing
 * Measures the DRDY pulse rate to verify ADC clock is running correctly.
 * Expected: ~3906 Hz (256 µs period) ± 10%
 */
bool postTestDrdyTiming() {
  Serial.print("POST: DRDY Timing... ");

  // Wait for DRDY to go high first
  elapsedMicros timeout = 0;
  while (digitalReadFast(PIN_ADC_DRDY) == LOW && timeout < 1000) {}

  // Measure time for 10 DRDY pulses
  const int numPulses = 10;
  elapsedMicros elapsed = 0;

  for (int i = 0; i < numPulses; i++) {
    // Wait for falling edge (data ready)
    timeout = 0;
    while (digitalReadFast(PIN_ADC_DRDY) == HIGH && timeout < 1000) {}
    if (timeout >= 1000) {
      Serial.println("FAIL (DRDY timeout high)");
      return false;
    }

    // Wait for rising edge
    timeout = 0;
    while (digitalReadFast(PIN_ADC_DRDY) == LOW && timeout < 1000) {}
    if (timeout >= 1000) {
      Serial.println("FAIL (DRDY timeout low)");
      return false;
    }
  }

  uint32_t totalUs = elapsed;
  float periodUs = (float)totalUs / numPulses;
  float freqHz = 1e6f / periodUs;

  // Expected: ~256 µs period (~3906 Hz), allow ±10%
  bool pass = (periodUs > 230) && (periodUs < 282);

  if (pass) {
    Serial.print("PASS (");
    Serial.print(freqHz, 1);
    Serial.println(" Hz)");
  } else {
    Serial.print("FAIL (");
    Serial.print(freqHz, 1);
    Serial.print(" Hz, expected ~3906 Hz)");
    Serial.println();
  }
  return pass;
}

/**
 * POST Test 4: Signal Chain (DAC to ADC)
 * Sets DAC to 0V and verifies ADC reads near zero.
 * This tests the entire analog signal path.
 */
bool postTestSignalChain() {
  Serial.print("POST: Signal Chain... ");

  // Set DAC to 0V (code 0x0000 in two's complement mode)
  dac.setCode(0);  // Includes 250ms filter settling delay

  // Discard first sample, read a few and average
  int32_t s;
  if (!adc.readSample24(s)) { Serial.println("FAIL (ADC timeout)"); return false; }
  int64_t sum = 0;
  const int numSamples = 5;
  for (int i = 0; i < numSamples; i++) {
    if (!adc.readSample24(s)) { Serial.println("FAIL (ADC timeout)"); return false; }
    sum += s;
  }
  int32_t avg = sum / numSamples;

  // With DAC at 0V and Vx presumably near 0V (or within range),
  // the ADC should not be in overflow. Allow up to 50% of full scale.
  const int32_t threshold = 4'000'000;  // ~50% of 8,388,607
  bool pass = (avg > -threshold) && (avg < threshold);

  if (pass) {
    Serial.print("PASS (ADC=");
    Serial.print(avg);
    Serial.println(")");
  } else {
    Serial.print("FAIL (ADC=");
    Serial.print(avg);
    Serial.print(", expected < ±");
    Serial.print(threshold);
    Serial.println(")");
  }
  return pass;
}

/**
 * POST Test 5: Polarity Check
 * Verifies that changing DAC in positive direction causes expected ADC change.
 * With TMUXSEL=LOW, increasing DAC should decrease ADC reading
 * because ADC measures (Vx - DAC) * gain.
 *
 * Hardware signal routing (from schematic):
 *   MUXSEL=0 (LOW):  DAC→pin1(V-), Vx→pin4(V+) → output = (Vx - DAC) × gain
 *   MUXSEL=1 (HIGH): DAC→pin4(V+), Vx→pin1(V-) → output = (DAC - Vx) × gain
 */
bool postTestPolarity() {
  Serial.print("POST: Polarity... ");

  // Ensure TMUXSEL=LOW so ADC reads (Vx - DAC) * gain
  if (chopper.state()) {
    chopper.toggle();
  }

  // Set DAC to 0, measure
  dac.setCode(0);  // Includes 250ms filter settling delay
  int32_t adcAt0, adcAt1000, discard;
  if (!adc.readSample24(discard) || !adc.readSample24(adcAt0)) {
    Serial.println("FAIL (ADC timeout)"); return false;
  }

  // Set DAC to +1000 (small positive step), measure
  dac.setCode(1000);  // Includes 250ms filter settling delay
  if (!adc.readSample24(discard) || !adc.readSample24(adcAt1000)) {
    Serial.println("FAIL (ADC timeout)"); return false;
  }

  // Restore DAC
  dac.setCode(0);

  // With TMUXSEL=LOW: ADC = (Vx - DAC) * gain
  // If we increase DAC (and Vx stays same), ADC should decrease
  int32_t delta = adcAt1000 - adcAt0;

  // Expect negative delta (ADC decreased when DAC increased)
  // Allow some tolerance for noise, but delta should be significantly negative
  bool pass = (delta < -10000);  // Expect large negative change

  if (pass) {
    Serial.print("PASS (delta=");
    Serial.print(delta);
    Serial.println(")");
  } else {
    Serial.print("FAIL (delta=");
    Serial.print(delta);
    Serial.println(", expected negative)");
  }
  return pass;
}

/**
 * POST Test 6: Chopping Symmetry
 * Verifies that the two chopping phases produce opposite polarity readings.
 *
 * Hardware signal routing (from schematic):
 *   MUXSEL=0 (LOW):  DAC→pin1(V-), Vx→pin4(V+) → output = (Vx - DAC) × gain
 *   MUXSEL=1 (HIGH): DAC→pin4(V+), Vx→pin1(V-) → output = (DAC - Vx) × gain
 *
 * These should be approximately equal and opposite.
 */
bool postTestChopSymmetry() {
  Serial.print("POST: Chop Symmetry... ");

  // Set DAC to a known value
  dac.setCode(0);  // Includes 250ms filter settling delay

  // Ensure TMUXSEL=LOW, measure (Vx - DAC) * gain
  if (chopper.state()) {
    chopper.toggle();
  }
  delayMicroseconds(SETTLE_US);  // Mux settling (300µs)
  int32_t phase1, phase2, discard;
  if (!adc.readSample24(discard) || !adc.readSample24(phase1)) {
    Serial.println("FAIL (ADC timeout)"); return false;
  }

  // Switch to TMUXSEL=HIGH, measure (DAC - Vx) * gain
  chopper.toggle();
  delayMicroseconds(SETTLE_US);  // Mux settling (300µs)
  if (!adc.readSample24(discard) || !adc.readSample24(phase2)) {
    Serial.println("FAIL (ADC timeout)"); return false;
  }

  // Switch back
  chopper.toggle();

  // Check symmetry: phase1 ≈ -phase2
  // Sum should be near zero (they cancel)
  int32_t sum = phase1 + phase2;
  int32_t magnitude = (abs(phase1) + abs(phase2)) / 2;

  // Allow residual to be < 10% of magnitude (accounts for offset/noise)
  // But also handle case where magnitude is small
  bool pass;
  if (magnitude < 1000) {
    // Low signal - just check sum is small
    pass = (abs(sum) < 5000);
  } else {
    // Check ratio
    float ratio = (float)abs(sum) / (float)magnitude;
    pass = (ratio < 0.10f);  // Less than 10% residual
  }

  if (pass) {
    Serial.print("PASS (P1=");
    Serial.print(phase1);
    Serial.print(", P2=");
    Serial.print(phase2);
    Serial.print(", sum=");
    Serial.print(sum);
    Serial.println(")");
  } else {
    Serial.print("FAIL (P1=");
    Serial.print(phase1);
    Serial.print(", P2=");
    Serial.print(phase2);
    Serial.print(", sum=");
    Serial.print(sum);
    Serial.println(")");
  }
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

  // Switch to GND
  inputMux.select(InputChannel::GND);

  // Set DAC to 0V for this test
  dac.setCode(0);

  // Measure
  int32_t s;
  if (!adc.readSample24(s)) { Serial.println("FAIL (ADC timeout)"); return false; }
  int64_t sum = 0;
  const int numSamples = 5;
  for (int i = 0; i < numSamples; i++) {
    if (!adc.readSample24(s)) { Serial.println("FAIL (ADC timeout)"); return false; }
    sum += s;
  }
  int32_t avg = sum / numSamples;

  // guard restores channel + DAC on scope exit

  // With GND selected and DAC at 0V, ADC should read near zero
  // Allow for preamp offset (chopping not running during this test)
  const int32_t threshold = 2'000'000;  // ~25% of full scale (accounts for offset)
  bool pass = (avg > -threshold) && (avg < threshold);

  if (pass) {
    Serial.print("PASS (GND reading=");
    Serial.print(avg);
    Serial.println(")");
  } else {
    Serial.print("FAIL (GND reading=");
    Serial.print(avg);
    Serial.print(", expected near 0)");
    Serial.println();
  }
  return pass;
}

/**
 * POST Test 8: Reference Verification
 * Measures VrefRaw (pre-filter ADR1001 average at J3) and verifies it is
 * close to the expected 5V, within the ADC's measurement range.
 * This catches a failed reference or signal path issue.
 */
bool postTestReferences() {
  Serial.print("POST: Reference... ");
  ScopedInstrumentState guard;

  // Set DAC to ~5V to match the expected reference voltage
  const int16_t dacAt5V = 32764;  // Near +5V, 14-bit aligned
  dac.setCode(dacAt5V);

  // Measure the raw reference average (J3)
  inputMux.select(InputChannel::VrefRaw);
  delayMicroseconds(SETTLE_US);

  int32_t s;
  if (!adc.readSample24(s)) { Serial.println("FAIL (ADC timeout)"); return false; }
  int64_t sum = 0;
  const int numSamples = 5;
  for (int i = 0; i < numSamples; i++) {
    if (!adc.readSample24(s)) { Serial.println("FAIL (ADC timeout)"); return false; }
    sum += s;
  }
  int32_t reading = sum / numSamples;

  // guard restores channel + DAC on scope exit

  // With DAC at ~5V and VrefRaw at ~5V, the ADC should read near zero
  // (the difference × 2000 gain). Allow up to 25% of full scale for
  // filter settling, temperature differences, etc.
  const int32_t refThreshold = 2'000'000;  // ~25% of 8,388,607

  bool pass = (reading > -refThreshold) && (reading < refThreshold);

  if (pass) {
    Serial.print("PASS (VrefRaw=");
    Serial.print(reading);
    Serial.println(")");
  } else {
    Serial.print("FAIL (VrefRaw=");
    Serial.print(reading);
    Serial.print(", expected near 0, threshold=±");
    Serial.print(refThreshold);
    Serial.println(")");
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
  dac.setCode(0);

  // Accumulate chopped measurements for accurate zero reading
  LowerMoments zeroStats;
  for (int i = 0; i < 10; i++) {
    if (!runOneChopCycle(zeroStats, /*searchOnOverflow=*/false)) {
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
  result.signal_chain = postTestSignalChain();
  result.polarity = postTestPolarity();
  result.chop_symmetry = postTestChopSymmetry();
  result.input_mux = postTestInputMux();
  result.references = postTestReferences();
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

  for (int i = 0; i < REF_MEASURE_ITERATIONS; i++) {
    // searchOnOverflow=false: DAC is deliberately set to ~5V; let caller handle unexpected overflow
    if (!runOneChopCycle(refStats, /*searchOnOverflow=*/false)) {
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

    Serial.print("Vx = ");
    Serial.print(vxBuf);
    Serial.print(" +/- ");
    Serial.print(uncBuf);
    Serial.print(" (n=");
    Serial.print(data.count, 0);
    Serial.print(", DAC=");
    Serial.print(data.dacCode);
    Serial.print(", range=");
    Serial.print(getInputChannelName(data.channel));
    Serial.println(")");

    Serial.print("  raw: mean=");
    Serial.print(data.adcMean, 3);
    Serial.print(", sd=");
    Serial.print(data.adcStdDev, 3);
    Serial.print(", min=");
    Serial.print(data.adcMin, 3);
    Serial.print(", max=");
    Serial.println(data.adcMax, 3);

    double filterErr = predictFilterError();
    Serial.print("  drift: filterErr=");
    Serial.print(filterErr * 1e9, 1);
    Serial.print(" nV, driftRate=");
    Serial.print(estimatedDriftRate * 1e9, 3);
    Serial.print(" nV/s, correction=");
    Serial.print(data.driftPpb, 3);
    Serial.println(" ppb");

    if (scanner.autoZeroValid) {
      Serial.print("  zero: offset=");
      Serial.print(scanner.autoZeroOffset * 1e9, 1);
      Serial.println(" nV");
    }

    if (allanDev.readingCount() >= 3) {
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

  // Accumulate chopped measurements
  LowerMoments gndStats;
  for (int i = 0; i < AUTOZERO_CYCLES; i++) {
    if (!runOneChopCycle(gndStats, /*searchOnOverflow=*/false)) {
      Serial.println("WARNING: Overflow during auto-zero!");
      return;  // guard restores state
    }
  }

  // Compute zero offset
  scanner.autoZeroOffset = computeInputVoltage(gndStats.mean(), dac.currentCode(), InputChannel::GND);
  scanner.autoZeroValid = true;
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

static constexpr uint32_t EEPROM_MAGIC = 0xD1FF0001UL;  // "DIFF" version 1
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
static constexpr uint32_t CAL_TABLE_MAGIC  = 0xDACC0001UL;  // "DACC" v1
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

  // Write preampGain + divider ratios to /cal_consts.bin
  // Format: [4] magic [2] version [8×4=32] four doubles [2] CRC16 = 40 bytes
  bool saveConstants() {
    if (!mounted_) return false;
    File f = fs_.open("/cal_consts.bin", FILE_WRITE);
    if (!f) return false;

    uint8_t buf[40];
    size_t pos = 0;

    uint32_t magic = CAL_CONSTS_MAGIC;
    memcpy(buf + pos, &magic, 4); pos += 4;
    uint16_t ver = 1;
    memcpy(buf + pos, &ver, 2); pos += 2;
    memcpy(buf + pos, &PREAMP_GAIN, 8); pos += 8;
    memcpy(buf + pos, &DIVIDER_RATIO_10, 8); pos += 8;
    memcpy(buf + pos, &DIVIDER_RATIO_100, 8); pos += 8;
    memcpy(buf + pos, &DIVIDER_RATIO_1000, 8); pos += 8;
    uint16_t crc = crc16(buf, pos);
    memcpy(buf + pos, &crc, 2); pos += 2;

    f.write(buf, pos);
    f.close();
    return true;
  }

  // Read constants from /cal_consts.bin and apply them if valid.
  bool loadConstants() {
    if (!mounted_) return false;
    File f = fs_.open("/cal_consts.bin", FILE_READ);
    if (!f) return false;

    uint8_t buf[40];
    size_t n = f.read(buf, sizeof(buf));
    f.close();
    if (n < 40) return false;

    uint32_t magic; memcpy(&magic, buf, 4);
    if (magic != CAL_CONSTS_MAGIC) return false;

    uint16_t storedCrc; memcpy(&storedCrc, buf + 38, 2);
    if (crc16(buf, 38) != storedCrc) return false;

    double pg, d10, d100, d1000;
    memcpy(&pg,    buf + 6,  8);
    memcpy(&d10,   buf + 14, 8);
    memcpy(&d100,  buf + 22, 8);
    memcpy(&d1000, buf + 30, 8);

    // Sanity bounds: refuse obviously wrong values
    if (pg < 100.0 || pg > 100000.0) return false;
    if (d10 < 1.0  || d10 > 100.0)   return false;
    if (d100 < 10.0 || d100 > 10000.0) return false;
    if (d1000 < 100.0 || d1000 > 100000.0) return false;

    PREAMP_GAIN       = pg;
    DIVIDER_RATIO_10  = d10;
    DIVIDER_RATIO_100 = d100;
    DIVIDER_RATIO_1000 = d1000;
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

  // Remove both calibration files from flash.
  bool eraseAll() {
    if (!mounted_) return false;
    bool ok = true;
    if (fs_.exists("/cal_consts.bin")) ok &= fs_.remove("/cal_consts.bin");
    if (fs_.exists("/dac_table.bin"))  ok &= fs_.remove("/dac_table.bin");
    return ok;
  }

  // Load constants and table from flash. Called from setup().
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
    Serial.print("  /cal_consts.bin: ");
    Serial.println(fs_.exists("/cal_consts.bin") ? "present" : "absent");
    Serial.print("  /dac_table.bin:  ");
    Serial.println(fs_.exists("/dac_table.bin") ? "present" : "absent");
    Serial.print("  PREAMP_GAIN:      "); Serial.println(PREAMP_GAIN, 6);
    Serial.print("  DIVIDER_RATIO_10: "); Serial.println(DIVIDER_RATIO_10, 6);
    Serial.print("  DIVIDER_RATIO_100: "); Serial.println(DIVIDER_RATIO_100, 6);
    Serial.print("  DIVIDER_RATIO_1000: "); Serial.println(DIVIDER_RATIO_1000, 6);
    Serial.print("  DAC cal table:    ");
    Serial.println(dacCalTable.isValid() ? "CALIBRATED" : "nominal");
  }

private:
  LittleFS_Program fs_;
  bool mounted_ = false;
};

static CalibrationStore calStore;

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
  int16_t bsCode = dac.currentCode();
  Serial.print("  Converged at code "); Serial.println(bsCode);

  int entriesGnd = 0;
  for (int delta = -CAL_BUILD_WINDOW; delta <= CAL_BUILD_WINDOW; delta++) {
    int32_t code32 = (int32_t)bsCode + (int32_t)(delta * 4);
    if (code32 < -32768) code32 = -32768;
    if (code32 >  32764) code32 =  32764;
    int16_t code = (int16_t)code32;

    dac.setCode(code);

    LowerMoments stats;
    bool ok = true;
    for (int i = 0; i < CAL_BUILD_CYCLES; i++) {
      if (!runOneChopCycle(stats, /*searchOnOverflow=*/false)) { ok = false; break; }
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
  bsCode = dac.currentCode();
  Serial.print("  Converged at code "); Serial.println(bsCode);

  int entriesVref = 0;
  for (int delta = -CAL_BUILD_WINDOW; delta <= CAL_BUILD_WINDOW; delta++) {
    int32_t code32 = (int32_t)bsCode + (int32_t)(delta * 4);
    if (code32 < -32768) code32 = -32768;
    if (code32 >  32764) code32 =  32764;
    int16_t code = (int16_t)code32;

    dac.setCode(code);

    LowerMoments stats;
    bool ok = true;
    for (int i = 0; i < CAL_BUILD_CYCLES; i++) {
      if (!runOneChopCycle(stats, /*searchOnOverflow=*/false)) { ok = false; break; }
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

  // Select Vx channel (user applies the known voltage here)
  inputMux.select(InputChannel::Vx);

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
  for (int i = 0; i < CAL_POINT_CYCLES; i++) {
    if (!runOneChopCycle(stats, /*searchOnOverflow=*/false)) {
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

/**
 * Handle 'cal' command and subcommands.
 * Defined here (after CalibrationStore) so calStore is fully typed.
 */
void cmdCal(const char* arg1, const char* arg2, const char* arg3) {
  if (!arg1) {
    Serial.println("Usage: cal status|save|load|erase|set|factory|build|point");
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
    } else {
      Serial.println("Unknown cal set parameter. Use: gain, div10, div100, div1000");
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

  } else {
    Serial.println("Unknown cal subcommand. Use: status, save, load, erase, set, factory, build, point");
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
  scanner.config.channels[0] = InputChannel::Vx;
  scanner.config.count = 1;
  scanner.config.integrationCycles = 100;
  scanner.config.autoZeroEnabled = true;
  scanner.config.autoZeroInterval = 10;
  scanner.config.outputMode = OutputMode::Human;
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

  double vxRaw = computeInputVoltage(adcMean, dacCode, channel);
  double vxUncertainty = computeInputUncertainty(adcStdDev, channel);

  double refCorrection = getRefCorrectionFactor();
  double vxCorrected = vxRaw * refCorrection;
  if (scanner.autoZeroValid) {
    vxCorrected -= scanner.autoZeroOffset;
  }

  double driftPpb = (refCorrection - 1.0) * 1e9;

  allanDev.addReading(vxCorrected);

  MeasurementData data{channel, vxCorrected, vxUncertainty,
                       adcMean, adcStdDev, stats.count(),
                       dacCode, driftPpb, stats.getMin(), stats.getMax()};

  IOutputFormatter& fmt = getFormatter();
  fmt.beginSweep();
  fmt.formatMeasurement(data);
  fmt.endSweep();
}

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 2000) {}

  // Configure ADC pins
  pinMode(PIN_CS_ADC,     OUTPUT);
  csHigh(PIN_CS_ADC);
  pinMode(PIN_ADC_DRDY,   INPUT);

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

  // Initialize SPI before using it
  SPI.begin();

  // Initialize DAC (SPI must be ready first)
  dac.begin();

  Serial.println("Starting ADC init...");
  adc.initAndConfigure();

  // Enable the mux
  digitalWriteFast(PIN_TMUX_EN, HIGH);

  Serial.print("Estimated fSPS = ");
  Serial.print(EST_FSPS, 2);
  Serial.print(" SPS, Ts = ");
  Serial.print(EST_TS_US, 1);
  Serial.println(" us");

  // Run Power-On Self-Test
  PostResult postResult = runPOST();
  
  if (!postResult.allPassed()) {
    pinMode(LED_BUILTIN, OUTPUT);
    if (POST_HALT_ON_FAIL) {
      Serial.println("POST failed - system halted.");
      Serial.println("Set POST_HALT_ON_FAIL=false to continue despite failures.");
      // Give config information

      while (true) {
        // Blink LED to indicate failure 
        digitalWrite(LED_BUILTIN, HIGH);
        delay(1500);
        digitalWrite(LED_BUILTIN, LOW);
        delay(1000);
      }
    } else {
      Serial.println("POST failed - continuing with warnings.");
    }
  }

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
 * @param stats  LowerMoments accumulator to add the demodulated sample to.
 *               In ONE_CHANNEL mode this is chopStats; in SCANNING mode it's
 *               channelStats[currentChannel].
 * @return       true if measurement succeeded, false if overflow occurred
 *               (caller should skip output and retry on next loop iteration)
 */
// @param searchOnOverflow  If true (default), triggers binarySearchDAC() + stats.clear() on
//                          overflow — correct for continuous measurement in loop().
//                          If false, returns false immediately without touching the DAC —
//                          correct for calibration routines that manage their own DAC state.
bool runOneChopCycle(LowerMoments &stats, bool searchOnOverflow) {
  HalfCycleResult resultA = acquireHalfCycle();

  if (resultA.overflow) {
    if (searchOnOverflow) { binarySearchDAC(); stats.clear(); }
    return false;
  }

  chopper.toggle();

  HalfCycleResult resultB = acquireHalfCycle();

  if (resultB.overflow) {
    chopper.toggle();  // Return to original state
    if (searchOnOverflow) { binarySearchDAC(); stats.clear(); }
    return false;
  }

  chopper.toggle();

  double demodulated = (double)(resultA.sum - resultB.sum) / (2.0 * GOOD_SAMPLES);
  stats.accumulate(demodulated);
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
 *     - Measures currentInputChannel continuously using chopStats
 *     - Outputs a reading every integrationCycles via outputMeasurement()
 *     - Stats accumulate indefinitely (not cleared between outputs)
 *
 *   SCANNING mode:
 *     - Cycles through scanner.config.channels[0..count-1]
 *     - For each channel:
 *         a. Switch mux + binarySearchDAC() (on channel change)
 *         b. Discard SCAN_SETTLE_CYCLES chop cycles for settling
 *         c. Accumulate integrationCycles into channelStats[channel]
 *         d. Output result via outputMeasurement() (or plotter assembly)
 *         e. Advance to next channel
 *     - After a full sweep: optionally run performAutoZero()
 *     - Multi-channel Plotter output is assembled here (not in outputMeasurement)
 *       because all channels must appear on one tab-separated line
 */
void loop() {
  // Process serial commands
  processSerialCommands();

  // Periodically measure reference filter error for drift compensation
  if (++refSampleCounter >= REF_SAMPLE_INTERVAL) {
    refSampleCounter = 0;
    measureFilterError();
  }

  // ---- ONE_CHANNEL MODE: measure current channel continuously ----
  if (scanner.isOneChannel()) {
    if (!runOneChopCycle(chopStats)) {
      return;  // Overflow handled, restart
    }

    // Output every integrationCycles, then clear for fixed-window integration
    static int loopCounter = 0;
    if (++loopCounter >= scanner.config.integrationCycles) {
      loopCounter = 0;
      outputMeasurement(inputMux.current(), chopStats, dac.currentCode());
      chopStats.clear();

      // Auto-zero: same interval logic as SCANNING mode (counts readings, not sweeps)
      scanner.incScanCycleCount();
      if (scanner.config.autoZeroEnabled &&
          (scanner.scanCycleCount() % scanner.config.autoZeroInterval) == 0) {
        performAutoZero();
        if (scanner.autoZeroValid && scanner.config.outputMode == OutputMode::Human) {
          Serial.print("  [auto-zero: offset=");
          Serial.print(scanner.autoZeroOffset * 1e9, 1);
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
  if (!runOneChopCycle(stats)) {
    scanner.setChopCycleCount(0);  // Reset after overflow/DAC change
    // Save the re-centered DAC code for this channel
    scanner.setChannelDac((uint8_t)scanCh, dac.currentCode());
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

      MeasurementData data{scanCh, vxCorrected, vxUncertainty,
                           adcMean, adcStdDev, stats.count(),
                           dac.currentCode(), driftPpb,
                           stats.getMin(), stats.getMax()};

      IOutputFormatter& fmt = getFormatter();
      if (scanner.currentScanIndex() == 0) fmt.beginSweep();
      fmt.formatMeasurement(data);
      if (scanner.currentScanIndex() == scanner.config.count - 1) fmt.endSweep();
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
      scanner.incScanCycleCount();

      // Check if auto-zero is due
      if (scanner.config.autoZeroEnabled &&
          (scanner.scanCycleCount() % scanner.config.autoZeroInterval) == 0) {
        performAutoZero();
        if (scanner.autoZeroValid && scanner.config.outputMode == OutputMode::Human) {
          Serial.print("  [auto-zero: offset=");
          Serial.print(scanner.autoZeroOffset * 1e9, 1);
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
