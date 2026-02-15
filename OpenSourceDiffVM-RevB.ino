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
   - Mux cycle = 400 Hz => 1.25 ms high, 1.25 ms low
*/

#include <Arduino.h>
#include <SPI.h>
#include <EEPROM.h>

// ---------------- Forward Declarations ----------------
// Types must be declared before Arduino auto-generates function prototypes
// (prototypes are inserted after the last #include, which is <limits> below)
struct HalfCycleResult {
  int64_t sum;
  bool overflow;
};
class LowerMoments;
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

// Track current input selection
static InputChannel currentInputChannel = InputChannel::Vx;

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

// Track current divider ratio selection
static DividerRatio currentDividerRatio = DividerRatio::Div10;

// ---------------- Voltage Divider Ratios ----------------
// Calculated ratios for Caddock 1776-C4815 serial-parallel divider
// Segments: 10M, 1.1111M, 101.01K, 10.01K (4 dividers as 2 parallel pairs in series)
// MUX path = Rseg/2 + Rseg/2 + 250Ω; base 1M arm = 1.1111M (always in circuit)
static constexpr double DIVIDER_RATIO_10   = 10.0;      // ±50V: mux bypassed
static constexpr double DIVIDER_RATIO_100  = 108.76;    // ±500V: 1.1111M || 101.26K
static constexpr double DIVIDER_RATIO_1000 = 984.65;    // ±5kV: 1.1111M || 10.26K

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
static constexpr double PREAMP_GAIN     = 2000.0;

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

// ---------------- DAC Calibration Table ----------------
// 16384-point calibration table for DAC INL correction
// Index: (dacCode + 32768) >> 2 gives 14-bit index (0..16383)
// Value: actual voltage at that code (measured against ADR1001 references)
// If not calibrated, table contains nominal values
static constexpr size_t DAC_CAL_TABLE_SIZE = 16384;
static double dacCalTable[DAC_CAL_TABLE_SIZE];  // Populated at calibration time
static bool dacCalTableValid = false;           // True after calibration

// Initialize DAC calibration table with nominal (uncalibrated) values
void initDacCalTable() {
  for (size_t i = 0; i < DAC_CAL_TABLE_SIZE; i++) {
    // Convert table index back to DAC code
    int16_t code = (int16_t)((i << 2) - 32768);
    // Nominal voltage = code × LSB
    dacCalTable[i] = (double)code * DAC_LSB_V;
  }
  dacCalTableValid = false;  // Mark as uncalibrated (using nominal values)
}

// Look up calibrated DAC voltage for a given code
// Uses linear interpolation between calibration points
double dacCodeToVoltage(int16_t code) {
  if (!dacCalTableValid) {
    // No calibration: use nominal linear conversion
    return (double)code * DAC_LSB_V;
  }

  // Convert code to table index (14-bit, 0..16383)
  uint16_t idx = ((uint16_t)(code + 32768)) >> 2;

  // Clamp to valid range
  if (idx >= DAC_CAL_TABLE_SIZE - 1) {
    return dacCalTable[DAC_CAL_TABLE_SIZE - 1];
  }

  // Linear interpolation between adjacent calibration points
  // The fractional part comes from the 2 LSBs we shifted away
  uint8_t frac = ((uint16_t)(code + 32768)) & 0x03;  // 0..3
  double v0 = dacCalTable[idx];
  double v1 = dacCalTable[idx + 1];
  return v0 + (v1 - v0) * (double)frac / 4.0;
}

// Store a calibration point (called during DAC calibration procedure)
void setDacCalPoint(int16_t code, double measuredVoltage) {
  uint16_t idx = ((uint16_t)(code + 32768)) >> 2;
  if (idx < DAC_CAL_TABLE_SIZE) {
    dacCalTable[idx] = measuredVoltage;
  }
}

// Mark calibration table as valid after full calibration
void markDacCalValid() {
  dacCalTableValid = true;
}

// ---------------- Voltage Divider Scale Factors ----------------
// Returns the divider ratio for the current input channel
// For Vx and references, ratio is 1.0 (no division)
double getInputDividerRatio(InputChannel channel) {
  // For HV divider channel, ratio depends on MUX36D04 address state
  if (channel == InputChannel::HVDivider) {
    switch (currentDividerRatio) {
      case DividerRatio::Div10:   return DIVIDER_RATIO_10;    // ÷10
      case DividerRatio::Div100:  return DIVIDER_RATIO_100;   // ÷100
      case DividerRatio::Div1000: return DIVIDER_RATIO_1000;  // ÷1000
      case DividerRatio::GND:     return DIVIDER_RATIO_10;    // GND cal uses base ÷10
      default:                    return DIVIDER_RATIO_10;    // Default to ÷10
    }
  }
  return 1.0;  // No division for other channels
}

// ---------------- MUX36D04 HV Divider Tap Control ----------------

/**
 * Initialize MUX36D04 address pins.
 * EN is hardwired to VDD (active high, always enabled).
 * Call this in setup() before using selectDividerRatio().
 */
void initDividerControl() {
  pinMode(PIN_DIVMUX_A0, OUTPUT);
  pinMode(PIN_DIVMUX_A1, OUTPUT);

  // Default to Div10 (address 0) - mux bypassed, only hardwired 1:10 divider
  digitalWrite(PIN_DIVMUX_A0, LOW);
  digitalWrite(PIN_DIVMUX_A1, LOW);

  currentDividerRatio = DividerRatio::Div10;
}

/**
 * Select HV divider ratio via MUX36D04 address lines.
 * The 1:10 divider is always in circuit (hardwired for safety).
 * Higher ratios bridge additional parallel resistance via the mux.
 * Also switches MUX36D08 to HVDivider channel.
 *
 * Address mapping:
 *   0 (A1=0, A0=0): Mux bypassed → ÷10 only (hardwired)
 *   1 (A1=0, A0=1): CD4-AB4 bridged → ÷100 (100k || 1M)
 *   2 (A1=1, A0=0): CD5-AB5 bridged → ÷1000 (10k || 1M)
 *   3 (A1=1, A0=1): Ground connection → zero calibration
 *
 * @param ratio The desired divider ratio
 */
void selectDividerRatio(DividerRatio ratio) {
  uint8_t addr = static_cast<uint8_t>(ratio);  // Enum values match addresses directly

  digitalWriteFast(PIN_DIVMUX_A0, (addr & 0x01) ? HIGH : LOW);
  digitalWriteFast(PIN_DIVMUX_A1, (addr & 0x02) ? HIGH : LOW);

  currentDividerRatio = ratio;

  // Switch MUX36D08 to HVDivider channel (always, since divider is always in circuit)
  selectInputChannel(InputChannel::HVDivider);

  // Allow MUX36D04 and signal path to settle
  delayMicroseconds(INMUX_SETTLE_US);
}

/**
 * Get current divider ratio setting.
 */
DividerRatio getDividerRatio() {
  return currentDividerRatio;
}

/**
 * Get human-readable name for a divider ratio.
 */
const char* getDividerRatioName(DividerRatio ratio) {
  switch (ratio) {
    case DividerRatio::Div10:   return "÷10 (±50V)";
    case DividerRatio::Div100:  return "÷100 (±500V)";
    case DividerRatio::Div1000: return "÷1000 (±5kV)";
    case DividerRatio::GND:     return "GND (cal)";
    default:                    return "Unknown";
  }
}

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
  double dacVoltage = dacCodeToVoltage(dacCode);

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

// ---------------- DAC State ----------------
// 16-bit DAC in two's complement mode: 0x0000 = 0V, 0x7FFF = +full scale, 0x8000 = -full scale
static int16_t currentDacCode = 0;  // Track current DAC setting

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

// chopping value
volatile bool tmux_sel_state = LOW;

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

// -------- DAC (DAC) register addresses (3-bit) --------
static constexpr uint8_t DAC_ADDRESS       = 0b001;
static constexpr uint8_t DAC_ADDR_CTRL     = 0b010;
static constexpr uint8_t DAC_ADDR_CLEAR    = 0b011;
static constexpr uint8_t DAC_ADDR_SOFTCTRL = 0b100;

// Low-level: write a 24-bit frame
static void DAC_write24(uint32_t frame)
{
  SPI.beginTransaction(dacSpiSettings);
  digitalWrite(PIN_CS_DAC, LOW);
  SPI.transfer((frame >> 16) & 0xFF);
  SPI.transfer((frame >>  8) & 0xFF);
  SPI.transfer((frame >>  0) & 0xFF);
  digitalWrite(PIN_CS_DAC, HIGH);
  SPI.endTransaction();
}

// DAC register + Clearcode register use 16-bit data in DB19..DB4 (lower 4 bits don't care)
static void DAC_writeCode(uint16_t code)
{
  const uint32_t rw   = 0u;
  const uint32_t addr = DAC_ADDRESS;
  const uint32_t data = (uint32_t)code << 4;           // aligns into DB19..DB4
  const uint32_t frame = (rw << 23) | (addr << 20) | (data & 0xFFFFF);
  DAC_write24(frame);
}

static void DAC_writeClearCode(uint16_t code)
{
  const uint32_t rw   = 0u;
  const uint32_t addr = DAC_ADDR_CLEAR;
  const uint32_t data = (uint32_t)code << 4;           // aligns into DB19..DB4
  const uint32_t frame = (rw << 23) | (addr << 20) | (data & 0xFFFFF);
  DAC_write24(frame);
}

// Control register uses named bits down in DB5..DB1 (no <<4 here)
static void DAC_writeControl(
  bool sdoDisable,     // SDODIS (DB5)
  bool offsetBinary,   // BIN/2sC (DB4) : 0=two's complement, 1=offset binary
  bool dacTriState,    // DACTRI (DB3)
  bool opGndClamp,     // OPGND  (DB2)
  bool rbufUnityMode   // RBUF   (DB1) : 0 = gain-of-two config mode, 1 = unity mode (default)
)
{
  uint32_t data = 0;
  data |= (uint32_t)(sdoDisable   ? 1 : 0) << 5;
  data |= (uint32_t)(offsetBinary ? 1 : 0) << 4;
  data |= (uint32_t)(dacTriState  ? 1 : 0) << 3;
  data |= (uint32_t)(opGndClamp   ? 1 : 0) << 2;
  data |= (uint32_t)(rbufUnityMode? 1 : 0) << 1;
  // DB0 reserved = 0; other reserved bits = 0

  const uint32_t rw   = 0u;
  const uint32_t addr = DAC_ADDR_CTRL;
  const uint32_t frame = (rw << 23) | (addr << 20) | (data & 0xFFFFF);
  DAC_write24(frame);
}

// Software control uses bits at DB2..DB0 (no <<4 here)
static void DAC_softCtrl(bool doReset, bool doClr, bool doLdac)
{
  uint32_t data = 0;
  data |= (uint32_t)(doReset ? 1 : 0) << 2;
  data |= (uint32_t)(doClr   ? 1 : 0) << 1;
  data |= (uint32_t)(doLdac  ? 1 : 0) << 0;

  const uint32_t rw   = 0u;
  const uint32_t addr = DAC_ADDR_SOFTCTRL;
  const uint32_t frame = (rw << 23) | (addr << 20) | (data & 0xFFFFF);
  DAC_write24(frame);
}

// Assumes: VREFP=5V, VREFN=0V, and you want "2x mode" => gain-of-two configuration (RBUF=0).
void initDAC_2xMode()
{
  // Optional: software reset (handy if RESET pin isn’t used)
  DAC_softCtrl(true, false, false);
  delayMicroseconds(10);

  // Program clearcode (choose what CLR should force the DAC to)
  // With two’s complement coding, code 0x0000 is a sensible “zero code”.
  DAC_writeClearCode(0x0000);

  // Preload a known DAC code before enabling output (keeps output deterministic)
  DAC_writeCode(0x0000);

  // Configure control register:
  // - SDODIS=0 (keep SDO enabled)
  // - BIN/2sC=0 (two’s complement)
  // - DACTRI=0 (normal)
  // - OPGND=0 (remove ground clamp)
  // - RBUF=0 (gain-of-two configuration mode)
  DAC_writeControl(
    /*sdoDisable=*/false,
    /*offsetBinary=*/false,
    /*dacTriState=*/false,
    /*opGndClamp=*/false,
    /*rbufUnityMode=*/false   // <-- 2x / gain-of-two config
  );

  // If your LDAC pin is tied HIGH, you can force an update via software LDAC pulse.
  // In our case LDAC is tied low so this is not necessary.
  //DAC_softCtrl(false, false, true);
}

// ================== Input Mux Control (MUX36S08) ==================

/**
 * Initialize input mux GPIO pins.
 * Call this in setup() before using selectInputChannel().
 */
void initInputMux() {
  pinMode(PIN_INMUX_A0, OUTPUT);
  pinMode(PIN_INMUX_A1, OUTPUT);
  pinMode(PIN_INMUX_A2, OUTPUT);
  // EN is hardwired to VDD (active high, always enabled)

  // Select Vx (address 0)
  digitalWrite(PIN_INMUX_A0, LOW);
  digitalWrite(PIN_INMUX_A1, LOW);
  digitalWrite(PIN_INMUX_A2, LOW);
  currentInputChannel = InputChannel::Vx;
}

/**
 * Select input channel on the MUX36S08.
 * Waits for mux settling before returning.
 *
 * @param channel The input channel to select
 */
void selectInputChannel(InputChannel channel) {
  if (channel == currentInputChannel) {
    return;  // Already selected, no action needed
  }

  uint8_t addr = static_cast<uint8_t>(channel);

  // Set address lines
  digitalWriteFast(PIN_INMUX_A0, (addr & 0x01) ? HIGH : LOW);
  digitalWriteFast(PIN_INMUX_A1, (addr & 0x02) ? HIGH : LOW);
  digitalWriteFast(PIN_INMUX_A2, (addr & 0x04) ? HIGH : LOW);

  currentInputChannel = channel;

  // Wait for mux and signal path to settle
  delayMicroseconds(INMUX_SETTLE_US);
}

/**
 * Get the current input channel.
 */
InputChannel getInputChannel() {
  return currentInputChannel;
}

/**
 * Get human-readable name for an input channel.
 */
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

// ================== DAC Functions ==================

/**
 * Calculate DAC filter settling time based on step size.
 * Uses sqrt scaling to maintain constant absolute error (~1 µV target).
 *
 * Formula: settle_ms = 80 + 220 × sqrt(stepSize / 32768)
 *
 * This gives:
 *   - 100 codes (15 mV):    ~92 ms
 *   - 1000 codes (153 mV):  ~118 ms
 *   - 10000 codes (1.5 V):  ~201 ms
 *   - 32768 codes (5 V):    ~300 ms
 */
uint32_t calculateDacSettleTime(int32_t stepSize) {
  if (stepSize <= 0) return 0;

  // Clamp to maximum step size
  if (stepSize > 32768) stepSize = 32768;

  // sqrt scaling: additional time proportional to sqrt(step_size)
  // Scale factor: (MAX - MIN) / sqrt(32768) ≈ 220 / 181 ≈ 1.215
  const float scale = (float)(DAC_SETTLE_MAX_MS - DAC_SETTLE_MIN_MS) / sqrtf(32768.0f);
  uint32_t additionalMs = (uint32_t)(scale * sqrtf((float)stepSize));

  return DAC_SETTLE_MIN_MS + additionalMs;
}

// Set DAC to a specific code, track it, and wait for filter to settle
void setDacCode(int16_t code) {
  if (code == currentDacCode) {
    return;  // No change, no settling needed
  }

  int32_t stepSize = abs((int32_t)code - (int32_t)currentDacCode);
  currentDacCode = code;
  DAC_writeCode((uint16_t)code);

  // Wait for Bessel filter to settle (time depends on step size)
  uint32_t settleMs = calculateDacSettleTime(stepSize);
  if (settleMs > 0) {
    delay(settleMs);
  }
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

// Read a single ADC sample for DAC adjustment (with settling)
int32_t readSettledSample() {
  delayMicroseconds(SETTLE_US);
  (void)adcReadSample24();  // discard one
  return adcReadSample24();
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
  // Use chop() to switch properly with CI_TMUXSEL sequence
  if (tmux_sel_state == HIGH) {
    chop();  // Toggle HIGH -> LOW using charge-injection-minimizing sequence
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
    setDacCode((int16_t)mid);  // Includes 250ms filter settling delay

    int32_t sample = readSettledSample();

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
      // In range - success
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

// End DAC code
uint8_t adcXfer(uint8_t v) { return SPI.transfer(v); }

void adcCommand(uint8_t cmd) {
  SPI.beginTransaction(adcSpiSettings);
  csLow(PIN_CS_ADC);
  adcXfer(cmd);
  csHigh(PIN_CS_ADC);
  SPI.endTransaction();
  delayMicroseconds(2);
}

void adcWriteReg(uint8_t addr, uint8_t value) {
  SPI.beginTransaction(adcSpiSettings);
  csLow(PIN_CS_ADC);
  adcXfer(OPCODE_WREG | (addr & 0x1F));
  adcXfer(0x00); // write 1 register (count-1)
  adcXfer(value);
  csHigh(PIN_CS_ADC);
  SPI.endTransaction();
  delayMicroseconds(2);
}

uint8_t adcReadReg(uint8_t addr) {
  SPI.beginTransaction(adcSpiSettings);
  csLow(PIN_CS_ADC);
  adcXfer(OPCODE_RREG | (addr & 0x1F));
  adcXfer(0x00); // read 1 register (count-1)
  uint8_t v = adcXfer(0x00);
  csHigh(PIN_CS_ADC);
  SPI.endTransaction();
  delayMicroseconds(2);
  return v;
}

// Blocking wait for DRDY active (assumes DRDY goes LOW when data ready).
// Note: elapsedMicros is a Teensy-specific class that automatically tracks
// elapsed time using hardware timers. It starts counting at initialization
// and increments in the background - no manual update needed.
bool waitForDrdyLow(uint32_t timeout_us) {
  elapsedMicros t = 0;  // Auto-incrementing timer, starts at 0
  while (t < timeout_us) {
    if (digitalReadFast(PIN_ADC_DRDY) == LOW) return true;
  }
  return false;
}

// Read one 24-bit sample frame.
int32_t adcReadSample24() {
  if (!waitForDrdyLow(1000)) {
    return INT32_MIN;
  }

  SPI.beginTransaction(adcSpiSettings);
  csLow(PIN_CS_ADC);

  uint8_t b2 = adcXfer(0x00);
  uint8_t b1 = adcXfer(0x00);
  uint8_t b0 = adcXfer(0x00);

  csHigh(PIN_CS_ADC);
  SPI.endTransaction();

  int32_t raw = (int32_t)((uint32_t)b2 << 16 | (uint32_t)b1 << 8 | (uint32_t)b0);

  // Sign-extend 24-bit to 32-bit
  if (raw & 0x00800000) raw |= 0xFF000000;
  return raw;
}

/**
 * @brief This method is called to switch (aka chop) the inputs to the instrumentation amplifier.
 *
 * It performs the core control logic:
 * 1. Pulses the CI_TMUXSEL pin HIGH.
 * 2. Toggles the state of the TMUXSEL pin.
 * 3. Pulses the CI_TMUXSEL pin LOW.
 * This sequence minimizes charge injection. Hopefully.
 */
void chop() {
    // --- Step 1 & 2: Pulse the CI_TMUXSEL pin ---
  // Use digitalWriteFast for minimum latency.
  digitalWriteFast(PIN_CI_TMUXSEL, HIGH);
  delayMicroseconds(PULSE_WIDTH_US); // This brief delay is acceptable in an ISR.
  // --- Step 3: Toggle the main switch select state ---
  tmux_sel_state = !tmux_sel_state; // Invert the state
  digitalWriteFast(PIN_TMUXSEL, tmux_sel_state);
  delayMicroseconds(PULSE_WIDTH_US);
  digitalWriteFast(PIN_CI_TMUXSEL, LOW);
}

// ---------------- ADC configuration ----------------
void adcInitAndConfigure() {
  adcCommand(CMD_RESET);
  delay(5);

  // CONFIG3: DELAY[7:5]=000, FILTER[4:0]=11000
  const uint8_t delayCode  = 0b000;
  const uint8_t filterCode = 0b11000;
  const uint8_t config3 = (delayCode << 5) | (filterCode & 0x1F);
  adcWriteReg(REG_CONFIG3, config3);

  // CONFIG4: external clock; other bits use defaults of 0.
  const uint8_t config4 = (1u << 7);
  adcWriteReg(REG_CONFIG4, config4);

  uint8_t r3 = adcReadReg(REG_CONFIG3);
  uint8_t r4 = adcReadReg(REG_CONFIG4);

  Serial.print("ADC CONFIG3 set/read: 0x"); Serial.print(config3, HEX);
  Serial.print(" / 0x"); Serial.println(r3, HEX);

  Serial.print("ADC CONFIG4 set/read: 0x"); Serial.print(config4, HEX);
  Serial.print(" / 0x"); Serial.println(r4, HEX);

  adcCommand(CMD_START);
}

// ---------------- Chopped acquisition ----------------

// Collect samples for one half-cycle, detect overflow on first sample
HalfCycleResult acquireHalfCycle() {
  HalfCycleResult result = {0, false};

  delayMicroseconds(SETTLE_US);

  // Discard initial samples after settling
  for (uint8_t i = 0; i < DISCARD_SAMPLES; i++) {
    (void)adcReadSample24();
  }

  // Read first good sample and check for overflow
  int32_t firstSample = adcReadSample24();
  if (isOverflow(firstSample)) {
    result.overflow = true;
    return result;
  }
  result.sum = firstSample;

  // Accumulate remaining good samples
  for (uint8_t i = 1; i < GOOD_SAMPLES; i++) {
    result.sum += adcReadSample24();
  }
  return result;
}

// Statistics accumulator for chopped measurements
LowerMoments chopStats;

// ================== Scan & Logging Configuration ==================
//
// These structures control the channel scanning state machine in loop() and
// the serial output format. ScanConfig is persisted to EEPROM via SavedConfig.

static constexpr int MAX_SCAN_CHANNELS = 7;  // GND excluded (used only for auto-zero)

/**
 * Output format for measurement results. Mutually exclusive; selected by
 * "log start", "plot start", and "log stop"/"plot stop" serial commands.
 * Checked by outputMeasurement() and the scanning state machine in loop().
 */
enum class OutputMode : uint8_t {
  Human,    // Default: multi-line human-readable text with SI prefixes
  CSV,      // Machine-readable CSV, one row per channel per integration period
  Plotter,  // Arduino Serial Plotter compatible: "label:value\t..." per line
};

/**
 * User-configurable scanning parameters. Stored in the global `scanConfig`
 * and persisted to EEPROM inside SavedConfig. Modified by serial commands
 * (scan add/remove, integrate, autozero, log/plot start/stop).
 */
struct ScanConfig {
  InputChannel channels[MAX_SCAN_CHANNELS];  // Ordered list of channels to scan
  int count;                    // Number of channels in scan list (0 = scan disabled)
  int integrationCycles;        // Chop cycles to accumulate per channel before reporting
  bool autoZeroEnabled;         // If true, periodically measure GND to track offset
  int autoZeroInterval;         // Full scan sweeps between auto-zero measurements
  OutputMode outputMode;        // Current serial output format
};

static ScanConfig scanConfig = {
  .channels = { InputChannel::Vx },
  .count = 1,
  .integrationCycles = 100,
  .autoZeroEnabled = true,
  .autoZeroInterval = 10,
  .outputMode = OutputMode::Human,
};

// Per-channel statistics, indexed by (uint8_t)InputChannel (0-7).
// In SCANNING mode, loop() accumulates into channelStats[currentChannel] and
// clears it on each channel switch. In IDLE mode, chopStats is used instead.
static LowerMoments channelStats[8];

// Per-channel saved DAC codes, indexed by (uint8_t)InputChannel (0-7).
// Allows fast channel switching during scanning without a full binary search.
// On first visit, binarySearchDAC() finds the code; subsequent visits restore it.
// If the saved code causes overflow (input changed), runOneChopCycle() triggers
// a fresh search and updates the saved code.
static int16_t channelDacCode[8];    // Saved DAC code per channel
static bool    channelDacValid[8];   // True after first successful measurement

void clearChannelDacCodes() {
  for (int i = 0; i < 8; i++) {
    channelDacCode[i] = 0;
    channelDacValid[i] = false;
  }
}

// Auto-zero offset in volts, measured by performAutoZero().
// Subtracted from all voltage results in outputMeasurement() when autoZeroValid is true.
static double autoZeroOffset = 0.0;
static bool autoZeroValid = false;

/**
 * Scanning state machine states. Governs the two main branches in loop():
 *   IDLE     - continuous measurement on currentInputChannel using chopStats
 *   SCANNING - cycles through scanConfig.channels[], uses channelStats[]
 *
 * Transitions: cmdScanStart() → SCANNING, cmdScanStop() → IDLE
 */
enum class ScanState : uint8_t { IDLE, SCANNING };
static ScanState scanState = ScanState::IDLE;
static int currentScanIndex = 0;   // Index into scanConfig.channels[] during SCANNING
static int scanCycleCount = 0;     // Full sweeps completed; used for auto-zero scheduling
static int chopCycleCount = 0;     // Chop cycles on current channel; includes settling
static constexpr int SCAN_SETTLE_CYCLES = 5;  // Chop cycles discarded after channel switch

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
 * Used by printCsvRow() and printPlotterLine() where compact names are needed.
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

// Forward declarations for command handlers
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
  // Check for duplicates
  for (int i = 0; i < scanConfig.count; i++) {
    if (scanConfig.channels[i] == ch) {
      Serial.println("Channel already in scan list.");
      return;
    }
  }
  if (scanConfig.count >= MAX_SCAN_CHANNELS) {
    Serial.println("ERROR: Scan list full (max 7 channels).");
    return;
  }
  scanConfig.channels[scanConfig.count++] = ch;
  Serial.print("Added ");
  Serial.print(getChannelShortName(ch));
  Serial.print(" to scan list (");
  Serial.print(scanConfig.count);
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
  for (int i = 0; i < scanConfig.count; i++) {
    if (scanConfig.channels[i] == ch) {
      // Shift remaining channels down
      for (int j = i; j < scanConfig.count - 1; j++) {
        scanConfig.channels[j] = scanConfig.channels[j + 1];
      }
      scanConfig.count--;
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
  Serial.print(scanConfig.count);
  Serial.println(" channels):");
  for (int i = 0; i < scanConfig.count; i++) {
    Serial.print("  ");
    Serial.print(i + 1);
    Serial.print(". ");
    Serial.println(getInputChannelName(scanConfig.channels[i]));
  }
  if (scanConfig.count == 0) {
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
  if (scanConfig.count == 0) {
    Serial.println("ERROR: Scan list is empty. Add channels first.");
    return;
  }
  scanState = ScanState::SCANNING;
  currentScanIndex = 0;
  scanCycleCount = 0;
  chopCycleCount = 0;

  // Switch to first channel
  InputChannel firstCh = scanConfig.channels[0];
  selectInputChannel(firstCh);
  uint8_t idx = (uint8_t)firstCh;
  if (channelDacValid[idx]) {
    // Restore saved DAC code from previous scan
    setDacCode(channelDacCode[idx]);
  } else {
    // First visit: full binary search
    binarySearchDAC();
    channelDacCode[idx] = currentDacCode;
    channelDacValid[idx] = true;
  }
  channelStats[(uint8_t)firstCh].clear();

  // Print CSV header if in CSV mode
  if (scanConfig.outputMode == OutputMode::CSV) {
    printCsvHeader();
  }

  Serial.println("Scanning started.");
}

void cmdScanStop() {
  scanState = ScanState::IDLE;
  chopCycleCount = 0;
  Serial.println("Scanning stopped.");
}

void cmdLogStart() {
  scanConfig.outputMode = OutputMode::CSV;
  Serial.println("CSV logging enabled.");
  if (scanState == ScanState::SCANNING) {
    printCsvHeader();
  }
}

void cmdLogStop() {
  scanConfig.outputMode = OutputMode::Human;
  Serial.println("CSV logging disabled, human-readable output restored.");
}

void cmdPlotStart() {
  scanConfig.outputMode = OutputMode::Plotter;
  Serial.println("Serial Plotter output enabled.");
}

void cmdPlotStop() {
  scanConfig.outputMode = OutputMode::Human;
  Serial.println("Serial Plotter output disabled, human-readable output restored.");
}

void cmdIntegrate(const char* arg) {
  int n = atoi(arg);
  if (n < 10 || n > 10000) {
    Serial.println("ERROR: Integration cycles must be 10-10000.");
    return;
  }
  scanConfig.integrationCycles = n;
  Serial.print("Integration set to ");
  Serial.print(n);
  Serial.print(" chop cycles (~");
  Serial.print((float)n / EST_FSPS * (DISCARD_SAMPLES + GOOD_SAMPLES + 1) * 2, 1);
  Serial.println(" sec per reading).");
}

void cmdAutoZero(const char* arg) {
  if (strcasecmp(arg, "on") == 0) {
    scanConfig.autoZeroEnabled = true;
    Serial.println("Auto-zero enabled.");
  } else if (strcasecmp(arg, "off") == 0) {
    scanConfig.autoZeroEnabled = false;
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
  scanConfig.autoZeroInterval = n;
  Serial.print("Auto-zero interval set to ");
  Serial.print(n);
  Serial.println(" scan sweeps.");
}

void cmdRange(const char* arg) {
  int r = atoi(arg);
  switch (r) {
    case 10:
      selectDividerRatio(DividerRatio::Div10);
      Serial.println("Range set to +/-50V (div 10).");
      break;
    case 100:
      selectDividerRatio(DividerRatio::Div100);
      Serial.println("Range set to +/-500V (div 100).");
      break;
    case 1000:
      selectDividerRatio(DividerRatio::Div1000);
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
  Serial.println(scanState == ScanState::SCANNING ? "SCANNING" : "IDLE");

  Serial.print("Output mode: ");
  switch (scanConfig.outputMode) {
    case OutputMode::Human:  Serial.println("Human-readable"); break;
    case OutputMode::CSV:    Serial.println("CSV logging"); break;
    case OutputMode::Plotter: Serial.println("Serial Plotter"); break;
  }

  Serial.print("Integration: ");
  Serial.print(scanConfig.integrationCycles);
  Serial.println(" chop cycles");

  Serial.print("Auto-zero: ");
  Serial.print(scanConfig.autoZeroEnabled ? "ON" : "OFF");
  Serial.print(", interval=");
  Serial.print(scanConfig.autoZeroInterval);
  Serial.println(" sweeps");

  if (autoZeroValid) {
    Serial.print("Auto-zero offset: ");
    Serial.print(autoZeroOffset * 1e9, 1);
    Serial.println(" nV");
  }

  Serial.print("Current channel: ");
  Serial.println(getInputChannelName(currentInputChannel));

  Serial.print("Current DAC code: ");
  Serial.println(currentDacCode);

  Serial.print("Divider ratio: ");
  Serial.println(getDividerRatioName(currentDividerRatio));

  Serial.print("DAC calibration: ");
  Serial.println(dacCalTableValid ? "CALIBRATED" : "nominal");

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
  Serial.println("help                Show this help");
  Serial.println("=======================\n");
}

// Forward declarations for config commands (implemented in EEPROM section)
void cmdConfigSave();
void cmdConfigLoad();
void cmdConfigFactory();
void cmdConfigAutostart(const char* arg);
void cmdConfigShow();

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
      scanConfig.count = 0;
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
    if (autoZeroValid) {
      Serial.print("Zero offset: ");
      Serial.print(autoZeroOffset * 1e9, 1);
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
  uint8_t devId = adcReadReg(REG_DEV_ID);

  // Write a test value to CONFIG1, read it back
  const uint8_t testVal = 0x55;
  uint8_t origVal = adcReadReg(REG_CONFIG1);
  adcWriteReg(REG_CONFIG1, testVal);
  uint8_t readBack = adcReadReg(REG_CONFIG1);
  adcWriteReg(REG_CONFIG1, origVal);  // Restore original

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
  setDacCode(0);  // Includes 250ms filter settling delay

  // Discard first sample, read a few and average
  (void)adcReadSample24();
  int64_t sum = 0;
  const int numSamples = 5;
  for (int i = 0; i < numSamples; i++) {
    sum += adcReadSample24();
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
  if (tmux_sel_state == HIGH) {
    chop();
  }

  // Set DAC to 0, measure
  setDacCode(0);  // Includes 250ms filter settling delay
  (void)adcReadSample24();
  int32_t adcAt0 = adcReadSample24();

  // Set DAC to +1000 (small positive step), measure
  setDacCode(1000);  // Includes 250ms filter settling delay
  (void)adcReadSample24();
  int32_t adcAt1000 = adcReadSample24();

  // Restore DAC
  setDacCode(0);

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
  setDacCode(0);  // Includes 250ms filter settling delay

  // Ensure TMUXSEL=LOW, measure (Vx - DAC) * gain
  if (tmux_sel_state == HIGH) {
    chop();
  }
  delayMicroseconds(SETTLE_US);  // Mux settling (300µs)
  (void)adcReadSample24();
  int32_t phase1 = adcReadSample24();

  // Switch to TMUXSEL=HIGH, measure (DAC - Vx) * gain
  chop();
  delayMicroseconds(SETTLE_US);  // Mux settling (300µs)
  (void)adcReadSample24();
  int32_t phase2 = adcReadSample24();

  // Switch back
  chop();

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

  // Save current channel
  InputChannel savedChannel = getInputChannel();

  // Switch to GND
  selectInputChannel(InputChannel::GND);

  // Set DAC to 0V for this test
  setDacCode(0);

  // Measure
  (void)adcReadSample24();
  int64_t sum = 0;
  const int numSamples = 5;
  for (int i = 0; i < numSamples; i++) {
    sum += adcReadSample24();
  }
  int32_t avg = sum / numSamples;

  // Restore original channel
  selectInputChannel(savedChannel);

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

  // Save current channel
  InputChannel savedChannel = getInputChannel();

  // Set DAC to ~5V to match the expected reference voltage
  const int16_t dacAt5V = 32764;  // Near +5V, 14-bit aligned
  setDacCode(dacAt5V);

  // Measure the raw reference average (J3)
  selectInputChannel(InputChannel::VrefRaw);
  delayMicroseconds(SETTLE_US);

  (void)adcReadSample24();
  int64_t sum = 0;
  const int numSamples = 5;
  for (int i = 0; i < numSamples; i++) {
    sum += adcReadSample24();
  }
  int32_t reading = sum / numSamples;

  // Restore original channel and DAC
  selectInputChannel(savedChannel);
  setDacCode(0);

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

  // Save current state
  InputChannel savedChannel = getInputChannel();

  // Switch to GND input and set DAC to 0V
  selectInputChannel(InputChannel::GND);
  setDacCode(0);

  // Accumulate chopped measurements for accurate zero reading
  LowerMoments zeroStats;
  const int numIterations = 10;

  for (int i = 0; i < numIterations; i++) {
    HalfCycleResult rA = acquireHalfCycle();
    chop();
    HalfCycleResult rB = acquireHalfCycle();
    chop();

    if (rA.overflow || rB.overflow) {
      Serial.println("FAIL (overflow during zero measurement)");
      selectInputChannel(savedChannel);
      return false;
    }

    double demod = (double)(rA.sum - rB.sum) / (2.0 * GOOD_SAMPLES);
    zeroStats.accumulate(demod);
  }

  // Restore original channel
  selectInputChannel(savedChannel);

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
  // Save current state
  InputChannel savedChannel = currentInputChannel;
  int16_t savedDac = currentDacCode;

  // Switch to raw reference input
  selectInputChannel(InputChannel::VrefRaw);

  // Set DAC to ~5V to measure against the reference
  setDacCode(REF_MEASURE_DAC_CODE);

  // Accumulate multiple chopped measurements for stability
  LowerMoments refStats;

  for (int i = 0; i < REF_MEASURE_ITERATIONS; i++) {
    HalfCycleResult rA = acquireHalfCycle();
    chop();
    HalfCycleResult rB = acquireHalfCycle();
    chop();

    // Check for overflow (shouldn't happen if references are working)
    if (rA.overflow || rB.overflow) {
      Serial.println("WARNING: Overflow during reference measurement!");
      // Restore state and abort
      selectInputChannel(savedChannel);
      setDacCode(savedDac);
      return;
    }

    // Demodulate and accumulate
    double demod = (double)(rA.sum - rB.sum) / (2.0 * GOOD_SAMPLES);
    refStats.accumulate(demod);
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

  // Restore previous state
  selectInputChannel(savedChannel);
  setDacCode(savedDac);
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
//
// Three output backends, selected by OutputMode in scanConfig:
//   Human   - multi-line text with SI prefixes (outputMeasurement, Human case)
//   CSV     - printCsvHeader() + printCsvRow() per measurement
//   Plotter - plotterBeginLine/AddChannel/EndLine() per scan sweep
//
// All output is routed through outputMeasurement() except for multi-channel
// Plotter mode, which is assembled directly in the SCANNING branch of loop()
// because it needs to collect all channels into a single tab-separated line.

/**
 * Print CSV column header. Called once by cmdScanStart() and cmdLogStart()
 * when CSV mode is active, so the host can parse column names.
 */
void printCsvHeader() {
  Serial.println("# timestamp_ms,channel,voltage_V,uncertainty_V,stddev_counts,mean_counts,n,dac_code,drift_ppb");
}

/**
 * Print one CSV data row for a completed channel measurement.
 * Called by outputMeasurement() when outputMode == CSV.
 *
 * Format: timestamp_ms,channel,voltage_V,uncertainty_V,stddev_counts,mean_counts,n,dac_code,drift_ppb
 * All values comma-separated with no spaces, suitable for direct import into
 * spreadsheets or the PC companion script (tools/diffvm.py DataLogger).
 *
 * @param voltage      Corrected voltage in volts (drift + auto-zero applied)
 * @param uncertainty  Standard deviation in volts at the input terminal
 * @param driftPpb     Reference drift correction magnitude in parts-per-billion
 */
void printCsvRow(InputChannel channel, double voltage, double uncertainty,
                 double stddevCounts, double meanCounts, int n,
                 int16_t dacCode, double driftPpb) {
  Serial.print(millis());
  Serial.print(',');
  Serial.print(getChannelShortName(channel));
  Serial.print(',');
  Serial.print(voltage, 12);  // Full precision in scientific notation
  Serial.print(',');
  Serial.print(uncertainty, 12);
  Serial.print(',');
  Serial.print(stddevCounts, 3);
  Serial.print(',');
  Serial.print(meanCounts, 3);
  Serial.print(',');
  Serial.print(n);
  Serial.print(',');
  Serial.print(dacCode);
  Serial.print(',');
  Serial.print(driftPpb, 3);
  Serial.println();
}

/**
 * Emit one channel's data in Arduino Serial Plotter format ("label:value").
 * Called by plotterAddChannel() to build a tab-separated line.
 *
 * Arduino IDE 2.x Serial Plotter parses "label:value\tlabel:value\n" lines
 * and plots each unique label as a separate trace with auto-legend.
 *
 * When singleChannel is true, appends a second trace "label_sd:value" so
 * the plotter shows both the measurement and its uncertainty.
 *
 * @param singleChannel  If true, append stddev as a second trace
 */
void printPlotterLine(InputChannel channel, double voltage, double stddev,
                      bool singleChannel) {
  Serial.print(getChannelShortName(channel));
  Serial.print(':');
  Serial.print(voltage, 12);
  if (singleChannel) {
    Serial.print('\t');
    Serial.print(getChannelShortName(channel));
    Serial.print("_sd:");
    Serial.print(stddev, 12);
  }
}

// Plotter line assembly state. The three functions below are used together
// to build one plotter output line across multiple channels:
//
//   In loop() SCANNING branch (multi-channel):
//     plotterBeginLine()                       — at first channel
//     plotterAddChannel(ch1, v1, sd1, false)   — for each channel
//     plotterAddChannel(ch2, v2, sd2, false)
//     plotterEndLine()                         — after last channel
//
//   In outputMeasurement() (single-channel or IDLE):
//     plotterBeginLine() → plotterAddChannel() → plotterEndLine()

static bool plotterLineStarted = false;

/** Reset line state. Call before the first plotterAddChannel() of a new line. */
void plotterBeginLine() {
  plotterLineStarted = false;
}

/** Append one channel to the current plotter line. Inserts tab separator
 *  between channels. Delegates to printPlotterLine() for formatting. */
void plotterAddChannel(InputChannel channel, double voltage, double stddev,
                       bool singleChannel) {
  if (plotterLineStarted) {
    Serial.print('\t');
  }
  printPlotterLine(channel, voltage, stddev, singleChannel);
  plotterLineStarted = true;
}

/** Terminate the current plotter line with a newline. */
void plotterEndLine() {
  Serial.println();
  plotterLineStarted = false;
}

// ================== Auto-Zero ==================

/**
 * Perform an auto-zero calibration by measuring the GND input channel.
 *
 * Called by:
 *   - processCommand() on "zero" command (immediate manual calibration)
 *   - loop() SCANNING branch every scanConfig.autoZeroInterval sweeps
 *
 * Procedure:
 *   1. Save current input channel and DAC code
 *   2. Switch mux to GND, run binarySearchDAC() to find DAC null point
 *   3. Run 20 chopped measurement cycles to determine offset
 *   4. Store result in autoZeroOffset (volts); set autoZeroValid = true
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
  // Save current state
  InputChannel savedChannel = currentInputChannel;
  int16_t savedDac = currentDacCode;

  // Switch to GND
  selectInputChannel(InputChannel::GND);
  setDacCode(0);

  // Run binary search for GND (should converge quickly near 0)
  binarySearchDAC();

  // Accumulate chopped measurements
  LowerMoments gndStats;
  const int numIterations = 20;  // More iterations for better accuracy

  for (int i = 0; i < numIterations; i++) {
    HalfCycleResult rA = acquireHalfCycle();
    chop();
    HalfCycleResult rB = acquireHalfCycle();
    chop();

    if (rA.overflow || rB.overflow) {
      Serial.println("WARNING: Overflow during auto-zero!");
      selectInputChannel(savedChannel);
      setDacCode(savedDac);
      return;
    }

    double demod = (double)(rA.sum - rB.sum) / (2.0 * GOOD_SAMPLES);
    gndStats.accumulate(demod);
  }

  // Compute zero offset
  autoZeroOffset = computeInputVoltage(gndStats.mean(), currentDacCode, InputChannel::GND);
  autoZeroValid = true;

  // Restore previous state
  selectInputChannel(savedChannel);
  setDacCode(savedDac);
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

/**
 * Save current runtime configuration to EEPROM.
 * Called by processCommand() on "config save". Serializes scanConfig,
 * configAutoStart, and currentDividerRatio into a SavedConfig blob
 * with magic number and CRC16, then writes byte-by-byte to EEPROM.
 */
void cmdConfigSave() {
  SavedConfig saved;
  saved.magic = EEPROM_MAGIC;
  saved.scanConfig = scanConfig;
  saved.autoStart = configAutoStart;
  saved.dividerRatio = currentDividerRatio;

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
  scanConfig = saved.scanConfig;
  configAutoStart = saved.autoStart;
  if (saved.dividerRatio != currentDividerRatio) {
    selectDividerRatio(saved.dividerRatio);
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
  scanConfig.channels[0] = InputChannel::Vx;
  scanConfig.count = 1;
  scanConfig.integrationCycles = 100;
  scanConfig.autoZeroEnabled = true;
  scanConfig.autoZeroInterval = 10;
  scanConfig.outputMode = OutputMode::Human;
  configAutoStart = false;
  autoZeroOffset = 0.0;
  autoZeroValid = false;
  clearChannelDacCodes();
  scanState = ScanState::IDLE;
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
 * Central output function: all three output modes (Human, CSV, Plotter) are
 * routed through here, except multi-channel Plotter which is assembled
 * directly in loop()'s SCANNING branch.
 *
 * Called by:
 *   - loop() IDLE branch: every integrationCycles with chopStats
 *   - loop() SCANNING branch: after each channel completes integration
 *
 * Voltage computation pipeline:
 *   1. computeInputVoltage() — DAC code + ADC mean → raw voltage
 *   2. getRefCorrectionFactor() — multiply by reference drift correction
 *   3. Subtract autoZeroOffset if valid
 *
 * @param channel  Which input channel was measured
 * @param stats    LowerMoments accumulator with the measurement data
 * @param dacCode  DAC code that was active during this measurement
 */
void outputMeasurement(InputChannel channel, LowerMoments &stats, int16_t dacCode) {
  double adcMean = stats.mean();
  double adcStdDev = stats.standardDeviation();

  // Compute voltage
  double vxRaw = computeInputVoltage(adcMean, dacCode, channel);
  double vxUncertainty = computeInputUncertainty(adcStdDev, channel);

  // Apply reference drift correction
  double refCorrection = getRefCorrectionFactor();
  double vxCorrected = vxRaw * refCorrection;

  // Apply auto-zero correction if valid
  if (autoZeroValid) {
    vxCorrected -= autoZeroOffset;
  }

  double driftPpb = (refCorrection - 1.0) * 1e9;

  switch (scanConfig.outputMode) {
    case OutputMode::CSV:
      printCsvRow(channel, vxCorrected, vxUncertainty,
                  adcStdDev, adcMean, (int)stats.count(),
                  dacCode, driftPpb);
      break;

    case OutputMode::Plotter:
      // Plotter output is handled at the scan-sweep level for multi-channel
      // For single-channel or IDLE mode, output directly
      if (scanState == ScanState::IDLE || scanConfig.count <= 1) {
        plotterBeginLine();
        plotterAddChannel(channel, vxCorrected, vxUncertainty,
                          scanConfig.count <= 1);
        plotterEndLine();
      }
      // Multi-channel plotter is assembled in the scanning state machine
      break;

    case OutputMode::Human:
    default: {
      char vxBuf[32], uncBuf[32];
      formatVoltage(vxCorrected, vxBuf, sizeof(vxBuf), 9);
      formatVoltage(vxUncertainty, uncBuf, sizeof(uncBuf), 3);

      Serial.print("Vx = ");
      Serial.print(vxBuf);
      Serial.print(" +/- ");
      Serial.print(uncBuf);
      Serial.print(" (n=");
      Serial.print(stats.count(), 0);
      Serial.print(", DAC=");
      Serial.print(dacCode);
      Serial.print(", range=");
      Serial.print(getInputChannelName(channel));
      Serial.println(")");

      Serial.print("  raw: mean=");
      Serial.print(adcMean, 3);
      Serial.print(", sd=");
      Serial.print(adcStdDev, 3);
      Serial.print(", min=");
      Serial.print(stats.getMin(), 3);
      Serial.print(", max=");
      Serial.println(stats.getMax(), 3);

      double filterErr = predictFilterError();
      Serial.print("  drift: filterErr=");
      Serial.print(filterErr * 1e9, 1);
      Serial.print(" nV, driftRate=");
      Serial.print(estimatedDriftRate * 1e9, 3);
      Serial.print(" nV/s, correction=");
      Serial.print(driftPpb, 3);
      Serial.println(" ppb");

      if (autoZeroValid) {
        Serial.print("  zero: offset=");
        Serial.print(autoZeroOffset * 1e9, 1);
        Serial.println(" nV");
      }
      break;
    }
  }
}

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 2000) {}

  // Configure all pins
  pinMode(PIN_CS_ADC,     OUTPUT);
  pinMode(PIN_CS_DAC,     OUTPUT);
  pinMode(PIN_DAC_RESET,  OUTPUT);
  pinMode(PIN_DAC_CLEAR,  OUTPUT);
  pinMode(PIN_ADC_DRDY,   INPUT); // no pullup needed, dedicated output from adc to teensy
  pinMode(PIN_TMUX_EN,    OUTPUT);
  pinMode(PIN_TMUXSEL,    OUTPUT);
  pinMode(PIN_CI_TMUXSEL, OUTPUT);

  // Set initial pin states
  csHigh(PIN_CS_ADC);
  csHigh(PIN_CS_DAC);
  digitalWrite(PIN_DAC_RESET, HIGH);  // Keep DAC out of reset
  digitalWrite(PIN_DAC_CLEAR, HIGH);  // Keep DAC clear inactive (active low)
  digitalWrite(PIN_TMUXSEL, LOW);
  digitalWrite(PIN_CI_TMUXSEL, LOW);
  digitalWriteFast(PIN_TMUX_EN, LOW);

  // Initialize input multiplexer (MUX36S08)
  initInputMux();

  // Initialize HV divider ratio control (MUX36D04)
  initDividerControl();

  // Initialize DAC calibration table with nominal values
  // (will be overwritten if calibration data is loaded)
  initDacCalTable();

  // Initialize per-channel DAC code cache
  clearChannelDacCodes();

  // Initialize reference drift tracking
  initRefTracking();

  // Set external clock for ADS127L11
  analogWriteFrequency(PIN_CLK_ADC, 25000000);
  analogWrite(PIN_CLK_ADC, 128);

  // Initialize SPI before using it
  SPI.begin();

  // Initialize DAC (SPI must be ready first)
  initDAC_2xMode();

  Serial.println("Starting ADC init...");
  adcInitAndConfigure();

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
    if (POST_HALT_ON_FAIL) {
      Serial.println("POST failed - system halted.");
      Serial.println("Set POST_HALT_ON_FAIL=false to continue despite failures.");
      while (true) {
        // Blink LED to indicate failure (if available)
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
  Serial.println(dacCalTableValid ? "CALIBRATED" : "nominal (uncalibrated)");
  Serial.println("====================================\n");

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
  if (configAutoStart && scanConfig.count > 0) {
    Serial.println("Auto-starting scan...");
    cmdScanStart();
  }

  Serial.println("Type 'help' for available commands.");
}

/**
 * Execute one complete chop cycle and accumulate the demodulated result.
 *
 * Called by loop() on every iteration (both IDLE and SCANNING modes).
 * Encapsulates the core measurement sequence that was previously inline in loop():
 *   1. acquireHalfCycle() — phase A (current TMUX state)
 *   2. chop() — toggle TMUX with charge-injection minimization
 *   3. acquireHalfCycle() — phase B (opposite TMUX state)
 *   4. chop() — restore original state for next cycle
 *   5. Demodulate: (sumA - sumB) / (2 × GOOD_SAMPLES) → cancels offset/1/f
 *   6. Accumulate into stats
 *
 * On overflow (preamp saturated), triggers binarySearchDAC() to re-center
 * the DAC and clears stats since the measurement baseline changed.
 *
 * @param stats  LowerMoments accumulator to add the demodulated sample to.
 *               In IDLE mode this is chopStats; in SCANNING mode it's
 *               channelStats[currentChannel].
 * @return       true if measurement succeeded, false if overflow occurred
 *               (caller should skip output and retry on next loop iteration)
 */
bool runOneChopCycle(LowerMoments &stats) {
  HalfCycleResult resultA = acquireHalfCycle();

  if (resultA.overflow) {
    binarySearchDAC();
    stats.clear();
    return false;
  }

  chop();

  HalfCycleResult resultB = acquireHalfCycle();

  if (resultB.overflow) {
    chop();  // Return to original state
    binarySearchDAC();
    stats.clear();
    return false;
  }

  chop();

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
 *   IDLE mode:
 *     - Measures currentInputChannel continuously using chopStats
 *     - Outputs a reading every integrationCycles via outputMeasurement()
 *     - Stats accumulate indefinitely (not cleared between outputs)
 *
 *   SCANNING mode:
 *     - Cycles through scanConfig.channels[0..count-1]
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

  // ---- IDLE MODE: measure current channel continuously ----
  if (scanState == ScanState::IDLE) {
    if (!runOneChopCycle(chopStats)) {
      return;  // Overflow handled, restart
    }

    // Output every integrationCycles
    static int idleCounter = 0;
    if (++idleCounter >= scanConfig.integrationCycles) {
      idleCounter = 0;
      outputMeasurement(currentInputChannel, chopStats, currentDacCode);
      // Don't clear stats in IDLE mode - accumulates continuously
    }
    return;
  }

  // ---- SCANNING MODE ----
  InputChannel scanCh = scanConfig.channels[currentScanIndex];

  // Run one chop cycle on the current channel
  LowerMoments &stats = channelStats[(uint8_t)scanCh];
  if (!runOneChopCycle(stats)) {
    chopCycleCount = 0;  // Reset after overflow/DAC change
    // Save the re-centered DAC code for this channel
    channelDacCode[(uint8_t)scanCh] = currentDacCode;
    channelDacValid[(uint8_t)scanCh] = true;
    return;
  }

  chopCycleCount++;

  // Discard initial settling cycles after channel switch
  if (chopCycleCount <= SCAN_SETTLE_CYCLES) {
    stats.clear();
    return;
  }

  // Check if integration is complete for this channel
  if (chopCycleCount >= scanConfig.integrationCycles + SCAN_SETTLE_CYCLES) {
    // Output this channel's result
    // For multi-channel plotter mode, we buffer all channels and print one line per sweep
    if (scanConfig.outputMode == OutputMode::Plotter && scanConfig.count > 1) {
      // Compute voltage for plotter
      double adcMean = stats.mean();
      double adcStdDev = stats.standardDeviation();
      double vxRaw = computeInputVoltage(adcMean, currentDacCode, scanCh);
      double refCorrection = getRefCorrectionFactor();
      double vxCorrected = vxRaw * refCorrection;
      if (autoZeroValid) vxCorrected -= autoZeroOffset;
      double vxUncertainty = computeInputUncertainty(adcStdDev, scanCh);

      if (currentScanIndex == 0) plotterBeginLine();
      plotterAddChannel(scanCh, vxCorrected, vxUncertainty, false);
      if (currentScanIndex == scanConfig.count - 1) plotterEndLine();
    } else {
      outputMeasurement(scanCh, stats, currentDacCode);
    }

    // Save DAC code for this channel (for fast restore on next visit)
    channelDacCode[(uint8_t)scanCh] = currentDacCode;
    channelDacValid[(uint8_t)scanCh] = true;

    // Advance to next channel
    currentScanIndex++;

    if (currentScanIndex >= scanConfig.count) {
      // Completed one full sweep
      currentScanIndex = 0;
      scanCycleCount++;

      // Check if auto-zero is due
      if (scanConfig.autoZeroEnabled &&
          (scanCycleCount % scanConfig.autoZeroInterval) == 0) {
        performAutoZero();
        if (autoZeroValid && scanConfig.outputMode == OutputMode::Human) {
          Serial.print("  [auto-zero: offset=");
          Serial.print(autoZeroOffset * 1e9, 1);
          Serial.println(" nV]");
        }
      }
    }

    // Switch to next channel
    InputChannel nextCh = scanConfig.channels[currentScanIndex];
    if (nextCh != currentInputChannel) {
      selectInputChannel(nextCh);
      uint8_t idx = (uint8_t)nextCh;
      if (channelDacValid[idx]) {
        // Restore saved DAC code (fast path: only filter settling, no search)
        setDacCode(channelDacCode[idx]);
      } else {
        // First visit to this channel: full binary search
        binarySearchDAC();
        channelDacCode[idx] = currentDacCode;
        channelDacValid[idx] = true;
      }
    }
    channelStats[(uint8_t)nextCh].clear();
    chopCycleCount = 0;
  }
}
