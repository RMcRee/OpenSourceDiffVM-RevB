// OhmsMeas — 4-wire-Kelvin polarity-reversed ratiometric resistance
// measurement. Drives an external breakout (see hardware-errata OHMS-1
// and docs/ohms-breakout.svg) consisting of ±1 V / ±2.5 V excitation, a
// manually-wired R_ref (selected on the host via the `rref` CLI), a
// TMUX7234 polarity switch, and 4-terminal Kelvin DUT posts.
//
// Topology in one line:
//   R_dut = R_ref × (Vx3 − Vx4) / (Vx2 − Vx3)
//
//   Vx3 = Kelvin sense at DUT high terminal
//   Vx4 = Kelvin sense at DUT low terminal
//   Vx2 = Kelvin sense at the R_ref-top node (after the Force MUX)
//
// V_exc, polarity switch R_on, and Force-MUX R_on cancel out of the ratio.
// Only the stored R_ref bank values need calibration (see Calibration.md
// CAL-12).
//
// Excitation magnitude (1 V vs 2.5 V) is auto-selected by R_ref value to
// keep R_ref dissipation reasonable: R_ref < 500 Ω (r2) uses 1 V, all
// others use 2.5 V. Either way V_exc cancels out of the ratio.
//
// This module is decoupled from the .ino: all DAC, ADC, chop, mux, and
// GPIO operations go through function pointers in OhmsMeasApi. The .ino
// builds the struct from local adapters and calls cmdMeasR().

#pragma once
#include <stdint.h>
#include <stddef.h>

class Stream;

struct OhmsMeasApi {
  // --- Chop / DAC controls --------------------------------------------------

  // Run a BinSrch (with backoff) at the current channel + polarity to bring
  // the preamp into linear range. Returns true on success.
  bool   (*binarySearchDac)();

  // Seeded variant: bisection restricted to [seed−halfBracket, seed+halfBracket]
  // with proactive AAF reset (input mux → GND, DAC → autozero null) between
  // railed probes. The reset keeps preamp rail saturation from accumulating
  // across probes — the mechanism that defeats full-range BinSrch in ohms mode
  // at high R_ref (OhmsMeasurement.md, "r5 high-ρ limit"). Use whenever the
  // target voltage is predictable: cached/mirrored codes, V_exc-derived senses,
  // or a --nominal DUT estimate.
  bool   (*binarySearchDacSeeded)(int16_t seedCode, int32_t halfBracket);

  // Run `nCycles` chop cycles at the locked DAC code; returns the demodulated
  // mean in ADC counts. Sets *overflow=true if the preamp railed during the
  // integration; in that case the returned value is undefined. If outSdCounts
  // is non-null, receives the population sd (ADC counts, same units as the
  // mean) of the per-chop-pair demod values — used to build a within-run
  // standard-error estimate instead of relying solely on across-repeat
  // scatter.
  double (*runChopCycles)(int nCycles, bool* overflow, double* outSdCounts);

  // Calibrated DAC voltage at the *current* DAC code. Used to convert the
  // demodulated ADC residual into an absolute input voltage:
  //   Vx = getCurrentDacVoltage() + adcMean × adcLsbV / preampGain
  double (*getCurrentDacVoltage)();

  // Pre-set the DAC code (for seeding from a cached per-sense hint, so we
  // can skip the full BinSrch when the cache is valid).
  void (*setDacCode)(int16_t code);

  // Read the current DAC code, for caching the BinSrch-converged value.
  int16_t (*getCurrentDacCode)();

  // Convert a predicted input voltage to the nearest DAC code. Nominal-LSB
  // inversion (no INL table) — seed accuracy is bracket-scale, not µV-scale.
  int16_t (*dacVoltsToCode)(double v);

  // --- Mux selectors --------------------------------------------------------

  // Selects which Kelvin sense feeds the preamp through the on-board MUX36D08.
  //   0 → Vx3 (DUT high)
  //   1 → Vx4 (DUT low)
  //   2 → Vx2 (R_ref top via Sense MUX)
  void   (*selectSense)(uint8_t senseIdx);

  // --- Breakout GPIO controls ----------------------------------------------

  // Set the excitation polarity at SW1.IN.
  //   pos = true  → +V_exc (TMUX7234 routes NO → COM)
  //   pos = false → −V_exc (NC → COM)
  void   (*setExcitationPolarity)(bool pos);

  // Select excitation magnitude.
  //   low = true  → 1 V rail (used when R_ref < 500 Ω to limit dissipation)
  //   low = false → 2.5 V rail (all other ranges)
  void   (*setExcitationVoltage)(bool low);

  // --- Cal store accessor ---------------------------------------------------

  // Returns the calibrated R_ref value (ohms) for the resistor currently
  // wired into the manual socket (set via the host `rref` CLI). Returns
  // NAN if no R_ref has been selected.
  double (*getCalRref)();

  // Returns the current signal-chain zero offset in volts, as tracked by
  // the auto-zero subsystem (GND-channel measurement at the DAC null code).
  // Subtracting this from each sense voltage removes DAC zero-code drift
  // (ε₀) and preamp chain residual from the reported V_dut and V_ref.
  // With EMF cancellation (default) a constant offset cancels in the ratio
  // anyway; the correction matters for --no-emf-cancel and for keeping the
  // ± polarity readings symmetric for mirror-seed bracket accuracy.
  // Returns 0.0 if no auto-zero measurement has been taken yet.
  double (*getSignalChainOffset)();

  // Triggers one auto-zero measurement (GND channel, DAC null code). Uses
  // ScopedInstrumentState internally so mux + DAC are restored on return.
  // Called at the start of cmdMeasR and after every other run to keep the
  // signal-chain offset current during long measurement sequences.
  void (*performAutoZero)();

  // --- Constants for voltage arithmetic ------------------------------------
  double adcLsbV;       // ADC LSB in volts at the input
  double preampGain;    // AD8428 chain cumulative gain (~2000)
  double dacLsbV;       // DAC LSB in volts; used to size seed brackets in codes

  // --- I/O -----------------------------------------------------------------
  Stream* io;
};

// CLI entry point. The R_ref is set on the host via `rref`; this command
// just measures the currently-wired DUT against it.
//   meas r
//   meas r --cycles 100
//   meas r --no-emf-cancel
//   meas r --cycles 100 --no-emf-cancel
//   meas r --nominal 200k     (seeds the Vx3 search; needed for cold starts
//                              at high R_ref, e.g. r5 + DUT ≥ ~100 kΩ)
void cmdMeasR(const OhmsMeasApi& api, int argc, const char* const* argv);
