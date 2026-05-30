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
// Excitation magnitude (1 V vs 2.5 V) is auto-selected by bank index to
// keep R_ref dissipation reasonable: banks 0 (20 Ω) and 1 (200 Ω) use 1 V,
// all other banks use 2.5 V. Either way V_exc cancels out of the ratio.
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

  // Run `nCycles` chop cycles at the locked DAC code; returns the demodulated
  // mean in ADC counts. Sets *overflow=true if the preamp railed during the
  // integration; in that case the returned value is undefined.
  double (*runChopCycles)(int nCycles, bool* overflow);

  // Calibrated DAC voltage at the *current* DAC code. Used to convert the
  // demodulated ADC residual into an absolute input voltage:
  //   Vx = getCurrentDacVoltage() + adcMean × adcLsbV / preampGain
  double (*getCurrentDacVoltage)();

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

  // --- Constants for voltage arithmetic ------------------------------------
  double adcLsbV;       // ADC LSB in volts at the input
  double preampGain;    // AD8428 chain cumulative gain (~2000)
  double dacLsbV;       // (not currently used; kept for parity with CalVerifyApi)

  // --- I/O -----------------------------------------------------------------
  Stream* io;
};

// CLI entry point. The R_ref is set on the host via `rref`; this command
// just measures the currently-wired DUT against it.
//   meas r
//   meas r --cycles 100
//   meas r --no-emf-cancel
//   meas r --cycles 100 --no-emf-cancel
void cmdMeasR(const OhmsMeasApi& api, int argc, const char* const* argv);
