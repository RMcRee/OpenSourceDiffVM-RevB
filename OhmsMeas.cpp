// OhmsMeas.cpp — see OhmsMeas.h for rationale and the public interface.

#include <Arduino.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "OhmsMeas.h"

namespace {

constexpr int  kDefaultCycles    = 285;     // ≈ 15 s at OSR=12800, samples=14 (matches CalVerify)
constexpr int  kSenseCount       = 3;       // Vx3, Vx4, Vx2 (R_ref top)
constexpr uint32_t kPolaritySettleMs = 200; // V_exc + AAF settle after polarity flip
constexpr uint32_t kExcVoltageSettleMs = 200; // V_exc rail change + AAF settle
constexpr double   kLowExcThresholdOhms = 500.0; // R_ref below this → 1 V rail

const char* kSenseName[kSenseCount] = { "Vx3", "Vx4", "Vx2" };

struct OhmsConfig {
  int     cycles;
  bool    emfCancel;
};

bool parseArgs(int argc, const char* const* argv, OhmsConfig& cfg, Stream& io) {
  cfg.cycles    = kDefaultCycles;
  cfg.emfCancel = true;

  for (int i = 0; i < argc; ++i) {
    const char* a = argv[i];
    if (!a) continue;
    if (!strcasecmp(a, "--cycles") || !strcasecmp(a, "-n")) {
      if (i + 1 >= argc || !argv[i+1]) {
        io.println(F("ERROR: --cycles needs an integer."));
        return false;
      }
      cfg.cycles = atoi(argv[++i]);
      if (cfg.cycles < 1 || cfg.cycles > 10000) {
        io.println(F("ERROR: --cycles out of range (1 to 10000)."));
        return false;
      }
    } else if (!strcasecmp(a, "--no-emf-cancel")) {
      cfg.emfCancel = false;
    } else {
      io.print(F("ERROR: unrecognized argument: ")); io.println(a);
      io.println(F("Usage: meas r [--cycles N] [--no-emf-cancel]"));
      return false;
    }
  }
  return true;
}

// Measure Vx at the current channel for the current polarity. Locks the DAC
// via BinSrch first, then integrates `cycles` chop cycles, then converts the
// demodulated ADC residual to volts at the input. Returns NAN on failure.
double measureSenseVolts(const OhmsMeasApi& api, int cycles, bool* outOverflow) {
  if (outOverflow) *outOverflow = false;
  if (!api.binarySearchDac()) {
    if (outOverflow) *outOverflow = true;
    return NAN;
  }
  bool ovf = false;
  double adcMean = api.runChopCycles(cycles, &ovf);
  if (ovf) {
    if (outOverflow) *outOverflow = true;
    return NAN;
  }
  double vdac  = api.getCurrentDacVoltage();
  double delta = adcMean * api.adcLsbV / api.preampGain;
  return vdac + delta;
}

void printOhms(Stream& io, double r) {
  // SI-prefix print: TΩ, GΩ, MΩ, kΩ, Ω, mΩ, µΩ
  if (!isfinite(r)) { io.print(F("NaN Ω")); return; }
  double ar = fabs(r);
  const char* unit = "Ω";
  double scale = 1.0;
  if      (ar >= 1.0e12) { unit = "TΩ"; scale = 1.0e-12; }
  else if (ar >= 1.0e9)  { unit = "GΩ"; scale = 1.0e-9; }
  else if (ar >= 1.0e6)  { unit = "MΩ"; scale = 1.0e-6; }
  else if (ar >= 1.0e3)  { unit = "kΩ"; scale = 1.0e-3; }
  else if (ar >= 1.0)    { unit = "Ω";  scale = 1.0; }
  else if (ar >= 1.0e-3) { unit = "mΩ"; scale = 1.0e3; }
  else                   { unit = "µΩ"; scale = 1.0e6; }
  io.print(r * scale, 6); io.print(' '); io.print(unit);
}

void printSignedV(Stream& io, double v, int decimals = 8) {
  if (v >= 0) io.print('+');
  io.print(v, decimals);
}

}  // namespace

void cmdMeasR(const OhmsMeasApi& api, int argc, const char* const* argv) {
  Stream& io = *api.io;

  OhmsConfig cfg{};
  if (!parseArgs(argc, argv, cfg, io)) return;

  double rRef = api.getCalRref();
  if (!isfinite(rRef) || rRef <= 0.0) {
    io.println(F("ERROR: no R_ref selected. Run 'rref <r1..r8 | color>' first."));
    return;
  }

  // Select excitation magnitude from R_ref value, then let V_exc + AAF settle.
  const bool lowExc = (rRef < kLowExcThresholdOhms);
  api.setExcitationVoltage(lowExc);
  delay(kExcVoltageSettleMs);

  // Storage: voltages[sense][polarity]
  //   sense:    0=Vx3, 1=Vx4, 2=Vx2
  //   polarity: 0=+,   1=−
  double v[kSenseCount][2];
  for (int i = 0; i < kSenseCount; ++i) v[i][0] = v[i][1] = NAN;

  const int polCount = cfg.emfCancel ? 2 : 1;
  const bool polVal[2] = { true, false };

  io.println();
  io.print(F("meas r: R_ref = ")); io.print(rRef, 6); io.print(F(" Ω"));
  io.print(F("  V_exc=")); io.print(lowExc ? F("1.0 V") : F("2.5 V"));
  io.print(F("  cycles=")); io.print(cfg.cycles);
  if (!cfg.emfCancel) io.print(F("  [no EMF cancel]"));
  io.println();

  for (int p = 0; p < polCount; ++p) {
    api.setExcitationPolarity(polVal[p]);
    delay(kPolaritySettleMs);

    for (int s = 0; s < kSenseCount; ++s) {
      api.selectSense((uint8_t)s);
      bool ovf = false;
      double vs = measureSenseVolts(api, cfg.cycles, &ovf);
      if (ovf || !isfinite(vs)) {
        io.print(F("  ")); io.print(kSenseName[s]);
        io.print(F(" @ pol ")); io.print(polVal[p] ? '+' : '-');
        io.println(F(": MEASUREMENT FAILED (preamp railed or BinSrch fail). Abort."));
        // Restore a sane polarity before returning.
        api.setExcitationPolarity(true);
        return;
      }
      v[s][p] = vs;
    }
  }

  // EMF-cancelled magnitudes (index 2 is the R_ref-top sense, on Vx2).
  double vx3, vx4, vxRef;
  if (cfg.emfCancel) {
    vx3   = (v[0][0] - v[0][1]) * 0.5;
    vx4   = (v[1][0] - v[1][1]) * 0.5;
    vxRef = (v[2][0] - v[2][1]) * 0.5;
  } else {
    vx3   = v[0][0];
    vx4   = v[1][0];
    vxRef = v[2][0];
  }

  double vDut = vx3 - vx4;
  double vRef = vxRef - vx3;

  if (vRef == 0.0 || !isfinite(vRef)) {
    io.println(F("ERROR: V_R_ref is zero or non-finite — check wiring & excitation polarity."));
    return;
  }

  double ratio = vDut / vRef;
  double rDut  = rRef * ratio;

  // --- Output ---------------------------------------------------------------
  io.println();
  io.print(F("R = ")); printOhms(io, rDut);
  io.println();

  for (int s = 0; s < kSenseCount; ++s) {
    io.print(F("  ")); io.print(kSenseName[s]); io.print(F(":  "));
    printSignedV(io, v[s][0], 8); io.print(F(" V"));
    if (cfg.emfCancel) {
      io.print(F(" / ")); printSignedV(io, v[s][1], 8); io.print(F(" V  ⇒  ±"));
      double mag = (s == 0 ? vx3 : (s == 1 ? vx4 : vxRef));
      io.print(fabs(mag), 8); io.print(F(" V"));
    }
    io.println();
  }
  io.print(F("  V_R_dut = ")); printSignedV(io, vDut, 8); io.print(F(" V"));
  io.print(F("   V_R_ref = ")); printSignedV(io, vRef, 8); io.print(F(" V"));
  io.print(F("   ratio = ")); io.print(ratio, 8); io.println();
  io.print(F("  R_ref = ")); io.print(rRef, 6); io.print(F(" Ω    cycles="));
  io.print(cfg.cycles); io.print(F("/polarity"));
  io.println();
}
