// OhmsMeas.cpp — see OhmsMeas.h for rationale and the public interface.

#include <Arduino.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "OhmsMeas.h"

namespace {

constexpr int  kDefaultCycles    = 75;      // ≈ 4 s per sense at OSR=12800, samples=14; → ~22 s per `meas r` (6 sense·pol combos). Override with --cycles N for higher precision.
constexpr int  kSenseCount       = 3;       // Vx3, Vx4, Vx2 (R_ref top)
constexpr uint32_t kPolaritySettleMs = 200; // V_exc + AAF settle after polarity flip
constexpr uint32_t kExcVoltageSettleMs = 200; // V_exc rail change + AAF settle
constexpr double   kLowExcThresholdOhms = 500.0; // R_ref below this → 1 V rail
constexpr double   kMinAbsVrefVolts     = 1.0e-4;  // 100 µV: below = R_ref sense path broken (DUT open or clip off)
constexpr double   kMaxAbsROhms         = 1.0e10;  // 10 GΩ ceiling; beyond is physically implausible for this design

const char* kSenseName[kSenseCount] = { "Vx3", "Vx4", "Vx2" };

struct OhmsConfig {
  int     cycles;
  bool    emfCancel;
  int     repeats;          // number of full measurements; >1 enables stats summary
  int8_t  excOverride;      // 0 = auto by R_ref, 1 = force 1.0 V, 2 = force 2.5 V
};

bool parseArgs(int argc, const char* const* argv, OhmsConfig& cfg, Stream& io) {
  cfg.cycles      = kDefaultCycles;
  cfg.emfCancel   = true;
  cfg.repeats     = 1;
  cfg.excOverride = 0;

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
    } else if (!strcasecmp(a, "--repeat") || !strcasecmp(a, "-r")) {
      if (i + 1 >= argc || !argv[i+1]) {
        io.println(F("ERROR: --repeat needs an integer."));
        return false;
      }
      cfg.repeats = atoi(argv[++i]);
      if (cfg.repeats < 1 || cfg.repeats > 1000) {
        io.println(F("ERROR: --repeat out of range (1 to 1000)."));
        return false;
      }
    } else if (!strcasecmp(a, "--exc")) {
      if (i + 1 >= argc || !argv[i+1]) {
        io.println(F("ERROR: --exc needs a value (1 or 2.5)."));
        return false;
      }
      double v = atof(argv[++i]);
      if      (v > 0.5 && v < 1.5) cfg.excOverride = 1;  // → 1.0 V rail
      else if (v > 2.0 && v < 3.0) cfg.excOverride = 2;  // → 2.5 V rail
      else {
        io.println(F("ERROR: --exc must be 1 or 2.5 (only those two rails exist)."));
        return false;
      }
    } else {
      io.print(F("ERROR: unrecognized argument: ")); io.println(a);
      io.println(F("Usage: meas r [--cycles N] [--no-emf-cancel] [--repeat N] [--exc 1|2.5]"));
      return false;
    }
  }
  return true;
}

// Per-(sense, polarity) DAC code cache. After a successful BinSrch, the
// converged code is stashed here and reused on subsequent calls — skipping
// the full BinSrch entirely (which can fail when many consecutive
// same-direction railed iterations saturate the AAF; see CHOP_SETTLE
// notes in binarySearchDAC). On overflow during a cached integration
// (the cached code is stale) the entry is invalidated and we fall back
// to a full BinSrch.
//
// Invalidated en masse at the start of each cmdMeasR when R_ref or V_exc
// changes (those shift every Vx value, so all cached codes are wrong).
static int16_t s_dacCache[kSenseCount][2]      = { {0,0}, {0,0}, {0,0} };
static bool    s_dacCacheValid[kSenseCount][2] = { {false,false}, {false,false}, {false,false} };

static void invalidateDacCache() {
  for (int s = 0; s < kSenseCount; ++s) {
    for (int p = 0; p < 2; ++p) s_dacCacheValid[s][p] = false;
  }
}

// Measure Vx at the current channel for the current polarity. Uses the
// per-(sense, polarity) cached DAC code when valid (skips BinSrch entirely);
// otherwise runs a full BinSrch and caches the converged code.
// Returns NAN on failure.
double measureSenseVolts(const OhmsMeasApi& api, int cycles, bool* outOverflow,
                         int senseIdx, int polIdx) {
  if (outOverflow) *outOverflow = false;

  // Cache hit: pre-set DAC, integrate directly. If overflow, the cached
  // code is stale (Vx drifted out of the preamp linear range); invalidate
  // and fall through to the BinSrch path.
  if (s_dacCacheValid[senseIdx][polIdx]) {
    api.setDacCode(s_dacCache[senseIdx][polIdx]);
    bool ovf = false;
    double adcMean = api.runChopCycles(cycles, &ovf);
    if (!ovf) {
      double vdac  = api.getCurrentDacVoltage();
      double delta = adcMean * api.adcLsbV / api.preampGain;
      return vdac + delta;
    }
    s_dacCacheValid[senseIdx][polIdx] = false;  // stale; fall back to BinSrch
  }

  // Cache miss (or no cache yet): full BinSrch, then cache the result.
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
  s_dacCache[senseIdx][polIdx]      = api.getCurrentDacCode();
  s_dacCacheValid[senseIdx][polIdx] = true;

  double vdac  = api.getCurrentDacVoltage();
  double delta = adcMean * api.adcLsbV / api.preampGain;
  return vdac + delta;
}

// SI-prefix print with `sigDigits` significant digits regardless of unit.
// Default 9 matches 8.5-9.5 digit DMM display precision. Use ~4 for sd and
// similar uncertainty quantities (a sample-sd estimate's own uncertainty
// is ~25% at n=9, so more digits than that are spurious).
void printOhms(Stream& io, double r, int sigDigits = 9) {
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
  const double scaled = r * scale;
  const double absScaled = fabs(scaled);
  int decimals = sigDigits;
  if (absScaled >= 1.0) {
    const int digitsBefore = (int)floor(log10(absScaled)) + 1;
    decimals = sigDigits - digitsBefore;
    if (decimals < 0) decimals = 0;
  }
  io.print(scaled, decimals); io.print(' '); io.print(unit);
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

  // Select excitation magnitude: --exc override wins over auto-select.
  bool lowExc;
  if      (cfg.excOverride == 1) lowExc = true;
  else if (cfg.excOverride == 2) lowExc = false;
  else                           lowExc = (rRef < kLowExcThresholdOhms);
  api.setExcitationVoltage(lowExc);
  delay(kExcVoltageSettleMs);

  // Invalidate the DAC-code cache when R_ref or V_exc changes — those
  // shift every Vx value so all cached per-sense codes are stale.
  static double s_cachedRref     = 0.0;
  static bool   s_cachedLowExc   = false;
  if (rRef != s_cachedRref || lowExc != s_cachedLowExc) {
    invalidateDacCache();
    s_cachedRref   = rRef;
    s_cachedLowExc = lowExc;
  }

  // Header (printed once)
  io.println();
  io.print(F("meas r: R_ref = ")); io.print(rRef, 6); io.print(F(" Ω"));
  io.print(F("  V_exc=")); io.print(lowExc ? F("1.0 V") : F("2.5 V"));
  if (cfg.excOverride) io.print(F(" [forced]"));
  io.print(F("  cycles=")); io.print(cfg.cycles);
  if (!cfg.emfCancel) io.print(F("  [no EMF cancel]"));
  if (cfg.repeats > 1) { io.print(F("  repeat=")); io.print(cfg.repeats); }
  io.println();

  const bool verbose = (cfg.repeats == 1);
  const int  polCount = cfg.emfCancel ? 2 : 1;
  const bool polVal[2] = { true, false };

  // Welford accumulators for the repeat-mode summary.
  double mean = 0.0, m2 = 0.0;
  double rMin = INFINITY, rMax = -INFINITY;
  int    nGood = 0;

  for (int rep = 0; rep < cfg.repeats; ++rep) {
    // Storage: voltages[sense][polarity]   sense: 0=Vx3, 1=Vx4, 2=Vx2;  pol: 0=+, 1=−
    double v[kSenseCount][2];
    for (int i = 0; i < kSenseCount; ++i) v[i][0] = v[i][1] = NAN;

    bool aborted = false;
    for (int p = 0; p < polCount && !aborted; ++p) {
      api.setExcitationPolarity(polVal[p]);
      delay(kPolaritySettleMs);
      for (int s = 0; s < kSenseCount; ++s) {
        api.selectSense((uint8_t)s);
        bool ovf = false;
        double vs = measureSenseVolts(api, cfg.cycles, &ovf, s, p);
        if (ovf || !isfinite(vs)) {
          if (!verbose) { io.print(F("  run ")); io.print(rep + 1); io.print(F(": ")); }
          io.print(kSenseName[s]); io.print(F(" @ pol ")); io.print(polVal[p] ? '+' : '-');
          io.println(F(": MEASUREMENT FAILED (preamp railed or BinSrch fail)."));
          aborted = true;
          break;
        }
        v[s][p] = vs;
      }
    }
    if (aborted) {
      if (verbose) {
        // Single-shot mode: restore a sane polarity and exit via cleanup.
        api.setExcitationPolarity(true);
        goto cleanup;
      }
      continue;  // multi-run: skip this iteration but keep going
    }

    // EMF-cancelled magnitudes (index 2 = R_ref top sense on Vx2).
    double vx3, vx4, vxRef;
    if (cfg.emfCancel) {
      vx3   = (v[0][0] - v[0][1]) * 0.5;
      vx4   = (v[1][0] - v[1][1]) * 0.5;
      vxRef = (v[2][0] - v[2][1]) * 0.5;
    } else {
      vx3 = v[0][0]; vx4 = v[1][0]; vxRef = v[2][0];
    }
    const double vDut  = vx3 - vx4;
    const double vRef  = vxRef - vx3;
    if (vRef == 0.0 || !isfinite(vRef)) {
      io.println(F("ERROR: V_R_ref is zero or non-finite — check wiring & excitation polarity."));
      if (verbose) goto cleanup;
      continue;
    }
    const double ratio = vDut / vRef;
    const double rDut  = rRef * ratio;

    // Physical-plausibility checks: catch fallen Kelvin clips, open DUTs,
    // sense lines disconnected, etc. Abort the whole batch on detection —
    // these failure modes are usually persistent, so continuing wastes time.
    const __FlashStringHelper* invalidReason = nullptr;
    if (fabs(vRef) < kMinAbsVrefVolts) {
      invalidReason = F("|V_R_ref| < 100 µV — R_ref sense disconnected, R_ref unwired, or DUT open?");
    } else if (!isfinite(rDut)) {
      invalidReason = F("R is non-finite (computation failure)");
    } else if (rDut < 0.0) {
      invalidReason = F("R is negative — Kelvin sense leads swapped, or a contact opened mid-measurement?");
    } else if (rDut > kMaxAbsROhms) {
      invalidReason = F("R exceeds 10 GΩ ceiling — likely an open circuit somewhere in the chain");
    }
    if (invalidReason) {
      io.print(F("ABORT: ")); io.println(invalidReason);
      io.print(F("       V_R_dut=")); io.print(vDut, 6);
      io.print(F("   V_R_ref=")); io.print(vRef, 6);
      io.print(F("   R=")); printOhms(io, rDut); io.println();
      break;  // exit the for-rep loop; falls through to summary (which prints partial stats)
    }

    // Per-run output
    if (verbose) {
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
    } else {
      io.print(F("  run ")); io.print(rep + 1); io.print('/'); io.print(cfg.repeats);
      io.print(F(": R = ")); printOhms(io, rDut);
      io.print(F("   V_dut=")); io.print(vDut, 6);
      io.print(F("   V_ref=")); io.print(vRef, 6);
      if (cfg.repeats > 1 && rep == 0) io.print(F("   [primer, excluded from stats]"));
      io.println();
    }

    // Discard the first run in repeat-mode as a thermal primer: when V_exc
    // steps from idle-1V to measurement-2.5V on meas-r entry, R_ref + DUT
    // self-heat for ~1-3 s (SMT thermal time constant). Run 1 captures the
    // transient; runs 2..N capture the steady state.
    if (cfg.repeats > 1 && rep == 0) continue;

    // Welford stats update
    nGood++;
    const double delta = rDut - mean;
    mean += delta / nGood;
    m2   += delta * (rDut - mean);
    if (rDut < rMin) rMin = rDut;
    if (rDut > rMax) rMax = rDut;
  }

  // Summary footer (repeat-mode only)
  if (cfg.repeats > 1) {
    io.println();
    if (nGood == 0) {
      io.println(F("No counted runs completed; no statistics."));
      goto cleanup;
    }
    const double sd        = (nGood > 1) ? sqrt(m2 / (double)(nGood - 1)) : 0.0;
    const double meanAbs   = fabs(mean);
    const double sdPpm     = (meanAbs > 0.0) ? (sd / meanAbs) * 1.0e6 : 0.0;
    const double rangeOhms = rMax - rMin;
    const double rangePpm  = (meanAbs > 0.0) ? (rangeOhms / meanAbs) * 1.0e6 : 0.0;

    io.print(F("R = ")); printOhms(io, mean);
    if (nGood > 1) {
      io.print(F("  ± ")); printOhms(io, sd, 4);
      io.print(F(" sd ("));  io.print(sdPpm, 3); io.print(F(" ppm)"));
    }
    io.println();
    io.print(F("  n=")); io.print(nGood); io.print('/'); io.print(cfg.repeats);
    io.print(F("  mean=")); io.print(mean, 6); io.print(F(" Ω"));
    if (nGood > 1) {
      io.print(F("  sd=")); printOhms(io, sd, 4);
      io.print(F("  min=")); io.print(rMin, 6);
      io.print(F("  max=")); io.print(rMax, 6);
      io.print(F("  range=")); io.print(rangePpm, 3); io.print(F(" ppm"));
    }
    io.println();
    io.print(F("  R_ref = ")); io.print(rRef, 6); io.print(F(" Ω    cycles="));
    io.print(cfg.cycles); io.print(F("/polarity"));
    io.println();
  }

cleanup:
  // Drop V_exc to 1 V between measurements so the breakout idles at
  // ~6× lower continuous dissipation. The next `meas r` re-escalates
  // (or not) based on its R_ref selection.
  api.setExcitationVoltage(true);
}
