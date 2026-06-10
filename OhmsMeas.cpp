// OhmsMeas.cpp — see OhmsMeas.h for rationale and the public interface.

#include <Arduino.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "OhmsMeas.h"

namespace {

constexpr int  kDefaultCycles    = 150;     // ≈ 8 s per sense at OSR=12800, samples=14; with default --repeat 5 → ~210 s per `meas r` with primer-shortening, sd ≈ 0.5 ppm on cert resistors. Sweet spot for sd/time per the optimum k=√(N/30) analysis.
constexpr int  kSenseCount       = 3;       // Vx3, Vx4, Vx2 (R_ref top)
constexpr uint32_t kPolaritySettleMs = 200; // V_exc + AAF settle after polarity flip
constexpr uint32_t kExcVoltageSettleMs = 200; // V_exc rail change + AAF settle
constexpr double   kLowExcThresholdOhms = 500.0; // R_ref below this → 1 V rail
constexpr double   kMinAbsVrefVolts     = 1.0e-4;  // 100 µV: below = R_ref sense path broken (DUT open or clip off)
constexpr double   kMaxAbsROhms         = 1.0e10;  // 10 GΩ ceiling; beyond is physically implausible for this design

const char* kSenseName[kSenseCount] = { "Vx3", "Vx4", "Vx2" };

// Acquisition order: low-impedance, predictable nodes first (Vx4 ≈ 0 V,
// Vx2 ≈ ±V_exc), then the hard one (Vx3: unknown voltage AND high source
// impedance R_ref ∥ R_dut). By the time Vx3 is attacked, the measured
// Vx2/Vx4 of the same polarity are available to seed its search.
const uint8_t kAcqOrder[kSenseCount] = { 1, 2, 0 };

// Seeded-search brackets (DAC codes; LSB ≈ 152.6 µV).
constexpr int32_t kMirrorHalfBracket = 64;   // opposite-polarity mirror: EMF ~60 µV ≈ 0.4 codes + offset/INL margin
constexpr int32_t kVx4HalfBracket    = 256;  // Vx4 ≈ 0 V + wiring drops (µV-scale)
constexpr int32_t kVx2HalfBracket    = 768;  // Vx2 ≈ ±V_exc nominal; covers ~±2 % rail tolerance
constexpr double  kHighRrefOhms      = 30.0e3;  // above this, cold full-range search gets the AAF-reset variant
// Front-end stability ceiling: the AD8428 bank is regenerative when the
// sense-node source impedance R_ref ∥ R_dut exceeds ~25 kΩ (input-current
// feedback through Z_src; static transfer compresses ~16×, chopping excites a
// limit cycle). Measured 2026-06-10 via dacsweep at r5 + 100 kΩ; see
// OhmsMeasurement.md "high source impedance ceiling".
constexpr double  kZsrcStabilityOhms = 25.0e3;

struct OhmsConfig {
  int     cycles;
  bool    emfCancel;
  int     repeats;          // number of full measurements; >1 enables stats summary
  int8_t  excOverride;      // 0 = auto by R_ref, 1 = force 1.0 V, 2 = force 2.5 V
  double  nominalOhms;      // user's rough DUT estimate (0 = unset); seeds the Vx3 search
};

// "470", "200k", "1.5M", "2G" → ohms; NAN if unparseable. 'm' counts as mega
// too — milliohm nominals don't occur at seed-bracket resolution.
double parseOhmsValue(const char* s) {
  if (!s) return NAN;
  char* end = nullptr;
  double v = strtod(s, &end);
  if (end == s) return NAN;
  if      (*end == 'k' || *end == 'K') { v *= 1.0e3; ++end; }
  else if (*end == 'M' || *end == 'm') { v *= 1.0e6; ++end; }
  else if (*end == 'G' || *end == 'g') { v *= 1.0e9; ++end; }
  if (*end != '\0') return NAN;
  return v;
}

bool parseArgs(int argc, const char* const* argv, OhmsConfig& cfg, Stream& io) {
  cfg.cycles      = kDefaultCycles;
  cfg.emfCancel   = true;
  cfg.repeats     = 5;            // default to repeat-mode with primer; gives sd ≈ 0.5 ppm at cycles=150
  cfg.excOverride = 0;
  cfg.nominalOhms = 0.0;

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
    } else if (!strcasecmp(a, "--nominal")) {
      if (i + 1 >= argc || !argv[i+1]) {
        io.println(F("ERROR: --nominal needs a resistance (e.g. 470, 200k, 1.5M)."));
        return false;
      }
      double r = parseOhmsValue(argv[++i]);
      if (!isfinite(r) || r <= 0.0 || r > 1.0e12) {
        io.println(F("ERROR: --nominal could not be parsed or is out of range."));
        return false;
      }
      cfg.nominalOhms = r;
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
      io.println(F("Usage: meas r [--cycles N] [--no-emf-cancel] [--repeat N] [--exc 1|2.5] [--nominal R]"));
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

// Measure Vx at the current channel for the current polarity. Seed ladder,
// first success wins:
//   1. cached DAC code for this (sense, polarity) — integrate directly
//   2. polarity mirror: negated opposite-polarity cache, tight bracket
//   3. physics seed (caller-supplied predicted voltage + bracket)
//   4. cold fallback: full-range search (AAF-reset variant at high R_ref)
// Every successful search caches its converged code. Returns NAN on failure.
double measureSenseVolts(const OhmsMeasApi& api, int cycles, bool* outOverflow,
                         int senseIdx, int polIdx,
                         double seedVolts, int32_t seedHalfBracket, double rRef) {
  if (outOverflow) *outOverflow = false;

  // Integrate at the current DAC code; cache it on success. (Recovery from
  // the static dwells around the search — DAC settle, channel switch — is the
  // runChopCycles adapter's flush cycles, not a delay here: at high source
  // impedance the node only re-centers *while chopping*.)
  auto integrateAndCache = [&](double& out) -> bool {
    bool ovf = false;
    double adcMean = api.runChopCycles(cycles, &ovf);
    if (ovf) return false;
    s_dacCache[senseIdx][polIdx]      = api.getCurrentDacCode();
    s_dacCacheValid[senseIdx][polIdx] = true;
    out = api.getCurrentDacVoltage() + adcMean * api.adcLsbV / api.preampGain;
    return true;
  };
  double result;

  // 1. Cache hit: pre-set DAC, integrate directly. If overflow, the cached
  // code is stale (Vx drifted out of the preamp linear range); invalidate
  // and fall down the ladder.
  if (s_dacCacheValid[senseIdx][polIdx]) {
    api.setDacCode(s_dacCache[senseIdx][polIdx]);
    if (integrateAndCache(result)) return result;
    s_dacCacheValid[senseIdx][polIdx] = false;
  }

  // 2. Polarity mirror: Vx(−pol) ≈ −Vx(+pol) (thermal EMF asymmetry is
  // sub-code), so the opposite polarity's converged code negated is a
  // near-exact seed.
  const int oppPol = 1 - polIdx;
  if (s_dacCacheValid[senseIdx][oppPol]) {
    int32_t mirror = -(int32_t)s_dacCache[senseIdx][oppPol];
    if (mirror > 32767) mirror = 32767;   // -(-32768)
    if (api.binarySearchDacSeeded((int16_t)mirror, kMirrorHalfBracket) &&
        integrateAndCache(result)) {
      return result;
    }
  }

  // 3. Physics seed from the caller (predicted node voltage).
  if (isfinite(seedVolts) && seedHalfBracket > 0) {
    if (api.binarySearchDacSeeded(api.dacVoltsToCode(seedVolts), seedHalfBracket) &&
        integrateAndCache(result)) {
      return result;
    }
  }

  // 4. Cold full-range fallback. At high R_ref use the seeded entry point for
  // its GND+null AAF reset — plain full-range bisection cannot converge there
  // (cumulative AAF saturation; see OhmsMeasurement.md "r5 high-ρ limit").
  const bool ok = (rRef >= kHighRrefOhms)
      ? api.binarySearchDacSeeded(0, 65536)
      : api.binarySearchDac();
  if (!ok || !integrateAndCache(result)) {
    if (outOverflow) *outOverflow = true;
    return NAN;
  }
  return result;
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

  // Pre-flight stability check when the operator told us the DUT size.
  if (cfg.nominalOhms > 0.0) {
    const double zSrc = rRef * cfg.nominalOhms / (rRef + cfg.nominalOhms);
    if (zSrc > kZsrcStabilityOhms) {
      io.println(F("WARNING: R_ref ∥ R_dut(nominal) exceeds the ~25 kΩ front-end stability"));
      io.println(F("         ceiling — the preamp limit-cycles at this source impedance and"));
      io.println(F("         the Vx3 search will not converge. Use a smaller R_ref: r4 (20 kΩ)"));
      io.println(F("         keeps R_ref ∥ R_dut < 20 kΩ for ANY DUT. Proceeding anyway..."));
    }
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
  if (cfg.nominalOhms > 0.0) { io.print(F("  nominal=")); printOhms(io, cfg.nominalOhms, 4); }
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
    // Primer (rep=0 in repeat mode) only needs to let R_ref + DUT reach
    // thermal equilibrium under load — the precision integration happens in
    // runs 2..N. Half-cycles for the primer cuts ~10% off total wall-clock
    // without changing the counted-run sd.
    const bool isPrimer = (cfg.repeats > 1 && rep == 0);
    const int  thisCycles = isPrimer
        ? (cfg.cycles / 2 > 20 ? cfg.cycles / 2 : 20)
        : cfg.cycles;

    // Storage: voltages[sense][polarity]   sense: 0=Vx3, 1=Vx4, 2=Vx2;  pol: 0=+, 1=−
    double v[kSenseCount][2];
    for (int i = 0; i < kSenseCount; ++i) v[i][0] = v[i][1] = NAN;

    bool aborted = false;
    for (int p = 0; p < polCount && !aborted; ++p) {
      api.setExcitationPolarity(polVal[p]);
      delay(kPolaritySettleMs);
      const double polSign = polVal[p] ? 1.0 : -1.0;
      for (int k = 0; k < kSenseCount; ++k) {
        const int s = kAcqOrder[k];   // Vx4, Vx2, then Vx3 (see kAcqOrder)
        api.selectSense((uint8_t)s);

        // Physics seed for the ladder in measureSenseVolts (rung 3).
        double  seedV  = NAN;
        int32_t halfBr = 0;
        if (s == 1) {                 // Vx4: DUT low Kelvin ≈ ground
          seedV  = 0.0;
          halfBr = kVx4HalfBracket;
        } else if (s == 2) {          // Vx2: R_ref top ≈ ±V_exc (SW1 drop is µV)
          seedV  = polSign * (lowExc ? 1.0 : 2.5);
          halfBr = kVx2HalfBracket;
        } else if (cfg.nominalOhms > 0.0 &&
                   isfinite(v[1][p]) && isfinite(v[2][p])) {
          // Vx3 from the divider: Vx3 = Vx4 + (Vx2−Vx4)·R_dut/(R_ref+R_dut),
          // with this polarity's measured Vx2/Vx4 and the user's nominal R_dut.
          // Bracket spans the divider evaluated at R_nom×[0.7, 1.4] + margin.
          const double vx4  = v[1][p], vx2 = v[2][p];
          const double rN   = cfg.nominalOhms;
          const double frac   = rN / (rRef + rN);
          const double fracLo = (0.7 * rN) / (rRef + 0.7 * rN);
          const double fracHi = (1.4 * rN) / (rRef + 1.4 * rN);
          seedV  = vx4 + (vx2 - vx4) * frac;
          halfBr = (int32_t)(fabs(vx2 - vx4) * (fracHi - fracLo) * 0.5 / api.dacLsbV) + 64;
        }

        bool ovf = false;
        double vs = measureSenseVolts(api, thisCycles, &ovf, s, p, seedV, halfBr, rRef);
        if (ovf || !isfinite(vs)) {
          if (!verbose) { io.print(F("  run ")); io.print(rep + 1); io.print(F(": ")); }
          io.print(kSenseName[s]); io.print(F(" @ pol ")); io.print(polVal[p] ? '+' : '-');
          io.println(F(": MEASUREMENT FAILED (preamp railed or BinSrch fail)."));
          if (s == 0 && rRef >= kHighRrefOhms) {
            io.println(F("  note: Vx3 failure at high R_ref usually means R_ref ∥ R_dut exceeds the"));
            io.println(F("  ~25 kΩ front-end stability ceiling — switch to a smaller R_ref (r4 = 20 kΩ"));
            io.println(F("  works for any DUT; ρ > 1 costs noise, not validity)."));
            if (cfg.nominalOhms <= 0.0) {
              io.println(F("  Supplying --nominal <approx DUT ohms> seeds the search and enables the"));
              io.println(F("  stability pre-flight check."));
            }
          }
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
      if (isPrimer) { io.print(F("   [primer @ ")); io.print(thisCycles); io.print(F(" cycles, excluded from stats]")); }
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
