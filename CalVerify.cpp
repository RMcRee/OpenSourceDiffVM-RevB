// CalVerify.cpp — implementation of the interactive Keithley-validated
// DAC calibration workflow. See CalVerify.h for the public interface and
// rationale; see the plan file for the design.

#include <Arduino.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "CalVerify.h"

// --- Exported defaults (also used by `cal smooth` in the .ino) -------------

// 25 default targets — dense across ±5 V at 1 V steps near rails plus 2.5 V
// cardinals, and sub-100 mV sampling down to ±1 mV near zero where offset
// and INL matter most for nV-scale work.
const double kCalVerifyDefaultTargets[] = {
  -5.0, -4.0, -3.0, -2.5, -2.0, -1.0, -0.5, -0.1, -0.05, -0.01,
  -0.005, -0.001, 0.0, 0.001, 0.005, 0.01, 0.05, 0.1, 0.5, 1.0,
  2.0, 2.5, 3.0, 4.0, 5.0,
};
const int kCalVerifyDefaultTargetCount =
    sizeof(kCalVerifyDefaultTargets) / sizeof(kCalVerifyDefaultTargets[0]);

namespace {

constexpr float  kDefaultThreshUv = 10.0f;
constexpr int    kDefaultCycles   = 285;   // ≈ 15 s at OSR=12800, samples=14
constexpr int    kMaxRetries      = 3;
constexpr int    kMaxOverrideTargets = 16;

// --- Code snapping ---------------------------------------------------------

// Snap a target voltage to the nearest 14-bit-aligned DAC code (multiple of 4
// in [-32768, 32764]). Special-cases the +32767 absolute-max slot for targets
// at or very near +5 V (within ½ LSB of the +32767 nominal).
int16_t snapToCardinalCode(double v_target, double dacLsbV) {
  // Special-case the +32767 absolute-max slot for targets within ½ LSB of it.
  double v_max = 32767.0 * dacLsbV;
  if (v_target >= v_max - 0.5 * dacLsbV) return 32767;
  // Round v_target to the nearest multiple-of-4 DAC code. v_target / (4 * LSB)
  // expresses the target in units of "4 DAC codes"; lround picks the nearest,
  // ×4 converts back to DAC codes.
  long snapped = (long)lround(v_target / (4.0 * dacLsbV)) * 4;
  if (snapped < -32768) snapped = -32768;
  if (snapped >  32764) snapped =  32764;
  return (int16_t)snapped;
}

// --- Blocking line read ----------------------------------------------------

enum class LineKind { Number, Skip, Abort, Quit, Invalid, Empty };

struct LineResult {
  LineKind kind;
  double   value;   // valid only when kind == Number
};

// Read one line of input from the stream, blocking. Handles CR/LF, parses
// single-letter commands (s/a/q) and floating-point voltages. Blank lines
// return Empty so caller can re-prompt without counting it as a retry.
LineResult readLine(Stream& io) {
  static char buf[64];
  size_t pos = 0;
  while (pos + 1 < sizeof(buf)) {
    while (!io.available()) { delay(1); /* yield */ }
    int c = io.read();
    if (c < 0) continue;
    if (c == '\r') continue;
    if (c == '\n') break;
    if (c == 0x1B /* ESC */) { return { LineKind::Abort, 0.0 }; }
    buf[pos++] = (char)c;
  }
  buf[pos] = '\0';

  // Trim leading whitespace
  char* p = buf;
  while (*p == ' ' || *p == '\t') ++p;
  if (*p == '\0') return { LineKind::Empty, 0.0 };

  // Single-letter commands (case-insensitive)
  if (p[1] == '\0' || p[1] == ' ' || p[1] == '\t') {
    char c = (char)tolower((unsigned char)p[0]);
    if (c == 's') return { LineKind::Skip,  0.0 };
    if (c == 'a') return { LineKind::Abort, 0.0 };
    if (c == 'q') return { LineKind::Quit,  0.0 };
  }

  // Numeric parse
  char* endp = nullptr;
  double v = strtod(p, &endp);
  if (endp == p) return { LineKind::Invalid, 0.0 };
  return { LineKind::Number, v };
}

// --- Argument parsing ------------------------------------------------------

struct VerifyConfig {
  float  threshUv;
  int    cycles;
  const double* targets;      // points at either kCalVerifyDefaultTargets or overrideBuf
  int    targetCount;
  double overrideBuf[kMaxOverrideTargets];
};

// Parse the optional "--thresh N", "--cycles N" flags and any trailing target
// voltages. argv tokens may be NULL once the dispatch ran out of args.
bool parseArgs(int argc, const char* const* argv, VerifyConfig& cfg, Stream& io) {
  cfg.threshUv    = kDefaultThreshUv;
  cfg.cycles      = kDefaultCycles;
  cfg.targets     = kCalVerifyDefaultTargets;
  cfg.targetCount = kCalVerifyDefaultTargetCount;
  int overrideN = 0;

  for (int i = 0; i < argc; ++i) {
    const char* a = argv[i];
    if (!a) continue;
    if (!strcasecmp(a, "--thresh") || !strcasecmp(a, "-t")) {
      if (i + 1 >= argc || !argv[i+1]) {
        io.println("ERROR: --thresh needs a µV value.");
        return false;
      }
      cfg.threshUv = (float)atof(argv[++i]);
      if (cfg.threshUv <= 0.0f || cfg.threshUv > 1e6f) {
        io.println("ERROR: --thresh out of range (try 1 to 1000).");
        return false;
      }
    } else if (!strcasecmp(a, "--cycles") || !strcasecmp(a, "-n")) {
      if (i + 1 >= argc || !argv[i+1]) {
        io.println("ERROR: --cycles needs an integer.");
        return false;
      }
      cfg.cycles = atoi(argv[++i]);
      if (cfg.cycles < 1 || cfg.cycles > 10000) {
        io.println("ERROR: --cycles out of range (1 to 10000).");
        return false;
      }
    } else {
      // Treat as a target voltage.
      if (overrideN >= kMaxOverrideTargets) {
        io.print("ERROR: too many override targets (max ");
        io.print(kMaxOverrideTargets); io.println(").");
        return false;
      }
      char* endp = nullptr;
      double v = strtod(a, &endp);
      if (endp == a) {
        io.print("ERROR: unrecognized argument: "); io.println(a);
        return false;
      }
      if (v < -5.5 || v > 5.5) {
        io.print("ERROR: target out of plausible DAC range: "); io.println(v, 6);
        return false;
      }
      cfg.overrideBuf[overrideN++] = v;
    }
  }
  if (overrideN > 0) {
    cfg.targets     = cfg.overrideBuf;
    cfg.targetCount = overrideN;
  }
  return true;
}

// --- Pretty print helpers --------------------------------------------------

void printVoltageSigned(Stream& io, double v, int decimals = 6) {
  if (v >= 0) io.print('+');
  io.print(v, decimals);
}

void printHeader(Stream& io, const VerifyConfig& cfg) {
  io.println();
  io.println(F("===== cal verify (Keithley-validated DAC cal) ====="));
  io.print  (F("Targets: ")); io.print(cfg.targetCount);
  io.print  (F("  Threshold: ")); io.print(cfg.threshUv, 1); io.print(F(" µV"));
  io.print  (F("  Cycles: "));    io.print(cfg.cycles);
  io.println();
  io.println(F("Per prompt: enter the Keithley reading (V), or:"));
  io.println(F("  s = skip this target   a = abort (no save)   q = quit + save"));
  io.println(F("=================================================="));
}

}  // namespace

// --- Main entry point ------------------------------------------------------

void cmdCalVerify(const CalVerifyApi& api, int argc, const char* const* argv) {
  Stream& io = *api.io;

  VerifyConfig cfg{};
  if (!parseArgs(argc, argv, cfg, io)) return;

  printHeader(io, cfg);
  api.selectCalInput();

  int updated = 0;
  int withinThresh = 0;
  int skipped = 0;
  bool quitAndSave = false;

  // Track the code of the last point we got a confirmed measurement on (either
  // Updated or Within-threshold). When the next confirmed point lands, we
  // blend the slots between the two so the cal table is smooth across each
  // pair of verify anchors. Skipped/garbled points don't move this anchor.
  int16_t lastConfirmedCode = INT16_MIN;

  for (int i = 0; i < cfg.targetCount; ++i) {
    double v_target = cfg.targets[i];
    int16_t code    = snapToCardinalCode(v_target, api.dacLsbV);
    double  v_nom   = (double)code * api.dacLsbV;

    // Apply DAC code (built-in settling).
    api.setDacCode(code);

    bool resolved = false;
    for (int retry = 0; retry < kMaxRetries && !resolved; ++retry) {
      io.println();
      io.print  (F("[")); io.print(i + 1); io.print(F("/"));
      io.print  (cfg.targetCount); io.print(F("] Target "));
      printVoltageSigned(io, v_target, 6); io.print(F(" V, DAC code "));
      io.print  (code); io.print(F(" (V_nom = "));
      printVoltageSigned(io, v_nom, 6); io.println(F(" V)"));
      io.print  (F("        Apply ~"));
      printVoltageSigned(io, v_target, 4);
      io.println(F(" V to DiffVM input; enter Keithley reading (V) or s/a/q:"));

      LineResult line = readLine(io);
      if (line.kind == LineKind::Empty) { --retry; continue; }
      if (line.kind == LineKind::Abort) {
        io.println(F("Aborted. Cal table changes (if any) are still in RAM."));
        io.println(F("Run 'cal load' to revert from flash, or 'cal save' to keep."));
        return;
      }
      if (line.kind == LineKind::Quit)  { quitAndSave = true; resolved = true; break; }
      if (line.kind == LineKind::Skip)  { skipped++; resolved = true; break; }
      if (line.kind == LineKind::Invalid) {
        io.println(F("  Could not parse a number. Try again, or s/a/q."));
        continue;
      }

      double v_keithley = line.value;
      if (v_keithley < -5.5 || v_keithley > 5.5) {
        io.println(F("  Reading out of plausible range (±5.5 V). Try again."));
        continue;
      }

      // Chop measurement at the deterministic DAC code — same path as a
      // normal scan reading, just integrated for a fixed cycle count.
      io.print(F("  Measuring ")); io.print(cfg.cycles);
      io.println(F(" chop cycles..."));
      bool overflow = false;
      double adcMean = api.runChopCycles(cfg.cycles, &overflow);
      if (overflow) {
        io.println(F("  PREAMP RAILED — set source closer to target, then try again."));
        continue;
      }

      double preampDelta = adcMean * api.adcLsbV / api.preampGain;
      double v_dvm       = api.calCodeToVoltage(code) + preampDelta;
      double err_uV      = (v_dvm - v_keithley) * 1.0e6;

      io.print  (F("  DiffVM = "));   printVoltageSigned(io, v_dvm, 9);
      io.print  (F(" V   Keithley = ")); printVoltageSigned(io, v_keithley, 9);
      io.print  (F(" V   diff = "));  printVoltageSigned(io, err_uV, 3);
      io.println(F(" µV"));

      if (fabs(err_uV) <= cfg.threshUv) {
        io.print  (F("  OK: within ±")); io.print(cfg.threshUv, 1);
        io.println(F(" µV — no update."));
        withinThresh++;
      } else {
        double v_old = api.calCodeToVoltage(code);
        double v_new = v_keithley - preampDelta;
        api.calSetCardinalPoint(code, v_new);
        io.print  (F("  Updated cardinal[")); io.print(code); io.print(F("]: was "));
        printVoltageSigned(io, v_old, 9); io.print(F(" V, now "));
        printVoltageSigned(io, v_new, 9); io.print(F(" V (Δ = "));
        printVoltageSigned(io, (v_new - v_old) * 1.0e6, 3); io.println(F(" µV)"));
        updated++;
      }
      // Both Updated and Within-threshold are "confirmed anchors". Blend the
      // slots between this anchor and the previous one so the table is smooth
      // across each verified pair. interpolateGaps() is intentionally NOT
      // called here — it's a no-op when the table is already dense (n>256)
      // and interpolateBetween is what gives us correct blending regardless.
      if (lastConfirmedCode != INT16_MIN) {
        int16_t lo = code < lastConfirmedCode ? code : lastConfirmedCode;
        int16_t hi = code < lastConfirmedCode ? lastConfirmedCode : code;
        api.calInterpolateBetween(lo, hi);
      }
      lastConfirmedCode = code;
      resolved = true;
    }

    if (!resolved) {
      io.println(F("  Max retries exceeded — skipping this target."));
      skipped++;
    }

    if (quitAndSave) break;
  }

  // Summary
  io.println();
  io.println(F("===== cal verify summary ====="));
  io.print  (F("  Updated:        ")); io.println(updated);
  io.print  (F("  Within thresh:  ")); io.println(withinThresh);
  io.print  (F("  Skipped:        ")); io.println(skipped);
  io.println(F("=============================="));
  if (updated > 0) {
    io.println(F("Cal table changed in RAM. Run 'cal save' to persist."));
  } else {
    io.println(F("No cal table changes."));
  }
}
