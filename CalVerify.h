// CalVerify — Keithley-validated interactive DAC calibration workflow.
//
// Walks the user through a list of target voltages, sets a deterministic
// 14-bit-aligned DAC code at each target, takes a chopped DiffVM measurement,
// compares against a user-typed Keithley reading, and updates the cal table
// entry only when the disagreement exceeds the threshold. Re-interpolates
// non-cardinal slots between adjacent cardinals after each update.
//
// This module is intentionally decoupled from the .ino: all interactions with
// the DAC, ADC, cal table, and mux happen through function pointers in
// CalVerifyApi. The .ino constructs the struct from local adapter functions
// and calls cmdCalVerify().

#pragma once
#include <stdint.h>
#include <stddef.h>

class Stream;

struct CalVerifyApi {
  // Set DAC code deterministically — applies built-in settling delay. NOT
  // binary search; same input code produces the same output every call.
  void   (*setDacCode)(int16_t code);

  // Run nCycles chop cycles at the current DAC code and return the mean ADC
  // count across the demodulated half-cycle differences. Sets *overflow=true
  // if any cycle railed the preamp; in that case the returned mean is
  // undefined.
  double (*runChopCycles)(int nCycles, bool* overflow);

  // Cal table — read the current entry (with interpolation) and overwrite a
  // 14-bit-aligned cardinal point.
  double (*calCodeToVoltage)(int16_t code);
  void   (*calSetCardinalPoint)(int16_t code, double vDac);

  // Re-interpolate all non-cardinal slots between adjacent cardinals. Returns
  // the number of cardinals found (purely informational).
  int    (*calInterpolateGaps)();

  // Linearly blend the slots strictly between two 14-bit-aligned codes,
  // leaving the endpoints alone. Called after each cardinal update so
  // newly-verified codes don't leave discontinuities surrounded by stale
  // cal-build-dac values.
  void   (*calInterpolateBetween)(int16_t codeLo, int16_t codeHi);

  // Switch the input mux to the calibration input channel (Vx2 — the
  // dedicated Keithley-reference port). Idempotent.
  void   (*selectCalInput)();

  // Constants the workflow uses for voltage arithmetic. Captured as values
  // (not pointers) since they shouldn't change within a verify session.
  double adcLsbV;
  double preampGain;
  double dacLsbV;

  // I/O — workflow prints prompts and reads response lines from this stream.
  Stream* io;
};

// Entry point. argv carries optional flags and an override target list:
//   cal verify
//   cal verify --thresh 5
//   cal verify --cycles 100
//   cal verify --thresh 5 --cycles 100 0.05 0.1 0.5
//   cal verify 0.05 0.1 0.5
void cmdCalVerify(const CalVerifyApi& api, int argc, const char* const* argv);

// The default 25-point target list. Exposed so the post-hoc `cal smooth`
// command can reuse the same target voltages without duplicating them.
extern const double kCalVerifyDefaultTargets[];
extern const int    kCalVerifyDefaultTargetCount;
