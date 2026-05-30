// RtsDetector — TEMA-based detector for random-telegraph-signal (RTS) events
// in the per-chop-cycle Vx stream.
//
// Algorithm:
//   1. TEMA smoother      : y = 3·EMA − 3·EMA(EMA) + EMA(EMA(EMA))
//   2. Residual r = x − y
//   3. EMA of clipped r²  : robust running estimate of noise σ²
//   4. Persistence gate   : same-sign |r| > k·σ for N consecutive samples → event
//   5. Dead time          : suppress detection for ~5/α samples after an event
//                           so TEMA ring-down does not double-trigger
//
// The module is self-contained: feed it one sample per chop cycle via
// process(); poll enabled()/eventCount()/printEvents() for state.
//
// Reference: see the discussion of TEMA frequency-response peaking
// (~+2 dB at the corner) for why the persistence + dead-time guards exist.

#pragma once

#include <stdint.h>

class Stream;

class RtsDetector {
public:
  struct Config {
    double   alpha          = 0.02;   // TEMA α (per-sample smoother coefficient)
    double   k_sigma        = 4.5;    // event threshold = k_sigma · σ_residual
    uint8_t  persistence_n  = 4;      // consecutive same-sign threshold crossings
    double   sigma_alpha    = 0.001;  // EMA α for σ² estimator (clipped, slow)
    uint16_t dead_samples   = 250;    // post-event suppression window (≈ 5/α)
  };

  struct Event {
    uint32_t sampleIdx;    // sample index where the confirming streak began
    double   valueBefore;  // TEMA value at streak start (best estimate of pre-step level)
    double   valueAfter;   // sample at confirm (best estimate of post-step level)
    double   magnitude;    // signed: valueAfter − valueBefore
    int8_t   direction;    // +1 / −1
  };

  static constexpr uint8_t MAX_LOG_EVENTS = 32;

  void begin(const Config& cfg);
  void reset();              // clear state, keep config and enabled flag intact
  void setEnabled(bool en);  // toggles; transition to enabled also resets state
  bool enabled() const { return enabled_; }

  // Feed one per-chop-cycle sample (volts at input).
  // Returns true when a new event is confirmed *this* call.
  // If `out` is non-null and the return value is true, *out is populated.
  bool process(double sample, Event* out = nullptr);

  // Read-only inspection
  double         tema()            const { return tema_; }
  double         residualSigma()   const;
  uint32_t       sampleCount()     const { return sampleIdx_; }
  uint32_t       eventCount()      const { return eventCount_; }
  uint8_t        logCount()        const { return logCount_; }
  const Event&   logAt(uint8_t i)  const;   // 0 = oldest in ring; clamped if out of range
  const Config&  config()          const { return cfg_; }
  Config&        mutableConfig()         { return cfg_; }

  void printStatus(Stream& io) const;
  void printEvents(Stream& io) const;

private:
  Config   cfg_;
  bool     enabled_     = false;
  bool     initialized_ = false;
  double   ema_         = 0.0;
  double   ema2_        = 0.0;
  double   ema3_        = 0.0;
  double   tema_        = 0.0;
  double   residualVar_ = 0.0;

  int8_t   streakDir_       = 0;
  uint8_t  streakLen_       = 0;
  double   streakStartTema_ = 0.0;
  uint32_t streakStartIdx_  = 0;

  uint16_t deadCount_       = 0;
  uint32_t sampleIdx_       = 0;
  uint32_t eventCount_      = 0;

  Event    log_[MAX_LOG_EVENTS];
  uint8_t  logHead_         = 0;   // next-write slot
  uint8_t  logCount_        = 0;   // 0..MAX_LOG_EVENTS
};
