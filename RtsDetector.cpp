// RtsDetector.cpp — see RtsDetector.h for the algorithm.

#include <Arduino.h>
#include <math.h>

#include "RtsDetector.h"

void RtsDetector::begin(const Config& cfg) {
  cfg_ = cfg;
  reset();
}

void RtsDetector::reset() {
  initialized_     = false;
  ema_ = ema2_ = ema3_ = tema_ = 0.0;
  residualVar_     = 0.0;
  streakDir_       = 0;
  streakLen_       = 0;
  streakStartTema_ = 0.0;
  streakStartIdx_  = 0;
  deadCount_       = 0;
  sampleIdx_       = 0;
  eventCount_      = 0;
  logHead_         = 0;
  logCount_        = 0;
}

void RtsDetector::setEnabled(bool en) {
  if (en && !enabled_) reset();   // fresh start on each enable
  enabled_ = en;
}

bool RtsDetector::process(double sample, Event* out) {
  if (!enabled_) return false;
  sampleIdx_++;

  if (!initialized_) {
    // Seed the cascade to the first sample to avoid the cold-start ramp.
    ema_ = ema2_ = ema3_ = tema_ = sample;
    residualVar_ = 0.0;
    initialized_ = true;
    return false;
  }

  // ---- TEMA update ---------------------------------------------------------
  const double a   = cfg_.alpha;
  const double oma = 1.0 - a;
  ema_  = a * sample + oma * ema_;
  ema2_ = a * ema_   + oma * ema2_;
  ema3_ = a * ema2_  + oma * ema3_;
  tema_ = 3.0 * ema_ - 3.0 * ema2_ + ema3_;

  // ---- Residual + robust σ² ------------------------------------------------
  const double r     = sample - tema_;
  const double sigma = sqrt(residualVar_);
  double r_clipped = r;
  if (sigma > 0.0) {
    const double cap = 3.0 * sigma;
    if (r_clipped >  cap) r_clipped =  cap;
    if (r_clipped < -cap) r_clipped = -cap;
  }
  residualVar_ = (1.0 - cfg_.sigma_alpha) * residualVar_
               + cfg_.sigma_alpha * r_clipped * r_clipped;

  // ---- Dead-time gate ------------------------------------------------------
  if (deadCount_ > 0) {
    deadCount_--;
    return false;
  }

  // ---- Threshold + persistence --------------------------------------------
  if (sigma <= 0.0) return false;     // σ not yet established
  const double thresh = cfg_.k_sigma * sigma;
  int8_t dir = 0;
  if      (r >  thresh) dir = +1;
  else if (r < -thresh) dir = -1;

  if (dir == 0) {
    streakDir_ = 0;
    streakLen_ = 0;
    return false;
  }

  if (dir != streakDir_) {
    streakDir_       = dir;
    streakLen_       = 1;
    streakStartTema_ = tema_;            // pre-step baseline estimate
    streakStartIdx_  = sampleIdx_;
    return false;
  }

  streakLen_++;
  if (streakLen_ < cfg_.persistence_n) return false;

  // ---- Confirmed event -----------------------------------------------------
  Event ev;
  ev.sampleIdx   = streakStartIdx_;
  ev.valueBefore = streakStartTema_;
  ev.valueAfter  = sample;
  ev.magnitude   = sample - streakStartTema_;
  ev.direction   = dir;

  log_[logHead_] = ev;
  logHead_ = (logHead_ + 1) % MAX_LOG_EVENTS;
  if (logCount_ < MAX_LOG_EVENTS) logCount_++;
  eventCount_++;

  streakDir_ = 0;
  streakLen_ = 0;
  deadCount_ = cfg_.dead_samples;

  if (out) *out = ev;
  return true;
}

double RtsDetector::residualSigma() const {
  return sqrt(residualVar_);
}

const RtsDetector::Event& RtsDetector::logAt(uint8_t i) const {
  // Map 0=oldest → physical ring index.
  if (logCount_ == 0) return log_[0];
  if (i >= logCount_) i = logCount_ - 1;
  const uint8_t start = (logCount_ == MAX_LOG_EVENTS) ? logHead_ : 0;
  return log_[(start + i) % MAX_LOG_EVENTS];
}

void RtsDetector::printStatus(Stream& io) const {
  io.println(F("RtsDetector status:"));
  io.print(F("  enabled        : ")); io.println(enabled_ ? F("yes") : F("no"));
  io.print(F("  alpha (TEMA)   : ")); io.println(cfg_.alpha, 6);
  io.print(F("  k_sigma        : ")); io.println(cfg_.k_sigma, 2);
  io.print(F("  persistence_n  : ")); io.println(cfg_.persistence_n);
  io.print(F("  sigma_alpha    : ")); io.println(cfg_.sigma_alpha, 6);
  io.print(F("  dead_samples   : ")); io.println(cfg_.dead_samples);
  io.print(F("  samples seen   : ")); io.println(sampleIdx_);
  io.print(F("  events seen    : ")); io.println(eventCount_);
  io.print(F("  tema (V)       : ")); io.println(tema_, 10);
  io.print(F("  residual σ (V) : ")); io.println(residualSigma(), 10);
  io.print(F("  dead remaining : ")); io.println(deadCount_);
}

void RtsDetector::printEvents(Stream& io) const {
  if (logCount_ == 0) {
    io.println(F("RtsDetector: no events logged."));
    return;
  }
  io.print(F("RtsDetector: last ")); io.print(logCount_); io.println(F(" events (oldest first):"));
  for (uint8_t i = 0; i < logCount_; ++i) {
    const Event& ev = logAt(i);
    io.print(F("  #"));      io.print(i + 1);
    io.print(F(" @cycle ")); io.print(ev.sampleIdx);
    io.print(F("  dir "));   io.print(ev.direction > 0 ? '+' : '-');
    io.print(F("  ΔV = "));  io.print(ev.magnitude * 1e9, 3); io.print(F(" nV"));
    io.print(F("  (before ")); io.print(ev.valueBefore, 10);
    io.print(F(" → after ")); io.print(ev.valueAfter, 10);
    io.println(F(")"));
  }
}
