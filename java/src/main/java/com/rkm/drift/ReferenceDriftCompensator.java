/*
 * SPDX-License-Identifier: CERN-OHL-P-2.0
 * SPDX-FileCopyrightText: 2025 rkm
 */
package com.rkm.drift;

import com.rkm.Constants;
import com.rkm.model.FilterSample;

/**
 * Reference drift compensation using step response LUT and filter error prediction.
 * Port of firmware lines 2009-2171.
 *
 * The DAC reference comes from a filtered average of three ADR1001 references.
 * By measuring the pre-filter voltage (J3/VrefRaw) against the DAC reference (TP1),
 * we can predict and correct for temperature-induced reference drift in real-time.
 */
public class ReferenceDriftCompensator {

    private final FilterSample[] filterHistory = new FilterSample[Constants.FILTER_HISTORY_SIZE];
    private int filterHistoryIdx = 0;
    private double estimatedDriftRate = 0.0;
    private int refSampleCounter = 0;
    private boolean initialized = false;

    public ReferenceDriftCompensator() {
        initTracking();
    }

    /**
     * Initialize reference tracking state.
     * Port of firmware initRefTracking().
     */
    public void initTracking() {
        for (int i = 0; i < Constants.FILTER_HISTORY_SIZE; i++) {
            filterHistory[i] = new FilterSample(0.0, 0);
        }
        filterHistoryIdx = 0;
        estimatedDriftRate = 0.0;
        refSampleCounter = 0;
        initialized = false;
    }

    /**
     * Record a filter error measurement and update drift rate estimate.
     * Port of firmware measureFilterError() (the state-update portion).
     *
     * In the firmware, this function also temporarily switches the mux/DAC to
     * measure VrefRaw. In the simulation, the caller provides the measured error.
     *
     * @param filterError measured filter error (J3 - TP1) in volts
     * @param timestampMs current time in milliseconds
     */
    public void measureFilterError(double filterError, long timestampMs) {
        // Update drift rate estimate using previous measurement
        int prevIdx = (filterHistoryIdx - 1 + Constants.FILTER_HISTORY_SIZE) % Constants.FILTER_HISTORY_SIZE;

        if (filterHistory[prevIdx].timestampMs() > 0) {
            double dt = (timestampMs - filterHistory[prevIdx].timestampMs()) / 1000.0;
            if (dt > 0.1) {
                double prevError = filterHistory[prevIdx].error();
                // driftRate ≈ dError/dt + error/τ_eff
                double dError = filterError - prevError;
                estimatedDriftRate = dError / dt + prevError / Constants.REF_FILTER_TAU_EFF;
            }
        }

        // Store new measurement in history
        filterHistory[filterHistoryIdx] = new FilterSample(filterError, timestampMs);
        filterHistoryIdx = (filterHistoryIdx + 1) % Constants.FILTER_HISTORY_SIZE;

        initialized = true;
    }

    /**
     * Predict the current filter error based on historical measurements.
     * Port of firmware predictFilterError().
     *
     * Uses the known filter step response to extrapolate from the last measurement.
     *
     * @param currentTimeMs current time in milliseconds
     * @return predicted filter error (J3 - TP1) in volts
     */
    public double predictFilterError(long currentTimeMs) {
        if (!initialized) {
            return 0.0;
        }

        // Get most recent measurement
        int lastIdx = (filterHistoryIdx - 1 + Constants.FILTER_HISTORY_SIZE) % Constants.FILTER_HISTORY_SIZE;
        double lastError = filterHistory[lastIdx].error();
        long lastTime = filterHistory[lastIdx].timestampMs();

        if (lastTime == 0) {
            return 0.0;
        }

        double dt = (currentTimeMs - lastTime) / 1000.0;

        // Look up decay factor from step response table
        int idx = (int) (dt / Constants.REF_STEP_DT);
        double stepVal;
        if (idx >= Constants.REF_STEP_LEN) {
            stepVal = 1.0;
        } else if (idx < 0) {
            stepVal = 0.0;
        } else {
            // Linear interpolation between table entries
            double frac = (dt / Constants.REF_STEP_DT) - idx;
            if (idx + 1 < Constants.REF_STEP_LEN) {
                stepVal = Constants.REF_STEP_RESPONSE[idx]
                        + frac * (Constants.REF_STEP_RESPONSE[idx + 1] - Constants.REF_STEP_RESPONSE[idx]);
            } else {
                stepVal = Constants.REF_STEP_RESPONSE[idx];
            }
        }

        // Decay factor: how much of the original error remains
        double decayFactor = 1.0 - stepVal;
        double decayedError = lastError * decayFactor;

        // New error from continued drift
        double newError = estimatedDriftRate * Constants.REF_FILTER_TAU_EFF * stepVal;

        return decayedError + newError;
    }

    /**
     * Get the correction factor to apply to voltage measurements.
     * Port of firmware getRefCorrectionFactor().
     *
     * @param currentTimeMs current time in milliseconds
     * @return multiplicative correction factor (close to 1.0)
     */
    public double getCorrectionFactor(long currentTimeMs) {
        double filterErr = predictFilterError(currentTimeMs);
        return 1.0 + filterErr / Constants.NOMINAL_REF_V;
    }

    /**
     * Increment the sample counter and check if it's time to measure.
     * Port of the counter logic in firmware loop().
     *
     * @return true if it's time to take a reference measurement
     */
    public boolean shouldMeasure() {
        if (++refSampleCounter >= Constants.REF_SAMPLE_INTERVAL) {
            refSampleCounter = 0;
            return true;
        }
        return false;
    }

    public double getEstimatedDriftRate() { return estimatedDriftRate; }
    public boolean isInitialized() { return initialized; }
    public int getRefSampleCounter() { return refSampleCounter; }
}
