/*
 * SPDX-License-Identifier: CERN-OHL-P-2.0
 * SPDX-FileCopyrightText: Copyright 2025, 2026 Randall K McRee
 */
package com.rkm.measurement;

import com.rkm.Constants;
import com.rkm.hw.SimulatedADC;
import com.rkm.hw.SimulatedDAC;
import com.rkm.hw.SimulatedMux;
import com.rkm.math.LowerMoments;
import com.rkm.model.HalfCycleResult;

/**
 * Chopped measurement algorithm: acquireHalfCycle, chop, demodulation.
 * Port of firmware lines 948-1013, 2781-2806.
 */
public class ChopMeasurement {

    /**
     * Collect samples for one half-cycle, detect overflow on first sample.
     * Port of firmware acquireHalfCycle().
     *
     * @param adc the simulated ADC
     * @param adcInputVoltage voltage at ADC input for this half-cycle
     * @return HalfCycleResult with sum and overflow flag
     */
    public HalfCycleResult acquireHalfCycle(SimulatedADC adc, double adcInputVoltage) {
        // Discard initial samples after settling
        for (int i = 0; i < Constants.DISCARD_SAMPLES; i++) {
            adc.readSample(adcInputVoltage);
        }

        // Read first good sample and check for overflow
        int firstSample = adc.readSample(adcInputVoltage);
        if (SimulatedADC.isOverflow(firstSample)) {
            return new HalfCycleResult(0, true);
        }
        long sum = firstSample;

        // Accumulate remaining good samples
        for (int i = 1; i < Constants.GOOD_SAMPLES; i++) {
            sum += adc.readSample(adcInputVoltage);
        }
        return new HalfCycleResult(sum, false);
    }

    /**
     * Execute one complete chop cycle and accumulate the demodulated result.
     * Port of firmware runOneChopCycle().
     *
     * Phase A: current TMUX state
     * Phase B: toggled TMUX state
     * Demodulated = (sumA - sumB) / (2 × GOOD_SAMPLES)
     *
     * @param stats accumulator for demodulated results
     * @param adc simulated ADC
     * @param dac simulated DAC
     * @param mux simulated mux (includes TMUX state)
     * @param preampOffset preamp DC offset in ADC counts
     * @return true if successful, false if overflow
     */
    public boolean runOneChopCycle(LowerMoments stats, SimulatedADC adc,
                                    SimulatedDAC dac, SimulatedMux mux,
                                    double preampOffset) {
        double dacVoltage = dac.getOutputVoltage();

        // Phase A: current TMUX state
        double voltageA = mux.getAdcInputVoltage(dacVoltage, preampOffset);
        HalfCycleResult resultA = acquireHalfCycle(adc, voltageA);

        if (resultA.overflow()) {
            return false;
        }

        // Chop: toggle TMUX
        mux.chop();

        // Phase B: opposite TMUX state
        double voltageB = mux.getAdcInputVoltage(dacVoltage, preampOffset);
        HalfCycleResult resultB = acquireHalfCycle(adc, voltageB);

        if (resultB.overflow()) {
            mux.chop();  // Restore original state
            return false;
        }

        // Restore TMUX to original state
        mux.chop();

        // Demodulate: (sumA - sumB) / (2 × GOOD_SAMPLES)
        double demodulated = (double) (resultA.sum() - resultB.sum())
                / (2.0 * Constants.GOOD_SAMPLES);
        stats.accumulate(demodulated);
        return true;
    }
}
