/*
 * SPDX-License-Identifier: CERN-OHL-P-2.0
 * SPDX-FileCopyrightText: 2025 rkm
 */
package com.rkm.hw;

import com.rkm.Constants;
import org.apache.commons.math3.distribution.NormalDistribution;
import org.apache.commons.math3.random.Well19937c;

/**
 * Simulated ADS127L11 24-bit ADC.
 * Quantizes the input voltage to 24-bit codes and adds Gaussian noise.
 * Port of firmware ADC read logic (lines 870-937).
 */
public class SimulatedADC {

    private final NormalDistribution noiseDist;
    private final double noiseRms;

    public SimulatedADC(double noiseRms, long seed) {
        this.noiseRms = noiseRms;
        Well19937c rng = new Well19937c(seed);
        this.noiseDist = new NormalDistribution(rng, 0.0, noiseRms > 0 ? noiseRms : 1e-15,
                NormalDistribution.DEFAULT_INVERSE_ABSOLUTE_ACCURACY);
    }

    /**
     * Read a simulated ADC sample.
     * The ADC measures the input voltage (which is the preamp output)
     * and quantizes it to 24-bit codes.
     *
     * @param inputVoltage voltage at ADC input (preamp output) in volts
     * @return 24-bit ADC code (clamped to ±8388607)
     */
    public int readSample(double inputVoltage) {
        // Convert voltage to ideal code
        double idealCode = inputVoltage / Constants.ADC_LSB_V;

        // Add Gaussian noise
        double noisyCode = idealCode + noiseDist.sample();

        // Round and clamp to 24-bit range
        long code = Math.round(noisyCode);
        code = Math.max(Constants.ADC_MIN_CODE, Math.min(Constants.ADC_MAX_CODE, code));

        return (int) code;
    }

    /**
     * Check if a sample exceeds the overflow threshold.
     * Port of firmware isOverflow().
     */
    public static boolean isOverflow(int sample) {
        return isOverflowPositive(sample) || isOverflowNegative(sample);
    }

    public static boolean isOverflowPositive(int sample) {
        return sample > Constants.ADC_OVERFLOW_POS;
    }

    public static boolean isOverflowNegative(int sample) {
        return sample < Constants.ADC_OVERFLOW_NEG;
    }

    public double getNoiseRms() {
        return noiseRms;
    }
}
