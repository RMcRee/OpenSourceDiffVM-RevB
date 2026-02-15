/*
 * SPDX-License-Identifier: CERN-OHL-P-2.0
 * SPDX-FileCopyrightText: 2025 rkm
 */
package com.rkm.sim;

/**
 * Configuration parameters for the DiffVM simulation.
 * Controls noise levels, drift rates, INL profile, and test voltages.
 */
public class SimulationConfig {

    /** Input voltage to simulate (volts) */
    private double inputVoltageVx = 1.0;

    /** ADC noise RMS in counts (typical ~8 counts for ADS127L11) */
    private double adcNoiseRms = 8.0;

    /** DAC INL error in ppm (relative to full scale) */
    private double dacInlPpm = 0.5;

    /** Reference drift rate (V/s). 0.0 = no drift. */
    private double refDriftRateVPerS = 0.0;

    /** Ground offset in volts (simulates residual offset after chopping) */
    private double gndOffset = 0.0;

    /** VrefRaw voltage (pre-filter ADR1001 average, nominally 5.0V) */
    private double vrefRawVoltage = 5.0;

    /** Preamp DC offset in ADC counts (cancelled by chopping) */
    private double preampOffsetCounts = 100.0;

    /** Random seed for reproducible tests */
    private long randomSeed = 42;

    // Getters and setters
    public double getInputVoltageVx() { return inputVoltageVx; }
    public void setInputVoltageVx(double v) { this.inputVoltageVx = v; }

    public double getAdcNoiseRms() { return adcNoiseRms; }
    public void setAdcNoiseRms(double rms) { this.adcNoiseRms = rms; }

    public double getDacInlPpm() { return dacInlPpm; }
    public void setDacInlPpm(double ppm) { this.dacInlPpm = ppm; }

    public double getRefDriftRateVPerS() { return refDriftRateVPerS; }
    public void setRefDriftRateVPerS(double rate) { this.refDriftRateVPerS = rate; }

    public double getGndOffset() { return gndOffset; }
    public void setGndOffset(double offset) { this.gndOffset = offset; }

    public double getVrefRawVoltage() { return vrefRawVoltage; }
    public void setVrefRawVoltage(double v) { this.vrefRawVoltage = v; }

    public double getPreampOffsetCounts() { return preampOffsetCounts; }
    public void setPreampOffsetCounts(double offset) { this.preampOffsetCounts = offset; }

    public long getRandomSeed() { return randomSeed; }
    public void setRandomSeed(long seed) { this.randomSeed = seed; }
}
