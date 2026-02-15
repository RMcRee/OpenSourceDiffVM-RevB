/*
 * SPDX-License-Identifier: CERN-OHL-P-2.0
 * SPDX-FileCopyrightText: Copyright 2025, 2026 Randall K McRee
 */
package com.rkm.hw;

import com.rkm.Constants;
import com.rkm.cal.DacCalibrationTable;

/**
 * Simulated AD5760 16-bit DAC.
 * Tracks current code, computes output voltage, and calculates settling time.
 * Port of firmware DAC functions (lines 559-776).
 */
public class SimulatedDAC {

    private short currentCode = 0;
    private final DacCalibrationTable calTable;

    public SimulatedDAC(DacCalibrationTable calTable) {
        this.calTable = calTable;
    }

    /**
     * Set DAC to a specific code and track it.
     * Port of firmware setDacCode() (without the actual delay).
     *
     * @param code 16-bit DAC code (-32768 to 32767)
     * @return settling time in ms that would be needed
     */
    public int setCode(short code) {
        int stepSize = Math.abs((int) code - (int) currentCode);
        currentCode = code;
        return calculateSettleTime(stepSize);
    }

    /**
     * Get the current DAC output voltage.
     * Uses the calibration table if valid, else nominal conversion.
     */
    public double getOutputVoltage() {
        return calTable.dacCodeToVoltage(currentCode);
    }

    /**
     * Calculate DAC filter settling time based on step size.
     * Port of firmware calculateDacSettleTime().
     *
     * Formula: settle_ms = 80 + 220 × sqrt(stepSize / 32768)
     */
    public static int calculateSettleTime(int stepSize) {
        if (stepSize <= 0) return 0;
        if (stepSize > 32768) stepSize = 32768;

        double scale = (double) (Constants.DAC_SETTLE_MAX_MS - Constants.DAC_SETTLE_MIN_MS)
                / Math.sqrt(32768.0);
        int additionalMs = (int) (scale * Math.sqrt(stepSize));

        return Constants.DAC_SETTLE_MIN_MS + additionalMs;
    }

    public short getCurrentCode() {
        return currentCode;
    }

    public DacCalibrationTable getCalTable() {
        return calTable;
    }
}
