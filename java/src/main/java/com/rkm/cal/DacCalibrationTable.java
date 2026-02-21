/*
 * SPDX-License-Identifier: CERN-OHL-P-2.0
 * SPDX-FileCopyrightText: Copyright 2025, 2026 Randall K McRee
 */
package com.rkm.cal;

import com.rkm.Constants;

/**
 * 16384-point DAC INL calibration table with linear interpolation.
 * Port of firmware lines 206-261.
 *
 * Index: (dacCode + 32768) >> 2 gives 14-bit index (0..16383)
 * Value: actual voltage at that code (measured against ADR1001 references)
 */
public class DacCalibrationTable {

    private final double[] table = new double[Constants.DAC_CAL_TABLE_SIZE];
    private boolean valid = false;

    public DacCalibrationTable() {
        initNominal();
    }

    /**
     * Initialize with nominal (uncalibrated) values.
     * Port of firmware initDacCalTable().
     */
    public void initNominal() {
        for (int i = 0; i < Constants.DAC_CAL_TABLE_SIZE; i++) {
            short code = (short) ((i << 2) - 32768);
            table[i] = (double) code * Constants.DAC_LSB_V;
        }
        valid = false;
    }

    /**
     * Look up calibrated DAC voltage for a given code.
     * Uses linear interpolation between calibration points.
     * Port of firmware dacCodeToVoltage().
     *
     * @param code 16-bit DAC code (short, -32768 to 32767)
     * @return voltage in volts
     */
    public double dacCodeToVoltage(short code) {
        if (!valid) {
            return (double) code * Constants.DAC_LSB_V;
        }

        // Convert code to table index (14-bit, 0..16383)
        int idx = ((code + 32768) & 0xFFFF) >>> 2;

        // Clamp to valid range
        if (idx >= Constants.DAC_CAL_TABLE_SIZE - 1) {
            return table[Constants.DAC_CAL_TABLE_SIZE - 1];
        }

        // Linear interpolation between adjacent calibration points
        int frac = ((code + 32768) & 0xFFFF) & 0x03;  // 0..3
        double v0 = table[idx];
        double v1 = table[idx + 1];
        return v0 + (v1 - v0) * (double) frac / 4.0;
    }

    /**
     * Store a calibration point.
     * Port of firmware DacCalibrationTable::setPoint().
     */
    public void setPoint(short code, double measuredVoltage) {
        int idx = ((code + 32768) & 0xFFFF) >>> 2;
        if (idx < Constants.DAC_CAL_TABLE_SIZE) {
            table[idx] = measuredVoltage;
        }
    }

    /**
     * Mark calibration table as valid.
     */
    public void markValid() {
        valid = true;
    }

    public boolean isValid() {
        return valid;
    }

    /**
     * Direct access to the table (for testing).
     */
    public double getTableEntry(int index) {
        return table[index];
    }
}
