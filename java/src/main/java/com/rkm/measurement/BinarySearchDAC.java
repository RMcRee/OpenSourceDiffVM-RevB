/*
 * SPDX-License-Identifier: CERN-OHL-P-2.0
 * SPDX-FileCopyrightText: Copyright 2025, 2026 Randall K McRee
 */
package com.rkm.measurement;

import com.rkm.Constants;
import com.rkm.hw.SimulatedADC;
import com.rkm.hw.SimulatedDAC;
import com.rkm.hw.SimulatedMux;

/**
 * Binary search algorithm to find DAC code that brings ADC into range.
 * Port of firmware binarySearchDAC() (lines 814-867).
 *
 * 14-bit aligned (multiples of 4) since only these codes have calibration data.
 */
public class BinarySearchDAC {

    private static final int MAX_ITERATIONS = 16;

    /**
     * Perform binary search to find correct DAC code.
     *
     * The search uses TMUXSEL=LOW where the preamp measures (Vx - DAC) × gain:
     *   - If ADC overflows positive: Vx > DAC, need to increase DAC
     *   - If ADC overflows negative: Vx < DAC, need to decrease DAC
     *
     * @param adc simulated ADC
     * @param dac simulated DAC
     * @param mux simulated mux
     * @param preampOffset preamp DC offset in counts
     * @return true if converged, false if failed
     */
    public boolean search(SimulatedADC adc, SimulatedDAC dac, SimulatedMux mux,
                          double preampOffset) {
        // Force TMUXSEL=LOW
        if (mux.isTmuxSelState()) {
            mux.chop();
        }

        int low = -32768;
        int high = 32764;  // max aligned value (32767 & ~3)

        for (int iter = 0; iter < MAX_ITERATIONS; iter++) {
            int mid = ((low + high) / 2) & ~3;  // Align to 14-bit boundary
            dac.setCode((short) mid);

            // Read a settled sample
            double dacVoltage = dac.getOutputVoltage();
            double adcInput = mux.getAdcInputVoltage(dacVoltage, preampOffset);
            // Discard one sample
            adc.readSample(adcInput);
            int sample = adc.readSample(adcInput);

            if (SimulatedADC.isOverflowPositive(sample)) {
                // Vx > DAC, increase DAC
                low = mid + 4;
            } else if (SimulatedADC.isOverflowNegative(sample)) {
                // Vx < DAC, decrease DAC
                high = mid - 4;
            } else {
                // In range - success
                return true;
            }

            if (low > high) {
                break;
            }
        }

        return false;  // Failed to converge
    }
}
