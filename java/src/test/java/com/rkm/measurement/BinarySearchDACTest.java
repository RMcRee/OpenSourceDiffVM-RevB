/*
 * SPDX-License-Identifier: CERN-OHL-P-2.0
 * SPDX-FileCopyrightText: Copyright 2025, 2026 Randall K McRee
 */
package com.rkm.measurement;

import com.rkm.Constants;
import com.rkm.cal.DacCalibrationTable;
import com.rkm.hw.SimulatedADC;
import com.rkm.hw.SimulatedDAC;
import com.rkm.hw.SimulatedMux;
import com.rkm.sim.SimulationConfig;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.ValueSource;

import static org.junit.jupiter.api.Assertions.*;

class BinarySearchDACTest {

    private SimulatedADC createAdc(long seed) {
        return new SimulatedADC(2.0, seed);  // Small noise for realistic test
    }

    @ParameterizedTest
    @ValueSource(doubles = {-4.0, -1.0, 0.0, 0.5, 1.0, 1.234, 3.5, 4.9})
    void testConvergesForVarious_Vx(double vx) {
        SimulationConfig config = new SimulationConfig();
        config.setInputVoltageVx(vx);
        config.setAdcNoiseRms(2.0);
        config.setPreampOffsetCounts(0.0);

        SimulatedADC adc = createAdc(42);
        SimulatedDAC dac = new SimulatedDAC(new DacCalibrationTable());
        SimulatedMux mux = new SimulatedMux(config);
        BinarySearchDAC search = new BinarySearchDAC();

        boolean converged = search.search(adc, dac, mux, 0.0);
        assertTrue(converged, "Should converge for Vx=" + vx);

        // After convergence, DAC should be near Vx
        double dacV = dac.getOutputVoltage();
        double diff = Math.abs(dacV - vx);
        // DAC resolution is ~152.6µV, with 14-bit alignment (×4) → ~610µV
        assertTrue(diff < 0.002,
                String.format("DAC should be within 2mV of Vx=%.3fV, got diff=%.6fV", vx, diff));
    }

    @Test
    void testDacCodeIs14BitAligned() {
        SimulationConfig config = new SimulationConfig();
        config.setInputVoltageVx(1.5);
        config.setAdcNoiseRms(0.0);

        SimulatedADC adc = new SimulatedADC(0.0, 42);
        SimulatedDAC dac = new SimulatedDAC(new DacCalibrationTable());
        SimulatedMux mux = new SimulatedMux(config);
        BinarySearchDAC search = new BinarySearchDAC();

        search.search(adc, dac, mux, 0.0);

        // DAC code should be a multiple of 4
        assertEquals(0, dac.getCurrentCode() % 4,
                "DAC code should be 14-bit aligned (multiple of 4), got: " + dac.getCurrentCode());
    }

    @Test
    void testTmuxForcedLow() {
        SimulationConfig config = new SimulationConfig();
        config.setInputVoltageVx(1.0);

        SimulatedADC adc = new SimulatedADC(0.0, 42);
        SimulatedDAC dac = new SimulatedDAC(new DacCalibrationTable());
        SimulatedMux mux = new SimulatedMux(config);
        mux.setTmuxSelState(true);  // Start HIGH

        BinarySearchDAC search = new BinarySearchDAC();
        search.search(adc, dac, mux, 0.0);

        // After search, TMUX should be LOW
        assertFalse(mux.isTmuxSelState(), "TMUX should be LOW after search");
    }

    @Test
    void testWithPreampOffset() {
        SimulationConfig config = new SimulationConfig();
        config.setInputVoltageVx(2.0);

        SimulatedADC adc = new SimulatedADC(2.0, 42);
        SimulatedDAC dac = new SimulatedDAC(new DacCalibrationTable());
        SimulatedMux mux = new SimulatedMux(config);
        BinarySearchDAC search = new BinarySearchDAC();

        // Should still converge with preamp offset
        boolean converged = search.search(adc, dac, mux, 100.0);
        assertTrue(converged, "Should converge even with preamp offset");
    }
}
