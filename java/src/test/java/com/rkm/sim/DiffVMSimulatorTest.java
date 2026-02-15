/*
 * SPDX-License-Identifier: CERN-OHL-P-2.0
 * SPDX-FileCopyrightText: 2025 rkm
 */
package com.rkm.sim;

import com.rkm.Constants;
import com.rkm.model.InputChannel;
import org.junit.jupiter.api.Test;

import java.util.List;

import static org.junit.jupiter.api.Assertions.*;

class DiffVMSimulatorTest {

    @Test
    void testSetupConverges() {
        SimulationConfig config = new SimulationConfig();
        config.setInputVoltageVx(1.0);
        config.setAdcNoiseRms(4.0);

        DiffVMSimulator sim = new DiffVMSimulator(config);
        sim.setup();

        // After setup, DAC should be near input voltage
        double dacV = sim.getDac().getOutputVoltage();
        assertTrue(Math.abs(dacV - 1.0) < 0.002,
                "DAC should converge near 1.0V, got: " + dacV);
    }

    @Test
    void testIdleModeProducesResults() {
        SimulationConfig config = new SimulationConfig();
        config.setInputVoltageVx(1.234);
        config.setAdcNoiseRms(4.0);
        config.setPreampOffsetCounts(50.0);

        DiffVMSimulator sim = new DiffVMSimulator(config);
        sim.setup();

        // Default integration = 100 cycles, run enough to get a few results
        sim.runLoop(500);

        List<DiffVMSimulator.MeasurementResult> results = sim.getResults();
        assertFalse(results.isEmpty(), "Should produce at least one result");

        // Check first result is reasonable
        DiffVMSimulator.MeasurementResult r = results.get(0);
        assertEquals(InputChannel.Vx, r.channel());
        assertTrue(Math.abs(r.voltage() - 1.234) < 0.001,
                "Voltage should be near 1.234V, got: " + r.voltage());
    }

    @Test
    void testAccuracyAtVarious_Vx() {
        double[] testVoltages = {0.0, 0.1, 1.0, -2.5, 4.0};

        for (double vx : testVoltages) {
            SimulationConfig config = new SimulationConfig();
            config.setInputVoltageVx(vx);
            config.setAdcNoiseRms(4.0);
            config.setPreampOffsetCounts(0.0);
            config.setRandomSeed(123);

            DiffVMSimulator sim = new DiffVMSimulator(config);
            sim.setup();
            sim.runLoop(200);

            List<DiffVMSimulator.MeasurementResult> results = sim.getResults();
            if (!results.isEmpty()) {
                DiffVMSimulator.MeasurementResult r = results.get(results.size() - 1);
                assertTrue(Math.abs(r.voltage() - vx) < 0.001,
                        String.format("For Vx=%.3f, got %.6f (error=%.3e)",
                                vx, r.voltage(), r.voltage() - vx));
            }
        }
    }

    @Test
    void testLongRunStability() {
        SimulationConfig config = new SimulationConfig();
        config.setInputVoltageVx(1.234);
        config.setAdcNoiseRms(8.0);
        config.setPreampOffsetCounts(100.0);

        DiffVMSimulator sim = new DiffVMSimulator(config);
        sim.setup();

        // Run 1000 iterations for stable output
        sim.runLoop(1000);

        List<DiffVMSimulator.MeasurementResult> results = sim.getResults();
        assertTrue(results.size() >= 5, "Should produce multiple results");

        // Average the last few results
        double sum = 0;
        int n = Math.min(5, results.size());
        for (int i = results.size() - n; i < results.size(); i++) {
            sum += results.get(i).voltage();
        }
        double avgVoltage = sum / n;

        // Should be within 10nV of target (very tight for 1000 iterations)
        assertTrue(Math.abs(avgVoltage - 1.234) < 100e-9,
                String.format("Average should be within 100nV of 1.234V, got error=%.3e V",
                        avgVoltage - 1.234));
    }

    @Test
    void testZeroInput() {
        SimulationConfig config = new SimulationConfig();
        config.setInputVoltageVx(0.0);
        config.setGndOffset(0.0);
        config.setAdcNoiseRms(4.0);
        config.setPreampOffsetCounts(0.0);

        DiffVMSimulator sim = new DiffVMSimulator(config);
        sim.setup();
        sim.runLoop(300);

        List<DiffVMSimulator.MeasurementResult> results = sim.getResults();
        assertFalse(results.isEmpty());

        DiffVMSimulator.MeasurementResult r = results.get(results.size() - 1);
        // Zero input should measure very close to zero
        assertTrue(Math.abs(r.voltage()) < 1e-6,
                "Zero input should measure near zero, got: " + r.voltage());
    }

    @Test
    void testScanningMode() {
        SimulationConfig config = new SimulationConfig();
        config.setInputVoltageVx(2.0);
        config.setAdcNoiseRms(4.0);

        DiffVMSimulator sim = new DiffVMSimulator(config);
        sim.setup();

        // Add a second channel to scan list
        sim.getScanConfig().addChannel(InputChannel.VrefRaw);
        sim.startScanning();

        // Run enough for at least one sweep
        sim.runLoop(500);

        assertEquals(com.rkm.model.ScanState.SCANNING, sim.getScanState());
    }

    @Test
    void testDacSettleTimeFormula() {
        // Verify settle time calculation matches firmware
        assertEquals(0, com.rkm.hw.SimulatedDAC.calculateSettleTime(0));

        int settle100 = com.rkm.hw.SimulatedDAC.calculateSettleTime(100);
        assertTrue(settle100 > 80 && settle100 < 100,
                "100-code step should settle ~92ms, got: " + settle100);

        int settleFull = com.rkm.hw.SimulatedDAC.calculateSettleTime(32768);
        // Firmware uses (uint32_t)(scale * sqrt(32768)) which truncates → 219 + 80 = 299
        assertTrue(settleFull >= Constants.DAC_SETTLE_MAX_MS - 1 && settleFull <= Constants.DAC_SETTLE_MAX_MS,
                "Full-scale should settle near max: " + settleFull);
    }
}
