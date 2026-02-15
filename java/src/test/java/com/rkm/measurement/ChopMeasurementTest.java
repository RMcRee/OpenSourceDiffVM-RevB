package com.rkm.measurement;

import com.rkm.Constants;
import com.rkm.hw.SimulatedADC;
import com.rkm.hw.SimulatedDAC;
import com.rkm.hw.SimulatedMux;
import com.rkm.cal.DacCalibrationTable;
import com.rkm.math.LowerMoments;
import com.rkm.sim.SimulationConfig;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

class ChopMeasurementTest {

    private SimulatedADC adc;
    private SimulatedDAC dac;
    private SimulatedMux mux;
    private ChopMeasurement chop;
    private SimulationConfig config;

    @BeforeEach
    void setUp() {
        config = new SimulationConfig();
        config.setAdcNoiseRms(0.0);  // No noise for deterministic tests
        config.setInputVoltageVx(0.001);  // 1mV input
        config.setPreampOffsetCounts(0.0);

        adc = new SimulatedADC(0.0, 42);
        dac = new SimulatedDAC(new DacCalibrationTable());
        mux = new SimulatedMux(config);
        chop = new ChopMeasurement();

        dac.setCode((short) 0);
        mux.setTmuxSelState(false);
    }

    @Test
    void testChopCancelsPreampOffset() {
        // With a 100-count preamp offset, chopping should cancel it
        double preampOffset = 100.0;
        config.setInputVoltageVx(0.0);  // zero input
        config.setGndOffset(0.0);
        dac.setCode((short) 0);

        LowerMoments stats = new LowerMoments();

        for (int i = 0; i < 50; i++) {
            boolean ok = chop.runOneChopCycle(stats, adc, dac, mux, preampOffset);
            assertTrue(ok, "Should not overflow");
        }

        // The demodulated result should cancel the offset to < 1 count
        double mean = stats.mean();
        assertTrue(Math.abs(mean) < 1.0,
                "Offset should cancel to < 1 count, got: " + mean);
    }

    @Test
    void testChopDemodulationSign() {
        // Positive Vx with DAC=0 should produce positive demodulated values
        config.setInputVoltageVx(0.001);  // 1mV
        dac.setCode((short) 0);

        LowerMoments stats = new LowerMoments();
        boolean ok = chop.runOneChopCycle(stats, adc, dac, mux, 0.0);
        assertTrue(ok);

        // (Vx - DAC) > 0, so demodulated should be positive
        assertTrue(stats.mean() > 0, "Demodulated should be positive for Vx > DAC");
    }

    @Test
    void testChopWithMatchedDacAndInput() {
        // When DAC matches Vx, demodulated result should be near zero
        // 1mV input → DAC code = 1e-3 / DAC_LSB_V ≈ 6.55
        short dacCode = (short) Math.round(0.001 / Constants.DAC_LSB_V);
        config.setInputVoltageVx(0.001);
        dac.setCode(dacCode);

        LowerMoments stats = new LowerMoments();
        for (int i = 0; i < 20; i++) {
            chop.runOneChopCycle(stats, adc, dac, mux, 0.0);
        }

        // Mean should be near zero (within quantization noise)
        double expectedDiff = 0.001 - dacCode * Constants.DAC_LSB_V;
        double expectedCounts = expectedDiff * Constants.PREAMP_GAIN / Constants.ADC_LSB_V;
        assertEquals(expectedCounts, stats.mean(), 1.0);
    }

    @Test
    void testOverflowDetection() {
        // Set input far from DAC to cause overflow
        config.setInputVoltageVx(4.0);  // 4V with DAC at 0 → huge signal
        dac.setCode((short) 0);

        LowerMoments stats = new LowerMoments();
        boolean ok = chop.runOneChopCycle(stats, adc, dac, mux, 0.0);
        assertFalse(ok, "Should detect overflow when Vx >> DAC");
    }

    @Test
    void testMultipleCyclesAccumulate() {
        config.setInputVoltageVx(0.0005);  // 0.5mV
        dac.setCode((short) 0);

        LowerMoments stats = new LowerMoments();
        int cycles = 10;
        for (int i = 0; i < cycles; i++) {
            chop.runOneChopCycle(stats, adc, dac, mux, 0.0);
        }

        assertEquals(cycles, (int) stats.count());
    }
}
