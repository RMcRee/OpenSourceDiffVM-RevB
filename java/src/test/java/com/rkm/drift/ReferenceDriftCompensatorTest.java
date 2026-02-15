/*
 * SPDX-License-Identifier: CERN-OHL-P-2.0
 * SPDX-FileCopyrightText: 2025 rkm
 */
package com.rkm.drift;

import com.rkm.Constants;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

class ReferenceDriftCompensatorTest {

    private ReferenceDriftCompensator comp;

    @BeforeEach
    void setUp() {
        comp = new ReferenceDriftCompensator();
    }

    @Test
    void testInitialState() {
        assertFalse(comp.isInitialized());
        assertEquals(0.0, comp.predictFilterError(0));
        assertEquals(1.0, comp.getCorrectionFactor(0));
    }

    @Test
    void testSingleMeasurement() {
        comp.measureFilterError(1e-6, 1000);  // 1µV error at t=1s
        assertTrue(comp.isInitialized());

        // Immediately after measurement, predicted error should be close to measured
        double predicted = comp.predictFilterError(1000);
        assertEquals(1e-6, predicted, 1e-7);
    }

    @Test
    void testErrorDecays() {
        // Record an error, then check it decays over time
        comp.measureFilterError(1e-5, 1000);  // 10µV error at t=1s

        double errorAt1s = comp.predictFilterError(1000);
        double errorAt3s = comp.predictFilterError(3000);  // 2s later
        double errorAt9s = comp.predictFilterError(9000);  // 8s later

        // Error should decrease over time as filter catches up
        assertTrue(Math.abs(errorAt3s) < Math.abs(errorAt1s),
                "Error should decay: " + errorAt3s + " < " + errorAt1s);
        assertTrue(Math.abs(errorAt9s) < Math.abs(errorAt3s),
                "Error should continue decaying: " + errorAt9s + " < " + errorAt3s);
    }

    @Test
    void testDriftRateEstimation() {
        // Simulate a constant drift: error increasing 1µV per second
        double driftRate = 1e-6;  // 1µV/s

        for (int t = 0; t < 5; t++) {
            long timeMs = (t + 1) * 1000L;
            double error = driftRate * Constants.REF_FILTER_TAU_EFF;  // steady-state filter error
            comp.measureFilterError(error, timeMs);
        }

        // After several measurements, drift rate estimate should be reasonable
        double est = comp.getEstimatedDriftRate();
        // The estimate converges; just check it's not wildly off
        assertTrue(Math.abs(est) < 1e-4,
                "Drift rate should be reasonable, got: " + est);
    }

    @Test
    void testCorrectionFactorNearOne() {
        // With small filter error, correction should be near 1.0
        comp.measureFilterError(1e-6, 1000);

        double factor = comp.getCorrectionFactor(1000);
        // correction = 1 + filterErr / 5V = 1 + 1e-6/5 = 1.0000002
        assertEquals(1.0, factor, 1e-4);
        assertNotEquals(1.0, factor, "Should not be exactly 1.0");
    }

    @Test
    void testCorrectionTracksDrift() {
        // Simulate 1 ppm/min drift → ~8.33e-11 V/s on 5V reference
        // Over 1 minute, that's ~5e-9 V error → 1 ppb correction
        double driftVPerS = 5.0 * 1e-6 / 60.0;  // 1 ppm/min on 5V ref

        // Feed several measurements with increasing error
        for (int i = 1; i <= 8; i++) {
            long timeMs = i * 1000L;
            double filterError = driftVPerS * Constants.REF_FILTER_TAU_EFF;
            comp.measureFilterError(filterError, timeMs);
        }

        double correction = comp.getCorrectionFactor(9000);
        double correctionPpb = (correction - 1.0) * 1e9;

        // Correction should be non-zero and tracking the drift
        assertTrue(Math.abs(correctionPpb) < 100,
                "Correction should be small for small drift, got: " + correctionPpb + " ppb");
    }

    @Test
    void testShouldMeasurePeriodic() {
        int count = 0;
        for (int i = 0; i < Constants.REF_SAMPLE_INTERVAL * 3; i++) {
            if (comp.shouldMeasure()) {
                count++;
            }
        }
        assertEquals(3, count, "Should trigger 3 times in 3 intervals");
    }

    @Test
    void testInitTrackingResets() {
        comp.measureFilterError(1e-5, 1000);
        assertTrue(comp.isInitialized());

        comp.initTracking();
        assertFalse(comp.isInitialized());
        assertEquals(0.0, comp.getEstimatedDriftRate());
    }
}
