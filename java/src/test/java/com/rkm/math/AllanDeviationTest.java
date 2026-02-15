/*
 * SPDX-License-Identifier: CERN-OHL-P-2.0
 * SPDX-FileCopyrightText: Copyright 2025, 2026 Randall K McRee
 */
package com.rkm.math;

import org.junit.jupiter.api.Test;

import java.util.List;
import java.util.Random;

import static org.junit.jupiter.api.Assertions.*;

class AllanDeviationTest {

    @Test
    void testWhiteNoiseSlope() {
        // White noise: OADEV(tau) proportional to 1/sqrt(tau), i.e. slope -1/2 on log-log
        Random rng = new Random(42);
        int n = 10000;
        double tau0 = 1.0;
        double sigma = 1.0;
        double[] readings = new double[n];
        for (int i = 0; i < n; i++) {
            readings[i] = sigma * rng.nextGaussian();
        }

        List<AllanDeviation.Result> results = AllanDeviation.computeOctave(readings, tau0);
        assertTrue(results.size() >= 4, "Should have at least 4 octave points");

        // Check slope between consecutive octave points
        // For white noise, doubling tau should reduce ADEV by factor ~sqrt(2)
        for (int i = 0; i < results.size() - 1; i++) {
            double tau1 = results.get(i).tau();
            double tau2 = results.get(i + 1).tau();
            double adev1 = results.get(i).adev();
            double adev2 = results.get(i + 1).adev();

            double slope = Math.log(adev2 / adev1) / Math.log(tau2 / tau1);
            // White noise slope should be approximately -0.5
            // Allow generous tolerance for statistical variation
            assertEquals(-0.5, slope, 0.2,
                    String.format("Slope at tau=%.1f->%.1f should be ~-0.5, got %.3f", tau1, tau2, slope));
        }
    }

    @Test
    void testKnownSequence() {
        // Hand-computed 5-point example: readings = {1, 3, 2, 5, 4}
        // m=1: pairs at i=0,1,2 → averages: y_bar = readings themselves
        //   diff(0) = 3-1=2, diff(1)=2-3=-1, diff(2)=5-2=3
        //   AVAR = (4+1+9)/(2*3) = 14/6 = 7/3
        //   ADEV = sqrt(7/3)
        double[] readings = {1, 3, 2, 5, 4};
        double expected = Math.sqrt(7.0 / 3.0);
        double actual = AllanDeviation.computeForM(readings, 1);

        assertEquals(expected, actual, 1e-12, "ADEV for known 5-point sequence with m=1");
    }

    @Test
    void testKnownSequenceM2() {
        // readings = {1, 3, 2, 5, 4}, m=2
        // N=5, pairs = N-2m = 1 → only i=0
        // y_bar_0 = (1+3)/2 = 2.0
        // y_bar_2 = (2+5)/2 = 3.5
        // diff = 3.5 - 2.0 = 1.5
        // AVAR = 1.5^2 / (2*1) = 2.25/2 = 1.125
        // ADEV = sqrt(1.125)
        double[] readings = {1, 3, 2, 5, 4};
        double expected = Math.sqrt(1.125);
        double actual = AllanDeviation.computeForM(readings, 2);

        assertEquals(expected, actual, 1e-12, "ADEV for known 5-point sequence with m=2");
    }

    @Test
    void testMinimumData() {
        // m=1 needs N >= 3
        double[] three = {1, 2, 3};
        assertFalse(Double.isNaN(AllanDeviation.computeForM(three, 1)),
                "N=3, m=1 should work");

        double[] two = {1, 2};
        assertTrue(Double.isNaN(AllanDeviation.computeForM(two, 1)),
                "N=2, m=1 should return NaN (need 2m+1=3)");

        // m=2 needs N >= 5
        double[] four = {1, 2, 3, 4};
        assertTrue(Double.isNaN(AllanDeviation.computeForM(four, 2)),
                "N=4, m=2 should return NaN (need 2m+1=5)");

        double[] five = {1, 2, 3, 4, 5};
        assertFalse(Double.isNaN(AllanDeviation.computeForM(five, 2)),
                "N=5, m=2 should work");
    }

    @Test
    void testConstantInput() {
        // All same value → OADEV = 0
        double[] readings = new double[100];
        java.util.Arrays.fill(readings, 3.14159);

        List<AllanDeviation.Result> results = AllanDeviation.computeOctave(readings, 1.0);
        assertFalse(results.isEmpty());

        for (AllanDeviation.Result r : results) {
            assertEquals(0.0, r.adev(), 1e-12,
                    String.format("ADEV should be ~0 for constant input at tau=%.1f", r.tau()));
        }
    }

    @Test
    void testInvalidM() {
        double[] readings = {1, 2, 3, 4, 5};
        assertTrue(Double.isNaN(AllanDeviation.computeForM(readings, 0)),
                "m=0 should return NaN");
        assertTrue(Double.isNaN(AllanDeviation.computeForM(readings, -1)),
                "m<0 should return NaN");
    }

    @Test
    void testOctaveSpacing() {
        // Verify that computed taus are octave-spaced (powers of 2 * tau0)
        double[] readings = new double[1000];
        Random rng = new Random(123);
        for (int i = 0; i < readings.length; i++) {
            readings[i] = rng.nextGaussian();
        }

        double tau0 = 0.26;
        List<AllanDeviation.Result> results = AllanDeviation.computeOctave(readings, tau0);

        for (int i = 0; i < results.size(); i++) {
            double expectedTau = Math.pow(2, i) * tau0;
            assertEquals(expectedTau, results.get(i).tau(), 1e-12,
                    "Tau at index " + i + " should be 2^" + i + " * tau0");
        }
    }

    @Test
    void testPairsCount() {
        // Verify pairs = N - 2m for each result
        int n = 500;
        double[] readings = new double[n];
        Random rng = new Random(999);
        for (int i = 0; i < n; i++) {
            readings[i] = rng.nextGaussian();
        }

        List<AllanDeviation.Result> results = AllanDeviation.computeOctave(readings, 1.0);
        int m = 1;
        for (AllanDeviation.Result r : results) {
            assertEquals(n - 2 * m, r.pairsUsed(),
                    String.format("Pairs at m=%d should be %d", m, n - 2 * m));
            m *= 2;
        }
    }
}
