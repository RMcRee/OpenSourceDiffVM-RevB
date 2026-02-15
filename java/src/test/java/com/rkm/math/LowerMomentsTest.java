/*
 * SPDX-License-Identifier: CERN-OHL-P-2.0
 * SPDX-FileCopyrightText: 2025 rkm
 */
package com.rkm.math;

import org.junit.jupiter.api.Test;

import java.util.Random;

import static org.junit.jupiter.api.Assertions.*;

class LowerMomentsTest {

    @Test
    void testSingleSample() {
        LowerMoments lm = new LowerMoments();
        lm.accumulate(42.0);

        assertEquals(1.0, lm.count());
        assertEquals(42.0, lm.mean(), 1e-15);
        assertTrue(Double.isNaN(lm.variance()), "Variance undefined for n=1");
        assertEquals(42.0, lm.getMin());
        assertEquals(42.0, lm.getMax());
    }

    @Test
    void testTwoSamples() {
        LowerMoments lm = new LowerMoments();
        lm.accumulate(10.0);
        lm.accumulate(20.0);

        assertEquals(2.0, lm.count());
        assertEquals(15.0, lm.mean(), 1e-12);
        assertEquals(50.0, lm.variance(), 1e-12);  // sample variance = (10-15)^2 + (20-15)^2 / 1
        assertEquals(10.0, lm.getMin());
        assertEquals(20.0, lm.getMax());
    }

    @Test
    void testKnownSequence() {
        LowerMoments lm = new LowerMoments();
        double[] data = {2, 4, 4, 4, 5, 5, 7, 9};

        for (double d : data) {
            lm.accumulate(d);
        }

        assertEquals(8.0, lm.count());
        assertEquals(5.0, lm.mean(), 1e-12);
        // Population variance = 4.0, sample variance = 4.0 * 8/7 = 32/7
        assertEquals(32.0 / 7.0, lm.variance(), 1e-12);
        assertEquals(2.0, lm.getMin());
        assertEquals(9.0, lm.getMax());
    }

    @Test
    void testGaussianDistribution() {
        LowerMoments lm = new LowerMoments();
        Random rng = new Random(42);
        double trueMean = 100.0;
        double trueStdDev = 5.0;

        for (int i = 0; i < 1000; i++) {
            lm.accumulate(trueMean + trueStdDev * rng.nextGaussian());
        }

        assertEquals(1000.0, lm.count());
        assertEquals(trueMean, lm.mean(), 0.5);
        assertEquals(trueStdDev, lm.standardDeviation(), 0.5);
    }

    @Test
    void testClear() {
        LowerMoments lm = new LowerMoments();
        lm.accumulate(1.0);
        lm.accumulate(2.0);
        lm.clear();

        assertEquals(0.0, lm.count());
        assertEquals(0.0, lm.mean());
        assertEquals(Double.POSITIVE_INFINITY, lm.getMin());
        assertEquals(Double.NEGATIVE_INFINITY, lm.getMax());
    }

    @Test
    void testNanInfIgnored() {
        LowerMoments lm = new LowerMoments();
        lm.accumulate(1.0);
        lm.accumulate(Double.NaN);
        lm.accumulate(Double.POSITIVE_INFINITY);
        lm.accumulate(Double.NEGATIVE_INFINITY);
        lm.accumulate(3.0);

        assertEquals(2.0, lm.count());
        assertEquals(2.0, lm.mean(), 1e-12);
    }

    @Test
    void testNegativeValues() {
        LowerMoments lm = new LowerMoments();
        lm.accumulate(-5.0);
        lm.accumulate(-3.0);
        lm.accumulate(-1.0);

        assertEquals(3.0, lm.count());
        assertEquals(-3.0, lm.mean(), 1e-12);
        assertEquals(-5.0, lm.getMin());
        assertEquals(-1.0, lm.getMax());
    }
}
