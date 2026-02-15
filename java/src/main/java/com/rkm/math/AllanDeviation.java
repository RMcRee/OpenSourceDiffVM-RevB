/*
 * SPDX-License-Identifier: CERN-OHL-P-2.0
 * SPDX-FileCopyrightText: Copyright 2025, 2026 Randall K McRee
 */
package com.rkm.math;

import java.util.ArrayList;
import java.util.List;

/**
 * Overlapping Allan Deviation (OADEV) computation for voltage stability analysis.
 *
 * For N voltage readings y_0..y_{N-1} at interval tau_0, OADEV at tau = m * tau_0:
 *
 *   OADEV^2(m) = 1 / (2(N - 2m)) * SUM_{i=0}^{N-2m-1} (y_bar_{i+m} - y_bar_i)^2
 *
 * where y_bar_i = (1/m) * SUM_{k=0}^{m-1} y_{i+k} (moving average of m consecutive readings).
 */
public class AllanDeviation {

    /**
     * A single OADEV result at one averaging time.
     *
     * @param tau       averaging time in seconds (m * tau0)
     * @param adev      Allan deviation value
     * @param pairsUsed number of overlapping pairs used in computation
     */
    public record Result(double tau, double adev, int pairsUsed) {}

    private AllanDeviation() {}

    /**
     * Compute overlapping Allan deviation at octave-spaced tau values.
     *
     * @param readings voltage readings at uniform interval tau0
     * @param tau0     base measurement interval in seconds
     * @return list of (tau, adev, pairsUsed) sorted by ascending tau
     */
    public static List<Result> computeOctave(double[] readings, double tau0) {
        List<Result> results = new ArrayList<>();
        int n = readings.length;

        for (int m = 1; m <= n / 3; m *= 2) {
            double adev = computeForM(readings, m);
            if (!Double.isNaN(adev)) {
                int pairs = n - 2 * m;
                results.add(new Result(m * tau0, adev, pairs));
            }
        }

        return results;
    }

    /**
     * Compute OADEV for a specific averaging factor m.
     *
     * @param readings voltage readings at uniform interval
     * @param m        averaging factor (tau = m * tau0)
     * @return OADEV value, or NaN if insufficient data (need N >= 2m + 1)
     */
    public static double computeForM(double[] readings, int m) {
        int n = readings.length;
        if (m < 1 || n < 2 * m + 1) {
            return Double.NaN;
        }

        // Compute prefix sums for efficient moving averages
        double[] prefix = new double[n + 1];
        prefix[0] = 0.0;
        for (int i = 0; i < n; i++) {
            prefix[i + 1] = prefix[i] + readings[i];
        }

        // Compute OADEV^2 using overlapping differences of block averages
        double sumSq = 0.0;
        int pairs = n - 2 * m;
        for (int i = 0; i < pairs; i++) {
            // y_bar_i     = (prefix[i+m]   - prefix[i])   / m
            // y_bar_{i+m} = (prefix[i+2m]  - prefix[i+m]) / m
            double avg1 = (prefix[i + m] - prefix[i]) / m;
            double avg2 = (prefix[i + 2 * m] - prefix[i + m]) / m;
            double diff = avg2 - avg1;
            sumSq += diff * diff;
        }

        double avar = sumSq / (2.0 * pairs);
        return Math.sqrt(avar);
    }
}
