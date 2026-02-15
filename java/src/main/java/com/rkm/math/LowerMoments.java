package com.rkm.math;

/**
 * Welford's online algorithm for computing mean and variance in a single pass.
 * Port of firmware lines 497-557.
 *
 * Uses the same moments_[3] array layout as the firmware:
 *   moments_[0] = count (n)
 *   moments_[1] = mean
 *   moments_[2] = 2nd central moment accumulator (M2/n)
 */
public class LowerMoments {

    private final double[] moments = new double[3];
    private double min;
    private double max;

    public LowerMoments() {
        clear();
    }

    /**
     * Add a sample and update mean/variance incrementally.
     * Exact port of firmware accumulate().
     */
    public void accumulate(double x) {
        if (Double.isNaN(x) || Double.isInfinite(x)) return;

        if (x < min) min = x;
        if (x > max) max = x;

        double n = moments[0];
        double n1 = n + 1.0;

        // Same math as the firmware:
        double delta = (moments[1] - x) / n1;
        double d2 = delta * delta;
        double r1 = n / n1;

        moments[2] += (1.0 + n) * d2;
        moments[2] *= r1;

        moments[1] -= delta;   // updates mean
        moments[0] = n1;       // updates count
    }

    public double mean() {
        return moments[1];
    }

    public double count() {
        return moments[0];
    }

    public double variance() {
        if (moments[0] < 2.0) return Double.NaN;
        return moments[2] * moments[0] / (moments[0] - 1.0);
    }

    public double standardDeviation() {
        return Math.sqrt(variance());
    }

    public double getMin() {
        return min;
    }

    public double getMax() {
        return max;
    }

    public void clear() {
        moments[0] = moments[1] = moments[2] = 0.0;
        min = Double.POSITIVE_INFINITY;
        max = Double.NEGATIVE_INFINITY;
    }

    @Override
    public String toString() {
        return String.format("%.12g sd=%.12g", mean(), standardDeviation());
    }
}
