package com.rkm.measurement;

import com.rkm.Constants;
import com.rkm.hw.SimulatedADC;
import com.rkm.hw.SimulatedDAC;
import com.rkm.hw.SimulatedMux;
import com.rkm.math.LowerMoments;
import com.rkm.model.InputChannel;

/**
 * Auto-zero calibration by measuring the GND input channel.
 * Port of firmware performAutoZero() (lines 2312-2352).
 *
 * Measures the residual offset when input is grounded, which is then
 * subtracted from all subsequent voltage readings.
 */
public class AutoZero {

    private static final int NUM_ITERATIONS = 20;

    private double offset = 0.0;
    private boolean valid = false;
    private final ChopMeasurement chopMeasurement = new ChopMeasurement();

    /**
     * Perform auto-zero calibration.
     * Port of firmware performAutoZero().
     *
     * 1. Switch mux to GND, set DAC to 0
     * 2. Run binary search for DAC null point
     * 3. Run 20 chopped measurement cycles
     * 4. Compute and store offset voltage
     *
     * @param adc simulated ADC
     * @param dac simulated DAC
     * @param mux simulated mux
     * @param binarySearch DAC binary search
     * @param voltageComputer for computing input voltage
     * @param preampOffset preamp DC offset in counts
     * @return the measured offset in volts, or NaN if failed
     */
    public double perform(SimulatedADC adc, SimulatedDAC dac, SimulatedMux mux,
                          BinarySearchDAC binarySearch, VoltageComputer voltageComputer,
                          double preampOffset) {
        // Save current state
        InputChannel savedChannel = mux.getCurrentChannel();
        short savedDac = dac.getCurrentCode();

        // Switch to GND
        mux.selectInputChannel(InputChannel.GND);
        dac.setCode((short) 0);

        // Run binary search for GND
        binarySearch.search(adc, dac, mux, preampOffset);

        // Accumulate chopped measurements
        LowerMoments gndStats = new LowerMoments();

        for (int i = 0; i < NUM_ITERATIONS; i++) {
            boolean ok = chopMeasurement.runOneChopCycle(gndStats, adc, dac, mux, preampOffset);
            if (!ok) {
                // Overflow during auto-zero - restore and abort
                mux.selectInputChannel(savedChannel);
                dac.setCode(savedDac);
                return Double.NaN;
            }
        }

        // Compute zero offset
        offset = voltageComputer.computeInputVoltage(
                gndStats.mean(), dac.getCurrentCode(), InputChannel.GND);
        valid = true;

        // Restore previous state
        mux.selectInputChannel(savedChannel);
        dac.setCode(savedDac);

        return offset;
    }

    public double getOffset() { return offset; }
    public boolean isValid() { return valid; }

    public void invalidate() {
        valid = false;
        offset = 0.0;
    }
}
