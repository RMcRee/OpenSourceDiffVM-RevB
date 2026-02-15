/*
 * SPDX-License-Identifier: CERN-OHL-P-2.0
 * SPDX-FileCopyrightText: Copyright 2025, 2026 Randall K McRee
 */
package com.rkm.sim;

import com.rkm.Constants;
import com.rkm.cal.DacCalibrationTable;
import com.rkm.drift.ReferenceDriftCompensator;
import com.rkm.hw.SimulatedADC;
import com.rkm.hw.SimulatedDAC;
import com.rkm.hw.SimulatedMux;
import com.rkm.math.AllanDeviation;
import com.rkm.math.LowerMoments;
import com.rkm.measurement.AutoZero;
import com.rkm.measurement.BinarySearchDAC;
import com.rkm.measurement.ChopMeasurement;
import com.rkm.measurement.VoltageComputer;
import com.rkm.model.InputChannel;
import com.rkm.model.ScanConfig;
import com.rkm.model.ScanState;

import java.util.ArrayList;
import java.util.List;

/**
 * Top-level DiffVM simulator that wires all components together and runs the
 * measurement loop. Port of firmware setup() + loop() (lines 2642-2927).
 */
public class DiffVMSimulator {

    // Hardware simulators
    private final SimulatedADC adc;
    private final SimulatedDAC dac;
    private final SimulatedMux mux;

    // Algorithm components
    private final DacCalibrationTable calTable;
    private final VoltageComputer voltageComputer;
    private final ChopMeasurement chopMeasurement;
    private final BinarySearchDAC binarySearch;
    private final ReferenceDriftCompensator driftCompensator;
    private final AutoZero autoZero;

    // Configuration
    private final SimulationConfig config;
    private final ScanConfig scanConfig;

    // State
    private final LowerMoments chopStats = new LowerMoments();
    private final LowerMoments[] channelStats = new LowerMoments[8];
    private final short[] channelDacCode = new short[8];   // Saved DAC code per channel
    private final boolean[] channelDacValid = new boolean[8]; // True after first successful measurement
    private ScanState scanState = ScanState.IDLE;
    private int currentScanIndex = 0;
    private int scanCycleCount = 0;
    private int chopCycleCount = 0;
    private int idleCounter = 0;
    private long simulatedTimeMs = 0;

    // Results collected during simulation
    private final List<MeasurementResult> results = new ArrayList<>();

    /**
     * A single measurement result.
     */
    public record MeasurementResult(
            InputChannel channel,
            double voltage,
            double uncertainty,
            double adcMean,
            double adcStdDev,
            int sampleCount,
            short dacCode,
            double driftCorrectionPpb
    ) {}

    public DiffVMSimulator(SimulationConfig config) {
        this.config = config;

        // Create hardware simulators
        calTable = new DacCalibrationTable();
        adc = new SimulatedADC(config.getAdcNoiseRms(), config.getRandomSeed());
        dac = new SimulatedDAC(calTable);
        mux = new SimulatedMux(config);

        // Create algorithm components
        voltageComputer = new VoltageComputer(calTable);
        chopMeasurement = new ChopMeasurement();
        binarySearch = new BinarySearchDAC();
        driftCompensator = new ReferenceDriftCompensator();
        autoZero = new AutoZero();

        scanConfig = new ScanConfig();

        for (int i = 0; i < 8; i++) {
            channelStats[i] = new LowerMoments();
        }
    }

    /**
     * Run the setup sequence (port of firmware setup()).
     * Initializes DAC cal table, runs binary search to find initial DAC setting.
     */
    public void setup() {
        calTable.initNominal();
        clearChannelDacCodes();
        driftCompensator.initTracking();

        // Initial binary search to bring ADC into range
        binarySearch.search(adc, dac, mux, config.getPreampOffsetCounts());
    }

    /**
     * Clear all saved per-channel DAC codes.
     * Port of firmware clearChannelDacCodes().
     */
    public void clearChannelDacCodes() {
        for (int i = 0; i < 8; i++) {
            channelDacCode[i] = 0;
            channelDacValid[i] = false;
        }
    }

    /**
     * Restore saved DAC code for a channel, or run binary search on first visit.
     * Port of firmware restore-or-search pattern.
     */
    private void restoreOrSearchDac(int channelIdx) {
        if (channelDacValid[channelIdx]) {
            dac.setCode(channelDacCode[channelIdx]);
        } else {
            binarySearch.search(adc, dac, mux, config.getPreampOffsetCounts());
            channelDacCode[channelIdx] = dac.getCurrentCode();
            channelDacValid[channelIdx] = true;
        }
    }

    /**
     * Run the main loop for a specified number of iterations.
     * Port of firmware loop().
     *
     * @param iterations number of loop iterations to run
     */
    public void runLoop(int iterations) {
        for (int i = 0; i < iterations; i++) {
            loopOnce();
        }
    }

    /**
     * Execute one iteration of the main loop.
     */
    private void loopOnce() {
        // Advance simulated time (~2.56ms per chop cycle at ~390 Hz)
        simulatedTimeMs += 3;  // approximate

        // Periodically measure reference filter error
        if (driftCompensator.shouldMeasure()) {
            // Simulate reference measurement
            // Filter error = (VrefRaw - filtered) which depends on drift
            double filterError = config.getRefDriftRateVPerS() * Constants.REF_FILTER_TAU_EFF;
            driftCompensator.measureFilterError(filterError, simulatedTimeMs);
        }

        if (scanState == ScanState.IDLE) {
            loopIdle();
        } else {
            loopScanning();
        }
    }

    private void loopIdle() {
        boolean ok = chopMeasurement.runOneChopCycle(
                chopStats, adc, dac, mux, config.getPreampOffsetCounts());

        if (!ok) {
            // Overflow - binary search and restart
            binarySearch.search(adc, dac, mux, config.getPreampOffsetCounts());
            chopStats.clear();
            idleCounter = 0;
            return;
        }

        if (++idleCounter >= scanConfig.getIntegrationCycles()) {
            idleCounter = 0;
            recordMeasurement(mux.getCurrentChannel(), chopStats, dac.getCurrentCode());
            // Don't clear stats in IDLE mode (matches firmware behavior)
        }
    }

    private void loopScanning() {
        InputChannel scanCh = scanConfig.getChannel(currentScanIndex);
        LowerMoments stats = channelStats[scanCh.getAddress()];

        boolean ok = chopMeasurement.runOneChopCycle(
                stats, adc, dac, mux, config.getPreampOffsetCounts());

        if (!ok) {
            binarySearch.search(adc, dac, mux, config.getPreampOffsetCounts());
            stats.clear();
            chopCycleCount = 0;
            // Save the re-centered DAC code for this channel
            channelDacCode[scanCh.getAddress()] = dac.getCurrentCode();
            channelDacValid[scanCh.getAddress()] = true;
            return;
        }

        chopCycleCount++;

        // Discard settling cycles
        if (chopCycleCount <= Constants.SCAN_SETTLE_CYCLES) {
            stats.clear();
            return;
        }

        // Check if integration is complete
        if (chopCycleCount >= scanConfig.getIntegrationCycles() + Constants.SCAN_SETTLE_CYCLES) {
            recordMeasurement(scanCh, stats, dac.getCurrentCode());

            // Save DAC code for this channel (for fast restore on next visit)
            channelDacCode[scanCh.getAddress()] = dac.getCurrentCode();
            channelDacValid[scanCh.getAddress()] = true;

            currentScanIndex++;
            if (currentScanIndex >= scanConfig.getCount()) {
                currentScanIndex = 0;
                scanCycleCount++;

                // Auto-zero check
                if (scanConfig.isAutoZeroEnabled() &&
                        (scanCycleCount % scanConfig.getAutoZeroInterval()) == 0) {
                    autoZero.perform(adc, dac, mux, binarySearch,
                            voltageComputer, config.getPreampOffsetCounts());
                }
            }

            // Switch to next channel
            InputChannel nextCh = scanConfig.getChannel(currentScanIndex);
            if (nextCh != mux.getCurrentChannel()) {
                mux.selectInputChannel(nextCh);
                restoreOrSearchDac(nextCh.getAddress());
            }
            channelStats[nextCh.getAddress()].clear();
            chopCycleCount = 0;
        }
    }

    private void recordMeasurement(InputChannel channel, LowerMoments stats, short dacCode) {
        double adcMean = stats.mean();
        double adcStdDev = stats.standardDeviation();

        double vxRaw = voltageComputer.computeInputVoltage(adcMean, dacCode, channel);
        double vxUncertainty = voltageComputer.computeInputUncertainty(adcStdDev, channel);

        double refCorrection = driftCompensator.getCorrectionFactor(simulatedTimeMs);
        double vxCorrected = vxRaw * refCorrection;

        if (autoZero.isValid()) {
            vxCorrected -= autoZero.getOffset();
        }

        double driftPpb = (refCorrection - 1.0) * 1e9;

        results.add(new MeasurementResult(
                channel, vxCorrected, vxUncertainty,
                adcMean, adcStdDev, (int) stats.count(),
                dacCode, driftPpb
        ));
    }

    // --- Control methods ---

    public void startScanning() {
        if (scanConfig.getCount() == 0) return;
        scanState = ScanState.SCANNING;
        currentScanIndex = 0;
        scanCycleCount = 0;
        chopCycleCount = 0;

        InputChannel firstCh = scanConfig.getChannel(0);
        mux.selectInputChannel(firstCh);
        restoreOrSearchDac(firstCh.getAddress());
        channelStats[firstCh.getAddress()].clear();
    }

    public void stopScanning() {
        scanState = ScanState.IDLE;
        chopCycleCount = 0;
    }

    // --- Accessors ---

    public List<MeasurementResult> getResults() { return results; }
    public void clearResults() { results.clear(); }
    public ScanConfig getScanConfig() { return scanConfig; }
    public SimulationConfig getConfig() { return config; }
    public SimulatedADC getAdc() { return adc; }
    public SimulatedDAC getDac() { return dac; }
    public SimulatedMux getMux() { return mux; }
    public VoltageComputer getVoltageComputer() { return voltageComputer; }
    public DacCalibrationTable getCalTable() { return calTable; }
    public ReferenceDriftCompensator getDriftCompensator() { return driftCompensator; }
    public AutoZero getAutoZero() { return autoZero; }
    public BinarySearchDAC getBinarySearch() { return binarySearch; }
    public ChopMeasurement getChopMeasurement() { return chopMeasurement; }
    public LowerMoments getChopStats() { return chopStats; }
    public ScanState getScanState() { return scanState; }

    /**
     * Compute overlapping Allan deviation from collected measurement results.
     * Uses voltage values and derives tau0 from integration cycles and chop timing.
     *
     * @return list of (tau, adev, pairsUsed) at octave-spaced taus
     */
    public List<AllanDeviation.Result> computeAllanDeviation() {
        double[] voltages = results.stream()
                .mapToDouble(MeasurementResult::voltage).toArray();
        double tau0 = scanConfig.getIntegrationCycles() * Constants.CHOP_CYCLE_SECONDS;
        return AllanDeviation.computeOctave(voltages, tau0);
    }
}
