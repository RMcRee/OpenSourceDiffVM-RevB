/*
 * SPDX-License-Identifier: CERN-OHL-P-2.0
 * SPDX-FileCopyrightText: Copyright 2025, 2026 Randall K McRee
 */
package com.rkm.hw;

import com.rkm.Constants;
import com.rkm.model.DividerRatio;
import com.rkm.model.InputChannel;
import com.rkm.sim.SimulationConfig;

/**
 * Simulated TMUX7234 + MUX36S08 multiplexer system.
 * Tracks current input channel, divider ratio, and TMUX select state.
 * Port of firmware mux functions (lines 666-731, 280-345).
 */
public class SimulatedMux {

    private InputChannel currentChannel = InputChannel.Vx;
    private DividerRatio currentDividerRatio = DividerRatio.Div10;
    private boolean tmuxSelState = false;  // false=LOW, true=HIGH
    private final SimulationConfig config;

    public SimulatedMux(SimulationConfig config) {
        this.config = config;
    }

    /**
     * Select input channel (MUX36S08).
     * Port of firmware selectInputChannel().
     */
    public void selectInputChannel(InputChannel channel) {
        currentChannel = channel;
    }

    /**
     * Select HV divider ratio (MUX36D04).
     * Also switches input to HVDivider channel.
     * Port of firmware selectDividerRatio().
     */
    public void selectDividerRatio(DividerRatio ratio) {
        currentDividerRatio = ratio;
        currentChannel = InputChannel.HVDivider;
    }

    /**
     * Get the signal voltage at the current input, based on channel selection.
     * This is the voltage that appears at the preamp input (before chopping).
     */
    public double getSignalVoltage() {
        return switch (currentChannel) {
            case Vx -> config.getInputVoltageVx();
            case GND -> config.getGndOffset();
            case VrefRaw -> config.getVrefRawVoltage();
            case HVDivider -> config.getInputVoltageVx() / getInputDividerRatio();
            default -> 0.0;  // Spare channels
        };
    }

    /**
     * Get the input divider ratio for the current channel.
     * Port of firmware getInputDividerRatio().
     */
    public double getInputDividerRatio() {
        if (currentChannel == InputChannel.HVDivider) {
            return currentDividerRatio.getRatio();
        }
        return 1.0;
    }

    /**
     * Toggle the TMUX select state (chop).
     * Port of firmware chop().
     */
    public void chop() {
        tmuxSelState = !tmuxSelState;
    }

    /**
     * Get the voltage at the ADC input based on current mux and chop state.
     * This implements the chopping signal routing:
     *   TMUX LOW:  ADC reads (Vx - DAC) × PREAMP_GAIN
     *   TMUX HIGH: ADC reads (DAC - Vx) × PREAMP_GAIN
     *
     * @param dacVoltage current DAC output voltage
     * @param preampOffset simulated DC offset in ADC counts (cancelled by chopping)
     * @return voltage at ADC input
     */
    public double getAdcInputVoltage(double dacVoltage, double preampOffset) {
        double signalV = getSignalVoltage();
        double diff;
        if (!tmuxSelState) {
            // TMUXSEL=LOW: (Vx - DAC) × gain
            diff = (signalV - dacVoltage) * Constants.PREAMP_GAIN;
        } else {
            // TMUXSEL=HIGH: (DAC - Vx) × gain
            diff = (dacVoltage - signalV) * Constants.PREAMP_GAIN;
        }
        // Add preamp DC offset (in volts at ADC input)
        return diff + preampOffset * Constants.ADC_LSB_V;
    }

    public InputChannel getCurrentChannel() { return currentChannel; }
    public DividerRatio getCurrentDividerRatio() { return currentDividerRatio; }
    public boolean isTmuxSelState() { return tmuxSelState; }
    public void setTmuxSelState(boolean state) { tmuxSelState = state; }
}
