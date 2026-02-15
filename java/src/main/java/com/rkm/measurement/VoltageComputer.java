/*
 * SPDX-License-Identifier: CERN-OHL-P-2.0
 * SPDX-FileCopyrightText: Copyright 2025, 2026 Randall K McRee
 */
package com.rkm.measurement;

import com.rkm.Constants;
import com.rkm.cal.DacCalibrationTable;
import com.rkm.model.DividerRatio;
import com.rkm.model.InputChannel;

/**
 * Precision voltage computation using 64-bit double precision.
 * Port of firmware lines 347-444.
 */
public class VoltageComputer {

    private final DacCalibrationTable calTable;
    private DividerRatio currentDividerRatio = DividerRatio.Div10;

    public VoltageComputer(DacCalibrationTable calTable) {
        this.calTable = calTable;
    }

    /**
     * Compute the measured input voltage.
     * Port of firmware computeInputVoltage().
     *
     * Vx = (DAC_voltage + preamp_delta) × divider_ratio
     *
     * @param adcMean  Mean ADC reading (counts)
     * @param dacCode  Current DAC code
     * @param channel  Current input channel
     * @return measured voltage at input terminal (volts)
     */
    public double computeInputVoltage(double adcMean, short dacCode, InputChannel channel) {
        double dacVoltage = calTable.dacCodeToVoltage(dacCode);
        double preampDelta = (adcMean * Constants.ADC_LSB_V) / Constants.PREAMP_GAIN;
        double vxAtPreamp = dacVoltage + preampDelta;
        double dividerRatio = getInputDividerRatio(channel);
        return vxAtPreamp * dividerRatio;
    }

    /**
     * Compute measurement uncertainty (standard deviation) in volts.
     * Port of firmware computeInputUncertainty().
     */
    public double computeInputUncertainty(double adcStdDev, InputChannel channel) {
        double uncertaintyAtPreamp = (adcStdDev * Constants.ADC_LSB_V) / Constants.PREAMP_GAIN;
        double dividerRatio = getInputDividerRatio(channel);
        return uncertaintyAtPreamp * dividerRatio;
    }

    /**
     * Get the divider ratio for an input channel.
     * Port of firmware getInputDividerRatio().
     */
    public double getInputDividerRatio(InputChannel channel) {
        if (channel == InputChannel.HVDivider) {
            return currentDividerRatio.getRatio();
        }
        return 1.0;
    }

    /**
     * Format voltage with appropriate SI prefix.
     * Port of firmware formatVoltage().
     */
    public static String formatVoltage(double voltage, int sigDigits) {
        double absV = Math.abs(voltage);

        String unit;
        double scale;

        if (absV >= 1.0) {
            unit = "V";
            scale = 1.0;
        } else if (absV >= 1e-3) {
            unit = "mV";
            scale = 1e3;
        } else if (absV >= 1e-6) {
            unit = "uV";
            scale = 1e6;
        } else if (absV >= 1e-9) {
            unit = "nV";
            scale = 1e9;
        } else {
            unit = "pV";
            scale = 1e12;
        }

        double scaledV = voltage * scale;
        int intDigits = (absV * scale >= 1.0)
                ? (int) Math.log10(absV * scale) + 1
                : 1;
        int decimals = sigDigits - intDigits;
        if (decimals < 0) decimals = 0;

        return String.format("%." + decimals + "f %s", scaledV, unit);
    }

    public void setCurrentDividerRatio(DividerRatio ratio) {
        this.currentDividerRatio = ratio;
    }

    public DividerRatio getCurrentDividerRatio() {
        return currentDividerRatio;
    }
}
