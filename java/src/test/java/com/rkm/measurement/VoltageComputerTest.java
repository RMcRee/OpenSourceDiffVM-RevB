/*
 * SPDX-License-Identifier: CERN-OHL-P-2.0
 * SPDX-FileCopyrightText: Copyright 2025, 2026 Randall K McRee
 */
package com.rkm.measurement;

import com.rkm.Constants;
import com.rkm.cal.DacCalibrationTable;
import com.rkm.model.DividerRatio;
import com.rkm.model.InputChannel;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

class VoltageComputerTest {

    private DacCalibrationTable calTable;
    private VoltageComputer computer;

    @BeforeEach
    void setUp() {
        calTable = new DacCalibrationTable();
        computer = new VoltageComputer(calTable);
    }

    @Test
    void testZeroVoltage() {
        // DAC=0, ADC mean=0 → Vx = 0V
        double v = computer.computeInputVoltage(0.0, (short) 0, InputChannel.Vx1);
        assertEquals(0.0, v, 1e-15);
    }

    @Test
    void testDacOnlyVoltage() {
        // DAC=32764 (~5V), ADC mean=0 → Vx = DAC voltage
        double v = computer.computeInputVoltage(0.0, (short) 32764, InputChannel.Vx1);
        double expected = 32764.0 * Constants.DAC_LSB_V;
        assertEquals(expected, v, 1e-12);
        assertTrue(v > 4.999, "Should be near 5V: " + v);
    }

    @Test
    void testAdcOnlyVoltage() {
        // DAC=0, ADC mean=1000 → Vx = (1000 × ADC_LSB_V) / PREAMP_GAIN
        double v = computer.computeInputVoltage(1000.0, (short) 0, InputChannel.Vx1);
        double expected = (1000.0 * Constants.ADC_LSB_V) / Constants.PREAMP_GAIN;
        assertEquals(expected, v, 1e-15);
    }

    @Test
    void testCombinedVoltage() {
        // DAC=1000, ADC mean=500
        double v = computer.computeInputVoltage(500.0, (short) 1000, InputChannel.Vx1);
        double dacV = 1000.0 * Constants.DAC_LSB_V;
        double adcV = (500.0 * Constants.ADC_LSB_V) / Constants.PREAMP_GAIN;
        assertEquals(dacV + adcV, v, 1e-15);
    }

    @Test
    void testHVDividerScaling() {
        computer.setCurrentDividerRatio(DividerRatio.Div10);
        double v10 = computer.computeInputVoltage(0.0, (short) 1000, InputChannel.HVDiv);
        double vx = computer.computeInputVoltage(0.0, (short) 1000, InputChannel.Vx1);

        assertEquals(vx * Constants.DIVIDER_RATIO_10, v10, 1e-12);

        computer.setCurrentDividerRatio(DividerRatio.Div100);
        double v100 = computer.computeInputVoltage(0.0, (short) 1000, InputChannel.HVDiv);
        assertEquals(vx * Constants.DIVIDER_RATIO_100, v100, 1e-12);
    }

    @Test
    void testNonHVChannelNoDivision() {
        computer.setCurrentDividerRatio(DividerRatio.Div1000);
        double vVx = computer.computeInputVoltage(0.0, (short) 1000, InputChannel.Vx1);
        double vGnd = computer.computeInputVoltage(0.0, (short) 1000, InputChannel.GND);
        double vRef = computer.computeInputVoltage(0.0, (short) 1000, InputChannel.VrefRaw);

        // Non-HV channels should not be affected by divider setting
        double expected = 1000.0 * Constants.DAC_LSB_V;
        assertEquals(expected, vVx, 1e-12);
        assertEquals(expected, vGnd, 1e-12);
        assertEquals(expected, vRef, 1e-12);
    }

    @Test
    void testUncertainty() {
        double unc = computer.computeInputUncertainty(10.0, InputChannel.Vx1);
        double expected = (10.0 * Constants.ADC_LSB_V) / Constants.PREAMP_GAIN;
        assertEquals(expected, unc, 1e-15);
    }

    @Test
    void testFormatVoltageVolts() {
        String s = VoltageComputer.formatVoltage(1.23456789, 9);
        assertTrue(s.contains("V"), "Should contain V: " + s);
        assertTrue(s.startsWith("1.23456789"), "Should have 9 sig digits: " + s);
    }

    @Test
    void testFormatVoltageMillivolts() {
        String s = VoltageComputer.formatVoltage(0.001234, 6);
        assertTrue(s.contains("mV"), "Should use mV: " + s);
    }

    @Test
    void testFormatVoltageMicrovolts() {
        String s = VoltageComputer.formatVoltage(0.00000123, 3);
        assertTrue(s.contains("uV"), "Should use uV: " + s);
    }

    @Test
    void testFormatVoltageNanovolts() {
        String s = VoltageComputer.formatVoltage(1.5e-9, 3);
        assertTrue(s.contains("nV"), "Should use nV: " + s);
    }

    @Test
    void testFormatVoltagePicovolts() {
        String s = VoltageComputer.formatVoltage(5e-11, 2);
        assertTrue(s.contains("pV"), "Should use pV: " + s);
    }

    @Test
    void testFormatVoltageNegative() {
        String s = VoltageComputer.formatVoltage(-0.5, 3);
        assertTrue(s.contains("-"), "Should be negative: " + s);
        assertTrue(s.contains("mV"), "Should use mV: " + s);
    }
}
