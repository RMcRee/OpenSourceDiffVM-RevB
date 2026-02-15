package com.rkm.cal;

import com.rkm.Constants;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

class DacCalibrationTableTest {

    @Test
    void testNominalAtZero() {
        DacCalibrationTable table = new DacCalibrationTable();
        double v = table.dacCodeToVoltage((short) 0);
        assertEquals(0.0, v, 1e-15, "Code 0 should produce 0V");
    }

    @Test
    void testNominalPositiveFullScale() {
        DacCalibrationTable table = new DacCalibrationTable();
        // Code 32767 → 32767 × DAC_LSB_V ≈ 4.99985V
        double v = table.dacCodeToVoltage((short) 32767);
        double expected = 32767.0 * Constants.DAC_LSB_V;
        assertEquals(expected, v, 1e-12);
    }

    @Test
    void testNominalNegativeFullScale() {
        DacCalibrationTable table = new DacCalibrationTable();
        // Code -32768 → -32768 × DAC_LSB_V ≈ -5.0V
        double v = table.dacCodeToVoltage((short) -32768);
        double expected = -32768.0 * Constants.DAC_LSB_V;
        assertEquals(expected, v, 1e-12);
    }

    @Test
    void testNominalAt32764() {
        DacCalibrationTable table = new DacCalibrationTable();
        // Code 32764 is 14-bit aligned, near +5V
        double v = table.dacCodeToVoltage((short) 32764);
        double expected = 32764.0 * Constants.DAC_LSB_V;
        assertEquals(expected, v, 1e-12);
        assertTrue(v > 4.999, "32764 should be near 5V: " + v);
    }

    @Test
    void testCalibratedInterpolation() {
        DacCalibrationTable table = new DacCalibrationTable();

        // Set two adjacent calibration points with known offsets
        // Index for code 0: ((0+32768) & 0xFFFF) >>> 2 = 8192
        // Code 0 → index 8192, code 4 → index 8193
        table.setCalPoint((short) 0, 0.001);     // 1mV offset at code 0
        table.setCalPoint((short) 4, 0.002);     // 2mV at code 4
        table.markValid();

        // Code 0 should return 0.001 (exact table entry)
        assertEquals(0.001, table.dacCodeToVoltage((short) 0), 1e-15);

        // Code 4 should return 0.002 (exact table entry)
        assertEquals(0.002, table.dacCodeToVoltage((short) 4), 1e-15);

        // Code 1 should interpolate: 0.001 + (0.002-0.001) * 1/4 = 0.00125
        assertEquals(0.00125, table.dacCodeToVoltage((short) 1), 1e-15);

        // Code 2 should interpolate: 0.001 + (0.002-0.001) * 2/4 = 0.0015
        assertEquals(0.0015, table.dacCodeToVoltage((short) 2), 1e-15);

        // Code 3 should interpolate: 0.001 + (0.002-0.001) * 3/4 = 0.00175
        assertEquals(0.00175, table.dacCodeToVoltage((short) 3), 1e-15);
    }

    @Test
    void testUncalibratedUsesNominal() {
        DacCalibrationTable table = new DacCalibrationTable();
        assertFalse(table.isValid());

        // Even with cal points set, if not marked valid, use nominal
        table.setCalPoint((short) 0, 999.0);
        double v = table.dacCodeToVoltage((short) 0);
        assertEquals(0.0, v, 1e-15, "Uncalibrated should use nominal");
    }

    @Test
    void testInitNominalResetsValid() {
        DacCalibrationTable table = new DacCalibrationTable();
        table.markValid();
        assertTrue(table.isValid());

        table.initNominal();
        assertFalse(table.isValid());
    }

    @Test
    void testTableSize() {
        DacCalibrationTable table = new DacCalibrationTable();
        // Should have 16384 entries
        // Verify a few entries in the nominal table
        // Index 0 → code -32768 → -32768 * DAC_LSB_V
        double v0 = table.getTableEntry(0);
        assertEquals(-32768.0 * Constants.DAC_LSB_V, v0, 1e-12);

        // Index 8192 → code 0 → 0V
        double vMid = table.getTableEntry(8192);
        assertEquals(0.0, vMid, 1e-12);
    }
}
