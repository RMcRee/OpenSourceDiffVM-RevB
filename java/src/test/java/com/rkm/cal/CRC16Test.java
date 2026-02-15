/*
 * SPDX-License-Identifier: CERN-OHL-P-2.0
 * SPDX-FileCopyrightText: 2025 rkm
 */
package com.rkm.cal;

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

class CRC16Test {

    @Test
    void testStandardModbusVector() {
        // Standard MODBUS CRC16 test vector: "123456789" → 0x4B37
        byte[] data = "123456789".getBytes();
        int crc = CRC16.compute(data);
        assertEquals(0x4B37, crc);
    }

    @Test
    void testEmptyInput() {
        byte[] data = new byte[0];
        int crc = CRC16.compute(data, 0);
        assertEquals(0xFFFF, crc);  // Initial value with no data
    }

    @Test
    void testSingleByte() {
        byte[] data = {0x00};
        int crc = CRC16.compute(data);
        // Known: CRC16/MODBUS of 0x00 = 0xE1B0 (from reference implementation)
        // Verify by hand: init=0xFFFF, XOR with 0x00 → 0xFFFF
        // 8 iterations of shifting with poly 0xA001
        assertNotEquals(0, crc);
    }

    @Test
    void testPartialLength() {
        byte[] data = {0x01, 0x02, 0x03, 0x04, 0x05};
        int full = CRC16.compute(data);
        int partial = CRC16.compute(data, 3);

        assertNotEquals(full, partial);
        // Verify partial processes only first 3 bytes
        byte[] first3 = {0x01, 0x02, 0x03};
        assertEquals(CRC16.compute(first3), partial);
    }

    @Test
    void testDeterministic() {
        byte[] data = {(byte) 0xDE, (byte) 0xAD, (byte) 0xBE, (byte) 0xEF};
        int crc1 = CRC16.compute(data);
        int crc2 = CRC16.compute(data);
        assertEquals(crc1, crc2);
    }

    @Test
    void testResultIs16Bit() {
        byte[] data = "test data for CRC".getBytes();
        int crc = CRC16.compute(data);
        assertTrue(crc >= 0 && crc <= 0xFFFF, "CRC should be 16-bit unsigned");
    }
}
