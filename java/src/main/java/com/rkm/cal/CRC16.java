/*
 * SPDX-License-Identifier: CERN-OHL-P-2.0
 * SPDX-FileCopyrightText: 2025 rkm
 */
package com.rkm.cal;

/**
 * CRC16 (MODBUS variant) checksum computation.
 * Port of firmware lines 2389-2399.
 * Polynomial: 0xA001 (bit-reversed 0x8005).
 */
public final class CRC16 {

    private CRC16() {}

    /**
     * Compute CRC16 MODBUS checksum over a byte buffer.
     *
     * @param data byte array to checksum
     * @param len  number of bytes to process
     * @return 16-bit CRC value (unsigned, returned as int)
     */
    public static int compute(byte[] data, int len) {
        int crc = 0xFFFF;
        for (int i = 0; i < len; i++) {
            crc ^= (data[i] & 0xFF);
            for (int j = 0; j < 8; j++) {
                if ((crc & 1) != 0) {
                    crc = (crc >>> 1) ^ 0xA001;
                } else {
                    crc >>>= 1;
                }
            }
        }
        return crc & 0xFFFF;
    }

    /**
     * Convenience: compute over the entire array.
     */
    public static int compute(byte[] data) {
        return compute(data, data.length);
    }
}
