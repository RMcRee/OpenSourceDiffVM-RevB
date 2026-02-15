/*
 * SPDX-License-Identifier: CERN-OHL-P-2.0
 * SPDX-FileCopyrightText: 2025 rkm
 */
package com.rkm.model;

/**
 * Input channel enumeration mapping to MUX36S08 address lines.
 * Maps to firmware lines 102-111.
 */
public enum InputChannel {
    Vx(0, "Vx (±5V)", "Vx"),
    GND(1, "GND", "GND"),
    VrefRaw(2, "VrefRaw", "VrefRaw"),
    Spare1(3, "Spare1", "Spare1"),
    Spare2(4, "Spare2", "Spare2"),
    HVDivider(5, "HVDivider", "HVDivider"),
    Spare3(6, "Spare3", "Spare3"),
    Spare4(7, "Spare4", "Spare4");

    private final int address;
    private final String displayName;
    private final String shortName;

    InputChannel(int address, String displayName, String shortName) {
        this.address = address;
        this.displayName = displayName;
        this.shortName = shortName;
    }

    public int getAddress() { return address; }
    public String getDisplayName() { return displayName; }
    public String getShortName() { return shortName; }

    /**
     * Parse channel name (case-insensitive).
     * Maps to firmware parseChannelName().
     */
    public static InputChannel fromName(String name) {
        for (InputChannel ch : values()) {
            if (ch.shortName.equalsIgnoreCase(name)) {
                return ch;
            }
        }
        return null;
    }
}
