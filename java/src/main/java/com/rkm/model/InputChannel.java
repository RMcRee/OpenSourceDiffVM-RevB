/*
 * SPDX-License-Identifier: CERN-OHL-P-2.0
 * SPDX-FileCopyrightText: Copyright 2025, 2026 Randall K McRee
 */
package com.rkm.model;

/**
 * Input channel enumeration mapping to MUX36S08 address lines.
 * Maps to firmware InputChannel enum.
 */
public enum InputChannel {
    Vx1(0, "Vx1 (\u00b15V)", "Vx1"),
    GND(1, "GND", "GND"),
    VrefRaw(2, "VrefRaw", "VrefRaw"),
    Vx2(3, "Vx2", "Vx2"),
    Vx3(4, "Vx3", "Vx3"),
    HVDiv(5, "HVDiv", "HVDiv"),
    Vx4(6, "Vx4", "Vx4"),
    Vx5(7, "Vx5", "Vx5");

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
