package com.rkm.model;

import com.rkm.Constants;

/**
 * HV divider ratio selection (MUX36D04 address).
 * Maps to firmware lines 120-125.
 */
public enum DividerRatio {
    Div10(0, Constants.DIVIDER_RATIO_10, "÷10 (±50V)"),
    Div100(1, Constants.DIVIDER_RATIO_100, "÷100 (±500V)"),
    Div1000(2, Constants.DIVIDER_RATIO_1000, "÷1000 (±5kV)"),
    GND(3, Constants.DIVIDER_RATIO_10, "GND (cal)");

    private final int address;
    private final double ratio;
    private final String displayName;

    DividerRatio(int address, double ratio, String displayName) {
        this.address = address;
        this.ratio = ratio;
        this.displayName = displayName;
    }

    public int getAddress() { return address; }
    public double getRatio() { return ratio; }
    public String getDisplayName() { return displayName; }
}
