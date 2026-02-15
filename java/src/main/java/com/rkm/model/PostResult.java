/*
 * SPDX-License-Identifier: CERN-OHL-P-2.0
 * SPDX-FileCopyrightText: 2025 rkm
 */
package com.rkm.model;

/**
 * POST (Power-On Self-Test) result structure.
 * Maps to firmware lines 39-55.
 */
public class PostResult {
    public boolean adcComm;
    public boolean dacComm;
    public boolean drdyTiming;
    public boolean signalChain;
    public boolean polarity;
    public boolean chopSymmetry;
    public boolean inputMux;
    public boolean references;
    public boolean zeroCal;

    public boolean allPassed() {
        return adcComm && dacComm && drdyTiming &&
               signalChain && polarity && chopSymmetry &&
               inputMux && references && zeroCal;
    }
}
