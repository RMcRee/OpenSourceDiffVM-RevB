/*
 * SPDX-License-Identifier: CERN-OHL-P-2.0
 * SPDX-FileCopyrightText: 2025 rkm
 */
package com.rkm.model;

/**
 * Output format for measurement results.
 * Maps to firmware lines 1030-1034.
 */
public enum OutputMode {
    Human,
    CSV,
    Plotter
}
