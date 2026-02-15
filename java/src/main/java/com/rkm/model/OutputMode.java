/*
 * SPDX-License-Identifier: CERN-OHL-P-2.0
 * SPDX-FileCopyrightText: Copyright 2025, 2026 Randall K McRee
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
