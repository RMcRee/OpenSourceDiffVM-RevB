/*
 * SPDX-License-Identifier: CERN-OHL-P-2.0
 * SPDX-FileCopyrightText: Copyright 2025, 2026 Randall K McRee
 */
package com.rkm.model;

/**
 * Scanning state machine states.
 * Maps to firmware line 1076.
 */
public enum ScanState {
    IDLE,
    SCANNING
}
