/*
 * SPDX-License-Identifier: CERN-OHL-P-2.0
 * SPDX-FileCopyrightText: 2025 rkm
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
