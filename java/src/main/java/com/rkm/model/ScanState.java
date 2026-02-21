/*
 * SPDX-License-Identifier: CERN-OHL-P-2.0
 * SPDX-FileCopyrightText: Copyright 2025, 2026 Randall K McRee
 */
package com.rkm.model;

/**
 * Scanning state machine states.
 * Maps to firmware ScanState enum (ONE_CHANNEL / SCANNING).
 */
public enum ScanState {
    ONE_CHANNEL,
    SCANNING
}
