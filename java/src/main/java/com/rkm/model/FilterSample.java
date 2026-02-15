/*
 * SPDX-License-Identifier: CERN-OHL-P-2.0
 * SPDX-FileCopyrightText: 2025 rkm
 */
package com.rkm.model;

/**
 * One filter error measurement with timestamp.
 * Maps to firmware lines 193-196.
 */
public record FilterSample(double error, long timestampMs) {
}
