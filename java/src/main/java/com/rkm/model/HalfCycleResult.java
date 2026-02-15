package com.rkm.model;

/**
 * Result of one half-cycle acquisition.
 * Maps to firmware lines 22-25.
 */
public record HalfCycleResult(long sum, boolean overflow) {
}
