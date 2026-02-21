/*
 * SPDX-License-Identifier: CERN-OHL-P-2.0
 * SPDX-FileCopyrightText: Copyright 2025, 2026 Randall K McRee
 */
package com.rkm;

/**
 * All system constants from the DiffVM firmware.
 * Maps to firmware lines 78-202.
 */
public final class Constants {

    private Constants() {}

    // ---- ADC: ADS127L11, 24-bit, differential ----
    public static final double ADC_VREF = 2.5;                                         // ADC reference voltage (V)
    public static final double ADC_FSR_CODES = 16777216.0;                              // 2^24 full scale codes
    public static final double ADC_LSB_V = (2.0 * ADC_VREF) / ADC_FSR_CODES;           // ~298 nV/LSB

    public static final int ADC_OVERFLOW_POS = 7_549_747;   // ~90% of 0x7FFFFF
    public static final int ADC_OVERFLOW_NEG = -7_549_747;   // ~90% of -0x800000
    public static final int ADC_MAX_CODE = 8_388_607;        // 2^23 - 1
    public static final int ADC_MIN_CODE = -8_388_608;       // -2^23

    // ---- DAC: AD5760, 16-bit, two's complement, 2x mode ----
    public static final double DAC_VREF = 5.0;                                         // DAC full-scale voltage (V)
    public static final double DAC_FSR_CODES = 65536.0;                                 // 2^16 codes
    public static final double DAC_LSB_V = (2.0 * DAC_VREF) / DAC_FSR_CODES;           // ~152.6 µV/LSB

    public static final int DAC_SETTLE_MIN_MS = 80;
    public static final int DAC_SETTLE_MAX_MS = 300;

    // ---- Preamp: 4x AD8428 cascade ----
    public static final double PREAMP_GAIN = 2000.0;

    // ---- Voltage divider ratios ----
    public static final double DIVIDER_RATIO_10 = 10.0;       // ±50V: mux bypassed
    public static final double DIVIDER_RATIO_100 = 108.76;     // ±500V
    public static final double DIVIDER_RATIO_1000 = 984.65;    // ±5kV

    // ---- Timing ----
    public static final int SETTLE_US = 300;          // Guard time after mux switch
    public static final int DISCARD_SAMPLES = 1;      // Samples tossed after edge
    public static final int GOOD_SAMPLES = 3;         // Samples averaged per half-cycle
    public static final int PULSE_WIDTH_US = 5;       // CI_TMUXSEL pulse duration

    // ---- ADC sample rate ----
    public static final double ADC_FCLK_HZ = 25_000_000.0;
    public static final double ADC_FMOD_HZ = ADC_FCLK_HZ / 2.0;
    public static final double OSR_TOTAL = 3200.0;
    public static final double EST_FSPS = ADC_FMOD_HZ / OSR_TOTAL;  // ~3906.25 SPS

    // ---- DAC calibration table ----
    public static final int DAC_CAL_TABLE_SIZE = 16384;

    // ---- Reference drift compensation ----
    public static final double NOMINAL_REF_V = 5.0;
    public static final short REF_MEASURE_DAC_CODE = 32764;
    public static final int REF_SAMPLE_INTERVAL = 385;
    public static final double REF_FILTER_TAU_EFF = 1.05;  // seconds
    public static final int FILTER_HISTORY_SIZE = 8;
    public static final int REF_MEASURE_ITERATIONS = 10;

    // Step response lookup table: fc=0.3Hz, Q=0.566 filter
    // Sampled at 0.1s intervals from 0 to 8 seconds (81 points)
    public static final double REF_STEP_DT = 0.1;
    public static final double[] REF_STEP_RESPONSE = {
        0.000, 0.019, 0.066, 0.132, 0.208, 0.289, 0.371, 0.450,  // 0.0-0.7s
        0.523, 0.592, 0.654, 0.711, 0.762, 0.808, 0.848, 0.884,  // 0.8-1.5s
        0.914, 0.940, 0.962, 0.979, 0.992, 1.001, 1.008, 1.013,  // 1.6-2.3s
        1.016, 1.018, 1.019, 1.019, 1.018, 1.016, 1.014, 1.012,  // 2.4-3.1s
        1.010, 1.008, 1.006, 1.004, 1.003, 1.002, 1.001, 1.000,  // 3.2-3.9s
        1.000, 1.000, 1.000, 1.000, 1.000, 1.000, 1.000, 1.000,  // 4.0-4.7s
        1.000, 1.000, 1.000, 1.000, 1.000, 1.000, 1.000, 1.000,  // 4.8-5.5s
        1.000, 1.000, 1.000, 1.000, 1.000, 1.000, 1.000, 1.000,  // 5.6-6.3s
        1.000, 1.000, 1.000, 1.000, 1.000, 1.000, 1.000, 1.000,  // 6.4-7.1s
        1.000, 1.000, 1.000, 1.000, 1.000, 1.000, 1.000, 1.000,  // 7.2-7.9s
        1.000                                                       // 8.0s
    };
    public static final int REF_STEP_LEN = REF_STEP_RESPONSE.length;  // 81

    // ---- Chop cycle timing ----
    // One chop cycle = 2 half-cycles, each with (DISCARD + GOOD + settle) samples
    public static final double CHOP_CYCLE_SECONDS =
            (DISCARD_SAMPLES + GOOD_SAMPLES + 1) * 2.0 / EST_FSPS;  // ~0.00256 s

    // ---- DAC calibration build ----
    public static final int CAL_BUILD_CYCLES = 5;   // Chop cycles per table entry during auto build
    public static final int CAL_POINT_CYCLES = 20;  // Chop cycles per cal point capture
    public static final int CAL_BUILD_WINDOW = 2;   // Table entries swept each side of anchor (±2 = 5 total)
    public static final int AUTOZERO_CYCLES  = 20;  // Chop cycles per auto-zero measurement

    // ---- Scanning ----
    public static final int SCAN_SETTLE_CYCLES = 5;
    public static final int MAX_SCAN_CHANNELS = 7;

    // ---- Input mux ----
    public static final int INMUX_SETTLE_US = 100;

    // ---- POST ----
    public static final double POST_ZERO_THRESHOLD_NV = 1000.0;
}
