#!/usr/bin/env python3
"""
SPDX-License-Identifier: CERN-OHL-P-2.0
SPDX-FileCopyrightText: Copyright 2025, 2026 Randall K McRee

Overlapping Allan Deviation (OADEV) post-processing for DiffVM CSV logs.

Reads CSV log files produced by the DiffVM firmware's "log start" command,
computes OADEV at octave-spaced averaging times, and optionally plots the
results on a log-log scale with noise-type slope guides.

Usage:
    python tools/adev.py measurement_log.csv --channel Vx --plot
    python tools/adev.py measurement_log.csv --plot --output adev.png
"""

import argparse
import csv
import math
import sys


def compute_oadev(readings, tau0):
    """Compute overlapping Allan deviation at octave-spaced taus.

    Args:
        readings: list of voltage readings at uniform interval tau0
        tau0: base measurement interval in seconds

    Returns:
        list of (tau, adev, pairs) tuples sorted by ascending tau
    """
    n = len(readings)
    results = []

    m = 1
    while m <= n // 3:
        adev = _compute_for_m(readings, m)
        if adev is not None:
            pairs = n - 2 * m
            results.append((m * tau0, adev, pairs))
        m *= 2

    return results


def _compute_for_m(readings, m):
    """Compute OADEV for a specific averaging factor m.

    Returns OADEV value, or None if insufficient data (need N >= 2m+1).
    """
    n = len(readings)
    if m < 1 or n < 2 * m + 1:
        return None

    # Prefix sums for efficient moving averages
    prefix = [0.0] * (n + 1)
    for i in range(n):
        prefix[i + 1] = prefix[i] + readings[i]

    # Overlapping differences of block averages
    pairs = n - 2 * m
    sum_sq = 0.0
    for i in range(pairs):
        avg1 = (prefix[i + m] - prefix[i]) / m
        avg2 = (prefix[i + 2 * m] - prefix[i + m]) / m
        diff = avg2 - avg1
        sum_sq += diff * diff

    avar = sum_sq / (2.0 * pairs)
    return math.sqrt(avar)


def parse_csv(filepath, channel_filter=None):
    """Parse a DiffVM CSV log file and extract voltage readings.

    CSV format: timestamp_ms,channel,voltage_V,uncertainty_V,stddev_counts,
                mean_counts,n,dac_code,drift_ppb

    Args:
        filepath: path to CSV file
        channel_filter: if set, only include rows matching this channel name

    Returns:
        (voltages, timestamps) tuple of lists
    """
    voltages = []
    timestamps = []

    with open(filepath, "r") as f:
        reader = csv.reader(f)
        for row in reader:
            # Skip comments and empty lines
            if not row or row[0].startswith("#"):
                continue

            try:
                ts_ms = float(row[0])
                channel = row[1].strip()
                voltage = float(row[2])
            except (ValueError, IndexError):
                continue

            if channel_filter and channel != channel_filter:
                continue

            timestamps.append(ts_ms / 1000.0)  # Convert to seconds
            voltages.append(voltage)

    return voltages, timestamps


def estimate_tau0(timestamps):
    """Estimate tau0 from median timestamp differences."""
    if len(timestamps) < 2:
        return None
    diffs = [timestamps[i + 1] - timestamps[i] for i in range(len(timestamps) - 1)]
    diffs.sort()
    return diffs[len(diffs) // 2]


def format_si(value):
    """Format a value with SI prefix."""
    if value == 0:
        return "0.00 V"
    prefixes = [
        (1e-15, "fV"), (1e-12, "pV"), (1e-9, "nV"), (1e-6, "uV"),
        (1e-3, "mV"), (1e0, "V"), (1e3, "kV"),
    ]
    for scale, unit in prefixes:
        if abs(value) < scale * 1000:
            return f"{value / scale:.3f} {unit}"
    return f"{value:.3e} V"


def print_table(results):
    """Print OADEV results as a formatted table."""
    print(f"{'tau (s)':>12}  {'OADEV':>14}  {'pairs':>8}")
    print(f"{'─' * 12}  {'─' * 14}  {'─' * 8}")
    for tau, adev, pairs in results:
        print(f"{tau:12.3f}  {format_si(adev):>14}  {pairs:8d}")


def plot_adev(results, title="Allan Deviation", output=None):
    """Log-log plot of OADEV vs tau with noise-type slope guides.

    Args:
        results: list of (tau, adev, pairs)
        title: plot title
        output: if set, save to this path instead of showing
    """
    try:
        import matplotlib.pyplot as plt
        import numpy as np
    except ImportError:
        print("Error: matplotlib and numpy required for plotting.", file=sys.stderr)
        print("Install with: pip install matplotlib numpy", file=sys.stderr)
        return

    taus = [r[0] for r in results]
    adevs = [r[1] for r in results]

    fig, ax = plt.subplots(figsize=(10, 7))
    ax.loglog(taus, adevs, "o-", color="#2563eb", linewidth=2,
              markersize=6, label="OADEV")

    # Slope guide lines
    tau_range = np.array([taus[0], taus[-1]])
    mid_adev = adevs[len(adevs) // 2]
    mid_tau = taus[len(taus) // 2]

    # White noise (slope -1/2)
    guide_wn = mid_adev * (tau_range / mid_tau) ** (-0.5)
    ax.loglog(tau_range, guide_wn, "--", color="#94a3b8", alpha=0.6,
              label=r"$\tau^{-1/2}$ (white noise)")

    # Flicker noise (slope 0)
    ax.axhline(y=mid_adev, linestyle=":", color="#cbd5e1", alpha=0.6,
               label=r"$\tau^{0}$ (flicker)")

    # Random walk (slope +1/2)
    guide_rw = mid_adev * (tau_range / mid_tau) ** (0.5)
    ax.loglog(tau_range, guide_rw, "-.", color="#94a3b8", alpha=0.6,
              label=r"$\tau^{+1/2}$ (random walk)")

    ax.set_xlabel("Averaging time τ (s)", fontsize=12)
    ax.set_ylabel("Allan Deviation σ(τ) (V)", fontsize=12)
    ax.set_title(title, fontsize=14)
    ax.legend(fontsize=10)
    ax.grid(True, which="both", alpha=0.3)

    plt.tight_layout()
    if output:
        plt.savefig(output, dpi=150)
        print(f"Plot saved to {output}")
    else:
        plt.show()


def main():
    parser = argparse.ArgumentParser(
        description="Compute Overlapping Allan Deviation from DiffVM CSV logs")
    parser.add_argument("csvfile", help="Input CSV log file")
    parser.add_argument("--channel", "-c", default=None,
                        help="Channel to analyze (e.g., Vx, GND, VrefRaw). "
                             "Default: use all rows.")
    parser.add_argument("--tau0", type=float, default=None,
                        help="Override base measurement interval (seconds). "
                             "Default: auto-detect from timestamps.")
    parser.add_argument("--plot", "-p", action="store_true",
                        help="Show log-log OADEV plot")
    parser.add_argument("--output", "-o", default=None,
                        help="Save plot to file instead of displaying")
    args = parser.parse_args()

    # Parse CSV
    voltages, timestamps = parse_csv(args.csvfile, args.channel)
    if len(voltages) < 3:
        ch_msg = f" for channel '{args.channel}'" if args.channel else ""
        print(f"Error: Need at least 3 readings{ch_msg}, got {len(voltages)}.",
              file=sys.stderr)
        sys.exit(1)

    # Determine tau0
    tau0 = args.tau0
    if tau0 is None:
        tau0 = estimate_tau0(timestamps)
        if tau0 is None or tau0 <= 0:
            print("Error: Could not determine tau0 from timestamps. "
                  "Use --tau0 to specify manually.", file=sys.stderr)
            sys.exit(1)

    print(f"Readings: {len(voltages)}, tau0: {tau0:.4f} s, "
          f"total span: {(len(voltages) - 1) * tau0:.1f} s")
    if args.channel:
        print(f"Channel: {args.channel}")
    print()

    # Compute OADEV
    results = compute_oadev(voltages, tau0)
    if not results:
        print("Error: Not enough data for any OADEV computation.", file=sys.stderr)
        sys.exit(1)

    print_table(results)

    # Plot if requested
    if args.plot or args.output:
        channel_label = f" ({args.channel})" if args.channel else ""
        plot_adev(results, title=f"Allan Deviation{channel_label}",
                  output=args.output)


if __name__ == "__main__":
    main()
