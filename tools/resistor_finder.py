#!/usr/bin/env python3
"""
Resistor combination finder for Susumu RR1220PD-KIT-FILE.
0.5% tolerance, 25 ppm/°C, 0402, 11.5 ohm – 931 kohm, 161 values.

Given a target resistance, finds the best single, series, and parallel
combinations from the kit that minimise error.

Usage:
    python resistor_finder.py <target> [--series | --parallel | --best] [-n N] [--tol T]

    <target>     Resistance value, e.g. 1450  or  1.45k  or  10.5k  or  2.2M
    --series     Search series combinations only (R1 + R2)
    --parallel   Search parallel combinations only (R1 || R2)
    --best       Show best result across all methods (default)
    -n N         Number of results to show per method (default: 5)
    --tol T      Only show results within T% of target (default: no filter)

Examples:
    python resistor_finder.py 1450
    python resistor_finder.py 1.45k --series
    python resistor_finder.py 10.5k --parallel -n 3
    python resistor_finder.py 2.2k --best
    python resistor_finder.py 1450 --tol 1
    python resistor_finder.py 1450 --tol 0.5 --series
    python resistor_finder.py 1450 --best --tol 0.1
"""

import sys
import itertools

# Susumu RR1220PD-KIT-FILE: 161 values, 0.5% tolerance, 25 ppm/°C
# NOTE: Kit is missing 665 ohm and 715 ohm — confirmed gap in this specific kit,
#       not a transcription error. The kohm-scale equivalents (665k, 715k) are
#       present. Verified by physical inspection.
KIT_VALUES = sorted([
    11.5, 13.3, 14.3, 14.7, 16.5, 17.4, 17.8, 20.5, 24.9, 25.5,
    29.4, 30.9, 34.8, 35.7, 40.2, 41.2, 47.5, 54.9, 56.2, 60.4,
    69.8, 80.6, 86.6, 88.7, 107, 124, 133, 143, 147, 162,
    165, 174, 178, 182, 205, 226, 232, 249, 255, 280,
    287, 309, 316, 365, 412, 475, 523, 549, 590, 604,
    649, 887, 976,  # NOTE: 665 and 715 missing from this kit (manufacturing gap)
    1130, 1180, 1270, 1330, 1370, 1400, 1540,
    1690, 1820, 1870, 1960, 2150, 2260, 2320, 2370, 2430, 2550,
    2610, 2800, 3010, 3090, 3160, 3240, 3400, 3480, 3650, 3740,
    3920, 4120, 4530, 4640, 4870, 5360, 5490, 5900, 6040, 6190,
    6810, 6980, 7150, 7870, 8060, 8250, 8870, 9760, 10200, 11300,
    11500, 12100, 12700, 13300, 13700, 14700, 16900, 17400, 19100, 19600,
    21500, 22100, 23700, 24900, 26100, 26700, 28700, 30900, 31600, 32400,
    34800, 35700, 37400, 39200, 40200, 47500, 48700, 54900, 56200, 57600,
    60400, 68100, 73200, 80600, 95300, 102000, 105000, 118000, 121000, 140000,
    143000, 187000, 210000, 226000, 232000, 280000, 301000, 332000, 357000, 365000,
    374000, 402000, 412000, 475000, 499000, 549000, 604000, 665000, 715000, 887000,
    931000,
])


def parse_value(s):
    """Parse a resistance string: '1450', '1.45k', '10.5K', '2.2M'."""
    s = s.strip().lower()
    if s.endswith('m'):
        return float(s[:-1]) * 1e6
    elif s.endswith('k'):
        return float(s[:-1]) * 1e3
    else:
        return float(s)


def format_ohms(r):
    """Format a resistance value as a human-readable string."""
    if r >= 1e6:
        return f"{r/1e6:.4g} Mohm"
    elif r >= 1e3:
        return f"{r/1e3:.4g} kohm"
    else:
        return f"{r:.4g} ohm"


def error_pct(actual, target):
    return (actual - target) / target * 100.0


def find_single(target, n=5, tol=None):
    ranked = sorted(KIT_VALUES, key=lambda r: abs(r - target))
    results = [(r, r, None, error_pct(r, target)) for r in ranked]
    if tol is not None:
        results = [r for r in results if abs(r[-1]) <= tol]
    return results[:n]


def find_series(target, n=5, tol=None):
    results = []
    for r1, r2 in itertools.combinations(KIT_VALUES, 2):
        total = r1 + r2
        results.append((total, r1, r2, error_pct(total, target)))
    results.sort(key=lambda x: abs(x[3]))
    if tol is not None:
        results = [r for r in results if abs(r[-1]) <= tol]
    return results[:n]


def find_parallel(target, n=5, tol=None):
    results = []
    for r1, r2 in itertools.combinations(KIT_VALUES, 2):
        total = (r1 * r2) / (r1 + r2)
        results.append((total, r1, r2, error_pct(total, target)))
    results.sort(key=lambda x: abs(x[3]))
    if tol is not None:
        results = [r for r in results if abs(r[-1]) <= tol]
    return results[:n]


def print_single(results):
    print("\n-- Single resistor --")
    if not results:
        print("  (no combinations within tolerance)")
        return
    for total, r1, _, err in results:
        print(f"  {format_ohms(r1):>12s}   error: {err:+.4f}%")


def print_series(results):
    print("\n-- Series R1 + R2 --")
    if not results:
        print("  (no combinations within tolerance)")
        return
    for total, r1, r2, err in results:
        print(f"  {format_ohms(r1):>12s} + {format_ohms(r2):<12s}"
              f" = {format_ohms(total):>12s}   error: {err:+.4f}%")


def print_parallel(results):
    print("\n-- Parallel R1 || R2 --")
    if not results:
        print("  (no combinations within tolerance)")
        return
    for total, r1, r2, err in results:
        print(f"  {format_ohms(r1):>12s} || {format_ohms(r2):<12s}"
              f" = {format_ohms(total):>12s}   error: {err:+.4f}%")


def main():
    args = sys.argv[1:]
    if not args or args[0] in ('-h', '--help', 'help'):
        print(__doc__)
        sys.exit(0)

    target_str = args[0]
    mode = 'all'
    n = 5
    tol = None

    i = 1
    while i < len(args):
        if args[i] == '--series':
            mode = 'series'
        elif args[i] == '--parallel':
            mode = 'parallel'
        elif args[i] == '--best':
            mode = 'best'
        elif args[i] == '-n' and i + 1 < len(args):
            n = int(args[i + 1])
            i += 1
        elif args[i] == '--tol' and i + 1 < len(args):
            tol = float(args[i + 1])
            i += 1
        i += 1

    target = parse_value(target_str)
    print(f"\nTarget: {format_ohms(target)}  ({target:.6g} ohm)")
    if tol is not None and tol < 0.5:
        print(f"WARNING: --tol {tol}% is tighter than the kit's 0.5% component tolerance;")
        print(f"         actual achieved accuracy may be worse than requested.")
    print("=" * 60)

    if mode == 'series':
        print_series(find_series(target, n, tol))

    elif mode == 'parallel':
        print_parallel(find_parallel(target, n, tol))

    elif mode == 'best':
        s = find_single(target, 1, tol)
        se = find_series(target, 1, tol)
        pa = find_parallel(target, 1, tol)
        if not s and not se and not pa:
            print("\n-- Best overall --")
            print("  (no combinations within tolerance)")
            print()
            return
        candidates_raw = []
        if s:
            candidates_raw.append(('single', s[0]))
        if se:
            candidates_raw.append(('series', se[0]))
        if pa:
            candidates_raw.append(('parallel', pa[0]))
        method, best = min(candidates_raw, key=lambda x: abs(x[1][3]))
        print(f"\n-- Best overall ({method}) --")
        if method == 'single':
            print(f"  {format_ohms(best[1]):>12s}   error: {best[3]:+.4f}%")
        elif method == 'series':
            print(f"  {format_ohms(best[1]):>12s} + {format_ohms(best[2]):<12s}"
                  f" = {format_ohms(best[0]):>12s}   error: {best[3]:+.4f}%")
        else:
            print(f"  {format_ohms(best[1]):>12s} || {format_ohms(best[2]):<12s}"
                  f" = {format_ohms(best[0]):>12s}   error: {best[3]:+.4f}%")

    else:  # all
        print_single(find_single(target, n, tol))
        print_series(find_series(target, n, tol))
        print_parallel(find_parallel(target, n, tol))

    print()


if __name__ == "__main__":
    main()
