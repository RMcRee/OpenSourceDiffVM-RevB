# Datron/Wavetek/Fluke 1281 — Selfcal Dual-Reference Architecture

## Instrument Overview

The **Datron 1281** (later sold as Wavetek 1281, then Fluke 1281) is an 8.5-digit metrology-grade
DC voltmeter, broadly comparable to the HP/Agilent 3458A and Fluke 8508A. It is notable for its
"Selfcal" self-calibration system, which allows reference drift to be detected and corrected without
an external standard between annual calibrations.

| Specification | Value |
|---|---|
| Resolution | 8.5 digits |
| DC voltage stability | 3 ppm / year (±5 °C) |
| 10 V range linearity | ±0.1 ppm |
| DC input amp TC | 0.25 µV/°C |

---

## Reference Hardware

The 1281 uses **two independent LTZ1000-based reference modules** (Datron part 400763-1), each
containing:
- Linear LTZ1000 subsurface Zener diode with integrated heater and temperature sensor
- Hermetically sealed Vishay metal-foil resistor networks
- Constant-temperature oven control (on-chip)

The two modules are kept **electrically separate** — they are NOT averaged together. This
independence is the foundation of the Selfcal technique.

---

## Selfcal Architecture

### Core Idea

Any single reference can drift without being detectable. Two independent references allow drift to
be seen as a change in their *ratio*. If the ratio is anchored to a known-good external calibration,
deviations from that anchor are measurable corrections.

### Phase 1: External Calibration (annual)

1. Instrument is calibrated against a traceable standard (e.g., Josephson voltage standard,
   calibrated Fluke 732, or national laboratory transfer standard).
2. Both reference modules are measured under identical conditions.
3. The ratio `R_char = Ref_A / Ref_B` is stored in NV memory as the **Selfcal characterization
   factor**, alongside the normal absolute calibration constants.
4. This ratio is now a traceable anchor — it encodes the relationship between the two references
   at the moment of known-good calibration.

### Phase 2: Self-Calibration (user-initiated, no external standard needed)

1. The instrument measures both reference modules using the same internal measurement chain.
2. Current ratio: `R_now = Ref_A / Ref_B`
3. Delta: `delta_R = R_now - R_char`
4. A third set of calibration constants ("Selfcal corrections") is computed from `delta_R` and
   applied multiplicatively to all subsequent readings.
5. The correction tracks absolute drift because `R_char` was established at external-cal time.

```
Vx_corrected = Vx_measured × (1 + delta_R / R_char)
```

### Limitation

If both references drift at **exactly the same rate** (perfectly correlated), the ratio is
unchanged and drift is invisible. In practice, same-batch LTZ1000 units have similar but not
identical aging curves, so this failure mode is rare. The technique is not a substitute for
periodic external calibration — it extends the validity of the last external cal between annual
intervals.

---

## Why This Is Relevant to the DiffVM

### Current DiffVM Architecture

The DiffVM averages three ADR1001 references in the **analog domain** before the firmware sees
them. The firmware can measure `VrefRaw` (pre-filter average at J3 via MUX channel S3), but
cannot distinguish the contribution of individual references. This means:

- A single ADR1001 that has drifted or failed is invisible to the firmware
- The drift compensation system (filter-lag prediction) operates on the aggregate average

### What a 1281-Inspired Enhancement Would Require

**Hardware change (future revision):** Route 2 of the 3 ADR1001 references to separate MUX inputs
instead of combining all three in the analog network. The firmware could then:
- Periodically measure each subgroup independently
- Compute the inter-reference ratio
- Detect and correct drift in one reference relative to the other

**Software-only analog (no hardware change):** Store a characterized `VrefRaw / nominal` ratio in
flash at external-calibration time (separate from the DAC calibration table). Track long-term
deviation from this ratio as an absolute drift anchor. This is distinct from the current short-term
filter-lag compensation — it would catch slow secular drift over days to months.

### Error Budget Impact

At 1 ppm/year ADR1001 drift (typical after burn-in), the current design with three averaged
references has a correlated drift floor. A dual-reference inter-comparison could detect uncorrelated
drift at the ~0.3 ppm level (one reference drifting against the other two), improving the
long-term absolute accuracy floor.

---

## External References

| Resource | URL |
|---|---|
| xDevs.com — detailed teardown, repair, calibration worklog | https://xdevs.com/fix/d1281/ |
| Datron 1281 datasheet | https://www.testequipmenthq.com/datasheets/DATRON-1281-Datasheet.pdf |
| Wavetek 1281 datasheet | https://www.testequipmenthq.com/datasheets/WAVETEK-1281-Datasheet.pdf |
| LTZ1000 product page (Analog Devices) | https://www.analog.com/en/products/ltz1000.html |
| LTZ1000 Wikipedia | https://en.wikipedia.org/wiki/LTZ1000 |

The **xDevs worklog** is the most technically detailed public resource — it includes high-resolution
PCB photos, circuit tracing, component identification, and a detailed account of the Selfcal
calibration procedure. Recommended as the primary reference.
