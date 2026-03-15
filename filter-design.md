# Precision Low-Pass Filter Design Workflow

This workflow produces filters whose actual characteristics (step response, −3 dB frequency, Q)
match the design intent, by calibrating resistor values to the measured capacitance of the
specific capacitors installed. 0.1% resistors compensate for 5% capacitor tolerance, so the
result is bang on rather than left to chance.

---

## Step 1 — Choose the Filter Type and Target Specification

Use a filter calculator to select topology (Butterworth, Chebyshev, Bessel, Sallen-Key, etc.),
order, −3 dB frequency, and Q. Good calculators:

- Okawa Electric (Sallen-Key and multi-feedback): http://sim.okawa-denshi.jp/en/OPseikiLowkeisan.htm
- Analog Devices Filter Wizard (full active filter synthesis): https://tools.analog.com/en/filterwizard/

Work through the trade-offs here — filter order, noise bandwidth, and the −3 dB point interact.
Higher order gives sharper roll-off but more poles to compensate. Use the calculator to make
these constraints visible before committing to a topology.

---

## Step 2 — Choose the Capacitors First

Good capacitor dielectric matters more than resistance precision at this stage.

**Dielectric preference (best to acceptable):**
1. Polypropylene film (e.g. Panasonic ECW series) — very low loss, excellent stability
2. Polyester film (MKS2 / MKP) — good for less critical nodes
3. C0G/NP0 ceramic — acceptable for small values only

Film capacitors are physically large. As part of the design, use the smallest capacitance values
that still meet the frequency and noise requirements. This is another trade-off to resolve at
the calculator stage: smaller C → larger R → more thermal noise from the resistors.

**Target part:** Panasonic ECW series polypropylene is a good default choice.

**Practical tip:** It is very useful to keep an assortment of film capacitors and 0.1% resistors
on hand for experimentation. Having a range of values available makes it easy to iterate at the
bench — measure a few candidates, pick the best pair, and calculate resistors on the spot without
waiting for parts to arrive.

**Recommended resistor kit:** Susumu RR1220PD-KIT-FILE — 0402 0.1% thin-film resistors covering
the full E96 range, 25 ppm/°C temperature coefficient. The low TC is important for precision
filters: a resistor that drifts with temperature shifts the −3 dB frequency and Q, degrading
the carefully calculated characteristic. Ideal for synthesizing precise values by series/parallel
combination.

---

## Step 3 — Measure the Actual Capacitor Values

Take a batch of capacitors of the desired nominal value and measure each one with an LCR meter.
Select one or two for the circuit. Record their **exact measured values** — these go into the
calculator in the next step, not the nominal values.

This is the key step that separates a precision filter from a typical one. A 5% capacitor
labeled 100 nF might measure anywhere from 95 nF to 105 nF; using the actual value ensures
the resistors will be calculated to compensate.

---

## Step 4 — Back-Calculate the Resistor Values

Return to the Okawa calculator (or equivalent). On the component-value sheet:

- Enter the **measured** capacitor values (from Step 3)
- Enter the **target Q** and **target −3 dB frequency** (from Step 1)
- Let the calculator solve for the required resistor values

The resulting R values will be non-standard — this is expected and correct.

---

## Step 5 — Synthesize the Resistor Values

Match the calculated R values using 0.1% tolerance resistors. Three approaches:

1. **Direct match** — if a standard 0.1% E96 value is within tolerance of the target, use it.
2. **Series combination** — R_total = R1 + R2 (easy, additive, no interaction)
3. **Parallel combination** — R_total = (R1 × R2) / (R1 + R2) (useful for fine trimming)

Target accuracy: get within ~0.1% of the calculated value. This keeps the filter characteristic
error well below what capacitor measurement uncertainty introduces.

---

## Step 5.5 — Verify "Close Enough" Before Synthesizing

Before going to the trouble of combining resistors, check whether the nearest standard 0.1%
values are already acceptable. Plug all values — measured capacitances and the candidate
standard resistors — back into the Okawa calculator and inspect the resulting Q, −3 dB
frequency, and step response.

If the deviation from the target is acceptable for your application, use those resistors as-is
and skip to Step 6. If not, proceed with Step 5's series/parallel synthesis to hit the
calculated values more precisely.

---

## Step 6 — Assemble and Verify

Build the circuit with the selected capacitors and synthesized resistors. Verify:

- **Step response** — rise time, settling behavior, overshoot should match simulation
- **−3 dB frequency** — measure with a signal generator and scope or network analyzer
- **Passband ripple / Q** — confirm the response shape matches the target topology

With measured capacitors and calculated resistors, the circuit should be correct on the first
build.
