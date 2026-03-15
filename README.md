# Who is responsible
```
Schematic by Randall McRee 
PCB layout by Krasimir Kostadinov 
Software mostly from Claude 
```

# OpenSourceDiffVM-RevB

A high-resolution differential voltmeter (8.5-9.5 digit) with nanovolt-level sensitivity, based on the Analog Devices article on [low-noise instrumentation amplifiers](https://www.analog.com/en/resources/analog-dialogue/articles/low-noise-inamp-nanovolt-sensitivity.html).

## How It Works

An unknown voltage Vx is compared against a precision 16-bit DAC using a chopped measurement topology. The DAC output tracks Vx to within a few millivolts, and a high-gain instrumentation amplifier amplifies the residual difference for digitization by a 24-bit ADC. Chopping at ~385 Hz cancels preamp offset and 1/f noise.

```
Unknown Vx ──┐
             ├──[TMUX SPDT]──► AD8428 x4 ──► ADS127L11 ──► Teensy 4.x
DAC Output ──┘   (chopped)    (×2000 gain)   (24-bit ADC)
```

**Measurement equation:** `Vx = DAC_voltage + (ADC_mean / preamp_gain) × divider_ratio`

The DAC is calibrated against three ADR1001 ultra-stable voltage references (0.05 ppm/C) with 1000+ hour burn-in. Real-time drift compensation tracks the reference filter lag.

## Key Components

| Component | Function |
|-----------|----------|
| 4x AD8428 | Ultra-low-noise instrumentation amplifier (1.5 nV/rtHz), gain = 2000 |
| TMUX7234 | Low-charge-injection SPDT switches for input chopping |
| ADS127L11 | 24-bit delta-sigma ADC, ~3906 SPS |
| AD5760 | 16-bit precision DAC, tracks unknown input |
| 3x ADR1001 | Ultra-stable voltage references, filtered and averaged |
| Teensy 4.1 | ARM Cortex-M7 @ 600 MHz, hardware 64-bit double |

## Voltage Ranges

| Range | Method | Resolution |
|-------|--------|------------|
| +/-5V | Direct input | ~0.15 nV/count |
| +/-50V | Caddock 1776-C4815 divider (1:10, hardwired) | ~1.5 nV |
| +/-500V | Parallel tap (1:108.76) | ~16 nV |
| +/-5kV | Parallel tap (1:984.65) | ~150 nV |

## Firmware

The firmware runs on Teensy 4.1 and provides:

- **Serial command interface** over USB (115200 baud) — type `help` for commands
- **Channel scanning** — configure a scan list of up to 7 input channels
- **Three output modes** — human-readable, CSV for logging, Arduino Serial Plotter
- **Configurable integration** — 10 to 10,000 chop cycles per reading
- **Auto-zero** — periodic GND measurement for offset tracking
- **Reference drift compensation** — real-time prediction of filter lag error
- **DAC auto-adjustment** — binary search to keep preamp in range
- **Power-on self-test** — 9 hardware verification tests
- **EEPROM persistence** — save/load configuration across power cycles

### Building

Requires [Teensy board support](https://www.pjrc.com/teensy/td_download.html) for Arduino.

```bash
# Arduino CLI
arduino-cli compile --fqbn teensy:avr:teensy41 OpenSourceDiffVM-RevB.ino
arduino-cli upload --fqbn teensy:avr:teensy41 -p COM_PORT OpenSourceDiffVM-RevB.ino
```

### Serial Commands

Connect at 115200 baud. Key commands:

```
scan add Vx          # Add channel to scan list
scan add VrefRaw     # Add another channel
scan start           # Begin scanning
scan stop            # Stop scanning
log start            # Switch to CSV output
plot start           # Switch to Serial Plotter output
integrate 200        # Set 200 chop cycles per reading
autozero on          # Enable periodic zero calibration
range 100            # Set HV divider to +/-500V
zero                 # Immediate zero calibration
status               # Show current configuration
config save          # Persist to EEPROM
help                 # List all commands
```

### CSV Output Format

```
# timestamp_ms,channel,voltage_V,uncertainty_V,stddev_counts,mean_counts,n,dac_code,drift_ppb
123456,Vx,1.234567890123e-03,1.230000000000e-09,8.234,12345.678,100,12345,5.060
```

## PC Companion Script

`tools/diffvm.py` provides host-side functionality:

```bash
pip install pyserial matplotlib pyyaml

# Interactive terminal
python tools/diffvm.py --port COM3

# Configure from file and start scanning
python tools/diffvm.py --port COM3 --config tools/diffvm_config.yaml

# Log CSV to disk
python tools/diffvm.py --port COM3 --config tools/diffvm_config.yaml --log ./logs

# Live matplotlib plot
python tools/diffvm.py --port COM3 --config tools/diffvm_config.yaml --plot

# Full operation: config + log + plot
python tools/diffvm.py --port COM3 --config tools/diffvm_config.yaml --log ./logs --plot
```

See `tools/diffvm_config.yaml` for configuration options.

## Hardware Design

KiCad 8 project files are in `DiffVM/`:

| File | Description |
|------|-------------|
| `DiffVM_4L.kicad_sch` | Root schematic (4-layer PCB) |
| `FrontEnd.kicad_sch` | TMUX switches, AD8428 preamp chain, AD5760 DAC |
| `DAC.kicad_sch` | ADS127L11 ADC and Teensy interface |
| `VRef.kicad_sch` | 3x ADR1001 voltage references |
| `DiffVM_4L.kicad_pcb` | PCB layout |

## Design Notes

| Document | Description |
|----------|-------------|
| [`filter-design.md`](filter-design.md) | Workflow for building precision low-pass filters with measured capacitors and calculated resistors |
| [`Calibration.md`](Calibration.md) | Calibration procedures and constants |
| [`ADC_analysis.md`](ADC_analysis.md) | ADS127L11 configuration, filter selection, and ENOB analysis |

## References

- [Low-Noise Instrumentation Amplifier with Nanovolt Sensitivity](https://www.analog.com/en/resources/analog-dialogue/articles/low-noise-inamp-nanovolt-sensitivity.html) (Analog Devices)
- [AD8428 datasheet](https://www.analog.com/media/en/technical-documentation/data-sheets/AD8428.PDF)
- [ADS127L11 datasheet](https://www.ti.com/lit/ds/symlink/ads127l11.pdf)
- [AD5760 datasheet](https://www.analog.com/media/en/technical-documentation/data-sheets/AD5760.pdf)

## License

Licensed under the [CERN Open Hardware Licence Version 2 - Permissive](LICENSE) (CERN-OHL-P v2).
