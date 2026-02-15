#!/usr/bin/env python3
# SPDX-License-Identifier: CERN-OHL-P-2.0
# SPDX-FileCopyrightText: 2025 rkm
"""
DiffVM Companion Script - PC-side interface for OpenSourceDiffVM-RevB

Provides:
  - Interactive serial terminal to send commands
  - YAML config file loading and instrument setup
  - CSV data logging to disk with timestamped filenames
  - Real-time matplotlib plotting

Dependencies:
  pip install pyserial matplotlib pyyaml

Usage:
  python diffvm.py --port COM3
  python diffvm.py --port COM3 --config diffvm_config.yaml
  python diffvm.py --port COM3 --config diffvm_config.yaml --log ./logs
  python diffvm.py --port COM3 --config diffvm_config.yaml --plot
  python diffvm.py --port COM3 --config diffvm_config.yaml --log ./logs --plot
"""

import argparse
import os
import sys
import time
import threading
import re
from datetime import datetime
from pathlib import Path
from collections import deque

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    print("ERROR: pyserial required. Install with: pip install pyserial")
    sys.exit(1)


# ---------------------------------------------------------------------------
# Serial Connection
# ---------------------------------------------------------------------------

class SerialConnection:
    """Manages serial port connection with auto-reconnect."""

    def __init__(self, port, baud=115200, timeout=1.0):
        self.port = port
        self.baud = baud
        self.timeout = timeout
        self.ser = None

    def connect(self):
        """Open the serial connection."""
        try:
            self.ser = serial.Serial(
                port=self.port,
                baudrate=self.baud,
                timeout=self.timeout,
            )
            # Flush any stale data
            self.ser.reset_input_buffer()
            return True
        except serial.SerialException as e:
            print(f"ERROR: Cannot open {self.port}: {e}")
            return False

    def disconnect(self):
        """Close the serial connection."""
        if self.ser and self.ser.is_open:
            self.ser.close()

    def is_connected(self):
        """Check if port is still open."""
        return self.ser is not None and self.ser.is_open

    def send_command(self, cmd):
        """Send a command string (appends newline)."""
        if not self.is_connected():
            return False
        try:
            self.ser.write((cmd.strip() + "\n").encode("ascii"))
            self.ser.flush()
            return True
        except serial.SerialException:
            return False

    def read_line(self):
        """Read one line from serial (blocking up to timeout). Returns str or None."""
        if not self.is_connected():
            return None
        try:
            raw = self.ser.readline()
            if raw:
                return raw.decode("ascii", errors="replace").rstrip("\r\n")
            return None
        except serial.SerialException:
            return None

    def reconnect(self, retries=5, delay=2.0):
        """Attempt to reconnect to the serial port."""
        self.disconnect()
        for attempt in range(1, retries + 1):
            print(f"Reconnect attempt {attempt}/{retries}...")
            if self.connect():
                print(f"Reconnected to {self.port}")
                return True
            time.sleep(delay)
        print("Failed to reconnect.")
        return False


# ---------------------------------------------------------------------------
# Config Manager
# ---------------------------------------------------------------------------

class ConfigManager:
    """Loads YAML config and translates to serial commands."""

    def __init__(self, path=None):
        self.config = {}
        if path:
            self.load(path)

    def load(self, path):
        """Load a YAML configuration file."""
        try:
            import yaml
        except ImportError:
            print("ERROR: pyyaml required for config files. Install with: pip install pyyaml")
            sys.exit(1)

        with open(path, "r") as f:
            self.config = yaml.safe_load(f) or {}
        return self.config

    def get_port(self):
        """Get serial port from config, or None."""
        return self.config.get("port")

    def get_baud(self):
        """Get baud rate from config."""
        return self.config.get("baud", 115200)

    def apply(self, conn):
        """Send configuration commands to the instrument."""
        commands = self._build_commands()
        for cmd in commands:
            print(f"  > {cmd}")
            conn.send_command(cmd)
            # Give the instrument a moment to process each command
            time.sleep(0.1)
            # Drain any response
            while True:
                line = conn.read_line()
                if line is None:
                    break
                print(f"  < {line}")

    def _build_commands(self):
        """Convert YAML config to a list of serial commands."""
        cmds = []

        # Scan channels
        scan = self.config.get("scan", {})
        channels = scan.get("channels", [])
        if channels:
            cmds.append("scan clear")
            for ch in channels:
                cmds.append(f"scan add {ch}")

        # Integration
        integration = scan.get("integration")
        if integration is not None:
            cmds.append(f"integrate {integration}")

        # Auto-zero
        az = self.config.get("autozero", {})
        if "enabled" in az:
            cmds.append(f"autozero {'on' if az['enabled'] else 'off'}")
        if "interval" in az:
            cmds.append(f"autozero interval {az['interval']}")

        # HV divider range
        rng = self.config.get("range")
        if rng is not None:
            cmds.append(f"range {rng}")

        return cmds


# ---------------------------------------------------------------------------
# Data Logger
# ---------------------------------------------------------------------------

class DataLogger:
    """Captures CSV lines to timestamped files on disk."""

    def __init__(self, directory, filename_template=None, max_size_mb=100):
        self.directory = Path(directory)
        self.filename_template = filename_template or "diffvm_{date}_{time}.csv"
        self.max_size_bytes = max_size_mb * 1024 * 1024
        self.file = None
        self.file_path = None
        self.bytes_written = 0

    def start(self, config_path=None, port=None, firmware_version=None):
        """Open a new log file with session header."""
        self.directory.mkdir(parents=True, exist_ok=True)

        now = datetime.now()
        fname = self.filename_template.replace("{date}", now.strftime("%Y%m%d"))
        fname = fname.replace("{time}", now.strftime("%H%M%S"))
        fname = fname.replace("{port}", (port or "unknown").replace("/", "_"))

        self.file_path = self.directory / fname
        self.file = open(self.file_path, "w", encoding="utf-8", newline="\n")
        self.bytes_written = 0

        # Write session header
        header = (
            f"# DiffVM Log - {now.strftime('%Y-%m-%d %H:%M:%S')}\n"
            f"# Config: {config_path or 'none'}\n"
            f"# Port: {port or 'unknown'}\n"
            f"# Firmware: {firmware_version or 'unknown'}\n"
        )
        self.file.write(header)
        self.bytes_written += len(header)
        self.file.flush()

        print(f"Logging to: {self.file_path}")
        return self.file_path

    def write_row(self, line):
        """Write a CSV line to the log file."""
        if self.file is None:
            return
        text = line + "\n"
        self.file.write(text)
        self.bytes_written += len(text)
        self.file.flush()

        # Check for rotation
        if self.bytes_written >= self.max_size_bytes:
            self.rotate()

    def rotate(self):
        """Close current file and open a new one."""
        old_path = self.file_path
        print(f"Log rotation: {old_path} ({self.bytes_written} bytes)")
        self.stop()
        self.start()

    def stop(self):
        """Close the log file."""
        if self.file:
            self.file.close()
            self.file = None

    def __del__(self):
        self.stop()


# ---------------------------------------------------------------------------
# Live Plotter
# ---------------------------------------------------------------------------

class LivePlotter:
    """Real-time matplotlib plotting of measurement data."""

    def __init__(self, channels, window_sec=300, y_auto=True):
        self.channel_names = channels
        self.window_sec = window_sec
        self.y_auto = y_auto

        # Per-channel ring buffers: {name: (times, voltages, stddevs)}
        maxpts = window_sec * 2  # Conservative: at most ~2 readings/sec per channel
        self.data = {}
        for ch in self.channel_names:
            self.data[ch] = {
                "times": deque(maxlen=maxpts),
                "voltages": deque(maxlen=maxpts),
                "stddevs": deque(maxlen=maxpts),
            }
        self.t0 = time.time()
        self.lock = threading.Lock()

    def update(self, channel, voltage, stddev=0.0):
        """Thread-safe: add a data point."""
        t = time.time() - self.t0
        with self.lock:
            if channel in self.data:
                self.data[channel]["times"].append(t)
                self.data[channel]["voltages"].append(voltage)
                self.data[channel]["stddevs"].append(stddev)

    def run(self, stop_event):
        """
        Blocking: run the matplotlib event loop.
        Call this from the main thread. stop_event is a threading.Event.
        """
        try:
            import matplotlib
            matplotlib.use("TkAgg")
            import matplotlib.pyplot as plt
            from matplotlib.animation import FuncAnimation
        except ImportError:
            print("ERROR: matplotlib required for plotting. Install with: pip install matplotlib")
            return

        n_channels = len(self.channel_names)
        fig, axes = plt.subplots(n_channels, 1, sharex=True, squeeze=False,
                                 figsize=(10, 3 * n_channels))
        fig.suptitle("DiffVM Live Data")

        lines = {}
        fill_between = {}
        readouts = {}
        for i, ch in enumerate(self.channel_names):
            ax = axes[i][0]
            ax.set_ylabel(f"{ch} (V)")
            ax.grid(True, alpha=0.3)
            line, = ax.plot([], [], linewidth=1, label=ch)
            lines[ch] = line
            fill_between[ch] = None
            readouts[ch] = ax.text(0.02, 0.95, "", transform=ax.transAxes,
                                   fontsize=9, verticalalignment="top",
                                   fontfamily="monospace",
                                   bbox=dict(boxstyle="round", facecolor="wheat", alpha=0.5))
        axes[-1][0].set_xlabel("Time (s)")

        def animate(_frame):
            with self.lock:
                for i, ch in enumerate(self.channel_names):
                    ax = axes[i][0]
                    d = self.data[ch]
                    if len(d["times"]) < 2:
                        continue
                    ts = list(d["times"])
                    vs = list(d["voltages"])
                    sds = list(d["stddevs"])

                    lines[ch].set_data(ts, vs)

                    # Update sigma band
                    if fill_between[ch] is not None:
                        fill_between[ch].remove()
                    upper = [v + s for v, s in zip(vs, sds)]
                    lower = [v - s for v, s in zip(vs, sds)]
                    fill_between[ch] = ax.fill_between(ts, lower, upper, alpha=0.2)

                    # Update readout
                    readouts[ch].set_text(f"{vs[-1]:.9e} V")

                    # Adjust axes
                    t_now = ts[-1]
                    ax.set_xlim(max(0, t_now - self.window_sec), t_now + 2)
                    if self.y_auto:
                        ax.relim()
                        ax.autoscale_view(scalex=False, scaley=True)

        anim = FuncAnimation(fig, animate, interval=500, cache_frame_data=False)
        plt.tight_layout()

        try:
            plt.show()
        except KeyboardInterrupt:
            pass
        finally:
            stop_event.set()


# ---------------------------------------------------------------------------
# CSV Line Parser
# ---------------------------------------------------------------------------

def parse_csv_line(line):
    """
    Parse a firmware CSV data line.
    Returns dict with keys: timestamp_ms, channel, voltage_V, uncertainty_V,
    stddev_counts, mean_counts, n, dac_code, drift_ppb
    or None if not a data line.
    """
    if not line or line.startswith("#"):
        return None
    parts = line.split(",")
    if len(parts) < 9:
        return None
    try:
        return {
            "timestamp_ms": int(parts[0]),
            "channel": parts[1],
            "voltage_V": float(parts[2]),
            "uncertainty_V": float(parts[3]),
            "stddev_counts": float(parts[4]),
            "mean_counts": float(parts[5]),
            "n": int(parts[6]),
            "dac_code": int(parts[7]),
            "drift_ppb": float(parts[8]),
        }
    except (ValueError, IndexError):
        return None


# ---------------------------------------------------------------------------
# POST Detection
# ---------------------------------------------------------------------------

def wait_for_post(conn, timeout=60):
    """
    Wait for POST to complete. Returns True if POST passed,
    False on timeout or failure.
    """
    print("Waiting for POST to complete...")
    start = time.time()
    while time.time() - start < timeout:
        line = conn.read_line()
        if line is None:
            continue
        print(f"  {line}")
        if "POST: ALL TESTS PASSED" in line:
            print("POST passed.")
            return True
        if "POST: *** FAILURES DETECTED ***" in line:
            print("WARNING: POST reported failures!")
            return True  # Continue anyway, instrument may be partially functional
        if "Type 'help'" in line:
            # Instrument already booted, POST already done
            return True
    print("WARNING: POST timeout - instrument may not be ready.")
    return False


# ---------------------------------------------------------------------------
# Main Application
# ---------------------------------------------------------------------------

def run_interactive(conn):
    """Interactive mode: type commands, see responses."""
    print("\nInteractive mode. Type commands, Ctrl+C to exit.\n")

    # Background reader thread
    stop = threading.Event()

    def reader():
        while not stop.is_set():
            line = conn.read_line()
            if line is not None:
                print(line)

    t = threading.Thread(target=reader, daemon=True)
    t.start()

    try:
        while True:
            cmd = input()
            if cmd.strip():
                conn.send_command(cmd)
    except (KeyboardInterrupt, EOFError):
        pass
    finally:
        stop.set()


def run_logging(conn, logger, plotter, stop_event):
    """
    Background thread: read serial, dispatch to logger and plotter.
    """
    csv_started = False
    while not stop_event.is_set():
        line = conn.read_line()
        if line is None:
            if not conn.is_connected():
                if not conn.reconnect():
                    stop_event.set()
                    break
            continue

        # Always print non-CSV lines
        if line.startswith("#"):
            if logger:
                logger.write_row(line)
            if not csv_started:
                print(line)
            continue

        # Try to parse as CSV data
        parsed = parse_csv_line(line)
        if parsed:
            csv_started = True
            if logger:
                logger.write_row(line)
            if plotter:
                plotter.update(
                    parsed["channel"],
                    parsed["voltage_V"],
                    parsed["uncertainty_V"],
                )
        else:
            # Non-CSV line (status messages, etc.)
            print(line)


def main():
    parser = argparse.ArgumentParser(
        description="DiffVM Companion - PC interface for OpenSourceDiffVM-RevB"
    )
    parser.add_argument("--port", type=str, help="Serial port (e.g., COM3, /dev/ttyACM0)")
    parser.add_argument("--baud", type=int, default=115200, help="Baud rate (default: 115200)")
    parser.add_argument("--config", type=str, help="YAML config file path")
    parser.add_argument("--log", type=str, help="Directory for CSV log files")
    parser.add_argument("--plot", action="store_true", help="Enable live plotting")
    parser.add_argument("--window", type=int, default=300, help="Plot time window in seconds")
    parser.add_argument("--no-wait-post", action="store_true",
                        help="Skip waiting for POST completion")
    args = parser.parse_args()

    # Load config file if provided
    config_mgr = None
    if args.config:
        config_mgr = ConfigManager(args.config)

    # Determine serial port
    port = args.port
    if not port and config_mgr:
        port = config_mgr.get_port()
    if not port:
        # Try auto-detection
        ports = list(serial.tools.list_ports.comports())
        teensy_ports = [p for p in ports if "teensy" in (p.description or "").lower()
                        or "usb serial" in (p.description or "").lower()]
        if len(teensy_ports) == 1:
            port = teensy_ports[0].device
            print(f"Auto-detected port: {port}")
        elif ports:
            print("Available serial ports:")
            for p in ports:
                print(f"  {p.device}: {p.description}")
            print("Specify port with --port")
            sys.exit(1)
        else:
            print("ERROR: No serial ports found.")
            sys.exit(1)

    baud = args.baud
    if config_mgr:
        baud = config_mgr.get_baud()

    # Connect
    conn = SerialConnection(port, baud)
    if not conn.connect():
        sys.exit(1)
    print(f"Connected to {port} at {baud} baud.")

    # Wait for POST
    if not args.no_wait_post:
        wait_for_post(conn)

    # Apply config
    if config_mgr:
        print("Applying configuration...")
        config_mgr.apply(conn)

    # If just interactive (no --log, no --plot, no config autostart)
    if not args.log and not args.plot:
        # If config was provided, start scanning and logging
        if config_mgr:
            scan_cfg = config_mgr.config.get("scan", {})
            if scan_cfg.get("autostart", False):
                conn.send_command("log start")
                time.sleep(0.1)
                conn.send_command("scan start")
                time.sleep(0.1)

        run_interactive(conn)
        conn.disconnect()
        return

    # Set up logger
    logger = None
    if args.log:
        log_cfg = {}
        if config_mgr:
            log_cfg = config_mgr.config.get("logging", {})
        logger = DataLogger(
            directory=args.log,
            filename_template=log_cfg.get("filename", "diffvm_{date}_{time}.csv"),
            max_size_mb=log_cfg.get("max_size_mb", 100),
        )
        logger.start(config_path=args.config, port=port)

    # Set up plotter
    plotter = None
    plot_channels = None
    if args.plot:
        if config_mgr:
            plot_cfg = config_mgr.config.get("plot", {})
            plot_channels = plot_cfg.get("channels")
            if not plot_channels:
                scan_cfg = config_mgr.config.get("scan", {})
                plot_channels = scan_cfg.get("channels", ["Vx"])
        else:
            plot_channels = ["Vx"]
        plotter = LivePlotter(
            channels=plot_channels,
            window_sec=args.window,
        )

    # Start CSV logging on the instrument
    conn.send_command("log start")
    time.sleep(0.1)

    # Start scanning if config says autostart
    if config_mgr:
        scan_cfg = config_mgr.config.get("scan", {})
        if scan_cfg.get("autostart", True):
            conn.send_command("scan start")
            time.sleep(0.1)

    # Background serial reader
    stop_event = threading.Event()
    reader_thread = threading.Thread(
        target=run_logging,
        args=(conn, logger, plotter, stop_event),
        daemon=True,
    )
    reader_thread.start()

    # If plotting, run matplotlib in main thread (required by matplotlib)
    if plotter:
        try:
            plotter.run(stop_event)
        except KeyboardInterrupt:
            pass
    else:
        # Just logging, block until Ctrl+C
        print("Logging active. Press Ctrl+C to stop.")
        try:
            while not stop_event.is_set():
                time.sleep(0.5)
        except KeyboardInterrupt:
            pass

    # Cleanup
    print("\nShutting down...")
    stop_event.set()
    conn.send_command("scan stop")
    time.sleep(0.2)
    conn.send_command("log stop")
    time.sleep(0.2)
    if logger:
        logger.stop()
    conn.disconnect()
    print("Done.")


if __name__ == "__main__":
    main()
