#!/usr/bin/env python3
"""Arm, stop and download the OpenFFBoard 1 kHz SlewTrace buffer as CSV."""
import argparse
import csv
import re
import time
from pathlib import Path

FIELDS = [
    "sequence", "time_us", "dt_raw_us", "dt_used_us", "nominal_period_us",
    "previous_output", "requested", "clipped_target", "slew_output",
    "final_output", "active_rate_counts_ms", "remainder_before",
    "remainder_after", "flags",
]
UINT32_MAX = (1 << 32) - 1


def read_reply(port, command, timeout=2.0):
    port.write((command + "\n").encode("ascii"))
    deadline = time.monotonic() + timeout
    buffer = bytearray()
    pattern = re.compile(rb"\[" + re.escape(command.encode("ascii")) + rb"\|([^\]]*)\]")
    while time.monotonic() < deadline:
        waiting = port.in_waiting
        buffer.extend(port.read(max(1, waiting)))
        match = pattern.search(buffer)
        if match:
            return match.group(1).decode("ascii").strip()
        if len(buffer) > 32768:
            raise RuntimeError("Unexpected serial output; close Configurator and other serial clients")
    raise TimeoutError("No reply to {}; check port and firmware".format(command))


def parse_rows(payload):
    lines = payload.splitlines()
    header = [int(x) for x in lines[0].split(",")]
    if len(header) != 5 or header[0] != 1:
        raise ValueError("Unexpected SlewTrace batch header: {}".format(lines[0]))
    oldest, newest, gap, count = header[1:]
    rows = []
    for line in lines[1:]:
        if not line:
            continue
        values = [int(x) for x in line.split(",")]
        if len(values) != len(FIELDS):
            raise ValueError("Bad sample row with {} fields".format(len(values)))
        rows.append(dict(zip(FIELDS, values)))
    if len(rows) != count:
        raise ValueError("Batch announced {} records but returned {}".format(count, len(rows)))
    return oldest, newest, gap, rows


def analyze(rows):
    rates = []
    violations = []
    for prev, cur in zip(rows, rows[1:]):
        dt = cur["dt_used_us"]
        if dt <= 0:
            continue
        delta = cur["final_output"] - prev["final_output"]
        observed = abs(delta) * 1000.0 / dt
        limit = cur["active_rate_counts_ms"]
        rates.append(observed)
        if limit and observed > limit + 1e-9:
            violations.append((cur["sequence"], observed, limit))
    return rates, violations


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("port", help="OpenFFBoard USB CDC, e.g. COM6")
    parser.add_argument("--axis", type=int, choices=(0, 1, 2), default=0)
    parser.add_argument("--output", type=Path, default=Path("slew_trace.csv"))
    args = parser.parse_args()
    if args.output.exists():
        parser.error("Output exists; choose a new --output path")
    try:
        import serial
    except ImportError:
        parser.exit(1, "Install pyserial: python -m pip install pyserial\n")

    prefix = "axis.{}.slewtrace".format(args.axis)
    start_cmd = "axis.{}.slewtracestart=1".format(args.axis)
    stop_cmd = "axis.{}.slewtracestop=1".format(args.axis)
    with serial.Serial(args.port, 115200, timeout=0.05, write_timeout=1) as port:
        port.reset_input_buffer()
        print(read_reply(port, prefix + "?"))
        print("Arming 256-sample ring (~256 ms at 1 kHz). Reproduce one transition, then press Enter to freeze.")
        read_reply(port, start_cmd)
        try:
            input()
        finally:
            read_reply(port, stop_cmd)

        status = read_reply(port, prefix + "?")
        status_values = [int(x) for x in status.split(",")]
        if len(status_values) != 7 or status_values[0] != 1:
            raise ValueError("Unexpected SlewTrace status: " + status)
        recording, stored, capacity, oldest, newest, overwritten = status_values[1:]
        cursor = -1
        all_rows = []
        total_gap = 0
        while len(all_rows) < stored:
            payload = read_reply(port, prefix + "?" + str(cursor))
            batch_oldest, batch_newest, gap, rows = parse_rows(payload)
            total_gap += gap
            if not rows:
                break
            all_rows.extend(rows)
            cursor = rows[-1]["sequence"]
            if cursor == batch_newest:
                break
        if not all_rows:
            raise RuntimeError("Capture is empty. Start the capture, reproduce the transition, then stop.")
        all_rows.sort(key=lambda row: row["sequence"])
        with args.output.open("x", newline="", encoding="utf-8-sig") as output:
            writer = csv.DictWriter(output, fieldnames=FIELDS)
            writer.writeheader()
            writer.writerows(all_rows)

    rates, violations = analyze(all_rows)
    print("Saved {} samples to {}".format(len(all_rows), args.output))
    print("Buffer: {} / {}; overwritten {}; read gaps {}".format(stored, capacity, overwritten, total_gap))
    if rates:
        print("Observed final-output rate: max {:.1f} counts/ms".format(max(rates)))
    print("Potential per-sample limit exceedances: {}".format(len(violations)))
    if violations:
        print("First exceedances (sequence, observed, configured):", violations[:10])
    print("Note: compare only contiguous samples with valid previous-output state; inspect flags and clipping before accepting a violation.")


if __name__ == "__main__":
    main()
