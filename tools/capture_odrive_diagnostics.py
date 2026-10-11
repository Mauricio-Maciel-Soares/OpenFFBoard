#!/usr/bin/env python3
"""Read passive OpenFFBoard diagnostics over USB CDC; never writes motor settings."""
import argparse
import csv
import math
import re
import time
from pathlib import Path

AXIS_FIELDS = "version sequence time_us dt_us min_dt_us max_dt_us clip_count slew_count effect_counts requested_counts clipped_counts output_counts position_speed_deg_s encoder_speed_deg_s speed_deg_s accel_deg_s2 encoder_valid encoder_used driver_ready".split()
ODRIVE_FIELDS = "version time_ms iq_sequence iq_age_ms iq_setpoint_a iq_measured_a encoder_sequence encoder_age_ms velocity_turns_s tx_sequence tx_age_ms queued_torque_nm tx_errors".split()
UINT32_MAX = (1 << 32) - 1


def parse_payload(payload, fields):
    values = payload.split(",")
    if len(values) != len(fields) or values[0] != "1":
        raise ValueError("Firmware diagnostic schema is missing or incompatible")
    numbers = [float(value) for value in values]
    if not all(math.isfinite(value) for value in numbers):
        raise ValueError("Non-finite diagnostic sample")
    return dict(zip(fields, numbers))


def read_snapshot(port, command, fields, timeout=1.0):
    port.write((command + "\n").encode("ascii"))
    deadline = time.monotonic() + timeout
    buffer = bytearray()
    pattern = re.compile(rb"\[" + re.escape(command.encode("ascii")) + rb"\|([^\]]*)\]")
    while time.monotonic() < deadline:
        buffer.extend(port.read(max(1, port.in_waiting)))
        match = pattern.search(buffer)
        if match:
            return parse_payload(match.group(1).decode("ascii").strip(), fields)
        if len(buffer) > 16384:
            raise ValueError("Unexpected data on the selected CDC port")
    raise TimeoutError(f"No response to {command}; close the configurator and check the firmware/port")


def torque_estimates(odrive, kt, max_age_ms):
    # This is an electromagnetic torque estimate, not a shaft torque sensor.
    if odrive["iq_sequence"] == 0 or odrive["iq_age_ms"] > max_age_ms:
        return {"iq_fresh": 0, "iq_setpoint_torque_nm": "", "estimated_torque_nm": "", "iq_tracking_error_nm": ""}
    return {"iq_fresh": 1,
            "iq_setpoint_torque_nm": kt * odrive["iq_setpoint_a"],
            "estimated_torque_nm": kt * odrive["iq_measured_a"],
            "iq_tracking_error_nm": kt * (odrive["iq_setpoint_a"] - odrive["iq_measured_a"])}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("port", help="OpenFFBoard USB CDC port, e.g. COM5")
    parser.add_argument("--seconds", type=float, default=30)
    parser.add_argument("--rate", type=float, default=20, help="host polls/s, 1..50 (not the FFB loop rate)")
    parser.add_argument("--kt", type=float, default=1.11, help="ODrive torque_constant in Nm/A")
    parser.add_argument("--axis", type=int, choices=(0, 1), default=0)
    parser.add_argument("--odrive", type=int, choices=(0, 1), default=0)
    parser.add_argument("--max-iq-age-ms", type=float, default=50)
    parser.add_argument("--output", type=Path, default=Path("odrive_diagnostics.csv"))
    args = parser.parse_args()
    if not (math.isfinite(args.seconds) and args.seconds > 0 and 1 <= args.rate <= 50
            and math.isfinite(args.kt) and args.kt > 0
            and math.isfinite(args.max_iq_age_ms) and args.max_iq_age_ms > 0):
        parser.error("Use positive finite duration/Kt/age and a rate from 1 to 50 Hz")
    try:
        import serial
    except ImportError:
        parser.exit(1, "Install dependency: python -m pip install pyserial\n")
    with serial.Serial(args.port, 115200, timeout=0.05, write_timeout=1) as port, args.output.open("x", newline="", encoding="utf-8") as out:
        port.reset_input_buffer()
        start = time.monotonic()
        writer = None
        count = fresh_count = 0
        previous_axis = None
        next_poll = start
        try:
            while time.monotonic() - start < args.seconds:
                t0 = time.monotonic()
                axis = read_snapshot(port, f"axis.{args.axis}.diag?", AXIS_FIELDS)
                odrive = read_snapshot(port, f"odrv.{args.odrive}.diag?", ODRIVE_FIELDS)
                t1 = time.monotonic()
                row = {"host_start_s": t0 - start, "host_end_s": t1 - start,
                       "host_read_ms": (t1 - t0) * 1000, "kt_nm_a": args.kt}
                row.update({"axis_" + k: v for k, v in axis.items()})
                row.update({"odrive_" + k: v for k, v in odrive.items()})
                estimates = torque_estimates(odrive, args.kt, args.max_iq_age_ms)
                row.update(estimates)
                row["loop_hz"] = row["clip_events_s"] = row["slew_events_s"] = ""
                if previous_axis:
                    elapsed = (int(axis["time_us"]) - int(previous_axis["time_us"])) & UINT32_MAX
                    if 0 < elapsed < 10_000_000:
                        for field, out_field in (("sequence", "loop_hz"), ("clip_count", "clip_events_s"), ("slew_count", "slew_events_s")):
                            delta = (int(axis[field]) - int(previous_axis[field])) & UINT32_MAX
                            row[out_field] = delta * 1_000_000 / elapsed
                previous_axis = axis
                if writer is None:
                    writer = csv.DictWriter(out, fieldnames=list(row))
                    writer.writeheader()
                writer.writerow(row)
                out.flush()
                count += 1
                fresh_count += estimates["iq_fresh"]
                next_poll += 1 / args.rate
                now = time.monotonic()
                if next_poll < now:
                    next_poll = now # never catch up with a burst of serial requests
                time.sleep(max(0, next_poll - now))
        except KeyboardInterrupt:
            pass
    print(f"Saved {count} rows to {args.output}; {fresh_count} with fresh Iq.")
    if not fresh_count:
        print("No fresh Iq: check ODrive axis0.config.can.iq_rate_ms and CAN connection.")
    print("Asynchronous snapshots: do not infer current-loop bandwidth or latency from this CSV.")


if __name__ == "__main__":
    main()
