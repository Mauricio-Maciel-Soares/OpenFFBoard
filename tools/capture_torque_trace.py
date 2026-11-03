#!/usr/bin/env python3
"""Capture passive CAN current observations and firmware timing to CSV."""
import argparse
import csv
import math
import time
from pathlib import Path
from capture_odrive_diagnostics import read_reply, read_snapshot, AXIS_FIELDS, ODRIVE_FIELDS
from torque_study import TraceCapture, parse_trace


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('port', help='USB CDC port, e.g. COM5; close other serial clients first')
    parser.add_argument('--seconds', type=float, default=60)
    parser.add_argument('--rate', type=float, default=25, help='host requests/s (1..50), not Iq/CAN rate')
    parser.add_argument('--kt', type=float, default=1.11, help='actual motor torque_constant, Nm/A')
    parser.add_argument('--axis', type=int, choices=(0,1,2), default=0)
    parser.add_argument('--odrive', type=int, choices=(0,1), default=0)
    parser.add_argument('--label', default='baseline')
    parser.add_argument('--output', type=Path, default=Path('torque_trace.csv'))
    args = parser.parse_args()
    if not (math.isfinite(args.seconds) and args.seconds > 0 and 1 <= args.rate <= 50
            and math.isfinite(args.kt) and args.kt > 0):
        parser.error('Use positive finite duration/Kt and a rate from 1 to 50')
    timing_path = args.output.with_name(args.output.stem + '_timing.csv')
    if args.output.exists() or timing_path.exists():
        parser.error('Output already exists; choose a new --output filename')
    try:
        import serial
    except ImportError:
        parser.exit(1, 'Install dependency: python -m pip install pyserial\n')
    capture = TraceCapture(args.kt)
    with serial.Serial(args.port, 115200, timeout=.05, write_timeout=1) as port, \
            args.output.open('x', newline='', encoding='utf-8-sig') as output, \
            timing_path.open('x', newline='', encoding='utf-8-sig') as timing_output:
        port.reset_input_buffer()
        capture.cursor = parse_trace(read_reply(port, 'odrv.{}.trace?-1'.format(args.odrive)))['newest']
        start = next_timing = time.monotonic()
        writer = timing_writer = None
        try:
            while time.monotonic() - start < args.seconds:
                before = time.monotonic()
                payload = read_reply(port, 'odrv.{}.trace?{}'.format(args.odrive,capture.cursor))
                rows = capture.accept(payload)
                for row in rows:
                    row.update(host_elapsed_s=time.monotonic()-start, label=args.label,
                               odrive_instance=args.odrive, axis_instance=args.axis)
                    if writer is None:
                        writer = csv.DictWriter(output,fieldnames=list(row));writer.writeheader()
                    writer.writerow(row)
                output.flush()
                if time.monotonic() >= next_timing:
                    axis=read_snapshot(port,'axis.{}.diag2?'.format(args.axis),AXIS_FIELDS)
                    drive=read_snapshot(port,'odrv.{}.diag2?'.format(args.odrive),ODRIVE_FIELDS)
                    row={'host_elapsed_s':time.monotonic()-start,'label':args.label}
                    row.update({'axis_'+k:v for k,v in axis.items()})
                    row.update({'odrive_'+k:v for k,v in drive.items()})
                    if timing_writer is None:
                        timing_writer=csv.DictWriter(timing_output,fieldnames=list(row));timing_writer.writeheader()
                    timing_writer.writerow(row);timing_output.flush()
                    next_timing=time.monotonic()+1
                time.sleep(max(0,1/args.rate-(time.monotonic()-before)))
        except KeyboardInterrupt:
            pass
    print('Captured {} Iq observations; {} lost; {} resets.'.format(capture.total,capture.lost,capture.restarts))
    if not capture.total:
        print('No Get_Iq frames received. Check ODrive iq_rate_ms; this tool does not change it.')
    print('Trace:',args.output,'Timing:',timing_path)


if __name__=='__main__':
    main()
