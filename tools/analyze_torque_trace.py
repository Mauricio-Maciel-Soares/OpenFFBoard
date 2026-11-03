#!/usr/bin/env python3
"""Summarize a captured current trace by motor RPM, direction and quadrant."""
import argparse
import csv
import json
from pathlib import Path
from torque_study import analyze


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('input',type=Path)
    parser.add_argument('--rpm-bin',type=float,default=30)
    parser.add_argument('--output',type=Path,default=Path('torque_study.json'))
    args=parser.parse_args()
    with args.input.open(encoding='utf-8-sig',newline='') as source:
        result=analyze(csv.DictReader(source),args.rpm_bin)
    with args.output.open('x',encoding='utf-8') as output:
        json.dump(result,output,ensure_ascii=False,indent=2,allow_nan=False)
    print('{} valid groups, {} rejected rows, {} lost samples. {}'.format(
        len(result['groups']),result['rejected_rows'],result['lost_samples'],args.output))


if __name__=='__main__':
    main()
