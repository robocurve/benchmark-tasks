# SPDX-License-Identifier: MIT
"""Recompute grades from raw JSONL measurements, ignoring recorded score flags.

Usage: python -m missing_washer_rework.regrade TRACE.jsonl [--output grade.json]
This is an independent measurement replay, using the same frozen numeric
contract as live scoring. It does not trust policy phase names or stored grades.
"""
from __future__ import annotations
import argparse
import json
from pathlib import Path
from .scoring import ReworkScorer, config_dict
from .contract import TRACE_SCHEMA


def regrade_records(records):
    scorer = ReworkScorer()
    count = 0
    maximum = 0.
    first_success = None
    for record in records:
        metrics = scorer.update(record)
        count += 1
        maximum = max(maximum, metrics['score'])
        if metrics['success'] and first_success is None:
            first_success = record['t']
    if not count:
        raise ValueError('empty measurement trace')
    return dict(schema=TRACE_SCHEMA, samples=count, terminal=metrics,
                maximum_score=maximum, first_success_time=first_success,
                thresholds=config_dict(),
                method='raw-measurement replay; saved scores and flags ignored')


def regrade_file(path):
    def records():
        with Path(path).open(encoding='utf-8-sig') as stream:
            for line_number, line in enumerate(stream, 1):
                if not line.strip():
                    continue
                try:
                    record = json.loads(line)
                except json.JSONDecodeError as exc:
                    raise ValueError(f'invalid JSON on line {line_number}: {exc.msg}') from exc
                if not isinstance(record, dict):
                    raise ValueError(f'line {line_number} must contain a measurement object')
                yield record
    return regrade_records(records())


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('trace', type=Path)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args(argv)
    try:
        report = regrade_file(args.trace)
    except (OSError, ValueError) as exc:
        parser.exit(2, f'regrade: {exc}\n')
    rendered = json.dumps(report, indent=2, allow_nan=False)
    if args.output:
        args.output.write_text(rendered+'\n', encoding='utf-8')
    print(rendered)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
