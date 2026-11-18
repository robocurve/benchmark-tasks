"""Grade the trace exported by a Tektronix scope or the native task."""

import argparse
import csv
import json
from dataclasses import asdict
from itertools import chain
from pathlib import Path

from waveform import evaluate_waveform


def read_trace(path):
    """Read Tektronix channel CSV or a two-column time_s,voltage_v CSV."""
    with Path(path).open(newline="", encoding="utf-8-sig") as source:
        rows = iter(csv.reader(source))
        first = next(rows, [])
        if first[:1] == ["Record Length"]:
            tektronix = True
            rows = chain((first,), rows)
        elif first == ["time_s", "voltage_v"]:
            tektronix = False
        else:
            raise ValueError("Expected a channel export or the header time_s,voltage_v.")

        time_s, voltage_v = [], []
        for row in rows:
            if not row:
                continue
            sample = row[3:] if tektronix else row
            if len(sample) != 2:
                raise ValueError(f"Expected a time and voltage pair in row {row!r}.")
            t, v = map(float, sample)
            time_s.append(t)
            voltage_v.append(v)
    return time_s, voltage_v


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("channel_csv", type=Path)
    args = parser.parse_args()
    result = evaluate_waveform(*read_trace(args.channel_csv))
    print(json.dumps(asdict(result), indent=2))


if __name__ == "__main__":
    main()
