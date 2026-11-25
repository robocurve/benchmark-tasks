#!/usr/bin/env python3
"""Read the independently computed score from one Inspect Robots report."""
import argparse
import json
from pathlib import Path
from evidence_results import trial_outcome
p = argparse.ArgumentParser(description=__doc__)
p.add_argument("report", type=Path)
a = p.parse_args()
d = json.loads(a.report.read_text())
outcome = trial_outcome(a.report.parent, d, allow_recorded_refusal=True)
print(json.dumps({**outcome, "termination_reasons": d["samples"][0]["termination_reasons"]}))
