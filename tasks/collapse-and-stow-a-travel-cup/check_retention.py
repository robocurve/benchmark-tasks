"""Check whether the seated sleeve lifts with the cup under collar-only support."""

import argparse
import json
from pathlib import Path

import numpy as np

from run import SIDE_SLICE, Trial
from scene import TABLE_TOP
from scoring import geometry


def check(reference, out):
    history = [json.loads(line) for line in (reference/"trace.jsonl").read_text().splitlines()]
    seat_end = max(index for index,row in enumerate(history)
                   if row["phase"] == "seat sleeve under collar") + 1
    trial = Trial()
    actions = np.load(reference/"commands.npz", allow_pickle=False)["actions"]
    for command in actions[:seat_end]:
        trial.commands = {side:command[cols].copy() for side,cols in SIDE_SLICE.items()}
        trial._tick("command replay through seating")
    start_z = trial.trace[-1]["positions"]["cup_base"][2]
    trial.relative("collar-only lift", "right", np.array((0.,0.,.035)), 1.1)
    trial.hold("collar-only high hold", .65)
    window = trial.trace[-13:]
    min_cup_rise = min(row["positions"]["cup_base"][2]-start_z for row in window)
    min_sleeve_z = min(row["positions"]["heat_sleeve"][2]-TABLE_TOP for row in window)
    max_offset = max(geometry(row)["sleeve_radial_mm"] for row in window)
    seated = all(geometry(row)["stowed"] for row in window)
    sleeve_touched = any(any("heat_sleeve" in name for name in row["contact_n"]["right"])
                         for row in window)
    result = {
        "minimum_cup_rise_mm": round(1000*min_cup_rise, 3),
        "minimum_sleeve_clearance_mm": round(1000*min_sleeve_z, 3),
        "max_sleeve_misalignment_mm": round(max_offset, 3),
        "model_stowed_through_hold": seated,
        "right_arm_touched_sleeve_during_hold": sleeve_touched,
        "passed": min_cup_rise >= .020 and min_sleeve_z >= .020 and seated and not sleeve_touched,
    }
    out.mkdir(parents=True, exist_ok=False)
    (out/"summary.json").write_text(json.dumps(result,indent=2)+"\n")
    (out/"lift-trace.json").write_text(json.dumps(trial.trace[seat_end:],indent=2)+"\n")
    print(json.dumps(result))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("reference", type=Path)
    parser.add_argument("out", type=Path)
    args = parser.parse_args()
    check(args.reference,args.out)
