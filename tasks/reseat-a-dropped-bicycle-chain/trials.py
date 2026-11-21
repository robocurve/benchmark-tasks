"""Run a named perturbation from a fresh reset and record the measured result."""

import argparse
import hashlib
import json
import platform
from pathlib import Path

import mujoco
import numpy as np
from fixtures import AXLES
from policy import reseat
from station import Station

from conditions import CASES, build


def execute(case, mode, tool="pick"):
    drop, reduction, drag = CASES[case]
    fixture = build(case, tool)
    trial = Station(fixture)
    trial.wait(0.45)
    lower_height = float((trial.data.xpos[fixture.ids, 2] - AXLES[0, 2]).min() * 1000)
    # Measure the chain near the front axle after gravity settles it.
    pos = trial.data.xpos[fixture.ids]
    top = pos[(abs(pos[:, 1] - AXLES[0, 1]) < 0.016) & (pos[:, 2] > AXLES[0, 2])]
    error = None
    start = trial.reading()
    top_height = float(np.max(top[:, 2]) - AXLES[0, 2]) if len(top) else None
    try:
        if mode == "idle":
            trial.wait(4)
        else:
            reseat(
                trial,
                guide=mode != "crank-only",
                test_before=mode == "repair",
                turn_budget=790,
            )
    except RuntimeError as exc:
        error = str(exc)
    fault = [r for r in trial.trace if r["phase"] == "Test the dropped chain"]
    return dict(
        case=case,
        mode=mode,
        tool=tool,
        drop_mm=drop,
        catcher_radius_reduction_mm=reduction,
        rear_drag_mNm=drag,
        starting_front_pockets=start["front_pockets"],
        starting_rear_pockets=start["rear_pockets"],
        lowest_roller_height_mm=round(lower_height, 3),
        upper_roller_height_mm=(
            round(top_height * 1000, 3) if top_height is not None else None
        ),
        before_repair_crank_turns=fault[-1]["front_from_reset"] if fault else None,
        before_repair_rear_turns=fault[-1]["rear_from_reset"] if fault else None,
        max_front_pockets_before_repair=max(
            (r["front_pockets"] for r in fault), default=None
        ),
        **trial.reading(),
        solver_warnings=int(sum(trial.data.warning.number)),
        error=error,
        command_sha256=hashlib.sha256(
            np.asarray(trial.commands, dtype="<f8").tobytes()
        ).hexdigest(),
        python=platform.python_version(),
        numpy=np.__version__,
        mujoco=mujoco.__version__
    )


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("case", choices=CASES)
    parser.add_argument(
        "--mode", choices=["repair", "idle", "crank-only"], default="repair"
    )
    parser.add_argument("--tool", choices=["pick", "flat"], default="pick")
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args()
    result = execute(args.case, args.mode, args.tool)
    args.out.parent.mkdir(exist_ok=True, parents=True)
    args.out.write_text(json.dumps(result, indent=2) + "\n")
    print(result["case"], result["score"], result["error"])
