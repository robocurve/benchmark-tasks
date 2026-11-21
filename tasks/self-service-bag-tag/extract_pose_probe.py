"""Replay the proven printer extraction and record full tag-panel orientation.

Right arm only; left stays at READY. Robot joint targets only.
"""
from __future__ import annotations

import math
import sys
from pathlib import Path

import numpy as np


def extraction_waypoints(hold=1.0):
    from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN
    from bagtag_probe import W, frame
    R = frame((1, 0, 0), (0, -1, 0))
    # Identical TCP path to printer-full-extract-001's right arm.
    return [
        W(1.2, (.350, -.230, .885), R, FINGER_OPEN, "approach exposed end"),
        W(1.0, (.505, -.230, .885), R, FINGER_OPEN, "enclose exposed end"),
        W(.8, (.505, -.230, .885), R, FINGER_CLOSED, "pinch exposed end"),
        W(2.0, (.285, -.230, .885), R, FINGER_CLOSED, "pull from physical feed channel"),
        W(.6, (.285, -.230, .885), R, FINGER_CLOSED, "hold after pull"),
        W(1.5, (.190, -.230, .970), R, FINGER_CLOSED, "clear remaining stock from mouth"),
        W(hold, (.190, -.230, .970), R, FINGER_CLOSED, "hold extracted tag"),
    ]


def panel_axes(q):
    x, y, z, w = q
    R = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                  [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                  [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])
    return {"length": R[:, 0].round(3).tolist(), "width": R[:, 1].round(3).tolist(),
            "normal": R[:, 2].round(3).tolist()}


def main(out: Path):
    from yam_arena import station
    from bagtag_probe import Plan, Run, hold_ready

    right = Plan(extraction_waypoints(), station.BASES["right"])
    run = Run(out, "extract_pose_probe")
    try:
        run.trace.append(run.state(("reset", "reset")))
        steps = round(right.duration * 20)
        run.go(steps, hold_ready(), right.action7, every=4, snaps=(44, 60, 100, 142, steps))
        final = run.trace[-1]
        summary = {
            "stage": "extraction_replay_orientation_probe_not_scored",
            "steps": steps,
            "full_extraction_conservative": all(p[0] + .0215 < .500 for p in final["panel_xyz"]),
            "right_worst_ik_mm": right.worst_ik_mm,
            "final_right_fingers": final["robots"]["right"]["joints"][6:8],
            "final_panel_axes": [panel_axes(q) for q in final["panel_quat_xyzw"]],
            "pinch_panel_axes_step60": [panel_axes(q) for q in
                                        next(s for s in run.trace if s["step"] == 60)["panel_quat_xyzw"]],
        }
        run.save(summary)
    finally:
        run.close()


if __name__ == "__main__":
    sys.path.insert(0, str(Path(__file__).resolve().parent))
    main(Path(sys.argv[1]))
