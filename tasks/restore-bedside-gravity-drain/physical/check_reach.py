"""Re-solve the committed reference, descending, aligned and shifted YAM paths on the installed URDF."""
from pathlib import Path
import json
import sys

import numpy as np

task_root = Path(sys.argv[1]).resolve() if len(sys.argv) > 1 else Path(__file__).resolve().parents[1]
sys.path.insert(0, str(task_root))
from evd.policy import make_plan
from evd.identity import source_id
from yam_arena import ik
from yam_arena.station import BASES

def run(mode, solve):
    residuals = []
    def record(*args, **kwargs):
        q, dp, dr = solve(*args, **kwargs)
        residuals.append((float(dp), float(dr)))
        return q, dp, dr
    ik.solve = record
    try:
        plans = make_plan(mode)
    finally:
        ik.solve = solve
    report = {}
    for side, points in plans.items():
        angles = points[:, 1:7]
        margins = np.minimum(angles - ik.LOWER, ik.UPPER - angles)
        if np.min(margins) < -1e-10:
            raise ValueError(f"{side} joint limit exceeded")
        report[side] = {
            "keyframes": len(points),
            "max_base_to_tcp_mm_at_keyframes": round(max(np.linalg.norm(ik.tcp_pose(q, BASES[side])[0] - BASES[side]) for q in angles)*1000, 2),
            "minimum_joint_limit_margin_deg": round(float(np.rad2deg(np.min(margins))), 4),
            "last_keyframe_s": round(float(points[-1, 0]), 3)
        }
    return {**report, "ik_solutions": len(residuals),
            "max_ik_position_residual_mm": round(max(r[0] for r in residuals)*1000, 4),
            "max_ik_rotation_residual_deg": round(float(np.rad2deg(max(r[1] for r in residuals))), 4)}

if __name__ == "__main__":
    print(json.dumps({"kind": "kinematic waypoint audit", "source_id": source_id(),
                      "cases": {m: run(m, ik.solve) for m in ("reference", "lowered", "aligned", "alternate")}}, indent=2))
