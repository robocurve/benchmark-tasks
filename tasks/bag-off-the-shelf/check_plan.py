"""Check the scripted demo's kinematics with yam_arena's own IK, with numpy only (no simulator): at every keyframe of each
arm's plan, how far the commanded joints are from the keyframe's pose.

Usage: BAG_DEMO=safe|naive python check_plan.py
"""

from __future__ import annotations

import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).parent))

import task  # noqa: E402
from bagkit import Plan  # noqa: E402
from yam_arena import ik, station  # noqa: E402

DT = 0.05


def main() -> None:
    bad = 0
    for arm, keys in task.demo().items():
        plan = Plan(list(keys), station.BASES[arm])
        print(f"{arm}: {len(keys)} keyframes, {plan.duration:.1f} s")
        t, q = 0.0, None
        for k in keys:
            t += k.duration
            s = t - k.duration
            worst = 0.0
            while s < t - 1e-6:  # step along the segment as the policy does
                s += DT
                a, _ = plan.action7(min(s, t - 1e-3))
                q = a[:6]
            pos, rot = ik.tcp_pose(q, np.asarray(station.BASES[arm], dtype=float))
            e_pos = float(np.linalg.norm(pos - np.asarray(k.pos))) * 1000
            r = rot.T @ k.rot
            e_rot = float(np.degrees(np.arccos(np.clip((np.trace(r) - 1) / 2, -1.0, 1.0))))
            flag = "  <--" if (e_pos > 5 or e_rot > 3) and not k.exact else ""
            bad += bool(flag)
            print(f"  t={t:5.1f} {k.label:18s} pos {np.round(k.pos, 3).tolist()}: {e_pos:6.1f} mm, {e_rot:5.1f} deg, q {np.round(q, 2).tolist()}{flag}")
    print("all keyframes within 5 mm and 3 degrees" if not bad else f"{bad} keyframes off")


if __name__ == "__main__":
    main()
