"""The scorer's rule on a gripper over a glass's mouth, tried offline at each grip, with the task's own points and test. Harness IK is not used: only rotations.

Run with the harness's Python, from this folder: HARNESS_DIR=... python check_cover.py
"""

from __future__ import annotations

import os
import sys

import numpy as np

sys.path.insert(0, os.environ["HARNESS_DIR"])

import beer_usd as u  # noqa: E402
from check_side_grip import PLAIN, TCP, grip  # noqa: E402
from task import gripper_points, mouth_covered  # noqa: E402

AXIS = np.array([u.SPOUT[0], u.SPOUT[1]])
RIM = u.SPOUT_TIP - 0.010
BASE = RIM - u.MUG_H


def case(label: str, rot: np.ndarray, tcp, opening: float, expect: bool) -> bool:
    origin = np.asarray(tcp, float) - rot @ TCP
    pts = gripper_points(opening) @ rot.T + origin
    got = mouth_covered(pts, AXIS, RIM)
    print(f"{'ok  ' if got == expect else 'FAIL'} {label}: covered {got}, expected {expect}")
    return got == expect


def main() -> None:
    ok = True
    outside = u.MUG_R - 0.002
    x, y = AXIS
    down = PLAIN(0.0)
    ok &= case("straight down, fingers outside the body, centred", down, (x, y, BASE + 0.062), outside, True)
    ok &= case("straight down, fingers outside, the grip 1.9 cm off the glass's axis", down, (x, y + 0.019, BASE + 0.062), outside, True)
    ok &= case("straight down, both fingers inside the glass", down, (x, y, BASE + 0.08), 0.02, True)
    ok &= case("straight down, fingers on the rim", down, (x, y, RIM + 0.02), outside, True)
    for tilt in (15, 30, 45):
        ok &= case(f"tilted {tilt} from straight down", grip(tilt, 60, False), (x, y, BASE + 0.05), outside, True)
    for tilt in (75, 90):
        ok &= case(f"tilted {tilt}: from the side", grip(tilt, 60, False), (x, y, BASE + 0.05), outside, False)
    ok &= case("the demo's grip, flipped", grip(90, 60, True), (x, y, BASE + 0.05), outside, False)
    ok &= case("straight down, the gripper well away from the glass", down, (x, y + 0.30, BASE + 0.062), 0.047, False)
    print("tilt 60, for the record:", mouth_covered(gripper_points(outside) @ grip(60, 60, False).T + (np.array([x, y, BASE + 0.05]) - grip(60, 60, False) @ TCP), AXIS, RIM))
    print("all as expected" if ok else "SOME FAILED")


if __name__ == "__main__":
    main()
