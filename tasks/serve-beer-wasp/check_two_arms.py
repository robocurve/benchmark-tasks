"""Could one arm do both jobs? Harness IK, no simulator. Run with the harness's Python, from this folder: HARNESS_DIR=... python check_two_arms.py

For each arm and target: the smallest position error, in mm, over grip directions, straight down, tilted 45 and level. "Across wing" keeps only grips whose fingers close across the handle's wing.
Under about 2 mm means the arm can get there.
"""

from __future__ import annotations

import numpy as np

from check_side_grip import TABLE_TOP, grip, solves


def best(arm: str, pos, tilt: float, phis) -> tuple[float, float, int, bool]:
    out = (1e9, 1e9, 0, False)
    for flip in (False, True):
        for phi in phis:
            mm, deg = solves(arm, pos, grip(tilt, phi, flip))
            if mm + deg < out[0] + out[1]:
                out = (mm, deg, phi, flip)
    return out


def main() -> None:
    import beer_usd as u

    every = list(range(-180, 180, 30))
    h = (u.VALVE[0], u.VALVE[1], TABLE_TOP + u.HANDLE_Z + 0.005)
    print(f"RIGHT arm to the handle at ({h[0]}, {h[1]}), TCP {u.HANDLE_Z + 0.005:.3f}")
    for tilt in (0, 45, 90):
        for name, phis in (("any direction", every), ("across the wing at 0 deg", [-90, 90]), ("across the wing at 90 deg", [0, 180])):
            mm, deg, phi, flip = best("right", h, tilt, phis)
            print(f"  tilt {tilt:2d}, {name}: {mm:6.1f} mm, {deg:5.1f} deg (phi {phi}{' flipped' if flip else ''})", flush=True)
    grip_h = 0.05
    places = {"clean glass 1": (*u.MUGS[0], 0.003 + grip_h), "clean glass 2": (*u.MUGS[1], 0.003 + grip_h), "pad": (*u.MAT, u.BAR[4] + 0.003 + grip_h), "spout": (*u.SPOUT, u.SPOUT_TIP - 0.010 - u.MUG_H + grip_h)}
    for name, (x, y, z) in places.items():
        print(f"LEFT arm to {name} at ({x}, {y}), TCP {z:.3f}")
        for tilt in (0, 45, 90):
            mm, deg, phi, flip = best("left", (x, y, TABLE_TOP + z), tilt, every)
            print(f"  tilt {tilt:2d}: {mm:6.1f} mm, {deg:5.1f} deg (phi {phi}{' flipped' if flip else ''})", flush=True)
    _ = np


if __name__ == "__main__":
    main()
