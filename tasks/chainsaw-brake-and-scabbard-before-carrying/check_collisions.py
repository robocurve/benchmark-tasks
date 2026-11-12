"""Offline clearance audit of the demo plan against the scene, in WORLD coordinates (no simulator).

Plays every mode's script against the IK, puts both arms' collision hulls (from the URDF) in the world at every control
step and measures their signed distance to the scene's solids: the table top (z = 0.7475 m), the saw (housing, battery,
sprocket cover, bar and chain, handles, hand guard), the scabbard's holder and the scabbard, and the other arm. The
pads touching the hand guard's grip bar or the cover's walls are the task (the grip is the contact), so for the arm
doing it that object is left out while the jaws are closing or closed on it; every other pair must keep at least 4 mm.
The saw's pieces are boxes that bound the real shapes (a cylinder is replaced by its bounding box), so the numbers
are conservative.

    PYTHONPATH=<yam-isaac-lab-arena checkout> python check_collisions.py [--mode MODE]

Prints the minimum scene clearance of each arm per mode, and the closest pair. The touch_chain ablation (a deliberate
touch) and settle (no motion) are not audited.
"""

from __future__ import annotations

import argparse
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import demo  # noqa: E402
import geometry as g  # noqa: E402
from demokit import Arm  # noqa: E402

MIN_CLEAR_MM = 4.0
STRIDE = 3
FINGER_OPEN = -0.04695


def world_box(x, y, z):
    """A box from its (lo, hi) ranges in the saw's frame, as (centre, rotation, size) in the world."""
    ys = sorted((g.SAW_Y + g.S * y[0], g.SAW_Y + g.S * y[1]))
    lo = np.array([g.SAW_X + x[0], ys[0], g.TOP + z[0]])
    hi = np.array([g.SAW_X + x[1], ys[1], g.TOP + z[1]])
    return (lo + hi) / 2, np.eye(3), hi - lo


def guard_box(phi):
    """The hand guard's plate swung to angle ``phi`` about its vertical hinge, as (centre, rotation, size)."""
    c, s_ = np.cos(g.S * phi), np.sin(g.S * phi)
    rot = np.array([[c, -s_, 0.0], [s_, c, 0.0], [0.0, 0.0, 1.0]])
    pivot = np.array([g.SAW_X + g.GUARD_PIVOT[0], g.SAW_Y + g.S * g.GUARD_PIVOT[1], g.TOP + g.GUARD_H / 2])
    return pivot + rot @ np.array([g.GUARD_LEN / 2, 0.0, 0.0]), rot, np.array([g.GUARD_LEN, g.GUARD_BAR_T, g.GUARD_H])


def obstacles():
    """name -> (centre, rotation, size) in the world, the scene at its start (the saw's bounding boxes)."""
    h = g.CHAIN_H / 2
    out = {
        "table": (np.array([0.3, 0.0, g.TOP - 0.03]), np.eye(3), np.array([1.2, 1.4, 0.06])),
        "housing": world_box(g.HOUSING_X, g.HOUSING_Y, g.HOUSING_Z),
        "battery": world_box(g.BATTERY_X, g.BATTERY_Y, g.BATTERY_Z),
        "sprocket cover": world_box(g.COVER_X, g.COVER_Y, g.COVER_Z),
        "bar and chain": world_box((-g.BAR_OUT_HALF_W, g.BAR_OUT_HALF_W), (-g.BAR_LEN - g.BAR_OUT_HALF_W, 0.05),
                                   (g.BAR_Z - h, g.BAR_Z + h)),
    }
    for name, hd in (("front handle", g.FRONT_HANDLE), ("rear handle", g.REAR_HANDLE)):
        out[name] = world_box((hd["x"] - hd["r"], hd["x"] + hd["r"]), (hd["y0"], hd["y1"]),
                              (hd["z"] - hd["r"], hd["z"] + hd["r"]))
    out["hand guard"] = guard_box(g.GUARD_REST)
    out["hand guard (brake on)"] = guard_box(g.BRAKE_STOP)
    out["holder"] = (np.array([g.SCAB_X, g.HOLDER_Y, g.TOP + g.HOLDER_TOP / 2]), np.eye(3),
                     np.array([g.HOLDER_W, g.HOLDER_L, g.HOLDER_TOP]))
    return out


def scabbard_box(y_centre):
    return (np.array([g.SCAB_X, y_centre, g.TOP + g.SCAB_CENTER_Z]), np.eye(3),
            np.array([g.SCAB_OUT_W, g.SCAB_LEN, g.SCAB_OUT_H]))


class Ctx:
    def __init__(self):
        self.left, self.right = Arm("left"), Arm("right")


HOLD_HALF = {g.GUARD_ARM: g.GUARD_BAR_T / 2, g.COVER_ARM: g.SCAB_OUT_W / 2}      # each pad stops on the part it holds


def hulls_of(arm: Arm, closed_once: bool):
    """The arm's hulls in the world. The jaws close only as far as the part they hold: a finger's travel is its pad's
    distance from the centre line, 4.7 cm open and 0 closed, so while closing on, holding or letting go of the guard's
    grip bar or the cover it is never less than half that part's thickness."""
    from yam_arena import analysis
    from yam_arena.constants import ARM_JOINTS, FINGER_JOINTS

    travel = -FINGER_OPEN * float(arm.grip)
    if closed_once and arm.grip < 0.995:
        travel = max(travel, HOLD_HALF[arm.side])
    q = {n: float(v) for n, v in zip(ARM_JOINTS, arm.q, strict=True)}
    q.update({n: -travel for n in FINGER_JOINTS})
    return analysis.world_hulls(analysis.fk_link_poses_world(q, arm.base))


def trace(mode: str):
    """Per step: each arm's hulls, grip (1 open .. 0 closed) and whether it has closed on its part yet."""
    ctx = Ctx()
    out = []
    closed = {"left": False, "right": False}
    for k, _ in enumerate(demo.script(ctx, mode)):
        for arm in (ctx.left, ctx.right):
            closed[arm.side] |= arm.grip < 0.995
        if k % STRIDE:
            continue
        out.append({arm.side: (hulls_of(arm, closed[arm.side]), float(arm.grip), closed[arm.side])
                    for arm in (ctx.left, ctx.right)})
    return out


def distance(hulls, box, skip_base=False):
    from yam_arena import analysis

    c, rot, size = box
    return min(analysis.box_signed_distance(h.vertices, c, rot, size).min() for name, h in hulls
               if not (skip_base and name == "base"))


def audit(mode: str) -> dict:
    from yam_arena import analysis

    obs = obstacles()
    scab_start = (*scabbard_box(g.SCAB_CENTER_Y_START),)
    scab_end = (*scabbard_box(g.SAW_Y - g.S * (demo.SCAB_END_GAP + g.SCAB_LEN / 2)),)
    samples = trace(mode)
    best = {"left": (np.inf, ""), "right": (np.inf, "")}
    arm_arm = (np.inf, "")
    for i, s in enumerate(samples):
        for side in ("left", "right"):
            hulls, grip, closed = s[side]
            busy = closed and grip < 0.995            # jaws closing on, holding or leaving the part
            checks = dict(obs)
            if side == g.GUARD_ARM and closed:
                checks.pop("hand guard")              # this arm's job is to push the hand guard
                checks.pop("hand guard (brake on)")
            if side == g.GUARD_ARM and not closed:
                checks.pop("hand guard (brake on)")
            if side == g.COVER_ARM and busy:
                checks.pop("holder")                  # the pads sit beside the cover, over the holder's edge
            if side == g.COVER_ARM:
                checks["scabbard (jaws open)"] = scab_end if closed else scab_start
                if busy:
                    checks.pop("scabbard (jaws open)")
            for name, box in checks.items():
                d = distance(hulls, box, skip_base=(name == "table")) * 1000
                if d < best[side][0]:
                    best[side] = (d, f"{name} at step {i * STRIDE}")
        if i % 4 == 0:
            d, pair = analysis.hull_set_distance(s["left"][0], s["right"][0])
            if d * 1000 < arm_arm[0]:
                arm_arm = (d * 1000, f"{pair} at step {i * STRIDE}")
    return {"left": best["left"], "right": best["right"], "arms": arm_arm}


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--mode", default=None, choices=demo.MODES)
    a = ap.parse_args()
    ok = True
    scene_min = np.inf
    for mode in ([a.mode] if a.mode else [m for m in demo.MODES if m not in ("settle", "touch_chain")]):
        r = audit(mode)
        scene = min(r["left"][0], r["right"][0])
        scene_min = min(scene_min, scene)
        arms = "more than 50" if r["arms"][0] == np.inf else f"{r['arms'][0]:.1f}"
        bad = scene < MIN_CLEAR_MM or r["arms"][0] < MIN_CLEAR_MM
        ok &= not bad
        print(f"{mode:14s} scene clearance: left arm {r['left'][0]:6.1f} mm ({r['left'][1]}), right arm "
              f"{r['right'][0]:6.1f} mm ({r['right'][1]}); arm to arm {arms} mm{'  FAIL' if bad else ''}", flush=True)
    print(f"Minimum scene clearance {scene_min:.1f} mm (required {MIN_CLEAR_MM} mm). Collision check:",
          "PASS" if ok else "FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
