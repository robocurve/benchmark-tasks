"""Offline clearance audit of the demo plan against the scene, in WORLD coordinates (no simulator).

Plays every mode's script against the IK, puts both arms' collision hulls (from the URDF) in the world at every control
step and measures their signed distance to the scene's solids: the table top (z = 0.7475 m), the footing form's floor, the
five bars (bounding boxes of the cylinders), the three staging pads and every cap (a head box and a stem box), plus the
other arm. A cap is where the plan has put it: on its pad until the jaws close on it, in the hand (left out: pinching it is
the task; one pad is fixed to the hand and still touches the cap after the jaws open) until the hand has lifted 5 mm,
seated on its bar after that. The jaws close only as far as the cap's 60 mm head. Every
other pair must keep at least 4 mm.

    PYTHONPATH=<yam-isaac-lab-arena checkout> python check_collisions.py [--mode MODE]

Prints the minimum scene clearance of each arm per mode and the closest pair. `settle` (no motion) is not audited.
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


def box(centre, size):
    return (np.array(centre, float), np.eye(3), np.array(size, float))


def static_obstacles():
    out = {"table": box((0.3, 0.0, g.TOP - 0.03), (1.2, 1.4, 0.06)),
           "form floor": box((g.FORM_X, 0.0, g.TOP + g.FORM_SIZE[2] / 2), g.FORM_SIZE)}
    for i, y in enumerate(g.STUB_Y):
        out[f"bar {i}"] = box((g.FORM_X, y, g.FORM_TOP + g.BAR_H / 2), (2 * g.BAR_R, 2 * g.BAR_R, g.BAR_H))
    for j, y in enumerate(g.CAP_START_Y):
        out[f"pad {j}"] = box((g.PAD_X, y, g.TOP + g.PAD_SIZE[2] / 2), g.PAD_SIZE)
    return out


def cap_boxes(x, y, z_mouth):
    return {"head": box((x, y, z_mouth + g.CAP_STEM_H + g.CAP_HEAD_H / 2), (g.CAP_HEAD, g.CAP_HEAD, g.CAP_HEAD_H)),
            "stem": box((x, y, z_mouth + g.CAP_STEM_H / 2), (g.CAP_STEM, g.CAP_STEM, g.CAP_STEM_H))}


class Ctx:
    def __init__(self):
        self.left, self.right = Arm("left"), Arm("right")


def hulls_of(arm: Arm, closed_once: bool):
    """The arm's hulls in the world; while closing on, holding or letting go of a cap the fingers stay at least half the
    head's width apart."""
    from yam_arena import analysis
    from yam_arena.constants import ARM_JOINTS, FINGER_JOINTS

    travel = -FINGER_OPEN * float(arm.grip)
    if closed_once and arm.grip < 0.995:
        travel = max(travel, g.CAP_HEAD / 2)
    q = {n: float(v) for n, v in zip(ARM_JOINTS, arm.q, strict=True)}
    q.update({n: -travel for n in FINGER_JOINTS})
    return analysis.world_hulls(analysis.fk_link_poses_world(q, arm.base))


def trace(mode: str):
    """Per sampled step: each arm's hulls and grip, and the job (loose cap index) each arm has closed on and let go of."""
    ctx = Ctx()
    out = []
    state = {"left": "open", "right": "open"}                  # open (never closed), holding, settling, released
    jobs = {"left": [], "right": []}
    cur = {"left": None, "right": None}
    z_rel = {"left": 0.0, "right": 0.0}
    n_started = 0
    for k, _ in enumerate(demo.script(ctx, mode)):
        for arm in (ctx.left, ctx.right):
            s = state[arm.side]
            if s in ("open", "released") and arm.grip < 0.995:
                state[arm.side] = "holding"
                cur[arm.side] = n_started
                n_started += 1
            elif s == "holding" and arm.grip >= 0.995:
                state[arm.side] = "settling"                    # open, but the fixed pad still touches the cap
                z_rel[arm.side] = float(arm.pos[2])
                jobs[arm.side].append(cur[arm.side])
            elif s == "settling" and float(arm.pos[2]) > z_rel[arm.side] + 0.005:
                state[arm.side] = "released"                    # lifted clear of the cap
        if k % STRIDE:
            continue
        out.append({"state": dict(state), "cur": dict(cur), "done": sorted(jobs["left"] + jobs["right"]),
                    "arms": {arm.side: (hulls_of(arm, state[arm.side] != "open"), float(arm.grip)) for arm in
                             (ctx.left, ctx.right)}})
    return out


def distance(hulls, bx, skip_base=False):
    from yam_arena import analysis

    c, rot, size = bx
    best = (np.inf, "")
    for name, h in hulls:
        if skip_base and name == "base":
            continue
        d = analysis.box_signed_distance(h.vertices, c, rot, size).min()
        if d < best[0]:
            best = (d, name)
    distance.last = best[1]
    return best[0]


def audit(mode: str) -> dict:
    from yam_arena import analysis

    static = static_obstacles()
    samples = trace(mode)
    best = {"left": (np.inf, ""), "right": (np.inf, "")}
    arm_arm = (np.inf, "")
    for i, s in enumerate(samples):
        for side in ("left", "right"):
            hulls, grip = s["arms"][side]
            checks = dict(static)
            for k, y_cap in enumerate(g.CAP_START_Y):
                held = s["state"][side] in ("holding", "settling") and s["cur"][side] == k
                if held:
                    continue                                   # the cap in this hand: pinching it is the task
                if k in s["done"]:
                    y_stub = g.STUB_Y[g.BARE[k]]
                    parts = cap_boxes(g.FORM_X, y_stub, g.SEATED_Z)
                elif any(s["state"][o] in ("holding", "settling") and s["cur"][o] == k for o in ("left", "right")):
                    continue                                   # in the other arm's hand: far from this one
                else:
                    parts = cap_boxes(g.PAD_X, y_cap, g.CAP_START_Z)
                for nm, b in parts.items():
                    checks[f"cap {k} {nm}"] = b
            for y_idx in g.PRE_CAPPED:
                for nm, b in cap_boxes(g.FORM_X, g.STUB_Y[y_idx], g.SEATED_Z).items():
                    checks[f"fitted cap {y_idx} {nm}"] = b
            for name, bx in checks.items():
                # a seated cap (done or fitted) stands on its bar: the bar is inside it, so skip that bar's box against its cap
                d = distance(hulls, bx, skip_base=(name == "table")) * 1000
                if d < best[side][0]:
                    best[side] = (d, f"{name} / {distance.last} at step {i * STRIDE} grip {grip:.3f} state {s['state'][side]}")
        if i % 4 == 0:
            d, pair = analysis.hull_set_distance(s["arms"]["left"][0], s["arms"]["right"][0])
            if d * 1000 < arm_arm[0]:
                arm_arm = (d * 1000, f"{pair} at step {i * STRIDE}")
    return {"left": best["left"], "right": best["right"], "arms": arm_arm}


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--mode", default=None, choices=demo.MODES)
    a = ap.parse_args()
    ok = True
    scene_min = np.inf
    for mode in ([a.mode] if a.mode else [m for m in demo.MODES if m != "settle"]):
        r = audit(mode)
        scene = min(r["left"][0], r["right"][0])
        scene_min = min(scene_min, scene)
        arms = "more than 50" if r["arms"][0] == np.inf else f"{r['arms'][0]:.1f}"
        bad = scene < MIN_CLEAR_MM or r["arms"][0] < MIN_CLEAR_MM
        ok &= not bad
        print(f"{mode:10s} scene clearance: left arm {r['left'][0]:6.1f} mm ({r['left'][1]}), right arm "
              f"{r['right'][0]:6.1f} mm ({r['right'][1]}); arm to arm {arms} mm{'  FAIL' if bad else ''}", flush=True)
    print(f"Minimum scene clearance {scene_min:.1f} mm (required {MIN_CLEAR_MM} mm). Collision check:",
          "PASS" if ok else "FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
