"""Scripted demo for hot-and-cold-holding-wells, run in the yam_arena embodiment (fixed layout).

    python demo.py task.py OUT_DIR [--variant full] [--no-record]   # demo.mp4, setup.png, result.json
    python demo.py task.py --plan-only                              # IK check of every variant's plan
    DEMO_DEBUG=10 python demo.py ...                                # print the pans' state every 10 steps

full        Both arms pinch the chili pan's rim at its two ends from above (one finger inside on the end wall, one
            outside on the flange's edge), lift the pan, carry it back over the block and across to the steam well,
            lower it in and let go.
Failure and cheating variants:
wrong_well  the same carry into the ice well (0)
one_arm     the right arm alone lifts the pan by the rim at its right end
drop        both arms let go of the pan 6 cm above the steam well (0)
"""

from __future__ import annotations

import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import democore as dc  # noqa: E402

VARIANTS = ("full", "wrong_well", "one_arm", "drop")
_CACHE: dict = {}
LIFT = 0.040  # m: the pan raised this far off the counter first
RIM_DEPTH = 0.012  # m: the TCP this far below the rim when pinching it (pad tips 2.5 cm below)
OPEN = 0.5  # gripper opening around the rim (4.7 cm between the pads)
CLEAR = 0.035  # m: the carried pan's underside this far above the block's top


def _closing(axis: int, tilts=(0, 10, 20, 30, 40, 50)):
    return [R for tl in tilts for az in range(0, 360, 45) for roll in (0, 90)
            for R in [dc.R_approach(tl, az, roll)] if abs(R[axis, 0]) >= 0.95]


def rim_offset(t, side: str) -> np.ndarray:
    """The TCP when pinching the rim at an end, fingers from above: one pad on the end wall's inner face, the other on
    the flange's outer edge (the pinch closes along y across wall and flange, 2.1 cm), pad tips 2.5 cm below the rim
    (clear of the chili, 5 cm below the rim)."""
    sy = 1.0 if side == "left" else -1.0
    return np.array([0.0, sy * (t.PAN_L / 2 + (t.FLANGE_W - t.PAN_T) / 2), t.PAN_H - RIM_DEPTH])


def build_plan(t, variant: str = "full", params: dict | None = None):
    from yam_arena import ik, station
    from yam_arena.constants import READY

    bases = station.BASES
    starts = {s: ik.tcp_pose(np.array(READY), bases[s]) for s in ("left", "right")}
    arms = {s: dc.Arm(s, bases[s], READY, *starts[s]) for s in ("left", "right")}
    L, Rt = arms["left"], arms["right"]
    hi = t.BLOCK_TOP + CLEAR
    target = "cold" if variant == "wrong_well" else t.GOAL["chili"]
    w = np.array([t.BLOCK_XY[0], t.WELL_Y[target], t.WELL_FLOOR + 0.0005])
    p0 = np.array(t.PAN_POS["chili"])
    p1, p2, p3 = p0 + [0, 0, LIFT], np.array([p0[0], p0[1], hi]), np.array([w[0], w[1], hi])

    key = target
    if key not in _CACHE:
        ry = lambda d: dc.rot([0.0, 1.0, 0.0], math.radians(d))  # noqa: E731
        G = {}
        for side in ("left", "right"):
            g = rim_offset(t, side)
            pick = [p0 + g + [0, 0, 0.06], p0 + g, p1 + g, p2 + g]
            place = [p3 + g, w + g, w + g - [0, 0, 0.004], w + g + [0, 0, 0.07]]
            # on the way the wrist may turn about the closing axis (y); the smallest turn wins: held in a firm pinch the
            # pan turns with the wrists (a large turn tipped it past 30 degrees)
            cands = _closing(1)
            ranked = sorted((dc.best_rotation(side, pick, [R])[1], i) for i, R in enumerate(cands))[:8]
            best = None
            for e1, i in ranked:
                if e1 > 0.002 and best is not None:
                    continue
                for d in sorted(range(-60, 61, 5), key=abs):
                    Rs_, e2_ = dc.best_rotation(side, place, [ry(d) @ cands[i]])
                    if e2_ <= 0.002:
                        break
                cand = (e2_ > 0.002, abs(d), max(e1, e2_), cands[i], Rs_, e1, e2_)
                if best is None or cand[:3] < best[:3]:
                    best = cand
            _, _, _, Rp, Rs, e1, e2 = best
            G[side] = (Rp, Rs)
            print(f"{side} rim grip: worst IK error {1000 * e1:.1f} picking up, {1000 * e2:.1f} setting down "
                  f"(mm + 0.05 rad)", flush=True)
        _CACHE[key] = G
    G = _CACHE[key]

    def carry(users=("left", "right")):
        for side in users:
            A, g, (R, Rs) = arms[side], rim_offset(t, side), G[side]
            A.add(2.5, p0 + g + [0, 0, 0.06], R, OPEN, "chili: above the rim")
            A.add(1.5, p0 + g, R, OPEN, "chili: around the rim")
            A.add(1.0, p0 + g, R, 0.0, "chili: grip")
            A.add(1.5, p1 + g, R, 0.0, "chili: lift")
            A.add(1.5, p2 + g, R, 0.0, "chili: up over the block")
            A.add(3.5, p3 + g, Rs, 0.0, "chili: over the well")
            if variant == "drop":
                A.add(1.5, w + g + [0, 0, 0.06], Rs, 0.0, "chili: above the well")
                A.add(0.8, grip=OPEN, label="chili: let go")
                A.add(1.5, w + g + [0, 0, 0.12], Rs, OPEN, "chili: up from the rim")
                continue
            A.add(2.5, w + g + [0, 0, 0.003], Rs, 0.0, "chili: down into the well")
            A.add(1.0, w + g - [0, 0, 0.004], Rs, 0.0, "chili: onto the well floor")
            A.add(0.5, w + g - [0, 0, 0.004], Rs, 0.0, "chili: settle")
            A.add(0.8, grip=OPEN, label="chili: let go")
            A.add(1.5, w + g + [0, 0, 0.07], Rs, OPEN, "chili: up from the rim")

    if variant == "one_arm":
        carry(users=("right",))
        Rt.add(2.0, starts["right"][0], starts["right"][1], 1.0, "right: home")
        return L, Rt, {}
    carry()
    for side in ("left", "right"):
        arms[side].add(2.0, starts[side][0], starts[side][1], 1.0, f"{side}: home")
    return L, Rt, {}


def servos_for(name, arm):
    return [TcpServo(f"{name}_robot")]


class TcpServo:
    """Steers the arm's measured TCP (from its finger pads) onto the planned TCP: on the way to the rim, so the pinch
    lands on it (folded low by its base, an arm tracked 1-2 cm off its plan), and more gently while carrying. The
    correction is kept while gripping, setting down and rising off the rim, and fades out after."""

    APPROACH = ("above the rim", "around the rim")
    CARRY = ("lift", "up over the block", "over the well", "down into the well")
    HOLD = ("grip", "onto the well floor", "settle", "let go", "up from the rim")  # kept until the fingers are off it
    PAD_C = np.array([0.0, 0.0, -0.008])  # the finger pads' centroid in the TCP frame (kitchenkit.PAD_Z mean)

    def __init__(self, robot: str):
        self.robot, self.off, self.cmd, self.R = robot, np.zeros(3), None, None

    def update(self, kk, label: str) -> None:
        key = label.split(": ")[-1]
        if key in self.HOLD:
            return
        if key not in self.APPROACH + self.CARRY:
            self.off = 0.8 * self.off
            return
        pts = kk.STATE.fingers.get(self.robot)
        if self.cmd is None or pts is None or len(pts) == 0:
            return
        meas = np.asarray(pts, float).mean(axis=0) - self.R @ self.PAD_C
        gain = 0.1 if key in self.APPROACH else 0.08
        self.off = np.clip(self.off + gain * (self.cmd - meas), -0.05, 0.05)

    def correct(self, pos, R, label):
        self.cmd, self.R = np.asarray(pos, float), np.asarray(R, float)
        return self.cmd + self.off, R



def frozen(label: str) -> bool:
    return label.endswith(("grip", "let go", "onto the well floor", "settle"))


def debug(k, t, kk, labL, labR):
    r = t.report()
    pans = r.get("pans", {})
    s = " ".join(f"{n}: well {p.get('well')} in {p.get('in_opening')} tilt {p.get('tilt')} z {p.get('z')} rel {p.get('released')}"
                 for n, p in pans.items())
    p, R = kk.pose_of("chili", t.PAN_POS["chili"])
    pads = []
    for arm, pts in sorted(kk.STATE.fingers.items()):  # finger pad points in the pan's frame, cm
        q = (np.asarray(pts, float) - p) @ R
        pads.append(f"{arm[0]} y {100 * q[:, 1].mean():.1f} z {100 * q[:, 2].min():.1f}..{100 * q[:, 2].max():.1f} "
                    f"x {100 * q[:, 0].min():.1f}..{100 * q[:, 0].max():.1f}")
    print(f"  dbg {k}: correct {r.get('correct')} wrong {r.get('wrong_well')} max_tilt {r.get('max_tilt')} spill "
          f"{r.get('spilled')} drop {r.get('drop_flag')} | {s} | {' ; '.join(pads)} | {labL} | {labR}", flush=True)


if __name__ == "__main__":
    dc.main(build_plan, VARIANTS, servos_for, frozen, debug)
