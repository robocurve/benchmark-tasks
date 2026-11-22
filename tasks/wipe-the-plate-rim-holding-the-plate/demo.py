"""Scripted demo for wipe-the-plate-rim-holding-the-plate, run in the yam_arena embodiment (fixed layout).

    python demo.py task.py OUT_DIR [--variant full] [--no-record]   # demo.mp4, setup.png, result.json
    python demo.py task.py --plan-only                              # IK check of every variant's plan
    DEMO_DEBUG=10 python demo.py ...                                # print the wipe state every 10 steps

full        The left arm pinches the plate's rim lip on its left side and holds the plate; the right arm picks up the
            towel pad, presses it on each smudge (near, right, far-right) and rubs it back and forth along the rim, then puts
            the towel back on its stand; the left arm lets go.
Failure and cheating variants:
no_hold     the right arm wipes the same way; nobody holds the plate (expected: it slides)
light       as full, but the towel only skims the rim (no pressure) (0)
two         as full, but the far-right smudge is left (0.67)
food        as full, but after the first smudge the towel is dragged across the food (0)
"""

from __future__ import annotations

import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import democore as dc  # noqa: E402

VARIANTS = ("full", "no_hold", "light", "two", "food")
_CACHE: dict = {}
LIP_OPEN = 0.30  # gripper opening while approaching a plate's rim lip (about 2.8 cm)
PRESS_DEPTH = 0.002  # m: the towel commanded this far into the rim (10 mm gave 7-11 N and the towel stuck; 5 mm gave
# 4-6 N, and rubbing the smudge opposite the hold turned the plate 5.5 deg)
STROKE = 0.025  # m: rub half-length along the rim (commanded; the soft wrist lags it)
TANGENT = {"far": (0.7071, 0.7071), "right": (1.0, 0.0), "near": (0.0, 1.0)}


def build_plan(t, variant: str = "full", params: dict | None = None):
    from yam_arena import ik, station
    _T["t"], _T["variant"] = t, variant
    from yam_arena.constants import READY

    bases = station.BASES
    starts = {s: ik.tcp_pose(np.array(READY), bases[s]) for s in ("left", "right")}
    arms = {s: dc.Arm(s, bases[s], READY, *starts[s]) for s in ("left", "right")}
    L, Rt = arms["left"], arms["right"]
    p0 = np.array(t.PLATE_POS)
    lip = p0 + [0.0, (t.PLATE - t.LIP_W) / 2, t.PLATE_T + t.LIP_H - 0.005]
    grip = np.array([0.0, 0.0, t.TOWEL[2] - 0.004])  # TCP above the towel's underside
    tw0 = np.array(t.TOWEL_POS)
    band = p0[2] + t.PLATE_T  # the rim band's surface
    order = ["near", "right"] + ([] if variant == "two" else ["far"])  # the far-right smudge last, then the stand
    depth = -0.002 if variant == "light" else PRESS_DEPTH  # light: skims 2 mm above the band
    if "grips" not in _CACHE:
        lipc = [R for tl in (0, 10, 20) for az in range(0, 360, 45) for roll in (0, 90, 180)
                for R in [dc.R_approach(tl, az, roll)] if abs(R[1, 0]) >= 0.95]
        RL, e1 = dc.best_rotation("left", [lip, lip + [0, 0, 0.05]], lipc)
        tc = [R for tl in (0, 10, 20, 30) for az in range(0, 360, 45) for roll in (0, 90, 180)
              for R in [dc.R_approach(tl, az, roll)] if abs(R[1, 0]) >= 0.95]
        pts = [tw0 + grip, tw0 + grip + [0, 0, 0.06]]
        for k, (sx, sy) in t.SMUDGES.items():
            c = np.array([p0[0] + sx, p0[1] + sy, band - PRESS_DEPTH]) + grip
            tx, ty = TANGENT[k]
            pts += [c + [tx * STROKE, ty * STROKE, 0], c - [tx * STROKE, ty * STROKE, 0], c + [0, 0, 0.06]]
        RW, e2 = dc.best_rotation("right", pts, tc)
        # at the right smudge the same grip turned a quarter turn about the vertical, so the fingers lie along the rim
        # (closing across y there, the inner finger came within 1 cm of the food); the towel's underside stays level
        sx, sy = t.SMUDGES["right"]
        c = np.array([p0[0] + sx, p0[1] + sy, band - PRESS_DEPTH]) + grip
        rp = [c + [STROKE, 0, 0], c - [STROKE, 0, 0], c + [0, 0, 0.06], c + [0, 0, 0.09]]
        RWr, e3 = dc.best_rotation("right", rp, [dc.rot([0.0, 0.0, 1.0], math.radians(a)) @ RW for a in (90, -90)])
        sx, sy = t.SMUDGES["far"]
        c = np.array([p0[0] + sx, p0[1] + sy, band - PRESS_DEPTH]) + grip
        d = np.array([0.7071, 0.7071, 0.0]) * STROKE
        RWd, e4 = dc.best_rotation("right", [c + d, c - d, c + [0, 0, 0.06], c + [0, 0, 0.09]],
                                   [dc.rot([0.0, 0.0, 1.0], math.radians(a)) @ RW for a in (45, -45, 135, -135)])
        e3 = max(e3, e4)
        _CACHE["grips"] = (RL, {"near": RW, "far": RWd, "right": RWr})
        print(f"lip grip {1000 * e1:.1f}, towel grip {1000 * e2:.1f}, turned at the right / far-right smudges {1000 * e3:.1f} "
              f"(mm + 0.05 rad)", flush=True)
    RL, RWk = _CACHE["grips"]
    RW = RWk["near"]

    if variant != "no_hold":
        L.add(2.5, lip + [0, 0, 0.05], RL, LIP_OPEN, "plate: above the lip")
        L.add(1.5, lip, RL, LIP_OPEN, "plate: around the lip")
        L.add(1.0, lip, RL, 0.0, "plate: grip")
    Rt.add(2.5, tw0 + grip + [0, 0, 0.05], RW, 1.0, "towel: above")
    Rt.add(1.5, tw0 + grip, RW, 1.0, "towel: around")
    Rt.add(1.0, tw0 + grip, RW, 0.0, "towel: grip")
    Rt.add(1.5, tw0 + grip + [0, 0, 0.06], RW, 0.0, "towel: lift")
    Rt.add(max(0.0, L.duration - Rt.duration), tw0 + grip + [0, 0, 0.06], RW, 0.0, "towel: wait")
    for k in order:
        RW = RWk[k]
        sx, sy = t.SMUDGES[k]
        c = np.array([p0[0] + sx, p0[1] + sy, band]) + grip
        tx, ty = TANGENT[k]
        d = np.array([tx * STROKE, ty * STROKE, 0.0])
        Rt.add(2.0, c + [0, 0, 0.09], RW, 0.0, f"towel: above the {k} smudge")
        Rt.add(1.0, c + [0, 0, 0.04], RW, 0.0, f"towel: down to the {k} smudge")
        Rt.add(1.0, c + [0, 0, 0.003], RW, 0.0, f"towel: onto the {k} smudge")
        Rt.add(0.6, c - [0, 0, depth], RW, 0.0, f"towel: press {k}")
        for sgn in (1, -1, 1, -1, 1, -1):
            Rt.add(0.4, c - [0, 0, depth] + sgn * d, RW, 0.0, f"towel: rub {k}")
        Rt.add(0.4, c - [0, 0, depth], RW, 0.0, f"towel: rub {k}")
        Rt.add(1.0, c + [0, 0, 0.09], RW, 0.0, f"towel: off the {k} smudge")
        if variant == "food" and k == order[0]:  # before the last smudge: the episode ends once all three are clean
            f = np.array([p0[0], p0[1], band]) + grip
            Rt.add(1.5, f + [-0.06, 0, 0.002], RW, 0.0, "towel: onto the food")
            Rt.add(1.5, f + [0.04, 0, 0.002], RW, 0.0, "towel: across the food")
            Rt.add(0.8, f + [0.04, 0, 0.05], RW, 0.0, "towel: up")
    hz = band + grip[2] + 0.09  # stay this high until clear of the plate (descending over it reached the food zone)
    Rt.add(2.5, np.array([tw0[0], tw0[1], hz]), RW, 0.0, "towel: over the stand")
    Rt.add(1.0, tw0 + grip + [0, 0, 0.05], RW, 0.0, "towel: above the stand")
    Rt.add(1.2, tw0 + grip + [0, 0, 0.003], RW, 0.0, "towel: down")
    Rt.add(0.8, grip=1.0, label="towel: let go")
    Rt.add(1.2, tw0 + grip + [0, 0, 0.06], RW, 1.0, "towel: up")
    Rt.add(2.0, starts["right"][0], starts["right"][1], 1.0, "right: home")
    if variant != "no_hold":
        L.add(Rt.duration - L.duration - 2.0, lip, RL, 0.0, "plate: hold")
        L.add(0.8, grip=LIP_OPEN, label="plate: let go")
        L.add(1.5, lip + [0, 0, 0.05], RL, LIP_OPEN, "plate: up")
        L.add(2.0, starts["left"][0], starts["left"][1], 1.0, "left: home")
    return L, Rt, {}


class PlateFollow:
    """The towel's targets on the plate follow the plate: shifted by its horizontal displacement and turned with its yaw
    about its centre since the start (the plan assumes it stays put; held by its lip it gives a centimetre or so). The
    applied shift moves toward the wanted one by at most 2 mm a step."""

    KEYS = ("smudge", "press ", "rub ", "food")
    TARGET, GAIN, LO, HI = 1.2, 0.0003, -0.004, 0.006  # the press: N, m per N per step, m (up / down)

    def __init__(self, t, regulate: bool = True):
        self.c0 = np.array(t.PLATE_POS, float)
        self.c, self.yaw, self.cur = self.c0.copy(), 0.0, np.zeros(3)
        self.regulate, self.dz = regulate, 0.0

    def update(self, kk, label: str) -> None:
        p, R = kk.pose_of("plate", self.c0)
        self.c = np.asarray(p, float)
        self.yaw = float(np.arctan2(R[1, 0], R[0, 0]))
        # pressing and rubbing, the towel is lowered or raised until the towel-plate contact is about 1.2 N (the
        # commanded 2 mm press gave 0.6-3.7 N depending on the arm's reach)
        if self.regulate and label.startswith(("towel: press", "towel: rub")):
            f = float(kk.STATE.contacts.get(("towel", "plate"), 0.0))
            self.dz = float(np.clip(self.dz + self.GAIN * (self.TARGET - f), self.LO, self.HI))
        elif self.dz:
            self.dz = self.dz - min(abs(self.dz), 0.001) * np.sign(self.dz)

    def correct(self, pos, R, label):
        key = label.split(": ")[-1]
        want = np.zeros(3)
        if label.startswith("towel: ") and any(k in key for k in self.KEYS):
            c, s = np.cos(self.yaw), np.sin(self.yaw)
            r = np.asarray(pos, float) - self.c0
            want[:2] = (self.c - self.c0)[:2] + np.array([c * r[0] - s * r[1], s * r[0] + c * r[1]]) - r[:2]
        step = want - self.cur
        n = float(np.linalg.norm(step))
        self.cur = self.cur + (step if n <= 0.002 else step * 0.002 / n)
        return np.asarray(pos, float) + self.cur - [0.0, 0.0, self.dz], R


_T = {}


def servos_for(name, arm):
    return [PlateFollow(_T["t"], _T.get("variant") != "light")] if name == "right" and "t" in _T else []


def frozen(label: str) -> bool:
    return label.endswith(("grip", "let go", "plate: hold")) or label.startswith(("towel: press", "towel: rub"))


def debug(k, t, kk, labL, labR):
    r = t.report()
    print(f"  dbg {k}: F {r.get('force')} over {r.get('over')} v {r.get('speed')} rub {r.get('rub_steps')} moved "
          f"{r.get('plate_moved')} yaw {r.get('plate_yaw')} food {r.get('food_touched')} | {labL} | {labR}", flush=True)


if __name__ == "__main__":
    dc.main(build_plan, VARIANTS, servos_for, frozen, debug)
