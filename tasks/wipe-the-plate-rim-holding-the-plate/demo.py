"""Scripted demo for wipe-the-plate-rim-holding-the-plate, run in the yam_arena embodiment (fixed layout).

    python demo.py task.py OUT_DIR [--variant full] [--no-record]   # demo.mp4, setup.png, result.json
    python demo.py task.py --plan-only                              # IK check of every variant's plan
    DEMO_DEBUG=10 python demo.py ...                                # print the wipe state every 10 steps
    DEMO_DEBUG=1 DEMO_TRACE=trace.jsonl python demo.py ...          # also write every step's poses and contacts

full        The left arm pinches the plate's rim lip on its left side and holds the plate; the right arm picks up the
            towel pad, presses it on each smudge (near, right, far-right) and rubs it back and forth along the rim over
            the whole smudge (eight strokes, the towel's own measured position keeping it centred across the rim), then
            lifts the towel off the plate. The recorded run ends there, on success (step 709, about 2 s after the towel
            left the plate), before the plan's last steps (towel back on its stand, the left arm letting go) are reached.
Failure and cheating variants:
no_hold     the right arm wipes the same way; nobody holds the plate (expected: it slides) (0)
light       as full, but the towel only skims the rim (no pressure) (0)
two         as full, but the far-right smudge is left (0.67)
partial     as full, but at each smudge the towel rubs only one side of it: centred 1.5 cm along the rim from the
            smudge's centre, in short strokes (the previous rule counted this as clean) (0)
food        as full, but after the right smudge the towel is lifted 2 mm and pushed in from the rim against the side of
            the food (0)
"""

from __future__ import annotations

import json
import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import democore as dc  # noqa: E402

VARIANTS = ("full", "no_hold", "light", "two", "partial", "food")
_CACHE: dict = {}
LIP_OPEN = 0.30  # gripper opening while approaching a plate's rim lip (about 2.8 cm)
PRESS_DEPTH = 0.002  # m: the towel commanded this far into the rim (10 mm gave 7-11 N and the towel stuck; 5 mm gave
# 4-6 N, and rubbing the smudge opposite the hold turned the plate 5.5 deg)
STROKE = 0.025  # m: rub half-length along the rim (commanded; the soft wrist lags it)
STROKES = 8  # strokes per smudge, alternating direction
SWEEP = 0.001  # m: pairs of strokes alternate this far out / in across the rim, so both edges of the smudge are covered
# (the 3 cm pad is as wide as the 3 cm smudge; the lip is 3.5 mm beyond the pad's outer edge)
PARTIAL_SHIFT, PARTIAL_STROKE = 0.015, 0.008  # m: partial: the towel centred this far along the rim, short strokes
FOOD_PUSH = 0.005  # m: food: the towel's inner face is commanded this far into the side of the food (open loop; the
# arm lags a few millimetres short of it)
FOOD_AT = "right"  # food: the smudge after which the towel is pushed in (pushed in from the near smudge, toward the
# holding hand, the plate and the holding hand were shoved 2 cm with no towel or food contact)
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

        def across(R, k):  # angle between the fingers' closing axis and the rim (0: the fingers lie along the rim)
            tx, ty = TANGENT[k]
            return round(math.degrees(math.acos(min(1.0, abs(R[0, 0] * tx + R[1, 0] * ty)))), 1)

        print(f"closing axis vs the rim: near {across(RW, 'near')}, right {across(RWr, 'right')}, far-right "
              f"{across(RWd, 'far')} deg", flush=True)
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
        u = np.array([*TANGENT[k], 0.0])  # along the rim
        n = np.array([sx, sy, 0.0]) / math.hypot(sx, sy)  # across the rim, outward
        shift, amp = (PARTIAL_SHIFT, PARTIAL_STROKE) if variant == "partial" else (0.0, STROKE)
        c = np.array([p0[0] + sx, p0[1] + sy, band]) + grip + shift * u
        Rt.add(2.0, c + [0, 0, 0.09], RW, 0.0, f"towel: above the {k} smudge")
        Rt.add(1.0, c + [0, 0, 0.04], RW, 0.0, f"towel: down to the {k} smudge")
        Rt.add(1.0, c + [0, 0, 0.003], RW, 0.0, f"towel: onto the {k} smudge")
        Rt.add(0.6, c - [0, 0, depth], RW, 0.0, f"towel: press {k}")
        for i in range(STROKES):
            sgn = 1 if i % 2 == 0 else -1
            lat = SWEEP if (i // 2) % 2 == 0 else -SWEEP
            Rt.add(0.4, c - [0, 0, depth] + sgn * amp * u + lat * n, RW, 0.0, f"towel: rub {k}")
        Rt.add(0.4, c - [0, 0, depth], RW, 0.0, f"towel: rub {k}")
        if variant == "food" and k == FOOD_AT:  # before the last smudge: the episode ends once all three are clean
            # lifted 2 mm off the band and pushed in toward the plate's centre until its inner face is FOOD_PUSH into
            # the side of the food (which is 9.5 cm across), then back over the smudge
            reach = math.hypot(sx, sy) - (t.FOOD_D / 2 + t.TOWEL[0] / 2 - FOOD_PUSH)
            cf = c - reach * n + [0.0, 0.0, 0.002]
            Rt.add(0.6, c + [0.0, 0.0, 0.002], RW, 0.0, "towel: up 2 mm (food)")
            Rt.add(1.5, cf, RW, 0.0, "towel: in to the food")
            Rt.add(0.8, cf, RW, 0.0, "towel: against the food")
            Rt.add(1.0, c + [0.0, 0.0, 0.004], RW, 0.0, "towel: back from the food")
        Rt.add(1.0, c + [0, 0, 0.09], RW, 0.0, f"towel: off the {k} smudge")
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


class TowelServo:
    """The towel's own measured position steers the hand on the smudges: the pad can sit a few millimetres off the TCP
    in the grip and the soft arm sags, while the 3 cm pad must cover the whole 3 cm smudge. Lowered onto a smudge and
    pressed, the towel's centre is brought onto the planned spot (the integral of the horizontal error is added to the
    TCP target); while rubbing, only its offset across the rim is corrected and the stroke along the rim is left to the
    plan. Lifting off a smudge and pushing against the food (the food variant), the correction is kept as it is.
    Applied after PlateFollow, so the planned spot already follows the plate."""

    GAIN_XY, GAIN_ACROSS, CLIP = 0.12, 0.05, 0.015  # per step, per step, m (slower across the rim: the strokes
    # shake the pad a little each 0.4 s, and only the slow offset matters there)

    def __init__(self, t):
        self.t = t
        self.c0 = np.array(t.PLATE_POS, float)
        self.hz = float(t.TOWEL[2] - 0.004)  # the TCP above the towel's underside, as in build_plan's grip
        self.off = np.zeros(3)
        self.want = None  # where the towel's underside centre should be (world), from the last commanded target

    @staticmethod
    def _mode(label: str):
        key = label.split(": ")[-1] if label.startswith("towel: ") else ""
        if "food" in key or key.startswith("off the"):  # kept, not integrated: pushed against something, the
            return "hold"  # integral winds up (it did so at the food, and the plate was shoved 2 cm)
        if key.startswith(("onto the", "press ")):  # not while descending: the arm's lag is no offset
            return "xy"
        if key.startswith("rub "):
            return "across"
        return None

    def update(self, kk, label: str) -> None:
        mode = self._mode(label)
        if mode is None:
            self.off = np.zeros(3)
            return
        if mode == "hold" or self.want is None or "towel" not in kk.STATE.poses:
            return
        e = self.want - np.asarray(kk.STATE.poses["towel"][0], float)
        e[2] = 0.0
        if mode == "across":
            k = label.split(" ")[-1]
            sx, sy = self.t.SMUDGES[k]
            _, R = kk.pose_of("plate", self.c0)
            n = R @ np.array([sx, sy, 0.0])
            n[2] = 0.0
            n = n / np.linalg.norm(n)
            e = (e @ n) * n
        gain = self.GAIN_ACROSS if mode == "across" else self.GAIN_XY
        self.off = np.clip(self.off + gain * e, -self.CLIP, self.CLIP)
        _T["servo"] = {"want": [round(float(v), 4) for v in self.want], "off_mm": [round(1000 * float(v), 2) for v in self.off]}

    def correct(self, pos, R, label):
        self.want = np.asarray(pos, float) - [0.0, 0.0, self.hz]
        if self._mode(label) is None:
            return pos, R
        return np.asarray(pos, float) + self.off, R


_T = {}


def servos_for(name, arm):
    if name != "right" or "t" not in _T:
        return []
    return [PlateFollow(_T["t"], _T.get("variant") != "light"), TowelServo(_T["t"])]


def frozen(label: str) -> bool:
    return (label.endswith(("grip", "let go", "plate: hold")) or label.startswith(("towel: press", "towel: rub"))
            or "food" in label)


def debug(k, t, kk, labL, labR):
    r = t.report()
    path = os.environ.get("DEMO_TRACE")
    if path:
        rec = {"step": k, "left": labL, "right": labR,
               "poses": {n: [[round(float(v), 5) for v in p], [round(float(v), 5) for v in q]]
                         for n, (p, q) in kk.STATE.poses.items()},
               "contacts": {f"{a}|{b}": round(float(v), 4) for (a, b), v in kk.STATE.contacts.items() if v},
               "peaks": {f"{a}|{b}": round(float(v), 4) for (a, b), v in getattr(kk.STATE, "peaks", {}).items() if v},
               "fingers": {a: [[round(float(v), 4) for v in pt] for pt in pts] for a, pts in kk.STATE.fingers.items()},
               "servo": _T.get("servo"), "report": r}
        with open(path, "a") as fh:
            fh.write(json.dumps(rec, default=float) + "\n")
    if k % 10 == 0:
        print(f"  dbg {k}: F {r.get('force')} v {r.get('speed')} under {r.get('under_pad')} cover {r.get('coverage')} "
              f"moved {r.get('plate_moved')} yaw {r.get('plate_yaw')} food {r.get('food_touched')} gap "
              f"{r.get('food_gap_min')} servo {(_T.get('servo') or {}).get('off_mm')} | {labL} | {labR}", flush=True)


if __name__ == "__main__":
    dc.main(build_plan, VARIANTS, servos_for, frozen, debug)
