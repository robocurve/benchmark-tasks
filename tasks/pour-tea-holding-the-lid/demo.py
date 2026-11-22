"""Scripted demo for pour-tea-holding-the-lid, run in the yam_arena embodiment (fixed layout).

    python demo.py task.py OUT_DIR [--variant full] [--no-record]   # demo.mp4, setup.png, result.json
    python demo.py task.py --plan-only                              # IK check of every variant's plan
    DEMO_DEBUG=10 python demo.py ...                                # print the pot's state every 10 steps

full        The left arm pinches the teapot's handle, lifts the pot off the trivet and holds it beside the cup; the
            right arm pinches the lid's knob. Together they tilt the pot 55 deg about the knob so the spout goes down
            over the cup, hold the pour 3.5 s and tilt it back; the right arm lets go of the knob and the left arm puts
            the pot back on its trivet.
Failure and cheating variants:
one_arm     the left arm pours alone; nobody holds the lid (0)
miss_cup    both arms pour with the pot 6 cm off to the side, so the spout pours beside the cup (0)
short       as full, but the pot is tilted only to 50 deg and held there 1 s (0.25)
no_return   as full, but the pot is left held upright beside the cup (0.5)
"""

from __future__ import annotations

import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import democore as dc  # noqa: E402

VARIANTS = ("full", "one_arm", "miss_cup", "short", "no_return")
_CACHE: dict = {}
PIVOT_Z = 0.171  # m: the knob grasp point above the table during the pour (the pot tilts about it; at 0.156 the
# spout poured from only 7-10 mm above the cup's rim, under the 1 cm the scorer takes as over the cup)
TILT = 55.0  # deg
KNOB_OPEN = 0.40  # opening around the 22 mm knob (about 3.8 cm): fully open, the fingertips would land on the cap
KNOB_UP = 0.006  # m: the knob grasped 6 mm above its centre, so the fingertips (13 mm past the TCP) clear the cap
HANDLE_OPEN = 0.36  # gripper opening around the 16 mm handle (about 3.4 cm), top and bottom
IN = 0.06  # m: the hand starts this far back along the handle and slides onto it
STEPS = (0, 10, 20, 30, 40, 50, 55)


def _closing_y():
    """Grips closing across y: the pot hangs from its handle, and pours, about x -- an axis the pads resist with their
    whole contact patch (a pinch closing along x would let the pot turn about the closing axis)."""
    return [R for tl in range(0, 75, 5) for az in (0, 90, 180, 270) for roll in (0, 90, 180, 270)
            for R in [dc.R_approach(tl, az, roll)] if abs(R[1, 0]) >= 0.95]


def _handle_grips():
    """Handle grips: the hand comes level along the side handle (+x), closing top and bottom (z), so pouring (about x)
    only rolls the wrist."""
    return [R for tl in (75, 80, 85, 90, 95, 100, 105) for az in (-20, -10, 0, 10, 20) for roll in (90, 270)
            for R in [dc.R_approach(tl, az, roll)] if abs(R[2, 0]) >= 0.95]


def best_rotation_seq(side: str, poses, cands, iters: int = 200):
    """The candidate R0 whose worst IK error over ``poses`` = [(TCP position, T)] with TCP rotation T @ R0 is smallest."""
    from yam_arena import ik, station
    from yam_arena.constants import READY

    best = (None, np.inf)
    for R0 in cands:
        worst = 0.0
        for pos, T in poses:
            e = np.inf
            for sd in [np.array(READY, float), *dc.SEEDS[:5]]:
                q, pe, re = ik.solve(np.asarray(pos, float), ik.matrix_to_quat_xyzw(T @ R0), sd,
                                     base_pos=station.BASES[side], iters=iters)
                e = min(e, pe + 0.05 * re)
                if e < 0.001:
                    break
            worst = max(worst, e)
            if worst >= best[1]:
                break
        if worst < best[1]:
            best = (R0, worst)
    return best


def build_plan(t, variant: str = "full", params: dict | None = None):
    from yam_arena import ik, station
    from yam_arena.constants import READY

    bases = station.BASES
    starts = {s: ik.tcp_pose(np.array(READY), bases[s]) for s in ("left", "right")}
    arms = {s: dc.Arm(s, bases[s], READY, *starts[s]) for s in ("left", "right")}
    L, Rt = arms["left"], arms["right"]
    knob_rel = np.array([0.0, 0.0, t.POT_H + t.KNOB_Z + KNOB_UP])  # the knob grasp point in the pot's frame (lid seated)
    handle_rel = np.array(t.HANDLE_XYZ) + [-0.005, 0.0, 0.0]  # the grasp point: pads along the handle, fingertips 2 cm
    # short of the pot's wall
    pot0 = np.array(t.POT_POS)
    dy = -0.06 if variant == "miss_cup" else 0.0
    pivot = np.array([t.CUP_XY[0], t.CUP_XY[1] + 0.048 + dy, t.TOP + PIVOT_Z])  # the knob's centre while pouring
    pot1 = pivot - knob_rel  # the pot's frame, upright, under the pivot

    def rx(deg):
        return dc.rot([1.0, 0.0, 0.0], math.radians(deg))

    ck = ("grips", dy)
    if ck not in _CACHE:
        cands = _closing_y()
        I3 = np.eye(3)
        hp = [(pot0 + handle_rel, I3), (pot0 + handle_rel - [IN, 0, 0], I3), (pot1 + handle_rel, I3)] + \
             [(pivot + rx(a) @ (handle_rel - knob_rel), rx(a)) for a in STEPS]
        RH, e1 = best_rotation_seq("left", hp, _handle_grips())
        kp = [(pivot + [0, 0, 0.05], I3)] + [(pivot, rx(a)) for a in STEPS]
        RK, e2 = best_rotation_seq("right", kp, cands)
        _CACHE[ck] = (RH, RK)
        print(f"handle grip {1000 * e1:.1f}, knob grip {1000 * e2:.1f} (mm + 0.05 rad)", flush=True)
    RH, RK = _CACHE[ck]
    _T["RH"], _T["handle_rel"], _T["RK"], _T["knob_rel"] = RH, handle_rel, RK, knob_rel
    hold = 1.0 if variant == "short" else 3.5

    back = pot0 + handle_rel - IN * RH[:, 2]
    L.add(2.5, back, RH, HANDLE_OPEN, "pot: behind the handle")
    L.add(2.0, pot0 + handle_rel, RH, HANDLE_OPEN, "pot: around the handle")
    L.add(1.0, pot0 + handle_rel, RH, 0.0, "pot: grip")
    L.add(2.5, pot0 + handle_rel + [0, 0, pot1[2] - pot0[2] + 0.01], RH, 0.0, "pot: lift")
    L.add(2.0, pot1 + handle_rel + [0, 0, 0.01], RH, 0.0, "pot: beside the cup")
    L.add(0.8, pot1 + handle_rel, RH, 0.0, "pot: settle")
    if variant != "one_arm":
        Rt.add(L.duration - 2.0, label="lid: wait")
        Rt.add(2.0, pivot + [0, 0, 0.05], RK, KNOB_OPEN, "lid: above the knob")
        Rt.add(1.5, pivot, RK, KNOB_OPEN, "lid: around the knob")
        Rt.add(1.0, pivot, RK, 0.0, "lid: grip")
        L.add(Rt.duration - L.duration, pot1 + handle_rel, RH, 0.0, "pot: hold")
    # short: tilted only to 50 deg and held there 1 s (held 1 s at 55 deg, the slow tilt in and out alone kept the
    # pot past the 45 deg pouring tilt for over 3 s)
    top = 50 if variant == "short" else TILT
    up = [a for a in STEPS[1:] if a <= top]
    seq = up + [None] + list(reversed([0] + up[:-1]))
    for a in seq:
        if a is None:
            L.add(hold, pivot + rx(top) @ (handle_rel - knob_rel), rx(top) @ RH, 0.0, "pot: pour")
            if variant != "one_arm":
                Rt.add(hold, pivot, rx(top) @ RK, 0.0, "lid: hold")
            continue
        # 10 deg a second: at 20 deg/s the lid hand lagged and lifted the lid 8 mm off its seat
        L.add(1.0, pivot + rx(a) @ (handle_rel - knob_rel), rx(a) @ RH, 0.0, f"pot: tilt {a}")
        if variant != "one_arm":
            Rt.add(1.0, pivot, rx(a) @ RK, 0.0, f"lid: tilt {a}")
    if variant != "one_arm":
        Rt.add(0.8, grip=KNOB_OPEN, label="lid: let go")
        Rt.add(1.5, pivot + [0, 0, 0.05], RK, KNOB_OPEN, "lid: up")
        Rt.add(2.0, starts["right"][0], starts["right"][1], 1.0, "right: home")
        L.add(2.3, pot1 + handle_rel, RH, 0.0, "pot: hold")
    if variant == "no_return":
        L.add(2.0, pot1 + handle_rel, RH, 0.0, "pot: hold")
        return L, Rt, {}
    L.add(2.0, pot0 + handle_rel + [0, 0, pot1[2] - pot0[2] + 0.01], RH, 0.0, "pot: over the trivet")
    L.add(1.8, pot0 + handle_rel + [0, 0, 0.002], RH, 0.0, "pot: down")
    L.add(0.8, pot0 + handle_rel - [0, 0, 0.004], RH, 0.0, "pot: onto the trivet")
    L.add(0.8, grip=HANDLE_OPEN, label="pot: let go")
    L.add(1.5, back, RH, HANDLE_OPEN, "pot: back off")
    L.add(2.0, starts["left"][0], starts["left"][1], 1.0, "left: home")
    return L, Rt, {}


def _rotvec(R) -> np.ndarray:
    c = float(np.clip((np.trace(R) - 1.0) / 2.0, -1.0, 1.0))
    a = math.acos(c)
    if a < 1e-6:
        return np.zeros(3)
    v = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])
    return v * (a / (2.0 * math.sin(a)))


def _rotm(v) -> np.ndarray:
    a = float(np.linalg.norm(v))
    return np.eye(3) if a < 1e-9 else dc.rot(np.asarray(v) / a, a)


class PotServo:
    """The held pot steered onto its planned pose: the left hand's target is shifted and turned by the integrated
    error between where the plan puts the pot and where it is (held by its side handle, the 0.3-0.45 kg pot hung
    11-15 deg nose-down in the pinch, so the knob was off its place for the other hand and the spout off the cup)."""

    KEYS = ("lift", "beside the cup", "settle", "hold", "pour", "over the trivet", "down")
    KP, KR, PMAX, RMAX = 0.1, 0.05, 0.04, math.radians(20.0)

    def __init__(self, RH, handle_rel):
        self.RH, self.off = RH, RH.T @ (-np.asarray(handle_rel, float))  # the pot's frame in the TCP frame
        self.dp, self.rv, self.plan = np.zeros(3), np.zeros(3), None

    def _active(self, label: str) -> bool:
        key = label.split(": ")[-1]
        return label.startswith("pot: ") and (key in self.KEYS or key.startswith("tilt"))

    def update(self, kk, label: str) -> None:
        if self.plan is None or not self._active(label) or "teapot" not in kk.STATE.poses:
            if not self._active(label):
                self.dp, self.rv = self.dp * 0.9, self.rv * 0.9
            return
        p, R = kk.pose_of("teapot")
        pp, Rp = self.plan
        self.dp = self.dp + self.KP * (pp - np.asarray(p, float))
        n = float(np.linalg.norm(self.dp))
        if n > self.PMAX:
            self.dp *= self.PMAX / n
        self.rv = self.rv + self.KR * _rotvec(Rp @ np.asarray(R, float).T)
        n = float(np.linalg.norm(self.rv))
        if n > self.RMAX:
            self.rv *= self.RMAX / n

    def correct(self, pos, R, label):
        pos, R = np.asarray(pos, float), np.asarray(R, float)
        self.plan = (pos + R @ self.off, R @ self.RH.T)  # the pot's planned position and rotation
        return pos + self.dp, _rotm(self.rv) @ R


class LidFollow:
    """While the right hand holds the lid by its knob, its target is the knob's place on the pot as the pot actually is
    (position and rotation), not where the plan puts it: each arm tracks its own plan only to about 1 cm, and the lid
    then shifted 2 cm on the pot during the pour. The change is applied at most 2 mm and 1 deg a step."""

    KEYS = ("grip", "hold") 

    def __init__(self, RK, knob_rel):
        self.RK, self.knob = RK, np.asarray(knob_rel, float)
        self.pot, self.dp, self.dr = None, np.zeros(3), np.eye(3)

    def _active(self, label: str) -> bool:
        key = label.split(": ")[-1]
        return label.startswith("lid: ") and (key in self.KEYS or key.startswith("tilt"))

    def update(self, kk, label: str) -> None:
        self.pot = kk.pose_of("teapot") if "teapot" in kk.STATE.poses else None

    def correct(self, pos, R, label):
        pos, R = np.asarray(pos, float), np.asarray(R, float)
        want_p, want_R = pos, R
        if self._active(label) and self.pot is not None:
            p, Rp = np.asarray(self.pot[0], float), np.asarray(self.pot[1], float)
            want_p, want_R = p + Rp @ self.knob, Rp @ self.RK
        step = (want_p - pos) - self.dp
        n = float(np.linalg.norm(step))
        self.dp = self.dp + (step if n <= 0.002 else step * 0.002 / n)
        v = _rotvec((want_R @ R.T) @ self.dr.T)
        a = float(np.linalg.norm(v))
        self.dr = _rotm(v if a <= math.radians(1.0) else v * math.radians(1.0) / a) @ self.dr
        return pos + self.dp, self.dr @ R


_T: dict = {}


def servos_for(name, arm):
    if name == "left" and "RH" in _T:
        return [PotServo(_T["RH"], _T["handle_rel"])]
    if name == "right" and "RK" in _T:
        return [LidFollow(_T["RK"], _T["knob_rel"])]
    return []


def frozen(label: str) -> bool:
    return label.endswith(("grip", "let go", "onto the trivet"))


def debug(k, t, kk, labL, labR):
    r = t.report()
    print(f"  dbg {k}: tilt {r.get('tilt')} off {r.get('spout_off')} above {r.get('spout_above')} over "
          f"{r.get('over_cup')} pour {r.get('pour_steps')} lid {r.get('lid_shift')}/{r.get('lid_lift')}/"
          f"{r.get('lid_tilt')} ok {r.get('lid_ok')} lost {r.get('lid_lost')} spill {r.get('spilled')} home "
          f"{r.get('home')} | {labL} | {labR}", flush=True)


if __name__ == "__main__":
    dc.main(build_plan, VARIANTS, servos_for, frozen, debug)
