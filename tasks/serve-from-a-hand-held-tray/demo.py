"""Scripted demo for serve-from-a-hand-held-tray, run in the yam_arena embodiment (fixed layout).

    python demo.py task.py OUT_DIR [--variant full] [--no-record]   # demo.mp4, setup.png, result.json
    python demo.py task.py --plan-only                              # IK check of every variant's plan
    DEMO_DEBUG=10 python demo.py ...                                # print the service state every 10 steps

full        The right arm takes the tray by its near handle, lifts it, turns it lengthwise and holds it level in front
            of the guest's table, clear of it; the left arm takes each glass low on its body from the robot's side and
            sets it on a coaster; the right arm carries the tray back to the station and lets go.
Failure and cheating variants:
tray_on_table  the right arm sets the tray down on the edge of the guest's table and the left arm serves from it (0)
rim_grip       as full, but the left arm takes the first glass by its rim (0)
one_glass      as full, but only one glass is served (0.5)
no_return      as full, but the right arm keeps holding the tray at the end (0.75)
"""

from __future__ import annotations

import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import democore as dc  # noqa: E402

VARIANTS = ("full", "tray_on_table", "rim_grip", "one_glass", "no_return")
_CACHE: dict = {}
HOLD_XY, HOLD_Z, HOLD_YAW = (0.26, -0.04), 0.090, 90.0  # tray centre, underside height above the table, deg
GLASS_OPEN = 0.82  # opening around a 6 cm glass (about 7.7 cm), so the outer finger clears the other glass
TRAY_OPEN = 0.45  # opening at the tray handle (about 4.2 cm): fully open, the inner finger came down on a glass
GRASP_Z = 0.060  # m: glasses taken this high above their base: the 12 cm glass's rim zone starts at 95 mm (pads end
# 1 cm above the TCP), and the gripper's housing, about 4 cm below the TCP on a level approach, clears the tray's rim (at
# 45 mm it pressed on the rim and dragged the tray's end on the way out; on a 10 cm glass neither margin held)
OFF = 0.060  # m: each glass lifted this far off the tray before it is carried level to its coaster
RIM_Z = 0.105  # m: the rim_grip variant's grasp height (in the rim zone)
BACK = 0.10  # m: the left gripper's stand-off along its approach before and after each glass (at 7 cm, arriving with
# its 1-2 cm tracking lag, the hand bumped the glass near its rim)


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
    tray0 = np.array(t.TRAY_POS)
    handle = np.array(t.TRAY_HANDLE_C["near"]) + [0.0, 0.0, t.HANDLE[2] / 2 - 0.014]  # TCP 14 mm below the bar's top:
    # the inner fingertip (13 mm past the TCP) just above the tray, the pads along most of the bar
    Rz = lambda deg: dc.rot((0, 0, 1), np.radians(deg))  # noqa: E731
    on_table = variant == "tray_on_table"
    hold = np.array([0.30, 0.06, t.TOP + 0.0025]) if on_table else np.array([*HOLD_XY, t.TOP + HOLD_Z])

    def tray_tcp(c, yaw):
        return c + Rz(yaw) @ handle

    def glass_at(name, c, yaw):  # a glass's base centre with the tray at c, turned by yaw
        gx, gy = t.GLASS_ON_TRAY[name]
        return c + Rz(yaw) @ np.array([gx, gy, t.TRAY_T])

    if "grips" not in _CACHE:
        hc = [R for tl in (0, 10, 20, 30) for az in range(0, 360, 45) for roll in (0, 90)
              for R in [dc.R_approach(tl, az, roll)] if abs(R[0, 0]) >= 0.95]  # closing across the bar (tray x)
        I3 = np.eye(3)  # the hand turns with the tray: score each grip at the rotation it actually has there
        hp = [(tray_tcp(tray0, 0.0), I3), (tray_tcp(tray0, 0.0) + [0, 0, 0.06], I3), (tray_tcp(hold, HOLD_YAW), Rz(HOLD_YAW)),
              (tray_tcp(hold + [0, 0, 0.04], HOLD_YAW), Rz(HOLD_YAW))]
        RH, e1 = best_rotation_seq("right", hp, hc)
        # left: glasses from the robot's side, closing across y, approach level
        gc = [dc.R_approach(tl, 0, roll) for tl in (70, 80, 90) for roll in (0, 180)]
        gpts = []
        for n in t.GLASS_ON_TRAY:
            g = glass_at(n, hold, HOLD_YAW) + [0, 0, GRASP_Z]
            gpts += [g - [BACK, 0, 0], g]
        for x, y in t.COASTERS.values():
            gpts += [np.array([x, y, t.CLOTH_TOP + GRASP_Z]), np.array([x - BACK, y, t.CLOTH_TOP + GRASP_Z])]
        RG, e2 = dc.best_rotation("left", gpts, gc)
        # reach check (printed; not used by the plan): each arm to both glasses on the station and to both coasters, at
        # any of 48 gripper rotations (first one within 2 mm). Each arm reaches all four, so one arm could serve from
        # the tray left on the station: the task is bimanual by service practice, not by reach
        rc_ = [dc.R_approach(tl, az, roll) for tl in (0, 45, 90) for az in range(0, 360, 45) for roll in (0, 90)]

        def reach_err(side, p):
            best = np.inf
            for R in rc_:
                best = min(best, dc.best_rotation(side, [p], [R])[1])
                if best < 0.002:
                    break
            return best

        targets = [tray0 + np.array([*t.GLASS_ON_TRAY[n], t.TRAY_T + GRASP_Z]) for n in t.GLASS_ON_TRAY]
        targets += [np.array([x, y, t.CLOTH_TOP + GRASP_Z]) for x, y in t.COASTERS.values()]
        reach = {side: max(reach_err(side, p) for p in targets) for side in ("left", "right")}
        _CACHE["grips"] = (RH, RG)
        print(f"tray grip {1000 * e1:.1f}, glass grip {1000 * e2:.1f} (mm + 0.05 rad); reach check, worst over both glasses "
              f"on the station and both coasters: left arm {1000 * reach['left']:.0f}, right arm {1000 * reach['right']:.0f}",
              flush=True)
    RH, RG = _CACHE["grips"]

    def tray_to(dur, c, yaw, label, grip=0.0):
        Rt.add(dur, tray_tcp(c, yaw), Rz(yaw) @ RH, grip, label)

    up = np.array([0.0, 0.0, 0.06])
    # in from high above the handle, clear of the 12 cm glasses: from the start pose straight to 6 cm above it, the open
    # fingers swept past a glass's rim (scored as touched)
    tray_to(3.0, tray0 + [0.0, 0.0, 0.16], 0.0, "tray: high above the handle", TRAY_OPEN)
    tray_to(1.5, tray0 + up, 0.0, "tray: above the handle", TRAY_OPEN)
    tray_to(1.5, tray0, 0.0, "tray: around the handle", TRAY_OPEN)
    tray_to(1.0, tray0, 0.0, "tray: grip")
    tray_to(1.5, tray0 + up, 0.0, "tray: lift")
    tray_to(3.5, hold + [0, 0, 0.04] if on_table else hold, HOLD_YAW, "tray: to the guest's table")
    if on_table:
        tray_to(1.5, hold, HOLD_YAW, "tray: down on the guest's table")
        Rt.add(0.8, grip=TRAY_OPEN, label="tray: let go")
        tray_to(1.2, hold + up, HOLD_YAW, "tray: up", TRAY_OPEN)
    t_ready = Rt.duration
    L.add(t_ready, starts["left"][0], starts["left"][1], 1.0, "left: wait for the tray")
    names = list(t.GLASS_ON_TRAY)[::-1]  # the glass nearer the left arm first
    _T["glasses"] = {n: glass_at(n, hold, HOLD_YAW) for n in t.GLASS_ON_TRAY}
    # the far coaster first: the hand comes in from the robot's side, so going to the far coaster second, it passed
    # over the glass already on the near one and knocked it over
    coasters = [t.COASTERS["far"], t.COASTERS["near"]]
    if variant == "one_glass":
        names = names[:1]
    for i, n in enumerate(names):
        z = RIM_Z if (variant == "rim_grip" and i == 0) else GRASP_Z
        g = glass_at(n, hold, HOLD_YAW) + [0, 0, z]
        x, y = coasters[i]
        c = np.array([x, y, t.CLOTH_TOP + 0.002 + z])
        L.add(2.5, g - [BACK, 0, 0], RG, GLASS_OPEN, f"{n}: behind it")  # at grasp height: fingertips 2.5 cm under the rim zone
        L.add(1.0, g - [BACK, 0, 0], RG, GLASS_OPEN, f"{n}: level")
        L.add(1.5, g, RG, GLASS_OPEN, f"{n}: around")
        L.add(1.0, g, RG, 0.0, f"{n}: grip")
        L.add(1.5, g + [0, 0, OFF], RG, 0.0, f"{n}: off the tray")
        L.add(2.5, np.array([x, y, g[2] + OFF]), RG, 0.0, f"{n}: over the coaster")  # level, clear of the tray's
        L.add(2.2, c, RG, 0.0, f"{n}: down")  # end, then straight down (lowering on the way swept the tray down)
        L.add(0.8, grip=GLASS_OPEN, label=f"{n}: let go")
        L.add(1.2, c - [BACK, 0, 0], RG, GLASS_OPEN, f"{n}: back")
        # straight up before anything else: going from the coaster to the next glass at cloth height, the hand ran
        # into the held tray's end from below, tipped it 28 deg and knocked the other glass over
        L.add(1.2, c - [BACK, 0, -0.135], RG, GLASS_OPEN, f"{n}: up")
    L.add(2.0, starts["left"][0], starts["left"][1], 1.0, "left: home")
    if not on_table:
        Rt.add(max(0.5, L.duration - Rt.duration - 2.0), label="tray: hold")
        if variant != "no_return":
            tray_to(3.5, tray0 + up, 0.0, "tray: over the station")
            tray_to(1.5, tray0 + [0, 0, 0.002], 0.0, "tray: down")
            Rt.add(0.8, grip=TRAY_OPEN, label="tray: let go")
            tray_to(1.2, tray0 + up, 0.0, "tray: up", TRAY_OPEN)
            if variant == "one_glass":  # a glass is still on the tray: rise above it first (going home from 6 cm
                # over the tray, the open hand swept through the unserved glass's rim band)
                tray_to(1.5, tray0 + [0, 0, 0.18], 0.0, "tray: clear of the glass", TRAY_OPEN)
            Rt.add(2.0, starts["right"][0], starts["right"][1], 1.0, "right: home")
        else:
            Rt.add(3.0, label="tray: keep holding")
    return L, Rt, {}


class GlassFollow:
    """The left hand's approach to a glass on the held tray follows the glass: shifted by where the glass actually is
    against where the plan put it (held by one handle, the tray hangs about 1 cm low and 4-5 deg tilted, so the far
    glass sits 2-3 cm lower than planned). Kept while gripping, then let go of gradually; at most 2 mm a step."""

    APPROACH, HOLD = ("behind it", "level", "around"), ("grip",)

    def __init__(self, planned: dict):
        self.planned, self.actual, self.cur = planned, {}, np.zeros(3)

    def update(self, kk, label: str) -> None:
        for n, p in self.planned.items():
            self.actual[n] = np.asarray(kk.pose_of(n, p)[0], float)

    def correct(self, pos, R, label):
        n, _, key = label.partition(": ")
        want = np.zeros(3)
        if n in self.planned and key in self.APPROACH + self.HOLD:
            want = self.actual.get(n, self.planned[n]) - self.planned[n]
        step = want - self.cur
        d = float(np.linalg.norm(step))
        self.cur = self.cur + (step if d <= 0.002 else step * 0.002 / d)
        return np.asarray(pos, float) + self.cur, R


_T: dict = {}


def servos_for(name, arm):
    return [GlassFollow(_T["glasses"])] if name == "left" and "glasses" in _T else []


def frozen(label: str) -> bool:
    return label.endswith(("grip", "let go", "tray: hold", "tray: keep holding"))


def debug(k, t, kk, labL, labR):
    r = t.report()
    g = {n: (v["coaster"], v["tilt"], v["z"]) for n, v in r.get("glasses", {}).items()}
    print(f"  dbg {k}: served {r.get('served')} glasses {g} rim {r.get('rim_touched')} on_table "
          f"{r.get('tray_on_guest_table')} fell {r.get('glass_fell')} tray tilt {r.get('tray_tilt')} z {r.get('tray_z')} "
          f"back {r.get('tray_back')} | {labL} | {labR}", flush=True)


if __name__ == "__main__":
    dc.main(build_plan, VARIANTS, servos_for, frozen, debug)
