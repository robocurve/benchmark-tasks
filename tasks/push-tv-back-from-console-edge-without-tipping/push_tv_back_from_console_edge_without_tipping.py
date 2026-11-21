"""Push a flat-screen TV back from the front edge of a dresser top by its stand, without tipping it.

A 32 in flat-screen TV (Samsung UN32N5300, 732.9 x 462.8 x 165.8 mm with its two Mini Arc feet, 4.0 kg) stands on a
bedroom dresser top with its feet 15 mm behind the front edge, the screen facing the room (the arms). Push it straight
back until both feet are at least SETBACK behind the front edge, square to the edge and upright, then take the hands
away. A TV pushed on the screen tips: with the feet 165.8 mm deep it tips over its rear foot edge when pushed higher
than (165.8 / 2) / mu above the top, 98 to 207 mm for mu 0.85 to 0.4, and the screen centre is 247 mm up. Pushed low,
at the bottom bezel just above the feet, it slides.

Scene (station frame: x forward from the arm bases, y left, z up from the table top):
- dresser top: the station table top; its front edge is the line x = EDGE_X (tape strip in front of it); the wall
  behind the dresser at x = WALL_X (IKEA MALM 6-drawer dresser, 48 cm deep);
- TV: one rigid body, the panel (78.5 mm deep) centred over two feet 165.8 mm deep whose outer edges are 684.1 mm apart.

Score (``score()``), all from the TV's pose: 0 if it ever tilts more than TIP_DEG, is lifted more than LIFT_M off the
top, or a foot comes over the front edge (all latched); 0.5 while both feet are wholly SETBACK or more behind the front
edge, the TV upright (UPRIGHT_DEG) and square to the edge (YAW_DEG); 1.0 with, as well, no hand on it (mk.released) and
the TV moved under STILL_MM and turned under STILL_DEG over the last SUCCESS_HOLD_S.
"""

from __future__ import annotations

import math
import pathlib
import sys

import numpy as np

_HERE = str(pathlib.Path(__file__).resolve().parent)
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

import arena_assets_fix  # noqa: E402, F401
import mechkit as mk  # noqa: E402
from yam_arena import station  # noqa: E402

SCENE = "station"
TOP = station.TABLE_TOP

BLACK, BEZEL, WHITE, TAPE = (0.05, 0.05, 0.06), (0.12, 0.12, 0.13), (0.93, 0.93, 0.91), (0.85, 0.65, 0.10)

# -- Samsung UN32N5300 (samsung.com/ca UN32N5300AFXZC spec table) --------------------------------------------------------
SET_W, SET_H, SET_D = 0.7329, 0.4628, 0.1658  # with stand
PANEL_H, PANEL_D = 0.4326, 0.0785  # without stand
STAND_W = 0.6841  # 'Stand (Basic) (WxD) 684.1 x 165.8 mm': outer edges of the two feet
FOOT_W = 0.137  # one Mini Arc foot, 5.4 in wide (retail stand dimension 5.4 x 6.5 x 1.8 in)
STAND_H = SET_H - PANEL_H  # 30.2 mm: panel bottom above the top
TV_M = 4.0  # 'Set Weight with Stand 4.0 kg'
FOOT_Y = STAND_W / 2 - FOOT_W / 2

# -- dresser top -------------------------------------------------------------------------------------------------------
EDGE_X = 0.320  # front edge of the dresser top
DRESSER_D = 0.480  # IKEA MALM 6-drawer dresser depth
WALL_X = EDGE_X + DRESSER_D
START_SETBACK = 0.015  # feet front this far behind the edge at the start
TV_X = EDGE_X + START_SETBACK + SET_D / 2  # TV frame: centre of the feet's footprint, on the top
PANEL_FRONT0 = TV_X - PANEL_D / 2

kit = mk.Kit()
kit.part("edge_tape", (EDGE_X - 0.010, 0.0, TOP), color=TAPE, boxes=[mk.box((0.020, 0.90, 0.001), (0.0, 0.0, 0.0005))])
kit.part("wall", (WALL_X + 0.005, 0.0, TOP), color=WHITE, boxes=[mk.box((0.010, 1.10, 0.70), (0.0, 0.0, 0.35))])
kit.part("tv", (TV_X, 0.0, TOP), mass=TV_M, color=BLACK, boxes=[
    mk.box((PANEL_D, SET_W, PANEL_H), (0.0, 0.0, STAND_H + PANEL_H / 2)),
    mk.box((SET_D, FOOT_W, STAND_H), (0.0, FOOT_Y, STAND_H / 2), BEZEL),
    mk.box((SET_D, FOOT_W, STAND_H), (0.0, -FOOT_Y, STAND_H / 2), BEZEL),
])
OBJECTS = kit.objects()

# -- scoring ------------------------------------------------------------------------------------------------------------
SETBACK = 0.100  # both feet wholly this far behind the front edge
TIP_DEG = 10.0
UPRIGHT_DEG = 2.0
YAW_DEG = 10.0
LIFT_M = 0.010
SUCCESS_HOLD_S = 1.0
CONTROL_HZ = 20.0  # the run's control rate: one tv_step per control step
STILL_MM, STILL_DEG = 2.0, 1.0
SCORE_OVER = "end"
FOOT_FRONT_CORNERS = [(-SET_D / 2, s * FOOT_Y + d * FOOT_W / 2, 0.0) for s in (-1, 1) for d in (-1, 1)]


def tv_rot(objects) -> np.ndarray:
    return mk.quat_to_mat(kit.pose(objects, "tv")[1])


def tilt_deg(objects) -> float:
    return float(np.degrees(np.arccos(np.clip(tv_rot(objects)[2, 2], -1.0, 1.0))))


def yaw_deg(objects) -> float:
    r = tv_rot(objects)
    return float(abs(np.degrees(math.atan2(r[1, 0], r[0, 0]))))


def setback(objects) -> float:
    """How far (m) the foremost front corner of either foot is behind the front edge."""
    return float(min(kit.point(objects, "tv", c)[0] for c in FOOT_FRONT_CORNERS) - EDGE_X)


def lift(objects) -> float:
    return float(kit.pose(objects, "tv")[0][2] - TOP)


def _mem():
    m = mk.MEMORY.get("tv")
    if m is None:
        m = {"spoiled": False}
        mk.MEMORY["tv"] = m
    return m


def tv_step(objects) -> None:
    m = _mem()
    hist = mk.MEMORY.setdefault("tv_hist", [])
    hist.append((np.array(kit.pose(objects, "tv")[0], float), tv_rot(objects)))
    del hist[:-(int(round(SUCCESS_HOLD_S * CONTROL_HZ)) + 1)]
    if tilt_deg(objects) > TIP_DEG or lift(objects) > LIFT_M or setback(objects) < 0.0:
        m["spoiled"] = True


kit.on_step(tv_step)


def tv_still() -> bool:
    hist = mk.MEMORY.get("tv_hist", [])
    if len(hist) < int(round(SUCCESS_HOLD_S * CONTROL_HZ)) + 1:
        return False
    (pa, ra), (pb, rb) = hist[0], hist[-1]
    turn = math.degrees(math.acos(float(np.clip((np.trace(ra.T @ rb) - 1.0) / 2.0, -1.0, 1.0))))
    return bool(np.linalg.norm(pb - pa) * 1000.0 < STILL_MM and turn < STILL_DEG)


def placed(objects) -> bool:
    return setback(objects) >= SETBACK and tilt_deg(objects) <= UPRIGHT_DEG and yaw_deg(objects) <= YAW_DEG


def score(objects) -> float:
    if _mem()["spoiled"] or not placed(objects):
        return 0.0
    if mk.released(objects["tv"]) and tv_still():
        return 1.0
    return 0.5


def success(objects) -> bool:
    return score(objects) >= 1.0 - 1e-9


def distance(objects) -> float:
    return float(max(0.0, SETBACK - setback(objects)))


# -- scripted demo: both hands push the bottom bezel above the feet -----------------------------------------------------
import yam_arena.demo as _yd  # noqa: E402
from yam_arena import ik  # noqa: E402
from yam_arena.constants import FINGER_CLOSED, PAD_TIP_PAST_TCP  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402


class RWaypoint(Waypoint):
    """A Waypoint with a TCP rotation ``rot`` (row-major 3x3); None: the stock top-down grip turned by yaw."""

    def __init__(self, *args, rot=None, **kwargs):
        super().__init__(*args, **kwargs)
        self.rot = rot


def _target(self, t: float):
    """yam_arena.demo.ArmPlan.target, reading a waypoint's ``rot`` when it has one (the stock plan is top-down)."""
    prev_pos, prev_rot, prev_finger = self.start_pos, self.start_rot, self.finger_start
    for w in self.waypoints:
        rot = np.asarray(w.rot, float) if getattr(w, "rot", None) is not None else _yd.grip_rotation(w.yaw)
        goal = np.asarray(w.pos, dtype=float)
        if t < w.duration:
            s = ik.smoothstep(t / w.duration)
            return (prev_pos + s * (goal - prev_pos), ik.slerp_matrix(prev_rot, rot, s),
                    prev_finger + s * (w.finger - prev_finger), w.label)
        t -= w.duration
        prev_pos, prev_rot, prev_finger = goal, rot, w.finger
    return prev_pos, prev_rot, prev_finger, "done"


_yd.ArmPlan.target = _target

R_FWD = np.array([[0.0, 0.0, 1.0], [0.0, -1.0, 0.0], [1.0, 0.0, 0.0]])  # tool axis +x, fingers closing along z
PITCH = math.radians(20.0)  # nose down: the closed fingertips reach low over the top
R_PUSH = mk.axis_angle((0.0, 1.0, 0.0), PITCH) @ R_FWD
PUSH_Y = FOOT_Y  # each hand pushes above its own foot
PUSH_Z = TOP + STAND_H + 0.018  # pad tip on the bottom bezel, 48 mm above the top
PUSH_D = 0.120  # feet front 15 -> 135 mm behind the edge
PRESS = 0.004  # pad tip this far past the bezel face at the end of the stroke
CLEAR_Y = 0.42


def tcp_for_tip(tip) -> np.ndarray:
    return np.asarray(tip, float) - R_PUSH[:, 2] * PAD_TIP_PAST_TCP


def demo():
    c = FINGER_CLOSED
    plans = {}
    for arm, s in (("left", 1.0), ("right", -1.0)):
        y = s * PUSH_Y
        p = []

        def add(dt, tip, label):
            p.append(RWaypoint(dt, tuple(float(v) for v in tcp_for_tip(tip)), c, label, rot=tuple(map(tuple, R_PUSH))))

        x0 = PANEL_FRONT0
        add(2.5, (x0 - 0.06, y, PUSH_Z + 0.04), "in front of the bottom bezel")
        add(1.2, (x0 - 0.02, y, PUSH_Z), "down to the bezel, above the foot")
        add(1.0, (x0 + 0.001, y, PUSH_Z), "touch the bezel")
        n = 6
        for i in range(1, n + 1):
            add(4.8 / n, (x0 + 0.001 + (PUSH_D + PRESS - 0.001) * i / n, y, PUSH_Z), "push the TV straight back")
        x1 = x0 + PUSH_D + PRESS
        add(1.2, (x1 - 0.06, y, PUSH_Z), "back off the bezel")
        clear = (x1 - 0.08, s * CLEAR_Y, PUSH_Z + 0.06)  # out beside the TV: the top cameras see the feet and the edge
        add(1.0, (x1 - 0.06, y, PUSH_Z + 0.04), "lift off the bezel")
        add(1.5, clear, "out beside the TV")
        add(SUCCESS_HOLD_S + 2.0, clear, "hold clear")
        plans[arm] = p
    return plans
