"""Put the long board away at the back of a countertop board rack, behind two boards leaning on its rail.

On the table, in front of the arms, stands a beech board rack: a floor 13 x 51 cm (x by y) and 1.2 cm thick, a
1 x 1 cm lip along its near edge, and a 2 x 2 cm back rail on two end posts with its top 10 cm above the table. The
rack's floor is 10 cm deep between the lip and the rail. Two bamboo serving boards (24 x 16 x 1.0 cm, 0.27 kg each)
stand on their long edges in the rack, feet against the lip, leaning back on the rail at about 40 degrees. A third,
longer bamboo board (32 x 12 x 1.2 cm, 0.30 kg) stands on its long edge in a beech drying stand (a slotted block)
nearer the arms, on the right.

The long board belongs at the back of the rack, nearest the rail, with the other two boards leaning on it. The
leaning boards cover the space at the rail, so the boards in front have to be tipped toward the robot and held while
the long board goes in behind them; they must stay in the rack the whole time.

Layout: trial k of a run uses row k mod 7 of LAYOUTS (the rack and the boards in it shift together; the drying stand
does not move). The scripted demo reads that table, never the simulator's object poses.
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
from partkit import finger_points, on_reset, on_step, part, write_pose  # noqa: E402
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
INSTRUCTION = "Put the long board away at the back of the rack, without taking the other boards out."

BEECH = (0.62, 0.43, 0.27)
BAMBOO = (0.86, 0.72, 0.47)
BAMBOO_LONG = (0.80, 0.64, 0.38)


def _cm(x0, x1, y0, y1, z0, z1, color):
    return (((x0 + x1) / 200, (y0 + y1) / 200, (z0 + z1) / 200),
            ((x1 - x0) / 100, (y1 - y0) / 100, (z1 - z0) / 100), color)


# ---- the rack, part frame in cm: origin on the table; y = 0 is where the long board's middle belongs -------------
FLOOR_T = 1.2           # floor top above the table
LIP_IN, LIP_H = -5.0, 1.0
RAIL_FRONT, RAIL_TOP = 5.0, 10.0          # 10 cm of floor between lip and rail
Y_IN = (-20.0, 27.0)    # inner length between the end posts
RACK_BOXES = [
    _cm(-6.0, RAIL_FRONT + 2, Y_IN[0] - 2, Y_IN[1] + 2, 0.0, FLOOR_T, BEECH),                    # floor
    _cm(-6.0, LIP_IN, Y_IN[0] - 2, Y_IN[1] + 2, FLOOR_T, FLOOR_T + LIP_H, BEECH),                # front lip
    _cm(RAIL_FRONT, RAIL_FRONT + 2, Y_IN[0] - 2, Y_IN[1] + 2, RAIL_TOP - 2, RAIL_TOP, BEECH),    # back rail
    _cm(RAIL_FRONT, RAIL_FRONT + 2, Y_IN[0] - 2, Y_IN[0], FLOOR_T, RAIL_TOP - 2, BEECH),         # end post
    _cm(RAIL_FRONT, RAIL_FRONT + 2, Y_IN[1], Y_IN[1] + 2, FLOOR_T, RAIL_TOP - 2, BEECH),         # end post
]
RACK_XY = (0.340, -0.130)    # world position of the rack origin in layout 0
# (dx, dy) in cm per trial, row k mod 7; published in the README
LAYOUTS = [(0.0, 0.0), (1.0, -5.0), (-1.0, 5.0), (0.5, 3.0), (-0.5, -3.0), (1.0, 4.0), (-1.0, -4.0)]
RACK = part("rack", pos=(*RACK_XY, TOP), mass=2.0, boxes=RACK_BOXES, color=BEECH, friction=1.2)

# ---- boards: x thickness, y length, z height (cm) when upright ------------------------------------------------------
TS, LS, HS, MS = 1.0, 24.0, 16.0, 0.27     # short boards
TL, LL, HL, ML = 1.2, 32.0, 12.0, 0.30     # long board
SHORT_Y = 13.0                              # short boards' middle, rack y (span 1 .. 25)


def _board(name, t, length, h, mass, pos, color):
    return part(name, pos=pos, mass=mass, boxes=[_cm(-t / 2, t / 2, -length / 2, length / 2, -h / 2, h / 2, color)],
                color=color, friction=1.2)


FRONT = _board("board_front", TS, LS, HS, MS, (RACK_XY[0] - 0.040, RACK_XY[1] + SHORT_Y / 100, TOP + 0.093), BAMBOO)
REAR = _board("board_rear", TS, LS, HS, MS, (RACK_XY[0] - 0.020, RACK_XY[1] + SHORT_Y / 100, TOP + 0.093), BAMBOO)

# drying stand (fixed): base 34 x 6 x 1 cm, two jaws 34 x 2 x 3 cm leaving a 1.5 cm slot; along y
STAND_XY = (0.200, -0.040)
SLOT = 1.5
STAND = [
    Box("stand_base", pos=(STAND_XY[0], STAND_XY[1], TOP + 0.005), size=(0.06, 0.34, 0.01), color=BEECH),
    Box("stand_jaw_near", pos=(STAND_XY[0] - (SLOT / 2 + 1.0) / 100, STAND_XY[1], TOP + 0.025),
        size=(0.02, 0.34, 0.03), color=BEECH),
    Box("stand_jaw_far", pos=(STAND_XY[0] + (SLOT / 2 + 1.0) / 100, STAND_XY[1], TOP + 0.025),
        size=(0.02, 0.34, 0.03), color=BEECH),
]
LONG = _board("board_long", TL, LL, HL, ML, (STAND_XY[0], STAND_XY[1], TOP + 0.011 + HL / 200), BAMBOO_LONG)

OBJECTS = [*STAND, RACK, FRONT, REAR, LONG]
SHORTS = (FRONT.name, REAR.name)
BOARDS = (*SHORTS, LONG.name)
SIZES = {FRONT.name: (TS, LS, HS), REAR.name: (TS, LS, HS), LONG.name: (TL, LL, HL)}


def _lean0() -> float:
    """Start lean (rad): front board's foot on the lip, rear board on it, its back face on the rail's top front edge."""
    d, h = RAIL_FRONT - LIP_IN, RAIL_TOP - FLOOR_T
    f = lambda a: (d - TS / math.cos(a)) * math.cos(a) - h * math.sin(a) - TS  # noqa: E731
    lo, hi = 0.05, 1.3
    for _ in range(60):
        mid = (lo + hi) / 2
        lo, hi = (mid, hi) if f(mid) > 0 else (lo, mid)
    return lo


LEAN0 = _lean0()
FOOT_FRONT = LIP_IN + 0.1                                       # front board's bottom-front edge, rack x
FOOT_REAR = FOOT_FRONT + TS / math.cos(LEAN0) + 0.1 / math.cos(LEAN0)


def layout(k: int):
    dx, dy = LAYOUTS[k % len(LAYOUTS)]
    return RACK_XY[0] + dx / 100, RACK_XY[1] + dy / 100


def _board_pose(origin, foot_x, lean, y_mid):
    u = np.array([math.sin(lean), math.cos(lean)])
    n = np.array([math.cos(lean), -math.sin(lean)])
    c = np.array([foot_x, FLOOR_T + 0.1]) + n * TS / 2 + u * HS / 2
    pos = (origin[0] + float(c[0]) / 100, origin[1] + y_mid / 100, TOP + float(c[1]) / 100)
    return pos, (0.0, math.sin(lean / 2), 0.0, math.cos(lean / 2))


# ---- trial record ----------------------------------------------------------------------------------------------------
_S: dict = {"trial": -1}
REST_STEPS = 20              # 1 s at 20 Hz
CLEAR = 0.030                # both grippers at least 3 cm from every board
OUT_DZ = 0.010               # a short board's lowest point more than 1 cm off the rack floor = taken out (latched)


@on_reset
def _reset(environment=None) -> None:
    _S["trial"] += 1
    _S.update(out=False, rest=0, clear=False, steps=0)
    if environment is None:
        return
    k = int(os.environ.get("BOARD_RACK_LAYOUT", _S["trial"]))
    _S["layout"] = k
    o = layout(k)
    write_pose(environment, RACK.name, (o[0], o[1], TOP), (0.0, 0.0, 0.0, 1.0))
    write_pose(environment, FRONT.name, *_board_pose(o, FOOT_FRONT, LEAN0, SHORT_Y))
    write_pose(environment, REAR.name, *_board_pose(o, FOOT_REAR, LEAN0, SHORT_Y))
    if os.environ.get("TASK_DEBUG"):
        print(f"debug reset: trial {_S['trial']} layout {k} rack {np.round(o, 3)}", flush=True)


_reset()
_S["trial"] = -1


def corners(st, size) -> np.ndarray:
    hx, hy, hz = (s / 200 for s in size)
    return np.array([st.to_world((sx * hx, sy * hy, sz * hz)) for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)])


def lean_deg(st) -> float:
    """Lean of the board's height axis from vertical; + = top toward the rail (away from the robot)."""
    up = st.rotation()[:, 2]
    return math.degrees(math.atan2(up[0], up[2]))


def box_distance(st, size, p) -> float:
    local = st.to_local(p)
    half = np.array(size) / 200
    return float(np.linalg.norm(local - np.clip(local, -half, half)))


def board_report(rack, st, size) -> dict:
    """Rack-frame facts about one board: lowest point above the floor (m), extent, foot x, lean."""
    c = np.array([rack.to_local(p) for p in corners(st, size)])
    bottom = rack.to_local(st.to_world((0.0, 0.0, -size[2] / 200)))
    return dict(low=float(c[:, 2].min()) - FLOOR_T / 100, xmin=float(c[:, 0].min()), xmax=float(c[:, 0].max()),
                ymin=float(c[:, 1].min()), ymax=float(c[:, 1].max()), foot_x=float(bottom[0]), lean=lean_deg(st))


@on_step
def _track(environment) -> None:
    from yam_arena.scene import object_states

    st = object_states(environment, [RACK.name, *BOARDS])
    rack = st[RACK.name]
    _S["steps"] += 1
    for n in SHORTS:
        if abs(board_report(rack, st[n], SIZES[n])["low"]) > OUT_DZ:
            _S["out"] = True                    # latched for the trial; cleared only in _reset
    still = all(float(np.linalg.norm(st[n].lin_vel)) < 0.01 and float(np.linalg.norm(st[n].ang_vel)) < 0.1
                for n in (*BOARDS, RACK.name))
    _S["rest"] = _S["rest"] + 1 if still else 0
    pts = finger_points(environment)
    _S["clear"] = all(box_distance(st[n], SIZES[n], p) >= CLEAR for n in BOARDS for p in pts)
    if os.environ.get("TASK_DEBUG") and (_S["steps"] % 20 == 0 or _S["steps"] in (1, 5)):
        rep = {n: board_report(rack, st[n], SIZES[n]) for n in BOARDS}
        txt = " | ".join(f"{n[6:]} lean {r['lean']:5.1f} low {1000 * r['low']:5.1f} foot {100 * r['foot_x']:5.2f} "
                         f"x {100 * r['xmin']:5.2f}..{100 * r['xmax']:5.2f} y {100 * r['ymin']:5.1f}..{100 * r['ymax']:5.1f}"
                         for n, r in rep.items())
        fp = [rack.to_local(p) for p in pts]
        lf, rf = fp[:4], fp[4:]
        print(f"debug step {_S['steps']}: {txt} | Lfing x {100 * min(p[0] for p in lf):5.2f} z {100 * min(p[2] for p in lf):5.1f}"
              f" Rfing x {100 * min(p[0] for p in rf):5.2f}..{100 * max(p[0] for p in rf):5.2f} z {100 * min(p[2] for p in rf):5.1f}"
              f" | out {_S['out']} rest {_S['rest']} clear {_S['clear']} score {_S.get('score')}", flush=True)


# ---- scoring -----------------------------------------------------------------------------------------------------
LOW_OK = 0.003               # resting on the rack floor: lowest point within 3 mm of it
LEAN_OK = (5.0, 40.0)        # short boards: leaning back, not flat
LEAN_OK_LONG = (-5.0, 40.0)  # the long board rests against the rail at 4 to 7 degrees, or stands upright if nothing
                             # leans on it; which depends on where its foot lands, not on the skill
LEAN_UPRIGHT = (-5.0, 40.0)  # partial credit: short boards still standing in the rack, upright or leaning back


def in_rack(r, lean_ok=LEAN_OK) -> bool:
    return (abs(r["low"]) <= LOW_OK and lean_ok[0] <= r["lean"] <= lean_ok[1]
            and Y_IN[0] / 100 - 0.003 <= r["ymin"] and r["ymax"] <= Y_IN[1] / 100 + 0.003
            and LIP_IN / 100 - 0.003 <= r["xmin"] and r["xmax"] <= (RAIL_FRONT + 2) / 100 + 0.003)


def score(objects) -> float:
    s = _score(objects)
    _S["score"] = s
    return s


def _score(objects) -> float:
    if _S.get("out") or not _S.get("clear") or _S.get("rest", 0) < REST_STEPS:
        return 0.0
    rack = objects[RACK.name]
    rep = {n: board_report(rack, objects[n], SIZES[n]) for n in BOARDS}
    # every board stands on the rack floor inside the rack; the short boards upright or leaning back
    if not (all(in_rack(rep[n], LEAN_UPRIGHT) for n in SHORTS) and in_rack(rep[LONG.name], LEAN_OK_LONG)):
        return 0.0
    nearest = all(rep[LONG.name]["foot_x"] > rep[n]["foot_x"] for n in SHORTS)
    leaning = all(in_rack(rep[n]) for n in SHORTS)
    # 1.0: long board at the back and the short boards leaning back on it; 0.5: all in the rack but one of those misses
    return 1.0 if nearest and leaning else 0.5


# ---- scripted demo ---------------------------------------------------------------------------------------------------
TIP = 0.013                    # pad tips below the TCP
FINGER_HALF = 0.008            # closed fingers' near face this far in front of the TCP (probe)
PULL_TIPS_Z = float(os.environ.get("DEMO_PULL_TIPS", "0.112"))   # pad tips above the table while pulling
PULL_X = float(os.environ.get("DEMO_PULL_X", "0.000"))           # TCP target, rack x (m), while holding
LONG_X = float(os.environ.get("DEMO_LONG_X", "0.034"))           # long board's middle, rack x (m), when set in
RELEASE = -0.010               # pads 20 mm apart: let go of the long board without reaching the boards beside it
GRIP_Y_LEFT = 0.22             # rack y of the left gripper: 3 cm in from the short boards' far end
YAW_LONG = math.pi / 2         # pads close along x, across the long board's thickness


def _plan_layout() -> int:
    k = os.environ.get("BOARD_RACK_LAYOUT")
    if k is not None:
        return int(k)
    return _S["trial"] + 1      # yam_scripted builds its plan just before the trial's reset, so plan the next row


def demo(k: int | None = None):
    k = _plan_layout() if k is None else k
    ox, oy = layout(k)
    if os.environ.get("TASK_DEBUG"):
        print(f"debug demo: trial {_S['trial']} plans layout {k}", flush=True)
    # left arm: behind the short boards' tops at their far end, pull toward the robot, hold, let back
    h = (PULL_TIPS_Z * 100) - FLOOR_T
    back = FOOT_REAR + TS * math.cos(LEAN0) + (h + TS * math.sin(LEAN0)) * math.tan(LEAN0)   # rear face, cm
    lx0 = ox + (back + 2.0) / 100 + FINGER_HALF
    lz = TOP + PULL_TIPS_Z + TIP
    ly = oy + GRIP_Y_LEFT
    lhold = ox + PULL_X
    # right arm: grip the long board's top edge in the stand, carry it to the rack's open end, slide it in
    gz = TOP + 0.011 + HL / 100 - 0.002                  # TCP: pads over the top 1.5 cm
    lift = 0.040                                        # board's bottom 4 cm higher: out of the 3 cm slot
    sx, sy = STAND_XY
    rx = ox + LONG_X
    y_in = oy - 0.18                                    # middle of the board at the open end, clear of the short boards
    y_set = oy + 0.0
    above_lip = 0.028                                   # bottom 2.5 cm above the table: over the lip
    above_floor = 0.012                                 # bottom 0.8 cm above the rack floor while sliding
    z_floor = gz + FLOOR_T / 100 - 0.010 + 0.001        # bottom 1 mm above the rack floor
    left = [
        Waypoint(2.5, (lx0, ly, lz + 0.08), FINGER_CLOSED, "above, behind the board tops"),
        Waypoint(1.5, (lx0, ly, lz), FINGER_CLOSED, "down behind the tops"),
        Waypoint(4.0, (lhold, ly, lz), FINGER_CLOSED, "tip them toward the robot"),
        Waypoint(13.0, (lhold, ly, lz), FINGER_CLOSED, "hold them"),
        Waypoint(3.0, (lx0, ly, lz), FINGER_CLOSED, "let them lean back"),
        Waypoint(1.0, (lx0 + 0.02, ly, lz + 0.08), FINGER_CLOSED, "lift clear"),
        Waypoint(2.0, (0.15, 0.40, TOP + 0.25), FINGER_CLOSED, "away"),
    ]
    right = [
        Waypoint(2.5, (sx, sy, gz + 0.06), FINGER_OPEN, "above the long board", YAW_LONG),
        Waypoint(1.5, (sx, sy, gz), FINGER_OPEN, "down to its top edge", YAW_LONG),
        Waypoint(1.0, (sx, sy, gz), FINGER_CLOSED, "grip", YAW_LONG),
        Waypoint(1.5, (sx, sy, gz + lift), FINGER_CLOSED, "lift out of the stand", YAW_LONG),
        Waypoint(2.0, (sx, y_in, gz + lift), FINGER_CLOSED, "to the right", YAW_LONG),
        Waypoint(2.5, (rx, y_in, gz + above_lip), FINGER_CLOSED, "over the rack's open end", YAW_LONG),
        Waypoint(1.0, (rx, y_in, gz + above_floor), FINGER_CLOSED, "down to the floor", YAW_LONG),
        Waypoint(4.0, (rx, y_set, gz + above_floor), FINGER_CLOSED, "slide in behind the boards", YAW_LONG),
        Waypoint(1.0, (rx, y_set, z_floor), FINGER_CLOSED, "set it down", YAW_LONG),
        Waypoint(1.0, (rx, y_set, z_floor), RELEASE, "let go", YAW_LONG),
        Waypoint(1.0, (rx, y_set, z_floor + 0.06), RELEASE, "up", YAW_LONG),
        Waypoint(2.0, (0.15, -0.40, TOP + 0.25), RELEASE, "away", YAW_LONG),
    ]
    return {"left": left, "right": right}
