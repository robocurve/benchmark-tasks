"""On the station table (it stands for the pavement of a bike park): a child's bike with 12-inch wheels, parked
across the table facing left, its back wheel in front of the right arm, and a steel Sheffield cycle stand (48 mm
tube, 75 cm tall) between the robot and the bike, one of its legs just in front of the back tyre. The back wheel
turns on its axle. A U-lock lies in two parts on the table: its steel shackle (a U, 102 mm between the legs inside, 229 mm deep
inside) on the left, and its crossbar on the right.

The robot must lock the back wheel to the stand: the shackle has to go round both the stand's leg and the wheel's
rim (one leg of the U through the spokes, the other outside the tyre) and the crossbar has to be pushed onto both
leg ends until it locks.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py --scorer task_score ...
Demo:    python demo.py task.py demo_out
"""

from __future__ import annotations

import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
from lockkit import Hinge, env, finger_points, on_reset, on_step, part, set_weld, weld  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
SUCCESS_HOLD_S = 1.0
TOP = TABLE_TOP[SCENE]

INSTRUCTION = "Lock the bike's back wheel to the bike stand with the U-lock."

STEEL = (0.62, 0.63, 0.65)
GALV = (0.55, 0.57, 0.58)
RUBBER = (0.08, 0.08, 0.09)
FRAME = (0.80, 0.12, 0.10)
DARK = (0.15, 0.15, 0.16)
SHACKLE_C = (0.70, 0.71, 0.72)
COATING = (0.95, 0.75, 0.05)
CROSSBAR_C = (0.10, 0.10, 0.11)
YELLOW = (0.95, 0.80, 0.10)
SADDLE = (0.12, 0.12, 0.12)

# -- layout ---------------------------------------------------------------------------------------------------------
# The bike and the stand are built in a "park" frame: origin on the table under the back hub, x across the bike (away
# from the robot), y along the bike (toward its front), z up. The park frame sits on the table with its origin at
# PARK_XY and turned PARK_YAW about the vertical, so the bike runs away from the robot to the left at 25 degrees.
PARK_XY = (0.38, -0.01)
PARK_YAW = -math.radians(25)
PARK_ROT = np.array([[math.cos(PARK_YAW), -math.sin(PARK_YAW), 0.0], [math.sin(PARK_YAW), math.cos(PARK_YAW), 0.0],
                     [0.0, 0.0, 1.0]])
PARK_P0 = np.array([PARK_XY[0], PARK_XY[1], 0.0])


def park_to_world(p) -> np.ndarray:
    return PARK_P0 + PARK_ROT @ np.asarray(p, float)


def world_to_park(p) -> np.ndarray:
    return PARK_ROT.T @ (np.asarray(p, float) - PARK_P0)


XB = 0.0                  # park frame: the bike's wheels turn in the plane x = 0
HY = 0.0                  # back hub at y = 0; the bike's front is toward +y
H = 0.159                 # hub height above the table (12-inch tyre, 317 mm across)
WHEELBASE = 0.48
HUB = np.array([XB, HY, TOP + H])

# 12-inch wheel (ISO 203 rim, 57-203 tyre): tyre 317 mm across, rim 170-203 mm across, 16 radial spokes.
TYRE_RING, TYRE_R = 0.130, 0.0285      # tyre: a ring of capsules, outer radius 0.1585
RIM_RING, RIM_R = 0.093, 0.0085        # rim: radius 0.0845 .. 0.1015
RIM_DISC = 0.093                       # the rim's mid-line: what "round the rim" is measured against
SPOKES = 16
SPOKE_START = math.radians(11.25)      # spoke angles in the wheel plane, from +y toward +z
FLANGE_X, FLANGE_R = 0.022, 0.022

# Sheffield stand: 48 mm galvanised tube, legs 700 mm apart, top bar 726 mm above the ground (to the tube axis).
STAND_R = 0.024
XS = XB - 0.0285 - STAND_R - 0.008     # the stand's plane, 8 mm in front of the tyre
YL = HY - 0.142                        # its leg beside the back wheel
STAND_W, STAND_TOP = 0.70, 0.726

# U-lock, the size of a common standard U-lock (inside 102 x 229 mm, 12.7 mm hardened steel shackle, 1.32 kg).
# Shackle frame: origin at the centre of the bend, legs along +x at y = +-S, in the plane z = 0.
LEG_R = 0.00635                        # bare steel at the leg ends
COAT_R = 0.010                         # a vinyl sleeve, 20 mm across, over the bend and the first 70 mm of each leg
COAT_END = 0.070
S = 0.051 + LEG_R                      # leg axis from the U's mid-line
TIP = 0.198 - LEG_R                    # centre of each leg's rounded end (the end itself is at x 0.198)
SHACKLE_KG, CROSSBAR_KG = 0.62, 0.70
# Crossbar frame: origin midway between its two sockets, long axis y, sockets open toward -x.
CB_HALF = 0.015                        # the bar is 30 x 30 mm
HOLE = 0.0085                          # sockets 17 x 17 mm, 20 mm deep
MOUTH, BOTTOM = -CB_HALF, 0.005
CB_END = 0.08                          # the bar runs to y +-0.08; the lock head continues to y -0.13
HEAD = (-0.13, -0.08)
SEAT_X = 0.198 - BOTTOM                # crossbar origin in the shackle frame when fully seated

# Start poses on the table.
SHACKLE0 = (0.13, 0.17, TOP + COAT_R)  # lying flat, legs pointing away from the robot, parallel to the stand
SHACKLE0_YAW = PARK_YAW
CROSSBAR0 = (0.25, -0.30, TOP + CB_HALF)   # lying flat, sockets toward the robot, lock head to the right
CROSSBAR0_YAW = PARK_YAW                 # parallel to the bike


def _ring(centre, radius, r, n, x=0.0):
    """Capsules round a circle in the plane x = const (part frame)."""
    pts = [np.array([centre[0] + x, centre[1] + radius * math.cos(a), centre[2] + radius * math.sin(a)])
           for a in np.linspace(0, 2 * math.pi, n + 1)]
    return [(tuple(pts[i]), tuple(pts[i + 1]), r) for i in range(n)]


def _wheel(hub, spokes=True):
    caps = [(a, b, TYRE_R, RUBBER) for a, b, _ in _ring(hub, TYRE_RING, TYRE_R, 32)]
    caps += [(a, b, RIM_R, STEEL) for a, b, _ in _ring(hub, RIM_RING, RIM_R, 32)]
    if spokes:
        for k in range(SPOKES):
            a = SPOKE_START + k * 2 * math.pi / SPOKES
            d = np.array([0.0, math.cos(a), math.sin(a)])
            side = FLANGE_X if k % 2 else -FLANGE_X
            caps.append((tuple(hub + [side, 0, 0] + FLANGE_R * d), tuple(hub + (RIM_RING - RIM_R) * d), 0.001, STEEL))
    cyls = [(tuple(hub), 0.014, 0.10, DARK, "X"),
            (tuple(hub + [FLANGE_X, 0, 0]), FLANGE_R + 0.002, 0.004, STEEL, "X"),
            (tuple(hub - [FLANGE_X, 0, 0]), FLANGE_R + 0.002, 0.004, STEEL, "X")]
    return caps, cyls


# The bike faces +y (left); its back wheel, nearest the stand's leg, turns on its axle. The frame and front wheel are
# fixed: the bike stands on its kickstand and is not part of the job.
FRONT = HUB + [0.0, WHEELBASE, 0.0]
_crown = FRONT + [0.0, -0.055, 0.20]
_head_top = FRONT + [0.0, -0.095, 0.30]
_head_low = FRONT + [0.0, -0.065, 0.22]
_bb = np.array([XB, FRONT[1] - 0.28, TOP + 0.165])
_seat_top = FRONT + [0.0, -0.345, 0.27]
_bar_c = FRONT + [0.0, -0.115, 0.36]
_front_caps, _front_cyls = _wheel(FRONT)
BIKE_CAPS = [
    *[(tuple(FRONT + [s, 0, 0]), tuple(_crown + [s, 0, 0]), 0.010, FRAME) for s in (-0.045, 0.045)],  # fork blades
    (tuple(_crown + [-0.05, 0, 0]), tuple(_crown + [0.05, 0, 0]), 0.012, FRAME),                     # fork crown
    (tuple(_crown), tuple(_head_top), 0.018, FRAME),                                                  # head tube
    (tuple(_head_top), tuple(_bar_c), 0.011, DARK),                                                   # stem
    (tuple(_bar_c + [-0.20, 0, 0]), tuple(_bar_c + [0.20, 0, 0]), 0.011, DARK),                       # handlebar
    *[(tuple(_bar_c + [s * 0.15, 0, 0]), tuple(_bar_c + [s * 0.21, 0, 0]), 0.016, RUBBER) for s in (-1, 1)],
    (tuple(_head_low), tuple(_bb), 0.020, FRAME),                                                     # down tube
    (tuple(_bb), tuple(_seat_top), 0.016, FRAME),                                                     # seat tube
    (tuple(_head_top - [0, 0, 0.03]), tuple(_seat_top + [0, 0.01, -0.03]), 0.015, FRAME),             # top tube
    *[(tuple(_bb + [s, 0, 0]), tuple(HUB + [s, 0, 0]), 0.009, FRAME) for s in (-0.035, 0.035)],       # chainstays
    *[(tuple(_seat_top + [s * 0.3, 0, 0]), tuple(HUB + [s, 0, 0]), 0.008, FRAME) for s in (-0.035, 0.035)],
    (tuple(_seat_top), tuple(_seat_top + [0, -0.01, 0.07]), 0.012, DARK),                             # seat post
    (tuple(_bb + [-0.05, 0, 0]), tuple(_bb + [0.05, 0, 0]), 0.010, DARK),                             # axle of cranks
    (tuple(_bb + [-0.05, 0, 0]), tuple(_bb + [-0.05, 0.08, -0.06]), 0.008, DARK),                     # cranks
    (tuple(_bb + [0.05, 0, 0]), tuple(_bb + [0.05, -0.08, 0.06]), 0.008, DARK),
    (tuple(_bb + [0.04, -0.03, 0]), (XB + 0.16, _bb[1] - 0.09, TOP + 0.008), 0.007, DARK),            # kickstand
    *_front_caps,
]
BIKE_BOXES = [
    (tuple(_seat_top + [0, -0.02, 0.085]), (0.10, 0.20, 0.035), SADDLE),                              # saddle
    (tuple(_bb + [-0.05 - 0.04, 0.08, -0.06]), (0.08, 0.035, 0.02), DARK),                            # pedals
    (tuple(_bb + [0.05 + 0.04, -0.08, 0.06]), (0.08, 0.035, 0.02), DARK),
]
_back_caps, _back_cyls = _wheel(HUB)
_pt = lambda y, z: (XS, YL + y, TOP + z)  # noqa: E731
STAND_CAPS = [
    (_pt(0, 0.0), _pt(0, 0.62), STAND_R, GALV), (_pt(0, 0.62), _pt(0.04, 0.70), STAND_R, GALV),
    (_pt(0.04, 0.70), _pt(0.106, STAND_TOP), STAND_R, GALV),
    (_pt(0.106, STAND_TOP), _pt(STAND_W - 0.106, STAND_TOP), STAND_R, GALV),
    (_pt(STAND_W - 0.106, STAND_TOP), _pt(STAND_W - 0.04, 0.70), STAND_R, GALV),
    (_pt(STAND_W - 0.04, 0.70), _pt(STAND_W, 0.62), STAND_R, GALV), (_pt(STAND_W, 0.62), _pt(STAND_W, 0.0), STAND_R, GALV),
]
WHEEL = Hinge("back_wheel", anchor=tuple(HUB), axis="X", capsules=tuple(_back_caps), cylinders=tuple(_back_cyls),
              damping=0.02, mass=0.80)
PARK = part("bike_and_stand", pos=tuple(PARK_P0), yaw=PARK_YAW, boxes=BIKE_BOXES, cylinders=tuple(_front_cyls),
            capsules=(*BIKE_CAPS, *STAND_CAPS), hinges=(WHEEL,), color=FRAME, friction=0.8)


def _shackle_caps():
    caps = [((0.0, s, 0.0), (COAT_END, s, 0.0), COAT_R, COATING) for s in (S, -S)]
    caps += [((COAT_END - 0.005, s, 0.0), (TIP, s, 0.0), LEG_R, SHACKLE_C) for s in (S, -S)]
    angles = np.linspace(math.pi / 2, 3 * math.pi / 2, 9)
    pts = [(S * math.cos(a), S * math.sin(a), 0.0) for a in angles]
    caps += [(pts[i], pts[i + 1], COAT_R, COATING) for i in range(8)]
    return caps


def _crossbar_boxes():
    d = (MOUTH + BOTTOM) / 2, BOTTOM - MOUTH            # socket region along x: centre, depth
    edge_in, edge_out = S - HOLE, S + HOLE
    return [
        (((BOTTOM + CB_HALF) / 2, 0.0, 0.0), (CB_HALF - BOTTOM, 2 * CB_END, 2 * CB_HALF), CROSSBAR_C),       # back
        ((d[0], 0.0, (HOLE + CB_HALF) / 2), (d[1], 2 * CB_END, CB_HALF - HOLE), CROSSBAR_C),                 # top
        ((d[0], 0.0, -(HOLE + CB_HALF) / 2), (d[1], 2 * CB_END, CB_HALF - HOLE), CROSSBAR_C),                # bottom
        ((d[0], 0.0, 0.0), (d[1], 2 * edge_in, 2 * HOLE), CROSSBAR_C),                                       # middle
        *(((d[0], s * (edge_out + CB_END) / 2, 0.0), (d[1], CB_END - edge_out, 2 * HOLE), CROSSBAR_C) for s in (1, -1)),
    ]


SHACKLE = part("shackle", pos=SHACKLE0, yaw=SHACKLE0_YAW, mass=SHACKLE_KG, capsules=tuple(_shackle_caps()),
               color=SHACKLE_C, friction=1.0, combine="max")
CROSSBAR = part("crossbar", pos=CROSSBAR0, yaw=CROSSBAR0_YAW, mass=CROSSBAR_KG, boxes=tuple(_crossbar_boxes()),
                cylinders=(((0.0, sum(HEAD) / 2, 0.0), CB_HALF, HEAD[1] - HEAD[0], CROSSBAR_C, "Y"),
                           ((0.0, HEAD[0] - 0.001, 0.0), 0.009, 0.002, YELLOW, "Y")),
                color=CROSSBAR_C, friction=1.0, combine="max")
OBJECTS = [PARK, SHACKLE, CROSSBAR]

# The crossbar fits either way round (lock head on either side); one weld per way, switched on when it locks.
weld("seated", CROSSBAR, SHACKLE, (SEAT_X, 0.0, 0.0), (1.0, 0.0, 0.0, 0.0))
weld("seated_flipped", CROSSBAR, SHACKLE, (SEAT_X, 0.0, 0.0), (0.0, 1.0, 0.0, 0.0))

# -- the latch ----------------------------------------------------------------------------------------------------------
# A real U-lock's crossbar catches notches in both leg ends when pushed home. Here it locks when both leg ends are in
# its sockets within 4 mm of the socket bottoms (16 mm in); from then on a fixed joint holds it in the seated pose.
LATCH_DEPTH = 0.004
LATCH_SIDE = 0.004
_S: dict = {}


@on_reset
def _reset(environment=None) -> None:
    for name in ("seated", "seated_flipped"):
        try:
            set_weld(name, False)
        except Exception:  # noqa: BLE001 - before the stage exists
            pass
    _S.update(step=0, scored=None, last=(0.0, False), latched=None, released=False, finger_check=None, recent=[])


_reset()


def _tips_in_crossbar(shackle, crossbar) -> np.ndarray:
    return np.array([crossbar.to_local(shackle.to_world((0.198, s, 0.0))) for s in (S, -S)])


def _seated(shackle, crossbar):
    """'seated' / 'seated_flipped' when both leg ends sit in the sockets (16 mm in or more), else None."""
    tips = _tips_in_crossbar(shackle, crossbar)
    for name, order in (("seated", (S, -S)), ("seated_flipped", (-S, S))):
        if all(t[0] > BOTTOM - LATCH_DEPTH and abs(t[1] - y) < LATCH_SIDE and abs(t[2]) < LATCH_SIDE
               for t, y in zip(tips, order)):
            return name
    return None


@on_step
def _latch(environment) -> None:
    from yam_arena.scene import object_states

    _S["step"] += 1
    if _S["latched"] is None:
        st = object_states(environment, ["shackle", "crossbar"])
        name = _seated(st["shackle"], st["crossbar"])
        if name:
            _S["latched"] = name
            set_weld(name, True)


# -- hands off ----------------------------------------------------------------------------------------------------------
CLEAR_M = 0.03


def _seg_dist(p, a, b) -> float:
    ab = b - a
    t = float(np.clip(np.dot(p - a, ab) / max(np.dot(ab, ab), 1e-12), 0.0, 1.0))
    return float(np.linalg.norm(p - (a + t * ab)))


def _lock_dist(p, shackle, crossbar) -> float:
    loc = shackle.to_local(p)
    d = min(_seg_dist(loc, np.array(a), np.array(b)) - r for a, b, r, _ in _shackle_caps())
    c = crossbar.to_local(p)
    lo, hi = np.array([-CB_HALF, HEAD[0], -CB_HALF]), np.array([CB_HALF, CB_END, CB_HALF])
    d2 = float(np.linalg.norm(np.maximum(np.maximum(lo - c, c - hi), 0.0)))
    return min(d, d2)


@on_step
def _hands(environment) -> None:
    from yam_arena.scene import object_states

    try:
        st = object_states(environment, ["shackle", "crossbar"])
        pts = finger_points(environment)
        _S["released"] = bool(min(_lock_dist(p, st["shackle"], st["crossbar"]) for p in pts) >= CLEAR_M)
        _S["finger_check"] = True
    except Exception as exc:  # noqa: BLE001 - report once; an unknown grip never counts as released
        if _S["finger_check"] is None:
            print(f"u-lock: finger check unavailable: {exc!r}", flush=True)
        _S["finger_check"], _S["released"] = False, False


# -- what the lock goes round -------------------------------------------------------------------------------------------
def lock_loop(shackle) -> np.ndarray:
    """The lock's closed loop (world): the shackle's centre line and the straight line across its leg ends, which the
    crossbar covers once it is on."""
    angles = np.linspace(math.pi / 2, 3 * math.pi / 2, 17)
    local = [(TIP, S, 0.0), *((S * math.cos(a), S * math.sin(a), 0.0) for a in angles), (TIP, -S, 0.0)]
    return np.array([shackle.to_world(p) for p in local])


def _crossings(loop, axis_x, inside) -> int:
    """Signed number of times the closed polyline crosses the plane x = axis_x at a point where inside(y, z)."""
    n = 0
    for i in range(len(loop)):
        a, b = loop[i], loop[(i + 1) % len(loop)]
        da, db = a[0] - axis_x, b[0] - axis_x
        if (da < 0) == (db < 0) or da == db:
            continue
        p = a + (b - a) * (da / (da - db))
        if inside(p[1], p[2]):
            n += 1 if db > da else -1
    return n


def round_rim(loop) -> bool:
    """The loop links the rim: it passes through the wheel's plane inside the rim a net odd number of times."""
    loop = np.array([world_to_park(p) for p in loop])
    return _crossings(loop, XB, lambda y, z: math.hypot(y - HUB[1], z - HUB[2]) < RIM_DISC) % 2 == 1


def round_stand(loop) -> bool:
    """The loop links the stand (its two legs, its top bar and the ground): it passes through the stand's opening."""
    loop = np.array([world_to_park(p) for p in loop])
    return _crossings(loop, XS, lambda y, z: YL < y < YL + STAND_W and TOP < z < TOP + STAND_TOP) % 2 == 1


def status(objects) -> dict:
    sh, cb = objects["shackle"], objects["crossbar"]
    loop = lock_loop(sh)
    recent = _S["recent"]
    recent.append((sh.pos.copy(), cb.pos.copy(), _S["released"]))
    del recent[:-20]
    still = len(recent) == 20 and all(np.linalg.norm(p - sh.pos) < 0.002 and np.linalg.norm(q - cb.pos) < 0.002
                                      and free for p, q, free in recent)
    return dict(latched=_S["latched"] is not None, rim=round_rim(loop), stand=round_stand(loop),
                released=_S["released"], still=still)


def _update(objects) -> tuple[float, bool]:
    # success() and score() both run every control step; evaluate each step once.
    if _S["scored"] == _S["step"]:
        return _S["last"]
    _S["scored"] = _S["step"]
    st = status(objects)
    both = st["rim"] and st["stand"]
    if st["latched"] and both and st["still"]:
        result = (1.0, True)
    elif st["latched"] and both:
        result = (0.5, False)
    elif both:
        result = (0.25, False)
    else:
        result = (0.0, False)
    _S["last"] = result
    return result


def success(objects) -> bool:
    """Locked round both the stand and the rim, both grippers 3 cm clear of the lock and the lock still over the last
    second; held 1 s."""
    return _update(objects)[1]


def score(objects) -> float:
    """1.0 locked round both the stand and the rim, let go and still; 0.5 locked round both but still held or
    moving (a lock round only one of them earns nothing); 0.25 the shackle (closed by the line across its leg ends)
    round both but the crossbar not locked on; else 0."""
    return _update(objects)[0]


assert env


def demo():
    """No waypoint plan here: the full-score demo is demo.py (it needs tilted grips that yam_scripted's straight-down
    waypoints cannot give). An empty plan makes ``--policy yam_scripted`` hold both arms still: the do-nothing check."""
    return {}
