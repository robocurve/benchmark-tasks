"""The scorer on hand-built trials, with no simulator: python3 check_score.py.

Loads task.py with stand-ins for the harness's modules, then feeds its real Trial step by step with made-up states (the honest placing and the lazy
routes) and checks each score against the scoring text. Nothing here runs a simulation.
"""

from __future__ import annotations

import dataclasses
import importlib.util
import math
import os
import pathlib
import sys
import types

import numpy as np

HERE = pathlib.Path(__file__).resolve().parent
os.environ["CHAIR_TRACE"] = "0"

ya = types.ModuleType("yam_arena")
tf = types.ModuleType("yam_arena.task_file")


@dataclasses.dataclass(frozen=True)
class Box:
    name: str
    pos: tuple = (0.0, 0.0, 0.0)
    size: tuple = (0.1, 0.1, 0.1)
    mass: float | None = None
    yaw_deg: float = 0.0
    start_quat: tuple | None = None
    solver_iters: int = 32


@dataclasses.dataclass(frozen=True)
class ObjectState:
    pos: np.ndarray
    quat_xyzw: np.ndarray
    lin_vel: np.ndarray
    ang_vel: np.ndarray
    low: np.ndarray
    high: np.ndarray


tf.Box, tf.ObjectState, tf.TABLE_TOP = Box, ObjectState, {"station": 0.7475}
ya.task_file = tf
sys.modules["yam_arena"], sys.modules["yam_arena.task_file"] = ya, tf
dk = types.ModuleType("demokit")
dk.TABLE_TOP, dk.DT, dk.CONTROL_HZ = 0.7475, 0.05, 20
sys.modules["demokit"] = dk
sys.path.insert(0, str(HERE))
import gatekit as gk  # noqa: E402
import mechkit as m  # noqa: E402

spec = importlib.util.spec_from_file_location("task", HERE / "task.py")
T = importlib.util.module_from_spec(spec)
spec.loader.exec_module(T)
TOP, SURF = T.TOP, T.SURF

RZ180 = np.diag([-1.0, -1.0, 1.0])          # the chair at the start: standing, facing the arms
FLIPPED = np.diag([1.0, -1.0, -1.0])        # turned over about world y: seat down, legs up, its back toward the arms
X_LAND = T.TX - 0.018 + T.SEAT_X / 2        # the seat centre's x with its rear edge 18 mm past the table's near edge


def Ry(deg):
    a = math.radians(deg)
    return np.array([[math.cos(a), 0, math.sin(a)], [0, 1, 0], [-math.sin(a), 0, math.cos(a)]])


def chair(pos, R, v=(0, 0, 0)):
    return gk.Pose(pos, R, lin_vel=v)


def at(P, R, v=(0, 0, 0)):
    """The chair whose seat plate's middle point is P in the world, turned by R."""
    return chair(np.asarray(P, float) - R @ np.array([0.0, 0.0, T.SEAT_Z - T.SEAT_T / 2]), R, v)


def grips(c, opening=T.SEAT_T / 2):
    """Both hands pinching the seat plate at its side edges, fingers pointing in, jaws across the plate."""
    out = []
    for sy in (1.0, -1.0):
        mid = c.to_world([[0.0, sy * (T.SEAT_Y / 2 - T.IN), T.SEAT_Z - T.SEAT_T / 2]])[0]
        z, fing = c.rotation() @ [0, 0, 1.0], c.rotation() @ [0, -sy, 0.0]
        R = np.column_stack([z, np.cross(fing, z), fing])
        out.append(gk.hand(mid, R, opening=opening))
    return gk.tips(out[0], out[1])


PARK = gk.tips()


def st(c, tips):
    return {"chair": c, "tips": tips}


P0 = np.array([T.X0, 0.0, TOP + 0.002 + T.SEAT_Z - T.SEAT_T / 2])


def lift_phase(n=10):
    start = at(P0, RZ180)
    s = [st(start, PARK)] * 5 + [st(start, grips(start))] * 3
    for k in range(n):  # up 25 cm held
        c = at(P0 + [0, 0, 0.25 * (k + 1) / n], RZ180)
        s.append(st(c, grips(c)))
    return s


def turn_phase(n=10):
    s = []
    for k in range(n):  # turned over about world y through the seat plate's middle, held
        c = at(P0 + [0, 0, 0.25], Ry(180 * (k + 1) / n) @ RZ180)
        s.append(st(c, grips(c)))
    return s


def carry_phase(x_end, n=8, h=None):
    """Held, seat down: across at height, then down until the seat's face is h over the top."""
    s = []
    top_z = SURF + T.SEAT_T / 2 + (T.H_STOP if h is None else h)
    for k in range(n):
        c = at([T.X0 + (x_end - T.X0) * (k + 1) / n, 0.0, P0[2] + 0.25], FLIPPED)
        s.append(st(c, grips(c)))
    for k in range(n):
        c = at([x_end, 0.0, P0[2] + 0.25 + (top_z - P0[2] - 0.25) * (k + 1) / n], FLIPPED)
        s.append(st(c, grips(c)))
    return s


PHYS_DT = 1.0 / 240                         # the harness's physics step


def falling(x, y, R, z_from, z_to, v0=0.0):
    """Free fall of the chair's seat-plate middle from z_from to z_to, one state per physics step, hands away."""
    out, z, v = [], z_from, v0
    while z > z_to:
        v += 9.81 * PHYS_DT
        z = max(z - v * PHYS_DT, z_to)
        out.append(st(at([x, y, z], R, v=(0, 0, -v if z > z_to else 0.0)), PARK))
    return out


def honest(x_end=X_LAND, hold=50, drop_h=None, tilt=0.0, tail="clear", y=0.0, held_down=False, v_land=0.0):
    """Lift, turn, carry, set down until the seat's face is drop_h over the top (default H_STOP, on the lower fingers), open, slide the
    fingers out so it settles onto the top. held_down: lowered while held all the way onto the top, arriving at v_land."""
    h = T.H_STOP if drop_h is None else drop_h
    s = lift_phase() + turn_phase() + carry_phase(x_end, h=h if not held_down else 0.02)
    R = Ry(tilt) @ FLIPPED
    z_down = SURF + T.SEAT_T / 2 + math.sin(math.radians(abs(tilt))) * T.SEAT_X / 2      # tilted: resting on one edge
    down = at([x_end, y, z_down], R)
    if held_down:
        a = at([x_end, y, z_down + 0.004], R, v=(0, 0, -v_land))
        s += [st(a, grips(a))] * 2 + [st(down, grips(down))] * 3 + [st(down, grips(down, opening=0.02))] * 2
    else:
        held = at([x_end, y, z_down + h], R)
        s += [st(held, grips(held))] * 3 + [st(held, grips(held, opening=0.02))] * 2       # opened: resting on the lower fingers
        s += falling(x_end, y, R, z_down + h, z_down)                                     # fingers slid out: it settles h
    after = [st(down, PARK)] if tail == "clear" else [st(down, grips(down, opening=0.02))] if tail == "near" else [st(down, grips(down))]
    return s + after * hold


def run(states) -> float:
    T.TRIAL.reset()
    cur = {}
    m.state, m.fingertips = (lambda env, name: cur[name]), (lambda env: cur["tips"])
    T.arm_points = lambda env: cur.get("arm", NO_ARM)
    for s in states:
        cur = {**cur, "arm": NO_ARM, **s}
        T.TRIAL.step(None)
    return T.score(None)


NO_ARM = np.zeros((0, 3))                   # the arm links: far from the chair unless a case puts them on it


def fall(x, z0):
    """The chair, seat down, let go with its seat plate's middle z0 and falling onto the top."""
    return falling(x, 0.0, FLIPPED, z0, SURF + T.SEAT_T / 2)


start = at(P0, RZ180)
side = chair([T.X0, 0.0, TOP + T.SEAT_Y / 2], Ry(90) @ RZ180)


def one_hand_lift():
    s = [st(start, PARK)] * 5
    for k in range(10):
        c = at(P0 + [0, 0, 0.25 * (k + 1) / 10], RZ180)
        g = grips(c)
        s.append(st(c, gk.tips(g[:12], gk.hand(gk.PARKED["right"], opening=0.03))))
    return s + [st(c, PARK)] * 20


def flat(x, y=0.0):
    return at([x, y, SURF + T.SEAT_T / 2], FLIPPED)


floor_flat = at([0.0, 0.0, TOP + 0.235], FLIPPED)  # origin: seat face down on the floor... legs up, back toward the arms
dropped_on_top = lift_phase() + turn_phase() + carry_phase(X_LAND)[:4] + fall(X_LAND, SURF + 0.20) + [st(flat(X_LAND), PARK)] * 50
set_down_floor_upright = lift_phase() + [st(at(P0, RZ180), grips(at(P0, RZ180)))] * 4 + [st(at(P0, RZ180), PARK)] * 50
turned_on_floor = lift_phase() + turn_phase() + [st(floor_flat, grips(floor_flat))] * 3 + [st(floor_flat, PARK)] * 50
upright_on_top = lift_phase() + [st(chair([T.TX + 0.15, 0.0, SURF + 0.001], RZ180), PARK)] * 50
dragged = honest()[:-50] + [st(flat(X_LAND + 0.01 * k), PARK) for k in range(1, 10)] * 6
legs_down_on_top = lift_phase() + turn_phase()[:5] + [st(at([T.TX + 0.25, 0.0, SURF + 0.01], Ry(120) @ RZ180), PARK)] * 30
feet_on_top_held = lift_phase() + [st(at([T.TX + 0.15, 0.0, SURF + 0.002 + T.SEAT_Z - T.SEAT_T / 2], RZ180), grips(at([T.TX + 0.15, 0.0, SURF + 0.002 + T.SEAT_Z - T.SEAT_T / 2], RZ180)))] * 5
leg_mid = 0.5 * (T.FEET[0] + np.array([T.SEAT_X / 2 - 0.02, T.SEAT_Y / 2 - 0.02, T.SEAT_Z - T.SEAT_T]))   # a leg's middle, chair frame
_c0 = at(P0 + [0, 0, 0.25], RZ180)
_lm = _c0.to_world(leg_mid[None, :])[0]
_cs = at(P0 + [0, 0, 0.25] + [T.TX - _lm[0] + 0.004, 0.0, (T.SURF - 0.002) - _lm[2]], RZ180)
shaft_on_edge = lift_phase() + [st(_cs, grips(_cs))] * 4
_r60 = Ry(60) @ RZ180
_c60 = at(P0 + [0, 0, 0.25], _r60)
_low = _c60.to_world(np.array([[x * T.SEAT_X, y * T.SEAT_Y, T.SEAT_Z] for x in np.linspace(-0.5, 0.5, 9) for y in np.linspace(-0.5, 0.5, 9)]))   # the seat's face
_pt = _low[np.argmin(_low[:, 2])]
_ce = at(P0 + [0, 0, 0.25] + [T.TX + 0.05 - _pt[0], 0.0, T.SURF + 0.001 - _pt[2]], _r60)
seat_edge_on_top = lift_phase() + turn_phase()[:3] + [st(_ce, grips(_ce))] * 4
_up = at([X_LAND, 0.0, SURF + T.SEAT_T / 2 + 0.10], FLIPPED)
slides_off = honest(hold=10) + [st(at([X_LAND - 0.10 - 0.05 * k, 0.0, max(TOP + 0.01, SURF + T.SEAT_T / 2 - 0.06 * k)], FLIPPED, v=(0, 0, -1.5)), PARK) for k in range(1, 12)] * 2

# a retry: set down on the seat's far edge 5 cm to the left of the landing, lifted 3 cm clear, moved back and set down as the honest route does
_n_pre = len(lift_phase() + turn_phase() + carry_phase(X_LAND))
_tail = honest()[_n_pre:]
_z = lambda h: SURF + T.SEAT_T / 2 + h
retry = lift_phase() + turn_phase() + carry_phase(X_LAND, h=0.03)
retry += [st(c, grips(c)) for c in [at([X_LAND, 0.05 * (k + 1) / 6, _z(0.03)], FLIPPED) for k in range(6)]]
retry += [st(c, grips(c)) for c in [at([X_LAND, 0.05, _z(0.03 * (1 - (k + 1) / 6))], FLIPPED) for k in range(6)]]
retry += [st(at([X_LAND, 0.05, _z(0.0)], FLIPPED), grips(at([X_LAND, 0.05, _z(0.0)], FLIPPED)))] * 5
retry += [st(c, grips(c)) for c in [at([X_LAND, 0.05, _z(0.03 * (k + 1) / 6)], FLIPPED) for k in range(6)]]
retry += [st(c, grips(c)) for c in [at([X_LAND, 0.05 * (1 - (k + 1) / 8), _z(0.03)], FLIPPED) for k in range(8)]]
retry += [st(c, grips(c)) for c in [at([X_LAND, 0.0, _z(0.03 - (0.03 - T.H_STOP) * (k + 1) / 8)], FLIPPED) for k in range(8)]]
retry += _tail
# the same first touch, but the chair pushed back along the top to the landing (no lift): a drag
redrag = retry[:len(lift_phase() + turn_phase() + carry_phase(X_LAND, h=0.03)) + 17]
redrag += [st(c, grips(c)) for c in [at([X_LAND, 0.05 * (1 - (k + 1) / 8), _z(0.0)], FLIPPED) for k in range(8)]] + _tail
# a hand resting on the upturned seat's underside, its fingertips on it at a quadrant's centre, fingers pointing down; the other hand away
_down = honest()[-1]["chair"]
_q = _down.to_world([[T.SEAT_X / 4, T.SEAT_Y / 4, T.SEAT_Z - T.SEAT_T]])[0]
_rest = gk.tips(gk.hand(_q + [0, 0, 0.001], np.column_stack([[0, 1.0, 0], [1.0, 0, 0], [0, 0, -1.0]]), opening=0.0))
palm_rest = honest()[:-50] + [st(_down, _rest)] * 50
# a forearm resting on a rear leg's middle at the end, the fingertips away
_leg = _down.to_world(np.array([0.5 * (T.LEG_ENDS[0][0] + T.LEG_ENDS[0][1])]))[0]
_fore = np.array([_leg + [-0.03, 0.0, 0.0] + [-0.2 * u, 0.0, 0.03 * u] for u in np.linspace(0, 1, 6)]) + [0.0, 0.0, T.LEG_R + 0.03]
link_rest = honest()[:-50] + [st(_down, PARK) | {"arm": _fore}] * 50
# both hands lift and turn it, set it seat down on the floor and let go; then the left hand alone lifts it 30 cm and carries it to the table
_one = lambda c: gk.tips(grips(c)[:12], gk.hand(gk.PARKED["right"], opening=0.03))
_mid0 = floor_flat.to_world([[0, 0, T.SEAT_Z - T.SEAT_T / 2]])[0]
onehand_carry = lift_phase() + turn_phase() + [st(floor_flat, grips(floor_flat))] * 3 + [st(floor_flat, PARK)] * 10
onehand_carry += [st(c, _one(c)) for c in [at(_mid0 + [0, 0, 0.30 * (k + 1) / 8], FLIPPED) for k in range(8)]]
onehand_carry += [st(c, _one(c)) for c in [at([_mid0[0] + (X_LAND - _mid0[0]) * (k + 1) / 8, 0, _mid0[2] + 0.30], FLIPPED) for k in range(8)]]
onehand_carry += [st(c, _one(c)) for c in [at([X_LAND, 0, _mid0[2] + 0.30 + (_z(T.H_STOP) - _mid0[2] - 0.30) * (k + 1) / 8], FLIPPED) for k in range(8)]]
onehand_carry += _tail
# the left hand alone tips it over on the floor (seat turning from up to down with its lowest point 2 mm over the floor), then both hands lift it
# upside down and carry it to the table as in the honest run
_tip = []
for k in range(1, 11):
    c0 = at(P0, Ry(180 * k / 10) @ RZ180)
    _fl = T.gaps(c0)[2].min()
    _tip.append(st(at(P0 + [0, 0, 0.002 - _fl], Ry(180 * k / 10) @ RZ180), gk.tips(grips(c0)[:12], gk.hand(gk.PARKED["right"], opening=0.03))))
floorflip = [st(start, PARK)] * 5 + _tip + [st(floor_flat, grips(floor_flat))] * 3
floorflip += [st(c, grips(c)) for c in [at(_mid0 + [0, 0, 0.30 * (k + 1) / 8], FLIPPED) for k in range(8)]]
floorflip += [st(c, grips(c)) for c in [at([_mid0[0] + (X_LAND - _mid0[0]) * (k + 1) / 8, 0, _mid0[2] + 0.30], FLIPPED) for k in range(8)]]
floorflip += [st(c, grips(c)) for c in [at([X_LAND, 0, _mid0[2] + 0.30 + (_z(T.H_STOP) - _mid0[2] - 0.30) * (k + 1) / 8], FLIPPED) for k in range(8)]]
floorflip += _tail
# both hands lift it; the right lets go (a long way before the turn) and the left alone carries it upright 20 cm sideways; both take it again for the turn
_lf = lift_phase()[-1]["chair"]
_side = [st(c, gk.tips(grips(c)[:12], gk.hand(gk.PARKED["right"], opening=0.03))) for c in [at(P0 + [0, 0.20 * (k + 1) / 8, 0.25], RZ180) for k in range(8)]]
_c1 = _side[-1]["chair"]
_h2 = honest(y=0.0)
onehand_carry_upright = lift_phase() + _side * 2 + [st(_c1, grips(_c1))] * 3 + [st(at(P0 + [0, 0.20, 0.25], Ry(180 * (k + 1) / 10) @ RZ180), grips(at(P0 + [0, 0.20, 0.25], Ry(180 * (k + 1) / 10) @ RZ180))) for k in range(10)] + _tail
# half way through the turn (seat 90 degrees from down), the back's edge swung onto the table's near face 15 mm under the top, held
_r90 = Ry(90) @ RZ180
_c = at(P0 + [0, 0, 0.25], _r90)
_bw = _c.to_world(T.BACK_PTS)
_k = int(np.argmax(_bw[:, 0]))
_cb = at(P0 + [0, 0, 0.25] + [T.TX - T.BACK_T / 2 + 0.001 - _bw[_k, 0], 0.0, SURF - 0.015 - _bw[_k, 2]], _r90)
back_on_face_turning = lift_phase() + turn_phase()[:5] + [st(_cb, grips(_cb))] * 4
# seat down 3 cm over the top, the back swung into the near face at 0.8 m/s, held
_xb = T.TX + T.SEAT_X / 2
back_knock = lift_phase() + turn_phase() + carry_phase(_xb - 0.03, h=0.03)
back_knock += [st(c, grips(c)) for c in [at([_xb - dx, 0.0, _z(0.03)], FLIPPED, v=(0.8, 0, 0)) for dx in (0.02, 0.0)]] + [st(at([_xb, 0.0, _z(0.03)], FLIPPED), grips(at([_xb, 0.0, _z(0.03)], FLIPPED)))] * 5

# ghost hands: the right hand closes its jaws to the plate's width but stands beside the chair, a few mm to 2 cm off it, never touching; the left alone does the act
def _ghost(c, kind):
    if kind == "seat":     # fingertips 10 mm outside the seat's right side edge, jaws straddling the plate's plane
        mid = c.to_world([[0.0, -(T.SEAT_Y / 2 + 0.010), T.SEAT_Z - T.SEAT_T / 2]])[0]
        z, fing = c.rotation() @ [0, 0, 1.0], c.rotation() @ [0, 1.0, 0.0]
        return gk.hand(mid, np.column_stack([z, np.cross(fing, z), fing]), opening=T.SEAT_T / 2)
    top, foot = T.LEG_ENDS[0 if T.LEG_ENDS[0][0][1] < 0 else 1]    # a rear leg on the right: jaws close along the chair's x, 2 cm to its side
    mid = c.to_world([0.5 * (top + foot) + [0.0, -0.02, 0.0]])[0]
    ex, ey, ez = (c.rotation() @ np.eye(3)).T
    return gk.hand(mid, np.column_stack([ex, ez, ey]), opening=T.LEG_R)


def ghosted(kind):
    out = []
    for x in honest():
        t = np.array(x["tips"], float)
        if not np.allclose(t[12:], PARK[12:]) and abs(gk.jaw_gap(t, "right") - 0.06) > 0.0:   # a state where the right hand is at the chair
            t = gk.tips(t[:12], _ghost(x["chair"], kind))
        out.append(st(x["chair"], t))
    return out


ghostseat, ghostleg = ghosted("seat"), ghosted("leg")

# held on its side, a rear leg running across the top's near edge rising 20 degrees, its shaft 1 mm off the edge at a point half way between
# two points 2.2 cm apart along it (an earlier, coarser sampling missed it at 3.6 mm)
def _rot_to(a, b):
    a, b = a / np.linalg.norm(a), b / np.linalg.norm(b)
    v, c = np.cross(a, b), float(a @ b)
    K = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    return np.eye(3) + K + K @ K / (1 + c)


_top, _foot = T.LEG_ENDS[1]
_al = math.radians(20.0)
_dir = np.array([math.cos(_al), 0.0, math.sin(_al)])
_K = np.array([[0, -_dir[2], _dir[1]], [_dir[2], 0, -_dir[0]], [-_dir[1], _dir[0], 0]])
_Rg = (np.eye(3) - _K + _K @ _K) @ _rot_to(_foot - _top, _dir)          # then 270 degrees about the leg, so the rest of the chair is clear
_pl = _foot + (_top - _foot) * (6.5 / 12 * 0.93)
_cg = chair(np.array([T.TX, 0.0, SURF]) + np.array([-math.sin(_al), 0.0, math.cos(_al)]) * (T.LEG_R + 0.001) - _Rg @ _pl, _Rg)
shaft_graze = lift_phase() + [st(_cg, grips(_cg))] * 4

# lifted by both hands, then the right hand lets go for 0.5 s half way through the turn and takes the seat again; then the honest placing
_h = honest()
_nl, _nt = len(lift_phase()), len(turn_phase())
_mid = _h[_nl + 5]["chair"]
regrip_air = _h[:_nl + 5] + [st(_mid, gk.tips(grips(_mid)[:12], gk.hand(gk.PARKED["right"], opening=0.03)))] * 10 + _h[_nl + 5:]
# carried upright, a rear foot 2 mm from the side of the table's near left leg, held
_tl_lo, _tl_hi = T._TLEG[2] if T._TLEG[2][0][1] > 0 else T._TLEG[0]
_cl0 = at(P0 + [0, 0, 0.10], RZ180)
_fw = _cl0.to_world(T.FEET)
_kf = int(np.argmax(_fw[:, 0] - 0 * _fw[:, 1]))
_tl = min(T._TLEG, key=lambda b: abs(b[0][0] - T.TX) + abs(b[0][1] - 0.3))
_target = np.array([_tl[0][0] - T.LEG_R - 0.002, 0.5 * (_tl[0][1] + _tl[1][1]), TOP + 0.10])
_ct = at(P0 + [0, 0, 0.10] + (_target - _fw[_kf]), RZ180)
leg_on_table_leg = lift_phase(4) + [st(_ct, grips(_ct))] * 4


def honest_yaw(deg, x_end=X_LAND):
    """The honest placing, but the chair lands turned about the vertical through its seat face's middle."""
    Rz = np.array([[math.cos(math.radians(deg)), -math.sin(math.radians(deg)), 0], [math.sin(math.radians(deg)), math.cos(math.radians(deg)), 0], [0, 0, 1.0]])
    s = honest(x_end=x_end)
    out = []
    for x in s[-50:]:
        c = x["chair"]
        mid = c.to_world([[0.0, 0.0, T.SEAT_Z]])[0]
        R = Rz @ c.rotation()
        out.append(st(chair(mid - R @ np.array([0.0, 0.0, T.SEAT_Z]), R), x["tips"]))
    return s[:-50] + out


CASES = {
    "the honest placing: lifted, turned over, seat down on the top, rear edge 18 mm past the edge, let go, hands clear, still 2 s": (honest(), 1.0),
    "the same, but the seat settles 11.9 mm (a thicker finger)": (honest(drop_h=0.0119), 1.0),
    "the same, set down while held all the way onto the top at 1 cm/s, then let go": (honest(held_down=True, v_land=0.01), 1.0),
    "the same, set down while held but arriving at 0.8 m/s": (honest(held_down=True, v_land=0.8), 0.0),
    "the same, let go 15 mm over the top (falls at 0.54 m/s)": (honest(drop_h=0.015), 0.0),
    "the same, let go 4 cm over the top (falls at 0.89 m/s)": (honest(drop_h=0.04), 0.0),
    "the same, let go 12 cm over the top (falls at 1.5 m/s)": (honest(drop_h=0.12), 0.0),
    "the same, rear edge 25 mm inside the edge (the back would stand on the top)": (honest(x_end=T.TX + 0.025 + T.SEAT_X / 2), 0.0),
    "the same, rear edge 25 mm past the edge": (honest(x_end=T.TX - 0.025 + T.SEAT_X / 2), 1.0),
    "the same, rear edge 45 mm past the edge": (honest(x_end=T.TX - 0.045 + T.SEAT_X / 2), 0.75),
    "seat tilted 8 degrees on the top": (honest(tilt=8.0), 0.5),
    "seat tilted 20 degrees on the top": (honest(tilt=20.0), 0.5),
    "seat tilted 3 degrees on the top": (honest(tilt=3.0), 1.0),
    "seat tilted 4.5 degrees on the top, its far side up": (honest(tilt=-4.5), 1.0),
    "a hand left 2 cm off the seat edge at the end": (honest(tail="near"), 0.75),
    "a hand still pinching the seat at the end": (honest(tail="pinch"), 0.5),
    "the end still held only 1 s": (honest(hold=26), 0.75),
    "lifted and turned, still held seat down 25 cm up at the end": (lift_phase() + turn_phase() + turn_phase()[-1:] * 20, 0.5),
    "lifted, still held upright 25 cm up at the end": (lift_phase() + lift_phase()[-1:] * 20, 0.25),
    "placed and held 2 s, then lifted straight up 10 cm and held there at the end": (honest() + [st(_up, grips(_up))] * 20, 0.5),
    "nothing done": ([st(start, PARK)] * 60, 0.0),
    "the chair left standing on the floor, hands clear": ([st(start, PARK)] * 60, 0.0),
    "the chair knocked onto its side on the floor": ([st(start, PARK)] * 5 + [st(side, PARK)] * 50, 0.0),
    "one hand lifts it 25 cm, then it is put down": (one_hand_lift(), 0.0),
    "lifted by both hands, set back down upright on the floor": (set_down_floor_upright, 0.0),
    "lifted, turned over, set down upside down on the floor beside the table": (turned_on_floor, 0.0),
    "lifted and put upright on the top, never turned": (upright_on_top, 0.25),
    "turned over and dropped from 20 cm onto the top": (dropped_on_top, 0.0),
    "placed, then the seat dragged 9 cm along the top": (dragged, 0.0),
    "placed, then pushed off the table's edge to the floor": (slides_off, 0.0),
    "turned, then put down on its side on the top, released and still": (legs_down_on_top, 0.25),
    "carried upright over the top and lowered until its feet touch it, held": (feet_on_top_held, 0.0),
    "carried upright, a leg's shaft (its middle, not its foot) touching the top's near edge": (shaft_on_edge, 0.0),
    "turned 60 degrees, the seat's edge brushing the top while held": (seat_edge_on_top, 0.0),
    "held on its side, a leg's shaft grazing the top's near edge 1 mm off, between two points 2.2 cm apart along it": (shaft_graze, 0.0),
    "a retry: set down 5 cm to the side, lifted 3 cm clear, moved back and set down again": (retry, 1.0),
    "the same first touch, then pushed 5 cm back along the top without lifting": (redrag, 0.0),
    "a hand left resting on the upturned seat's underside at a quadrant's centre": (palm_rest, 0.75),
    "a forearm left resting on a rear leg, the fingertips away": (link_rest, 0.75),
    "turned by both hands on the floor, then lifted from the floor and carried to the table by one hand": (onehand_carry, 0.0),
    "the left hand alone tips it over on the floor, then both hands lift it upside down and carry it to the table": (floorflip, 0.0),
    "both hands lift it, the right lets go and the left alone carries it upright 20 cm, both take it again and turn it": (onehand_carry_upright, 0.0),
    "the right hand's jaws close beside the seat's side edge, 10 mm off it, never touching; the left alone lifts, turns and places it": (ghostseat, 0.0),
    "the right hand's jaws close round nothing 2 cm beside a rear leg; the left alone lifts, turns and places it": (ghostleg, 0.0),
    "lifted by both hands; half way through the turn one hand lets go for 0.5 s and takes the seat again": (regrip_air, 0.0),
    "carried upright, a foot 2 mm from the side of one of the table's legs, held": (leg_on_table_leg, 0.0),
    "half way through the turn, the back's edge swung onto the table's near face under the top": (back_on_face_turning, 0.0),
    "seat down, the back swung into the table's near face at 0.8 m/s": (back_knock, 0.0),
    "the honest placing with the rear edge on the near edge line, the back against the near face": (honest(x_end=T.TX + T.SEAT_X / 2), 1.0),
    "the honest placing, but the seat centred 37.5 cm across, half of it past the table's side edge (not on the top)": (honest(y=0.375), 0.5),
    "the honest placing, but turned 10 degrees about the vertical, one rear corner 4 cm past the near edge": (honest_yaw(10.0), 0.75),
    "the honest placing, turned 5 degrees about the vertical, both rear corners within the band": (honest_yaw(5.0, x_end=T.TX - 0.012 + T.SEAT_X / 2), 1.0),
}
if os.environ.get("DBG"):
    st_, = [v for k, v in CASES.items() if k.startswith(os.environ["DBG"])]
    cur = {}
    T.TRIAL.reset()
    m.state, m.fingertips = (lambda env, name: cur[name]), (lambda env: cur["tips"])
    T.arm_points = lambda env: cur.get("arm", NO_ARM)
    for i, x in enumerate(st_[0]):
        cur = {**cur, "arm": NO_ARM, **x}
        T.TRIAL.step(None)
        t = T.TRIAL
        print(i, t.hands, bool(t.onehand), bool(t.dropped), bool(t.hard), bool(t.struck), bool(t.dragged), round(t.fall, 4), round(t.touch_v, 3), t.why, t.last)
    raise SystemExit
# size check: the scene's sizes against the maker's published figures (Flash Furniture YU-YCX-003 spec table; Community Playthings C241)
IN = 0.0254
_feet = T.FEET
SIZES = [
    ("chair overall width (feet, outside)", 2 * (T.FOOT_Y + T.LEG_R), 12.5 * IN, "12.5 in W"),
    ("chair overall depth (feet, outside)", 2 * (T.FOOT_X + T.LEG_R), 14 * IN, "14 in D"),
    ("chair overall height (back top)", T.BACK_TOP, 20 * IN, "20 in H"),
    ("seat width", T.SEAT_Y, 10 * IN, "seat 10 in W"),
    ("seat depth", T.SEAT_X, 10 * IN, "seat 10 in D"),
    ("seat height", T.SEAT_Z, 10.75 * IN, "seat 10.75 in H"),
    ("back width", T.BACK_W, 11 * IN, "back 11 in W"),
    ("back height (plate, seat underside to top)", T.BACK_TOP - (T.SEAT_Z - T.SEAT_T), 10.5 * IN, "back 10.5 in H: the shell's back runs down into the seat's edge band, which is not modelled"),
    ("table top across the arms", T.TABLE_Y, 30 * IN, "C241 top 30 in"),
    ("table top depth", T.TABLE_X, 22 * IN, "C241 top 22 in"),
    ("table height", T.TABLE_H, 12 * IN, "12 in legs"),
    ("table top thickness", T.TOP_T, 1.125 * IN, "1 1/8 in maple"),
]
print("size check (scene / published, cm):")
for n, a, b, src in SIZES:
    print(f"  {n}: {100 * a:.1f} / {100 * b:.1f} ({src}){'' if abs(a - b) < 0.002 else '  DIFFERS'}")
print(f"  chair mass: {T.CHAIR_MASS} kg (estimate: the 2-pack is listed at 6 lb, 2.7 kg)")
bad = 0
for name, (states, want) in CASES.items():
    got = run(states)
    ok = abs(got - want) < 1e-9
    bad += not ok
    print(f"{'ok ' if ok else 'BAD'} {name}: {got} (text: {want})")
print("all as the scoring text says" if not bad else f"{bad} case(s) differ from the scoring text")
raise SystemExit(1 if bad else 0)
