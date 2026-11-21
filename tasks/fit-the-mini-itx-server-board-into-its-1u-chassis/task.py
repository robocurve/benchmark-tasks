"""On the bench, in front of the robot: an open 1U rack server chassis with its lid off, its front toward the robot.
Inside it, behind a fan holder and beside a power supply, four brass standoffs stand on the chassis floor, and the
chassis's rear wall has a steel I/O shield with cut-outs for a board's connectors. To the robot's left, on a black
antistatic foam pad, lies a green Mini-ITX server board, components up, with a silver heat sink near its middle and
a row of connectors along one edge (two blue D-sub connectors with zinc flanges, a network jack over two USB ports,
and two pairs of stacked network jacks).
"""

from __future__ import annotations

import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import parts  # noqa: E402  (before lots, so its reset hooks run first)
import lots  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
EPS = 1e-6

GREEN, STEEL, ZINC, BRASS = (0.10, 0.40, 0.18), (0.55, 0.57, 0.60), (0.72, 0.73, 0.75), (0.80, 0.62, 0.25)
BLACK, DARK, BLUE, SILVER = (0.05, 0.05, 0.06), (0.15, 0.15, 0.16), (0.15, 0.25, 0.70), (0.80, 0.81, 0.83)
GOLD, WHITE = (0.85, 0.68, 0.25), (0.92, 0.92, 0.92)

# --- the board (loose). Frame origin: the centre of the PCB's underside; +x is the connector (rear I/O) edge -------
PCB_W, PCB_T = 0.170, 0.0016  # Mini-ITX: 170 x 170 mm, 1.6 mm FR-4
XE = PCB_W / 2  # the connector edge, board frame
BOARD_MASS = 0.45  # kg, estimate (board with heat sink)
# Mini-ITX mounting holes C, F, H, J (3.96 mm), board frame, m
HOLES = {"C": (0.07484, -0.07865), "F": (0.05198, 0.07883), "H": (-0.08010, -0.07865), "J": (-0.08010, 0.07883)}
HOLE_D = 0.00396
SINK = (0.060, 0.060, 0.024)  # passive heat sink, estimate
SINK_AT = (-0.010, 0.0)
GRIP_AT = (0.005, 0.0)  # where the demo grips the heat sink: over the board's balance point (its connector edge is heavier)
PT = PCB_T  # connector bodies stand on the PCB's top face

# Rear connectors along the +x edge: (name, kind, y centre, width, height). D-sub: a 31 x 12.5 mm flange 1-2 mm past
# the PCB edge, a moulded body behind it, and a 17 x 8.5 mm shell 6 mm in front of it. Jacks overhang the edge 4 mm.
PORTS = (
    ("vga", "dsub", -0.0465, 0.031, 0.0125),
    ("lan_usb", "jack", -0.0210, 0.016, 0.030),  # management network jack over two USB ports
    ("lan12", "jack", -0.0030, 0.016, 0.027),
    ("lan34", "jack", 0.0160, 0.016, 0.027),
    ("com", "dsub", 0.0435, 0.031, 0.0125),
)
SHELL_W, SHELL_H, SHELL_Z = 0.017, 0.0085, 0.002  # D-sub shell: width, height, height of its bottom above the PCB
FLANGE_FRONT = XE + 0.002  # flange front face, board frame
SHELL_FRONT = XE + 0.008
JACK_FRONT = XE + 0.004
JACK_BACK = {"lan_usb": XE - 0.018, "lan12": XE - 0.018, "lan34": XE - 0.018}


def _board_shapes():
    s = [parts.box((PCB_W, PCB_W, PCB_T), at=(0.0, 0.0, PCB_T / 2), color=GREEN)]
    s.append(parts.box(SINK, at=(SINK_AT[0], SINK_AT[1], PT + SINK[2] / 2), color=SILVER))
    for name, kind, yc, w, h in PORTS:
        if kind == "dsub":
            s.append(parts.box((0.011, w, h), at=(XE - 0.0045, yc, PT + h / 2), color=BLUE))  # moulded body
            s.append(parts.box((0.001, w, h), at=(FLANGE_FRONT - 0.0005, yc, PT + h / 2), color=ZINC))  # flange
            s.append(parts.box((SHELL_FRONT - FLANGE_FRONT, SHELL_W, SHELL_H),
                               at=((SHELL_FRONT + FLANGE_FRONT) / 2, yc, PT + SHELL_Z + SHELL_H / 2), color=ZINC))
        else:
            back = JACK_BACK[name]
            s.append(parts.box((JACK_FRONT - back, w, h), at=((JACK_FRONT + back) / 2, yc, PT + h / 2), color=ZINC))
    for i in range(4):  # DIMM slots
        s.append(parts.box((0.0065, 0.135, 0.0060), at=(-0.052 - 0.0085 * i, 0.0, PT + 0.003), color=BLACK))
    for hx, hy in HOLES.values():  # plated ring and the hole drawn on the top face
        s.append(parts.cylinder(0.0035, 0.0002, at=(hx, hy, PT + 0.0001), color=GOLD))
        s.append(parts.cylinder(HOLE_D / 2, 0.0003, at=(hx, hy, PT + 0.00015), color=DARK))
    return s


# --- the chassis (fixed): CSE-505-203B, 437 x 249 x 43 mm, front toward the robot ------------------------------------
BX, BY = 0.360, 0.050  # the seated board's frame origin, world
FLOOR_T, WALL_T, CH_H = 0.001, 0.001, 0.043
SO_R, SO_H = 0.00275, 0.00635  # brass M3 standoffs (5.5 mm, 6.35 mm high)
SEAT_Z = TOP + FLOOR_T + SO_H  # the standoff tops: the PCB's underside when seated
XS = BX + FLANGE_FRONT  # the I/O shield's inner face: the D-sub flanges bear on it when the board is home
CH_X0, CH_X1 = XS + WALL_T - 0.249, XS + WALL_T  # chassis outer faces, front and rear
CH_Y1 = BY + XE + 0.010 + WALL_T  # left outer face
CH_Y0 = CH_Y1 - 0.437
CHX, CHY = (CH_X0 + CH_X1) / 2, (CH_Y0 + CH_Y1) / 2
DSUB_CLR, JACK_CLR = 0.00045, 0.0010  # shield cut-out clearance per side


def openings():
    """The shield's cut-outs in world y and z: (name, y0, y1, z0, z1)."""
    out = []
    pz = SEAT_Z + PT
    for name, kind, yc, w, h in PORTS:
        if kind == "dsub":
            out.append((name, BY + yc - SHELL_W / 2 - DSUB_CLR, BY + yc + SHELL_W / 2 + DSUB_CLR,
                        pz + SHELL_Z - DSUB_CLR, pz + SHELL_Z + SHELL_H + DSUB_CLR))
        else:
            out.append((name, BY + yc - w / 2 - JACK_CLR, BY + yc + w / 2 + JACK_CLR, pz - JACK_CLR,
                        pz + h + JACK_CLR))
    return out


def _plate(x0, x1, y0, y1, z0, z1, holes, color):
    """A vertical plate between x0 and x1 with rectangular cut-outs, as boxes (world, made relative later)."""
    b, y = [], y0
    for _, ya, yb, za, zb in sorted(holes, key=lambda h: h[1]):
        if ya > y:
            b.append(((x0, y, z0), (x1, ya, z1)))
        if za > z0:
            b.append(((x0, ya, z0), (x1, yb, za)))
        if zb < z1:
            b.append(((x0, ya, zb), (x1, yb, z1)))
        y = yb
    if y1 > y:
        b.append(((x0, y, z0), (x1, y1, z1)))
    return [(lo, hi, color) for lo, hi in b]


def _chassis_shapes():
    w = []  # (low corner, high corner, colour), world
    z0, z1 = TOP, TOP + CH_H
    w.append(((CH_X0, CH_Y0, z0), (CH_X1, CH_Y1, z0 + FLOOR_T), STEEL))  # floor
    w.append(((CH_X0, CH_Y0, z0), (CH_X0 + WALL_T, CH_Y1, z1), STEEL))  # front wall
    w.append(((CH_X0, CH_Y0, z0), (CH_X1, CH_Y0 + WALL_T, z1), STEEL))  # right wall
    w.append(((CH_X0, CH_Y1 - WALL_T, z0), (CH_X1, CH_Y1, z1), STEEL))  # left wall
    # rear wall: the I/O shield across the board's window, plain wall either side
    sy0, sy1 = BY - 0.075, BY + 0.075
    w += _plate(XS, XS + WALL_T, sy0, sy1, z0 + FLOOR_T, z1, openings(), ZINC)
    w.append(((XS, CH_Y0, z0), (XS + WALL_T, sy0, z1), STEEL))
    w.append(((XS, sy1, z0), (XS + WALL_T, CH_Y1 - WALL_T, z1), STEEL))
    # power supply at the rear right, a fan holder ahead of the board, a 2.5" drive bracket at the front right
    w.append(((XS - 0.150, -0.180, z0 + FLOOR_T), (XS, -0.0985, z0 + FLOOR_T + 0.0405), DARK))
    w.append(((CH_X0 + 0.006, BY - 0.020, z0 + FLOOR_T), (CH_X0 + 0.034, BY + 0.020, z0 + FLOOR_T + 0.040), BLACK))
    w.append(((CH_X0 + 0.010, -0.270, z0 + FLOOR_T), (CH_X0 + 0.110, -0.200, z0 + FLOOR_T + 0.012), STEEL))
    origin = np.array([CHX, CHY, TOP])
    s = []
    for lo, hi, c in w:
        lo, hi = np.asarray(lo, float), np.asarray(hi, float)
        s.append(parts.box(tuple(hi - lo), at=tuple((lo + hi) / 2 - origin), color=c))
    for hx, hy in HOLES.values():  # standoffs, with the tapped hole drawn on top
        s.append(parts.cylinder(SO_R, SO_H, at=(BX + hx - CHX, BY + hy - CHY, FLOOR_T + SO_H / 2), color=BRASS))
        s.append(parts.cylinder(0.0013, 0.0002, at=(BX + hx - CHX, BY + hy - CHY, FLOOR_T + SO_H + 0.0001),
                                color=BLACK))
    return s


SLIDE_FRICTION = (0.45, 0.35)  # static, dynamic: solder-masked FR-4 on brass and steel, estimate
CHASSIS = parts.Part("chassis", pos=(CHX, CHY, TOP), shapes=_chassis_shapes(), friction=SLIDE_FRICTION)

# --- the foam pad (fixed) and the board's start ---------------------------------------------------------------------
PAD = (0.150, 0.150, 0.010)
PAD_X, PAD_Y = 0.272, 0.330
FOAM_PAD = parts.Part("foam_pad", pos=(PAD_X, PAD_Y, TOP),
                      shapes=[parts.box(PAD, at=(0.0, 0.0, PAD[2] / 2), color=BLACK)])
BOARD_Z0 = TOP + PAD[2]
BOARD = parts.Part("server_board", pos=(PAD_X, PAD_Y, BOARD_Z0), shapes=_board_shapes(), mass=BOARD_MASS,
                   friction=SLIDE_FRICTION)

OBJECTS = [CHASSIS, FOAM_PAD, BOARD]
LOTS = lots.declare(
    OBJECTS,
    lots.Lot("A", place={"server_board": (PAD_X, PAD_Y, BOARD_Z0)}),
    lots.Lot("B", place={"server_board": (PAD_X + 0.012, PAD_Y + 0.008, BOARD_Z0)},
             yaw={"server_board": -math.pi / 2}),
)

# --- scoring geometry -----------------------------------------------------------------------------------------------
ON_TOL = 0.0010  # on a standoff: the underside within 1.0 mm of its top
TILT_MAX = 2.0  # degrees
YAW_MAX = 5.0  # degrees: the connector edge toward the shield
NEAR = 0.010  # in its place: every mounting hole within 10 mm of its standoff
ENTERED = 0.001  # a D-sub shell has entered its cut-out: its front at least 1 mm past the shield's inner face
HOME_GAP = 0.0003  # home: both D-sub flanges within 0.3 mm of the shield
HOLE_TOL = 0.00048  # centred: each hole's axis within 0.48 mm of its standoff's (an M3 screw starts: (3.96 - 3.0) / 2)
FLOOR_GAP = 0.003  # fell off: any corner of the PCB within 3 mm of the chassis floor
DROP_V = 0.6  # m/s downward: a free fall of about 18 mm
CLEAR = 0.005
STILL_STEPS = 20
STILL_POS, STILL_DEG = 0.0002, 0.2


def _rot(q) -> np.ndarray:
    return parts._quat_to_matrix(q)


def _world(board, p) -> np.ndarray:
    return _rot(board.quat_xyzw) @ np.asarray(p, float) + np.asarray(board.pos, float)


def tilt_deg(board) -> float:
    return math.degrees(math.acos(max(-1.0, min(1.0, float(_rot(board.quat_xyzw)[2, 2])))))


def hole_offsets(board) -> dict:
    """Each mounting hole: (horizontal offset of its axis from its standoff's axis, height of the PCB underside there
    above the standoff top), m."""
    out = {}
    for k, (hx, hy) in HOLES.items():
        p = _world(board, (hx, hy, 0.0))
        out[k] = (float(math.hypot(p[0] - (BX + hx), p[1] - (BY + hy))), float(p[2] - SEAT_Z))
    return out


def yaw_deg(board) -> float:
    r = _rot(board.quat_xyzw)
    return math.degrees(math.atan2(r[1, 0], r[0, 0]))


def on_standoffs(board) -> bool:
    """Flat on all four standoffs in its place: the PCB lies within 1 mm of every standoff top, over each standoff
    (the standoff's top face fully under the PCB), tilts at most 2 degrees, its connector edge faces the shield
    (turned at most 5 degrees) and every mounting hole is within 10 mm of its standoff."""
    if tilt_deg(board) > TILT_MAX + EPS or abs(yaw_deg(board)) > YAW_MAX + EPS:
        return False
    if any(off > NEAR + EPS for off, _ in hole_offsets(board).values()):
        return False
    r = _rot(board.quat_xyzw)
    p = np.asarray(board.pos, float)
    for hx, hy in HOLES.values():
        so = np.array([BX + hx, BY + hy, SEAT_Z])
        local = r.T @ (so - p)  # the standoff top in the board frame
        if abs(local[0]) > XE - SO_R + EPS or abs(local[1]) > XE - SO_R + EPS:
            return False
        if abs(local[2]) > ON_TOL + EPS:
            return False
    return True


def port_fronts(board) -> dict:
    """World x of each connector's front face (D-sub: the shell), and of the two D-sub flanges."""
    out = {}
    for name, kind, yc, _, _ in PORTS:
        front = SHELL_FRONT if kind == "dsub" else JACK_FRONT
        out[name] = float(_world(board, (front, yc, PT + 0.004))[0])
        if kind == "dsub":
            out[name + "_flange"] = float(_world(board, (FLANGE_FRONT, yc, PT + 0.006))[0])
    return out


def entered(board) -> bool:
    """Both D-sub shells (the connectors that locate the board in the shield) into their cut-outs."""
    f = port_fronts(board)
    return all(f[name] >= XS + ENTERED - EPS for name, kind, *_ in PORTS if kind == "dsub")


def home(board) -> bool:
    f = port_fronts(board)
    return all(f[n] >= XS - HOME_GAP - EPS for n in ("vga_flange", "com_flange"))


def centred(board) -> bool:
    return all(off <= HOLE_TOL + EPS for off, _ in hole_offsets(board).values())


def _lowest_corner(board) -> float:
    return min(float(_world(board, (sx * XE, sy * XE, 0.0))[2]) for sx in (-1, 1) for sy in (-1, 1))


def _over_chassis(board) -> bool:
    p = board.pos
    return CH_X0 <= p[0] <= CH_X1 and CH_Y0 <= p[1] <= CH_Y1


# --- whole-trial record, kept in the parts module so near_misses.py shares it ----------------------------------------
_grip = parts.__dict__.setdefault("_itx_task_grip", {"clear": None})
_rec = parts.__dict__.setdefault("_itx_task_record", {})


def _clear_record():
    _rec.clear()
    _rec.update(zero=None, prev=None, still=0, still_pose=None)


_clear_record()


def _start_pos():
    return np.asarray(lots.current().pos("server_board"), float)


def _update(objects) -> None:
    b = objects["server_board"]
    pos = np.asarray(b.pos, float)
    prev = _rec["prev"]
    if (prev is not None and np.linalg.norm(pos - _start_pos()) < 0.0005 and b.resting()
            and np.linalg.norm(pos - prev) > 0.020):
        _clear_record()  # a scene reset: back at the start by a jump no robot can make
    _rec["prev"] = pos
    vz = float(b.lin_vel[2]) if getattr(b, "lin_vel", None) is not None else 0.0
    if vz < -DROP_V - EPS:
        _rec["zero"] = _rec["zero"] or "dropped"
    if _over_chassis(b) and _lowest_corner(b) < TOP + FLOOR_T + FLOOR_GAP - EPS:
        _rec["zero"] = _rec["zero"] or "fell onto the chassis floor"
    if pos[2] < TOP - 0.02:
        _rec["zero"] = _rec["zero"] or "off the bench"
    now = (pos, _rot(b.quat_xyzw))
    was = _rec["still_pose"]
    same = (was is not None and np.linalg.norm(now[0] - was[0]) <= STILL_POS + EPS
            and math.degrees(math.acos(max(-1.0, min(1.0, (np.trace(now[1].T @ was[1]) - 1) / 2)))) <= STILL_DEG)
    # the 1 s stillness counts only once the arms are measured clear of the board
    _rec["still"] = _rec["still"] + 1 if (same and _grip["clear"] is True) else 0
    _rec["still_pose"] = now


def _clear(objects):
    """The arms' measured clearance from the board (None until measured: no credit)."""
    if "offline_clear" in _grip:
        _grip["clear"] = _grip["offline_clear"](objects)
    return _grip["clear"]


def _value(objects) -> float:
    b = objects["server_board"]
    if _rec["zero"]:
        return 0.0
    if _grip["clear"] is not True or _rec["still"] < STILL_STEPS:
        return 0.0
    if not on_standoffs(b):
        return 0.0
    if not entered(b):
        return 0.25
    if home(b) and centred(b):
        return 1.0
    return 0.5


def success(objects):
    """The board home on its four standoffs: every connector through the shield, both D-sub flanges against it,
    every mounting hole centred on its standoff, the board let go and still for 1 s."""
    _clear(objects)
    return bool(_value(objects) == 1.0)


def distance(objects):
    return float(max(off for off, _ in hole_offsets(objects["server_board"]).values()))


def score(objects):
    _clear(objects)
    _update(objects)
    return _value(objects)


# --- the let-go measurement: arm hull faces sampled every 3 mm -------------------------------------------------------
FACE_STEP = 0.003
ARM_LINKS = ("link3", "link4", "link5", "gripper", "tip_left", "tip_right")


def _face_points(vertices, planes, step=FACE_STEP) -> np.ndarray:
    pts = [np.asarray(vertices, float)]
    _, first = np.unique(np.round(planes / np.array([1e-4, 1e-4, 1e-4, 1e-5])).astype(np.int64), axis=0,
                         return_index=True)
    for n, d in zip(planes[np.sort(first), :3], planes[np.sort(first), 3]):
        on = vertices[np.abs(vertices @ n - d) < 1e-6]
        if len(on) < 3:
            continue
        c = on.mean(axis=0)
        e1 = on[0] - c
        if np.linalg.norm(e1) < 1e-9:
            continue
        e1 = e1 / np.linalg.norm(e1)
        e2 = np.cross(n, e1)
        ring = on[np.argsort(np.arctan2((on - c) @ e2, (on - c) @ e1))]
        for a, b in zip(ring, np.roll(ring, -1, axis=0)):
            m = max(1, int(math.ceil(max(np.linalg.norm(a - c), np.linalg.norm(b - c)) / step)))
            for i in range(m + 1):
                for j in range(m + 1 - i):
                    pts.append((c + (a - c) * (i / m) + (b - c) * (j / m))[None, :])
    pts = np.concatenate(pts)
    _, keep = np.unique(np.round(pts / (step / 3)).astype(np.int64), axis=0, return_index=True)
    return np.concatenate([np.asarray(vertices, float), pts[np.sort(keep)]])


_LINK_POINTS: dict = {}


def _link_points(name) -> list:
    if name not in _LINK_POINTS:
        from yam_arena import analysis

        out = []
        for h in analysis.link_hulls().get(name, []):
            pts = _face_points(h.vertices, h.planes)
            c = pts.mean(axis=0)
            out.append((c, float(np.linalg.norm(pts - c, axis=1).max()), pts))
        _LINK_POINTS[name] = out
    return _LINK_POINTS[name]


def board_boxes(board) -> list:
    return [(c, r, s) for _, c, r, s in BOARD.world_boxes({"base": (np.asarray(board.pos), _rot(board.quat_xyzw))})]


def _arms_clear(points, boxes) -> bool:
    from yam_arena import analysis

    if not len(points):
        return True
    pts = np.concatenate(points)
    for c, r, s in boxes:
        reach = float(np.linalg.norm(s)) / 2 + CLEAR + 1e-3
        near = pts[np.einsum("ij,ij->i", pts - c, pts - c) <= reach * reach]
        if len(near) and float(analysis.box_signed_distance(near, c, r, s).min()) <= CLEAR:
            return False
    return True


@parts.on_step
def _measure_grip(sim) -> None:
    base = sim.env.unwrapped
    origin = parts._np(base.scene.env_origins[0])
    boxes = board_boxes(sim.states()["server_board"])
    lo = np.min([c - np.linalg.norm(sz) / 2 for c, _, sz in boxes], axis=0) - CLEAR
    hi = np.max([c + np.linalg.norm(sz) / 2 for c, _, sz in boxes], axis=0) + CLEAR
    points, found = [], 0
    for arm in ("left_robot", "right_robot"):
        asset = base.scene[arm]
        poses = parts._np(asset.data.body_link_pose_w)[0]
        for i, n in enumerate(asset.body_names):
            if n in ARM_LINKS:
                rot, pos = _rot(poses[i, 3:]), poses[i, :3] - origin
                hulls = _link_points(n)
                found += 1 if hulls else 0
                for c, rad, p in hulls:
                    cw = rot @ c + pos
                    if np.all(cw + rad >= lo) and np.all(cw - rad <= hi):
                        points.append(p @ rot.T + pos)
    # fail closed: unless every listed link of both arms was found, the state stays unknown (None, no credit)
    _grip["clear"] = _arms_clear(points, boxes) if found == 2 * len(ARM_LINKS) else None


@parts.on_reset
def _reset(sim) -> None:
    _grip["clear"] = None
    _clear_record()


# --- demo: open loop from the visible layout (where the board lies and which way its connector edge faces) -----------
TIP = 0.013  # pad tips below the TCP
GRIP_ABOVE = PCB_T + 0.004 + TIP  # TCP above the PCB's underside: pad tips 4 mm above the PCB's top
OPEN = -0.042  # pads 84 mm apart round the 60 mm heat sink
SQUEEZE = -0.027  # pads commanded 54 mm apart: 3 mm past each face
TRAVEL_Z = TOP + 0.110
APPROACH = 0.012  # lowered into the chassis this far short of home: the connectors stay well clear of the shield
HOVER = 0.018  # then held this far above the standoff tops (the loaded arm sags 4-5 mm and the board may lean 3-4
# degrees) while it is moved back until its D-sub shells meet the shield plate above their cut-outs: the PCB stays
# well clear of the rear standoff on the way
AT_FACE = 0.006  # where that leaves it: the shells (6 mm proud of the flanges) against the shield's inner face
SHORT = AT_FACE  # (near_misses.py: the board set down short of the shield)
PUSH = 0.020  # then slid toward the shield, commanded this far past home (the arm yields)
EASE = 0.006  # eased to this far past home before letting go (still pushing)
# Measured repeatable offsets of the left arm at the chassis (TCP target minus where the board went), m: carrying
# the board (set-down), and with the board resting on the standoffs (re-grip and slide).
CARRY_FIX = (0.0, -0.0036)  # sideways only: where along x the board comes down is set by the shield face
SLIDE_FIX = (0.0, 0.0022)  # the unloaded arm's sideways offset where it grips the heat sink again
VARIANTS = ("short", "off_side", "no_push")  # traces only (DEMO_VARIANT=...)


def demo():
    """The left arm grips the heat sink, lifts the board off the foam, turns it so its connector edge faces the
    chassis's rear wall and sets it down flat on the four standoffs just short of the shield. It lets go, grips the
    heat sink again, slides the board back until the D-sub flanges meet the shield, lets go and moves away. The right
    arm stays parked."""
    import os

    variant = os.environ.get("DEMO_VARIANT", "")
    lot = lots.current()
    sx, sy, sz = lot.pos("server_board")
    yaw0 = lot.yaw_of("server_board")
    c, s = math.cos(yaw0), math.sin(yaw0)
    gx, gy = sx + c * GRIP_AT[0] - s * GRIP_AT[1], sy + s * GRIP_AT[0] + c * GRIP_AT[1]
    gz = sz + GRIP_ABOVE
    hx, hy = BX + GRIP_AT[0], BY + GRIP_AT[1]  # the grip point with the board home
    cx, cy = hx - APPROACH + CARRY_FIX[0], hy + CARRY_FIX[1]  # lowered in short of home
    bx = hx + 0.010  # then moved back past the shield face: the arm yields once the shells meet it
    rx, ry = hx - AT_FACE + SLIDE_FIX[0], hy + SLIDE_FIX[1]  # re-grip where it was set down
    if variant == "off_side":
        ry += 0.002
    down = SEAT_Z + GRIP_ABOVE
    push = 0.0 if variant == "no_push" else PUSH
    ease = 0.0 if variant == "no_push" else EASE
    w = [
        Waypoint(2.0, (gx, gy, TRAVEL_Z), OPEN, "over the heat sink", yaw0),
        Waypoint(1.5, (gx, gy, gz), OPEN, "down round the heat sink", yaw0),
        Waypoint(0.8, (gx, gy, gz), SQUEEZE, "grip", yaw0),
        Waypoint(1.2, (gx, gy, gz + 0.060), SQUEEZE, "lift off the foam", yaw0),
        Waypoint(1.5, (gx, gy, TRAVEL_Z), SQUEEZE, "up", yaw0),
        Waypoint(2.5, (cx, cy, TRAVEL_Z), SQUEEZE, "over the chassis, connectors toward the shield", 0.0),
        Waypoint(1.5, (cx, cy, down + HOVER), SQUEEZE, "lower inside, short of the shield", 0.0),
        Waypoint(0.8, (cx, cy, down + HOVER), SQUEEZE, "steady", 0.0),
        Waypoint(1.5, (bx, cy, down + HOVER), SQUEEZE, "back until the D-sub shells meet the shield", 0.0),
        Waypoint(1.0, (bx, cy, down - 0.002), SQUEEZE, "down onto the standoffs", 0.0),
        Waypoint(0.8, (bx, cy, down - 0.002), OPEN, "let go: it settles flat", 0.0),
        Waypoint(0.8, (bx, cy, down + 0.030), OPEN, "up off it", 0.0),
    ]
    if variant == "short":  # leave it on the standoffs short of the shield
        last = w[-1].pos
    else:
        w += [
            Waypoint(1.0, (rx, ry, down + 0.030), OPEN, "over the heat sink", 0.0),
            Waypoint(1.0, (rx, ry, down), OPEN, "down round it", 0.0),
            Waypoint(0.8, (rx, ry, down), SQUEEZE, "grip again", 0.0),
            Waypoint(3.0, (hx + push + SLIDE_FIX[0], ry, down), SQUEEZE, "slide back into the shield", 0.0),
            Waypoint(1.0, (hx + push + SLIDE_FIX[0], ry, down), SQUEEZE, "hold", 0.0),
            Waypoint(1.0, (hx + ease + SLIDE_FIX[0], ry, down), SQUEEZE, "ease", 0.0),
            Waypoint(0.8, (hx + ease + SLIDE_FIX[0], ry, down), OPEN, "let go", 0.0),
        ]
        last = w[-1].pos
    w += [
        Waypoint(1.2, (last[0], last[1], TRAVEL_Z), OPEN, "up", 0.0),
        Waypoint(1.5, (0.20, 0.36, TRAVEL_Z), OPEN, "away", 0.0),
        Waypoint(3.0, (0.20, 0.36, TRAVEL_Z), OPEN, "wait clear", 0.0),
    ]
    return {"left": w, "right": []}


DEMO_TOUCHES = ()
