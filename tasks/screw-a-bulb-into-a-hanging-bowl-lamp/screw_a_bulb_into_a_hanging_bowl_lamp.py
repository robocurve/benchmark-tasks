"""Over the station table hangs an uplight: a shallow opal-glass drum shade, 30 cm across, on three chains from a
ceiling rose, with an empty lamp socket in its middle facing up. It hangs free: it swings, and it spins on its chains
at the slightest turn. A slim tubular bulb stands cap down in a holder on the table to the right.

Task: screw the bulb into the lamp's socket until it is seated, and let go. Screwing it in, the thread's drag turns the
whole lamp along with the bulb, and it hardly goes in: one hand has to hold the shade still by its rim while the other
screws the bulb in.

Run it:  inspect-robots run --embodiment yam_arena -E task=screw_a_bulb_into_a_hanging_bowl_lamp.py \\
           --instruction "..." --scorer task_score ...
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

from bodykit import (Body, BoxShape, CylShape, SphereShape, allow_matrix_grips, as_numpy, ball, on_reset,  # noqa: E402
                     on_step, rotation, segment, set_slide_target, set_thread_on, thread)
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, TCP_OFFSET  # noqa: E402
from yam_arena.demo import Waypoint, grip_rotation  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

allow_matrix_grips()

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# -- the lamp: origin at the middle of the shade's underside, z up the socket's axis --------------------------------
LX, LY = 0.28, 0.0
FLOOR_Z = TOP + 0.030  # the shade's underside (hung low, to change the bulb)
R_OUT, WALL_T, WALL_H, FLOOR_T = 0.150, 0.008, 0.050, 0.005  # the drum: outer radius, its thick wall, height, floor
SOCKET_R, SOCKET_IN, SOCKET_H = 0.013, 0.0095, 0.030  # the socket, a tube standing on the floor in the middle
THREAD_START = 0.004  # the socket's brass thread starts this deep (shown only: the bulb passes through the lamp)
SOCKET_TOP = FLOOR_T + SOCKET_H  # its mouth, in the lamp's frame
LAMP_MASS = 0.50
ANCHOR_Z = TOP + 0.70  # the chains meet at a ball joint under the ceiling rose
CHAIN_ANGLES = (60.0, 180.0, 300.0)  # degrees round the rim from +x (none at +90: the hand holds the rim there)
SWING_DAMPING, SPIN_DAMPING = 0.15, 0.0  # N m s/rad: it swings a little damped, and spins freely

# -- the bulb: a slim tubular bulb with a screw cap. Its frame: origin at the cap's tip, z up the bulb's axis -----------
CAP_R, CAP_L = 0.0075, 0.018
GLASS_R, GLASS_L = 0.009, 0.040  # a glass tube 18 mm across (narrow enough for a centred grip), domed on top
GLASS_Z = CAP_L + GLASS_L / 2 + 0.002  # where the hand holds it, from the cap's tip
BULB_MASS = 0.03
HOLDER_XY, HOLDER_IN, HOLDER_H, HOLDER_BASE = (0.20, -0.22), 0.009, 0.012, 0.006

# -- the thread: switched on once it catches; draws the bulb in PITCH per turn ---------------------------------------
PITCH = 0.008  # a coarse, quick thread: seated in under 2 turns
CATCH = math.radians(90.0)  # catches after a quarter turn clockwise (from above) with the cap in the socket, lined up
ENGAGE_TILT, ENGAGE_RADIAL, ENGAGE_DEPTH = math.radians(12.0), 0.006, (0.0, 0.012)
THREAD_DRAG = 0.3  # N m s/rad: turning the bulb in the socket drags (it turns the lamp, which spins freely)
SCREW_K, SCREW_D, SCREW_MAX = 3000.0, 20.0, 30.0
SEAT = 0.016  # the cap's tip this deep in the socket: seated
DEPTH_MAX = 0.017

GLASS_C, OPAL, BRASS, DARK, BULB_C, HOLDER_C, WHITE = (0.93, 0.92, 0.88), (0.95, 0.94, 0.90), (0.78, 0.64, 0.30), \
    (0.12, 0.10, 0.09), (0.98, 0.96, 0.85), (0.25, 0.25, 0.27), (0.95, 0.95, 0.95)

# -- scoring ----------------------------------------------------------------------------------------------------------
TURNS_IN = 1.0
LET_GO = 0.03  # each gripper's TCP at least this far from the bulb's glass and from the shade's rim
SUCCESS_HOLD_S = 1.0


def _ring(r_mid, thick, z0, z1, color, n=36) -> list:
    """A ring of boxes about the z axis; neighbours overlap (no slots)."""
    out = []
    chord = 2 * (r_mid + thick / 2) * math.tan(math.pi / n) + 0.001
    for i in range(n):
        a = 2 * math.pi * (i + 0.5) / n
        out.append(BoxShape((r_mid * math.cos(a), r_mid * math.sin(a), (z0 + z1) / 2), (thick, chord, z1 - z0), color,
                            yaw=a))
    return out


def _lamp() -> Body:
    shapes = [CylShape((0.0, 0.0, FLOOR_T / 2), R_OUT - WALL_T / 2, FLOOR_T, OPAL),
              *_ring(R_OUT - WALL_T / 2, WALL_T, 0.0, WALL_H, OPAL),
              *_ring((SOCKET_R + SOCKET_IN) / 2, SOCKET_R - SOCKET_IN, FLOOR_T, SOCKET_TOP, DARK, n=16),
              CylShape((0.0, 0.0, (FLOOR_T + SOCKET_TOP - THREAD_START) / 2), SOCKET_IN + 0.0003,
                       SOCKET_H - THREAD_START, BRASS, collide=False)]  # the brass thread inside
    top = np.array([0.0, 0.0, ANCHOR_Z - FLOOR_Z])
    for a in CHAIN_ANGLES:
        foot = np.array([(R_OUT - WALL_T / 2) * math.cos(math.radians(a)), (R_OUT - WALL_T / 2) * math.sin(math.radians(a)),
                         WALL_H])
        shapes.append(segment(tuple(foot), tuple(top), 0.003, BRASS, collide=False))
    return Body.build("pendant", pos=(LX, LY, FLOOR_Z), mass=LAMP_MASS, shapes=shapes, friction=(1.0, 0.9), color=OPAL,
                      com=(0.0, 0.0, 0.02))


def _rose() -> Body:
    shapes = [CylShape((0.0, 0.0, 0.012), 0.045, 0.024, WHITE), CylShape((0.0, 0.0, -0.004), 0.006, 0.008, BRASS)]
    return Body.build("rose", pos=(LX, LY, ANCHOR_Z), mass=1.0, shapes=shapes, kinematic=True, color=WHITE)


def _holder() -> Body:
    shapes = [BoxShape((0.0, 0.0, HOLDER_BASE / 2), (0.05, 0.05, HOLDER_BASE), HOLDER_C),
              *_ring(HOLDER_IN + 0.003, 0.006, HOLDER_BASE, HOLDER_BASE + HOLDER_H, HOLDER_C, n=16)]
    return Body.build("holder", pos=(*HOLDER_XY, TOP), mass=1.0, shapes=shapes, kinematic=True, friction=(0.4, 0.3),
                      color=HOLDER_C)


def _bulb() -> Body:
    shapes = [CylShape((0.0, 0.0, 0.0015), CAP_R - 0.0025, 0.003, DARK),  # the foot contact: a narrower tip
              CylShape((0.0, 0.0, (CAP_L + 0.003) / 2), CAP_R, CAP_L - 0.003, BRASS),
              CylShape((0.0, 0.0, CAP_L + GLASS_L / 2), GLASS_R, GLASS_L, BULB_C),
              SphereShape((0.0, 0.0, CAP_L + GLASS_L), GLASS_R, BULB_C)]
    return Body.build("bulb", pos=(*HOLDER_XY, TOP + HOLDER_BASE + 0.0005), mass=BULB_MASS, shapes=shapes,
                      friction=(1.2, 1.0), color=BULB_C, com=(0.0, 0.0, 0.022), ignore=("pendant",))


ROSE, HOLDER, LAMP, BULB = _rose(), _holder(), _lamp(), _bulb()
OBJECTS = [ROSE, HOLDER, LAMP, BULB]
ball("hang", "rose", "pendant", anchor=(LX, LY, ANCHOR_Z), axis=(0.0, 0.0, -1.0), swing_damping=SWING_DAMPING,
     twist_damping=SPIN_DAMPING)
# the bulb's thread in the socket, switched on once it catches: the cap's tip this deep below the socket's mouth
thread("screw", "pendant", "bulb", anchor0=(0.0, 0.0, SOCKET_TOP), axis0=(0.0, 0.0, -1.0), anchor1=(0.0, 0.0, 0.0),
       axis1=(0.0, 0.0, -1.0), slide_limits=(0.0, DEPTH_MAX), turn_damping=THREAD_DRAG, slide_stiffness=SCREW_K,
       slide_damping=SCREW_D, slide_max_force=SCREW_MAX)


# -- followed after every control step: the thread and the grippers --------------------------------------------------

def _fresh() -> dict:
    return {"pre": 0.0, "caught": False, "x0": 0.0, "turns": 0.0, "depth": 0.0, "phi": None, "tcps": None}


_EP = _fresh()
_DEBUG = {"on": bool(os.environ.get("BULB_DEBUG")), "steps": 0}


@on_reset
def _new_episode() -> None:
    _EP.clear()
    _EP.update(_fresh())
    _DEBUG["steps"] = 0


def _tcps(base, origin) -> list[np.ndarray]:
    out = []
    for arm in ("left_robot", "right_robot"):
        robot = base.scene[arm]
        i = next(j for j, n in enumerate(robot.body_names) if n == "gripper" or n.endswith("/gripper"))
        g = as_numpy(robot.data.body_link_pose_w)[0][i]
        out.append(g[:3] - origin + rotation(g[3:7]) @ np.array(TCP_OFFSET))
    return out


def _geometry(l_pos, l_rot, b_pos, b_rot) -> dict:
    """The bulb's cap against the socket: how deep its tip is below the mouth, how far off the axis, how lined up, and
    the bulb's turn against the lamp about the socket's axis."""
    up = l_rot @ np.array([0.0, 0.0, 1.0])
    mouth = np.asarray(l_pos, float) + l_rot @ np.array([0.0, 0.0, SOCKET_TOP])
    v = np.asarray(b_pos, float) - mouth
    depth = -float(v @ up)
    u = l_rot @ np.array([1.0, 0.0, 0.0])
    w = b_rot @ np.array([1.0, 0.0, 0.0])
    return {"depth": depth, "radial": float(np.linalg.norm(v + depth * up)),
            "cos": float((b_rot @ np.array([0.0, 0.0, 1.0])) @ up),
            "phi": math.atan2(float(np.cross(u, w) @ up), float(u @ w))}


@on_step
def _follow(env) -> None:
    base = env.unwrapped
    origin = as_numpy(base.scene.env_origins[0])
    la = as_numpy(base.scene["pendant"].data.root_link_pose_w)[0]
    bu = as_numpy(base.scene["bulb"].data.root_link_pose_w)[0]
    lr, br = rotation(la[3:7]), rotation(bu[3:7])
    g = _geometry(la[:3] - origin, lr, bu[:3] - origin, br)
    _EP["tcps"] = _tcps(base, origin)
    _EP["depth"] = g["depth"]
    tighten = 0.0  # clockwise seen from above
    if _EP["phi"] is not None:
        tighten = -((g["phi"] - _EP["phi"] + math.pi) % (2 * math.pi) - math.pi)
    _EP["phi"] = g["phi"]
    if _EP["caught"]:
        _EP["turns"] += tighten / (2 * math.pi)
        target = _EP["x0"] + PITCH * _EP["turns"]
        if target < -PITCH / 4:  # unscrewed out of the socket
            set_thread_on("screw", False)
            _EP.update(caught=False, pre=0.0, turns=0.0)
        else:
            set_slide_target("screw", min(max(target, 0.0), DEPTH_MAX))
    else:
        engaged = (g["cos"] >= math.cos(ENGAGE_TILT) and g["radial"] <= ENGAGE_RADIAL
                   and ENGAGE_DEPTH[0] <= g["depth"] <= ENGAGE_DEPTH[1])
        _EP["pre"] = _EP["pre"] + tighten if engaged else 0.0
        if _EP["pre"] >= CATCH:
            set_slide_target("screw", g["depth"])
            set_thread_on("screw", True)
            _EP.update(caught=True, x0=g["depth"], turns=0.0)
    if _DEBUG["on"]:
        _DEBUG["steps"] += 1
        if _DEBUG["steps"] % 10 == 0:
            st = _status_from(la[:3] - origin, lr, bu[:3] - origin, br)
            tl, tr = _EP["tcps"]
            spin = math.degrees(math.atan2(float(lr[1, 0]), float(lr[0, 0])))
            swing = math.degrees(math.acos(max(-1.0, min(1.0, float(lr[2, 2])))))
            rim = la[:3] - origin + lr @ np.array([0.0, R_OUT - WALL_T / 2, WALL_H])
            glass = bu[:3] - origin + br @ np.array([0.0, 0.0, GLASS_Z])
            print(f"[bulb] t={_DEBUG['steps'] / 20:5.1f}s lamp={np.round((la[:3] - origin) * 1000).astype(int).tolist()} "
                  f"spin={spin:6.1f} swing={swing:4.1f} depth={g['depth'] * 1000:6.1f}mm radial={g['radial'] * 1000:4.1f} "
                  f"tilt={math.degrees(math.acos(max(-1.0, min(1.0, g['cos'])))):4.1f} pre={math.degrees(_EP['pre']):6.1f} "
                  f"caught={_EP['caught']} turns={_EP['turns']:5.2f} seated={st['seated']} let_go={st['let_go']} "
                  f"left TCP-rim={np.round((tl - rim) * 1000).astype(int).tolist()} "
                  f"right TCP-glass={np.round((tr - glass) * 1000).astype(int).tolist()} score={_points(st)}",
                  file=sys.stderr, flush=True)


# -- scoring --------------------------------------------------------------------------------------------------------

def _status_from(l_pos, l_rot, b_pos, b_rot) -> dict:
    g = _geometry(l_pos, l_rot, b_pos, b_rot)
    seated = _EP["caught"] and g["depth"] >= SEAT - 0.001
    let_go = True
    tcps = _EP["tcps"]
    if tcps is not None:
        glass = np.asarray(b_pos, float) + b_rot @ np.array([0.0, 0.0, GLASS_Z])
        centre = np.asarray(l_pos, float) + l_rot @ np.array([0.0, 0.0, WALL_H])
        up = l_rot @ np.array([0.0, 0.0, 1.0])
        for t in tcps:
            v = np.asarray(t, float) - centre
            h = float(v @ up)
            r = float(np.linalg.norm(v - h * up))
            to_rim = math.hypot(r - (R_OUT - WALL_T / 2), h)
            let_go &= float(np.linalg.norm(t - glass)) >= LET_GO and to_rim >= LET_GO
    return {"caught": _EP["caught"], "turns": _EP["turns"], "seated": seated, "let_go": let_go}


def _status(objects) -> dict:
    la, bu = objects["pendant"], objects["bulb"]
    return _status_from(la.pos, rotation(la.quat_xyzw), bu.pos, rotation(bu.quat_xyzw))


def _points(st) -> float:
    if st["seated"] and st["let_go"]:
        return 1.0
    if st["caught"] and st["turns"] >= TURNS_IN:
        return 0.5
    return 0.25 if st["caught"] else 0.0


def score(objects) -> float:
    """1.0 with the bulb screwed in until seated and both grippers let go (3 cm clear of the bulb and the shade's rim);
    0.5 with its thread caught and turned at least 1 turn in; 0.25 with it caught (a quarter turn in); else 0.0."""
    return _points(_status(objects))


def success(objects) -> bool:
    """Full score with the lamp and the bulb still."""
    return _points(_status(objects)) >= 1.0 and objects["pendant"].resting() and objects["bulb"].resting()


# -- scripted demo ----------------------------------------------------------------------------------------------------
# The left hand comes straight down on the shade's rim on its left side, fingers closing across the rim, and holds it.
# The right hand comes straight down on the bulb's glass, lifts it out of its holder, carries it over the socket, sets
# the cap in the socket's mouth and screws it in, in strokes of a third of a turn clockwise (the wrist turns only so
# far): turn, let go, turn back open, take the glass again, following the bulb in. Then both hands let go and rise.


def _rz(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


RIM_GRIP = grip_rotation(0.0)  # straight down, fingers closing along y (across the rim on the shade's left side)
RIM_IN_HAND = -0.004  # the left TCP this far above the rim's top (below: the fingers straddle the wall)
L_FIX = np.array([-0.005, -0.001, 0.0])  # the left hand grips 5 mm forward of where it aims, pulling the lamp: aim back
GLASS_IN_HAND = 0.0  # the right TCP this far above GLASS_Z
YAW0, STROKE, STROKES = math.radians(60.0), math.radians(120.0), 8  # strokes clockwise from yaw +60 to -60
SLIP = 0.7  # the bulb turns about this much of each stroke (it slips in the fingers)
START_DEPTH, READY_GAP = 0.007, 0.02  # the hand holds the bulb ~3 mm higher than it aims
LEAD_S, LEAD_STEPS = 3.0, 15
PICK = [1.0, 0.8, 0.3]
TURN_S, RELEASE_S, BACK_S, REGRIP_S = 1.6, 0.4, 0.8, 0.4
ABOVE = 0.05
MOUTH = np.array([LX, LY, FLOOR_Z + SOCKET_TOP])


def _good_ik(arm: str, pos, rot) -> np.ndarray:
    import itertools

    from yam_arena import ik, station

    best = None
    for seed in itertools.product((-0.5, 0.0, 0.5), (0.8, 1.3), (0.4, 1.0), (-0.8, 0.0, 0.8), (-1.4, 0.0, 1.4), (0.0,)):
        q, pe, re = ik.solve(np.asarray(pos, float), ik.matrix_to_quat_xyzw(rot), np.array(seed), base_pos=station.BASES[arm],
                             iters=200)
        if pe < 0.001 and re < math.radians(0.5):
            room = float(np.minimum(q - ik.LOWER, ik.UPPER - q).min())
            if best is None or room > best[1]:
                best = (q, room)
    if best is None:
        raise ValueError(f"no IK solution for the {arm} grip at {pos}")
    return best[0]


def _lead_in(arm: str, pos, rot, label: str) -> list[Waypoint]:
    from yam_arena import ik, station
    from yam_arena.constants import READY

    q0, q1 = np.array(READY, float), _good_ik(arm, pos, rot)
    out = []
    for k in range(1, LEAD_STEPS + 1):
        p, r = ik.tcp_pose(q0 + (q1 - q0) * k / LEAD_STEPS, station.BASES[arm])
        out.append(Waypoint(LEAD_S / LEAD_STEPS, tuple(float(v) for v in p), FINGER_OPEN, label, r))
    return out


def _w(t, pos, finger, label, rot) -> Waypoint:
    return Waypoint(t, tuple(float(v) for v in pos), finger, label, rot)


def _tcp_for_depth(depth: float) -> np.ndarray:
    """The right TCP with the cap's tip ``depth`` below the socket's mouth (negative: above it)."""
    return MOUTH + np.array([0.0, 0.0, GLASS_Z + GLASS_IN_HAND - depth])


def _depth_after(turned: float) -> float:
    return min(START_DEPTH + PITCH * max(0.0, SLIP * turned - CATCH) / (2 * math.pi), DEPTH_MAX)


def _right() -> list[Waypoint]:
    pick = np.array([*HOLDER_XY, TOP + HOLDER_BASE + 0.0005 + GLASS_Z + GLASS_IN_HAND])
    rot0 = grip_rotation(YAW0)
    out = _lead_in("right", pick + [0.0, 0.0, ABOVE], rot0, "over the bulb")
    out += [_w(PICK[0], pick, FINGER_OPEN, "down round its glass", rot0),
            _w(PICK[1], pick, FINGER_CLOSED, "grip it", rot0),
            _w(PICK[2], pick, FINGER_CLOSED, "settle", rot0),
            _w(1.0, pick + [0.0, 0.0, 0.08], FINGER_CLOSED, "lift it out of the holder", rot0),
            _w(2.0, _tcp_for_depth(-READY_GAP), FINGER_CLOSED, "over the socket", rot0),
            _w(1.0, _tcp_for_depth(START_DEPTH), FINGER_CLOSED, "set the cap in the socket's mouth", rot0)]
    turned = 0.0
    for k in range(STROKES):
        for j in range(1, 5):  # a quarter of the stroke at a time
            done = turned + STROKE * j / 4
            out.append(_w(TURN_S / 4, _tcp_for_depth(_depth_after(done)), FINGER_CLOSED, f"screw it in ({k + 1})",
                          _rz(-STROKE * j / 4) @ rot0))
        turned += STROKE
        at = _tcp_for_depth(_depth_after(turned))
        if k < STROKES - 1:
            out.append(_w(RELEASE_S, at, FINGER_OPEN, "let go", _rz(-STROKE) @ rot0))
            for j in range(1, 5):
                out.append(_w(BACK_S / 4, at, FINGER_OPEN, "turn back, open", _rz(-STROKE * (1 - j / 4)) @ rot0))
            out.append(_w(REGRIP_S, at, FINGER_CLOSED, "take the glass again", rot0))
    end = _tcp_for_depth(_depth_after(turned))
    out += [_w(0.5, end, FINGER_OPEN, "let go", _rz(-STROKE) @ rot0),
            _w(1.0, end + [0.0, -0.03, 0.03], FINGER_OPEN, "rise and back off", _rz(-STROKE) @ rot0)]
    return out


def _left(release_at: float, total: float) -> list[Waypoint]:
    rim = np.array([LX, LY + R_OUT - WALL_T / 2, FLOOR_Z + WALL_H + RIM_IN_HAND]) + L_FIX
    out = _lead_in("left", rim + [0.0, 0.0, ABOVE], RIM_GRIP, "over the shade's rim, on its left")
    out += [_w(PICK[0], rim, FINGER_OPEN, "down round the rim", RIM_GRIP),
            _w(PICK[1], rim, FINGER_CLOSED, "grip it", RIM_GRIP)]
    t = LEAD_S + PICK[0] + PICK[1]
    out.append(_w(max(release_at - t, 0.5), rim, FINGER_CLOSED, "hold the lamp still", RIM_GRIP))
    out += [_w(0.5, rim, FINGER_OPEN, "let go", RIM_GRIP),
            _w(1.0, rim + [0.0, 0.0, 0.06], FINGER_OPEN, "rise", RIM_GRIP)]
    t = sum(w.duration for w in out)
    out.append(_w(max(total - t, 0.5), rim + [0.0, 0.0, 0.06], FINGER_OPEN, "wait", RIM_GRIP))
    return out


def demo():
    right = _right()
    total = sum(w.duration for w in right)
    left = _left(total - 1.5, total + 4.0)
    right.append(_w(4.0, right[-1].pos, FINGER_OPEN, "wait", right[-1].yaw))
    return {"left": left, "right": right}
