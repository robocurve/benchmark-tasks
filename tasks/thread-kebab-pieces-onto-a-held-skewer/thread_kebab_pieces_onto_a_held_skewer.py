"""On the station table, a metal kebab skewer with a wooden handle lies across a small charcoal grill (no fire), its
handle on the grill's left side wall and its point on the right one. Four kebab pieces, each with a hole through it,
lie in a row on a cutting board in front of the grill, and a small prep stand stands just past the skewer's point.

Task: hold the skewer up off the grill with one hand and with the other put all four pieces on it (set each in the
prep stand's channel, hole toward the point, and push it onto the point and up the skewer), then lay the skewer back
across the grill and let go. The skewer's point rests on the grill's wall, so no piece can go on while it lies there:
it has to be lifted, and held level at the stand's height while the pieces go on.

Run it:  inspect-robots run --embodiment yam_arena -E task=thread_kebab_pieces_onto_a_held_skewer.py \\
           --instruction "..." --scorer task_score ...
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

from bodykit import (Body, BoxShape, CylShape, allow_matrix_grips, as_numpy, on_reset, on_step, rotation,  # noqa: E402
                     set_slide_target, set_thread_on, thread)
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, TCP_OFFSET  # noqa: E402
from yam_arena.demo import Waypoint, grip_rotation  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

allow_matrix_grips()

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# -- the skewer. Its frame: origin at the middle of the collar's blade-side face, the blade along -y (toward the right
# arm), the handle along +y. "u" below is the distance along the blade from that face toward the point. ---------------
HANDLE_L, HANDLE_W, HANDLE_H = 0.120, 0.014, 0.026  # a flat wooden handle: long, across (x), tall (z)
COLLAR_T, COLLAR_W = 0.003, 0.028  # a steel collar between handle and blade: the pieces stop against it
ROD_R, ROD_L = 0.002, 0.225  # the blade: a round steel rod 4 mm across
TAPER = ((0.0014, 0.008), (0.0008, 0.008))  # then narrower, to a point: (radius, length) of each step
TIP_U = ROD_L + sum(t[1] for t in TAPER)  # the point, 24.1 cm from the collar
SKEWER_MASS = 0.07
SKEWER_COM = (0.0, -0.018, 0.0)  # 1.8 cm onto the blade: 24 g of steel rod and 18 g of steel collar outweigh 26 g of wood
GRIP_U = -0.085  # where the demo holds the handle (u < 0: on the handle side)

# -- the pieces: blocks with a hole along their y axis (frame at their centre). The hole is not a cavity in the
# simulation (the gripper's finger hulls, pinching a hollow block, end up inside it): each piece is solid, the hole's
# mouths are printed on its two ends, the skewer and the pieces never collide, and a joint stands in for the blade in
# the hole (below) ------------------------------------------------------------------------------------------------------
PIECE = (0.030, 0.026, 0.030)  # x (across the grip), y (along the hole), z
HOLE = (0.012, 0.016)  # the hole's mouth, 12 mm wide and 16 mm tall, round a 4 mm blade
PIECE_MASS = 0.010
# A piece goes onto the skewer when the skewer's point is at the mouth of its hole (between 4 mm short of the end face
# and 8 mm into the hole) and inside its outline, the two within 15 degrees of each other. A joint then keeps
# the piece on the blade's axis, free to turn about it and slide along it against a drag (food grips a skewer), from
# the collar out to the point; pushed or pulled past the point, it comes off.
ENGAGE_TILT, ENGAGE_MOUTH = math.radians(15.0), 0.004
SLIDE_DRAG, TURN_DRAG = 3.0, 0.02  # N s/m along the blade, N m s/rad about it
PLAY, TILT_DEG = 0.003, 12.0  # the hole's play round the blade: 3 mm off its axis, 12 degrees of tilt
MEAT, PEPPER, ONION = (0.45, 0.22, 0.14), (0.20, 0.55, 0.15), (0.90, 0.86, 0.72)
PIECE_COLORS = (MEAT, PEPPER, MEAT, ONION)
BOARD_POS, BOARD_SIZE = (0.38, -0.18, TOP), (0.07, 0.17, 0.012)
PIECE_XY = ((0.38, -0.12), (0.38, -0.16), (0.38, -0.20), (0.38, -0.24))  # in a row, 4 cm apart

# -- the grill (frame at (X0, 0, TOP)): a tray whose two side walls run front to back; the skewer lies across them ----
X0, Y_C = 0.25, 0.10  # the skewer's line (x) and its collar (y) at the start
WALL_H = 0.050  # the right wall's top: the blade lies on it
L_WALL = (Y_C + 0.020, Y_C + 0.045)  # the left wall (y from, to): under the handle, between collar and grip
R_WALL = (Y_C - TIP_U + 0.001, Y_C - 0.200)  # the right wall: under the blade's last 4 cm, the point flush with its face
L_WALL_H = WALL_H + ROD_R - HANDLE_H / 2  # lower, so that the skewer lies level (the handle is thicker than the blade)
GRILL_X = 0.07  # the walls run x0 +- 7 cm
Z_AX = TOP + WALL_H + ROD_R + 0.0005  # the skewer's axis at the start

# -- the prep stand, just past the skewer's point: a pillar with a shallow channel on top, at the height where a piece
# set in the channel has its hole at the height of the skewer's point when the skewer is held level 11 cm up --------
HOLD_Z = TOP + 0.11  # the skewer's axis, held up (its point droops 2-10 mm under its weight and the pieces')
STAND_TOP = HOLD_Z - 0.004 - PIECE[2] / 2
STAND_Y = (Y_C - TIP_U - 0.003, Y_C - TIP_U - 0.050)  # from 3 mm past the point, 4.7 cm long
STAND_W, RAIL_H, RAIL_T = 0.034, 0.004, 0.004  # the channel 34 mm wide (2 mm round a piece), its rails 4 mm high

STEEL, WOOD, GRILL_C, COAL, EMBER, BOARD_C, STAND_C = (0.70, 0.71, 0.74), (0.55, 0.36, 0.20), (0.20, 0.20, 0.22), \
    (0.12, 0.12, 0.12), (0.75, 0.30, 0.10), (0.80, 0.68, 0.50), (0.85, 0.85, 0.88)

# -- scoring ----------------------------------------------------------------------------------------------------------
ON_RADIAL = 0.008  # a piece's centre within 8 mm of the blade's axis, along the blade: the blade goes through its hole
LEVEL_TILT = math.radians(10.0)
ABOVE_WALL = 0.025  # lying across a wall: the axis no more than 2.5 cm above its top
WALL_MARGIN = 0.015
LET_GO = 0.04  # both grippers' TCPs at least this far from the skewer's axis and from every piece's centre
SUCCESS_HOLD_S = 1.0


def _skewer() -> Body:
    shapes = [BoxShape((0.0, COLLAR_T + HANDLE_L / 2, 0.0), (HANDLE_W, HANDLE_L, HANDLE_H), WOOD),
              BoxShape((0.0, COLLAR_T / 2, 0.0), (COLLAR_W, COLLAR_T, COLLAR_W), STEEL),
              CylShape((0.0, -ROD_L / 2, 0.0), ROD_R, ROD_L, STEEL, axis="Y")]
    u = ROD_L
    for r, length in TAPER:
        shapes.append(CylShape((0.0, -(u + length / 2), 0.0), r, length, STEEL, axis="Y"))
        u += length
    shapes.append(BoxShape((0.0, COLLAR_T + HANDLE_L - 0.004, HANDLE_H / 2 + 0.0002), (0.010, 0.003, 0.0004),
                           (0.25, 0.15, 0.08), collide=False))  # a burnt mark near the handle's end
    return Body.build("skewer", pos=(X0, Y_C, Z_AX), mass=SKEWER_MASS, shapes=shapes, friction=(1.2, 1.0), color=STEEL,
                      com=SKEWER_COM)


def _piece(i: int) -> Body:
    sx, sy, sz = PIECE
    c = PIECE_COLORS[i]
    dark = tuple(0.35 * v for v in c)
    shapes = [BoxShape((0.0, 0.0, 0.0), PIECE, c)]
    shapes += [BoxShape((0.0, s * (sy / 2 + 0.0002), 0.0), (HOLE[0], 0.0004, HOLE[1]), dark, collide=False)
               for s in (-1.0, 1.0)]  # the hole's mouths
    x, y = PIECE_XY[i]
    return Body.build(f"piece{i + 1}", pos=(x, y, TOP + BOARD_SIZE[2] + sz / 2 + 0.0005), mass=PIECE_MASS, shapes=shapes,
                      friction=(1.2, 1.0), color=c, ignore=("skewer",))  # grippy: the jaws' V would push it out


def _grill() -> Body:
    def box(x0, x1, y0, y1, z0, z1, color=GRILL_C, collide=True):
        return BoxShape(((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2), (x1 - x0, y1 - y0, z1 - z0), color,
                        collide=collide)

    g, t = GRILL_X, 0.006
    shapes = [box(-g, g, R_WALL[0], L_WALL[1], 0.0005, 0.0045),  # the floor
              box(-g, g, *L_WALL, 0.0005, L_WALL_H), box(-g, g, *R_WALL, 0.0005, WALL_H),  # the side walls
              box(-g, -g + t, R_WALL[1], L_WALL[0], 0.0005, 0.030),  # low back and front walls
              box(g - t, g, R_WALL[1], L_WALL[0], 0.0005, 0.030)]
    rng = np.random.default_rng(3)  # charcoal on the floor (looks only)
    for _ in range(14):
        x, y = rng.uniform(-g + 0.015, g - 0.015), rng.uniform(R_WALL[1] + 0.012, L_WALL[0] - 0.012)
        s = rng.uniform(0.012, 0.020)
        shapes.append(BoxShape((x, y, 0.0045 + s / 3), (s, s * 0.8, s * 0.6), EMBER if rng.random() < 0.3 else COAL,
                               yaw=float(rng.uniform(0, math.pi)), collide=False))
    return Body.build("grill", pos=(X0, 0.0, TOP), mass=1.0, shapes=shapes, kinematic=True, friction=(0.6, 0.5),
                      color=GRILL_C)


def _board() -> Body:
    sx, sy, sz = BOARD_SIZE
    return Body.build("board", pos=BOARD_POS, mass=0.3, shapes=[BoxShape((0.0, 0.0, sz / 2 + 0.0005), BOARD_SIZE,
                                                                         BOARD_C)], kinematic=True, color=BOARD_C)


def _stand() -> Body:
    """The prep stand: a pillar, frame at its foot, with a channel along y on top."""
    y0, y1 = STAND_Y[1] - Y_C, STAND_Y[0] - Y_C
    h, w = STAND_TOP - TOP, STAND_W / 2 + RAIL_T
    shapes = [BoxShape((0.0, (y0 + y1) / 2, h / 2 + 0.0005), (2 * w, y1 - y0, h), STAND_C)]
    shapes += [BoxShape((s * (STAND_W + RAIL_T) / 2, (y0 + y1) / 2, h + RAIL_H / 2 + 0.0005), (RAIL_T, y1 - y0, RAIL_H),
                        STAND_C) for s in (-1.0, 1.0)]
    return Body.build("stand", pos=(X0, Y_C, TOP), mass=1.0, shapes=shapes, kinematic=True, friction=(0.02, 0.02),
                      slick=True, color=STAND_C)  # polished steel: a piece slides off it without tipping over


GRILL, BOARD, STAND, SKEWER = _grill(), _board(), _stand(), _skewer()
PIECES = [_piece(i) for i in range(4)]
OBJECTS = [GRILL, BOARD, STAND, SKEWER, *PIECES]
NAMES = [p.name for p in PIECES]
U_MIN, U_OFF = COLLAR_T * 0 + PIECE[1] / 2, TIP_U + PIECE[1] / 2  # a piece's centre on the blade: against the collar; off
for _n in NAMES:  # the skewer through the piece's hole, the piece either way round
    for _way, _axis in (("fwd", (0.0, -1.0, 0.0)), ("rev", (0.0, 1.0, 0.0))):
        thread(f"{_n}_{_way}", "skewer", _n, anchor0=(0.0, 0.0, 0.0), axis0=(0.0, -1.0, 0.0), anchor1=(0.0, 0.0, 0.0),
               axis1=_axis, slide_limits=(U_MIN, U_OFF + 0.01), turn_damping=TURN_DRAG, slide_stiffness=0.0,
               slide_damping=SLIDE_DRAG, slide_max_force=1.0e3, play=PLAY, tilt_deg=TILT_DEG)


# -- the pieces on the skewer and the grippers, followed after every control step ----------------------------------------

_EP = {"tcps": None, "on": {}}
_DEBUG = {"on": bool(os.environ.get("KEBAB_DEBUG")), "steps": 0}


@on_reset
def _new_episode() -> None:
    _EP.update(tcps=None, on={})
    _DEBUG["steps"] = 0


def _skewered(base, origin) -> None:
    """Put a piece on the skewer when the skewer's point is at the mouth of its hole, lined up; take it off past the
    point."""
    s = as_numpy(base.scene["skewer"].data.root_link_pose_w)[0]
    sp, a = s[:3] - origin, rotation(s[3:7]) @ np.array([0.0, -1.0, 0.0])
    for n in NAMES:
        p = as_numpy(base.scene[n].data.root_link_pose_w)[0]
        d = p[:3] - origin - sp
        u = float(d @ a)
        joint = _EP["on"].get(n)
        if joint is not None:
            if u > U_OFF + 2 * ENGAGE_MOUTH:  # pulled off past the point
                set_thread_on(joint, False)
                _EP["on"][n] = None
            else:  # the slide drive only drags: its target follows the piece
                set_slide_target(joint, min(max(u, U_MIN), U_OFF + 0.01))
            continue
        cos = float((rotation(p[3:7]) @ np.array([0.0, 1.0, 0.0])) @ a)
        side = d - u * a  # the point's offset from the hole's axis: across (horizontal) and up
        across = float(np.linalg.norm(side - (side @ np.array([0.0, 0.0, 1.0])) * np.array([0.0, 0.0, 1.0])))
        if (abs(cos) >= math.cos(ENGAGE_TILT) and across <= HOLE[0] / 2 and abs(float(side[2])) <= HOLE[1] / 2
                and U_OFF - 2 * ENGAGE_MOUTH <= u <= U_OFF + ENGAGE_MOUTH):
            joint = f"{n}_{'fwd' if cos < 0 else 'rev'}"
            set_slide_target(joint, u)
            set_thread_on(joint, True)
            _EP["on"][n] = joint


def _tcps(base, origin) -> list[np.ndarray]:
    out = []
    for arm in ("left_robot", "right_robot"):
        robot = base.scene[arm]
        i = next(j for j, n in enumerate(robot.body_names) if n == "gripper" or n.endswith("/gripper"))
        g = as_numpy(robot.data.body_link_pose_w)[0][i]
        out.append(g[:3] - origin + rotation(g[3:7]) @ np.array(TCP_OFFSET))
    return out


@on_step
def _follow(env) -> None:
    base = env.unwrapped
    origin = as_numpy(base.scene.env_origins[0])
    _EP["tcps"] = _tcps(base, origin)
    _skewered(base, origin)
    if not _DEBUG["on"]:
        return
    _DEBUG["steps"] += 1
    if _DEBUG["steps"] % 10:
        return
    pose = {}
    for name in ("skewer", *NAMES):
        p = as_numpy(base.scene[name].data.root_link_pose_w)[0]
        pose[name] = (p[:3] - origin, rotation(p[3:7]))
    sp, sr = pose["skewer"]
    st = _status_from(sp, sr, {n: pose[n][0] for n in NAMES})
    tl, tr = _EP["tcps"]
    tip = sp + sr @ np.array([0.0, -TIP_U, 0.0])
    grip = sp + sr @ np.array([0.0, -GRIP_U, 0.0])
    near = min(NAMES, key=lambda n: float(np.linalg.norm(pose[n][0] - tr)))  # the piece nearest the right TCP
    pc = pose[near][0]
    u = float((pc - sp) @ st["axis"])
    off = pc - (sp + u * st["axis"])  # that piece's centre from the blade's axis
    print(f"[kebab] t={_DEBUG['steps'] / 20:5.1f}s skewer axis={np.round(st['axis'], 3).tolist()} "
          f"tip={np.round(tip * 1000).astype(int).tolist()} left TCP-grip={np.round((tl - grip) * 1000).astype(int).tolist()} "
          f"right TCP={np.round(tr * 1000).astype(int).tolist()} TCP-{near}={np.round((tr - pc) * 1000, 1).tolist()} "
          f"{near} off axis={np.round(off * 1000, 1).tolist()} pieces(u,r mm)="
          f"{[(round(u * 1000), round(r * 1000, 1)) for u, r in st['ur']]} on={st['on']} across={st['across']} "
          f"let_go={st['let_go']} joints={[_EP['on'].get(n) for n in NAMES]} score={_points(st)}", file=sys.stderr, flush=True)


# -- scoring ----------------------------------------------------------------------------------------------------------


def _axis_point_at_y(sp, a, y):
    """Where the skewer's axis crosses the plane at height-independent ``y`` (None if it runs along it)."""
    if abs(a[1]) < 1e-6:
        return None
    return sp + ((y - sp[1]) / a[1]) * a


def _status_from(sp, sr, piece_pos: dict) -> dict:
    sp = np.asarray(sp, float)
    a = sr @ np.array([0.0, -1.0, 0.0])  # the skewer's axis, collar to point
    ur, on = [], 0
    for n in NAMES:
        d = np.asarray(piece_pos[n], float) - sp
        u = float(d @ a)
        r = float(np.linalg.norm(d - u * a))
        ur.append((u, r))
        on += int(bool(_EP["on"].get(n)) and r <= ON_RADIAL and 0.0 <= u <= TIP_U)
    across = False
    if abs(a[2]) <= math.sin(LEVEL_TILT) and abs(a[1]) >= math.cos(math.radians(30.0)):
        across = True
        for (y0, y1), top in ((L_WALL, TOP + L_WALL_H), (R_WALL, TOP + WALL_H)):
            p = _axis_point_at_y(sp, a, (y0 + y1) / 2)
            across &= (p is not None and abs(p[0] - X0) <= GRILL_X + WALL_MARGIN
                       and top - 0.005 <= p[2] <= top + ABOVE_WALL)
        lo, hi = R_WALL[1], L_WALL[0]  # the pieces hang between the walls
        across &= all(lo < float(piece_pos[n][1]) < hi for n in NAMES)
    tcps = _EP["tcps"]
    let_go = True
    if tcps is not None:
        for t in tcps:
            d = np.asarray(t, float) - sp
            u = float(np.clip(d @ a, -(COLLAR_T + HANDLE_L), TIP_U))
            let_go &= float(np.linalg.norm(d - u * a)) >= LET_GO
            let_go &= all(float(np.linalg.norm(np.asarray(t, float) - np.asarray(piece_pos[n], float))) >= LET_GO
                          for n in NAMES)
    return {"axis": a, "ur": ur, "on": on, "across": across, "let_go": let_go}


def _status(objects) -> dict:
    s = objects["skewer"]
    return _status_from(s.pos, rotation(s.quat_xyzw), {n: objects[n].pos for n in NAMES})


def _points(st) -> float:
    if st["on"] == 4 and st["across"] and st["let_go"]:
        return 1.0
    if st["on"] == 4:
        return 0.5
    return 0.25 if st["on"] >= 1 else 0.0


def score(objects) -> float:
    """1.0 with all four pieces on the skewer, the skewer lying across the grill's two side walls (within 10 degrees of
    level, no more than 2.5 cm above either wall) with the pieces between the walls, and both grippers let go; 0.5 with
    all four pieces on the skewer; 0.25 with at least one; else 0.0."""
    return _points(_status(objects))


def success(objects) -> bool:
    """Full score with the skewer and every piece still."""
    return _points(_status(objects)) >= 1.0 and objects["skewer"].resting() and all(objects[n].resting()
                                                                                    for n in NAMES)


# -- scripted demo ----------------------------------------------------------------------------------------------------
# The left hand comes straight down on the skewer's handle, fingers closing front and back, and lifts the skewer level
# off the grill to the height of the prep stand, its point at the stand's channel. The right hand takes each piece in
# turn straight down off the board, fingers closing front and back, sets it in the channel with its hole toward the
# point and lets go; then, fingers half open round the blade, it pushes the piece from behind onto the point and up the
# skewer toward the handle, next to the piece before, and goes back for the next. Then the left hand lays the skewer
# back on the grill, lets go and backs away.


def _rz(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def _rx(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


DOWN_L, DOWN_R = grip_rotation(-math.pi / 2), grip_rotation(math.pi / 2)  # straight down, fingers closing along x
L_YAW, R_YAW = -0.052, -0.100  # turns (rad, about the vertical) against how the grips hold the parts askew
L_FIX = np.array([0.0, 0.0, 0.0])  # where the left hand misses by, holding the skewer (sag): aim the other way
R_FIX = np.array([-0.0015, 0.0, -0.004])  # ... and the right hand, holding a piece (the held skewer droops 3.5 mm)
Z_HOLD = HOLD_Z
PICK_IN_HAND = 0.003  # the right TCP this far above a piece's centre to take it (18 mm above its underside)
PIECE_IN_HAND = 0.003  # ... and once lifted
FIRST_U, STEP_U = 0.075, 0.031  # where the pieces' centres end up along the blade (5 mm apart)
PLACE_U = Y_C - (STAND_Y[0] + STAND_Y[1]) / 2  # a piece set in the stand's channel: its centre this far along the line
LEAD_S, LEAD_STEPS = 3.0, 15
PICK = [1.0, 0.8, 0.3]  # down, grip, settle
LIFT_L, HOLD_SETTLE = 1.5, 0.5
UP_R, CARRY_R, SET_R, OPEN_R, CLEAR_R, BEHIND_R, PUSH_R, BACK_R = 0.8, 2.0, 0.8, 0.4, 0.5, 0.6, 2.5, 1.8
FORK, FORK_U, FORK_Z = -0.012, 0.030, 0.006  # pushing from behind: fingers 24 mm apart round the blade, the TCP 3 cm
# past the piece's centre (the finger hulls reach 16 mm ahead of it), 6 mm above the blade
LOWER_L, OPEN_L, AWAY_L = 1.5, 0.6, 1.2
DROOP = (0.0, 0.011, 0.019, 0.027, 0.035)  # rad: how much more the held skewer droops with 0..4 pieces on than bare
TILT_S = 0.8
ABOVE = 0.05


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


def _on_blade(u: float) -> np.ndarray:
    """The right TCP with a piece's centre ``u`` along the held skewer's blade."""
    return np.array([X0, Y_C - u, Z_HOLD + PIECE_IN_HAND]) + R_FIX


def _fork(u: float, up: float = 0.0) -> np.ndarray:
    """The right TCP with its fingers, half open round the blade, ``u`` along it (``up`` higher)."""
    return np.array([X0 + R_FIX[0], Y_C - u, Z_HOLD + R_FIX[2] + FORK_Z + up])


def _right() -> tuple[list[Waypoint], float]:
    rot = _rz(R_YAW) @ DOWN_R
    out = []
    for k, piece in enumerate(PIECES):
        pick = np.array(piece.pos) + [0.0, 0.0, PICK_IN_HAND]
        if k == 0:
            out += _lead_in("right", pick + [0.0, 0.0, ABOVE], rot, "over the first piece")
        end = FIRST_U + STEP_U * k
        behind = PLACE_U + FORK_U + 0.012
        out += [_w(PICK[0], pick, FINGER_OPEN, f"down round piece {k + 1}", rot),
                _w(PICK[1], pick, FINGER_CLOSED, "grip it", rot),
                _w(PICK[2], pick, FINGER_CLOSED, "settle", rot),
                _w(UP_R, pick + [0.0, 0.0, ABOVE], FINGER_CLOSED, "lift it", rot),
                _w(CARRY_R, _on_blade(PLACE_U) + [0.0, 0.0, 0.03], FINGER_CLOSED, "over the stand", rot),
                _w(SET_R, _on_blade(PLACE_U) + [0.0, 0.0, 0.001], FINGER_CLOSED, "set it in the channel, hole to the point",
                   rot),
                _w(OPEN_R, _on_blade(PLACE_U) + [0.0, 0.0, 0.001], FINGER_OPEN, "let go", rot),
                _w(CLEAR_R, _on_blade(PLACE_U) + [0.0, 0.0, 0.04], FINGER_OPEN, "up off it", rot),
                _w(BEHIND_R, _fork(behind, 0.035), FORK, "behind it, fingers half open", rot),
                _w(CLEAR_R, _fork(behind), FORK, "down behind it", rot),
                _w(PUSH_R, _fork(end + FORK_U), FORK, "push it onto the point and up the skewer", rot),
                _w(CLEAR_R, _fork(end + FORK_U + 0.01, 0.04), FORK, "up and away", rot)]
        if k < len(PIECES) - 1:
            nxt = np.array(PIECES[k + 1].pos) + [0.0, 0.0, PICK_IN_HAND + ABOVE]
            out.append(_w(BACK_R, nxt, FINGER_OPEN, "over the next piece", rot))
    park = np.array([X0 + 0.05, -0.20, Z_HOLD + 0.04])
    out.append(_w(BACK_R, park, FINGER_OPEN, "out of the way", rot))
    return out, sum(w.duration for w in out)


RIGHT, RIGHT_S = _right()


def _left() -> list[Waypoint]:
    rot = _rz(L_YAW) @ DOWN_L  # once lifted: the grip holds the skewer turned by -L_YAW
    pick = np.array([X0, Y_C - GRIP_U, Z_AX])
    hold = np.array([X0, Y_C - GRIP_U, Z_HOLD]) + L_FIX
    out = _lead_in("left", pick + [0.0, 0.0, ABOVE], DOWN_L, "over the skewer's handle")
    out += [_w(PICK[0], pick, FINGER_OPEN, "down round the handle", DOWN_L),
            _w(PICK[1], pick, FINGER_CLOSED, "grip it", DOWN_L),
            _w(PICK[2], pick, FINGER_CLOSED, "settle", DOWN_L),
            _w(LIFT_L, hold, FINGER_CLOSED, "lift the skewer level off the grill", rot)]
    t = LEAD_S + sum(PICK) + LIFT_L
    sets, clock = [], 0.0  # when the right hand sets each piece on the stand
    for w in RIGHT:
        if w.label.startswith("set it in the channel"):
            sets.append(clock)
        clock += w.duration
    for k, when in enumerate(sets[1:], start=1):  # before each piece, tip the skewer up against the droop so far
        out += [_w(when - TILT_S - t, hold, FINGER_CLOSED, "hold it up while the pieces go on", _tilt(k - 1)),
                _w(TILT_S, hold, FINGER_CLOSED, "tip it up a little against its droop", _tilt(k))]
        t = when
    out += [_w(max(RIGHT_S - t, 0.5) + HOLD_SETTLE, hold, FINGER_CLOSED, "hold it up while the pieces go on", _tilt(3)),
            _w(TILT_S, hold, FINGER_CLOSED, "tip it up against the last piece's weight", _tilt(4)),
            _w(LOWER_L, pick + [0.0, 0.0, 0.002], FINGER_CLOSED, "lay it back across the grill", _tilt(4)),
            _w(OPEN_L, pick + [0.0, 0.0, 0.002], FINGER_OPEN, "let go", _tilt(4)),
            _w(AWAY_L, pick + [0.0, 0.0, 0.08], FINGER_OPEN, "back away", _tilt(4)),
            _w(4.0, pick + [0.0, 0.0, 0.08], FINGER_OPEN, "wait", _tilt(4))]
    return out


def _tilt(k: int) -> np.ndarray:
    """The left hand's grip with ``k`` pieces on: tipped up by the extra droop they cause."""
    return _rx(-DROOP[k]) @ _rz(L_YAW) @ DOWN_L


def demo():
    right = list(RIGHT) + [_w(LOWER_L + OPEN_L + AWAY_L + 4.0 + HOLD_SETTLE, RIGHT[-1].pos, FINGER_OPEN, "wait",
                              RIGHT[-1].yaw)]
    return {"left": _left(), "right": right}
