"""On the station table: an ordinary quartz three-hand wristwatch with a date window (38 mm steel case, like a Timex
Easy Reader Date 38 mm) lies unbuckled on a short stack of hardcover books, with its crown over the edge. Its hands
read 9:46. Task (instruction): set the watch to 10:10 without changing its date, and leave its crown pushed fully in.

On a two-click date watch, the middle crown position adjusts the date and the outer position adjusts the hands.
Push the crown fully in when setting is complete.

The hand skill: the crown is 5.5 mm across and stands 4 mm out from the case, so fingertips can only take it by its
end, 0.6 mm or so off the case; it is drawn out past two clicks 0.9 mm apart (the first is the date position: turning
the crown there changes the date); it is turned two full turns, in strokes because a wrist turns only so far, and must
stop within 30 deg of the crown (one minute of the minute hand); then it is pushed back in past both clicks.

World frame (m): x forward from the arm bases, y to the robot's left, z up; the table top is at z 0.7475.
- books: three 260 x 150 x 45 mm hardcovers, stacked flush, with their common near edge at y=-0.0185 and centre x=0.28.
- watch: loose, dial up, centred over (0.28, 0.00), 12 o'clock toward +x and 3 o'clock (the crown) toward -y.
  Case 38 mm across and 9.5 mm thick, its back 13.5 cm above the table; white dial with hour
  marks, a date window at 3 o'clock, black hour and minute hands, a red seconds hand (the battery is out, so the hands
  move only when the crown turns them); 20 mm leather strap. The crown: 5.5 mm across, 4 mm long, its axis 4.7 mm
  above the case back (14.0 cm above the table), its inner face 0.5 mm off the case when home.
  Crown positions: home, 1st click 0.9 mm out (date), 2nd click 1.8 mm out (time; it stops there). A spring pulls
  the stem toward its nearest click (at most 1.5 N, half way between). In the 2nd click one turn of the crown moves
  the minute hand 12 minutes (forward when the crown's top turns toward 12 o'clock) and the hour hand with it; in the
  1st click half a turn of the crown changes the date; home, the crown turns freely and moves nothing. The crown
  turns against 1.5 mN m of friction.

Score at the end of the run ("the hands read" refers to the time on the 12-hour dial):
- 0.25: the crown has been out at its 2nd click (the time position) at some time.
- 0.5: the hands read within 5 minutes of 10:10.
- 0.75: the hands read within 1 minute of 10:10 and the date has not been changed.
- 1.0: as 0.75, with the crown fully home, the watch resting dial up on the top book, and all fingers at least 1 cm
  clear. Maintain these conditions for one second, with case movement of at most 1 mm.
Once the date has been changed (half a turn of the crown in the 1st click), the score is at most 0.5.
"""

from __future__ import annotations

import itertools
import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

from bodykit import Body, BoxShape, CylShape, Link, Linkage, allow_matrix_grips  # noqa: E402
from bodykit import as_numpy, on_reset, on_step, rotation  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

allow_matrix_grips()

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 0.0  # The scorer tracks the full release interval before reporting success.
RELEASE_HOLD_S = 1.0
SCORE_OVER = "end"
_E = os.environ.get

# -- the layout (world, m) -------------------------------------------------------------------------------------------
W = np.array([0.28, 0.0])  # the watch's centre (x, y); 12 o'clock toward +x, 3 o'clock (the crown) toward -y
CASE_R, CASE_H = 0.019, 0.0095  # 38 mm case, 9.5 mm thick, dial up
DIAL_R = 0.0160
BOOK_LENGTH, BOOK_WIDTH, BOOK_HEIGHT = 0.260, 0.150, 0.045
BOOK_EDGE_Y = -0.0185
Z0 = 3 * BOOK_HEIGHT  # caseback on the top book (above the table)
CZ = 0.0047  # the crown's axis above the caseback
CROWN_R, CROWN_L = 0.00275, 0.0040  # 5.5 mm crown, 4 mm long
CROWN_Y0 = -(CASE_R + 0.0005)  # the crown's inner face (pushed home), from the watch's centre
CLICK = float(_E("WATCH_CLICK", "0.0009"))  # crown travel per click (m)
K_DET = float(_E("WATCH_KDET", "3300"))  # N/m: the stem's detent spring toward the nearest click (peak 1.5 N)
D_STEM = float(_E("WATCH_DSTEM", "2.0"))  # N s/m
CROWN_FRIC = float(_E("WATCH_CFRIC", "0.0015"))  # N m
MIN_PER_TURN = float(_E("WATCH_MPT", "12.0"))  # minutes of the minute hand per crown turn in the 2nd click
DATE_TURN = math.pi  # crown turn in the 1st click that moves the date
T0 = 9 * 60 + 46.0  # the hands at the start: 9:46
TARGET = 10 * 60 + 10.0  # 10:10
TOL_FULL, TOL_PART = 1.0, 5.0  # minutes

STEEL, DARK, WHITE, BLACK, RED, BROWN = ((0.75, 0.76, 0.78), (0.15, 0.15, 0.16), (0.96, 0.96, 0.94),
                                       (0.05, 0.05, 0.06), (0.85, 0.1, 0.1), (0.42, 0.25, 0.13))


def _v(p) -> tuple:
    return tuple(float(x) for x in p)


def _hand(length, width, tail, z, color, minutes, per_rev) -> list:
    """A hand from the centre, pointing at the time ``minutes`` on a dial where it turns once per ``per_rev``."""
    a = 2 * math.pi * (minutes % per_rev) / per_rev  # clockwise from 12 o'clock (+x), seen from above
    d = np.array([math.cos(a), -math.sin(a)])
    c = d * (length - tail) / 2
    return [BoxShape((float(c[0]), float(c[1]), z), (length + tail, width, 0.0004), color, yaw=-a, collide=False)]


def _watch() -> Linkage:
    z_dial = CASE_H + 0.0002
    base = [CylShape((0.0, 0.0, CASE_H / 2), CASE_R, CASE_H, STEEL),
            CylShape((0.0, 0.0, z_dial), DIAL_R + 0.0012, 0.0003, DARK, collide=False),  # bezel ring
            CylShape((0.0, 0.0, z_dial + 0.0001), DIAL_R, 0.0003, WHITE, collide=False),  # dial
            CylShape((0.0, CROWN_Y0 + 0.0008, CZ), 0.0016, 0.0016, STEEL, axis="Y")]  # crown tube
    for h in range(12):  # hour markers
        a = 2 * math.pi * h / 12
        d = np.array([math.cos(a), -math.sin(a)])
        r, ln = (0.0135, 0.004) if h % 3 == 0 else (0.0140, 0.0025)
        c = d * r
        base.append(BoxShape((float(c[0]), float(c[1]), z_dial + 0.0003), (ln, 0.0012 if h % 3 == 0 else 0.0008,
                                                                           0.0002), BLACK, yaw=-a, collide=False))
    # Outline of the date window at 3 o'clock.
    base.append(BoxShape((0.0, -0.0105, z_dial + 0.0003), (0.0035, 0.0040, 0.0002), WHITE, collide=False))
    base.append(BoxShape((0.0, -0.0105, z_dial + 0.00025), (0.0043, 0.0048, 0.0002), BLACK, collide=False))
    # Lugs and the unbuckled 20 mm leather strap, spread out along the length of the case.
    for sx in (1, -1):
        for sy in (1, -1):
            base.append(BoxShape((sx * 0.0205, sy * 0.0115, 0.0055), (0.0060, 0.0025, 0.0050), STEEL))
        length = 0.105 if sx == 1 else 0.067
        base.append(BoxShape((sx * 0.022, 0.0, 0.005), (0.006, 0.020, 0.003), BROWN, collide=False))
        base.append(BoxShape((sx * (0.024 + length / 2), 0.0, 0.0015),
                             (length, 0.020, 0.003), BROWN, collide=False))
    base.append(BoxShape((-0.092, 0.0, 0.0025), (0.010, 0.024, 0.003), STEEL, collide=False))
    a0 = CROWN_Y0 - 0.0005
    stem = [CylShape((0.0, a0 + 0.002, CZ), 0.0004, 0.004, STEEL, collide=False, axis="Y")]
    cy = CROWN_Y0 - CROWN_L / 2
    crown = [CylShape((0.0, cy, CZ), CROWN_R, CROWN_L, STEEL, axis="Y"),
             # a dark groove line along the crown, so its turning shows
             BoxShape((0.0, cy, CZ + CROWN_R + 0.0001), (0.0008, CROWN_L - 0.0006, 0.0004), BLACK, collide=False)]
    zh = z_dial + 0.0006
    hour = _hand(0.0095, 0.0018, 0.002, zh, BLACK, T0, 720.0)
    minute = _hand(0.0145, 0.0012, 0.002, zh + 0.0004, BLACK, T0, 60.0)
    second = _hand(0.0150, 0.0005, 0.003, zh + 0.0008, RED, 0.0, 60.0)
    base += second + [CylShape((0.0, 0.0, zh + 0.0010), 0.0010, 0.0006, BLACK, collide=False)]
    o = np.array([W[0], W[1], TOP + Z0 + 0.0003])
    anchor = _v(o + [0.0, CROWN_Y0, CZ])
    links = [Link("stem", tuple(stem), mass=0.002, anchor=anchor, axis="Y", slide=True,
                  limits=(-2 * CLICK - 0.0001, 0.0001), stiffness=K_DET, spring_damping=D_STEM),
             Link("crown", tuple(crown), mass=0.0015, anchor=anchor, axis="Y", parent="stem", friction=CROWN_FRIC,
                  damping=0.00005),
             Link("hour", tuple(hour), mass=0.0005, anchor=_v(o), axis="Z", stiffness=0.05, spring_damping=0.0005),
             Link("minute", tuple(minute), mass=0.0005, anchor=_v(o), axis="Z", stiffness=0.05,
                  spring_damping=0.0005)]
    return Linkage.build("watch", pos=_v(o), mass=0.040, color=STEEL, floating=True,
                         friction=(0.6, 0.5), shapes=base, links=links, contact_offset=0.0005, solver_iterations=48)


def _books() -> list[Body]:
    covers = ((0.23, 0.34, 0.41), (0.46, 0.25, 0.18), (0.22, 0.30, 0.25))
    pages = (0.84, 0.79, 0.65)
    books = []
    for i, color in enumerate(covers):
        shapes = [
            BoxShape((0, 0, z), (BOOK_LENGTH, BOOK_WIDTH, 0.003), color)
            for z in (0.0015, BOOK_HEIGHT - 0.0015)
        ]
        shapes.extend([
            BoxShape((0, 0, BOOK_HEIGHT / 2), (BOOK_LENGTH - 0.004, BOOK_WIDTH - 0.006, BOOK_HEIGHT - 0.006), pages),
            BoxShape((0, BOOK_WIDTH / 2 - 0.002, BOOK_HEIGHT / 2), (BOOK_LENGTH, 0.004, BOOK_HEIGHT - 0.006), color),
        ])
        books.append(Body.build(f"book_{i}", pos=(float(W[0]), BOOK_EDGE_Y + BOOK_WIDTH / 2,
                                                 TOP + i * BOOK_HEIGHT), mass=1.0, color=color, fixed=True,
                                friction=(0.7, 0.6), shapes=shapes, contact_offset=0.0005))
    return books


WATCH = _watch()
OBJECTS = [*_books(), WATCH]

# -- tracking and scoring --------------------------------------------------------------------------------------------
CLEAR = 0.010
STILL_TOL = 0.001
_DEBUG = _E("WATCH_DEBUG") == "1"
_DEBUG_EVERY = int(_E("WATCH_DEBUG_EVERY", "5"))
_EP: dict = {}
_IDS: dict = {}
_ANG = np.linspace(0, 2 * math.pi, 16, endpoint=False)
# points on the watch head (its frame): the case's rim top and bottom, the dial, the crown (pulled out or not)
SAMPLES = np.array([[r * math.cos(a), r * math.sin(a), z] for a in _ANG for r, z in
                    ((CASE_R, 0.0), (CASE_R, CASE_H), (0.010, CASE_H))] + [[0.0, 0.0, CASE_H]]
                   + [[0.0, CROWN_Y0 - d, CZ + dz] for d in (0.0, 0.002, 0.004, 0.0058) for dz in (-CROWN_R, 0.0, CROWN_R)])


@on_reset
def _reset() -> None:
    _EP.update(tick=0, step=0, minutes=T0, crown_prev=None, click=0, max_click=0, date_turn=0.0,
               date_moved=False, near=1.0, stem=0.0, crown=0.0, watch_pos=None, flat=False,
               released_from=None, released_s=0.0, tcp={}, gaps={})


_reset()


def _finger_points() -> dict:
    import struct
    import xml.etree.ElementTree as ET

    from yam_arena import YAM_URDF
    from yam_arena.urdf import rpy_matrix

    root = pathlib.Path(YAM_URDF).parent
    out = {}
    for link in ET.parse(YAM_URDF).getroot().iter("link"):
        name = link.get("name")
        if name not in ("tip_left", "tip_right"):
            continue
        pts = []
        for col in link.findall("collision"):
            o = col.find("origin")
            xyz = np.array([float(v) for v in (o.get("xyz") or "0 0 0").split()]) if o is not None else np.zeros(3)
            rpy = np.array([float(v) for v in (o.get("rpy") or "0 0 0").split()]) if o is not None else np.zeros(3)
            b = (root / col.find("geometry").find("mesh").get("filename")).read_bytes()
            n = struct.unpack("<I", b[80:84])[0]
            tri = np.frombuffer(b[84:84 + n * 50], dtype=np.dtype([("n", "<3f4"), ("v", "<9f4"), ("a", "<u2")]))
            v = tri["v"].reshape(-1, 3).astype(float)
            pts.append(np.unique(np.round(v, 4), axis=0) @ rpy_matrix(rpy).T + xyz)
        out[name] = np.concatenate(pts)
    return out


@on_step
def _track(env) -> None:
    try:
        _track_inner(env)
    except Exception:
        import traceback

        print("WATCH_TRACEBACK\n" + traceback.format_exc(), flush=True)
        raise


def _track_inner(env) -> None:
    import torch
    from scipy.spatial import cKDTree

    from yam_arena import ik, station
    from yam_arena.constants import ARM_JOINTS, FINGER_JOINTS

    _EP["tick"] += 1
    base = getattr(env, "unwrapped", env)
    origin = as_numpy(base.scene.env_origins[0])
    if "fingers" not in _IDS:
        _IDS["fingers"] = _finger_points()
    fingers = {}
    for side in ("left", "right"):
        robot = base.scene[f"{side}_robot"]
        if side not in _IDS:
            _IDS[side] = list(robot.find_joints(ARM_JOINTS, preserve_order=True)[0])
            _IDS[side + "_f"] = list(robot.find_joints(FINGER_JOINTS, preserve_order=True)[0])
            _IDS[side + "_tips"] = [robot.body_names.index(n) for n in ("tip_left", "tip_right")]
        q = as_numpy(robot.data.joint_pos)[0]
        _EP["tcp"][side] = np.asarray(ik.tcp_pose(q[_IDS[side]], station.BASES[side])[0], float)
        _EP["gaps"][side] = float(-q[_IDS[side + "_f"]].sum())
        poses = as_numpy(robot.data.body_link_pose_w)[0]
        fingers[side] = np.concatenate([_IDS["fingers"][n] @ rotation(poses[i][3:7]).T + poses[i][:3] - origin
                                        for n, i in zip(("tip_left", "tip_right"), _IDS[side + "_tips"])])
    w = base.scene["watch"]
    if "stem" not in _IDS:
        jn = list(w.joint_names)
        for k in ("stem", "crown", "hour", "minute"):
            _IDS[k] = jn.index(f"{k}_turn")
    q = as_numpy(w.data.joint_pos)[0]
    stem, crown = float(q[_IDS["stem"]]), float(q[_IDS["crown"]])
    click = int(min(2, max(0, math.floor(-stem / CLICK + 0.5))))
    # the gear train: in the 2nd click the crown drives the hands; in the 1st it moves the date
    if _EP["crown_prev"] is not None:
        dc = (crown - _EP["crown_prev"] + math.pi) % (2 * math.pi) - math.pi  # the joint angle wraps at 2 turns
        if click == 2:
            _EP["minutes"] += MIN_PER_TURN * dc / (2 * math.pi)
        elif click == 1:
            _EP["date_turn"] += abs(dc)
            if _EP["date_turn"] >= DATE_TURN:
                _EP["date_moved"] = True
    _EP["crown_prev"] = crown
    if click != 1:
        _EP["date_turn"] = 0.0
    dm = _EP["minutes"] - T0
    dev = w.device
    ids = [_IDS["stem"], _IDS["hour"], _IDS["minute"]]
    tgt = [-click * CLICK, -2 * math.pi * dm / 720.0, -2 * math.pi * dm / 60.0]
    w.set_joint_position_target_index(target=torch.tensor([tgt], dtype=torch.float32, device=dev), joint_ids=ids)
    w.write_data_to_sim()
    pose = as_numpy(w.data.root_link_pose_w)[0]
    pp, pr = pose[:3] - origin, rotation(pose[3:7])
    pts = pp + SAMPLES @ pr.T
    tree = cKDTree(pts)
    near = min(float(tree.query(fp, k=1)[0].min()) for fp in fingers.values())
    flat = pr[2, 2] >= math.cos(math.radians(10))
    _EP.update(stem=stem, crown=crown, click=click, max_click=max(_EP["max_click"], click), near=near,
               watch_pos=pp, flat=flat)
    if _reading_error() > TOL_FULL or click != 0 or _EP["date_moved"] or near < CLEAR or not _on_top_book(pp, flat):
        _EP.update(released_from=None, released_s=0.0)
    elif _EP["released_from"] is None or np.linalg.norm(pp - _EP["released_from"]) > STILL_TOL:
        _EP.update(released_from=pp.copy(), released_s=0.0)
    else:
        _EP["released_s"] += float(base.step_dt)


def _reading_error() -> float:
    """Minutes between the hands' reading and 10:10 on a 12-hour dial."""
    d = (_EP["minutes"] - TARGET) % 720.0
    return min(d, 720.0 - d)


def _on_top_book(pos, flat) -> bool:
    return bool(flat and abs(pos[2] - (TOP + Z0)) < 0.002 and
                abs(pos[0] - W[0]) < BOOK_LENGTH / 2 and BOOK_EDGE_Y <= pos[1] <= BOOK_EDGE_Y + BOOK_WIDTH)


def _debug(err, done) -> None:
    _EP["step"] += 1
    if (_EP["step"] - 1) % _DEBUG_EVERY:
        return
    m = _EP["minutes"] % 720.0
    pp = _EP["watch_pos"]
    watch_delta = (pp - [W[0], W[1], TOP + Z0]) * 1000 if pp is not None else np.zeros(3)
    print(f"DBG step={_EP['step']} time={int(m // 60)}:{m % 60:05.2f} err={err:.2f}min "
          f"stem={_EP['stem'] * 1000:+.2f}mm click={_EP['click']} max={_EP['max_click']} "
          f"crown={math.degrees(_EP['crown']):+.1f}deg date_turn={math.degrees(_EP['date_turn']):.0f} "
          f"date_moved={int(_EP['date_moved'])} near={_EP['near'] * 1000:.0f}mm done={int(done)} "
          f"watch_delta={np.round(watch_delta, 2)} flat={int(_EP['flat'])} | "
          + " ".join(f"{k} tcp=({v[0]:.3f},{v[1]:.3f},{(v[2] - TOP) * 1000:.1f}mm) gap={_EP['gaps'][k] * 1000:.0f}"
                     for k, v in _EP["tcp"].items()), flush=True)


def score(objects) -> float:
    err = _reading_error()
    done = success(objects)
    if _DEBUG:
        _debug(err, done)
    best = 0.25 if _EP["max_click"] >= 2 else 0.0
    if err <= TOL_PART:
        best = 0.5
    if _EP["date_moved"]:
        return best
    if err <= TOL_FULL:
        best = 0.75
        if done:
            best = 1.0
    return best


def success(objects) -> bool:
    return _EP["released_s"] >= RELEASE_HOLD_S - 1e-9


# -- scripted demo ---------------------------------------------------------------------------------------------------
# Open loop from the known layout. The left gripper braces the loose case against the book. The right
# gripper, level and pointing along the crown's axis (+y) with its fingers closing across the crown, slides its
# fingertips either side of the crown (their ends 0.6 mm off the case) and pinches it; it draws the crown out past both
# clicks, eases off and, keeping hold, turns its wrist 110 deg forward; then
# it turns the crown in strokes, as a wrist turns only +-120 deg: let go, turn the open hand back, pinch again, turn
# forward (220, 220 and 190 deg). In all two turns of the crown: 24 minutes of the minute hand, from 9:46 to 10:10. It
# lets go, backs off 8 mm, closes its fingers and pushes the crown home with the fingertips' ends, and backs away to the
# ready pose. The left gripper then releases the case and backs away.
# WATCH_NOPULL=1 (the wrong way): the same strokes and push without pulling the crown out first. In its home position
# the crown turns freely and turns nothing, so the hands stay at 9:46.
OPEN_C = -0.047
NOPULL = _E("WATCH_NOPULL") == "1"
NOBRACE = _E("WATCH_NOBRACE") == "1"
P = {k: float(_E("WATCH_" + k.upper(), v)) for k, v in dict(
    gap_case="0.0006", open="-0.007", pull="0.0035", set_bias="0.0007", push_over="0.004",
    brace_y="0.012", brace_depth="0.010").items()}
STROKES = [float(v) for v in _E("WATCH_STROKES", "110,220,220,190").split(",")]  # crown turns per stroke (deg)
WRIST_MAX = 110.0  # deg: each stroke ends with the hand turned this far (the wrist turns +-120 deg)
CZW = TOP + Z0 + CZ  # the crown's axis (world z)
TIP_AHEAD = 0.0128  # the closed fingertips' end, ahead of the TCP along the approach axis


def _R(c1, c3) -> np.ndarray:
    c1, c3 = np.asarray(c1, float), np.asarray(c3, float)
    c1, c3 = c1 / np.linalg.norm(c1), c3 / np.linalg.norm(c3)
    return np.column_stack([c1, np.cross(c3, c1), c3])


LEVEL_Y_X = _R((1.0, 0.0, 0.0), (0.0, 1.0, 0.0))  # pointing +y, level, fingers closing along x


def _good_ik(arm: str, pos, rot):
    from yam_arena import ik, station

    best = None
    b = station.BASES[arm]
    j1 = math.atan2(float(pos[1]) - b[1], float(pos[0]) - b[0])
    for sd in itertools.product((j1,), (0.8, 1.3), (0.6, 1.2), (-0.8, 0.8), (0.0,), (-1.8, -0.9, 0.0, 0.9, 1.8)):
        q, pe, re = ik.solve(np.asarray(pos, float), ik.matrix_to_quat_xyzw(rot), np.asarray(sd, float),
                             base_pos=station.BASES[arm], iters=150)
        if pe < 0.001 and re < math.radians(0.5):
            room = float(np.minimum(q - ik.LOWER, ik.UPPER - q).min())
            if best is None or room > best[1]:
                best = (q, room)
                if room > 0.3:
                    break
    if best is None:
        raise ValueError(f"no IK solution for the {arm} arm at {pos}")
    return best[0]


def _joint_path(arm: str, q0, q1, finger: float, label: str, seconds: float, steps: int = 15) -> list:
    from yam_arena import ik, station

    out = []
    for k in range(1, steps + 1):
        p, r = ik.tcp_pose(q0 + (q1 - q0) * k / steps, station.BASES[arm])
        out.append(Waypoint(seconds / steps, tuple(float(v) for v in p), finger, label, r))
    return out


def _lead_in(arm: str, pos, rot, finger: float, label: str, seconds: float = 3.0) -> list:
    from yam_arena.constants import READY

    return _joint_path(arm, np.array(READY, float), _good_ik(arm, pos, rot), finger, label, seconds)


def _lead_out(arm: str, pos, rot, finger: float, label: str, seconds: float = 3.0) -> list:
    from yam_arena.constants import READY

    return _joint_path(arm, _good_ik(arm, pos, rot), np.array(READY, float), finger, label, seconds)


def _w(t, p, finger, label, rot) -> Waypoint:
    return Waypoint(t, tuple(float(v) for v in p), finger, label, rot)


def _ready(arm: str):
    from yam_arena import ik, station
    from yam_arena.constants import READY

    return ik.tcp_pose(np.array(READY, float), station.BASES[arm])


def _dur(wps) -> float:
    return float(sum(w.duration for w in wps))


def _tcp_for_tip(tip, rot) -> np.ndarray:
    """The TCP that puts the closed fingertips' end at ``tip``."""
    return np.asarray(tip, float) - rot[:, 2] * TIP_AHEAD


def _level(phi_deg: float) -> np.ndarray:
    """Pointing +y (along the crown's axis, toward the case), level, the fingers closing along x when ``phi_deg`` is
    0, turned ``phi_deg`` right-handed about +y (the way that turns the crown forward)."""
    t = math.radians(phi_deg)
    roty = np.array([[math.cos(t), 0.0, math.sin(t)], [0.0, 1.0, 0.0], [-math.sin(t), 0.0, math.cos(t)]])
    return roty @ LEVEL_Y_X


def _turn(p, finger, a, b, seconds, label) -> list:
    n = max(3, int(abs(b - a) / 5.0))
    return [_w(seconds / n, p, finger, label, _level(a + (b - a) * k / n)) for k in range(1, n + 1)]


def _right() -> list:
    """Coaxial with the crown: pinch it with the fingertips, draw it out, turn it in strokes, push it home."""
    a = np.array([0.0, 1.0, 0.0])
    face_home = W[1] + CROWN_Y0 - CROWN_L  # the crown's outer face, pushed home (world y)
    tip_home = np.array([W[0], W[1] + CROWN_Y0 - P["gap_case"], CZW])  # the fingertips' ends just off the case
    p_home = tip_home - a * TIP_AHEAD
    pulled = 0.0 if NOPULL else 2 * CLICK + P["set_bias"]
    p_set = p_home - a * pulled  # the TCP holding the crown at its second click
    p_back = p_home - a * 0.030
    out = _lead_in("right", p_back, _level(0.0), P["open"], "in line with the crown, fingers open")
    out += [_w(2.0, p_home, P["open"], "fingertips either side of the crown", _level(0.0)),
            _w(1.0, p_home, 0.0, "pinch the crown", _level(0.0)),
            _w(0.5, p_home, 0.0, "settle", _level(0.0))]
    if not NOPULL:
        out += [_w(1.5, p_home - a * P["pull"], 0.0, "draw the crown out past both clicks", _level(0.0)),
                _w(0.5, p_home - a * P["pull"], 0.0, "settle", _level(0.0)),
                _w(1.0, p_set, 0.0, "ease off", _level(0.0))]
    cur = 0.0
    for k, ang in enumerate(STROKES):
        start = 0.0 if k == 0 else WRIST_MAX - ang
        if k:
            out += [_w(0.8, p_set, P["open"], "let go", _level(cur))]
            out += _turn(p_set, P["open"], cur, start, max(1.0, abs(cur - start) / 120.0), "turn the open hand back")
            out += [_w(0.8, p_set, 0.0, "pinch the crown again", _level(start)),
                    _w(0.3, p_set, 0.0, "settle", _level(start))]
        cur = start + ang
        out += _turn(p_set, 0.0, start, cur, abs(ang) / 90.0, "turn the crown forward")
        out += [_w(0.4, p_set, 0.0, "settle", _level(cur))]
    out += [_w(0.8, p_set, P["open"], "let go", _level(cur)),
            _w(0.8, p_set - a * 0.008, P["open"], "back off", _level(cur))]
    out += _turn(p_set - a * 0.008, P["open"], cur, 0.0, 1.0, "turn the open hand back")
    p_push = np.array([W[0], face_home + P["push_over"], CZW]) - a * TIP_AHEAD
    out += [_w(0.8, p_set - a * 0.008, 0.0, "close the fingers", _level(0.0)),
            _w(2.0, p_push, 0.0, "push the crown home with the fingertips' ends", _level(0.0)),
            _w(0.5, p_push, 0.0, "settle", _level(0.0)),
            _w(1.5, p_back, 0.0, "back away", _level(0.0))]
    out += _lead_out("right", p_back, _level(0.0), OPEN_C, "back to the ready pose")
    return out


def _left(hold: float) -> tuple[list, list]:
    """Brace the back half of the case with the ends of the closed fingertips."""
    rot = _R((-1, 0, 0), (0, 0, -1))
    tip = [W[0], W[1] + P["brace_y"], TOP + Z0 + CASE_H - P["brace_depth"]]
    press = _tcp_for_tip(tip, rot)
    pre = press + [0, 0, 0.025]
    if NOBRACE:
        lp, lq = _ready("left")
        prepare = [_w(6.0, lp, OPEN_C, "leave the watch loose", lq)]
        return prepare, [_w(hold, lp, OPEN_C, "stay at ready", lq)]
    prepare = _lead_in("left", pre, rot, 0.0, "above the case")
    prepare += [_w(2.0, press, 0.0, "press the back half of the case against the book", rot),
                _w(1.0, press, 0.0, "settle the brace", rot)]
    release = [_w(hold, press, 0.0, "hold the case still", rot),
               _w(1.0, pre, 0.0, "lift clear of the watch", rot)]
    release += _lead_out("left", pre, rot, OPEN_C, "back to the ready pose")
    return prepare, release


def demo():
    manipulation = _right()
    prepare, release = _left(_dur(manipulation))
    rp, rq = _ready("right")
    lp, lq = _ready("left")
    right = [_w(_dur(prepare), rp, OPEN_C, "wait for the watch to be braced", rq)] + manipulation
    right += [_w(_dur(release) - _dur(manipulation) + 4.0, rp, OPEN_C, "stay clear", rq)]
    left = prepare + release + [_w(4.0, lp, OPEN_C, "stay clear", lq)]
    return {"left": left, "right": right}
