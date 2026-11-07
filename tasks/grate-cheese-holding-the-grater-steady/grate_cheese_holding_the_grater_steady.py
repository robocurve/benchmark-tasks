"""On the station table, a box grater stands on a cutting board, its grating face toward the right arm and a flat
handle reaching left from its top; a block of cheese lies on the board to its right.

Task: grate the cheese: hold the grater by its handle with one hand, and with the other rub the cheese up and down its
grating face, pressed against it, so that the shreds fall inside the grater onto the board. The grater is light and
tall: pressed from the side, unheld, it slides away or tips over.

Run it:  inspect-robots run --embodiment yam_arena -E task=grate_cheese_holding_the_grater_steady.py \\
           --instruction "..." --scorer task_score ...
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

from bodykit import Body, BoxShape, CylShape, allow_matrix_grips, as_numpy, on_reset, on_step, rotation  # noqa: E402
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, TCP_OFFSET  # noqa: E402
from yam_arena.demo import Waypoint, grip_rotation  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

allow_matrix_grips()

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# -- the cutting board (fixed) ------------------------------------------------------------------------------------------
BOARD = (0.30, 0.34, 0.012)  # along x, along y, thick
BX, BY = 0.30, -0.01
BOARD_TOP = TOP + BOARD[2]

# -- the grater: a box of thin steel walls, open at the bottom, a top plate and a flat handle reaching left; its frame:
# origin at the middle of its foot. Its right wall (-y) is the grating face ------------------------------------------
GX, GY = 0.30, 0.0
G_X, G_Y, G_H, G_T = 0.080, 0.070, 0.130, 0.002  # across (x), across (y), tall, wall
HANDLE = (0.016, 0.100, 0.015)  # across, long (+y from the top's left edge), tall
GRATER_MASS = 0.18
FACE_Z = (0.020, 0.115)  # the grating area on the face, from the foot
STEEL, HOLE, CHEESE_C, BOARD_C, HANDLE_C = (0.78, 0.79, 0.81), (0.25, 0.25, 0.27), (0.98, 0.82, 0.30), \
    (0.80, 0.66, 0.46), (0.10, 0.10, 0.11)

# -- the cheese: a block, along y ---------------------------------------------------------------------------------------
CHEESE = (0.024, 0.060, 0.035)  # across (x), long (y), tall
CHEESE_MASS = 0.05
CX0, CY0 = 0.30, -0.14

# -- the shreds: parked out of sight; each 2 cm the cheese is rubbed along the face (pressed on it) drops one inside the
# grater, behind the face, at the cheese's height ---------------------------------------------------------------------
N_SHREDS = 12
SHRED = (0.004, 0.004, 0.012)
PER_SHRED = 0.020  # m of rubbing per shred
ON_FACE = 0.004  # the cheese's end within this of the face's outer side: rubbing on it
PARK = (0.30, 0.0, TOP + 1.5)  # a closed box high above the scene, out of the cameras' view
DRAG_K, DRAG_MAX = 60.0, 4.0  # the teeth bite: rubbing drags the grater along the stroke at the cheese (N per m/s, at
# most N); an upstroke tips an unheld grater over

# -- scoring ----------------------------------------------------------------------------------------------------------
FULL, HALF, SOME = 9, 5, 1  # shreds on the board
UPRIGHT = math.radians(20.0)
SUCCESS_HOLD_S = 1.0
HOME_WITHIN = 0.05  # each TCP this close to where it started: the arms are back home


def _board() -> Body:
    return Body.build("board", pos=(BX, BY, TOP), mass=1.0, kinematic=True, friction=(0.5, 0.4), color=BOARD_C,
                      shapes=[BoxShape((0.0, 0.0, BOARD[2] / 2), BOARD, BOARD_C)])


def _grater() -> Body:
    hx, hy = G_X / 2, G_Y / 2
    shapes = [BoxShape((s * (hx - G_T / 2), 0.0, G_H / 2), (G_T, G_Y, G_H), STEEL) for s in (-1.0, 1.0)]
    shapes += [BoxShape((0.0, s * (hy - G_T / 2), G_H / 2), (G_X - 2 * G_T, G_T, G_H), STEEL) for s in (-1.0, 1.0)]
    shapes += [BoxShape((0.0, 0.0, G_H + G_T / 2), (G_X, G_Y, G_T), STEEL),
               BoxShape((0.0, hy + HANDLE[1] / 2 - 0.004, G_H - HANDLE[2] / 2 + G_T), HANDLE, HANDLE_C)]
    for i in range(5):  # the grating's holes, drawn on the face
        for j in range(4):
            shapes.append(BoxShape((-0.027 + j * 0.018, -hy - 0.0003, FACE_Z[0] + 0.01 + i * 0.019), (0.009, 0.0004, 0.005),
                                   HOLE, collide=False))
    return Body.build("grater", pos=(GX, GY, BOARD_TOP + 0.0005), mass=GRATER_MASS, shapes=shapes, friction=(0.5, 0.4),
                      color=STEEL, com=(0.0, 0.01, 0.06))


def _cheese() -> Body:
    return Body.build("cheese", pos=(CX0, CY0, BOARD_TOP + 0.0005), mass=CHEESE_MASS, friction=(1.0, 0.9),
                      color=CHEESE_C, shapes=[BoxShape((0.0, 0.0, CHEESE[2] / 2), CHEESE, CHEESE_C)])


def _park() -> Body:
    """A closed box where the shreds wait, out of sight."""
    w, t = 0.05, 0.004
    shapes = [BoxShape((0.0, 0.0, t / 2), (w, w, t), HOLE), BoxShape((0.0, 0.0, w - t / 2), (w, w, t), HOLE)]
    shapes += [BoxShape((s * (w / 2 - t / 2), 0.0, w / 2), (t, w, w), HOLE) for s in (-1.0, 1.0)]
    shapes += [BoxShape((0.0, s * (w / 2 - t / 2), w / 2), (w, t, w), HOLE) for s in (-1.0, 1.0)]
    return Body.build("shred_store", pos=PARK, mass=1.0, shapes=shapes, kinematic=True, color=HOLE)


def _shreds() -> list[Body]:
    out = []
    for i in range(N_SHREDS):
        dx, dy = (i % 4 - 1.5) * 0.008, (i // 4 - 1.0) * 0.010
        out.append(Body.build(f"shred_{i:02d}", pos=(PARK[0] + dx, PARK[1] + dy, PARK[2] + 0.005), mass=0.0003,
                              shapes=[BoxShape((0.0, 0.0, SHRED[2] / 2), SHRED, CHEESE_C)], friction=(0.6, 0.5),
                              damping=(0.2, 0.5), color=CHEESE_C))
    return out


BOARD_B, STORE, GRATER, CHEESE_B = _board(), _park(), _grater(), _cheese()
SHREDS = _shreds()
OBJECTS = [BOARD_B, STORE, GRATER, CHEESE_B, *SHREDS]
SHRED_NAMES = [s.name for s in SHREDS]


# -- followed after every control step: the rubbing, the shreds, the grippers -------------------------------------------

def _fresh() -> dict:
    return {"rubbed": 0.0, "dropped": 0, "last_z": None, "tcps": None}


_EP = _fresh()
_DEBUG = {"on": bool(os.environ.get("GRATE_DEBUG")), "steps": 0}


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


def _on_face(g_pos, g_rot, c_pos, c_rot):
    """(rubbing?, the cheese's height on the face): the cheese's end against the grating face, within its area."""
    best = None
    for sx in (-1.0, 1.0):  # the cheese's two ends; whichever is nearer the face
        end = np.asarray(c_pos, float) + c_rot @ np.array([0.0, sx * CHEESE[1] / 2, CHEESE[2] / 2])
        v = g_rot.T @ (end - np.asarray(g_pos, float))
        gap = -G_Y / 2 - v[1]  # how far it stands off the face's outer side (negative: pressed in)
        if best is None or gap < best[0]:
            best = (gap, v)
    gap, v = best
    rubbing = gap <= ON_FACE and abs(v[0]) < G_X / 2 and FACE_Z[0] - 0.01 < v[2] < FACE_Z[1] + 0.02
    return rubbing, float(v[2])


def _drop(base, origin, g_pos, g_rot, z) -> None:
    """Teleport the next parked shred inside the grater, just behind the face, at the cheese's height."""
    import torch

    name = SHRED_NAMES[_EP["dropped"]]
    asset = base.scene[name]
    pose = as_numpy(asset.data.root_link_pose_w)[0].copy()
    jitter = ((_EP["dropped"] % 3) - 1) * 0.012
    p = g_pos + g_rot @ np.array([jitter, -G_Y / 2 + 0.010, min(max(z, FACE_Z[0]), FACE_Z[1]) - 0.01])
    pose[:3] = p + origin
    t = torch.tensor(pose, dtype=torch.float32, device=asset.device).view(1, 7)
    asset.write_root_pose_to_sim(t)
    asset.write_root_velocity_to_sim(torch.zeros(1, 6, dtype=torch.float32, device=asset.device))
    _EP["dropped"] += 1


def _drag(asset, g_rot, f_up: float, z: float) -> None:
    """Pull the grater along the face (its own z) by f_up, at the face where the cheese rubs, until the next step."""
    import torch

    f = torch.tensor([0.0, 0.0, f_up], dtype=torch.float32, device=asset.device).view(1, 1, 3)
    at = torch.tensor([0.0, -G_Y / 2, z], dtype=torch.float32, device=asset.device).view(1, 1, 3)
    asset.permanent_wrench_composer.set_forces_and_torques(forces=f, torques=torch.zeros_like(f), positions=at,
                                                           is_global=False)


@on_step
def _follow(env) -> None:
    base = env.unwrapped
    origin = as_numpy(base.scene.env_origins[0])
    g = as_numpy(base.scene["grater"].data.root_link_pose_w)[0]
    c = as_numpy(base.scene["cheese"].data.root_link_pose_w)[0]
    g_pos, g_rot, c_pos, c_rot = g[:3] - origin, rotation(g[3:7]), c[:3] - origin, rotation(c[3:7])
    _EP["tcps"] = _tcps(base, origin)
    rubbing, z = _on_face(g_pos, g_rot, c_pos, c_rot)
    upright = math.acos(max(-1.0, min(1.0, float(g_rot[2, 2])))) < UPRIGHT
    drag = 0.0
    if rubbing and _EP["last_z"] is not None:
        dz = z - _EP["last_z"]
        drag = float(np.clip(DRAG_K * dz * 20.0, -DRAG_MAX, DRAG_MAX))
        if upright:
            _EP["rubbed"] += abs(dz)
            while _EP["dropped"] < N_SHREDS and _EP["rubbed"] >= (_EP["dropped"] + 1) * PER_SHRED:
                _drop(base, origin, g_pos, g_rot, z)
    _EP["last_z"] = z if rubbing else None
    _drag(base.scene["grater"], g_rot, drag, z)
    if _DEBUG["on"]:
        _DEBUG["steps"] += 1
        if _DEBUG["steps"] % 10 == 0:
            shreds = [as_numpy(base.scene[n].data.root_link_pose_w)[0][:3] - origin for n in SHRED_NAMES]
            tl, tr = _EP["tcps"]
            print(f"[grate] t={_DEBUG['steps'] / 20:5.1f}s grater={np.round(g_pos * 1000).astype(int).tolist()} "
                  f"tilt={math.degrees(math.acos(max(-1.0, min(1.0, float(g_rot[2, 2]))))):4.1f} rubbing={rubbing} "
                  f"cheese_z_on_face={z * 1000:5.0f}mm rubbed={_EP['rubbed'] * 100:5.1f}cm dropped={_EP['dropped']} "
                  f"on_board={_count(shreds, g_rot)} left TCP={np.round(tl * 1000).astype(int).tolist()} "
                  f"right TCP-cheese={np.round((tr - c_pos) * 1000).astype(int).tolist()} score={_points(shreds, g_rot)}",
                  file=sys.stderr, flush=True)


# -- scoring --------------------------------------------------------------------------------------------------------

def _count(shreds, g_rot) -> int:
    """Shreds lying on the board (fallen out of the grater or still inside it)."""
    return int(sum(abs(p[0] - BX) < BOARD[0] / 2 and abs(p[1] - BY) < BOARD[1] / 2 and p[2] < BOARD_TOP + 0.02
                   for p in shreds))


def _points(shreds, g_rot) -> float:
    n = _count(shreds, g_rot)
    upright = math.acos(max(-1.0, min(1.0, float(g_rot[2, 2])))) < UPRIGHT
    if n >= FULL and upright:
        return 1.0
    if n >= HALF:
        return 0.5
    return 0.25 if n >= SOME else 0.0


def score(objects) -> float:
    """1.0 with at least 9 shreds of cheese on the board and the grater standing, the cheese put down and both arms back
    where they started; 0.5 with at least 5 shreds; 0.25 with at least 1; else 0.0. (A shred falls inside the grater
    for each 2 cm the cheese is rubbed up or down the grating face, pressed on it, with the grater standing.)"""
    pts = _points([objects[n].pos for n in SHRED_NAMES], rotation(objects["grater"].quat_xyzw))
    if pts >= 1.0 and not (_cheese_down(objects) and _arms_home()):
        return 0.5  # grated, but the cheese not put down or the arms not back
    return pts


_HOME: dict = {}


def _arms_home() -> bool:
    """Both arms back where they started (each TCP within HOME_WITHIN of its start)."""
    from yam_arena import ik, station
    from yam_arena.constants import READY

    tcps = _EP.get("tcps")
    if tcps is None:
        return False
    if "tcps" not in _HOME:
        _HOME["tcps"] = [np.asarray(ik.tcp_pose(np.array(READY, float), station.BASES[a])[0], float)
                         for a in ("left", "right")]
    return all(float(np.linalg.norm(t - h)) < HOME_WITHIN for t, h in zip(tcps, _HOME["tcps"]))


def _cheese_down(objects) -> bool:
    """The cheese put down: lying on the board or the table (its middle no higher than its length's half above the
    board), not held."""
    return float(objects["cheese"].pos[2]) < BOARD_TOP + CHEESE[1] / 2


def success(objects) -> bool:
    """Full score with the shreds, the grater and the cheese still: the trial ends there."""
    return score(objects) >= 1.0 and all(objects[n].resting() for n in SHRED_NAMES[:FULL] + ["grater", "cheese"])


# -- scripted demo ----------------------------------------------------------------------------------------------------
# The left hand comes straight down on the grater's handle near its end and holds it. The right hand comes straight
# down on the cheese, near its right end, lifts it and presses its left end against the grating face, then rubs it up
# and down the face a few times, and puts it back down. Then the left lets go of the grater and both arms go back to
# where they started, leaving the grated cheese on the board.

GRIP_X = grip_rotation(-math.pi / 2)  # straight down, fingers closing along x (the left hand)
GRIP_R = grip_rotation(math.pi / 2)  # the same for the right hand, turned the other way round (its wrist's range)
CHEESE_GRIP_AT = -0.015  # the right hand holds the cheese this far right of its middle
CHEESE_IN_HAND = 0.020  # the right TCP this far above the cheese's underside
PRESS = 0.006  # aim the cheese's end this far into the face
STROKE_Z = (0.028, 0.100)  # the cheese's underside travels between these heights above the grater's foot
STROKES, STROKE_S = 6, 1.0
LEAD_S, LEAD_STEPS = 3.0, 15


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


def _go_home(arm: str, pos, rot, finger) -> list[Waypoint]:
    """From a pose back to the arm's start, straight through joint space (the lead-in, reversed)."""
    from yam_arena import ik, station
    from yam_arena.constants import READY

    q0, q1 = _good_ik(arm, pos, rot), np.array(READY, float)
    out = []
    for k in range(1, LEAD_STEPS + 1):
        p, r = ik.tcp_pose(q0 + (q1 - q0) * k / LEAD_STEPS, station.BASES[arm])
        out.append(Waypoint(LEAD_S / LEAD_STEPS, tuple(float(v) for v in p), finger, "back to where it started", r))
    return out


def _cheese_tcp(cheese_mid_y: float, underside_z: float) -> np.ndarray:
    return np.array([GX, cheese_mid_y + CHEESE_GRIP_AT, underside_z + CHEESE_IN_HAND])


HELD_AT = LEAD_S + 1.0 + 0.8


def _right() -> list[Waypoint]:
    rest = _cheese_tcp(CY0, BOARD_TOP + 0.0005)
    on_face_y = GY - G_Y / 2 - CHEESE[1] / 2 + PRESS  # the cheese's middle with its end pressed into the face
    off_y = on_face_y - 0.02
    z0, z1 = BOARD_TOP + STROKE_Z[0], BOARD_TOP + STROKE_Z[1]
    out = _lead_in("right", rest + [0.0, 0.0, 0.05], GRIP_R, "over the cheese")
    out += [_w(1.0, rest, FINGER_OPEN, "down round the cheese", GRIP_R),
            _w(0.8, rest, FINGER_CLOSED, "grip it", GRIP_R),
            _w(1.0, rest + [0.0, 0.0, 0.06], FINGER_CLOSED, "lift it", GRIP_R)]
    t = sum(w.duration for w in out)
    out += [_w(max(HELD_AT - t, 0.3) + 1.5, _cheese_tcp(off_y, z1), FINGER_CLOSED, "beside the grating face, at the top",
               GRIP_R),
            _w(0.8, _cheese_tcp(on_face_y, z1), FINGER_CLOSED, "press it on the face", GRIP_R)]
    for k in range(STROKES):
        z = z0 if k % 2 == 0 else z1
        out.append(_w(STROKE_S, _cheese_tcp(on_face_y, z), FINGER_CLOSED, "rub it down and up the face", GRIP_R))
    out += [_w(0.8, _cheese_tcp(off_y, z1), FINGER_CLOSED, "off the face", GRIP_R),
            _w(1.5, rest + [0.0, 0.0, 0.03], FINGER_CLOSED, "back over its place", GRIP_R),
            _w(0.8, rest + [0.0, 0.0, 0.003], FINGER_CLOSED, "put it down", GRIP_R),
            _w(0.5, rest + [0.0, 0.0, 0.003], FINGER_OPEN, "let go", GRIP_R),
            _w(1.0, rest + [0.0, 0.0, 0.06], FINGER_OPEN, "rise", GRIP_R)]
    out += _go_home("right", rest + [0.0, 0.0, 0.06], GRIP_R, FINGER_OPEN)
    return out


def _left(release_at: float, total: float) -> list[Waypoint]:
    handle = np.array([GX, GY + G_Y / 2 + HANDLE[1] - 0.025, BOARD_TOP + 0.0005 + G_H + G_T - HANDLE[2] / 2 + 0.003])
    out = _lead_in("left", handle + [0.0, 0.0, 0.05], GRIP_X, "over the grater's handle")
    out += [_w(1.0, handle, FINGER_OPEN, "down round the handle", GRIP_X),
            _w(0.8, handle, FINGER_CLOSED, "hold the grater", GRIP_X)]
    t = sum(w.duration for w in out)
    out += [_w(max(release_at - t, 0.5), handle, FINGER_CLOSED, "hold it steady while the cheese is grated", GRIP_X),
            _w(0.5, handle, FINGER_OPEN, "let go", GRIP_X),
            _w(1.0, handle + [0.0, 0.0, 0.05], FINGER_OPEN, "rise", GRIP_X)]
    out += _go_home("left", handle + [0.0, 0.0, 0.05], GRIP_X, FINGER_OPEN)
    t = sum(w.duration for w in out)
    out.append(_w(max(total - t, 0.5), out[-1].pos, FINGER_OPEN,
                  "hold still: the trial ends once the full score has held 1 s with both arms home", out[-1].yaw))
    return out


def demo():
    right = _right()
    put_down = sum(w.duration for w in right) - LEAD_S  # the cheese is down and the right hand on its way home
    total = sum(w.duration for w in right) + LEAD_S + 3.0
    right.append(_w(LEAD_S + 3.0, right[-1].pos, FINGER_OPEN, "wait at the start", right[-1].yaw))
    return {"left": _left(put_down, total), "right": right}
