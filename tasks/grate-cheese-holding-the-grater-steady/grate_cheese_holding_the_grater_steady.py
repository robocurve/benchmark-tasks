"""On the station table, a box grater stands on a cutting board, its grating face toward the right arm and a flat
handle reaching left from its top; a block of cheese lies on the board to its right.

Task: grate the cheese onto the board on the box grater, put the cheese down, leave the grater standing on the board
and take both arms home. The grater is light and tall: pressed on its face unheld, it slides away or tips over, so one
hand holds it by its handle while the other presses the cheese on its grating face and rubs it down across the teeth.
Scored by the grated mass: in the simulation, the length of the strokes made with the cheese's end measurably pressed
on the perforated face (PhysX contact force, normal to the face) and sliding down across the teeth, converted to grams
at a stated rate; in a real trial, the grated cheese weighed on a kitchen scale.

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

import bodykit  # noqa: E402
from bodykit import Body, BoxShape, CylShape, allow_matrix_grips, as_numpy, on_reset, on_step, rotation  # noqa: E402
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, PHYSICS_DT, TCP_OFFSET  # noqa: E402
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
FACE_Z = (0.020, 0.115)  # the perforated (grating) area on the face, from the foot
HOLE = (0.008, 0.004)  # each cutting hole: wide (across the face), tall
ROW_PITCH = 0.0095  # the rows of holes, staggered: two rows together cut the whole width once
COL_PITCH = 0.016  # holes in a row
HOLES_X = 0.028  # the perforated area reaches this far either side of the face's middle
STEEL, HOLE_C, CHEESE_C, BOARD_C, HANDLE_C = (0.78, 0.79, 0.81), (0.25, 0.25, 0.27), (0.98, 0.82, 0.30), \
    (0.80, 0.66, 0.46), (0.10, 0.10, 0.11)

# -- the cheese: a block, along y ---------------------------------------------------------------------------------------
CHEESE = (0.024, 0.060, 0.035)  # across (x), long (y), tall
CHEESE_MASS = 0.05
CX0, CY0 = 0.30, -0.14

# -- grating: credited only while the cheese's end is pressed on the perforated face and slides down across the teeth ---
PRESS_N = 3.0  # N: the contact force between the cheese and the grater, normal to the face, at least this (PhysX)
END_GAP = (-0.003, 0.005)  # the middle of the cheese's end this near the face's surface (m): its end, not a side,
# is what touches
G_PER_CM = 0.25  # g of cheese grated per cm of pressed downstroke: a coarse face shaves a ~0.57 mm layer off the
# cheese's 8.4 cm2 end for every two staggered rows (1.9 cm) it crosses, at 0.99 g/cm3
FULL_G, HALF_G, SOME_G = 5.0, 2.5, 0.5  # g grated: the score tiers (20, 10 and 2 cm of pressed downstroke)

# -- the shreds: parked out of sight; each 0.5 g grated drops one inside the grater, behind the face, at the cheese's
# height (a visual cue only: the score uses the grated mass) ---------------------------------------------------------
N_SHREDS = 14
SHRED = (0.003, 0.006, 0.028)  # 0.5 cm3: about 0.5 g of cheese
G_PER_SHRED = 0.5
PARK = (0.30, 0.0, TOP + 1.5)  # a closed box high above the scene, out of the cameras' view

# -- scoring ----------------------------------------------------------------------------------------------------------
UPRIGHT = math.radians(20.0)
SUCCESS_HOLD_S = 1.0
HOME_WITHIN = 0.05  # each TCP this close to where it started: the arms are back home


def _board() -> Body:
    return Body.build("board", pos=(BX, BY, TOP), mass=1.0, kinematic=True, friction=(0.5, 0.4), color=BOARD_C,
                      shapes=[BoxShape((0.0, 0.0, BOARD[2] / 2), BOARD, BOARD_C)])


def _hole_centres() -> list[tuple[float, float]]:
    """(across, up) of every cutting hole: 10 staggered rows, 4 and 3 holes, over the perforated area."""
    out = []
    for i in range(int(round((FACE_Z[1] - FACE_Z[0]) / ROW_PITCH))):
        z = FACE_Z[0] + ROW_PITCH / 2 + i * ROW_PITCH
        xs = (-1.5, -0.5, 0.5, 1.5) if i % 2 == 0 else (-1.0, 0.0, 1.0)
        out += [(k * COL_PITCH, z) for k in xs]
    return out


def _grater() -> Body:
    hx, hy = G_X / 2, G_Y / 2
    shapes = [BoxShape((s * (hx - G_T / 2), 0.0, G_H / 2), (G_T, G_Y, G_H), STEEL) for s in (-1.0, 1.0)]
    shapes += [BoxShape((0.0, s * (hy - G_T / 2), G_H / 2), (G_X - 2 * G_T, G_T, G_H), STEEL) for s in (-1.0, 1.0)]
    shapes += [BoxShape((0.0, 0.0, G_H + G_T / 2), (G_X, G_Y, G_T), STEEL),
               BoxShape((0.0, hy + HANDLE[1] / 2 - 0.004, G_H - HANDLE[2] / 2 + G_T), HANDLE, HANDLE_C)]
    for x, z in _hole_centres():  # the cutting holes, drawn on the face
        shapes.append(BoxShape((x, -hy - 0.0003, z), (HOLE[0], 0.0004, HOLE[1]), HOLE_C, collide=False))
    return Body.build("grater", pos=(GX, GY, BOARD_TOP + 0.0005), mass=GRATER_MASS, shapes=shapes, friction=(0.5, 0.4),
                      color=STEEL, com=(0.0, 0.01, 0.06))


def _cheese() -> Body:
    return Body.build("cheese", pos=(CX0, CY0, BOARD_TOP + 0.0005), mass=CHEESE_MASS, friction=(1.0, 0.9),
                      color=CHEESE_C, shapes=[BoxShape((0.0, 0.0, CHEESE[2] / 2), CHEESE, CHEESE_C)])


def _park() -> Body:
    """A closed box where the shreds wait, out of sight."""
    w, t = 0.05, 0.004
    shapes = [BoxShape((0.0, 0.0, t / 2), (w, w, t), HOLE_C), BoxShape((0.0, 0.0, w - t / 2), (w, w, t), HOLE_C)]
    shapes += [BoxShape((s * (w / 2 - t / 2), 0.0, w / 2), (t, w, w), HOLE_C) for s in (-1.0, 1.0)]
    shapes += [BoxShape((0.0, s * (w / 2 - t / 2), w / 2), (w, t, w), HOLE_C) for s in (-1.0, 1.0)]
    return Body.build("shred_store", pos=PARK, mass=1.0, shapes=shapes, kinematic=True, color=HOLE_C)


def _shreds() -> list[Body]:
    out = []
    for i in range(N_SHREDS):
        dx, dy = (i % 5 - 2.0) * 0.008, (i // 5 - 1.0) * 0.012
        out.append(Body.build(f"shred_{i:02d}", pos=(PARK[0] + dx, PARK[1] + dy, PARK[2] + 0.005), mass=0.0005,
                              shapes=[BoxShape((0.0, 0.0, SHRED[2] / 2), SHRED, CHEESE_C)], friction=(0.6, 0.5),
                              damping=(0.2, 0.5), color=CHEESE_C))
    return out


BOARD_B, STORE, GRATER, CHEESE_B = _board(), _park(), _grater(), _cheese()
SHREDS = _shreds()
OBJECTS = [BOARD_B, STORE, GRATER, CHEESE_B, *SHREDS]
SHRED_NAMES = [s.name for s in SHREDS]

# -- contact sensing: the cheese and the grater report their contacts ------------------------------------------------
_PATHS: dict = {}
_author_plain = bodykit._author_sliders


def _author_sliders(stage, prim_path: str, spec) -> None:
    _author_plain(stage, prim_path, spec)
    if spec.name in ("cheese", "grater"):
        from pxr import PhysxSchema

        prim = stage.GetPrimAtPath(f"{prim_path}/body")
        PhysxSchema.PhysxContactReportAPI.Apply(prim).CreateThresholdAttr(0.0)
        _PATHS[spec.name] = f"{prim_path}/body"


bodykit._author_sliders = _author_sliders


# -- followed after every control step: the pressed strokes, the shreds, the grippers -----------------------------------

def _fresh() -> dict:
    return {"stroke": 0.0, "dropped": 0, "last_z": None, "tcps": None, "view": None, "view_err": "",
            "fn_max": 0.0, "near": 0.0, "near_z": None, "pressed_steps": 0}


_EP = _fresh()
_DEBUG = {"on": bool(os.environ.get("GRATE_DEBUG")), "steps": 0}


@on_reset
def _new_episode() -> None:
    _EP.clear()
    _EP.update(_fresh())
    _DEBUG["steps"] = 0


def _np(a) -> np.ndarray:
    if hasattr(a, "numpy") and not hasattr(a, "detach"):
        return np.asarray(a.numpy(), float)
    return as_numpy(a)


def _contact_force():
    """The contact force on the cheese from the grater (world, N), from PhysX's rigid-contact view; None until the
    view exists."""
    if _EP["view"] is None and not _EP["view_err"] and len(_PATHS) == 2:
        try:
            from isaaclab_physx.physics import PhysxManager

            sim = PhysxManager.get_physics_sim_view()
            _EP["view"] = sim.create_rigid_contact_view([_PATHS["cheese"]], filter_patterns=[[_PATHS["grater"]]],
                                                        max_contact_data_count=0)
        except Exception as e:  # noqa: BLE001
            _EP["view_err"] = repr(e)[:300]
    if _EP["view"] is None:
        return None
    return _np(_EP["view"].get_contact_force_matrix(dt=PHYSICS_DT)).reshape(-1, 3).sum(axis=0)


def _tcps(base, origin) -> list[np.ndarray]:
    out = []
    for arm in ("left_robot", "right_robot"):
        robot = base.scene[arm]
        i = next(j for j, n in enumerate(robot.body_names) if n == "gripper" or n.endswith("/gripper"))
        g = as_numpy(robot.data.body_link_pose_w)[0][i]
        out.append(g[:3] - origin + rotation(g[3:7]) @ np.array(TCP_OFFSET))
    return out


def _end_on_face(g_pos, g_rot, c_pos, c_rot):
    """(gap, where): the middle of whichever end of the cheese is nearer the grating face, in the grater's frame, and
    how far it stands off the face's outer surface (negative: pushed into it)."""
    best = None
    for sx in (-1.0, 1.0):
        end = np.asarray(c_pos, float) + c_rot @ np.array([0.0, sx * CHEESE[1] / 2, CHEESE[2] / 2])
        v = g_rot.T @ (end - np.asarray(g_pos, float))
        gap = -G_Y / 2 - v[1]
        if best is None or gap < best[0]:
            best = (float(gap), v)
    return best


def _standing(g_pos, g_rot) -> bool:
    """The grater standing on the board: within 20 degrees of upright, its foot on the board inside its edges."""
    up = math.acos(max(-1.0, min(1.0, float(g_rot[2, 2])))) < UPRIGHT
    on = (abs(g_pos[0] - BX) < BOARD[0] / 2 - G_X / 2 and abs(g_pos[1] - BY) < BOARD[1] / 2 - G_Y / 2
          and abs(g_pos[2] - BOARD_TOP) < 0.01)
    return bool(up and on)


def grated_g() -> float:
    """The grated mass so far (g): the pressed downstroke length (cm) at G_PER_CM."""
    return float(_EP.get("stroke", 0.0)) * 100.0 * G_PER_CM


def _drop(base, origin, g_pos, g_rot, z) -> None:
    """Teleport the next parked shred inside the grater, just behind the face, at the cheese's height."""
    import torch

    name = SHRED_NAMES[_EP["dropped"]]
    asset = base.scene[name]
    pose = as_numpy(asset.data.root_link_pose_w)[0].copy()
    jitter = ((_EP["dropped"] % 3) - 1) * 0.012
    p = g_pos + g_rot @ np.array([jitter, -G_Y / 2 + 0.010, min(max(z, FACE_Z[0]), FACE_Z[1] - 0.02) - 0.01])
    pose[:3] = p + origin
    t = torch.tensor(pose, dtype=torch.float32, device=asset.device).view(1, 7)
    asset.write_root_pose_to_sim(t)
    asset.write_root_velocity_to_sim(torch.zeros(1, 6, dtype=torch.float32, device=asset.device))
    _EP["dropped"] += 1


@on_step
def _follow(env) -> None:
    base = env.unwrapped
    origin = as_numpy(base.scene.env_origins[0])
    g = as_numpy(base.scene["grater"].data.root_link_pose_w)[0]
    c = as_numpy(base.scene["cheese"].data.root_link_pose_w)[0]
    g_pos, g_rot, c_pos, c_rot = g[:3] - origin, rotation(g[3:7]), c[:3] - origin, rotation(c[3:7])
    _EP["tcps"] = _tcps(base, origin)
    f = _contact_force()
    fn = 0.0 if f is None else float(np.dot(f, -g_rot[:, 1]))  # pressing on the face: along its outward normal
    gap, v = _end_on_face(g_pos, g_rot, c_pos, c_rot)
    z = float(v[2])
    on_holes = END_GAP[0] <= gap <= END_GAP[1] and abs(v[0]) <= HOLES_X and FACE_Z[0] <= z <= FACE_Z[1]
    standing = _standing(g_pos, g_rot)
    pressed = bool(on_holes and fn >= PRESS_N and standing)
    _EP["fn_max"] = max(_EP["fn_max"], fn)
    if pressed:
        _EP["pressed_steps"] += 1
    if pressed and _EP["last_z"] is not None and z < _EP["last_z"]:  # pressed at both ends of the step, moving down
        _EP["stroke"] += _EP["last_z"] - z
        while _EP["dropped"] < N_SHREDS and grated_g() >= (_EP["dropped"] + 1) * G_PER_SHRED:
            _drop(base, origin, g_pos, g_rot, z)
    _EP["last_z"] = z if pressed else None
    # for the record only (not scored): the earlier distance-only rule, the end's travel within 4 mm of the face
    near = gap <= 0.004 and abs(v[0]) < G_X / 2 and FACE_Z[0] - 0.01 < z < FACE_Z[1] + 0.02
    if near and _EP["near_z"] is not None and standing:
        _EP["near"] += abs(z - _EP["near_z"])
    _EP["near_z"] = z if near else None
    if _DEBUG["on"]:
        _DEBUG["steps"] += 1
        if _DEBUG["steps"] % 5 == 0:
            tl, tr = _EP["tcps"]
            fw = [round(float(x), 2) for x in f] if f is not None else _EP["view_err"] or None
            print(f"[grate] t={_DEBUG['steps'] / 20:5.2f}s grater={np.round(g_pos * 1000).astype(int).tolist()} "
                  f"tilt={math.degrees(math.acos(max(-1.0, min(1.0, float(g_rot[2, 2]))))):4.1f} "
                  f"F={fw} Fn={fn:5.2f}N gap={gap * 1000:5.1f}mm x={v[0] * 1000:4.0f} z={z * 1000:4.0f}mm "
                  f"pressed={pressed} stroke={_EP['stroke'] * 100:5.1f}cm grated={grated_g():4.2f}g "
                  f"dropped={_EP['dropped']} near_rule={_EP['near'] * 100:5.1f}cm fn_max={_EP['fn_max']:5.1f} "
                  f"left TCP={np.round(tl * 1000).astype(int).tolist()} "
                  f"right TCP-cheese={np.round((tr - c_pos) * 1000).astype(int).tolist()} score={_points():.2f}",
                  file=sys.stderr, flush=True)


# -- scoring --------------------------------------------------------------------------------------------------------

def _points() -> float:
    """By the grated mass: the 1.0-tier at 5 g, 0.5 at 2.5 g, 0.25 at 0.5 g."""
    m = grated_g()
    if m >= FULL_G - 1e-9:
        return 1.0
    if m >= HALF_G - 1e-9:
        return 0.5
    return 0.25 if m >= SOME_G - 1e-9 else 0.0


def _grater_standing(objects) -> bool:
    g = objects["grater"]
    return _standing(np.asarray(g.pos, float), rotation(g.quat_xyzw))


def score(objects) -> float:
    """By the grated mass (the cheese's end pressed on the perforated face with at least 3 N, measured contact force,
    and sliding down across the teeth with the grater standing; 0.25 g per cm of such stroke): 1.0 with at least 5 g
    grated and, at the end, the grater standing on the board, the cheese put down and both arms back where they
    started; 0.5 with at least 2.5 g (or 5 g without that end state); 0.25 with at least 0.5 g; else 0.0."""
    pts = _points()
    if pts >= 1.0 and not (_grater_standing(objects) and _cheese_down(objects) and _arms_home()):
        return 0.5  # grated, but the end state not reached
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
    dropped = SHRED_NAMES[:int(_EP.get("dropped", 0))]
    return score(objects) >= 1.0 and all(objects[n].resting() for n in dropped + ["grater", "cheese"])


# -- scripted demo ----------------------------------------------------------------------------------------------------
# The left hand comes straight down on the grater's handle near its end and holds it. The right hand comes straight
# down on the cheese, near its right end, lifts it and grates: it presses the cheese's left end on the grating face at
# the top and rubs it down across the teeth, eases the press and brings it back up lightly, five times. Then it puts the
# cheese back down and goes home. The left then lets go, takes the handle near its root, lifts the grater 3 cm, stands
# it on the board 7 cm to the left (showing the grated cheese) and goes home too.

GRIP_X = grip_rotation(-math.pi / 2)  # straight down, fingers closing along x (the left hand)
GRIP_R = grip_rotation(math.pi / 2)  # the same for the right hand, turned the other way round (its wrist's range)
CHEESE_GRIP_AT = -0.015  # the right hand holds the cheese this far right of its middle
CHEESE_IN_HAND = 0.020  # the right TCP this far above the cheese's underside
PRESS = 0.050  # on a downstroke aim the cheese's end this far into the face (both arms give: it presses about 5 N)
PRESS_UP = 0.010  # on the way back up only lightly (the return stroke grates nothing)
LEAN = 0.035  # on each downstroke the left hand braces the grater, aimed this far toward the cheese (the arm gives)
BRACE_EARLY = 0.1  # s: the brace starts this much before each press (the left arm lags)
STROKE_Z = (0.025, 0.090)  # the cheese's underside travels between these heights above the grater's foot
STROKES = 5
IN_S, DOWN_S, OUT_S, UP_S = 0.3, 1.2, 0.2, 0.8
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
    at_face_y = GY - G_Y / 2 - CHEESE[1] / 2  # the cheese's middle with its end just touching the face
    off_y = at_face_y - 0.02
    z0, z1 = BOARD_TOP + STROKE_Z[0], BOARD_TOP + STROKE_Z[1]
    out = _lead_in("right", rest + [0.0, 0.0, 0.05], GRIP_R, "over the cheese")
    out += [_w(1.0, rest, FINGER_OPEN, "down round the cheese", GRIP_R),
            _w(0.8, rest, FINGER_CLOSED, "grip it", GRIP_R),
            _w(1.0, rest + [0.0, 0.0, 0.06], FINGER_CLOSED, "lift it", GRIP_R)]
    t = sum(w.duration for w in out)
    out.append(_w(max(HELD_AT - t, 0.3) + 1.5, _cheese_tcp(off_y, z1), FINGER_CLOSED,
                  "beside the grating face, at the top", GRIP_R))
    for _ in range(STROKES):
        out += [_w(IN_S, _cheese_tcp(at_face_y + PRESS, z1), FINGER_CLOSED, "press it on the face", GRIP_R),
                _w(DOWN_S, _cheese_tcp(at_face_y + PRESS, z0), FINGER_CLOSED, "rub it down across the teeth", GRIP_R),
                _w(OUT_S, _cheese_tcp(at_face_y + PRESS_UP, z0), FINGER_CLOSED, "ease the press", GRIP_R),
                _w(UP_S, _cheese_tcp(at_face_y + PRESS_UP, z1), FINGER_CLOSED, "back up, lightly", GRIP_R)]
    out += [_w(0.8, _cheese_tcp(off_y, z1), FINGER_CLOSED, "away from the face", GRIP_R),
            _w(1.5, rest + [0.0, 0.0, 0.03], FINGER_CLOSED, "back over its place", GRIP_R),
            _w(0.8, rest + [0.0, 0.0, 0.003], FINGER_CLOSED, "put it down", GRIP_R),
            _w(0.5, rest + [0.0, 0.0, 0.003], FINGER_OPEN, "let go", GRIP_R),
            _w(1.0, rest + [0.0, 0.0, 0.06], FINGER_OPEN, "rise", GRIP_R)]
    out += _go_home("right", rest + [0.0, 0.0, 0.06], GRIP_R, FINGER_OPEN)
    return out


SET_ASIDE = 0.07  # after grating, the left hand lifts the grater this far to the left, off the shreds, to show them
LIFT = 0.03


def _left(press_at: float, release_at: float, right_home_at: float) -> list[Waypoint]:
    handle = np.array([GX, GY + G_Y / 2 + HANDLE[1] - 0.025, BOARD_TOP + 0.0005 + G_H + G_T - HANDLE[2] / 2 + 0.003])
    root = handle + [0.0, -(HANDLE[1] - 0.045), 0.0]  # near the handle's root: a short lever to carry the grater by
    out = _lead_in("left", handle + [0.0, 0.0, 0.05], GRIP_X, "over the grater's handle")
    out += [_w(1.0, handle, FINGER_OPEN, "down round the handle", GRIP_X),
            _w(0.8, handle, FINGER_CLOSED, "hold the grater", GRIP_X)]
    t = sum(w.duration for w in out)
    lean = handle + [0.0, -LEAN, 0.0]
    out.append(_w(max(press_at - BRACE_EARLY - t, 0.5), handle, FINGER_CLOSED, "hold the grater steady", GRIP_X))
    for _ in range(STROKES):  # brace against each downstroke's press, in time with the right hand's strokes
        out += [_w(IN_S, lean, FINGER_CLOSED, "brace it against the press", GRIP_X),
                _w(DOWN_S, lean, FINGER_CLOSED, "hold it steady while the cheese is grated", GRIP_X),
                _w(OUT_S, handle, FINGER_CLOSED, "ease the brace", GRIP_X),
                _w(UP_S, handle, FINGER_CLOSED, "hold it steady", GRIP_X)]
    t = sum(w.duration for w in out)
    out += [_w(max(release_at - t, 0.5), handle, FINGER_CLOSED, "hold it while the cheese is put down", GRIP_X),
            _w(0.5, handle, FINGER_OPEN, "let go", GRIP_X),
            _w(1.0, handle + [0.0, 0.0, 0.04], FINGER_OPEN, "rise", GRIP_X)]
    t = sum(w.duration for w in out)
    out += [_w(max(right_home_at - t, 0.5), root + [0.0, 0.0, 0.04], FINGER_OPEN,
               "over the handle's root, once the right hand is home", GRIP_X),
            _w(1.0, root, FINGER_OPEN, "down round the handle's root", GRIP_X),
            _w(0.8, root, FINGER_CLOSED, "grip it", GRIP_X),
            _w(1.2, root + [0.0, 0.0, LIFT], FINGER_CLOSED, "lift the grater off the shreds", GRIP_X),
            _w(1.5, root + [0.0, SET_ASIDE, LIFT], FINGER_CLOSED, "carry it aside", GRIP_X),
            _w(1.2, root + [0.0, SET_ASIDE, 0.002], FINGER_CLOSED, "stand it on the board beside the grated cheese",
               GRIP_X),
            _w(0.6, root + [0.0, SET_ASIDE, 0.002], FINGER_OPEN, "let go", GRIP_X),
            _w(1.0, root + [0.0, SET_ASIDE, 0.05], FINGER_OPEN, "rise", GRIP_X)]
    out += _go_home("left", root + [0.0, SET_ASIDE, 0.05], GRIP_X, FINGER_OPEN)
    out.append(_w(LEAD_S, out[-1].pos, FINGER_OPEN,
                  "hold still: the trial ends once the full score has held 1 s with both arms home", out[-1].yaw))
    return out


def _starts(wps: list[Waypoint], label: str) -> float:
    """When the first waypoint with this label starts."""
    t = 0.0
    for w in wps:
        if w.label == label:
            return t
        t += w.duration
    raise ValueError(label)


def demo():
    right = _right()
    put_down = sum(w.duration for w in right) - LEAD_S  # the cheese is down and the right hand on its way home
    right_home = sum(w.duration for w in right) + 0.3
    left = _left(_starts(right, "press it on the face"), put_down, right_home)
    total = sum(w.duration for w in left)
    right.append(_w(max(total - sum(w.duration for w in right), 0.5), right[-1].pos, FINGER_OPEN, "wait at the start",
                    right[-1].yaw))
    return {"left": left, "right": right}
