"""On the station table: a hollow wall panel lying face-up, 12.7 mm grey wallboard over a wooden 2x3 frame with a
closed cavity under it and a pencil line drawn across its face. At a prepared 12.7 mm hole in the line a steel hook
rests on the board, a 3/16 in (10-24) spring-wing toggle bolt through its plate; the toggle's wings have opened in the cavity
behind the board. Beside the panel a stubby ratcheting Phillips screwdriver stands in a bench holder.
"""

from __future__ import annotations

import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import parts  # noqa: E402
import tilt  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]

BOARD_GREY, WOOD, PENCIL = (0.80, 0.80, 0.78), (0.78, 0.62, 0.40), (0.20, 0.20, 0.22)
STEEL, ZINC, DARK = (0.55, 0.57, 0.60), (0.74, 0.75, 0.77), (0.15, 0.15, 0.16)

# --- the panel (fixed): frame on the table, board on top; panel frame at the table under the hole --------------------
HOLE = (0.30, -0.035)  # world xy of the hole centre
BOARD_T = 0.0127
STUD_D = 0.0635  # 2x3 stud depth: the cavity height
STUD_T = 0.038
FACE = STUD_D + BOARD_T  # board face height above the table
PANEL_X = (0.175 - HOLE[0], 0.425 - HOLE[0])  # 250 mm, panel frame
PANEL_Y = (-0.150 - HOLE[1], 0.150 - HOLE[1])  # 300 mm
HOLE_R = 0.00635
CUT = 0.015  # half side of the square opening the octagonal hole ring fills


def _slab(x0, x1, y0, y1, z0, z1, color, **kw):
    return parts.box((x1 - x0, y1 - y0, z1 - z0), at=((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2), color=color, **kw)


def _panel_shapes():
    (x0, x1), (y0, y1) = PANEL_X, PANEL_Y
    zb = (STUD_D, FACE)
    out = [  # frame: four studs round the cavity
        _slab(x0, x1, y0, y0 + STUD_T, 0.0, STUD_D, WOOD), _slab(x0, x1, y1 - STUD_T, y1, 0.0, STUD_D, WOOD),
        _slab(x0, x0 + STUD_T, y0 + STUD_T, y1 - STUD_T, 0.0, STUD_D, WOOD),
        _slab(x1 - STUD_T, x1, y0 + STUD_T, y1 - STUD_T, 0.0, STUD_D, WOOD),
        # board round a square opening at the hole
        _slab(x0, x1, y0, -CUT, *zb, BOARD_GREY), _slab(x0, x1, CUT, y1, *zb, BOARD_GREY),
        _slab(x0, -CUT, -CUT, CUT, *zb, BOARD_GREY), _slab(CUT, x1, -CUT, CUT, *zb, BOARD_GREY),
    ]
    depth = 0.016  # eight tangent slabs fill the opening and leave an octagonal hole, 12.7 mm across flats
    width = 2 * math.tan(math.pi / 8) * (HOLE_R + depth)
    for i in range(8):
        a = i * math.pi / 4
        c = HOLE_R + depth / 2
        out.append(parts.box((depth, width, BOARD_T), at=(c * math.cos(a), c * math.sin(a), (zb[0] + zb[1]) / 2),
                             color=BOARD_GREY, yaw=a))
    # pencil line across the face, along y through the hole (drawn, no collider)
    out.append(_slab(-0.0015, 0.0015, y0 + 0.02, y1 - 0.02, FACE, FACE + 0.0003, PENCIL, collide=False))
    return out


PANEL = parts.Part("panel", pos=(*HOLE, TOP), shapes=_panel_shapes(), contact_offset=0.002,
                   compliance=(2.0e4, 40.0))  # wallboard face and back give a little under load

# --- the hook on its toggle bolt (loose): frame on the board face at the hole --------------------------------------
PLATE = (0.040, 0.125, 0.003)  # x, y, thickness; the hole is 40 mm from the plate's -y end
PLATE_Y = (-0.040, 0.085)
UPRIGHT = (0.005, 0.024, 0.032)  # x (its thickness), y, height
UPRIGHT_Y = 0.080
LEAD = 0.0254 / 24  # 3/16 in (10-24) thread: 1.06 mm per turn
GAP0 = 0.003  # the wings start 3 mm behind the board's back face (about 2.8 turns to go)
HEAD_R, HEAD_H = 0.006, 0.0046
SLOT_W = 0.0026  # cross recess: square-sided slots 2.6 mm wide, 3 mm deep
SHANK_R, SHANK_L = 0.0024, 0.0635  # 3/16 in (4.8 mm) x 2-1/2 in
WING = (0.048, 0.011, 0.006)  # opened wing span, width, thickness
NUT_Z = -BOARD_T - GAP0 - WING[2] / 2  # nut centre, hook frame
RUN_OFF = -(SHANK_L - PLATE[2]) - NUT_Z + 0.003  # nut travel (negative) at which it leaves the bolt end


def _head_shapes():
    z0 = PLATE[2]
    out = [parts.cylinder(HEAD_R, HEAD_H - 0.003, at=(0, 0, z0 + (HEAD_H - 0.003) / 2), color=ZINC)]
    q = (HEAD_R * 1.7 - SLOT_W) / 2  # four blocks inside the head's round, leaving the cross recess
    for sx in (-1, 1):
        for sy in (-1, 1):
            c = SLOT_W / 2 + q / 2
            out.append(parts.box((q, q, 0.003), at=(sx * c, sy * c, z0 + HEAD_H - 0.0015), color=ZINC))
    out.append(parts.cylinder(SHANK_R, SHANK_L, at=(0, 0, z0 - SHANK_L / 2), color=ZINC))
    return out


BOLT = parts.Hinge("bolt", shapes=_head_shapes(), pivot=(0.0, 0.0, PLATE[2]), axis=(0.0, 0.0, 1.0), lower=-250.0,
                   upper=250.0, mass=0.012, damping=0.01)  # damping: the head bearing on the plate
THREAD = parts.Slider("thread", shapes=[parts.box((0.004, 0.004, 0.004), at=(0, 0, NUT_Z), color=ZINC,
                                                  collide=False)],
                      axis=(0.0, 0.0, 1.0), travel=GAP0 + 0.0005, back=-RUN_OFF, mass=0.01, damping=0.0,
                      parent="bolt", thread=("nut", LEAD))
NUT = parts.Hinge("nut", shapes=[parts.box(WING, at=(0, 0, NUT_Z), color=ZINC),
                                 parts.box((0.016, 0.008, 0.010), at=(0, 0, NUT_Z - 0.002), color=DARK)],
                  pivot=(0.0, 0.0, NUT_Z), axis=(0.0, 0.0, 1.0), lower=-250.0, upper=250.0, mass=0.010,
                  damping=0.002, parent="thread")
HOOK = parts.Part(
    "hook", pos=(*HOLE, TOP + FACE), mass=0.15, contact_offset=0.0005, self_collision=False,
    shapes=[
        _slab(-PLATE[0] / 2, PLATE[0] / 2, *PLATE_Y, 0.0, PLATE[2], STEEL),
        parts.box(UPRIGHT, at=(0.0, UPRIGHT_Y, PLATE[2] + UPRIGHT[2] / 2), color=STEEL),
        # the hook's lip: out along the line from the upright's top, then up
        parts.box((0.005, 0.016, 0.005), at=(0.0, UPRIGHT_Y + 0.020, PLATE[2] + UPRIGHT[2] - 0.0025), color=STEEL),
        parts.box((0.005, 0.005, 0.010), at=(0.0, UPRIGHT_Y + 0.0255, PLATE[2] + UPRIGHT[2] + 0.005), color=STEEL),
    ],
    joints=[BOLT, THREAD, NUT],
)

# --- the screwdriver (loose) and its bench holder (fixed) -------------------------------------------------------------
BLADE = (0.0055, 0.0012, 0.003)  # #2 Phillips tip: two crossed blades, 3 mm long
SHAFT_R, SHAFT_L = 0.003, 0.035
HANDLE_R, HANDLE_L = 0.0175, 0.050
HANDLE_Z0 = BLADE[2] + SHAFT_L  # handle's lower end above the tip
DRIVER_H = HANDLE_Z0 + HANDLE_L  # 88 mm
HOLDER = (0.050, 0.050, 0.045)
HOLDER_XY = (0.30, -0.215)
BORE_R = 0.007  # holder bore, 14 mm across flats
# A stubby ratcheting screwdriver: the shaft drives the screw clockwise and freewheels in the handle the other way
# (the pawl is the shaft joint's upper limit, moved down after it every control step).
RATCHET_SLACK = 0.02  # rad of play at the pawl
SHAFT = parts.Hinge(
    "shaft", pivot=(0.0, 0.0, 0.0), axis=(0.0, 0.0, 1.0), lower=-80.0, upper=RATCHET_SLACK, mass=0.015,
    damping=1e-4, shapes=[
        parts.box(BLADE, at=(0, 0, BLADE[2] / 2), color=DARK),
        parts.box((BLADE[1], BLADE[0], BLADE[2]), at=(0, 0, BLADE[2] / 2), color=DARK),
        parts.cylinder(SHAFT_R, SHAFT_L, at=(0, 0, BLADE[2] + SHAFT_L / 2), color=ZINC),
    ])
DRIVER = parts.Part(
    "screwdriver", pos=(*HOLDER_XY, TOP + HOLDER[2] - HANDLE_Z0), mass=0.075, contact_offset=0.0003,
    self_collision=False,
    shapes=[parts.cylinder(HANDLE_R, HANDLE_L, at=(0, 0, HANDLE_Z0 + HANDLE_L / 2), color=(0.85, 0.15, 0.12)),
            parts.cylinder(0.006, 0.004, at=(0, 0, DRIVER_H + 0.002), color=(0.12, 0.12, 0.13))],  # ratchet cap
    joints=[SHAFT],
)
_pawl = {"upper": None}


@parts.on_step
def _ratchet(sim) -> None:
    """The pawl: the shaft may turn anticlockwise in the handle (q falling) but not clockwise past where it has been."""
    import torch

    q = sim.states()["screwdriver"].joints["shaft"]
    up = min(RATCHET_SLACK if _pawl["upper"] is None else _pawl["upper"], q + RATCHET_SLACK)
    if _pawl["upper"] is not None and up >= _pawl["upper"] - 1e-4:
        return
    _pawl["upper"] = up
    base = sim.env.unwrapped
    tool = base.scene["screwdriver"]
    ids = tool.find_joints(["shaft"], preserve_order=True)[0]
    lim = torch.tensor([[[SHAFT.lower, up]]], dtype=torch.float32, device=base.device)
    tool.write_joint_position_limit_to_sim_index(limits=lim, joint_ids=ids, warn_limit_violation=False)


@parts.on_reset
def _ratchet_reset(sim) -> None:
    import torch

    _pawl["upper"] = RATCHET_SLACK
    base = sim.env.unwrapped
    tool = base.scene["screwdriver"]
    ids = tool.find_joints(["shaft"], preserve_order=True)[0]
    lim = torch.tensor([[[SHAFT.lower, RATCHET_SLACK]]], dtype=torch.float32, device=base.device)
    tool.write_joint_position_limit_to_sim_index(limits=lim, joint_ids=ids, warn_limit_violation=False)


def _holder_shapes():
    w, d, h = HOLDER
    c, black = BORE_R + 0.006, (0.22, 0.22, 0.24)
    out = [_slab(-w / 2, w / 2, -d / 2, -c, 0, h, black), _slab(-w / 2, w / 2, c, d / 2, 0, h, black),
           _slab(-w / 2, -c, -c, c, 0, h, black), _slab(c, w / 2, -c, c, 0, h, black)]
    depth = 0.007
    width = 2 * math.tan(math.pi / 8) * (BORE_R + depth)
    for i in range(8):
        a = i * math.pi / 4 + math.pi / 8
        r = BORE_R + depth / 2
        out.append(parts.box((depth, width, h), at=(r * math.cos(a), r * math.sin(a), h / 2), color=black, yaw=a))
    return out


HOLDER_PART = parts.Part("driver_holder", pos=(*HOLDER_XY, TOP), shapes=_holder_shapes())

OBJECTS = [PANEL, HOOK, DRIVER, HOLDER_PART]

# --- scoring ----------------------------------------------------------------------------------------------------------
EPS = 1e-6
FLUSH = 0.0005  # every corner of the plate's underside within 0.5 mm of the face
SQUARE_DEG = 3.0
CROOKED_DEG = 10.0
CLEAR = 0.005
FACE_Z = TOP + FACE
# The pull test, run by the scorer once the hook has been let go and has stayed still for 1 s: a steady pull of PULL_N
# straight out from the face at the foot of the hook's upright (80 mm from the bolt) for PULL_STEPS control steps, then
# none for SETTLE_STEPS. Tight: no corner of the plate rises more than LIFT_MAX above where it was before the pull, and
# the hook comes back to rest where it was (within 0.2 mm and 0.2 degrees).
PULL_N = 5.0
PULL_AT = (0.0, UPRIGHT_Y, PLATE[2])  # hook frame: the upright's foot, on the plate's centre line
PULL_STEPS = 10  # 0.5 s at 20 Hz
SETTLE_STEPS = 10  # 0.5 s
LIFT_MAX = 0.0005
LIFT_MIN = 0.00005  # the pull must visibly act: a test in which no corner rose even 0.05 mm is not a test

# Released: in the simulator an on_step callback measures every step how close both arms' fingers and grippers come
# to the hook (the screwdriver may stay in a hand). Until it has measured (or when it cannot), "clear" is None and nothing scores.
# (Kept in the parts module so near_misses.py, which imports this file separately, can set it for offline cases.)
_grip = parts.__dict__.setdefault("_hook_task_grip", {"clear": None})
# The pull test's progress, read by the on_step callback that applies the pull (also kept in the parts module).
_pull = parts.__dict__.setdefault("_hook_task_pull", {"phase": "idle"})
# Whole-trial record: did the toggle ever run off the bolt's end? Cleared only by a scene reset (the hook back within
# 0.5 mm of its start and at rest, after a jump of over 20 mm in one control step).
_record = {"ran_off": False, "prev": None, "still": 0}
STILL_STEPS = 20  # the hook at rest for 1 s (20 control steps) in a row


def _rot(q) -> np.ndarray:
    return parts._quat_to_matrix(q)


def _hook_boxes(hook):
    poses = {"base": (np.asarray(hook.pos), _rot(hook.quat_xyzw))}
    poses.update({k: (np.asarray(p), _rot(q)) for k, (p, q) in hook.links.items()})
    return HOOK.world_boxes(poses)


def _plate_corners(hook) -> np.ndarray:
    """World corners of the plate's underside."""
    r, p = _rot(hook.quat_xyzw), np.asarray(hook.pos)
    return np.array([p + r @ np.array([sx * PLATE[0] / 2, y, 0.0]) for sx in (-1, 1) for y in PLATE_Y])


def _yaw_off_deg(hook) -> float:
    """Angle between the plate's long axis (bolt end to hook end) and the pencil line's +y direction, degrees."""
    axis = _rot(hook.quat_xyzw) @ np.array([0.0, 1.0, 0.0])
    return abs(math.degrees(math.atan2(axis[0], axis[1])))


def _flush(hook) -> bool:
    z = _plate_corners(hook)[:, 2]
    return bool(np.all(np.abs(z - FACE_Z) <= FLUSH + EPS))


def _driver_clear(objects) -> bool:
    d = objects["screwdriver"]
    lo, hi = np.asarray(d.low), np.asarray(d.high)
    for _, c, r, s in _hook_boxes(objects["hook"]):
        half = np.abs(r) @ (np.asarray(s) / 2)
        gap = np.maximum(np.maximum(lo - (c + half), (c - half) - hi), 0.0)
        if np.linalg.norm(gap) < CLEAR - EPS:
            return False
    return True


def _update(objects) -> None:
    hook = objects["hook"]
    pos = np.asarray(hook.pos, float)
    prev = _record["prev"]
    if (prev is not None and np.linalg.norm(pos - np.asarray(HOOK.pos)) < 0.0005 and hook.resting()
            and np.linalg.norm(pos - prev) > 0.020):
        _record["ran_off"] = False
    _record["prev"] = pos
    if hook.joints.get("thread", 0.0) <= RUN_OFF + 0.0005:
        _record["ran_off"] = True


def _pose(hook):
    return np.asarray(hook.pos, float), np.asarray(hook.quat_xyzw, float), hook.joints.get("thread", 0.0)


def _same_pose(a, b) -> bool:
    """Within 0.2 mm, 0.2 degrees and 0.2 mm of thread travel."""
    return bool(np.linalg.norm(a[0] - b[0]) <= 0.0002 + EPS and abs(a[2] - b[2]) <= 0.0002 + EPS
                and abs(float(np.dot(a[1], b[1]))) >= math.cos(math.radians(0.2) / 2) - 1e-9)


def _count_still(objects) -> None:
    """Once per control step (from score()): consecutive steps with the hook at rest and its pose and thread
    unchanged (within 0.2 mm and 0.2 degrees of the previous step)."""
    hook = objects["hook"]
    now = _pose(hook)
    was = _record.get("still_pose")
    same = was is not None and _same_pose(now, was)
    _record["still"] = _record["still"] + 1 if (same and hook.resting()) else 0
    _record["still_pose"] = now


def _ready(objects) -> bool:
    """Let go, the screwdriver clear, the plate flush on the face and within 10 degrees of the line."""
    hook = objects["hook"]
    return bool(not _record["ran_off"] and _grip["clear"] is True and _flush(hook) and _driver_clear(objects)
                and _yaw_off_deg(hook) <= CROOKED_DEG + EPS)


def _pull_step(objects) -> None:
    """Once per control step (from score()): run the pull test. It starts when the hook is ready and has been still
    for 1 s, and starts over whenever anything touches or moves the hook (a gripper within 5 mm, the screwdriver
    near it, the hook not back where it was after the test)."""
    hook = objects["hook"]
    p = _pull
    if p["phase"] == "idle":
        if _ready(objects) and _record["still"] >= STILL_STEPS:
            p.update(phase="pull", k=0, ref=_pose(hook), ref_z=_plate_corners(hook)[:, 2], rise=0.0, passed=False,
                     pulled=0)
        return
    if p["phase"] in ("pull", "settle"):
        if _grip["clear"] is not True or not _driver_clear(objects) or _record["ran_off"]:
            p.update(phase="idle")
            return
        p["rise"] = max(p["rise"], float(np.max(_plate_corners(hook)[:, 2] - p["ref_z"])))
        if p["phase"] == "pull":  # fail closed: count the steps on which the wrench was really set to PULL_N
            if "offline_applied" in p:  # near_misses.py stands in for the simulator's wrench offline
                p["applied"] = p["offline_applied"](objects)
            p["pulled"] += int(abs(float(p.get("applied") or 0.0) - PULL_N) < 1e-6)
        p["k"] += 1
        if p["phase"] == "pull" and p["k"] >= PULL_STEPS:
            p.update(phase="settle", k=0)
        elif p["phase"] == "settle" and p["k"] >= SETTLE_STEPS:
            back = _same_pose(_pose(hook), p["ref"]) and hook.resting()
            pulled = p["pulled"] >= PULL_STEPS - 1 and p["rise"] >= LIFT_MIN - EPS
            p.update(phase="done", passed=bool(back and pulled and p["rise"] <= LIFT_MAX + EPS))
        return
    if not (_ready(objects) and hook.resting() and _same_pose(_pose(hook), p["ref"])):  # done, then disturbed
        p.update(phase="idle")


def _done(objects) -> float:
    """1.0 tight (passed the pull test) and square, 0.5 tight but 3 to 10 degrees out of square, else 0; the hook
    released, at rest and where the test left it."""
    hook = objects["hook"]
    if _pull["phase"] != "done" or not _pull["passed"] or not _ready(objects) or not hook.resting():
        return 0.0
    off = _yaw_off_deg(hook)
    if off <= SQUARE_DEG + EPS:
        return 1.0
    return 0.5 if off <= CROOKED_DEG + EPS else 0.0


def success(objects):
    """The hook plate flush on the face, held tight under the pull test, square to the line, let go and still."""
    _update(objects)
    return bool(_done(objects) == 1.0)


def score(objects):
    _update(objects)
    _count_still(objects)
    _pull_step(objects)
    return _done(objects)


@parts.on_step
def _measure_grip(sim) -> None:
    from yam_arena import analysis

    base = sim.env.unwrapped
    origin = parts._np(base.scene.env_origins[0])
    st = sim.states()
    boxes = [(c, r, s) for _, c, r, s in _hook_boxes(st["hook"])]
    hulls = []
    for arm in ("left_robot", "right_robot"):
        asset = base.scene[arm]
        poses = parts._np(asset.data.body_link_pose_w)[0]
        for i, n in enumerate(asset.body_names):
            if n in ("tip_left", "tip_right", "gripper"):
                m = np.eye(4)
                m[:3, :3] = _rot(poses[i, 3:])
                m[:3, 3] = poses[i, :3] - origin
                hulls += analysis.world_hulls({n: m})
    _grip["clear"] = bool(hulls) and all(analysis.hulls_to_box(hulls, c, r, s)[0] > CLEAR for c, r, s in boxes)


def _set_pull(sim, force: float) -> None:
    import torch

    base = sim.env.unwrapped
    asset = base.scene["hook"]
    hook = sim.states()["hook"]
    at = np.asarray(hook.pos) + _rot(hook.quat_xyzw) @ np.asarray(PULL_AT) + parts._np(base.scene.env_origins[0])
    dev = base.device
    asset.permanent_wrench_composer.set_forces_and_torques_index(
        forces=torch.tensor([[[0.0, 0.0, force]]], dtype=torch.float32, device=dev),
        positions=torch.tensor([[[*at]]], dtype=torch.float32, device=dev),
        body_ids=[list(asset.body_names).index("base")], env_ids=torch.tensor([0], dtype=torch.int32, device=dev),
        is_global=True)
    _pull["applied"] = force


@parts.on_step
def _apply_pull(sim) -> None:
    """The pull test's force: on while the test is in its pull phase (set each step at the upright's foot), else off."""
    want = PULL_N if _pull.get("phase") == "pull" else 0.0
    if want or _pull.get("applied"):
        _set_pull(sim, want)


@parts.on_reset
def _reset_grip(sim) -> None:
    _grip["clear"] = None
    _pull.clear()
    _pull["phase"] = "idle"
    _set_pull(sim, 0.0)


# --- demo: open loop from the published start layout ------------------------------------------------------------------
TIP = 0.013  # finger tips below the TCP
GRIP_UP = 0.010  # finger tips 10 mm above the handle's lower end
TIP_BELOW_TCP = TIP + GRIP_UP + HANDLE_Z0  # driver tip below the TCP while held
HANDLE_GRIP = -0.0135  # pads 27 mm apart on the 35 mm handle
HANDLE_LOOSE = -0.025  # 50 mm: round the handle without touching it
UPRIGHT_GRIP = -0.0008
RECESS_FLOOR = FACE_Z + PLATE[2] + HEAD_H - 0.003
# Each stroke turns the driver 190 degrees clockwise, from yaw +145 to -45 degrees: the right wrist's range there.
YAW_A, YAW_MID, YAW_B = math.radians(145.0), math.radians(50.0), math.radians(-45.0)
YAW_PICK = YAW_A - math.pi / 2  # the cross tip looks the same a quarter turn round
N_STROKES = 7  # 7 x 190 = 1330 degrees: 3.9 mm commanded for the 3 mm gap (the arm and ratchet lose some)


def _driver_plan():
    hx, hy = HOLDER_XY
    bx, by = HOLE
    # the held driver's tip settles 0.6 mm short in x and 0.2 mm in y of where the arm is sent here (the arm's
    # measured tracking offset at this pose, open loop): aim that much past the head's centre
    ax, ay = bx + 0.0006, by + 0.0002
    pick_z = TOP + HOLDER[2] + GRIP_UP + TIP
    carry = TOP + 0.150
    seat = RECESS_FLOOR + TIP_BELOW_TCP - 0.004  # 4 mm below seated: a light push keeps the tip in
    w = [
        Waypoint(1.5, (hx, hy, carry), HANDLE_LOOSE, "over the screwdriver", YAW_PICK),
        Waypoint(1.5, (hx, hy, pick_z), HANDLE_LOOSE, "down round the handle", YAW_PICK),
        Waypoint(0.8, (hx, hy, pick_z), HANDLE_GRIP, "grip the handle", YAW_PICK),
        Waypoint(1.5, (hx, hy, carry), HANDLE_GRIP, "lift it out", YAW_PICK),
        Waypoint(2.0, (bx, by, carry), HANDLE_GRIP, "over the bolt", YAW_PICK),
        Waypoint(1.5, (bx, by, carry), HANDLE_GRIP, "a quarter turn, blades on the recess", YAW_A),
        Waypoint(1.5, (ax, ay, seat + 0.008), HANDLE_GRIP, "tip just over the head", YAW_A),
        Waypoint(1.5, (ax, ay, seat), HANDLE_GRIP, "into the recess", YAW_A),
    ]
    w.append(Waypoint(1.0, (ax, ay, seat), HANDLE_GRIP, "wait for the pull", YAW_A))
    n_seat = len(w)
    for k in range(N_STROKES):  # noqa: B007
        w += [  # two halves: a waypoint turns the short way round, so no step may exceed 180 degrees
            Waypoint(1.25, (ax, ay, seat), HANDLE_GRIP, f"turn {k + 1}", YAW_MID),
            Waypoint(1.25, (ax, ay, seat), HANDLE_GRIP, f"turn {k + 1}", YAW_B),
        ]
        if k < N_STROKES - 1:
            w += [
                Waypoint(0.75, (ax, ay, seat), HANDLE_GRIP, f"swing back {k + 1}", YAW_MID),
                Waypoint(0.75, (ax, ay, seat), HANDLE_GRIP, f"swing back {k + 1}", YAW_A),
            ]
    end = YAW_B
    w += [
        Waypoint(0.6, (ax, ay, seat + 0.004), HANDLE_GRIP, "ease the push", end),
        Waypoint(2.5, (ax, ay, carry), HANDLE_GRIP, "lift the driver off", end),
        Waypoint(2.0, (hx, hy, carry), HANDLE_GRIP, "hold the driver clear, over its holder", end),
        Waypoint(10.0, (hx, hy, carry), HANDLE_GRIP, "wait (the pull test runs)", end),
    ]
    return w, n_seat


def _duration(ws):
    return sum(x.duration for x in ws)


def demo():
    right, n_seat = _driver_plan()
    # the left arm pinches the hook's upright and pulls it outward while the right arm turns, then lets go once the
    # driver is off the bolt
    t_seated = _duration(right[:n_seat]) - 1.0
    t_turned = _duration(right[: n_seat + 4 * N_STROKES - 2]) + 3.1
    # Leaning 45 degrees (finger tips toward the bolt, fingers closing across the upright's 5 mm) keeps the left
    # forearm clear of the right one: straight down, the two wrists 8 cm apart collide.
    x, y = HOLE[0], HOLE[1] + UPRIGHT_Y
    lean = dict(tilt=45.0, heading=-math.pi / 2)
    yaw = -math.pi / 2  # fingers close along x (+90 puts joint6 at its limit)
    z = FACE_Z + 0.030  # finger tips about 21 mm above the face, on the upright's lower half
    pull = 0.035  # commanded: the soft arm lifts the hook end less, and the bolt end about 3 mm

    def lw(d, pos, f, label):
        return tilt.Waypoint(d, pos, f, label, yaw, **lean)

    left = [
        lw(t_seated - 3.5, (x, y + 0.04, z + 0.05), -0.010, "over the hook"),
        lw(1.5, (x, y, z), -0.010, "down"),
        lw(0.8, (x, y, z), UPRIGHT_GRIP, "pinch the upright"),
        lw(1.2, (x, y, z + pull), UPRIGHT_GRIP, "pull outward"),
    ]
    left.append(lw(t_turned - _duration(left), (x, y, z + pull), UPRIGHT_GRIP, "keep pulling"))
    left += [
        lw(1.0, (x, y, z), UPRIGHT_GRIP, "ease off"),
        lw(0.6, (x, y, z), -0.010, "let go"),
        lw(1.5, (x, y + 0.04, z + 0.07), -0.010, "up"),
        lw(9.0, (x, y + 0.04, z + 0.07), -0.010, "wait clear (the pull test runs)"),
    ]
    return {"left": left, "right": right}
