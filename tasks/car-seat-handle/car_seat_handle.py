"""An empty infant car seat (carrier) stands on the station table, foot end toward the robot, carry handle upright.

The carrier is a rigid plastic tub on two runners. Its U-shaped carry handle turns on a hub at each side. Each handle
arm carries a release button on its outer face, 10 cm up from the hub. The button, its stem and a lock pin are one
sprung part: at rest the pin sits in a hole of the fixed hub cap, and the handle cannot turn. Pressed in 6 mm, the pin
is clear. The hub cap has holes at 0 (carry), 35, 65 and 85 degrees (rear). A pin that is let go between holes rides
the cap's face and drops into the next hole it reaches.
The task: turn the handle back from carry to the rear position (85 degrees, toward the head end), leave it locked
there, do not move the seat, and move both grippers clear.

Frame: x forward from the arm bases, y left, z up, metres.
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import bodykit  # noqa: E402  (hooks yam_arena so multi-shape bodies and joints are simulated)
from bodykit import Body, BoxShape, CylShape, plate  # noqa: E402
from yam_arena.constants import FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint, grip_rotation  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

bodykit.allow_matrix_grips()

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SCORE_OVER = "end"
CONTROL_HZ = 20.0
FAIL = os.environ.get("CSH_FAIL", "")  # a wrong outcome on purpose (README): fixed, early, nopress, holding

# -- geometry (shell frame: origin on the hub axis midway between the hubs; x toward the head end, y left, z up) -------
HX, HZ = 0.290, 0.280  # nominal hub axis: 29 cm in front of the arm bases, 28 cm above the table
SHELL_X = (-0.300, 0.400)  # foot end to head end (70 cm)
SHELL_HALF_W = 0.185  # side walls' outer faces; with the hubs and handle the seat is 44 cm wide
ARM_Y, ARM_T, ARM_W = 0.211, 0.016, 0.035  # handle arm: centre plane, thickness (y), width
HANDLE_R = 0.300  # hub axis to the middle of the top bar
BTN_R = 0.100  # button cap centre, up the arm from the hub axis
PIN_R = 0.045  # lock pin centre, from the hub axis
CAP_Y = (0.2245, 0.2285)  # the fixed hub cap's inner and outer faces (|y|)
CAP_R = (0.039, 0.051)  # its notched ring (the pin's track)
PIN = (0.005, 0.006, 0.005)  # tangential, along the hub axis, radial (m)
BTN = (0.015, 0.006, 0.025)  # cap: across the arm, proud of it, along it
BTN_TRAVEL = 0.0065  # the button's stop (it unlocks at 4 mm: the pin's 4 mm engagement)
BTN_K, BTN_PRELOAD = 1500.0, 0.001  # N/m, and the spring's pull 1 mm past the rest stop (1.5 N at rest, 11 N pressed)
NOTCHES = (0.0, 35.0, 65.0, 85.0)  # degrees back from upright
REAR = NOTCHES[-1]
HOLE_W = 0.007  # hole width at the pin's radius (pin 5 mm)
HOLE_HALF = math.degrees(math.asin(HOLE_W / 2 / PIN_R))
LIMITS = (-5.0, 86.0)  # the handle's stops: the shell rim in front, the head end behind
HINGE_DAMPING = 0.004  # N m s per degree
MASS = {"shell": 4.30, "handle": 0.45, "button": 0.040}  # button: cap, stem and pin in one part

SHELL_C = (0.30, 0.31, 0.33)  # dark grey plastic
PAD_C = (0.16, 0.17, 0.19)
HANDLE_C = (0.20, 0.21, 0.23)
GRIP_C = (0.10, 0.10, 0.11)
BTN_C = (0.80, 0.35, 0.10)  # orange buttons
CAP_C = (0.55, 0.56, 0.58)


def _ry(a: float) -> np.ndarray:
    return np.array([[math.cos(a), 0.0, math.sin(a)], [0.0, 1.0, 0.0], [-math.sin(a), 0.0, math.cos(a)]])


def _rz(a: float) -> np.ndarray:
    return np.array([[math.cos(a), -math.sin(a), 0.0], [math.sin(a), math.cos(a), 0.0], [0.0, 0.0, 1.0]])


def _yquat(a: float) -> tuple[float, float, float, float]:
    return (0.0, math.sin(a / 2), 0.0, math.cos(a / 2))


def _zquat(a: float) -> tuple[float, float, float, float]:
    return (0.0, 0.0, math.sin(a / 2), math.cos(a / 2))


def _ring_spans() -> list[tuple[float, float]]:
    """Angular spans (degrees) of the hub cap's solid ring between its holes."""
    lo, hi = LIMITS[0] - 8.0, LIMITS[1] + 8.0
    cuts = [(n - HOLE_HALF, n + HOLE_HALF) for n in NOTCHES]
    spans, a = [], lo
    for c0, c1 in cuts:
        spans.append((a, c0))
        a = c1
    spans.append((a, hi))
    return [s for s in spans if s[1] - s[0] > 0.2]


def _ring_boxes(s: float) -> list:
    """The notched ring as boxes of at most 4 degrees each, on the side ``s`` (+1 left, -1 right)."""
    out = []
    y = s * (CAP_Y[0] + CAP_Y[1]) / 2
    rm = (CAP_R[0] + CAP_R[1]) / 2
    for a0, a1 in _ring_spans():
        n = max(1, int(math.ceil((a1 - a0) / 4.0)))
        for k in range(n):
            b0, b1 = a0 + (a1 - a0) * k / n, a0 + (a1 - a0) * (k + 1) / n
            mid = math.radians((b0 + b1) / 2)
            half = math.radians((b1 - b0) / 2)
            width = 2 * PIN_R * math.tan(half) + (0.0006 if (k > 0 and k < n - 1) else 0.0)
            out.append(BoxShape((rm * math.sin(mid), y, rm * math.cos(mid)),
                                (width, CAP_Y[1] - CAP_Y[0], CAP_R[1] - CAP_R[0]), CAP_C, quat_xyzw=_yquat(mid)))
    return out


def _shell_shapes() -> list:
    x0, x1 = SHELL_X
    xc, L = (x0 + x1) / 2, x1 - x0
    z_tab = -HZ
    wall_top = -0.030
    sh = [BoxShape((xc, s * 0.130, z_tab + 0.015), (L, 0.050, 0.030), PAD_C) for s in (1, -1)]  # runners
    sh.append(BoxShape((xc, 0.0, z_tab + 0.040), (L - 0.04, 0.340, 0.020), SHELL_C))  # floor
    for s in (1, -1):
        sh.append(BoxShape((xc, s * (SHELL_HALF_W - 0.0075), (z_tab + 0.030 + wall_top) / 2),
                           (L, 0.015, wall_top - z_tab - 0.030), SHELL_C))  # side wall
        sh.append(BoxShape((0.0, s * 0.1935, -0.045), (0.100, 0.017, 0.090), SHELL_C))  # hub boss mount
        sh.append(CylShape((0.0, s * 0.1935, 0.0), 0.050, 0.017, SHELL_C, axis="Y"))  # hub boss
        sh.append(CylShape((0.0, s * (CAP_Y[0] + CAP_Y[1]) / 2, 0.0), CAP_R[0], CAP_Y[1] - CAP_Y[0], CAP_C, axis="Y"))
        sh.append(CylShape((0.0, s * 0.207, 0.0), 0.008, 0.036, CAP_C, axis="Y", collide=False))  # axle
        sh += _ring_boxes(s)
    sh.append(BoxShape((x0 + 0.0075, 0.0, (z_tab + 0.030 - 0.070) / 2), (0.015, 0.340, -0.070 - z_tab - 0.030),
                       SHELL_C))  # foot wall
    sh.append(BoxShape((x1 - 0.0075, 0.0, (z_tab + 0.030 - 0.010) / 2), (0.015, 0.340, -0.010 - z_tab - 0.030),
                       SHELL_C))  # head wall
    sh.append(plate((-0.02, 0.0, z_tab + 0.055), (x1 - 0.02, 0.0, -0.030), 0.330, 0.012, (1.0, 0.0, -1.0), PAD_C,
                    collide=False))  # the reclined seat back (cushion side), inside the tub
    sh.append(BoxShape((-0.16, 0.0, z_tab + 0.060), (0.26, 0.330, 0.030), PAD_C, collide=False))  # seat cushion
    return sh


def _handle_shapes() -> list:
    sh = []
    for s in (1, -1):
        r0 = 0.030
        sh.append(BoxShape((0.0, s * ARM_Y, (r0 + HANDLE_R) / 2), (ARM_W, ARM_T, HANDLE_R - r0), HANDLE_C))
        sh.append(CylShape((0.0, s * ARM_Y, 0.0), 0.040, ARM_T, HANDLE_C, axis="Y"))
    sh.append(CylShape((0.0, 0.0, HANDLE_R), 0.016, 2 * ARM_Y + ARM_T, HANDLE_C, axis="Y"))
    sh.append(CylShape((0.0, 0.0, HANDLE_R), 0.019, 0.160, GRIP_C, axis="Y"))  # rubber grip
    return sh


def _btn_rest(s: float) -> np.ndarray:
    """The button's origin (cap centre) at rest, upright handle, shell frame."""
    return np.array([0.0, s * (ARM_Y + ARM_T / 2 + BTN[1] / 2), BTN_R])


def _button_shapes(s: float) -> list:
    c = _btn_rest(s)
    face = ARM_Y + ARM_T / 2  # the arm's outer face (|y|)
    stem_y = face + 0.0005 + 0.0015
    pin_y = CAP_Y[1] - PIN[1] / 2
    return [BoxShape((0.0, 0.0, 0.0), BTN, BTN_C),
            BoxShape((0.0, s * stem_y - c[1], (PIN_R + BTN_R) / 2 - BTN_R), (0.008, 0.003, BTN_R - PIN_R + 0.005),
                     BTN_C),
            BoxShape((0.0, s * pin_y - c[1], PIN_R - BTN_R), PIN, BTN_C)]


SIDES = {"l": 1.0, "r": -1.0}
HUB0 = np.array([HX, 0.0, TOP + HZ])


def _build() -> list:
    objs = [Body.build("shell", pos=tuple(HUB0.tolist()), mass=MASS["shell"], shapes=_shell_shapes(), color=SHELL_C,
                       friction=(0.6, 0.5), solver_iterations=64, com=(0.05, 0.0, -HZ + 0.10)),
            Body.build("handle", pos=tuple(HUB0.tolist()), mass=MASS["handle"], shapes=_handle_shapes(), color=HANDLE_C,
                       friction=(0.7, 0.6), solver_iterations=64, ignore=("shell",))]
    bodykit.hinge("hub", "shell", "handle", anchor=tuple(HUB0.tolist()), axis="Y", limits_deg=LIMITS, spring=0.0,
                  damping=HINGE_DAMPING)
    for k, s in SIDES.items():
        p = HUB0 + _btn_rest(s)
        objs.append(Body.build(f"button_{k}", pos=tuple(p.tolist()), mass=MASS["button"], shapes=_button_shapes(s),
                               color=BTN_C, friction=(0.8, 0.7), solver_iterations=64, ignore=("handle",)))
        lim = (-BTN_TRAVEL, 0.0) if s > 0 else (0.0, BTN_TRAVEL)
        bodykit.slider(f"button_slide_{k}", "handle", f"button_{k}", anchor=tuple(p.tolist()), axis="Y", limits=lim,
                       stiffness=BTN_K, damping=2.0, target=s * BTN_PRELOAD, max_force=60.0)
    return objs


OBJECTS = _build()
NAMES = [o.name for o in OBJECTS]
SPEC = {o.name: o for o in OBJECTS}


# -- reading the mechanism from poses ------------------------------------------------------------------------------------
def handle_deg(shell, handle) -> float:
    """The handle's angle back from upright (degrees, toward the head end)."""
    rel = shell.rotation().T @ handle.rotation()
    return math.degrees(math.atan2(rel[0, 2], rel[2, 2]))


def button_travel(handle, button, k: str) -> float:
    """How far button ``k`` is pressed in (m), 0 at rest."""
    local = handle.rotation().T @ (np.asarray(button.pos, float) - np.asarray(handle.pos, float))
    s = SIDES[k]
    return float(s * (_btn_rest(s)[1] - local[1]))


def shell_yaw(shell) -> float:
    r = shell.rotation()
    return math.atan2(r[1, 0], r[0, 0])


# -- episode state --------------------------------------------------------------------------------------------------------
_EP: dict = {}
_DBG = open(os.environ["CSH_DEBUG"], "w") if os.environ.get("CSH_DEBUG") else None


def _blank() -> None:
    _EP.clear()
    _EP.update(step=0, hands={}, report={}, start=None, now=None, q=0.0, travel={"l": 0.0, "r": 0.0}, q_max=0.0,
               forced=None)


_blank()


@bodykit.on_reset
def _reset() -> None:
    _blank()


def finger_points(p, r, half: float) -> list[np.ndarray]:
    """Points on the two fingers of a gripper whose TCP is at ``p`` with rotation ``r`` and whose pads are each
    ``half`` m from the centre line: pad tips, pad middles and finger roots."""
    pts = []
    for side in (-1.0, 1.0):
        for x, y, z in ((half + 0.002, 0.0, 0.013), (half + 0.002, 0.0077, 0.008), (half + 0.002, -0.0077, 0.008),
                        (half + 0.004, 0.0, -0.005), (half + 0.008, 0.0, -0.030), (half + 0.015, 0.0, -0.055)):
            pts.append(np.asarray(p, float) + np.asarray(r, float) @ np.array([side * x, y, z]))
    return pts


def _hand_points(base) -> dict:
    """Points on each gripper's fingers (world), from the arm's own joint readings."""
    from bodykit import as_numpy
    from yam_arena import ik, station

    out = {}
    for arm in ("left", "right"):
        robot = base.scene[f"{arm}_robot"]
        qj = as_numpy(robot.data.joint_pos)[0]
        names = list(robot.data.joint_names)
        q6 = np.array([qj[names.index(f"joint{i}")] for i in range(1, 7)])
        half = abs(float(qj[names.index("joint7")]))
        p, r = ik.tcp_pose(q6, station.BASES[arm])
        out[arm] = finger_points(p, r, half)
    return out


@bodykit.on_step
def _watch(env) -> None:
    from yam_arena import scene as _scene

    st = _scene.object_states(env, NAMES)
    shell, handle = st["shell"], st["handle"]
    pose = (np.asarray(shell.pos, float).copy(), shell_yaw(shell), float(shell.tilt()))
    if _EP["start"] is None:
        _EP["start"] = pose
    _EP["now"] = pose
    _EP["q"] = handle_deg(shell, handle)
    _EP["q_max"] = max(_EP["q_max"], _EP["q"])
    for k in SIDES:
        _EP["travel"][k] = button_travel(handle, st[f"button_{k}"], k)
        off = min(abs(_EP["q"] - n) for n in NOTCHES)
        if _EP["forced"] is None and _EP["travel"][k] < ENGAGED and off > FORCED_DEG:
            _EP["forced"] = (_EP["step"], k, round(_EP["q"], 2))  # a pin still in its hole, the handle well off it
    try:
        _EP["hands"] = _hand_points(env.unwrapped)
    except Exception as err:  # noqa: BLE001
        if _DBG is not None and not _EP.get("hand_err"):
            _DBG.write(f"hand points failed: {err!r}\n")
            _EP["hand_err"] = True
    _EP["step"] += 1
    if _DBG is not None:
        _debug()


def _debug() -> None:
    rep = _EP.get("report") or {}
    p, yaw, tilt = _EP["now"]
    _DBG.write(f"{_EP['step']:4d} q={_EP['q']:7.2f} btn_l={_EP['travel']['l'] * 1000:5.2f} "
               f"btn_r={_EP['travel']['r'] * 1000:5.2f} shell=({p[0]:.4f},{p[1]:.4f},{p[2]:.4f}) "
               f"yaw={math.degrees(yaw):6.2f} tilt={math.degrees(tilt):5.2f} clr={rep.get('clearance_mm')} "
               f"score={rep.get('score')}\n")
    _DBG.flush()


# -- scoring ---------------------------------------------------------------------------------------------------------------
CLEAR_MM = 10.0  # both grippers at least this far from the handle and its buttons (at the ready pose the arms
#                  pass 15 mm from a handle at 65 degrees, so a wider margin would fail a hand that has let go)
REAR_TOL = 3.0  # degrees
OUT_MM = 1.0  # a button pressed in less than this is out (its pin in a hole)
MOVE_MM, TURN_DEG, TILT_DEG = 30.0, 5.0, 5.0
POINTS = {"angle": 0.5, "locked": 0.3, "seat_still": 0.2}
# Forced lock: a pin at least 2 mm into the hub cap (button in less than 2 mm of its 6) can only sit in a hole, within
# 1.3 degrees of a lock position. A handle turned more than 2.5 degrees off one with a pin that deep has been forced
# past the lock (a real hub would break): the trial scores 0.
ENGAGED, FORCED_DEG = 0.002, 2.5
ANGLE_FROM = 5.0  # angle credit starts here (a locked handle has 1.3 degrees of play at carry)


def _shape_boxes(spec: Body, state) -> list[tuple[np.ndarray, np.ndarray, np.ndarray]]:
    """(centre, rotation, half extents) of every shape of a body (cylinders as their bounding boxes)."""
    r_body = state.rotation()
    out = []
    for sh in spec.shapes:
        if isinstance(sh, BoxShape):
            rs, half = bodykit.rotation(sh.quat()), np.array(sh.size) / 2
        else:
            rs, half = np.eye(3), sh.half_extent()
        out.append((np.asarray(state.pos, float) + r_body @ np.array(sh.centre), r_body @ rs, half))
    return out


def _box_dist(p, box) -> float:
    c, r, half = box
    d = np.abs(r.T @ (np.asarray(p, float) - c)) - half
    return float(np.linalg.norm(np.maximum(d, 0.0)))


def clearance_mm(objects) -> float | None:
    hands = _EP.get("hands") or {}
    if not hands:
        return None
    boxes = []
    for name in ("handle", "button_l", "button_r"):
        boxes += _shape_boxes(SPEC[name], objects[name])
    return 1000.0 * min(_box_dist(p, b) for pts in hands.values() for p in pts for b in boxes)


def report(objects) -> dict:
    shell, handle = objects["shell"], objects["handle"]
    q = handle_deg(shell, handle)
    travel = {k: button_travel(handle, objects[f"button_{k}"], k) for k in SIDES}
    start = _EP.get("start")
    p0, yaw0 = (start[0], start[1]) if start else (np.asarray(shell.pos, float), shell_yaw(shell))
    moved = 1000.0 * float(np.linalg.norm(np.asarray(shell.pos, float)[:2] - p0[:2]))
    turned = abs(math.degrees((shell_yaw(shell) - yaw0 + math.pi) % (2 * math.pi) - math.pi))
    tilt = math.degrees(float(shell.tilt()))
    clr = clearance_mm(objects)
    angle_part = POINTS["angle"] * min(1.0, max(0.0, (q - ANGLE_FROM) / (REAR - ANGLE_FROM)))
    if abs(q - REAR) <= REAR_TOL:
        angle_part = POINTS["angle"]
    at_rear = abs(q - REAR) <= REAR_TOL
    locked = at_rear and all(t * 1000.0 < OUT_MM for t in travel.values())
    still = moved < MOVE_MM and turned < TURN_DEG and tilt < TILT_DEG
    total = angle_part + POINTS["locked"] * locked + POINTS["seat_still"] * (locked and still)
    clear = clr is None or clr >= CLEAR_MM
    forced = _EP.get("forced")
    if forced is not None:
        total = 0.0
    out = {"forced": forced, "handle_deg": round(q, 2), "button_mm": {k: round(v * 1000, 2) for k, v in travel.items()},
           "locked_rear": bool(locked), "shell_moved_mm": round(moved, 1), "shell_turned_deg": round(turned, 2),
           "shell_tilt_deg": round(tilt, 2), "clearance_mm": None if clr is None else round(clr, 1),
           "grippers_clear": bool(clear), "score": round(total if clear else 0.0, 3)}
    return out


def score(objects) -> float:
    """0.5 x (handle angle back - 5) / 80 degrees, full 0.5 within 3 degrees of the rear position; +0.3 if the handle is
    at the rear position with both buttons out (each pin in its rear hole); +0.2 if, also, the seat moved under 30 mm,
    turned under 5 degrees and tilts under 5 degrees. 0 if the handle was ever forced past a lock; 0 unless both grippers end at least 10 mm from the handle and
    its buttons."""
    rep = report(objects)
    _EP["report"] = rep
    return float(rep["score"])


def success(objects) -> bool:
    return score(objects) >= 1.0 - 1e-9


def distance(objects) -> float:
    return abs(handle_deg(objects["shell"], objects["handle"]) - REAR) / 100.0


# -- the seeded start ------------------------------------------------------------------------------------------------------
VARY_X, VARY_Y, VARY_YAW = 0.040, 0.006, 6.0  # the seat moves up to this in x and y (m) and turns up to this (deg)
# (y stays small: the hubs pass 15 mm from the grippers at the ready pose every trial starts from)
MIN_OFF = 0.025  # and the draw is repeated until one grip point is at least this far off its nominal place in x


def grip_point(s: float, hub, yaw: float, q_deg: float, y: float = 0.212) -> np.ndarray:
    """World point 10 cm up the arm on side ``s``, between its inner face and the button, for the handle at
    ``q_deg``, the seat's hub centre at ``hub`` turned ``yaw``."""
    a = math.radians(q_deg)
    return np.asarray(hub, float) + _rz(yaw) @ np.array([BTN_R * math.sin(a), s * y, BTN_R * math.cos(a)])


def draw(seed: int | None) -> tuple[np.ndarray, float]:
    """The seat's hub centre and turn for a seed (None or 0: the nominal layout)."""
    if not seed:
        return HUB0.copy(), 0.0
    rng = np.random.default_rng(seed)
    while True:
        d = np.array([rng.uniform(-VARY_X, VARY_X), rng.uniform(-VARY_Y, VARY_Y), 0.0])
        yaw = math.radians(rng.uniform(-VARY_YAW, VARY_YAW))
        hub = HUB0 + d
        off = max(abs(float(grip_point(s, hub, yaw, 0.0)[0] - grip_point(s, HUB0, 0.0, 0.0)[0])) for s in (1, -1))
        if off >= MIN_OFF:
            return hub, yaw


def layout(seed: int | None) -> dict:
    hub, yaw = draw(seed)
    q = _zquat(yaw)
    pose = {"shell": (hub, q), "handle": (hub, q)}
    for k, s in SIDES.items():
        pose[f"button_{k}"] = (hub + _rz(yaw) @ _btn_rest(s), q)
    pose["_hub"] = (hub, yaw)
    return pose


@bodykit.on_reset_env
def _lay_out(emb, seed):
    """Put the bodies where this trial's seed says, let them settle, and return the first observation."""
    import torch
    from yam_arena import inspect_embodiment as ie

    base = emb._base
    lay = layout(seed)
    _blank()
    for name, (pos, quat) in lay.items():
        if name.startswith("_"):
            continue
        obj = base.scene[name]
        pose = obj.data.default_root_pose.torch.clone()
        pose[:, :3] = torch.tensor(pos, dtype=pose.dtype, device=pose.device) + base.scene.env_origins
        pose[:, 3:7] = torch.tensor(quat, dtype=pose.dtype, device=pose.device)
        obj.write_root_pose_to_sim_index(root_pose=pose)
        obj.write_root_velocity_to_sim_index(root_velocity=torch.zeros_like(obj.data.default_root_vel.torch))
    for _ in range(max(1, round(getattr(ie, "SETTLE_S", 0.5) / base.step_dt))):
        emb._env.step(emb._ready)
    for _ in range(getattr(ie, "RESET_RENDERS", 3)):
        base.sim.render()
    _blank()
    _EP["seed"] = seed
    if _DBG is not None:
        hub, yaw = lay["_hub"]
        _DBG.write(f"seed {seed}: hub=({hub[0]:.4f},{hub[1]:.4f}) yaw={math.degrees(yaw):.2f}\n")
    return emb._observation()


# -- the scripted demo (reads the seat's pose from the simulator to plan; moves the handle only through the grippers) ---
DOWN = grip_rotation(0.0)  # straight down; the fingers close along world y
OPEN = -0.024  # finger joint target: 48 mm opening (arm and button are 22 mm)
GRIP = -0.002  # 4 mm opening target: the pads stop at 16 mm (button in) and squeeze about 30 N
APPROACH = 0.050  # the hand comes in (and backs off) this far along its own axis
Q_END = 85.5  # the handle is carried to here (the stop is at 86)
Q_EARLY = 45.0  # "early": both hands let go here
SEC_PER_DEG = 0.075
UP_LIFT = float(os.environ.get("CSH_UP_LIFT", "0.015"))  # after backing off, the hand rises this far
UP_OUT = float(os.environ.get("CSH_UP_OUT", "0.09"))  # and moves this far outward, over the handle arm


def lag(q: float) -> float:
    """Degrees the hand's axis trails the arm's normal (keeps the wrist in reach near the rear position)."""
    return -min(28.0, max(0.0, (q - 55.0) * 0.9))


def hand_rot(yaw: float, q: float) -> np.ndarray:
    """TCP rotation: fingers closing along the hub axis, the hand's axis normal to the arm (toward +x at carry,
    straight down at 90 degrees), trailing by lag(q)."""
    return _rz(yaw) @ _ry(math.radians(q - 90.0 + lag(q))) @ DOWN


def ready_pose(arm: str) -> tuple[np.ndarray, np.ndarray]:
    from yam_arena import ik, station
    from yam_arena.constants import READY

    return ik.tcp_pose(np.array(READY, float), station.BASES[arm])


def park(arm: str, seconds: float, finger: float = FINGER_OPEN, label: str = "back to the ready pose") -> Waypoint:
    p, r = ready_pose(arm)
    return Waypoint(seconds, tuple(p.tolist()), finger, label, r)


def placeholder_plans() -> dict[str, list[Waypoint]]:
    return {arm: [park(arm, 1.0, label="look")] for arm in ("left", "right")}


def demo():
    return placeholder_plans()


LOOKS = 1


def arm_plan(arm: str, hub, yaw: float, fail: str = "") -> list[Waypoint]:
    s = 1.0 if arm == "left" else -1.0

    def at(q, back=0.0, lift=0.0, y=0.212):
        r = hand_rot(yaw, q)
        p = grip_point(s, hub, yaw, q, y) - back * r[:, 2] + np.array([0.0, 0.0, lift])
        return tuple(p.tolist()), r

    out = []

    def go(sec, q, finger, label, back=0.0, lift=0.0, y=0.212):
        p, r = at(q, back, lift, y)
        out.append(Waypoint(sec, p, finger, label, r))

    # up beside the seat first, outboard of the hubs (straight from the ready pose, a hand clipped a hub on seed 2)
    go(2.0, 0.0, OPEN, "up beside the seat", back=APPROACH, lift=0.06, y=0.212 + 0.09)
    go(1.5, 0.0, OPEN, "in front of the handle arm", back=APPROACH, lift=0.02)
    go(1.0, 0.0, OPEN, "line up with the arm", back=APPROACH)
    if fail == "nopress":  # wrong on purpose: closed hands push the arms back above the buttons, buttons untouched
        go(0.8, 0.0, 0.0, "close the hand", back=APPROACH, lift=0.10)
        go(1.5, 0.0, 0.0, "fingertips behind the arm", back=0.02, lift=0.10)
        for k in range(1, 9):
            go(0.6, 10.0 * k, 0.0, "push the handle back", back=0.02, lift=0.10)
        go(1.0, 80.0, 0.0, "ease off", back=APPROACH, lift=0.10)
        out.append(park(arm, 2.5))
        out.append(park(arm, 1.0, label="look"))
        return out
    go(1.5, 0.0, OPEN, "round the arm and button")
    go(1.0, 0.0, GRIP, "squeeze: button in")
    go(0.5, 0.0, GRIP, "hold the squeeze")
    q_to = Q_EARLY if fail == "early" else Q_END
    n = int(math.ceil(q_to / 5.0))
    for k in range(1, n + 1):
        go(q_to / n * SEC_PER_DEG, q_to * k / n, GRIP, "swing the handle back, button in")
    go(0.6, q_to, GRIP, "at the rear position")
    if fail == "holding":  # wrong on purpose: the hands stay on the handle
        go(4.0, q_to, GRIP, "hold the squeeze")
        return out
    go(0.8, q_to, OPEN, "let go")
    go(1.5, q_to, OPEN, "back off the arm", back=0.035)
    go(1.2, q_to, OPEN, "out over the handle arm", back=0.035, lift=UP_LIFT, y=0.212 + UP_OUT)
    out.append(park(arm, 2.5))
    out.append(park(arm, 1.0, label="look"))
    return out


def plan_stage(stage: int) -> dict[str, list[Waypoint]]:
    """Both hands' plans, from the seat's pose read from the simulator (or, "fixed", the nominal layout)."""
    if FAIL == "fixed" or _EP.get("start") is None:
        hub, yaw = HUB0.copy(), 0.0
    else:
        hub, yaw = np.asarray(_EP["start"][0], float), float(_EP["start"][1])
    print(f"handledemo: hub centre ({hub[0]:.4f}, {hub[1]:.4f}, {hub[2]:.4f}), turned {math.degrees(yaw):+.2f} deg",
          flush=True)
    fail = "" if FAIL == "fixed" else FAIL
    return {arm: arm_plan(arm, hub, yaw, fail) for arm in ("left", "right")}


import handledemo  # noqa: E402  (the demo's control hooks for yam_scripted)

handledemo.install()
handledemo_task = True
