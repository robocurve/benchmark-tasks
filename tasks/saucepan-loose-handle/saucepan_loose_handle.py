"""Three small stainless saucepans stand in a row on the counter in front of a cold, unplugged double hot plate, their
black handles pointing toward the robot. Asked for: two pans on the hot plate's two burners (for soup and a sauce) and
the third in the cupboard tray, an aluminium quarter-sheet tray taped to the counter at the right.

One pan's handle screw has worked loose: its handle can be rocked up about 5 degrees against the pan before it binds.
Untouched, it rests on its lower stop and looks the same as the two sound handles. Which pan it is changes from trial to
trial (the episode seed); nothing else does.

World frame: x forward from the arm bases, y left, z up, metres. Pans (6 in across, 3 in tall, 0.59 kg) at x 0.28,
y +0.18 / 0 / -0.18. Hot plate 22.9 x 41.9 x 7.6 cm centred at (0.50, 0); burner plates at (0.50, +0.10) and
(0.50, -0.10), their tops 7.6 cm above the counter. Tray 33 x 24.4 x 2.8 cm centred at (0.34, -0.43).
"""

from __future__ import annotations

import math
import os
import pathlib
import sys
from types import SimpleNamespace

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).parent))
import bodykit  # noqa: E402
import layout as L  # noqa: E402
from bodykit import Body, BoxShape, CylShape  # noqa: E402
from yam_arena.task_file import Box, Cylinder  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
TOP = L.TOP
pandemo_task = True  # pandemo.py plays this file's stage-by-stage demo

STEEL = (0.80, 0.80, 0.82)
PHENOLIC = (0.07, 0.07, 0.08)
IRON = (0.16, 0.16, 0.17)
HOUSING = (0.09, 0.09, 0.10)
ALU = (0.70, 0.71, 0.72)
DOT = (0.85, 0.10, 0.10)
WHITE = (0.92, 0.92, 0.92)
BEAD_T, BEAD_H = 0.005, 0.008  # the rolled rim: 5 mm thick, the top 8 mm of the wall (estimate)


# -- the pans ----------------------------------------------------------------------------------------------------------
def _ry(deg: float) -> tuple[float, float, float, float]:
    a = math.radians(deg) / 2
    return (0.0, math.sin(a), 0.0, math.cos(a))


def _grip_shapes(origin) -> list:
    """The phenolic grip, from the screw at ``origin`` (pan frame) outward along -x, rising GRIP_PITCH_DEG."""
    p = math.radians(L.GRIP_PITCH_DEG)
    c = np.asarray(origin, float) + np.array([-math.cos(p), 0.0, math.sin(p)]) * L.GRIP_LEN / 2
    return [BoxShape(tuple(c.tolist()), (L.GRIP_LEN, L.GRIP_W, L.GRIP_T), PHENOLIC, quat_xyzw=_ry(L.GRIP_PITCH_DEG))]


def _pan_shapes(with_grip: bool, dot: bool) -> list:
    n = L.WALL_SEGS
    chord = 2 * L.PAN_R * math.tan(math.pi / n) * 1.08
    wall_h = L.PAN_H - BEAD_H
    out = [CylShape((0.0, 0.0, L.FLOOR_T / 2), L.PAN_R - L.WALL_T, L.FLOOR_T, STEEL)]
    for k in range(n):
        a = 2 * math.pi * (k + 0.5) / n
        ca, sa = math.cos(a), math.sin(a)
        rw = L.PAN_R - L.WALL_T / 2
        out.append(BoxShape((rw * ca, rw * sa, wall_h / 2), (L.WALL_T, chord, wall_h), STEEL, yaw=a))
        rb = L.PAN_R - L.WALL_T / 2  # the bead stands 1 mm proud of the wall inside and out
        out.append(BoxShape((rb * ca, rb * sa, L.PAN_H - BEAD_H / 2), (BEAD_T, chord, BEAD_H), STEEL, yaw=a))
    # the steel bracket welded to the wall, out to the screw
    out.append(BoxShape(((-L.PAN_R + L.HINGE_X) / 2 - 0.002, 0.0, L.HINGE_Z), (-L.HINGE_X - L.PAN_R + 0.006, 0.016,
                                                                              0.008), STEEL))
    if with_grip:
        out += _grip_shapes((L.HINGE_X, 0.0, L.HINGE_Z))
    # the tester's marker: a numbered paper dot stuck under the base (visual only; nobody sees it from above)
    out.append(CylShape((0.0, 0.0, -0.0004), 0.010, 0.0006, DOT if dot else (0.95, 0.95, 0.95), collide=False))
    return out


def _grip_com() -> np.ndarray:
    p = math.radians(L.GRIP_PITCH_DEG)
    return np.array([-math.cos(p), 0.0, math.sin(p)]) * L.GRIP_LEN / 2


_HANDLE_C = np.array([L.HINGE_X, 0.0, L.HINGE_Z]) + _grip_com()
FIRM_COM = tuple(((L.BODY_MASS * np.array([0.0, 0.0, 0.030]) + L.HANDLE_MASS * _HANDLE_C) / L.PAN_MASS).tolist())
PAN_NAMES = ("pan_loose", "pan_a", "pan_b")


def _build() -> list:
    objs: list = []
    # the hot plate (fixed): housing and the two cast-iron plates
    objs.append(Box("hot_plate", pos=(L.PLATE_C[0], L.PLATE_C[1], TOP + L.PLATE_H / 2), color=HOUSING,
                    size=(L.PLATE_D, L.PLATE_W, L.PLATE_H)))
    for i, (bx, by, br) in enumerate(L.BURNERS):
        objs.append(Cylinder(f"burner_{'left' if i == 0 else 'right'}", pos=(bx, by, TOP + L.PLATE_H + L.BURNER_T / 2),
                             color=IRON, radius=br, height=L.BURNER_T))
    # its two temperature dials with their scale marks and the red power light, on the front face (visual only)
    face: list = [BoxShape((-0.001, 0.0, 0.052), (0.002, 0.010, 0.006), (0.90, 0.08, 0.05), collide=False)]
    for dy in L.DIALS_Y:
        face.append(CylShape((-L.DIAL_D / 2, dy, L.DIAL_Z), L.DIAL_R, L.DIAL_D, (0.12, 0.12, 0.13), collide=False,
                             axis="X"))
        face.append(BoxShape((-L.DIAL_D - 0.001, dy, L.DIAL_Z + 0.010), (0.002, 0.004, 0.012), WHITE, collide=False))
        for k in range(5):
            a = math.radians(-120 + 60 * k)
            face.append(BoxShape((-0.001, dy + 0.025 * math.sin(a), L.DIAL_Z + 0.025 * math.cos(a)),
                                 (0.002, 0.003, 0.003), WHITE, collide=False))
    objs.append(Body.build("hot_plate_dials", pos=(L.FRONT_EDGE_X, L.PLATE_C[1], TOP), mass=0.05, shapes=face,
                           color=HOUSING, kinematic=True))
    # the fix-it tray (fixed: taped down), with the screwdriver lying along the back of its floor
    tx, ty = L.TRAY_C
    blade = L.DRIVER_LEN - L.DRIVER_HANDLE
    objs.append(Body.build("screwdriver", pos=(L.DRIVER_X, ty, TOP + L.TRAY_T), mass=0.08, kinematic=True, shapes=[
        CylShape((0.0, -L.DRIVER_HANDLE / 2, L.DRIVER_R), L.DRIVER_R, L.DRIVER_HANDLE, (0.85, 0.12, 0.08), axis="Y"),
        CylShape((0.0, blade / 2, L.DRIVER_R), 0.003, blade, STEEL, axis="Y")], color=(0.85, 0.12, 0.08)))
    objs.append(Box("tray_floor", pos=(tx, ty, TOP + L.TRAY_T / 2), color=ALU, size=(L.TRAY_L, L.TRAY_W, L.TRAY_T)))
    rim_t = (L.TRAY_L - L.TRAY_IN_L) / 2
    for nm, dx, sz in (("tray_back", +1, (rim_t, L.TRAY_W, L.TRAY_H)), ("tray_front", -1, (rim_t, L.TRAY_W, L.TRAY_H))):
        objs.append(Box(nm, pos=(tx + dx * (L.TRAY_L - rim_t) / 2, ty, TOP + L.TRAY_H / 2), color=ALU, size=sz))
    rim_w = (L.TRAY_W - L.TRAY_IN_W) / 2
    for nm, dy in (("tray_left", +1), ("tray_right", -1)):
        objs.append(Box(nm, pos=(tx, ty + dy * (L.TRAY_W - rim_w) / 2, TOP + L.TRAY_H / 2), color=ALU,
                        size=(L.TRAY_IN_L, rim_w, L.TRAY_H)))
    # the pans: the loose one (body + grip on a hinge) and two sound ones (one body each)
    y0 = L.SLOTS_Y[0]
    objs.append(Body.build("pan_loose", pos=(L.PAN_X, y0, TOP), mass=L.BODY_MASS, shapes=_pan_shapes(False, True),
                           color=STEEL, friction=L.PAN_FRICTION, solver_iterations=32, com=(0.0, 0.0, 0.030)))
    anchor = (L.PAN_X + L.HINGE_X, y0, TOP + L.HINGE_Z)
    objs.append(Body.build("handle", pos=anchor, mass=L.HANDLE_MASS, shapes=_grip_shapes((0.0, 0.0, 0.0)),
                           color=PHENOLIC, friction=L.GRIP_FRICTION, solver_iterations=32,
                           com=tuple(_grip_com().tolist()), ignore=("pan_loose",)))
    bodykit.hinge("loose_screw", "pan_loose", "handle", anchor=anchor, axis="Y", limits_deg=(0.0, L.PLAY_DEG),
                  damping=L.PLAY_DAMPING)
    for name, y in zip(("pan_a", "pan_b"), L.SLOTS_Y[1:]):
        objs.append(Body.build(name, pos=(L.PAN_X, y, TOP), mass=L.PAN_MASS, shapes=_pan_shapes(True, False),
                               color=STEEL, friction=L.PAN_FRICTION, solver_iterations=32, com=FIRM_COM))
    return objs


OBJECTS = _build()
NAMES = [o.name for o in OBJECTS]
SPEC = {o.name: o for o in OBJECTS}
BODY_NAMES = ("pan_loose", "handle", "pan_a", "pan_b")


# -- the hidden fact: which slot holds the loose-handled pan -----------------------------------------------------------
def loose_slot(seed: int) -> int:
    """0 (left), 1 (middle) or 2 (right): the episode seed picks it. Nothing else changes."""
    return int(seed) % 3


def poses(seed: int) -> dict:
    s = loose_slot(seed)
    ident = (0.0, 0.0, 0.0, 1.0)
    z = TOP + 0.0003
    out = {"pan_loose": (np.array([L.PAN_X, L.SLOTS_Y[s], z]), ident),
           "handle": (np.array([L.PAN_X + L.HINGE_X, L.SLOTS_Y[s], z + L.HINGE_Z]), ident)}
    for name, i in zip(("pan_a", "pan_b"), [i for i in range(3) if i != s]):
        out[name] = (np.array([L.PAN_X, L.SLOTS_Y[i], z]), ident)
    return out


# -- geometry the scorer reads -------------------------------------------------------------------------------------------
_TIP_LOCAL = np.array([L.HINGE_X, 0.0, L.HINGE_Z]) + 2 * _grip_com()


def handle_angle(st) -> float:
    """Degrees the loose grip has risen against its pan about the screw (0 on its lower stop)."""
    rel = st["pan_loose"].rotation().T @ st["handle"].rotation()
    return math.degrees(math.atan2(rel[0, 2], rel[0, 0]))


def handle_tip(st, pan: str) -> np.ndarray:
    if pan == "pan_loose":
        return st["handle"].pos + st["handle"].rotation() @ (2 * _grip_com())
    return st[pan].pos + st[pan].rotation() @ _TIP_LOCAL


def _rest(st, pan: str) -> bool:
    return st[pan].tilt() < math.radians(10.0) and st[pan].resting(0.02, 0.5)


def burner_of(st, pan: str) -> int | None:
    """Index of the burner the pan sits on: base centre within 25 mm of the plate's centre, base on the plate (within
    10 mm of its top), upright within 10 degrees."""
    p = st[pan].pos
    for i, (bx, by, _) in enumerate(L.BURNERS):
        if (math.hypot(p[0] - bx, p[1] - by) <= 0.025 and abs(p[2] - (TOP + L.BURNER_TOP)) <= 0.010
                and st[pan].tilt() < math.radians(10.0)):
            return i
    return None


def on_hot_plate(st, pan: str) -> bool:
    """The pan stands on the hot plate: base centre over the housing's footprint, base 60 mm or more above the counter."""
    p = st[pan].pos
    return (abs(p[0] - L.PLATE_C[0]) <= L.PLATE_D / 2 and abs(p[1] - L.PLATE_C[1]) <= L.PLATE_W / 2
            and p[2] >= TOP + 0.060)


def touches_plate(st, pan: str) -> bool:
    """The pan's base is on the hot plate: over its footprint and within 10 mm of the plates' top."""
    return on_hot_plate(st, pan) and st[pan].pos[2] <= TOP + L.BURNER_TOP + 0.010


def loose_at_start(st) -> bool:
    """The loose pan still stands at its start place: base centre within START_TOL of it, base on the counter (within
    10 mm), upright within 10 degrees. Fails closed when the start place is unknown."""
    s = _EP.get("start")
    if s is None:
        return False
    p = st["pan_loose"].pos
    return (math.hypot(p[0] - s[0], p[1] - s[1]) <= L.START_TOL and abs(p[2] - TOP) <= 0.010
            and st["pan_loose"].tilt() < math.radians(10.0))


def handle_in(st, pan: str, burner: int) -> bool:
    """The handle's tip is behind the hot plate's front edge and not over the other burner's plate."""
    tip = handle_tip(st, pan)
    ox, oy, orad = L.BURNERS[1 - burner]
    return tip[0] >= L.FRONT_EDGE_X and math.hypot(tip[0] - ox, tip[1] - oy) > orad


def in_tray(st, pan: str) -> bool:
    """Base wholly inside the tray's rim (base centre at least one pan radius from each inner wall), on the tray floor
    (within 10 mm), upright within 10 degrees."""
    p = st[pan].pos
    hx, hy = L.TRAY_IN_L / 2 - L.PAN_R, L.TRAY_IN_W / 2 - L.PAN_R
    return (abs(p[0] - L.TRAY_C[0]) <= hx and abs(p[1] - L.TRAY_C[1]) <= hy
            and abs(p[2] - (TOP + L.TRAY_T)) <= 0.010 and st[pan].tilt() < math.radians(10.0))


def where(st, pan: str) -> str:
    b = burner_of(st, pan)
    if b is not None:
        return "left burner" if b == 0 else "right burner"
    if in_tray(st, pan):
        return "tray"
    if on_hot_plate(st, pan):
        return "hot plate, off the burner marks"
    return "elsewhere"


# -- the hands ---------------------------------------------------------------------------------------------------------
def finger_points(p, r, half: float) -> list[np.ndarray]:
    """Points on both fingers of a gripper whose TCP is at ``p`` with rotation ``r`` and pads ``half`` m off centre."""
    out = []
    for side in (-1.0, 1.0):
        for lx, ly, lz in ((half + 0.003, 0.0, 0.013), (half + 0.003, 0.009, 0.004), (half + 0.003, -0.009, 0.004),
                           (half + 0.005, 0.0, -0.010), (half + 0.009, 0.0, -0.035), (half + 0.016, 0.0, -0.060)):
            out.append(np.asarray(p, float) + np.asarray(r, float) @ np.array([side * lx, ly, lz]))
    return out


def hand_points(base) -> list[np.ndarray]:
    from yam_arena import ik, station

    pts = []
    for arm in ("left", "right"):
        robot = base.scene[f"{arm}_robot"]
        qj = bodykit.as_numpy(robot.data.joint_pos)[0]
        names = list(robot.data.joint_names)
        q6 = np.array([qj[names.index(f"joint{i}")] for i in range(1, 7)])
        half = abs(float(qj[names.index("joint7")]))
        p, r = ik.tcp_pose(q6, station.BASES[arm])
        pts += finger_points(p, r, half)
    if not pts or not all(np.all(np.isfinite(q)) for q in pts):
        raise ValueError("a gripper position is missing or not finite")
    return pts


_GRIP_HALF = np.array([L.GRIP_LEN, L.GRIP_W, L.GRIP_T]) / 2
_GRIP_R = bodykit.rotation(_ry(L.GRIP_PITCH_DEG))


def clearance(st, points) -> float:
    """Smallest distance (m) from the finger points to any pan (each body taken as a solid 6 in x 3 in cylinder, so a
    finger inside a pan counts as touching it) or any handle (its box)."""
    best = float("inf")
    for pan in PAN_NAMES:
        for q in points:
            loc = st[pan].to_local(q)
            dr = max(0.0, math.hypot(loc[0], loc[1]) - L.PAN_R)
            dz = max(0.0, abs(loc[2] - L.PAN_H / 2) - L.PAN_H / 2)
            best = min(best, math.hypot(dr, dz))
            if pan == "pan_loose":
                h = st["handle"]
                hl = _GRIP_R.T @ (h.to_local(q) - _grip_com())
            else:
                hl = _GRIP_R.T @ (loc - _HANDLE_C)
            best = min(best, float(np.linalg.norm(np.maximum(0.0, np.abs(hl) - _GRIP_HALF))))
    return best


# -- episode state and the score ---------------------------------------------------------------------------------------
_EP: dict = {}
_DBG = open(os.environ["PAN_DEBUG"], "w") if os.environ.get("PAN_DEBUG") else None
HANDS_CLEAR = 0.03  # every finger point at least 30 mm from every pan and handle ...
CLEAR_STEPS = 20  # ... for the last 1 s (20 control steps)
FAIL_STEPS = 20  # the loose pan standing still on the hot plate for 1 s ends the run's credit


def _blank() -> None:
    _EP.clear()
    _EP.update(step=0, clear_hist=[], st=None, clear=float("inf"), on_plate=0, failed=False, angle=0.0, angle_max=0.0,
               report={}, start=None, plate_touched=False, raised=False, rocked=False, rock_max=0.0, rocked_step=None)


_blank()


@bodykit.on_reset
def _reset() -> None:
    keep = {k: _EP.get(k) for k in ("variant",)}
    _blank()
    _EP.update(keep)


def evaluate(st, clear_hist, failed: bool, rocked: bool) -> dict:
    places = {n: where(st, n) for n in PAN_NAMES}
    clear = len(clear_hist) >= CLEAR_STEPS and min(clear_hist[-CLEAR_STEPS:]) >= HANDS_CLEAR
    parts = {}
    for pan in ("pan_a", "pan_b"):
        b = burner_of(st, pan)
        on = b is not None and _rest(st, pan)
        parts[pan] = (0.3 if on else 0.0) + (0.1 if on and handle_in(st, pan, b) else 0.0)
    firm_in_tray = any(in_tray(st, p) for p in ("pan_a", "pan_b"))
    tray = 0.2 if (in_tray(st, "pan_loose") and _rest(st, "pan_loose") and not firm_in_tray) else 0.0
    score = 0.0 if (failed or not clear or not rocked) else round(parts["pan_a"] + parts["pan_b"] + tray, 3)
    return dict(score=score, failed_loose_on_plate=failed, loose_handle_rocked=rocked, hands_clear=clear, places=places,
                firm_parts=parts, loose_in_tray=tray > 0,
                tips_x=[round(float(handle_tip(st, p)[0]), 3) for p in ("pan_a", "pan_b")],
                min_clear_mm=round(min(clear_hist[-CLEAR_STEPS:]) * 1000, 1) if clear_hist else None,
                rock_max_deg=round(float(_EP.get("rock_max", 0.0)), 2), rocked_step=_EP.get("rocked_step"),
                variant=_EP.get("variant"))


@bodykit.on_step
def _watch(env) -> None:
    from yam_arena import scene as _scene

    st = _scene.object_states(env, NAMES)
    _EP["st"] = st
    try:
        clear = clearance(st, hand_points(env.unwrapped))
    except Exception as err:  # noqa: BLE001  (fails closed: a missing hand counts as a hand on a pan)
        clear = 0.0
        if _DBG is not None and not _EP.get("hand_err"):
            _DBG.write(f"hand points failed: {err!r}\n")
            _EP["hand_err"] = True
    _EP["clear"] = clear
    _EP["clear_hist"].append(clear)
    if len(_EP["clear_hist"]) > CLEAR_STEPS:
        _EP["clear_hist"].pop(0)
    if on_hot_plate(st, "pan_loose") and st["pan_loose"].resting(0.02, 0.5):
        _EP["on_plate"] += 1
    else:
        _EP["on_plate"] = 0
    if _EP["on_plate"] >= FAIL_STEPS:
        _EP["failed"] = True
    a = handle_angle(st)
    _EP["angle"] = a
    _EP["angle_max"] = max(_EP["angle_max"], a)
    # the test rule: the loose grip rocked up ROCK_DEG and back down to ROCK_BACK_DEG against its pan, while that pan
    # stands at its start place, before any pan has touched the hot plate
    if any(touches_plate(st, n) for n in PAN_NAMES):
        _EP["plate_touched"] = True
    if not _EP["plate_touched"] and loose_at_start(st) and math.isfinite(a):
        _EP["rock_max"] = max(_EP["rock_max"], a)
        if a >= L.ROCK_DEG:
            _EP["raised"] = True
        elif _EP["raised"] and a <= L.ROCK_BACK_DEG and not _EP["rocked"]:
            _EP["rocked"], _EP["rocked_step"] = True, _EP["step"]
    else:
        _EP["raised"] = False
    _EP["step"] += 1
    if _DBG is not None and _EP["step"] % 4 == 0:
        _debug(st)


def report(objects=None) -> dict:
    st = _EP.get("st")
    if st is None:
        return dict(score=0.0)
    return evaluate(st, _EP["clear_hist"], _EP["failed"], bool(_EP.get("rocked")))


def score(objects) -> float:
    """Read at the end of the trial (see task.json "scoring")."""
    rep = report(objects)
    _EP["report"] = rep
    if _DBG is not None:
        _DBG.write(f"score report: {rep}\n")
        _DBG.flush()
    return float(rep["score"])


def distance(objects) -> float:
    st = _EP.get("st")
    if st is None:
        return 1.0
    p = st["pan_loose"].pos
    return float(math.hypot(p[0] - L.TRAY_C[0], p[1] - L.TRAY_C[1]))


def _debug(st) -> None:
    pans = " ".join(f"{n}=({st[n].pos[0]:.3f},{st[n].pos[1]:.3f},{st[n].pos[2] - TOP:.3f}) t{math.degrees(st[n].tilt()):.0f}"
                    for n in PAN_NAMES)
    _DBG.write(f"{_EP['step']:4d} angle={_EP['angle']:5.2f} {pans} clear={_EP['clear']:.3f} "
               f"plate={_EP['on_plate']} failed={_EP['failed']} rocked={_EP['rocked']}\n")
    _DBG.flush()


@bodykit.on_reset_env
def _lay_out(emb, seed):
    """Put each pan at its slot for this seed, let everything settle, return the first observation."""
    import torch
    from yam_arena import inspect_embodiment as ie
    from yam_arena import scene as _scene

    seed = int(os.environ.get("PAN_VARIANT_SEED", seed if seed is not None else 0))
    base = emb._base
    _blank()
    for name, (pos, quat) in poses(seed).items():
        obj = base.scene[name]
        pose = obj.data.default_root_pose.torch.clone()
        pose[:, :3] = torch.tensor(np.asarray(pos, float), dtype=pose.dtype, device=pose.device) + base.scene.env_origins
        pose[:, 3:7] = torch.tensor(quat, dtype=pose.dtype, device=pose.device)
        obj.write_root_pose_to_sim_index(root_pose=pose)
        obj.write_root_velocity_to_sim_index(root_velocity=torch.zeros_like(obj.data.default_root_vel.torch))
    for _ in range(max(1, round(getattr(ie, "SETTLE_S", 0.5) / base.step_dt))):
        emb._env.step(emb._ready)
    for _ in range(getattr(ie, "RESET_RENDERS", 3)):
        base.sim.render()
    _blank()
    _EP["variant"] = f"seed {seed}: loose handle on the pan in slot {loose_slot(seed) + 1}"
    st = _scene.object_states(emb._env, NAMES)
    _EP["st"] = st
    _EP["start"] = np.array(st["pan_loose"].pos[:2], float)  # the loose pan's start place, for the test rule
    if _DBG is not None:
        _DBG.write(_EP["variant"] + "\n")
        _debug(st)
    print(f"pandemo: {_EP['variant']}", flush=True)
    return emb._observation()


# -- the scripted demo: plans stage by stage from the pans' poses and, while it rocks a handle, the handle's angle
#    against its pan, both read from the simulator; it moves the pans only through the grippers, by contact ------------
from pandemo import Planner, demo, install as _install_demo  # noqa: E402,F401

PLANNER = Planner(SimpleNamespace(PAN_NAMES=PAN_NAMES, _EP=_EP))


@bodykit.on_reset
def _new_planner() -> None:
    global PLANNER
    PLANNER = Planner(SimpleNamespace(PAN_NAMES=PAN_NAMES, _EP=_EP))


def next_stage(st):
    return PLANNER.next_stage(st)


_install_demo()
