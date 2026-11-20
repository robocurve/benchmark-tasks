"""A bathroom spray cleaner, put back in the cleaning caddy with its nozzle turned to OFF.

A 32 oz trigger-spray bottle stands on the bathroom counter with its nozzle on SPRAY. The nozzle is a square cap that
turns about the spray axis and clicks into four positions a quarter turn apart: OFF, SPRAY, OFF, STREAM. Left on SPRAY
or STREAM, a small child can work the trigger and spray the cleaner into their eyes or mouth. The caddy (two long
compartments) already holds a second bottle with its nozzle on OFF.

Mechanism: the nozzle is its own body on a hinge about the spray axis. A detent spring (a joint drive re-aimed every
control step at the nearest click, with a stiffness set so its torque follows the detent's profile) pulls it into the
nearest click with at most DETENT_TORQUE; between clicks it is free. Nothing turns the nozzle except contact.

Frame: x forward from the arm bases, y left, z up, metres. All sizes, masses and places: layout.py.
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import bodykit  # noqa: E402  (hooks yam_arena so multi-shape bodies and joints are simulated)
import layout as L  # noqa: E402
from bodykit import Body, BoxShape, CylShape  # noqa: E402
from yam_arena.constants import FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402

bodykit.allow_matrix_grips()

SCENE = "station"
TOP = L.TOP
SCORE_OVER = "end"
CONTROL_HZ = 20.0

WHITE, BLACK, CADDY_C = (0.93, 0.93, 0.91), (0.08, 0.08, 0.09), (0.16, 0.16, 0.17)
OFF_C, SPRAY_C, STREAM_C = (0.85, 0.10, 0.08), (0.95, 0.95, 0.95), (0.15, 0.35, 0.90)
CUP_C, BRUSH_C = (0.98, 0.62, 0.15), (0.30, 0.75, 0.35)


def _box(lo, hi, color, collide: bool = True) -> BoxShape:
    lo, hi = np.asarray(lo, float), np.asarray(hi, float)
    return BoxShape(tuple(((lo + hi) / 2).tolist()), tuple((hi - lo).tolist()), color, collide=collide)


def _cyl(r, z0, z1, color, xy=(0.0, 0.0), collide: bool = True) -> CylShape:
    return CylShape((xy[0], xy[1], (z0 + z1) / 2), r, z1 - z0, color, axis="Z", collide=collide)


NOZ_C = (L.NOZ_FRONT_X + L.NOZ_LEN / 2, 0.0, L.NOZ_AXIS_Z)  # the nozzle body's origin, on the spray axis (bottle frame)


def _trigger_box():
    """The trigger as an oriented box in the bottle frame: centre, axes (along, through, across) and half sizes."""
    p1, p2, width, thick = (np.asarray(L.TRIGGER[0], float), np.asarray(L.TRIGGER[1], float), L.TRIGGER[2],
                            L.TRIGGER[3])
    d = (p2 - p1) / np.linalg.norm(p2 - p1)
    n = np.array([-d[2], 0.0, d[0]])  # in the x-z plane, perpendicular to the trigger's length
    if n[0] < 0:
        n = -n  # thickness goes back (+x) from the front face
    c = (p1 + p2) / 2 + n * thick / 2
    ax = np.column_stack([d, n, [0.0, 1.0, 0.0]])
    ax[:, 2] *= np.sign(np.linalg.det(ax))  # right-handed
    return c, ax, np.array([np.linalg.norm(p2 - p1) / 2, thick / 2, width / 2])


def _bottle_shapes(with_nozzle_marks: str | None = None) -> list:
    """The bottle and the sprayer's fixed parts (bottle frame). ``with_nozzle_marks``: also add the nozzle cap with
    the given word on top (for the second bottle, whose nozzle is not a separate body)."""
    r = L.BOTTLE_R
    s = [_cyl(r, 0.0, 0.195, WHITE), _cyl(0.036, 0.195, 0.213, WHITE), _cyl(0.024, 0.213, 0.226, WHITE),
         _cyl(0.0145, 0.226, L.CLOSURE_Z[0], WHITE), _cyl(L.CLOSURE_R, *L.CLOSURE_Z, BLACK)]
    hw = L.SHROUD_HW
    noz_back = L.NOZ_FRONT_X + L.NOZ_LEN
    s += [_box((-0.012, -0.0125, L.CLOSURE_Z[1]), (0.014, 0.0125, 0.273), BLACK),  # the pump body over the closure
          _box((noz_back, -hw, 0.272), (0.010, hw, L.SHROUD_TOP), BLACK),  # the shroud, nozzle end
          _box((0.010, -0.014, 0.268), (0.0357, 0.014, 0.288), BLACK),  # its tail
          _box((-0.036, -0.006, 0.266), (-0.012, 0.006, 0.273), BLACK)]  # the trigger's hinge web
    c, ax, half = _trigger_box()
    s.append(BoxShape(tuple(c.tolist()), tuple((2 * half).tolist()), BLACK, quat_xyzw=bodykit._quat_from_matrix(ax)))
    if with_nozzle_marks:
        s += [BoxShape(NOZ_C, (L.NOZ_LEN, 2 * L.NOZ_HALF, 2 * L.NOZ_HALF), BLACK)]
        s += _marks(np.array(NOZ_C), with_nozzle_marks)
    return s


def _marks(c, top: str) -> list:
    """Thin printed labels on the nozzle's four side faces (no collision). ``top`` is the word on the top face; the two
    faces a quarter turn away say OFF; the bottom face says the other word."""
    words = {"SPRAY": SPRAY_C, "STREAM": STREAM_C, "OFF": OFF_C}
    if top == "SPRAY":
        up, down, side = "SPRAY", "STREAM", "OFF"
    else:  # OFF on top: SPRAY and STREAM face the sides
        up, down, side = "OFF", "OFF", None
    h, t, ln = L.NOZ_HALF, 0.0006, L.NOZ_LEN * 0.8
    out = [_box(c + (-ln / 2, -h * 0.8, h), c + (ln / 2, h * 0.8, h + t), words[up], collide=False),
           _box(c + (-ln / 2, -h * 0.8, -h - t), c + (ln / 2, h * 0.8, -h), words[down], collide=False)]
    for sy, word in ((1.0, side or "SPRAY"), (-1.0, side or "STREAM")):
        y0, y1 = sorted((sy * h, sy * (h + t)))
        out.append(_box(c + (-ln / 2, y0, -h * 0.8), c + (ln / 2, y1, h * 0.8), words[word], collide=False))
    return out


def _nozzle_shapes() -> list:
    return [BoxShape((0.0, 0.0, 0.0), (L.NOZ_LEN, 2 * L.NOZ_HALF, 2 * L.NOZ_HALF), BLACK),
            _box((-L.NOZ_LEN / 2 - 0.0005, -0.0012, -0.0012), (-L.NOZ_LEN / 2, 0.0012, 0.0012), (0.4, 0.4, 0.4),
                 collide=False)] + _marks(np.zeros(3), "SPRAY")


def _caddy_shapes() -> list:
    w, f = L.CADDY_WALL, L.CADDY_FLOOR
    hx, hy, hz = L.CADDY_W / 2, L.CADDY_L / 2, L.COMP_H + f
    end = (L.CADDY_L - L.COMP_L) / 2
    cw = L.CENTER_W / 2
    s = [_box((-hx, -hy, 0.0), (hx, hy, f), CADDY_C)]
    for sx in (-1.0, 1.0):
        x0, x1 = sorted((sx * hx, sx * (hx - w)))
        s.append(_box((x0, -hy, 0.0), (x1, hy, hz), CADDY_C))  # outer long walls
        x0, x1 = sorted((sx * cw, sx * (cw - w)))
        s.append(_box((x0, -hy, 0.0), (x1, hy, hz), CADDY_C))  # the two walls of the middle divider
    for sy in (-1.0, 1.0):
        y0, y1 = sorted((sy * hy, sy * (hy - end)))
        s.append(_box((-hx, y0, 0.0), (hx, y1, hz), CADDY_C))  # end walls (with the hand holds)
        y0, y1 = sorted((sy * 0.150, sy * 0.170))
        s.append(_box((-cw, y0, hz), (cw, y1, L.HANDLE_TOP), CADDY_C))  # handle posts
    s.append(_box((-0.012, -0.170, L.HANDLE_TOP - 0.030), (0.012, 0.170, L.HANDLE_TOP), CADDY_C))  # the handle grip
    return s


def _cup_shapes() -> list:
    return [_cyl(L.CUP_R, 0.0, L.CUP_H, CUP_C), _box((-0.004, -0.003, L.CUP_H - 0.01), (0.004, 0.003, L.CUP_H + 0.075),
                                                     BRUSH_C)]


def _build() -> list:
    bx, by = L.BOTTLE_XY
    bottle = Body.build("bottle", pos=(bx, by, TOP), mass=L.BOTTLE_MASS, shapes=_bottle_shapes(), color=WHITE,
                        friction=L.BOTTLE_FRICTION, solver_iterations=32, com=L.BOTTLE_COM, contact_offset=0.002)
    noz_w = (bx + NOZ_C[0], by + NOZ_C[1], TOP + NOZ_C[2])
    nozzle = Body.build("nozzle", pos=noz_w, mass=L.NOZ_MASS, inertia=L.NOZ_INERTIA, shapes=_nozzle_shapes(),
                        color=BLACK, friction=(1.0, 0.9), solver_iterations=32, contact_offset=0.001,
                        ignore=("bottle",))
    bodykit.hinge("nozzle_turn", "bottle", "nozzle", anchor=noz_w, axis="X", spring=_k_small(),
                  damping=L.NOZ_FRICTION_DAMP, target_deg=0.0)
    caddy = Body.build("caddy", pos=(L.CADDY_XY[0], L.CADDY_XY[1], TOP), mass=L.CADDY_MASS, shapes=_caddy_shapes(),
                       color=CADDY_C, friction=(0.6, 0.5), solver_iterations=16)
    fx, fy = L.FULL_XY
    other = Body.build("bottle_2", pos=(fx, fy, TOP + L.CADDY_FLOOR), mass=L.BOTTLE_MASS,
                       shapes=_bottle_shapes("OFF"), color=WHITE, friction=L.BOTTLE_FRICTION, solver_iterations=16,
                       com=L.BOTTLE_COM)
    cup = Body.build("cup", pos=(L.CUP_XY[0], L.CUP_XY[1], TOP), mass=0.05, shapes=_cup_shapes(), color=CUP_C,
                     kinematic=True)
    return [bottle, nozzle, caddy, other, cup]


def _k_small() -> float:
    """The detent spring's stiffness at a click (N m per degree): the slope of DETENT_TORQUE * sin(4 * error)."""
    return 4.0 * L.DETENT_TORQUE * math.pi / 180.0


OBJECTS = _build()
NAMES = [o.name for o in OBJECTS]
SPEC = {o.name: o for o in OBJECTS}


# -- reading the scene ------------------------------------------------------------------------------------------------
def nozzle_angle(st) -> float:
    """The nozzle's turn against the bottle about the spray axis, degrees in (-180, 180]: 0 SPRAY, +-90 OFF, 180
    STREAM."""
    r = st["bottle"].rotation().T @ st["nozzle"].rotation()
    return math.degrees(math.atan2(r[2, 1], r[1, 1]))


def nozzle_off(st) -> bool:
    return abs(abs(nozzle_angle(st)) - 90.0) <= L.OFF_WITHIN


def setting(st) -> str:
    a = abs(nozzle_angle(st))
    return "OFF" if abs(a - 90.0) <= 45.0 else ("SPRAY" if a < 45.0 else "STREAM")


def tilt_deg(st, name: str) -> float:
    return math.degrees(math.acos(max(-1.0, min(1.0, float(st[name].rotation()[2, 2])))))


def compartment(st) -> str | None:
    """Which caddy compartment the bottle's base is fully inside ("near", "far"), resting on its floor; else None."""
    loc = st["caddy"].to_local(st["bottle"].pos)
    if not (-0.005 <= loc[2] - L.CADDY_FLOOR <= 0.012):
        return None
    room_x = L.COMP_W / 2 - L.BOTTLE_R
    room_y = L.COMP_L / 2 - L.BOTTLE_R
    for name, cx in (("near", -(L.CENTER_W / 2 + L.COMP_W / 2)), ("far", L.CENTER_W / 2 + L.COMP_W / 2)):
        if abs(loc[0] - cx) <= room_x + 0.002 and abs(loc[1]) <= room_y + 0.002:
            return name
    return None


def finger_points(p, r, half: float) -> list[np.ndarray]:
    pts = []
    for side in (-1.0, 1.0):
        for x, y, z in ((half + 0.002, 0.0, 0.013), (half + 0.002, 0.0077, 0.008), (half + 0.002, -0.0077, 0.008),
                        (half + 0.004, 0.0, -0.005), (half + 0.008, 0.0, -0.030), (half + 0.015, 0.0, -0.055)):
            pts.append(np.asarray(p, float) + np.asarray(r, float) @ np.array([side * x, y, z]))
    return pts


def _hand_points(base) -> list:
    from bodykit import as_numpy
    from yam_arena import ik, station

    out = []
    for arm in ("left", "right"):
        robot = base.scene[f"{arm}_robot"]
        qj = as_numpy(robot.data.joint_pos)[0]
        names = list(robot.data.joint_names)
        q6 = np.array([qj[names.index(f"joint{i}")] for i in range(1, 7)])
        half = abs(float(qj[names.index("joint7")]))
        p, r = ik.tcp_pose(q6, station.BASES[arm])
        out.extend(finger_points(p, r, half))
    if not all(np.all(np.isfinite(p)) for p in out):
        raise ValueError("a gripper position is not finite")
    return out


def _shape_boxes(name: str) -> list[tuple[np.ndarray, np.ndarray]]:
    """Each colliding shape of a body as a box (centre, half sizes) in the body frame (a turned box: its bounding box)."""
    return [(np.asarray(sh.centre, float), sh.half_extent()) for sh in SPEC[name].shapes if getattr(sh, "collide", True)]


def clearance(st, points) -> float:
    """Smallest distance (m) from the finger points to the bottle's and its nozzle's parts (each part's box)."""
    best = float("inf")
    for name in ("bottle", "nozzle"):
        boxes = _SHAPES[name]
        for p in points:
            loc = st[name].to_local(p)
            for c, h in boxes:
                d = np.maximum(0.0, np.abs(loc - c) - h)
                best = min(best, float(np.linalg.norm(d)))
    return best


_SHAPES = {n: _shape_boxes(n) for n in ("bottle", "nozzle")}
TRIG_C, TRIG_AX, TRIG_HALF = _trigger_box()
TRIGGER_GAP = 0.002  # a finger point this close to the trigger counts as pressing it


def on_trigger(st, points) -> bool:
    """Is any finger point within TRIGGER_GAP of the trigger (bottle frame)?"""
    for p in points:
        loc = TRIG_AX.T @ (st["bottle"].to_local(p) - TRIG_C)
        d = np.maximum(0.0, np.abs(loc) - TRIG_HALF)
        if float(np.linalg.norm(d)) <= TRIGGER_GAP:
            return True
    return False


# -- the detent ---------------------------------------------------------------------------------------------------------
def detent_torque(angle_deg: float) -> float:
    """The detent's restoring torque (N m) at a nozzle angle: DETENT_TORQUE * sin(4 * (click - angle)), zero at every
    click and half-way between clicks, at most DETENT_TORQUE 22.5 degrees from a click."""
    click = 90.0 * round(angle_deg / 90.0)
    return L.DETENT_TORQUE * math.sin(math.radians(4.0 * (click - angle_deg)))


def _set_detent(angle_deg: float) -> None:
    """Re-aim the hinge drive at the nearest click with the stiffness that gives detent_torque() here."""
    click = 90.0 * round(angle_deg / 90.0)
    err = click - angle_deg
    k = _k_small() if abs(err) < 0.5 else detent_torque(angle_deg) / err
    bodykit.set_hinge_drive("nozzle_turn", click, max(k, 0.0), L.NOZ_FRICTION_DAMP)


# -- episode state, the score -----------------------------------------------------------------------------------------
_EP: dict = {}
_DBG = open(os.environ["SPRAY_DEBUG"], "w") if os.environ.get("SPRAY_DEBUG") else None
HANDS_CLEAR = 0.05  # every finger point at least 50 mm from the bottle ...
CLEAR_STEPS = 20  # ... for the last 1 s of the trial
UPRIGHT_DEG, FALLEN_DEG = 15.0, 45.0
OTHER_MOVE, OTHER_TILT = 0.020, 10.0  # the second bottle: moved at most 20 mm, tilted at most 10 degrees
CADDY_MOVE = 0.050  # the caddy moved at most 50 mm


def _blank() -> None:
    _EP.clear()
    _EP.update(step=0, clear_hist=[], st=None, clear=float("inf"), start=None, trigger_breach=False, report={})


_blank()


@bodykit.on_reset
def _reset() -> None:
    start = _EP.get("start")
    _blank()
    _EP["start"] = start


def evaluate(st, clear_hist, start, breach: bool) -> dict:
    ang = nozzle_angle(st)
    off = nozzle_off(st)
    tilt = tilt_deg(st, "bottle")
    comp = compartment(st)
    on_counter = float(st["bottle"].pos[2]) >= TOP - 0.01
    clear = len(clear_hist) >= CLEAR_STEPS and min(clear_hist[-CLEAR_STEPS:]) >= HANDS_CLEAR
    other_moved = (float(np.linalg.norm(np.asarray(st["bottle_2"].pos) - start["bottle_2"])) if start else 0.0)
    caddy_moved = (float(np.linalg.norm(np.asarray(st["caddy"].pos) - start["caddy"])) if start else 0.0)
    other_ok = other_moved <= OTHER_MOVE and tilt_deg(st, "bottle_2") <= OTHER_TILT and caddy_moved <= CADDY_MOVE
    gate = off and not breach and tilt <= FALLEN_DEG and on_counter and clear and other_ok
    score = 0.0
    if gate:
        if comp is not None:
            score = 1.0 if tilt <= UPRIGHT_DEG else 0.7
        else:
            score = 0.4
    return dict(score=score, nozzle_deg=round(ang, 1), setting=setting(st), nozzle_off=off, trigger_pressed=breach,
                tilt_deg=round(tilt, 1), compartment=comp, on_counter=on_counter, hands_clear=clear,
                min_clear_mm=round(min(clear_hist[-CLEAR_STEPS:]) * 1000, 1) if clear_hist else None,
                bottle_2_moved_mm=round(other_moved * 1000, 1), bottle_2_tilt=round(tilt_deg(st, "bottle_2"), 1),
                caddy_moved_mm=round(caddy_moved * 1000, 1),
                bottle_xy=[round(float(v), 4) for v in st["bottle"].pos[:2]])


@bodykit.on_step
def _watch(env) -> None:
    from yam_arena import scene as _scene

    st = _scene.object_states(env, NAMES)
    _EP["st"] = st
    ang = nozzle_angle(st)
    _set_detent(ang)
    try:
        pts = _hand_points(env.unwrapped)
        clear = clearance(st, pts)
        pressed = on_trigger(st, pts)
    except Exception as err:  # noqa: BLE001  (fails closed: not released, and a press on the trigger)
        clear, pressed = 0.0, True
        if _DBG is not None and not _EP.get("hand_err"):
            _DBG.write(f"hand points failed: {err!r}\n")
            _EP["hand_err"] = True
    if pressed and not nozzle_off(st):
        if not _EP["trigger_breach"] and _DBG is not None:
            _DBG.write(f"{_EP['step']:4d} TRIGGER touched with the nozzle at {ang:.1f} deg\n")
        _EP["trigger_breach"] = True
    _EP["clear"] = clear
    _EP["clear_hist"].append(clear)
    if len(_EP["clear_hist"]) > CLEAR_STEPS:
        _EP["clear_hist"].pop(0)
    _EP["step"] += 1
    if _DBG is not None and _EP["step"] % 2 == 0:
        _debug(st, ang)


def report(objects=None) -> dict:
    st = _EP.get("st")
    if st is None:
        return dict(score=0.0)
    return evaluate(st, _EP["clear_hist"], _EP.get("start"), _EP["trigger_breach"])


def score(objects) -> float:
    """Read at the end of the trial (see task.json "scoring"). 0 unless the nozzle is within 20 degrees of an OFF
    click, no finger point came within 2 mm of the trigger while the nozzle was not OFF, the bottle is tilted at most
    45 degrees and still on the counter, every finger point has been at least 50 mm from the bottle for the last 1 s,
    the second bottle moved at most 20 mm and tilted at most 10 degrees, and the caddy moved at most 50 mm. Then 1.0 with
    the bottle's base fully inside a caddy compartment, on its floor, tilted at most 15 degrees; 0.7 the same but tilted
    15 to 45 degrees; 0.4 anywhere else."""
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
    p = np.asarray(st["bottle"].pos, float)
    return float(np.linalg.norm(p[:2] - np.asarray(L.SLOT_XY)))


def _debug(st, ang) -> None:
    b = st["bottle"].pos
    _DBG.write(f"{_EP['step']:4d} noz={ang:7.2f} tq={detent_torque(ang):+.4f} off={int(nozzle_off(st))} "
               f"b=({b[0]:.4f},{b[1]:.4f},{b[2] - TOP:.4f}) tilt={tilt_deg(st, 'bottle'):5.1f} "
               f"comp={compartment(st)} clear={_EP['clear']:.3f} trig={int(_EP['trigger_breach'])} "
               f"b2=({st['bottle_2'].pos[0]:.4f},{st['bottle_2'].pos[1]:.4f}) caddy=({st['caddy'].pos[0]:.4f},"
               f"{st['caddy'].pos[1]:.4f})\n")
    _DBG.flush()


# -- the fixed start --------------------------------------------------------------------------------------------------
def poses() -> dict:
    bx, by = L.BOTTLE_XY
    ident = (0.0, 0.0, 0.0, 1.0)
    return {
        "bottle": (np.array([bx, by, TOP + 0.0005]), ident),
        "nozzle": (np.array([bx + NOZ_C[0], by, TOP + 0.0005 + NOZ_C[2]]), ident),
        "caddy": (np.array([L.CADDY_XY[0], L.CADDY_XY[1], TOP + 0.0005]), ident),
        "bottle_2": (np.array([L.FULL_XY[0], L.FULL_XY[1], TOP + L.CADDY_FLOOR + 0.001]), ident),
        "cup": (np.array([L.CUP_XY[0], L.CUP_XY[1], TOP]), ident),
    }


@bodykit.on_reset_env
def _lay_out(emb, seed):
    """Put every body at its one fixed place (no seed changes it), let them settle, return the first observation."""
    import torch
    from yam_arena import inspect_embodiment as ie
    from yam_arena import scene as _scene

    base = emb._base
    _blank()
    for name, (pos, quat) in poses().items():
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
    st = _scene.object_states(emb._env, NAMES)
    _EP["st"] = st
    _EP["start"] = {"bottle_2": np.asarray(st["bottle_2"].pos, float).copy(),
                    "caddy": np.asarray(st["caddy"].pos, float).copy()}
    if _DBG is not None:
        _DBG.write(f"seed {seed}: fixed layout\n")
        _debug(st, nozzle_angle(st))
    return emb._observation()


# -- the scripted demo (reads object poses from the simulator to plan; moves the parts only through the grippers) ------
VARIANT = os.environ.get("DEMO_VARIANT", "")  # "" (full), noturn, turnonly, nothing, onearm, trigger (README)
NOZ_OPEN, NOZ_SQUEEZE = -0.020, -(L.NOZ_HALF - 0.002)  # 2 mm past contact on each side of the 15.2 mm cap
NECK_OPEN, NECK_SQUEEZE = -0.032, -(L.CLOSURE_R - 0.003)  # 3 mm past contact on the 31.5 mm closure
TURN_DEG = float(os.environ.get("SPRAY_TURN_DEG", "95"))  # the wrist turns this far: 5 degrees past the click
TURN_SIGN = float(os.environ.get("SPRAY_TURN_SIGN", "1"))
PLACE_AZ = math.radians(float(os.environ.get("SPRAY_PLACE_AZ", "60")))  # the carrying hand's heading at the caddy


def ready_pose(arm: str) -> tuple[np.ndarray, np.ndarray]:
    from yam_arena import ik, station
    from yam_arena.constants import READY

    return ik.tcp_pose(np.array(READY, float), station.BASES[arm])


def park(arm: str, seconds: float, finger: float = FINGER_OPEN, label: str = "back to the ready pose") -> Waypoint:
    p, r = ready_pose(arm)
    return Waypoint(seconds, tuple(p.tolist()), finger, label, r)


def rest(arm: str, seconds: float) -> Waypoint:
    """The end pose: the ready pose drawn back 12 cm and out 4 cm, well away from the caddy."""
    p, r = ready_pose(arm)
    side = 1.0 if arm == "left" else -1.0
    return Waypoint(seconds, tuple((p + np.array([-0.12, 0.04 * side, 0.0])).tolist()), FINGER_OPEN,
                    "back to the ready pose", r)


def demo():
    """Both arms wait at the ready pose; the real plan is made on the first control step (spraydemo.py)."""
    return {arm: [park(arm, 0.5, label="look")] for arm in ("left", "right")}


def _frame(approach, close) -> np.ndarray:
    z = np.asarray(approach, float) / np.linalg.norm(approach)
    x = np.asarray(close, float) / np.linalg.norm(close)
    x = x - z * float(x @ z)
    x /= np.linalg.norm(x)
    return np.column_stack([x, np.cross(z, x), z])


def _rx(a: float) -> np.ndarray:
    return np.array([[1.0, 0.0, 0.0], [0.0, math.cos(a), -math.sin(a)], [0.0, math.sin(a), math.cos(a)]])


def _wp(sec, pos, finger, label, rot) -> Waypoint:
    return Waypoint(float(sec), tuple(np.asarray(pos, float).tolist()), float(finger), label, np.asarray(rot, float))


def _az(a: float) -> np.ndarray:
    """The neck grip's frame for a heading ``a`` (0 along +x, pi/2 along +y): fingers level, closing across."""
    d = np.array([math.cos(a), math.sin(a), 0.0])
    return _frame(d, (math.sin(a), -math.cos(a), 0.0))


def nozzle_moves(arm: str, b_p: np.ndarray) -> list[Waypoint]:
    """Pinch the nozzle's two side faces from the front (fingers along +x, the spray axis), turn the wrist a quarter
    turn about that axis, let go and back straight out."""
    tcp = b_p + np.array([L.NOZ_FRONT_X - L.NOZ_TCP_BACK, 0.0, L.NOZ_AXIS_Z])
    r0 = _frame((1.0, 0.0, 0.0), (0.0, -1.0, 0.0))  # closing across y (wrist roll 0)
    back = tcp - np.array([0.06, 0.0, 0.0])
    moves = [
        _wp(3.0, back + np.array([0.0, 0.0, 0.03]), NOZ_OPEN, "in front of the nozzle", r0),
        _wp(0.8, back, NOZ_OPEN, "in front of the nozzle", r0),
        _wp(1.6, tcp, NOZ_OPEN, "fingers either side of the nozzle", r0),
        _wp(0.8, tcp, NOZ_SQUEEZE, "pinch the nozzle", r0),
    ]
    for k in (1, 2, 3):
        moves.append(_wp(0.8, tcp, NOZ_SQUEEZE, "turn the nozzle", _rx(TURN_SIGN * math.radians(TURN_DEG) * k / 3) @ r0))
    r1 = _rx(TURN_SIGN * math.radians(90.0)) @ r0
    moves += [
        _wp(0.6, tcp, NOZ_SQUEEZE, "turn the nozzle", r1),  # back to the click: the detent holds it there
        _wp(0.8, tcp, NOZ_OPEN, "let go of the nozzle", r1),
        _wp(1.2, back, NOZ_OPEN, "back off", r1),
    ]
    return moves


def neck_pick(b_p: np.ndarray) -> list[Waypoint]:
    r = _az(math.pi / 2)  # from the -y side, closing along x (clear of the trigger in front)
    neck = b_p + np.array([0.0, -0.004, sum(L.CLOSURE_Z) / 2])
    return [
        _wp(3.0, neck - np.array([0.0, 0.07, 0.0]) + np.array([0.0, 0.0, 0.03]), NECK_OPEN, "beside the neck", r),
        _wp(0.8, neck - np.array([0.0, 0.07, 0.0]), NECK_OPEN, "beside the neck", r),
        _wp(1.8, neck, NECK_OPEN, "fingers either side of the neck", r),
        _wp(0.8, neck, NECK_SQUEEZE, "hold the neck", r),
    ]


def trigger_press(b_p: np.ndarray) -> list[Waypoint]:
    """Push the trigger's front face with the closed fingertips from the front, then back away (a wrong move)."""
    c, ax, half = TRIG_C, TRIG_AX, TRIG_HALF
    face = b_p + c - ax[:, 1] * half[1]  # the middle of the trigger's front face
    r = _frame((1.0, 0.0, 0.0), (0.0, 0.0, 1.0))
    tcp = face - np.array([0.013 - 0.002, 0.0, 0.0])  # the fingertips (13 mm past the TCP) 2 mm into the face
    return [
        _wp(3.0, tcp - np.array([0.07, 0.0, 0.0]), 0.0, "in front of the trigger", r),
        _wp(1.5, tcp, 0.0, "press the trigger", r),
        _wp(0.8, tcp, 0.0, "press the trigger", r),
        _wp(1.2, tcp - np.array([0.07, 0.0, 0.0]), 0.0, "back off", r),
    ]


def carry(b_p: np.ndarray) -> list[Waypoint]:
    neck = b_p + np.array([0.0, -0.004, sum(L.CLOSURE_Z) / 2])
    r = _az(math.pi / 2)
    rp = _az(PLACE_AZ)
    lift = neck + np.array([0.0, 0.0, L.LIFT])
    off = np.array([0.0, -0.004, 0.0])  # the TCP's offset from the bottle's axis, turned with the hand
    off_p = _rz(PLACE_AZ - math.pi / 2) @ off
    over = np.array([L.SLOT_XY[0], L.SLOT_XY[1], lift[2]]) + off_p
    down = np.array([over[0], over[1], TOP + L.CADDY_FLOOR + sum(L.CLOSURE_Z) / 2 + 0.004])
    a = rp[:, 2]
    return [
        _wp(2.0, lift, NECK_SQUEEZE, "lift the bottle", r),
        _wp(3.0, over, NECK_SQUEEZE, "over the caddy", rp),
        _wp(0.8, over, NECK_SQUEEZE, "steady", rp),  # spraydemo re-aims here from the bottle's pose
        _wp(2.5, down, NECK_SQUEEZE, "lower into the compartment", rp),
        _wp(0.6, down, NECK_SQUEEZE, "set down", rp),
        _wp(0.8, down, NECK_OPEN, "let go of the bottle", rp),
        _wp(1.2, down - 0.07 * a, NECK_OPEN, "back off", rp),
        _wp(1.0, down - 0.07 * a + np.array([0.0, 0.0, 0.08]), NECK_OPEN, "back off up", rp),
    ]


def _rz(a: float) -> np.ndarray:
    return np.array([[math.cos(a), -math.sin(a), 0.0], [math.sin(a), math.cos(a), 0.0], [0.0, 0.0, 1.0]])


def plan_from_state(st) -> dict[str, list[Waypoint]]:
    """Both hands' plans, from the bottle's start pose."""
    b_p = np.asarray(st["bottle"].pos, float)
    b_p = np.array([b_p[0], b_p[1], TOP])
    if VARIANT == "nothing":
        return {arm: [park(arm, 30.0, label="wait")] for arm in ("left", "right")}
    if VARIANT == "onearm":  # the right hand does it all; the left hand never moves
        right = nozzle_moves("right", b_p)
        right.append(_wp(1.5, b_p + np.array([-0.08, -0.12, 0.30]), NECK_OPEN, "round to the neck", _az(math.pi / 2)))
        right += neck_pick(b_p) + carry(b_p) + [rest("right", 2.5), rest("right", 1.5)]
        return {"left": [park("left", 1.0, label="wait")], "right": right}
    right = neck_pick(b_p)
    if VARIANT == "trigger":  # wrong on purpose: a closed fingertip presses the trigger while the nozzle is on SPRAY
        right = trigger_press(b_p) + right
    t_hold = sum(w.duration for w in right)
    if VARIANT == "noturn":
        left = [park("left", 1.0, label="wait")]
    else:
        left = [park("left", max(0.5, t_hold - 5.4), label="wait")] + nozzle_moves("left", b_p) + [park("left", 2.5)]
    t_left = sum(w.duration for w in left) - 2.5  # the right hand lifts once the left hand has backed off
    if VARIANT == "noturn":
        t_left = t_hold
    right.append(_wp(max(0.5, t_left - t_hold), right[-1].pos, NECK_SQUEEZE, "hold the neck", right[-1].yaw))
    if VARIANT == "turnonly":  # the nozzle is turned OFF but the bottle stays on the counter
        right += [_wp(0.8, right[-1].pos, NECK_OPEN, "let go of the bottle", right[-1].yaw),
                  _wp(1.2, np.asarray(right[-1].pos) - np.array([0.0, 0.07, 0.0]), NECK_OPEN, "back off",
                      right[-1].yaw), rest("right", 2.5), rest("right", 1.5)]
    else:
        right += carry(b_p) + [rest("right", 2.5), rest("right", 1.5)]
    tl, tr = sum(w.duration for w in left), sum(w.duration for w in right)
    print(f"spraydemo: variant '{VARIANT}', bottle ({b_p[0]:.3f}, {b_p[1]:.3f}), plan {max(tl, tr):.1f} s", flush=True)
    return {"left": left, "right": right}


def place_correction(st) -> np.ndarray:
    """How far the bottle's base is from over the compartment's middle (world xy, m; z 0)."""
    p = np.asarray(st["bottle"].pos, float)
    want = np.asarray(st["caddy"].pos, float) + st["caddy"].rotation() @ np.array(
        [-(L.CENTER_W / 2 + L.COMP_W / 2), 0.0, 0.0])
    return np.array([want[0] - p[0], want[1] - p[1], 0.0])


import spraydemo  # noqa: E402  (the demo's control hooks for yam_scripted)

spraydemo.install()
spraydemo_task = True
