"""On the station table: an open-frame 5 W diode laser engraver (Creality CR-Laser Falcon 5W: space-grey frame, grey
gantry), unplugged, with a 3 mm aluminium plate under the work and a 3 mm basswood plywood sheet lying loose on it. The
laser module hangs from its fixing plate near the front of the frame, 12 mm too high: its fixed focus bar is stowed level
on its magnet and both hand screws are tight (both on its -y side).

The robot turns the bar down, holds the module while the other hand loosens both hand screws, lowers the module until
the hanging bar just touches the sheet, tightens both screws, turns the bar back up onto its magnet and lets go.

Simplified physics: rigid bodies. The hand screws' clamp, the screws' turning by the hand, the bar's magnet and the
screws' thread friction are rules in this file (task.json, "fidelity").
"""

from __future__ import annotations

import importlib.abc
import importlib.util
import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))  # lets this file import helpers beside it

import focus_usd as F  # noqa: E402
import steptime  # noqa: E402
from scenekit import UsdBody, before_reset, on_reset, on_step  # noqa: E402
from yam_arena.analysis import box_signed_distance, link_hulls  # noqa: E402
from yam_arena.constants import FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
HZ = 20.0

# -- layout (station frame, m) -------------------------------------------------------------------------------------------
MOD_XY = F.MODULE_XY                                   # the module's axis (0.30, 0.05)
PLATE_C = np.array([(F.PLATE[0] + F.PLATE[1]) / 2, (F.PLATE[2] + F.PLATE[3]) / 2])
SHEET_C = np.array([0.35, 0.05])                       # sheet x 0.25..0.45, y -0.10..+0.20
SHEET_TOP0 = F.PLATE[4] + F.SHEET[2]                   # 0.6 cm above the table
G = F.bar_reach()                                      # the hanging bar's lowest point below the shield edge (5 mm)
E0 = 0.012                                             # start: the hanging bar's end would be 12 mm above the sheet
START_Z = SHEET_TOP0 + E0 + G                          # shield edge above the table at the start (2.3 cm)
SCREWS = ("screw_u", "screw_l")

USD = F.build()

# -- the rules' constants (task.json, "fidelity") -----------------------------------------------------------------------
K_LOCK, D_LOCK = 3.0e5, 1000.0        # the clamp: a stiff drive holds the slide where it was when a screw took hold
D_FREE = 5.0                          # N s/m: the free slide's damping (estimate)
LOCK_DEG = 30.0                       # a screw within this of its snug stop clamps the module; both: locked
PINCH = 14.0                          # N: each pad of one gripper pressing a screw's head this hard turns it: enough
                                      # friction (0.8 at the 7 mm rim, two pads) for a finger-tight screw's 0.15 N m
K_SCREW, D_SCREW = 2.0, 0.02          # the screw's drive (N m/rad, N m s/rad)
T_TRACK = 0.5                         # N m: the drive's cap while a pinch turns the screw
T_REST = 0.15                         # N m: a finger-tight screw's breakaway, holding a screw nobody pinches (estimate)
MAG_DEG, MAG_TORQUE = 5.0, 0.02       # the magnet holds the bar within 5 deg of stowed with up to 0.02 N m (estimate)
K_MAG, D_BAR = 1.0, 1.0e-4
SCREW_MIN = math.radians(F.SCREW_LOW + 5.0)

OBJECTS = [
    UsdBody("engraver", usd=USD["machine"], pos=(MOD_XY[0], MOD_XY[1], TOP), size=(0.568, 0.632, 0.17), mass=None,
            friction=(0.5, 0.45)),
    UsdBody("plate", usd=USD["plate"], pos=(PLATE_C[0], PLATE_C[1], TOP), size=(0.30, 0.40, F.PLATE[4]), mass=None,
            friction=(0.5, 0.45)),
    UsdBody("sheet", usd=USD["sheet"], pos=(SHEET_C[0], SHEET_C[1], TOP + F.PLATE[4]), size=F.SHEET,
            mass=F.SHEET_MASS, friction=(0.5, 0.45), solver_iters=32),
    UsdBody("laser", usd=USD["laser"], pos=(MOD_XY[0], MOD_XY[1], TOP + START_Z), size=(0.046, 0.05, F.BODY_H),
            articulation=True, fix_root=True,
            joint_pos=(("slide", 0.0), ("bar", 0.0), ("screw_u", 0.0), ("screw_l", 0.0)),
            drives=(("slide", K_LOCK, D_LOCK, 2000.0), ("bar", K_MAG, D_BAR, MAG_TORQUE),
                    ("screw_u", K_SCREW, D_SCREW, T_REST), ("screw_l", K_SCREW, D_SCREW, T_REST)),
            friction=(0.8, 0.7), solver_iters=32, torsional_patch=0.006),
]

# -- scoring terms (task.json, "scoring") ---------------------------------------------------------------------------------
IN_FOCUS = 0.0005
NEAR_FOCUS = 0.002
STOW_TOL = 10.0                       # degrees from the stowed angle
DROP = 0.002                          # m of descent while no gripper touches the module (body or either screw)
DROP_SPEED = 0.03                     # m/s: the module descending faster than this, touched or not, has dropped
                                      # (read as more than 1.5 mm of descent in one 50 ms control step)
F_TOUCH = 0.05                        # N: contact
SHEET_MOVED = 0.003                   # m, any corner
CLEAR = 0.03                          # fingertips this far from the module, its bar and screws, and the sheet
STILL_STEP = 0.01 / HZ                # 1 cm/s, as displacement over one 50 ms control step
HOLD = round(1.0 * HZ)                # the harness's 1 s (SUCCESS_HOLD_S)
GRIP_LINKS = ("gripper", "tip_left", "tip_right")
FINGERS = [f"{s}:{n}" for s in ("L", "R") for n in GRIP_LINKS]
PADS = {"left": (1, 2), "right": (4, 5)}   # tip_left, tip_right of each side in FINGERS
SENSORS = ("module",) + SCREWS


def _grid_points(h, step=0.006) -> np.ndarray:
    lo, hi = h.vertices.min(0), h.vertices.max(0)
    g = np.stack(np.meshgrid(*[np.arange(lo[i], hi[i] + step, step) for i in range(3)], indexing="ij"), -1).reshape(-1, 3)
    inside = (g @ h.planes[:, :3].T - h.planes[:, 3]).max(axis=1) <= 0
    return np.vstack([h.vertices, g[inside]])


GRIP_POINTS = {link: np.vstack([_grid_points(h) for h in link_hulls()[link]]) for link in GRIP_LINKS}


def _rot(q) -> np.ndarray:
    x, y, z, w = np.asarray(q, float) / np.linalg.norm(q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def _ry(deg) -> np.ndarray:
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def _np(x) -> np.ndarray:
    x = getattr(x, "torch", x)
    if hasattr(x, "detach"):
        return x.detach().cpu().numpy().astype(np.float64)
    return np.asarray(x.numpy() if hasattr(x, "numpy") else x, dtype=np.float64)


def _turn_about_y(R0, R1) -> float:
    """The rotation from R0 to R1 (world frame), as its component about world +y (rad)."""
    dR = R1 @ R0.T
    s = 0.5 * np.array([dR[2, 1] - dR[1, 2], dR[0, 2] - dR[2, 0], dR[1, 0] - dR[0, 1]])
    n = float(np.linalg.norm(s))
    if n < 1e-12:
        return 0.0
    ang = math.atan2(n, 0.5 * (float(np.trace(dR)) - 1.0))
    return float(ang * s[1] / n)


# Boxes for the fingertip clearance, each in its link's authored frame (centre, rotation, size).
_T_BAR = F.BAR_L - F.BAR_W / 2
BOXES = {
    "module": [(np.array([0, 0, F.SHIELD_H + (F.BODY_H - F.SHIELD_H) / 2]), np.eye(3),
                np.array([F.BODY, F.BODY, F.BODY_H - F.SHIELD_H])),
               (np.array([0, 0, F.SHIELD_H / 2]), np.eye(3), np.array([F.SHIELD_X, F.SHIELD_Y, F.SHIELD_H]))],
    "bar": [(F.BAR_PIVOT + (_T_BAR / 2) * F.bar_dir(F.STOW_DEG), _ry(F.STOW_DEG),
             np.array([F.BAR_W, F.BAR_COL_T, _T_BAR + F.BAR_W])),
            (F.BAR_PIVOT, _ry(F.STOW_DEG), np.array([F.BAR_HEAD, F.BAR_COL_T, F.BAR_HEAD_H]))],
    "screw_u": [(F.screw_axis("screw_u"), np.eye(3), np.array([2 * F.HEAD_R, F.HEAD_T + 2 * F.SHANK, 2 * F.HEAD_R]))],
    "screw_l": [(F.screw_axis("screw_l"), np.eye(3), np.array([2 * F.HEAD_R, F.HEAD_T + 2 * F.SHANK, 2 * F.HEAD_R]))],
}
SHEET_CORNERS = np.array([[sx * F.SHEET[0] / 2, sy * F.SHEET[1] / 2, F.SHEET[2]] for sx in (-1, 1) for sy in (-1, 1)])
TIP_XY = MOD_XY + (F.BAR_PIVOT + F.BAR_L * F.bar_dir(F.HANG_DEG))[:2]     # the hanging bar's end (station xy)


# -- per-physics-step monitor: fingers on the module and its screws, the drop rule, the pinch forces ----------------------
_MON: dict = {"view": None, "err": None, "asset": None, "slide": None, "touch_steps": 0, "polls": 0, "peak": 0.0,
              "pad_sum": None}


def _enable_contact_processing() -> None:
    try:
        import carb

        carb.settings.get_settings().set_bool("/physics/disableContactProcessing", False)
    except Exception:  # pragma: no cover
        pass


def _make_view(mgr) -> None:
    from isaaclab.sim.utils.stage import get_current_stage
    from pxr import UsdPhysics

    sv = mgr.get_physics_sim_view()
    if sv is None:
        return
    stage = get_current_stage()
    sens, filt = {}, {}
    for prim in stage.Traverse():
        p = prim.GetPath().pathString
        if "/env_0/" not in p or not prim.HasAPI(UsdPhysics.RigidBodyAPI):
            continue
        parts = p.split("/")
        name = parts[-1]
        if name in SENSORS and parts[-2] == "laser":
            sens[name] = p
        elif name in GRIP_LINKS and ("/LeftArm/" in p or "/RightArm/" in p):
            filt[("L:" if "/LeftArm/" in p else "R:") + name] = p
    if len(sens) != len(SENSORS) or len(filt) != len(FINGERS):
        raise RuntimeError(f"focus monitor: sensors {sorted(sens)}, fingers {sorted(filt)}")
    view = sv.create_rigid_contact_view([sens[n] for n in SENSORS], filter_patterns=[[filt[n] for n in FINGERS]] * len(SENSORS),
                                        max_contact_data_count=16 * len(FINGERS) * len(SENSORS))
    _MON.update(view=view)
    print(f"[focus] contact monitor: module body and both screws x {len(FINGERS)} finger links", flush=True)


def _poll(mgr) -> None:
    """After every physics step (240 Hz): is any gripper touching the module or a screw, how far has the module
    descended since it was last touched, and how hard does each pad press each screw?"""
    try:
        if _MON["view"] is None:
            _make_view(mgr)
        if _MON["view"] is None:
            return
        f = _np(_MON["view"].get_contact_force_matrix(dt=mgr.get_physics_dt())).reshape(len(SENSORS), len(FINGERS), 3)
    except Exception as e:  # pragma: no cover
        if _MON["err"] is None:
            _MON["err"] = repr(e)
            print(f"[focus] monitor error: {e!r}", flush=True)
        return
    mag = np.linalg.norm(f, axis=2)                     # sensors x fingers
    _MON["pad_sum"] = mag if _MON["pad_sum"] is None else _MON["pad_sum"] + mag
    peak = float(mag.max())
    touch = peak > F_TOUCH
    _MON["polls"] += 1
    _MON["peak"] = max(_MON["peak"], peak)
    _MON["touch_steps"] += int(touch)
    asset = _MON["asset"]
    if asset is None or not _T:
        return
    q = float(_np(asset.data.joint_pos)[0][_MON["slide"]])
    T = _T
    if touch or T["z_hi"] is None:
        T["z_hi"] = q
    else:
        T["z_hi"] = max(T["z_hi"], q)
        T["drop_max"] = max(T["drop_max"], T["z_hi"] - q)
        if T["z_hi"] - q >= DROP:
            _fail(f"dropped: the module descended {1000 * (T['z_hi'] - q):.1f} mm with no gripper on it")


_PHYSX_MGR = "isaaclab_physx.physics.physx_manager"


def _patch_manager(mod) -> None:
    mgr = mod.PhysxManager
    if getattr(mgr.step, "_focus_hooked", False):
        return
    base_cfg = mgr._configure_physics.__func__
    base_step = mgr.step.__func__

    def _configure_physics(cls) -> None:
        base_cfg(cls)
        _enable_contact_processing()

    def step(cls) -> None:
        base_step(cls)
        _poll(cls)

    step._focus_hooked = True
    mgr._configure_physics = classmethod(_configure_physics)
    mgr.step = classmethod(step)


class _MgrFinder(importlib.abc.MetaPathFinder):
    """Patches Isaac Lab's PhysxManager when it is first imported (it needs Kit, so not at task import)."""

    def find_spec(self, fullname, path, target=None):
        if fullname != _PHYSX_MGR:
            return None
        sys.meta_path.remove(self)
        try:
            spec = importlib.util.find_spec(fullname)
        finally:
            sys.meta_path.insert(0, self)
        if spec is None or spec.loader is None:
            return None
        orig_exec = spec.loader.exec_module

        def exec_module(module):
            orig_exec(module)
            _patch_manager(module)

        spec.loader.exec_module = exec_module
        return spec


if _PHYSX_MGR in sys.modules:
    _patch_manager(sys.modules[_PHYSX_MGR])
elif not any(isinstance(f, _MgrFinder) for f in sys.meta_path):
    sys.meta_path.insert(0, _MgrFinder())


# -- the rules: the screws' turning, the clamp, the bar's magnet ---------------------------------------------------------
_IDS: dict = {}


def _joint_ids(asset) -> dict:
    if not _IDS:
        names = list(asset.joint_names)
        _IDS.update({n: names.index(n) for n in ("slide", "bar") + SCREWS})
        bnames = list(asset.body_names)
        _IDS.update({f"body:{n}": bnames.index(n) for n in ("module", "bar") + SCREWS})
    return _IDS


def _write(asset, what, joint, value) -> None:
    ids = [_IDS[joint]]
    if what == "stiffness":
        asset.write_joint_stiffness_to_sim_index(stiffness=float(value), joint_ids=ids)
    elif what == "damping":
        asset.write_joint_damping_to_sim_index(damping=float(value), joint_ids=ids)
    elif what == "effort":
        asset.write_joint_effort_limit_to_sim_index(limits=float(value), joint_ids=ids)


def _targets(asset, values: dict) -> None:
    import torch

    names = list(values)
    ids = [_IDS[n] for n in names]
    v = torch.tensor([[float(values[n]) for n in names]], dtype=torch.float32, device=asset.device)
    asset.actuators.target_command.set_position_index(value=v, joint_ids=ids)


def _apply_rules(asset, q, W) -> None:
    """Every control step: each screw turns by a pinching gripper's own turn (up to snug), else its thread holds it; the
    clamp (locked while either screw is within 30 deg of snug); the magnet."""
    T = _T
    pads = W["pads"]                                   # sensors x fingers, mean force over the step (N)
    for k, name in enumerate(SCREWS, start=1):
        S = T["screws"][name]
        by = None
        if pads is not None:
            for side, (a, b) in PADS.items():
                if pads[k, a] >= PINCH and pads[k, b] >= PINCH:
                    by = side
        turn = W["turn"].get(by, 0.0) if by else 0.0
        if by is not None:
            S["target"] = min(0.0, max(SCREW_MIN, S["target"] + turn))
            S["turned"] += turn
            cap = T_TRACK
        else:
            err = S["target"] - q[name]                # thread friction: the target sticks until a push saturates it
            if abs(K_SCREW * err) > T_REST:
                S["target"] = q[name] + math.copysign(T_REST / K_SCREW, err)
            cap = T_REST
        if cap != S["cap"]:
            _write(asset, "effort", name, cap)
            S["cap"] = cap
        if by != S["by"]:
            print(f"[focus] step {T['step']}: {name} {'pinched by the ' + by + ' hand' if by else 'let go'} at "
                  f"{math.degrees(q[name]):+.1f} deg", flush=True)
            S["by"] = by
    locked = any(math.degrees(q[n]) >= -LOCK_DEG for n in SCREWS)
    if locked and not T["clamped"]:
        T["q_lock"] = q["slide"]
        _write(asset, "stiffness", "slide", K_LOCK)
        _write(asset, "damping", "slide", D_LOCK)
        print(f"[focus] step {T['step']}: clamped at slide {1000 * q['slide']:+.2f} mm (screws "
              f"{math.degrees(q['screw_u']):+.0f}, {math.degrees(q['screw_l']):+.0f} deg)", flush=True)
    elif not locked and T["clamped"]:
        _write(asset, "stiffness", "slide", 0.0)
        _write(asset, "damping", "slide", D_FREE)
        print(f"[focus] step {T['step']}: freed at slide {1000 * q['slide']:+.2f} mm (screws "
              f"{math.degrees(q['screw_u']):+.0f}, {math.degrees(q['screw_l']):+.0f} deg)", flush=True)
    T["clamped"] = locked
    mag = abs(math.degrees(q["bar"])) <= MAG_DEG
    if mag != T["magnet"]:
        _write(asset, "stiffness", "bar", K_MAG if mag else 0.0)
        T["magnet"] = mag
    # the clamp carries the module's weight (like the real screws' friction): its target sits mg/K above the position
    # where it took hold, so the bar is not left holding the module up
    hold = F.MODULE_MASS * 9.81 / K_LOCK
    vals = {"slide": T["q_lock"] + hold, "bar": 0.0}
    vals.update({n: T["screws"][n]["target"] for n in SCREWS})
    _targets(asset, vals)


# -- the trial-long tracker ----------------------------------------------------------------------------------------------
_T: dict = {}


def _fresh_screws() -> dict:
    return {n: {"target": 0.0, "cap": T_REST, "by": None, "turned": 0.0} for n in SCREWS}


@on_reset
def _reset(_env=None) -> None:
    """Only a reset clears the tracker (and with it any latched failure)."""
    _T.clear()
    _T.update(step=0, failed=None, z_hi=None, drop_max=0.0, clamped=True, q_lock=0.0, magnet=True, world=None, prev=None,
              hold=0, state=None, sheet0=None, sheet_max=0.0, e_min=9.0, v_max=0.0, screws=_fresh_screws(), grip_R={})
    _MON.update(pad_sum=None, polls=0, touch_steps=0, peak=0.0)
    if _env is not None:
        try:
            asset = _env.unwrapped.scene["laser"]
            _joint_ids(asset)
            _write(asset, "stiffness", "slide", K_LOCK)
            _write(asset, "damping", "slide", D_LOCK)
            _write(asset, "stiffness", "bar", K_MAG)
            for n in SCREWS:
                _write(asset, "effort", n, T_REST)
            _targets(asset, {"slide": 0.0, "bar": 0.0, "screw_u": 0.0, "screw_l": 0.0})
        except Exception as e:  # pragma: no cover
            print(f"[focus] reset rules: {e!r}", flush=True)


_reset()


@before_reset
def _before(environment) -> None:
    try:
        asset = environment.unwrapped.scene["laser"]
        _joint_ids(asset)
        _write(asset, "stiffness", "slide", K_LOCK)
        _write(asset, "damping", "slide", D_LOCK)
        _targets(asset, {"slide": 0.0, "bar": 0.0, "screw_u": 0.0, "screw_l": 0.0})
    except Exception as e:  # pragma: no cover
        print(f"[focus] before reset: {e!r}", flush=True)


def _fail(why) -> None:
    if _T.get("failed") is None and _T:
        _T["failed"] = why
        print(f"[focus] latched 0 at step {_T['step']}: {why}", flush=True)


@on_step
def _read(environment) -> None:
    base = environment.unwrapped
    origin = _np(base.scene.env_origins[0])
    asset = base.scene["laser"]
    ids = _joint_ids(asset)
    if _MON["asset"] is None:
        _MON["asset"], _MON["slide"] = asset, ids["slide"]
    qa = _np(asset.data.joint_pos)[0]
    q = {n: float(qa[ids[n]]) for n in ("slide", "bar") + SCREWS}
    poses = _np(asset.data.body_link_pose_w)[0]
    links = {n: (poses[ids[f"body:{n}"]][:3] - origin, _rot(poses[ids[f"body:{n}"]][3:7])) for n in ("module", "bar") + SCREWS}
    sh = base.scene["sheet"]
    sp = _np(sh.data.root_link_pose_w)[0]
    sheet = (sp[:3] - origin, _rot(sp[3:7]))
    W = {"q": q, "links": links, "sheet": sheet, "fingers": {}, "tcp": {}, "turn": {}}
    for side in ("left", "right"):
        robot = base.scene[f"{side}_robot"]
        rp = _np(robot.data.body_link_pose_w)[0]
        pts = []
        for link in GRIP_LINKS:
            i = robot.body_names.index(link)
            R = _rot(rp[i][3:7])
            pts.append(GRIP_POINTS[link] @ R.T + (rp[i][:3] - origin))
            if link == "gripper":
                W["tcp"][side] = rp[i][:3] - origin + R @ np.array([0.0, 0.0, 0.134])
                R_prev = _T["grip_R"].get(side)
                W["turn"][side] = 0.0 if R_prev is None else _turn_about_y(R_prev, R)
                _T["grip_R"][side] = R
        W["fingers"][side] = np.vstack(pts)
    W["touch_steps"], W["polls"], W["peak"] = _MON["touch_steps"], _MON["polls"], _MON["peak"]
    W["pads"] = None if _MON["pad_sum"] is None else _MON["pad_sum"] / max(_MON["polls"], 1)
    _MON.update(touch_steps=0, polls=0, peak=0.0, pad_sum=None)
    _T["step"] += 1
    _apply_rules(asset, q, W)
    _track(W)


def focus_error(W) -> float:
    """e: the hanging bar's lowest point (computed with the bar on its stop) above the sheet's top surface there."""
    mpos, _ = W["links"]["module"]
    z_tip = mpos[2] - G
    p, R = W["sheet"]
    top = p + R @ np.array([0, 0, F.SHEET[2]])
    n = R[:, 2]
    z_sheet = top[2] - (n[0] * (TIP_XY[0] - top[0]) + n[1] * (TIP_XY[1] - top[1])) / n[2]
    return float(z_tip - z_sheet)


def _sheet_corners(W) -> np.ndarray:
    p, R = W["sheet"]
    return SHEET_CORNERS @ R.T + p


def finger_gap(W) -> float:
    """Least distance from any gripper's finger links to the module (body, shield), bar, either screw or the sheet."""
    best = 9.0
    pts = np.vstack(list(W["fingers"].values()))
    for link, boxes in BOXES.items():
        lp, lR = W["links"][link]
        for c, r, s in boxes:
            d = box_signed_distance(pts, lp + lR @ c, lR @ r, s)
            best = min(best, float(d.min()))
    p, R = W["sheet"]
    d = box_signed_distance(pts, p + R @ np.array([0, 0, F.SHEET[2] / 2]), R, np.array(F.SHEET))
    return min(best, float(d.min()))


def _track(W) -> None:
    T = _T
    q = W["q"]
    corners = _sheet_corners(W)
    if T["sheet0"] is None:
        T["sheet0"] = corners
    moved = float(np.linalg.norm(corners - T["sheet0"], axis=1).max())
    T["sheet_max"] = max(T["sheet_max"], moved)
    if moved > SHEET_MOVED:
        _fail(f"sheet moved: a corner is {1000 * moved:.1f} mm from where it lay")
    e = focus_error(W)
    T["e_min"] = min(T["e_min"], e)
    gap = finger_gap(W)
    if T["prev"] is not None:
        pq, pc = T["prev"]
        still = (abs(q["slide"] - pq["slide"]) < STILL_STEP and abs(q["bar"] - pq["bar"]) * F.BAR_L < STILL_STEP
                 and all(abs(q[n] - pq[n]) * F.HEAD_R < STILL_STEP for n in SCREWS)
                 and float(np.linalg.norm(corners - pc, axis=1).max()) < STILL_STEP)
        dz = pq["slide"] - q["slide"]                                # descent over this 50 ms step
        T["v_max"] = max(T["v_max"], dz * HZ)
        if dz * HZ > DROP_SPEED:
            _fail(f"dropped: the module descended {1000 * dz:.2f} mm in one 50 ms step ({100 * dz * HZ:.1f} cm/s)")
    else:
        still = False
    T["prev"] = (dict(q), corners)
    locked = all(math.degrees(q[n]) >= -LOCK_DEG for n in SCREWS)
    state = dict(e=e, in_focus=abs(e) <= IN_FOCUS, near=abs(e) <= NEAR_FOCUS, locked=locked,
                 stowed=abs(math.degrees(q["bar"])) <= STOW_TOL, released=gap >= CLEAR, still=still, gap=gap)
    state["full"] = state["in_focus"] and state["locked"] and state["stowed"] and state["released"] and still
    T["hold"] = T["hold"] + 1 if (state["full"] and T["failed"] is None) else 0
    T["state"] = state
    T["world"] = W
    if T["step"] % 10 == 0 or T["step"] < 3 or any(T["screws"][n]["by"] for n in SCREWS):
        _log(W, state)


def _tier() -> float:
    T = _T
    s = T["state"]
    if T["failed"] is not None or s is None:
        return 0.0
    if T["hold"] >= HOLD:
        return 1.0
    rs = s["released"] and s["still"]
    if s["in_focus"] and s["locked"] and rs:
        return 0.75
    if s["near"] and not s["in_focus"] and s["locked"] and rs:
        return 0.5
    if s["in_focus"] and not s["locked"] and rs:
        return 0.25
    return 0.0


def score(objects) -> float:
    """task.json, "scoring": 0 once a failure has latched; otherwise the end state's tier (1.0 after 1 s of it)."""
    return _tier()


def success(objects) -> bool:
    """The 1.0 state now; the harness holds it 1 s (SUCCESS_HOLD_S), the same 20 steps score() counts."""
    steptime.tick()
    T = _T
    return bool(T and T["failed"] is None and T["state"] is not None and T["state"]["full"])


def _log(W, s) -> None:
    T = _T
    q = W["q"]
    tl, tr = W["tcp"]["left"], W["tcp"]["right"]
    pads = W["pads"]
    pad_txt = ""
    if pads is not None:
        pad_txt = " pads U " + "/".join(f"{pads[1, i]:.0f}" for i in (1, 2, 4, 5)) + " L " + "/".join(f"{pads[2, i]:.0f}" for i in (1, 2, 4, 5))
    print(f"[focus] step {T['step']} slide {1000 * q['slide']:+.2f}mm e {1000 * s['e']:+.2f}mm screws "
          f"{math.degrees(q['screw_u']):+.1f}/{math.degrees(q['screw_l']):+.1f} bar {math.degrees(q['bar']):+.1f} "
          f"clamp {int(T['clamped'])} locked {int(s['locked'])} turnR {math.degrees(W['turn'].get('right', 0.0)):+.1f} "
          f"touch {W['touch_steps']}/{W['polls']} peak {W['peak']:.1f}N drop {1000 * T['drop_max']:.2f}mm "
          f"vmax {1000 * T['v_max']:.1f}mm/s sheet {1000 * T['sheet_max']:.2f}mm gap {1000 * s['gap']:.0f}mm "
          f"still {int(s['still'])} hold {T['hold']} tcpL {np.round(1000 * (tl - [0, 0, TOP]), 1).tolist()} "
          f"tcpR {np.round(1000 * (tr - [0, 0, TOP]), 1).tolist()} tier {_tier()} fail {T['failed']}{pad_txt}", flush=True)


# -- scripted demo ----------------------------------------------------------------------------------------------------------
import yam_arena.demo as _demo  # noqa: E402


class Rot(float):
    """A waypoint 'yaw' that carries a full TCP rotation (columns: pinch axis, gy, approach)."""

    def __new__(cls, R):
        g = float.__new__(cls, 0.0)
        g.R = np.asarray(R, float)
        return g


_straight_down = getattr(_demo.grip_rotation, "straight_down", _demo.grip_rotation)


def _grip_rotation(yaw: float = 0.0) -> np.ndarray:
    R = getattr(yaw, "R", None)
    return R if R is not None else _straight_down(float(yaw))


_grip_rotation.straight_down = _straight_down
_demo.grip_rotation = _grip_rotation


def tilted(tilt_deg: float, toward_y: float) -> Rot:
    """Fingers closing along x; the approach tilted tilt_deg from straight down, toward +y (toward_y=1) or -y."""
    t = math.radians(tilt_deg)
    gz = np.array([0.0, toward_y * math.sin(t), -math.cos(t)])
    gx = np.array([1.0, 0.0, 0.0])
    return Rot(np.c_[gx, np.cross(gz, gx), gz])


def side_pinch(roll_deg: float, down_deg: float = 0.0) -> Rot:
    """The right hand pointing +y (tipped down_deg), fingers closing along x, turned roll_deg about its approach (+ =
    about +y, which tightens a screw)."""
    a, d = math.radians(roll_deg), math.radians(down_deg)
    gz = np.array([0.0, math.cos(d), -math.sin(d)])
    gx = np.array([1.0, 0.0, 0.0])
    R0 = np.c_[gx, np.cross(gz, gx), gz]
    Rz = np.array([[math.cos(a), -math.sin(a), 0.0], [math.sin(a), math.cos(a), 0.0], [0.0, 0.0, 1.0]])
    return Rot(R0 @ Rz)


PAD = 0.013                                         # fingertips past the TCP along the approach
def stow_grip(kx: float, tilt_deg: float) -> Rot:
    """The right hand from -y, pointing down toward +y (tilt_deg from straight down) and leaning kx forward (+x), its
    fingers closing along x (as near as the approach allows): the wrist stays in front of the gantry beam."""
    t = math.radians(tilt_deg)
    a = np.array([kx, math.sin(t), -math.cos(t)])
    a /= np.linalg.norm(a)
    gx = np.array([1.0, 0.0, 0.0]) - a[0] * a
    gx /= np.linalg.norm(gx)
    return Rot(np.c_[gx, np.cross(a, gx), a])


STOW_GRIP = stow_grip(float(os.environ.get("STOW_KX", 0.35)), float(os.environ.get("STOW_TILT", 45.0)))
LIFT_PRE = float(os.environ.get("LIFT_PRE", 0.010))   # the holding hand pushes up this much before the screws free it
OVER = float(os.environ.get("OVER", 0.002))           # lowered this far past the bar's contact
LOWER_S = float(os.environ.get("LOWER_S", 6.0))       # seconds for the lowering (a slow, steady descent)
GRIP_DZ = float(os.environ.get("GRIP_DZ", 0.009))     # the left pads' lower ends this far below the module's top
TIP_Y = float(os.environ.get("TIP_Y", 0.0235))        # the right pads' ends on a screw (station y)
OPEN_MM = float(os.environ.get("OPEN_MM", 26.0))      # right fingers open round a screw head
R_SWEEP = float(os.environ.get("R_SWEEP", 0.018))     # the stow pusher's circle round the bar's axis
PUSH_IN = float(os.environ.get("PUSH_IN", 0.003))     # the pusher is aimed this far into the leg, so it pushes it
STOW_TCP_Y = float(os.environ.get("STOW_TCP_Y", 0.010))   # the tilted hand's TCP y during the sweep: the fingertips
                                                          # reach 23.5 mm, 1.5 mm short of the shield's side
PUSH_HX, PUSH_HZ = 0.0072, 0.011                      # the closed fingertips' half-size in the bar's plane (x, z)
PUSH_DZ = 0.00085                                     # ... and their centre there, above the TCP
PRESS = float(os.environ.get("PRESS", 0.020))         # how far the closed fingers press the stowed bar's leg down


def _wp(t, p, finger, label, yaw) -> Waypoint:
    return Waypoint(t, (float(p[0]), float(p[1]), float(p[2])), finger, label, yaw)


class Plan:
    def __init__(self, side, yaw):
        self.side, self.w, self.t, self.yaw = side, [], 0.0, yaw
        self.p, self.finger = None, FINGER_OPEN

    def go(self, t, p=None, finger=None, label="", yaw=None):
        self.p = self.p if p is None else np.asarray(p, float)
        self.finger = self.finger if finger is None else finger
        self.yaw = self.yaw if yaw is None else yaw
        self.w.append(_wp(t, self.p, self.finger, label, self.yaw))
        self.t += t
        return self

    def tip(self, t, tip, finger=None, label="", yaw=None):
        """Go with the fingertips' centre at `tip` (TCP = tip - PAD along the approach)."""
        rot = (yaw if yaw is not None else self.yaw)
        R = getattr(rot, "R", None)
        a = R[:, 2] if R is not None else np.array([0.0, 0.0, -1.0])
        return self.go(t, np.asarray(tip, float) - PAD * a, finger, label, yaw)

    def until(self, t, label="wait"):
        if t > self.t + 1e-9:
            self.go(t - self.t, label=label)
        return self


def sep(mm: float) -> float:
    """Finger target for pads `mm` apart."""
    return -mm / 2000.0


def _w(p) -> np.ndarray:
    """Station point from (x, y, height above the table)."""
    return np.array([p[0], p[1], TOP + p[2]], float)


def demo():
    """Open loop. Right: press the stowed bar down off its magnet; loosen both hand screws a quarter turn each by pinching
    and turning the wrist. Left: grip the module's top, carry its weight, lower it onto the bar's contact. Right: tighten
    both screws back to snug; after the left hand lets go, sweep the bar forward and up onto its magnet."""
    mode = os.environ.get("TASK_DEMO", "success")
    if mode == "nothing":
        return {}
    from yam_arena import ik
    from yam_arena.constants import READY
    from yam_arena.station import BASES

    L = Plan("left", 0.0)
    R = Plan("right", side_pinch(0.0))
    mx, my = MOD_XY
    piv0 = np.array([mx + F.BAR_PIVOT[0], my + F.BAR_PIVOT[1], START_Z + F.BAR_PIVOT[2]])   # the bar's axis at the start
    piv1 = piv0 - [0, 0, E0]                                                                   # ... once focused
    sx = mx + F.SCREW_X
    zs = {n: START_Z + F.SCREW_Z[n] for n in SCREWS}
    pR, rR = ik.tcp_pose(np.array(READY, float), np.array(BASES["right"]))
    out_y = -0.025

    # -- right: press the stowed bar's free end down off its magnet (level, closed fingers above the leg)
    bx = piv0[0] - 0.030                                   # 3 cm forward of the pivot along the stowed leg
    top_leg = piv0[2] + F.BAR_W / 2
    zp = top_leg + 0.0137 + 0.002                          # finger blocks reach 13.7 mm below the TCP
    R.tip(2.5, _w([bx, out_y, zp + 0.010]), sep(0), "beside the stowed bar", yaw=side_pinch(0.0, 8.0))
    R.tip(1.0, _w([bx, TIP_Y, zp]), label="over the bar's leg")
    R.tip(1.0, _w([bx, TIP_Y, zp - PRESS]), label="press the bar down")
    R.go(0.4, label="the bar falls to hang")
    R.tip(0.8, _w([bx, out_y - 0.005, zp - PRESS]), label="out")
    R.tip(0.8, _w([bx, out_y - 0.015, zs["screw_u"]]), label="out, at the upper screw's height", yaw=side_pinch(0.0))

    # -- left: grip the module's top, straight down, fingers across y
    top = START_Z + F.BODY_H
    grip = np.array([mx, my, top - GRIP_DZ + PAD])          # TCP: the pads' lower ends GRIP_DZ below the top
    hover = top + 0.004 + PAD
    L.go(2.5, _w([mx, my + 0.07, hover + 0.005]), sep(70), "beside the module, open", yaw=0.0)
    L.go(1.2, _w([mx, my, hover]), label="just over the top")
    L.go(1.0, _w(grip), label="fingers round the top")
    L.go(0.8, finger=sep(26), label="grip")
    if mode == "unheld":                                   # no hand on the module: open and leave
        L.go(0.6, finger=sep(70), label="let go")
        L.go(1.0, _w([mx - 0.02, my + 0.08, hover + 0.02]), label="away")
    else:
        L.go(0.8, _w(grip + [0, 0, LIFT_PRE]), label="take the module's weight")
    T_GRIPPED = L.t

    # -- right: loosen both screws a quarter turn each (pre-turned +45 deg, turned to -45 deg)
    def turn_screw(z, a0, a1, label, n=None):
        n = n or max(2, int(round(abs(a1 - a0) / 15.0)))
        R.tip(1.5, _w([sx, out_y, z]), sep(OPEN_MM), f"beside the {label} screw, open", yaw=side_pinch(a0))
        R.tip(1.0, _w([sx, TIP_Y, z]), label=f"round the {label} screw")
        R.go(0.3, label="settle")
        R.go(0.6, finger=sep(0), label="pinch")
        for k in range(1, n + 1):
            R.go(1.5 / n, yaw=side_pinch(a0 + (a1 - a0) * k / n), label=f"turn the {label} screw")
        R.go(0.3, label="hold")
        R.go(0.5, finger=sep(OPEN_MM), label="let go of the screw")
        R.tip(0.8, _w([sx, out_y, z]), label="out")

    R.until(T_GRIPPED, "wait for the left grip")
    turn_screw(zs["screw_u"], 45.0, -45.0, "upper")
    turn_screw(zs["screw_l"], 45.0, -45.0, "lower")
    T_LOOSE = R.t
    if mode == "unheld":                                   # the module is left to itself once both screws are loose
        R.go(3.0, pR, label="back to ready", yaw=Rot(rR))
        L.until(R.t, "away")
        return {"left": L.w, "right": R.w}

    # -- left: lower onto the bar's contact
    L.until(T_LOOSE, "hold the module")
    if mode == "slip":                                     # a careless hand: the grip eases and the module slides
        L.go(0.4, finger=sep(52), label="grip loosens")
    L.go(LOWER_S, _w(grip - [0, 0, E0 + OVER]), label="lower onto the bar")
    L.go(0.6, label="settle on the bar")
    T_DOWN = L.t
    if mode == "slip":                                     # the drop has latched: end the trial here
        L.go(2.0, label="still")
        R.until(L.t, "still")
        return {"left": L.w, "right": R.w}

    # -- right: tighten the lower screw, then the upper one (pre-turned -50 deg, turned to +50 deg: stops at snug)
    R.tip(1.0, _w([sx, out_y, zs["screw_l"] - E0]), sep(OPEN_MM), "beside the lowered screw", yaw=side_pinch(-50.0))
    R.until(T_DOWN, "wait for the module")
    turn_screw(zs["screw_l"] - E0, -50.0, 50.0, "lower")
    turn_screw(zs["screw_u"] - E0, -50.0, 50.0, "upper")
    T_LOCKED = R.t

    # -- left: let go and clear
    L.until(T_LOCKED, "hold the module")
    L.go(0.6, finger=sep(70), label="let go")
    L.go(1.0, _w([mx, my, hover + 0.03 - E0]), label="rise")
    L.go(1.2, _w([mx - 0.02, my + 0.08, hover + 0.05]), label="clear")

    # -- right: sweep the hanging bar forward and up onto its magnet (tilted 45 deg toward +y, fingers closed)
    def pusher(a):
        """TCP for the pusher's centre on the circle at bar angle a (deg from hanging), behind the leg."""
        t = math.radians(a)
        n = np.array([math.cos(t), 0.0, -math.sin(t)])
        off = F.BAR_W / 2 + PUSH_HX * abs(n[0]) + PUSH_HZ * abs(n[2]) - PUSH_IN
        c = piv1 + R_SWEEP * F.bar_dir(a) + off * n
        return np.array([c[0], STOW_TCP_Y, c[2] - PUSH_DZ])

    p0 = pusher(-6.0)
    R.tip(0.8, _w([sx, out_y - 0.035, zs["screw_u"] - E0]), label="out from the screws", yaw=side_pinch(0.0))
    R.go(1.2, _w([0.26, -0.08, 0.10]), sep(0), "forward and out")
    R.go(2.0, label="turn into the stow grip", yaw=STOW_GRIP)
    R.go(1.2, _w([p0[0] - 0.02, out_y - 0.01, p0[2] + 0.05]), label="beside the hanging bar, tilted")
    R.until(T_LOCKED + 1.0, "wait for the left hand to clear")
    R.go(1.0, _w([p0[0], p0[1] - 0.012, p0[2] + 0.012]), label="toward the leg")
    R.go(0.8, _w(p0), label="behind the bar's leg")
    for a in list(np.arange(6.0, 90.0, 8.0)) + [96.0, 102.0]:
        R.go(0.3, _w(pusher(float(a))), label="sweep the bar up")
    R.go(0.8, label="onto the magnet")
    pe = pusher(102.0)
    R.go(0.6, _w(pe + [0, -0.02, 0.006]), label="out of the bar's plane")
    R.go(1.0, _w(pe + [0.01, -0.05, 0.05]), label="clear")
    R.go(1.5, _w([pe[0] - 0.03, -0.10, 0.18]), label="up and away")
    R.go(2.5, pR, label="back to ready", yaw=Rot(rR))
    end_t = max(L.t, R.t) + 2.5
    L.until(end_t, "still")
    R.until(end_t, "still")
    return {"left": L.w, "right": R.w}
