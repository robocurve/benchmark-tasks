"""On the station table, in front of the arms: a training thigh (a firm foam cylinder, 12.7 cm across) lying across the
table, hip end on the robot's left. A blue windlass tourniquet (C-A-T style) is strapped round it, snug, its black
windlass rod on top. The tourniquet's own clip sits on the band beside the windlass, toward the arms, with a hook on
each edge of the band. Beyond the thigh, an aneroid gauge on a bladder under the band reads 60 mmHg; its dial is green
from 250 to 300.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \\
         --scorer task_score ...
Demo:    python demo.py task.py demo_out
"""

from __future__ import annotations

import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

from scenekit import Shape, UsdBody, before_reset, on_reset, on_step  # noqa: E402

import geometry as g  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
SUCCESS_HOLD_S = 1.0

INSTRUCTION = "Tighten the tourniquet on the training leg until its gauge reads green, then lock the rod in its clip."

TOP = g.TOP
GREY = (0.20, 0.22, 0.25)

# -- objects --------------------------------------------------------------------------------------------------------
TOURNIQUET = UsdBody("tourniquet", usd="assets/tourniquet.usda", pos=(g.LEG_X, g.BAND_Y, TOP),
                     size=(2 * g.LEG_R, g.LEG_L, 2 * g.LEG_R), articulation=True,
                     joint_pos=(("twist", 0.0), ("press", 0.0), ("needle", -g.needle_angle(g.P_SNUG))),
                     drives=(("twist", 0.0, g.TWIST_DAMPING), ("press", g.TILT_K, g.TILT_D), ("needle", 0.05, 0.002)),
                     friction=(0.8, 0.7))

# the clip: a hook on each edge of the band, toward the arms. Each hook is a short channel along where a seated rod lies
# (tipped SEAT_TILT down, PHI to the side of the band's line): a floor, a wall either side up to the lips' top with a
# lead-in above it, all colliding, and the lips themselves, shown but not colliding: the rod passes them as it snaps in,
# and task code then holds it down as they would. A plate joins the hooks across the band (shown only).
def _quat(R) -> tuple[float, float, float, float]:
    """XYZW quaternion of a rotation matrix."""
    w = math.sqrt(max(0.0, 1.0 + R[0, 0] + R[1, 1] + R[2, 2])) / 2
    x = math.copysign(math.sqrt(max(0.0, 1.0 + R[0, 0] - R[1, 1] - R[2, 2])) / 2, R[2, 1] - R[1, 2])
    y = math.copysign(math.sqrt(max(0.0, 1.0 - R[0, 0] + R[1, 1] - R[2, 2])) / 2, R[0, 2] - R[2, 0])
    z = math.copysign(math.sqrt(max(0.0, 1.0 - R[0, 0] - R[1, 1] + R[2, 2])) / 2, R[1, 0] - R[0, 1])
    return (x, y, z, w)


def _rz(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def _ry(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def _rx(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


AXIS = np.array([g.LEG_X, g.BAND_Y, TOP + g.ROD_Z])   # the rod's middle: it turns and tips about here
_IN = g.ROD_R + g.CLIP_GAP                 # the walls' inner faces, from the rod's line (6.5 mm)
_LIP_TOP = g.LIP_Z[1]                      # the walls stand up to the lips' top (8 mm above the floor)
_LEAD = g.WALL_H - _LIP_TOP                # then the lead-in rises to the walls' top (9 mm)
HOOK = []
for name, psi in zip(("hip", "knee"), g.hook_angles()):
    frame = _rz(psi) @ _ry(g.SEAT_TILT)    # x along the seated rod toward its end, z up from it, y across
    floor_z = -g.ROD_R                      # the floor's top is the seated rod's underside

    def put(label, centre, size, R_local=None, collide=True):
        R = frame if R_local is None else frame @ R_local
        HOOK.append(Shape(f"clip_{name}_{label}", pos=tuple(AXIS + frame @ np.asarray(centre, float)), size=size,
                          rot_xyzw=_quat(R), color=GREY, collide=collide))

    put("floor", (g.HOOK_R, 0.0, floor_z - g.FLOOR_T / 2), (g.HOOK_L, 2 * (_IN + g.PRONG_T), g.FLOOR_T))
    for s in (-1, 1):
        side = "ab"[s > 0]
        put(f"wall_{side}", (g.HOOK_R, s * (_IN + g.PRONG_T / 2), floor_z + _LIP_TOP / 2), (g.HOOK_L, g.PRONG_T, _LIP_TOP))
        # the lead-in: a slab whose top face runs from the channel's edge at the lips' top out to the wall's outer top
        run = math.hypot(g.PRONG_T, _LEAD)
        ang = math.atan2(_LEAD, g.PRONG_T)
        mid = np.array([s * (_IN + g.PRONG_T / 2), _LIP_TOP + _LEAD / 2])
        normal = np.array([-s * math.sin(ang), math.cos(ang)])   # up and in, toward the channel
        c = mid - normal * 0.0005
        put(f"lead_{side}", (g.HOOK_R, c[0], floor_z + c[1]), (g.HOOK_L, run, 0.001), _rx(s * ang))
        put(f"lip_{side}", (g.HOOK_R, s * (_IN - g.LIP_IN / 2), floor_z + sum(g.LIP_Z) / 2),
            (g.HOOK_L, g.LIP_IN, g.LIP_Z[1] - g.LIP_Z[0]), collide=False)
# the clip's plate across the band, between and under the hooks (shown only): on the band's surface where it curves down
_PX = g.HOOK_XY[0] - 0.004
_SLOPE = math.asin(_PX / g.BAND_R)
HOOK.append(Shape("clip_plate", pos=(g.LEG_X - (g.BAND_R + 0.0015) * math.sin(_SLOPE), g.BAND_Y,
                                     TOP + g.LEG_Z + (g.BAND_R + 0.0015) * math.cos(_SLOPE)),
                  size=(0.012, 2 * (g.HOOK_XY[1] + _IN + g.PRONG_T), 0.003),
                  rot_xyzw=_quat(_ry(-_SLOPE)), color=GREY, collide=False))
OBJECTS = [TOURNIQUET] + HOOK

# -- monitor: the band's pull on the windlass, the gauge, the clip, and the score's state ----------------------------
_STATE: dict = {}
HOLD_STEPS = 20           # control steps (1 s) locked, green and with the arms clear before it counts as let go
ROD_SIZE = np.array([g.ROD_L, 2 * g.ROD_R, 2 * g.ROD_R])


def _edge_points(size, step: float = 0.005) -> np.ndarray:
    """Points every 5 mm or less along a box's 12 edges, in its frame (origin at its centre)."""
    half, pts = np.asarray(size) / 2, []
    for a in range(3):
        b, c = [i for i in range(3) if i != a]
        t = np.linspace(-half[a], half[a], max(2, int(np.ceil(size[a] / step)) + 1))
        for sb in (-1.0, 1.0):
            for sc in (-1.0, 1.0):
                p = np.zeros((len(t), 3))
                p[:, a], p[:, b], p[:, c] = t, sb * half[b], sc * half[c]
                pts.append(p)
    return np.concatenate(pts)


ROD_EDGES = _edge_points(ROD_SIZE)


def _rotation(xyzw) -> np.ndarray:
    x, y, z, w = xyzw
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


FIXED_BOXES = [(_rotation(s.rot_xyzw), np.asarray(s.pos, float) - [0, 0, TOP], np.asarray(s.size, float))
               for s in HOOK]
FIXED_BOXES = [(R, c, size, _edge_points(size)) for R, c, size in FIXED_BOXES]


def _asset(environment):
    return environment.unwrapped.scene["tourniquet"]


def _torque(environment, tau: float) -> None:
    """The band's pull on the windlass: a torque about the vertical on the twisted strap, applied every physics step."""
    import torch

    asset = _asset(environment)
    ids = asset.find_bodies(["windlass"])[0]
    value = torch.tensor([[[0.0, 0.0, tau]]], dtype=torch.float32, device=asset.device)
    asset.permanent_wrench_composer.set_forces_and_torques_index(torques=value, body_ids=ids, is_global=True)


def _set_needle(environment, p: float) -> None:
    from scenekit import set_joint_targets

    set_joint_targets(environment, "tourniquet", {"needle": -g.needle_angle(p)})


def _set_press_target(environment, tilt: float) -> None:
    from scenekit import set_joint_targets

    set_joint_targets(environment, "tourniquet", {"press": tilt})


def near_tilt(theta: float, tilt: float) -> float:
    """How far the rod's end toward the arms (the clip's side) is tipped down, in radians (the press joint tips the
    rod's +x end down for a positive tilt)."""
    return tilt if math.cos(theta) < 0 else -tilt


def aligned(theta: float) -> bool:
    """Whether the rod lies along one of the clip's hooks, seen from above (either end toward the arms)."""
    return any(abs(math.remainder(theta - psi, math.pi)) <= g.LOCK_ALIGN for psi in g.hook_angles())


@on_reset
def _reset_state(environment=None) -> None:
    _STATE.update(twist=0.0, last_q=0.0, tilt=0.0, dip=0.0, locked=False, pressure=g.P_SNUG, aligned=False, theta=g.THETA0,
                  clearance=1.0, clear=True, clear_steps=0, held=False, rod_gap=1.0,
                  body_gap=1.0, pressing=False, green=False)
    if environment is not None:
        _restart(environment)


@before_reset
def _restart(environment) -> None:
    """Untwist the windlass, lift the rod out of the clip and level it, and reset the gauge before a reset (the reset
    moves the tourniquet only by its root, not its joints)."""
    from scenekit import write_joint_positions

    _torque(environment, 0.0)
    _set_press_target(environment, 0.0)
    _set_needle(environment, g.P_SNUG)
    write_joint_positions(environment, "tourniquet", {"twist": 0.0, "press": 0.0,
                                                      "needle": -g.needle_angle(g.P_SNUG)})


_reset_state()


def _distance(hulls, boxes) -> float:
    """The smallest gap between convex hulls and boxes (rotation, centre, size, edge points): from every hull vertex
    to each box, and from points along each box's edges to each nearby hull (by the hull's face planes, which never
    overestimate it). Negative when they overlap."""
    from yam_arena.analysis import box_signed_distance

    if not boxes or not hulls:
        return 1.0
    verts = np.concatenate([h.vertices for h in hulls])
    h_lo = np.array([h.vertices.min(axis=0) for h in hulls])
    h_hi = np.array([h.vertices.max(axis=0) for h in hulls])
    best = 1.0
    for R, centre, size, local_edges in boxes:
        best = min(best, float(box_signed_distance(verts, centre, R, size).min()))
        edges = centre + local_edges @ R.T
        lo, hi = edges.min(axis=0) - 0.02, edges.max(axis=0) + 0.02
        for i in np.flatnonzero(np.all(h_hi >= lo, axis=1) & np.all(h_lo <= hi, axis=1)):
            p = hulls[i].planes
            best = min(best, float((edges @ p[:, :3].T - p[:, 3]).max(axis=1).min()))
    return best


def rod_box(theta: float, tilt: float):
    """The rod as a box: (rotation, centre in the station frame above the table, size)."""
    c, s = math.cos(theta), math.sin(theta)
    ct, st = math.cos(tilt), math.sin(tilt)
    Rz = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
    Ry = np.array([[ct, 0.0, st], [0.0, 1.0, 0.0], [-st, 0.0, ct]])
    return Rz @ Ry, np.array([g.LEG_X, g.BAND_Y, g.ROD_Z]), ROD_SIZE


def _robot_hulls(environment) -> list:
    """The URDF's collision hulls of every link, the gripper body and both fingers of both arms at their simulated
    poses, with heights above the table top."""
    from yam_arena.analysis import world_hulls

    from scenekit import robot_link_poses

    hulls = []
    for poses in robot_link_poses(environment).values():
        for _, h in world_hulls(poses):
            h.vertices[:, 2] -= TOP
            h.planes[:, 3] -= h.planes[:, 2] * TOP
            hulls.append(h)
    return hulls


def _clearance(hulls, theta: float, tilt: float) -> tuple[float, float]:
    """How far the arms are from the rod, and from the rod or the clip."""
    R, centre, size = rod_box(theta, tilt)
    rod = _distance(hulls, [(R, centre, size, ROD_EDGES)])
    return rod, min(rod, _distance(hulls, FIXED_BOXES))


_DIAL_N = np.array([-math.cos(g.GAUGE_TILT), 0.0, math.sin(g.GAUGE_TILT)])   # out of the dial's face
_DIAL_C = np.array([g.GAUGE_XY[0], g.GAUGE_XY[1], g.GAUGE_Z]) - _DIAL_N * 0.014   # the middle of the gauge's case


def _body_gap(hulls) -> float:
    """How far the arms are from the rest of the tourniquet, the thigh and the gauge: the thigh with the band and
    bladder round it (a cylinder of the band's radius along y) and the gauge (a disc of its case's size). Pressing any of
    them could push the gauge's reading up, so the gauge only counts with the robot off them."""
    v = np.concatenate([h.vertices for h in hulls])
    along = np.abs(v[:, 1] - g.BAND_Y) <= g.LEG_L / 2
    radial = np.hypot(v[:, 0] - g.LEG_X, v[:, 2] - g.LEG_Z) - g.BAND_R
    thigh = float(radial[along].min()) if along.any() else 1.0
    d = v - _DIAL_C
    a = d @ _DIAL_N
    r = np.linalg.norm(d - np.outer(a, _DIAL_N), axis=1)
    gauge = float(np.hypot(np.maximum(r - (g.DIAL_R + 0.007), 0.0), np.maximum(np.abs(a) - 0.0165, 0.0)).min())
    return min(thigh, gauge)


@on_step
def _monitor(environment) -> None:
    from yam_arena.scene import object_states

    S = _STATE
    joints = object_states(environment, ["tourniquet"])["tourniquet"].joints
    q = float(joints["twist"])
    S["twist"] += math.degrees(math.remainder(q - S["last_q"], 2 * math.pi))   # the joint comes back wrapped
    S["last_q"] = q
    S["tilt"] = tilt = float(joints["press"])
    S["theta"] = theta = g.THETA0 + math.radians(S["twist"])
    S["aligned"] = aligned(theta)
    S["dip"] = dip = near_tilt(theta, tilt)
    if not S["locked"] and S["aligned"] and dip >= g.LOCK_TILT:
        S["locked"] = True                                  # seated in a hook: it snaps in and stays down
        _set_press_target(environment, g.TILT_MAX if math.cos(theta) < 0 else -g.TILT_MAX)
    elif S["locked"] and dip < g.UNLOCK_TILT:
        S["locked"] = False                                 # lifted back out past the lips
        _set_press_target(environment, 0.0)
    S["pressure"] = p = g.pressure(S["twist"])
    _set_needle(environment, p)
    _torque(environment, -g.K_TWIST * math.radians(S["twist"]))
    hulls = _robot_hulls(environment)
    S["rod_gap"], S["clearance"] = _clearance(hulls, theta, tilt)
    S["held"] = S["rod_gap"] < g.CLEAR                       # part of the robot within 1 cm of the rod
    S["clear"] = S["clearance"] >= g.CLEAR
    S["body_gap"] = _body_gap(hulls)
    S["pressing"] = S["body_gap"] < g.TOUCH                 # on the band, bladder, thigh or gauge
    S["green"] = green = p >= g.GREEN and not S["pressing"]  # the gauge counts only with the robot off them
    S["clear_steps"] = S["clear_steps"] + 1 if S["locked"] and green and S["clear"] else 0


def success(objects) -> bool:
    """The gauge in the green (250 mmHg or more, with the robot off the band, bladder, thigh and gauge) with the rod
    locked in the clip and no part of either arm within 1 cm of the rod or the clip (yam_arena ends the trial once
    this has held for SUCCESS_HOLD_S)."""
    return bool(_STATE["locked"] and _STATE["green"] and _STATE["clear"])


def score(objects) -> float:
    """At the end of the trial, the first that applies (success scores 1.0). The gauge counts as green only at 250 mmHg
    or more with no part of the robot touching the band, bladder, thigh or gauge.
    - 1.0: for the whole last 1 s, green, the rod locked, and the arms at least 1 cm from the rod and the clip;
    - 0.5: green with the rod locked, or held (part of the robot within 1 cm of the rod) level, not pressed down onto
      the band; but not 1.0;
    - 0.25: the rod locked with the gauge not green, wherever the robot is;
    - 0: anything else (not tightened, or let go unlocked so it unwound)."""
    S = _STATE
    if S["locked"] and S["green"]:
        return 1.0 if S["clear_steps"] >= HOLD_STEPS else 0.5
    if S["green"] and S["held"] and abs(S["tilt"]) <= g.HELD_LEVEL:
        return 0.5
    if S["locked"]:
        return 0.25
    return 0.0
