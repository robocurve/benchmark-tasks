"""On the station table: a phone on a small tabletop tripod at the far side of the table films toward the robot; a
film clapperboard (a 30 x 25 cm acrylic slate with a hinged clapstick, like the Neewer acrylic clapboard) leans back on
a small hard equipment case in front of it, its face toward the phone. Task (instruction): slate the take: hold the
clapperboard facing the phone camera, fully in its frame, and clap its stick shut once, sharply. The instruction does
not say how.

How it is done (ProVideo Coalition, "The secret art of the slate: 25 tips to help you slate like a pro"): "Proper
slating technique dictates holding the slate with one hand and closing the clapper with the other"; "THE SLATE MUST
NOT MOVE WHILE THE CLAPPER IS IN MOTION"; "If both the slate and the clapper show motion blur it becomes very hard to
figure out when the clapper stops moving"; "Do not let the clapper bounce."

The hand skill: one hand holds the slate still and square to the camera while the other opens the stick and drives it
shut fast. The 5 mm acrylic board does not stand by itself (it leans on the case, its face 28 deg off the camera's
axis); a hand that lets go of it to work the stick lets it fall back; one hand cannot hold the board and work the
stick at once. The hinge has a tension screw (0.12 N m of friction), so the stick stays where it is left: letting go of
it does not clap it; the hand must drive it shut. A gripper holding the stick by its end swings it and drags the light
slate with it; holding the slate still through a fast strike is the difficulty.

World frame (m): x forward from the arm bases, y to the robot's left, z up; the table top is at z 0.7475.
- clapperboard: loose; black acrylic writing board 300 x 200 x 5 mm, with a fixed lower clapstick (300 x 25 x 8 mm)
  along its top edge and the hinged upper clapstick (300 x 25 x 8 mm) on it, its hinge pin 10 mm in from the board's
  left end (as the robot sees it, +y), so its free end is on the right (-y); 300 x 250 mm overall; 0.45 kg board and
  lower stick, 0.07 kg upper stick. The upper stick turns about the axis through its hinge normal to the board face,
  from shut (0 deg, resting on the lower stick) up to 45 deg open, against 0.12 N m of Coulomb friction. It starts
  with its bottom edge on the table, its middle at (0.270, 0.020), the face toward +x (the phone), leaning back 28 deg
  on the case.
- hard case: loose, 0.20 (x) x 0.16 (y) x 0.15 m, 2.0 kg, its far top edge where the slate leans on it.
- phone on a tabletop tripod: fixed (decor); its camera lens at (0.700, 0.020, 0.130 above the table), looking along -x,
  16:9 landscape video, 69 x 43 deg field of view (a 26 mm-equivalent main camera).

Score (read at 240 Hz, every physics step):
- A clap: the stick shuts (within 1 deg of closed) after being open 20 deg or more. A later shut from 10 deg or more is
  another clap. The first clap is the take's sync mark and is judged:
- 0: no clap in the run; or, at the first clap, the slate's face more than 20 deg off the camera's axis (the line from
  the slate's middle to the lens) or a corner of the slate outside the camera's field of view.
- 0.5: the first clap faces the camera and is in frame, but it was slow (it took longer than 50 ms from 10 deg open
  to shut), or the slate moved 1 cm or more in the 0.5 s around it, or there was more than one clap in the run.
- 1.0 (success, held 1 s): one clap only, facing and in frame, from 10 deg to shut within 50 ms, the slate's middle
  within 1 cm of where it was over the 0.25 s before and after.
The clap is read from the hinge's angle; the stick's stop is the joint's limit (the two sticks, one articulation, do not
collide with each other).
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
from bodykit import as_numpy, on_reset, on_step, rotation, segment  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

allow_matrix_grips()

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"
_E = os.environ.get

# -- the layout ----------------------------------------------------------------------------------------------------
BOARD_W, WRITE_H, BOARD_T = 0.300, 0.200, 0.005  # the acrylic writing board
STICK_H, STICK_T = 0.025, 0.008  # each clapstick
TOTAL_H = WRITE_H + 2 * STICK_H  # 0.25
HINGE_IN = 0.010  # the hinge pin, in from the board's left end
XB, YB = float(_E("SLATE_XB", "0.270")), float(_E("SLATE_YB", "0.020"))  # the middle of the slate's bottom edge (on the table)
LEAN = math.radians(float(_E("SLATE_LEAN", "28")))  # leaning back on the case (top toward the robot)
CASE_H, CASE_D, CASE_W = 0.150, 0.200, 0.160
HINGE_NM = float(_E("SLATE_HINGE", "0.12"))  # the hinge's tension: friction torque (N m)
OPEN_MAX = 45.0  # deg

# the camera: a phone on a tabletop tripod at the far side, looking back along -x
CAM = np.array([0.700, YB, TOP + 0.130])
HFOV, VFOV = math.radians(69.4), math.radians(42.6)

BLACK, WHITE, STICK_W, STICK_B, CASE_C, PHONE_C, TRIPOD_C, TAPE = (
    (0.06, 0.06, 0.07), (0.92, 0.92, 0.9), (0.95, 0.95, 0.95), (0.05, 0.05, 0.05), (0.25, 0.27, 0.3),
    (0.1, 0.1, 0.12), (0.15, 0.15, 0.15), (0.9, 0.85, 0.3))


def _v(p) -> tuple:
    return tuple(float(x) for x in p)


def _ry(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def _rx(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


def _quat_y(a: float) -> tuple:
    return (0.0, math.sin(a / 2), 0.0, math.cos(a / 2))


# The slate's frame: origin at the middle of its bottom edge (mid-thickness), x out of its face, y along it, z up it.
# Leaning back 35 deg = turned -35 deg about y. It rests on the back corner of its bottom edge.
R_LEAN = _ry(-LEAN)
ORIGIN0 = np.array([XB, YB, TOP]) - R_LEAN @ np.array([-BOARD_T / 2, 0.0, 0.0]) + np.array([0.0, 0.0, 0.0008])
# where its back face meets the case's far top edge
_S_CASE = (CASE_H + 0.0008 - (ORIGIN0[2] - TOP) - (R_LEAN @ np.array([-BOARD_T / 2, 0.0, 0.0]))[2]) / R_LEAN[2, 2]
_EDGE = ORIGIN0 + R_LEAN @ np.array([-BOARD_T / 2, 0.0, _S_CASE])
CASE_X1 = float(_EDGE[0])  # the case's far face
HINGE_L = np.array([0.0, BOARD_W / 2 - HINGE_IN, WRITE_H + STICK_H])  # the hinge, in the slate's frame


def _stripes(z0: float, h: float, x: float, phase: int) -> list:
    """Black diagonal stripes on a white clapstick's face (visual only)."""
    out = []
    n = 8
    for k in range(n):
        if (k + phase) % 2:
            continue
        yc = -BOARD_W / 2 + (k + 0.5) * BOARD_W / n
        out.append(BoxShape((x, yc, z0 + h / 2), (0.0006, BOARD_W / n * 0.75, h * 0.98), STICK_B, collide=False,
                            quat_xyzw=(math.sin(math.radians(-25) / 2), 0.0, 0.0, math.cos(math.radians(-25) / 2))))
    return out


def _slate() -> Linkage:
    front = BOARD_T / 2 + 0.0004
    sf = STICK_T / 2 + 0.0004
    base = [BoxShape((0.0, 0.0, WRITE_H / 2), (BOARD_T, BOARD_W, WRITE_H), BLACK),
            BoxShape((0.0, 0.0, WRITE_H + STICK_H / 2), (STICK_T, BOARD_W, STICK_H), STICK_W)]
    # the printed grid on the face (white lines: PROD / ROLL SCENE TAKE / DIRECTOR CAMERA DATE), visual only
    for z in (0.150, 0.100, 0.050):
        base.append(BoxShape((front, 0.0, z), (0.0006, BOARD_W - 0.012, 0.0018), WHITE, collide=False))
    for y in (-0.050, 0.050):
        base.append(BoxShape((front, y, 0.125), (0.0006, 0.0018, 0.050), WHITE, collide=False))
    base.append(BoxShape((front, 0.0, 0.075), (0.0006, 0.0018, 0.050), WHITE, collide=False))
    base += _stripes(WRITE_H, STICK_H, sf, 0) + _stripes(WRITE_H, STICK_H, -sf, 0)  # the sticks: striped both sides
    # the hinge plate on the back
    base.append(BoxShape((-STICK_T / 2 - 0.0015, HINGE_L[1], HINGE_L[2]), (0.003, 0.022, 0.030), (0.5, 0.5, 0.52)))
    upper = [BoxShape((0.0, 0.0, WRITE_H + 1.5 * STICK_H), (STICK_T, BOARD_W, STICK_H), STICK_W)]
    upper += _stripes(WRITE_H + STICK_H, STICK_H, sf, 1) + _stripes(WRITE_H + STICK_H, STICK_H, -sf, 1)
    upper.append(CylShape((STICK_T / 2 + 0.002, HINGE_L[1], HINGE_L[2] + 0.004), 0.005, 0.004, (0.6, 0.6, 0.62),
                          axis="X"))
    link = Link("stick", tuple(upper), mass=0.07, anchor=_v(HINGE_L), axis="X", friction=HINGE_NM, damping=0.0002,
                limits=tuple(float(v) for v in _E("SLATE_LIMITS", f"{-OPEN_MAX},0").split(",")))
    return Linkage.build("slate", pos=(0.0, 0.0, 0.0), mass=0.45, color=BLACK, floating=True, friction=(0.5, 0.4),
                         shapes=base, links=[link], contact_offset=0.0015, solver_iterations=48)


def _place(spec: Linkage) -> Linkage:
    import dataclasses

    # a Link's anchor is in world coordinates for the unturned linkage: the hinge, moved with the origin
    links = tuple(dataclasses.replace(k, anchor=_v(np.asarray(k.anchor) + ORIGIN0)) for k in spec.links)
    return dataclasses.replace(spec, pos=_v(ORIGIN0), init_quat=_quat_y(-LEAN), links=links)


def _case() -> Body:
    sh = [BoxShape((0.0, 0.0, 0.0), (CASE_D, CASE_W, CASE_H), CASE_C),
          BoxShape((0.0, 0.0, CASE_H / 2 + 0.006), (0.030, 0.080, 0.012), (0.1, 0.1, 0.1), collide=False),
          BoxShape((0.0, CASE_W / 2 + 0.0005, -0.01), (CASE_D - 0.02, 0.001, 0.012), (0.6, 0.6, 0.6),
                   collide=False),
          BoxShape((0.0, -CASE_W / 2 - 0.0005, -0.01), (CASE_D - 0.02, 0.001, 0.012), (0.6, 0.6, 0.6),
                   collide=False)]
    return Body.build("case", pos=(CASE_X1 - CASE_D / 2, YB, TOP + CASE_H / 2 + 0.0008), mass=2.0, color=CASE_C,
                      friction=(0.6, 0.5), shapes=sh)


def _phone() -> Body:
    """The phone (16:9, landscape) on a small tabletop tripod; its lens faces -x. Decor, fixed."""
    c = CAM
    rel = lambda p: _v(np.asarray(p) - c)  # noqa: E731
    sh = [BoxShape(rel(c + np.array([0.006, -0.055, 0.0])), (0.008, 0.160, 0.075), PHONE_C),
          CylShape(rel(c + np.array([0.0015, 0.0, 0.0])), 0.006, 0.003, (0.02, 0.02, 0.03), axis="X"),
          BoxShape(rel(c + np.array([0.014, -0.055, 0.0])), (0.010, 0.050, 0.085), TRIPOD_C),  # clamp
          segment(rel(c + np.array([0.02, -0.055, -0.045])), rel(c + np.array([0.02, -0.055, -0.070])), 0.012,
                  TRIPOD_C)]
    foot = np.array([0.02, -0.055, -0.070])
    for a in (0.0, 2.1, 4.2):
        tip = np.array([0.02 + 0.07 * math.cos(a), -0.055 + 0.07 * math.sin(a), -(c[2] - TOP) + 0.004])
        sh.append(segment(rel(c + foot), rel(c + tip), 0.008, TRIPOD_C))
    return Body.build("phone", pos=_v(c), mass=0.3, color=PHONE_C, fixed=True, shapes=sh)


def _tape() -> Body:
    """A strip of tape on the table along the camera's axis (for measuring the slate's angle in a real trial)."""
    return Body.build("tape", pos=(0.48, YB, TOP + 0.0003), mass=0.01, color=TAPE, fixed=True,
                      shapes=[BoxShape((0.0, 0.0, 0.0), (0.200, 0.012, 0.0006), TAPE, collide=False)])


SLATE = _place(_slate())
CASE = _case()
PHONE = _phone()
TAPE_B = _tape()
OBJECTS = [SLATE, CASE, PHONE, TAPE_B]

# -- tracking and scoring -------------------------------------------------------------------------------------------
OPEN_DEG = 20.0  # a clap needs the stick open this far first
REOPEN_DEG = 10.0  # a later shut from this far open is another clap
SHUT_DEG = 1.0  # shut: within this of closed
FAST_MS = float(_E("SLATE_FAST_MS", "50"))  # from 10 deg open to shut within this
STILL_M = 0.010  # the slate's middle moves less than this ...
WINDOW_S = 0.25  # ... over this before and after the clap
FACE_DEG = 20.0
_DEBUG = _E("SLATE_DEBUG") == "1"
_DEBUG_EVERY = int(_E("SLATE_DEBUG_EVERY", "5"))
_EP: dict = {}
_IDS: dict = {}
MID_L = np.array([0.0, 0.0, TOTAL_H / 2])  # the slate's middle, in its frame
CORNERS_L = np.array([[BOARD_T / 2, y, z] for y in (-BOARD_W / 2, BOARD_W / 2) for z in (0.0, TOTAL_H)])


@on_reset
def _reset() -> None:
    _EP.update(t=0.0, tick=0, step=0, angle=0.0, peak=0.0, armed=False, reopen=False, t10=None, claps=[],
               hist=[], pending=None, verdict=None, face=90.0, inframe=False, mid=np.zeros(3), wrapped=False,
               vmax=0.0)


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


def _box_dist(pts, centre, half) -> float:
    """Smallest distance from points (box frame) to a box (0 inside)."""
    d = np.maximum(np.abs(pts - centre) - half, 0.0)
    return float(np.linalg.norm(d, axis=1).min())


def _view(mid, rot) -> tuple:
    """The slate's face angle off the camera's axis (deg) and whether all four corners are in the camera's view."""
    normal = rot @ np.array([1.0, 0.0, 0.0])
    to_cam = CAM - mid
    face = math.degrees(math.acos(max(-1.0, min(1.0, float(normal @ to_cam / np.linalg.norm(to_cam))))))
    inside = True
    for c in CORNERS_L:
        p = mid + rot @ (c - MID_L)
        d = p - CAM  # the camera looks along -x
        ahead = -d[0]
        if ahead <= 0.0 or abs(math.atan2(d[1], ahead)) > HFOV / 2 or abs(math.atan2(d[2], ahead)) > VFOV / 2:
            inside = False
    return face, inside


def _sample(base) -> None:
    """Every physics step: the stick's angle, the slate's pose; detect claps."""
    origin = as_numpy(base.scene.env_origins[0])
    slate = base.scene["slate"]
    if "slate" not in _IDS:
        _IDS["slate"] = list(slate.joint_names).index("stick_turn")
        _IDS["base"] = list(slate.body_names).index("base")
        _IDS["dt"] = float(base.physics_dt)
    dt = _IDS["dt"]
    _EP["t"] += dt
    t = _EP["t"]
    q = as_numpy(slate.data.joint_pos)[0]
    vq = as_numpy(slate.data.joint_vel)[0]
    ang = -math.degrees(float(q[_IDS["slate"]]))  # opening angle, deg (the stick turns -x to open)
    vel = math.degrees(float(vq[_IDS["slate"]]))  # deg/s, positive while closing
    bp = as_numpy(slate.data.body_link_pose_w)[0][_IDS["base"]]
    rot = rotation(bp[3:7])
    mid = bp[:3] - origin + rot @ MID_L
    _EP["angle"], _EP["mid"], _EP["rot"] = ang, mid, rot
    _EP["peak"] = max(_EP["peak"], ang)
    hist = _EP["hist"]
    hist.append((t, mid.copy()))
    while hist and hist[0][0] < t - 2 * WINDOW_S - 0.05:
        hist.pop(0)
    if ang >= OPEN_DEG:
        _EP["armed"] = True
    if ang >= REOPEN_DEG:
        _EP["reopen"] = True
        _EP["t10"] = t
    if (_EP["armed"] or _EP["reopen"]) and ang > SHUT_DEG:
        _EP["vmax"] = max(_EP["vmax"], vel)
    if ang <= SHUT_DEG and (_EP["armed"] or _EP["reopen"]):
        full = _EP["armed"]
        dur = (t - _EP["t10"]) * 1000 if _EP["t10"] is not None else float("inf")
        face, inside = _view(mid, rot)
        clap = dict(t=t, full=full, ms=dur, face=face, inside=inside, vmax=_EP["vmax"], moved=None)
        if full or _EP["claps"]:
            _EP["claps"].append(clap)
            if len(_EP["claps"]) == 1:
                _EP["pending"] = clap
        _EP.update(armed=False, reopen=False, t10=None, vmax=0.0)
    pend = _EP["pending"]
    if pend is not None and t >= pend["t"] + WINDOW_S:
        pts = np.array([m for (tt, m) in hist if pend["t"] - WINDOW_S <= tt <= pend["t"] + WINDOW_S])
        pend["moved"] = float(np.linalg.norm(pts - pts[0], axis=1).max()) if len(pts) else 0.0
        if _DEBUG:
            for (tt, m) in hist[::6]:
                if pend["t"] - WINDOW_S <= tt <= pend["t"] + WINDOW_S:
                    print(f"DBGWIN t={tt:.3f} mid=({m[0]:.4f},{m[1]:.4f},{m[2] - TOP:.4f})", flush=True)
        _EP["pending"] = None


@on_step
def _track(env) -> None:
    try:
        _track_inner(env)
    except Exception:
        import traceback

        print("SLATE_TRACEBACK\n" + traceback.format_exc(), flush=True)
        raise


def _track_inner(env) -> None:
    from yam_arena import ik, station
    from yam_arena.constants import ARM_JOINTS, FINGER_JOINTS

    base = getattr(env, "unwrapped", env)
    if not _EP["wrapped"] or getattr(base.scene, "_slate_hook", None) is not _sample:
        orig = base.scene.update

        def update(*a, **k):
            out = orig(*a, **k)
            _sample(base)
            return out

        base.scene.update = update
        base.scene._slate_hook = _sample
        _EP["wrapped"] = True
    _EP["tick"] += 1
    if _DEBUG:
        tcp = {}
        for side in ("left", "right"):
            robot = base.scene[f"{side}_robot"]
            if side not in _IDS:
                _IDS[side] = list(robot.find_joints(ARM_JOINTS, preserve_order=True)[0])
                _IDS[side + "_f"] = list(robot.find_joints(FINGER_JOINTS, preserve_order=True)[0])
            q = as_numpy(robot.data.joint_pos)[0]
            tcp[side] = (np.asarray(ik.tcp_pose(q[_IDS[side]], station.BASES[side])[0], float),
                         float(-q[_IDS[side + "_f"]].sum()))
        _EP["tcp"] = tcp
        if "fingers" not in _IDS:
            _IDS["fingers"] = _finger_points()
        robot = base.scene["right_robot"]
        if "r_tips" not in _IDS:
            _IDS["r_tips"] = [robot.body_names.index(n) for n in ("tip_left", "tip_right")]
        origin = as_numpy(base.scene.env_origins[0])
        poses = as_numpy(robot.data.body_link_pose_w)[0]
        fp = np.concatenate([_IDS["fingers"][n] @ rotation(poses[i][3:7]).T + poses[i][:3] - origin
                             for n, i in zip(("tip_left", "tip_right"), _IDS["r_tips"])])
        slate = base.scene["slate"]
        bp = as_numpy(slate.data.body_link_pose_w)[0]
        names = list(slate.body_names)
        b, st = bp[names.index("base")], bp[names.index("stick")]
        lb = (fp - (b[:3] - origin)) @ rotation(b[3:7])
        ls = (fp - (st[:3] - origin)) @ rotation(st[3:7])
        _EP["d_low"] = _box_dist(lb, np.array([0.0, 0.0, WRITE_H + STICK_H / 2]), np.array([STICK_T, BOARD_W, STICK_H]) / 2)
        _EP["d_up"] = _box_dist(ls, np.array([0.0, 0.0, WRITE_H + 1.5 * STICK_H]), np.array([STICK_T, BOARD_W, STICK_H]) / 2)


def _judge() -> tuple[float, str]:
    claps = _EP["claps"]
    if not claps:
        return 0.0, "no clap"
    c = claps[0]
    if c["face"] > FACE_DEG or not c["inside"]:
        return 0.0, f"clap off camera (face {c['face']:.0f} deg, in frame {c['inside']})"
    if c["moved"] is None:
        return 0.0, "clap being judged"
    slow = c["ms"] > FAST_MS
    moved = c["moved"] >= STILL_M
    more = len(claps) > 1
    if slow or moved or more:
        return 0.5, f"clap flawed (slow {slow}, moved {moved}, more {more})"
    return 1.0, "clean clap"


def _debug(s: float, why: str) -> None:
    _EP["step"] += 1
    if (_EP["step"] - 1) % _DEBUG_EVERY and not (_EP["claps"] and _EP.get("printed", 0) < len(_EP["claps"])):
        return
    _EP["printed"] = len(_EP["claps"])
    mid = _EP["mid"]
    face, inside = _view(mid, _EP.get("rot", np.eye(3))) if "rot" in _EP else (90.0, False)
    cl = "; ".join(f"t={c['t']:.3f} full={int(c['full'])} ms={c['ms']:.1f} vmax={c['vmax']:.0f}dps "
                   f"face={c['face']:.1f} in={int(c['inside'])} moved={(c['moved'] or 0) * 1000:.1f}mm"
                   for c in _EP["claps"])
    tcp = " ".join(f"{k} tcp=({v[0][0]:.3f},{v[0][1]:.3f},{v[0][2] - TOP:.3f}) gap={v[1] * 1000:.0f}"
                   for k, v in _EP.get("tcp", {}).items())
    print(f"DBG step={_EP['step']} t={_EP['t']:.2f} ang={_EP['angle']:.1f} peak={_EP['peak']:.1f} "
          f"mid=({mid[0]:.3f},{mid[1]:.3f},{mid[2] - TOP:.3f}) face={face:.1f} in={int(inside)} "
          f"d_low={_EP.get('d_low', 1) * 1000:.1f} d_up={_EP.get('d_up', 1) * 1000:.1f} score={s} ({why}) claps=[{cl}] | {tcp}", flush=True)


def score(objects) -> float:
    s, why = _judge()
    if _DEBUG:
        _debug(s, why)
    return s


def success(objects) -> bool:
    return _judge()[0] >= 1.0


# -- scripted demo --------------------------------------------------------------------------------------------------
# Open loop, from the known layout. The left hand, pointing -y with its fingers closing across the board, comes down
# along the leaning slate beside its left edge, pinches the board 3.5 cm in from that edge and 10 cm up, and turns it
# upright about its bottom edge (aimed 1.5 cm down along the board, so the bottom edge stays pressed on the table,
# and 3 deg past upright), then holds it there to the end. The right hand comes down over the clapstick's free end,
# pointing along the stick, and pinches only its last 2 cm with the fingertips (clear of the lower stick); it opens the
# stick to 25 deg along its arc and lets go (the hinge holds it open), rises 1 cm, closes its fingers, sets the
# fingertips on the stick's top edge 4 cm in from its end and strikes it shut: two waypoints of 0.05 s along the arc,
# aimed 8 deg past shut and 6 cm in toward the hinge (a fast move of the arm swings out across the stick's line
# first). It rests on the shut stick 1 s and lifts off.
# SLATE_ONE=1 (wrong way): the left hand stays at the ready pose; the right hand opens and strikes the stick on the
# slate where it leans.
OPEN_C = -0.047
ONE = _E("SLATE_ONE") == "1"
OPEN_TO = float(_E("SLATE_OPEN_TO", "25"))  # deg the stick is told to open
SNAP_S = float(_E("SLATE_SNAP_S", "0.10"))  # s the snap shut is given
SNAP_N = int(_E("SLATE_SNAP_N", "2"))  # waypoints along the arc for the snap
SNAP_IN = float(_E("SLATE_SNAP_IN", "0.06"))  # during the snap the hand is aimed this far along the stick toward the
# hinge (a fast move lags most across the stick's line and would drag the stick, and the slate, outward)
SNAP_FIXED = _E("SLATE_SNAP_FIXED", "0") == "1"  # (pinch-snap only) keep the hand's angle during the snap
STRIKE = _E("SLATE_STRIKE", "1") == "1"  # strike the open stick shut with the closed fingertips on its top edge (0:
# snap it shut still pinched, which drags the slate 1 to 2.5 cm with the arm's swing)
# the fingertips reach 13.7 mm below the TCP (pointing along the stick): the TCP this far above the stick's middle line
STRIKE_DZ = float(_E("SLATE_STRIKE_DZ", "0.030"))
STRIKE_IN = float(_E("SLATE_STRIKE_IN", "0.035"))  # strike this far in along the stick from where it was pinched
STICK_PINCH = float(_E("SLATE_STICK_PINCH", "-0.003"))  # finger command on the 8 mm stick
PAST = float(_E("SLATE_PAST", "8"))  # deg aimed past shut
HOLD_S = float(_E("SLATE_HOLD_S", "0.10"))  # board: pinch height along the board
BOARD_IN = float(_E("SLATE_BOARD_IN", "0.035"))  # board: TCP this far in from the left edge
# The fingertips reach 13 mm past the TCP and are 27 mm wide for their last 23 mm; behind that the fingers are 40 mm
# wide. The TCP goes 6 mm in from the stick's end and 6 mm above its middle, so only the narrow tips close on the
# stick, clear of the lower stick (the wide part stays beyond the stick's end).
TIP_IN = float(_E("SLATE_TIP_IN", "0.006"))  # stick: TCP this far in from its free end
TIP_DZ = float(_E("SLATE_TIP_DZ", "0.006"))  # stick: TCP this far above the stick's middle
STICK_DY = float(_E("SLATE_STICK_DY", "0.0"))  # the slate ends up this far right of where it started (y)
UP_LEAN = float(_E("SLATE_UP_LEAN", "-3"))  # deg: the board hand aims this far past upright
PRESS_IN = float(_E("SLATE_PRESS_IN", "0.015"))  # board: while turning it up, aimed this far down along the board
# (pressing its bottom edge on the table, so the edge does not skid forward)
TURN_S = float(_E("SLATE_TURN_S", "4.0"))


def _R(c1, c3) -> np.ndarray:
    c1, c3 = np.asarray(c1, float), np.asarray(c3, float)
    c1, c3 = c1 / np.linalg.norm(c1), c3 / np.linalg.norm(c3)
    return np.column_stack([c1, np.cross(c3, c1), c3])


def approach_rot(a, roll_deg: float) -> np.ndarray:
    """Pointing along ``a``, the fingers closing along the reference axis (z, or x for a vertical ``a``) turned
    ``roll_deg`` right-handed about ``a``."""
    a = np.asarray(a, float) / np.linalg.norm(a)
    ref = np.array([0.0, 0.0, 1.0]) if abs(a[2]) < 0.9 else np.array([1.0, 0.0, 0.0])
    u = ref - a * np.dot(ref, a)
    u /= np.linalg.norm(u)
    v = np.cross(a, u)
    t = math.radians(roll_deg)
    return _R(math.cos(t) * u + math.sin(t) * v, a)


LEVEL_L = approach_rot((0, -1, 0), 90)  # left: pointing -y, fingers closing along x
STICK_PITCH = math.radians(float(_E("SLATE_STICK_PITCH", "5")))  # deg the right hand points below the stick's line
LEVEL_R = approach_rot((0, math.cos(STICK_PITCH), -math.sin(STICK_PITCH)), 90)  # right: pointing +y (and 5 deg
# down along the shut stick), fingers closing along x


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


def _lead_in(arm: str, pos, rot, finger: float, label: str, seconds: float = 3.0, steps: int = 15) -> list:
    from yam_arena import ik, station
    from yam_arena.constants import READY

    q0, q1 = np.array(READY, float), _good_ik(arm, pos, rot)
    out = []
    for k in range(1, steps + 1):
        p, r = ik.tcp_pose(q0 + (q1 - q0) * k / steps, station.BASES[arm])
        out.append(Waypoint(seconds / steps, tuple(float(v) for v in p), finger, label, r))
    return out


def _w(t, p, finger, label, rot) -> Waypoint:
    return Waypoint(t, tuple(float(v) for v in p), finger, label, rot)


def _stay(arm: str, t: float = 120.0) -> list:
    from yam_arena import ik, station
    from yam_arena.constants import READY

    pos, rot = ik.tcp_pose(np.array(READY, float), station.BASES[arm])
    return [Waypoint(t, tuple(float(v) for v in pos), OPEN_C, "stay put", rot)]


def _slate_pose(lean_deg: float) -> tuple:
    """The slate's origin and rotation when turned up about the back corner of its bottom edge to ``lean_deg``."""
    r = _ry(-math.radians(lean_deg))
    pivot = np.array([XB, YB, TOP])
    return pivot - r @ np.array([-BOARD_T / 2, 0.0, 0.0]), r


def _board_grip(lean_deg: float, out: float = 0.0) -> tuple:
    o, r = _slate_pose(lean_deg)
    p = o + r @ np.array([0.0, BOARD_W / 2 - BOARD_IN, HOLD_S + out])
    return p, r @ LEVEL_L


def _stick_grip(open_deg: float, out: float = 0.0, lean_deg: float = 0.0, inward: float = 0.0) -> tuple:
    """The right hand on the stick's free end with the stick open ``open_deg`` (it turns -x about the hinge), aimed
    ``inward`` along the stick toward the hinge."""
    o, r = _slate_pose(lean_deg)
    rs = _rx(-math.radians(open_deg))
    local = HINGE_L + rs @ (np.array([0.0, -BOARD_W / 2 + TIP_IN + inward, WRITE_H + 1.5 * STICK_H + TIP_DZ + out])
                            - HINGE_L)
    return o + r @ local + np.array([0.0, STICK_DY, 0.0]), r @ rs @ LEVEL_R


T_LEFT_READY = 12.0  # s: by then the board is upright and held


def _left_path() -> list:
    lean0 = math.degrees(LEAN)
    p, r = _board_grip(lean0, 0.17)
    out = _lead_in("left", p, r, OPEN_C, "above the slate's left edge, open")
    p1, _ = _board_grip(lean0, 0.0)
    out += [_w(2.0, p1, OPEN_C, "down along the board, fingers either side of it", r), _w(0.4, p1, OPEN_C, "settle", r),
            _w(1.0, p1, 0.0, "pinch the board", r), _w(0.6, p1, 0.0, "settle", r)]
    n = 12
    for k in range(1, n + 1):
        lean = lean0 + (UP_LEAN - lean0) * k / n
        pk, rk = _board_grip(lean, -PRESS_IN * min(1.0, 3.0 * k / n))
        out.append(_w(TURN_S / n, pk, 0.0, "turn the slate upright on its bottom edge", rk))
    pu, ru = _board_grip(UP_LEAN, -PRESS_IN)
    out.append(_w(1.0, pu, 0.0, "hold it upright", ru))
    out.append(_w(40.0, pu, 0.0, "hold the slate still, facing the camera", ru))
    return out


def _right_path() -> list:
    lean = math.degrees(LEAN) if ONE else UP_LEAN
    wait = 0.5 if ONE else T_LEFT_READY
    p, r = _stick_grip(0.0, 0.05, lean)
    out = _stay("right", wait)
    out += _lead_in("right", p, r, OPEN_C, "above the stick's free end, open")
    p1, r1 = _stick_grip(0.0, 0.0, lean)
    out += [_w(2.0, p1, OPEN_C, "down over the stick's end, fingertips either side of it", r1), _w(0.4, p1, OPEN_C, "settle", r1),
            _w(1.0, p1, STICK_PINCH, "pinch the stick's end", r1), _w(0.5, p1, STICK_PINCH, "settle", r1)]
    n = 10
    for k in range(1, n + 1):
        pk, rk = _stick_grip(OPEN_TO * k / n, 0.0, lean)
        out.append(_w(2.0 / n, pk, STICK_PINCH, "open the stick", rk))
    po, ro = _stick_grip(OPEN_TO, 0.0, lean)
    out.append(_w(0.8, po, STICK_PINCH, "hold it open", ro))
    if STRIKE:
        # let go of the open stick (its hinge holds it), close the fingers above its end and strike it shut
        out.append(_w(0.6, po, OPEN_C, "let go of the open stick", ro))
        pu, _ = _stick_grip(OPEN_TO, STRIKE_DZ + 0.010, lean, STRIKE_IN)
        out.append(_w(1.0, pu, OPEN_C, "rise clear of the stick", ro))
        out.append(_w(0.8, pu, 0.0, "close the fingers", ro))
        pt, _ = _stick_grip(OPEN_TO, STRIKE_DZ, lean, STRIKE_IN)
        out.append(_w(1.0, pt, 0.0, "fingertips on the stick's top edge", ro))
        for k in range(1, SNAP_N + 1):
            pk, _ = _stick_grip(OPEN_TO + (-PAST - OPEN_TO) * k / SNAP_N, STRIKE_DZ, lean, STRIKE_IN + SNAP_IN)
            out.append(_w(SNAP_S / SNAP_N, pk, 0.0, "strike it shut", ro))
        pe, _ = _stick_grip(-PAST, STRIKE_DZ, lean, STRIKE_IN)
        out.append(_w(1.0, pe, 0.0, "rest on it", ro))
        pb, _ = _stick_grip(0.0, 0.06, lean)
        out.append(_w(1.5, pb, 0.0, "lift off the stick", ro))
        out.append(_w(30.0, pb + np.array([-0.03, 0.0, 0.02]), 0.0, "stay clear", ro))
        return out
    for k in range(1, SNAP_N + 1):  # along the stick's arc, one control step per waypoint
        pk, rk = _stick_grip(OPEN_TO + (-PAST - OPEN_TO) * k / SNAP_N, 0.0, lean, SNAP_IN)
        if SNAP_FIXED:  # the hand keeps its angle; the stick turns between the fingertips
            rk = ro
        out.append(_w(SNAP_S / SNAP_N, pk, STICK_PINCH, "snap it shut", rk))
    ps, rs = _stick_grip(-PAST, 0.0, lean)
    out.append(_w(1.0, ps, STICK_PINCH, "keep it shut", rs))
    out.append(_w(0.8, ps, OPEN_C, "let go", rs))
    pb, _ = _stick_grip(-PAST, 0.05, lean)
    out.append(_w(1.5, pb, OPEN_C, "rise off the stick", rs))
    out.append(_w(30.0, pb + np.array([-0.03, 0.0, 0.02]), OPEN_C, "stay clear", rs))
    return out


def demo():
    if ONE:
        return {"left": _stay("left"), "right": _right_path()}
    return {"left": _left_path(), "right": _right_path()}
