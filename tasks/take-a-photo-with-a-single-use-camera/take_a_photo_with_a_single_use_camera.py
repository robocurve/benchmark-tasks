"""On the station table: a single-use 35 mm film camera (like a Kodak FunSaver, 27 exposures: 120 x 59 x 35 mm,
117 g) stands on its base, lens away from the robot. Its film was last advanced and fired, so its thumbwheel is
unwound and its shutter locked. Task (instruction): take one photo with it and leave it standing on the table.

How it is done: the camera's back sticker and its product descriptions say to advance the film with the thumbwheel
"until it stops turning" and then press the shutter button; "The film advance wheel is interlocked with the shutter
mechanism, to avoid the risk of double-exposures" (Digital Camera World review of the FunSaver); "The film winder sits
at the top right of the rear panel".

The hand skill: the thumbwheel is a ridged 24 mm wheel standing only 3 mm proud of the camera's back, turned by
rubbing its edge sideways in repeated strokes (about 1.5 turns, our estimate, so many fingertip strokes: ten in the demo);
each stroke pushes the 117 g camera, which slides away unless the other hand holds it. Pressing the shutter
before the wheel reaches its stop does nothing.

World frame (m): x forward from the arm bases, y to the robot's left, z up; the table top is at z 0.7475.
- camera: loose, standing on its base, its body (120 mm along y, 59 mm tall, 30 mm deep plus a 5 mm lens barrel in
  front) centred at (0.225, 0.02), back (x 0.21) toward the robot. Seen from the back, its right end is at -y.
  Thumbwheel: 24 mm across its 36 ridges, 7 mm thick, axis vertical at (0.219, -0.024), its top 1.5 mm under the
  camera's top, in a recess (20 mm deep, 42 mm wide, 10 mm tall) at the camera's top-back-right corner; it stands
  3 mm proud of the back. It turns clockwise seen from above (its back edge moving toward +y) against 0.01 N m of friction; an anti-reverse pawl stops
  it turning back by more than about 1 degree; it stops at 540 degrees (one frame) until the shutter has fired.
  Shutter button: a 9 x 9 mm cap 1.5 mm proud of the top, its centre at (0.229, 0.012); 1.0 mm down fires it. Its
  spring needs about 2.5 N to start and 3.5 N at 1 mm; before the wheel is at its stop it is locked (about 100 N
  would not move it 1 mm).
- frame counter: 20 exposures left at the start (tracked, shown in the debug lines).

Score:
- 0.4: the thumbwheel has been wound to its stop (within 2 degrees of 540 degrees).
- 0.6: the shutter fired after winding (a photo taken).
- 1.0 (success, held 1 s): exactly one photo taken, the camera standing on its base on the table (tilted under 10
  degrees, its lowest point within 3 mm of the table), every finger point 1 cm or more from it, and still (under 1 mm
  in 1 s).
A second photo (wound and fired again) keeps 0.6. Pressing the shutter before winding does nothing (score 0).
"""

from __future__ import annotations

import itertools
import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

from bodykit import BoxShape, CylShape, Link, Linkage, allow_matrix_grips  # noqa: E402
from bodykit import as_numpy, on_reset, on_step, rotation  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

allow_matrix_grips()

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"
_E = os.environ.get

# -- the camera, in its own frame (origin at the body's centre, x toward the lens, y to the camera's left seen from
# the back... i.e. world-aligned at the start) ---------------------------------------------------------------------
CAM_XY = np.array([0.225, 0.02])
W, H, D = 0.120, 0.059, 0.030  # body width (y), height (z), depth (x, without the lens barrel)
BACK, FRONT, TOPZ = -D / 2, D / 2, H / 2
CAM_Z = TOP + H / 2 + 0.0005  # spawned 0.5 mm above the table
RECESS_X, RECESS_Y, RECESS_Z = 0.020, 0.042, 0.010  # the wheel's recess at the top-back-right corner

WHEEL_R = 0.012  # to the ridge tips
WHEEL_T = 0.007
WHEEL_PROUD = 0.003
WHEEL_AXIS = np.array([BACK + WHEEL_R - WHEEL_PROUD, -W / 2 + 0.016, TOPZ - 0.0015 - WHEEL_T / 2])
N_RIDGES = 36
WIND_DEG = 540.0  # one frame: 1.5 turns (estimate)
WHEEL_LIMITS = tuple(float(v) for v in _E("FUNSAVER_WHEEL_LIMITS", "-1200,30").split(","))  # deg (winding is negative)
WOUND_TOL = 2.0
WHEEL_FRIC = float(_E("FUNSAVER_WHEEL_FRIC", "0.01"))  # N m
K_PAWL = float(_E("FUNSAVER_K_PAWL", "2.0"))  # N m/rad: the anti-reverse pawl and the wound stop
PAWL_SLACK = 1.0  # deg

CAP = np.array([0.004, -0.008])  # the shutter cap's centre (x, y)
CAP_W, CAP_PROUD = 0.009, 0.0015
PIVOT_R = 0.030  # the button lever's pivot, inside the body, this far toward +y from the cap's centre
PIVOT = np.array([CAP[0], CAP[1] + PIVOT_R, TOPZ - 0.002])
FIRE_MM, BOTTOM_MM = 1.0, 1.4
FIRE_DEG = math.degrees(FIRE_MM / 1000 / PIVOT_R)
BOTTOM_DEG = math.degrees(BOTTOM_MM / 1000 / PIVOT_R)
PRELOAD_DEG = 4.8  # the spring's target above the cap's rest stop: about 2.5 N to start, 3.5 N at 1 mm
K_BUTTON = float(_E("FUNSAVER_K_BUTTON", "0.9"))  # N m/rad, armed
K_LOCKED = float(_E("FUNSAVER_K_LOCKED", "30.0"))  # N m/rad, locked
FRAMES0 = 20

YELLOW, BLACK, DARK, GREY, WHITE, RED, GLASS = ((0.95, 0.75, 0.10), (0.08, 0.08, 0.09), (0.18, 0.18, 0.2),
                                                (0.5, 0.5, 0.52), (0.95, 0.95, 0.92), (0.85, 0.12, 0.1),
                                                (0.25, 0.3, 0.38))


def _v(p) -> tuple:
    return tuple(float(x) for x in p)


def _camera() -> Linkage:
    c = np.array([CAM_XY[0], CAM_XY[1], CAM_Z])
    lo_h = H - RECESS_Z
    zt = TOPZ - RECESS_Z / 2
    base = [BoxShape((0.0, 0.0, -H / 2 + lo_h / 2), (D, W, lo_h), YELLOW),
            BoxShape((0.0, RECESS_Y / 2, zt), (D, W - RECESS_Y, RECESS_Z), YELLOW),
            BoxShape((BACK + RECESS_X + (D - RECESS_X) / 2, -W / 2 + RECESS_Y / 2, zt), (D - RECESS_X, RECESS_Y,
                                                                                       RECESS_Z), YELLOW),
            # lens barrel and glass, front
            CylShape((FRONT + 0.0025, 0.012, 0.004), 0.013, 0.005, BLACK, axis="X"),
            CylShape((FRONT + 0.0052, 0.012, 0.004), 0.007, 0.0006, GLASS, collide=False, axis="X"),
            # flash window (front), viewfinder windows (front and back), counter window and the label band (visual)
            BoxShape((FRONT + 0.0003, 0.042, 0.016), (0.0006, 0.026, 0.014), WHITE, collide=False),
            BoxShape((FRONT + 0.0003, -0.030, 0.019), (0.0006, 0.012, 0.009), GLASS, collide=False),
            BoxShape((BACK - 0.0003, -0.010, 0.019), (0.0006, 0.012, 0.009), GLASS, collide=False),
            BoxShape((0.004, 0.008, TOPZ + 0.0002), (0.007, 0.007, 0.0004), DARK, collide=False),
            BoxShape((BACK - 0.0003, 0.010, -0.012), (0.0006, 0.090, 0.022), RED, collide=False),
            BoxShape((0.0, 0.0, -H / 2 + 0.002), (D + 0.0004, W + 0.0004, 0.004), BLACK, collide=False)]
    wa = WHEEL_AXIS
    core_r = WHEEL_R - 0.0008
    wheel = [CylShape(_v(wa), core_r, WHEEL_T, BLACK, axis="Z")]
    for k in range(N_RIDGES):
        a = 2 * math.pi * k / N_RIDGES
        r = WHEEL_R - 0.0008
        wheel.append(BoxShape((wa[0] + r * math.cos(a), wa[1] + r * math.sin(a), wa[2]), (0.0016, 0.0011, WHEEL_T),
                              DARK, yaw=a))
    wheel.append(BoxShape((wa[0] + core_r / 2, wa[1], wa[2] + WHEEL_T / 2 + 0.0002), (core_r, 0.002, 0.0004), WHITE,
                          collide=False))  # a white line on its top, so its turning shows
    cap = [BoxShape((CAP[0], CAP[1], TOPZ + CAP_PROUD - 0.0015), (CAP_W, CAP_W, 0.003), BLACK)]
    links = [Link("wheel", tuple(wheel), mass=0.008, anchor=_v(c + wa), axis="Z", friction=WHEEL_FRIC,
                  stiffness=K_PAWL, spring_damping=0.0005, limits=WHEEL_LIMITS),
             Link("button", tuple(cap), mass=0.004, anchor=_v(c + PIVOT), axis="X", stiffness=K_LOCKED,
                  spring_damping=0.002, limits=(0.0, BOTTOM_DEG))]
    return Linkage.build("camera", pos=_v(c), mass=0.105, color=YELLOW, floating=True, friction=(0.8, 0.7),
                         shapes=base, links=links, contact_offset=0.001, solver_iterations=48)


CAMERA = _camera()
OBJECTS = [CAMERA]

# -- tracking and scoring -------------------------------------------------------------------------------------------
CLEAR = 0.010
UPRIGHT_DEG = 10.0
REST_MM = 3.0
STILL_STEPS, STILL_MM = 20, 1.0
_DEBUG = _E("FUNSAVER_DEBUG") == "1"
_DEBUG_EVERY = int(_E("FUNSAVER_DEBUG_EVERY", "5"))
_EP: dict = {}
_IDS: dict = {}
_HALF = np.array([D / 2 + 0.005, W / 2, H / 2])  # the body's half size (with the lens barrel, roughly)
_CORNERS = np.array([[sx * D / 2, sy * W / 2, sz * H / 2] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)])


@on_reset
def _reset() -> None:
    _EP.update(tick=0, step=0, counted=-1, recent=[], wind=0.0, peak=0.0, stop=WIND_DEG, wound_ever=False,
               armed=False, shots=0, frames=FRAMES0, early_press=0, btn=0.0, relock=False, k=(None, None),
               near=1.0, low=1.0, tilt=0.0, pos=np.zeros(3), tcp={}, gaps={})


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


def _drives(cam, wheel_k: float, wheel_target_deg: float, button_k: float) -> None:
    """The pawl and stop on the wheel (a stiff spring when engaged, free otherwise) and the shutter's spring
    (soft when armed, stiff when locked), both through the joints' drives."""
    import torch

    dev = cam.device
    ids = [_IDS["wheel"], _IDS["button"]]
    if _EP["k"] != (wheel_k, button_k):
        _EP["k"] = (wheel_k, button_k)
        cam.write_joint_stiffness_to_sim_index(stiffness=torch.tensor([[wheel_k, button_k]], dtype=torch.float32,
                                                                      device=dev), joint_ids=ids)
    tgt = [math.radians(wheel_target_deg), -math.radians(PRELOAD_DEG)]
    cam.set_joint_position_target_index(target=torch.tensor([tgt], dtype=torch.float32, device=dev), joint_ids=ids)
    cam.write_data_to_sim()


@on_step
def _track(env) -> None:
    try:
        _track_inner(env)
    except Exception:
        import traceback

        print("FUNSAVER_TRACEBACK\n" + traceback.format_exc(), flush=True)
        raise


def _track_inner(env) -> None:
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
    cam = base.scene["camera"]
    if "wheel" not in _IDS:
        jn = list(cam.joint_names)
        _IDS["wheel"] = jn.index("wheel_turn")
        _IDS["button"] = jn.index("button_turn")
    q = as_numpy(cam.data.joint_pos)[0]
    wind = -math.degrees(float(q[_IDS["wheel"]]))  # clockwise seen from above is winding
    btn = math.degrees(float(q[_IDS["button"]]))
    # the wheel: free between the pawl and the stop
    _EP["peak"] = min(max(_EP["peak"], wind), _EP["stop"])
    if wind > _EP["stop"]:
        wk, wt = K_PAWL, _EP["stop"]
    elif wind < _EP["peak"] - PAWL_SLACK:
        wk, wt = K_PAWL, _EP["peak"]
    else:
        wk, wt = 0.0, wind
    wound = wind >= _EP["stop"] - WOUND_TOL
    if wound and not _EP["armed"] and _EP["shots"] < (_EP["stop"] / WIND_DEG):
        _EP["armed"] = True
        _EP["wound_ever"] = True
        _EP["frames"] = FRAMES0 - int(_EP["stop"] / WIND_DEG)  # the counter moves on as the film advances
    if _EP["armed"] and btn >= FIRE_DEG:  # the shutter fires
        _EP["armed"] = False
        _EP["shots"] += 1
        _EP["stop"] += WIND_DEG
        _EP["relock"] = True
    if not _EP["armed"] and btn >= FIRE_DEG * 0.5 and not _EP["relock"]:
        _EP["early_press"] += 1
    if _EP["relock"] and btn < 0.3:
        _EP["relock"] = False
    bk = K_BUTTON if (_EP["armed"] or _EP["relock"]) else K_LOCKED
    _drives(cam, wk, -wt, bk)
    pose = as_numpy(cam.data.root_link_pose_w)[0]
    pp, pr = pose[:3] - origin, rotation(pose[3:7])
    low = float((pp + _CORNERS @ pr.T)[:, 2].min())
    near = 1.0
    for fp in fingers.values():
        rel = np.abs((fp - pp) @ pr) - _HALF
        near = min(near, float(np.linalg.norm(np.maximum(rel, 0.0), axis=1).min()))
    tilt = math.degrees(math.acos(min(1.0, abs(float(pr[2, 2])))))
    _EP.update(wind=wind, btn=btn, near=near, low=low, tilt=tilt, pos=pp, wk=wk)


def _tally(o) -> dict:
    recent = _EP["recent"]
    if _EP["counted"] != _EP["tick"]:
        _EP["counted"] = _EP["tick"]
        recent.append(np.asarray(_EP["pos"], float).copy())
        del recent[:-STILL_STEPS]
    still = len(recent) == STILL_STEPS and all(np.abs(p - recent[-1]).max() * 1000 < STILL_MM for p in recent)
    standing = _EP["tilt"] <= UPRIGHT_DEG and (_EP["low"] - TOP) * 1000 <= REST_MM
    done = bool(_EP["shots"] == 1 and standing and _EP["near"] >= CLEAR and still)
    return dict(still=still, standing=standing, done=done)


def _debug(t) -> None:
    _EP["step"] += 1
    if (_EP["step"] - 1) % _DEBUG_EVERY:
        return
    p = _EP["pos"]
    print(f"DBG step={_EP['step']} wind={_EP['wind']:.1f} peak={_EP['peak']:.1f} stop={_EP['stop']:.0f} "
          f"wk={_EP.get('wk', 0):.1f} armed={int(_EP['armed'])} btn={_EP['btn']:.2f}deg shots={_EP['shots']} "
          f"frames={_EP['frames']} early={_EP['early_press']} tilt={_EP['tilt']:.1f} "
          f"low={(_EP['low'] - TOP) * 1000:.1f}mm near={_EP['near'] * 1000:.0f}mm pos=({p[0]:.3f},{p[1]:.3f},"
          f"{p[2] - TOP:.3f}) still={int(t['still'])} stand={int(t['standing'])} | "
          + " ".join(f"{k} tcp=({v[0]:.3f},{v[1]:.3f},{v[2] - TOP:.3f}) gap={_EP['gaps'][k] * 1000:.0f}"
                     for k, v in _EP["tcp"].items()), flush=True)


def score(objects) -> float:
    t = _tally(objects)
    if _DEBUG:
        _debug(t)
    if _EP["shots"] >= 1:
        return 1.0 if t["done"] else 0.6
    return 0.4 if _EP["wound_ever"] else 0.0


def success(objects) -> bool:
    return bool(_tally(objects)["done"])


# -- scripted demo --------------------------------------------------------------------------------------------------
# Open loop, from the known layout. The left gripper takes the camera's left end from above (fingers across its
# 30 mm depth) and lifts it 6 cm. The right gripper, level and pointing forward (+x) with its fingers closed, sets its
# fingertips on the thumbwheel's back edge and strokes it toward +y, backs off 8 mm, returns and strokes again, until
# the wheel stops. The left hand sets the camera back down, lets go and leaves; the right hand turns straight down and
# presses the shutter button, then leaves.
# FUNSAVER_MODE=press: the wrong way: the right hand presses the shutter without winding first (nothing happens).
# FUNSAVER_MODE=one: one arm: the right hand strokes the wheel of the standing camera without the left holding it.
OPEN_C = -0.047
MODE = _E("FUNSAVER_MODE", "demo")
L_PINCH = float(_E("FUNSAVER_LPINCH", "-0.0115"))  # 3.5 mm inside each face of the 30 mm body
LIFT = float(_E("FUNSAVER_LIFT", "0.06"))
GRIP_Y = 0.030  # the left grip's centre from the camera's centre (toward +y)
GRIP_DZ = -0.010  # the left TCP this far under the camera's top
TIP_AHEAD = 0.0125
PRESS = float(_E("FUNSAVER_PRESS", "0.0035"))  # the right fingertips aimed this far into the wheel's edge
STROKE = float(_E("FUNSAVER_STROKE", "0.012"))  # each stroke from -STROKE to +STROKE (y) about the wheel's axis
N_STROKES = int(_E("FUNSAVER_STROKES", "13"))
TIP_DZ = float(_E("FUNSAVER_TIP_DZ", "0.0"))  # the fingertips' middle this far over the wheel's mid-height
SHUTTER_DOWN = float(_E("FUNSAVER_SHUTTER_DOWN", "0.018"))  # fingertips aimed this far under the cap's top


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


DOWN_L = approach_rot((0, 0, -1), 180.0)  # left: straight down, fingers closing along x
DOWN_R = approach_rot((0, 0, -1), 0.0)  # right: straight down
PITCH = float(_E("FUNSAVER_PITCH", "30"))
AIM = np.array([math.cos(math.radians(PITCH)), 0.0, -math.sin(math.radians(PITCH))])
LEVEL_R = approach_rot(AIM, 0.0)  # right: pointing forward (+x) and 30 deg down, fingers closing in the x-z plane


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


def _cam(local, lift: float = 0.0) -> np.ndarray:
    """A point given in the camera's frame, the camera standing at its start place raised ``lift``."""
    return np.array([CAM_XY[0] + local[0], CAM_XY[1] + local[1], CAM_Z + local[2] + lift])


def _left_take() -> list:
    g = _cam((0.0, GRIP_Y, TOPZ + GRIP_DZ))
    out = _lead_in("left", g + [0, 0, 0.06], DOWN_L, OPEN_C, "over the camera's left end")
    out += [_w(2.0, g, OPEN_C, "fingers either side of it", DOWN_L),
            _w(1.0, g, L_PINCH, "hold the camera", DOWN_L),
            _w(0.5, g, L_PINCH, "settle", DOWN_L),
            _w(2.0, g + [0, 0, LIFT], L_PINCH, "lift it", DOWN_L)]
    return out


def _left_return() -> list:
    g = _cam((0.0, GRIP_Y, TOPZ + GRIP_DZ))
    down = g + [0, 0, -0.0015]  # aimed a little low: it settles on its base
    out = [_w(2.5, down, L_PINCH, "set it down", DOWN_L),
           _w(0.7, down, L_PINCH, "settle", DOWN_L),
           _w(1.0, down, OPEN_C, "let go", DOWN_L),
           _w(1.5, g + [0, 0, 0.07], OPEN_C, "rise clear", DOWN_L),
           _w(2.0, g + [-0.03, 0.12, 0.10], OPEN_C, "out of the way", DOWN_L)]
    return out


def _strokes(lift: float) -> list:
    wa = _cam(WHEEL_AXIS, lift)
    rim_x = wa[0] - WHEEL_R
    x_on = rim_x - TIP_AHEAD * AIM[0] + PRESS
    x_off = rim_x - TIP_AHEAD * AIM[0] - 0.008
    z = wa[2] + TIP_DZ - TIP_AHEAD * AIM[2]
    y0, y1 = wa[1] - STROKE, wa[1] + STROKE
    out = [_w(1.0, (x_off, y0, z), 0.0, "fingertips behind the wheel's edge", LEVEL_R)]
    for k in range(N_STROKES):
        out += [_w(0.6, (x_on, y0, z), 0.0, f"onto the wheel's edge ({k + 1})", LEVEL_R),
                _w(1.4, (x_on, y1, z), 0.0, "stroke it round", LEVEL_R),
                _w(0.4, (x_off, y1, z), 0.0, "off the wheel", LEVEL_R),
                _w(0.6, (x_off, y0, z), 0.0, "back for the next stroke", LEVEL_R)]
    return out


def _right_wind(lift: float) -> list:
    wa = _cam(WHEEL_AXIS, lift)
    p0 = (wa[0] - WHEEL_R - TIP_AHEAD * AIM[0] - 0.025, wa[1] - STROKE, wa[2] + TIP_DZ - TIP_AHEAD * AIM[2] + 0.01)
    out = _lead_in("right", p0, LEVEL_R, 0.0, "behind the camera, fingers closed")
    out += _strokes(lift)
    out += [_w(1.0, np.array(p0) + [0.0, -0.02, 0.0], 0.0, "back away", LEVEL_R)]
    return out


def _right_shutter() -> list:
    c = _cam((CAP[0], CAP[1], TOPZ + CAP_PROUD))
    over = c + [0, 0, TIP_AHEAD + 0.03]
    press = c + [0, 0, TIP_AHEAD - SHUTTER_DOWN]
    out = [_w(2.0, over, 0.0, "over the shutter button", DOWN_R),
           _w(1.5, c + [0, 0, TIP_AHEAD + 0.004], 0.0, "onto the button", DOWN_R),
           _w(1.2, press, 0.0, "press the shutter", DOWN_R),
           _w(1.5, press, 0.0, "hold", DOWN_R),
           _w(1.0, over, 0.0, "let go", DOWN_R),
           _w(1.5, over + [-0.04, -0.06, 0.05], 0.0, "away", DOWN_R)]
    return out


def demo():
    ready_r, rot_r = _ready("right")
    ready_l, rot_l = _ready("left")
    if MODE == "press":
        right = [_w(1.0, ready_r, 0.0, "start", rot_r)] + _right_shutter()
        return {"left": [_w(_dur(right) + 6.0, ready_l, OPEN_C, "stay put", rot_l)],
                "right": right + [_w(6.0, right[-1].pos, 0.0, "stay", DOWN_R)]}
    if MODE == "one":
        right = _right_wind(0.0)
        return {"left": [_w(_dur(right) + 6.0, ready_l, OPEN_C, "stay put", rot_l)],
                "right": right + [_w(6.0, right[-1].pos, 0.0, "stay", LEVEL_R)]}
    left_a = _left_take()
    wind = _right_wind(LIFT)
    right = [_w(_dur(left_a) + 0.5, ready_r, 0.0, "wait", rot_r)] + wind
    left = left_a + [_w(_dur(right) - _dur(left_a), _cam((0.0, GRIP_Y, TOPZ + GRIP_DZ), LIFT), L_PINCH,
                        "hold it while the right hand winds", DOWN_L)] + _left_return()
    wa = _cam(WHEEL_AXIS, LIFT)
    p_back = np.array(right[-1].pos)
    c = _cam((CAP[0], CAP[1], TOPZ + CAP_PROUD))
    over = c + [0, 0, TIP_AHEAD + 0.03]
    swap = _joint_path("right", _good_ik("right", p_back, LEVEL_R), _good_ik("right", over + [-0.03, -0.03, 0.03],
                                                                             DOWN_R), 0.0, "turn the hand down", 2.5)
    right += [_w(max(0.5, _dur(left) - _dur(right) - 2.5), p_back, 0.0, "wait", LEVEL_R)] + swap + _right_shutter()
    left += [_w(_dur(right) - _dur(left) + 5.0, left[-1].pos, OPEN_C, "stay clear", DOWN_L)]
    right += [_w(5.0, right[-1].pos, 0.0, "stay clear", DOWN_R)]
    return {"left": left, "right": right}
