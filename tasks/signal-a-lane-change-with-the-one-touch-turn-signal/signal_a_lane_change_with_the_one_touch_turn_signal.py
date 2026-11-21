"""On the station table: a car's steering column section on a bench stand, as a driver sees it from the seat: a 37 cm
steering wheel facing the robot, the column shroud behind it with the combination switch, and the turn-signal stalk
standing out of the switch on the driver's left, behind the wheel's rim. A small instrument panel behind the wheel has
the two green turn-signal arrows. Task (instruction): give the three-flash lane-change signal for a lane change to the
left. The instruction does not say how.

How it is done (Kia owner's manual, "One-touch lane change function"): "To activate an one-touch lane change
function, move the turn signal lever slightly and then release it. The lane change signals will blink 3 times." Moved
all the way ("move the lever up or down"), the stalk clicks into its locked turn position and the signal keeps flashing
until the wheel is turned back or the driver returns the stalk by hand (Mazda: "The operation can be canceled by moving
the lever in the direction opposite to which it was operated"). In a left-hand-drive car the stalk is on the column's
left; down signals left.

The hand skill: pushed 2 cm from its end, where a fingertip reaches it, the stalk closes its switch contact after
7.6 mm, meets its light lane-change stop at 13 mm (5 deg) and the crest of its detent at 28 mm (11 deg), past which it
snaps into the locked turn position at 14 deg. The finger must push it into that window, against a spring that rises
from 1.9 N at the stop to 6.3 N at the crest, and let go within about 2 s (sooner than the third flash ends); the wheel's
rim, whose back is 4.3 cm in front of the push point, and the shroud behind it leave a narrow way in from above. One
arm does it; the other is not needed.

World frame (m): x forward from the arm bases, y to the robot's left, z up; the table top is at z 0.7475.
- column: fixed. Wheel: rim 37.2 cm across (17.2 cm to the middle of its 2.8 cm grip), in the plane x 0.165, centred
  at y -0.05, 20.5 cm above the table; three spokes and a 15 cm hub. Shroud (7 cm round) from x 0.195 to 0.47; the
  combination switch housing at x 0.23 to 0.29, with the wiper stalk (fixed here) on its right; a post and a plate
  under the shroud; the instrument panel (18 x 7 cm, its face turned 50 deg up) at x 0.33, 28 cm above the table.
- stalk: on a hinge about the x axis at (0.260, -0.002), 18.3 cm above the table; 17 cm long, pointing out to the left
  (+y), 15 deg back toward the driver and 8 deg down; a 19 mm grip on its last 6 cm, its end at (0.216, 0.161), 15.9 cm
  above the table. Its spring and detent (per side, angle from rest): 0-5 deg the light lane-change spring (0.28 N m
  at 5 deg); 5-11 deg a stiffer rise (0.92 N m at 11 deg); past 11 deg it snaps into the locked turn position at
  14 deg (stop at 15 deg), held there until pushed back inside 8 deg. Its switch contact closes at 3 deg (opens below
  2.5 deg). At the push point (14.5 cm from the hinge axis) that is 1.9 N at the stop and 6.3 N at the crest.
- flasher (the device's output): a contact closed starts the flashing (0.75 s period, lamp lit 0.4 s); the flasher
  gives at least 3 flashes, and goes on flashing while the contact stays closed or the stalk is locked. Each flash
  lights the green arrow on that side.

Score (read at the end of the run; 0 for the rest of the run once the right arrow has flashed):
- 0: no left signal; the right signal ever flashed (the stalk went up); or the left signal still locked at the end.
- 0.25: the stalk was locked into the left turn position (the click) at some point, then returned to rest.
- 0.5: a left lane-change signal (the stalk pushed down far enough to close its contact, never locked), but more
  than 3 flashes, or more than one push, or at the end the stalk is not at rest (within 1 deg of where it rests), the
  flasher still flashing or a finger within 1 cm of the stalk.
- 1.0 (success, held 1 s): exactly 3 left flashes from one push that closed the contact without passing the detent's
  crest, never locked, the stalk at rest and the flasher stopped, every finger point 1 cm or more from the stalk.
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
from bodykit import as_numpy, capsule, on_reset, on_step, rotation, segment  # noqa: E402
from yam_arena.demo import Waypoint, grip_rotation  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

allow_matrix_grips()

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"

# -- the layout ----------------------------------------------------------------------------------------------------
XW = 0.165  # the wheel's plane (the middle of its rim)
YC = -0.05  # the column's axis
ZC = TOP + 0.205
RIM_R, RIM_T = 0.172, 0.014  # rim: radius to the middle of the grip, grip radius (37.2 cm across)
SHROUD_R = 0.035
PIVOT = np.array([XW + 0.095, YC + 0.048, ZC - 0.022])
_d = np.array([-math.sin(math.radians(15.0)), math.cos(math.radians(15.0)), -math.tan(math.radians(8.0))])
STALK_D = _d / np.linalg.norm(_d)  # pivot -> tip
STALK_L = 0.170
GRIP_L, GRIP_R, SHAFT_R = 0.060, 0.0095, 0.007
PUSH_S = STALK_L - 0.020  # where the finger pushes, along the stalk from the pivot
PUSH0 = PIVOT + STALK_D * PUSH_S
_rel = PUSH0 - PIVOT
ARM_R = float(math.hypot(_rel[1], _rel[2]))  # lever arm about the hinge axis (x) of the push point

# the stalk's spring and detent (angles from rest, either side)
CONTACT_ON, CONTACT_OFF = 3.0, 2.5  # deg: the switch contact closes / opens
SOFT, CREST, LOCK, UNLOCK, LIMIT = 5.0, 11.0, 14.0, 8.0, 15.0  # deg
T_SOFT, T_CREST = 0.28, 0.92  # N m at the soft stop and at the detent's crest
K1 = T_SOFT / math.radians(SOFT)
K2 = (T_CREST - T_SOFT) / math.radians(CREST - SOFT)
K3 = float(os.environ.get("TS_K3", "8.0"))  # N m/rad: the locked position's detent spring
SPRING_C = float(os.environ.get("TS_DAMP", "0.05"))  # N m s/rad
# the flasher
DT = 0.05  # s per control step (20 Hz)
PERIOD, LIT = 15, 8  # control steps: 0.75 s period, lamp lit 0.4 s
MIN_FLASHES = 3

BLACK, DARKG, SHROUD, STEEL, PANEL, ARROW_OFF, ARROW_ON, WHITE = (
    (0.07, 0.07, 0.08), (0.16, 0.16, 0.17), (0.22, 0.22, 0.24), (0.55, 0.57, 0.6), (0.05, 0.05, 0.06),
    (0.05, 0.12, 0.06), (0.25, 1.0, 0.35), (0.85, 0.85, 0.85))
SILVER = (0.78, 0.79, 0.82)


def _v(p) -> tuple:
    return tuple(float(x) for x in p)


ORIGIN = np.array([XW, YC, ZC])  # the linkage origin: the wheel's centre
PANEL_C = np.array([XW + 0.165, YC, ZC + 0.075])  # the instrument panel's face centre
PANEL_TILT = math.radians(50.0)  # its face turned up from vertical toward the camera above
_PN = np.array([-math.cos(PANEL_TILT), 0.0, math.sin(PANEL_TILT)])  # the face's normal (toward driver and up)
_PU = np.array([math.sin(PANEL_TILT), 0.0, math.cos(PANEL_TILT)])  # up along the face


def _arrow(centre, side: int, color, lift: float = 0.0, collide: bool = False) -> list:
    """A turn-signal arrow on the panel's face, 3.2 cm long: a bar and a head, pointing to +y (side=+1, left) or -y."""
    c = np.asarray(centre) + _PN * (0.004 + lift)
    y = lambda v: np.array([0.0, side * v, 0.0])  # noqa: E731
    bar = BoxShape(_v(c - y(0.007)), (0.004, 0.020, 0.010), color, collide=collide, quat_xyzw=_quat_face())
    head = [segment(_v(c + y(0.016)), _v(c + y(0.002) + _PU * 0.014), 0.007, color, collide=collide),
            segment(_v(c + y(0.016)), _v(c + y(0.002) - _PU * 0.014), 0.007, color, collide=collide)]
    return [bar] + head


def _quat_face() -> tuple:
    a = PANEL_TILT  # turned about y so that the box's x axis is the panel's normal and its z axis runs up the face
    return (0.0, math.sin(a / 2), 0.0, math.cos(a / 2))


ARROW_L = PANEL_C + np.array([0.0, 0.05, 0.0])
ARROW_R = PANEL_C + np.array([0.0, -0.05, 0.0])


def _column() -> Linkage:
    rel = lambda p: _v(np.asarray(p, float) - ORIGIN)  # noqa: E731
    o = ORIGIN
    sh = []
    # the wheel: the rim as 28 short capsules round a circle in the y-z plane, three spokes, the hub (airbag cover)
    n = 28
    for k in range(n):
        a0, a1 = 2 * math.pi * k / n, 2 * math.pi * (k + 1) / n
        p0 = o + RIM_R * np.array([0.0, math.cos(a0), math.sin(a0)])
        p1 = o + RIM_R * np.array([0.0, math.cos(a1), math.sin(a1)])
        sh.append(capsule(rel(p0), rel(p1), RIM_T, BLACK))
    for a in (0.0, math.pi, -math.pi / 2):  # spokes to the left, right and bottom
        p = o + (RIM_R - 0.01) * np.array([0.0, math.cos(a), math.sin(a)])
        sh.append(segment(rel(o + np.array([0.01, 0.0, 0.0])), rel(p + np.array([0.01, 0.0, 0.0])), 0.022, DARKG))
    sh += [CylShape(rel(o + np.array([0.0, 0.0, 0.0])), 0.075, 0.05, DARKG, axis="X"),
           CylShape(rel(o + np.array([-0.026, 0.0, 0.0])), 0.06, 0.002, BLACK, collide=False, axis="X"),
           CylShape(rel(o + np.array([0.04, 0.0, 0.0])), 0.045, 0.03, BLACK, axis="X")]  # the boss behind the hub
    # the column shroud, the combination switch housing (with the wiper stalk's stub on the right), the stand
    x0, x1 = XW + 0.03, XW + 0.305
    sh += [CylShape(rel(((x0 + x1) / 2, YC, ZC)), SHROUD_R, x1 - x0, SHROUD, axis="X"),
           BoxShape(rel((XW + 0.095, YC, ZC)), (0.060, 0.11, 0.08), SHROUD),
           capsule(rel((XW + 0.095, YC - 0.055, ZC - 0.005)), rel((XW + 0.07, YC - 0.15, ZC - 0.012)), 0.0065,
                   DARKG)]
    post_x = XW + 0.24
    sh += [BoxShape(rel((post_x, YC, (TOP + ZC - SHROUD_R) / 2)), (0.04, 0.05, ZC - SHROUD_R - TOP), STEEL),
           BoxShape(rel((post_x, YC, TOP + 0.005)), (0.16, 0.20, 0.01), STEEL)]
    # the instrument panel behind the wheel: a dark face turned up, on a block, with the two arrows (unlit)
    sh += [BoxShape(rel(PANEL_C - _PN * 0.012), (0.024, 0.18, 0.07), PANEL, quat_xyzw=_quat_face()),
           BoxShape(rel(PANEL_C - _PN * 0.03 - np.array([0.0, 0.0, 0.045])), (0.04, 0.18, 0.07), SHROUD)]
    for c, s in ((ARROW_L, 1), (ARROW_R, -1)):
        sh += _arrow(np.asarray(rel(c)), s, ARROW_OFF)
    # the stalk: a 14 mm shaft from inside the housing, a 19 mm grip on its last 6 cm, a white mark on the grip
    tip = PIVOT + STALK_D * STALK_L
    g0 = PIVOT + STALK_D * (STALK_L - GRIP_L)
    stalk = [capsule(rel(PIVOT), rel(g0), SHAFT_R, DARKG),
             capsule(rel(g0 + STALK_D * 0.004), rel(tip - STALK_D * GRIP_R), GRIP_R, BLACK),
             CylShape(rel(PIVOT + STALK_D * (STALK_L - 0.030) + np.array([0.0, 0.0, GRIP_R - 0.0005])), 0.003,
                      0.002, WHITE, collide=False),
             # the headlight switch's silver ring between the shaft and the grip
             capsule(rel(g0 - STALK_D * 0.001), rel(g0 + STALK_D * 0.003), GRIP_R + 0.0005, SILVER, collide=False)]
    links = [Link("stalk", tuple(stalk), mass=0.04, anchor=_v(PIVOT), axis="X", limits=(-LIMIT, LIMIT),
                  stiffness=K1, spring_damping=SPRING_C, damping=0.0)]
    return Linkage.build("column", pos=_v(ORIGIN), mass=3.0, color=SHROUD, shapes=sh, links=links,
                         friction=(0.7, 0.6), contact_offset=0.002, solver_iterations=32)


COLUMN = _column()
# the lit arrows: visual only (nothing collides with them), kept below the table and laid over an arrow while it is lit
HIDE = np.array([0.20, -0.10, TOP - 0.30])
LAMPS = {s: Body.build(f"lamp_{s}", pos=_v(HIDE + np.array([0.0, 0.0, -0.05 * i])), mass=0.001, color=ARROW_ON,
                       gravity=False, damping=(5.0, 5.0),
                       shapes=_arrow(np.zeros(3), 1 if s == "left" else -1, ARROW_ON, lift=0.001))
         for i, s in enumerate(("left", "right"))}
OBJECTS = [COLUMN, LAMPS["left"], LAMPS["right"]]

# -- tracking and scoring -------------------------------------------------------------------------------------------
REST_DEG = 1.0
CLEAR = 0.01
_DEBUG = os.environ.get("TS_DEBUG") == "1"
_DEBUG_EVERY = int(os.environ.get("TS_DEBUG_EVERY", "2"))
_EP: dict = {}
_IDS: dict = {}
STALK_SAMPLES = np.array([list(STALK_D * s) for s in np.linspace(0.02, STALK_L, 14)])  # from the pivot


@on_reset
def _reset() -> None:
    _EP.update(tick=0, step=0, counted=-1, failed="", theta=0.0, peak=0.0, locked=False, locked_ever=False,
               contact={"left": False, "right": False}, presses={"left": 0, "right": 0}, flashes={"left": 0,
               "right": 0}, run_side=None, n0=0, until=0, lit={"left": False, "right": False}, near=1.0, zone="rest",
               drive=None, tcp={}, gaps={}, lamp_pose={})


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


def _set_drive(asset, j: int, k: float, target_deg: float) -> None:
    """The stalk's spring for the coming control step: ``k`` N m/rad toward ``target_deg`` (joint angle)."""
    import torch

    key = (round(k, 6), round(target_deg, 6))
    if _EP["drive"] == key:
        return
    _EP["drive"] = key
    dev = asset.device
    asset.write_joint_stiffness_to_sim_index(stiffness=torch.tensor([[float(k)]], dtype=torch.float32, device=dev),
                                             joint_ids=[j])
    asset.write_joint_damping_to_sim_index(damping=torch.tensor([[float(SPRING_C)]], dtype=torch.float32,
                                                                device=dev), joint_ids=[j])
    asset.set_joint_position_target_index(target=torch.tensor([[math.radians(target_deg)]], dtype=torch.float32,
                                                              device=dev), joint_ids=[j])


def _spring(theta: float) -> tuple[float, float, str]:
    """The stalk's spring and detent as a piecewise-linear spring: (stiffness, target joint angle in deg, zone) for
    the stalk at joint angle ``theta`` (deg; negative = down = left)."""
    s = -1.0 if theta < 0 else 1.0
    a = abs(theta)
    if _EP["locked"]:
        if a < UNLOCK:
            _EP["locked"] = False
        else:
            return K3, s * LOCK, "locked"
    if a >= CREST:
        _EP["locked"] = True
        _EP["locked_ever"] = True
        if s > 0 and not _EP["failed"]:
            _EP["failed"] = "the stalk locked into the right turn position"
        return K3, s * LOCK, "locked"
    if a <= SOFT:
        return K1, 0.0, "light"
    # 5-11 deg: torque T_SOFT + K2 (a - SOFT), written as K2 (a - t) with t = SOFT - T_SOFT / K2
    return K2, s * (SOFT - math.degrees(T_SOFT / K2)), "stiff"


def _flasher(theta: float) -> None:
    """The flasher module: counts flashes per side from the stalk's switch contacts (the device's own output)."""
    t = _EP["tick"]
    down = -theta  # deg, positive = down = left
    for side, a in (("left", down), ("right", -down)):
        was = _EP["contact"][side]
        now = a >= CONTACT_ON or (was and a >= CONTACT_OFF) or (_EP["locked"] and a >= UNLOCK)
        _EP["contact"][side] = now
        if now and not was:
            _EP["presses"][side] += 1
            if _EP["run_side"] != side or t >= _EP["until"]:
                _EP.update(run_side=side, n0=t)
                for s2 in ("left", "right"):
                    _EP["lit"][s2] = False
            _EP["until"] = max(_EP["until"] if _EP["run_side"] == side else 0, _EP["n0"] + MIN_FLASHES * PERIOD)
        if now and _EP["run_side"] == side:  # held or locked: flash on to the end of the cycle
            k = (t - _EP["n0"]) // PERIOD + 1
            _EP["until"] = max(_EP["until"], _EP["n0"] + k * PERIOD)
    side = _EP["run_side"]
    for s2 in ("left", "right"):
        lit = bool(side == s2 and t < _EP["until"] and (t - _EP["n0"]) % PERIOD < LIT)
        if lit and not _EP["lit"][s2]:
            _EP["flashes"][s2] += 1
            if s2 == "right" and not _EP["failed"]:
                _EP["failed"] = "the right signal flashed (the stalk went up)"
        _EP["lit"][s2] = lit


def _show_lamps(base, origin) -> None:
    import torch

    for side in ("left", "right"):
        lit = _EP["lit"][side]
        if _EP["lamp_pose"].get(side) == lit:
            continue
        _EP["lamp_pose"][side] = lit
        p = (ARROW_L if side == "left" else ARROW_R) if lit else HIDE + np.array([0.0, 0.0, -0.05 * (side == "right")])
        p = np.asarray(p) + origin
        pose = np.array([[p[0], p[1], p[2], 0.0, 0.0, 0.0, 1.0]])
        obj = base.scene[f"lamp_{side}"]
        obj.write_root_link_pose_to_sim_index(root_pose=torch.tensor(pose, dtype=torch.float32, device=base.device))
        obj.write_root_com_velocity_to_sim_index(root_velocity=torch.zeros((1, 6), dtype=torch.float32,
                                                                           device=base.device))


@on_step
def _track(env) -> None:
    try:
        _track_inner(env)
    except Exception:
        import traceback

        print("TS_TRACEBACK\n" + traceback.format_exc(), flush=True)
        raise


def _track_inner(env) -> None:
    from scipy.spatial import cKDTree

    from yam_arena import ik, station
    from yam_arena.constants import ARM_JOINTS, FINGER_JOINTS

    _EP["tick"] += 1
    base = getattr(env, "unwrapped", env)
    origin = as_numpy(base.scene.env_origins[0])
    col = base.scene["column"]
    if "stalk" not in _IDS:
        _IDS["stalk"] = list(col.joint_names).index("stalk_turn")
        _IDS["stalk_body"] = list(col.body_names).index("stalk")
        _IDS["fingers"] = _finger_points()
    j = _IDS["stalk"]
    theta = math.degrees(float(as_numpy(col.data.joint_pos)[0][j]))
    k, target, zone = _spring(theta)
    _set_drive(col, j, k, target)
    _flasher(theta)
    _show_lamps(base, origin)
    # fingers near the stalk
    sp = as_numpy(col.data.body_link_pose_w)[0][_IDS["stalk_body"]]
    sr = rotation(sp[3:7])
    pts = (sp[:3] - origin) + (PIVOT - ORIGIN + STALK_SAMPLES) @ sr.T  # the link frame sits at the linkage origin
    tree = cKDTree(pts)
    near = 1.0
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
        fp = np.concatenate([_IDS["fingers"][n] @ rotation(poses[i][3:7]).T + poses[i][:3] - origin
                             for n, i in zip(("tip_left", "tip_right"), _IDS[side + "_tips"])])
        near = min(near, float(tree.query(fp, k=1)[0].min()))
    _EP.update(theta=theta, peak=max(_EP["peak"], -theta), zone=zone, near=near, tipz=float(pts[-1][2] - TOP))


def _tally() -> dict:
    rest = abs(_EP["theta"]) <= REST_DEG
    running = _EP["tick"] < _EP["until"]
    one_touch = (_EP["presses"]["left"] >= 1 and not _EP["locked_ever"] and _EP["flashes"]["right"] == 0)
    done = bool(one_touch and _EP["presses"]["left"] == 1 and _EP["flashes"]["left"] == MIN_FLASHES and rest
                and not running and _EP["near"] >= CLEAR and not _EP["failed"])
    return dict(rest=rest, running=running, one_touch=one_touch, done=done)


def _debug(t) -> None:
    _EP["step"] += 1
    if (_EP["step"] - 1) % _DEBUG_EVERY:
        return
    print(f"DBG step={_EP['step']} theta={_EP['theta']:+.2f}deg zone={_EP['zone']} peak_down={_EP['peak']:.2f} "
          f"locked={int(_EP['locked'])}/{int(_EP['locked_ever'])} contact=L{int(_EP['contact']['left'])}"
          f"R{int(_EP['contact']['right'])} presses=L{_EP['presses']['left']}R{_EP['presses']['right']} "
          f"flashes=L{_EP['flashes']['left']}R{_EP['flashes']['right']} lit=L{int(_EP['lit']['left'])}"
          f"R{int(_EP['lit']['right'])} running={int(t['running'])} near={_EP['near'] * 1000:.0f}mm "
          f"tipz={_EP.get('tipz', 0) * 1000:.0f}mm rest={int(t['rest'])} done={int(t['done'])} "
          f"failed={_EP['failed'] or '-'} | "
          + " ".join(f"{k} tcp=({v[0]:.3f},{v[1]:.3f},{v[2] - TOP:.3f}) gap={_EP['gaps'][k] * 1000:.0f}"
                     for k, v in _EP["tcp"].items()), flush=True)


def score(objects) -> float:
    """0 once the right signal has flashed or while the left signal is locked; 0.25 locked then returned; 0.5 a
    lane-change signal from the light stop that is not yet the clean one-touch signal. Full score is success()."""
    t = _tally()
    if _DEBUG:
        _debug(t)
    if _EP["failed"] or _EP["locked"] or _EP["presses"]["left"] == 0:
        return 0.0
    if _EP["locked_ever"]:
        return 0.25 if t["rest"] else 0.0
    return 0.5


def success(objects) -> bool:
    return bool(_tally()["done"])


# -- scripted demo --------------------------------------------------------------------------------------------------
# Open loop, from the known layout; the right arm stays at the ready pose. The left gripper, closed, its fingers
# closing along y (so the hand is 6.5 cm thick front to back where it passes the rim), comes round the left of the
# wheel's rim, over the stalk's grip, and down on its top 2 cm from the end. It pushes the stalk down about 7.5 deg
# (aimed 2.4 cm below where it touched: 1.9 cm of stalk travel and about 5 mm the arm gives under the 3.7 N), holds it
# 0.3 s, lets it come back up by rising, and moves up and away to the left.
# TS_MODE=full pushes it down 4.5 cm as for a turn (it locks: the wrong way); TS_MODE=hold holds it 3 s (5 flashes).
MODE = os.environ.get("TS_MODE", "demo")
CLOSED = 0.0
DOWN = grip_rotation(0.0)  # fingers closing along y, side by side along the stalk
PRESS = float(os.environ.get("TS_PRESS", "0.024"))  # how far below the touch the TCP is told to go
TIP_BELOW = float(os.environ.get("TS_TIP_BELOW", "0.013"))  # the fingertips' lowest point below the TCP
HOLD_S = float(os.environ.get("TS_HOLD", "0.3"))
if MODE == "full":
    PRESS = 0.045
if MODE == "hold":
    HOLD_S = 3.0
TOUCH = PUSH0 + np.array([0.0, 0.0, GRIP_R + TIP_BELOW])  # the TCP with the fingertips on the grip's top


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
    return [Waypoint(t, tuple(float(v) for v in pos), CLOSED, "stay put", rot)]


def _arm_path() -> list:
    up = np.array([0.0, 0.0, 1.0])
    above = TOUCH + up * 0.03
    out = _lead_in("left", above, DOWN, CLOSED, "over the stalk's grip, behind the rim, fingers closed")
    out += [_w(1.5, TOUCH + up * 0.005, CLOSED, "down to just above the grip", DOWN),
            _w(0.5, TOUCH + up * 0.005, CLOSED, "settle", DOWN),
            _w(0.6, TOUCH - up * PRESS, CLOSED, "push the stalk down to its light stop", DOWN),
            _w(HOLD_S, TOUCH - up * PRESS, CLOSED, "hold it there", DOWN),
            _w(0.5, TOUCH + up * 0.02, CLOSED, "rise: the stalk springs back", DOWN),
            _w(1.0, TOUCH + up * 0.035, CLOSED, "up, clear of the stalk", DOWN),
            _w(2.0, TOUCH + up * 0.035 + np.array([-0.01, 0.07, 0.0]), CLOSED, "out to the left of the rim", DOWN),
            _w(12.0, TOUCH + up * 0.02 + np.array([-0.04, 0.10, 0.0]), CLOSED, "wait there", DOWN)]
    return out


def demo():
    return {"left": _arm_path(), "right": _stay("right")}
