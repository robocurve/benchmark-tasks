"""On the station table: a car's hand-operated parking brake (handbrake) lever at true size, a small hatchback's
lever sub-assembly bolted through its own bracket to a board, in a low centre-console trim with a slot. The lever lies
along y, its pivot at the right (-y) end, its grip and release button at the left (+y) end. It starts applied, pulled
up to its 4th click. Task (instruction): release the handbrake fully.

How it is done (Toyota Yaris owner's manual, "Parking brake", release): "Slightly raise the lever", "Press the button",
"Lower it completely". Under the load of the brake cable the ratchet's pawl is pressed into its tooth: the button will
not go in until the lever is raised a little off the pawl. The button must then be held in the whole way down: let go
part-way, the pawl drops into the next tooth and the lever stops there. A gripper that holds the lever cannot also push
the button along the lever's axis at its end, so one hand holds and moves the lever and the other keeps the button
pressed while the lever's end swings down through its arc.

World frame (m): x forward from the arm bases, y to the robot's left, z up; the table top is at z 0.7475.
- console: fixed. A board and trim 18 x 43 cm, 1.2 cm tall, centred on x 0.22, y -0.01, with a 6 cm slot along the
  lever; two steel bracket plates either side of the pivot.
- lever: pivot at (0.22, -0.15), 4.0 cm above the table, axis along x. Pivot to the button's end 297 mm. A pressed
  steel bar 12 mm thick and 26 mm deep from the pivot to 170 mm; a black grip 35 mm across from 170 to 272 mm; a
  25 mm end cap to 289 mm; the release button, 15 mm across, standing 8 mm out of the cap. Lever 0.5 kg.
- ratchet: teeth every 4 deg from the bottom stop (0 deg, the lever level) to 32 deg. Start: on the 4th tooth (16 deg).
  The pawl holds the lever on the highest tooth below it unless the button is pressed in 5 mm or more (its travel is
  8 mm; its spring gives 5 N at 5 mm and 8 N fully in). Lifting the lever, the pawl clicks over the teeth.
- the cable: a spring torque pulling the lever down, 0.24 N m at the bottom stop + 1.3 N m/rad above it (0.60 N m at
  the 4th click), on top of the lever's own weight (0.7 N m).
- the jam: while the lever rests on its pawl (within 0.6 deg of the tooth) the button goes in at most 1 mm.
In the simulation the pawl's stop is a jointed drum round the pivot under the lever, held by a stiff drive on the tooth
angle (the arm feels a hard stop); the jammed pawl is a stiff stop behind the button.

Score (at the end of the run; "released" = every finger point of both grippers 3 cm or more from the lever):
- 1.0 (success, held 1 s): the lever within 2 deg of its bottom stop, released.
- 0.75: the lever within 2 deg of its bottom stop, not released.
- 0.5: the lever lowered at least one tooth (14 deg or lower) but caught above the bottom.
- 0: the lever at its 4th tooth or higher.
"""

from __future__ import annotations

import itertools
import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

from bodykit import BoxShape, CylShape, Link, Linkage, allow_matrix_grips, as_numpy, on_reset, on_step  # noqa: E402
from bodykit import rotation  # noqa: E402
from yam_arena.demo import Waypoint, grip_rotation  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

allow_matrix_grips()

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"
_E = os.environ.get

# -- the layout (world, m; heights above the table) ------------------------------------------------------------------
X0 = 0.22  # the lever's plane (x)
PY = -0.15  # the pivot (y)
H_P = 0.040  # the pivot above the table
L_END = 0.297  # pivot to the button's outer face (button out)
GRIP = (0.170, 0.272, 0.0175)  # the grip: from, to (along the lever), radius
CAP = (0.272, 0.289, 0.0125)  # the grip's chrome end cap round the button: from, to, radius
BAR = (-0.030, 0.170, 0.012, 0.026)  # the steel bar: from, to, thickness (x), depth (z)
BTN_R, BTN_OUT, BTN_TRAVEL, BTN_RELEASE = 0.0075, 0.008, 0.008, 0.005
BTN_K = 1000.0  # N/m: the button's return spring (8 N fully in)
JAM_PLAY = 0.001  # how far the button goes in while the pawl is loaded
PITCH = 4.0  # deg between ratchet teeth
N_TEETH = 8  # teeth 0..8: 0 .. 32 deg
START_TOOTH = 4
LOAD_TOL = 0.6  # deg: the lever rests on its pawl when within this of it
K_CABLE = float(_E("HB_KCABLE", "1.3"))  # N m/rad
SLACK = -math.degrees(0.24 / K_CABLE)  # deg: where the cable spring would be slack (0.24 N m at the bottom stop)
K_PAWL = 1500.0  # N m/rad: the pawl's hold on its tooth (a stiff stop)
K_STOP = 30000.0  # N/m: the jammed pawl's hold on the button
LEVER_KG = 0.5
PAWL_DRUM = (0.028, 0.008, float(_E("HB_DRUM_KG", "3.0")))  # radius, length, mass of the pawl-stop drum (sim helper)
DOWN_TOL, ONE_TOOTH = 2.0, START_TOOTH * PITCH - 2.0  # deg
CLEAR = 0.030

STEEL, DARK, BLACK, CHROME, TRIM = ((0.55, 0.56, 0.58), (0.20, 0.20, 0.21), (0.06, 0.06, 0.06),
                                    (0.80, 0.81, 0.83), (0.13, 0.13, 0.14))
PIVOT = np.array([X0, PY, TOP + H_P])
ORIGIN = np.array([X0, 0.0, TOP])  # the linkage's origin (shapes are relative to it)


def _v(p) -> tuple:
    return tuple(float(x) for x in p)


def _rel(p) -> tuple:
    return _v(np.asarray(p, float) - ORIGIN)


def _qx(a: float) -> tuple:
    """Quaternion (x, y, z, w) of a turn ``a`` (rad) about +x."""
    return (math.sin(a / 2), 0.0, 0.0, math.cos(a / 2))


def _on_lever(r: float, dz: float = 0.0) -> np.ndarray:
    """A point on the lever lying level (as built), ``r`` along it from the pivot, ``dz`` above its line."""
    return PIVOT + np.array([0.0, r, dz])


def _console() -> list:
    sh = []
    y0, y1, slot = -0.225, 0.205, 0.030
    t = 0.012
    for s in (-1.0, 1.0):  # the trim either side of the slot
        sh.append(BoxShape(_rel((X0 + s * (slot + 0.030), (y0 + y1) / 2, TOP + t / 2)), (0.060, y1 - y0, t), TRIM))
    for a, b in ((y0, -0.185), (0.185, y1)):  # the slot's ends
        sh.append(BoxShape(_rel((X0, (a + b) / 2, TOP + t / 2)), (2 * slot, b - a, t), TRIM))
    sh.append(BoxShape(_rel((X0, -0.155, TOP + 0.003)), (2 * slot, 0.060, 0.006), DARK))  # the bracket's foot
    for s in (-1.0, 1.0):  # bracket plates either side of the pivot
        sh.append(BoxShape(_rel((X0 + s * 0.011, PY, TOP + (H_P + 0.012) / 2)), (0.004, 0.044, H_P + 0.012), STEEL))
    sh.append(CylShape(_rel(PIVOT), 0.006, 0.030, CHROME, axis="X"))  # the pivot pin
    # the ratchet sector beside the lever (drawn only): teeth every PITCH deg on an arc round the pivot
    for k in range(N_TEETH + 1):
        a = math.radians(k * PITCH)
        c = PIVOT + np.array([0.0, 0.040 * math.cos(a), 0.040 * math.sin(a)])
        sh.append(BoxShape(_rel(c + [0.016, 0.0, 0.0]), (0.003, 0.006, 0.004), DARK, collide=False,
                           quat_xyzw=_qx(a)))
    sh.append(BoxShape(_rel(PIVOT + [0.016, 0.026, 0.006]), (0.003, 0.030, 0.012), DARK, collide=False))
    # angle marks on the trim beside the grip's end: the bottom stop and each tooth, as a real trial's protractor
    return sh


def _lever() -> Linkage:
    # the stop the pawl gives: a drum round the pivot between the bracket plates (heavy, so the solver holds the lever
    # on it firmly; its weight sits on the pivot)
    pawl = [CylShape(_rel(PIVOT), PAWL_DRUM[0], PAWL_DRUM[1], DARK, axis="X")]
    a0, a1, th, dp = BAR
    lever = [BoxShape(_rel(_on_lever((a0 + a1) / 2)), (th, a1 - a0, dp), STEEL),
             CylShape(_rel(PIVOT), 0.015, th + 0.002, STEEL, axis="X"),
             CylShape(_rel(_on_lever((GRIP[0] + GRIP[1]) / 2)), GRIP[2], GRIP[1] - GRIP[0], BLACK, axis="Y"),
             CylShape(_rel(_on_lever((CAP[0] + CAP[1]) / 2)), CAP[2], CAP[1] - CAP[0], STEEL, axis="Y"),
             CylShape(_rel(_on_lever(CAP[1] - 0.0005)), BTN_R + 0.0015, 0.0012, BLACK, axis="Y", collide=False),
             BoxShape(_rel(_on_lever(0.025, -0.010)), (0.006, 0.040, 0.006), DARK, collide=False)]  # the pawl
    stop = [BoxShape(_rel(_on_lever(GRIP[1] - 0.020)), (0.006, 0.006, 0.006), BLACK)]  # inside the grip
    blen = BTN_OUT + 0.008
    button = [CylShape(_rel(_on_lever(L_END - blen / 2)), BTN_R, blen, CHROME, axis="Y")]
    piv = _v(PIVOT)
    btn_axis = _v(_on_lever(L_END))
    links = [Link("pawl", tuple(pawl), mass=PAWL_DRUM[2], anchor=piv, axis="X", limits=(0.0, N_TEETH * PITCH),
                  stiffness=K_PAWL, spring_damping=8.0, init=START_TOOTH * PITCH),
             Link("lever", tuple(lever), mass=LEVER_KG, anchor=piv, axis="X", parent="pawl",
                  limits=(0.0, N_TEETH * PITCH + 4.0), stiffness=K_CABLE, spring_damping=0.05, friction=0.02),
             Link("stop", tuple(stop), mass=0.05, anchor=btn_axis, axis="Y", parent="lever", slide=True,
                  limits=(-BTN_TRAVEL, 0.0), stiffness=K_STOP, spring_damping=80.0, init=-JAM_PLAY),
             Link("button", tuple(button), mass=0.02, anchor=btn_axis, axis="Y", parent="stop", slide=True,
                  limits=(0.0, BTN_TRAVEL + 0.001), stiffness=BTN_K, spring_damping=3.0, init=JAM_PLAY)]
    return Linkage.build("handbrake", pos=_v(ORIGIN), mass=4.0, color=TRIM, floating=False, friction=(0.8, 0.7),
                         shapes=_console(), links=links, contact_offset=0.002, solver_iterations=48)


HANDBRAKE = _lever()
OBJECTS = [HANDBRAKE]

# -- tracking and scoring --------------------------------------------------------------------------------------------
_DEBUG = _E("HB_DEBUG") == "1"
_DEBUG_EVERY = int(_E("HB_DEBUG_EVERY", "5"))
_EP: dict = {}
_IDS: dict = {}


@on_reset
def _reset() -> None:
    _EP.update(tick=0, step=0, theta=START_TOOTH * PITCH, phi=0.0, btn=0.0, tooth=START_TOOTH, engaged=True,
               loaded=True, stop_t=-JAM_PLAY, near=1.0, min_theta=START_TOOTH * PITCH, max_press=0.0, tcp={},
               gaps={})


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


def _lever_distance(pts: np.ndarray, theta_deg: float) -> float:
    """The least distance from points (world, n x 3) to the lever's surface (a rod of the grip's radius from the
    pivot to the button's end) at angle ``theta_deg``."""
    a = math.radians(theta_deg)
    u = np.array([0.0, math.cos(a), math.sin(a)])
    d = pts - PIVOT
    s = np.clip(d @ u, 0.0, L_END)
    return float(np.linalg.norm(d - s[:, None] * u, axis=1).min() - GRIP[2])


@on_step
def _track(env) -> None:
    try:
        _track_inner(env)
    except Exception:
        import traceback

        print("HB_TRACEBACK\n" + traceback.format_exc(), flush=True)
        raise


def _track_inner(env) -> None:
    import torch

    from yam_arena import ik, station
    from yam_arena.constants import ARM_JOINTS, FINGER_JOINTS

    _EP["tick"] += 1
    base = getattr(env, "unwrapped", env)
    origin = as_numpy(base.scene.env_origins[0])
    if "fingers" not in _IDS:
        _IDS["fingers"] = _finger_points()
    fingers = []
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
        fingers += [_IDS["fingers"][n] @ rotation(poses[i][3:7]).T + poses[i][:3] - origin
                    for n, i in zip(("tip_left", "tip_right"), _IDS[side + "_tips"])]
    hb = base.scene["handbrake"]
    if "pawl" not in _IDS:
        jn = list(hb.joint_names)
        for k in ("pawl", "lever", "stop", "button"):
            _IDS[k] = jn.index(f"{k}_turn")
    q = as_numpy(hb.data.joint_pos)[0]
    q_pawl, phi = math.degrees(float(q[_IDS["pawl"]])), math.degrees(float(q[_IDS["lever"]]))
    s_stop, s_rel = float(q[_IDS["stop"]]), float(q[_IDS["button"]])
    theta = q_pawl + phi
    press = -(s_stop + s_rel)  # how far the button is in (m)
    engaged = press < BTN_RELEASE
    tooth = _EP["tooth"]
    if not engaged:
        tooth = 0  # the pawl lifted clear: only the bottom stop holds the lever
    else:
        if not _EP["engaged"]:  # the button let go: the pawl drops into the highest tooth below the lever
            tooth = int(max(0, min(N_TEETH, math.floor((theta + 0.3) / PITCH))))
        while tooth < N_TEETH and theta >= (tooth + 1) * PITCH + 0.3:  # lifted past the next tooth: it clicks in
            tooth += 1
    loaded = engaged and phi < LOAD_TOL and tooth > 0
    stop_t = _EP["stop_t"]
    if loaded and not _EP["loaded"]:
        stop_t = min(-JAM_PLAY, -press)  # the loaded pawl jams the button where it is (at most 1 mm in)
    elif not loaded:
        stop_t = -BTN_TRAVEL
    pawl_t = math.radians(tooth * PITCH)
    lever_t = math.radians(SLACK - q_pawl)  # the cable's spring acts between the lever and the console
    button_t = -stop_t  # the button's spring pulls it out to its rest (relative to the stop, which it rides on)
    dev = hb.device
    ids = [_IDS["pawl"], _IDS["lever"], _IDS["stop"], _IDS["button"]]
    hb.set_joint_position_target_index(target=torch.tensor([[pawl_t, lever_t, stop_t, button_t]], dtype=torch.float32,
                                                           device=dev), joint_ids=ids)
    hb.write_data_to_sim()
    near = min(_lever_distance(fp, theta) for fp in fingers)
    if _DEBUG:  # the left fingers in the lever's frame: along it (r), across it up (n), sideways (x)
        a_ = math.radians(theta)
        u_, n_ = np.array([0.0, math.cos(a_), math.sin(a_)]), np.array([0.0, -math.sin(a_), math.cos(a_)])
        lp = np.concatenate(fingers[:2]) - PIVOT
        r_, nn_, xx_ = lp @ u_, lp @ n_, lp[:, 0]
        m_ = (np.abs(nn_) < 0.03) & (np.abs(xx_) < 0.02)
        _EP["lf"] = ((float(r_[m_].min()), float(nn_[m_][np.argmin(r_[m_])]), float(nn_[m_].min()),
                      float(nn_[m_].max())) if m_.any() else None)
    _EP.update(q_pawl=q_pawl, tgt=(math.degrees(pawl_t), math.degrees(lever_t), stop_t, button_t),
               theta=theta, phi=phi, btn=press, tooth=tooth, engaged=engaged, loaded=loaded, stop_t=stop_t,
               near=near, min_theta=min(_EP["min_theta"], theta), max_press=max(_EP["max_press"], press))


def _tally(o) -> dict:
    down = _EP["theta"] <= DOWN_TOL
    return dict(down=down, done=bool(down and _EP["near"] >= CLEAR), lowered=_EP["theta"] <= ONE_TOOTH)


def _debug(t) -> None:
    _EP["step"] += 1
    if (_EP["step"] - 1) % _DEBUG_EVERY:
        return
    print(f"DBG step={_EP['step']} theta={_EP['theta']:.2f}deg phi={_EP['phi']:.2f} tooth={_EP['tooth']} "
          f"pawl={_EP.get('q_pawl', 0):.2f} tgt={tuple(round(v, 4) for v in _EP.get('tgt', ()))} "
          f"btn={_EP['btn'] * 1000:.1f}mm engaged={int(_EP['engaged'])} loaded={int(_EP['loaded'])} "
          f"stop={_EP['stop_t'] * 1000:.1f}mm near={_EP['near'] * 1000:.0f}mm min={_EP['min_theta']:.1f} "
          f"maxpress={_EP['max_press'] * 1000:.1f} "
          f"lf={tuple(round(v * 1000, 1) for v in _EP['lf']) if _EP.get('lf') else None} done={int(t['done'])} | "
          + " ".join(f"{k} tcp=({v[0]:.3f},{v[1]:.3f},{(v[2] - TOP) * 1000:.1f}mm) gap={_EP['gaps'][k] * 1000:.0f}"
                     for k, v in _EP["tcp"].items()), flush=True)


def score(objects) -> float:
    t = _tally(objects)
    if _DEBUG:
        _debug(t)
    if t["done"]:
        return 1.0
    if t["down"]:
        return 0.75
    if t["lowered"]:
        return 0.5
    return 0.0


def success(objects) -> bool:
    return bool(_tally(objects)["done"])


# -- scripted demo ---------------------------------------------------------------------------------------------------
# Open loop, from the known layout. The right gripper, fingers closing along x and leaning 30 deg (its fingertips
# toward +y, its wrist away from the left arm), closes on the steel bar 15.5 cm from the pivot and raises the lever
# about 4 deg, off its pawl (aimed 3 cm high: the arm gives under the lever's load). The left gripper, fingers closed,
# turned so their outer face is square to the lever's axis and leaning 35 deg toward the robot (its wrist clear of the
# right arm's), has come down beside the lever's end; it pushes the button in along the axis (aimed 4 cm past its full
# travel: the arm gives under load). Then both move together over 5 s: the right lowers the lever to its bottom stop
# while the left follows the button's end round its arc, keeping it pressed. At the bottom the left lets the button
# out and backs off, the right lets go and rises, and both go back to their ready poses.
# HB_NOBUTTON=1 (the wrong way): the left arm stays at its ready pose and the right raises the lever and pushes it down
# without the button: the pawl holds it on its tooth. HB_NOLIFT=1: the button is pushed without first raising the
# lever: it jams, and the lever stays on its tooth when pushed down.
OPEN_C = -0.047
NOBUTTON = _E("HB_NOBUTTON") == "1"
NOLIFT = _E("HB_NOLIFT") == "1"
P = {k: float(_E("HB_" + k.upper(), v)) for k, v in dict(
    r_hold="0.155", lift="3.0", sag="0.030", press_at="0.0", press_over="0.040", lead="1.5", lower_s="5.0",
    bottom_over="3.0", grip_gap="0.004", finger="0.0064").items()}
T0 = START_TOOTH * PITCH
TCP_UP = float(_E("HB_TCPUP", "0.0026"))  # the left TCP this far back from the button's axis (away from the
# fingertips): the fingertips' outer face (26 mm long) then covers the button, and the finger's thicker part behind
# it clears the end cap
ALPHA = float(_E("HB_ALPHA", "35"))  # deg: the left gripper leans its fingertips toward +x (its wrist toward the
# robot), keeping its wrist clear of the right arm's


def _R(c1, c3) -> np.ndarray:
    c1, c3 = np.asarray(c1, float), np.asarray(c3, float)
    c1, c3 = c1 / np.linalg.norm(c1), c3 / np.linalg.norm(c3)
    return np.column_stack([c1, np.cross(c3, c1), c3])


def _press_rot(theta_deg: float) -> np.ndarray:
    """Left gripper: pointing down, fingers closing along the lever's axis; tilted with the lever so the closed
    fingers' outer (-y) face is square to the axis."""
    a, b = math.radians(theta_deg), math.radians(ALPHA)
    down = np.array([0.0, math.sin(a), -math.cos(a)])  # square to the lever, down
    return _R((0.0, -math.cos(a), -math.sin(a)), math.cos(b) * down + math.sin(b) * np.array([1.0, 0.0, 0.0]))


def _hold_rot(lean_deg: float) -> np.ndarray:
    """Right gripper: fingers closing along x, pointing down and leaning ``lean_deg`` toward +y."""
    b = math.radians(lean_deg)
    return _R((1.0, 0.0, 0.0), (0.0, math.sin(b), -math.cos(b)))


PRESS_16 = _press_rot(16.0)
PRESS_0 = _press_rot(0.0)
HOLD_20 = _hold_rot(float(_E("HB_LEAN", "30")))
DOWN_X = grip_rotation(math.pi / 2)


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


def _u(theta_deg: float) -> np.ndarray:
    a = math.radians(theta_deg)
    return np.array([0.0, math.cos(a), math.sin(a)])


def _hold_point(theta_deg: float) -> np.ndarray:
    """The right TCP on the bar ``r_hold`` from the pivot, at lever angle ``theta_deg``."""
    return PIVOT + P["r_hold"] * _u(theta_deg)


def _press_point(theta_deg: float, depth: float) -> np.ndarray:
    """The left TCP with its fingers' outer face ``depth`` in from the button's outer face (button out)."""
    c3 = _press_rot(theta_deg)[:, 2]
    return PIVOT + (L_END - depth + P["finger"]) * _u(theta_deg) - TCP_UP * c3


def _timeline():
    """Both arms' waypoints, built step by step so the arms move together."""
    L, R = [], []
    t_up = T0 + P["lift"]
    # lead in: the right above its hold point, fingers open; the left beside the lever's end, fingers closed
    R += _lead_in("right", _hold_point(T0) + [0.0, 0.0, 0.06], HOLD_20, OPEN_C, "above the bar, fingers open")
    if NOBUTTON:
        lp, lq = _ready("left")
        L += [_w(_dur(R), lp, OPEN_C, "stay at the ready pose", lq)]
    else:
        L += _lead_in("left", _press_point(T0, -0.030) + [0.0, 0.0, 0.04], PRESS_16, 0.0,
                      "beside the lever's end, fingers closed")
    # the right comes down onto the bar and closes on it
    R += [_w(2.0, _hold_point(T0), OPEN_C, "fingers either side of the bar", HOLD_20),
          _w(1.2, _hold_point(T0), -P["grip_gap"], "close on the bar", HOLD_20),
          _w(0.5, _hold_point(T0), -P["grip_gap"], "settle", HOLD_20)]
    if not NOBUTTON:
        L += [_w(2.0, _press_point(T0, -0.030), 0.0, "down level with the button", PRESS_16),
              _w(1.7, _press_point(T0, -0.003), 0.0, "up to the button", PRESS_16)]
    if NOLIFT:  # the wrong way: push the button with the lever still on its pawl, then push the lever down
        L += [_w(1.5, _press_point(T0, BTN_TRAVEL + P["press_over"]), 0.0, "push the button (jammed)", PRESS_16),
              _w(1.0, _press_point(T0, BTN_TRAVEL + P["press_over"]), 0.0, "keep pushing", PRESS_16)]
        R += [_w(2.5, _hold_point(T0), -P["grip_gap"], "wait", HOLD_20)]
    else:
        # raise the lever a little off its pawl
        steps = 8
        for k in range(1, steps + 1):
            th = T0 + P["lift"] * k / steps
            R.append(_w(1.5 / steps, _hold_point(th) + [0.0, 0.0, P["sag"] * k / steps], -P["grip_gap"],
                        "raise the lever slightly", HOLD_20))
            if not NOBUTTON:
                L.append(_w(1.5 / steps, _press_point(th, -0.003), 0.0, "follow the button up", _press_rot(th)))
        up = [0.0, 0.0, P["sag"]]  # the arm gives this much under the lever's load
        t_pr = t_up + P["press_at"]  # where the lever is expected to be (deg)
        R += [_w(0.4, _hold_point(t_up) + up, -P["grip_gap"], "hold it up", HOLD_20)]
        if not NOBUTTON:
            L += [_w(0.4, _press_point(t_pr, -0.003), 0.0, "at the button", _press_rot(t_pr)),
                  _w(1.5, _press_point(t_pr, BTN_TRAVEL + P["press_over"]), 0.0, "press the button in",
                      _press_rot(t_pr)),
                  _w(0.5, _press_point(t_pr, BTN_TRAVEL + P["press_over"]), 0.0, "keep it in", _press_rot(t_pr))]
            R += [_w(2.0, _hold_point(t_up) + up, -P["grip_gap"], "hold it up", HOLD_20)]
    # lower the lever to its bottom stop; the left keeps the button pressed along the axis
    t_lo = T0 + (0.0 if NOLIFT else P["lift"])
    steps = 16
    for k in range(1, steps + 1):
        th = t_lo + (-P["bottom_over"] - t_lo) * k / steps
        sag = 0.0 if NOLIFT else P["sag"] * max(0.0, 1.0 - 1.25 * k / steps)
        R.append(_w(P["lower_s"] / steps, _hold_point(th) + [0.0, 0.0, sag], -P["grip_gap"], "lower the lever",
                    HOLD_20))
        if not NOBUTTON:
            thl = max(t_lo + P["press_at"] + (0.0 - t_lo - P["press_at"]) * k / steps - P["lead"], 0.0)
            L.append(_w(P["lower_s"] / steps, _press_point(thl, BTN_TRAVEL + P["press_over"]), 0.0,
                        "keep the button in, round the arc", _press_rot(thl)))
    R += [_w(1.0, _hold_point(-P["bottom_over"]), -P["grip_gap"], "on the bottom stop", HOLD_20)]
    if not NOBUTTON:
        L += [_w(1.5, _press_point(0.0, BTN_TRAVEL + P["press_over"]), 0.0, "on the bottom stop", PRESS_0),
              _w(1.0, _press_point(0.0, -0.030), 0.0, "let the button out, back off", PRESS_0),
              _w(1.0, _press_point(0.0, -0.030) + [0.0, 0.0, 0.08], 0.0, "rise", PRESS_0)]
        R += [_w(2.0, _hold_point(-P["bottom_over"]), -P["grip_gap"], "wait", HOLD_20)]
    R += [_w(1.0, _hold_point(0.0) + [0.0, 0.0, 0.002], OPEN_C, "let go", HOLD_20),
          _w(1.5, _hold_point(0.0) + [0.0, 0.0, 0.08], OPEN_C, "rise", HOLD_20)]
    R += _lead_out("right", _hold_point(0.0) + [0.0, 0.0, 0.08], HOLD_20, OPEN_C, "back to the ready pose")
    if not NOBUTTON:
        L += _lead_out("left", _press_point(0.0, -0.030) + [0.0, 0.0, 0.08], PRESS_0, OPEN_C,
                       "back to the ready pose")
    return L, R


def demo():
    left, right = _timeline()
    lp, lq = _ready("left")
    rp, rq = _ready("right")
    end = max(_dur(left), _dur(right)) + 3.0
    left += [_w(end - _dur(left), lp, OPEN_C, "stay clear", lq)]
    right += [_w(end - _dur(right), rp, OPEN_C, "stay clear", rq)]
    return {"left": left, "right": right}
