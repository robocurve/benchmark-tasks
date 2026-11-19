"""On the station table: an adjustable single-channel 100-1000 uL air-displacement pipette (like Rainin Pipet-Lite
XLS+ L-1000XLS+) hangs upright in a single-pipette stand, its volume locked at 200 uL. Task (instruction): set it to
250 uL as its manual says (dial down onto 250 from at least a third of a turn above), lock the volume again and hang
it back in its stand.

How it is done (Rainin Pipet-Lite XLS+ operating instructions, 1.2 Setting Volume): "Turn the volume lock lever to the
'unlock' position so the volume setting mechanism is unlocked and free to turn"; "rotate the plunger button to change
volume - counter-clockwise to increase, and clockwise to decrease volume"; "To eliminate errors due to mechanical
backlash: when setting the desired volume, first turn the knob 1/3 turn above the desired volume. Then turn the knob
slowly clockwise until the desired volume is displayed. Always dial down to the desired volume"; "Turn the volume lock
to the 'lock' position ... to prevent changes to the volume setting". Gilson (PIPETMAN L) and Eppendorf (Research
plus) give the same approach-from-above rule.

The hand skill: the plunger button is a 14 mm knob on the end of a 25 cm, 0.1 kg pipette that rests loose in its
stand. Turned on its own, the whole pipette turns with it (it rests on its shoulder in the stand, or lies loose in an
open hand), so one hand must hold the handle while the other turns the knob; a wrist turns only about 240 degrees, so
the knob is turned in strokes, the fingers opening and turning back between them; the reading must stop on one
2 uL mark (3.6 degrees of the knob either side), coming down onto it after passing a third of a turn above.

World frame (m): x forward from the arm bases, y to the robot's left, z up; the table top is at z 0.7475.
- stand: fixed. A base plate 12 x 10 x 0.8 cm with its middle at (0.30, 0.00); a post 1.6 x 3 cm up to 13.5 cm; an
  arm 6 mm thick at the top (13.5 cm above the table) ending in a U-slot 23 mm wide, open toward the robot (-x), its
  centre (the round end's centre) at (0.25, 0.00), its open end at x 0.236.
- pipette: loose, hanging in the slot with its handle's shoulder resting on the arm. Pipette axis vertical through
  (0.25, 0.00). From the shoulder (z 0.135 above the table): tip cone (7 mm) and ejector sleeve (13 mm) 125 mm
  down; lower taper 16 mm across, 30 mm; handle 28 mm across, 100 mm up, a finger hook on its far (+x) side, the
  display window on its near (-x) side; the volume-lock collar (22 mm across, 8 mm tall, with a 7 mm lever tab) on top
  of the handle; the plunger button (14 mm across, 17 mm tall) on top of the collar. 0.097 kg in all.
  Collar: turns 0 to 40 deg about the pipette's axis (0 locked, 40 unlocked; 0.02 N m friction); the lock is open
  above 30 deg. Plunger button: turns about the axis; counter-clockwise seen from the button's end raises the volume,
  100 uL per turn. Unlocked, a weak spring pulls it to its nearest 2 uL step (a detent, at most 0.013 N m); locked, a
  stiff spring (20 N m/rad) holds it at the step where it was locked.
- display: reads the volume to the nearest 2 uL increment (a click every 7.2 deg of the button).

Score (0 for the rest of the run once the pipette is dropped: it reaches the table or the floor faster than 0.3 m/s,
or with no finger within 1 cm of it):
- 0.1: the lock has been released (collar past 30 deg) at some time.
- 0.3: the display reads 250 uL (volume 249.0 to 250.99 uL).
- 0.5: it reads 250 uL, set from above: since the reading was last below 250 it has been at 284 uL or more (a third of
  a turn above).
- 0.75: as 0.5, with the volume locked (collar at 10 deg or less).
- 1.0 (success, held 1 s): as 0.75, the pipette hanging in its stand (axis within 10 deg of vertical and in the slot:
  within 3 mm of its centre line, from 9 mm toward the open end to 4 mm toward the round end of the round end's
  centre; the shoulder within 3 mm of the arm's top), every finger point 1 cm or more from it, and it still (under
  1 mm in 1 s).
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
from bodykit import as_numpy, on_reset, on_step, rotation  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

allow_matrix_grips()

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"
MM = 0.001
_E = os.environ.get

# -- the layout ----------------------------------------------------------------------------------------------------
SLOT = np.array([0.25, 0.0])  # the stand's slot centre (x, y)
ARM_TOP = 0.135  # the stand arm's top above the table
ARM_T = 0.006
SLOT_W = 0.023
PRONG_W = 0.010
POST_X = 0.315

# the pipette, in its own frame: origin on its axis at the handle's shoulder, z toward the plunger button
HANDLE_R, HANDLE_L = 0.014, 0.100
TAPER_R, TAPER_L = 0.008, 0.030
SLEEVE_R, SLEEVE_L = 0.0065, 0.085
CONE_R, CONE_L = 0.0035, 0.010
COLLAR_R, COLLAR_Z0, COLLAR_L = 0.011, HANDLE_L, 0.008
KNOB_R, KNOB_Z0, KNOB_L = 0.007, HANDLE_L + COLLAR_L + 0.001, 0.017
PIP_LOW = -(TAPER_L + SLEEVE_L + CONE_L)  # the tip cone's end
PIP_TOP = KNOB_Z0 + KNOB_L

V0 = 200.0  # uL at the start
UL_PER_TURN = 100.0
INCR = 2.0  # uL per click
TARGET = 250.0
ABOVE = 284.0  # a third of a turn above 250 (rounded up to a click)
V_MIN, V_MAX = 0.0, 1000.0
COLLAR_MAX = 40.0  # deg: unlocked end stop
LOCKED_DEG, UNLOCKED_DEG = 10.0, 30.0  # scored as locked at 10 deg or less; the lock opens above 30 deg
# The button's feel: while unlocked a weak spring toward the nearest 2 uL step (a detent: at most 0.013 N m, half a
# step away); while locked a stiff spring holds it at its step. The task file moves the spring's target every step.
K_DETENT = float(_E("PIPV_K_DETENT", "0.2"))  # N m/rad
K_LOCK = float(_E("PIPV_K_LOCK", "20.0"))  # N m/rad
D_BUTTON = float(_E("PIPV_D_BUTTON", "0.003"))  # N m s/rad
STEP_RAD = 2 * math.pi * INCR / UL_PER_TURN  # one 2 uL step of the button (7.2 deg)
COLLAR_FRIC = float(_E("PIPV_COLLAR_FRIC", "0.02"))

WHITE, GREY, DARK, BLUE, STEEL, RED = ((0.93, 0.93, 0.9), (0.62, 0.64, 0.66), (0.12, 0.12, 0.14), (0.15, 0.35, 0.75),
                                       (0.72, 0.74, 0.78), (0.8, 0.15, 0.12))


def _v(p) -> tuple:
    return tuple(float(x) for x in p)


def _pipette() -> Linkage:
    base = [CylShape((0.0, 0.0, PIP_LOW + CONE_L / 2), CONE_R, CONE_L, GREY),
            CylShape((0.0, 0.0, -TAPER_L - SLEEVE_L / 2), SLEEVE_R, SLEEVE_L, WHITE),
            CylShape((0.0, 0.0, -TAPER_L / 2), TAPER_R, TAPER_L, GREY),
            CylShape((0.0, 0.0, HANDLE_L / 2), HANDLE_R, HANDLE_L, GREY),
            BoxShape((HANDLE_R + 0.005, 0.0, 0.088), (0.012, 0.012, 0.010), GREY),  # finger hook (far side)
            # the display window on the near side (visual)
            BoxShape((-HANDLE_R - 0.0002, 0.0, 0.065), (0.0006, 0.009, 0.028), DARK, collide=False)]
    cz = COLLAR_Z0 + COLLAR_L / 2
    collar = [CylShape((0.0, 0.0, cz), COLLAR_R, COLLAR_L, DARK),
              BoxShape((-(COLLAR_R + 0.0035), 0.0, cz), (0.007, 0.005, COLLAR_L), DARK)]  # the lever tab
    kz = KNOB_Z0 + KNOB_L / 2
    knob = [CylShape((0.0, 0.0, kz), KNOB_R, KNOB_L, BLUE),
            # a white line down the button's side, so its turning shows
            BoxShape((KNOB_R + 0.0002, 0.0, kz), (0.0006, 0.0025, KNOB_L - 0.002), WHITE, collide=False)]
    o = np.array([SLOT[0], SLOT[1], TOP + ARM_TOP + 0.0005])
    links = [Link("lock", tuple(collar), mass=0.004, anchor=_v(o), axis="Z", friction=COLLAR_FRIC, damping=0.0005,
                  limits=(0.0, COLLAR_MAX)),
             Link("button", tuple(knob), mass=0.008, anchor=_v(o), axis="Z", stiffness=K_LOCK,
                  spring_damping=D_BUTTON)]
    return Linkage.build("pipette", pos=_v(o), mass=0.085, color=GREY, floating=True, friction=(0.6, 0.5),
                         shapes=base, links=links, contact_offset=0.001, solver_iterations=48)


def _stand() -> Body:
    zt = TOP + ARM_TOP
    sx, sy = SLOT
    h = SLOT_W / 2
    x_open = sx - 0.014  # the prongs' ends (toward the robot)
    x_bridge = sx + h
    pz = zt - ARM_T / 2
    sh = [BoxShape((0.30, 0.0, TOP + 0.004), (0.12, 0.10, 0.008), DARK),
          BoxShape((POST_X, 0.0, TOP + 0.008 + (ARM_TOP - 0.008) / 2), (0.016, 0.03, ARM_TOP - 0.008), GREY),
          # the arm: two prongs either side of the slot and a bridge closing it at the far end
          BoxShape(((x_open + POST_X) / 2, sy + h + PRONG_W / 2, pz), (POST_X - x_open, PRONG_W, ARM_T), GREY),
          BoxShape(((x_open + POST_X) / 2, sy - h - PRONG_W / 2, pz), (POST_X - x_open, PRONG_W, ARM_T), GREY),
          BoxShape(((x_bridge + POST_X) / 2, sy, pz), (POST_X - x_bridge, SLOT_W, ARM_T), GREY)]
    return Body.build("stand", pos=(0.30, 0.0, TOP + 0.004), mass=1.0, color=GREY, fixed=True, friction=(0.5, 0.4),
                      shapes=[BoxShape((s.centre[0] - 0.30, s.centre[1], s.centre[2] - TOP - 0.004), s.size, s.color)
                              for s in sh])


PIPETTE = _pipette()
STAND = _stand()
OBJECTS = [STAND, PIPETTE]

# -- tracking and scoring -------------------------------------------------------------------------------------------
DROP_V = 0.3  # m/s: reaching the table faster than this, or with no finger within 1 cm of it, is a drop
CLEAR = 0.010
UPRIGHT_DEG = 10.0
SEAT_Y, SEAT_X, SEAT_Z = 0.003, (-0.009, 0.004), 0.003  # the axis in the slot (from its centre), shoulder on the arm
STILL_STEPS, STILL_MM = 20, 1.0
_DEBUG = _E("PIPV_DEBUG") == "1"
_DEBUG_EVERY = int(_E("PIPV_DEBUG_EVERY", "5"))
_EP: dict = {}
_IDS: dict = {}

# points along the pipette (its frame) for the finger-clearance and lowest-point checks
_ZS = np.linspace(PIP_LOW, PIP_TOP, 26)
_RAD = np.interp(_ZS, [PIP_LOW, -TAPER_L - SLEEVE_L, -TAPER_L, 0.0, HANDLE_L, PIP_TOP],
                 [CONE_R, SLEEVE_R, TAPER_R, HANDLE_R, COLLAR_R, KNOB_R])
SAMPLES = np.array([[r * math.cos(a), r * math.sin(a), z] for z, r in zip(_ZS, _RAD)
                    for a in np.linspace(0, 2 * math.pi, 8, endpoint=False)])


@on_reset
def _reset() -> None:
    _EP.update(touching=False, lock_at=None, tick=0, step=0, counted=-1, dropped="", released=False, above=False, locked=True, recent=[], vol=V0,
               collar=0.0, knob=0.0, near=1.0, low=1.0, prev_low=None, vz=0.0, tcp={}, gaps={}, fric=None, peak=V0,
               upright=0.0, seat=((0.0, 0.0), 0.0), pos=np.zeros(3))


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


def _set_button(pip, unlocked: bool, q: float) -> None:
    """The volume lock and the detent: locked, a stiff spring holds the button at the step where it was locked;
    unlocked, a weak spring pulls it toward its nearest 2 uL step."""
    import torch

    k = K_DETENT if unlocked else K_LOCK
    if unlocked or _EP.get("lock_at") is None:
        _EP["lock_at"] = STEP_RAD * math.floor(q / STEP_RAD + 0.5)
    target = _EP["lock_at"]
    dev = pip.device
    if _EP["fric"] != k:
        _EP["fric"] = k
        pip.write_joint_stiffness_to_sim_index(stiffness=torch.tensor([[k]], dtype=torch.float32, device=dev),
                                               joint_ids=[_IDS["button"]])
    pip.set_joint_position_target_index(target=torch.tensor([[target]], dtype=torch.float32, device=dev),
                                        joint_ids=[_IDS["button"]])
    pip.write_data_to_sim()


@on_step
def _track(env) -> None:
    try:
        _track_inner(env)
    except Exception:
        import traceback

        print("PIPV_TRACEBACK\n" + traceback.format_exc(), flush=True)
        raise


def _track_inner(env) -> None:
    from scipy.spatial import cKDTree

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
        _EP.setdefault("wrist", {})[side] = math.degrees(float(q[_IDS[side][5]]))
        poses = as_numpy(robot.data.body_link_pose_w)[0]
        fingers[side] = np.concatenate([_IDS["fingers"][n] @ rotation(poses[i][3:7]).T + poses[i][:3] - origin
                                        for n, i in zip(("tip_left", "tip_right"), _IDS[side + "_tips"])])
    pip = base.scene["pipette"]
    if "button" not in _IDS:
        jn = list(pip.joint_names)
        _IDS["button"] = jn.index("button_turn")
        _IDS["lock"] = jn.index("lock_turn")
    q = as_numpy(pip.data.joint_pos)[0]
    collar = math.degrees(float(q[_IDS["lock"]]))
    knob = float(q[_IDS["button"]])
    vol = V0 + UL_PER_TURN * knob / (2 * math.pi)
    pose = as_numpy(pip.data.root_link_pose_w)[0]
    pp, pr = pose[:3] - origin, rotation(pose[3:7])
    pts = pp + SAMPLES @ pr.T
    low = float(pts[:, 2].min())
    tree = cKDTree(pts)
    near = min(float(tree.query(fp, k=1)[0].min()) for fp in fingers.values())
    vz = 0.0 if _EP["prev_low"] is None else (low - _EP["prev_low"]) / 0.05
    _EP["prev_low"] = low
    # the lock: unlocked only with the lever past 30 deg; anything short of that keeps the button held
    unlocked = collar > UNLOCKED_DEG
    _set_button(pip, unlocked, knob)
    if unlocked:
        _EP["released"] = True
    touching = low < TOP + 0.002
    if not _EP["dropped"] and touching and not _EP["touching"] and (vz < -DROP_V or near > CLEAR):
        _EP["dropped"] = (f"the pipette reached the table at {-vz:.2f} m/s with the nearest finger "
                          f"{near * 1000:.0f} mm from it (dropped)")
    _EP["touching"] = touching
    if not _EP["dropped"] and low < TOP - 0.05:
        _EP["dropped"] = "the pipette fell off the table"
    reading = INCR * math.floor(vol / INCR + 0.5)
    if vol >= ABOVE:
        _EP["above"] = True
    if vol < TARGET - 1.0:
        _EP["above"] = False
    upright = math.degrees(math.acos(min(1.0, abs(float(pr[2, 2])))))
    seat_xy = (float(pp[0] - SLOT[0]), float(pp[1] - SLOT[1]))
    seat_z = float(pp[2] - (TOP + ARM_TOP))
    # the right fingers' closest radial distance to the axis within the button's and the collar's lengths (debug)
    rel = (fingers["right"] - pp) @ pr  # in the pipette's frame
    rr = np.hypot(rel[:, 0], rel[:, 1])
    kz = (rel[:, 2] > KNOB_Z0) & (rel[:, 2] < KNOB_Z0 + KNOB_L)
    cz = (rel[:, 2] > COLLAR_Z0) & (rel[:, 2] < COLLAR_Z0 + COLLAR_L)
    _EP["kn_r"] = float(rr[kz].min()) if kz.any() else 1.0
    _EP["co_r"] = float(rr[cz].min()) if cz.any() else 1.0
    _EP["fz"] = (float(rel[:, 2][rr < 0.02].min()) if (rr < 0.02).any() else 1.0)
    _EP["spin"] = math.degrees(math.atan2(float(pr[2, 0]), float(pr[0, 0])))
    if _DEBUG:
        bl = as_numpy(pip.data.body_link_pose_w)[0]
        names = list(pip.body_names)
        sp = []
        for i, nm in enumerate(names):
            rb = rotation(bl[i][3:7])
            sp.append(f"{nm}:{math.degrees(math.atan2(float(rb[2, 0]), float(rb[0, 0]))):.1f}")
        _EP["spins"] = " ".join(sp)
    _EP.update(collar=collar, knob=knob, vol=vol, reading=reading, locked=collar <= LOCKED_DEG, unlocked=unlocked,
               near=near, low=low, vz=vz, upright=upright, seat=(seat_xy, seat_z), pos=pp,
               peak=max(_EP["peak"], vol))


def _tally(o) -> dict:
    recent = _EP["recent"]
    if _EP["counted"] != _EP["tick"]:
        _EP["counted"] = _EP["tick"]
        recent.append(np.asarray(_EP["pos"], float).copy())
        del recent[:-STILL_STEPS]
    still = len(recent) == STILL_STEPS and all(np.abs(p - recent[-1]).max() * 1000 < STILL_MM for p in recent)
    on_250 = TARGET - INCR / 2 <= _EP["vol"] < TARGET + INCR / 2  # the display shows the nearest 2 uL step
    seat_xy, seat_z = _EP["seat"]
    in_stand = (_EP["upright"] <= UPRIGHT_DEG and SEAT_X[0] <= seat_xy[0] <= SEAT_X[1] and abs(seat_xy[1]) <= SEAT_Y
                and abs(seat_z) <= SEAT_Z)
    done = bool(on_250 and _EP["above"] and _EP["locked"] and in_stand and _EP["near"] >= CLEAR and still)
    return dict(still=still, on_250=on_250, in_stand=in_stand, done=done)


def _debug(t) -> None:
    _EP["step"] += 1
    if (_EP["step"] - 1) % _DEBUG_EVERY:
        return
    p = _EP["pos"]
    print(f"DBG step={_EP['step']} vol={_EP['vol']:.2f}uL read={_EP.get('reading', V0):.0f} peak={_EP['peak']:.1f} "
          f"knob={math.degrees(_EP['knob']):+.1f}deg collar={_EP['collar']:.1f}deg locked={int(_EP['locked'])} "
          f"rel={int(_EP['released'])} above={int(_EP['above'])} up={_EP['upright']:.1f} "
          f"kn_r={_EP.get('kn_r', 1) * 1000:.1f} co_r={_EP.get('co_r', 1) * 1000:.1f} fz={_EP.get('fz', 1) * 1000:.1f} "
          f"spin={_EP.get('spin', 0):.1f} [{_EP.get('spins', '')}] "
          f"seat=({_EP['seat'][0][0] * 1000:.1f},{_EP['seat'][0][1] * 1000:.1f},{_EP['seat'][1] * 1000:.1f})mm "
          f"near={_EP['near'] * 1000:.0f}mm "
          f"low={(_EP['low'] - TOP) * 1000:.0f}mm pos=({p[0]:.3f},{p[1]:.3f},{p[2] - TOP:.3f}) "
          f"still={int(t['still'])} in_stand={int(t['in_stand'])} drop={_EP['dropped'] or '-'} | "
          + " ".join(f"{k} tcp=({v[0]:.3f},{v[1]:.3f},{v[2] - TOP:.3f}) gap={_EP['gaps'][k] * 1000:.0f}"
                     + f" w={_EP['wrist'][k]:.0f}" for k, v in _EP["tcp"].items())
          + f" fric={_EP['fric']}", flush=True)


def score(objects) -> float:
    t = _tally(objects)
    if _DEBUG:
        _debug(t)
    if _EP["dropped"]:
        return 0.0
    if t["on_250"] and _EP["above"] and _EP["locked"]:
        return 1.0 if t["done"] else 0.75
    if t["on_250"] and _EP["above"]:
        return 0.5
    if t["on_250"]:
        return 0.3
    return 0.1 if _EP["released"] else 0.0


def success(objects) -> bool:
    t = _tally(objects)
    return bool(t["done"] and not _EP["dropped"])


# -- scripted demo --------------------------------------------------------------------------------------------------
# Open loop, from the known layout. The left gripper takes the handle level from the robot's side (fingers closing
# across it), lifts the pipette 4 cm out of the slot, draws it back 3.5 cm and turns its wrist a quarter turn, so the
# pipette lies level along y with its plunger button toward the right arm, and lowers it 2.5 cm. The right gripper,
# pointing along +y, takes the lock collar and turns it 45 deg (to its 40 deg stop: unlocked), opens and backs onto
# the plunger button. It turns the button in strokes, opening and turning back between them: 180 deg and 140 deg
# counter-clockwise (seen from the button's end: up to about 289 uL), then 140 deg clockwise (down to 250 uL). It takes
# the collar again, turns it back past its 0 stop (locked), lets go and leaves. The left gripper turns the pipette
# upright, carries it over the slot, lowers it onto the arm, lets go and backs away.
# PIPV_MODE=loose: once the pipette is level the left fingers ease off to touch the handle without squeezing it
# (it does not hold the handle) while the right turns the button; PIPV_MODE=nolock: the volume is set but not locked
# again.
OPEN_C = -0.047
MODE = _E("PIPV_MODE", "demo")
L_PINCH = float(_E("PIPV_LPINCH", "-0.004"))  # closed on the 28 mm handle (squeezing it hard)
L_LOOSE = float(_E("PIPV_LLOOSE", "-0.0143"))  # not squeezing: a 28.6 mm gap round the 28 mm handle
COLLAR_PINCH = float(_E("PIPV_CPINCH", "-0.0075"))  # on the 22 mm collar
KNOB_PINCH = float(_E("PIPV_KPINCH", "-0.0035"))  # on the 14 mm button
R_OPEN = -0.020  # open round the button / collar while turning back

GRIP_LOCAL = 0.045  # the left grip this far up the handle from the shoulder
L_X0 = SLOT[0]
L_Z0 = ARM_TOP + GRIP_LOCAL  # TCP height above the table while in the stand
LIFT = 0.040
BACK = float(_E("PIPV_BACK", "0.035"))
DROP = float(_E("PIPV_DROP", "0.025"))  # lowered this far once level
WORK = np.array([L_X0 - BACK, SLOT[1], TOP + L_Z0 + LIFT - DROP])  # the left TCP while the right works

STROKES = [float(v) for v in _E("PIPV_STROKES", "180,140,-140").split(",")]  # button turns, deg (+ raises)
UNLOCK_TURN = float(_E("PIPV_UNLOCK", "45"))
LOCK_TURN = float(_E("PIPV_LOCK", "50"))
TIP_AHEAD = 0.0125  # the fingertips reach this far beyond the TCP along the gripper (measured in the sim)
COLLAR_TCP = COLLAR_Z0 + 0.0005 + TIP_AHEAD  # along the pipette from the shoulder: fingertips 0.5 mm onto the collar
KNOB_TCP = KNOB_Z0 + 0.0015 + TIP_AHEAD  # fingertips just past the collar, the pads over the button
R_ROLL0 = float(_E("PIPV_R0", "180"))  # the right hand's roll at the start of each stroke


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


LEVEL_X_UPRIGHT = approach_rot((1, 0, 0), 90.0)  # left: pointing +x, fingers across a vertical pipette (along y)
LEVEL_X_FLAT = approach_rot((1, 0, 0), 180.0)  # left: after the quarter turn, fingers along z
SIDE_Y = approach_rot((0, 1, 0), 180.0)  # right: pointing +y, fingers along z


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


def _ready(arm: str):
    from yam_arena import ik, station
    from yam_arena.constants import READY

    return ik.tcp_pose(np.array(READY, float), station.BASES[arm])


def _dur(wps) -> float:
    return float(sum(w.duration for w in wps))


def _roll_l(a, b, p, finger, seconds, label, n=9) -> list:
    return [_w(seconds / n, p, finger, label, approach_rot((1, 0, 0), a + (b - a) * k / n)) for k in range(1, n + 1)]


def _left_take() -> list:
    p0 = np.array([L_X0, SLOT[1], TOP + L_Z0])
    out = _lead_in("left", p0 + np.array([-0.06, 0.08, 0.02]), LEVEL_X_UPRIGHT, OPEN_C, "to the robot's side of it")
    out += [_w(1.5, p0 + np.array([-0.06, 0.0, 0.0]), OPEN_C, "in front of the handle", LEVEL_X_UPRIGHT),
            _w(2.0, p0, OPEN_C, "fingers either side of the handle", LEVEL_X_UPRIGHT),
            _w(1.0, p0, L_PINCH, "hold the handle", LEVEL_X_UPRIGHT),
            _w(0.5, p0, L_PINCH, "settle", LEVEL_X_UPRIGHT),
            _w(2.0, p0 + np.array([0.0, 0.0, LIFT]), L_PINCH, "lift it out of the slot", LEVEL_X_UPRIGHT),
            _w(1.5, p0 + np.array([-BACK, 0.0, LIFT]), L_PINCH, "draw it back clear of the stand", LEVEL_X_UPRIGHT)]
    out += _roll_l(90.0, 180.0, p0 + np.array([-BACK, 0.0, LIFT]), L_PINCH, 3.0,
                   "turn it level, the button toward the right hand")
    hold = L_LOOSE if MODE == "loose" else L_PINCH
    out += [_w(1.5, WORK, L_PINCH, "lower it", LEVEL_X_FLAT),
            _w(1.0, WORK, hold, "hold it there" if hold == L_PINCH else "fingers on the handle without squeezing it", LEVEL_X_FLAT)]
    return out


def _along(local_z: float, dy: float = 0.0) -> np.ndarray:
    """The point on the level pipette's axis ``local_z`` from its shoulder (button toward -y), the right TCP there."""
    return WORK + np.array([0.0, -(local_z - GRIP_LOCAL) + dy, 0.0])


def _side(roll: float) -> np.ndarray:
    return approach_rot((0, 1, 0), roll)


def _turn(p, finger, r_a, r_b, seconds, label) -> list:
    n = max(4, int(abs(r_b - r_a) / 10.0))
    return [_w(seconds / n, p, finger, label, _side(r_a + (r_b - r_a) * k / n)) for k in range(1, n + 1)]


def _right_work() -> list:
    """The right hand: unlock, the strokes, lock. The pipette's +z (toward the button) is world -y here, and the
    right hand points +y, so turning the button counter-clockwise seen from its end (raising the volume) is a turn of
    the hand about -y: a negative roll."""
    r0 = R_ROLL0
    pc, pk = _along(COLLAR_TCP), _along(KNOB_TCP)
    out = [_w(2.5, _along(COLLAR_TCP, -0.06), OPEN_C, "in line with the pipette, open", _side(r0)),
           _w(2.0, pc, OPEN_C, "fingers either side of the lock collar", _side(r0)),
           _w(1.0, pc, COLLAR_PINCH, "take the collar", _side(r0))]
    out += _turn(pc, COLLAR_PINCH, r0, r0 - UNLOCK_TURN, 1.5, "turn the lock lever to unlock")
    out += [_w(0.8, pc, R_OPEN, "let go of the collar", _side(r0 - UNLOCK_TURN))]
    out += _turn(pc, R_OPEN, r0 - UNLOCK_TURN, r0, 1.0, "turn the open hand back")
    out += [_w(1.0, pk, R_OPEN, "back onto the plunger button", _side(r0))]
    for k, s in enumerate(STROKES):
        start = r0 if s > 0 else r0 - abs(s)
        if k:
            out += _turn(pk, R_OPEN, cur, start, max(1.0, abs(cur - start) / 90.0), "turn the open hand back")
        out += [_w(0.4, pk, R_OPEN, "settle", _side(start)),
                _w(1.0, pk, KNOB_PINCH, "take the button", _side(start)),
                _w(0.4, pk, KNOB_PINCH, "settle", _side(start))]
        end = start - s
        label = "turn the button up (counter-clockwise)" if s > 0 else "dial down onto 250 (clockwise)"
        out += _turn(pk, KNOB_PINCH, start, end, abs(s) / 50.0, label)
        out += [_w(0.6, pk, KNOB_PINCH, "settle", _side(end)),
                _w(0.8, pk, R_OPEN, "let go of the button", _side(end))]
        cur = end
    if MODE != "nolock":
        ra = r0 - UNLOCK_TURN
        out += _turn(pk, R_OPEN, cur, ra, max(1.0, abs(cur - ra) / 90.0), "turn the open hand")
        out += [_w(1.0, pc, R_OPEN, "onto the lock collar", _side(ra)),
                _w(1.0, pc, COLLAR_PINCH, "take the collar", _side(ra))]
        out += _turn(pc, COLLAR_PINCH, ra, ra + LOCK_TURN, 1.5, "turn the lock lever back to lock")
        out += [_w(0.8, pc, R_OPEN, "let go", _side(ra + LOCK_TURN))]
        cur = ra + LOCK_TURN
    out += _turn(pk if MODE == "nolock" else pc, R_OPEN, cur, 170.0, 0.8, "turn the open hand")
    p_out = _along(KNOB_TCP, -0.035)
    out += [_w(1.5, p_out, OPEN_C, "back away along the pipette", _side(170.0))]
    out += _lead_out("right", p_out, _side(170.0), OPEN_C, "back to the ready pose")
    return out


def _lead_out(arm: str, pos, rot, finger: float, label: str, seconds: float = 3.0, steps: int = 15) -> list:
    """From ``pos``/``rot`` back to the ready pose, moving in joint space (no awkward wrist turns on the way)."""
    from yam_arena import ik, station
    from yam_arena.constants import READY

    q0, q1 = _good_ik(arm, pos, rot), np.array(READY, float)
    out = []
    for k in range(1, steps + 1):
        p, r = ik.tcp_pose(q0 + (q1 - q0) * k / steps, station.BASES[arm])
        out.append(Waypoint(seconds / steps, tuple(float(v) for v in p), finger, label, r))
    return out


def _left_return() -> list:
    pl = WORK + np.array([0.0, 0.0, DROP])
    p_up = np.array([L_X0 - 0.003, SLOT[1], TOP + L_Z0 + LIFT])
    p_in = np.array([L_X0 - 0.003, SLOT[1], TOP + L_Z0 - 0.002])  # aimed 2 mm low: it settles on its shoulder
    out = [_w(1.5, pl, L_PINCH, "raise it", LEVEL_X_FLAT)]
    out += _roll_l(180.0, 90.0, pl, L_PINCH, 3.0, "turn it upright")
    out += [_w(2.0, p_up, L_PINCH, "carry it over the slot", LEVEL_X_UPRIGHT),
            _w(2.5, p_in, L_PINCH, "lower it into the slot onto its shoulder", LEVEL_X_UPRIGHT),
            _w(0.7, p_in, L_PINCH, "settle", LEVEL_X_UPRIGHT),
            _w(1.0, p_in, OPEN_C, "let go", LEVEL_X_UPRIGHT),
            _w(2.0, p_in + np.array([-0.07, 0.0, 0.0]), OPEN_C, "back away", LEVEL_X_UPRIGHT),
            _w(8.0, p_in + np.array([-0.08, 0.0, 0.03]), OPEN_C, "stay clear", LEVEL_X_UPRIGHT)]
    return out


def demo():
    left_a = _left_take()
    right = _right_work()
    ready_r, rot_r = _ready("right")
    t_wait = _dur(left_a) + 0.5
    right = [_w(t_wait, ready_r, OPEN_C, "wait at the ready pose", rot_r)] + right
    left = left_a + [_w(_dur(right) - _dur(left_a) - 3.0, WORK, L_PINCH if MODE != "loose" else L_LOOSE,
                        "hold it while the right hand works", LEVEL_X_FLAT)] + _left_return()
    right += [_w(max(5.0, _dur(left) - _dur(right)), ready_r, OPEN_C, "stay put", rot_r)]
    return {"left": left, "right": right}
