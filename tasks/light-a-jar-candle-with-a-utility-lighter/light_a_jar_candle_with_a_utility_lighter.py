"""On the station table: a burned-down glass jar candle (its wick 45 mm below the rim) and a long-reach utility lighter
lying on its side. Task (instruction): "Light the candle in the glass jar with the utility lighter, then put the
lighter down on the table." The instruction does not say how.

How a utility lighter is used (BIC Consumer Products, answer on the BIC Multi-Purpose Lighter's retail page): "hold
the utility lighter with your thumb resting on the latch and your index finger on the trigger" ... "depress the latch
with the tip of your thumb and hold in" ... "pull the trigger with your index finger" ... "The child-resistant
mechanism will automatically reset after each use." The trigger is blocked unless the latch is held in, and the gas
only flows while the trigger is pulled, so the latch and the trigger must be worked together, and the nozzle tip must
be brought down inside the jar to the wick without knocking the jar.

World frame (m): x forward from the arm bases, y to the robot's left, z up; the table top is at z TOP.
- jar candle (loose, 0.65 kg): clear glass jar 85 mm across, 100 mm tall (4 mm wall, 8 mm base), centred at
  (JX, JY); dark red wax inside up to 55 mm above the table; a cotton wick 2 mm thick standing 8 mm out of the wax in the
  middle (its tip 63 mm above the table, 37 mm below the rim).
- utility lighter (loose, 60 g in all), sized on the BIC Multi-Purpose Classic: a 105 mm handle (30 mm from latch side
  to trigger side, 24 mm thick), a 150 mm metal wand (10 mm across) from the handle's front end, 255 mm overall. It lies
  on its side at (LX, LY) with the wand pointing to +y and its latch side facing +x. The latch: a 14 x 12 mm button on
  the handle's top, 25 mm behind the handle's front end, standing 3 mm proud, pressed in 3 mm against its spring (4.5 N
  fully in). The trigger: a 24 x 10 mm bar standing 12 mm out of the handle's underside, right under the latch, pulled
  10 mm into the handle against its spring (12 N fully in). While the latch is in less than 2.5 mm the trigger stops
  after 2 mm (the child-resistant block).

The gas and flame rule (every 50 ms step):
- gas flows while the latch is in 2.5 mm or more and the trigger is in 8 mm or more (80 % of its travel); a flame
  25 mm long then stands on the nozzle tip, along the wand (the lighter's spark is not modelled separately);
- the wick catches once its tip has been within 6 mm of that flame (of the 25 mm line ahead of the tip) for 1.0 s in
  all, then the candle burns for the rest of the run.
Score (best stage reached):
- 0.5: the candle lit.
- 0.75: the candle lit, and the lighter put down: resting on the table (lowest point under 2 cm up, still), its gas off,
  every finger point 2 cm or more from it.
- 1.0 (success, held 1 s): as 0.75, and the jar upright (tilt under 10 deg) and within 2 cm of where it started.
- 0 for the rest of the run if the jar is ever tipped 30 deg or more (a lit candle knocked over).
Doing nothing, pulling the trigger without the latch held (blocked: no gas), or holding the flame outside the jar: 0.
"""

from __future__ import annotations

import itertools
import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

from bodykit import Body, BoxShape, CylShape, Link, Linkage, allow_matrix_grips, as_numpy, on_reset, on_step  # noqa
from bodykit import plate, rotation  # noqa: E402
from yam_arena.demo import Waypoint, grip_rotation  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

allow_matrix_grips()

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"


def _f(name: str, default: str) -> float:
    return float(os.environ.get(name, default))


# -- the layout ----------------------------------------------------------------------------------------------------
JX, JY = 0.240, 0.000  # the jar's axis
JAR_R, JAR_H, WALL, BASE_T = 0.0425, 0.100, 0.004, 0.008
WAX_TOP = 0.055  # the wax surface above the table
WICK_H, WICK_R = 0.008, 0.001
WICK_TIP = np.array([0.0, 0.0, WAX_TOP + WICK_H])  # in the jar frame (origin on the table, on its axis)
LX, LY = 0.150, -0.170  # the lighter's handle centre (lying on its side)
H_LEN, H_DEPTH, H_THICK = 0.105, 0.030, 0.024  # handle: length (y), latch side to trigger side (x), thickness (z)
WAND_LEN, WAND_R, WAND_X = 0.150, 0.005, 0.007  # the wand, its axis offset toward the latch side
TRIG_Y = 0.0525 - 0.022  # the trigger's centre (y, from the handle centre)
LATCH_Y, LATCH = TRIG_Y - 0.020, (0.004, 0.014, 0.012)  # latch centre (y, 20 mm behind the trigger); size (x, y, z)
LATCH_PROUD, LATCH_TRAVEL, LATCH_K = 0.003, 0.003, 1500.0
TRIG = (0.020, 0.024, 0.010)  # trigger centre (y); size: depth (x, 8 mm hidden), length, thickness
TRIG_PROUD, TRIG_TRAVEL, TRIG_K = 0.012, 0.010, 1200.0
TRIG_BLOCK = 0.002  # trigger travel allowed while the latch is not held
LATCH_HELD, TRIG_ON = 0.0025, 0.008
FLAME_LEN, CATCH_R, CATCH_S = 0.025, 0.006, 1.0
TIP_Y = 0.0525 + WAND_LEN  # the nozzle tip (y, from the handle centre)
LZ = TOP + H_THICK / 2 + 0.001

GLASS, WAX, WICK = (0.72, 0.80, 0.84), (0.55, 0.08, 0.12), (0.10, 0.09, 0.08)
RED, BLACK, STEEL, GREY = (0.80, 0.12, 0.10), (0.08, 0.08, 0.09), (0.70, 0.71, 0.72), (0.35, 0.36, 0.38)
FLAME_Y, FLAME_O, FLAME_B = (1.0, 0.85, 0.30), (1.0, 0.55, 0.10), (0.30, 0.45, 1.0)


def _v(p) -> tuple:
    return tuple(float(x) for x in p)


def _jar() -> Body:
    """The jar candle in the frame of its base's middle, on the table."""
    sh = [CylShape((0.0, 0.0, BASE_T / 2), JAR_R, BASE_T, GLASS)]
    n = 20
    ri = JAR_R - WALL / 2
    w = 2 * math.pi * JAR_R / n * 1.08
    for k in range(n):
        a = 2 * math.pi * k / n
        c, s = math.cos(a), math.sin(a)
        sh.append(plate((ri * c, ri * s, BASE_T), (ri * c, ri * s, JAR_H), w, WALL, (c, s, 0.0), GLASS))
    sh.append(CylShape((0.0, 0.0, (BASE_T + WAX_TOP) / 2), JAR_R - WALL - 0.0005, WAX_TOP - BASE_T, WAX))
    sh.append(CylShape((0.0, 0.0, WAX_TOP + WICK_H / 2), WICK_R, WICK_H, WICK, collide=False))
    sh.append(CylShape((0.0, 0.0, WAX_TOP + 0.0004), 0.010, 0.0008, (0.45, 0.05, 0.08), collide=False))  # wax pool
    return Body.build("jar", pos=(JX, JY, TOP + 0.001), mass=0.65, color=GLASS, friction=(0.7, 0.6),
                      solver_iterations=32, shapes=sh)


def _lighter() -> Linkage:
    """The lighter, lying on its side: shapes relative to the handle centre (world-aligned at the start)."""
    hx = H_DEPTH / 2
    base = [BoxShape((0.0, 0.0, 0.0), (H_DEPTH, H_LEN, H_THICK), RED),
            BoxShape((0.0, -H_LEN / 2 + 0.006, 0.0), (H_DEPTH + 0.002, 0.012, H_THICK + 0.002), BLACK),  # end cap
            CylShape((WAND_X, H_LEN / 2 + 0.006, 0.0), WAND_R + 0.002, 0.012, BLACK, axis="Y"),  # collar
            CylShape((WAND_X, H_LEN / 2 + (WAND_LEN - 0.015) / 2, 0.0), WAND_R, WAND_LEN - 0.015, GREY, axis="Y"),
            CylShape((WAND_X, TIP_Y - 0.0075, 0.0), WAND_R - 0.0005, 0.015, STEEL, axis="Y")]
    ly = LATCH_Y
    latch = [BoxShape((hx + LATCH_PROUD - LATCH[0] / 2, ly, 0.0), LATCH, BLACK)]
    trig = [BoxShape((-hx - TRIG_PROUD + TRIG[0] / 2, TRIG_Y, 0.0), TRIG, BLACK)]
    o = (LX, LY, LZ)
    links = [Link("latch", tuple(latch), mass=0.002, anchor=o, axis="X", slide=True, limits=(-LATCH_TRAVEL, 0.0),
                  stiffness=LATCH_K, spring_damping=2.0),
             Link("trigger", tuple(trig), mass=0.004, anchor=o, axis="X", slide=True, limits=(0.0, TRIG_TRAVEL),
                  stiffness=TRIG_K, spring_damping=2.0)]
    return Linkage.build("lighter", pos=o, mass=0.054, color=RED, shapes=base, links=links, floating=True,
                         friction=(0.9, 0.8), solver_iterations=32)


def _hide(i: int) -> np.ndarray:
    return np.array([0.30 + 0.12 * i, 0.0, TOP - 0.40])


def _candle_flame() -> Body:
    sh = [CylShape((0.0, 0.0, 0.006), 0.0055, 0.012, FLAME_O, collide=False),
          CylShape((0.0, 0.0, 0.016), 0.0040, 0.010, FLAME_Y, collide=False),
          CylShape((0.0, 0.0, 0.0235), 0.0020, 0.006, FLAME_Y, collide=False)]
    return Body.build("candle_flame", pos=_v(_hide(0)), mass=0.01, color=FLAME_Y, gravity=False, damping=(5.0, 5.0),
                      shapes=sh)


def _lighter_flame() -> Body:
    """In the lighter's frame (the handle centre): a blue-cored flame along +y from the nozzle tip."""
    sh = [CylShape((WAND_X, TIP_Y + 0.005, 0.0), 0.0035, 0.010, FLAME_B, axis="Y", collide=False),
          CylShape((WAND_X, TIP_Y + 0.015, 0.0), 0.0045, 0.012, FLAME_O, axis="Y", collide=False),
          CylShape((WAND_X, TIP_Y + 0.023, 0.0), 0.0025, 0.006, FLAME_Y, axis="Y", collide=False)]
    return Body.build("lighter_flame", pos=_v(_hide(1)), mass=0.01, color=FLAME_O, gravity=False,
                      damping=(5.0, 5.0), shapes=sh)


JAR = _jar()
LIGHTER = _lighter()
CANDLE_FLAME = _candle_flame()
LIGHTER_FLAME = _lighter_flame()
OBJECTS = [JAR, LIGHTER, CANDLE_FLAME, LIGHTER_FLAME]

# -- tracking and scoring -------------------------------------------------------------------------------------------
DT = 0.05
CLEAR = 0.02
DOWN_Z = 0.020
TIP_LATCH_DEG, TILT_OK_DEG, MOVE_OK = 30.0, 10.0, 0.020
STILL_STEPS, STILL_MM = 20, 1.5
_DEBUG = os.environ.get("CANDLE_DEBUG") == "1"
_DEBUG_EVERY = int(os.environ.get("CANDLE_DEBUG_EVERY", "5"))
_EP: dict = {}
_IDS: dict = {}
# points on the lighter (handle corners and the wand) for "lowest point" and finger clearance
_HP = [[x, y, z] for x in (-H_DEPTH / 2 - TRIG_PROUD, H_DEPTH / 2 + LATCH_PROUD) for y in np.linspace(-H_LEN / 2,
       H_LEN / 2, 6) for z in (-H_THICK / 2, H_THICK / 2)]
_WP = [[WAND_X + WAND_R * c, y, WAND_R * s] for y in np.linspace(H_LEN / 2, TIP_Y, 10)
       for c, s in ((1, 0), (-1, 0), (0, 1), (0, -1))]
LIGHTER_PTS = np.array(_HP + _WP)


@on_reset
def _reset() -> None:
    _EP.update(step=0, tick=0, counted=-1, t=0.0, latch=0.0, trig=0.0, gas=False, near=1.0, heat=0.0, lit=False,
               lit_t=None, tipped=False, tilt=0.0, moved=0.0, low=1.0, wick_d=1.0, recent=[], tcp={}, gaps={},
               shown=[False, False], blocked=None, max_tilt=0.0, gas_s=0.0)


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


def _place(base, origin, name: str, pos, quat) -> None:
    import torch

    p = np.asarray(pos, float) + origin
    pose = np.array([[p[0], p[1], p[2], *[float(v) for v in quat]]])
    obj = base.scene[name]
    obj.write_root_link_pose_to_sim_index(root_pose=torch.tensor(pose, dtype=torch.float32, device=base.device))
    obj.write_root_com_velocity_to_sim_index(root_velocity=torch.zeros((1, 6), dtype=torch.float32,
                                                                       device=base.device))


@on_step
def _track(env) -> None:
    try:
        _track_inner(env)
    except Exception:
        import traceback

        print("CANDLE_TRACEBACK\n" + traceback.format_exc(), flush=True)
        raise


def _set_block(base, lt, ids, blocked: bool) -> None:
    """The child-resistant block: the trigger's travel limit (2 mm while the latch is not held, 10 mm when it is)."""
    import torch

    hi = TRIG_BLOCK if blocked else TRIG_TRAVEL
    lim = torch.tensor([[[0.0, hi]]], dtype=torch.float32, device=base.device)
    lt.write_joint_position_limit_to_sim(lim, joint_ids=[ids["trigger"]], warn_limit_violation=False)


def _track_inner(env) -> None:
    from scipy.spatial import cKDTree

    from yam_arena import ik, station
    from yam_arena.constants import ARM_JOINTS, FINGER_JOINTS

    _EP["tick"] += 1
    _EP["t"] += DT
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
        fingers.append(np.concatenate([_IDS["fingers"][n] @ rotation(poses[i][3:7]).T + poses[i][:3] - origin
                                       for n, i in zip(("tip_left", "tip_right"), _IDS[side + "_tips"])]))
    lt = base.scene["lighter"]
    if "lj" not in _IDS:
        names = list(lt.joint_names)
        _IDS["lj"] = {k: names.index(f"{k}_turn") for k in ("latch", "trigger")}
    qj = as_numpy(lt.data.joint_pos)[0]
    _EP["latch"] = float(-qj[_IDS["lj"]["latch"]])
    _EP["trig"] = float(qj[_IDS["lj"]["trigger"]])
    blocked = _EP["latch"] < LATCH_HELD
    if blocked != _EP["blocked"] and (not blocked or _EP["trig"] <= TRIG_BLOCK + 0.0005):
        _set_block(base, lt, _IDS["lj"], blocked)
        _EP["blocked"] = blocked
    lp = as_numpy(lt.data.root_link_pose_w)[0]
    lr = rotation(lp[3:7])
    lpos = lp[:3] - origin
    pts = lpos + LIGHTER_PTS @ lr.T
    _EP["low"] = float(pts[:, 2].min() - TOP)
    tree = cKDTree(pts)
    _EP["near"] = min(float(tree.query(fp, k=1)[0].min()) for fp in fingers)
    jp = as_numpy(base.scene["jar"].data.root_link_pose_w)[0]
    jr = rotation(jp[3:7])
    jpos = jp[:3] - origin
    _EP["tilt"] = math.degrees(math.acos(max(-1.0, min(1.0, float(jr[2, 2])))))
    _EP["max_tilt"] = max(_EP["max_tilt"], _EP["tilt"])
    _EP["moved"] = float(np.linalg.norm(jpos[:2] - np.array([JX, JY])))
    if _EP["tilt"] >= TIP_LATCH_DEG:
        _EP["tipped"] = True
    wick = jpos + jr @ WICK_TIP
    tip = lpos + lr @ np.array([WAND_X, TIP_Y, 0.0])
    ax = lr @ np.array([0.0, 1.0, 0.0])
    s = float(np.clip(np.dot(wick - tip, ax), 0.0, FLAME_LEN))
    _EP["wick_d"] = float(np.linalg.norm(wick - (tip + s * ax)))
    _EP["wvec"] = wick - tip
    _EP["gas"] = bool(_EP["latch"] >= LATCH_HELD and _EP["trig"] >= TRIG_ON)
    if _EP["gas"]:
        _EP["gas_s"] += DT
        if _EP["wick_d"] <= CATCH_R:
            _EP["heat"] += DT
    if not _EP["lit"] and _EP["heat"] >= CATCH_S - 1e-9:
        _EP["lit"] = True
        _EP["lit_t"] = _EP["t"]
    if _EP["lit"]:
        _place(base, origin, "candle_flame", wick, jp[3:7])
        _EP["shown"][0] = True
    if _EP["gas"]:
        _place(base, origin, "lighter_flame", lpos, lp[3:7])
        _EP["shown"][1] = True
    elif _EP["shown"][1]:
        _place(base, origin, "lighter_flame", _hide(1), (0.0, 0.0, 0.0, 1.0))
        _EP["shown"][1] = False


def _tally(o) -> dict:
    recent = _EP["recent"]
    if _EP["counted"] != _EP["tick"]:
        _EP["counted"] = _EP["tick"]
        recent.append(_EP["low"])
        del recent[:-STILL_STEPS]
    still = len(recent) == STILL_STEPS and max(recent) - min(recent) < STILL_MM / 1000
    down = bool(_EP["lit"] and not _EP["gas"] and _EP["low"] < DOWN_Z and still and _EP["near"] >= CLEAR)
    jar_ok = _EP["tilt"] < TILT_OK_DEG and _EP["moved"] <= MOVE_OK
    return dict(still=still, down=down, done=bool(down and jar_ok and not _EP["tipped"]), jar_ok=jar_ok)


def _debug(t) -> None:
    _EP["step"] += 1
    if (_EP["step"] - 1) % _DEBUG_EVERY:
        return
    print(f"DBG step={_EP['step']} t={_EP['t']:.2f} latch={_EP['latch'] * 1000:.1f} trig={_EP['trig'] * 1000:.1f} "
          f"blocked={int(bool(_EP['blocked']))} gas={int(_EP['gas'])} gas_s={_EP['gas_s']:.2f} "
          f"wick_d={_EP['wick_d'] * 1000:.0f}mm wvec=({','.join(f'{v * 1000:.0f}' for v in _EP.get('wvec', np.zeros(3)))})  heat={_EP['heat']:.2f} lit={int(_EP['lit'])} | jar tilt="
          f"{_EP['tilt']:.1f} moved={_EP['moved'] * 1000:.0f}mm tipped={int(_EP['tipped'])} | lighter low="
          f"{_EP['low'] * 1000:.0f}mm near={_EP['near'] * 1000:.0f}mm still={int(t['still'])} down={int(t['down'])} | "
          + " ".join(f"{k} tcp=({v[0]:.3f},{v[1]:.3f},{v[2] - TOP:.3f}) gap={_EP['gaps'][k] * 1000:.0f}"
                     for k, v in _EP["tcp"].items()), flush=True)


def score(objects) -> float:
    t = _tally(objects)
    if _DEBUG:
        _debug(t)
    if _EP["tipped"]:
        return 0.0
    if t["done"]:
        return 1.0
    if t["down"]:
        return 0.75
    if _EP["lit"]:
        return 0.5
    return 0.0


def success(objects) -> bool:
    return bool(_tally(objects)["done"])


# -- scripted demo --------------------------------------------------------------------------------------------------
# Open loop, from the known layout. The right gripper, straight down with its fingers closing along x, takes the lying
# lighter's handle with the latch under one fingertip and the trigger under the other and closes to a 40 mm gap (both
# go about 2 mm in: no gas), lifts it, turns the wand down over the jar (62 deg below level), slides the nozzle 8 cm
# along the wand's line into the jar to about 1 cm from the wick, squeezes (24 mm commanded: latch fully in, trigger
# fully in, gas) for 2.5 s, opens back to 40 mm (flame off), draws the nozzle out, turns the lighter level, lays it down
# where it was, lets go and rises. The left arm stays at its ready pose. CANDLE_NOLATCH=1 grips 2 cm further toward the
# wand (the trigger but not the latch under the fingertips) and squeezes hard throughout: the trigger stays on its
# block, no gas, score 0. CANDLE_DX/DY/DZ aim the tip past where it ends up (the lighter settles a few mm in the grip).
OPEN_C = -0.047
NOLATCH = os.environ.get("CANDLE_NOLATCH") == "1"


def _R(c1, c3) -> np.ndarray:
    c1, c3 = np.asarray(c1, float), np.asarray(c3, float)
    c1, c3 = c1 / np.linalg.norm(c1), c3 / np.linalg.norm(c3)
    return np.column_stack([c1, np.cross(c3, c1), c3])


DOWN_X = grip_rotation(math.pi / 2)  # straight down, fingers closing along x (the right arm)


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


def _ready(arm: str, t: float) -> list:
    from yam_arena import ik, station
    from yam_arena.constants import READY

    pos, rot = ik.tcp_pose(np.array(READY, float), station.BASES[arm])
    return [Waypoint(t, tuple(float(v) for v in pos), OPEN_C, "stay put", rot)]


PINCH = _f("CANDLE_PINCH", "-0.012")  # finger value to fire: a 24 mm gap commanded (latch and trigger stop it at 32)
HOLD = _f("CANDLE_HOLD", "-0.020")  # finger value to carry: a 40 mm gap (latch in, trigger at its 2 mm block)
if NOLATCH:  # the wrong way squeezes the trigger hard all along (it stops at its 2 mm block: no gas)
    HOLD = PINCH
GRIP_Z = _f("CANDLE_GRIP_Z", "0.020")  # the TCP above the table when picking the lighter up
GRIP_DY = 0.020 if NOLATCH else 0.0  # the wrong way: the fingers 20 mm further toward the wand: trigger only
AIM_AZ, AIM_EL, AIM_ROLL = _f("CANDLE_AZ", "105"), _f("CANDLE_EL", "62"), _f("CANDLE_ROLL", "30")
TIP_BACK = _f("CANDLE_TIP_BACK", "0.010")  # the nozzle tip this far short of the wick, along the wand
APPROACH = 0.080  # the pre-aim pose: the tip this much further back along the wand (above the rim)
FIRE_S = _f("CANDLE_FIRE_S", "2.5")


def _grip_point() -> np.ndarray:
    xc = LX + (H_DEPTH / 2 + LATCH_PROUD - H_DEPTH / 2 - TRIG_PROUD) / 2
    return np.array([xc, LY + (LATCH_Y + TRIG_Y) / 2 + GRIP_DY, TOP + GRIP_Z])


def _tip_in_tcp() -> np.ndarray:
    """The nozzle tip in the TCP frame (columns of the grip rotation: closing, pad width, approach) as picked up."""
    g = _grip_point()
    tip = np.array([LX + WAND_X, LY + TIP_Y, LZ])
    return DOWN_X.T @ (tip - g)


def _aim_rot() -> tuple:
    a, e, r = math.radians(AIM_AZ), math.radians(AIM_EL), math.radians(AIM_ROLL)
    w = np.array([math.cos(e) * math.cos(a), math.cos(e) * math.sin(a), -math.sin(e)])
    u = np.cross(w, [0.0, 0.0, 1.0])
    u /= np.linalg.norm(u)
    v = np.cross(w, u)
    c3 = math.cos(r) * u + math.sin(r) * v
    if c3[0] < 0:
        c3 = -c3
    c2 = -w
    return np.column_stack([np.cross(c2, c3), c2, c3]), w


def _tcp_for_tip(tip, rot) -> np.ndarray:
    return np.asarray(tip, float) - rot @ _tip_in_tcp()


def _qpath(arm, p0, r0, p1, r1, finger, label, seconds, steps) -> list:
    """Joint-space interpolation between two IK solutions (for the big turns of the wrist in free space)."""
    from yam_arena import ik, station

    q0, q1 = _good_ik(arm, p0, r0), _good_ik(arm, p1, r1)
    out = []
    for k in range(1, steps + 1):
        p, r = ik.tcp_pose(q0 + (q1 - q0) * k / steps, station.BASES[arm])
        out.append(Waypoint(seconds / steps, tuple(float(v) for v in p), finger, label, r))
    return out


def _line(p0, p1, rot, finger, label, seconds, steps) -> list:
    return [_w(seconds / steps, p0 + (p1 - p0) * k / steps, finger, label, rot) for k in range(1, steps + 1)]


def _light() -> list:
    g = _grip_point()
    up = g + np.array([0.0, 0.0, 0.17])
    rot, w = _aim_rot()
    wick = np.array([JX, JY, TOP + 0.001]) + WICK_TIP
    t_fire = wick - TIP_BACK * w + np.array([_f("CANDLE_DX", "0.003"), _f("CANDLE_DY", "0.0085"), _f("CANDLE_DZ", "0.007")])
    t_pre = t_fire - APPROACH * w
    p_fire, p_pre = _tcp_for_tip(t_fire, rot), _tcp_for_tip(t_pre, rot)
    out = _lead_in("right", g + np.array([0.0, 0.0, 0.10]), DOWN_X, OPEN_C, "above the lighter's grip")
    out += [_w(1.5, g + np.array([0.0, 0.0, 0.01]), OPEN_C, "fingers either side of latch and trigger", DOWN_X),
            _w(0.5, g, OPEN_C, "down", DOWN_X),
            _w(1.2, g, HOLD, "close on the grip, trigger not pulled", DOWN_X),
            _w(0.5, g, HOLD, "settle", DOWN_X),
            _w(1.5, up, HOLD, "lift", DOWN_X)]
    out += _qpath("right", up, DOWN_X, p_pre, rot, HOLD, "turn the wand down over the jar", 3.0, 15)
    out += _line(p_pre, p_fire, rot, HOLD, "nozzle down into the jar to the wick", 2.0, 10)
    out += [_w(0.6, p_fire, HOLD, "settle", rot),
            _w(0.8, p_fire, PINCH, "latch held in, pull the trigger", rot),
            _w(FIRE_S, p_fire, PINCH, "flame at the wick", rot),
            _w(0.6, p_fire, HOLD, "let go of the trigger", rot),
            _w(0.4, p_fire, HOLD, "flame off", rot)]
    out += _line(p_fire, p_pre, rot, HOLD, "nozzle back out of the jar", 1.6, 8)
    out += _qpath("right", p_pre, rot, up, DOWN_X, HOLD, "turn the lighter back level", 3.0, 15)
    down = g + np.array([0.0, 0.0, 0.004])
    out += [_w(1.5, down, HOLD, "lay it down where it was", DOWN_X),
            _w(0.6, down, OPEN_C, "open the fingers", DOWN_X),
            _w(1.0, g + np.array([0.0, 0.0, 0.10]), OPEN_C, "rise", DOWN_X)]
    out += _ready("right", 6.0)
    return out


def demo():
    right = _light()
    t = sum(w.duration for w in right)
    return {"right": right, "left": _ready("left", t + 1.0)}
