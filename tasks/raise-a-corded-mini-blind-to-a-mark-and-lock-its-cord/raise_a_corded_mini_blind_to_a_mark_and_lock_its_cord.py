"""At the back of the station table stands a wall section with a window opening (59 x 50 cm, sill 2 cm above the
table). A corded 1-inch aluminium mini blind is mounted inside the opening, fully lowered: 57.2 cm wide (a 23 in
blind, 22.5 in actual), shortened to 20 slats, its bottom rail hanging 1 cm above the sill. Its pull cord leaves
the cord lock under the headrail's right end (the robot's right) and hangs straight down with its tassel. A strip of blue painter's tape on
the wall beside the opening marks 30 cm above the rail's start. Task (instruction): raise the blind so that its
bottom rail is level with the tape, and leave it there. The instruction does not say how.

How it is done (the manufacturer's operating instructions for its stock aluminium and vinyl blinds): "To raise or
lower the installed blind, pull the cords to the LEFT (Fig. I) to disengage. Gently raise or lower to desired
position." "To lock blind into desired position, pull the cords to the RIGHT (Fig. J)." The cord lock sits at the
right end: pulled toward the blind's centre (left) the lock is held open and the cord runs both ways; hanging
straight or swung to the right, its cam catches as soon as the cord starts back up, so the blind stays where the
hand left it.

The hand skill: catch a 3 mm cord that hangs free in front of the slats and pull it down, hand over hand
(30 cm of rise is more than one arm's stroke within reach: the cord can be held from 44 cm down to 14 cm above the
table), keeping it taut while the other hand takes a new grip higher up, stop with the rail level with the tape,
then let go with the cord hanging straight so the lock catches. Let go with the cord pulled toward the centre and the
blind runs back down. A single hand can also do it in strokes, letting the lock catch between them.

World frame (m): x forward from the arm bases, y to the robot's left, z up; the table top is at z TOP. Heights below
are above the table top.
- blind: one articulation fixed to the world. Root: the wall section (face at x 0.290, 11 cm thick, y -0.36 to 0.66,
  0.90 tall) with the opening y -0.145 to 0.445, z 0.02 to 0.52, a sill board, a glass pane at x 0.39, the tape
  (y -0.215 to -0.150, centred 0.330 up, 12 mm tall) and the blind's headrail (steel, x 0.300-0.335, y -0.136 to
  0.436, z 0.495-0.520) with its tilt wand at the left end. Parts:
  - rail: the bottom rail (steel, 25 x 570 x 10 mm, 100 g) on a vertical slide, start bottom 0.030, lift 0 to 0.44.
  - slat_0..19: 25 x 566 mm aluminium slats (2 mm collision thickness, 8 g) on vertical slides, each resting on its
    ladder rung (lower stop) 22 mm apart, the lowest 22 mm above the rail; the rising rail lifts them and stacks them.
  - swivel: the cord's pivot in the cord lock at (0.298, -0.070, 0.493), a ball joint.
  - cord0: the pull cord (two lift cords side by side, drawn as one 3 mm cord) on a slide along itself from the
    swivel: pay-out. Its rigid top length runs up through the lock into the wall (hidden); 40 mm hang below the lock
    at the start. A PhysX mimic joint couples pay-out to the rail's lift one to one (the lift cords run from the
    rail up through the slats, over the headrail's cradles and out through the lock).
  - link_1..9: 15 mm cord lengths on ball joints, and the tassel (white plastic, 12 mm across, 30 mm long).
- the lock (a rule on the pay-out, applied each control step as the rail's lower limit): while the cord hangs within
  15 deg of straight or to the right (its top length less than 15 deg toward the centre, +y), the cam engages: the
  rail can go up freely but not down more than 8 mm below its highest point since the cam engaged. With the cord 15
  deg or more toward the centre the cam is held open and the rail runs both ways.

Score (latched hazard first, then the end state):
- 0 if the blind ever runs down: the rail falls 4 cm or more within 0.5 s (latched; a hand lowering it slower is
  allowed).
- At the end, with the cord let go (no finger within 15 mm of the cord or tassel) and the rail still (under 2 mm of
  movement) for the last 5 s: 1.0 when the rail's underside is within 2 cm of the tape's centre line; 0.5 when it is
  elsewhere but at least 5 cm above its start. Otherwise 0.
"""

from __future__ import annotations

import itertools
import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

from bodykit import BoxShape, CapsuleShape, CylShape, SphereShape, allow_matrix_grips  # noqa: E402
from bodykit import as_numpy, on_reset, on_step, rotation  # noqa: E402
import blindkit  # noqa: E402,F401  (hooks yam_arena so the blind below is simulated)
from blindkit import Part, Rig  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

allow_matrix_grips()

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"

# -- the layout (heights above the table top) ------------------------------------------------------------------------
WALL_X, WALL_T = 0.290, 0.110
WALL_Y0, WALL_Y1, WALL_H = -0.36, 0.66, 0.90
OY0, OY1, SILL, HEAD = -0.145, 0.445, 0.020, 0.520
GLASS_X = 0.390
HR_X0, HR_X1, HR_Y0, HR_Y1, HR_Z0 = 0.300, 0.335, -0.136, 0.436, 0.495
BL_X, BL_Y = (HR_X0 + HR_X1) / 2, (HR_Y0 + HR_Y1) / 2
SLAT_W, SLAT_L, SLAT_T, PITCH, N_SLATS = 0.025, 0.566, 0.002, 0.022, 20
RAIL_W, RAIL_L, RAIL_H, RAIL0 = 0.025, 0.570, 0.010, 0.030
LIFT_MAX = 0.44
MARK = 0.330  # the tape's centre line: 30 cm above the rail's start
BAND = 0.020
TAPE_Y0, TAPE_Y1, TAPE_H = -0.215, -0.150, 0.012
CORD_X, CORD_Y, PIVOT_Z = 0.298, -0.070, 0.493
CORD_R = 0.0015
HIDE = 0.40  # the cord's rigid top length runs this far up through the lock into the wall
S0 = 0.040  # and hangs this far below the lock at the start
N_LINKS, LINK_L = 9, 0.015
TASSEL_R, TASSEL_L = 0.006, 0.030
LOCK_DEG = 15.0
SLIP = 0.008
DROP_M, DROP_STEPS = 0.040, 10
CLEAR = 0.015
HOLD_STEPS, STILL_M = 100, 0.002
RAISED_MIN = 0.05

RAIL_MASS, SLAT_MASS = float(os.environ.get("BLIND_RAILM", "0.10")), 0.008
CORD_MASS, LINK_MASS, TASSEL_MASS = 0.004, 0.002, 0.006

WHITE, WALLC, STEEL, SLATC = (0.93, 0.93, 0.91), (0.86, 0.84, 0.79), (0.90, 0.90, 0.88), (0.94, 0.94, 0.92)
CORDC, TAPE, GLASSC, SILLC = (0.97, 0.97, 0.95), (0.20, 0.45, 0.85), (0.62, 0.74, 0.82), (0.80, 0.70, 0.55)
STRINGC = (0.85, 0.85, 0.82)

ORIGIN = (WALL_X + WALL_T / 2, (WALL_Y0 + WALL_Y1) / 2, TOP + WALL_H / 2)


def _r(x: float, y: float, z: float) -> tuple:
    """A world point (x, y, height above the table) relative to the rig's origin."""
    return (float(x - ORIGIN[0]), float(y - ORIGIN[1]), float(TOP + z - ORIGIN[2]))


def _w(x: float, y: float, z: float) -> tuple:
    return (float(x), float(y), float(TOP + z))


def _box(x0, x1, y0, y1, z0, z1, color, collide=True) -> BoxShape:
    return BoxShape(_r((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2), (x1 - x0, y1 - y0, z1 - z0), color,
                    collide=collide)


def _vcap(x, y, z_top, z_bot, radius, color, collide=True) -> CapsuleShape:
    """A vertical capsule whose ends (with their round caps) lie at z_top and z_bot."""
    return CapsuleShape(_r(x, y, (z_top + z_bot) / 2), radius, max(z_top - z_bot - 2 * radius, 1e-4), color,
                        collide=collide, axis="Z")


def _root_shapes() -> list:
    wx1 = WALL_X + WALL_T
    sh = [_box(WALL_X, wx1, WALL_Y0, OY0, 0.0, WALL_H, WALLC),  # right of the opening
          _box(WALL_X, wx1, OY1, WALL_Y1, 0.0, WALL_H, WALLC),  # left of it
          _box(WALL_X, wx1, OY0, OY1, HEAD, WALL_H, WALLC),  # above it (the lintel)
          _box(WALL_X - 0.02, wx1, OY0, OY1, 0.0, SILL, SILLC),  # the sill board
          _box(GLASS_X, GLASS_X + 0.005, OY0, OY1, SILL, HEAD, GLASSC),  # the glass pane
          _box(WALL_X - 0.001, WALL_X, TAPE_Y0, TAPE_Y1, MARK - TAPE_H / 2, MARK + TAPE_H / 2, TAPE, collide=False),
          # the headrail and its end caps, the cord lock under its right end, the tilter and wand at its left end
          _box(HR_X0, HR_X1, HR_Y0, HR_Y1, HR_Z0, HEAD, STEEL),
          _box(HR_X0 - 0.001, HR_X1, HR_Y0, HR_Y0 + 0.004, HR_Z0, HEAD, (0.85, 0.85, 0.83), collide=False),
          _box(HR_X0 - 0.001, HR_X1, HR_Y1 - 0.004, HR_Y1, HR_Z0, HEAD, (0.85, 0.85, 0.83), collide=False),
          _box(CORD_X - 0.003, HR_X0 + 0.012, CORD_Y - 0.011, CORD_Y + 0.011, HR_Z0 - 0.004, HR_Z0, (0.92, 0.92, 0.9),
               collide=False),
          CylShape(_r(HR_X0 - 0.004, HR_Y1 - 0.045, HR_Z0 - 0.004), 0.004, 0.010, WHITE, collide=False),
          _vcap(HR_X0 - 0.004, HR_Y1 - 0.045, HR_Z0 - 0.010, HR_Z0 - 0.300, 0.0035, (0.85, 0.88, 0.9))]
    # the top lengths of the ladders and lift cords, headrail down to the top slat
    for y in _string_ys():
        for x in _string_xs():
            sh.append(CylShape(_r(x, y, (HR_Z0 + _slat_z(N_SLATS - 1)) / 2), 0.0005, HR_Z0 - _slat_z(N_SLATS - 1),
                               STRINGC, collide=False))
    return sh


def _string_ys() -> tuple:
    return (BL_Y - 0.20, BL_Y + 0.20)


def _string_xs() -> tuple:
    return (BL_X - SLAT_W / 2 + 0.0006, BL_X, BL_X + SLAT_W / 2 - 0.0006)


def _slat_z(k: int) -> float:
    """Slat k's middle height at rest (on its rung)."""
    return RAIL0 + RAIL_H + PITCH * (k + 1) - SLAT_T / 2


def _strings_up(z0: float, length: float) -> list:
    """The ladder strings and lift cords from a slat (or the rail) up toward the next rung (drawn only)."""
    return [CylShape(_r(x, y, z0 + length / 2), 0.0005, length, STRINGC, collide=False)
            for y in _string_ys() for x in _string_xs()]


def _parts() -> list:
    parts = [Part("rail", [_box(BL_X - RAIL_W / 2, BL_X + RAIL_W / 2, BL_Y - RAIL_L / 2, BL_Y + RAIL_L / 2, RAIL0,
                                RAIL0 + RAIL_H, STEEL)] + _strings_up(RAIL0 + RAIL_H, PITCH),
                  RAIL_MASS, "prismatic", _w(BL_X, BL_Y, RAIL0), axis="Z", limits=(0.0, LIFT_MAX),
                  damping=float(os.environ.get("BLIND_RDAMP", "1.0")),
                  friction=float(os.environ.get("BLIND_RFRIC", "0.3")), material=(0.4, 0.35))]
    for k in range(N_SLATS):
        z = _slat_z(k)
        top = HR_Z0 - z if k == N_SLATS - 1 else PITCH
        parts.append(Part(f"slat_{k}", [_box(BL_X - SLAT_W / 2, BL_X + SLAT_W / 2, BL_Y - SLAT_L / 2,
                                             BL_Y + SLAT_L / 2, z - SLAT_T / 2, z + SLAT_T / 2, SLATC)]
                          + ([] if k == N_SLATS - 1 else _strings_up(z + SLAT_T / 2, top)),
                          SLAT_MASS, "prismatic", _w(BL_X, BL_Y, z), axis="Z", limits=(0.0, LIFT_MAX),
                          damping=0.2, material=(0.3, 0.25)))
    parts.append(Part("swivel", [SphereShape(_r(CORD_X, CORD_Y, PIVOT_Z), 0.002, CORDC, collide=False)], 0.002,
                      "ball", _w(CORD_X, CORD_Y, PIVOT_Z), limits=(60.0,), damping=0.0005,
                      com=_w(CORD_X, CORD_Y, PIVOT_Z), inertia=(1e-8, 1e-8, 1e-8)))
    hide_col = os.environ.get("BLIND_HIDECOL", "1") == "1"
    parts.append(Part("cord0", [_vcap(CORD_X, CORD_Y, PIVOT_Z + HIDE, PIVOT_Z + 0.01, CORD_R, CORDC, collide=hide_col),
                                _vcap(CORD_X, CORD_Y, PIVOT_Z + 0.012, PIVOT_Z - S0 + 0.001, CORD_R, CORDC)],
                      CORD_MASS, "prismatic", _w(CORD_X, CORD_Y, PIVOT_Z), parent="swivel", axis="Z",
                      limits=(-LIFT_MAX, 0.0), damping=0.2, material=(0.9, 0.8),
                      com=_w(CORD_X, CORD_Y, PIVOT_Z - S0 / 2), inertia=(2e-7, 2e-7, 2e-8)))
    z = PIVOT_Z - S0
    prev = "cord0"
    for i in range(1, N_LINKS + 1):
        parts.append(Part(f"link_{i}", [_vcap(CORD_X, CORD_Y, z - 0.001, z - LINK_L + 0.001, CORD_R, CORDC)],
                          LINK_MASS, "ball", _w(CORD_X, CORD_Y, z), parent=prev, limits=(70.0,), damping=2e-4,
                          stiffness=2e-4, material=(0.9, 0.8)))
        prev = f"link_{i}"
        z -= LINK_L
    parts.append(Part("tassel", [CylShape(_r(CORD_X, CORD_Y, z - 0.004), 0.002, 0.008, CORDC),
                                 CylShape(_r(CORD_X, CORD_Y, z - 0.008 - TASSEL_L / 2), TASSEL_R, TASSEL_L, WHITE),
                                 SphereShape(_r(CORD_X, CORD_Y, z - 0.008 - TASSEL_L), TASSEL_R, WHITE)],
                      TASSEL_MASS, "ball", _w(CORD_X, CORD_Y, z), parent=prev, limits=(70.0,), damping=2e-4,
                      material=(0.6, 0.5)))
    return parts


TASSEL_TOP = PIVOT_Z - S0 - N_LINKS * LINK_L
BLIND = Rig.build("blind", pos=ORIGIN, size=(WALL_T, WALL_Y1 - WALL_Y0, WALL_H), shapes=tuple(_root_shapes()),
                  parts=tuple(_parts()), mimic=(("cord0", "rail", 1.0),), filtered=((None, "cord0"),))

OBJECTS = [BLIND]


# -- tracking (every control step) ---------------------------------------------------------------------------------
_DEBUG = os.environ.get("BLIND_DEBUG") == "1"
_DEBUG_EVERY = int(os.environ.get("BLIND_DEBUG_EVERY", "5"))
_EP: dict = {}
_IDS: dict = {}


@on_reset
def _reset() -> None:
    _EP.clear()
    _EP.update(tick=0, step=0, counted=-1, engaged=True, high=0.0, lower=0.0, written=0.0, dropped=False, hist=[],
               near_hist=[], near=1.0, near_s={}, tcp={}, gaps={}, events=[], lift=0.0, peak=0.0, theta=0.0,
               max_theta=0.0)


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


def _cord_local() -> dict:
    """Points along the visible cord in each cord body's frame (the rig's origin, world-aligned at the start)."""
    o = np.asarray(ORIGIN, float)
    # the whole rigid top length: the part still above the lock is inside the wall, where no finger gets
    pts = {"cord0": np.array([_w(CORD_X, CORD_Y, z) for z in np.arange(PIVOT_Z + HIDE, PIVOT_Z - S0, -0.004)]) - o}
    z = PIVOT_Z - S0
    for i in range(1, N_LINKS + 1):
        pts[f"link_{i}"] = np.array([_w(CORD_X, CORD_Y, zz) for zz in np.linspace(z, z - LINK_L, 5)]) - o
        z -= LINK_L
    pts["tassel"] = np.array([_w(CORD_X, CORD_Y, zz) for zz in np.linspace(z, z - 0.008 - TASSEL_L, 9)]) - o
    return pts


def _set_lower(art, lower: float) -> None:
    import torch

    if abs(lower - _EP.get("written", -1.0)) < 0.0005:
        return
    lim = torch.tensor([[[float(lower), LIFT_MAX]]], dtype=torch.float32, device=art.device)
    art.write_joint_position_limit_to_sim_index(limits=lim, joint_ids=[_IDS["rail_j"]], warn_limit_violation=False)
    _EP["written"] = lower


@on_step
def _track(env) -> None:
    try:
        _track_inner(env)
    except Exception:
        import traceback

        print("BLIND_TRACEBACK\n" + traceback.format_exc(), flush=True)
        raise


def _track_inner(env) -> None:
    from scipy.spatial import cKDTree

    from yam_arena import ik, station
    from yam_arena.constants import ARM_JOINTS, FINGER_JOINTS

    _EP["tick"] += 1
    base = getattr(env, "unwrapped", env)
    origin = as_numpy(base.scene.env_origins[0])
    art = base.scene["blind"]
    if "fingers" not in _IDS:
        _IDS["fingers"] = _finger_points()
        _IDS["cord"] = _cord_local()
        jn = list(art.joint_names)
        _IDS["rail_j"] = jn.index("rail_j")
        _IDS["cord_j"] = jn.index("cord0_j")
        bn = list(art.body_names)
        _IDS["bodies"] = {n: bn.index(n) for n in _IDS["cord"]}
        if _DEBUG:
            print(f"DBG joints={jn}", flush=True)
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
    q = as_numpy(art.data.joint_pos)[0]
    lift = float(q[_IDS["rail_j"]])
    pay = float(-q[_IDS["cord_j"]])
    pose = as_numpy(art.data.body_link_pose_w)[0]
    cpts = []
    for n, loc in _IDS["cord"].items():
        p = pose[_IDS["bodies"][n]]
        cpts.append(loc @ rotation(p[3:7]).T + p[:3] - origin)
    cpts = np.concatenate(cpts)
    r0 = rotation(pose[_IDS["bodies"]["cord0"]][3:7])
    down = r0 @ np.array([0.0, 0.0, -1.0])
    theta = math.degrees(math.atan2(float(down[1]), float(-down[2])))  # the cord's top length toward +y (centre)
    # the cord lock: engaged unless the cord is pulled toward the centre; engaged, it lets the rail rise but not fall
    # more than SLIP below its highest point since it engaged
    if theta >= LOCK_DEG:
        if _EP["engaged"]:
            _EP["events"].append((_EP["tick"], f"lock released at lift {lift * 1000:.0f} mm"))
        _EP["engaged"] = False
        lower = 0.0
    else:
        if not _EP["engaged"]:
            _EP["engaged"] = True
            _EP["high"] = lift
            _EP["events"].append((_EP["tick"], f"lock engaged at lift {lift * 1000:.0f} mm"))
        _EP["high"] = max(_EP["high"], lift)
        lower = max(0.0, _EP["high"] - SLIP)
    _set_lower(art, lower)
    hist = _EP["hist"]
    hist.append(lift)
    del hist[:-DROP_STEPS - 1]
    if not _EP["dropped"] and max(hist) - lift >= DROP_M:
        _EP["dropped"] = True
        _EP["events"].append((_EP["tick"], f"ran down from {max(hist) * 1000:.0f} to {lift * 1000:.0f} mm"))
    tree = cKDTree(cpts)
    near = {s: float(tree.query(fp, k=1)[0].min()) for s, fp in fingers.items()}
    _EP.update(lift=lift, pay=pay, theta=theta, lower=lower, near=min(near.values()), near_s=near,
               peak=max(_EP["peak"], lift), max_theta=max(_EP["max_theta"], theta), tassel=cpts[-1])


def _tally(o) -> dict:
    if _EP["counted"] != _EP["tick"] and _EP["tick"] > 0:
        _EP["counted"] = _EP["tick"]
        _EP["near_hist"].append((_EP["near"], _EP["lift"]))
        del _EP["near_hist"][:-HOLD_STEPS]
    nh = _EP["near_hist"]
    full = len(nh) == HOLD_STEPS
    released = full and min(n for n, _ in nh) >= CLEAR
    lifts = [h for _, h in nh]
    still = full and (max(lifts) - min(lifts)) < STILL_M
    bottom = RAIL0 + _EP["lift"]
    in_band = abs(bottom - MARK) <= BAND
    if _EP["dropped"]:
        s = 0.0
    elif released and still and in_band:
        s = 1.0
    elif released and still and _EP["lift"] >= RAISED_MIN:
        s = 0.5
    else:
        s = 0.0
    return dict(released=released, still=still, in_band=in_band, bottom=bottom, score=s)


def _debug(t) -> None:
    _EP["step"] += 1
    if (_EP["step"] - 1) % _DEBUG_EVERY:
        return
    ts = _EP.get("tassel", np.zeros(3))
    print(f"DBG step={_EP['step']} lift={_EP['lift'] * 1000:.1f}mm pay={_EP.get('pay', 0) * 1000:.1f} "
          f"bottom={t['bottom']:.4f} peak={_EP['peak'] * 1000:.1f} theta={_EP['theta']:.1f} "
          f"max_theta={_EP['max_theta']:.1f} eng={int(_EP['engaged'])} lower={_EP['lower'] * 1000:.1f} "
          f"drop={int(_EP['dropped'])} tassel=({ts[0]:.3f},{ts[1]:.3f},{ts[2] - TOP:.3f}) "
          f"near={_EP['near'] * 1000:.0f}mm rel={int(t['released'])} still={int(t['still'])} "
          f"band={int(t['in_band'])} score={t['score']:.2f} ev={_EP['events'][-3:]} | "
          + " ".join(f"{k} tcp=({v[0]:.3f},{v[1]:.3f},{v[2] - TOP:.3f}) gap={_EP['gaps'][k] * 1000:.0f} "
                     f"near={_EP['near_s'].get(k, 1) * 1000:.0f}"
                     for k, v in _EP["tcp"].items()), flush=True)


def score(objects) -> float:
    t = _tally(objects)
    if _DEBUG:
        _debug(t)
    return float(t["score"])


def success(objects) -> bool:
    return bool(_tally(objects)["score"] >= 0.999)


# -- scripted demo --------------------------------------------------------------------------------------------------
# Open loop, from the known layout. Both grippers level, pointing forward (+x), fingers opening across y, so the
# vertical cord lies along the pads. The right hand takes the cord 42 cm up and pulls it down to 17 cm, 4 cm out toward
# the robot so the hand stays clear of the rising rail, then brings it back under the lock (about 24 cm of cord); the
# left hand takes it again 40 cm up, above the right hand, the right hand lets go and backs away, the left pulls on
# down to 33.5 cm (4 cm out), holds, lets go with the cord hanging straight (the lock catches 8 mm lower) and backs
# away.
# BLIND_WRONG=centre: the same, but the left hand swings the cord 6 cm toward the centre (20 deg) before letting go.
# BLIND_ONE=1: the right hand alone: a 25 cm stroke, let go (the lock catches), a new grip higher up, the rest.
OPEN_C = -0.047
OPEN_P = float(os.environ.get("BLIND_OPEN", "-0.030"))  # a 6 cm opening around the cord (the jamb is 7.5 cm away)
GRIP = float(os.environ.get("BLIND_GRIP", "0.0"))
WRONG = os.environ.get("BLIND_WRONG", "")
ONE = os.environ.get("BLIND_ONE") == "1"
TCP_DX = float(os.environ.get("BLIND_DX", "0.001"))
GRAB_A = float(os.environ.get("BLIND_ZA", "0.42"))
LOW_A = float(os.environ.get("BLIND_LA", "0.17"))
GRAB_B = float(os.environ.get("BLIND_ZB", "0.40"))
PULL_B = float(os.environ.get("BLIND_PB", "0.065"))
FWD = float(os.environ.get("BLIND_FWD", "0.04"))  # pulls go this far out toward the robot, clear of the rail
ROT = np.column_stack([(0.0, -1.0, 0.0), (0.0, 0.0, -1.0), (1.0, 0.0, 0.0)])  # level, toward +x, closing along y


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


def _wp(t, p, finger, label, rot=ROT) -> Waypoint:
    return Waypoint(float(t), tuple(float(v) for v in p), float(finger), label, rot)


def _at(z: float, dx: float = 0.0, dy: float = 0.0) -> np.ndarray:
    return np.array([CORD_X + TCP_DX + dx, CORD_Y + dy, TOP + z])


def _take(z: float, label: str) -> list:
    """From in front of the cord, in round it (fingers open across y), and close."""
    return [_wp(2.0, _at(z, -0.10), OPEN_P, f"in front of the cord, {label}"),
            _wp(2.0, _at(z), OPEN_P, "in round the cord"),
            _wp(0.4, _at(z), OPEN_P, "settle"),
            _wp(1.0, _at(z), GRIP, "close on the cord"),
            _wp(0.6, _at(z), GRIP, "settle")]


def _away(z: float, label: str, dx: float = 0.0) -> list:
    return [_wp(0.8, _at(z, dx), OPEN_P, label),
            _wp(1.5, _at(z, -0.10), OPEN_P, "back out, clear of the cord")]


def _right() -> list:
    out = _lead_in("right", _at(GRAB_A, -0.10), ROT, OPEN_P, "level, in front of the cord", 3.0)
    out += _take(GRAB_A, "42 cm up")
    out += [_wp(5.0, _at(LOW_A, -FWD), GRIP, "pull it down 25 cm, a little out toward the robot, clear of the rail"),
            _wp(1.0, _at(LOW_A), GRIP, "back under the lock: the cord hangs straight")]
    if ONE:
        out += _away(LOW_A, "let go: the lock catches")
        out += _take(GRAB_B, "40 cm up")
        out += [_wp(2.0, _at(GRAB_B - PULL_B, -FWD), GRIP, "pull on down"),
                _wp(1.5, _at(GRAB_B - PULL_B, -FWD), GRIP, "hold a moment")]
        out += _away(GRAB_B - PULL_B, "let go with the cord hanging straight", -FWD)
        out += [_wp(15.0, _at(GRAB_B - PULL_B, -0.10, -0.08), OPEN_P, "stay clear")]
        return out
    out += [_wp(7.0, _at(LOW_A), GRIP, "hold it taut while the left hand takes it higher up")]
    out += _away(LOW_A, "let go")
    out += [_wp(2.0, _at(LOW_A, -0.12, -0.08), OPEN_P, "out of the way, low and to the right"),
            _wp(20.0, _at(LOW_A, -0.12, -0.08), OPEN_P, "stay clear")]
    return out


def _left() -> list:
    from yam_arena import ik, station
    from yam_arena.constants import READY

    if ONE:
        pos, rot = ik.tcp_pose(np.array(READY, float), station.BASES["left"])
        return [_wp(120.0, pos, OPEN_C, "stay put", rot)]
    wait = np.array([0.18, 0.10, TOP + 0.50])
    out = _lead_in("left", wait, ROT, OPEN_P, "level, up and to the left, out of the right hand's way", 3.0)
    out += [_wp(10.0, wait, OPEN_P, "wait while the right hand pulls")]
    out += _take(GRAB_B, "40 cm up, above the right hand")
    out += [_wp(3.3, _at(GRAB_B), GRIP, "hold while the right hand lets go")]
    end = _at(GRAB_B - PULL_B, -FWD)
    out += [_wp(2.0, end, GRIP, "pull on down to the mark, a little out toward the robot"),
            _wp(1.5, end, GRIP, "hold a moment")]
    if WRONG == "centre":
        side = end + np.array([0.0, 0.06, 0.0])
        out += [_wp(1.5, side, GRIP, "swing the cord toward the blind's centre"),
                _wp(0.5, side, GRIP, "settle")]
        end = side
    out += [_wp(0.8, end, OPEN_P, "let go"),
            _wp(1.5, end + np.array([-0.10, 0.0, 0.0]), OPEN_P, "back out, clear of the cord"),
            _wp(2.0, np.array([0.18, 0.10, TOP + 0.45]), OPEN_P, "up and to the left"),
            _wp(20.0, np.array([0.18, 0.10, TOP + 0.45]), OPEN_P, "stay clear")]
    return out


def demo():
    return {"left": _left(), "right": _right()}
