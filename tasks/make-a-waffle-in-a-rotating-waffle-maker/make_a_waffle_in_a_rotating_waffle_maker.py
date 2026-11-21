"""On the station table: a rotating Belgian waffle maker of the kind used at hotel breakfast buffets (like the Waring
WW180: a round 7-inch Belgian grid set in a cast housing that turns 180 deg on its stand, worked by one handle on the
lid), unplugged and cold, its lid standing open; beside it a stainless batter cup holding a cup of batter (30 beads).
Task (instruction): make a waffle: pour the cup of batter onto the waffle grid, close and latch the lid, turn the waffle
maker over and leave it to bake, then turn it back and open the lid.

How it is done (Waring WW180 instructions): "Fill measuring scoop with batter to fill line. Pour batter evenly into
the center of the waffle grid ... Close the lid and rotate the unit 180 degrees." "Once the waffle is ready ... rotate
the waffle maker 180 degrees, open the lid and remove waffle". On models whose lid latches, the latch is closed before
the turn ("Make sure your lid has latched into place ... Turn the handle so that the waffle plates flip 180 degrees",
Wayfair's guide): turned over unlatched, the lid swings open underneath and the batter falls out.

The hand skill: a cup of batter poured over its lip onto the 18 cm grid, none over the edge; the lid (1.2 kg) tipped
over from its open stop so it falls shut, then pressed down at its handle until the latch clicks (10 N); the handle
held in a pinch and turned 180 deg about its own axis, a full wrist roll with the whole housing on it; let go while
it bakes; turned back; the handle pulled up against the latch (about 17 N) and the lid lifted open.

World frame (m): x forward from the arm bases, y to the robot's left, z up; heights are above the table (top at
z 0.7475).
- waffle maker: fixed (a 6 kg unit on rubber feet). Base plate 26 x 30 x 2 cm centred at (0.25, 0.12); a post at its
  rear (y 0.245) carries the turning shaft. The housing turns about an axis along y at x 0.25, 14.5 cm up; it is
  23 cm across and 9 cm thick when shut (two 4.5 cm halves meeting at the axis height), centred at y 0.095. Each half
  holds a 7-inch (18 cm) Belgian grid 1.2 cm deep. The lid is hinged along x at the housing's rear edge (y 0.21,
  14.5 cm up); its handle (11 cm long, 3 cm wide, 2.6 cm deep) runs out along the turning axis from the lid's front
  edge to y -0.125. The housing rests on its stops at 0 and 180 deg (turned 180 deg, the lid is underneath).
  At the start the lid stands open, just past its balance point.
- latch: a spring catch between the lid and the bottom half: it clicks shut when the lid, closed, is pressed down with
  10 N (PhysX contact force on the lid, with the housing upright), then holds the lid shut in any position (300 N m/rad
  at its catch) until the lid is pulled up at its handle with about 17 N (opened 0.5 deg against it).
- batter cup: stainless, no handle, 7.6 cm across the top, 6.0 cm at the base, 7.5 cm tall, 0.12 kg, standing at
  (0.21, -0.18); in it 30 beads 10 mm across, 8 g each (240 g, a cup of batter).
Batter: rigid beads, as liquids are modelled with beads in accepted tasks. A bead is on the grid when its centre lies
over the grid (within 9 cm of the grid's centre, in the housing's frame) and within the space between the two grids.

Score (the best stage reached; "in" = at least 27 of the 30 beads on the grid):
- 0.25: the lid closed (within 2 deg) with the batter in.
- 0.5: the lid latched with the batter in.
- 0.75: the housing turned over (175 deg or more) with the lid latched and the batter in, and left there for 3 s with
  both hands off it (every finger point 3 cm or more from the waffle maker): the bake, shortened.
- 1.0 (success, held 1 s): after that, the housing turned back (within 5 deg of upright), the lid unlatched and open
  30 deg or more, the batter in.
0 for the whole run once the lid opens 20 deg or more while the housing is turned more than 60 deg from upright (the
batter falls out of an upside-down waffle maker: on a hot unit, onto the counter and the hotplate).
"""

from __future__ import annotations

import itertools
import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

from bodykit import Body, BoxShape, CylShape, Link, Linkage, SphereShape, allow_matrix_grips, as_numpy  # noqa: E402
from bodykit import on_reset, on_step, plate, rotation  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

allow_matrix_grips()

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"
_E = os.environ.get

# -- the layout (world, m; heights above the table) ------------------------------------------------------------------
AX, AZ = 0.25, 0.145  # the turning axis (along y): x, height
YC = 0.095  # the housing's centre (y)
R_H = 0.115  # the housing's radius
R_G = 0.090  # the grid's radius (7-inch grid)
HALF = 0.045  # each half's thickness
CAV = 0.012  # each grid's depth
YH = YC + R_H  # the lid's hinge axis (along x): y; height AZ
H_LEN, H_W, H_T = 0.110, 0.030, 0.026  # the lid's handle: length (y), width (x), depth (z)
H_END = YC - R_H - H_LEN  # the handle's end (y)
LID_OPEN = -92.0  # deg: the lid's open stop (opening is negative about +x)
LID_CLOSE_STOP = 1.0  # deg: the lid's stop past touching (the latch holds it here)
LID_M, LOW_M = 1.2, 2.4
D_LID = float(_E("WAFFLE_DLID", "0.5"))  # N m s/rad on the lid's hinge (the lid falls shut in about 2 s)
HEAD_FRIC = float(_E("WAFFLE_HFRIC", "0.25"))  # N m: the turning shaft's friction
K_LATCH = float(_E("WAFFLE_KLATCH", "300.0"))  # N m/rad at the catch
LATCH_SET = math.radians(0.3)  # the catch pulls the lid to 0.3 deg past touching
LATCH_RELEASE = math.radians(-0.5)  # it lets go when the lid is opened 0.5 deg against it
PRESS_N = 10.0  # N: the press that clicks the latch shut
BASE_T = 0.020
POST_Y = 0.245
N_BEADS = int(_E("WAFFLE_BEADS", "30"))
BEAD_R, BEAD_M = 0.005, 0.008
N_IN = 27
CUP = np.array([0.21, -0.18])  # the batter cup's centre
CUP_RB, CUP_RT, CUP_H, CUP_W = 0.030, 0.038, 0.075, 0.0025
CUP_GZ = 0.055  # the height on the cup where the demo grips it
BAKE_S = 3.0
OFF_D = 0.030
TURNED, UPRIGHT = 175.0, 5.0
OPEN_DONE = 30.0
DUMP_OPEN, DUMP_TURN = 20.0, 60.0

STEEL, DARK, GRID, BAKE, BATTER, CUPC, RED = ((0.72, 0.73, 0.75), (0.10, 0.10, 0.11), (0.20, 0.20, 0.22),
                                              (0.15, 0.15, 0.16), (0.93, 0.80, 0.50), (0.80, 0.81, 0.83),
                                              (0.75, 0.12, 0.10))


def _v(p) -> tuple:
    return tuple(float(x) for x in p)


def _ring(zc: float, h: float, r_in: float, r_out: float, color, n: int = 28) -> list:
    """A ring wall from r_in to r_out (in x, y about the housing's centre), centred at height zc (relative), h tall."""
    out = []
    r = (r_in + r_out) / 2
    w = (r_out - r_in)
    seg = 2 * math.pi * r / n * 1.25  # overlapping segments
    for k in range(n):
        a = 2 * math.pi * k / n
        out.append(BoxShape((r * math.cos(a), YC - 0.0 + r * math.sin(a) - YC, zc), (w, seg, h), color,
                            yaw=a))
    return out


def _grid_marks(z: float) -> list:
    """The Belgian grid's squares, drawn only."""
    out = []
    for k in range(-3, 4):
        d = k * 0.024
        half = math.sqrt(max(R_G * R_G - d * d, 0.0)) - 0.004
        out.append(BoxShape((d, 0.0, z), (0.004, 2 * half, 0.0006), DARK, collide=False))
        out.append(BoxShape((0.0, d, z), (2 * half, 0.004, 0.0006), DARK, collide=False))
    return out


def _waffle_maker() -> Linkage:
    o = np.array([AX, YC, TOP])  # the linkage origin: shapes below are relative to it
    zs = AZ  # the axis height
    base = [BoxShape((0.0, 0.025, BASE_T / 2), (0.26, 0.30, BASE_T), DARK),
            BoxShape((0.0, POST_Y - YC, (zs + 0.02) / 2), (0.05, 0.03, zs + 0.02), DARK),  # the rear post
            CylShape((0.0, POST_Y - YC - 0.025, zs), 0.012, 0.05, STEEL, axis="Y", collide=False),  # the shaft
            # the control panel on the base's front edge (drawn): dial and lights
            CylShape((-0.10, -0.08, BASE_T + 0.004), 0.012, 0.008, STEEL, collide=False),
            CylShape((-0.10, -0.03, BASE_T + 0.002), 0.004, 0.004, RED, collide=False),
            CylShape((-0.10, -0.015, BASE_T + 0.002), 0.004, 0.004, (0.1, 0.7, 0.2), collide=False)]
    # the bottom half (turns with the shaft): housing, grid floor, the grid's rim
    floor_t = 0.006
    low = [CylShape((0.0, 0.0, zs - HALF + (HALF - CAV - floor_t) / 2), R_H, HALF - CAV - floor_t, STEEL),
           CylShape((0.0, 0.0, zs - CAV - floor_t / 2), R_G + 0.004, floor_t, GRID)]
    low += _ring(zs - CAV / 2, CAV, R_G, R_H, STEEL)
    low += _grid_marks(zs - CAV + 0.0004)
    low += [BoxShape((0.0, R_H + 0.010, zs - 0.012), (0.06, 0.03, 0.02), DARK),  # the shaft's boss at the rear
            BoxShape((0.0, -R_H - 0.006, zs - 0.010), (0.034, 0.014, 0.016), DARK)]  # the latch's catch at the front
    # the lid: housing, grid, rim, hinge knuckle and handle
    lid = [CylShape((0.0, 0.0, zs + CAV + floor_t + (HALF - CAV - floor_t) / 2), R_H, HALF - CAV - floor_t, STEEL),
           CylShape((0.0, 0.0, zs + CAV + floor_t / 2), R_G + 0.004, floor_t, GRID)]
    lid += _ring(zs + CAV / 2, CAV, R_G, R_H, STEEL)
    lid += [BoxShape((0.0, R_H - 0.004, zs + 0.010), (0.08, 0.016, 0.016), DARK),  # the hinge knuckle
            BoxShape((0.0, YC - YC - R_H - H_LEN / 2 + 0.006, zs), (H_W, H_LEN + 0.012, H_T), DARK),  # the handle
            BoxShape((0.0, -R_H - 0.004, zs + 0.012), (0.036, 0.012, 0.020), DARK)]  # the handle's root
    links = [Link("head", tuple(low), mass=LOW_M, anchor=_v([AX, YC, TOP + zs]), axis="Y",
                  limits=(-181.0, 1.0), friction=HEAD_FRIC, damping=0.2, stiffness=1.0e-4, spring_damping=0.0),
             Link("lid", tuple(lid), mass=LID_M, anchor=_v([AX, YH, TOP + zs]), axis="X", parent="head",
                  limits=(LID_OPEN, LID_CLOSE_STOP), init=LID_OPEN, stiffness=1.0e-4, spring_damping=D_LID,
                  report_contacts=True)]
    return Linkage.build("waffle", pos=_v(o), mass=2.4, color=DARK, floating=False, friction=(0.6, 0.5),
                         shapes=base, links=links, contact_offset=0.002, solver_iterations=48)


def _cup() -> Body:
    sh = [CylShape((0.0, 0.0, 0.002), CUP_RB, 0.004, CUPC)]
    n = 20
    for k in range(n):
        a = 2 * math.pi * k / n
        c, s = math.cos(a), math.sin(a)
        p1 = (CUP_RB * c, CUP_RB * s, 0.004)
        p2 = (CUP_RT * c, CUP_RT * s, CUP_H)
        seg = 2 * math.pi * CUP_RT / n * 1.3
        sh.append(plate(p1, p2, seg, CUP_W, (c, s, 0.0), CUPC))
    return Body.build("cup", pos=(float(CUP[0]), float(CUP[1]), TOP + 0.001), mass=0.12, color=CUPC,
                      friction=(0.7, 0.6), shapes=sh, solver_iterations=24)


def _bead_spots() -> list:
    out = []
    lay = 0
    while len(out) < N_BEADS:
        z = 0.004 + BEAD_R + 0.0015 + lay * (2 * BEAD_R + 0.001)
        r_ok = CUP_RB + (CUP_RT - CUP_RB) * (z / CUP_H) - CUP_W - BEAD_R - 0.0015
        off = 0.0 if lay % 2 == 0 else 0.0055
        for i in range(-3, 4):
            for j in range(-3, 4):
                x, y = i * 0.0110 + off, j * 0.0110 + off
                if math.hypot(x, y) <= r_ok and len(out) < N_BEADS:
                    out.append((x, y, z))
        lay += 1
    return out


def _beads() -> list:
    out = []
    for k, (x, y, z) in enumerate(_bead_spots()):
        out.append(Body.build(f"bead_{k}", pos=(float(CUP[0] + x), float(CUP[1] + y), TOP + 0.001 + z), mass=BEAD_M,
                              color=BATTER, friction=(0.5, 0.4), damping=(0.05, 0.05),
                              shapes=[SphereShape((0.0, 0.0, 0.0), BEAD_R, BATTER)], solver_iterations=16))
    return out


WAFFLE = _waffle_maker()
CUP_BODY = _cup()
BEADS = _beads()
OBJECTS = [WAFFLE, CUP_BODY] + BEADS

# -- tracking and scoring --------------------------------------------------------------------------------------------
_DEBUG = _E("WAFFLE_DEBUG") == "1"
_DEBUG_EVERY = int(_E("WAFFLE_DEBUG_EVERY", "5"))
_EP: dict = {}
_IDS: dict = {}
_CV: dict = {}


@on_reset
def _reset() -> None:
    _EP.update(tick=0, step=0, head=0.0, lid=LID_OPEN, latched=False, press=0.0, press_n=0, n_in=0,
               stage=0.0, dumped=False, off_t=0.0, baked=False, near=1.0, k_on=None, tgt=None, fz=0.0, tcp={},
               gaps={})
    _CV.clear()


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


def _lid_force(base) -> np.ndarray:
    """The net contact force (N, world) on the lid, from PhysX."""
    from isaaclab_physx.physics import PhysxManager

    sim = PhysxManager.get_physics_sim_view()
    view = _CV.get("view") if _CV.get("sim") is sim else None
    if view is None:
        env = base.scene.env_prim_paths[0]
        view = sim.create_rigid_contact_view([f"{env}/waffle/lid"], filter_patterns=[], max_contact_data_count=64)
        _CV.update(sim=sim, view=view, dt=float(base.physics_dt))
    f = view.get_net_contact_forces(dt=_CV["dt"])
    a = f.numpy() if hasattr(f, "numpy") else as_numpy(f)
    return np.asarray(a, float).reshape(-1, 3)[0]


def _set_lid_drive(wm, k: float, target: float) -> None:
    import torch

    j = _IDS["lid"]
    if _EP["k_on"] != k:
        wm.write_joint_stiffness_to_sim_index(stiffness=torch.tensor([[k]], dtype=torch.float32, device=wm.device),
                                              joint_ids=[j])
        _EP["k_on"] = k
    if _EP["tgt"] != target:
        wm.set_joint_position_target_index(target=torch.tensor([[target]], dtype=torch.float32, device=wm.device),
                                           joint_ids=[j])
        _EP["tgt"] = target
    wm.write_data_to_sim()


def _head_rot(h: float) -> np.ndarray:
    c, s = math.cos(h), math.sin(h)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


@on_step
def _track(env) -> None:
    try:
        _track_inner(env)
    except Exception:
        import traceback

        print("WAFFLE_TRACEBACK\n" + traceback.format_exc(), flush=True)
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
    fingers = np.concatenate(fingers)
    wm = base.scene["waffle"]
    if "lid" not in _IDS:
        jn = list(wm.joint_names)
        _IDS["head"] = jn.index("head_turn")
        _IDS["lid"] = jn.index("lid_turn")
    qj = as_numpy(wm.data.joint_pos)[0]
    head, lid = float(qj[_IDS["head"]]), float(qj[_IDS["lid"]])
    f = _lid_force(base)
    # the latch: clicks when the closed lid is pressed down with PRESS_N (housing upright), lets go when pulled open
    upright = abs(math.degrees(head)) <= 10.0
    if not _EP["latched"]:
        if upright and lid >= math.radians(-1.0) and -f[2] >= PRESS_N:
            _EP["press_n"] += 1
        else:
            _EP["press_n"] = 0
        if _EP["press_n"] >= 2:
            _EP["latched"] = True
    elif lid <= LATCH_RELEASE:
        _EP["latched"] = False
    if _EP["latched"]:
        _set_lid_drive(wm, K_LATCH, LATCH_SET)
    else:
        _set_lid_drive(wm, 1.0e-4, lid)
    # the beads on the grid, in the housing's frame (origin on the axis at the housing's centre)
    rot = _head_rot(head)
    cen = np.array([AX, YC, TOP + AZ])
    n_in = 0
    for k in range(N_BEADS):
        p = as_numpy(base.scene[f"bead_{k}"].data.root_link_pose_w)[0][:3] - origin
        lp = rot.T @ (p - cen)
        if math.hypot(lp[0], lp[1]) <= R_G and -CAV - 0.001 <= lp[2] <= CAV + 0.001 + (0.0 if lid > -0.035 else 0.03):
            n_in += 1
    # finger points near the waffle maker: distance to the handle's axis segment and to the housing's disc
    hy0, hy1 = H_END, YC
    hp = rot.T @ (fingers - cen).T
    hp = hp.T
    dy = np.clip(hp[:, 1] + YC, hy0, hy1) - (hp[:, 1] + YC)
    d_handle = np.sqrt(hp[:, 0] ** 2 + dy ** 2 + hp[:, 2] ** 2) - H_T / 2
    d_disc = np.maximum(np.hypot(hp[:, 0], hp[:, 1]) - R_H, np.abs(hp[:, 2]) - HALF)
    near = float(min(d_handle.min(), d_disc.min()))
    hd, ld = math.degrees(head), math.degrees(lid)
    if abs(hd) > DUMP_TURN and ld <= -DUMP_OPEN:
        _EP["dumped"] = True
    batter = n_in >= N_IN
    closed = ld >= -2.0
    st = _EP["stage"]
    if batter and closed:
        st = max(st, 0.25)
        if _EP["latched"]:
            st = max(st, 0.5)
    turned = abs(hd) >= TURNED
    if turned and _EP["latched"] and batter and near >= OFF_D:
        _EP["off_t"] += 0.05
    else:
        _EP["off_t"] = 0.0
    if _EP["off_t"] >= BAKE_S - 1e-6:
        _EP["baked"] = True
        st = max(st, 0.75)
    if _DEBUG:
        bp = np.array([as_numpy(base.scene[f"bead_{k}"].data.root_link_pose_w)[0][:3] - origin for k in range(N_BEADS)])
        _EP["beads"] = (bp.mean(0), bp[:, 2].min() - TOP, bp[:, 2].max() - TOP)
        _EP["cup"] = as_numpy(base.scene["cup"].data.root_link_pose_w)[0][:3] - origin
    _EP.update(head=hd, lid=ld, n_in=n_in, near=near, stage=st, fz=float(f[2]))


def _tally() -> dict:
    done = bool(_EP["baked"] and not _EP["dumped"] and abs(_EP["head"]) <= UPRIGHT and not _EP["latched"]
                and _EP["lid"] <= -OPEN_DONE and _EP["n_in"] >= N_IN)
    return dict(done=done)


def _debug(t) -> None:
    _EP["step"] += 1
    if (_EP["step"] - 1) % _DEBUG_EVERY:
        return
    print(f"DBG step={_EP['step']} head={_EP['head']:+.1f} lid={_EP['lid']:+.2f} latched={int(_EP['latched'])} "
          f"fz={_EP['fz']:+.1f}N in={_EP['n_in']} near={_EP['near'] * 1000:.0f}mm off_t={_EP['off_t']:.2f} "
          f"baked={int(_EP['baked'])} dumped={int(_EP['dumped'])} stage={_EP['stage']} done={int(t['done'])} "
          f"beads=({_EP['beads'][0][0]:.3f},{_EP['beads'][0][1]:.3f}) z={_EP['beads'][1]*1000:.0f}..{_EP['beads'][2]*1000:.0f}mm "
          f"cup=({_EP['cup'][0]:.3f},{_EP['cup'][1]:.3f},{(_EP['cup'][2]-TOP)*1000:.0f}mm) | "
          + " ".join(f"{k} tcp=({v[0]:.3f},{v[1]:.3f},{(v[2] - TOP) * 1000:.1f}mm) gap={_EP['gaps'][k] * 1000:.0f}"
                     for k, v in _EP["tcp"].items()), flush=True)


def score(objects) -> float:
    t = _tally()
    if _DEBUG:
        _debug(t)
    if _EP["dumped"]:
        return 0.0
    if t["done"]:
        return 1.0
    return float(_EP["stage"])


def success(objects) -> bool:
    return bool(_tally()["done"])


# -- scripted demo ---------------------------------------------------------------------------------------------------
# Open loop, from the known layout. The right gripper takes the batter cup round its top (fingers closing along x),
# carries it over the grid, tipped 30 deg, and tips it on to 100 deg about x so the beads run out over its -y lip onto
# the grid, then sets it back down. The left
# gripper, its fingers closed, pushes the open lid's back near the hinge so it tips past upright and falls shut. The
# right presses the lid's handle down until the latch clicks; pinches the handle's end (level, pointing along the
# handle, fingers above and below) and rolls its wrist 192 deg (the arm lags under load; the housing stops at 180);
# lets go and keeps clear for the bake; pinches the handle again and rolls back; then pulls the handle up along the
# lid's arc, unlatching it and lifting the lid open (about 45 deg).
# WAFFLE_NOLATCH=1 (the wrong way): the lid is closed but never pressed down to latch; turned over and let go, the lid
# swings open underneath and the batter falls out.
OPEN_C = -0.047
NOLATCH = _E("WAFFLE_NOLATCH") == "1"
P = {k: float(_E("WAFFLE_" + k.upper(), v)) for k, v in dict(
    cup_grip="-0.030", handle_grip="-0.009", press_over="0.035", pour_deg="100", lift_deg="42", lift_over="4",
    roll_over="12", pour_x="0.0", pour_y="0.0", pour_z="0.0", push_y="0.240", push_z="0.200").items()}


def _R(c1, c3) -> np.ndarray:
    c1, c3 = np.asarray(c1, float), np.asarray(c3, float)
    c1, c3 = c1 / np.linalg.norm(c1), c3 / np.linalg.norm(c3)
    return np.column_stack([c1, np.cross(c3, c1), c3])


def _axis_rot(axis, t: float) -> np.ndarray:
    a = np.asarray(axis, float)
    a = a / np.linalg.norm(a)
    k = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + math.sin(t) * k + (1 - math.cos(t)) * k @ k


DOWN_RY = _R((0.0, -1.0, 0.0), (0.0, 0.0, -1.0))  # right arm, pointing down, fingers closing along y
DOWN_RX = _R((1.0, 0.0, 0.0), (0.0, 0.0, -1.0))  # right arm, pointing down, fingers closing along x
PUSH_L = _R((0.0, -1.0, 0.0), (0.0, 0.0, -1.0))  # left arm, pointing down, fingers closing along y
LVL_Y = _R((0.0, 0.0, 1.0), (0.0, 1.0, 0.0))  # right arm, level, pointing +y, fingers closing along z


def _roll(ph_deg: float) -> np.ndarray:
    """LVL_Y rolled ``ph_deg`` about +y (the handle's axis)."""
    return _axis_rot((0.0, 1.0, 0.0), math.radians(ph_deg)) @ LVL_Y


def _pour_rot(deg: float) -> np.ndarray:
    """DOWN_RY rolled ``deg`` about +x (the cup handle's axis): the tool turns from down toward +y."""
    return _axis_rot((1.0, 0.0, 0.0), math.radians(deg)) @ DOWN_RX


def _good_ik(arm: str, pos, rot):
    from yam_arena import ik, station

    best = None
    b = station.BASES[arm]
    j1 = math.atan2(float(pos[1]) - b[1], float(pos[0]) - b[0])
    for sd in itertools.product((j1,), (0.8, 1.3, 1.8), (0.6, 1.2, 1.8), (-0.8, 0.0, 0.8), (0.0, -1.0, 1.0),
                                (-1.8, -0.9, 0.0, 0.9, 1.8)):
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


def _wait(wps, t, label="wait") -> None:
    if t > 1e-6:
        last = wps[-1]
        wps.append(Waypoint(t, last.pos, last.finger, label, last.yaw))


def _sync(L, R) -> None:
    dl, dr = _dur(L), _dur(R)
    _wait(L, dr - dl)
    _wait(R, dl - dr)


def _handle_tcp(alpha_deg: float, r: float) -> np.ndarray:
    """The TCP pinching the lid's handle ``r`` from the hinge, the lid opened ``alpha_deg`` (housing upright)."""
    a = math.radians(alpha_deg)
    return np.array([AX, YH - r * math.cos(a), TOP + AZ + r * math.sin(a)])


def _handle_rot(alpha_deg: float) -> np.ndarray:
    """Pointing along the handle toward the hinge, rolled 180 deg (fingers below and above), lid open alpha_deg."""
    return _axis_rot((1.0, 0.0, 0.0), -math.radians(alpha_deg)) @ _roll(180.0)


def _timeline():
    L, R = [], []
    n = 16
    # 1. the right takes the cup and pours it over the grid
    hpos = np.array([CUP[0], CUP[1], TOP + 0.001 + CUP_GZ])
    R += _lead_in("right", hpos + [0, 0, 0.08], DOWN_RX, OPEN_C, "above the cup")
    R += [_w(1.5, hpos, OPEN_C, "fingers either side of the cup", DOWN_RX),
          _w(1.0, hpos, P["cup_grip"], "grip the cup", DOWN_RX),
          _w(0.3, hpos, P["cup_grip"], "settle", DOWN_RX),
          _w(1.5, hpos + [0, 0, 0.08], P["cup_grip"], "lift the cup", DOWN_RX),
          _w(1.5, hpos + [0.0, 0.0, 0.165], P["cup_grip"], "raise it, tipped a little toward the grid", _pour_rot(30.0))]
    # the pour: hand over the grid's front so the cup's lip (toward -y once rolled) is over the grid's centre
    pour = np.array([AX + P["pour_x"], YC + 0.015 + P["pour_y"],
                     TOP + AZ + 0.085 + P["pour_z"]])
    R += [_w(2.5, pour, P["cup_grip"], "carry the cup over the grid", _pour_rot(30.0)),
          _w(0.5, pour, P["cup_grip"], "settle", _pour_rot(30.0))]
    for k in range(1, n + 1):
        d = 30.0 + (P["pour_deg"] - 30.0) * k / n
        R.append(_w(3.0 / n, pour, P["cup_grip"], "tip the cup: pour", _pour_rot(d)))
    R += [_w(2.0, pour, P["cup_grip"], "let it run out", _pour_rot(P["pour_deg"]))]
    for k in range(1, n // 2 + 1):
        d = 30.0 + (P["pour_deg"] - 30.0) * (1 - k / (n // 2))
        R.append(_w(1.5 / (n // 2), pour, P["cup_grip"], "tip it back", _pour_rot(d)))
    R += [_w(2.5, hpos + [0.0, 0.0, 0.165], P["cup_grip"], "carry the cup back", _pour_rot(30.0)),
          _w(1.5, hpos + [0, 0, 0.08], P["cup_grip"], "level it", DOWN_RX),
          _w(1.5, hpos + [0, 0, 0.002], P["cup_grip"], "set it down", DOWN_RX),
          _w(0.8, hpos + [0, 0, 0.002], OPEN_C, "let go", DOWN_RX),
          _w(1.0, hpos + [0, 0, 0.08], OPEN_C, "rise", DOWN_RX)]
    # 2. the left pushes the open lid over near its hinge, so it falls shut
    lp0 = np.array([AX, P["push_y"] + 0.060, TOP + P["push_z"] + 0.02])
    L += _lead_in("left", lp0, PUSH_L, 0.0, "behind the open lid, fingers closed")
    _sync(L, R)
    lp1 = np.array([AX, P["push_y"] + 0.060, TOP + P["push_z"]])
    lp2 = np.array([AX, P["push_y"], TOP + P["push_z"]])
    L += [_w(1.0, lp1, 0.0, "down behind the lid", PUSH_L),
          _w(2.0, lp2, 0.0, "push the lid over, low near its hinge", PUSH_L),
          _w(1.0, lp2 + [0, 0.02, 0.0], 0.0, "it falls shut", PUSH_L),
          _w(1.5, lp2 + [0, 0.04, 0.02], 0.0, "rise clear", PUSH_L),
          _w(1.5, lp2 + [0, 0.04, 0.02], 0.0, "wait", PUSH_L)]
    L += _lead_out("left", lp2 + [0, 0.04, 0.02], PUSH_L, 0.0, "back to the ready pose")
    _sync(L, R)
    # 3. the right presses the closed lid's handle down until the latch clicks
    press = np.array([AX, YC - R_H - 0.050, TOP + AZ + H_T / 2 + 0.013])
    if not NOLATCH:
        R += [_w(2.0, press + [0, 0, 0.05], 0.0, "over the lid's handle, fingers closed", DOWN_RX),
              _w(1.0, press, 0.0, "onto the handle", DOWN_RX),
              _w(1.5, press - [0, 0, P["press_over"]], 0.0, "press it down: the latch clicks", DOWN_RX),
              _w(0.5, press - [0, 0, P["press_over"]], 0.0, "hold", DOWN_RX),
              _w(1.0, press + [0, 0, 0.05], 0.0, "rise", DOWN_RX)]
    # 4. pinch the handle's end, fingers above and below, and roll it over (180 -> 0)
    grip = np.array([AX, H_END + 0.018, TOP + AZ])
    side = grip + [0.085, 0.0, 0.0]
    clear = side + [0.0, 0.0, 0.03]
    R += [_w(2.0, side + [0, 0, 0.03], OPEN_C, "beside the handle's end, level along it", _roll(180.0)),
          _w(1.0, side, OPEN_C, "level with it", _roll(180.0)),
          _w(1.5, grip, OPEN_C, "fingers above and below the handle", _roll(180.0)),
          _w(1.0, grip, P["handle_grip"], "pinch the handle", _roll(180.0)),
          _w(0.3, grip, P["handle_grip"], "settle", _roll(180.0))]
    for k in range(1, n + 1):
        R.append(_w(4.0 / n, grip, P["handle_grip"], "roll the wrist: turn it over", _roll(180.0 - (180.0 + P["roll_over"]) * k / n)))
    R += [_w(0.5, grip, P["handle_grip"], "settle", _roll(-P["roll_over"])),
          _w(0.8, grip, OPEN_C, "let go", _roll(0.0)),
          _w(1.5, side, OPEN_C, "back off sideways", _roll(0.0)),
          _w(1.0, clear, OPEN_C, "rise", _roll(0.0)),
          _w(BAKE_S + 1.0, clear, OPEN_C, "keep clear while it bakes", _roll(0.0)),
          _w(1.0, side, OPEN_C, "back down beside the handle", _roll(0.0))]
    # 5. pinch it again and roll it back (0 -> 180)
    R += [_w(1.5, grip, OPEN_C, "fingers above and below the handle", _roll(0.0)),
          _w(1.0, grip, P["handle_grip"], "pinch the handle", _roll(0.0)),
          _w(0.3, grip, P["handle_grip"], "settle", _roll(0.0))]
    for k in range(1, n + 1):
        R.append(_w(4.0 / n, grip, P["handle_grip"], "roll it back upright", _roll(182.0 * k / n)))
    R += [_w(0.5, grip, P["handle_grip"], "settle", _roll(180.0))]
    # 6. pull the handle up along the lid's arc: the latch lets go and the lid opens
    r = YH - grip[1]
    for k in range(1, n + 1):
        a = (P["lift_deg"] + P["lift_over"]) * k / n
        R.append(_w(4.0 / n, _handle_tcp(a, r), P["handle_grip"], "lift the lid open by its handle",
                    _handle_rot(a)))
    R += [_w(2.5, _handle_tcp(P["lift_deg"] + P["lift_over"], r), P["handle_grip"], "hold it open",
             _handle_rot(P["lift_deg"] + P["lift_over"]))]
    _sync(L, R)
    return L, R


def demo():
    left, right = _timeline()
    lp, lq = _ready("left")
    end = max(_dur(left), _dur(right)) + 1.0
    left += [_w(end - _dur(left), lp, OPEN_C, "stay clear", lq)]
    _wait(right, end - _dur(right), "hold it open")
    return {"left": left, "right": right}
