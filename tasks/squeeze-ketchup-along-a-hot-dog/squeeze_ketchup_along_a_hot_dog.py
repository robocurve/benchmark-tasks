"""On the station table: a hot dog in its bun on a paper plate, and beside it an upside-down ketchup squeeze bottle
standing on its cap, the flip lid open. Task (instruction): put a line of ketchup along the sausage from end to end,
keeping it on the sausage, and stand the bottle back on its cap. The instruction does not say how.

How it is done: the bottle is held tip down a little above one end of the sausage, squeezed steadily and carried
along the sausage at a speed matching the flow, so that one even line runs from end to end; the squeeze is let off
just before the far end. The National Hot Dog and Sausage Council's etiquette: "Always 'dress the dog,' not the bun."
Squeezing in one place makes a pool that runs off the sausage onto the bun; a line off the crest of the sausage runs
down its side into the bun.

The hand skill: one hand keeps a steady squeeze on the bottle's walls (the walls give 2 N per mm; ketchup comes out
only above 5 N, yet the 0.43 kg bottle must be held by the same walls, so the hold sits at about 4 N and the squeeze
at about 9.5 N) while the same hand carries the nozzle 16 cm along the sausage at about 3 cm/s, 1-2 cm above it and
within 6 mm of its crest. Flow and travel must match: too slow (or stopping) pools ketchup that runs off; too fast
leaves cells uncovered and breaks the line.

World frame (m): x forward from the arm bases, y to the robot's left, z up; the table top is at z 0.7475.
- paper plate: loose, 23 cm across, 2 mm thick, 12 g; centred at (0.190, -0.060).
- hot dog: loose, one rigid body (bun and sausage), 0.11 kg, on the plate, its long axis along x, centred at
  (0.190, -0.060). Sausage 160 mm long, 22 mm across, its crest 40 mm above the plate; bun 150 x 82 mm, two halves
  32 mm thick either side of the sausage, their tops 36 mm above the plate.
- ketchup bottle: loose, a 14 oz (397 g) upside-down squeeze bottle, 0.43 kg full. Standing on its cap (36 mm across,
  28 mm tall, the nozzle at its centre) at (0.170, -0.215); body 76 mm (along x) by 48 mm (along y), the bottle 175 mm
  tall from the nozzle to its base. The two wide faces (normal +-y) are squeezable walls: each is a panel that moves
  inward on a spring of 2 N/mm (up to 8 mm).

Ketchup (a deposition rule, as liquids are modelled with beads or rules in accepted tasks): each control step the
squeeze force F is the mean of the two walls' spring forces (from the walls' real displacement by the fingers). With
the bottle within 60 deg of nozzle-down and F above 5 N, ketchup leaves the nozzle at 0.15 ml/s per N above 5 N
(9.5 N: 0.68 ml/s) and falls straight down from the nozzle to the first surface under it, in the hot dog's frame:
- on the sausage within 6 mm of its crest: it adds to that 1 cm cell of the sausage's 16 cells; a cell holds at most
  0.6 ml (the line's ridge), and ketchup beyond that runs off onto the bun (off target);
- on the sausage's side (6 to 11 mm off its crest), on the bun, the plate or the table: off target.
A cell is covered when it holds 0.12 ml (a 4 mm bead along that centimetre). 0.1 ml off target counts as 0.4 cm^2 of
spot (2.5 mm thick).

Score:
- 0: more than 2.0 cm^2 of ketchup off target (0.5 ml).
- otherwise 0.8 x min(1, L / 14 cm), L = the longest unbroken run of covered cells (cm).
- 1.0 (success, held 1 s): L >= 14 cm, off target <= 0.5 cm^2, the bottle standing on its cap (within 15 deg of
  nozzle-down, cap within 1 cm of the table), released (squeeze under 1 N, every finger point 2 cm or more from it)
  and still, and the hot dog still on the plate (its middle within 6 cm of the plate's middle).
"""

from __future__ import annotations

import itertools
import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

from bodykit import Body, BoxShape, CapsuleShape, CylShape, Link, Linkage, allow_matrix_grips  # noqa: E402
from bodykit import as_numpy, on_reset, on_step, rotation  # noqa: E402
from yam_arena.demo import Waypoint, grip_rotation  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

allow_matrix_grips()

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"
MM = 0.001
DT = 0.05

# -- the layout ----------------------------------------------------------------------------------------------------
PLATE_C = (0.190, -0.060)
PLATE_R, PLATE_T = 0.115, 0.002
HD_C = (0.190, -0.060)
S_LEN, S_R = 0.160, 0.011  # sausage
S_TOP = 0.040  # crest above the plate
BUN_L, BUN_HALF_R, BUN_Y, BUN_Z = 0.150, 0.016, 0.025, 0.020  # bun halves: capsules along x
BUN_BASE = (0.150, 0.050, 0.014)
BOTTLE_C = (0.170, -0.215)
CAP_R, CAP_H = 0.018, 0.028
BODY_X, BODY_Y = 0.076, 0.048
BOTTLE_H = 0.175
WALL_Z0, WALL_Z1 = 0.080, BOTTLE_H  # the squeezable walls (bottle-local height above the nozzle)
WALL_T = 0.002
K_WALL = float(os.environ.get("KETCHUP_K", "2000"))  # N/m
WALL_TRAVEL = 0.008
CORE_Y = 0.028  # the stiff core's width (collision only): the fingers meet the walls before it

RED, SAUSAGE, BUN, BUN_TOP, WHITE, PAPER, CAPC, LABEL = (
    (0.85, 0.04, 0.03), (0.58, 0.30, 0.20), (0.90, 0.72, 0.45), (0.78, 0.50, 0.22), (0.96, 0.96, 0.95),
    (0.97, 0.97, 0.94), (0.95, 0.95, 0.95), (0.70, 0.08, 0.06))


def _v(p) -> tuple:
    return tuple(float(x) for x in p)


def CapsuleX(p1, p2, r, color, collide=True) -> CapsuleShape:
    """A capsule along x from ``p1`` to ``p2`` (the centres of its end caps)."""
    a, b = np.asarray(p1, float), np.asarray(p2, float)
    return CapsuleShape(_v((a + b) / 2), r, float(abs(b[0] - a[0])), color, collide=collide, axis="X")


def _plate() -> Body:
    return Body.build("plate", pos=(PLATE_C[0], PLATE_C[1], TOP + PLATE_T / 2 + 0.0005), mass=0.012, color=PAPER,
                      friction=(0.6, 0.5),
                      shapes=[CylShape((0.0, 0.0, 0.0), PLATE_R, PLATE_T, PAPER),
                              CylShape((0.0, 0.0, 0.0006), PLATE_R - 0.022, PLATE_T + 0.0002, (0.93, 0.93, 0.9),
                                       collide=False)])


def _hotdog() -> Body:
    """Origin on the plate's top under the sausage's middle; long axis along x."""
    sz = S_TOP - S_R
    hl = BUN_L / 2 - BUN_HALF_R
    sh = [BoxShape((0.0, 0.0, BUN_BASE[2] / 2), BUN_BASE, BUN),
          CapsuleX((-hl, BUN_Y, BUN_Z), (hl, BUN_Y, BUN_Z), BUN_HALF_R, BUN),
          CapsuleX((-hl, -BUN_Y, BUN_Z), (hl, -BUN_Y, BUN_Z), BUN_HALF_R, BUN),
          # browned tops of the bun halves (visual)
          CapsuleX((-hl + 0.004, BUN_Y + 0.003, BUN_Z + 0.003), (hl - 0.004, BUN_Y + 0.003, BUN_Z + 0.003),
                       BUN_HALF_R - 0.002, BUN_TOP, collide=False),
          CapsuleX((-hl + 0.004, -BUN_Y - 0.003, BUN_Z + 0.003), (hl - 0.004, -BUN_Y - 0.003, BUN_Z + 0.003),
                       BUN_HALF_R - 0.002, BUN_TOP, collide=False),
          CapsuleX((-S_LEN / 2 + S_R, 0.0, sz), (S_LEN / 2 - S_R, 0.0, sz), S_R, SAUSAGE)]
    return Body.build("hotdog", pos=(HD_C[0], HD_C[1], TOP + PLATE_T + 0.001), mass=0.11, color=BUN,
                      friction=(0.8, 0.7), shapes=sh)


NOZZLE_REL = np.array([0.0, 0.0, 0.0])  # the nozzle tip: the bottle's origin (bottom centre of the cap)


def _bottle() -> Linkage:
    """Origin at the nozzle tip (the cap's flat end, on the table at the start); bottle upside down."""
    bz0 = CAP_H + 0.012  # the body's lower end (after the shoulder)
    hb = BOTTLE_H - bz0
    base = [CylShape((0.0, 0.0, CAP_H / 2), CAP_R, CAP_H, CAPC),
            CylShape((0.0, 0.0, 0.0004), 0.003, 0.0010, (0.25, 0.05, 0.03), collide=False),  # the valve's slit
            # the flip lid, open, hanging beside the cap (visual)
            BoxShape((CAP_R + 0.003, 0.0, CAP_H * 0.55), (0.003, 0.030, 0.022), CAPC, collide=False),
            BoxShape((0.0, 0.0, CAP_H + 0.006), (0.050, 0.034, 0.012), RED),  # shoulder
            # the stiff core (collision) and the shell (visual) of the body
            BoxShape((0.0, 0.0, bz0 + hb / 2), (BODY_X, CORE_Y, hb), RED),
            BoxShape((0.0, 0.0, bz0 + hb / 2), (BODY_X, BODY_Y - 2 * WALL_T - 0.001, hb), RED, collide=False),
            BoxShape((0.0, 0.0, 0.112), (BODY_X + 0.0005, BODY_Y - 2 * WALL_T, 0.050), LABEL, collide=False)]
    wz = (WALL_Z0 + WALL_Z1) / 2
    wh = WALL_Z1 - WALL_Z0
    yw = BODY_Y / 2 - WALL_T / 2
    links = []
    for name, sgn in (("wall_p", 1.0), ("wall_n", -1.0)):
        shapes = (BoxShape((0.0, sgn * yw, wz), (BODY_X - 0.006, WALL_T, wh), RED),
                  BoxShape((0.0, sgn * (yw + 0.0006), 0.112), (BODY_X - 0.010, 0.0004, 0.046), WHITE, collide=False))
        lim = (-WALL_TRAVEL, 0.0) if sgn > 0 else (0.0, WALL_TRAVEL)
        links.append(Link(name, shapes, mass=0.012, anchor=(BOTTLE_C[0], BOTTLE_C[1] + sgn * yw, TOP + wz), axis="Y",
                          slide=True, limits=lim, stiffness=K_WALL, spring_damping=40.0, damping=5.0))
    return Linkage.build("bottle", pos=(BOTTLE_C[0], BOTTLE_C[1], TOP + 0.0008), mass=0.40, color=RED, floating=True,
                         friction=(1.0, 0.9), shapes=base, links=links, contact_offset=0.002, solver_iterations=48)


PLATE = _plate()
HOTDOG = _hotdog()
BOTTLE = _bottle()

# -- the ketchup (visual only, nothing collides with it) -------------------------------------------------------------
CELLS = 16
CELL = S_LEN / CELLS
CREST = 0.006  # on the sausage within this of its crest
CAP_ML = 0.6  # a cell's ridge holds this much
COVER_ML = 0.12  # a cell holding this is covered
F_OPEN = float(os.environ.get("KETCHUP_FOPEN", "5.0"))  # N: the valve opens
Q_PER_N = 0.15  # ml/s per N above F_OPEN
INVERT_DEG = 60.0
SPOT_CM2_PER_ML = 4.0  # 0.1 ml off target: a 0.4 cm^2 spot (2.5 mm thick)
OFF_ZERO = 2.0  # cm^2
OFF_FULL = 0.5  # cm^2
FULL_CM = 14
OFF_N = 12  # off-target spots drawn (each 0.1 ml)
OFF_ML = 0.1
HIDE = np.array([0.40, 0.30, TOP - 0.30])
LINE_VIS = [Body.build(f"line_{k}", pos=_v(HIDE + np.array([0.0, 0.0, -0.01 * k])), mass=0.001, color=RED,
                       gravity=False, damping=(5.0, 5.0),
                       shapes=[CapsuleX((-CELL / 2 - 0.0005, 0.0, 0.0), (CELL / 2 + 0.0005, 0.0, 0.0), 0.0028,
                                            RED, collide=False)]) for k in range(CELLS)]
OFF_VIS = [Body.build(f"spot_{k}", pos=_v(HIDE + np.array([0.02, 0.0, -0.01 * k])), mass=0.001, color=RED,
                      gravity=False, damping=(5.0, 5.0),
                      shapes=[CylShape((0.0, 0.0, 0.0), 0.0036, 0.0016, RED, collide=False)]) for k in range(OFF_N)]
OBJECTS = [PLATE, HOTDOG, BOTTLE] + LINE_VIS + OFF_VIS

# -- tracking and scoring -------------------------------------------------------------------------------------------
CLEAR = 0.02
STILL_STEPS, STILL_MM = 20, 1.0
STAND_DEG = 15.0
_DEBUG = os.environ.get("KETCHUP_DEBUG") == "1"
_DEBUG_EVERY = int(os.environ.get("KETCHUP_DEBUG_EVERY", "5"))
_EP: dict = {}
_IDS: dict = {}


@on_reset
def _reset() -> None:
    _EP.update(tick=0, step=0, counted=-1, cells=[0.0] * CELLS, cell_y=[[] for _ in range(CELLS)], off=0.0,
               out=0.0, off_shown=0, line_shown=set(), off_pending=0.0, recent=[], force=0.0, peak_f=0.0, tilt=0.0,
               near=1.0, gaps={}, tcp={}, where="-", noz_h=0.0, last_off=None)


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


def _put(base, name, p, yaw) -> None:
    import torch

    pose = np.array([[p[0], p[1], p[2], 0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)]])
    obj = base.scene[name]
    obj.write_root_link_pose_to_sim_index(root_pose=torch.tensor(pose, dtype=torch.float32, device=base.device))
    obj.write_root_com_velocity_to_sim_index(root_velocity=torch.zeros((1, 6), dtype=torch.float32,
                                                                       device=base.device))


def _land(p, hd_pos, hd_rot, pl_pos):
    """Where ketchup falling straight down from world point ``p`` lands: (kind, cell, y_in_hotdog, surface point)."""
    q = hd_rot.T @ (p - hd_pos)  # in the hot dog's frame (origin on the plate under the sausage's middle)
    x, y = float(q[0]), float(q[1])
    if abs(x) <= S_LEN / 2 and abs(y) <= S_R:
        zs = S_TOP - S_R + math.sqrt(max(0.0, S_R ** 2 - y ** 2))
        if abs(y) <= CREST and q[2] > zs - 0.002:
            cell = min(CELLS - 1, max(0, int((x + S_LEN / 2) / CELL)))
            return "sausage", cell, y, hd_pos + hd_rot @ np.array([x, y, zs])
        return "side", None, y, hd_pos + hd_rot @ np.array([x, y + math.copysign(0.004, y), BUN_Z + 0.010])
    if abs(x) <= BUN_L / 2 and abs(y) <= BUN_Y + BUN_HALF_R:
        return "bun", None, y, hd_pos + hd_rot @ np.array([x, y, BUN_Z + BUN_HALF_R * 0.8])
    if float(np.linalg.norm((p - pl_pos)[:2])) <= PLATE_R:
        return "plate", None, y, np.array([p[0], p[1], pl_pos[2] + PLATE_T / 2 + 0.0008])
    return "table", None, y, np.array([p[0], p[1], TOP + 0.0008])


@on_step
def _track(env) -> None:
    try:
        _track_inner(env)
    except Exception:
        import traceback

        print("KETCHUP_TRACEBACK\n" + traceback.format_exc(), flush=True)
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
        poses = as_numpy(robot.data.body_link_pose_w)[0]
        fingers[side] = np.concatenate([_IDS["fingers"][n] @ rotation(poses[i][3:7]).T + poses[i][:3] - origin
                                        for n, i in zip(("tip_left", "tip_right"), _IDS[side + "_tips"])])
    bot = base.scene["bottle"]
    if "walls" not in _IDS:
        jn = list(bot.joint_names)
        _IDS["walls"] = [jn.index("wall_p_turn"), jn.index("wall_n_turn")]
    jq = as_numpy(bot.data.joint_pos)[0]
    disp = [abs(float(jq[i])) for i in _IDS["walls"]]
    force = K_WALL * sum(disp) / 2.0
    bp = as_numpy(bot.data.root_link_pose_w)[0]
    b_pos, b_rot = bp[:3] - origin, rotation(bp[3:7])
    up = b_rot @ np.array([0.0, 0.0, 1.0])  # nozzle -> base; straight up when standing on the cap
    tilt = math.degrees(math.acos(max(-1.0, min(1.0, float(up[2])))))
    nozzle = b_pos + b_rot @ NOZZLE_REL
    hd = as_numpy(base.scene["hotdog"].data.root_link_pose_w)[0]
    hd_pos, hd_rot = hd[:3] - origin, rotation(hd[3:7])
    pl = as_numpy(base.scene["plate"].data.root_link_pose_w)[0]
    pl_pos = pl[:3] - origin
    hd_yaw = math.atan2(hd_rot[1, 0], hd_rot[0, 0])
    kind, cell, yh, surf = _land(nozzle, hd_pos, hd_rot, pl_pos)
    flow = 0.0
    if tilt <= INVERT_DEG and force > F_OPEN:
        flow = Q_PER_N * (force - F_OPEN) * DT  # ml this step
    if flow > 0.0:
        _EP["out"] += flow
        if kind == "sausage":
            room = max(0.0, CAP_ML - _EP["cells"][cell])
            keep = min(room, flow)
            _EP["cells"][cell] += keep
            _EP["cell_y"][cell].append(yh)
            spill = flow - keep
            if spill > 0.0:  # the pool runs off the sausage onto the bun beside it
                _EP["off"] += spill
                _EP["off_pending"] += spill
                xr = -S_LEN / 2 + (cell + 0.5) * CELL
                side = 1.0 if (_EP["off_shown"] % 2 == 0) else -1.0
                _EP["last_off"] = hd_pos + hd_rot @ np.array([xr, side * (BUN_Y - 0.004), BUN_Z + BUN_HALF_R * 0.85])
        else:
            _EP["off"] += flow
            _EP["off_pending"] += flow
            _EP["last_off"] = surf
    # show what has landed
    for k in range(CELLS):
        if k not in _EP["line_shown"] and _EP["cells"][k] >= COVER_ML:
            _EP["line_shown"].add(k)
            xr = -S_LEN / 2 + (k + 0.5) * CELL
            yr = float(np.clip(np.mean(_EP["cell_y"][k]), -CREST, CREST))
            zs = S_TOP - S_R + math.sqrt(max(0.0, S_R ** 2 - yr ** 2))
            _put(base, f"line_{k}", hd_pos + hd_rot @ np.array([xr, yr, zs + 0.0012]) + origin, hd_yaw)
    while _EP["off_pending"] >= OFF_ML and _EP["off_shown"] < OFF_N and _EP["last_off"] is not None:
        _EP["off_pending"] -= OFF_ML
        k = _EP["off_shown"]
        _EP["off_shown"] += 1
        jit = np.array([0.006 * math.cos(2.4 * k), 0.006 * math.sin(2.4 * k), 0.0]) * min(1.0, k / 2)
        _put(base, f"spot_{k}", np.asarray(_EP["last_off"]) + jit + origin, 0.0)
    pts = b_pos + (b_rot @ np.array([[x, y, z] for x in (-BODY_X / 2, 0.0, BODY_X / 2)
                                     for y in (-BODY_Y / 2, BODY_Y / 2)
                                     for z in np.linspace(0.0, BOTTLE_H, 8)]).T).T
    tree = cKDTree(pts)
    near = min(float(tree.query(fp, k=1)[0].min()) for fp in fingers.values())
    _EP.update(force=force, peak_f=max(_EP["peak_f"], force), tilt=tilt, near=near, where=kind, b_pos=b_pos,
               nozzle=nozzle, noz_h=float(nozzle[2] - surf[2]), yh=yh, hd_pos=hd_pos, pl_pos=pl_pos)


def _run_cm() -> int:
    best = cur = 0
    for v in _EP["cells"]:
        cur = cur + 1 if v >= COVER_ML else 0
        best = max(best, cur)
    return best


def _tally(o) -> dict:
    recent = _EP["recent"]
    if _EP["counted"] != _EP["tick"] and "b_pos" in _EP:
        _EP["counted"] = _EP["tick"]
        recent.append(np.asarray(_EP["b_pos"], float).copy())
        del recent[:-STILL_STEPS]
    still = len(recent) == STILL_STEPS and all(np.abs(p - recent[-1]).max() * 1000 < STILL_MM for p in recent)
    off_cm2 = _EP["off"] * SPOT_CM2_PER_ML
    run = _run_cm()
    standing = _EP["tilt"] <= STAND_DEG and float(_EP.get("b_pos", np.ones(3))[2]) < TOP + 0.010
    on_plate = float(np.linalg.norm((_EP.get("hd_pos", np.zeros(3)) - _EP.get("pl_pos", np.ones(3)))[:2])) <= 0.06
    done = bool(run >= FULL_CM and off_cm2 <= OFF_FULL and standing and _EP["force"] < 1.0 and _EP["near"] >= CLEAR
                and still and on_plate)
    return dict(still=still, off=off_cm2, run=run, standing=standing, on_plate=on_plate, done=done)


def _debug(t) -> None:
    _EP["step"] += 1
    if (_EP["step"] - 1) % _DEBUG_EVERY:
        return
    bp = _EP.get("b_pos", np.zeros(3))
    nz = _EP.get("nozzle", np.zeros(3))
    print(f"DBG step={_EP['step']} F={_EP['force']:.2f}N peak={_EP['peak_f']:.2f} tilt={_EP['tilt']:.1f} "
          f"out={_EP['out']:.3f}ml off={_EP['off']:.3f}ml ({t['off']:.2f}cm2) run={t['run']}cm "
          f"cells=[{' '.join(f'{c:.2f}' for c in _EP['cells'])}] where={_EP['where']} yh={_EP.get('yh', 0) * 1000:.1f}mm "
          f"noz=({nz[0]:.3f},{nz[1]:.3f},{nz[2] - TOP:.3f}) noz_h={_EP['noz_h'] * 1000:.0f}mm "
          f"bottle=({bp[0]:.3f},{bp[1]:.3f},{bp[2] - TOP:.3f}) near={_EP['near'] * 1000:.0f}mm "
          f"stand={int(t['standing'])} still={int(t['still'])} plate={int(t['on_plate'])} | "
          + " ".join(f"{k} tcp=({v[0]:.3f},{v[1]:.3f},{v[2] - TOP:.3f}) gap={_EP['gaps'][k] * 1000:.1f}"
                     for k, v in _EP["tcp"].items()), flush=True)


def score(objects) -> float:
    """0 with more than 2.0 cm^2 off target; otherwise 0.8 x min(1, L / 14 cm). Full score is success()."""
    t = _tally(objects)
    if _DEBUG:
        _debug(t)
    if t["off"] > OFF_ZERO:
        return 0.0
    return 0.8 * min(1.0, t["run"] / FULL_CM)


def success(objects) -> bool:
    return bool(_tally(objects)["done"])


# -- scripted demo --------------------------------------------------------------------------------------------------
# Open loop, from the known layout; the left arm stays at the ready pose. The right gripper comes down over the
# bottle, fingers closing across its 48 mm (along y), and holds it with a light grip (about 4 N on the walls, below the
# valve's 5 N). It lifts the bottle, carries it over the near end of the sausage with the nozzle aimed 2.2 cm above
# the crest (it sits about 1.2 cm above it: the arm sags under the load), squeezes (about 9.5 N) and carries it along the sausage at 3 cm/s, lets the squeeze off just before the far
# end, lifts, carries the bottle back, stands it on its cap, lets go and rises.
# KETCHUP_MODE=pool (wrong way): squeezes for 6 s over the middle of the sausage without moving.
OPEN_C = -0.047
MODE = os.environ.get("KETCHUP_MODE", "demo")
HOLD = float(os.environ.get("KETCHUP_HOLD", "-0.0213"))  # finger value: light hold on the 48 mm walls
SQUEEZE = float(os.environ.get("KETCHUP_SQUEEZE", "-0.0172"))  # finger value: the squeeze
GRIP_H = float(os.environ.get("KETCHUP_GRIP_H", "0.128"))  # TCP above the nozzle when holding
NOZ_ABOVE = float(os.environ.get("KETCHUP_NOZ", "0.022"))  # nozzle above the crest while squeezing
SPEED = float(os.environ.get("KETCHUP_SPEED", "0.030"))  # m/s along the sausage
X_START = float(os.environ.get("KETCHUP_X0", "-0.076"))  # along the sausage from its middle
X_STOP = float(os.environ.get("KETCHUP_X1", "0.070"))
Y_OFF = float(os.environ.get("KETCHUP_YOFF", "0.0"))
LAG = float(os.environ.get("KETCHUP_LAG", "0.004"))  # aim this far ahead along the travel (the arm trails)
ROT = grip_rotation(0.0)  # fingers close along world y


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


def _w(t, p, finger, label, rot=ROT) -> Waypoint:
    return Waypoint(t, tuple(float(v) for v in p), finger, label, rot)


def _stay(arm: str, t: float = 120.0) -> list:
    from yam_arena import ik, station
    from yam_arena.constants import READY

    pos, rot = ik.tcp_pose(np.array(READY, float), station.BASES[arm])
    return [Waypoint(t, tuple(float(v) for v in pos), OPEN_C, "stay put", rot)]


def _tcp(nx, ny, nz) -> np.ndarray:
    """The TCP holding the bottle with its nozzle at (nx, ny, nz above the table)."""
    return np.array([nx, ny, TOP + nz + GRIP_H])


def _right() -> list:
    bx, by = BOTTLE_C
    crest = PLATE_T + S_TOP  # the sausage's crest above the table
    nz = crest + NOZ_ABOVE
    hx, hy = HD_C[0], HD_C[1] + Y_OFF
    carry = nz + 0.012
    out = _lead_in("right", _tcp(bx, by, 0.05), ROT, OPEN_C, "above the bottle, open")
    out += [_w(2.0, _tcp(bx, by, 0.0015), OPEN_C, "lower round the bottle"),
            _w(0.5, _tcp(bx, by, 0.0015), OPEN_C, "settle"),
            _w(1.2, _tcp(bx, by, 0.0015), HOLD, "hold it lightly"),
            _w(3.0, _tcp(bx, by, carry), HOLD, "lift it"),
            _w(3.0, _tcp(hx + X_START, hy, carry), HOLD, "carry it over the near end of the sausage"),
            _w(1.0, _tcp(hx + X_START, hy, nz), HOLD, "lower the nozzle to 2 cm above the crest"),
            _w(0.6, _tcp(hx + X_START, hy, nz), HOLD, "settle")]
    if MODE == "pool":  # wrong: squeeze over the middle without moving
        out += [_w(2.0, _tcp(hx, hy, nz), HOLD, "move over the middle"),
                _w(0.3, _tcp(hx, hy, nz), SQUEEZE, "squeeze"),
                _w(6.0, _tcp(hx, hy, nz), SQUEEZE, "keep squeezing in one place"),
                _w(0.4, _tcp(hx, hy, nz), HOLD, "let off")]
        x_end = hx
    else:
        travel = X_STOP - X_START
        n = max(4, int(travel / 0.01))
        out += [_w(0.25, _tcp(hx + X_START, hy, nz), SQUEEZE, "squeeze")]
        for k in range(1, n + 1):
            x = hx + X_START + travel * k / n + LAG
            out.append(_w(travel / SPEED / n, _tcp(x, hy, nz), SQUEEZE, "squeeze, carrying it along the sausage"))
        out += [_w(0.3, _tcp(hx + X_STOP + LAG + 0.004, hy, nz), HOLD, "let the squeeze off")]
        x_end = hx + X_STOP + LAG + 0.004
    out += [_w(1.0, _tcp(x_end - 0.02, hy, carry), HOLD, "lift"),
            _w(3.0, _tcp(bx, by, 0.02), HOLD, "carry it back"),
            _w(1.5, _tcp(bx, by, 0.001), HOLD, "stand it on its cap"),
            _w(0.6, _tcp(bx, by, 0.001), HOLD, "settle"),
            _w(1.0, _tcp(bx, by, 0.001), OPEN_C, "let go"),
            _w(2.0, _tcp(bx, by, 0.06), OPEN_C, "rise clear"),
            _w(8.0, _tcp(bx - 0.06, by, 0.06), OPEN_C, "stay clear")]
    return out


def demo():
    return {"right": _right(), "left": _stay("left")}
