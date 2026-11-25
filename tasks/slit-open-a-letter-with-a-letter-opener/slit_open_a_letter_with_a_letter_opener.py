"""On the station table: a sealed #10 business envelope (241 x 105 mm) with a letter folded in thirds inside lies flat
on the desk, flap side up, its top fold (the edge the flap folds over) along y on the far side. A steel letter opener
(229 mm, 9 in) lies on the desk beyond the fold, pointing left. Task (instruction): open the envelope along its top
edge with the letter opener without cutting the letter inside, then put the opener down on the desk.

How it is done (the usual way a letter opener is used, e.g. Merriam-Webster: "a knife for slitting envelopes"; office
mail-handling guides: "insert the tip of the opener into the corner where the flap does not reach, then slide it along
the top edge"): one hand holds the envelope flat; the other slides the blade's tip into the small gap at the flap's
corner, between the two layers, and draws the blade along the top fold so that its edge slits the fold from inside,
to the far corner. The letter sits below the fold (a letter folded in thirds is 93 mm tall in a 105 mm envelope), so
the tip must stay in the 12 mm band above the letter's top edge: pushed deeper, it cuts the letter.

The hand skill: a 1.5 mm blade tip into a 12 mm wide, 4 mm tall corner gap; the blade held at a shallow angle across
the fold, edge outward, its tip 5-7 mm inside the fold, for the whole 241 mm; and the other hand pinning the 80 g
envelope, which slides away with the blade otherwise (the fold resists the blade with 1.5 N).

World frame (m): x forward from the arm bases, y to the robot's left, z up; the table top is at z 0.7475.
- envelope: loose (a floating articulation). 241 mm along y, 105 mm along x, 5 mm thick (a letter of about 12 sheets
  folded in thirds inside), centred at (0.190, 0.000); the top fold at x 0.2425. Two layers 0.5 mm thick; between them,
  the letter (a link on a stiff slide, 3 N/mm within 1 mm, with a compliant 3 N/mm contact: the give of
  a paper stack; 4 mm thick) from the bottom edge to 12 mm below the fold. Above the letter's top
  edge a band 12 mm wide and 4 mm tall is empty and open at both ends (the flap's corners are not glued: the side
  seams stop 12 mm below the fold). The fold and the top layer's last 14 mm next to it
  are 24 pieces 10 mm long (links on hinges along y, sprung shut), so a slit fold can open.
- letter opener: loose, 40 g. Handle 105 x 15 x 9 mm; a steel blade 124 mm long, 1.5 mm thick, 16 mm wide at its root
  tapering to a 3 mm point; lying on the desk with its handle's middle at (0.300, -0.250) and the point toward +y.

Paper cutting is a stated rule (the paper is rigid panels): a 10 mm piece of the fold is slit when the blade's tip is
inside the envelope's band, the piece before it (toward the start corner) is already slit (the first piece is at the
start corner's gap), and the blade pushes on that piece with 1.5 N or more (PhysX contact force). The slit piece then
turns up 25 deg about its hinge, opening the slit. The letter is cut when the blade pushes on the letter with 1.0 N or
more (PhysX contact force).

Score (the best stage reached; 0 for the rest of the run once the letter is cut):
- 0.25: at least 6 of the 24 fold pieces slit (a quarter of the edge).
- 0.5: at least 12 slit (half).
- 0.75: all 24 slit (the whole top edge, corner to corner).
- 1.0 (success, held 1 s): all 24 slit and the opener put down: resting on the desk (its lowest point within 5 mm of
  the table top), still, and every finger point 1 cm or more from it.
"""

from __future__ import annotations

import itertools
import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

from bodykit import Body, BoxShape, Link, Linkage, allow_matrix_grips  # noqa: E402
from bodykit import as_numpy, on_reset, on_step, rotation  # noqa: E402
from yam_arena.demo import Waypoint, grip_rotation  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

allow_matrix_grips()

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"
_E = os.environ.get

# -- the layout (world, m) -------------------------------------------------------------------------------------------
EC = (0.190, 0.000)  # the envelope's centre (x, y)
ENV_L, ENV_W = 0.241, 0.105  # #10 envelope: 241 mm along y, 105 mm along x
X_F = ENV_W / 2  # the top fold, from the centre (+x side)
SKIN = 0.0005  # each paper layer (rigid stand-in)
CAV = 0.004  # the space between the layers (the letter's thickness)
ENV_T = 2 * SKIN + CAV  # 5 mm
BAND = 0.012  # empty band between the letter's top edge and the fold
GAP = 0.012  # the unglued corner gap at each end, below the fold (the whole band above the letter)
LIP_W = 0.014  # the hinged strip of the top layer next to the fold
N_SEG = 24
SEG = ENV_L / N_SEG  # 10.04 mm
LIP_OPEN = math.radians(25.0)
F_CUT = float(_E("LETTER_FCUT", "1.5"))  # N on a fold piece to slit it
F_LETTER = 1.0  # N on the letter: cut
K_LIP = 0.4  # N m/rad, the spring that holds a fold piece shut until slit
LETTER_W = 0.216  # 8.5 in letter, folded in thirds: 216 x 93 mm

OP0 = (0.300, -0.250)  # the opener's handle middle, lying on the desk
OP_YAW0 = math.pi / 2  # pointing +y
H_L, H_W, H_T = 0.105, 0.015, 0.009  # handle
B_T = 0.0015  # blade thickness
# blade pieces along the opener's axis (from the handle's middle): (start, end, width)
BLADE = ((0.0525, 0.095, 0.016), (0.095, 0.130, 0.0125), (0.130, 0.150, 0.009), (0.150, 0.165, 0.006),
         (0.165, 0.1765, 0.003))
TIP_S = 0.1765  # the point, from the handle's middle

PAPER, INSIDE, LINE, LETTER_C = (0.93, 0.92, 0.88), (0.45, 0.44, 0.42), (0.70, 0.68, 0.62), (0.98, 0.98, 0.98)
STEEL, WOOD = (0.78, 0.79, 0.81), (0.40, 0.22, 0.10)


def _v(p) -> tuple:
    return tuple(float(x) for x in p)


def _seg_y(k: int) -> float:
    return -ENV_L / 2 + (k + 0.5) * SEG


def _envelope() -> Linkage:
    cx, cy = EC
    zb, zt = SKIN / 2, SKIN + CAV + SKIN / 2
    base = [BoxShape((0.0, 0.0, zb), (ENV_W, ENV_L, SKIN), PAPER),  # the address side, on the desk
            # the band above the letter's edge, seen through a slit: darker
            BoxShape((X_F - BAND / 2, 0.0, SKIN + 0.00005), (BAND, ENV_L - 0.002, 0.0001), INSIDE, collide=False),
            # the top (flap side) layer, up to the hinged strip
            BoxShape((-LIP_W / 2, 0.0, zt), (ENV_W - LIP_W, ENV_L, SKIN), PAPER),
            # the bottom edge (a fold too) and the side seams; the side seams leave the corner gaps open
            BoxShape((-X_F + SKIN / 2, 0.0, SKIN + CAV / 2), (SKIN, ENV_L, CAV), PAPER)]
    side_len = ENV_W - GAP
    for sy in (1, -1):
        base.append(BoxShape((-X_F + side_len / 2, sy * (ENV_L / 2 - SKIN / 2), SKIN + CAV / 2),
                             (side_len, SKIN, CAV), PAPER))
    # the flap's outline on the top layer (drawn only): a V from the fold's ends to 55 mm below the fold
    for sy in (1, -1):
        a = np.array([X_F - LIP_W, sy * ENV_L / 2])
        b = np.array([X_F - 0.055, 0.0])
        d = b - a
        ln = float(np.linalg.norm(d))
        base.append(BoxShape((float((a[0] + b[0]) / 2), float((a[1] + b[1]) / 2), ENV_T + 0.0001),
                             (ln, 0.0012, 0.0001), LINE, yaw=math.atan2(d[1], d[0]), collide=False))
    base.append(BoxShape((X_F - LIP_W, 0.0, ENV_T + 0.0001), (0.0008, ENV_L, 0.0001), LINE, collide=False))
    links = [Link("letter", (BoxShape((-X_F + 0.003 + (ENV_W - BAND - 0.003) / 2 - 0.0, 0.0, SKIN + CAV / 2),
                                      (ENV_W - BAND - 0.003, LETTER_W, CAV - 0.0002), LETTER_C),),
                  mass=0.040, anchor=(cx, cy, TOP + 0.0005 + SKIN + CAV / 2), axis="X", slide=True,
                  limits=(-0.001, 0.001), stiffness=3000.0, spring_damping=5.0, report_contacts=True,
                  compliant=(3000.0, 10.0))]
    hz = TOP + 0.0005 + ENV_T
    for k in range(N_SEG):
        y = _seg_y(k)
        sh = (BoxShape((X_F - LIP_W / 2, y, zt), (LIP_W, SEG - 0.0002, SKIN), PAPER),
              BoxShape((X_F - SKIN / 2, y, SKIN + CAV / 2), (SKIN, SEG - 0.0002, CAV), PAPER))
        links.append(Link(f"lip{k:02d}", sh, mass=0.0008, anchor=(cx + X_F - LIP_W, cy + y, hz), axis="Y",
                          limits=(-30.0, 0.5), stiffness=K_LIP, spring_damping=0.002, report_contacts=True))
    return Linkage.build("envelope", pos=(cx, cy, TOP + 0.0005), mass=0.020, color=PAPER, floating=True,
                         friction=(0.45, 0.4), shapes=base, links=links, contact_offset=0.001, solver_iterations=48)


def _opener() -> Body:
    sh = [BoxShape((0.0, 0.0, 0.0), (H_L, H_W, H_T), WOOD)]
    for a, b, w in BLADE:
        sh.append(BoxShape(((a + b) / 2, 0.0, 0.0), (b - a, w, B_T), STEEL))
    return Body.build("opener", pos=(OP0[0], OP0[1], TOP + H_T / 2 + 0.001), mass=0.040, yaw=OP_YAW0, color=STEEL,
                      friction=(0.5, 0.4), shapes=sh, contact_offset=0.001, solver_iterations=32)


ENVELOPE = _envelope()
OPENER = _opener()
OBJECTS = [ENVELOPE, OPENER]

# -- tracking and scoring --------------------------------------------------------------------------------------------
CLEAR = 0.010
_DEBUG = _E("LETTER_DEBUG") == "1"
_DEBUG_EVERY = int(_E("LETTER_DEBUG_EVERY", "5"))
_EP: dict = {}
_IDS: dict = {}
_CV: dict = {}
# points on the opener (its frame): the handle's corners and the blade's edges
OP_SAMPLES = np.array([[sx * H_L / 2, sy * H_W / 2, sz * H_T / 2] for sx in (-1, 0, 1) for sy in (-1, 1)
                       for sz in (-1, 1)] + [[s, sy * w / 2, sz * B_T / 2] for a, b, w in BLADE for s in (a, b)
                                             for sy in (-1, 1) for sz in (-1, 1)] + [[TIP_S, 0.0, 0.0]])


@on_reset
def _reset() -> None:
    _EP.update(tick=0, step=0, cut=[False] * N_SEG, letter_cut=False, f_lip=np.zeros(N_SEG), f_letter=0.0,
               tip_in=False, tip_env=np.zeros(3), low=1.0, still=False, near=1.0, prev=None, tcp={}, gaps={},
               env_shift=0.0, op=(np.zeros(3), 0.0, 0.0))


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


def _contact_forces(base) -> np.ndarray:
    """|force| (N) between the opener and each fold piece, then the letter, from PhysX's pair-wise contact forces."""
    from isaaclab_physx.physics import PhysxManager

    sim = PhysxManager.get_physics_sim_view()
    view = _CV.get("view") if _CV.get("sim") is sim else None
    if view is None:
        env = base.scene.env_prim_paths[0]
        sensors = [f"{env}/envelope/lip{k:02d}" for k in range(N_SEG)] + [f"{env}/envelope/letter"]
        view = sim.create_rigid_contact_view(sensors, filter_patterns=[[f"{env}/opener/body"]] * len(sensors),
                                             max_contact_data_count=256)
        _CV.update(sim=sim, view=view, dt=float(base.physics_dt))
    forces = view.get_contact_force_matrix(dt=_CV["dt"])
    a = forces.numpy() if hasattr(forces, "numpy") else as_numpy(forces)
    return np.linalg.norm(np.asarray(a, float).reshape(N_SEG + 1, -1, 3), axis=-1).max(axis=1)


@on_step
def _track(env) -> None:
    try:
        _track_inner(env)
    except Exception:
        import traceback

        print("LETTER_TRACEBACK\n" + traceback.format_exc(), flush=True)
        raise


def _track_inner(env) -> None:
    import torch
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
    env_a = base.scene["envelope"]
    if "lips" not in _IDS:
        jn = list(env_a.joint_names)
        _IDS["lips"] = [jn.index(f"lip{k:02d}_turn") for k in range(N_SEG)]
    ep = as_numpy(env_a.data.root_link_pose_w)[0]
    epos, er = ep[:3] - origin, rotation(ep[3:7])
    op = as_numpy(base.scene["opener"].data.root_link_pose_w)[0]
    opos, orr = op[:3] - origin, rotation(op[3:7])
    tip = opos + orr @ np.array([TIP_S, 0.0, 0.0])
    t_env = er.T @ (tip - epos)  # the point in the envelope's frame (origin at its centre, on the desk)
    tip_in = bool(X_F - BAND - 0.002 <= t_env[0] <= X_F and -0.0005 <= t_env[2] <= ENV_T
                  and -ENV_L / 2 - 0.002 <= t_env[1] <= ENV_L / 2 + 0.03)
    f = _contact_forces(base)
    cut = _EP["cut"]
    newly = []
    if tip_in:
        for k in range(N_SEG):
            if not cut[k] and (k == 0 or cut[k - 1]) and f[k] >= F_CUT:
                cut[k] = True
                newly.append(k)
                break  # one piece per step
    if f[N_SEG] >= F_LETTER:
        _EP["letter_cut"] = True
    if newly or _EP["tick"] == 1:
        tgt = [[-LIP_OPEN if c else 0.0 for c in cut]]
        env_a.set_joint_position_target_index(target=torch.tensor(tgt, dtype=torch.float32, device=env_a.device),
                                              joint_ids=_IDS["lips"])
        env_a.write_data_to_sim()
    pts = opos + OP_SAMPLES @ orr.T
    low = float(pts[:, 2].min() - TOP)
    still = _EP["prev"] is not None and float(np.linalg.norm(opos - _EP["prev"])) < 0.0005
    near = min(float(cKDTree(pts).query(fp, k=1)[0].min()) for fp in fingers.values())
    yaw = math.degrees(math.atan2(orr[1, 0], orr[0, 0]))
    pitch = math.degrees(math.asin(max(-1.0, min(1.0, float(orr[2, 0])))))
    _EP.update(op=(opos, yaw, pitch))
    _EP.update(f_lip=f[:N_SEG], f_letter=float(f[N_SEG]), tip_in=tip_in, tip_env=t_env, low=low, still=still,
               near=near, prev=opos, env_shift=float(np.linalg.norm(epos[:2] - np.array(EC))))


def _tally(o) -> dict:
    n = int(sum(_EP["cut"]))
    down = bool(_EP["low"] <= 0.005 and _EP["still"] and _EP["near"] >= CLEAR)
    return dict(n=n, down=down, done=bool(n == N_SEG and down and not _EP["letter_cut"]))


def _debug(t) -> None:
    _EP["step"] += 1
    if (_EP["step"] - 1) % _DEBUG_EVERY:
        return
    k = int(np.argmax(_EP["f_lip"]))
    te = _EP["tip_env"]
    print(f"DBG step={_EP['step']} slit={t['n']}/{N_SEG} letter_cut={int(_EP['letter_cut'])} "
          f"f_letter={_EP['f_letter']:.2f} fmax=lip{k:02d}:{_EP['f_lip'][k]:.2f} tip_in={int(_EP['tip_in'])} "
          f"tip_env=({te[0] * 1000:+.1f},{te[1] * 1000:+.1f},{te[2] * 1000:+.1f})mm low={_EP['low'] * 1000:.1f}mm "
          f"near={_EP['near'] * 1000:.0f}mm op=({_EP['op'][0][0]:.4f},{_EP['op'][0][1]:.4f},{(_EP['op'][0][2] - TOP) * 1000:.1f}mm,"
          f"yaw={_EP['op'][1]:.1f},pitch={_EP['op'][2]:.1f}) env_shift={_EP['env_shift'] * 1000:.1f}mm down={int(t['down'])} | "
          + " ".join(f"{s} tcp=({v[0]:.3f},{v[1]:.3f},{(v[2] - TOP) * 1000:.1f}mm) gap={_EP['gaps'][s] * 1000:.0f}"
                     for s, v in _EP["tcp"].items()), flush=True)


def score(objects) -> float:
    t = _tally(objects)
    if _DEBUG:
        _debug(t)
    if _EP["letter_cut"]:
        return 0.0
    n = t["n"]
    if n == N_SEG:
        return 1.0 if t["done"] else 0.75
    if n >= N_SEG // 2:
        return 0.5
    if n >= N_SEG // 4:
        return 0.25
    return 0.0


def success(objects) -> bool:
    return bool(_tally(objects)["done"])


# -- scripted demo ---------------------------------------------------------------------------------------------------
# Open loop, from the known layout. The left gripper, closed, presses the envelope onto the desk near its bottom edge
# (about 5 N) and holds it there while the right hand works. The right gripper takes the opener by its handle from
# above, lifts it and turns it so that its blade lies across the fold at 10 deg, point inward and tilted 2.4 deg down,
# the point level with the middle of the gap; it slides the point into the corner gap at the start (-y) end and draws
# the opener along +y the full 241 mm, the blade's outer edge on the fold 15 mm behind the point (the point 5-7 mm
# inside the fold, the letter's edge 12 mm inside), and on past the far corner, where the point comes out through the
# other gap. It lifts the opener clear, lays it on the desk beyond the envelope and lets go; both arms go back.
# LETTER_DEEP=1 (the wrong way): from 4 cm along, over the next 4 cm, the blade is moved 15 mm deeper into the envelope, so its point
# runs into the letter's top edge: the letter is cut, which scores 0.
# LETTER_NOPIN=1 (one hand): the same right-hand moves with the left arm left at its ready pose; the envelope is not held.
OPEN_C = -0.047
DEEP = _E("LETTER_DEEP") == "1"
NOPIN = _E("LETTER_NOPIN") == "1"
P = {k: float(_E("LETTER_" + k.upper(), v)) for k, v in dict(
    alpha="10", out="0.004", pitch="2.4", cut_s="0.015", tip_z="0.0025", grip_dz="0.0095", grip_z="0.014", hold="-0.004",
    pin_z="0.006", over="0.03", speed="0.015", depth="0.015", pin_x="0.160", pin_y="0.090").items()}
DOWN_L = grip_rotation(-math.pi / 2)
DOWN_R = grip_rotation(math.pi / 2)


def _R(c1, c3) -> np.ndarray:
    c1, c3 = np.asarray(c1, float), np.asarray(c3, float)
    c1, c3 = c1 / np.linalg.norm(c1), c3 / np.linalg.norm(c3)
    return np.column_stack([c1, np.cross(c3, c1), c3])


def _rot_axis(axis, ang) -> np.ndarray:
    k = np.asarray(axis, float) / np.linalg.norm(axis)
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + math.sin(ang) * K + (1 - math.cos(ang)) * K @ K


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


def _width_at(s: float) -> float:
    """The blade's width at ``s`` from its point."""
    x = TIP_S - s
    for a, b, w in BLADE:
        if a <= x <= b:
            return w
    return BLADE[0][2]


def _slit_pose(alpha_deg: float, cut_y: float, inward: float = 0.0):
    """The TCP and rotation that hold the opener with its blade across the fold at ``alpha_deg``, its outer edge on
    the fold ``cut_s`` behind the point, at the fold's world y ``cut_y``; ``inward`` moves it further in (-x)."""
    a = math.radians(alpha_deg)
    u = np.array([-math.sin(a), math.cos(a), 0.0])  # toward the point
    n = np.array([math.cos(a), math.sin(a), 0.0])  # across the blade, outward
    s = P["cut_s"]
    xf = EC[0] + X_F
    edge = np.array([xf, cut_y])  # where the outer edge crosses the fold
    tip_xy = edge + s * u[:2] - (_width_at(s) / 2) * n[:2] + (P["out"] - inward) * np.array([1.0, 0.0])
    beta = math.radians(P["pitch"])
    ud = math.cos(beta) * u - math.sin(beta) * np.array([0.0, 0.0, 1.0])  # the axis, point down
    tip = np.array([tip_xy[0], tip_xy[1], TOP + P["tip_z"]])
    grip = tip - TIP_S * ud  # the handle's middle
    rot = _rot_axis(n, -beta) @ _R(n, (0.0, 0.0, -1.0))
    tcp = grip + rot[:, 2] * -P["grip_dz"]  # the TCP sits grip_dz above the handle's middle, along the gripper
    return tcp, rot


def _right() -> list:
    g0 = np.array([OP0[0], OP0[1], TOP + P["grip_z"]])
    r0 = _R((1.0, 0.0, 0.0), (0.0, 0.0, -1.0))  # fingers closing along x, across the handle (it points +y)
    up = np.array([0.0, 0.0, 0.05])
    out = _lead_in("right", g0 + up, r0, OPEN_C, "above the opener's handle")
    out += [_w(1.5, g0, OPEN_C, "down round the handle", r0),
            _w(1.0, g0, P["hold"], "take the handle", r0),
            _w(0.5, g0, P["hold"], "settle", r0)]
    y0 = EC[1] - ENV_L / 2
    p_in, r_in = _slit_pose(P["alpha"], y0 - 0.030)  # the point 15 mm short of the corner gap
    out += [_w(1.5, g0 + up, P["hold"], "lift the opener", r0),
            _w(2.5, p_in + up * 0.6, P["hold"], "turn it across the fold, beside the start corner", r_in),
            _w(1.5, p_in, P["hold"], "point level with the corner gap", r_in),
            _w(0.5, p_in, P["hold"], "settle", r_in)]
    y_end = EC[1] + ENV_L / 2 + P["over"]
    n = 40
    for k in range(1, n + 1):
        cy = y0 - 0.030 + (y_end - y0 + 0.030) * k / n
        inward = P["depth"] * min(1.0, max(0.0, (cy - y0 - 0.04) / 0.04)) if DEEP else 0.0  # eased in over 4 cm
        p, r = _slit_pose(P["alpha"], cy, inward)
        out.append(_w((y_end - y0 + 0.030) / n / P["speed"], p, P["hold"], "slit along the fold", r))
    p_end, r_end = _slit_pose(P["alpha"], y_end)
    out += [_w(1.0, p_end, P["hold"], "settle", r_end),
            _w(1.0, p_end + np.array([0.02, 0.0, 0.03]), P["hold"], "out past the far corner, up", r_end)]
    put = np.array([0.300, -0.060, TOP + P["grip_z"] + 0.002])
    rp = _R((1.0, 0.0, 0.0), (0.0, 0.0, -1.0))
    out += [_w(2.5, put + up, P["hold"], "carry it to the desk beyond the envelope", rp),
            _w(1.5, put, P["hold"], "lay it down", rp),
            _w(1.0, put, OPEN_C, "let go", rp),
            _w(1.5, put + up, OPEN_C, "rise clear", rp)]
    out += _lead_out("right", put + up, rp, OPEN_C, "back to the ready pose")
    return out


def _left(t_cut_end: float, t_total: float) -> list:
    pin = np.array([P["pin_x"], P["pin_y"], TOP + P["pin_z"]])
    up = np.array([0.0, 0.0, 0.05])
    out = _lead_in("left", pin + up, DOWN_L, 0.0, "above the envelope's near half, fingers closed")
    out += [_w(1.5, pin, 0.0, "press the envelope onto the desk", DOWN_L)]
    wait = t_cut_end - _dur(out)
    out += [_w(max(1.0, wait), pin, 0.0, "hold it there", DOWN_L),
            _w(1.5, pin + up, 0.0, "let it go", DOWN_L)]
    out += _lead_out("left", pin + up, DOWN_L, OPEN_C, "back to the ready pose")
    rest = t_total - _dur(out)
    lp, lq = _ready("left")
    out += [_w(max(0.5, rest), lp, OPEN_C, "stay clear", lq)]
    return out


def demo():
    right = _right()
    # the left hand holds the envelope until the opener has come out past the far corner
    t = 0.0
    t_cut_end = 0.0
    for w in right:
        t += w.duration
        if w.label == "out past the far corner, up":
            t_cut_end = t
    right_pre = 4.5  # the left hand gets there first: the right arm waits at the ready pose
    rp, rq = _ready("right")
    right = [_w(right_pre, rp, OPEN_C, "wait", rq)] + right
    t_cut_end += right_pre
    rp2, rq2 = _ready("right")
    right += [_w(3.0, rp2, OPEN_C, "stay clear", rq2)]
    if NOPIN:  # one hand only: the left arm stays at its ready pose
        lp, lq = _ready("left")
        return {"left": [_w(_dur(right), lp, OPEN_C, "stay at the ready pose", lq)], "right": right}
    left = _left(t_cut_end, _dur(right))
    return {"left": left, "right": right}
