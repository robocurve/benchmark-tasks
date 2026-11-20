"""On the station table: the lid of a square board-game box lying open side up (taped down), used as the dice tray,
and beside it, to the robot's right, a felt-lined leather dice cup standing upright with five 16 mm dice inside.
Task (instruction): roll the five dice out of the cup into the box lid. The instruction does not say how.

How it is done: in Yahtzee, Farkle, Liar's dice and the like, a player shakes the dice in the cup and rolls them out
onto the table or into the box lid; a die that lands off the table or "cocked" (resting tilted, against a wall or on
another die) does not count and must be rolled again ("A cocked die is a rolled die that has not landed cleanly on one
of its faces"; players re-roll it). Dice let slide out of the cup without tumbling are not a roll.

The hand skill: one hand takes the round cup by its side (its fingers must stay off the dice) and lifts it so its
mouth is about 20 cm above the lid floor; the wrist then turns the cup mouth-down in one quick turn (150 deg in 0.6 s)
while sweeping it 6 cm toward the open part of the lid, so the dice fall out together and tumble as they land. Tipped
slowly, low, with the lip just over the floor, the dice only slide out and lie as they were (no roll); turned less
than about 120 deg, some stay in the cup (the felt holds them); dropped in a heap, a die can come to rest cocked on
another.

World frame (m): x forward from the arm bases, y to the robot's left, z up; the table top is at z 0.7475.
- box lid: fixed (taped down), 254 x 254 mm outside, 30 mm tall, 2 mm cardboard; inside 250 x 250 mm, centred at
  (0.250, 0.075).
- dice cup: loose, upright, centred at (0.270, -0.165): 83 mm across, 95 mm tall, a 3 mm wall and a 4 mm base, 75 g.
- dice: five 16 mm dice with edges and corners rounded to 2 mm, 4.5 g each, standing in one layer on the cup's floor
  (one in the middle, four around it, 19.5 mm apart).
Restitution 0.30 for the dice and the lid (cardboard), 0.05 for the cup's felt lining; PhysX averages the two touching
materials' values.

Score (0.2 per die, from the poses; full score held 1 s). A die scores when all of these hold:
- it left the cup (its centre is outside the cup's inner cylinder);
- it was never touched by a gripper finger during the episode (no finger point within 1.5 mm of it: latched);
- it rolled: between leaving the cup and now, at some moment it had turned over by 90 deg or more (the angle by which
  its turn since leaving the cup tips the vertical: spinning flat does not count);
- it rests flat on the lid floor: its lowest corner within 1.5 mm of the floor, its faces within 5 deg of level (not
  cocked), its centre inside the lid's inner walls, at least 14 mm from the cup (not touching it), and still (moved
  under 1 mm in the last 1 s).
Score = 0.2 x the number of such dice. Success (1.0): all five, held 1 s.
"""

from __future__ import annotations

import itertools
import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

from bodykit import Body, BoxShape, CapsuleShape, CylShape, SphereShape, allow_matrix_grips, as_numpy, on_reset, on_step, plate, rotation  # noqa: E402
from yam_arena.demo import Waypoint, grip_rotation  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

allow_matrix_grips()

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"
MM = 0.001

# -- the layout ----------------------------------------------------------------------------------------------------
LID_C = (0.250, 0.075)
LID_IN = 0.250  # inside, square
LID_T = 0.002  # cardboard
LID_H = 0.030  # outside height
FLOOR = TOP + LID_T  # the lid floor's top
CUP_C = (0.270, -0.165)
CUP_R = 0.0415  # outside radius (83 mm across)
CUP_WALL = 0.003
CUP_RIN = CUP_R - CUP_WALL
CUP_H = 0.095
CUP_BASE = 0.004
CUP_M = 0.075
DIE = 0.016
DIE_M = 0.0045
DIE_RC = float(os.environ.get("DICE_ROUND", "2.0")) * MM  # the dice's rounded edges and corners (radius)
DICE_REST = float(os.environ.get("DICE_REST", "0.30"))  # restitution of the dice and the lid
DIE_OFFS = [(0.0, 0.0), (0.0195, 0.0), (-0.0195, 0.0), (0.0, 0.0195), (0.0, -0.0195)]
DIE_YAW = [0.0, 0.0, 0.0, 0.0, 0.0]

WHITE, PIP = (0.95, 0.94, 0.90), (0.05, 0.05, 0.06)
LEATHER, FELT, RIM = (0.30, 0.17, 0.09), (0.55, 0.06, 0.08), (0.20, 0.11, 0.06)
LID_IN_C, LID_OUT_C = (0.12, 0.22, 0.45), (0.85, 0.82, 0.74)


def _lid() -> Body:
    w = LID_IN + 2 * LID_T
    sh = [BoxShape((0.0, 0.0, LID_T / 2), (w, w, LID_T), LID_IN_C)]
    hz = LID_H - LID_T
    for s in (-1, 1):
        off = LID_IN / 2 + LID_T / 2
        sh.append(BoxShape((s * off, 0.0, LID_T + hz / 2), (LID_T, w, hz), LID_OUT_C))
        sh.append(BoxShape((0.0, s * off, LID_T + hz / 2), (w, LID_T, hz), LID_OUT_C))
    return Body.build("lid", pos=(LID_C[0], LID_C[1], TOP), mass=0.25, fixed=True, color=LID_IN_C,
                      friction=(0.40, 0.35), restitution=DICE_REST, shapes=sh, contact_offset=0.002)


def _cup() -> Body:
    """Origin at the centre of the base's underside. The wall is 24 flat panels."""
    sh = [CylShape((0.0, 0.0, CUP_BASE / 2), CUP_R, CUP_BASE, LEATHER),
          CylShape((0.0, 0.0, CUP_BASE + 0.0002), CUP_RIN - 0.0005, 0.0004, FELT, collide=False)]
    n = 24
    rm = CUP_R - CUP_WALL / 2
    width = 2 * CUP_R * math.tan(math.pi / n) * 1.02
    for k in range(n):
        a = 2 * math.pi * k / n
        c, s = math.cos(a), math.sin(a)
        sh.append(plate((rm * c, rm * s, CUP_BASE), (rm * c, rm * s, CUP_H), width, CUP_WALL, (c, s, 0.0), LEATHER))
        # the felt lining (visual only) just inside each panel
        ri = CUP_RIN - 0.0003
        sh.append(plate((ri * c, ri * s, CUP_BASE), (ri * c, ri * s, CUP_H - 0.003), width * 0.92, 0.0004,
                        (c, s, 0.0), FELT, collide=False))
    sh.append(CylShape((0.0, 0.0, CUP_H - 0.004), CUP_R + 0.0004, 0.008, RIM, collide=False))
    return Body.build("cup", pos=(CUP_C[0], CUP_C[1], TOP + 0.0005), mass=CUP_M, color=LEATHER,
                      friction=(0.60, 0.50), restitution=0.05, shapes=sh, contact_offset=0.002,
                      solver_iterations=32)


_PIPS = {1: [(0, 0)], 2: [(-1, -1), (1, 1)], 3: [(-1, -1), (0, 0), (1, 1)], 4: [(-1, -1), (-1, 1), (1, -1), (1, 1)],
         5: [(-1, -1), (-1, 1), (0, 0), (1, -1), (1, 1)], 6: [(-1, -1), (-1, 0), (-1, 1), (1, -1), (1, 0), (1, 1)]}
# faces: +z 1, -z 6, +x 2, -x 5, +y 3, -y 4 (opposite faces add up to 7)
_FACES = {1: (2, 1), 6: (2, -1), 2: (0, 1), 5: (0, -1), 3: (1, 1), 4: (1, -1)}


def _die(k: int) -> Body:
    h = DIE / 2
    r = DIE_RC
    c = h - r
    if r > 0:  # a cube with rounded edges and corners: three crossed boxes, a capsule on each edge, a ball per corner
        sh = [BoxShape((0.0, 0.0, 0.0), (DIE, DIE - 2 * r, DIE - 2 * r), WHITE),
              BoxShape((0.0, 0.0, 0.0), (DIE - 2 * r, DIE, DIE - 2 * r), WHITE),
              BoxShape((0.0, 0.0, 0.0), (DIE - 2 * r, DIE - 2 * r, DIE), WHITE)]
        for ax in range(3):
            o = [i for i in range(3) if i != ax]
            for su in (-1, 1):
                for sv in (-1, 1):
                    ctr = [0.0, 0.0, 0.0]
                    ctr[o[0]], ctr[o[1]] = su * c, sv * c
                    sh.append(CapsuleShape(tuple(ctr), r, DIE - 2 * r, WHITE, axis="XYZ"[ax]))
        sh += [SphereShape((a * c, b * c, d * c), r, WHITE) for a in (-1, 1) for b in (-1, 1) for d in (-1, 1)]
    else:
        sh = [BoxShape((0.0, 0.0, 0.0), (DIE, DIE, DIE), WHITE)]
    g = 0.0045
    for val, (ax, sg) in _FACES.items():
        others = [i for i in range(3) if i != ax]
        for (u, v) in _PIPS[val]:
            c = [0.0, 0.0, 0.0]
            c[ax] = sg * (h + 0.0001)
            c[others[0]], c[others[1]] = u * g, v * g
            sh.append(CylShape(tuple(c), 0.0016, 0.0004, PIP, collide=False, axis="XYZ"[ax]))
    ox, oy = DIE_OFFS[k]
    z = TOP + 0.0005 + CUP_BASE + h + 0.001
    return Body.build(f"die_{k}", pos=(CUP_C[0] + ox, CUP_C[1] + oy, z), mass=DIE_M, yaw=DIE_YAW[k], color=WHITE,
                      friction=(0.40, 0.35), restitution=DICE_REST, shapes=sh, contact_offset=0.002,
                      solver_iterations=32)


LID = _lid()
CUP = _cup()
DICE = [_die(k) for k in range(5)]
OBJECTS = [LID, CUP] + DICE

# -- tracking and scoring -------------------------------------------------------------------------------------------
TOUCH = 1.5 * MM  # a finger point this close to a die: touched
ROLL_DEG = 90.0
ON_FLOOR = 1.5 * MM
FLAT_DEG = 5.0
CUP_CLEAR = 0.014
STILL_STEPS, STILL_MM = 20, 1.0
_DEBUG = os.environ.get("DICE_DEBUG") == "1"
_DEBUG_EVERY = int(os.environ.get("DICE_DEBUG_EVERY", "5"))
_EP: dict = {}
_IDS: dict = {}
_CORNERS = np.array([[a, b, c] for a in (-0.5, 0.5) for b in (-0.5, 0.5) for c in (-0.5, 0.5)]) * DIE


@on_reset
def _reset() -> None:
    _EP.update(tick=0, step=0, counted=-1, hist=[], dice=[dict(exited=False, R_exit=None, swing=0.0, touched=False,
                                                                 good=False, info={}) for _ in DICE],
               n_good=0, cup=None, tcp={}, gaps={})


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


def _cup_dist(p, cup) -> tuple[float, float, float]:
    """(radial, axial, distance to the cup's solid) of world point ``p`` in the cup's frame."""
    loc = cup[1].T @ (np.asarray(p) - cup[0])
    rho = math.hypot(loc[0], loc[1])
    a = float(loc[2])
    d = math.hypot(max(rho - CUP_R, 0.0), max(-a, 0.0, a - CUP_H))
    return rho, a, d


@on_step
def _track(env) -> None:
    try:
        _track_inner(env)
    except Exception:
        import traceback

        print("DICE_TRACEBACK\n" + traceback.format_exc(), flush=True)
        raise


def _track_inner(env) -> None:
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
    fingers = np.concatenate(fingers)
    cp = as_numpy(base.scene["cup"].data.root_link_pose_w)[0]
    cup = (cp[:3] - origin, rotation(cp[3:7]))
    _EP["cup"] = cup
    n_good = 0
    for k, d in enumerate(DICE):
        p = as_numpy(base.scene[d.name].data.root_link_pose_w)[0]
        pos, R = p[:3] - origin, rotation(p[3:7])
        st = _EP["dice"][k]
        # touched by a finger: any finger point within TOUCH of the die's box
        loc = (fingers - pos) @ R
        out = np.maximum(np.abs(loc) - DIE / 2, 0.0)
        if float(np.sqrt((out ** 2).sum(1)).min()) < TOUCH:
            st["touched"] = True
        rho, a, dcup = _cup_dist(pos, cup)
        inside = rho < CUP_RIN and -0.005 < a < CUP_H
        if not st["exited"] and not inside:
            st["exited"], st["R_exit"] = True, R.copy()
        if st["exited"]:
            rel = R @ st["R_exit"].T
            st["swing"] = max(st["swing"], math.degrees(math.acos(max(-1.0, min(1.0, rel[2, 2])))))
        low = float((pos + _CORNERS @ R.T)[:, 2].min() - FLOOR)
        flat = math.degrees(math.acos(min(1.0, float(np.abs(R[2, :]).max()))))
        in_lid = abs(pos[0] - LID_C[0]) < LID_IN / 2 and abs(pos[1] - LID_C[1]) < LID_IN / 2
        st["info"] = dict(pos=pos, low=low, flat=flat, in_lid=in_lid, dcup=dcup)
        st["rest"] = bool(st["exited"] and not st["touched"] and st["swing"] >= ROLL_DEG and abs(low) <= ON_FLOOR
                          and flat <= FLAT_DEG and in_lid and dcup >= CUP_CLEAR)
    _EP["dice_pos"] = np.array([_EP["dice"][k]["info"]["pos"] for k in range(len(DICE))])


def _tally() -> dict:
    hist = _EP["hist"]
    if _EP["counted"] != _EP["tick"] and "dice_pos" in _EP:
        _EP["counted"] = _EP["tick"]
        hist.append(_EP["dice_pos"].copy())
        del hist[:-STILL_STEPS]
    n_good = 0
    for k, st in enumerate(_EP["dice"]):
        still = len(hist) == STILL_STEPS and all(np.abs(h[k] - hist[-1][k]).max() * 1000 < STILL_MM for h in hist)
        st["good"] = bool(st.get("rest") and still)
        n_good += st["good"]
    _EP["n_good"] = n_good
    return dict(n_good=n_good, done=n_good == len(DICE))


def _debug() -> None:
    _EP["step"] += 1
    if (_EP["step"] - 1) % _DEBUG_EVERY or "dice_pos" not in _EP:
        return
    ds = []
    for k, st in enumerate(_EP["dice"]):
        i = st["info"]
        p = i["pos"]
        ds.append(f"d{k}=({p[0]:.3f},{p[1]:.3f},{i['low'] * 1000:.1f}mm f{i['flat']:.0f} sw{st['swing']:.0f}"
                  f"{' X' if st['exited'] else ''}{' T' if st['touched'] else ''}{'' if i['in_lid'] else ' OUT'}"
                  f"{' G' if st['good'] else ''})")
    c = _EP["cup"]
    tilt = math.degrees(math.acos(max(-1.0, min(1.0, c[1][2, 2]))))
    r = _EP["tcp"].get("right", np.zeros(3))
    print(f"DBG step={_EP['step']} good={_EP['n_good']} cup=({c[0][0]:.3f},{c[0][1]:.3f},{(c[0][2] - TOP) * 1000:.0f}mm"
          f",t{tilt:.0f}) R tcp=({r[0]:.3f},{r[1]:.3f},{(r[2] - TOP) * 1000:.0f}mm) gap={_EP['gaps'].get('right', 0) * 1000:.0f} | "
          + " ".join(ds), flush=True)


def score(objects) -> float:
    """0.2 per die that left the cup untouched, rolled and rests flat in the lid. Full score is success()."""
    t = _tally()
    if _DEBUG:
        _debug()
    return 0.2 * t["n_good"]


def success(objects) -> bool:
    return bool(_tally()["done"])


# -- scripted demo --------------------------------------------------------------------------------------------------
# Open loop, from the known layout; the left arm stays at its ready pose. The right hand comes from behind the cup,
# pointing forward and 20 deg down, its fingers closing across the cup (along y) 60 mm above the table, and closes
# 4 mm inside the cup's sides. It lifts the cup, gives it a gentle rattle (two small sideways moves) and brings it,
# upright, over the right part of the lid, the grip 25 cm above the table. Then, in one turn of POUR_S seconds, it
# turns the wrist about its pointing direction by POUR_DEG (the cup's mouth turns down toward the open lid on the left;
# the cup ends tilted 130 deg) while sweeping the cup 6 cm left: the dice fall about 15 to 20 cm and tumble. It holds
# 0.6 s, turns the cup back upright while moving it 10 cm right, clear of the lid, carries it back and sets it down
# where it stood, lets go and rises clear.
# DICE_MODE=slide is the wrong way: the cup tipped over 1.6 s with its lip about 15 mm over the lid floor and no sweep,
# so the dice slide out without turning over.
OPEN_C = -0.047
MODE = os.environ.get("DICE_MODE", "demo")
GRIP_Z = float(os.environ.get("DICE_GRIP_Z", "60")) * MM
SQUEEZE = float(os.environ.get("DICE_SQUEEZE", "4")) * MM
POUR_DEG = float(os.environ.get("DICE_POUR_DEG", "150"))
POUR_YAW = float(os.environ.get("DICE_POUR_YAW", "15"))
POUR_S = float(os.environ.get("DICE_POUR_S", "0.6"))
POUR_X = float(os.environ.get("DICE_POUR_X", "0.250"))
POUR_Y0 = float(os.environ.get("DICE_POUR_Y0", "-0.060"))
POUR_Y1 = float(os.environ.get("DICE_POUR_Y1", "0.000"))
POUR_Z0 = float(os.environ.get("DICE_POUR_Z0", "250")) * MM
POUR_Z1 = float(os.environ.get("DICE_POUR_Z1", "250")) * MM
if MODE == "slide":  # the wrong way: tipped slowly, low, with the lip just over the floor
    POUR_Z0 = 0.110
    POUR_Z1 = float(os.environ.get("DICE_SLIDE_Z", "80")) * MM
    POUR_S = float(os.environ.get("DICE_SLIDE_S", "1.6"))
    POUR_Y0, POUR_Y1 = -0.040, -0.010


def _rot_axis(axis, a: float) -> np.ndarray:
    axis = np.asarray(axis, float) / np.linalg.norm(axis)
    k = np.array([[0.0, -axis[2], axis[1]], [axis[2], 0.0, -axis[0]], [-axis[1], axis[0], 0.0]])
    return np.eye(3) + math.sin(a) * k + (1 - math.cos(a)) * (k @ k)


R_LEVEL = _rot_axis((0.0, 1.0, 0.0), -math.pi / 2) @ grip_rotation(0.0)  # pointing +x, fingers closing along y
R_GRIP = _rot_axis((0.0, 1.0, 0.0), math.radians(20)) @ R_LEVEL  # pointing forward and 20 deg down
APPROACH = R_GRIP[:, 2]


def _r_pour(deg: float, yaw: float = POUR_YAW) -> np.ndarray:
    """The grip turned ``deg`` about its pointing direction (mouth toward +y) and ``yaw`` clockwise about z."""
    return _rot_axis((0.0, 0.0, 1.0), -math.radians(yaw)) @ _rot_axis(APPROACH, -math.radians(deg)) @ R_GRIP


R_POUR = _r_pour(POUR_DEG)
R_POUR_MID = _r_pour(POUR_DEG * 0.5)


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


def demo():
    from yam_arena import ik, station
    from yam_arena.constants import READY

    sq = -(CUP_R - SQUEEZE)
    grip = np.array([CUP_C[0], CUP_C[1], TOP + GRIP_Z])
    behind = grip - 0.07 * APPROACH
    up = np.array([CUP_C[0], CUP_C[1], TOP + 0.15])
    p0 = np.array([POUR_X, POUR_Y0, TOP + POUR_Z0])
    p1 = np.array([POUR_X, POUR_Y1, TOP + POUR_Z1])
    pm = (p0 + p1) / 2
    right = _lead_in("right", behind, R_GRIP, OPEN_C, "behind the cup, open")
    right += [_w(2.0, grip, OPEN_C, "in round the cup", R_GRIP),
              _w(1.2, grip, sq, "close on the cup's sides", R_GRIP),
              _w(0.8, grip, sq, "settle", R_GRIP),
              _w(1.5, up, sq, "lift the cup", R_GRIP)]
    for k in range(2):  # a gentle rattle
        right += [_w(0.35, up + [0.0, 0.010, 0.0], sq, "rattle", R_GRIP),
                  _w(0.35, up + [0.0, -0.010, 0.0], sq, "rattle", R_GRIP)]
    right += [_w(0.4, up, sq, "steady", R_GRIP),
              _w(2.5, p0, sq, "over the lid's right part, upright", R_GRIP),
              _w(0.6, p0, sq, "steady", R_GRIP),
              _w(POUR_S / 2, pm, sq, "tip: half way", R_POUR_MID),
              _w(POUR_S / 2, p1, sq, "tip: mouth down, lip low over the floor", R_POUR),
              _w(0.6, p1, sq, "hold for the last die", R_POUR),
              _w(1.5, p1 + [0.0, -0.10, 0.0], sq, "turn the cup back upright, away to the right", R_GRIP),
              _w(2.5, up, sq, "back over its place", R_GRIP),
              _w(1.5, grip + [0.0, 0.0, 0.002], sq, "set it down", R_GRIP),
              _w(1.0, grip + [0.0, 0.0, 0.002], OPEN_C, "let go", R_GRIP),
              _w(1.5, behind, OPEN_C, "back out", R_GRIP),
              _w(1.5, behind + [0.0, 0.0, 0.08], OPEN_C, "rise clear", R_GRIP),
              _w(4.0, behind + [0.0, 0.0, 0.08], OPEN_C, "wait for the dice to settle", R_GRIP)]
    lp, lr = ik.tcp_pose(np.array(READY, float), station.BASES["left"])
    left = [Waypoint(sum(w.duration for w in right), tuple(float(v) for v in lp), 0.0, "wait", lr)]
    return {"left": left, "right": right}
