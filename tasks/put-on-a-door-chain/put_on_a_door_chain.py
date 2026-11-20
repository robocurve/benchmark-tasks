"""On the station table: the latch-side corner of a real interior door, shut in its frame, with a chain door guard on
it. The guard's slide track (a brass channel 100 mm long with a slot 6.4 mm wide and a 14 mm keyhole opening at its
far end) is screwed to the door's face; the chain holder is screwed to the frame's casing beside it; the chain (8
welded links, 152 mm) hangs from the holder with its slide knob dangling at the bottom. Task (instruction): put the
door chain on. The instruction does not say how.

How it is done (the guard's fitting sheet): "Mount slide plate 1/4" from edge of door at convenient height. Large hole
at end of slot faces away from door jamb. Mount chain holder on door jamb so that knob on chain will reach hole in
slide plate." To put the chain on, the knob's head is pushed in through the large hole and slid along the slot to the
slot's closed end, next to the jamb, where it stays; opened, the door then stops after a few centimetres.

The hand skill: catch the small knob (10 mm across) on its light chain, which swings away when bumped, without
turning it round; carry it up 15 cm to the 14 mm keyhole on the vertical door face and push its 11 mm head in through
it, square to the door (the head enters only there, with 1.5 mm of play each way); then slide it 8 cm along the 6.4 mm
slot to the closed end without pulling it back out, with the chain dragging on it, and let go. Let go early and it
stays part way along the slot.

World frame (m): x forward from the arm bases, y to the robot's left, z up; the table top is at z 0.7475. Heights
below are above the table top.
- door: a hollow-core interior door cut down to the part by its latch edge, 35 mm thick, its face (toward the robot)
  at x 0.280, its latch edge at y -0.030, running 20 cm to y +0.170 and 45 cm up from the table. Shut: fixed.
- frame: the jamb (20 mm) beyond the door's edge (3 mm gap), and its casing (70 x 12 mm moulding, 12 mm proud of the
  door's face) from y -0.036 to -0.106. Fixed.
- slide track (fixed to the door): a brass channel along y, 18 mm tall, standing 6 mm off the door's face, its middle
  0.300 up; closed end at y -0.024 (6 mm from the door's edge), far end at y +0.076. Front lips 1.2 mm thick with a
  slot 6.4 mm wide along the middle; the keyhole: a 14 x 14 mm opening centred at y +0.064. Inside: a cavity 3.8 mm
  deep, 14 mm tall, in which the knob's head rides.
- chain holder (fixed to the casing): a brass plate 22 x 45 mm on the casing's middle, its stud 16 mm proud; the chain
  hangs from the stud at (0.252, -0.071, 0.310).
- chain: 8 links, 19 mm pitch (152 mm), 3.2 mm steel wire, 2.5 g each, each hung from the one above by a ball joint
  (80 deg each way).
- slide knob (brass, 9 g): a body 10 mm across and 18 mm long, a neck 4.8 mm across and 3.5 mm long, a head 11 mm
  across and 2 mm thick, and a tongue on top of the body that the last link hooks through (ball joint, 90 deg). At the
  start it hangs level under the chain, head toward the door (4.5 mm off the casing), its axis 0.146 up, its body's
  middle at (0.249, -0.071).

Score (end state):
- 1.0 (success, held 1 s) when the knob's head is captured in the track (inside the channel behind the lips, so it
  cannot be pulled straight out) within 8 mm of where it stops at the slot's closed end (neck against the end), the
  knob is let go (no finger point within 10 mm of it) and still (under 1 mm in 1 s).
- 0.5 when the head is captured in the track at least 10 mm along the slot from the keyhole's centre (past the hole)
  but not at the end, or at the end and still held.
- 0 otherwise (never put in, still in the keyhole, or pulled back out).
Real trial: the same, with a ruler from the knob's neck to the slot's end, the trial video for the release, and a tug
test (open the door: it stops on the chain).
"""

from __future__ import annotations

import itertools
import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

from bodykit import Body, BoxShape, CapsuleShape, CylShape, Joint, allow_matrix_grips  # noqa: E402
from bodykit import as_numpy, on_reset, on_step, rotation  # noqa: E402
from yam_arena.demo import Waypoint, grip_rotation  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

allow_matrix_grips()

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"
MM = 0.001

# -- the layout ----------------------------------------------------------------------------------------------------
XF = 0.280  # the door's face
YE = -0.030  # the door's latch edge
DOOR_T, DOOR_W, DOOR_H = 0.035, 0.20, 0.45
GAP, JAMB_T, JAMB_D = 0.003, 0.020, 0.110
CAS_W, CAS_T = 0.070, 0.012
CAS_Y0 = YE - GAP - 0.003  # the casing's door-side edge (a 3 mm reveal on the jamb)
Z_T = 0.300  # the track's middle line (height)
TR_H, TR_D = 0.018, 0.006  # track height (z) and depth off the door (x)
YC = YE + 0.006  # the track's closed end
TR_L = 0.100
YF = YC + TR_L
WALL = 0.003  # end walls
LIP_T = 0.0012
BACK_T = 0.001
SLOT = 0.0064
KEY = 0.014  # keyhole opening (square)
Y_K = YF - 0.012  # keyhole centre
X_LIP_IN = XF - TR_D + LIP_T  # the lips' inner face (x)
CAV_Z = 0.007  # the cavity: +-7 mm
# knob (its frame: origin at the tongue's hole, x along its axis toward the head)
BODY_R, BODY_L = 0.005, 0.018
NECK_R, NECK_L = 0.0024, 0.0035
HEAD_R, HEAD_T = 0.0055, 0.002
TONGUE_Z = 0.012  # the knob's axis this far below the tongue's hole
BODY_X0 = -0.012  # the body from x -12 mm to +6 mm, the neck to +9.5, the head to +11.5
NECK_X0 = BODY_X0 + BODY_L
HEAD_X0 = NECK_X0 + NECK_L
HEAD_C = np.array([HEAD_X0 + HEAD_T / 2, 0.0, -TONGUE_Z])  # head centre, knob frame
GRASP_C = np.array([-0.010, 0.0, -TONGUE_Z])  # where the fingers close (the TCP): the body's back part
# chain
N_LINK, PITCH, WIRE_R = 8, 0.019, 0.0016
ANCHOR = (XF - CAS_T - 0.016, CAS_Y0 - CAS_W / 2, Z_T + 0.010)  # the holder's stud (x, y, height)
KNOB_O = (ANCHOR[0], ANCHOR[1], ANCHOR[2] - N_LINK * PITCH)  # the tongue's hole at the start
LINK_MASS, KNOB_MASS = 0.0025, 0.009
DAMP = (float(os.environ.get("DCHAIN_LDAMP", "0.1")), float(os.environ.get("DCHAIN_ADAMP", "0.3")))
# scoring geometry: where the head centre is when the knob is at the slot's closed end, and the keyhole's centre
Y_STOP = YC + WALL + NECK_R
END_OK = 0.008
PAST_KEY = 0.010
CAPTURE_X = X_LIP_IN + HEAD_T / 2 - 0.0005  # head centre at least this far in (x): its rim behind the lips

WOOD, WOOD_D, PAINT, BRASS, DARK_BRASS, STEEL = ((0.93, 0.92, 0.88), (0.80, 0.78, 0.74), (0.96, 0.95, 0.92),
                                                (0.80, 0.63, 0.28), (0.55, 0.42, 0.18), (0.45, 0.46, 0.50))


def _v(p) -> tuple:
    return tuple(float(x) for x in p)


def _w3(x: float, y: float, z: float) -> tuple:
    """A world point (x, y, height above the table)."""
    return (float(x), float(y), float(TOP + z))


DOOR_O = _w3(XF, YE, 0.0)  # the fixed body's origin: the door's face at its latch edge, on the table


def _d(x: float, y: float, z: float) -> tuple:
    """A world point (x, y, height above the table) relative to the door's origin."""
    return (x - DOOR_O[0], y - DOOR_O[1], TOP + z - DOOR_O[2])


def _box(x0, x1, y0, y1, z0, z1, color, collide=True) -> BoxShape:
    return BoxShape(_d((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2), (x1 - x0, y1 - y0, z1 - z0), color,
                    collide=collide)


def _door_shapes() -> list:
    z0, z1 = Z_T - TR_H / 2, Z_T + TR_H / 2
    xl0, xl1 = XF - TR_D, XF - TR_D + LIP_T  # the lips
    xb0 = XF - BACK_T
    sh = [
        # the door (hollow core: skins and an edge stile; drawn as one slab) and a strip of its edge
        _box(XF, XF + DOOR_T, YE, YE + DOOR_W, 0.0, DOOR_H, PAINT),
        _box(XF + 0.002, XF + DOOR_T - 0.002, YE - 0.0005, YE, 0.0, DOOR_H, WOOD_D, collide=False),
        # the jamb and its stop (behind the door) and the casing
        _box(XF - 0.001, XF + JAMB_D, YE - GAP - JAMB_T, YE - GAP, 0.0, DOOR_H, WOOD),
        _box(XF + DOOR_T, XF + DOOR_T + 0.012, YE - GAP - 0.002, YE + 0.012, 0.0, DOOR_H, WOOD),
        _box(XF - CAS_T, XF - 0.001, CAS_Y0 - CAS_W, CAS_Y0, 0.0, DOOR_H, WOOD),
        _box(XF - CAS_T - 0.0005, XF - CAS_T, CAS_Y0 - 0.004, CAS_Y0 - 0.002, 0.0, DOOR_H, WOOD_D, collide=False),
        # the chain holder on the casing: a plate and the stud the first link hangs from
        _box(XF - CAS_T - 0.0025, XF - CAS_T, ANCHOR[1] - 0.011, ANCHOR[1] + 0.011, ANCHOR[2] - 0.030,
             ANCHOR[2] + 0.015, BRASS),
        CylShape(_d((XF - CAS_T + ANCHOR[0]) / 2, ANCHOR[1], ANCHOR[2] + 0.002), 0.003,
                 XF - CAS_T - ANCHOR[0], DARK_BRASS, collide=False, axis="X"),  # drawn: the first link hangs on it
        # the track: back plate, top and bottom walls, end walls
        _box(xb0, XF, YC, YF, z0, z1, BRASS),
        _box(xl0, xb0, YC, YF, Z_T + CAV_Z, z1, BRASS),
        _box(xl0, xb0, YC, YF, z0, Z_T - CAV_Z, BRASS),
        _box(xl0, xb0, YC, YC + WALL, Z_T - CAV_Z, Z_T + CAV_Z, BRASS),
        _box(xl0, xb0, YF - WALL, YF, Z_T - CAV_Z, Z_T + CAV_Z, BRASS),
    ]
    # the lips: along the slot (above and below it) from the closed end to the keyhole, around the keyhole, beyond it
    ks0, ks1 = Y_K - KEY / 2, Y_K + KEY / 2
    for zs0, zs1 in ((Z_T + SLOT / 2, Z_T + CAV_Z), (Z_T - CAV_Z, Z_T - SLOT / 2)):
        sh.append(_box(xl0, xl1, YC + WALL, ks0, zs0, zs1, BRASS))
    for zs0, zs1 in ((Z_T + KEY / 2, Z_T + CAV_Z), (Z_T - CAV_Z, Z_T - KEY / 2)):
        if zs1 > zs0:
            sh.append(_box(xl0, xl1, ks0, ks1, zs0, zs1, BRASS))
    sh.append(_box(xl0, xl1, ks1, YF - WALL, Z_T - CAV_Z, Z_T + CAV_Z, BRASS))
    # the dark inside of the slot and keyhole (drawn on the back plate)
    sh.append(_box(xb0 - 0.0003, xb0, YC + WALL, ks0, Z_T - SLOT / 2, Z_T + SLOT / 2, DARK_BRASS, collide=False))
    sh.append(_box(xb0 - 0.0003, xb0, ks0, ks1, Z_T - KEY / 2, Z_T + KEY / 2, DARK_BRASS, collide=False))
    return sh


DOOR = Body.build("door_and_frame", pos=DOOR_O, mass=5.0, fixed=True, color=PAINT, shapes=_door_shapes(),
                  friction=(0.35, 0.3), contact_offset=0.001, solver_iterations=32)

LINK_O = [_w3(ANCHOR[0], ANCHOR[1], ANCHOR[2] - i * PITCH) for i in range(N_LINK)]  # each link's top joint (world)


def _link(i: int) -> Body:
    o = LINK_O[i]
    sh = [CapsuleShape((0.0, 0.0, -PITCH / 2), WIRE_R, PITCH - 2 * WIRE_R - 0.0004, STEEL, axis="Z")]
    parent = None if i == 0 else f"chain_{i - 1}"
    return Body.build(f"chain_{i}", pos=o, mass=LINK_MASS, shapes=sh, color=STEEL, friction=(0.4, 0.3),
                      damping=DAMP, contact_offset=0.001, solver_iterations=32,
                      joint=Joint(anchor=o, kind="ball", parent=parent, limits=(80.0,)))


def _knob_shapes() -> list:
    zc = -TONGUE_Z
    return [CylShape((BODY_X0 + BODY_L / 2, 0.0, zc), BODY_R, BODY_L, BRASS, axis="X"),
            CylShape((NECK_X0 + NECK_L / 2, 0.0, zc), NECK_R, NECK_L, DARK_BRASS, axis="X"),
            CylShape((HEAD_X0 + HEAD_T / 2, 0.0, zc), HEAD_R, HEAD_T, BRASS, axis="X"),
            BoxShape((0.0, 0.0, zc / 2 - 0.001), (0.008, 0.0025, TONGUE_Z - 0.002), BRASS)]


KNOB_W = _w3(*KNOB_O)
KNOB = Body.build("chain_knob", pos=KNOB_W, mass=KNOB_MASS, shapes=_knob_shapes(), color=BRASS, friction=(0.4, 0.3),
                  damping=DAMP, contact_offset=0.001, solver_iterations=32,
                  joint=Joint(anchor=KNOB_W, kind="ball", parent=f"chain_{N_LINK - 1}", limits=(90.0,)))
CHAIN = [_link(i) for i in range(N_LINK)]
OBJECTS = [DOOR, *CHAIN, KNOB]

# -- tracking and scoring -------------------------------------------------------------------------------------------
CLEAR = 0.010
STILL_STEPS, STILL_MM = 20, 1.0
_DEBUG = os.environ.get("DCHAIN_DEBUG") == "1"
_DEBUG_EVERY = int(os.environ.get("DCHAIN_DEBUG_EVERY", "5"))
_EP: dict = {}
_IDS: dict = {}


@on_reset
def _reset() -> None:
    _EP.clear()
    _EP.update(tick=0, step=0, counted=-1, recent=[], near=1.0, gaps={}, tcp={}, events=[], captured=False,
               best_y=None)


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


def _knob_points() -> np.ndarray:
    """Points on the knob's surface (knob frame): its body, neck, head and tongue."""
    pts = []
    for x0, x1, r in ((BODY_X0, NECK_X0, BODY_R), (NECK_X0, HEAD_X0, NECK_R), (HEAD_X0, HEAD_X0 + HEAD_T, HEAD_R)):
        for x in np.linspace(x0, x1, 4):
            for a in np.linspace(0, 2 * math.pi, 12, endpoint=False):
                pts.append((x, r * math.cos(a), -TONGUE_Z + r * math.sin(a)))
    for z in np.linspace(-TONGUE_Z + BODY_R, 0.0, 4):
        for x in (-0.004, 0.0, 0.004):  # the tongue
            pts.append((x, 0.0, z))
    return np.array(pts)


@on_step
def _track(env) -> None:
    try:
        _track_inner(env)
    except Exception:
        import traceback

        print("DCHAIN_TRACEBACK\n" + traceback.format_exc(), flush=True)
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
        _IDS["knob_pts"] = _knob_points()
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
    kq = as_numpy(base.scene["chain_knob"].data.root_link_pose_w)[0]
    kp, kr = kq[:3] - origin, rotation(kq[3:7])
    head = kp + kr @ HEAD_C
    axis = kr[:, 0]
    kpts = kp + _IDS["knob_pts"] @ kr.T
    tree = cKDTree(kpts)
    near = {s: float(tree.query(fp, k=1)[0].min()) for s, fp in fingers.items()}
    # captured: the head's centre inside the channel behind the lips, level with the slot, between the end walls
    hz = head[2] - TOP
    captured = (head[0] >= CAPTURE_X and head[0] <= XF and abs(hz - Z_T) <= CAV_Z - HEAD_R + 0.0015
                and YC + WALL <= head[1] <= YF - WALL)
    if captured and not _EP["captured"]:
        _EP["events"].append((_EP["tick"], f"head in the channel at y {head[1]:.3f}"))
    if not captured and _EP["captured"]:
        _EP["events"].append((_EP["tick"], f"head out of the channel at y {head[1]:.3f}"))
    _EP["captured"] = captured
    fl = fingers["left"]
    _EP.update(head=head, axis=axis, near=min(near.values()), near_s=near,
               fext=(*fl[int(fl[:, 1].argmin())].round(4), *fl[int(fl[:, 0].argmax())].round(4)))


def _tally(o) -> dict:
    recent = _EP["recent"]
    if _EP["counted"] != _EP["tick"] and "head" in _EP:
        _EP["counted"] = _EP["tick"]
        recent.append(np.asarray(_EP["head"], float).copy())
        del recent[:-STILL_STEPS]
    still = len(recent) == STILL_STEPS and all(np.abs(p - recent[-1]).max() * 1000 < STILL_MM for p in recent)
    released = _EP["near"] >= CLEAR
    head = _EP.get("head")
    cap = bool(_EP.get("captured")) and head is not None
    along = (Y_K - head[1]) if head is not None else 0.0  # how far along the slot from the keyhole's centre
    at_end = cap and head[1] <= Y_STOP + END_OK
    past = cap and along >= PAST_KEY
    if at_end and released and still:
        s = 1.0
    elif past:
        s = 0.5
    else:
        s = 0.0
    return dict(still=still, released=released, at_end=at_end, past=past, along=along, score=s)


def _debug(t) -> None:
    _EP["step"] += 1
    if (_EP["step"] - 1) % _DEBUG_EVERY:
        return
    h = _EP.get("head", np.zeros(3) + [0, 0, TOP])
    a = _EP.get("axis", np.zeros(3))
    print(f"DBG step={_EP['step']} head=({h[0]:.4f},{h[1]:.4f},{h[2] - TOP:.4f}) axis=({a[0]:.2f},{a[1]:.2f},"
          f"{a[2]:.2f}) cap={int(_EP['captured'])} along={t['along'] * 1000:.1f}mm end={int(t['at_end'])} "
          f"near={_EP['near'] * 1000:.0f}mm rel={int(t['released'])} still={int(t['still'])} score={t['score']:.2f} "
          f"fext={tuple(round(v, 4) for v in _EP.get('fext', ()))} ev={_EP['events'][-3:]} | "
          + " ".join(f"{k} tcp=({v[0]:.3f},{v[1]:.3f},{v[2] - TOP:.3f}) gap={_EP['gaps'][k] * 1000:.0f}"
                     for k, v in _EP["tcp"].items()), flush=True)


def score(objects) -> float:
    t = _tally(objects)
    if _DEBUG:
        _debug(t)
    return float(t["score"])


def success(objects) -> bool:
    return bool(_tally(objects)["score"] >= 0.999)


# -- scripted demo --------------------------------------------------------------------------------------------------
# Open loop, from the known layout. The left gripper, level and pointing forward (+x) with its fingers open across y,
# comes in at the hanging knob's height so the knob (lying level, head toward the door) sits between its fingers, and
# closes on the knob's body. It lifts the knob to the track's height, carries it 2 cm out from the door along the
# track to the keyhole, pushes the head in through the keyhole, slides the knob along the slot to the closed end
# (commanded a little past it: the arm lags sideways), lets go and backs away.
# DCHAIN_WRONG=slot: the same, but the head is pushed at the slot 4 cm short of the keyhole: it cannot go in, and the
#   knob is slid along the outside of the lips and let go there (falls back on its chain): 0.
# DCHAIN_WRONG=early: let go half way along the slot: 0.5.
OPEN_C = -0.047
WRONG = os.environ.get("DCHAIN_WRONG", "")
GRIP = float(os.environ.get("DCHAIN_GRIP", "-0.0015"))  # finger value: a 3 mm commanded gap on the 10 mm body
DX_IN = float(os.environ.get("DCHAIN_DXIN", "0.0015"))  # push this far past the engaged depth
DZ = float(os.environ.get("DCHAIN_DZ", "0.0"))
ROT_LEVEL = np.column_stack([(0.0, -1.0, 0.0), (0.0, 0.0, -1.0), (1.0, 0.0, 0.0)])  # left: level, toward +x
ROT_DOWN_L = grip_rotation(-math.pi / 2)


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


def _back_off(arm: str) -> list:
    """From the ready pose (its gripper beside the door) straight back toward the robot first."""
    from yam_arena import ik, station
    from yam_arena.constants import READY

    pos, rot = ik.tcp_pose(np.array(READY, float), station.BASES[arm])
    return [Waypoint(1.5, (0.20, float(pos[1]), float(pos[2])), OPEN_C, "back toward the robot, clear of the door",
                     rot)]


def _stay(arm: str, t: float = 120.0) -> list:
    from yam_arena import ik, station
    from yam_arena.constants import READY

    pos, rot = ik.tcp_pose(np.array(READY, float), station.BASES[arm])
    return [Waypoint(t, tuple(float(v) for v in pos), OPEN_C, "stay put", rot)]


# The knob's body centre at the start (world), and where it is when the head rides in the channel's middle.
HANG = np.array(KNOB_W) + GRASP_C
X_ENG = (XF - BACK_T - 0.0009) - (HEAD_X0 + HEAD_T - GRASP_C[0])  # head's front face 0.9 mm off the back plate
Z_ENG = TOP + Z_T + DZ


def _left() -> list:
    rl = ROT_LEVEL
    front = HANG + np.array([-0.060, 0.0, 0.0])
    lift = HANG + np.array([-0.004, 0.0, 0.035])
    y_in = Y_K if WRONG != "slot" else Y_K - 0.040
    out_k = np.array([X_ENG - 0.020, y_in, Z_ENG])
    in_k = np.array([X_ENG + DX_IN, y_in, Z_ENG])
    out = _back_off("left") + [_w(2.0, np.array([0.19, 0.10, TOP + 0.22]), OPEN_C, "turn level, in front of the door",
                                  rl),
                               _w(2.0, front + np.array([0.0, 0.0, 0.03]), OPEN_C, "in front of the hanging knob", rl)]
    out += [_w(1.5, front, OPEN_C, "down to the knob's height", rl),
            _w(2.0, HANG, OPEN_C, "in, the knob between the open fingers", rl),
            _w(0.5, HANG, OPEN_C, "settle", rl),
            _w(1.0, HANG, GRIP, "close on the knob's body", rl),
            _w(0.6, HANG, GRIP, "settle", rl),
            _w(1.5, lift, GRIP, "lift it a little, out from the casing", rl),
            _w(3.0, out_k + np.array([0.0, 0.0, 0.0]), GRIP, "up and along to 2 cm in front of the keyhole", rl),
            _w(0.8, out_k, GRIP, "settle (let the chain stop swinging)", rl),
            _w(2.0, in_k, GRIP, "push the head in through the keyhole", rl),
            _w(0.5, in_k, GRIP, "settle", rl)]
    end = np.array([X_ENG + DX_IN, Y_STOP - 0.010, Z_ENG])
    if WRONG == "early":
        mid = np.array([X_ENG + DX_IN, (Y_K + Y_STOP) / 2, Z_ENG])
        out += [_w(2.5, mid, GRIP, "slide it half way along the slot", rl),
                _w(0.5, mid, GRIP, "settle", rl)]
        stop = mid
    else:
        out += [_w(5.0, end, GRIP, "slide it along the slot to the closed end", rl),
                _w(0.8, end, GRIP, "settle", rl)]
        stop = end
    out += [_w(1.0, stop, OPEN_C, "let go", rl),
            _w(1.5, stop + np.array([-0.050, 0.0, 0.0]), OPEN_C, "back away from the door", rl),
            _w(1.5, stop + np.array([-0.080, 0.0, 0.04]), OPEN_C, "rise clear", rl),
            _w(10.0, stop + np.array([-0.080, 0.0, 0.04]), OPEN_C, "stay clear", rl)]
    return out


def demo():
    return {"left": _left(), "right": _stay("right")}
