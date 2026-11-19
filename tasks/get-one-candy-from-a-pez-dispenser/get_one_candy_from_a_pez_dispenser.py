"""On the station table: a PEZ candy dispenser (a plain one-piece stem with a cartoon head on a spring hinge, loaded
with 12 candies) lies on its back on the table, and a small white ceramic dish stands beside its head. Task
(instruction): get exactly one candy out of the dispenser into the dish, then leave the dispenser on the table. The
instruction does not say how.

How it works (the dispenser's patent, US 2,620,061, "Pocket article dispensing container"): a spring under a movable
base pushes the stack of candies up the stem; the head is a lid on a hinge at the back of the stem's top; tipping it
back opens the top of the stem so the top candy can come out, and "the lid is automatically returned to the closed
position by the spring" when the thumb lets go. The candies (Wikipedia, "Pez"): "a pressed, dry, straight-edged,
curved-corner block 15 mm x 8 mm x 5 mm", "each Pez dispenser holding 12 candy pieces".

The hand skill: the dispenser weighs 22 g and the head opens only against its spring, so pushing the head on a
dispenser nobody holds just shoves the dispenser (on the table the head cannot even turn: its crown would have to go
down through the table top). One hand holds the stem, off the table and tilted head-down over the dish; the other
pushes the head back by its face and keeps it there until the top candy has slid out over the open head and dropped
into the dish; let go early and the spring shuts the head on the candy again. Then the head is let go (it
springs shut) and the dispenser is laid back on the table.

World frame (m): x forward from the arm bases, y to the robot's left, z up; the table top is at z 0.7475.
- dispenser (lying on its back, its axis along y, the foot at +y, the head toward -y): loose, a PhysX articulation.
  Stem 78 mm long, 19 mm wide (x), 12 mm deep (z, back on the table), 18 g with the 11 candies below the top one (the
  stem is opaque; the stack and its spring follower are part of the stem body here). Its top end has the top candy's
  seat: a channel 16 x 9 mm and 6 mm long, open at the top end only. Head: 33 mm wide (x), 24 mm deep (z), 30 mm tall
  (along -y), 4 g, on a hinge along x at the back edge of the stem's top (a torsion spring, 0.03 N m/rad, damping
  0.0005 N m s/rad; 0-100 deg). Closed, its underside covers the seat.
- candy: loose, 15 x 8 x 5 mm (x, z, y here: lying flat in its seat), 0.71 g, friction 0.22 / 0.19 (the dispenser's
  material takes the lower friction of a pair).
- dish: loose white ceramic sauce dish, 100 mm across, 25 mm tall, 4 mm walls and floor, 120 g, at (0.15, -0.07).

Opening: with the stem tilted head-down by T and the head tipped back by angle a, the head's underside becomes a ramp
beyond the open end of the seat; the candy slides out over it once it slopes down enough, T + a >= about 103 deg (with
the demo's 55 deg tilt, a head angle of about 48 deg: the candy started out at 48 deg and was in the dish by 54 deg in
the demo run; with the head pushed only to 40 deg it stays in). Below that
the underside still stands across the seat's mouth. A head let go before the candy has left swings shut on it (the
candy then stays in, or is knocked off wherever it lies).

Score (from the trial's end; 0 for the rest of the run once the dispenser is dropped):
- 0: the dispenser was dropped (it hit something moving down at more than 0.5 m/s with no finger within 1 cm of it).
- candies in the dish: a candy counts when its centre is within the dish's inner radius less 4 mm, at most 12 mm above
  the dish's floor, and it is still. 1 candy: 1.0 when the dispenser is set down (its lowest point within 5 mm of the
  table top), let go (every finger point 2 cm or more from it) and still (moved under 1 mm in 1 s), else 0.5.
  2 or more (a real trial only: in simulation only the top candy is loose): 0.5 set down, else 0.25.
- no candy in the dish: 0.1 when the head was tipped back 45 deg or more at some time while the dispenser was off the
  table (its lowest point 2 cm or more above the table top), else 0.
- 1.0 is success (held 1 s).
"""

from __future__ import annotations

import itertools
import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

from bodykit import Body, BoxShape, CylShape, Link, Linkage, SphereShape, allow_matrix_grips  # noqa: E402
from bodykit import as_numpy, on_reset, on_step, plate, rotation  # noqa: E402
from yam_arena.demo import Waypoint, grip_rotation  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

allow_matrix_grips()

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"
MM = 0.001


def _f(name: str, default: str) -> float:
    return float(os.environ.get(f"PEZ_{name}", default))


# -- the layout ----------------------------------------------------------------------------------------------------
X0 = 0.15  # the dispenser's axis (x)
Y_FOOT = 0.115  # the stem's foot end
STEM_L, STEM_W, STEM_D = 0.078, 0.019, 0.012
Y_TOP = Y_FOOT - STEM_L  # the stem's top end (the hinge line)
SEAT_L = 0.006  # the top candy's seat, open at the top end
CH_W, CH_D = 0.016, 0.009  # the channel's inside, width (x) and depth (z)
WALL = 0.0015
HEAD_W, HEAD_D, HEAD_H = 0.033, 0.024, 0.030
GAP = 0.0003  # between the stem's top and the closed head's underside
LIFT = 0.0005  # everything spawns this far above the table
CANDY = (0.015, 0.005, 0.008)  # x, y, z as it lies in the seat
CANDY_M = 0.00071
CANDY_MU = _f("CANDY_MU", "0.22")  # smooth pressed sugar on glossy plastic (estimate)
HINGE = (X0, Y_TOP, TOP + LIFT + 0.0005)
SPRING = _f("SPRING", "0.03")  # N m/rad
HEAD_MAX = 100.0  # deg

DISH_C = (0.15, -0.07)
DISH_R, DISH_H, DISH_T = 0.050, 0.025, 0.004

BLUE = (0.12, 0.35, 0.80)
YELLOW = (0.98, 0.80, 0.15)
DARK = (0.08, 0.08, 0.08)
RED = (0.85, 0.15, 0.15)
PINK = (0.95, 0.45, 0.60)
WHITE = (0.94, 0.94, 0.92)


def _v(p) -> tuple:
    return tuple(float(x) for x in p)


def _dispenser() -> Linkage:
    o = np.array([X0, (Y_TOP + Y_FOOT) / 2, TOP + LIFT])  # the linkage origin: under the stem's middle
    rel = lambda p: _v(np.asarray(p, float) - o)  # noqa: E731
    z0 = TOP + LIFT
    yc_seat = Y_TOP + SEAT_L / 2
    # the stem below the seat (with the stack and its follower inside: one body)
    base = [BoxShape(rel((X0, (Y_TOP + SEAT_L + Y_FOOT) / 2, z0 + STEM_D / 2)), (STEM_W, STEM_L - SEAT_L, STEM_D), BLUE),
            # the seat: back wall (on the table), front wall, two side walls
            BoxShape(rel((X0, yc_seat, z0 + WALL / 2)), (STEM_W, SEAT_L, WALL), BLUE),
            BoxShape(rel((X0, yc_seat, z0 + STEM_D - WALL / 2)), (STEM_W, SEAT_L, WALL), BLUE),
            BoxShape(rel((X0 + (CH_W + STEM_W) / 4, yc_seat, z0 + STEM_D / 2)), ((STEM_W - CH_W) / 2, SEAT_L, STEM_D), BLUE),
            BoxShape(rel((X0 - (CH_W + STEM_W) / 4, yc_seat, z0 + STEM_D / 2)), ((STEM_W - CH_W) / 2, SEAT_L, STEM_D), BLUE),
            # the next candy's face at the seat's floor (visual)
            BoxShape(rel((X0, Y_TOP + SEAT_L + 0.0002, z0 + STEM_D / 2)), (CH_W - 0.001, 0.0004, CH_D - 0.001), PINK,
                     collide=False)]
    yh = Y_TOP - GAP - HEAD_H / 2
    zf = z0 + HEAD_D  # the face (up while lying on its back)
    head = [BoxShape(rel((X0, yh, z0 + HEAD_D / 2)), (HEAD_W, HEAD_H, HEAD_D), YELLOW),
            # hair on the crown, eyes, nose and mouth on the face (visual)
            BoxShape(rel((X0, Y_TOP - GAP - HEAD_H - 0.002, z0 + HEAD_D / 2)), (HEAD_W - 0.004, 0.004, HEAD_D - 0.004), RED,
                     collide=False),
            CylShape(rel((X0 - 0.007, yh - 0.004, zf + 0.0004)), 0.0035, 0.0008, DARK, collide=False),
            CylShape(rel((X0 + 0.007, yh - 0.004, zf + 0.0004)), 0.0035, 0.0008, DARK, collide=False),
            CylShape(rel((X0, yh + 0.003, zf + 0.0006)), 0.003, 0.0012, RED, collide=False),
            BoxShape(rel((X0, yh + 0.009, zf + 0.0004)), (0.014, 0.002, 0.0008), DARK, collide=False)]
    links = [Link("head", tuple(head), mass=0.004, anchor=HINGE, axis="X", limits=(0.0, HEAD_MAX),
                  stiffness=SPRING, spring_damping=0.0005)]
    return Linkage.build("pez", pos=_v(o), mass=0.018, color=BLUE, floating=True, friction=(0.45, 0.40),
                         friction_combine="min", shapes=base, links=links, contact_offset=0.001, solver_iterations=48)


def _dish() -> Body:
    n = 28
    shapes = [CylShape((0.0, 0.0, DISH_T / 2), DISH_R - 0.001, DISH_T, WHITE)]
    r = DISH_R - DISH_T / 2
    for k in range(n):
        a0, a1 = 2 * math.pi * k / n, 2 * math.pi * (k + 1) / n
        p1 = (r * math.cos(a0), r * math.sin(a0), DISH_H / 2)
        p2 = (r * math.cos(a1), r * math.sin(a1), DISH_H / 2)
        mid = ((a0 + a1) / 2)
        shapes.append(plate(p1, p2, DISH_H, DISH_T, (math.cos(mid), math.sin(mid), 0.0), WHITE))
    return Body.build("dish", pos=(DISH_C[0], DISH_C[1], TOP + 0.0005), mass=0.12, shapes=shapes, color=WHITE,
                      friction=(0.8, 0.7), contact_offset=0.001)


CANDY_P0 = np.array([X0, Y_TOP + SEAT_L - CANDY[1] / 2 - 0.0005, TOP + LIFT + WALL + CANDY[2] / 2 + 0.0004])

PEZ = _dispenser()
DISH = _dish()
CANDY_B = Body.build("candy", pos=_v(CANDY_P0), mass=CANDY_M, color=PINK,
                     shapes=[BoxShape((0.0, 0.0, 0.0), CANDY, PINK)], friction=(CANDY_MU, CANDY_MU * 0.85), contact_offset=0.001,
                     solver_iterations=32)
OBJECTS = [DISH, PEZ, CANDY_B]

# -- tracking and scoring -------------------------------------------------------------------------------------------
DROP_V = 0.5  # m/s
DROP_CLEAR = 0.01
CLEAR = 0.02
SET_DOWN = 0.005
OFF_TABLE = 0.02
OPEN_DEG = 45.0
IN_DISH_R = DISH_R - DISH_T - 0.004
IN_DISH_H = 0.012
STILL_MM = 1.0
STILL_STEPS = 20

_DEBUG = os.environ.get("PEZ_DEBUG") == "1"
_DEBUG_EVERY = int(os.environ.get("PEZ_DEBUG_EVERY", "5"))
_EP: dict = {}
_IDS: dict = {}

# the dispenser's outline in its base frame (closed head), for clearances and its lowest point
_O = np.array(PEZ.pos)
_HULL = np.array([[x, y, z] for x in (X0 - HEAD_W / 2, X0 + HEAD_W / 2)
                  for y in (Y_TOP - GAP - HEAD_H - 0.004, Y_FOOT) for z in (TOP + LIFT, TOP + LIFT + HEAD_D)]) - _O
_STEM_PTS = np.array([[x, y, z] for x in (X0 - STEM_W / 2, X0 + STEM_W / 2)
                      for y in (Y_TOP, Y_FOOT) for z in (TOP + LIFT, TOP + LIFT + STEM_D)]) - _O
_HEAD_PTS = np.array([[x, y, z] for x in (X0 - HEAD_W / 2, X0 + HEAD_W / 2)
                      for y in (Y_TOP - GAP - HEAD_H - 0.004, Y_TOP - GAP) for z in (TOP + LIFT, TOP + LIFT + HEAD_D)]) - _O
_HINGE_B = np.array(HINGE) - _O


@on_reset
def _reset() -> None:
    _EP.update(tick=0, step=0, failed="", opened=False, max_head=0.0, prev=None, recent=[], crecent=[], near=1.0,
               head=0.0, low=0.0, vz=0.0, n_in=0, gaps={}, tcp={}, cp=None)


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


def _box_dist(pts_local: np.ndarray, lo: np.ndarray, hi: np.ndarray) -> float:
    d = np.maximum(np.maximum(lo - pts_local, pts_local - hi), 0.0)
    return float(np.linalg.norm(d, axis=1).min())


@on_step
def _track(env) -> None:
    try:
        _track_inner(env)
    except Exception:
        import traceback

        print("PEZ_TRACEBACK\n" + traceback.format_exc(), flush=True)
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
        mine = []
        for i, n in zip(_IDS[side + "_tips"], ("tip_left", "tip_right")):
            mine.append(_IDS["fingers"][n] @ rotation(poses[i][3:7]).T + poses[i][:3] - origin)
        fingers += mine
    fingers = np.concatenate(fingers)

    pez = base.scene["pez"]
    pose = as_numpy(pez.data.root_link_pose_w)[0]
    p, r = pose[:3] - origin, rotation(pose[3:7])
    names = list(pez.joint_names)
    head = math.degrees(float(as_numpy(pez.data.joint_pos)[0][names.index("head_turn")]))
    # the lowest point: the stem's corners and the head's corners turned about the hinge
    a = math.radians(head)
    rh = np.array([[1, 0, 0], [0, math.cos(a), -math.sin(a)], [0, math.sin(a), math.cos(a)]])
    head_pts = (_HEAD_PTS - _HINGE_B) @ rh.T + _HINGE_B
    pts = np.concatenate([_STEM_PTS, head_pts]) @ r.T + p
    low = float(pts[:, 2].min()) - TOP
    loc = (fingers - p) @ r  # finger points in the base frame
    lo, hi = _HULL.min(0), _HULL.max(0)
    near = _box_dist(loc, lo, hi)
    prev = _EP["prev"]
    vz = 0.0 if prev is None else (p[2] - prev[2]) * 20.0
    _EP["prev"] = p.copy()
    if (not _EP["failed"] and vz < -DROP_V and near > DROP_CLEAR and _EP["tick"] > 5):
        _EP["failed"] = f"dropped (falling at {-vz:.2f} m/s with no finger within 1 cm) at step {_EP['tick']}"
    if head >= OPEN_DEG and low >= OFF_TABLE:
        _EP["opened"] = True
    candy = base.scene["candy"]
    cp = as_numpy(candy.data.root_link_pose_w)[0][:3] - origin
    dish = base.scene["dish"]
    dp = as_numpy(dish.data.root_link_pose_w)[0][:3] - origin
    _EP.update(head=head, max_head=max(_EP["max_head"], head), low=low, near=near, vz=vz, p=p, cp=cp, dp=dp)


def _tally(o) -> dict:
    if "p" not in _EP:
        return dict(n=0, down=False, still=False, done=False)
    if _EP.get("counted") != _EP["tick"]:
        _EP["counted"] = _EP["tick"]
        for key, val in (("recent", _EP["p"]), ("crecent", _EP["cp"])):
            _EP[key].append(np.asarray(val, float).copy())
            del _EP[key][:-STILL_STEPS]
    full = len(_EP["recent"]) == STILL_STEPS
    still = full and all(np.abs(q - _EP["recent"][-1]).max() * 1000 < STILL_MM for q in _EP["recent"])
    cstill = full and all(np.abs(q - _EP["crecent"][-1]).max() * 1000 < STILL_MM for q in _EP["crecent"])
    cp, dp = _EP["cp"], _EP["dp"]
    in_dish = (math.hypot(cp[0] - dp[0], cp[1] - dp[1]) <= IN_DISH_R and cp[2] - (dp[2] + DISH_T) <= IN_DISH_H
               and cp[2] > dp[2] and cstill)
    n = 1 if in_dish else 0
    down = _EP["low"] <= SET_DOWN and _EP["near"] >= CLEAR and still
    return dict(n=n, down=down, still=still, done=bool(n == 1 and down))


def _debug(t) -> None:
    _EP["step"] += 1
    if (_EP["step"] - 1) % _DEBUG_EVERY:
        return
    p, cp = _EP.get("p", np.zeros(3)), _EP.get("cp")
    cp = np.zeros(3) if cp is None else cp
    print(f"DBG step={_EP['step']} head={_EP['head']:.1f}deg max={_EP['max_head']:.1f} low={_EP['low'] * 1000:.1f}mm "
          f"near={_EP['near'] * 1000:.0f}mm vz={_EP['vz']:+.2f} pez=({p[0]:.3f},{p[1]:.3f},{p[2] - TOP:.3f}) "
          f"candy=({cp[0]:.3f},{cp[1]:.3f},{cp[2] - TOP:.3f}) n={t['n']} down={int(t['down'])} "
          f"opened={int(_EP['opened'])} failed={_EP['failed'] or '-'} | "
          + " ".join(f"{k} tcp=({v[0]:.3f},{v[1]:.3f},{v[2] - TOP:.3f}) gap={_EP['gaps'][k] * 1000:.0f}"
                     for k, v in _EP["tcp"].items()), flush=True)


def score(objects) -> float:
    t = _tally(objects)
    if _DEBUG:
        _debug(t)
    if _EP["failed"]:
        return 0.0
    if t["n"] == 1:
        return 1.0 if t["down"] else 0.5
    if t["n"] >= 2:
        return 0.5 if t["down"] else 0.25
    return 0.1 if _EP["opened"] else 0.0


def success(objects) -> bool:
    t = _tally(objects)
    return bool(t["done"] and not _EP["failed"])


# -- scripted demo --------------------------------------------------------------------------------------------------
# Open loop, from the known layout. The left gripper, tilted 50 deg (fingers toward the head), closes across the stem
# 15 mm from its foot, lifts the dispenser and turns it head-down 55 deg over the dish. The right gripper, pointing
# toward +y 75 deg from straight down (nearly level), sets its fingertips on the head's face 15 mm to one side of the
# axis (clear of the candy's path) and pushes it back round its hinge, aiming 8 mm past the face and at 90 deg (it gets
# to about 70 deg: the arm gives), and holds it there while the candy slides out over the open head and drops into the
# dish. It backs off (the head springs shut); the left turns the dispenser back level, lays it on its back where it
# was, lets go and rises clear. PEZ_ONE=1 (wrong way): the right hand alone pushes the face of the dispenser lying on
# the table; the left stays at the ready pose.
OPEN_C = -0.047
ONE = os.environ.get("PEZ_ONE") == "1"
TILT = _f("TILT", "55")  # deg, head-down
PINCH = -(STEM_W / 2 - 0.0035)
G_DY = _f("G_DY", "0.015")  # the grip this far from the foot
G_DZ = _f("G_DZ", "0.011")  # the TCP this far above the table at the grip
HOLD = np.array([X0, _f("HOLD_Y", "-0.012"), TOP + _f("HOLD_Z", "0.150")])
HEAD_TO = _f("HEAD_TO", "90")  # deg the right hand pushes the head to
PUSH_H = _f("PUSH_H", "0.022")  # the push point on the face this far out from the stem's top
PUSH_X = _f("PUSH_X", "0.015")  # the right TCP this far beside the axis (+x)
TIP_DEPTH = 0.0136  # the closed fingertips' deepest point past the TCP
PUSH_OVER = _f("PUSH_OVER", "0.008")  # commanded this far past the contact
PSI_R = _f("PSI_R", "75")  # the right hand points this far from straight down toward +y
CLOSED = 0.0


def _rx(deg: float) -> np.ndarray:
    a = math.radians(deg)
    return np.array([[1.0, 0.0, 0.0], [0.0, math.cos(a), -math.sin(a)], [0.0, math.sin(a), math.cos(a)]])


ROT_DOWN_L = grip_rotation(-math.pi / 2)  # left, fingers closing along x
PICK_TILT = _f("PICK_TILT", "50")
ROT_PICK = _rx(-PICK_TILT) @ ROT_DOWN_L  # fingers toward -y (the head)
ROT_HOLD = _rx(TILT) @ ROT_PICK
ROT_R = _rx(PSI_R) @ grip_rotation(math.pi / 2)  # right, fingers closing along x, pointing down and toward +y

G0 = np.array([X0, Y_FOOT - G_DY, TOP + G_DZ])  # the left TCP on the lying dispenser


def held(p, tilt: float = TILT, at=HOLD) -> np.ndarray:
    """A point of the dispenser (lying pose) with the dispenser held: turned ``tilt`` head-down about the grip, the
    grip at ``at``."""
    return np.asarray(at, float) + _rx(tilt) @ (np.asarray(p, float) - G0)


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
    return [Waypoint(t, tuple(float(v) for v in pos), OPEN_C, "stay put", rot)]


UP = np.array([0.0, 0.0, 1.0])
T_HOLD_START = 9.0  # s: by then the dispenser hangs head-down over the dish
T_PUSH = 7.0  # s: the head pushed open and held while the candy drops


def _left() -> list:
    out = _lead_in("left", G0 + 0.06 * UP, ROT_PICK, OPEN_C, "above the stem near its foot, tilted, open")
    out += [_w(1.5, G0 + 0.015 * UP, OPEN_C, "lower round the stem", ROT_PICK),
            _w(1.0, G0, OPEN_C, "lower round the stem", ROT_PICK),
            _w(1.0, G0, PINCH, "close on the stem", ROT_PICK),
            _w(0.5, G0, PINCH, "close on the stem", ROT_PICK),
            _w(1.5, G0 + 0.06 * UP, PINCH, "lift it", ROT_PICK)]
    # turn head-down about the grip while carrying it over the dish
    mid = G0 + 0.06 * UP
    for k in range(1, 4):
        f = k / 3
        out.append(_w(1.0, mid + (HOLD - mid) * f, PINCH, "turn it head-down over the dish", _rx(TILT * f) @ ROT_PICK))
    out += [_w(T_PUSH + 4.0, HOLD, PINCH, "hold it while the head is pushed open", ROT_HOLD)]
    back = G0 + 0.004 * UP
    for k in range(1, 4):
        f = k / 3
        out.append(_w(1.0, HOLD + (back + 0.04 * UP - HOLD) * f, PINCH, "turn it back level over the table",
                      _rx(TILT * (1 - f)) @ ROT_PICK))
    out += [_w(1.5, back, PINCH, "lay it on its back", ROT_PICK),
            _w(1.0, back, OPEN_C, "let go", ROT_PICK),
            _w(1.5, back + 0.07 * UP, OPEN_C, "rise clear", ROT_PICK),
            _w(20.0, back + 0.07 * UP, OPEN_C, "stay clear", ROT_PICK)]
    return out


def _push_tcp(angle: float, over: float = 0.0, lying: bool = False) -> np.ndarray:
    """The right TCP whose fingertips touch the head's face ``PUSH_H`` out from the stem's top, the head at
    ``angle``."""
    h = np.array([X0, Y_TOP - GAP - PUSH_H, TOP + LIFT + HEAD_D])  # lying pose, head closed
    c = np.array(HINGE) + _rx(angle) @ (h - np.array(HINGE))
    c = c if lying else held(c)
    u = _rx(PSI_R) @ np.array([0.0, 0.0, -1.0])  # the gripper's approach direction (TCP -> fingertips)
    out = c + np.array([PUSH_X, 0.0, 0.0]) - u * (TIP_DEPTH - over)
    if lying:
        out[2] = max(out[2], TOP + 0.02)
    return out


def _right() -> list:
    lying = ONE
    wait = 0.0 if ONE else T_HOLD_START
    start = _push_tcp(0.0, -0.004, lying)
    out = _lead_in("right", start + 0.05 * UP, ROT_R, CLOSED, "above the head's face, closed")
    out += [_w(wait + 0.5, start + 0.05 * UP, CLOSED, "wait for the dispenser to be held", ROT_R),
            _w(1.5, start, CLOSED, "come down onto the face", ROT_R)]
    n = 8
    for k in range(1, n + 1):
        a = HEAD_TO * k / n
        out.append(_w(3.0 / n, _push_tcp(a, PUSH_OVER, lying), CLOSED, "push the head back round its hinge", ROT_R))
    out += [_w(T_PUSH - 3.0, _push_tcp(HEAD_TO, PUSH_OVER, lying), CLOSED, "hold it open while the candy drops", ROT_R),
            _w(1.0, _push_tcp(HEAD_TO, -0.02, lying) - 0.03 * np.array([0, 1.0, 0]), CLOSED, "let the head go", ROT_R),
            _w(1.5, _push_tcp(HEAD_TO, -0.02, lying) + np.array([0.0, -0.05, 0.08]), CLOSED, "back away", ROT_R),
            _w(30.0, _push_tcp(HEAD_TO, -0.02, lying) + np.array([0.0, -0.05, 0.08]), CLOSED, "stay clear", ROT_R)]
    return out


def demo():
    if ONE:
        return {"right": _right(), "left": _stay("left")}
    return {"left": _left(), "right": _right()}
