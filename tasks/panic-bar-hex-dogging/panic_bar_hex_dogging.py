"""The lock-edge corner of a closed door and its frame lie flat on the bench, push face up, with a rim exit device
(push bar) on the door. Its latch bolt sticks out past the door edge into a rim strike on the frame jamb, so the
door is latched. The 29.8 cm push bar rides 25 mm down on a slider against a return spring (15 N at rest, 40 N
fully down) and draws the latch bolt into the mechanism case as it goes; with the bolt drawn in 18 mm or more its
end is clear of the strike and the door is free to swing. The case top has a 6 mm dogging hole; a 4 mm hex L-key
(60 mm long leg) stands in a key stand nearby.

The job ("hex dogging"): hold the bar fully down, put the short leg of the key into the dogging hole and turn it a
quarter turn against its detent, so the dogging cam catches the bar and holds it down (latch retracted). Then let
go of the bar, pull the key out and lay it down. The cam only catches the bar if the bar is down 23 mm or more;
otherwise it springs back to 0 when the key is let go.

Mechanism parts come from mechkit.py (in this folder), which adds joints to the stock runtime in memory. The key
couples to the cam through six pre-built, disabled prismatic joints (one per 60 deg seat angle, as a hex key has),
switched on when the key's short leg is down in the hole and off when it is drawn 10 mm back out.

Run it:  inspect-robots run --embodiment yam_arena -E task=tasks/panic-bar-hex-dogging/panic_bar_hex_dogging.py \\
           -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 --max-steps 1200 \\
           --instruction "<the instruction in task.json>" --scorer task_score ...
"""

import math
import os
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import mechkit as mk  # noqa: E402

from yam_arena.constants import FINGER_CLOSED  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]

WOOD = (0.72, 0.58, 0.40)
DARK = (0.18, 0.19, 0.21)
STEEL = (0.70, 0.72, 0.74)
BRASS = (0.78, 0.62, 0.25)
BLACK = (0.06, 0.06, 0.07)
BLUE = (0.15, 0.35, 0.75)

# -- door, frame and device --------------------------------------------------------------------------------------
Z0 = TOP + 0.012  # push face of the door (and face of the frame), 1.2 cm above the bench top
LEAF_T = 0.044  # door leaf thickness; it lies in a recess in the bench, so only its top 1.2 cm shows
DOOR_EDGE = -0.247  # the door's lock edge (y)
JAMB_Y = (-0.300, -0.250)  # frame jamb face, 3 mm gap to the door edge
STRIKE_X, STRIKE_Y, STRIKE_T = (0.252, 0.288), (-0.292, -0.252), Z0 + 0.032  # rim strike on the jamb face
POCKET_X, POCKET_Y0, POCKET_FLOOR = (0.259, 0.281), -0.274, Z0 + 0.009  # keeper pocket, open toward the door
BX = 0.270  # bar / case centre line (x)
CASE_X, CASE_Y = (0.245, 0.295), (-0.245, -0.150)
CT = Z0 + 0.045  # case top
HX, HY = 0.270, -0.190  # dogging hole axis
HOLE_H = 0.0033  # half-width of the hole (6.6 mm in the model, 6 mm nominal)
HOLE_D = 0.016  # hole depth
INSERT = 0.010  # a seated key's short leg goes 10 mm down the hole, onto the cam
SEAT_Z = CT - INSERT

BAR = (0.035, 0.298, 0.015)
BAR_Z = Z0 + 0.030 + BAR[2] / 2  # bar centre at rest
BAR_TRAVEL = 0.025
BAR_K, BAR_REST = 1000.0, -0.015  # 15 N at rest, 40 N fully down
DOG_TRAVEL = 0.023  # the cam only catches the bar down this far
DOG_HOLD = 0.024  # where the caught bar is held
LATCH_PER_BAR = 0.84  # latch bolt retraction per mm of bar travel (21 mm at full travel)
LATCH_OK = 0.018  # bolt end at y -0.248 or more: 4 mm clear of the strike face, the door free to swing


def _holed_block(x0, x1, y0, y1, z0, z1, cx, cy, h, depth, color):
    """A block with a square hole of half-width h, depth deep, down from its top at (cx, cy)."""
    zm = z1 - depth
    return [mk.box((x1 - x0, y1 - y0, zm - z0), at=((x0 + x1) / 2, (y0 + y1) / 2, (z0 + zm) / 2), color=color),
            mk.box((cx - h - x0, y1 - y0, depth), at=((x0 + cx - h) / 2, (y0 + y1) / 2, (zm + z1) / 2), color=color),
            mk.box((x1 - cx - h, y1 - y0, depth), at=((cx + h + x1) / 2, (y0 + y1) / 2, (zm + z1) / 2), color=color),
            mk.box((2 * h, cy - h - y0, depth), at=(cx, (y0 + cy - h) / 2, (zm + z1) / 2), color=color),
            mk.box((2 * h, y1 - cy - h, depth), at=(cx, (cy + h + y1) / 2, (zm + z1) / 2), color=color)]


kit = mk.Kit()
kit.part("case", (0, 0, 0), _holed_block(*CASE_X, *CASE_Y, Z0, CT, HX, HY, HOLE_H, HOLE_D, DARK), mass=None,
         color=DARK)
kit.part("bar", (BX, 0.0, BAR_Z), [mk.box(BAR, color=STEEL)], mass=0.40, color=STEEL)
kit.slider("bar_slide", "case", "bar", axis=(0, 0, -1), lower=0.0, upper=BAR_TRAVEL, stiffness=BAR_K, rest=BAR_REST,
           damping=20.0)
BOLT0 = (BX, -0.255, Z0 + 0.020)
kit.part("bolt", BOLT0, [mk.box((0.016, 0.022, 0.016))], mass=0.05, color=BRASS)
kit.slider("bolt_slide", "case", "bolt", axis=(0, 1, 0), lower=0.0, upper=0.021, stiffness=2000.0, rest=-0.002,
           damping=10.0)
# dogging cam: a square drive at the bottom of the hole on a disc inside the case
kit.part("cam", (HX, HY, SEAT_Z), [mk.box((0.0055, 0.0055, 0.006), at=(0, 0, -0.003)),
                                    mk.box((0.020, 0.020, 0.004), at=(0, 0, -0.010))], mass=0.05, color=BRASS)
CAM_STOP_DEG = 94.0  # the cam's hard stop: it cannot be turned further than this
kit.hinge("cam_turn", "case", "cam", anchor=(HX, HY, SEAT_Z), axis=(0, 0, 1), lower_deg=-10.0, upper_deg=CAM_STOP_DEG,
          stiffness=0.15, rest_deg=0.0, damping=0.004)
_dog = kit.parts["bar"].ref.copy()
_dog[2, 3] -= DOG_HOLD
kit.catch("bar_dog", "bar", _dog)

# hex key: frame at the tip of the short leg, z up the short leg, long leg along +x at the top
KEY_SHORT, KEY_LONG, KEY_W = 0.026, 0.060, 0.004
KEY_YAW0 = math.radians(-45.0)
KEY_BOXES = [mk.box((KEY_W, KEY_W, KEY_SHORT - KEY_W), at=(0, 0, (KEY_SHORT - KEY_W) / 2)),
             mk.box((KEY_LONG, KEY_W, KEY_W), at=(KEY_LONG / 2 - KEY_W / 2, 0, KEY_SHORT - KEY_W / 2))]
STAND = (0.200, -0.100)
STAND_TOP, STAND_D, STAND_H = Z0 + 0.030, 0.025, HOLE_H
# the long leg lies on the stand's top and on a post near its end; the short leg hangs in the stand's hole
KEY0 = (STAND[0], STAND[1], STAND_TOP - (KEY_SHORT - KEY_W))
REST = (STAND[0] + 0.052 * math.cos(KEY_YAW0), STAND[1] + 0.052 * math.sin(KEY_YAW0))
kit.part("key", KEY0, KEY_BOXES, mass=0.020, color=BLACK, yaw=KEY_YAW0)

FRAME = (0.42, 0.43, 0.45)
STRIKE = (0.55, 0.56, 0.58)
_SX, _SY = STRIKE_X, STRIKE_Y
_STRIKE_BOXES = [  # floor under the bolt, two cheeks and a back wall: a pocket open toward the door and upward
    ((_SX[0], _SX[1]), (_SY[0], _SY[1]), (Z0, POCKET_FLOOR)),
    ((_SX[0], POCKET_X[0]), (_SY[0], _SY[1]), (POCKET_FLOOR, STRIKE_T)),
    ((POCKET_X[1], _SX[1]), (_SY[0], _SY[1]), (POCKET_FLOOR, STRIKE_T)),
    ((POCKET_X[0], POCKET_X[1]), (_SY[0], POCKET_Y0), (POCKET_FLOOR, STRIKE_T)),
]
FIXTURES = [
    Box("door_leaf", pos=(0.270, (DOOR_EDGE + 0.283) / 2, Z0 - LEAF_T / 2), size=(0.300, 0.283 - DOOR_EDGE, LEAF_T),
        color=WOOD),
    Box("frame_jamb", pos=(0.270, sum(JAMB_Y) / 2, Z0 - LEAF_T / 2), size=(0.300, JAMB_Y[1] - JAMB_Y[0], LEAF_T),
        color=FRAME),
    *[Box(f"strike{i}", pos=((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2), size=(x1 - x0, y1 - y0, z1 - z0),
          color=STRIKE) for i, ((x0, x1), (y0, y1), (z0, z1)) in enumerate(_STRIKE_BOXES)],
    Box("end_cap", pos=(BX, 0.1645, (Z0 + CT) / 2), size=(0.050, 0.030, CT - Z0), color=DARK),
    *[Box(f"stand{i}", pos=b.at, size=b.size, color=BLUE)
      for i, b in enumerate(_holed_block(STAND[0] - 0.010, STAND[0] + 0.010, STAND[1] - 0.010, STAND[1] + 0.010,
                                         Z0, STAND_TOP, *STAND, STAND_H, STAND_D, BLUE))],
    Box("stand_post", pos=(REST[0], REST[1], (Z0 + STAND_TOP) / 2), size=(0.008, 0.008, STAND_TOP - Z0), color=BLUE),
]
OBJECTS = [*kit.objects(), *FIXTURES]

# -- the key-in-socket joints (authored disabled with the scene, switched on and off by _key_rule) -----------------
N_FLATS = 6


def _seat_pose(i):
    return mk._T(mk.axis_angle((0, 0, 1), KEY_YAW0 + i * math.pi / 3), (HX, HY, SEAT_Z))


def _author_key_joints(stage):
    from pxr import Gf, Sdf, UsdPhysics

    paths = mk._SIM["paths"]
    if "key" not in paths or "cam" not in paths:
        return
    C = kit.parts["cam"].ref
    F = mk._T(mk._frame_with_x((0, 0, 1)), (HX, HY, SEAT_Z))
    f0 = mk._inv(C) @ F

    def gq(r):
        x, y, z, w = mk.mat_to_quat(r)
        return Gf.Quatf(float(w), Gf.Vec3f(float(x), float(y), float(z)))

    for i in range(N_FLATS):
        f1 = mk._inv(_seat_pose(i)) @ F
        path = f"{paths['key'].rsplit('/', 1)[0]}/mechkit_joints/key_socket_{i}"
        j = UsdPhysics.PrismaticJoint.Define(stage, path)
        j.CreateBody0Rel().SetTargets([Sdf.Path(paths["cam"])])
        j.CreateBody1Rel().SetTargets([Sdf.Path(paths["key"])])
        j.CreateLocalPos0Attr().Set(Gf.Vec3f(*[float(v) for v in f0[:3, 3]]))
        j.CreateLocalRot0Attr().Set(gq(f0[:3, :3]))
        j.CreateLocalPos1Attr().Set(Gf.Vec3f(*[float(v) for v in f1[:3, 3]]))
        j.CreateLocalRot1Attr().Set(gq(f1[:3, :3]))
        j.CreateAxisAttr().Set("X")
        j.CreateLowerLimitAttr().Set(0.0)
        j.CreateUpperLimitAttr().Set(0.016)
        j.CreateCollisionEnabledAttr().Set(False)
        j.CreateExcludeFromArticulationAttr().Set(True)
        j.CreateJointEnabledAttr().Set(False)
        mk._SIM["catch_paths"][(kit.id, f"key_socket_{i}")] = path  # opened at every reset with the catches


_author_all0 = mk._author_all


def _author_all(stage, env_path):
    _author_all0(stage, env_path)
    _author_key_joints(stage)


mk._author_all = _author_all


def _disengage(name):
    path = mk._SIM["catch_paths"].get((kit.id, name))
    if path and mk._SIM["stage"] is not None:
        from pxr import UsdPhysics

        UsdPhysics.Joint(mk._SIM["stage"].GetPrimAtPath(path)).GetJointEnabledAttr().Set(False)


# -- readouts ----------------------------------------------------------------------------------------------------


def travel(objects) -> float:
    return kit.q(objects, "bar_slide")


def latch(objects) -> float:
    return kit.q(objects, "bolt_slide")


def cam_deg(objects) -> float:
    return kit.q_deg(objects, "cam_turn")


def _key_tip(objects):
    return kit.point(objects, "key", (0, 0, 0))


def _key_axis(objects):
    _, q = kit.pose(objects, "key")
    return mk.quat_to_mat(q)


def key_in_hole(objects) -> bool:
    """The short leg's tip is inside the dogging hole (within 3 mm of its axis, below the case top)."""
    t = _key_tip(objects)
    return math.hypot(float(t[0]) - HX, float(t[1]) - HY) < 0.003 + 1e-9 and float(t[2]) < CT - 0.001


def key_seated(objects) -> bool:
    t = _key_tip(objects)
    return key_in_hole(objects) and float(t[2]) <= SEAT_Z + 0.003 and mk.MEMORY.get("key_in") is not None


def key_down(objects) -> bool:
    """The key lies on the door, frame or bench (its centre no lower than 5 mm below the bench top and 15 mm or
    less above the door face), 3 cm or more from the hole and the stand (so out of both)."""
    c = kit.point(objects, "key", (0.014, 0, 0.018))
    far = all(math.hypot(float(c[0]) - x, float(c[1]) - y) >= 0.030 for x, y in ((HX, HY), STAND))
    return far and TOP - 0.005 <= float(c[2]) <= Z0 + 0.015


_TEST_TCPS = None


def _released(st, margin=0.020) -> bool:
    tcps = mk.tcps() if _TEST_TCPS is None else _TEST_TCPS
    lo, hi = np.asarray(st.low) - margin, np.asarray(st.high) + margin
    return not any(bool(np.all(np.asarray(t) >= lo) and np.all(np.asarray(t) <= hi)) for t in tcps)


# -- rules ---------------------------------------------------------------------------------------------------------


def _yaw_rel(objects):
    _, qc = kit.pose(objects, "cam")
    r = mk.quat_to_mat(qc).T @ _key_axis(objects)
    return math.atan2(r[1, 0], r[0, 0])


def _key_rule(objects):
    m = mk.MEMORY
    m["n"] = m.get("n", 0) + 1
    t = _key_tip(objects)
    if m.get("key_in") is None:
        up = _key_axis(objects)[:, 2]
        tilt = math.degrees(math.acos(max(-1.0, min(1.0, float(up[2])))))
        if (math.hypot(float(t[0]) - HX, float(t[1]) - HY) < 0.002 and float(t[2]) <= SEAT_Z + 0.006
                and tilt < 8.0):
            i = round((_yaw_rel(objects) - KEY_YAW0) / (math.pi / 3)) % N_FLATS
            kit.engage(f"key_socket_{i}")
            m["key_in"] = i
    elif float(t[2]) >= SEAT_Z + 0.010:  # drawn 10 mm back up the hole: the key comes free
        _disengage(f"key_socket_{m['key_in']}")
        m["key_in"] = None


def _cam_rule(objects):
    m = mk.MEMORY
    a, d = cam_deg(objects), travel(objects)
    if m.get("dogged"):
        if a < 45.0:
            _disengage("bar_dog")
            m["dogged"] = False
    elif a >= 80.0 and d >= DOG_TRAVEL:
        kit.engage("bar_dog")
        m["dogged"] = True
    target = 90.0 if (a >= 45.0 and (d >= DOG_TRAVEL or m.get("dogged"))) else 0.0
    if m.get("cam_target") != target:
        m["cam_target"] = target
        kit.drive("cam_turn", target)
    if d >= DOG_TRAVEL and key_seated(objects):
        m["held"] = True


def _latch_rule(objects):
    tgt = min(0.021, travel(objects) * LATCH_PER_BAR)
    tgt = tgt if tgt > 0.0005 else -0.002
    if abs(mk.MEMORY.get("bolt_target", 9) - tgt) > 0.0003:
        mk.MEMORY["bolt_target"] = tgt
        kit.drive("bolt_slide", tgt)


def _debug_rule(objects):
    if os.environ.get("PANIC_DEBUG") and mk.MEMORY.get("n", 0) % 20 == 0:
        t = _key_tip(objects)
        print(f"[panic] n={mk.MEMORY.get('n')} bar={travel(objects) * 1000:.1f} latch={latch(objects) * 1000:.1f} "
              f"cam={cam_deg(objects):.1f} tip=({float(t[0]):.4f},{float(t[1]):.4f},{float(t[2]):.4f}) "
              f"yaw={math.degrees(_yaw_rel(objects)):.1f} tcps={[tuple(round(float(v), 4) for v in p) for p in mk.tcps()]} "
              f"mem={ {k: v for k, v in mk.MEMORY.items() if k not in ('n',)} } score={score(objects)}", flush=True)


kit.on_step(_key_rule)
kit.on_step(_cam_rule)
kit.on_step(_latch_rule)
kit.on_step(_debug_rule)

SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"


# -- scoring -------------------------------------------------------------------------------------------------------


def dogged(objects) -> bool:
    return (bool(mk.MEMORY.get("dogged")) and cam_deg(objects) >= 80.0
            and travel(objects) >= DOG_TRAVEL - 0.001 and latch(objects) >= LATCH_OK)


def score(objects) -> float:
    """0.25 if the bar was ever held 23 mm or more down while the key was seated on the cam; plus 0.5 if the cam
    is at 80 deg or more (its hard stop is at 94 deg) and has caught the bar, the bar 22 mm or more down and
    the latch drawn in 18 mm or more (clear of the strike); with that, plus 0.25 if the key is out of the hole and
    lies on the door, frame or bench, no gripper is within 2 cm of the bar or the key, and both are still. A key
    left in the hole of a dogged bar scores 0.75."""
    s = 0.25 * bool(mk.MEMORY.get("held"))
    if dogged(objects):
        s += 0.5
        if (key_down(objects) and _released(objects["bar"]) and _released(objects["key"])
                and objects["key"].resting(max_speed=0.01, max_spin=0.5) and objects["bar"].resting(0.01, 0.5)):
            s += 0.25
    return round(s, 4)


def success(objects) -> bool:
    return score(objects) >= 1.0 - 1e-9


def distance(objects) -> float:
    return float(max(0.0, DOG_HOLD - travel(objects)) + abs(90.0 - cam_deg(objects)) / 1000.0)


# -- the scripted demo -------------------------------------------------------------------------------------------
# Left arm: closed fingers down on the bar at y +6 cm, commanded 7.5 cm below the bottom of its travel (the arm
# gives under the 40 N spring), held there. Right arm: pinches the key's long leg 2.5 cm out from the short leg's
# axis (fingers across the long leg), lifts it out of the stand, lowers the short leg into the dogging hole, turns the wrist a quarter
# turn (-45 to +45 deg), and once the left arm has let go of the dogged bar draws the key straight up and lays it
# on the door.

TIP = 0.013
TRAVEL = 0.840
KEY_OPEN = -0.012
GRIP_DZ = KEY_SHORT - KEY_W - 0.007 + TIP  # TCP above the key tip: pad tips 7 mm below the long leg
GRIP_OUT = 0.025  # pinched 2.5 cm out along the long leg from the short leg's axis
PRESS = (BX, 0.060)
PRESS_Z = BAR_Z + BAR[2] / 2 - BAR_TRAVEL + TIP - 0.075
DROP = (0.180, -0.200)
PARK_L, PARK_R = (0.20, 0.25, TRAVEL), (0.18, -0.26, TRAVEL)
Y0, Y1 = KEY_YAW0, KEY_YAW0 + math.pi / 2


class _Seq:
    def __init__(self):
        self.w, self.t = [], 0.0
        self.last = None

    def go(self, d, pos, f, label, yaw=0.0):
        self.w.append(Waypoint(d, tuple(float(v) for v in pos), f, label, yaw))
        self.t += d
        self.last = (tuple(pos), f, yaw)
        return self

    def until(self, t, label="wait"):
        pos, f, yaw = self.last
        return self.go(max(0.1, t - self.t), pos, f, label, yaw)


def _plans(variant=""):
    L, R = _Seq(), _Seq()
    press = variant != "no_bar"
    key = variant != "hold_only"
    if press:
        L.go(2.5, (*PRESS, TRAVEL), FINGER_CLOSED, "over the bar")
        L.go(1.5, (*PRESS, BAR_Z + BAR[2] / 2 + TIP + 0.002), FINGER_CLOSED, "onto the bar")
        L.go(2.5, (*PRESS, PRESS_Z), FINGER_CLOSED, "push the bar all the way down")
    else:
        L.go(2.0, PARK_L, FINGER_CLOSED, "park")
    if not key:
        R.go(2.0, PARK_R, FINGER_CLOSED, "park")
        L.until(55.0, "hold the bar down")
        R.until(55.0)
        return {"left": L.w, "right": R.w}
    sx, sy, sz = KEY0

    def kp(c, th, z):  # the TCP pinching the long leg, the key's axis at c, the long leg at angle th
        return (c[0] + GRIP_OUT * math.cos(th), c[1] + GRIP_OUT * math.sin(th), z)

    zs = SEAT_Z + GRIP_DZ - 0.001
    hole = (HX + 0.0026, HY)  # the key hangs ~2.6 mm behind the hand in x when lowered (measured in sim)
    R.go(2.5, kp(KEY0, Y0, TRAVEL), KEY_OPEN, "over the key", Y0)
    R.go(1.5, kp(KEY0, Y0, sz + GRIP_DZ), KEY_OPEN, "round its long leg", Y0)
    R.go(0.8, kp(KEY0, Y0, sz + GRIP_DZ), FINGER_CLOSED, "pinch the key", Y0)
    R.go(0.5, kp(KEY0, Y0, sz + GRIP_DZ), FINGER_CLOSED, "settle", Y0)
    R.go(1.5, kp(KEY0, Y0, TRAVEL + 0.002), FINGER_CLOSED, "lift it out of the stand", Y0)
    R.go(2.5, kp(hole, Y0, TRAVEL + 0.002), FINGER_CLOSED, "over the dogging hole", Y0)
    R.go(1.0, kp(hole, Y0, TRAVEL + 0.002), FINGER_CLOSED, "settle", Y0)
    R.go(1.5, kp(hole, Y0, zs), FINGER_CLOSED, "short leg down the hole", Y0)
    R.go(0.8, kp(hole, Y0, zs), FINGER_CLOSED, "settle", Y0)
    R.until(max(R.t, L.t + 0.5), "wait for the bar to be down")
    n = 9
    for k in range(1, n + 1):
        th = Y0 + (Y1 - Y0) * k / n
        R.go(2.7 / n, kp((HX, HY), th, zs), FINGER_CLOSED, "turn the key a quarter turn", th)
    if variant == "overturn":  # try to turn 30 deg past dogged: the cam's hard stop at 94 deg stops it
        for k in range(1, 4):
            th = Y1 + math.radians(10.0) * k
            R.go(0.4, kp((HX, HY), th, zs), FINGER_CLOSED, "try to turn past the stop", th)
        R.go(0.8, kp((HX, HY), Y1 + math.radians(30.0), zs), FINGER_CLOSED, "against the stop", Y1 + math.radians(30.0))
    R.go(1.0, kp((HX, HY), Y1, zs), FINGER_CLOSED, "hold", Y1)
    t_turned = R.t
    if press:
        L.until(t_turned, "hold the bar down")
        L.go(1.5, (*PRESS, TRAVEL), FINGER_CLOSED, "let the bar go")
        L.go(1.5, PARK_L, FINGER_CLOSED, "park")
    if variant == "no_bar":
        R.go(0.8, kp((HX, HY), Y1, zs), KEY_OPEN, "let go of the key", Y1)
        R.go(2.0, kp((HX, HY), Y1, zs), KEY_OPEN, "the cam springs back", Y1)
        R.go(0.5, kp((HX, HY), Y1, zs + 0.012), KEY_OPEN, "up off it", Y1)
        R.go(1.0, kp((HX, HY), Y0, zs + 0.012), KEY_OPEN, "back to it", Y0)
        R.go(0.8, kp((HX, HY), Y0, zs), KEY_OPEN, "round the key again", Y0)
        R.go(0.8, kp((HX, HY), Y0, zs), FINGER_CLOSED, "pinch it", Y0)
    else:
        R.until(L.t + 0.5, "wait for the bar to be let go")
    if variant == "key_left":
        R.go(0.8, kp((HX, HY), Y1, zs), KEY_OPEN, "let go of the key", Y1)
        R.go(1.5, kp((HX, HY), Y1, TRAVEL), KEY_OPEN, "up", Y1)
        R.go(2.0, PARK_R, KEY_OPEN, "park", 0.0)
        R.go(4.0, PARK_R, KEY_OPEN, "wait", 0.0)
        return {"left": L.w, "right": R.w}
    yk = Y0 if variant == "no_bar" else Y1
    R.go(1.5, kp((HX, HY), yk, zs + 0.020), FINGER_CLOSED, "draw the key straight out", yk)
    R.go(2.5, (*DROP, Z0 + GRIP_DZ + 0.020), FINGER_CLOSED, "over the door", yk)
    R.go(1.0, (*DROP, Z0 + GRIP_DZ + 0.006), FINGER_CLOSED, "down", yk)
    R.go(0.8, (*DROP, Z0 + GRIP_DZ + 0.006), KEY_OPEN, "let go", yk)
    R.go(1.5, (DROP[0], DROP[1], TRAVEL), KEY_OPEN, "up", yk)
    R.go(2.0, PARK_R, KEY_OPEN, "park", 0.0)
    R.go(4.0, PARK_R, KEY_OPEN, "wait", 0.0)
    L.until(R.t)
    return {"left": L.w, "right": R.w}


def demo():
    # DEMO_VARIANT: hold_only | no_bar (degenerate checks, score 0) | key_left (stage credit, 0.75) |
    # overturn (the full demo, but the key is forced 30 deg past dogged against the cam's hard stop)
    return _plans(os.environ.get("DEMO_VARIANT", ""))


# -- offline self-test -------------------------------------------------------------------------------------------


def self_test():
    global _TEST_TCPS
    I = np.array([0, 0, 0, 1.0])
    seat = (np.array([HX, HY, SEAT_Z]), mk.mat_to_quat(mk.axis_angle((0, 0, 1), Y1)))
    floor = (np.array([0.18, -0.20, 0.02]), mk.mat_to_quat(mk.axis_angle((1, 0, 0), math.pi / 2)))
    flat = (np.array([0.18, -0.20, Z0 + 0.002]), mk.mat_to_quat(mk.axis_angle((1, 0, 0), math.pi / 2)))
    stand = (np.array(KEY0), mk.mat_to_quat(mk.axis_angle((0, 0, 1), Y0)))

    def st(d=0.0, a=0.0, lat=None, key=stand):
        lat = min(0.021, d * LATCH_PER_BAR) if lat is None else lat
        return kit.fake_states(q={"bar_slide": d, "cam_turn": math.radians(a), "bolt_slide": lat},
                               moved={"key": key})

    D = {"dogged": True, "held": True}
    cases = {
        "start": (st(), [], {}, 0.0),
        "bar held down, no key": (st(0.025), [], {}, 0.0),
        "bar held, key seated, cam 0": (st(0.025, key=seat), [], {"held": True, "key_in": 0}, 0.25),
        "held then key out on the door": (st(0.0, key=flat), [], {"held": True}, 0.25),
        "dogged, key left in": (st(DOG_HOLD, 90, key=seat), [], {**D, "key_in": 0}, 0.75),
        "dogged, key on the door, all still": (st(DOG_HOLD, 90, key=flat), [], D, 1.0),
        "dogged, key on the door, hand on the bar":
            (st(DOG_HOLD, 90, key=flat), [kit.point(st(DOG_HOLD), "bar", (0, 0.06, 0.02))], D, 0.75),
        "dogged, key back in its stand": (st(DOG_HOLD, 90, key=stand), [], D, 0.75),
        "cam at 90, never dogged": (st(0.0, 90, key=flat), [], {"held": True}, 0.25),
        "dogged but cam turned back": (st(DOG_HOLD, 30, key=flat), [], D, 0.25),
        "dogged, latch not in": (st(DOG_HOLD, 90, lat=0.010, key=flat), [], D, 0.25),
        "dogged, cam against its stop": (st(DOG_HOLD, CAM_STOP_DEG, key=flat), [], D, 1.0),
        "dogged, key dropped on the floor": (st(DOG_HOLD, 90, key=floor), [], D, 0.75),
        "not dogged, key left in": (st(0.0, 0, key=seat), [], {"key_in": 0}, 0.0),
        "dogged without the seated hold (impossible)": (st(DOG_HOLD, 90, key=flat), [], {"dogged": True}, 0.75),
    }
    try:
        for name, (s_, tcps, mem, want) in cases.items():
            mk.new_episode()
            _TEST_TCPS = tcps
            mk.MEMORY.update(mem)
            got = score(s_)
            assert abs(got - want) < 1e-9, (name, got, want)
        assert not key_in_hole(st()) and key_in_hole(st(key=seat))
    finally:
        mk.new_episode()
        _TEST_TCPS = None
    return len(cases)


if __name__ == "__main__":
    from yam_arena import ik, station
    from yam_arena.demo import ArmPlan
    from yam_arena.task_file import load

    print(f"scoring: {self_test()} cases pass")
    task = load(__file__)
    hz = 20.0
    for v in ("", "hold_only", "no_bar", "key_left", "overturn"):
        os.environ["DEMO_VARIANT"] = v
        for arm, waypoints in task.demo().items():
            plan = ArmPlan(list(waypoints), station.BASES[arm])
            q = np.array([plan.action7((k + 1) / hz)[0] for k in range(round(plan.duration * hz) + 5)])
            assert plan.worst_ik_mm < 0.5 and plan.worst_ik_deg < 0.1, (v, arm, plan.worst_ik_mm, plan.worst_ik_deg)
            assert np.all(q[:, :6] >= ik.LOWER) and np.all(q[:, :6] <= ik.UPPER), (v, arm)
            print(f"{v or 'demo'} {arm}: {plan.duration:.1f} s plan, worst IK {plan.worst_ik_mm:.3f} mm / "
                  f"{plan.worst_ik_deg:.3f} deg")
    os.environ.pop("DEMO_VARIANT")
