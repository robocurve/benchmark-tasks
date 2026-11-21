"""Spin a lazy Susan on a pantry shelf to bring the spice jar at the back to the front, then take it out.

A pantry shelf section (IKEA IVAR, 42 x 30 cm shelves) stands on the counter against the kitchen wall. On its bottom shelf,
at the front edge, sits an 11 in lazy Susan turntable (OXO Good Grips 1071352) carrying six 2.37 oz spice jars (McCormick
ground cinnamon size) in a ring; the shelf above is 40 mm over the jar tops, so nothing can be lifted out over the jars, and
the jars stand too close for fingers to reach between them. The jar wanted (red cap) stands at the back, by the wall.

Turn the turntable by its rim until the red-capped jar is at the front, without tipping or flinging any jar off it, then
lift the red-capped jar out and stand it on the counter mark in front of the shelf. Move no other jar.

Scene (station frame: x forward from the arm bases, y left, z up from the table top):
- shelf section: bottom shelf on the counter, front edge at x = CAB_X0, centred on y = TT_Y; upper shelf underside at
  SHELF_Z; four posts; the kitchen wall behind;
- turntable: base fixed on the bottom shelf; tray (279 mm, 10 mm lip around a 235 mm usable floor) on a steel ball bearing
  = a vertical hinge with Coulomb friction BEARING_NM (not published; the demo scores 1.0 over the sweep 0.005-0.05 N m);
- six jars, 45 x 45 x 112 mm, 89 g, on a ring of radius RING_R; the target's cap is red.

Score (``score()``), all from poses: 0 if any non-target jar is tipped (> TIP_DEG) or off the tray, if the target tips while
on the tray, or if the tray was lifted off its bearing (all latched); 0.5 once the target has come round to the front
sector (within FRONT_DEG of the shelf's front axis) upright on the tray; 1.0 with, as well, the target upright on the
counter mark (within MARK_R), no jar held and every jar still (jars_still) for SUCCESS_HOLD_S.
"""

from __future__ import annotations

import math
import pathlib
import sys

import numpy as np

_HERE = str(pathlib.Path(__file__).resolve().parent)
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

import arena_assets_fix  # noqa: E402, F401
import mechkit as mk  # noqa: E402
from yam_arena import station  # noqa: E402

SCENE = "station"
TOP = station.TABLE_TOP

WHITE, PANEL, GREY, AMBER, RED, CAP = (0.93, 0.93, 0.91), (0.88, 0.87, 0.84), (0.75, 0.76, 0.78), (0.55, 0.30, 0.12), \
    (0.80, 0.10, 0.08), (0.12, 0.12, 0.13)

# -- pantry shelf section (IKEA IVAR: 42 x 30 cm shelves, 18 mm thick) on the counter, against the kitchen wall ----------
CAB_W, CAB_D, PANEL_T = 0.420, 0.300, 0.018
POST = 0.030  # IVAR side-unit posts
CAB_X0 = 0.320  # front edge of the shelves
TT_Y = 0.020  # the shelf section stands this far toward the left arm (+y): the turning hand's stroke start is inside IK
CAB_XC = CAB_X0 + CAB_D / 2
IN_W = CAB_W
JAR = (0.045, 0.045, 0.112)
JAR_M = 0.089
TRAY_Z = 0.030  # tray floor top above the shelf
FLOOR_Z = TOP + PANEL_T  # bottom shelf top
SHELF_CLEAR = 0.040  # upper shelf underside above the jar tops
SHELF_Z = FLOOR_Z + TRAY_Z + JAR[2] + SHELF_CLEAR  # upper shelf underside
WALL_X = CAB_X0 + CAB_D + 0.005
kit = mk.Kit()
_side = [mk.box((POST, POST, SHELF_Z + PANEL_T - TOP), (dx, s * (CAB_W / 2 + POST / 2), (SHELF_Z + PANEL_T - TOP) / 2))
         for s in (-1, 1) for dx in (-CAB_D / 2 + POST / 2, CAB_D / 2 - POST / 2)]
kit.part("pantry_shelf", (CAB_XC, TT_Y, TOP), color=PANEL, boxes=[
    mk.box((CAB_D, CAB_W, PANEL_T), (0.0, 0.0, PANEL_T / 2)),  # bottom shelf
    mk.box((CAB_D, CAB_W, PANEL_T), (0.0, 0.0, SHELF_Z - TOP + PANEL_T / 2), WHITE),  # upper shelf
] + _side)
kit.part("kitchen_wall", (WALL_X + 0.005, TT_Y, TOP), color=WHITE, boxes=[mk.box((0.010, 0.70, 0.45), (0.0, 0.0, 0.225))])

# -- turntable (OXO 1071352: 11 x 1.5 in, usable 9.25 in) ---------------------------------------------------------------
TT_X = CAB_X0 + 0.1397  # set at the front edge of the shelf so it can be turned
TT_R, USE_R = 0.1397, 0.1175
LIP_H, FLOOR_T = 0.010, 0.004
N_SEG = 24
TRAY_M = 0.30  # of the 0.39 kg turntable; the rest is the fixed base ring
BEARING_NM = 0.02  # Coulomb friction of the bearing (N m); sweep 0.005-0.05 (not published)


def _ring(r_in, r_out, h, z0, n=N_SEG, color=WHITE):
    seg = 2 * math.pi * r_out / n * 1.05
    return [mk.box((r_out - r_in, seg, h), ((r_in + r_out) / 2 * math.cos(a), (r_in + r_out) / 2 * math.sin(a), z0 + h / 2),
                   color, yaw=a) for a in np.arange(n) * 2 * math.pi / n]


def _disc(r, h, z0, n=N_SEG, color=WHITE):
    seg = 2 * math.pi * r / n * 1.05
    return [mk.box((r, seg, h), (r / 2 * math.cos(a), r / 2 * math.sin(a), z0 + h / 2), color, yaw=a)
            for a in np.arange(n) * 2 * math.pi / n]


kit.part("turntable_base", (TT_X, TT_Y, FLOOR_Z), color=GREY, boxes=_disc(0.11, TRAY_Z - FLOOR_T - 0.001, 0.0, 12, GREY))
kit.part("tray", (TT_X, TT_Y, FLOOR_Z + TRAY_Z - FLOOR_T), mass=TRAY_M, color=WHITE,
         boxes=_disc(TT_R, FLOOR_T, 0.0) + _ring(USE_R, TT_R, FLOOR_T + LIP_H, 0.0))
TRAY_HINGE = kit.hinge("bearing", None, "tray", anchor=(TT_X, TT_Y, FLOOR_Z), axis=(0.0, 0.0, 1.0),
                       lower_deg=-720.0, upper_deg=720.0, friction=BEARING_NM)

# -- jars -----------------------------------------------------------------------------------------------------------------
RING_R = 0.085
N_JAR = 6
TARGET = "jar_0"
JARS = [f"jar_{i}" for i in range(N_JAR)]
JAR_Z = FLOOR_Z + TRAY_Z + JAR[2] / 2
for i, name in enumerate(JARS):
    a = i * 2 * math.pi / N_JAR  # jar_0 at angle 0 = the back (+x)
    kit.part(name, (TT_X + RING_R * math.cos(a), TT_Y + RING_R * math.sin(a), JAR_Z), mass=JAR_M, color=AMBER, yaw=a,
             boxes=[mk.box((JAR[0], JAR[1], JAR[2] - 0.018), (0.0, 0.0, -0.009)),
                    mk.box((JAR[0], JAR[1], 0.018), (0.0, 0.0, JAR[2] / 2 - 0.009), RED if name == TARGET else CAP)])

MARK = (CAB_X0 - 0.080, TT_Y)  # counter mark in front of the shelf
OBJECTS = kit.objects()

# -- scoring --------------------------------------------------------------------------------------------------------------
TIP_DEG = 15.0
FRONT_DEG = 30.0
MARK_R = 0.020
TRAY_LIFT = 0.005
SUCCESS_HOLD_S = 1.0
CONTROL_HZ = 20.0  # the run's control rate (README): one susan_step per control step
STILL_MM, STILL_DEG = 2.0, 2.0  # 'at rest': every jar moved less than this over the last SUCCESS_HOLD_S (measured from poses;
#                                  PhysX can report a few cm/s on a box that is not moving, so the reported velocity is not used)
SCORE_OVER = "end"


def tilt_deg(objects, name) -> float:
    return float(np.degrees(np.arccos(np.clip(mk.quat_to_mat(kit.pose(objects, name)[1])[2, 2], -1.0, 1.0))))


def on_tray(objects, name) -> bool:
    p = kit.pose(objects, name)[0]
    return bool(np.hypot(p[0] - TT_X, p[1] - TT_Y) <= USE_R and abs(p[2] - JAR_Z) <= 0.006)


def jar_angle_deg(objects, name) -> float:
    p = kit.pose(objects, name)[0]
    return float(np.degrees(math.atan2(p[1] - TT_Y, p[0] - TT_X)))


def _mem():
    m = mk.MEMORY.get("susan")
    if m is None:
        m = {"spoiled": False, "fronted": False}
        mk.MEMORY["susan"] = m
    return m


def susan_step(objects) -> None:
    m = _mem()
    hist = mk.MEMORY.setdefault("jar_hist", [])
    hist.append({j: (np.array(kit.pose(objects, j)[0], float), mk.quat_to_mat(kit.pose(objects, j)[1])) for j in JARS})
    del hist[:-(int(round(SUCCESS_HOLD_S * CONTROL_HZ)) + 1)]
    mk.MEMORY["tray_deg"] = math.degrees(kit.q(objects, TRAY_HINGE))
    mk.MEMORY["target_pos"] = np.array(kit.pose(objects, TARGET)[0], float)
    if abs(kit.pose(objects, "tray")[0][2] - (FLOOR_Z + TRAY_Z - FLOOR_T + 0.0)) > TRAY_LIFT + 0.01:
        m["spoiled"] = True
    for j in JARS[1:]:
        if tilt_deg(objects, j) > TIP_DEG or not on_tray(objects, j):
            m["spoiled"] = True
    if on_tray(objects, TARGET) and tilt_deg(objects, TARGET) <= TIP_DEG \
            and abs(abs(jar_angle_deg(objects, TARGET)) - 180.0) <= FRONT_DEG:
        m["fronted"] = True
    if tilt_deg(objects, TARGET) > TIP_DEG and on_tray(objects, TARGET):
        m["spoiled"] = True


kit.on_step(susan_step)


def jars_still() -> bool:
    """Every jar moved under STILL_MM and turned under STILL_DEG over the last SUCCESS_HOLD_S (needs that long a history)."""
    hist = mk.MEMORY.get("jar_hist", [])
    if len(hist) < int(round(SUCCESS_HOLD_S * CONTROL_HZ)) + 1:
        return False
    a, b = hist[0], hist[-1]
    for j in JARS:
        turn = math.degrees(math.acos(float(np.clip((np.trace(a[j][1].T @ b[j][1]) - 1.0) / 2.0, -1.0, 1.0))))
        if np.linalg.norm(b[j][0] - a[j][0]) * 1000.0 >= STILL_MM or turn >= STILL_DEG:
            return False
    return True


def target_on_mark(objects) -> bool:
    p = kit.pose(objects, TARGET)[0]
    return bool(np.hypot(p[0] - MARK[0], p[1] - MARK[1]) <= MARK_R and abs(p[2] - (TOP + JAR[2] / 2)) <= 0.004
                and tilt_deg(objects, TARGET) <= 5.0)


def score(objects) -> float:
    m = _mem()
    if m["spoiled"] or not m["fronted"]:
        return 0.0
    if target_on_mark(objects) and all(mk.released(objects[n]) for n in JARS) \
            and jars_still():
        return 1.0
    return 0.5


def success(objects) -> bool:
    return score(objects) >= 1.0 - 1e-9


def distance(objects) -> float:
    p = kit.pose(objects, TARGET)[0]
    return float(np.hypot(p[0] - MARK[0], p[1] - MARK[1]))


# -- scripted demo: horizontal grips ------------------------------------------------------------------------------------
import yam_arena.demo as _yd  # noqa: E402
from yam_arena import ik  # noqa: E402
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, PAD_TIP_PAST_TCP  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402


class Live:
    """A waypoint pose decided when the waypoint starts, from what the scene shows then (the tray's angle, the jar's
    position as the cameras see it), and kept for the rest of the episode. fn() -> (pos, rot)."""

    def __init__(self, key, fn):
        self.key, self.fn = key, fn

    def resolve(self):
        cache = mk.MEMORY.setdefault("live_wp", {})
        if self.key not in cache:
            pos, rot = self.fn()
            cache[self.key] = (np.asarray(pos, float), np.asarray(rot, float))
        return cache[self.key]


class LiveFinger(float):
    """A finger target decided when the waypoint starts (cached like Live). As a number it is the nominal (open) target,
    which is what planning-time tools that read waypoint fingers see."""

    def __new__(cls, key, fn, nominal):
        obj = super().__new__(cls, nominal)
        obj.key, obj.fn = key, fn
        return obj

    def resolve(self):
        cache = mk.MEMORY.setdefault("live_fg", {})
        if self.key not in cache:
            cache[self.key] = float(self.fn())
        return cache[self.key]


class RWaypoint(Waypoint):
    """A Waypoint with a TCP rotation ``rot`` (row-major 3x3); None: the stock top-down grip turned by yaw."""

    def __init__(self, *args, rot=None, **kwargs):
        super().__init__(*args, **kwargs)
        self.rot = rot


def _target(self, t: float):
    """yam_arena.demo.ArmPlan.target, reading a waypoint's ``rot`` when it has one (the stock plan is top-down)."""
    prev_pos, prev_rot, prev_finger = self.start_pos, self.start_rot, self.finger_start
    for w in self.waypoints:
        if isinstance(w.pos, Live):
            goal, rot = w.pos.resolve()
        else:
            rot = np.asarray(w.rot, float) if getattr(w, "rot", None) is not None else _yd.grip_rotation(w.yaw)
            goal = np.asarray(w.pos, dtype=float)
        finger = w.finger.resolve() if isinstance(w.finger, LiveFinger) else w.finger
        if t < w.duration:
            s = ik.smoothstep(t / w.duration)
            return (prev_pos + s * (goal - prev_pos), ik.slerp_matrix(prev_rot, rot, s),
                    prev_finger + s * (finger - prev_finger), w.label)
        t -= w.duration
        prev_pos, prev_rot, prev_finger = goal, rot, finger
    return prev_pos, prev_rot, prev_finger, "done"


_yd.ArmPlan.target = _target

R_FWD_V = np.array([[0.0, 0.0, 1.0], [0.0, -1.0, 0.0], [1.0, 0.0, 0.0]])  # approach +x, fingers closing along z
R_FWD_H = mk.axis_angle((1.0, 0.0, 0.0), math.pi / 2) @ R_FWD_V  # approach +x, fingers closing along y
PITCH = math.radians(20.0)
RIM_PITCH = math.radians(20.0)  # steeper on the rim: the palm stays above the cabinet floor's front edge  # tool tilted nose-down: level grips do not reach 50-60 mm above the counter


def _rot(base, yaw=0.0, pitch=PITCH):
    return mk.axis_angle((0.0, 0.0, 1.0), yaw) @ mk.axis_angle((0.0, 1.0, 0.0), pitch) @ base


def _tcp_for_tip(tip, r):
    return np.asarray(tip, float) - r[:, 2] * PAD_TIP_PAST_TCP


RIM_Z = FLOOR_Z + TRAY_Z + LIP_H - 0.003  # just under the lip's top edge
PRESS = 0.010  # pad tip this far into the lip's outer face
STROKE_DEG = 45.0
TAKE_YAW_DEG = 20.0  # the taking hand comes in from the front-right, clear of the holding hand
PINCH_IN = 0.005  # TCP this far inside the rim: the pads reach over the whole lip
STROKE_MID = 150.0  # strokes on the front-left of the lip (azimuth from +x, the back): the turning hand stays on its own side
N_STROKES = 7
BACK_OFF = 0.050  # the open fingers' return chord stays clear of the lip
LIFT = 0.016  # jar bottom over the 10 mm lip


def rim_tcp(phi, r_tip):
    """TCP aimed inward (nose-down PITCH) at azimuth phi (from +x, the back), pad tip on the lip at radius r_tip."""
    d = np.array([math.cos(phi), math.sin(phi), 0.0])
    r = _rot(R_FWD_V, phi + math.pi)
    return _tcp_for_tip(np.array([TT_X, TT_Y, RIM_Z]) + d * r_tip, r), r


LIP_MID_Z = FLOOR_Z + TRAY_Z - FLOOR_T + (FLOOR_T + LIP_H) / 2


def sep(mm):
    return max(FINGER_OPEN, min(FINGER_CLOSED, -mm / 2000.0))


def rim_pinch(phi, r_tcp):
    """TCP aimed inward (nose-down PITCH) at azimuth phi, on the lip's mid-height at radius r_tcp (pads reach 13 mm in)."""
    d = np.array([math.cos(phi), math.sin(phi), 0.0])
    r = _rot(R_FWD_V, phi + math.pi, RIM_PITCH)
    return np.array([TT_X, TT_Y, LIP_MID_Z]) + d * r_tcp, r


def demo():
    o, c = FINGER_OPEN, FINGER_CLOSED

    def add(plan, dt, pos, finger, label, rot):
        plan.append(RWaypoint(dt, tuple(float(v) for v in pos), finger, label, rot=tuple(map(tuple, rot))))

    # turning hand: up to N_STROKES strokes on the lip's front-left, each from azimuth STROKE_MID+22.5 to STROKE_MID-22.5 deg, turning the tray clockwise seen
    # from above; the back jar comes round by the left side to the front
    right = []
    pinch_open = sep(30.0)
    a0 = STROKE_MID + STROKE_DEG / 2  # strokes run from the right toward the left hand's own side

    def size(k):
        """Degrees stroke k turns, decided as it starts: what is still needed to bring the target round, <= STROKE_DEG."""
        cache = mk.MEMORY.setdefault("stroke_size", {})
        if k not in cache:
            done = sum(cache.get(j, 0.0) for j in range(k))
            if k > 0 and cache.get(k - 1, 0.0) < STROKE_DEG - 0.5:
                cache[k] = 0.0  # the previous stroke finished the turn: this hand holds on
            else:
                cache[k] = float(np.clip(180.0 + mk.MEMORY.get("tray_deg", 0.0), 0.0, STROKE_DEG))
            del done
        return cache[k]

    def hold_pose():
        """Where the hand holds the lip after the stroke that finished the turn."""
        cache = mk.MEMORY.setdefault("stroke_size", {})
        k = max([j for j, v in cache.items() if v > 0.0] or [0])
        return rim_pinch(math.radians(a0 - cache.get(k, 0.0)), TT_R - PINCH_IN)

    def stroke_pose(k, frac, r):
        def fn():
            if size(k) == 0.0:
                return hold_pose()
            return rim_pinch(math.radians(a0 - size(k) * frac), r)
        return Live((k, frac, r), fn)

    def finishing(k):
        return size(k) < STROKE_DEG - 0.5  # this stroke completes the turn (or there was none to make)

    def fg(k, normal):
        return LiveFinger((k, normal), lambda: FINGER_CLOSED if finishing(k) else normal, normal)

    def approach_fg(k):
        """Open for a stroke; closed on a hold-on stroke (the turn is already done): the holding hand never lets go."""
        return LiveFinger((k, "approach"), lambda: FINGER_CLOSED if size(k) == 0.0 else pinch_open, pinch_open)

    def addl(plan, dt, live, finger, label):
        plan.append(RWaypoint(dt, live, finger, label, rot=None))

    p_a, r_a = rim_pinch(math.radians(a0), TT_R + BACK_OFF)
    add(right, 1.5, (CAB_X0 - 0.06, p_a[1], p_a[2] + 0.03), pinch_open, "in front of the shelf", r_a)
    for k in range(N_STROKES):
        addl(right, 1.2, stroke_pose(k, 0.0, TT_R + BACK_OFF), approach_fg(k), "fingers open beside the lip")
        addl(right, 0.8, stroke_pose(k, 0.0, TT_R - PINCH_IN), approach_fg(k), "fingers over and under the lip")
        addl(right, 0.6, stroke_pose(k, 0.0, TT_R - PINCH_IN), c, "pinch the lip")
        n = 6
        for i in range(1, n + 1):
            addl(right, 3.0 / n, stroke_pose(k, i / n, TT_R - PINCH_IN), c, f"carry the rim round (stroke {k + 1})")
        addl(right, 0.8, stroke_pose(k, 1.0, TT_R - PINCH_IN), fg(k, pinch_open), "let go of the lip (or hold on)")
        addl(right, 1.2, Live((k, "out"), lambda k=k: hold_pose() if finishing(k) else
                               rim_pinch(math.radians(a0 - size(k)), TT_R + BACK_OFF)),
             fg(k, pinch_open), "back off the lip (or hold on)")
    t_brake = sum(w.duration for w in right)
    # left: the red-capped jar where it now stands: grip, lift over the lip, draw out, stand on the counter mark
    rg = _rot(R_FWD_H, math.radians(TAKE_YAW_DEG))
    jar_open = sep(JAR[1] * 1000 + 15.0)  # fingers just wider than the jar: they pass between its neighbours

    def at_jar(back=0.0, dz=0.0):
        def fn():
            jp = mk.MEMORY.get("target_pos", np.array([TT_X - RING_R, TT_Y, JAR_Z]))
            return np.asarray(jp) - rg[:, 2] * (0.004 + back) + np.array([0.0, 0.0, dz]), rg
        return fn

    up = np.array([0.0, 0.0, LIFT])
    z_grip = JAR_Z - rg[2, 2] * 0.004
    on_mark = np.array([MARK[0], MARK[1], z_grip - (JAR_Z - (TOP + JAR[2] / 2)) + 0.002])
    left = []
    add(left, t_brake, (CAB_X0 - 0.10, -0.08, z_grip + 0.05), jar_open, "wait clear of the shelf", rg)
    addl(left, 2.0, Live("jar_front", at_jar(0.06, 0.01)), jar_open, "in front of the red-capped jar")
    addl(left, 1.5, Live("jar_at", at_jar()), jar_open, "fingers either side of it")
    addl(left, 0.8, Live("jar_at", at_jar()), c, "close on the jar")
    addl(left, 1.0, Live("jar_up", at_jar(0.0, LIFT)), c, "lift it over the lip")
    add(left, 2.0, np.array([MARK[0], MARK[1], z_grip]) + up, c, "draw it off the shelf", rg)
    add(left, 1.5, on_mark, c, "stand it on the counter mark", rg)
    add(left, 0.8, on_mark, jar_open, "let go", rg)
    add(left, 1.5, on_mark - rg[:, 2] * 0.08 + np.array([0, 0, 0.04]), jar_open, "back away", rg)
    t_left = sum(w.duration for w in left)
    # right: hold until the jar is clear of the tray, then let go and back away
    addl(right, max(0.2, t_left - 4.5 - t_brake), Live("hold", hold_pose), c, "hold the tray still")
    addl(right, 0.8, Live("hold", hold_pose), pinch_open, "let go of the lip")
    addl(right, 1.2, Live("hold_out", lambda: (hold_pose()[0] - hold_pose()[1][:, 2] * 0.05, hold_pose()[1])),
         pinch_open, "back off the lip")
    add(right, 1.5, (CAB_X0 - 0.10, 0.20, RIM_Z + 0.06), c, "clear of the shelf", _rot(R_FWD_V))
    end = max(sum(w.duration for w in left), sum(w.duration for w in right)) + SUCCESS_HOLD_S + 1.5
    for plan in (left, right):
        last = plan[-1]
        pos, rot = (last.pos.resolve() if isinstance(last.pos, Live) else (last.pos, last.rot))
        add(plan, max(0.2, end - sum(w.duration for w in plan)), pos, last.finger, "hold clear", np.asarray(rot))
    return {"left": right, "right": left}  # the turning/holding hand is the left arm; the taking hand the right
