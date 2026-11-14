"""On the station table, a non-powered roller conveyor runs straight away from the robot, in front of the right
arm, from just ahead of the arm bases, past the far edge of the table on a leg, to an end stop. The last 18 cm
before the end stop is the packing station, marked with yellow tape on both side rails and a yellow line across the
rollers. An open plastic tote stands on the rollers at the near end; a tall square HDPE bottle (a 500 mL reagent
bottle, full, 500 g) stands loose in the middle of it.

The rollers are free-running: a tote given a push rolls on and slows down only by the rollers' rolling resistance
(about 2 % of its weight, so it loses about 0.2 m/s every second). The station starts 13 cm beyond the farthest point
either arm can reach along the conveyor (0.79 m ahead of the arm bases, from the URDF's kinematics), so the tote can
only get there under its own way: pushed too softly, it stops short in the lane; shoved too hard, it meets the end
stop too fast (over 0.40 m/s), stops dead and the tall bottle falls over. The scorer also never lets a robot point
come within 25 cm of the station, so no hand can follow the tote there.

The job: send the tote down the conveyor to the packing station with the bottle still standing.

Run it:  inspect-robots run --embodiment yam_arena -E task=tasks/coast-tote-to-far-stop/coast_tote_to_far_stop.py \\
           -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \\
           --instruction "<the instruction in task.json>" --scorer task_score ...
"""

import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import mechkit as mk  # noqa: E402  (parts, per-step rules, per-episode memory)

from yam_arena.constants import FINGER_CLOSED, PAD_TIP_PAST_TCP  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
DEBUG = False  # sim only: print the tote's travel and speed and the bottle's tilt every 5 steps
VARIANT = "demo"  # sim smoke tests: "soft" (push at 0.30 m/s), "hard" (a 2.5 m/s^2 shove to 0.75 m/s)

# Frame: x forward from the arm bases, y left, z up, metres. Left arm base (0, +0.305, 0.75), right (0, -0.305).
# -- the roller conveyor ------------------------------------------------------------------------------------------
LY = -0.200  # lane centre line
X_IN, X_END = 0.105, 1.115  # conveyor frame, near and far ends (it runs on past the table's edge at 0.91)
BED_H = 0.030  # roller tops this far above the table
BED_Z = TOP + BED_H
LANE_W = 0.130  # between the side rails
RAIL_W, RAIL_UP = 0.012, 0.020  # side rails: width, height above the rollers
STOP_X = 1.100  # the end stop's face
STOP_UP = 0.045  # end stop height above the rollers
MU_ROLL = 0.02  # rolling resistance of the free roller bed (the bed's friction coefficient)
ROLL_PITCH, ROLL_W = 0.040, 0.016  # rollers (drawn)
# -- the tote and its load ----------------------------------------------------------------------------------------
TOTE = (0.140, 0.120, 0.060)  # outside length (x), width, height
WALL, FLOOR = 0.004, 0.005
TOTE_KG = 0.35
X_T0 = 0.245  # tote centre at the start (rear face at x 0.175)
BOTTLE = (0.060, 0.060, 0.160)
CAP = (0.028, 0.028, 0.014)
BOTTLE_KG = 0.50
# -- the station ----------------------------------------------------------------------------------------------------
ZONE_REAR = 0.920  # the tote is at the station once its rear face is past this line (its front then at 1.06+)
REAR_MAX = STOP_X - TOTE[0]  # rear face with the tote against the end stop (0.960)
# -- reach (offline from the URDF's kinematics; README "Reach and distances") ---------------------------------------
REACH_X = 0.791  # farthest x any robot point (a pad tip) reaches in the lane at the tote's wall heights (right arm)
GATE = 0.25  # no robot point may come within this of the station (plan view) at any step: none past x 0.67
STATION_LO = np.array([ZONE_REAR, LY - LANE_W / 2])  # the station's footprint (plan view)
STATION_HI = np.array([STOP_X, LY + LANE_W / 2])

GREY, STEEL, DARK = (0.55, 0.57, 0.60), (0.40, 0.42, 0.45), (0.20, 0.21, 0.23)
BLUE, WHITE, YELLOW, RED = (0.15, 0.35, 0.70), (0.92, 0.92, 0.90), (0.95, 0.80, 0.10), (0.80, 0.15, 0.12)

kit = mk.Kit()
_XC = (X_IN + X_END) / 2
_LEN = X_END - X_IN
kit.part("conveyor", (_XC, LY, TOP), [
    mk.box((_LEN, LANE_W, BED_H), at=(0, 0, BED_H / 2), color=GREY),  # the bed (the rollers' common top plane)
    mk.box((_LEN, RAIL_W, BED_H + RAIL_UP), at=(0, (LANE_W + RAIL_W) / 2, (BED_H + RAIL_UP) / 2)),
    mk.box((_LEN, RAIL_W, BED_H + RAIL_UP), at=(0, -(LANE_W + RAIL_W) / 2, (BED_H + RAIL_UP) / 2)),
    mk.box((X_END - STOP_X, LANE_W, BED_H + STOP_UP), at=((STOP_X + X_END) / 2 - _XC, 0, (BED_H + STOP_UP) / 2),
           color=RED),  # end stop
    mk.box((0.040, LANE_W, TOP - 0.02), at=(X_END - 0.03 - _XC, 0, -(TOP - 0.02) / 2)),  # leg past the table's edge
], color=STEEL)
# The rollers and the station's tape are drawn on the bed (no collision: the bed's flat top carries the tote).
_rollers = [mk.box((ROLL_W, LANE_W - 0.002, 0.0008), at=(x - _XC, 0, BED_H + 0.0004), color=DARK)
            for x in np.arange(X_IN + 0.020, STOP_X - 0.005, ROLL_PITCH)]
_tape = [mk.box((STOP_X - ZONE_REAR, 0.008, 0.0008), at=((STOP_X + ZONE_REAR) / 2 - _XC, s * (LANE_W / 2 + RAIL_W / 2),
                                                         BED_H + RAIL_UP + 0.0004), color=YELLOW) for s in (1, -1)]
_tape.append(mk.box((0.010, LANE_W - 0.002, 0.0010), at=(ZONE_REAR - _XC, 0, BED_H + 0.0005), color=YELLOW))
kit.part("rollers", (_XC, LY, TOP), _rollers + _tape, color=DARK)

kit.part("tote", (X_T0, LY, BED_Z + 0.001), [
    mk.box((TOTE[0], TOTE[1], FLOOR), at=(0, 0, FLOOR / 2)),
    mk.box((WALL, TOTE[1], TOTE[2]), at=(TOTE[0] / 2 - WALL / 2, 0, TOTE[2] / 2)),
    mk.box((WALL, TOTE[1], TOTE[2]), at=(-TOTE[0] / 2 + WALL / 2, 0, TOTE[2] / 2)),
    mk.box((TOTE[0] - 2 * WALL, WALL, TOTE[2]), at=(0, TOTE[1] / 2 - WALL / 2, TOTE[2] / 2)),
    mk.box((TOTE[0] - 2 * WALL, WALL, TOTE[2]), at=(0, -TOTE[1] / 2 + WALL / 2, TOTE[2] / 2)),
], mass=TOTE_KG, color=BLUE)
kit.part("bottle", (X_T0, LY, BED_Z + FLOOR + 0.002), [
    mk.box(BOTTLE, at=(0, 0, BOTTLE[2] / 2)),
    mk.box(CAP, at=(0, 0, BOTTLE[2] + CAP[2] / 2), color=RED),
], mass=BOTTLE_KG, color=WHITE)

OBJECTS = [*kit.objects()]

SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"


# -- sim set-up: the roller bed's rolling resistance -----------------------------------------------------------------
def _roller_bed(stage) -> None:
    """Sim only, once the scene is built: give the conveyor's contact surfaces the roller bed's rolling resistance
    (friction MU_ROLL, combined by minimum, so a tote on it rolls on), and make the drawn rollers and tape visual."""
    from pxr import PhysxSchema, Usd, UsdPhysics

    paths = mk._SIM.get("paths") or {}
    n_mat = n_vis = 0
    if "conveyor" in paths:
        for prim in Usd.PrimRange(stage.GetPrimAtPath(paths["conveyor"])):
            if prim.HasAPI(UsdPhysics.MaterialAPI):
                m = UsdPhysics.MaterialAPI(prim)
                m.CreateStaticFrictionAttr().Set(MU_ROLL)
                m.CreateDynamicFrictionAttr().Set(MU_ROLL)
                PhysxSchema.PhysxMaterialAPI.Apply(prim).CreateFrictionCombineModeAttr().Set("min")
                n_mat += 1
    if "rollers" in paths:
        for prim in Usd.PrimRange(stage.GetPrimAtPath(paths["rollers"])):
            if prim.HasAPI(UsdPhysics.CollisionAPI):
                UsdPhysics.CollisionAPI(prim).CreateCollisionEnabledAttr().Set(False)
                n_vis += 1
    print(f"[coast] roller bed: {n_mat} bed materials set to mu {MU_ROLL} (min); {n_vis} drawn parts non-colliding",
          flush=True)


if not getattr(mk, "_coast_bed_hook", False):
    _author0 = mk._author_all

    def _author_with_bed(stage, env_path):
        _author0(stage, env_path)
        _roller_bed(stage)

    mk._author_all = _author_with_bed
    mk._coast_bed_hook = True

# -- the per-step watch and the scoring ---------------------------------------------------------------------------
TIP_DEG = 15.0  # the bottle has fallen once it leans past this (latched)
TOTE_TILT_DEG = 8.0  # the tote was lifted or tipped off the rollers (latched)
LIFT = 0.012  # ... or raised this far off the bed (latched)
STILL_M = 0.00025  # still: moved under 0.25 mm in a control step (5 mm/s)
CLEAR = 0.03  # both grippers this far from the tote and the bottle
SLAM_V = 0.40  # m/s: the tote met the end stop faster than this (latched; the bottle's tip-over speed, README)
BANDS = ((ZONE_REAR - 0.12, 0.25), (ZONE_REAR - 0.05, 0.5))  # rear face past these lines, stopped short of the zone
DT = 0.05
M = "coast"
_TEST_TCPS = None  # self-test stand-in for the two grippers' TCPs (also their robot points)
# Robot points per arm, in the `gripper` link frame (approach +z): the link origin, the TCP (134 mm out), each pad's
# tip (147 mm, PAD_TIP_PAST_TCP past the TCP) and each finger's middle (100 mm). Each pad sits on the closing axis
# (gripper x) at its finger joint's travel: closed both at 0, open at -/+47 mm (left / right). The finger joint's
# travel is read from the finger link origins (URDF: left at x +44.5 mm closed, moving -x as it opens; right mirrored).
FINGER_X0 = 0.0445
PAD_TIP_Z = 0.134 + PAD_TIP_PAST_TCP
FINGER_MID_Z = 0.100
POINT_NAMES = ("gripper link", "TCP", "left pad tip", "right pad tip", "left finger middle", "right finger middle")
INF = float("inf")


def _mem() -> dict:
    m = mk.MEMORY.get(M)
    if m is None:
        m = mk.MEMORY[M] = {"n": 0, "tipped": False, "out": False, "lifted": False, "slammed": False,
                            "reached": False, "near": INF, "still": True, "prev": None, "v": 0.0, "v_prev": 0.0,
                            "vmax": 0.0, "v_stop": None, "rear_max": 0.0}
    return m


def robot_points() -> list:
    """World positions of every robot point of both arms (6 per arm); offline, the self-test's TCPs."""
    if _TEST_TCPS is not None or mk._EP.get("env") is None:
        return [np.asarray(t, float) for t in (_TEST_TCPS or [])]
    from yam_arena.runtime import link_poses

    base = mk._EP["env"].unwrapped
    o = base.scene.env_origins[0]
    o = getattr(o, "torch", o).detach().cpu().numpy().astype(float)
    out = []
    for key in ("left_robot", "right_robot"):
        P = link_poses(base.scene[key])
        g = P["gripper"]
        r, p = g[:3, :3], g[:3, 3]
        xl = float((r.T @ (P["tip_left"][:3, 3] - p))[0]) - FINGER_X0  # left pad on the closing axis
        xr = float((r.T @ (P["tip_right"][:3, 3] - p))[0]) + FINGER_X0  # right pad
        for loc in ((0, 0, 0), (0, 0, 0.134), (xl, 0, PAD_TIP_Z), (xr, 0, PAD_TIP_Z), (xl, 0, FINGER_MID_Z),
                    (xr, 0, FINGER_MID_Z)):
            out.append(p + r @ np.asarray(loc, float) - o)
    return out


def station_gap(pt) -> float:
    """Plan-view distance (m) from a point to the station's footprint (0 inside it)."""
    d = np.maximum(0.0, np.maximum(STATION_LO - np.asarray(pt, float)[:2], np.asarray(pt, float)[:2] - STATION_HI))
    return float(np.hypot(*d))


def _tilt_deg(objects, part) -> float:
    r = mk.quat_to_mat(kit.pose(objects, part)[1])
    return math.degrees(math.acos(max(-1.0, min(1.0, float(r[2, 2])))))


def rear(objects) -> float:
    """x of the tote's rear face (at its floor, on the centre line)."""
    return float(kit.point(objects, "tote", (-TOTE[0] / 2, 0.0, 0.0))[0])


def bottle_in_tote(objects) -> bool:
    """The bottle's base centre lies inside the tote, on or near its floor (tote frame)."""
    t = kit._pose_T(objects, "tote")
    b = mk._inv(t) @ np.r_[kit.point(objects, "bottle", (0.0, 0.0, 0.0)), 1.0]
    return (abs(b[0]) < TOTE[0] / 2 - WALL and abs(b[1]) < TOTE[1] / 2 - WALL
            and -0.005 < b[2] < FLOOR + 0.015)


def _tcps():
    return mk.tcps() if _TEST_TCPS is None else _TEST_TCPS


def _released(st, margin=CLEAR) -> bool:
    lo, hi = np.asarray(st.low) - margin, np.asarray(st.high) + margin
    return not any(bool(np.all(np.asarray(t) >= lo) and np.all(np.asarray(t) <= hi)) for t in _tcps())


def _say(msg) -> None:
    if DEBUG and mk._EP.get("env") is not None:
        print(f"[coast] step {mk._EP.get('step')}: {msg}", flush=True)


def _log(msg) -> None:
    """A latch firing: printed in sim whether or not DEBUG is on."""
    if mk._EP.get("env") is not None:
        print(f"[coast] step {mk._EP.get('step')}: {msg}", flush=True)


def _watch(objects) -> None:
    """Every control step: latch a fallen bottle, a bottle taken out of the tote, a tote lifted off the rollers, a
    tote that met the end stop faster than SLAM_V, and any robot point within GATE of the station; track the tote's
    speed and whether everything is still."""
    m = _mem()
    m["n"] += 1
    r = rear(objects)
    p = np.r_[kit.point(objects, "tote", (0, 0, 0)), kit.point(objects, "bottle", (0, 0, BOTTLE[2]))]
    prev = m["prev"]
    if prev is not None:
        m["v_prev"] = m["v"]
        m["v"] = float(p[0] - prev[0]) / DT
        m["still"] = bool(np.all(np.abs(p - prev) < STILL_M))
    m["prev"] = p
    m["vmax"] = max(m["vmax"], m["v"])
    m["rear_max"] = max(m["rear_max"], r)
    if m["v_stop"] is None and r > REAR_MAX - 0.003:
        # Speed as the tote meets the end stop: the faster of this step's and the last step's (this step's mean
        # speed may already include the stop).
        m["v_stop"] = max(m["v"], m["v_prev"])
        _say(f"tote reached the end stop at {m['v_stop'] * 1000:.0f} mm/s")
        if m["v_stop"] > SLAM_V and not m["slammed"]:
            m["slammed"] = True
            _log(f"LATCH slam: the tote met the end stop at {m['v_stop'] * 1000:.0f} mm/s (limit "
                 f"{SLAM_V * 1000:.0f})")
    gaps = [station_gap(q) for q in robot_points()]
    if gaps:
        m["near"] = min(m["near"], min(gaps))
        if not m["reached"] and min(gaps) < GATE:
            m["reached"] = True
            k = int(np.argmin(gaps))
            q, per = robot_points()[k], max(1, len(gaps) // 2)
            name = POINT_NAMES[k % per] if per == 6 else "TCP"
            _log(f"LATCH reach-gate: robot point {k} ({'left' if k < per else 'right'} arm, {name}) at "
                 f"({q[0]:.3f}, {q[1]:.3f}, {q[2] - TOP:.3f} above the table) is {gaps[k] * 100:.1f} cm from the "
                 f"station (gate {GATE * 100:.0f} cm); tote rear at {r:.3f}")
    tilt = _tilt_deg(objects, "bottle")
    if not m["tipped"] and tilt > TIP_DEG:
        m["tipped"] = True
        _say(f"BOTTLE FELL ({tilt:.1f} deg) with the tote rear at {r:.3f}")
    if not m["out"] and not bottle_in_tote(objects):
        m["out"] = True
        _say("bottle out of the tote")
    lift = kit.point(objects, "tote", (0, 0, 0))[2] - (BED_Z + 0.001)
    if not m["lifted"] and (_tilt_deg(objects, "tote") > TOTE_TILT_DEG or lift > LIFT):
        m["lifted"] = True
        _say(f"tote lifted / tipped off the rollers (rise {lift * 1000:.1f} mm)")
    if DEBUG and m["n"] % 5 == 0:
        tc = mk.tcps()
        tz = f"({tc[1][0]:.3f},{tc[1][1]:.3f},{tc[1][2] - TOP:.3f})" if tc else "-"
        _say(f"nearest robot point to the station {m['near'] * 100:.1f} cm; R tcp {tz}; tote rear {r:.4f} v {m['v'] * 1000:.0f} mm/s (max {m['vmax'] * 1000:.0f}); bottle tilt "
             f"{tilt:.1f} deg; tote rise {lift * 1000:.1f} mm, tilt {_tilt_deg(objects, 'tote'):.1f} deg")


kit.on_step(_watch)


def score(objects) -> float:
    """0 once (latched for the trial) the bottle has fallen (leaned past 15 deg) or left the tote, the tote was lifted
    or tipped off the rollers, the tote met the end stop faster than 0.40 m/s, or any robot point came within 25 cm
    of the station; 0 while the tote or bottle is moving or a gripper is within 3 cm of either. Otherwise, by where
    the tote stopped: rear face past the station line -> 1.0; within 5 cm short of it -> 0.5; within 12 cm short ->
    0.25; else 0."""
    if mk._EP.get("env") is None and _mem().get("seen") is not objects:  # offline (once per state); in sim the
        _watch(objects)  # per-step watch has already run
        _mem()["seen"] = objects
    m = _mem()
    if m["tipped"] or m["out"] or m["lifted"] or m["slammed"] or m["reached"]:
        return 0.0
    if not m["still"] or not _released(objects["tote"]) or not _released(objects["bottle"]):
        return 0.0
    r = rear(objects)
    if r >= ZONE_REAR:
        return 1.0
    out = 0.0
    for line, s in BANDS:
        if r >= line:
            out = s
    return out


def success(objects) -> bool:
    """The tote stopped at the packing station with the bottle standing, released; holds for 1 s."""
    return score(objects) >= 1.0 - 1e-9


def distance(objects) -> float:
    """How far the tote's rear is short of the station line (m), plus 1 once any latch has fired."""
    m = _mem()
    return float(max(0.0, ZONE_REAR - rear(objects))
                 + (m["tipped"] or m["out"] or m["lifted"] or m["slammed"] or m["reached"]))


# -- the scripted demo ----------------------------------------------------------------------------------------------
# Open loop from the scene's geometry: the right hand, fingers closed and turned so the closed pads' flat face
# points forward, comes down behind the tote's rear wall at its mid-height, eases onto it, then pushes it along the
# lane with a steady 1 m/s^2 up to PUSH_V and stops hard (4 m/s^2). The tote leaves the pads as the hand slows and
# rolls on alone; the roller bed's resistance (about 0.2 m/s^2) slows it to the station. The push is set by one
# command per control step (exact positions at the step times), so the hand's speed profile is the one planned.
# Release-speed window at the demo's release point (model below): 0.48-0.64 m/s at 0.196 m/s^2. The demo's
# 0.54 m/s meets the stop at about 0.22 m/s in the model. In sim the tote left the pads at 0.56 m/s, met the stop at
# 0.26 m/s and came to rest with its rear at 0.942. Once the tote is away, the hand rises to a viewing pose behind
# the gate (TCP x 0.52, 30 cm up, pointing down the lane 5 deg down), so the wrist camera watches the tote come to
# rest. No robot point comes within 38 cm of the station.

PUSH_V = {"soft": 0.30, "hard": 0.75}.get(VARIANT, 0.54)  # m/s at the end of the push
PUSH_A, STOP_A = (2.5 if VARIANT == "hard" else 1.0), 4.0  # m/s^2 speeding up, slowing down
PAD_HALF = 0.0064  # closed pads: 12.8 mm thick along x, centred on the TCP
YAW = math.pi / 2  # fingers close along x: the closed pads present their 26 mm flat face forward
TCP_Z = BED_Z + FLOOR + 0.030 + 0.005  # pads' mid-height at the rear wall's mid-height (TCP 5 mm above it)
GAP = 0.006  # the hand waits this far behind the wall
PARK = (0.200, -0.330, TOP + 0.220)
VIEW = (0.520, LY, TOP + 0.300)  # viewing pose (TCP): its pad tips at x 0.533, 38.7 cm from the station
VIEW_PITCH = 5.0  # deg: the wrist camera (25 deg below the fingers' axis) then looks 30 deg down, at the station
V_TIP = 0.40  # m/s at the end stop above which the bottle falls (tip-over model; sim: stood at 0.14, fell at 0.42)


def push_profile(v=None, a=PUSH_A, d=STOP_A):
    """The push's TCP x at each control step: ease on, speed up at ``a`` to ``v``, stop at ``d``."""
    v = PUSH_V if v is None else v
    x0 = X_T0 - TOTE[0] / 2 - PAD_HALF  # pads' face on the wall
    xs, x, s = [], x0, 0.0
    while s < v - 1e-9:
        s = min(v, s + a * DT)
        x += s * DT
        xs.append(x)
    while s > 1e-9:
        s = max(0.0, s - d * DT)
        x += s * DT
        xs.append(x)
    return x0, xs


def demo():
    """Come down behind the tote; ease onto its rear wall; push to PUSH_V and stop; rise to the viewing pose."""
    x0, xs = push_profile()
    R = [Waypoint(2.0, PARK, FINGER_CLOSED, "start", YAW),
         Waypoint(2.0, (x0 - GAP - 0.02, LY, TCP_Z + 0.08), FINGER_CLOSED, "over the lane, behind the tote", YAW),
         Waypoint(1.5, (x0 - GAP, LY, TCP_Z), FINGER_CLOSED, "down behind the rear wall", YAW),
         Waypoint(0.5, (x0 - GAP, LY, TCP_Z), FINGER_CLOSED, "settle", YAW),
         Waypoint(0.6, (x0 + 0.0005, LY, TCP_Z), FINGER_CLOSED, "ease onto the wall", YAW)]
    R += [Waypoint(DT, (x, LY, TCP_Z), FINGER_CLOSED, "push", YAW) for x in xs]
    xe = xs[-1]
    view = forward_grip(VIEW_PITCH)
    R += [Waypoint(0.4, (xe, LY, TCP_Z), FINGER_CLOSED, "hold still: the tote rolls on", YAW),
          Waypoint(1.6, VIEW, FINGER_CLOSED, "rise to the viewing pose: wrist camera down the lane", view),
          Waypoint(5.0, VIEW, FINGER_CLOSED, "watch the tote come to rest", view)]
    total = sum(w.duration for w in R)
    left = [Waypoint(2.0, (0.150, 0.250, 0.950), FINGER_CLOSED, "park"),
            Waypoint(total - 2.0, (0.150, 0.250, 0.950), FINGER_CLOSED, "park")]
    return {"left": left, "right": R}


class Tilt(float):
    """A Waypoint yaw that also carries a full TCP rotation (yam_arena's Waypoint has only the down grip's yaw);
    ArmPlan uses the rotation through the in-memory patch of grip_rotation below."""

    def __new__(cls, rot):
        self = super().__new__(cls, 0.0)
        self.rot = np.asarray(rot, dtype=float)
        return self


def _patch_grip_rotation() -> None:
    import yam_arena.demo as _demo

    if getattr(_demo, "_tilt_patch", False):
        return
    base = _demo.grip_rotation

    def grip_rotation(yaw=0.0):
        rot = getattr(yaw, "rot", None)
        return rot.copy() if rot is not None else base(yaw)

    _demo.grip_rotation = grip_rotation
    _demo._tilt_patch = True


_patch_grip_rotation()


def forward_grip(pitch_deg: float) -> Tilt:
    """The hand pointing forward along the lane (+x), tipped ``pitch_deg`` down, fingers closing along y."""
    p = math.radians(pitch_deg)
    a = np.array([math.cos(p), 0.0, -math.sin(p)])
    c = np.array([0.0, -1.0, 0.0])
    return Tilt(np.column_stack([c, np.cross(a, c), a]))


WALK_PITCH = 20.0  # deg: the forward-pointing hand's tip-down (its reach in the lane: TCP x 0.755, pad tip 0.767)
WALK_TO = 0.720  # the walk's last TCP x, inside that reach


def control_walk():
    """Sim control (the gate): push the tote with no momentum as far as the arm reaches. The down-pointing hand
    walks it slowly to TCP x 0.38; the hand lifts, turns to point forward along the lane (20 deg down) and walks it
    on slowly to TCP x 0.72 (pad tips at 0.73, tote rear about 0.74), 3.5 cm inside the arm's reach there. Every
    slow-down is gentler than the rollers' 0.2 m/s^2, so the tote never runs ahead of the hand. A robot point passes
    x 0.67 (25 cm from the station) on the way: the reach gate must latch, and the trial scores 0."""
    x0 = X_T0 - TOTE[0] / 2 - PAD_HALF
    fw = forward_grip(WALK_PITCH)
    p = math.radians(WALK_PITCH)
    tip_dx = PAD_TIP_PAST_TCP * math.cos(p)
    x1 = 0.380  # end of the down-pointing walk (TCP)
    r1 = x1 + PAD_HALF  # the tote's rear there
    xf = r1 - tip_dx - 0.008  # the forward hand's TCP, its pad tips 8 mm behind the rear wall
    R = [Waypoint(2.0, PARK, FINGER_CLOSED, "start", YAW),
         Waypoint(2.0, (x0 - GAP - 0.02, LY, TCP_Z + 0.08), FINGER_CLOSED, "over the lane, behind the tote", YAW),
         Waypoint(1.5, (x0 - GAP, LY, TCP_Z), FINGER_CLOSED, "down behind the rear wall", YAW),
         Waypoint(0.5, (x0 - GAP, LY, TCP_Z), FINGER_CLOSED, "settle", YAW),
         Waypoint(0.6, (x0 + 0.0005, LY, TCP_Z), FINGER_CLOSED, "ease onto the wall", YAW),
         Waypoint(4.0, (x1, LY, TCP_Z), FINGER_CLOSED, "walk the tote slowly (hand pointing down)", YAW),
         Waypoint(1.0, (x1, LY, TCP_Z), FINGER_CLOSED, "hold", YAW),
         Waypoint(1.2, (x1 - 0.04, LY, TCP_Z + 0.10), FINGER_CLOSED, "lift", YAW),
         Waypoint(1.5, (xf - 0.03, LY, TCP_Z + 0.05), FINGER_CLOSED, "turn to point forward", fw),
         Waypoint(1.2, (xf, LY, TCP_Z), FINGER_CLOSED, "down behind the rear wall, pointing forward", fw),
         Waypoint(6.0, (WALK_TO, LY, TCP_Z), FINGER_CLOSED, "walk the tote on as far as the arm reaches", fw),
         Waypoint(1.0, (WALK_TO, LY, TCP_Z), FINGER_CLOSED, "hold at the reach limit", fw),
         Waypoint(1.2, (WALK_TO - 0.04, LY, TCP_Z + 0.08), FINGER_CLOSED, "back off and lift", fw),
         Waypoint(2.0, PARK, FINGER_CLOSED, "park", YAW),
         Waypoint(1.0, PARK, FINGER_CLOSED, "wait", YAW)]
    total = sum(w.duration for w in R)
    left = [Waypoint(2.0, (0.150, 0.250, 0.950), FINGER_CLOSED, "park"),
            Waypoint(total - 2.0, (0.150, 0.250, 0.950), FINGER_CLOSED, "park")]
    return {"left": left, "right": R}


def coast(v0, rear0, a=MU_ROLL * 9.81):
    """Model: the tote leaves the hand at ``v0`` with its rear at ``rear0`` and rolls on, slowed at ``a``. Returns
    the rear face's x per control step and the speed at the end stop (None if it stops short)."""
    xs, x, v = [], rear0, v0
    while v > 0:
        v2 = max(0.0, v - a * DT)
        x += 0.5 * (v + v2) * DT
        v = v2
        if x >= REAR_MAX:
            vi = math.sqrt(max(0.0, v0 * v0 - 2 * a * (REAR_MAX - rear0)))
            xs.append(REAR_MAX)
            return xs, vi
        xs.append(x)
    return xs, None


def window(rear0, a=MU_ROLL * 9.81, v_tip=V_TIP):
    """Release speeds (m/s) that bring the tote to rest at the station with the bottle standing (model)."""
    return math.sqrt(2 * a * (ZONE_REAR - rear0)), math.sqrt(v_tip ** 2 + 2 * a * (REAR_MAX - rear0))


# -- offline self-test ------------------------------------------------------------------------------------------------


def self_test():
    """The scoring on hand-built trials, one control step at a time: the hand pushes the tote (rear face at the
    pads), the tote then rolls on by the model above; a tote meeting the end stop faster than V_TIP throws the
    bottle over against the front wall (``tip=False`` keeps it standing, to test the end-stop speed latch alone)."""
    global _TEST_TCPS
    far = [(0.15, 0.25, 0.95), PARK]
    lean = (math.sin(math.radians(17.5)), 0.0, 0.0, math.cos(math.radians(17.5)))  # 35 deg forward about y

    def state(r, tipped=False, bottle_dz=0.0, tote_dz=0.0, bottle_dx=0.0):
        tc = r + TOTE[0] / 2
        moved = {"tote": ((tc, LY, BED_Z + 0.001 + tote_dz), (0, 0, 0, 1.0))}
        bz = BED_Z + 0.001 + FLOOR + bottle_dz
        moved["bottle"] = ((tc + bottle_dx + (0.03 if tipped else 0.0), LY, bz), lean if tipped else (0, 0, 0, 1.0))
        return kit.fake_states(moved=moved)

    def run(rears, hand=None, tip_at=None, end_hand=None, bottle=None, tote_dz=None, settle=30, down=None,
            poke=None):
        """Feed the tote's rear x per step (then ``settle`` still steps). ``hand``: steps with a gripper on the tote;
        ``tip_at``: first step with the bottle fallen (``down``: or step -> fallen); ``bottle``/``tote_dz``: step ->
        (dz, dx) / dz; ``poke``: step -> where the right hand is instead (touching nothing)."""
        global _TEST_TCPS
        rears = list(rears) + [rears[-1]] * settle
        got = st = None
        for k, r in enumerate(rears):
            bz, bx = bottle(k) if bottle else (0.0, 0.0)
            fell = down(k) if down else (tip_at is not None and k >= tip_at)
            st = state(r, tipped=fell, bottle_dz=bz, bottle_dx=bx,
                       tote_dz=tote_dz(k) if tote_dz else 0.0)
            on = hand is not None and k in hand
            _TEST_TCPS = [far[0], (r - PAD_HALF, LY, TCP_Z)] if on else list(far)
            if poke is not None and poke(k) is not None:
                _TEST_TCPS = [far[0], poke(k)]
            if k == len(rears) - 1 and end_hand is not None:
                _TEST_TCPS = list(end_hand)
            got = score(st)
        return got, st

    def trial(v, a_push=PUSH_A, tip=True, **kw):
        """A push like the demo's to ``v``, then the coast; the bottle falls if the tote meets the stop too fast."""
        x0, xs = push_profile(v, a_push)
        pr = [x + PAD_HALF for x in xs]
        k_sep = int(np.argmax(np.diff([x0] + xs)))  # the hand slows from here on: the tote rolls on alone
        r0 = pr[k_sep]
        cx, vi = coast(v, r0)
        rears = [X_T0 - TOTE[0] / 2] * 10 + pr[:k_sep + 1] + cx
        tip_at = (10 + k_sep + 1 + len(cx)) if (tip and vi is not None and vi > V_TIP) else None
        hand = set(range(10, 10 + k_sep + 1))
        return run(rears, hand=hand, tip_at=tip_at, **kw), vi, rears[-1]

    x0d, xsd = push_profile()
    lo, hi = window(xsd[int(np.argmax(np.diff([x0d] + xsd)))] + PAD_HALF)  # at the demo's release point
    cases = {}

    def case(name, fn, want):
        mk.new_episode()
        out = fn()
        (got, st), extra = (out[0], out[1:]) if isinstance(out[0], tuple) else (out, ())
        cases[name] = (got, rear(st), extra)
        assert abs(got - want) < 1e-9, (name, got, want, round(rear(st), 4), _mem())
        assert success(st) == (want >= 1.0), name

    r_start = X_T0 - TOTE[0] / 2
    case("start", lambda: run([r_start], settle=0), 0.0)
    case(f"demo push to {PUSH_V} m/s: rolls to rest at the station", lambda: trial(PUSH_V), 1.0)
    case("slow end of the window (0.52 m/s)", lambda: trial(0.52), 1.0)
    case("fast end of the window (0.60 m/s): meets the stop at 0.36 m/s, bottle stands", lambda: trial(0.60), 1.0)
    case("too soft (0.30 m/s): stops in the lane", lambda: trial(0.30), 0.0)
    case("a bit soft (0.46 m/s): within 12 cm, 0.25", lambda: trial(0.46), 0.25)
    case("nearly (0.48 m/s): within 5 cm, 0.5", lambda: trial(0.48), 0.5)
    case("too hard (0.62 m/s): meets the stop at 0.41 m/s, bottle falls", lambda: trial(0.62), 0.0)
    case("too hard (0.62 m/s), bottle stays up: the end-stop speed latch alone", lambda: trial(0.62, tip=False), 0.0)
    case("shoved (0.85 m/s): slams the stop, bottle falls", lambda: trial(0.85), 0.0)
    case("shoved hard from a standing start (0.70 m/s)", lambda: trial(0.70, a_push=2.0), 0.0)
    case("walked by the down-pointing hand to rear 0.39, let go",
         lambda: run(list(np.linspace(r_start, 0.39, 80)), hand=set(range(80))), 0.0)
    case("walked by hand only up to the gate (hand to x 0.665, 25.5 cm from the station), let go: 25 cm short",
         lambda: run(list(np.linspace(r_start, 0.665 + PAD_HALF, 120)), hand=set(range(120))), 0.0)
    case("walked by hand, pointing forward, to its reach limit (rear 0.79), let go: the gate",
         lambda: run(list(np.linspace(r_start, REACH_X, 120)), hand=set(range(120))), 0.0)
    case("walked by hand to the reach limit and held there",
         lambda: run(list(np.linspace(r_start, REACH_X, 120)), hand=set(range(150)),
                     end_hand=[far[0], (REACH_X - PAD_HALF, LY, TCP_Z)]), 0.0)
    case("tote lifted and carried to the station",
         lambda: run(list(np.linspace(r_start, 0.94, 80)), tote_dz=lambda k: 0.05 if 10 < k < 70 else 0.0), 0.0)
    case("bottle lifted out and set back in the tote, then the tote sent to the station",
         lambda: run(list(np.linspace(r_start, 0.94, 80)), bottle=lambda k: (0.08, 0.0) if 5 < k < 20 else (0, 0)),
         0.0)
    case("bottle knocked over at the start, set upright again, then the tote sent to the station",
         lambda: run([r_start] * 40 + list(np.linspace(r_start, 0.94, 40)), down=lambda k: 10 <= k < 20), 0.0)
    case("demo, but a gripper is left near the tote", lambda: trial(PUSH_V, end_hand=[far[0], (0.95, LY, TCP_Z)]), 0.0)
    case("demo, but the hand reached out over the lane to x 0.68 for one step mid-coast (touching nothing)",
         lambda: trial(PUSH_V, poke=lambda k: (0.68, LY, TCP_Z + 0.10) if k == 40 else None), 0.0)
    case("demo, the hand reaching out only to x 0.66 (26 cm from the station) mid-coast",
         lambda: trial(PUSH_V, poke=lambda k: (0.66, LY, TCP_Z + 0.10) if k == 40 else None), 1.0)
    case("tote still rolling at the last step", lambda: run(list(np.linspace(r_start, 0.94, 40)), settle=0), 0.0)
    mk.new_episode()
    assert score(state(r_start)) == 0.0 and not _mem()["tipped"], "a new trial forgets the last one"
    mk.new_episode()
    _TEST_TCPS = None
    return len(cases) + 1, cases, (lo, hi)


def max_reach(samples=600, seed=0):
    """Offline (``--reach``): the farthest x any robot point of either arm reaches in the lane (y -0.265..-0.135) at
    the heights where it can push the tote's wall (3-9 cm above the table), with every link origin above the table.
    Random joint sets, each refined (L-BFGS) to push the pad tip forward; the pad tip (13 mm past the TCP on the
    approach axis) is the arm's farthest point forward."""
    from scipy.optimize import minimize
    from yam_arena import YAM_URDF, ik, station
    from yam_arena.constants import ARM_JOINTS, TCP_OFFSET
    from yam_arena.urdf import forward_kinematics

    rng = np.random.default_rng(seed)
    y_lo, y_hi, z_lo, z_hi = LY - LANE_W / 2, LY + LANE_W / 2, TOP + BED_H, TOP + BED_H + TOTE[2]
    best = {}
    for arm in ("right", "left"):
        b = np.asarray(station.BASES[arm])

        def pts(q):
            fk = forward_kinematics(YAM_URDF, {n: float(v) for n, v in zip(ARM_JOINTS, q)})
            g = fk["gripper"]
            tcp = g[:3, :3] @ np.array(TCP_OFFSET) + g[:3, 3] + b
            tip = tcp + PAD_TIP_PAST_TCP * g[:3, 2]
            return tip, [fk[k][:3, 3] + b for k in ("link2", "link3", "link4", "link5", "gripper")] + [tcp]

        def pen(q):
            tip, links = pts(q)
            out = (max(0, y_lo - tip[1]) ** 2 + max(0, tip[1] - y_hi) ** 2 + max(0, z_lo - tip[2]) ** 2
                   + max(0, tip[2] - z_hi) ** 2)
            return out + sum(max(0, TOP + 0.01 - p[2]) ** 2 for p in links)

        top = -1.0
        for _ in range(samples):
            q = rng.uniform(ik.LOWER, ik.UPPER)
            if pts(q)[0][0] < 0.4:
                continue
            r = minimize(lambda q: -pts(q)[0][0] + 1e4 * pen(q), q, method="L-BFGS-B",
                         bounds=list(zip(ik.LOWER, ik.UPPER)))
            if pen(r.x) < 1e-8:
                top = max(top, float(pts(r.x)[0][0]))
        best[arm] = top
    return best


def plan_gap(plan_fn):
    """Offline: the nearest any arm's gripper link, TCP or pad tip comes to the station over a plan (m)."""
    from yam_arena import station
    from yam_arena.demo import ArmPlan

    near = INF
    for arm, waypoints in plan_fn().items():
        plan = ArmPlan(list(waypoints), station.BASES[arm])
        for k in range(round(plan.duration * 20.0) + 5):
            plan.action7((k + 1) / 20.0)
            from yam_arena import ik
            tcp, r = ik.tcp_pose(plan.q, station.BASES[arm])
            for pt in (tcp - 0.134 * r[:, 2], tcp, tcp + PAD_TIP_PAST_TCP * r[:, 2]):
                near = min(near, station_gap(pt))
    return near


if __name__ == "__main__":
    # Offline checks, no GPU: the scoring on hand-built trials, then both arms' plans replayed through the kinematics
    # at 20 Hz, every joint inside the URDF limits with an IK error under 0.5 mm.
    from yam_arena import ik, station
    from yam_arena.demo import ArmPlan
    from yam_arena.task_file import load

    n, res, (lo, hi) = self_test()
    for k, (s, r, extra) in res.items():
        print(f"  {s:5.2f}  tote rear stopped at x {r:.3f}  | {k}")
    print(f"scoring: {n} cases pass; release-speed window {lo:.3f}-{hi:.3f} m/s, demo {PUSH_V} m/s")
    task = load(__file__)
    hz = 20.0
    for arm, waypoints in task.demo().items():
        plan = ArmPlan(list(waypoints), station.BASES[arm])
        q = np.array([plan.action7((k + 1) / hz)[0] for k in range(round(plan.duration * hz) + 5)])
        assert plan.worst_ik_mm < 0.5 and plan.worst_ik_deg < 0.1, (arm, plan.worst_ik_mm, plan.worst_ik_deg)
        assert np.all(q[:, :6] >= ik.LOWER) and np.all(q[:, :6] <= ik.UPPER), arm
        print(f"{arm}: {plan.duration:.1f} s plan, worst IK {plan.worst_ik_mm:.3f} mm / {plan.worst_ik_deg:.3f} deg")
    for arm, waypoints in control_walk().items():
        plan = ArmPlan(list(waypoints), station.BASES[arm])
        q = np.array([plan.action7((k + 1) / hz)[0] for k in range(round(plan.duration * hz) + 5)])
        assert plan.worst_ik_mm < 0.5 and plan.worst_ik_deg < 0.1, (arm, plan.worst_ik_mm, plan.worst_ik_deg)
        assert np.all(q[:, :6] >= ik.LOWER) and np.all(q[:, :6] <= ik.UPPER), arm
        print(f"control_walk {arm}: {plan.duration:.1f} s plan, worst IK {plan.worst_ik_mm:.3f} mm / "
              f"{plan.worst_ik_deg:.3f} deg")
    g_demo, g_walk = plan_gap(task.demo), plan_gap(control_walk)
    assert g_demo > GATE + 0.10 and g_walk < GATE - 0.03, (g_demo, g_walk)
    print(f"nearest robot point to the station: demo {g_demo * 100:.1f} cm, control_walk {g_walk * 100:.1f} cm "
          f"(gate {GATE * 100:.0f} cm)")
    if "--reach" in sys.argv:
        best = max_reach()
        print("farthest x of any robot point in the lane at the tote's wall heights: "
              + ", ".join(f"{a} arm {x:.3f} m" for a, x in best.items()) + f" (station line {ZONE_REAR})")
