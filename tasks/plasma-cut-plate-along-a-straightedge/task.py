"""Hand plasma cutting along a straightedge. A 3 mm mild-steel plate is clamped on a cutting stand with a 55 mm strip
overhanging; a white scribed line marks the cut, 200 mm from the plate's far edge to its near edge. A steel straightedge with
a red handle block lies on a pad to the left; a hand plasma torch with a copper drag shield lies on its rest posts to the
right. One arm sets the straightedge on the plate, one shield radius from the line, and holds it down; the other takes the
torch and drags its shield along the straightedge from one edge of the plate to the other, until the strip drops into the
tray. Then the torch goes back on its rest and the straightedge back on its pad.

Run it:  inspect-robots run --embodiment yam_plasma -E task=task.py -E object_jitter=0 --policy yam_scripted --no-prompt \\
           --instruction "Cut the overhanging strip off the plate along the white line with the plasma torch, guiding the torch along the straightedge, then put the torch and the straightedge back where they were and move both hands away." --scorer task_score

`yam_plasma` (plasma_cut/embodiment.py) spawns the stand, offcut, torch and straightedge, reports each gripper's fingertip
midpoint, switches off the offcut's joints that note_step reports as cut, and shows the kerf.

The cut is bookkeeping over rigid bodies, per 5 mm segment of the 200 mm line (the arc, heat and molten metal aren't simulated):
- Arc: the torch is in a hand, within 30 degrees of square to the plate, its nozzle 1-4 mm above the plate (the shield's rim
  from 1 mm into the plate's top to 2 mm above it: the nozzle's orifice is recessed 2 mm), over the plate between its two edges and
  within 10 mm of the line. In the real trial the supervisor's remote switch fires the torch by the same criterion.
- Travel: each segment gains the arc time spent with the nozzle over it. It is cut through from 0.24 s (20.75 mm/s or
  slower: the 20 A maximum for 3 mm) and overheated beyond 0.79 s (slower than 6.35 mm/s, half the rated 12.7 mm/s: heavy dross).
- A cut-through segment's joint to the offcut is switched off. The offcut drops when all 40 are. A joint also breaks at
  the load that folds or tears a 5 x 3 mm steel bridge (2.8 N m, 6 kN); a broken joint's segment is not cut through.
- A segment is sound when cut through, not overheated, the nozzle never more than 2 mm from the line or the torch more than
  10 degrees from square while cutting it, and the shield bearing on the straightedge (which lies flat on the plate) for
  80% or more of its arc time.
"""

import math
import os

import numpy as np

from plasma_cut import mech as M
from yam_arena import ik
from yam_arena.constants import FINGER_OPEN, READY
from yam_arena.demo import Waypoint, grip_rotation
from yam_arena.station import BASES
from yam_arena.task_file import TABLE_TOP, Box

SCENE = "station"
TOP = TABLE_TOP[SCENE]
PLATE_Z = TOP + M.PT  # the plate's top, world
N = M.N_SEG

OBJECTS = [
    Box("bench", pos=(0.0, 0.0, TOP), size=(0.01, 0.01, 0.01)),
    Box("offcut", pos=(M.OFFCUT_C[0], M.OFFCUT_C[1], TOP + M.OFFCUT_C[2]), size=(0.01, 0.01, 0.01), mass=M.OFFCUT_MASS),
    Box("torch", pos=(M.TORCH_REST["x"], M.TORCH_REST["y"], TOP + M.TORCH_REST_Z), size=(0.01, 0.01, 0.01), mass=M.TORCH_TOTAL,
        yaw=M.TORCH_REST["heading"]),
    Box("guide", pos=(M.GUIDE_REST["x"], M.GUIDE_REST["y"], TOP + M.GUIDE_REST_Z), size=(0.01, 0.01, 0.01), mass=M.GUIDE_TOTAL,
        yaw=M.GUIDE_REST["heading"]),
]
TOOLS = ("torch", "guide")

# -- the cut's criteria: a 30 A-class air plasma cutter set to 20 A on 3 mm mild steel (see the README for the sources) -----
V_RATED, V_MAX = 0.0127, 0.02075  # m/s: rated (762 mm/min) and maximum (1,245 mm/min) travel for 3 mm at 20 A
V_MIN = V_RATED / 2  # slower than half the rated speed: low-speed dross, an overheated edge
T_CUT, T_HOT = M.SEG_L / V_MAX, M.SEG_L / V_MIN  # s of arc over a 5 mm segment: cut through from 0.241; overheated beyond 0.787
STANDOFF = (0.001, 0.004)  # nozzle above the plate: the shield's rim from 1 mm into the plate's top (contact) to 2 mm above it
ZONE = 0.010  # the arc is fired only with the nozzle within this of the line
ARC_TILT = math.radians(30.0)
LAT = 0.002  # sound: the nozzle within this of the line
TILT = math.radians(10.0)  # sound: the torch within this of square to the plate
BEAR_GAP = 0.00075  # the shield bears on the straightedge when its side is within this of the working edge
BEAR_DOWN = 0.001  # ... and the straightedge's underside is within this of the plate's top, flat within 3 degrees
BEAR_FRAC = 0.8
HAND_ARC = 0.045  # the other hand's fingertip midpoint closer than this to the nozzle with the arc on: 0
HELD = 0.025  # a gripper's fingertip midpoint within this of a tool's grip centre holds it
DROP = 0.025  # the offcut has dropped when its centre is this far below where it started
HANDS_CLEAR, LIN, ANG, STILL_S = 0.06, 0.01, 0.5, 1.0
REST_XY, REST_Z, REST_TILT, REST_TURN = 0.015, 0.006, math.radians(10.0), math.radians(20.0)
GUIDE_REST_XY = 0.020


def _fresh():
    return {"dwell": np.zeros(N), "bear": np.zeros(N), "lat": np.zeros(N), "tilt": np.zeros(N), "arc": False, "bad": False, "unsafe": False,
            "still": 0.0, "last_x": None, "v": 0.0}


_trial = _fresh()


def reset_latch():
    """Forget the trial: the embodiment calls this at every reset. Nothing a robot does clears a latch."""
    _trial.clear()
    _trial.update(_fresh())


def _rot(q):
    x, y, z, w = q
    return np.array(((1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)), (2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)),
                     (2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y))))


def _world(part, p):
    return np.asarray(part.pos, float) + _rot(part.quat_xyzw) @ np.asarray(p, float)


def holder(objects, name):
    """Which hand ('left_tcp' or 'right_tcp') holds a tool: its fingertip midpoint within HELD of the grip's centre."""
    p = np.asarray(objects[name].pos, float)
    best = min(("left_tcp", "right_tcp"), key=lambda a: np.linalg.norm(np.asarray(objects[a].pos, float) - p))
    return best if np.linalg.norm(np.asarray(objects[best].pos, float) - p) <= HELD else None


def _box_dist(objects, part, lo, hi, hand):
    R = _rot(objects[part].quat_xyzw)
    d = R.T @ (np.asarray(objects[hand].pos, float) - np.asarray(objects[part].pos, float))
    return float(np.linalg.norm(d - np.clip(d, lo, hi)))


_OFF_HALF = np.array(((M.X1 - M.X0) / 2, (M.OFFCUT["y"][1] - M.OFFCUT["y"][0]) / 2, M.PLATE_T / 2))


def hands_clear(objects):
    """Both hands 6 cm or more from every part of the torch, the straightedge and the (hot) offcut."""
    for hand in ("left_tcp", "right_tcp"):
        if (_box_dist(objects, "torch", *M.TORCH_HULL, hand) < HANDS_CLEAR or _box_dist(objects, "guide", *M.GUIDE_HULL, hand) < HANDS_CLEAR
                or _box_dist(objects, "offcut", -_OFF_HALF, _OFF_HALF, hand) < HANDS_CLEAR):
            return False
    return True


def _turn(R):
    return math.atan2(R[1, 0], R[0, 0])


def _rested(part, x, y, z, heading, xy):
    R = _rot(part.quat_xyzw)
    p = np.asarray(part.pos, float)
    dturn = abs(math.atan2(math.sin(_turn(R) - heading), math.cos(_turn(R) - heading)))
    return bool(math.hypot(p[0] - x, p[1] - y) <= xy and abs(p[2] - z) <= REST_Z and R[2, 2] >= math.cos(REST_TILT) and dturn <= REST_TURN)


def torch_rested(objects):
    return _rested(objects["torch"], M.TORCH_REST["x"], M.TORCH_REST["y"], TOP + M.TORCH_REST_Z, M.TORCH_REST["heading"], REST_XY)


def guide_rested(objects):
    return _rested(objects["guide"], M.GUIDE_REST["x"], M.GUIDE_REST["y"], TOP + M.GUIDE_REST_Z, M.GUIDE_REST["heading"], GUIDE_REST_XY)


def offcut_dropped(objects):
    """The offcut's centre 25 mm or more below where it started, over the tray."""
    p = np.asarray(objects["offcut"].pos, float)
    (x0, x1), (y0, y1) = M.TRAY["x"], M.TRAY["y"]
    return bool(p[2] <= TOP + M.OFFCUT_C[2] - DROP and x0 <= p[0] <= x1 and y0 <= p[1] <= y1)


def cut_state(objects):
    """This step's geometry: the shield rim's centre, nozzle standoff, distance from the line, tilt, gap to the straightedge."""
    t, g = objects["torch"], objects["guide"]
    R = _rot(t.quat_xyzw)
    rim = _world(t, M.RIM)
    tilt = math.acos(min(1.0, max(-1.0, R[2, 2])))
    Rg = _rot(g.quat_xyzw)
    c = Rg.T @ (rim - np.asarray(g.pos, float))
    gap = (M.EDGE_Y - c[1]) - M.SHIELD_R  # the shield's side to the working edge, in the straightedge's plane
    down = abs(float(g.pos[2]) + M.GUIDE_Z0 - PLATE_Z) <= BEAR_DOWN and Rg[2, 2] >= math.cos(math.radians(3.0))
    beside = abs(c[0]) <= M.BAR["len"] / 2 and c[2] <= M.GUIDE_Z0 + M.BAR["t"]  # the rim below the bar's top, within its length
    return dict(rim=rim, h=float(rim[2] - PLATE_Z + M.RECESS), x=float(rim[0]), lat=float(rim[1] - M.LINE_Y), tilt=tilt, gap=float(gap),
                bearing=bool(down and beside and abs(gap) <= BEAR_GAP), torch_hand=holder(objects, "torch"), guide_hand=holder(objects, "guide"))


def note_step(objects, dt):
    """Called by the embodiment after every control step: arc, travel per segment, faults, and the clock for all at rest."""
    tr = _trial
    if any(objects[n].pos[2] < TOP - 0.01 for n in TOOLS):
        tr["bad"] = True
    w = cut_state(objects)
    arc = bool(not tr["bad"] and w["torch_hand"] is not None and w["tilt"] <= ARC_TILT and STANDOFF[0] <= w["h"] <= STANDOFF[1]
               and M.X0 <= w["x"] <= M.X1 and abs(w["lat"]) <= ZONE)
    if arc:
        i = min(N - 1, int((w["x"] - M.X0) / M.SEG_L))
        tr["dwell"][i] += dt
        tr["bear"][i] += dt if w["bearing"] else 0.0
        tr["lat"][i] = max(tr["lat"][i], abs(w["lat"]))
        tr["tilt"][i] = max(tr["tilt"][i], w["tilt"])
        other = "left_tcp" if w["torch_hand"] == "right_tcp" else "right_tcp"
        nozzle = w["rim"] + (0.0, 0.0, M.RECESS)
        if np.linalg.norm(np.asarray(objects[other].pos, float) - nozzle) < HAND_ARC:
            tr["unsafe"] = True
    tr["arc"] = arc
    tr["v"] = 0.0 if tr["last_x"] is None else (w["x"] - tr["last_x"]) / dt
    tr["last_x"] = w["x"]
    still = all(np.linalg.norm(objects[n].lin_vel) <= LIN and np.linalg.norm(objects[n].ang_vel) <= ANG for n in (*TOOLS, "offcut")) and hands_clear(objects)
    tr["still"] = tr["still"] + dt if still else 0.0


def cut_through():
    return _trial["dwell"] >= T_CUT - 1e-9


def overheated():
    return _trial["dwell"] > T_HOT + 1e-9


def sound_segments():
    tr = _trial
    return cut_through() & ~overheated() & (tr["lat"] <= LAT) & (tr["tilt"] <= TILT) & (tr["bear"] >= BEAR_FRAC * tr["dwell"] - 1e-9)


def severed():
    """Segment indices whose joint to the offcut is to be switched off."""
    return [int(i) for i in np.flatnonzero(cut_through())]


def faults():
    """Per segment, for the record: 'sound', 'dross' (overheated), 'off-line', 'bevel', 'unguided', 'uncut' (arc, not through) or ''."""
    tr = _trial
    out = []
    for i in range(N):
        if tr["dwell"][i] <= 0:
            out.append("")
        elif not cut_through()[i]:
            out.append("uncut")
        elif overheated()[i]:
            out.append("dross")
        elif tr["lat"][i] > LAT:
            out.append("off-line")
        elif tr["tilt"][i] > TILT:
            out.append("bevel")
        elif tr["bear"][i] < BEAR_FRAC * tr["dwell"][i] - 1e-9:
            out.append("unguided")
        else:
            out.append("sound")
    return out


def shown():
    """The kerf pieces and the arc's glow to show now (prim names)."""
    on = set()
    through, hot = cut_through(), overheated()
    for i in range(N):
        if through[i]:
            on.add(f"kerf_{i}")
            if hot[i]:
                on.add(f"dross_{i}")
        elif _trial["dwell"][i] > 0:
            on.add(f"gouge_{i}")
    if _trial["arc"]:
        on.add("fx_arc")
    return on


def score(objects):
    """Read at the end of the run; there's no early end. 0 for good if the torch or the straightedge falls off the table, or the
    other hand was within 45 mm of the nozzle with the arc on. Otherwise 0.8 x the fraction of the 40 segments that are sound;
    0.9 with all 40 sound and the offcut dropped into the tray; 1.0 with, as well, the torch back on its rest, the straightedge
    back on its pad, both hands clear and everything at rest for the last 1 s."""
    tr = _trial
    if tr["bad"] or tr["unsafe"]:
        return 0.0
    good = sound_segments()
    if good.all() and offcut_dropped(objects):
        if torch_rested(objects) and guide_rested(objects) and hands_clear(objects) and tr["still"] >= STILL_S:
            return 1.0
        return 0.9
    return round(0.8 * float(good.sum()) / N, 3)


def debug_line(objects, t):
    w, tr = cut_state(objects), _trial
    r = lambda v: np.round(1000 * np.asarray(v), 1).tolist()  # noqa: E731
    g = objects["guide"]
    gyaw = math.degrees(_turn(_rot(g.quat_xyzw)))
    return (f"PLASMA t={t:6.2f} arc {int(tr['arc'])} x {1000 * w['x']:6.1f} lat {1000 * w['lat']:6.2f} h {1000 * w['h']:5.2f} tilt {math.degrees(w['tilt']):4.1f} "
            f"gap {1000 * w['gap']:6.2f} bear {int(w['bearing'])} v {1000 * tr['v']:6.1f} hands {w['torch_hand']},{w['guide_hand']} L {r(objects['left_tcp'].pos)} "
            f"R {r(objects['right_tcp'].pos)} torch {r(objects['torch'].pos)} guide {r(g.pos)} gyaw {gyaw:6.2f} off {r(objects['offcut'].pos)} "
            f"cut {int(cut_through().sum())} sound {int(sound_segments().sum())} hot {int(overheated().sum())} unsafe {int(tr['unsafe'])} "
            f"rested {int(torch_rested(objects))}{int(guide_rested(objects))} clear {int(hands_clear(objects))} still {tr['still']:.2f} score {score(objects):.3f}")


# -- scripted demo -----------------------------------------------------------------------------------------------------
# Open loop from the known start: the plan reads no simulator state. PLASMA_DEMO switches the shortcut variants:
# "nothing": no motion. "freehand": the straightedge stays on its pad and the torch runs along the line alone.
# "unheld": the straightedge is set on the plate and let go. "fast": travel at 30 mm/s. "slow": travel at 4 mm/s.
# "half": the cut stops halfway. "push": no torch; the right gripper pushes down on the offcut.
# "snap": the cut stops two segments short, then the torch's shield pushes the offcut's far end down to snap the last bridge.
VARIANT = os.environ.get("PLASMA_DEMO", "")


def _env(name, default):
    return float(os.environ.get(name, default))


FINGER_PER_M = 0.04695 / 0.094
SIDE = {"left": 1, "right": -1}
PARK = 0.12, 0.34, TOP + 0.18
SEGMENT = 0.03
DT = 0.05
SPEED = {"fast": 0.030, "slow": 0.004}.get(VARIANT, _env("PLASMA_SPEED", V_RATED))
GRIP_DZ = 0.003  # TCP above each grip block's centre when holding
SAG_T = _env("PLASMA_SAG_T", 0.0100)  # the torch hand sits this much below its target with the torch in it
SIDE_T = _env("PLASMA_SIDE_T", 0.0)  # ... and this much toward +y
LAG_T = _env("PLASMA_LAG_T", 0.0)
HOVER = _env("PLASMA_HOVER", 0.0015)  # the rim's aimed height above the plate as it comes onto the far edge
DROP_Z = _env("PLASMA_DROP", 0.003)  # then the aim goes this much lower, so the shield rides on the plate
PUSH = _env("PLASMA_PUSH", 0.012)  # the rim's centre is aimed this far past where the straightedge stops it
PRESS = _env("PLASMA_PRESS", 0.010)  # the straightedge hand is aimed this far below the plate
GUIDE_DX, GUIDE_DY = _env("PLASMA_GUIDE_DX", 0.005), _env("PLASMA_GUIDE_DY", -0.0097)  # aim offsets for the straightedge hand's give
GUIDE_YAW = math.radians(_env("PLASMA_GUIDE_YAW", -0.39))
RISE = _env("PLASMA_RISE", 0.0012)  # the torch hand rides this much higher by the near edge than at the far one
RUN_IN, RUN_OUT = 0.018, 0.006  # the torch meets the straightedge this far before the far edge and stops this far past the near one


def _fingers(width):
    return -FINGER_PER_M * width


TORCH_OPEN, TORCH_GRIP = _fingers(M.GRIP["x"] + 0.03), _fingers(M.GRIP["x"] - 0.004)
GUIDE_OPEN, GUIDE_GRIP = _fingers(M.HBLOCK + 0.03), _fingers(M.HBLOCK - 0.004)
T_YAW = M.TORCH_REST["heading"] - math.pi / 2  # the torch hand's fingers close along the handle
G_YAW = 0.0  # the straightedge hand's fingers close across the bar, so the bar's weight can't turn the block in them


def _rz(a):
    c, s = math.cos(a), math.sin(a)
    return np.array(((c, -s, 0.0), (s, c, 0.0), (0.0, 0.0, 1.0)))


def torch_tcp(rim):
    """The torch hand's TCP for the shield rim's centre at ``rim`` (world), the handle pointing back along -y."""
    return np.asarray(rim, float) - _rz(M.TORCH_REST["heading"]) @ M.RIM + (0.0, 0.0, GRIP_DZ)


class _Plan:
    def __init__(self):
        self.wps = {a: [] for a in SIDE}
        self.t = {a: 0.0 for a in SIDE}
        self.last = {}
        for a in SIDE:
            pos, _ = ik.tcp_pose(np.array(READY, float), np.array(BASES[a]))
            self.last[a] = (tuple(float(v) for v in pos), FINGER_OPEN, 0.0)

    def go(self, arm, dur, pos=None, finger=None, label="", yaw=None):
        p0, f0, y0 = self.last[arm]
        p = p0 if pos is None else tuple(float(v) for v in pos)
        f = f0 if finger is None else finger
        y = y0 if yaw is None else yaw
        n = 1 if label == "down" else max(1, math.ceil(np.linalg.norm(np.subtract(p, p0)) / SEGMENT), math.ceil(abs(y - y0) / math.radians(15)))
        for k in range(1, n + 1):
            q = tuple(float(v) for v in np.asarray(p0) + (np.asarray(p) - np.asarray(p0)) * k / n)
            self.wps[arm].append(Waypoint(dur / n, q, f0 + (f - f0) * k / n, label, y0 + (y - y0) * k / n))
        self.last[arm] = (p, f, y)
        self.t[arm] += dur

    def step(self, arm, pos, label):
        """One control step's target: the cut is planned step by step so the torch moves steadily."""
        _, f0, y0 = self.last[arm]
        self.wps[arm].append(Waypoint(DT, tuple(float(v) for v in pos), f0, label, y0))
        self.last[arm] = (tuple(float(v) for v in pos), f0, y0)
        self.t[arm] += DT

    def until(self, arm, t):
        if t > self.t[arm] + 1e-9:
            self.go(arm, t - self.t[arm], label="wait")


def _park(plan, arm, dur=1.6):
    plan.go(arm, dur, (PARK[0], SIDE[arm] * PARK[1], PARK[2]), None, "away")


def _torch_rest_tcp():
    return np.array((M.TORCH_REST["x"], M.TORCH_REST["y"], TOP + M.TORCH_REST_Z + GRIP_DZ))


def _guide_rest_tcp():
    return np.array((M.GUIDE_REST["x"], M.GUIDE_REST["y"], TOP + M.GUIDE_REST_Z + GRIP_DZ))


def _pick(plan, arm, p, yaw, opened, closed, what):
    plan.go(arm, 1.6, p + (0, 0, 0.05), opened, f"{what}: above it", yaw)
    plan.go(arm, 0.3, label="settle")
    plan.go(arm, 0.8, p, label="descend")
    plan.go(arm, 0.6, finger=closed, label="grip")
    plan.go(arm, 0.8, p + (0, 0, 0.04), label="lift")


def _put_back(plan, arm, p, yaw, opened, what):
    up = 0.10 if what == "straightedge" else 0.04  # the straightedge goes back over the clamps
    plan.go(arm, 0.8, np.array(plan.last[arm][0]) + (0, 0, up), label=f"{what}: up")
    plan.go(arm, 3.0, p + (0, 0, up), label=f"{what}: over its rest", yaw=yaw)
    plan.go(arm, 1.0, p + (0, 0, 0.001), label=f"{what}: down")
    plan.go(arm, 0.6, finger=opened, label=f"{what}: let go")
    plan.go(arm, 0.8, p + (0, 0, 0.06), label="up")
    _park(plan, arm)


def _set_guide(plan):
    """The left hand sets the straightedge on the plate, its edge one shield radius from the line, and presses it down."""
    a = "left"
    _pick(plan, a, _guide_rest_tcp(), G_YAW, GUIDE_OPEN, GUIDE_GRIP, "straightedge")
    use = np.array((M.GUIDE_USE["x"] + GUIDE_DX, M.GUIDE_USE["y"] + GUIDE_DY, TOP + M.GUIDE_USE["z"] + GRIP_DZ))
    plan.go(a, 0.8, _guide_rest_tcp() + (0, 0, 0.11), label="straightedge: up, clear of the clamps")
    plan.go(a, 3.0, use + (0, 0, 0.06), label="straightedge: over the plate", yaw=G_YAW + GUIDE_YAW)
    plan.go(a, 0.5, label="settle")
    plan.go(a, 1.2, use + (0, 0, 0.004), label="straightedge: down")
    if VARIANT == "unheld":
        plan.go(a, 0.6, finger=GUIDE_OPEN, label="straightedge: let go")
        plan.go(a, 0.8, use + (0, 0, 0.06), label="up")
        _park(plan, a)
    else:
        plan.go(a, 1.0, use - (0, 0, PRESS), label="straightedge: hold down")


def _cut(plan):
    """The right hand brings the shield against the straightedge beyond the plate's far edge and drags it to the near edge."""
    a = "right"
    guided = VARIANT != "freehand"
    y_run = M.LINE_Y + (PUSH if guided else 0.0) - SIDE_T
    z = lambda h: PLATE_Z + h + SAG_T  # noqa: E731
    x = M.X1 + RUN_IN
    plan.go(a, 3.0, torch_tcp((x, M.LINE_Y - 0.012, PLATE_Z + 0.03)), label="torch: beyond the far edge", yaw=T_YAW)
    plan.go(a, 1.0, torch_tcp((x, M.LINE_Y - 0.012, z(HOVER))), label="torch: down to the plate's level")
    plan.until(a, plan.t["left"] + 0.5)  # only once the straightedge is down and held: the shield must meet it, not the other way round
    plan.go(a, 1.0, torch_tcp((x, y_run, z(HOVER))), label="torch: against the straightedge")
    plan.go(a, 0.4, label="settle")
    return x, y_run, z


def _travel(plan, x, y_run, z):
    a = "right"
    stop = (M.X0 + M.X1) / 2 if VARIANT == "half" else M.X0 + 0.0058 if VARIANT == "snap" else M.X0 - RUN_OUT
    while x > stop + 1e-9:
        x = max(stop, x - SPEED * DT)
        k = min(max((M.X1 - 0.004 - x) / 0.010, 0.0), 1.0)  # once on the plate, ease the shield down onto it
        plan.step(a, torch_tcp((x - LAG_T, y_run, z(HOVER - k * DROP_Z - RISE * (M.X1 - x) / (M.X1 - M.X0)))), "cut")
        plan.step("left", plan.last["left"][0], "hold")
    plan.go(a, 0.5, torch_tcp((x, M.LINE_Y - 0.004, z(0.012))), label="torch: lift off")
    if VARIANT == "snap":
        far = (M.X1 - 0.012, M.LINE_Y - 0.038)
        plan.go(a, 2.0, torch_tcp((*far, z(0.02))), label="torch: over the offcut's far end")
        plan.go(a, 1.5, torch_tcp((*far, z(-0.06))), label="push the offcut down")
        plan.go(a, 1.5, label="hold")
        plan.go(a, 1.0, torch_tcp((*far, z(0.03))), label="up")


def demo():
    if VARIANT == "nothing":
        return {}
    plan = _Plan()
    for arm in SIDE:  # down from the ready pose first, so the wrist turns low
        p0 = plan.last[arm][0]
        plan.go(arm, 1.0, (p0[0], p0[1], TOP + 0.17), label="down")
    if VARIANT == "push":
        a = "right"
        p = np.array((M.OFFCUT_C[0], M.OFFCUT_C[1] - 0.012, PLATE_Z + 0.013))
        plan.go(a, 2.5, p + (0, 0, 0.05), 0.0, "over the offcut", T_YAW)
        plan.go(a, 1.5, p - (0, 0, 0.04), label="push down on it")
        plan.go(a, 3.0, label="hold")
        plan.go(a, 1.0, p + (0, 0, 0.08), label="up")
        _park(plan, a)
        return {a: plan.wps[a]}
    if VARIANT != "freehand":
        _set_guide(plan)
    _pick(plan, "right", _torch_rest_tcp(), T_YAW, TORCH_OPEN, TORCH_GRIP, "torch")
    x, y_run, z = _cut(plan)
    t0 = max(plan.t.values())
    for a in SIDE:
        plan.until(a, t0)
    _travel(plan, x, y_run, z)
    _put_back(plan, "right", _torch_rest_tcp(), T_YAW, TORCH_OPEN, "torch")
    if VARIANT not in ("freehand", "unheld"):
        _put_back(plan, "left", _guide_rest_tcp(), G_YAW, GUIDE_OPEN, "straightedge")
    end = max(plan.t.values()) + 2.0
    for a in SIDE:
        plan.until(a, end)
    return plan.wps


def ik_errors(wps):
    worst_p = worst_r = 0.0
    bad = []
    for arm, ws in wps.items():
        q = np.array(READY, float)
        for w in ws:
            q, pe, re = ik.solve(np.asarray(w.pos, float), ik.matrix_to_quat_xyzw(grip_rotation(w.yaw)), q, base_pos=BASES[arm])
            worst_p, worst_r = max(worst_p, pe), max(worst_r, re)
            if pe > 1e-3 or re > math.radians(1.0):
                bad.append((arm, w.label[:24], tuple(round(v, 3) for v in w.pos), round(pe * 1000, 1), round(math.degrees(re), 2)))
    return worst_p, worst_r, bad


# -- self-test (no simulator): the demo fits the run limit, IK reaches every waypoint, the tools' masses and balance -----
if __name__ == "__main__":
    wps = demo()
    t = max((sum(w.duration for w in ws) for ws in wps.values()), default=0.0)
    pe, re, bad = ik_errors(wps)
    print(f"demo {t:.1f} s ({t * 20:.0f} steps), IK {pe * 1000:.2f} mm / {math.degrees(re):.2f} deg, {len(bad)} bad", bad[:6])
    print("torch", round(M.TORCH_TOTAL, 4), "kg, COM from under the grip (mm):", np.round(1000 * M.TORCH_COM[:2], 2), "| straightedge", round(M.GUIDE_TOTAL, 4),
          "kg, COM (mm):", np.round(1000 * M.GUIDE_COM, 1), "| offcut", round(M.OFFCUT_MASS, 4), "kg | T_CUT, T_HOT", round(T_CUT, 3), round(T_HOT, 3))
    assert t * 20 < 2400 and not bad and np.linalg.norm(M.TORCH_COM[:2]) < 0.001
