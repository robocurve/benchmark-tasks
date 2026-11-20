"""Four baked cookies on a quarter sheet pan, a wire cooling rack to its right, and an offset turner on a rest.

Created 11_15_26 18:55 EST.
Run it:  inspect-robots run --embodiment yam_cookie -E task=task.py -E object_jitter=0 -E control_hz=20 --policy yam_scripted \\
             --no-prompt --instruction "<see task.json>" --scorer task_score --max-steps 2400

On the station table (x forward from the arms, y left, z up; the arms stand at y +0.305 and -0.305):
- An aluminium quarter sheet pan (330 x 241 mm rim, 25 mm deep, 0.40 kg), long side pointing away from the robot, centred
  at x 0.335, y -0.060. It is loose and slides on the table.
- Four baked cookies (76 mm across, 10 mm thick, 35 g) on the far two thirds of the pan, at x 0.335 and 0.440,
  y -0.115 and -0.005. They are brittle: a cookie breaks if it is squeezed, pressed, pushed hard against something,
  levered or dropped.
- A wire cooling rack (305 x 216 mm, 17 mm high, 0.22 kg), loose, right of the pan, centred at x 0.360, y -0.330.
- An offset turner (cookie spatula: a 90 x 65 mm steel blade 0.6 mm thick, 30 mm below a 110 mm handle, 69 g) lying
  on a rest (two posts under the handle, a pad under the blade) right of the rack, its handle's middle at x 0.280, y -0.600, 39 mm above the table, blade forward.
The cookies must not be touched by the grippers or arms. Scoring is described in score().
"""

import math
import os

import numpy as np
from cookie_rack import parts as P
from yam_arena.task_file import TABLE_TOP, Box

SCENE = "station"
TOP = TABLE_TOP[SCENE]
HZ = 20.0
VARIANT = os.environ.get("COOKIE_VARIANT", "")
COUNT = int(os.environ.get("COOKIE_N", "4"))  # dev only: how many cookies the demo moves

PAN_XY = (0.335, -0.060)
RACK_XY = (0.360, -0.330)
REST_XY = (0.280, -0.600)
COOKIES = ((0.335, -0.115, 0.0), (0.335, -0.005, 45.0), (0.440, -0.115, 90.0), (0.440, -0.005, 135.0))  # x, y, heading (deg)
SPOTS = ((0.460, -0.382), (0.460, -0.278), (0.355, -0.382), (0.355, -0.278))  # where the demo aims each cookie on the rack
N = len(COOKIES)
FLOOR_Z = TOP + P.PAN_FLOOR_T
TURNER_Z = TOP + P.POST_H + P.HANDLE[2] / 2


def _b(name, pos, yaw=0.0, mass=0.1):
    return Box(name, pos=pos, size=(0.01, 0.01, 0.01), mass=mass, yaw=math.radians(yaw))


OBJECTS = [
    _b("pan", (*PAN_XY, TOP + 0.0003), mass=P.PAN_MASS),
    _b("rack", (*RACK_XY, TOP + 0.0003), mass=P.RACK_MASS),
    Box("rest", pos=(*REST_XY, TOP), size=(0.01, 0.01, 0.01)),
    _b("turner", (*REST_XY, TURNER_Z + 0.0005), mass=P.TURNER_TOTAL),
]
COOKIE_NAMES = tuple(f"cookie_{i}" for i in range(1, len(COOKIES) + 1))
for _n, (_x, _y, _a) in zip(COOKIE_NAMES, COOKIES):
    OBJECTS.append(_b(_n, (_x, _y, FLOOR_Z + 0.0005), yaw=_a, mass=P.COOKIE_MASS))

# -- scoring thresholds --------------------------------------------------------------------------------------------
PER_COOKIE = 0.2
SPOILED = 0.1
FINISH = 1.0 - N * PER_COOKIE
TOUCH_N = 0.01  # N: a robot link pressing on a cookie with at least this force has touched it
DROP_SPEED = 0.6  # m/s downward: a fall of about 18 mm or more
FLAT_DEG = 8.0
ON_RACK_Z = (-0.002, 0.004)  # the cookie's base relative to the wires' top
PAN_TIP_DEG = 10.0
PAN_LIFT = 0.020
PAN_MOVED = 0.030  # the pan's centre may end this far from its start
PAN_TURNED_DEG = 10.0
REST_XY_TOL, REST_Z_TOL, REST_TURN_DEG = 0.015, 0.006, 15.0
CLEAR = 0.08  # each fingertip midpoint ends at least this far from the pan, the rack and the turner
STILL_S = 1.0


def _fresh():
    return {"broken": [False] * N, "touched": [False] * N, "dropped": [False] * N, "pan_tipped": False, "still": 0.0,
            "peak": dict.fromkeys(("touch", "bend", "squeeze", "press", "fall"), 0.0), "pan0": None}


_trial = _fresh()


def reset_trial():
    global _trial
    _trial = _fresh()


def trial():
    return _trial


def _rot(q):
    x, y, z, w = (float(v) for v in q)
    return np.array(((1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)),
                     (2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)),
                     (2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y))))


def _tilt_deg(q):
    return math.degrees(math.acos(max(-1.0, min(1.0, _rot(q)[2, 2]))))


def _yaw(q):
    r = _rot(q)
    return math.atan2(r[1, 0], r[0, 0])


def _local(obj, p):
    return _rot(obj.quat_xyzw).T @ (np.asarray(p, float) - np.asarray(obj.pos, float))


def _box_dist(obj, lo, hi, p):
    q = _local(obj, p)
    return float(np.linalg.norm(np.maximum(np.maximum(np.asarray(lo) - q, q - np.asarray(hi)), 0.0)))


def note_step(objects, dt, loads):
    """Latch what cannot be read off the final layout. loads: {cookie: {"touch": N, "bend": N m, "squeeze": N,
    "press": N, "fall": m/s}}, each the peak the simulator measured on that cookie during this control step (it looks
    after every physics step, 240 times a second)."""
    t = _trial
    pan = objects["pan"]
    if t["pan0"] is None:
        t["pan0"] = (np.array(pan.pos, float), _yaw(pan.quat_xyzw))
    if _tilt_deg(pan.quat_xyzw) > PAN_TIP_DEG or pan.pos[2] - t["pan0"][0][2] > PAN_LIFT:
        t["pan_tipped"] = True
    moving = False
    for i, name in enumerate(COOKIE_NAMES):
        load = loads.get(name, {})
        for k in t["peak"]:
            t["peak"][k] = max(t["peak"][k], float(load.get(k, 0.0)))
        if load.get("touch", 0.0) >= TOUCH_N:
            t["touched"][i] = True
        if load.get("bend", 0.0) >= P.BREAK_BEND or load.get("squeeze", 0.0) >= P.BREAK_SQUEEZE or load.get("press", 0.0) >= P.BREAK_PRESS:
            t["broken"][i] = True
        if load.get("fall", 0.0) > DROP_SPEED or objects[name].lin_vel[2] < -DROP_SPEED:
            t["dropped"][i] = True
        moving = moving or not objects[name].resting(0.02, 1.0)
    for name in ("pan", "rack", "turner"):
        moving = moving or not objects[name].resting(0.02, 1.0)
    t["still"] = 0.0 if moving or not hands_clear(objects) else t["still"] + dt


def broken_names():
    return {n for n, b in zip(COOKIE_NAMES, _trial["broken"]) if b}


def where(objects, i):
    """Cookie i by its centre: "rack_ok" (flat, wholly on the rack's wires, at rest), "rack", "pan" or "off"."""
    a = objects[COOKIE_NAMES[i]]
    c = np.asarray(a.pos, float)
    rack, pan = objects["rack"], objects["pan"]
    q = _local(rack, c)
    hx, hy = P.RACK[0] / 2, P.RACK[1] / 2
    if abs(q[0]) <= hx and abs(q[1]) <= hy and q[2] > 0.004:
        inside = abs(q[0]) <= hx - P.COOKIE_R + 0.004 and abs(q[1]) <= hy - P.COOKIE_R + 0.004
        level = ON_RACK_Z[0] <= q[2] - P.RACK_TOP <= ON_RACK_Z[1] and _tilt_deg(a.quat_xyzw) <= FLAT_DEG
        on_turner = _box_dist(objects["turner"], (P.BLADE_X0, -P.BLADE_W / 2, P.BLADE_TOP - 0.002), (P.BLADE_X0 + P.BLADE_L, P.BLADE_W / 2, P.BLADE_TOP + 0.012), c + (0, 0, 0.003)) < 1e-9
        return "rack_ok" if inside and level and a.resting(0.02, 1.0) and not on_turner else "rack"
    q = _local(pan, c)
    if abs(q[0]) <= P.PAN_OUT[0] / 2 and abs(q[1]) <= P.PAN_OUT[1] / 2 and q[2] < P.PAN_RIM + 0.02:
        return "pan"
    return "off"


def cookie_status(objects, i):
    """"good" (unbroken, untouched, never dropped, flat on the rack), "spoiled" or "left"."""
    t = _trial
    w = where(objects, i)
    if t["broken"][i] or t["touched"][i] or t["dropped"][i] or w == "off":
        return "spoiled"
    return "good" if w == "rack_ok" else "left"


def pan_home(objects):
    t = _trial
    if t["pan0"] is None or t["pan_tipped"]:
        return False
    pan = objects["pan"]
    turn = abs((_yaw(pan.quat_xyzw) - t["pan0"][1] + math.pi) % (2 * math.pi) - math.pi)
    return (float(np.linalg.norm(np.asarray(pan.pos)[:2] - t["pan0"][0][:2])) <= PAN_MOVED and math.degrees(turn) <= PAN_TURNED_DEG
            and _tilt_deg(pan.quat_xyzw) <= 3.0 and abs(pan.pos[2] - t["pan0"][0][2]) <= 0.003)


def turner_rested(objects):
    tu = objects["turner"]
    d = np.asarray(tu.pos, float) - (*REST_XY, TURNER_Z)
    turn = abs((_yaw(tu.quat_xyzw) + math.pi) % (2 * math.pi) - math.pi)
    return bool(np.linalg.norm(d[:2]) <= REST_XY_TOL and abs(d[2]) <= REST_Z_TOL and math.degrees(turn) <= REST_TURN_DEG and _tilt_deg(tu.quat_xyzw) <= 6.0)


def hands_clear(objects):
    boxes = (("pan", (-P.PAN_OUT[0] / 2, -P.PAN_OUT[1] / 2, 0.0), (P.PAN_OUT[0] / 2, P.PAN_OUT[1] / 2, P.PAN_RIM)),
             ("rack", (-P.RACK[0] / 2, -P.RACK[1] / 2, 0.0), (P.RACK[0] / 2, P.RACK[1] / 2, P.RACK_TOP + P.COOKIE_T)),
             ("turner", (-P.HANDLE[0] / 2, -P.BLADE_W / 2, P.TIP[2]), (P.TIP[0], P.BLADE_W / 2, P.HANDLE[2] / 2)))
    return all(_box_dist(objects[n], lo, hi, objects[f"{arm}_tcp"].pos) >= CLEAR for n, lo, hi in boxes for arm in ("left", "right"))


def counts(objects):
    s = [cookie_status(objects, i) for i in range(N)]
    return {k: s.count(k) for k in ("good", "spoiled", "left")}


def finished(objects):
    return pan_home(objects) and turner_rested(objects) and hands_clear(objects) and _trial["still"] >= STILL_S


def score(objects):
    """0.2 for each cookie that ends flat and wholly on the rack, in one piece, never touched by the robot and never
    dropped; minus 0.1 for each cookie spoiled (broken, touched by a gripper or arm, dropped 18 mm or more, or ending
    anywhere but the pan or the rack); never below 0. The last 0.2 only when all four are good, the pan is where it
    started (within 30 mm and 10 degrees, flat), the turner is back on its rest, and both hands are 80 mm clear and
    everything has been still for a second. 0 for the whole trial if the pan was ever tipped past 10 degrees or lifted
    20 mm off the counter."""
    if _trial["pan_tipped"]:
        return 0.0
    c = counts(objects)
    s = PER_COOKIE * c["good"] - SPOILED * c["spoiled"]
    if c["good"] == N and finished(objects):
        s += FINISH
    return float(min(1.0, max(0.0, s)))


def success(objects):
    return score(objects) >= 1.0 - 1e-9


def debug_line(objects):
    t = _trial
    r = lambda v: np.round(1000 * np.asarray(v), 1).tolist()  # noqa: E731
    tu = objects["turner"]
    tip = np.asarray(tu.pos) + _rot(tu.quat_xyzw) @ P.TIP
    rt = _rot(tu.quat_xyzw)
    roll = math.degrees(math.atan2(rt[2, 1], rt[2, 2]))
    ck = " ".join(f"c{i + 1} {r(objects[n].pos)} onblade {r(_local(tu, objects[n].pos))} tilt {_tilt_deg(objects[n].quat_xyzw):4.1f} gap {int(t['broken'][i])} "
                  f"{where(objects, i)}/{cookie_status(objects, i)}" for i, n in enumerate(COOKIE_NAMES))
    pk = t["peak"]
    return (f"tip {r(tip)} pitch {math.degrees(math.asin(max(-1, min(1, -rt[2, 0])))):5.1f} roll {roll:5.1f} turner {r(tu.pos)} R {r(objects['right_tcp'].pos)} L {r(objects['left_tcp'].pos)} "
            f"pan {r(objects['pan'].pos)} rack {r(objects['rack'].pos)} | {ck} | touch {pk['touch']:.3f} bend {pk['bend']:.4f} squeeze {pk['squeeze']:.2f} press {pk['press']:.2f} fall {pk['fall']:.2f} "
            f"tipped {int(t['pan_tipped'])} home {int(pan_home(objects))} rested {int(turner_rested(objects))} clear {int(hands_clear(objects))} still {t['still']:.1f} score {score(objects):.2f}")


# -- scripted demo: open loop from the layout above; reads nothing from the simulator ---------------------------------
SCOOP_PITCH = math.radians(float(os.environ.get("COOKIE_PITCH", "6")))
CARRY_PITCH = math.radians(float(os.environ.get("COOKIE_CARRY", "-1")))
TIP_PITCH = math.radians(float(os.environ.get("COOKIE_TIP", "18")))
PRESS = float(os.environ.get("COOKIE_PRESS", "0.003"))  # the blade's front edge is commanded this far into the pan's floor
SCOOP_SPEED = float(os.environ.get("COOKIE_SPEED", "0.05"))
SET_DOWN = float(os.environ.get("COOKIE_SETDOWN", "0.004"))  # the front edge is held this far above the wires to let the cookie off
# Finger target on the handle: each pad 3 mm past the handle's face, about 15 N a side. Fully closed, both fingers sit at
# their force limit, the handle is left floating between them and it buzzes sideways, which walks the cookie off the blade.
DROP_H = float(os.environ.get("COOKIE_DROP", "0.040"))  # dev: the "drop" variant tips the cookie off from this far above the wires
GRIP = -(P.HANDLE[1] / 2 - float(os.environ.get("COOKIE_GRIP", "0.003")))
LEAD = float(os.environ.get("COOKIE_LEAD", "0.005"))  # the blade's front edge stops this far short of the spot


def tcp_for(tip, pitch):
    """Where the fingertip midpoint goes to put the blade's front edge at tip with the turner pitched (held at its middle)."""
    from cookie_rack.plan import ry

    return np.asarray(tip, float) - ry(pitch) @ P.TIP


def _right_path():
    from cookie_rack.plan import Path
    from yam_arena.station import BASES

    p = Path(BASES["right"], 1.0 / HZ)
    rest = np.array((*REST_XY, TURNER_Z))
    up = np.array((0.0, 0.0, 0.07))
    if VARIANT == "hand":  # pick the first cookie up in the gripper and set it on the rack
        x, y, _ = COOKIES[0]
        grab = np.array((x, y, FLOOR_Z + 0.004 + 0.013))
        p.to(2.0, grab + (0, 0, 0.10), label="over cookie")
        p.to(1.5, grab, label="down")
        p.wait(1.0, "grip", finger=-0.030)
        p.to(1.5, grab + (0, 0, 0.10), label="lift")
        put = np.array((*SPOTS[0], TOP + P.RACK_TOP + 0.004 + 0.013 + 0.004))
        p.to(2.5, put + (0, 0, 0.06), label="carry")
        p.to(1.5, put, label="lower")
        p.wait(0.8, "release", finger=p.open)
        p.to(1.5, put + (0, 0, 0.06), label="up")
        p.home(3.0)
        return p
    p.to(2.0, rest + up, label="over turner")
    p.to(1.2, rest, label="down to handle")
    p.wait(0.8, "grip", finger=GRIP)
    p.to(1.2, rest + up, label="lift turner")
    for i in range(COUNT):
        x, y, _ = COOKIES[i]
        sx, sy = SPOTS[i]
        if VARIANT == "shove":  # blade held clear of the floor: it can only push the cookie along
            a = np.array((x - P.COOKIE_R - 0.03, y, FLOOR_Z + 0.004))
            p.to(2.5, tcp_for(a + (0, 0, 0.05), 0.0), 0.0, label="over pan")
            p.to(1.0, tcp_for(a, 0.0), 0.0, label="down")
            p.to(3.0, tcp_for(a + (0.12, 0, 0), 0.0), 0.0, label="shove")
            p.to(1.0, tcp_for(a + (0.12, 0, 0.06), 0.0), 0.0, label="up")
            continue
        start = np.array((x - P.COOKIE_R - 0.025, y, FLOOR_Z - PRESS))
        end = np.array((x + 0.043, y, FLOOR_Z - PRESS))
        p.to(2.5, tcp_for(start + (0, 0, 0.05), SCOOP_PITCH), SCOOP_PITCH, label=f"over pan {i + 1}")
        p.to(1.0, tcp_for(start, SCOOP_PITCH), SCOOP_PITCH, label="blade down")
        p.to((end[0] - start[0]) / SCOOP_SPEED, tcp_for(end, SCOOP_PITCH), SCOOP_PITCH, label=f"slide under {i + 1}")
        p.wait(0.3)
        p.to(1.2, tcp_for(end + (0, 0, 0.012), 0.5 * SCOOP_PITCH), 0.5 * SCOOP_PITCH, label="ease up")
        p.to(2.0, tcp_for(end + (0, 0, 0.06), CARRY_PITCH), CARRY_PITCH, label="lift level")
        over = np.array((sx - LEAD, sy, TOP + P.RACK_TOP + (DROP_H + 0.01 if VARIANT == "drop" else 0.045)))
        p.to(3.0, tcp_for(over, CARRY_PITCH), CARRY_PITCH, label="carry")
        low = np.array((sx - LEAD, sy, TOP + P.RACK_TOP + (DROP_H if VARIANT == "drop" else SET_DOWN)))
        p.to(1.2, tcp_for(low + (0, 0, 0.004), 0.0), 0.0, label="lower")
        p.to(2.5, tcp_for(low, TIP_PITCH), TIP_PITCH, label="tip")
        p.wait(1.0, "let it slide")
        p.to(2.0, tcp_for(low - (0.04, 0, 0), TIP_PITCH), TIP_PITCH, label="draw back")
        p.to(1.2, tcp_for(low + (-0.04, 0, 0.06), 0.0), 0.0, label="up")
    p.to(2.5, rest + up, label="over rest")
    p.to(1.5, rest + (0, 0, 0.002), label="set down")
    p.wait(0.6, "release", finger=p.open)
    p.to(1.2, rest + up, label="up")
    p.home(3.0)
    return p


def demo():
    if VARIANT == "idle":
        from cookie_rack.plan import JointPlan
        from yam_arena.constants import READY

        return {"right": [JointPlan([READY] * 40, [1.0] * 40, 1.0 / HZ, ["idle"] * 40)]}
    return {"right": [_right_path().joints()]}
