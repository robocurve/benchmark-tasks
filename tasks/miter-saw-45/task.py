"""On the station table, a SKIL MS6316-00 sliding miter saw (7-1/4 in, 11.1 kg), unplugged, its blade taken off, bolted
to a 19 mm plywood board, its front toward the arms: set at 0 degrees, its miter-detent pin in the 0 degree notch, its
miter-lock lever up (unlocked), its head raised.

At the front end of the saw's miter arm a black lock lever sticks out level toward the robot; under it is the black
miter-detent release lever. Squeezing the release lever up toward the lock lever lifts the detent pin out, and the arm
can be swung; with the lever let go, the pin drops into a notch (0, 15, 22.5, 31.6, 45 degrees each side) as soon as the
arm is at one. Pressing the lock lever down until it clicks clamps the arm. A red pointer beside the arm reads the angle
on a white scale.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \\
           --instruction "Set the miter saw to 45 degrees and lock it." --scorer task_score --max-steps 1850 ...
"""

import dataclasses
import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import mechkit as mk  # noqa: E402
from closedloop import grip_rot, placeholder_demo  # noqa: E402  (hooks yam_scripted to run demo_script)
from mechkit import Joint, Mech, box, cyl, finger_points, on_reset, on_step, rod, set_hold, set_rest  # noqa: E402
from yam_arena.constants import FINGER_CLOSED, PAD_TIP_PAST_TCP  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SCORE_OVER = "end"

# -- the saw. Frame: on the table under the turntable's pivot; -x is the saw's front (toward the robot), z up. Sizes are
# read off the maker's drawings and photos (about +-5 mm).
SAW_X = 0.60
B = 0.019  # plywood mounting board
H = B + 0.069  # the saw's table surface
LOCK_AT, REL_AT = (-0.262, 0.0, 0.082), (-0.200, 0.0, 0.049)  # hinges of the lock lever and of the release lever
REL_PAD = 0.088  # centre of the release lever's finger pad, from its hinge
POINTER = -18.5  # the pointer sits this many degrees to the arm's left, and the scale is turned with it
DARK, BLACK, RED = (0.20, 0.21, 0.22), (0.05, 0.05, 0.055), (0.78, 0.09, 0.09)
ALU, STEEL, PLY, SMOKE = (0.70, 0.71, 0.73), (0.80, 0.81, 0.83), (0.80, 0.66, 0.43), (0.42, 0.42, 0.43)
WHITE = (0.95, 0.95, 0.95)


def _look(shape):
    return dataclasses.replace(shape, collide=False)


def _ty(a):
    return (math.cos(a / 2), 0.0, math.sin(a / 2), 0.0)


def _tick(deg, r0, r1, z, w, color, h=0.0008):
    a, r = math.radians(deg), (r0 + r1) / 2
    return _look(box((-r * math.cos(a), -r * math.sin(a), z), (r1 - r0, w, h), color,
                     (math.cos(a / 2), 0.0, 0.0, math.sin(a / 2))))


BASE = [box((0.035, 0, B / 2), (0.350, 0.420, B), PLY),
        *[box((x, s * 0.150, B + 0.011), (0.060, 0.050, 0.022), DARK) for x in (-0.105, 0.115) for s in (-1, 1)],
        *[box((0.005, s * 0.128, B + 0.0345), (0.160, 0.076, 0.069), DARK) for s in (-1, 1)],
        *[box((0.0, s * 0.190, H - 0.009), (0.090, 0.048, 0.018), RED) for s in (-1, 1)],  # table extensions
        cyl((0, 0, B + 0.014), 0.085, 0.028, DARK), cyl((0, 0, B + 0.032), 0.104, 0.010, DARK),  # hub, notched ring
        *[box((0.011, s * 0.108, H + 0.030), (0.010, 0.150, 0.060), ALU) for s in (-1, 1)],  # fence halves
        *[_tick(d + POINTER, 0.092, 0.103, B + 0.0374, 0.0012, WHITE) for d in (-45, -31.6, -22.5, -15, 0, 15, 22.5, 31.6, 45)],
        *[_tick(d + POINTER, 0.098, 0.103, B + 0.0374, 0.0006, WHITE) for d in range(-50, 51, 5)]]
TABLE = [cyl((0, 0, H - 0.013), 0.085, 0.026, ALU), box((-0.1625, 0, H - 0.0184), (0.205, 0.056, 0.0360), DARK),
         *[_look(box((-0.135, s * 0.011, H + 0.0006), (0.230, 0.014, 0.0012), RED)) for s in (-1, 1)],  # kerf insert
         _tick(POINTER, 0.0865, 0.0905, B + 0.052, 0.018, RED, 0.030),  # pointer: a red bracket on the turntable's edge
         _tick(POINTER, 0.0905, 0.1015, B + 0.0386, 0.0016, RED),  # and its needle over the scale
         rod((-0.268, -0.014, 0.034), (-0.268, -0.014, 0.060), 0.0045, STEEL),  # end of the lock rod
         cyl((0.115, 0, H + 0.014), 0.042, 0.070, DARK, axis="x"), box((0.165, 0, H + 0.055), (0.050, 0.070, 0.150), DARK),
         box((0.175, 0, H + 0.135), (0.045, 0.110, 0.045), DARK),
         *[cyl((0.085, s * 0.034, H + 0.135), 0.011, 0.200, STEEL, axis="x") for s in (-1, 1)],  # fixed rails
         box((0.020, 0, H + 0.135), (0.056, 0.104, 0.050), DARK),
         rod((0.020, 0, H + 0.145), (-0.020, 0, H + 0.235), 0.024, DARK),
         cyl((-0.030, 0, H + 0.222), 0.106, 0.046, DARK, axis="y"), cyl((-0.034, 0, H + 0.214), 0.110, 0.030, SMOKE, axis="y"),
         cyl((-0.005, -0.088, H + 0.240), 0.036, 0.130, RED, axis="y"), cyl((-0.005, -0.158, H + 0.240), 0.030, 0.014, BLACK, axis="y"),
         rod((-0.020, 0, H + 0.310), (-0.030, 0, H + 0.368), 0.014, RED), rod((-0.105, 0, H + 0.300), (-0.120, 0, H + 0.352), 0.013, BLACK),
         rod((-0.030, 0, H + 0.368), (-0.120, 0, H + 0.352), 0.015, BLACK)]
LOCK = [box((-0.026, 0, 0.001), (0.052, 0.022, 0.006), BLACK), box((-0.068, 0, -0.007), (0.032, 0.030, 0.022), BLACK),
        cyl((0, 0, 0), 0.006, 0.030, BLACK, axis="y"),
        _look(box((-0.068, 0, 0.0042), (0.020, 0.016, 0.0006), SMOKE))]  # grip patch
RELEASE = [box((-0.040, 0, 0), (0.080, 0.016, 0.007), BLACK), box((-REL_PAD, 0, 0), (0.016, 0.022, 0.008), BLACK),
           *[_look(box((-REL_PAD + dx, 0, 0.0042), (0.002, 0.020, 0.0006), SMOKE)) for dx in (-0.005, 0.0, 0.005)]]

# -- forces (NOT measured: estimates, see README "Forces")
DRAG = 4.0  # N m to turn the table with the pin out: 13 N at the levers (0.30 m)
REL_K, REL_PRE, REL_STOP = 12.0, 0.098, 0.100  # release lever: N m/rad, preload angle, travel: 13 N to start, 27 N at its stop
PIN_OUT = 0.065  # the pin is out of its notch from this lever angle (5.7 mm at the pad, 22 N)
LOCK_DOWN = math.radians(-35.0)  # the lock lever's down position (not taken from the real saw)
LOCK_K, LOCK_HOLD = 5.74, 1.0  # over-centre: 2.75 N m peak: 33 N at its end, 46 N at 6 cm
CLAMP = 40.0  # N m the clamped lock holds
PIN_K = 1000.0  # N m/rad: the pin in its notch
CATCH = math.radians(1.0)  # with the release lever let go, this close to a notch the pin's wedge draws the table in
NOTCHES = tuple(math.radians(d) for d in (-45, -31.6, -22.5, -15, 0, 15, 22.5, 31.6, 45))

SAW = Mech("miter_saw", pos=(SAW_X, 0.0, TOP + 0.0005), mass=5.9, friction=0.8, shapes=BASE, joints=[
    Joint("table", "hinge", axis="z", low=math.radians(-47), high=math.radians(50), spring=PIN_K, damping=2.0, mass=6.9,
          hold=DRAG, shapes=TABLE),
    Joint("lock", "hinge", axis="y", at=LOCK_AT, parent="table", low=LOCK_DOWN, high=0.0, spring=LOCK_K, hold=LOCK_HOLD,
          detents=(LOCK_DOWN, 0.0), damping=0.05, mass=0.12, friction=0.8, armature=2e-4, shapes=LOCK),
    Joint("release", "hinge", axis="y", at=REL_AT, parent="table", low=0.0, high=REL_STOP, spring=REL_K, damping=0.10,
          mass=0.05, friction=0.8, armature=2e-4, shapes=RELEASE)])
OBJECTS = [SAW]

# -- the detent pin and the clamp, once every control step
_M = {}


@on_reset
def _reset(env):
    _M.clear()
    _M.update(pin=0.0, clamped=False, stiff=PIN_K, t=0.0, hist=[], start=None)
    if env is not None:
        set_rest(env, SAW.name, "release", -REL_PRE)
        set_rest(env, SAW.name, "table", 0.0)
        _set_stiffness(env, PIN_K)
        set_hold(env, SAW.name, "table", DRAG)


def _set_stiffness(env, k):
    asset = env.unwrapped.scene[SAW.name]
    write = getattr(asset, "write_joint_stiffness_to_sim_index", None)
    i, v = mk._joint_id(asset, "table"), mk._sim_tensor(asset, [float(k)])
    if write is not None:
        write(stiffness=v, joint_ids=[i])
    else:
        asset.write_joint_stiffness_to_sim(v, joint_ids=[i])
    _M["stiff"] = k


def pin_rule(pin, angle, release):
    """The notch the pin is in after this step (rad), or None. Squeezed past PIN_OUT the pin is out. With the lever let go
    it drops in wherever the arm comes within CATCH of a notch (let go there, or moved on to it), and it stays in that
    notch until the release lever is squeezed again."""
    if release >= PIN_OUT:
        return None
    if pin is not None:
        return pin
    near = min(NOTCHES, key=lambda n: abs(n - angle))
    return near if abs(near - angle) <= CATCH else None


@on_step
def _mechanism(env):
    s = mk._mech_state(env, SAW)
    pin = pin_rule(_M["pin"], s.joints["table"], s.joints["release"])
    if pin is not None and pin != _M["pin"]:
        set_rest(env, SAW.name, "table", pin)
    if (pin is not None) != (_M["stiff"] > 0):
        _set_stiffness(env, PIN_K if pin is not None else 0.0)
    _M["pin"] = pin
    clamped = s.joints["lock"] <= LOCKED
    if clamped != _M["clamped"]:
        set_hold(env, SAW.name, "table", CLAMP if clamped else DRAG, CLAMP if clamped else None)
        _M["clamped"] = clamped
    try:
        _M["fingers"] = [np.asarray(p, float) for p in finger_points(env)]
    except Exception:  # noqa: BLE001 - grippers that cannot be read never count as clear
        _M["fingers"] = None
    _remember(s, float(env.unwrapped.step_dt))


# -- scoring. Every limit is inclusive (EPS only absorbs floating-point rounding).
EPS = 1e-9
LOCKED = math.radians(-30.0)  # the lock lever pressed this far down has clicked
AT = math.radians(0.5)  # the pointer this close to a notch's angle
RELEASED = math.radians(1.5)  # the release lever this close to its rest is let go
CLEAR, SHIFT = 0.030, 0.010
STILL_S = 0.5
STILL_TABLE, STILL_LEVER, STILL_BOARD = math.radians(0.2), math.radians(1.0), 0.001
CORNERS = [np.array([x, y, 0.0]) for x in (-0.140, 0.210) for y in (-0.210, 0.210)]  # the board's marked corners


def _remember(s, dt):
    """Keep the poses that span the last 0.5 s, sampled every control step (20 Hz): a resting part's reported velocity is
    noisy, so stillness is judged by how far things moved between samples."""
    at = np.array([s.to_world(c) for c in CORNERS])
    _M["t"] += dt
    _M["hist"].append((_M["t"], s.joints["table"], s.joints["lock"], s.joints["release"], at))
    if _M["start"] is None:
        _M["start"] = at
    while len(_M["hist"]) > 1 and _M["hist"][1][0] <= _M["t"] - STILL_S + 1e-6:
        _M["hist"].pop(0)


def _still():
    h = _M.get("hist")
    if not h or h[-1][0] - h[0][0] < STILL_S - 1e-6:  # not yet a full half second
        return False
    d = [np.ptp([k[i] for k in h]) for i in (1, 2, 3)]
    board = np.linalg.norm(np.ptp([k[4] for k in h], axis=0), axis=1).max()
    return bool(d[0] <= STILL_TABLE + EPS and max(d[1], d[2]) <= STILL_LEVER + EPS and board <= STILL_BOARD + EPS)


def hand_gap(fingers, s):
    """Least distance (m) from any fingertip point of either arm to any part of the saw, each where it is now (a round
    part counts as the box round it); the plywood board does not count."""
    if not fingers:
        return 0.0
    pts, best = np.array([s.to_local(p) for p in fingers]), math.inf
    for name, shapes in (("", SAW.shapes[1:]), *((j.name, j.shapes) for j in SAW.joints)):
        rot, at = SAW.joint_pose(name, s.joints) if name else (np.eye(3), np.zeros(3))
        for sh in (k for k in shapes if k.collide):
            r, _, h = sh.size
            half = np.array(sh.size) / 2 if sh.kind == "box" else np.array(
                {"x": (h / 2, r, r), "y": (r, h / 2, r), "z": (r, r, h / 2)}[sh.axis])
            q = ((pts - at) @ rot - np.array(sh.at)) @ mk._quat_wxyz_matrix(sh.tilt)
            best = min(best, float(np.linalg.norm(np.maximum(np.abs(q) - half, 0.0), axis=1).min()))
    return best


def facts(objects):
    s = objects["miter_saw"]
    at = np.array([s.to_world(c) for c in CORNERS])
    start = at if _M.get("start") is None else _M["start"]
    return {"angle": s.joints["table"], "released": s.joints["release"] <= RELEASED + EPS,
            "locked": s.joints["lock"] <= LOCKED + EPS, "gap": hand_gap(_M.get("fingers"), s), "still": _still(),
            "shift": float(np.linalg.norm(at - start, axis=1).max())}


def level(f):
    if f["shift"] > SHIFT + EPS:
        return 0.0
    a = abs(f["angle"])
    at = [n for n in (15.0, 22.5, 31.6, 45.0) if abs(a - math.radians(n)) <= AT + EPS]
    if f["released"] and at == [45.0]:
        return 1.0 if f["locked"] and f["gap"] >= CLEAR - EPS and f["still"] else 0.75
    if f["released"] and at:
        return 0.5
    return 0.25 if a >= math.radians(10.0) - EPS else 0.0


def score(objects):
    """End state only. 1.0: pointer within 0.5 degrees of 45 (either side), release lever let go, lock lever down, every
    fingertip 3 cm clear of the saw, nothing moved for 0.5 s. 0.75: the same but not locked, or a fingertip too close, or
    moving. 0.5: let go within 0.5 degrees of 15, 22.5 or 31.6. 0.25: any other angle of 10 degrees or more. Else 0; 0
    whenever a corner of the saw's board is more than 10 mm from where it started."""
    f = facts(objects)
    if os.environ.get("MITER_DEBUG"):
        _debug(objects, f)
    return level(f)


def success(objects):
    return score(objects) >= 1.0


demo = placeholder_demo

# -- the scripted demo (right arm): squeeze the two levers together between the fingers, one above the other, the gripper
# 15 degrees below level; swing the arm along its arc to 45 degrees; let go; press the lock lever down from above.
GRIP_R, GRIP_Z, SQUEEZE_Z = 0.296, 0.068, 0.072  # TCP from the pivot; above the table to slide on and when squeezed
PITCH, ROLL = math.radians(75), -math.pi / 2
OPEN, SHUT, TIGHT = -0.033, -0.019, -0.0105  # finger joints: 66 mm apart to pass the levers, touching, tightest squeeze
GOAL = math.radians(45.0)


def _pose(th, r=GRIP_R, z=GRIP_Z):
    return np.array([SAW_X - r * math.cos(th), -r * math.sin(th), TOP + z]), grip_rot(th, PITCH, ROLL)


def _joints(ctx):
    return ctx.obj("miter_saw").joints


def _go(R, p, rot, t=0.1, finger=None):
    yield from R.move(p, t, finger, rot=rot)


def demo_script(ctx):
    R = ctx.right
    far, rot = _pose(0.0, GRIP_R + 0.085)
    yield from _go(R, (0.24, -0.14, TOP + 0.24), rot, 2.0, OPEN)
    yield from _go(R, far + [0, 0, 0.10], rot, 1.5)
    yield from _go(R, far, rot, 1.2)
    yield from R.trim_to_measured(far, max_duration=1.5)
    fix = R.pos - far  # what the arm's sag takes
    near = _pose(0.0)[0]
    for k in range(1, 9):
        yield from _go(R, far + fix + (near - far) * k / 8, rot, 0.25)
    yield from R.trim_to_measured(near, max_duration=1.5)
    fix = R.pos - near
    near, finger = near + [0, 0, SQUEEZE_Z - GRIP_Z], SHUT
    yield from _go(R, near + fix, rot, 2.0, finger)
    while finger < TIGHT and (ctx.offline or _joints(ctx)["release"] < 0.085):  # squeeze until the pin is well out
        finger += 0.0005
        yield from _go(R, near + fix, rot, 0.1, finger)
    yield from R.hold(0.3)
    if os.environ.get("MITER_DEBUG"):
        print(f"MITERDBG squeeze: finger {finger:.4f} measured {R.measured_finger:.4f} -> about "
              f"{5000 * (finger - R.measured_finger):.0f} N a finger; tcp {np.round(R.measured_pos - near, 4)}", flush=True)
    cmd = th = 0.0
    for _ in range(400):  # swing: lead the table's measured angle, at 10 degrees a second
        j = None if ctx.offline else _joints(ctx)
        th = cmd if j is None else j["table"]
        if th >= GOAL - math.radians(0.4):
            break
        if j is not None and j["release"] < PIN_OUT + 0.012 and finger < TIGHT:
            finger += 0.0005
        cmd = min(cmd + math.radians(0.5), th + math.radians(24), math.radians(64))
        p, rot = _pose(cmd, z=SQUEEZE_Z)
        yield from _go(R, p + fix, rot, 0.05, finger)
    for _ in range(60):  # take the sideways spring out of the arm before letting go
        m = R.measured_pos if not ctx.offline else p
        got = math.atan2(-m[1], SAW_X - m[0])
        if abs(got - cmd) < math.radians(1.0):
            break
        cmd -= math.copysign(min(math.radians(0.5), abs(cmd - got)), cmd - got)
        p, rot = _pose(cmd, z=SQUEEZE_Z)
        yield from _go(R, p + fix, rot, 0.05, finger)
    if os.environ.get("MITER_DEBUG"):
        print(f"MITERDBG let go: finger {finger:.4f} measured {R.measured_finger:.4f}, command {math.degrees(cmd):.1f} deg", flush=True)
    yield from _go(R, R.pos + [0, 0, GRIP_Z - SQUEEZE_Z], rot, 1.0, OPEN)
    yield from R.hold(0.5)
    here, out = R.pos.copy(), _pose(cmd, GRIP_R + 0.090)[0] - _pose(cmd)[0]
    for k in range(1, 9):
        yield from _go(R, here + out * k / 8, rot, 0.2)
    yield from _go(R, here + out + [0, 0, 0.14], rot, 1.2)
    yield from R.home()
    yield from _press_lock(ctx, R)
    yield from R.home()
    yield from R.hold(1.5)


def _press_lock(ctx, R):
    """Closed fingertips, straight down on the lock lever 56 mm from its hinge, lower and lower until it clicks."""
    th = GOAL if ctx.offline else _joints(ctx)["table"]
    r = 0.318
    at = np.array([SAW_X - r * math.cos(th), -r * math.sin(th), TOP + 0.086 + PAD_TIP_PAST_TCP])
    rot = grip_rot(th)
    yield from _go(R, at + [0, 0, 0.065], rot, 2.0, FINGER_CLOSED)
    yield from _go(R, at + [0, 0, 0.03], rot, 1.2)
    yield from R.trim_to_measured(at + [0, 0, 0.03], max_duration=1.2)
    fix = R.pos - at - [0, 0, 0.03]
    for k in range(1, 100):  # 2 mm lower each tenth of a second; the arm gives about 2.3 mm per N
        if not ctx.offline and _joints(ctx)["lock"] <= math.radians(-34):
            break
        yield from _go(R, at + fix + [0, 0, 0.03 - 0.002 * k], rot, 0.1)
    yield from R.hold(0.2)
    yield from _go(R, at + fix + [0, 0, 0.065], rot, 1.5)


def _debug(objects, f):
    n = _debug.n = getattr(_debug, "n", 0) + 1
    if n % 10 == 0:
        s = objects["miter_saw"]
        print(f"MITERDBG {n} table {math.degrees(s.joints['table']):.2f} release {math.degrees(s.joints['release']):.2f} lock "
              f"{math.degrees(s.joints['lock']):.1f} pin {_M.get('pin')} clamp {_M.get('clamped')} gap {f['gap'] * 1000:.0f}mm "
              f"still {f['still']} shift {f['shift'] * 1000:.1f}mm score {level(f)}", flush=True)
