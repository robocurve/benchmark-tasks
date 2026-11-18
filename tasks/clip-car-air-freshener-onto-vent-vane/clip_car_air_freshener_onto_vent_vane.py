"""Clip a car air freshener onto a dashboard vent vane: push its sprung two-prong fork over one pivoting vane, home
along the vane's plane, leaving the vent's aim unchanged.

A dashboard section stands on the table with a rectangular centre air vent in its front face: four horizontal vanes,
each a 2 mm blade on pins with a friction detent, so a vane stays at whatever angle it is pushed to. A Febreze CAR
vent clip (clip already clicked open) stands on its flat bottom edge on a rubber console mat, its fork pointing at the
dashboard. The fork is two prongs 1.6 mm apart (one sprung, about 1.5 N to spread them over a 2 mm vane) flared to a
4.5 mm mouth: it has to meet one vane's front edge within about a millimetre and be pushed straight along that vane's
plane until the edge reaches the fork's root. Metres; x forward from the arm bases, y left, z up.
"""
import math
import pathlib
import sys
from collections import deque

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
from yam_parts import Part, box  # noqa: E402
from yam_joints import Hinge, Link, Mechanism, Slide  # noqa: E402
from yam_mesh import MeshMechanism, prism  # noqa: E402
from yam_arena.constants import FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TEST_GRIPPERS = None
TEST_HOLD = None  # offline layouts: the hold check's verdict (None: passed when the clip is engaged)
DIAG = False  # GPU diagnostic copies print FRESH_DIAG lines
DEMO_WAY = "along"  # "along": pushed along the vane's plane; "tilted": wrong way, fork pitched 30 deg to the vane
DEMO_WAY_BY_EPISODE = {}  # probe copies only: {episode: way}
DEMO_PACE = 1.0


def _run_context():
    context = {"env": None, "t": None, "step": 0, "episode": 0}
    try:
        from yam_arena import inspect_embodiment as _ie
    except ImportError:
        return context
    cls = _ie.YamArenaEmbodiment
    if hasattr(cls, "_rc_run"):
        return cls._rc_run
    original_step, original_reset = cls._task_step, cls.reset

    def task_step(self, *args, **kwargs):
        context["env"], context["t"] = self._env, self._t
        context["step"] += 1
        return original_step(self, *args, **kwargs)

    def reset(self, *args, **kwargs):
        result = original_reset(self, *args, **kwargs)
        context["episode"] += 1
        context["t"] = 0.0
        context["env"] = self._env
        return result

    task_step._rc_wrapped = reset._rc_wrapped = True
    cls._rc_run = context
    cls._task_step, cls.reset = task_step, reset
    return context


_RUN = _run_context()
_MEMORY = {"episode": None, "timers": {}}


def held_for(key, cond):
    """Seconds a condition has held over consecutive control steps (offline: inf when true)."""
    if _MEMORY["episode"] != _RUN["episode"]:
        _MEMORY.update(episode=_RUN["episode"], timers={})
    t, step = _RUN["t"], _RUN["step"]
    if t is None:
        return math.inf if cond else 0.0
    timers = _MEMORY["timers"]
    previous_step, start = timers.get(key, (None, None))
    if not cond:
        timers[key] = (step, None)
        return 0.0
    if start is None or step not in (previous_step, previous_step + 1) or t < start:
        start = t
    timers[key] = (step, start)
    return float(t - start)


TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S, SCORE_OVER = 1.0, "end"
RUN_LIMIT_STEPS = 1200  # 60 s at 20 Hz
MM = 0.001

DASH_GREY = (0.22, 0.22, 0.24)
VANE_BLACK = (0.06, 0.06, 0.07)
DUCT_BLACK = (0.03, 0.03, 0.03)
MAT_BLACK = (0.10, 0.10, 0.11)
CLIP_BLUE = (0.55, 0.70, 0.92)  # the Linen & Sky clip's pale blue
PRONG_GREY = (0.30, 0.31, 0.34)

# ---------------------------------------------------------------- the dashboard and its centre vent (fixed)
# Vent frame: origin at the vent's centre on the dashboard's front face; x into the dashboard, y left, z up.
# Sizes are estimates for a standard rectangular centre register (no maker publishes them); vent-mount makers'
# published limits for real louvres: blades under 3.5-4.2 mm thick, 10-34 mm deep, at least 8 mm between blades.
FACE_X, VENT_Y, VENT_Z = 0.240, -0.120, 0.130  # front face x; the vent's centre (world y, height above the table)
DASH_DEPTH, DASH_H, DASH_Y = 0.150, 0.190, (-0.300, 0.300)  # 60 x 15 x 19 cm section, front face at x = 0.240
OPEN_W, OPEN_H = 0.110, 0.070  # the vent opening in the face (y x z)
DUCT_DEPTH = 0.045  # the duct behind the opening
VANE_L, VANE_DEPTH, VANE_T = 0.108, 0.017, 0.002  # each horizontal vane: length (y), depth (x), thickness
PIN_BACK = 0.0085  # the vane's pins are this far behind its front edge (mid-depth)
VANE_Z = (-0.0225, -0.0075, 0.0075, 0.0225)  # vane centre lines relative to the vent's centre (15 mm pitch)
VANE_NAMES = ("vane1", "vane2", "vane3", "vane4")  # bottom to top
VANE_MASS = 3.0e-3  # ABS blade 108 x 17 x 2 mm (estimate)
VANE_LIMIT_DEG = 30.0
VANE_DETENT, VANE_DETENT_MOVING = 0.030, 0.025  # N m about the pins (estimate; no maker figure)
VANE_DAMPING = 2.0e-4
VANE_ARMATURE = 2.0e-6  # kg m^2: numerical inertia for the solver (the blade's own is ~7e-8)
REAR_X, REAR_DEPTH, REAR_T = 0.030, 0.008, 0.0015  # vertical vanes 30 mm behind the face, out of the fork's reach
REAR_Y = tuple(-0.045 + 0.015 * k for k in range(7))
ABS_FRICTION = (0.40, 0.35)


def make_vent():
    yl, yr = DASH_Y[0] - VENT_Y, DASH_Y[1] - VENT_Y
    z0, z1 = -VENT_Z, DASH_H - VENT_Z
    ow, oh = OPEN_W / 2, OPEN_H / 2

    def block(x0, x1, ya, yb, za, zb, color=DASH_GREY):
        return box(((x0 + x1) / 2, (ya + yb) / 2, (za + zb) / 2), (x1 - x0, yb - ya, zb - za), color)

    shapes = [
        block(0.0, DASH_DEPTH, yl, yr, z0, -oh),  # below the vent
        block(0.0, DASH_DEPTH, yl, yr, oh, z1),  # above it
        block(0.0, DASH_DEPTH, ow, yr, -oh, oh),  # to its left
        block(0.0, DASH_DEPTH, yl, -ow, -oh, oh),  # to its right
        block(DUCT_DEPTH, DASH_DEPTH, -ow, ow, -oh, oh, DUCT_BLACK),  # the duct's back
    ]
    shapes += [box((REAR_X, y, 0.0), (REAR_DEPTH, REAR_T, OPEN_H - 0.002), VANE_BLACK) for y in REAR_Y]
    links = []
    for name, z in zip(VANE_NAMES, VANE_Z):
        blade = box((VANE_DEPTH / 2, 0.0, z), (VANE_DEPTH, VANE_L, VANE_T), VANE_BLACK)
        links.append(Link(name, [blade], Hinge("y", (PIN_BACK, 0.0, z), -VANE_LIMIT_DEG, VANE_LIMIT_DEG,
                                                damping=VANE_DAMPING, armature=VANE_ARMATURE,
                                                static_friction=VANE_DETENT, dynamic_friction=VANE_DETENT_MOVING),
                          VANE_MASS, color=VANE_BLACK, friction=ABS_FRICTION))
    return Mechanism("vent", (FACE_X, VENT_Y, TOP + VENT_Z), shapes, links, mass=None, color=DASH_GREY,
                     friction=ABS_FRICTION, contact_offset=0.001)


# ---------------------------------------------------------------- the air freshener: Febreze CAR vent clip
# Clip frame: origin at the body's centre; x along the fork (out of the back), y across, z up (the prongs' gap axis).
# Published: 2 mL scented oil, clip clicked open before use, scent dial on the top edge. Everything below is an
# estimate (no maker figure): body 41 x 38 x 16 mm, 7 g; prongs 20 mm long, 7 mm wide, 1.4 mm thick, 1.6 mm apart,
# flared over their last 3 mm to a 4.5 mm mouth; spreading them to 2.0 mm takes 1.5 N.
BODY_W, BODY_H, BODY_T, BODY_R = 0.041, 0.038, 0.016, 0.008
BACK_X = BODY_T / 2  # the back face, where the fork's root is
PRONG_L, PRONG_W, PRONG_T = 0.020, 0.007, 0.0014
GAP = 0.0016  # between the prongs' inner faces at rest
FLARE_L, FLARE_OUT = 0.003, 0.00145  # each tip turns out 1.45 mm over its last 3 mm: mouth 4.5 mm
TIP_X = BACK_X + PRONG_L
CLIP_MASS = 7.0e-3
PRONG_MASS = 0.3e-3  # the sprung (upper) prong
PRONG_OPEN_MAX = 0.0008  # the sprung prong opens at most 0.8 mm (gap 2.4 mm): stiff plastic
PRONG_PRELOAD, PRONG_RATE = 1.0, 1250.0  # N at rest on its stop, N/m: 1.5 N at 0.4 mm (a 2.0 mm vane)
PRONG_ARMATURE = 2.0e-2  # kg along the slide: numerical inertia for the solver (the prong is 0.3 g)
PRONG_DAMPING = 5.0  # N s/m: about half of critical with the armature
CLIP_FRICTION = (0.50, 0.45)  # polypropylene (estimate)
VANE_MID = 0.0002  # a gripped vane's centre plane sits 0.2 mm above the clip's (on the fixed lower prong)


def _rounded_rect(half_w, half_h, r, n=7):
    pts = []
    for cx, cy, a0 in ((half_w - r, half_h - r, 0.0), (-half_w + r, half_h - r, 90.0),
                       (-half_w + r, -half_h + r, 180.0), (half_w - r, -half_h + r, 270.0)):
        for k in range(n):
            a = math.radians(a0 + 90.0 * k / (n - 1))
            pts.append((cx + r * math.cos(a), cy + r * math.sin(a)))
    return pts


def _prong(name, sign, color):
    """One prong as ONE convex piece: inner face flat from the root, then turned out over the last 3 mm (sign +1 up,
    -1 down); the outer face runs straight from the root to the flared tip. One piece, so the inner face has no seam
    where a vane's corner could catch (two pieces jammed the vane at the flare's end in the GPU probe)."""
    zi0, zi1 = sign * GAP / 2, sign * (GAP / 2 + FLARE_OUT)
    zo0, zo1 = zi0 + sign * PRONG_T, zi1 + sign * PRONG_T
    x0, x1 = TIP_X - FLARE_L, TIP_X
    xz = [(BACK_X, zi0), (x0, zi0), (x1, zi1), (x1, zo1), (BACK_X, zo0)]
    outline = [(x, -z) for x, z in xz]  # prism about y: outline (u, v) -> (x, z) = (u, -v)
    proxy = (box(((BACK_X + x1) / 2, 0.0, (zi0 + zo1) / 2), (PRONG_L, PRONG_W, abs(zo1 - zi0))),)
    return prism(outline, PRONG_W, axis="y", name=name, collision="convex", color=color, proxy=proxy)


def make_clip(pos):
    # body: a rounded rectangle 41 wide (y) x 38 tall (z), 16 thick (x); prism about x maps (u, v) -> (y, z) = (v, -u)
    outline = [(-z, y) for y, z in _rounded_rect(BODY_W / 2, BODY_H / 2, BODY_R)]
    # offline stand-in only (the GPU uses the mesh): it reaches 4 mm in front of the body because the offline replay
    # tests a grasp top-down (tool point inside the gripped footprint) and this pitched pinch's tool point sits 3 mm
    # in front of the body
    body = prism(outline, BODY_T, axis="x", name="body", collision="convex", color=CLIP_BLUE,
                 proxy=(box((-0.002, 0.0, 0.0), (BODY_T + 0.004, BODY_W, BODY_H)),))
    # the prongs' roots as small boxes set 0.2 mm inside each prong's piece (they carry the link; never touched)
    root, xm = 0.004, BACK_X + 0.002
    lower = box((xm, 0.0, -(GAP / 2 + PRONG_T / 2)), (root, PRONG_W - 0.0004, PRONG_T - 0.0004), PRONG_GREY)
    upper = box((xm, 0.0, GAP / 2 + PRONG_T / 2), (root, PRONG_W - 0.0004, PRONG_T - 0.0004), PRONG_GREY)
    prong = Link("prong", [upper], Slide("z", (xm, 0.0, GAP / 2 + PRONG_T / 2), 0.0, PRONG_OPEN_MAX,
                                         damping=PRONG_DAMPING, spring=PRONG_RATE, rest=-PRONG_PRELOAD / PRONG_RATE,
                                         armature=PRONG_ARMATURE), PRONG_MASS, color=PRONG_GREY)
    return MeshMechanism("freshener", pos, [lower], [prong], mass=CLIP_MASS - PRONG_MASS, color=CLIP_BLUE,
                         friction=CLIP_FRICTION, contact_offset=0.001,
                         meshes={"base": (body, _prong("lower_prong", -1.0, PRONG_GREY)),
                                 "prong": (_prong("upper_prong", 1.0, PRONG_GREY),)})


# ---------------------------------------------------------------- the console mat (fixed) and the starting place
MAT_XY, MAT_SIZE, MAT_T = (0.165, -0.060), (0.150, 0.120), 0.003
CLIP_START = np.array([0.170, -0.060, TOP + MAT_T + BODY_H / 2])  # standing on its flat bottom edge, fork toward +x
MAT = Part("mat", (MAT_XY[0], MAT_XY[1], TOP), [box((0.0, 0.0, MAT_T / 2), (*MAT_SIZE, MAT_T), MAT_BLACK)],
           mass=None, color=MAT_BLACK, friction=(0.80, 0.70))
VENT = make_vent()
CLIP = make_clip(tuple(float(v) for v in CLIP_START))
OBJECTS = [VENT, MAT, CLIP]
PRESSED_PARTS = ("vent",)  # the fork presses a vane; replay can't move a vane

# ---------------------------------------------------------------- scoring
EDGE_SLACK = 0.0008
ENGAGED_GAP = 0.012  # engaged: at least 8 mm of the 20 mm prongs over the vane (root gap 12 mm or less)
HOME_GAP = 0.002  # home: the vane's front edge within 2 mm of the fork's root
AIM_OK, AIM_MINOR = 5.0, 15.0  # the vent's aim (vane angle from where it started, degrees): ok / minor / 0 beyond
SQUARE_OK, SQUARE_MINOR = 15.0, 30.0  # the clip's face to the dashboard's face (degrees): ok / minor / major
RELEASE_CLEAR = 0.030
STILL_S, STILL_MOVE, STILL_DEG = 1.0, 0.001, 1.0
HOLD_N, HOLD_RAMP_S, HOLD_S, HOLD_MOVE = 1.0, 0.5, 2.0, 0.002  # the hold check: 1 N down for 2 s, moves < 2 mm


def vent_point(vent, link, p_vent):
    return np.asarray(vent.link_to_world(link, p_vent), float)


def vane_edge_in_clip(vent, clip, k):
    """Vane k's front edge in the clip's frame: the point where the edge line crosses the clip's x-z plane, its
    distance along the edge from the vane's middle, and the edge's direction."""
    name, z = VANE_NAMES[k], VANE_Z[k]
    o, R = np.asarray(clip.origin, float), clip.rotation()
    a = R.T @ (vent_point(vent, name, (0.0, 0.0, z)) - o)
    b = R.T @ (vent_point(vent, name, (0.0, 0.01, z)) - o)
    d = (b - a) / 0.01
    if abs(d[1]) < 0.5:
        return None, math.inf, d
    s = -a[1] / d[1]
    return a + s * d, abs(s), d


def engagement(vent, clip):
    """The vane the fork is on (the one with the smallest root gap among those between the prongs), or None."""
    q = float(clip.joints.get("prong", 0.0))
    best = None
    for k in range(len(VANE_NAMES)):
        p, along, _ = vane_edge_in_clip(vent, clip, k)
        if p is None or along > VANE_L / 2 - PRONG_W / 2:
            continue
        # the vane's centre plane at its front edge between the prongs (its faces on their inner faces, 0.8 mm slack)
        between = -GAP / 2 + VANE_T / 2 - EDGE_SLACK <= p[2] <= GAP / 2 + q - VANE_T / 2 + EDGE_SLACK
        gap = float(p[0] - BACK_X)
        if between and -0.002 <= gap <= ENGAGED_GAP and (best is None or gap < best[1]):
            best = (k, gap)
    return best


def vent_aim_deg(vent):
    return max(abs(math.degrees(float(vent.joints[n]))) for n in VANE_NAMES)


def square_deg(vent, clip):
    a, b = clip.rotation()[:, 0], vent.rotation()[:, 0]
    return math.degrees(math.acos(float(np.clip(a @ b, -1.0, 1.0))))


def _clear(state_low, state_high, clear):
    hands = grippers()
    if hands is None:
        return True
    low, high = np.asarray(state_low, float), np.asarray(state_high, float)
    return all(float(np.linalg.norm(np.maximum(np.maximum(low - tcp, tcp - high), 0.0))) >= clear
               for tcp, _ in hands.values())


def vent_box():
    lo = np.array([FACE_X, VENT_Y - OPEN_W / 2, TOP + VENT_Z - OPEN_H / 2])
    return lo, lo + np.array([DUCT_DEPTH, OPEN_W, OPEN_H])


def released(clip):
    return _clear(clip.low, clip.high, RELEASE_CLEAR) and _clear(*vent_box(), RELEASE_CLEAR)


_WINDOW = {"episode": None, "hist": deque()}


def _key_points(clip):
    return np.array([clip.to_world((0.0, 0.0, 0.0)), clip.to_world((TIP_X, 0.0, 0.0)),
                     clip.to_world((0.0, BODY_W / 2, 0.0))])


def still(vent, clip):
    """Over the last second three points on the clip moved under 1 mm and no vane turned more than 1 degree
    (positions, not solver velocities). Offline (single frame): velocities."""
    t = _RUN["t"]
    if t is None:
        return bool(np.linalg.norm(clip.lin_vel) < 0.01)
    hist = _WINDOW["hist"]
    if _WINDOW["episode"] != _RUN["episode"] or (hist and t < hist[-1][0]):
        hist.clear()
        _WINDOW["episode"] = _RUN["episode"]
    now = (_key_points(clip), np.array([float(vent.joints[n]) for n in VANE_NAMES]))
    if not hist or t > hist[-1][0] + 1e-9:
        hist.append((t, now))
    while len(hist) > 1 and hist[1][0] <= t - STILL_S + 1e-9:
        hist.popleft()
    if t - hist[0][0] < STILL_S - 0.051:
        return False
    for _, old in hist:
        if (np.max(np.linalg.norm(now[0] - old[0], axis=1)) > STILL_MOVE
                or np.max(np.abs(np.degrees(now[1] - old[1]))) > STILL_DEG):
            return False
    return True


# The hold check: once the clip is on a vane, let go and still for 1 s, the scorer pulls the clip straight down with
# 1 N (a 100 g weight on a thread, in the real trial) for 2 s, ramped in over 0.5 s, then lets go of it again.
_HOLD = {"episode": None, "phase": 0, "t0": None, "ref": None, "moved": 0.0, "passed": None, "api": None,
         "failed": False}


def _apply_down(force_n):
    """A downward force on the clip's body at its centre (as a force at the base's centre of mass plus its moment)."""
    env = _RUN["env"]
    if env is None or _HOLD["failed"]:
        return
    try:
        import torch
        art = env.unwrapped.scene["freshener"]
        dev = art.device
        ids = art.find_bodies("base")[0]
        origin = np.asarray(env.unwrapped.scene.env_origins[0].detach().cpu().numpy(), float)
        com = None
        for name in ("body_com_pos_w", "body_link_com_pos_w"):
            try:
                v = getattr(art.data, name)
                v = getattr(v, "torch", v)
                com = np.asarray(v[0][ids[0]].detach().cpu().numpy(), float) - origin
                break
            except Exception:  # noqa: BLE001
                continue
        pose = art.data.root_pos_w
        pose = getattr(pose, "torch", pose)
        centre = np.asarray(pose[0].detach().cpu().numpy(), float) - origin
        if com is None:
            com = centre
        f = np.array([0.0, 0.0, -float(force_n)])
        tq = np.cross(centre - com, f)
        F = torch.tensor(f, dtype=torch.float32, device=dev).view(1, 1, 3)
        T = torch.tensor(tq, dtype=torch.float32, device=dev).view(1, 1, 3)
    except Exception as e:  # noqa: BLE001
        _HOLD["failed"] = True
        print(f"FRESH_HOLD_FAILED setup {e!r}", flush=True)
        return
    comp = getattr(art, "permanent_wrench_composer", None)
    calls = []
    if comp is not None:
        calls += [("set_forces_and_torques(body_ids, global)",
                   lambda: comp.set_forces_and_torques(forces=F, torques=T, body_ids=ids, is_global=True)),
                  ("set_forces_and_torques_index(body_ids, global)",
                   lambda: comp.set_forces_and_torques_index(forces=F, torques=T, body_ids=ids, is_global=True))]
    calls.append(("set_external_force_and_torque(body_ids, global)",
                  lambda: art.set_external_force_and_torque(F, T, body_ids=ids, is_global=True)))
    order = [c for c in calls if c[0] == _HOLD["api"]] or calls
    errors = []
    for name, call in order:
        try:
            call()
            if _HOLD["api"] != name:
                _HOLD["api"] = name
                print(f"FRESH_HOLD api={name}", flush=True)
            return
        except Exception as e:  # noqa: BLE001
            errors.append(f"{name}: {e!r}")
    _HOLD["failed"] = True
    print(f"FRESH_HOLD_FAILED {errors}", flush=True)


def hold_check(clip, ready):
    """None until the check has run; then whether the clip moved less than 2 mm under the 1 N pull."""
    if _RUN["t"] is None:
        if not ready:
            return None
        return bool(TEST_HOLD) if TEST_HOLD is not None else True
    if _HOLD["episode"] != _RUN["episode"]:
        if _HOLD["phase"] == 1:
            _apply_down(0.0)
        _HOLD.update(episode=_RUN["episode"], phase=0, t0=None, ref=None, moved=0.0, passed=None)
    t = float(_RUN["t"])
    if _HOLD["phase"] == 0:
        if not ready:
            return None
        _HOLD.update(phase=1, t0=t, ref=_key_points(clip))
        print(f"FRESH_HOLD start t={t:.2f}", flush=True)
    if _HOLD["phase"] == 1:
        dt = t - _HOLD["t0"]
        _apply_down(HOLD_N * min(1.0, dt / HOLD_RAMP_S))
        moved = float(np.max(np.linalg.norm(_key_points(clip) - _HOLD["ref"], axis=1)))
        _HOLD["moved"] = max(_HOLD["moved"], moved)
        if dt >= HOLD_RAMP_S + HOLD_S:
            _apply_down(0.0)
            _HOLD.update(phase=2, passed=bool(_HOLD["moved"] < HOLD_MOVE))
            print(f"FRESH_HOLD end t={t:.2f} moved_mm={_HOLD['moved'] * 1000:.2f} passed={_HOLD['passed']}",
                  flush=True)
        return None if _HOLD["phase"] == 1 else _HOLD["passed"]
    return _HOLD["passed"]


def terms(objects):
    vent, clip = objects["vent"], objects["freshener"]
    on = engagement(vent, clip)
    return {"on": on, "aim": vent_aim_deg(vent), "square": square_deg(vent, clip), "released": released(clip),
            "home": bool(on is not None and on[1] <= HOME_GAP)}


_SCORE = {"token": None, "value": 0.0}


def score(objects):
    token = (_RUN["episode"], _RUN["step"]) if _RUN["t"] is not None else None
    if token is not None and token == _SCORE["token"]:
        return _SCORE["value"]
    value = _score(objects)
    if token is not None:
        _SCORE.update(token=token, value=value)
    return value


def _score(objects):
    vent, clip = objects["vent"], objects["freshener"]
    tm = terms(objects)
    quiet = still(vent, clip)
    ready = tm["on"] is not None and tm["released"] and quiet
    ready = ready and held_for("on a vane, let go, still", ready) + 1e-9 >= STILL_S
    testing = _HOLD["episode"] == _RUN["episode"] and _HOLD["phase"] >= 1 and _RUN["t"] is not None
    passed = hold_check(clip, ready or testing)
    if DIAG and _RUN["t"] is not None:
        _diag(vent, clip, tm, quiet, passed)
    if not passed or tm["on"] is None or not tm["released"]:
        return 0.0
    return grade(tm["home"], tm["aim"], tm["square"])


def grade(home, aim, square):
    """The score of a clip on a vane, let go and through the hold check."""
    if aim > AIM_MINOR:
        return 0.0  # the vent knocked off its aim: the one thing the instruction says not to do
    if square > SQUARE_MINOR:
        return 0.25
    minors = int(aim > AIM_OK) + int(square > SQUARE_OK)
    if home:
        return (1.0, 0.75, 0.5)[minors]
    return 0.5 if minors == 0 else 0.25


def success(objects):
    return score(objects) >= 1.0


# ---------------------------------------------------------------- diagnostics (GPU copies only)
_DIAG = {"step": -1}


def grippers():
    if TEST_GRIPPERS is not None:
        return TEST_GRIPPERS
    env = _RUN["env"]
    if env is None:
        return None
    from yam_arena import ik, station
    from yam_arena.constants import ARM_JOINTS, FINGER_JOINTS
    out = {}
    for arm in ("left", "right"):
        art = env.unwrapped.scene[arm + "_robot"]
        arm_ids = art.find_joints(ARM_JOINTS, preserve_order=True)[0]
        finger_ids = art.find_joints(FINGER_JOINTS, preserve_order=True)[0]
        joints = art.data.joint_pos
        q = getattr(joints, "torch", joints)[0]
        if hasattr(q, "detach"):
            q = q.detach().cpu().numpy()
        q = np.asarray(q)
        tcp = ik.tcp_pose(q[arm_ids], station.BASES[arm])[0]
        out[arm] = (tcp, float(np.mean(q[finger_ids])))
    return out


def _hand_force():
    """The right hand's force on what it holds (N, world x, y, z) from the joints' PD error. Diagnostic only."""
    env = _RUN["env"]
    if env is None:
        return "-"
    try:
        from yam_arena import ik, station
        from yam_arena.constants import ARM_JOINTS, SHOULDER, WRIST
        k = np.array([SHOULDER["stiffness"]] * 3 + [WRIST["stiffness"]] * 3)
        art = env.unwrapped.scene["right_robot"]
        ids = art.find_joints(ARM_JOINTS, preserve_order=True)[0]
        q = art.data.joint_pos[0][ids].detach().cpu().numpy().astype(float)
        qt = art.data.joint_pos_target[0][ids].detach().cpu().numpy().astype(float)
        tau = k * (qt - q)
        p0, r0 = ik.tcp_pose(q, station.BASES["right"])
        jac = np.zeros((6, 6))
        for j in range(6):
            dq = q.copy()
            dq[j] += 1e-5
            p1, r1 = ik.tcp_pose(dq, station.BASES["right"])
            ax, a = _axis_angle(r1 @ r0.T)
            jac[:3, j] = (p1 - p0) / 1e-5
            jac[3:, j] = ax * a / 1e-5
        w = np.linalg.lstsq(jac.T, tau, rcond=None)[0]
        return f"({w[0]:.1f},{w[1]:.1f},{w[2]:.1f})"
    except Exception as e:  # noqa: BLE001
        return f"err:{type(e).__name__}"


def _diag(vent, clip, tm, quiet, passed):
    step = _RUN["step"]
    if step == _DIAG["step"] or step % 4:
        return
    _DIAG["step"] = step
    o = np.asarray(clip.origin, float)
    pitch = math.degrees(math.asin(float(np.clip(-clip.rotation()[2, 0], -1, 1))))
    edges = []
    for k in range(len(VANE_NAMES)):
        p, along, _ = vane_edge_in_clip(vent, clip, k)
        if p is not None and abs(p[2]) < 0.006 and p[0] < TIP_X + 0.010:
            edges.append(f"v{k + 1}(gap={(p[0] - BACK_X) * 1000:.1f},z={p[2] * 1000:.2f},s={along * 1000:.0f})")
    hands = grippers() or {}
    h = " ".join(f"{a[0]}=({','.join(f'{v * 1000:.1f}' for v in hands[a][0])};{hands[a][1] * 1000:.1f})"
                 for a in hands)
    angles = ",".join(f"{math.degrees(float(vent.joints[n])):.2f}" for n in VANE_NAMES)
    print(f"FRESH_DIAG t={_RUN['t']:.2f} clip=({o[0] * 1000:.1f},{o[1] * 1000:.1f},{(o[2] - TOP) * 1000:.1f};"
          f"pitch {pitch:.2f};tilt {clip.tilt_deg():.2f}) prong={float(clip.joints.get('prong', 0)) * 1000:.2f} "
          f"edges {' '.join(edges) or '-'} on={tm['on']} aim={tm['aim']:.2f} vanes=({angles}) sq={tm['square']:.1f} "
          f"free={int(tm['released'])} still={int(quiet)} hold={passed} v={np.linalg.norm(clip.lin_vel) * 1000:.0f} "
          f"{h} F={_hand_force()}", flush=True)


# ---------------------------------------------------------------- demo (right arm; the left stays at its ready pose)
# The right hand points straight at the dashboard, pitched 30 degrees down, and pinches the clip's two flat sides; the
# clip sits in the pinch turned 30 degrees, so its fork stays level. It reads where the standing clip is, comes down
# behind it, pinches, lifts it and carries it in front of the third vane, fork tips 8 mm short of the vane's edge.
# There it reads the clip (twice) and corrects, so the fork's mouth is centred on the vane's front edge and the fork is
# level; then it pushes straight along the vane's plane in eight steps, re-reading the clip's height and pitch at
# each, 5 mm past home (the vane's edge stops at the fork's root), lets go, backs away and parks.
# The demo reads the simulator's object poses (as scripted demos may); it never moves an object.
PITCH = math.radians(30.0)  # the hand's pitch below level, and so the clip's turn in the pinch
TILT_WRONG = math.radians(30.0)  # wrong way: the fork pitched this much nose-down to the vane's plane
PAD_TIP = 0.0128  # the pad tips are 12.8 mm past the TCP
PAD_HALF_W = 0.0077
PAD_CLEAR = 0.004  # the pads' leading corner stays this far short of the clip's back (and so of the vent's face)
PAD_TIP_X = BACK_X - PAD_CLEAR - PAD_HALF_W * math.sin(PITCH)  # pad tip centre in the clip frame
PAD_TIP_Z = -0.003
GRIP_OPEN = FINGER_OPEN  # fully open: pads 94 mm apart round the 41 mm body
GRIP_PINCH = -0.0115  # pads commanded 23 mm apart on the 41 mm body: 9 mm short each, about 45 N (the cap is 50):
# a firm hold, so the clip does not turn in the fingers when pushed (a 10 N pinch let it turn 35 degrees)
TARGET_VANE = 2  # the third vane from the bottom (index)
PRE_TIPS = 0.008  # the fork tips this far short of the vane's edge before the push
OVERDRIVE = 0.005  # the push goes this far past home (the vane's edge stops at the root)
PUSH_STEPS, PUSH_STEP_S = 8, 0.35
PARK = {"right": np.array([0.150, -0.250, TOP + 0.220])}
CLEAR_POINT = np.array([0.170, -0.150, TOP + 0.200])  # in front of the dashboard, above its top
FINAL_HOLD_S = 5.0


def _rot_y(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def _T(R, p):
    m = np.eye(4)
    m[:3, :3], m[:3, 3] = R, p
    return m


def _inv(m):
    R, p = m[:3, :3], m[:3, 3]
    return _T(R.T, -R.T @ p)


def _hand_rotation(pitch):
    """Hand pointing +x pitched `pitch` down; fingers close along y (pad width in the x-z plane)."""
    z = np.array([math.cos(pitch), 0.0, -math.sin(pitch)])
    x = np.array([0.0, -1.0, 0.0])
    return np.column_stack([x, np.cross(z, x), z])


def _hand_in_clip():
    """The hand's pose in the clip's frame (clip level, fork +x): pad tip centre at (PAD_TIP_X, 0, PAD_TIP_Z)."""
    R = _hand_rotation(PITCH)
    tip = np.array([PAD_TIP_X, 0.0, PAD_TIP_Z])
    return _T(R, tip - PAD_TIP * R[:, 2])


HAND_IN_CLIP = _hand_in_clip()


def _axis_angle(rot):
    c = (np.trace(rot) - 1.0) / 2.0
    a = math.acos(float(np.clip(c, -1.0, 1.0)))
    if a < 1e-9:
        return np.array([0.0, 0.0, 1.0]), 0.0
    axis = np.array([rot[2, 1] - rot[1, 2], rot[0, 2] - rot[2, 0], rot[1, 0] - rot[0, 1]]) / (2 * math.sin(a))
    return axis, a


def _turn(axis, ang):
    k = np.asarray(axis, float) / np.linalg.norm(axis)
    kx = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + math.sin(ang) * kx + (1 - math.cos(ang)) * (kx @ kx)


def demo_way():
    return DEMO_WAY_BY_EPISODE.get(_RUN["episode"], DEMO_WAY)


def edge_point(k=TARGET_VANE):
    """The middle of vane k's front edge (world) with the vent as built (vanes level)."""
    return np.array([FACE_X, VENT_Y, TOP + VENT_Z + VANE_Z[k]])


def clip_at_vane(depth, way=None):
    """Planned clip pose with its fork `depth` metres onto the target vane past the tips meeting the edge (0: the
    tips at the edge; PRONG_L: home), the vane's centre plane VANE_MID above the clip's."""
    way = way or demo_way()
    tilt = TILT_WRONG if way == "tilted" else 0.0
    R = _rot_y(tilt)  # positive: fork nose-down
    mouth = np.array([TIP_X, 0.0, VANE_MID])  # the mouth's centre in the clip frame
    u = R[:, 0]
    target = edge_point() + u * depth  # the mouth moves along the fork's axis from the vane's edge
    return _T(R, target - R @ mouth)


def clip_start():
    return _T(np.eye(3), CLIP_START)


_READS = {"episode": None, "values": {}}
_PLAN = {"keys": [], "planned": {}, "modes": {}}
# (follow or move, limit on the shift, limit on the turn in degrees, fresh, level-only)
READ_MODES = {
    "follow": ("follow", 0.025, 15.0, True, False),  # the hand goes to where the standing clip is
    "move": ("move", 0.015, 10.0, True, False),  # bring the held clip to its planned pose (from no correction)
    "again": ("move", 0.008, 5.0, False, False),  # once more, chained
    "level": ("move", 0.0008, 1.5, False, True),  # in the push: height and pitch only, small steps
}
LEVEL_TOTAL, LEVEL_TOTAL_DEG = 0.003, 4.0  # the push's re-aims add up to at most this beyond the line-up


def _clip_now():
    env = _RUN["env"]
    if env is None:
        return None
    try:
        from yam_arena import scene as arena_scene
        st = arena_scene.object_states(env, ["freshener"])["freshener"]
        return _T(np.asarray(st.rotation(), float), np.asarray(st.origin, float))
    except Exception as e:  # noqa: BLE001
        print(f"FRESH_READ failed: {e!r}", flush=True)
        return None


def _read(key):
    """World transform applied to the planned hand poses from this read on (strawlid-style chained reads)."""
    if _READS["episode"] != _RUN["episode"]:
        _READS["episode"], _READS["values"] = _RUN["episode"], {}
    values = _READS["values"]
    if key in values:
        return values[key]
    keys = _PLAN["keys"]
    i = keys.index(key)
    kind, max_shift, max_deg, fresh, level = READ_MODES[_PLAN["modes"][key]]
    prior = np.eye(4) if fresh or i == 0 else _read(keys[i - 1])
    planned = _PLAN["planned"][key]
    planned = planned() if callable(planned) else planned
    actual = _clip_now()
    if actual is None:
        values[key] = prior
        return prior
    if level and demo_way() == "tilted":  # the careless push does not re-aim
        values[key] = prior
        return prior
    if kind == "follow":
        src, dst = planned, actual
    else:
        src, dst = actual, planned
    rot = dst[:3, :3] @ src[:3, :3].T
    if level:  # keep only the turn about y (pitch)
        ax, ang = _axis_angle(rot)
        ang_y = ang * ax[1]
        rot = _rot_y(ang_y)
    ax, ang = _axis_angle(rot)
    if ang > math.radians(max_deg):
        rot = _turn(ax, math.radians(max_deg))
    c_src = src[:3, 3]
    move = dst[:3, 3] - c_src
    if level:
        move[:2] = 0.0
    n = float(np.linalg.norm(move))
    if n > max_shift:
        move *= max_shift / n
    d = _T(rot, c_src + move - rot @ c_src)  # turn about the clip's centre, then shift
    values[key] = d @ prior
    if level and "line" in values:  # bound the push's re-aims in all
        base = values["line"]
        rel = values[key] @ _inv(base)
        n = abs(float(rel[2, 3]))
        if n > LEVEL_TOTAL:
            rel[2, 3] *= LEVEL_TOTAL / n
        ax, ang = _axis_angle(rel[:3, :3])
        if ang > math.radians(LEVEL_TOTAL_DEG):
            rel[:3, :3] = _turn(ax, math.radians(LEVEL_TOTAL_DEG))
        values[key] = rel @ base
    err = planned[:3, 3] - actual[:3, 3]
    ang_err = math.degrees(_axis_angle(planned[:3, :3] @ actual[:3, :3].T)[1])
    print(f"FRESH_READ {key} t={_RUN['t']:.2f} err_mm=({err[0] * 1000:.2f},{err[1] * 1000:.2f},{err[2] * 1000:.2f}) "
          f"turn_deg={ang_err:.2f} prong_mm={_prong_now():.2f}", flush=True)
    return values[key]


def _prong_now():
    env = _RUN["env"]
    if env is None:
        return float("nan")
    try:
        from yam_arena import scene as arena_scene
        return float(arena_scene.object_states(env, ["freshener"])["freshener"].joints["prong"]) * 1000
    except Exception:  # noqa: BLE001
        return float("nan")


class Grip(float):
    def __new__(cls, yaw, matrix=None):
        value = super().__new__(cls, yaw)
        value.grip_matrix = None if matrix is None else np.array(matrix, float)
        return value


def _install_grip_matrix():
    from yam_arena import demo as arena_demo
    original = arena_demo.grip_rotation
    if getattr(original, "_grip_matrix", False):
        return

    def grip_rotation(yaw=0.0):
        matrix = getattr(yaw, "grip_matrix", None)
        return matrix.copy() if matrix is not None else original(float(yaw))

    grip_rotation._grip_matrix = True
    grip_rotation.__wrapped__ = original
    arena_demo.grip_rotation = grip_rotation


_install_grip_matrix()


class ClipWaypoint(Waypoint):
    """A right-hand waypoint worked out when played: the planned clip pose (or a function of the way), the hand's
    grasp on it, the reads so far, and a world offset."""

    def __init__(self, duration, clip, finger, label, key, offset=(0.0, 0.0, 0.0), keep=False, back=0.0):
        self.duration = duration * (1.0 if keep else DEMO_PACE)
        self.clip, self.finger, self.label, self.key = clip, finger, label, key
        self.offset, self.back = np.asarray(offset, float), float(back)

    def _clip(self):
        return self.clip() if callable(self.clip) else self.clip

    def _hand(self):
        d = _read(self.key) if self.key else np.eye(4)
        h = d @ self._clip() @ HAND_IN_CLIP
        h[:3, 3] += self.offset - self.back * h[:3, 2]
        return h

    @property
    def pos(self):
        return tuple(float(v) for v in self._hand()[:3, 3])

    @property
    def yaw(self):
        return Grip(0.0, self._hand()[:3, :3])


VENT_FROM = "carry it in front of the vane"


def demo_plan():
    """[(label, seconds, planned clip pose (4x4 or callable), finger, read key or None, read mode, offset, back, keep)].
    A read happens when its waypoint starts and compares the clip with the planned pose of the previous waypoint."""
    s0 = clip_start()
    up = lambda dz: _T(s0[:3, :3], s0[:3, 3] + np.array([0.0, 0.0, dz]))  # noqa: E731
    at = lambda depth: (lambda: clip_at_vane(depth))  # noqa: E731
    O, P = GRIP_OPEN, GRIP_PINCH
    plan = [
        ("over the clip", 1.5, s0, O, "grasp", "follow", (0.0, 0.0, 0.030), 0.045, False),
        ("down behind the clip", 1.2, s0, O, None, None, (0.0, 0.0, 0.0), 0.0, False),
        ("pinch its sides", 0.8, s0, P, None, None, (0.0, 0.0, 0.0), 0.0, True),
        ("settle", 0.3, s0, P, None, None, (0.0, 0.0, 0.0), 0.0, False),
        ("lift it", 1.0, up(0.050), P, None, None, (0.0, 0.0, 0.0), 0.0, False),
        ("carry it in front of the vane", 2.5, at(-PRE_TIPS - 0.012), P, None, None, (0.0, 0.0, 0.0), 0.0, False),
        ("hold in front of the vane", 0.6, at(-PRE_TIPS), P, "carry", "move", (0.0, 0.0, 0.0), 0.0, False),
        ("line up with the vane", 0.6, at(-PRE_TIPS), P, "line", "again", (0.0, 0.0, 0.0), 0.0, False),
    ]
    total = PRE_TIPS + PRONG_L + OVERDRIVE
    for k in range(PUSH_STEPS):
        depth = -PRE_TIPS + total * (k + 1) / PUSH_STEPS
        plan.append((f"push along the vane ({k + 1})", PUSH_STEP_S, at(depth), P, f"push{k}", "level",
                     (0.0, 0.0, 0.0), 0.0, False))
    home = at(PRONG_L + OVERDRIVE)
    plan += [
        ("hold it home", 0.6, home, P, None, None, (0.0, 0.0, 0.0), 0.0, False),
        ("let go", 0.8, home, O, None, None, (0.0, 0.0, 0.0), 0.0, True),
        ("back away", 1.0, home, O, None, None, (0.0, 0.0, 0.0), 0.050, False),
        ("rise clear", 1.0, home, O, None, None, (0.0, 0.0, 0.040), 0.080, False),
    ]
    return plan


def demo():
    plan = demo_plan()
    _PLAN["keys"], _PLAN["planned"], _PLAN["modes"] = [], {}, {}
    first_vent = [p[0] for p in plan].index(VENT_FROM)  # from here on the pick's read no longer applies
    key_now, waypoints = None, []
    for i, (label, secs, clip, finger, key, mode, offset, back, keep) in enumerate(plan):
        if key:
            _PLAN["keys"].append(key)
            _PLAN["planned"][key] = plan[i - 1][2] if i > 0 else clip_start()  # evaluated when the read happens
            _PLAN["modes"][key] = mode
            key_now = key
        if i == first_vent:
            key_now = None
        waypoints.append(ClipWaypoint(secs, clip, finger, label, key_now, offset, keep, back))
    # first, from the ready pose (over the dashboard) out in front of it, so the hand does not cut its top edge
    waypoints.insert(0, Waypoint(1.5 * DEMO_PACE, tuple(float(v) for v in CLEAR_POINT), GRIP_OPEN,
                                 "out in front of the dashboard", Grip(0.0, _hand_rotation(PITCH))))
    park = Grip(0.0, _hand_rotation(math.radians(60.0)))
    waypoints.append(Waypoint(2.0 * DEMO_PACE, tuple(float(v) for v in PARK["right"]), GRIP_OPEN, "park", park))
    waypoints.append(Waypoint(FINAL_HOLD_S, tuple(float(v) for v in PARK["right"]), GRIP_OPEN, "hold clear", park))
    return {"right": waypoints}  # the left arm stays at its ready pose


# ---------------------------------------------------------------- offline test layouts (states.yaml)
def _quat(rot):
    from yam_arena.ik import matrix_to_quat_xyzw
    return [float(v) for v in matrix_to_quat_xyzw(np.asarray(rot, float))]


def clip_on(k=TARGET_VANE, gap=0.0, dz=0.0, pitch_deg=0.0, yaw_deg=0.0, dy=0.0, vane_deg=0.0):
    """[origin, quat] of the clip with vane k's front edge `gap` metres short of the fork's root, the vane's centre
    plane VANE_MID + dz above the clip's, the clip turned pitch_deg (nose down +) about the edge point and yaw_deg
    about z, moved dy along the vane; the vane itself turned vane_deg (front edge up +) about its pins."""
    pin = np.array([FACE_X + PIN_BACK, VENT_Y + dy, TOP + VENT_Z + VANE_Z[k]])
    rv = _rot_y(math.radians(vane_deg))  # a positive hinge angle about +y lifts the front edge
    edge = pin + rv @ np.array([-PIN_BACK, 0.0, 0.0])
    R = _rot_y(math.radians(pitch_deg)) @ np.array([[math.cos(math.radians(yaw_deg)), -math.sin(math.radians(yaw_deg)), 0],
                                                     [math.sin(math.radians(yaw_deg)), math.cos(math.radians(yaw_deg)), 0],
                                                     [0, 0, 1]])
    local_edge = np.array([BACK_X + gap, 0.0, VANE_MID + dz])
    origin = edge - R @ local_edge
    return [float(v) for v in origin], _quat(R)


def vane_angle(deg):
    """Hinge value (radians) for a vane turned `deg` with its front edge up (+)."""
    return math.radians(deg)
