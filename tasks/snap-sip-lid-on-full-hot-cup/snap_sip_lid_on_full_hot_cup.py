"""Snap a dome sip lid onto a full 16 oz paper hot cup, all the way round the rim, without tipping or squeezing it.

A counter worker's lidding step. A full Dart Solo 316W 16 oz single-wall paper hot cup (88.9 mm rim, 58.4 mm base,
134.6 mm tall, 451 g of water) stands on the counter in front of the robot. A black Dart Solo TLB316 Traveler dome
sip lid (93.0 mm across, 18.5 mm tall, 4.2 g) lies top-up on the counter to the robot's left. The lid has to go on
the cup and be pressed home round the whole rim: its rim is eight 45-degree sectors, each a short vertical slide in
the lid that only moves when pressed with 3 N or more (and then stays). The cup is light and tall: a press of about
9.0 N on one side of the rim tips it, and its wall is eight sprung panels that dent in under a squeeze.
Metres; zero jitter.
"""
import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
from yam_parts import cyl  # noqa: E402
from yam_joints import Link, Slide  # noqa: E402
from yam_mesh import Mesh, MeshMechanism, revolve  # noqa: E402
from yam_arena.constants import FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
# Release-and-stillness block, verbatim from hang_holder_on_keyhole_studs.py (see release-stillness.md).
# The task module must define: import math; import numpy as np; from yam_arena.constants import FINGER_OPEN;
# and SCENE ("station" or "tabletop"). Paste after the imports.

# Release and stillness
# Released means every gripper TCP is at least RELEASE_CLEAR from the object's box.
# Still means below the task's linear and angular speed limits for STILL_S seconds.
# Evaluate every sticky condition and timer at the top of score() on every call,
# before any early return, so interrupted conditions cannot retain elapsed time.
RELEASE_CLEAR = 0.030
STILL_S = 1.0
TEST_GRIPPERS = None


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
_MEMORY_EPISODE = None
_TIMERS = {}
_EVER = {}


def grippers():
    if TEST_GRIPPERS is not None:
        return TEST_GRIPPERS
    env = _RUN["env"]
    if env is None:
        return None
    from yam_arena import ik, station
    from yam_arena.constants import ARM_JOINTS, FINGER_JOINTS, LEFT_BASE_POS, RIGHT_BASE_POS

    bases = station.BASES if SCENE == "station" else {"left": LEFT_BASE_POS, "right": RIGHT_BASE_POS}
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
        tcp = ik.tcp_pose(q[arm_ids], bases[arm])[0]
        opening = float(np.clip(np.mean(q[finger_ids]) / FINGER_OPEN, 0.0, 1.0))
        out[arm] = (tcp, opening)
    return out


def released(state, clear=RELEASE_CLEAR):
    hands = grippers()
    if hands is None:
        return True  # Offline arm poses are unknown, so release is assumed.
    low, high = np.asarray(state.low), np.asarray(state.high)
    return all(np.linalg.norm(np.maximum(np.maximum(low - tcp, tcp - high), 0.0)) >= clear
               for tcp, _ in hands.values())


def _episode_memory():
    global _MEMORY_EPISODE
    if _MEMORY_EPISODE != _RUN["episode"]:
        _TIMERS.clear()
        _EVER.clear()
        _MEMORY_EPISODE = _RUN["episode"]


def held_for(key, cond):
    """Elapsed time over consecutive observed control steps; repeat calls are harmless."""
    _episode_memory()
    t, step = _RUN["t"], _RUN["step"]
    if t is None:
        return math.inf if cond else 0.0
    previous_step, start = _TIMERS.get(key, (None, None))
    if not cond:
        _TIMERS[key] = (step, None)
        return 0.0
    if start is None or step not in (previous_step, previous_step + 1) or t < start:
        start = t
    _TIMERS[key] = (step, start)
    return float(t - start)


def ever(key, cond):
    """Remember a satisfied condition until the next episode; offline each call stands alone."""
    if _RUN["t"] is None: return bool(cond)
    _episode_memory()
    _EVER[key] = bool(_EVER.get(key, False) or cond)
    return _EVER[key]


# ---------------------------------------------------------------------------------------------------------------
# Scene
DEMO_PACE = 1.0
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 1.0
RUN_LIMIT_STEPS = 2800
MM = 0.001
TAU = 2 * math.pi


def _arc_mesh(name, profile_rz, a0, a1, segments, color, proxy=()):
    """A closed (r, z) section swept about the z axis from angle a0 to a1 (rad), with flat end caps.

    The section must be convex and listed in order round its edge; the result is used as a convex-hull collider.
    """
    profile = np.asarray(profile_rz, float)
    n = len(profile)
    angles = np.linspace(a0, a1, segments + 1)
    points = np.array([(r * math.cos(a), r * math.sin(a), z) for a in angles for r, z in profile])
    faces = []
    for i in range(segments):
        for j in range(n):
            a, b = i * n + j, i * n + (j + 1) % n
            c, d = a + n, b + n
            faces += [(a, b, d), (a, d, c)]
    faces += [(0, j + 1, j) for j in range(1, n - 1)]
    last = segments * n
    faces += [(last, last + j, last + j + 1) for j in range(1, n - 1)]
    faces = np.asarray(faces, int)
    tri = (points - points.mean(axis=0))[faces]
    if np.einsum("ij,ij->i", tri[:, 0], np.cross(tri[:, 1], tri[:, 2])).sum() < 0:
        faces = faces[:, ::-1]
    return Mesh(name, points, faces, collision="convex", color=color, proxy=tuple(proxy))


def _polar(r, angle, z=0.0):
    return (r * math.cos(angle), r * math.sin(angle), z)


# --- The cup: Dart Solo 316W-2050, 16 oz single-wall poly-coated paper hot cup, white. Frame: bottom centre. -----
CUP_XY = (0.120, 0.0)
CUP_H = 134.6 * MM  # height; the rim's top
CUP_RIM_R = 44.45 * MM  # 88.9 mm across the rolled lip
CUP_BASE_R = 29.2 * MM  # 58.4 mm across the bottom
WALL_TOP_Z, WALL_TOP_R = 131.9 * MM, 43.2 * MM  # the straight tapered wall ends where the rolled lip starts
LIP_Z = 132.6 * MM  # the rolled lip: 88.9 mm across from here to the rim's top
WALL_T = 0.45 * MM
BOTTOM_Z = 6.0 * MM  # recessed bottom disc
BAND_Z = 130.6 * MM  # the stiff band under the lip (the base body); the sprung wall panels end here
WATER_Z = CUP_H - 15.0 * MM  # coffee (room-temperature water in a real trial) 15 mm below the rim
CUP_PAPER_MASS = 14.0e-3
WATER_MASS = 0.451
PANEL_COUNT = 8
PANEL_MASS = 1.25e-3
PANEL_K = 630.0  # N/m per panel, radial: 3.0 N for 9.5 mm across the cup (two walls), after GB/T 27590's test
PANEL_TRAVEL = 8.5 * MM  # radial
PANEL_PRELOAD = 0.5 * MM
PANEL_DAMPING = 2.0
PANEL_ARMATURE = 0.01
PANEL_SEAM_DEG = 0.3
CUP_WHITE = (0.93, 0.93, 0.91)
COFFEE = (0.30, 0.19, 0.11)


def r_wall(z):
    """Outer radius of the cup wall at height z above its bottom (straight taper from base to lip)."""
    return CUP_BASE_R + (WALL_TOP_R - CUP_BASE_R) * z / WALL_TOP_Z


def _panel_axis(k):
    """(slide axis, sign of q that pushes the panel in, radial share of the slide) for wall panel k."""
    c, s = math.cos(k * TAU / PANEL_COUNT), math.sin(k * TAU / PANEL_COUNT)
    if abs(c) >= abs(s) - 1e-9:
        return "x", (-1.0 if c > 0 else 1.0), abs(c)
    return "y", (-1.0 if s > 0 else 1.0), abs(s)


def make_cup(pos, yaw=0.0):
    bottom = revolve([(0, 0), (CUP_BASE_R, 0), (r_wall(BOTTOM_Z), BOTTOM_Z), (0, BOTTOM_Z)], samples=30,
                     name="bottom", collision="convex", color=CUP_WHITE)
    # PhysX keeps at most 64 hull vertices, so each convex collider is kept under that: three outer rings here.
    band_profile = [(r_wall(BAND_Z) - 0.7 * MM, BAND_Z), (r_wall(BAND_Z), BAND_Z), (CUP_RIM_R, LIP_Z),
                    (CUP_RIM_R, CUP_H), (42.8 * MM, CUP_H), (42.4 * MM, CUP_H - 0.8 * MM)]
    # The offline stand-in for the whole cup: a 88.9 mm cylinder to the rim (the hull of the rim band is solid).
    band = revolve(band_profile, samples=20, name="rim", collision="convex", color=CUP_WHITE,
                   proxy=(cyl((0, 0, CUP_H / 2), CUP_RIM_R, CUP_H),))
    water = revolve([(0, BOTTOM_Z), (r_wall(BOTTOM_Z) - WALL_T - 0.15 * MM, BOTTOM_Z),
                     (r_wall(WATER_Z) - WALL_T - 0.15 * MM, WATER_Z), (0, WATER_Z)], samples=30, name="coffee",
                    collision="convex", color=COFFEE)
    links, meshes = [], {"base": (bottom, band, water)}
    half = math.radians(22.5 - PANEL_SEAM_DEG)
    zm = (BOTTOM_Z + BAND_Z) / 2
    for k in range(PANEL_COUNT):
        a = k * TAU / PANEL_COUNT
        axis, inward, share = _panel_axis(k)
        travel = PANEL_TRAVEL / share
        lower, upper = (-travel, 0.0) if inward < 0 else (0.0, travel)
        name = f"panel_{k}"
        section = [(r_wall(BOTTOM_Z) - WALL_T, BOTTOM_Z), (r_wall(BOTTOM_Z), BOTTOM_Z),
                   (r_wall(BAND_Z), BAND_Z), (r_wall(BAND_Z) - WALL_T, BAND_Z)]
        mesh = _arc_mesh(name, section, a - half, a + half, 12, CUP_WHITE,
                         proxy=(cyl(_polar(r_wall(zm) - 2 * MM, a, zm), 0.5 * MM, BAND_Z - BOTTOM_Z - 2 * MM),))
        meshes[name] = (mesh,)
        links.append(Link(name, [], Slide(axis, _polar(r_wall(zm), a, zm), lower, upper,
                                          damping=PANEL_DAMPING, spring=PANEL_K * share * share,
                                          rest=-inward * PANEL_PRELOAD / share, armature=PANEL_ARMATURE),
                          PANEL_MASS, color=CUP_WHITE))
    return MeshMechanism("cup", pos, (), links, mass=WATER_MASS + CUP_PAPER_MASS - PANEL_COUNT * PANEL_MASS,
                         color=CUP_WHITE, yaw=yaw, friction=(0.4, 0.35), contact_offset=0.001, meshes=meshes)


# --- The lid: Dart Solo TLB316-0004 Traveler dome sip lid, black polystyrene. Frame: centre of the deck's underside.
LID_XY = (0.150, 0.165)
LID_YAW = 0.0
LID_MASS = 4.2e-3
SECTOR_COUNT = 8
SECTOR_MASS = 0.25e-3
SEAT_TRAVEL = 2.5 * MM  # an unseated sector stands this much proud of its seated height
# The slide's own stop is the seated height, and a seated channel stops SEAT_GAP short of the rim, so the flat ring
# alone carries the lid and every later press (it sinks up to 0.27 mm into the rim's contact under an 8 N press on the
# GPU): a snapped section then moves with the lid and is never pushed back up by the rim.
SLIDE_LOWER = -SEAT_TRAVEL
SEAT_GAP = 0.5 * MM
SEAT_STATIC_N, SEAT_DYNAMIC_N = 3.0, 2.2  # a sector moves when pressed with 3 N, and stays where it is left
# A large slide armature (inertia carried by the lid's base, not mass): under a press the 2.2 g base jitters in its
# contact with the rim, and the slides take up that jitter in proportion to 0.25 g / (0.25 g + armature). On the GPU,
# 5 g let seated sectors walk off their stops at up to 0.9 mm/s while another pair was pressed, and 0.5 g flung them
# back to the top; 50 g shields them. Armature does not change the 3 N hold or what a press must overcome.
SECTOR_DAMPING, SECTOR_ARMATURE = 0.5, 0.05
# The flat ring reaches out over the rim's whole top (42.8-44.45 mm), so the lid rests on the rim where a real lid's rim
# channel does; it runs on under the channel tops (the lid's base and its sectors do not collide with each other).
DECK_R, DECK_T = 44.6 * MM, 1.2 * MM
DOME_PROFILE = [(0, 1.0 * MM), (38.5 * MM, 1.0 * MM), (37.6 * MM, 8.5 * MM), (32.0 * MM, 12.0 * MM),
                (0, 12.5 * MM)]  # three rings of 20: under PhysX's 64-vertex hull limit
DOME_R, DOME_TOP = 38.5 * MM, 12.5 * MM
SLAB_R = (42.0 * MM, 46.0 * MM)  # the rim channel's top: what a finger presses
SLAB_Z = (SEAT_TRAVEL + SEAT_GAP, SEAT_TRAVEL + SEAT_GAP + 1.5 * MM)  # at q = 0; seated, SEAT_GAP above the rim's top
SKIRT = [(45.65 * MM, SEAT_TRAVEL + SEAT_GAP), (46.15 * MM, SEAT_TRAVEL + SEAT_GAP), (46.5 * MM, -3.5 * MM),
         (46.05 * MM, -3.5 * MM), (45.65 * MM, -2.0 * MM)]  # (r, z) at q = 0: wall 0.5 mm, flared mouth 93.0 mm across
LID_REST_Z = 3.5 * MM  # lying top-up on the counter, the lid stands on its skirt
SECTOR_SEAM_DEG = 0.25
LID_BLACK = (0.05, 0.05, 0.055)


def make_lid(pos, yaw=0.0):
    deck = cyl((0, 0, DECK_T / 2), DECK_R, DECK_T, color=LID_BLACK)
    dome = revolve(DOME_PROFILE, samples=20, name="dome", collision="convex", color=LID_BLACK,
                   proxy=(cyl((0, 0, (DOME_PROFILE[0][1] + DOME_TOP) / 2), DOME_R, DOME_TOP - DOME_PROFILE[0][1]),))
    links, meshes = [], {"base": (dome,)}
    half = math.radians(22.5 - SECTOR_SEAM_DEG)
    r_mid = sum(SLAB_R) / 2
    for k in range(SECTOR_COUNT):
        a = k * TAU / SECTOR_COUNT
        name = f"sector_{k}"
        slab_proxy = tuple(cyl(_polar(r_mid, a + d, sum(SLAB_Z) / 2), 2.0 * MM, SLAB_Z[1] - SLAB_Z[0])
                           for d in (-math.radians(15), 0.0, math.radians(15)))
        slab = _arc_mesh(f"{name}_top", [(SLAB_R[0], SLAB_Z[0]), (SLAB_R[1], SLAB_Z[0]), (SLAB_R[1], SLAB_Z[1]),
                                          (SLAB_R[0], SLAB_Z[1])], a - half, a + half, 12, LID_BLACK, slab_proxy)
        skirts = []
        for i, (b0, b1) in enumerate(((a - half, a - half / 3), (a - half / 3, a + half / 3), (a + half / 3, a + half))):
            pin = cyl(_polar(45.9 * MM, (b0 + b1) / 2, (SKIRT[0][1] + SKIRT[2][1]) / 2), 0.4 * MM,
                      SKIRT[0][1] - SKIRT[2][1])
            skirts.append(_arc_mesh(f"{name}_skirt{i}", SKIRT, b0, b1, 4, LID_BLACK, (pin,)))
        meshes[name] = (slab, *skirts)
        links.append(Link(name, [], Slide("z", _polar(r_mid, a, sum(SLAB_Z) / 2), SLIDE_LOWER, 0.0,
                                          damping=SECTOR_DAMPING, armature=SECTOR_ARMATURE,
                                          static_friction=SEAT_STATIC_N, dynamic_friction=SEAT_DYNAMIC_N),
                          SECTOR_MASS, color=LID_BLACK))
    # grip 1: the dome's offline stand-in (after the one primitive, the deck), which the pick pinches
    return MeshMechanism("lid", pos, (deck,), links, mass=LID_MASS - SECTOR_COUNT * SECTOR_MASS, color=LID_BLACK,
                         yaw=yaw, friction=(0.3, 0.25), contact_offset=0.001, grip=1, meshes=meshes)


CUP = make_cup((*CUP_XY, TOP))
LID = make_lid((*LID_XY, TOP + LID_REST_Z), LID_YAW)
SECTOR_NAMES = tuple(link.name for link in LID.links)
PANEL_NAMES = tuple(link.name for link in CUP.links)
OBJECTS = [CUP, LID]
PRESSED_PARTS = ("lid", "cup")  # the closed pads press the lid's rim onto the cup's; replay cannot move a sector

# ---------------------------------------------------------------------------------------------------------------
# Scorer
SEATED_Q = -(SEAT_TRAVEL - 0.8 * MM)  # a sector within 0.8 mm of its seated height (-2.5 mm) is seated
ON_CUP_XY, ON_CUP_DZ, ON_CUP_TILT_DEG = 3.0 * MM, 1.5 * MM, 5.0
AWAY_XY, AWAY_DZ, AWAY_TILT_DEG = 10.0 * MM, 10.0 * MM, 20.0
FORCED_Q = -1.0 * MM  # a sector pushed this far down while the lid is away from the cup
CUP_TIP_DEG = 15.0
CUP_LIFT = 10.0 * MM
CUP_SHIFT = 30.0 * MM
SQUEEZE_MM = 5.0
SPEED = 0.01
POS_STILL, ROT_STILL_DEG, Q_STILL = 1.0 * MM, 2.0, 0.2 * MM
HELD_CAP = 0.5  # the most a run earns while a gripper is still on the lid or cup, or they are still moving


def _rotation_deg(a, b):
    return math.degrees(math.acos(float(np.clip((np.trace(a.T @ b) - 1) / 2, -1.0, 1.0))))


def lid_on_cup(lid, cup):
    """(on, away, offset_mm, height_mm, tilt_deg): where the lid sits in the cup's frame."""
    rc = cup.rotation()
    rel = rc.T @ (np.asarray(lid.origin, float) - np.asarray(cup.origin, float))
    offset = float(np.hypot(rel[0], rel[1]))
    height = float(rel[2] - CUP_H)
    tilt = math.degrees(math.acos(float(np.clip((rc.T @ lid.rotation())[2, 2], -1.0, 1.0))))
    on = offset <= ON_CUP_XY and abs(height) <= ON_CUP_DZ and tilt <= ON_CUP_TILT_DEG
    away = offset > AWAY_XY or abs(height) > AWAY_DZ or tilt > AWAY_TILT_DEG
    return on, away, offset / MM, height / MM, tilt


def seated_sectors(lid):
    return [name for name in SECTOR_NAMES if float(lid.joints[name]) <= SEATED_Q + 1e-9]  # 1e-9: float rounding


def squeeze_mm(cup):
    """The largest radial distance any wall panel has been pushed in, mm."""
    out = 0.0
    for k, name in enumerate(PANEL_NAMES):
        _, inward, share = _panel_axis(k)
        out = max(out, max(0.0, inward * float(cup.joints[name])) * share / MM)
    return out


_STATE = {"episode": None, "step": None, "samples": [], "result": None}


def _fresh_episode():
    if _STATE["episode"] != _RUN["episode"]:
        _STATE.update(episode=_RUN["episode"], step=None, samples=[], result=None)


def _windows_still(lid, cup):
    """Pose and joint windows over the last STILL_S (contact-driven joint speeds chatter; poses do not)."""
    if _RUN["t"] is None:
        return True  # a single offline layout cannot show motion over time; speeds are checked separately
    t, step, samples = _RUN["t"], _RUN["step"], _STATE["samples"]
    now = (np.array(lid.origin, float), lid.rotation(), np.array(cup.origin, float), cup.rotation(),
           np.array([float(lid.joints[n]) for n in SECTOR_NAMES]))
    if samples and (step != samples[-1][0] + 1 or t <= samples[-1][1]):
        samples.clear()  # a missed step or a clock rewind needs a fresh window
    samples.append((step, t, now))
    while len(samples) > 1 and samples[1][1] <= t - STILL_S + 1e-9:
        samples.pop(0)
    if samples[0][1] > t - STILL_S + 1e-9:
        return False  # less than a full window observed
    for _, _, old in samples:
        if (np.linalg.norm(now[0] - old[0]) > POS_STILL or _rotation_deg(now[1], old[1]) > ROT_STILL_DEG
                or np.linalg.norm(now[2] - old[2]) > POS_STILL or _rotation_deg(now[3], old[3]) > ROT_STILL_DEG
                or np.max(np.abs(now[4] - old[4])) > Q_STILL):
            return False
    return True


def _evaluate(objects):
    lid, cup = objects["lid"], objects["cup"]
    on, away, _, _, _ = lid_on_cup(lid, cup)
    lowest_q = min(float(lid.joints[n]) for n in SECTOR_NAMES)
    origin = np.asarray(cup.origin, float)
    # Sticky rules first, on every call: a tipped, lifted or squeezed cup, or a lid sector forced down off the cup.
    tipped = ever("tipped", cup.tilt_deg() > CUP_TIP_DEG)
    lifted = ever("lifted", origin[2] - TOP > CUP_LIFT)
    squeezed = ever("squeezed", squeeze_mm(cup) > SQUEEZE_MM)
    forced = ever("forced", away and lowest_q < FORCED_Q)
    shifted = float(np.hypot(origin[0] - CUP_XY[0], origin[1] - CUP_XY[1])) > CUP_SHIFT  # end state
    still = (_windows_still(lid, cup) and np.linalg.norm(lid.lin_vel) < SPEED
             and np.linalg.norm(cup.lin_vel) < SPEED)
    hands_off = released(lid) and released(cup)
    done = bool(still and hands_off)
    if tipped or lifted or squeezed or forced or shifted or not on:
        return 0.0
    n = len(seated_sectors(lid))
    tier = 1.0 if n == SECTOR_COUNT else 0.75 if n >= 6 else 0.5 if n >= 3 else 0.25
    return tier if done else min(tier, HELD_CAP)


def score(objects):
    if _RUN["t"] is not None:
        _fresh_episode()
        if _STATE["step"] == _RUN["step"] and _STATE["result"] is not None:
            return _STATE["result"]  # success() and score() share one control step
        _STATE["step"] = _RUN["step"]
        _STATE["result"] = _evaluate(objects)
        return _STATE["result"]
    return _evaluate(objects)


def success(objects):
    return score(objects) >= 1.0


# ---------------------------------------------------------------------------------------------------------------
# Demo: open loop from the known start layout and time; it reads no object or joint state.
PAD_TIP = 0.013  # the pad tips are 13 mm past the TCP
PINCH_TIP_Z = 6.0 * MM  # pad tips this far above the lid's deck underside: 2 mm over the rim channel's top
PINCH_OPEN = -36.6 * MM  # pads 73.2 mm apart on the 75-76 mm dome wall: a light squeeze
LET_GO_OPEN = -45.0 * MM  # pads 90 mm apart: 7 mm clear of the dome on each side
CARRY_CLEAR = 10.0 * MM  # the carried lid's skirt passes this far above the rim
PLACE_GAP = 0.5 * MM  # the deck is let go this far above the rim
PRESS_R = 44.0 * MM  # the rim channel's mid-radius: where a press on one sector acts (the setup's 9.0 N tipping figure)
# The demo presses two opposite sectors at once, one open fingertip on each side of the dome, so the push is centred
# on the cup and the lid is held level between the two tips. Each tip is 3.45 mm thick along the closing axis.
PAIR_OPEN = -42.3 * MM  # tips span 42.3-45.75 mm from the cup's axis: centred on the 42-46 mm channel, 3.8 mm off the dome
PAIR_DEPTH = 15.0 * MM  # tips commanded this far below the seated channel top: about 7.9 N in all before seating
HOVER_ABOVE = 8.0 * MM  # pad tips this far above an unseated sector before and after a press
# Open-loop trim for the arm's own compliance and joint friction (measured on the GPU), as offsets along the horizontal
# line from the left arm's base out through the cup's axis: hovering, the TCP sits 0.8 mm farther out than commanded;
# the 9 mm descent to the channel leaves it 1.5 mm nearer the base; and an 8 N downward push draws it 3.2 mm toward the
# base. So the aim is 0.8 mm in at the hover, 0.7 mm out at the touch, and moves to 3.9 mm out as the push builds,
# keeping the tips over the cup's axis instead of dragging the lid and cup sideways.
TRIM_HOVER, TRIM_TOUCH, TRIM_LOADED = -0.8 * MM, 0.7 * MM, 3.9 * MM
# (sectors, grip yaw in degrees): yaw 0 closes the fingers along y. These four keep wrist joint 6 within 69 degrees.
PAIRS = (((2, 6), 0.0), ((1, 5), -45.0), ((0, 4), -90.0), ((3, 7), -135.0))
TAIL = 3.0  # stillness after both arms are clear: the 1 s window and the 1 s success hold, with margin
LEFT_PARK = (0.110, 0.220, TOP + 0.170)
RIGHT_PARK = (0.110, -0.220, TOP + 0.170)

RIM_TOP = TOP + CUP_H  # the cup's rim, world z
SEATED_TOP = RIM_TOP + (SLAB_Z[1] - SEAT_TRAVEL)  # a seated sector's top face
UNSEATED_TOP = SEATED_TOP + SEAT_TRAVEL
PICK_TCP_Z = TOP + LID_REST_Z + PINCH_TIP_Z + PAD_TIP
CARRY_TCP_Z = RIM_TOP + (LID_REST_Z + CARRY_CLEAR) + PINCH_TIP_Z + PAD_TIP
PLACE_TCP_Z = RIM_TOP + PLACE_GAP + PINCH_TIP_Z + PAD_TIP
HOVER_TCP_Z = UNSEATED_TOP + HOVER_ABOVE + PAD_TIP
TOUCH_TCP_Z = UNSEATED_TOP + PAD_TIP
PRESS_TCP_Z = SEATED_TOP - PAIR_DEPTH + PAD_TIP
ABOVE_LID_Z = TOP + 0.175  # pad tips 15 mm over the dome: where the fingers change their opening


def press_point(k):
    """World (x, y) of the rim channel's middle over lid sector k once the lid is on the cup at its start yaw."""
    a = LID_YAW + k * TAU / SECTOR_COUNT
    return (CUP_XY[0] + PRESS_R * math.cos(a), CUP_XY[1] + PRESS_R * math.sin(a))


def demo():
    plans, clock = {"left": [], "right": []}, {"left": 0.0, "right": 0.0}

    def add(arm, label, dt, pos, finger, yaw=0.0, keep=False):
        dt = dt if keep else dt * DEMO_PACE
        plans[arm].append(Waypoint(dt, tuple(float(v) for v in pos), finger, label, float(yaw)))
        clock[arm] += dt

    def hold_until(arm, t, label):
        last = plans[arm][-1]
        if t - clock[arm] > 1e-9:
            add(arm, label, t - clock[arm], last.pos, last.finger, last.yaw, keep=True)

    # Left arm: pinch the lid's dome, carry it over the cup, lower it on, let go; then, over the cup's axis with its
    # fingers open either side of the dome, press opposite sectors home two at a time, turning 45 degrees between pairs.
    lid_xy, cup_xy = LID_XY, CUP_XY
    add("left", "over the lid", 2.0, (*lid_xy, PICK_TCP_Z + 0.040), FINGER_OPEN, LID_YAW)
    add("left", "down to the dome", 1.2, (*lid_xy, PICK_TCP_Z), FINGER_OPEN, LID_YAW)
    add("left", "pinch the dome", 0.6, (*lid_xy, PICK_TCP_Z), PINCH_OPEN, LID_YAW, keep=True)
    add("left", "lift the lid", 1.0, (*lid_xy, CARRY_TCP_Z), PINCH_OPEN, LID_YAW)
    add("left", "carry over the cup", 1.6, (*cup_xy, CARRY_TCP_Z), PINCH_OPEN, LID_YAW)
    add("left", "settle over the cup", 0.6, (*cup_xy, CARRY_TCP_Z), PINCH_OPEN, LID_YAW, keep=True)
    add("left", "lower the lid on", 1.4, (*cup_xy, PLACE_TCP_Z), PINCH_OPEN, LID_YAW, keep=True)
    add("left", "let go of the lid", 0.5, (*cup_xy, PLACE_TCP_Z), LET_GO_OPEN, LID_YAW, keep=True)
    add("left", "rise off the lid", 0.8, (*cup_xy, ABOVE_LID_Z), LET_GO_OPEN, LID_YAW)
    add("left", "set the fingers over the rim", 0.5, (*cup_xy, ABOVE_LID_Z), PAIR_OPEN, LID_YAW)
    from yam_arena import station
    out = np.asarray(cup_xy) - np.asarray(station.BASES["left"][:2])
    out /= np.linalg.norm(out)  # horizontal, from the left arm's base out through the cup's axis
    hover_xy, touch_xy, loaded_xy = (np.asarray(cup_xy) + t * out for t in (TRIM_HOVER, TRIM_TOUCH, TRIM_LOADED))
    for (a, b), yaw_deg in PAIRS:
        yaw = LID_YAW + math.radians(yaw_deg)
        add("left", f"turn to sectors {a} and {b}", 0.8, (*hover_xy, HOVER_TCP_Z), PAIR_OPEN, yaw)
        add("left", f"touch sectors {a} and {b}", 0.5, (*touch_xy, TOUCH_TCP_Z), PAIR_OPEN, yaw, keep=True)
        add("left", f"press sectors {a} and {b}", 0.8, (*loaded_xy, PRESS_TCP_Z), PAIR_OPEN, yaw, keep=True)
        add("left", f"hold sectors {a} and {b}", 0.3, (*loaded_xy, PRESS_TCP_Z), PAIR_OPEN, yaw, keep=True)
        add("left", f"lift off sectors {a} and {b}", 0.5, (*hover_xy, HOVER_TCP_Z), PAIR_OPEN, yaw)
    add("left", "rise clear of the lid", 0.8, (*cup_xy, ABOVE_LID_Z), PAIR_OPEN, yaw)
    add("left", "park", 1.2, LEFT_PARK, FINGER_OPEN, 0.0)

    # Right arm: not needed; it moves to its park point and stays there.
    add("right", "park", 1.0, RIGHT_PARK, FINGER_OPEN, 0.0)

    end = max(clock.values()) + TAIL
    for arm in ("left", "right"):
        hold_until(arm, end, "stillness tail")
    return plans
