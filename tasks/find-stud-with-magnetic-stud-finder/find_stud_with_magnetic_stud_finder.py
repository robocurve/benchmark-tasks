"""Find a stud with a magnetic stud finder: slide it flat over a painted wall until it locks onto a hidden drywall
screw, then let go so it hangs there on its magnets.

On the table: a section of stud wall stands across the robot's path, 660 mm long and 230 mm tall, its painted
drywall face toward the robot. Behind the paint, two studs 16 in (406 mm) apart each hold the board with one
drywall screw, its head sunk under joint compound and paint, so nothing on the face shows where they are. Where
the studs and screws are changes from trial to trial (the trial's seed picks one of four wall sections that look
alike). A magnetic stud finder (black rubber body, yellow badges, side tabs and pointer, about 90 x 38 x 20 mm,
68 g) stands upright in a grey holder in front of the left arm, its flat back toward the wall.

Two rare-earth magnets sit behind the finder's back. Slid flat over the wall, the finder is tugged sideways and
pulled onto the wall when a magnet passes over a screw head; left there, it hangs on the wall by itself, marking
the stud. Lifted or tilted off the wall, its magnets are too far from a screw to catch it, and let go anywhere
but over a screw, it falls.
"""
import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
from yam_parts import Part, box, cyl  # noqa: E402
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"

# Release-and-stillness block, verbatim from the parts kit's docs (release_stillness_block.py).
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


# ---------------------------------------------------------------- layout (metres; x forward, y left, z up)
TOP = TABLE_TOP[SCENE]
DEMO_PACE = 1.0
SUCCESS_HOLD_S = 1.0
RUN_LIMIT_STEPS = 3600

# Colours (sRGB)
WALL_WHITE = (0.93, 0.93, 0.90)      # flat white latex paint
PINE = (0.86, 0.70, 0.45)            # kiln-dried pine plate
FINDER_BLACK = (0.07, 0.07, 0.08)    # the finder's black rubber body
FINDER_YELLOW = (0.98, 0.80, 0.06)   # its yellow badges, side tabs, level ring and pointer
VIAL_GREEN = (0.62, 0.80, 0.36)      # the level's vial
HOLDER_GREY = (0.52, 0.54, 0.57)


def _b(x0, x1, y0, y1, z0, z1, color=None):
    """A box from its extents (part frame)."""
    return box(((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2), (x1 - x0, y1 - y0, z1 - z0), color=color)


# Wall section (made for the trial, a true-size slice of an ordinary stud wall, only its height cut to what the arms
# can work over): 1/2 in (12.7) gypsum drywall, 660 long and 230 tall, face the plane x = WALL_X toward the robot,
# primed and painted flat white. Behind it: a pine 2x4 bottom plate laid flat (38 x 89) and, across the top, a 1/2 in
# plywood cap painted white, so that the studs cannot be seen from above; between them two 2x4 studs 406 apart (where
# depends on the layout). The studs are hidden and never touched, so they are not simulated as bodies; the screws
# exist only as the magnets' targets. 230 tall keeps the wall's top 18 mm under both arms' fingertips in the ready
# pose they start in (URDF meshes).
WALL_X = 0.255
DRYWALL_T = 0.0127
WALL_Y0, WALL_Y1 = -0.330, 0.330
WALL_H = 0.230
STUD_W, STUD_D = 0.038, 0.089
CAP_T = 0.0127
_FX0, _FX1 = DRYWALL_T, DRYWALL_T + STUD_D
WALL = Part("wall", (WALL_X, 0.0, TOP), [
    _b(0.0, DRYWALL_T, WALL_Y0, WALL_Y1, 0.0, WALL_H),                                   # 0 drywall, painted face
    _b(_FX0, _FX1, WALL_Y0, WALL_Y1, 0.0, STUD_W, color=PINE),                           # 1 bottom plate
    _b(_FX0, _FX1, WALL_Y0, WALL_Y1, WALL_H - CAP_T, WALL_H),                             # 2 plywood cap, painted
], color=WALL_WHITE, friction=(0.40, 0.35), contact_offset=0.001)

# The screws: #6 x 1-1/4 in coarse-thread drywall screws (steel, bugle head 8.3 across), one in each stud on the
# stud's centre line, the head driven 1.0 below the face and covered with joint compound and paint. Real walls are
# screwed at most 16 in (406) apart up each stud, so a 230-tall section holds one screw per stud, both at about
# the same height. Four wall sections are built alike; the trial's seed picks one (seed mod 4):
#   (left stud's centre y, screws' height above the table); the right stud is 406 to the right.
STUD_SPACING = 0.4064
SCREW_SET = 0.0010
LAYOUTS = ((0.190, 0.114), (0.110, 0.126), (0.150, 0.114), (0.230, 0.126))
ACTIVE_LAYOUT = 0            # set from the seed at every reset (stud_runtime)
TEST_LAYOUT = None           # tests only
FORCE_LAYOUT = None          # a diagnostic copy may pin a layout; the task never sets it


def layout_for_seed(seed):
    """The wall section a trial uses: seed mod 4 (section 0 with no seed)."""
    return 0 if seed is None else int(seed) % len(LAYOUTS)


def layout_index():
    for k in (TEST_LAYOUT, FORCE_LAYOUT):
        if k is not None:
            return int(k)
    return int(ACTIVE_LAYOUT)


def screw_heads(k=None):
    """The two screw heads' top-face centres (world), left stud first."""
    y, z = LAYOUTS[layout_index() if k is None else k]
    return [np.array([WALL_X + SCREW_SET, yy, TOP + z]) for yy in (y, y - STUD_SPACING)]


# ---------------------------------------------------------------- the stud finder
# Magnetic stud finder (CH Hanson 03040 class: two rare-earth magnets, rotating two-way level, soft rubber body),
# in its own frame F: origin at the centre of its flat back, x out of the back (toward the wall when it is on the
# wall), y across, z along it toward its top. Body 82 long (two lobes 32 across joined by a 29-wide waist) and 20
# deep; yellow side tabs 7.5 long stand 2.5-4.5 proud of the body (38 across them); a yellow pointer loop below it
# makes it 90 long. 68 g (the maker's 0.15 lb). The magnets sit behind the two badges, 25 above and below centre.
LOBE_R, LOBE_Z = 0.016, 0.025
BODY_D = 0.020
WAIST_HALF = 0.0145
TAB_HALF, TAB_Z, TAB_H = 0.019, (LOBE_Z, 0.0, -LOBE_Z), 0.0075
TAB_X = (-0.018, -0.0015)          # the tabs stop 1.5 short of the back
POINTER = (-0.012, 0.0, -0.007, 0.007, -0.049, -0.041)
FINDER_TOP, FINDER_BOTTOM = LOBE_Z + LOBE_R, POINTER[4]
FINDER_MASS = 0.068
MAGNET_Z = (LOBE_Z, -LOBE_Z)       # upper, lower
SKIN = 0.0010                      # plastic between each magnet's face and the back

_finder_shapes = [
    _b(-BODY_D, 0.0, -WAIST_HALF, WAIST_HALF, -LOBE_Z, LOBE_Z),                          # 0 waist
    cyl((-BODY_D / 2, 0.0, LOBE_Z), LOBE_R, BODY_D, axis="x"),                            # 1 upper lobe
    cyl((-BODY_D / 2, 0.0, -LOBE_Z), LOBE_R, BODY_D, axis="x"),                           # 2 lower lobe
]
for _z in TAB_Z:                                                                           # 3-8 side tabs
    for _s in (1, -1):
        _inner = WAIST_HALF if _z == 0.0 else 0.0140
        _finder_shapes.append(_b(TAB_X[0], TAB_X[1], *sorted((_s * _inner, _s * TAB_HALF)), _z - TAB_H / 2,
                                 _z + TAB_H / 2, color=FINDER_YELLOW))
_finder_shapes += [
    cyl((-BODY_D - 0.0005, 0.0, LOBE_Z), 0.0105, 0.001, axis="x", color=FINDER_YELLOW),  # 9 upper badge
    cyl((-BODY_D - 0.0005, 0.0, -LOBE_Z), 0.0105, 0.001, axis="x", color=FINDER_YELLOW), # 10 lower badge
    cyl((-BODY_D - 0.0005, 0.0, 0.0), 0.0110, 0.001, axis="x", color=FINDER_YELLOW),     # 11 level ring
    _b(-BODY_D - 0.0012, -BODY_D - 0.0008, -0.006, 0.006, -0.004, 0.004, color=VIAL_GREEN),  # 12 level vial
    _b(*POINTER, color=FINDER_YELLOW),                                                    # 13 pointer loop
]
FINDER_SHAPES = tuple(_finder_shapes)

# The holder (made for the trial): grey ABS, a 6 mm base and a slot 25 x 44 x 28 deep that keeps the finder upright,
# back face toward the wall, standing on its pointer.
X_HOME, Y_HOME = 0.190, 0.300           # the finder's back-face centre (x, y) standing in the holder
HOLD_BASE, HOLD_TOP = 0.006, 0.034
SLOT_X = (-BODY_D - 0.003, 0.002)       # slot walls relative to X_HOME: 2 mm behind the back, 2 in front of the ring
SLOT_Y = 0.022
HOLDER = Part("holder", (X_HOME, Y_HOME, TOP), [
    _b(-0.050, 0.020, -0.040, 0.040, 0.0, HOLD_BASE),                                     # 0 base
    _b(-0.050, SLOT_X[0], -0.040, 0.040, HOLD_BASE, HOLD_TOP),                           # 1 front wall
    _b(SLOT_X[1], 0.020, -0.040, 0.040, HOLD_BASE, HOLD_TOP),                            # 2 back wall
    _b(SLOT_X[0], SLOT_X[1], SLOT_Y, 0.040, HOLD_BASE, HOLD_TOP),                        # 3 side wall
    _b(SLOT_X[0], SLOT_X[1], -0.040, -SLOT_Y, HOLD_BASE, HOLD_TOP),                      # 4 side wall
], color=HOLDER_GREY, friction=(0.50, 0.40), contact_offset=0.001)

FINDER_HOME = np.array([X_HOME, Y_HOME, TOP + HOLD_BASE - FINDER_BOTTOM])
FINDER = Part("finder", FINDER_HOME + [0.0, 0.0, 0.0005], FINDER_SHAPES, mass=FINDER_MASS, color=FINDER_BLACK,
              friction=(0.40, 0.35), contact_offset=0.001, grip=0)

OBJECTS = [WALL, HOLDER, FINDER]
COM_F = FINDER.volume_centroid()          # PhysX: uniform density over the colliders


# ---------------------------------------------------------------- geometry the scorer reads
def magnet_faces(finder):
    """World centres of the two magnets' working faces (upper, lower) and the axis out of the back."""
    R = finder.rotation()
    return [finder.to_world((-SKIN, 0.0, z)) for z in MAGNET_Z], R[:, 0]


BACK_POINTS = [(0.0, s * WAIST_HALF, z) for s in (1, -1) for z in MAGNET_Z]   # beside both badges


def back_gaps(finder):
    """How far each of the four back points stands off the wall's face plane (m; negative: pressed into it)."""
    return [WALL_X - float(finder.to_world(p)[0]) for p in BACK_POINTS]


FLAT_GAP = 0.002
ON_SCREW = 0.010


def flat_on_wall(finder):
    """The back lies flat on the wall: all four points within 2 mm of its face plane, the finder within the
    wall's length and its lower badge below the wall's top."""
    gaps = back_gaps(finder)
    lower = magnet_faces(finder)[0][1]
    return bool(all(-FLAT_GAP <= g <= FLAT_GAP for g in gaps) and WALL_Y0 <= lower[1] <= WALL_Y1
                and lower[2] <= TOP + WALL_H)


def screw_offset(finder):
    """The smallest distance, measured in the wall's plane, from either magnet's centre to either screw's axis."""
    faces, _ = magnet_faces(finder)
    return min(math.hypot(f[1] - h[1], f[2] - h[2]) for f in faces for h in screw_heads())


def on_screw(finder):
    return flat_on_wall(finder) and screw_offset(finder) <= ON_SCREW


def off_supports(finder):
    """Not standing on the table or in the holder: its lowest point at least 20 mm above the table."""
    return bool(finder.low[2] >= TOP + 0.020)


# ---------------------------------------------------------------- stillness over a window of poses
POS_STILL, ANG_STILL_DEG = 0.001, 2.0
_WINDOW = {"episode": None, "samples": []}


def _angle_deg(a, b):
    return math.degrees(math.acos(float(np.clip((np.trace(a.T @ b) - 1) / 2, -1, 1))))


def still(finder):
    """Moved under 1 mm and 2 degrees over the last second (pose samples, one per control step). Offline, with no
    history, the state's own speeds decide."""
    if _RUN["t"] is None:
        return bool(finder.resting(max_speed=0.002, max_spin=0.05))
    if _WINDOW["episode"] != _RUN["episode"]:
        _WINDOW.update(episode=_RUN["episode"], samples=[])
    step, t = _RUN["step"], float(_RUN["t"])
    samples = _WINDOW["samples"]
    if not samples or samples[-1][0] != step:
        if samples and (samples[-1][0] + 1 != step or t <= samples[-1][1]):
            samples.clear()
        samples.append((step, t, np.array(finder.origin, float), finder.rotation()))
        while len(samples) > 1 and samples[1][1] <= t - STILL_S + 1e-9:
            samples.pop(0)
    if samples[0][1] > t - STILL_S + 1e-9:
        return False
    return all(np.linalg.norm(a[2] - b[2]) <= POS_STILL + 1e-12 and _angle_deg(a[3], b[3]) <= ANG_STILL_DEG + 1e-9
               for i, a in enumerate(samples) for b in samples[:i])


# ---------------------------------------------------------------- score
def score(objects):
    finder = objects["finder"]
    flat = flat_on_wall(finder)
    screw = flat and screw_offset(finder) <= ON_SCREW
    free = released(finder)
    up = off_supports(finder)
    hanging = screw and free and up
    hang_s = held_for("hanging", hanging)      # timers run on every call
    calm = still(finder)
    if hanging and hang_s + 1e-9 >= STILL_S and calm:
        return 1.0
    if screw and up:
        return 0.5                              # on a screw, but still held (or not yet still)
    if flat and up and not free:
        return 0.25                             # held flat on the wall, not over a screw
    return 0.0


def success(objects):
    return score(objects) >= 1.0


# ---------------------------------------------------------------- offline fixtures (tests only; the demo never uses them)
def rot(axis, deg):
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    return {"x": np.array([[1, 0, 0], [0, c, -s], [0, s, c]]), "y": np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]]),
            "z": np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])}[axis]


def on_wall_origin(magnet_y, magnet_z, which=1, gap=0.0):
    """F's origin that puts magnet `which` (0 upper, 1 lower) at (y, z above the table) with the back `gap` off
    the wall (finder upright)."""
    return np.array([WALL_X - gap, magnet_y, TOP + magnet_z - MAGNET_Z[which]])


# ---------------------------------------------------------------- scripted demo (left arm; the right arm waits)
# The left arm lifts the finder out of its holder by its middle side tabs, presses its back lightly onto the wall's
# face, and sweeps it along the wall in two rows 13 mm apart. The hand is pitched 22 degrees about the finger axis,
# its top leaning back toward the robot: the YAM's fingers taper toward their tips, and at this pitch the whole hand
# (fingers, pads and palm) stays in front of the wall's face, so the finder works anywhere on the face rather than
# only at the wall's top. The demo reads the finder's pose and where this trial's screws are (simulator state, as a
# scripted demo may); a person feels the tug instead. Where the lower magnet comes over a screw it stops, lets go,
# and backs away; the magnets hold the finder on the wall.
GRIP_PITCH = math.radians(22.0)                # the hand's approach tipped toward the wall (its top away from it)
TCP_IN_F = np.array([-0.0143, 0.0, 0.0040])    # the pads' flat faces cover about 90% of the middle tabs' sides;
HAND_FRONT = 0.0113                            # the hand's front-most point (URDF collision meshes, pitched) is
                                               # this far ahead of the TCP, so it stays 3 mm behind the back
V0 = -TCP_IN_F                                 # F's origin minus the TCP, nominal
LOWER_FACE_F = np.array([-SKIN, 0.0, MAGNET_Z[1]])
PRESS_WALL = 0.0025                            # back commanded this far into the wall (about 1 N with the arm's give)
RELEASE_PRESS = 0.0005                         # eased to this before the fingers open
STANDOFF = 0.015                               # back this far off the wall while approaching
ROWS = (0.1265, 0.1135)                        # the lower magnet's height above the table on each pass
SWEEP_Y = (0.265, 0.085)                       # the lower magnet's y along the first pass (the second goes back)
SWEEP_STEPS, SWEEP_STEP_S = 20, 0.35           # 9 mm a step, about 26 mm/s
CAPTURE_DZ = 0.004                             # a pass this close to a screw's height stops over it
SKIPPED_S = 0.05
_CLAMP = 0.020                                 # the reads correct up to 20 mm (the grip slides ~16 mm on the lift)
LET_GO_MID = -0.022                            # the fingers first open to 22 mm each while the hand centres itself
_DEMO = {}


def _finder_now():
    env = _RUN["env"]
    if env is None:
        return None
    try:
        from yam_arena import scene as arena_scene
        return arena_scene.object_states(env, ["finder"])["finder"]
    except Exception as e:  # noqa: BLE001 - an unreadable pose falls back to the open-loop plan
        print(f"STUD_READ failed: {e!r}", flush=True)
        return None


class Grip(float):
    """A grip yaw (rad) that also carries a pitch (rad) about the horizontal finger axis, through the TCP."""

    def __new__(cls, yaw, pitch=0.0):
        value = super().__new__(cls, yaw)
        value.grip_pitch = float(pitch)
        return value

    def __repr__(self):
        return f"Grip({float(self)!r}, pitch={self.grip_pitch!r})"


def _install_grip_pitch():
    """Let a Grip tip the straight-down grip about its own finger axis; a plain float yaw is unchanged."""
    from yam_arena import demo as arena_demo
    from yam_arena.urdf import axis_angle_matrix

    original = arena_demo.grip_rotation
    if getattr(original, "_grip_pitch", False):
        return

    def grip_rotation(yaw=0.0):
        rotation = original(float(yaw))
        pitch = float(getattr(yaw, "grip_pitch", 0.0))
        if pitch == 0.0:
            return rotation
        axis = np.array([-math.sin(float(yaw)), math.cos(float(yaw)), 0.0])  # the finger axis, horizontal
        return axis_angle_matrix(axis, pitch) @ rotation

    grip_rotation._grip_pitch = True
    grip_rotation.__wrapped__ = original
    arena_demo.grip_rotation = grip_rotation


_install_grip_pitch()


class LazyWaypoint(Waypoint):
    """A Waypoint whose target and yaw are worked out when it is first played (they may use a pose read then), and
    that takes almost no time once the search it belongs to has ended. Every grip is pitched GRIP_PITCH."""

    def __init__(self, duration, where, finger, label, yaw_fn=None, keep=False, skip=None):
        self._duration = duration * (1.0 if keep else DEMO_PACE)
        self.where, self.finger, self.label, self._yaw_fn, self._skip = where, finger, label, yaw_fn, skip
        self._pos = None

    @property
    def duration(self):
        return SKIPPED_S if self._skip is not None and self._skip() else self._duration

    @property
    def pos(self):
        if self._pos is None:
            self._pos = tuple(float(v) for v in self.where())
        return self._pos

    @property
    def yaw(self):
        return Grip(0.0 if self._yaw_fn is None else float(self._yaw_fn()), -GRIP_PITCH)


def demo():
    """Left arm: finder out of its holder, onto the wall, sweep until the lower magnet is over a screw, let go."""
    plan = []
    S = _DEMO
    S.clear()
    S.update(start=None, v=None, yaw=0.0, mag=None, hold=None, captured_at=None, tcp=None, centre=None)

    def start_shift():
        """Where the finder actually stands, minus where the holder puts it (x, y; read once, before the pick)."""
        if S["start"] is None:
            S["start"] = np.zeros(3)
            f = _finder_now()
            if f is not None:
                d = np.asarray(f.origin, float) - FINDER_HOME
                d[2] = 0.0
                S["start"] = np.clip(d, -_CLAMP, _CLAMP)
                print(f"STUD_START layout={layout_index()} shift_mm=({d[0] * 1000:.2f},{d[1] * 1000:.2f}) "
                      f"tilt_deg={f.tilt_deg():.2f}", flush=True)
        return S["start"]

    def read_hand():
        """Read once, as the carry starts: F's origin relative to the commanded hand (hand's unturned frame), and
        the hand yaw that squares the finder's back to the wall."""
        if S["v"] is None:
            S["v"], S["yaw"] = V0.copy(), 0.0
            f = _finder_now()
            if f is not None and S["tcp"] is not None:
                v = np.asarray(f.origin, float) - np.asarray(S["tcp"], float)
                R = f.rotation()
                yaw = math.atan2(R[1, 0], R[0, 0])
                S["v"] = V0 + np.clip(v - V0, -_CLAMP, _CLAMP)
                S["yaw"] = float(np.clip(-yaw, -math.radians(4), math.radians(4)))
                pitch = math.degrees(math.atan2(-R[2, 0], R[0, 0]))
                print(f"STUD_IN_HAND offset_mm=({(v - V0)[0] * 1000:.2f},{(v - V0)[1] * 1000:.2f},"
                      f"{(v - V0)[2] * 1000:.2f}) yaw_deg={math.degrees(yaw):.2f} pitch_deg={pitch:.2f}", flush=True)
        return S["v"], S["yaw"]

    def tcp_for_magnet(mag):
        """The hand target that puts the lower magnet's face centre at `mag` (world), the finder square to the wall."""
        v, yaw = read_hand()
        origin = np.asarray(mag, float) - LOWER_FACE_F
        return origin - rot("z", math.degrees(yaw)) @ v

    def captured():
        return S["captured_at"] is not None

    def add(dt, where, label, finger=FINGER_CLOSED, keep=False, turned=False, skip=None):
        def fn(w=where):
            p = np.asarray(w(), float)
            S["tcp"] = p
            return p
        plan.append(LazyWaypoint(dt, fn, finger, label, (lambda: S["yaw"]) if turned else None, keep, skip))

    def M(y, z, x=WALL_X + PRESS_WALL):
        """A lower-magnet target: y, height above the table, and the back's x."""
        return np.array([x - SKIN, y, TOP + z])

    pick = lambda dz=0.0: FINDER_HOME + start_shift() + TCP_IN_F + [0.0, 0.0, dz]  # noqa: E731
    # From the ready pose (hand over the wall's top) straight back toward the robot at its height, then down over the
    # holder.
    add(1.4, lambda: np.array([X_HOME + 0.010, Y_HOME, TOP + 0.265]), "back from the wall", FINGER_OPEN)
    add(1.2, lambda: pick(0.060), "over the finder", FINGER_OPEN)
    add(1.2, lambda: pick(), "hand down beside the finder", FINGER_OPEN)
    add(0.8, lambda: pick(), "close on the side tabs", keep=True)
    add(1.4, lambda: pick(0.060), "lift out of the holder")
    add(2.0, lambda: tcp_for_magnet(M(SWEEP_Y[0], ROWS[0], WALL_X - STANDOFF)), "carry to the wall", turned=True)
    add(1.0, lambda: tcp_for_magnet(M(SWEEP_Y[0], ROWS[0])), "back onto the wall", keep=True, turned=True)
    add(0.4, lambda: tcp_for_magnet(M(SWEEP_Y[0], ROWS[0])), "settle on the wall", keep=True, turned=True)
    S["mag"] = M(SWEEP_Y[0], ROWS[0])

    def sweep_point(y, z, index):
        def where():
            if captured():
                return S["hold"]
            prev, goal = S["mag"], M(y, z)
            heads = [h for h in screw_heads() if abs(h[2] - (TOP + z)) <= CAPTURE_DZ]
            for h in heads:
                lo, hi = sorted((prev[1], goal[1]))
                if lo - 1e-6 <= h[1] <= hi + 1e-6 and abs(prev[2] - goal[2]) < 1e-9:
                    goal = M(h[1], z)
                    S["captured_at"] = index
                    print(f"STUD_CAPTURE layout={layout_index()} step={index} magnet_target_mm=({h[1] * 1000:.1f},"
                          f"{z * 1000:.1f}) screw_mm=({h[1] * 1000:.1f},{(h[2] - TOP) * 1000:.1f})", flush=True)
                    break
            S["mag"] = goal
            tcp = tcp_for_magnet(goal)
            if captured():
                S["hold"] = tcp
            return tcp
        return where

    index = 0
    for row, (y0, y1) in enumerate(((SWEEP_Y[0], SWEEP_Y[1]), (SWEEP_Y[1], SWEEP_Y[0]))):
        z = ROWS[row]
        if row == 1:
            index += 1
            add(0.8, sweep_point(y0, z, index), "slide down to the second pass", turned=True,
                skip=lambda i=index: captured() and S["captured_at"] < i)
        for k in range(1, SWEEP_STEPS + 1):
            index += 1
            y = y0 + (y1 - y0) * k / SWEEP_STEPS
            add(SWEEP_STEP_S, sweep_point(y, z, index), "sweep the wall", turned=True,
                skip=lambda i=index: captured() and S["captured_at"] < i)

    def held():
        """Where the search stopped, with the press into the wall eased to 0.5 mm: the magnets hold the finder on
        the wall, and the opening hand does not spring toward the wall by the press."""
        p = S["hold"] if S["hold"] is not None else S["tcp"]
        return np.asarray(p, float) - [PRESS_WALL - RELEASE_PRESS, 0.0, 0.0]

    def centred():
        """Read once, as the fingers start to open: the hand's place with the finder in the middle of its fingers.
        The YAM's two fingers are driven separately toward one target, so the grip can sit off the hand's middle
        (it slides about 16 mm along the finger axis as the finder is lifted). Opening then, the far finger, still
        short of its target, would push the finder that far along the wall, off the screw. So while the fingers open
        to 22 mm each the hand moves along the finger axis by the finder's offset, and the finder stays put."""
        if S["centre"] is None:
            p = held()
            S["centre"] = p
            f = _finder_now()
            if f is not None:
                hands = grippers()
                tcp = np.asarray(hands["left"][0] if hands else S["tcp"], float)
                yaw = S["yaw"]
                axis = np.array([-math.sin(yaw), math.cos(yaw), 0.0])  # the finger axis
                off = float((np.asarray(f.origin, float) - tcp - rot("z", math.degrees(yaw)) @ V0) @ axis)
                S["centre"] = p + float(np.clip(off, -_CLAMP, _CLAMP)) * axis
                print(f"STUD_LET_GO offset_along_fingers_mm={off * 1000:.2f} screw_offset_mm={screw_offset(f) * 1000:.2f}",
                      flush=True)
        return S["centre"]

    add(0.6, held, "dwell on the screw", keep=True, turned=True)
    add(1.2, centred, "let go", LET_GO_MID, keep=True, turned=True)
    add(1.0, centred, "open the fingers", FINGER_OPEN, keep=True, turned=True)
    add(1.0, lambda: centred() + [-0.050, 0.0, 0.0], "back away from the wall", FINGER_OPEN, turned=True)
    add(1.4, lambda: np.array([0.150, 0.400, TOP + 0.150]), "park", FINGER_OPEN)
    final = plan[-1]
    plan.append(LazyWaypoint(3.0, final.where, FINGER_OPEN, "stillness tail", keep=True))
    return {"left": plan}


import stud_runtime  # noqa: E402

stud_runtime.install(sys.modules.get(__name__) or globals())
