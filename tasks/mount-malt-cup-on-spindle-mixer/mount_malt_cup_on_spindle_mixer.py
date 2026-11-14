"""Mount a filled 32 oz stainless malt cup on a single-spindle drink mixer so that the mixer runs.

The mixer stands on the counter in front of the right arm, turned to face it, column at the back: a single-spindle
commercial drink mixer (20.5 x 6.5 x 6.75 in, die-cast housing). A steel spindle hangs from the head with a solid
agitator at its lower end. Under the head, behind the spindle, hangs the cup guide: a curved plate that sits just
inside the cup's mouth, with the switch pin (ON/OFF actuator) above the gap behind it. On the column face below the
agitator is the two-position cup rest: the upper (run) step and, 8 mm lower and further out, the lower (standby)
step.

To mix, the cup is tipped so its mouth takes the agitator in, lifted up around the spindle with its back rim going
up behind the guide plate, swung upright over the rest and set down. On the upper step the rim holds the switch pin
up and the motor runs; on the lower step the cup hangs in the guide but the pin is out and the motor is off.

The filled cup (the maker's recipe: 3 No. 24 scoops of ice cream, 4 oz milk, 1.5 oz syrup; 0.58 kg in all) stands
on the counter to the right of the mixer. Metres; x forward from the arm bases, y left, z up; zero jitter for demos.
"""
import math
import pathlib
import sys
from types import SimpleNamespace

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
from yam_parts import PartState, box, cyl  # noqa: E402
from yam_joints import Hinge, Link, Mechanism, Slide  # noqa: E402
from yam_mesh import Mesh, MeshMechanism, MeshPart, prism  # noqa: E402
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
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


TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S, SCORE_OVER = 1.0, "end"
RUN_LIMIT_STEPS = 3600  # 180 s at 20 Hz
DEMO_PACE = 1.0  # scales every demo duration (0.8 for the timing-margin copy)

STEEL, CHROME, HOUSING, HEAD_GREY = (0.50, 0.53, 0.58), (0.84, 0.85, 0.87), (0.50, 0.51, 0.53), (0.58, 0.59, 0.61)
WHITE, MIX_BROWN = (0.93, 0.93, 0.91), (0.45, 0.28, 0.16)  # the mix: ice cream with chocolate syrup

# ---------------------------------------------------------------- the cup: 32 oz stainless malt cup (universal mixer cup)
# Part frame: origin at the centre of the cup's bottom face, z up its axis.
CUP_H = 0.1806  # 7.11 in tall
CUP_R_TOP = 0.04826  # 3.80 in across the top, below the rolled lip
CUP_R_LIP = 0.05118  # 4.03 in across the rolled lip
CUP_R_BASE = 0.0350  # 70 mm across the base (scaled from the maker's photo of two mounted cups; not published)
LIP_H = 0.0030  # the rolled lip's height
WALL = 0.0010  # modelled wall (the real sheet is about 0.6 mm; the outside surface is true)
N_STAVES = 32
BOTTOM_T = 0.0010
CUP_MASS = 0.58  # cup about 0.28 kg + the recipe about 0.30 kg (all estimates)
BALLAST_T = 0.0074  # the mix's mass rides in a rigid layer in the base: it puts the centre of mass 64 mm up
CUP_FRICTION = (0.30, 0.25)  # stainless (estimate)
CUP_CONTACT_OFFSET = 0.0010
R_IN_TOP = CUP_R_TOP - WALL
CUP_TAPER = (CUP_R_TOP - CUP_R_BASE) / CUP_H
# The maker's recipe: 3 No. 24 scoops of ice cream (1/24 qt each), 4 fl oz milk, 1.5 fl oz syrup: 281 ml. Treated as a
# liquid in the true 0.6 mm-wall cup, its surface reaches the lip once the cup is tipped about 69 degrees.
MIX_ML = 3 * 946.353 / 24 + 4 * 29.5735 + 1.5 * 29.5735
SPILL_DEG = 65.0  # sticky spill: tipped past 65 degrees (4 degrees short of the static 69, for slosh)


def cup_r_out(h):
    """Outside radius of the cup at height h above its bottom (the lip at the top)."""
    h = np.asarray(h, float)
    return np.where(h < CUP_H - LIP_H, CUP_R_BASE + CUP_TAPER * h, CUP_R_LIP)


def _stave(name, phi, half, r0_out, r1_out, r0_in, r1_in, z0, z1, color):
    """One flat wall panel as a convex hexahedron: outer and inner faces are planar trapezoids whose centre lines
    sit at the given (apothem) radii, between heights z0 and z1."""
    c = 1.0 / math.cos(half)
    pts = []
    for z, r_out, r_in in ((z0, r0_out, r0_in), (z1, r1_out, r1_in)):
        for r, a in ((r_out, phi - half), (r_out, phi + half), (r_in, phi + half), (r_in, phi - half)):
            pts.append((r * c * math.cos(a), r * c * math.sin(a), z))
    faces = [(0, 2, 1), (0, 3, 2), (4, 5, 6), (4, 6, 7)]
    for i in range(4):
        j = (i + 1) % 4
        faces += [(i, j, j + 4), (i, j + 4, i + 4)]
    return Mesh(name, np.array(pts), np.array(faces), collision="convex", color=color)


def _cup_meshes():
    half = math.pi / N_STAVES
    out = []
    zw = CUP_H - LIP_H
    for k in range(N_STAVES):
        phi = 2 * math.pi * k / N_STAVES  # a panel's centre faces +x (and so +-y, the grip axis) at k = 0, 8, 16, 24
        out.append(_stave(f"wall_{k:02d}", phi, half, CUP_R_BASE, CUP_R_BASE + CUP_TAPER * zw,
                          CUP_R_BASE - WALL, CUP_R_BASE + CUP_TAPER * zw - WALL, BOTTOM_T * 0.5, zw, STEEL))
        out.append(_stave(f"lip_{k:02d}", phi, half, CUP_R_LIP, CUP_R_LIP, R_IN_TOP, R_IN_TOP, zw, CUP_H, CHROME))
    return tuple(out)


# Both the cup and the mixer sit 30 degrees off the line straight out from the right arm's base, the cup to its right
# and the mixer toward the middle, so that neither is in the arms' ready pose (the right hand's ready pose is right
# where a mixer straight ahead would hang its spindle).
CUP_BEARING = math.radians(-30.0)
CUP_START = (0.347 * math.cos(CUP_BEARING), -0.305 + 0.347 * math.sin(CUP_BEARING), TOP)  # bottom centre
START_YAW = CUP_BEARING  # the hand comes in radially from the right arm
CUP = MeshPart(
    "cup", CUP_START, yaw=START_YAW,  # turned so flat wall panels face the closing pads
    shapes=[cyl((0.0, 0.0, BOTTOM_T / 2), CUP_R_BASE - WALL * 0.5, BOTTOM_T, color=STEEL),  # bottom
            cyl((0.0, 0.0, BOTTOM_T + BALLAST_T / 2), CUP_R_BASE - WALL - 0.0005, BALLAST_T, color=MIX_BROWN)],
    mass=CUP_MASS, color=STEEL, friction=CUP_FRICTION, contact_offset=CUP_CONTACT_OFFSET, grip=1,
    meshes=_cup_meshes())

# ---------------------------------------------------------------- the mixer: single-spindle drink mixer, fixed
# Mixer frame: origin on the counter under the spindle axis; +x toward the column (away from the robot). Its suction
# feet hold it to the counter, so the base is fixed.
MIXER_YAW = math.radians(30.0)  # turned to face the right arm's base: its +x points straight away from it
XS, YS = 0.320 * math.cos(MIXER_YAW), -0.305 + 0.320 * math.sin(MIXER_YAW)  # the spindle axis, 0.32 m out
BASE_FRONT, BASE_BACK, BASE_HALF_W, BASE_TOP = -0.060, 0.111, 0.0825, 0.025
COLUMN_FACE, COLUMN_HALF_W = 0.060, 0.060
HEAD_BOTTOM, HEAD_TOP, HEAD_FRONT = 0.375, 0.521, -0.075  # 20.5 in overall
SHAFT_R, CHUCK_R, CHUCK_Z0 = 0.004, 0.010, 0.340
AGITATOR_R, AGITATOR_Z0, AGITATOR_Z1 = 0.0135, 0.208, 0.214
AGITATOR_MID = (AGITATOR_Z0 + AGITATOR_Z1) / 2
SPINDLE_MASS, SPINDLE_DAMP = 0.10, 0.002
# Cup rest (J bracket on the column face): the run step against the column, the standby step 8 mm lower in front.
SEAT_RUN, SEAT_STANDBY = 0.160, 0.152
STEP_RUN_X = (0.026, COLUMN_FACE - 0.004)
STEP_STANDBY_X = (0.012, 0.026)
REST_HALF_W, REST_BOTTOM, REST_BACK_TOP = 0.009, 0.130, 0.178
# Cup guide: a curved plate hanging inside the seated cup's mouth, its back face 3 mm in front of the cup's inner wall
# (stepped to follow the wall's curve), lower edge 14.6 mm below the run-step rim.
SEATED_AXIS_X = 0.005  # where the demo seats the cup's axis (upright: the pin's push and friction hold it there)
PLATE_DESIGN_AXIS_X = 0.002
PLATE_LOW, PLATE_T, PLATE_GAP = 0.326, 0.003, 0.003
PLATE_SEGMENTS = ((0.000, 0.006), (0.006, 0.012), (0.012, 0.018))  # |y| ranges, each a flat segment


def plate_back_x(y_outer):
    return PLATE_DESIGN_AXIS_X + math.sqrt(R_IN_TOP ** 2 - y_outer ** 2) - PLATE_GAP


PLATE_BACK = plate_back_x(PLATE_SEGMENTS[0][1])  # the centre segment's back face
# Switch pin (ON/OFF actuator): slides up in the guide; the switch closes after 2 mm of lift. It spans the slot behind
# the plate far enough back to stay over the rim of a seated cup, upright (rim x 52-56) or leaned onto the plate.
PIN_X, PIN_HALF_W, PIN_LEN = (0.0470, 0.0560), 0.004, 0.020
PIN_Z0 = SEAT_RUN + CUP_H - 0.004  # its lower end, at rest: 4 mm below the run-step rim, 4 mm above the standby rim
PIN_TRAVEL, PIN_ON = 0.012, 0.002
PIN_K, PIN_REST, PIN_DAMP, PIN_MASS = 120.0, -0.010, 2.0, 0.004  # about 1.2 N at rest, 1.7 N held 4 mm up (estimates)
MIXER_FRICTION = (0.30, 0.25)
MIXER_CONTACT_OFFSET = 0.0010


def _b(x0, x1, y0, y1, z0, z1, color=None):
    return box(((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2), (x1 - x0, y1 - y0, z1 - z0), color=color)


def _mixer_base():
    rf = BASE_HALF_W
    shapes = [
        # base: a slab with a rounded front
        _b(BASE_FRONT + rf, BASE_BACK, -BASE_HALF_W, BASE_HALF_W, 0.0, BASE_TOP, HOUSING),
        cyl((BASE_FRONT + rf, 0.0, BASE_TOP / 2), rf, BASE_TOP, color=HOUSING),
        # column at the back, with rounded front edges
        _b(COLUMN_FACE + 0.012, BASE_BACK, -COLUMN_HALF_W, COLUMN_HALF_W, BASE_TOP, HEAD_BOTTOM, HOUSING),
        _b(COLUMN_FACE, COLUMN_FACE + 0.012, -COLUMN_HALF_W + 0.012, COLUMN_HALF_W - 0.012, BASE_TOP, HEAD_BOTTOM,
           HOUSING),
        cyl((COLUMN_FACE + 0.012, COLUMN_HALF_W - 0.012, (BASE_TOP + HEAD_BOTTOM) / 2), 0.012,
            HEAD_BOTTOM - BASE_TOP, color=HOUSING),
        cyl((COLUMN_FACE + 0.012, -COLUMN_HALF_W + 0.012, (BASE_TOP + HEAD_BOTTOM) / 2), 0.012,
            HEAD_BOTTOM - BASE_TOP, color=HOUSING),
        # cup rest: back plate on the column face, run step, standby step
        _b(STEP_RUN_X[1], COLUMN_FACE, -REST_HALF_W, REST_HALF_W, REST_BOTTOM, REST_BACK_TOP, CHROME),
        _b(STEP_RUN_X[0], STEP_RUN_X[1], -REST_HALF_W, REST_HALF_W, REST_BOTTOM, SEAT_RUN, CHROME),
        _b(STEP_STANDBY_X[0], STEP_STANDBY_X[1], -REST_HALF_W, REST_HALF_W, REST_BOTTOM, SEAT_STANDBY, CHROME),
    ]
    # cup guide plate segments, hanging from the head
    for y0, y1 in PLATE_SEGMENTS:
        back = plate_back_x(y1)
        for s in (-1.0, 1.0):
            lo, hi = sorted((s * y0, s * y1))
            shapes.append(_b(back - PLATE_T, back, lo, hi, PLATE_LOW, HEAD_BOTTOM, CHROME))
    return shapes


def _head_mesh():
    """The head, a slab with a rounded front, as one convex mesh. It has no offline proxy: the offline checker models
    a straight-down hand at the commanded TCP, and this demo's hand is pitched and commanded high to offset the cup's
    weight, so the checker would see the head where the hand is not; the task's own geometry check covers the head."""
    rf, cx = BASE_HALF_W, HEAD_FRONT + BASE_HALF_W
    arc = [(cx + rf * math.cos(a), rf * math.sin(a)) for a in np.radians(np.linspace(90.0, 270.0, 25))]
    outline = [(BASE_BACK, -rf), (BASE_BACK, rf), *arc]
    m = prism(outline, HEAD_TOP - HEAD_BOTTOM, name="head", proxy=())
    return Mesh("head", m.points + np.array([0.0, 0.0, (HEAD_BOTTOM + HEAD_TOP) / 2]), m.faces, collision="convex",
                color=HEAD_GREY)


SPINDLE = Link(
    "spindle",
    [cyl((0.0, 0.0, (AGITATOR_Z1 + HEAD_BOTTOM) / 2), SHAFT_R, HEAD_BOTTOM - AGITATOR_Z1),
     cyl((0.0, 0.0, (CHUCK_Z0 + HEAD_BOTTOM) / 2), CHUCK_R, HEAD_BOTTOM - CHUCK_Z0),
     cyl((0.0, 0.0, AGITATOR_MID), AGITATOR_R, AGITATOR_Z1 - AGITATOR_Z0)],
    Hinge("z", (0.0, 0.0, 0.300), -180.0, 180.0, damping=SPINDLE_DAMP), SPINDLE_MASS, color=CHROME)
PIN = Link(
    "pin",
    [_b(PIN_X[0], PIN_X[1], -PIN_HALF_W, PIN_HALF_W, PIN_Z0, PIN_Z0 + PIN_LEN)],
    Slide("z", ((PIN_X[0] + PIN_X[1]) / 2, 0.0, PIN_Z0 + PIN_LEN / 2), 0.0, PIN_TRAVEL, damping=PIN_DAMP,
          spring=PIN_K, rest=PIN_REST), PIN_MASS, color=WHITE)
MIXER = MeshMechanism("mixer", (XS, YS, TOP), _mixer_base(), [SPINDLE, PIN], yaw=MIXER_YAW, color=HOUSING,
                      friction=MIXER_FRICTION, meshes={"base": (_head_mesh(),)},
                      contact_offset=MIXER_CONTACT_OFFSET)
OBJECTS = [MIXER, CUP]


# ---------------------------------------------------------------- geometry the scorer and the demo share
def _rot_z(psi):
    c, s = math.cos(psi), math.sin(psi)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


R_MIXER = _rot_z(MIXER_YAW)  # mixer frame to world


def to_mixer(p_world):
    return R_MIXER.T @ (np.asarray(p_world, float) - np.array([XS, YS, TOP]))


def to_world(p_mixer):
    return np.array([XS, YS, TOP]) + R_MIXER @ np.asarray(p_mixer, float)


def mixer_xyz(x, y, z):
    """World position of a point given in the mixer frame (for the offline layouts)."""
    return tuple(float(v) for v in to_world((x, y, z)))


def cup_frame(cup):
    """(bottom centre in the mixer frame, rotation matrix) of a cup state."""
    return to_mixer(cup.origin), R_MIXER.T @ np.asarray(cup.rotation(), float)


def tilt_deg(R):
    return math.degrees(math.acos(float(np.clip(R[2, 2], -1.0, 1.0))))


def back_rim_top(B, R):
    """The rim's inner top edge on the side toward the column (mixer frame): the rim point whose direction from the
    cup's axis points most toward +x."""
    phi = math.atan2(R[0, 1], R[0, 0])
    return B + R @ np.array([R_IN_TOP * math.cos(phi), R_IN_TOP * math.sin(phi), CUP_H])


def agitator_in_cup(B, R):
    """The agitator's centre in the cup frame (radius from the cup's axis, height above its bottom)."""
    p = R.T @ (np.array([0.0, 0.0, AGITATOR_MID]) - B)
    return float(math.hypot(p[0], p[1])), float(p[2])


# ---------------------------------------------------------------- scoring
SEAT_TOL_LOW, SEAT_TOL_HIGH = 0.002, 0.003  # bottom height on a step: 2 mm below (penetration) to 3 mm above
SEAT_TILT_DEG = 5.0  # seated: axis within 5 degrees of vertical
SEAT_Y_TOL = 0.010  # bottom centre within 10 mm of the spindle's line across the rest
RUN_OVERLAP = 0.002  # the bottom reaches at least 2 mm over the run step's front edge
RIM_IN_GUIDE_Z = 0.003  # the back rim top at least 3 mm above the guide plate's lower edge ...
RIM_BEHIND_X = 0.0015  # ... and behind its back face (1.5 mm contact tolerance)
SPINDLE_IN_BELOW_LIP = 0.040  # spindle in: the agitator at least 40 mm below the lip, inside the wall
SPINDLE_IN_WALL_GAP = 0.002
OFF_COUNTER = 0.030  # spindle-in credit needs the cup's bottom 30 mm above the counter
LINEAR_SPEED, ANGULAR_SPEED = 0.010, 0.5
TABLE_X, TABLE_Y = (-0.19, 0.91), (-0.65, 0.65)
FALL_DEPTH = 0.020


def _pin_q(mixer):
    return float(mixer.joints["pin"])


def seat_terms(cup, mixer):
    B, R = cup_frame(cup)
    tilt = tilt_deg(R)
    rim = back_rim_top(B, R)
    rim_in_guide = bool(rim[2] >= PLATE_LOW + RIM_IN_GUIDE_Z and rim[0] >= PLATE_BACK - RIM_BEHIND_X
                        and abs(rim[1]) <= PLATE_SEGMENTS[-1][1])
    centred_y = abs(B[1]) <= SEAT_Y_TOL
    upright = tilt <= SEAT_TILT_DEG
    # the bottom's back edge, in the vertical plane through the spindle (mixer frame x)
    back_edge_x = B[0] + CUP_R_BASE * math.cos(math.radians(tilt))
    on_run = bool(-SEAT_TOL_LOW <= B[2] - SEAT_RUN <= SEAT_TOL_HIGH and back_edge_x >= STEP_RUN_X[0] + RUN_OVERLAP)
    on_standby = bool(-SEAT_TOL_LOW <= B[2] - SEAT_STANDBY <= SEAT_TOL_HIGH
                      and back_edge_x >= STEP_STANDBY_X[0] + RUN_OVERLAP)
    pin = _pin_q(mixer)
    run = bool(on_run and upright and centred_y and rim_in_guide and pin >= PIN_ON)
    standby = bool(on_standby and upright and centred_y and rim_in_guide and pin < PIN_ON)
    r_ag, h_ag = agitator_in_cup(B, R)
    spindle_in = bool(h_ag <= CUP_H - SPINDLE_IN_BELOW_LIP and h_ag >= BOTTOM_T + BALLAST_T
                      and r_ag + AGITATOR_R + SPINDLE_IN_WALL_GAP <= CUP_R_BASE - WALL + CUP_TAPER * h_ag
                      and B[2] >= OFF_COUNTER)
    return {"tilt": tilt, "rim": rim, "rim_in_guide": rim_in_guide, "on_run": on_run, "on_standby": on_standby,
            "pin": pin, "run": run, "standby": standby, "spindle_in": spindle_in, "agitator": (r_ag, h_ag)}


def cup_bounds(cup):
    """The cup's world box from its true shape (its offline state's box covers only its bottom and mix layer)."""
    a = np.linspace(0.0, 2 * math.pi, 48, endpoint=False)
    ring = np.stack([np.cos(a), np.sin(a), np.zeros_like(a)], 1)
    pts = np.concatenate([ring * CUP_R_BASE, ring * CUP_R_LIP + [0.0, 0.0, CUP_H - LIP_H],
                          ring * CUP_R_LIP + [0.0, 0.0, CUP_H]])
    P = np.asarray(cup.origin, float) + pts @ np.asarray(cup.rotation(), float).T
    return SimpleNamespace(low=P.min(0), high=P.max(0))


def dropped_now(cup):
    p = np.asarray(cup.origin, float)
    return bool(p[2] < TOP - FALL_DEPTH or not TABLE_X[0] <= p[0] <= TABLE_X[1] or not TABLE_Y[0] <= p[1] <= TABLE_Y[1])


def score_terms(objects):
    cup, mixer = objects["cup"], objects["mixer"]
    seat = seat_terms(cup, mixer)
    spilled = ever("mix spilled: cup tipped past the spill angle", seat["tilt"] > SPILL_DEG)
    dropped = ever("cup off the counter", dropped_now(cup))
    still = bool(np.linalg.norm(cup.lin_vel) < LINEAR_SPEED and np.linalg.norm(cup.ang_vel) < ANGULAR_SPEED)
    free = released(cup_bounds(cup))
    run_done = held_for("seated on the run step, released, still", seat["run"] and free and still)
    standby_done = held_for("seated on the standby step, released, still", seat["standby"] and free and still)
    return {**seat, "spilled": spilled, "dropped": dropped, "released": free, "still": still,
            "run_s": run_done, "standby_s": standby_done}


def score(objects):
    t = score_terms(objects)
    if t["spilled"] or t["dropped"]:
        return 0.0
    if t["run"] and t["run_s"] + 1e-10 >= STILL_S:
        return 1.0
    if t["run"] or (t["standby"] and t["standby_s"] + 1e-10 >= STILL_S):
        return 0.5
    if t["standby"] or t["spindle_in"]:
        return 0.25
    return 0.0


def success(objects):
    return score(objects) >= 1.0


# ---------------------------------------------------------------- demo: open loop from the known start layout
class Grip(float):
    """A grip yaw (rad) that also carries a pitch (rad) about the horizontal finger axis, through the TCP, or a whole
    TCP rotation matrix (the weight-compensated commands)."""

    def __new__(cls, yaw, pitch=0.0, matrix=None):
        value = super().__new__(cls, yaw)
        value.grip_pitch = float(pitch)
        value.grip_matrix = None if matrix is None else np.array(matrix, float)
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
        matrix = getattr(yaw, "grip_matrix", None)
        if matrix is not None:
            return matrix.copy()
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


def _ry(theta):
    c, s = math.cos(theta), math.sin(theta)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def _rz(psi):
    c, s = math.cos(psi), math.sin(psi)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


# The grasp, fixed in the cup's frame: the hand comes in from the robot's side with the gripper pitched 20 degrees
# below horizontal (a horizontal grip at counter height is out of the arm's reach) and the pads closing across the
# cup's lower body, 52-60 mm up, just under the filled cup's centre of mass. The TCP sits 12 mm short of the axis, so
# the palm and upper fingers stay clear of the wider wall above (4.8 and 3.7 mm at the closest).
GRIP_BETA = math.radians(20.0)
GRIP_HC = 0.040  # the grasped diameter's height above the bottom
GRIP_S = -0.012
GRIP_OFFSET = np.array([GRIP_S * math.cos(GRIP_BETA), 0.0, GRIP_HC - GRIP_S * math.sin(GRIP_BETA)])
PARK_L, PARK_R = (0.100, 0.270, TOP + 0.160), (0.100, -0.300, TOP + 0.160)
GRIP_S_CLOSE = 1.9  # smoothstep grip ramp; the action guard allows 0.05 per 50 ms tick (0.8x too)
FINAL_HOLD_S = 4.0
APPROACH_BACK, APPROACH_UP = 0.070, math.radians(60.0)  # the open hand comes down onto the cup from behind and above
RETREAT = 0.095

# The arm is a stiff spring, not a rigid positioner: each joint is driven by PD with the stiffnesses below (gravity
# compensation covers the arm's own links, not a held object). The 0.58 kg cup in the hand bends the joints by tau/K
# (about 50 mm and 10 degrees at the mixer), so while the arm carries the cup the demo commands every pose that much
# past the wanted one: open-loop, from the arm's model, the cup's mass and centre of mass and the switch pin's spring.
RIGHT_BASE = (0.0, -0.305, 0.75)
JOINT_K = np.array([80.0, 80.0, 80.0, 10.0, 10.0, 10.0])  # N m/rad: joints 1-3 and 4-6
CUP_COM_H = 0.064  # the filled cup's centre of mass above its bottom (the walls' and the base layer's volumes)
GRAVITY = 9.81
RIM_PIN_R = (R_IN_TOP + CUP_R_LIP) / 2  # where the switch pin bears on the rim, from the cup's axis


# Where the closed hand holds the cup (measured on the GPU): both fingers close at their effort limit, so the cup is
# not centred between them; it ends against the pad of the finger that stays at its open stop, 8.2 mm off the TCP
# along the finger axis, and the grip then holds it there (no slip, under 0.4 degrees of turn) to the release. The
# carried poses allow for it, so the cup itself keeps to the planned path on the spindle's line.
HELD_SHIFT = np.array([0.0082, -0.0006, 0.0002])  # the held cup's origin from its centred place, TCP frame (x fingers)


def tcp_for(B_world, theta, yaw=0.0, held=False):
    """Wanted TCP position (world) and Grip for the cup's bottom centre at B_world, tipped theta (top toward +x of
    the heading `yaw`), the hand coming in along `yaw` (`held`: the closed hand, which holds the cup off-centre)."""
    R = _rz(yaw) @ _ry(theta)
    return (np.asarray(B_world, float) + R @ (_held_offset() if held else GRIP_OFFSET),
            Grip(yaw, -(math.pi / 2) + GRIP_BETA + theta))


def _grip0():
    from yam_arena import demo as arena_demo
    return arena_demo.grip_rotation(Grip(0.0, -(math.pi / 2) + GRIP_BETA))


def _held_offset():
    """The TCP in the held cup's frame."""
    return GRIP_OFFSET - _grip0() @ HELD_SHIFT


def cup_pose_from_tcp(p, R):
    """(bottom centre, rotation) of the held cup for a TCP pose (world)."""
    Rc = np.asarray(R, float) @ _grip0().T
    return np.asarray(p, float) - Rc @ _held_offset(), Rc


def pin_push(B_mixer, R):
    """The switch pin's downward push (N) on the rim for a cup pose in the mixer frame (0 when it is not lifted)."""
    rim = back_rim_top(B_mixer, R)
    lift = rim[2] - PIN_Z0
    under = PIN_X[0] - 0.002 <= rim[0] + (CUP_R_LIP - R_IN_TOP) and rim[0] <= PIN_X[1]
    return PIN_K * (min(lift, PIN_TRAVEL) - PIN_REST) if lift > 0.0 and under else 0.0


def weight_compensated(p, R, q_seed, pin=0.0):
    """(commanded TCP position, commanded rotation, wanted joints) for holding the cup at the TCP pose (p, R) while
    its weight and the pin's push `pin` (N, down on the back rim) bend the joints: q_cmd = q - J^T F / K."""
    from yam_arena import ik
    q, _, _ = ik.solve(p, ik.matrix_to_quat_xyzw(R), q_seed, base_pos=RIGHT_BASE, iters=300)

    def points(qq):
        pp, RR = ik.tcp_pose(qq, RIGHT_BASE)
        B, Rc = cup_pose_from_tcp(pp, RR)
        return B + Rc @ np.array([0.0, 0.0, CUP_COM_H]), B + Rc @ np.array([RIM_PIN_R, 0.0, CUP_H])

    com0, rim0 = points(q)
    tau = np.zeros(6)
    for j in range(6):
        dq = q.copy()
        dq[j] += 1e-6
        com1, rim1 = points(dq)
        tau[j] = (np.dot((com1 - com0) / 1e-6, [0.0, 0.0, -CUP_MASS * GRAVITY])
                  + np.dot((rim1 - rim0) / 1e-6, [0.0, 0.0, -pin]))
    p_cmd, R_cmd = ik.tcp_pose(q - tau / JOINT_K, RIGHT_BASE)
    return p_cmd, R_cmd, q


# The mount (mixer frame): keyframes of a path planned through the clear space (cup walls, lip and bottom and the
# gripper against every mixer part but the pin): the cup comes up tipped 30 degrees with its mouth under the agitator,
# rises around the spindle while it straightens, its back wall passing over the cup rest's steps (4 mm to spare); its
# rim goes up behind the guide plate with the bottom held out in front of both steps, lifts the switch pin, and the cup
# swings upright over the run step (2.5 mm to spare: the plate and the column leave the rim a 10 mm slot).
MOUNT = [  # (label, seconds, bottom centre, tip degrees)
    ("carry the cup to the mixer, tipped", 3.0, (-0.150, 0.055), 30.0),
    ("slide the mouth under the agitator", 1.4, (-0.1125, 0.0725), 30.0),
    ("take the agitator into the cup", 1.8, (-0.070, 0.1075), 23.0),
    ("rise and straighten around the spindle", 1.4, (-0.0425, 0.135), 15.0),
    ("rim under the guide plate", 1.0, (-0.030, 0.1475), 10.0),
    ("rim up behind the guide plate", 1.0, (-0.025, 0.1575), 10.0),
    ("rim past the plate's lower edge", 0.6, (-0.025, 0.160), 10.0),
    ("lift the pin while straightening", 1.8, (-0.005, 0.1675), 3.0),
    ("upright over the run step", 1.2, (SEATED_AXIS_X, SEAT_RUN + 0.0025), 0.0),
]
SEAT_PRESS = 0.0015  # the last command sets the bottom 1.5 mm into the run step


def mount_poses():
    """[(label, seconds, bottom centre (mixer frame), tip degrees)] while the arm carries the cup, in order."""
    out = [(label, secs, np.array([x, 0.0, z]), th) for label, secs, (x, z), th in MOUNT]
    out.append(("lower onto the run step", 1.4, np.array([SEATED_AXIS_X, 0.0, SEAT_RUN + 0.0005]), 0.0))
    return out


MOUNT_STEP, MOUNT_STEP_DEG = 0.003, 1.0
_PLAN = []


def demo_plan():
    """Right-arm waypoints as (label, seconds, commanded TCP, finger, Grip, cup carried by the arm)."""
    if not _PLAN:
        _PLAN.append(_demo_plan())
    return list(_PLAN[0])


def _demo_plan():
    O, C = FINGER_OPEN, FINGER_CLOSED
    start = np.array(CUP_START, float)
    heading = np.array([math.cos(START_YAW), math.sin(START_YAW), 0.0])
    grasp_p, grasp_g = tcp_for(start, 0.0, START_YAW)
    back = grasp_p + APPROACH_BACK * (-math.cos(APPROACH_UP) * heading + np.array([0.0, 0.0, math.sin(APPROACH_UP)]))
    plan = [("park", 1.5, np.array(PARK_R), O, 0.0, False),
            ("open hand above and behind the cup", 2.2, back, O, grasp_g, False),
            ("bring the pads down around the cup", 1.6, grasp_p, O, grasp_g, False),
            ("close on the cup", GRIP_S_CLOSE, grasp_p, C, grasp_g, False),
            ("settle the grip", 0.4, grasp_p, C, grasp_g, False)]
    from yam_arena import demo as arena_demo
    from yam_arena.constants import READY
    q = np.array(READY, float)

    def carried(label, secs, B_world, theta, yaw, pin=0.0):
        nonlocal q
        p, g = tcp_for(B_world, theta, yaw, held=True)
        p_cmd, R_cmd, q = weight_compensated(p, arena_demo.grip_rotation(g), q, pin)
        plan.append((label, secs, p_cmd, C, Grip(float(g), g.grip_pitch, R_cmd), True))

    carried("take the cup's weight", 1.0, start, 0.0, START_YAW)
    carried("lift the cup", 1.2, start + np.array([0.0, 0.0, 0.040]), 0.0, START_YAW)
    prev = None
    for label, secs, B, th in mount_poses():
        # in the mixer, each move is cut into short pieces, each with its own compensation, so the carried cup keeps
        # to the planned straight line between keyframes (the pin's push comes and goes along the way)
        n = 1 if prev is None else max(1, math.ceil(max(np.linalg.norm(B - prev[0]) / MOUNT_STEP,
                                                         abs(th - prev[1]) / MOUNT_STEP_DEG) - 1e-9))
        for k in range(1, n + 1):
            Bk, thk = (B, th) if prev is None else (prev[0] + k / n * (B - prev[0]), prev[1] + k / n * (th - prev[1]))
            carried(label, secs / n, to_world(Bk), math.radians(thk), MIXER_YAW,
                    pin_push(Bk, _ry(math.radians(thk))))
        prev = (B, th)
    seat_x = mount_poses()[-1][2][0]
    seat_p, seat_g = tcp_for(to_world(np.array([seat_x, 0.0, SEAT_RUN - SEAT_PRESS])), 0.0, MIXER_YAW, held=True)
    plan.append(("hand the weight to the run step", 1.2, seat_p, C, seat_g, False))
    plan.append(("let go", GRIP_S_CLOSE, seat_p, O, seat_g, False))
    out_dir = R_MIXER @ np.array([math.cos(GRIP_BETA), 0.0, -math.sin(GRIP_BETA)])
    plan += [("back the open hand out", 1.6, seat_p - RETREAT * out_dir, O, seat_g, False),
             ("park", 2.0, np.array(PARK_R), O, 0.0, False),
             ("hold clear", FINAL_HOLD_S, np.array(PARK_R), O, 0.0, False)]
    return plan


def _leaned_seat():
    """The cup set down upright on the run step at SEATED_AXIS_X, then tipped forward about the step's front edge
    (its centre of mass is in front of it) until its inner wall meets the guide plate: (bottom centre, world), lean
    (degrees), switch pin lift (m). Used by the offline layouts."""
    pivot = np.array([STEP_RUN_X[0], 0.0, SEAT_RUN])
    B0 = np.array([SEATED_AXIS_X, 0.0, SEAT_RUN])
    h = np.linspace(0.0, CUP_H, 400)
    wall = np.stack([CUP_R_BASE - WALL + CUP_TAPER * h, np.zeros_like(h), h], 1)
    for lean in np.radians(np.arange(0.0, 8.0, 0.01)):
        R = _ry(-lean)
        B = pivot + R @ (B0 - pivot)
        P = B + wall @ R.T
        if np.any((P[:, 2] >= PLATE_LOW) & (P[:, 0] <= PLATE_BACK)):
            break
    return to_world(B), math.degrees(lean), float(back_rim_top(B, R)[2] - PIN_Z0)


SEATED_POS, SEATED_LEAN_DEG, SEATED_PIN = _leaned_seat()


def demo():
    # Known start layout, the arm's model and time only: no sensing, no object or scorer reads.
    right = [Waypoint(secs * DEMO_PACE, tuple(float(v) for v in p), finger, label, g)
             for label, secs, p, finger, g, _ in demo_plan()]
    left = [Waypoint(1.5 * DEMO_PACE, tuple(float(v) for v in PARK_L), FINGER_OPEN, "park", 0.0)]
    end = sum(w.duration for w in right)
    left.append(Waypoint(end - left[0].duration, tuple(float(v) for v in PARK_L), FINGER_OPEN, "hold clear", 0.0))
    return {"left": left, "right": right}
