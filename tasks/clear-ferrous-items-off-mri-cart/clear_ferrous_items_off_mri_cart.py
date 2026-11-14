"""Screening a patient cart before it is wheeled into the MRI scanner room. A green ABS tray (the cart top,
260 x 720 mm inside, 25 mm rim) holds the patient's and staff's loose things, all real size: steel Lister bandage
scissors, a penlight (blue aluminium barrel, steel head and pocket clip), a smartphone in a black case lying screen
down under a blue foam positioning pad (12 mm of the phone shows past the pad's end), a stethoscope (steel
chestpiece on the tray, burgundy tubing draped up over a grey foam wedge and the pad, headset lying on the pad),
and a closed PET water bottle. An empty grey polypropylene bin (150 x 350 x 80 mm) stands between the tray and the
robot. Tray and bin are clamped down. Zero object jitter; 20 Hz.
"""
import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
from yam_parts import Part, ball, box, cyl
from yam_joints import Hinge, Link
from yam_mesh import Mesh, MeshMechanism, MeshPart
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN
from yam_arena.demo import Waypoint
from yam_arena.task_file import TABLE_TOP

SCENE = "station"

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



# ------------------------------------------------------------------ layout (metres, world frame)
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"
MM = 0.001

STEEL = (0.80, 0.81, 0.84)       # bright stainless steel
STEEL_DARK = (0.55, 0.56, 0.60)
ALU_BLUE = (0.10, 0.28, 0.70)    # blue anodised aluminium
BLACK = (0.05, 0.05, 0.06)
TPU = (0.09, 0.09, 0.10)
GLASS = (0.02, 0.02, 0.04)
TRAY_GREEN = (0.20, 0.45, 0.36)
BIN_GREY = (0.42, 0.44, 0.47)
FOAM_BLUE = (0.16, 0.38, 0.80)
FOAM_DARK = (0.30, 0.32, 0.36)
PET = (0.72, 0.88, 0.96)
CAP_BLUE = (0.08, 0.25, 0.75)
LABEL = (0.96, 0.96, 0.92)
TUBE_RED = (0.60, 0.07, 0.16)    # burgundy PVC tubing

# Cart-top tray (fixed): inner 260 mm (x) by 720 mm (y), 3 mm floor, 25 mm rim.
WALL = 3.0 * MM
TRAY_X0, TRAY_X1 = 0.140, 0.400            # inner faces
TRAY_Y0, TRAY_Y1 = -0.360, 0.360
TRAY_RIM = 25.0 * MM                       # rim height above the tray floor
FLOOR = WALL                               # tray floor top above the table
# Bin (fixed): outer 150 mm (x) by 350 mm (y) by 80 mm, 3 mm walls and floor.
BIN_X0, BIN_X1 = -0.020, 0.130             # outer faces
BIN_Y0, BIN_Y1 = -0.175, 0.175
BIN_H = 80.0 * MM
BIN_XC = (BIN_X0 + BIN_X1) / 2


def _open_box(name, x0, x1, y0, y1, wall, height, color, outer):
    """A floor and four walls; (x0..y1) are the outer faces when `outer`, else the inner faces."""
    if not outer:
        x0, x1, y0, y1 = x0 - wall, x1 + wall, y0 - wall, y1 + wall
    cx, cy, lx, ly = (x0 + x1) / 2, (y0 + y1) / 2, x1 - x0, y1 - y0
    shapes = [box((cx, cy, wall / 2), (lx, ly, wall))]
    for x in (x0 + wall / 2, x1 - wall / 2):
        shapes.append(box((x, cy, height / 2), (wall, ly, height)))
    for y in (y0 + wall / 2, y1 - wall / 2):
        shapes.append(box((cx, y, height / 2), (lx - 2 * wall, wall, height)))
    return Part(name, (0.0, 0.0, TOP), shapes, mass=None, color=color, friction=(0.4, 0.3))


TRAY = _open_box("tray", TRAY_X0, TRAY_X1, TRAY_Y0, TRAY_Y1, WALL, WALL + TRAY_RIM, TRAY_GREEN, outer=False)
BIN = _open_box("bin", BIN_X0, BIN_X1, BIN_Y0, BIN_Y1, WALL, BIN_H, BIN_GREY, outer=True)


# ------------------------------------------------------------------ mesh helpers (closed, outward-wound)
def _outward(pts, faces):
    """Faces as an integer array, flipped if their signed volume is negative."""
    f = np.array(faces, int)
    t = pts[f]
    volume = float(np.einsum("ij,ij->i", t[:, 0], np.cross(t[:, 1], t[:, 2])).sum())
    return f if volume > 0 else f[:, ::-1].copy()


def _slab(name, outline_xy, z0, z1, color):
    """A convex polygon (part-frame x-y, metres) extruded from z0 to z1."""
    pts2 = [tuple(map(float, p)) for p in outline_xy]
    n = len(pts2)
    pts = np.array([(x, y, z) for z in (z0, z1) for (x, y) in pts2], float)
    faces = []
    for i in range(n):
        j = (i + 1) % n
        faces += [(i, j, n + j), (i, n + j, n + i)]
    for k in range(1, n - 1):
        faces += [(0, k + 1, k), (n, n + k, n + k + 1)]
    proxy = (box((pts.min(0) + pts.max(0)) / 2, np.ptp(pts, axis=0)),)
    return Mesh(name, pts, _outward(pts, faces), collision="convex", color=color, proxy=proxy)


def _ring(name, centre_xy, r_out, r_in, z0, z1, color, n=28):
    """A flat ring of rectangular section (a finger ring); it collides as its convex hull, a disc."""
    cx, cy = centre_xy
    ang = [2 * math.pi * i / n for i in range(n)]
    pts = []
    for r, z in ((r_out, z0), (r_out, z1), (r_in, z1), (r_in, z0)):
        pts += [(cx + r * math.cos(a), cy + r * math.sin(a), z) for a in ang]
    faces = []
    for band in range(4):
        a0, b0 = band * n, ((band + 1) % 4) * n
        for i in range(n):
            j = (i + 1) % n
            faces += [(a0 + i, a0 + j, b0 + j), (a0 + i, b0 + j, b0 + i)]
    proxy = (cyl((cx, cy, (z0 + z1) / 2), r_out, z1 - z0),)
    pts = np.array(pts, float)
    return Mesh(name, pts, _outward(pts, faces), collision="convex", color=color, proxy=proxy)


def _poly_area(outline):
    x, y = np.array(outline, float).T
    return 0.5 * abs(float(np.dot(x, np.roll(y, -1)) - np.dot(y, np.roll(x, -1))))


# ------------------------------------------------------------------ hazards
# Lister bandage scissors, 7.25 in (184 mm), stainless steel, closed, lying flat. Part frame: y along the length
# (finger rings at +y, blunt probe tip at -y), origin on the tray floor under the middle of the length.
SC_T = 5.0 * MM                                        # thickness of shanks, rings and the stacked blades
SC_RING_R, SC_RING_BORE, SC_RING_Y, SC_RING_X = 14.0 * MM, 9.5 * MM, 78.0 * MM, 14.5 * MM
SC_SHANK_L = [(-17 * MM, 66 * MM), (-7 * MM, -10 * MM), (-1 * MM, -10 * MM), (-11 * MM, 66 * MM)]
SC_SHANK_R = [(11 * MM, 66 * MM), (1 * MM, -10 * MM), (7 * MM, -10 * MM), (17 * MM, 66 * MM)]
SC_NECK = [(-7 * MM, -32 * MM), (7 * MM, -32 * MM), (7 * MM, -10 * MM), (-7 * MM, -10 * MM)]
SC_BLADE = [(-7 * MM, -30 * MM), (28 * MM, -90 * MM), (38 * MM, -84 * MM), (7 * MM, -30 * MM)]   # 30 deg Lister angle
SC_TIP = (34 * MM, -87.5 * MM)                         # blunt probe tip, radius 4.5 mm
SC_PIVOT = (0.0, -12 * MM)
SC_VOLUME = (SC_T * (_poly_area(SC_SHANK_L) + _poly_area(SC_SHANK_R) + _poly_area(SC_NECK) + _poly_area(SC_BLADE)
                     + 2 * math.pi * (SC_RING_R ** 2 - SC_RING_BORE ** 2)) + math.pi * (4.5 * MM) ** 2 * 2.5 * MM)
SCISSORS_MASS = round(7750.0 * SC_VOLUME, 3)           # 420 stainless, 7,750 kg/m3
SCISSORS_POS = (0.185, 0.170)
SC_GRIP_Y = 20.0 * MM                                  # demo grips the shanks 20 mm toward the rings


def _scissors():
    meshes = (
        _ring("ring_l", (-SC_RING_X, SC_RING_Y), SC_RING_R, SC_RING_BORE, 0.0, SC_T, STEEL),
        _ring("ring_r", (SC_RING_X, SC_RING_Y), SC_RING_R, SC_RING_BORE, 0.0, SC_T, STEEL),
        _slab("shank_l", SC_SHANK_L, 0.0, SC_T, STEEL),
        _slab("shank_r", SC_SHANK_R, 0.0, SC_T, STEEL),
        _slab("neck", SC_NECK, 0.0, SC_T, STEEL),
        _slab("blade", SC_BLADE, 0.0, SC_T, STEEL),
    )
    shapes = [cyl((SC_PIVOT[0], SC_PIVOT[1], 3.0 * MM), 4.5 * MM, 6.0 * MM, color=STEEL_DARK),   # pivot screw
              cyl((SC_TIP[0], SC_TIP[1], 1.25 * MM), 4.5 * MM, 2.5 * MM, color=STEEL)]           # probe tip
    return MeshPart("scissors", (SCISSORS_POS[0], SCISSORS_POS[1], TOP + FLOOR + 0.3 * MM), shapes,
                    mass=SCISSORS_MASS, color=STEEL, friction=(0.4, 0.3), contact_offset=0.001, meshes=meshes)


# Reusable penlight, 150 mm x 15 mm: blue anodised aluminium barrel, bright steel head, tail cap and pocket clip,
# two AAA cells inside. Part frame: y along the barrel (lamp end at +y), origin on the tray floor under the middle.
PEN_R, PEN_HEAD_R = 6.5 * MM, 7.5 * MM
PENLIGHT_MASS = 0.045
PENLIGHT_POS = (0.320, 0.190)
PEN_GRIP_Y = 4.0 * MM


def _penlight():
    z = PEN_HEAD_R
    shapes = [
        cyl((0, -3 * MM, z), PEN_R, 118 * MM, axis="y", color=ALU_BLUE),              # barrel, y -62 .. +56
        cyl((0, 63 * MM, z), PEN_HEAD_R, 14 * MM, axis="y", color=STEEL),              # lamp head, +56 .. +70
        cyl((0, 72 * MM, z), 5.0 * MM, 4 * MM, axis="y", color=(1.0, 0.95, 0.6)),      # lens, +70 .. +74
        cyl((0, -67 * MM, z), PEN_HEAD_R, 10 * MM, axis="y", color=STEEL),             # tail cap, -72 .. -62
        cyl((0, -74 * MM, z), 3.0 * MM, 4 * MM, axis="y", color=BLACK),                # switch button, -76 .. -72
        box((9.6 * MM, -38 * MM, 2.75 * MM), (1.2 * MM, 46 * MM, 5.5 * MM), STEEL),    # pocket clip, on the tray side
        box((7.0 * MM, -63 * MM, 4.0 * MM), (6.4 * MM, 5 * MM, 8.0 * MM), STEEL),      # clip root on the tail cap
    ]
    return Part("penlight", (PENLIGHT_POS[0], PENLIGHT_POS[1], TOP + FLOOR + 0.3 * MM), shapes,
                mass=PENLIGHT_MASS, color=ALU_BLUE, friction=(0.4, 0.3), contact_offset=0.001)


# Smartphone in a black TPU case, lying screen down (camera side up). Case 150.4 x 74.4 x 10.6 mm; the camera
# island stands 1.6 mm proud at the -y end. Part frame: y along the length, origin on the tray floor.
PH_L, PH_W, PH_T, PH_BUMP = 150.4 * MM, 74.4 * MM, 10.6 * MM, 1.6 * MM
PHONE_MASS = 0.200
LINE_X = 0.250                                         # the pad, phone, wedge and stethoscope lie along x = 0.25
PHONE_POS = (LINE_X, -0.267)


def _phone():
    shapes = [
        box((0, 0, PH_T / 2), (PH_W, PH_L, PH_T), TPU),
        box((-15.2 * MM, -53.2 * MM, PH_T + PH_BUMP / 2), (36 * MM, 36 * MM, PH_BUMP), (0.22, 0.22, 0.25)),
        cyl((-23.5 * MM, -61.5 * MM, PH_T + PH_BUMP + 0.5 * MM), 7.0 * MM, 1.0 * MM, color=GLASS),
        cyl((-7.0 * MM, -45.0 * MM, PH_T + PH_BUMP + 0.5 * MM), 7.0 * MM, 1.0 * MM, color=GLASS),
        cyl((-6.5 * MM, -62.5 * MM, PH_T + PH_BUMP + 0.25 * MM), 2.5 * MM, 0.5 * MM, color=(1.0, 1.0, 0.9)),
    ]
    return Part("phone", (PHONE_POS[0], PHONE_POS[1], TOP + FLOOR + 0.3 * MM), shapes, mass=PHONE_MASS,
                color=TPU, friction=(0.5, 0.4), contact_offset=0.001)


# ------------------------------------------------------------------ MR-safe items
# Blue polyurethane-foam positioning pad 190 x 80 x 30 mm, lying on the phone.
PAD_L, PAD_W, PAD_T = 190.0 * MM, 80.0 * MM, 30.0 * MM
PAD_MASS = 0.025                                       # 456 cm3 of 55 kg/m3 foam
PAD_POS = (LINE_X, -0.235)
PAD_Z0 = FLOOR + PH_T + PH_BUMP + 1.0 * MM + 0.4 * MM  # pad underside above the table at the start (on the lenses)


def _pad():
    return Part("pad", (PAD_POS[0], PAD_POS[1], TOP + PAD_Z0), [box((0, 0, PAD_T / 2), (PAD_W, PAD_L, PAD_T))],
                mass=PAD_MASS, color=FOAM_BLUE, friction=(0.8, 0.7), contact_offset=0.001)


# Grey foam positioning wedge 120 x 80 x 40 mm: a ramp rising toward -y. Part frame origin under the middle.
WEDGE_L, WEDGE_W, WEDGE_H = 120.0 * MM, 80.0 * MM, 40.0 * MM
WEDGE_MASS = 0.011                                     # 192 cm3 of 55 kg/m3 foam
WEDGE_POS = (LINE_X, -0.060)


def _wedge():
    h, l, w = WEDGE_H, WEDGE_L / 2, WEDGE_W / 2
    pts = np.array([(-w, l, 0), (w, l, 0), (-w, -l, 0), (w, -l, 0), (-w, -l, h), (w, -l, h)], float)
    faces = [(0, 2, 3), (0, 3, 1), (2, 4, 5), (2, 5, 3), (0, 1, 5), (0, 5, 4), (0, 4, 2), (1, 3, 5)]
    n = 6                                  # offline stand-in: steps that stay under the ramp surface
    steps = tuple(box((0, -l + (i + 0.5) * WEDGE_L / n, h * (n - 1 - i) / (2 * n)), (WEDGE_W, WEDGE_L / n, h * (n - 1 - i) / n))
                  for i in range(n - 1))
    mesh = Mesh("ramp", pts, _outward(pts, faces), collision="convex", color=FOAM_DARK, proxy=steps)
    return MeshPart("wedge", (WEDGE_POS[0], WEDGE_POS[1], TOP + FLOOR + 0.3 * MM), (), mass=WEDGE_MASS,
                    color=FOAM_DARK, friction=(0.8, 0.7), contact_offset=0.001, meshes=(mesh,))


# Closed 0.33 L PET water bottle, 60 mm diameter, 165 mm tall, full.
BOTTLE_MASS = 0.350
BOTTLE_POS = (0.360, 0.050)


def _bottle():
    shapes = [
        cyl((0, 0, 65 * MM), 30 * MM, 130 * MM, color=PET),
        cyl((0, 0, 70 * MM), 30.4 * MM, 45 * MM, color=LABEL),
        cyl((0, 0, 137.5 * MM), 22 * MM, 15 * MM, color=PET),
        cyl((0, 0, 149 * MM), 13 * MM, 8 * MM, color=PET),
        cyl((0, 0, 159 * MM), 15 * MM, 12 * MM, color=CAP_BLUE),
    ]
    return Part("bottle", (BOTTLE_POS[0], BOTTLE_POS[1], TOP + FLOOR + 0.3 * MM), shapes, mass=BOTTLE_MASS,
                color=PET, friction=(0.4, 0.3), contact_offset=0.001)


# ------------------------------------------------------------------ stethoscope
# One free articulation: a 440 mm PVC tube as 22 capsule segments of 20 mm with alternate horizontal and vertical
# bend hinges, a rigid steel chestpiece on one end and a rigid headset (yoke, two ear tubes, ear tips) on the other.
# The middle tube segment is the root, so both chains have equal length. Part frame (every joint at 0): the tube is
# straight along y, chestpiece toward +y, headset toward -y.
SEG = 20.0 * MM
N_SIDE = 10                             # tube segments on each side of the middle one
TUBE_R = 5.0 * MM
TUBE_K, TUBE_C, TUBE_ARM = 0.006, 0.002, 5e-6   # bend spring N m/rad, damping N m s/rad, armature kg m2
BEND_LIMIT = 60.0
SEG_MASS = 0.040 / 22                   # 40 g of tubing
CHEST_MASS, HEADSET_MASS = 0.085, 0.035
CHEST_R, CHEST_Y = 22.5 * MM, 262.5 * MM          # chestpiece centre in the part frame
CHEST_HALF = 11.0 * MM                             # chestpiece is 22 mm thick, centred on the tube axis
YOKE_Y, EAR_X, EAR_Y0, EAR_Y1, TIP_Y = -220 * MM, 30 * MM, -280 * MM, -385 * MM, -390 * MM
EAR_GRIP_Y = -335 * MM                             # the demo pinches across both ear tubes here
STETH_ORIGIN = (LINE_X, 0.064, FLOOR + CHEST_HALF + 0.3 * MM)   # middle tube segment; z above the table
START_BENDS = {"b04": -18.6, "b10": 18.6, "a03": -15.0, "a09": 15.0}


def _bend(i):
    return "z" if i % 2 else "x"


def _stethoscope():
    links = []
    for side, s in (("a", 1.0), ("b", -1.0)):
        parent = None
        for i in range(1, N_SIDE + 1):
            y0 = s * (SEG / 2 + (i - 1) * SEG)
            name = f"{side}{i:02d}"
            links.append(Link(name, [ball((0, y0, 0), TUBE_R), cyl((0, y0 + s * SEG / 2, 0), TUBE_R, SEG, axis="y")],
                              Hinge(_bend(i), (0, y0, 0), -BEND_LIMIT, BEND_LIMIT, damping=TUBE_C, spring=TUBE_K,
                                    initial=START_BENDS.get(name, 0.0), armature=TUBE_ARM),
                              mass=SEG_MASS, parent=parent))
            parent = name
    y = SEG / 2 + N_SIDE * SEG           # 210 mm: where the end bodies hinge on
    chest = [
        ball((0, y, 0), TUBE_R), cyl((0, y + 5 * MM, 0), TUBE_R, 10 * MM, axis="y"),
        cyl((0, y + 20 * MM, 0), 3.5 * MM, 20 * MM, axis="y", color=STEEL),                       # stem
        cyl((0, CHEST_Y, -6 * MM), CHEST_R, 10 * MM, color=STEEL),                                # diaphragm side
        cyl((0, CHEST_Y, -9.5 * MM), CHEST_R + 0.5 * MM, 3 * MM, color=BLACK),                    # non-chill rim
        cyl((0, CHEST_Y, 5 * MM), 16.5 * MM, 12 * MM, color=STEEL),                               # bell side
        cyl((0, CHEST_Y, 11.2 * MM), 13.0 * MM, 0.6 * MM, color=BLACK),                           # bell opening
    ]
    links.append(Link("chest", chest, Hinge("z", (0, y, 0), -BEND_LIMIT, BEND_LIMIT, damping=TUBE_C, spring=TUBE_K,
                                            armature=TUBE_ARM), mass=CHEST_MASS, parent=f"a{N_SIDE:02d}"))
    headset = [ball((0, -y, 0), TUBE_R), cyl((0, -y - 5 * MM, 0), TUBE_R, 10 * MM, axis="y")]
    for s in (-1.0, 1.0):
        headset += [cyl((s * EAR_X, (EAR_Y0 + EAR_Y1) / 2, 0), 3.0 * MM, EAR_Y0 - EAR_Y1, axis="y", color=STEEL),
                    ball((s * (EAR_X - 3 * MM), TIP_Y, 0), 5.0 * MM, color=(0.25, 0.25, 0.27))]
    links.append(Link("headset", headset, Hinge("z", (0, -y, 0), -BEND_LIMIT, BEND_LIMIT, damping=TUBE_C,
                                                spring=TUBE_K, armature=TUBE_ARM),
                      mass=HEADSET_MASS, parent=f"b{N_SIDE:02d}"))
    h = 3.5 * MM
    meshes = {"headset": (
        _slab("branch_l", [(-h, YOKE_Y), (-EAR_X - h, EAR_Y0), (-EAR_X + h, EAR_Y0), (h, YOKE_Y)], -3 * MM, 3 * MM, TUBE_RED),
        _slab("branch_r", [(-h, YOKE_Y), (EAR_X - h, EAR_Y0), (EAR_X + h, EAR_Y0), (h, YOKE_Y)], -3 * MM, 3 * MM, TUBE_RED),
    )}
    base = [cyl((0, 0, 0), TUBE_R, SEG, axis="y")]
    return MeshMechanism("steth", (STETH_ORIGIN[0], STETH_ORIGIN[1], TOP + STETH_ORIGIN[2]), base, links,
                         mass=SEG_MASS, color=TUBE_RED, friction=(0.5, 0.4), contact_offset=0.0015,
                         self_collide=True, meshes=meshes)


SCISSORS, PENLIGHT, PHONE, STETH = _scissors(), _penlight(), _phone(), _stethoscope()
PAD, WEDGE, BOTTLE = _pad(), _wedge(), _bottle()
OBJECTS = [TRAY, BIN, SCISSORS, PENLIGHT, PHONE, STETH, PAD, WEDGE, BOTTLE]
PRESSED_PARTS = ("steth", "pad", "phone", "scissors", "penlight", "wedge")
RIGID_HAZARDS = ("scissors", "penlight", "phone")
SAFE = ("pad", "wedge", "bottle")

# ------------------------------------------------------------------ scoring
EDGE_TOL = 2.0 * MM          # tolerance against fixed walls and floors
BOTTLE_UPRIGHT_DEG = 20.0
TABLE_CLEAR = 10.0 * MM      # tubing outside the bin must hang at least this far above the table
STEP_M = 0.0005              # at rest: no tracked point moves 0.5 mm between control steps (1 cm/s at 20 Hz)
def _in_rect(p, x0, x1, y0, y1):
    return bool(x0 <= p[0] <= x1 and y0 <= p[1] <= y1)


def in_bin_xy(p, outer=False):
    w = 0.0 if outer else WALL
    return _in_rect(p, BIN_X0 + w, BIN_X1 - w, BIN_Y0 + w, BIN_Y1 - w)


def inside_bin(p):
    """A point inside the bin's inner walls, above its floor and below its rim."""
    return in_bin_xy(p) and TOP + WALL - EDGE_TOL <= p[2] <= TOP + BIN_H


def over_tray(p, outer=False):
    w = WALL if outer else 0.0
    return _in_rect(p, TRAY_X0 - w, TRAY_X1 + w, TRAY_Y0 - w, TRAY_Y1 + w)


def steth_points(st):
    """World points on the stethoscope: chestpiece centre, headset points (yoke, ear tubes, ear tips), tube joints."""
    y = SEG / 2 + N_SIDE * SEG
    chest = st.link_to_world("chest", (0.0, CHEST_Y, 0.0))
    head = [st.link_to_world("headset", p) for p in
            ((0.0, YOKE_Y, 0.0), (-EAR_X, EAR_GRIP_Y, 0.0), (EAR_X, EAR_GRIP_Y, 0.0),
             (-EAR_X + 3 * MM, TIP_Y, 0.0), (EAR_X - 3 * MM, TIP_Y, 0.0))]
    tube = [st.to_world((0.0, 0.0, 0.0))]
    for side, s in (("a", 1.0), ("b", -1.0)):
        for i in range(1, N_SIDE + 1):
            y0 = s * (SEG / 2 + (i - 1) * SEG)
            tube.append(st.link_to_world(f"{side}{i:02d}", (0.0, y0, 0.0)))
            tube.append(st.link_to_world(f"{side}{i:02d}", (0.0, y0 + s * SEG / 2, 0.0)))
    tube.append(st.link_to_world("chest", (0.0, y, 0.0)))
    tube.append(st.link_to_world("headset", (0.0, -y, 0.0)))
    return np.asarray(chest, float), np.asarray(head, float), np.asarray(tube, float)


def steth_in_bin(st):
    """Chestpiece and headset inside the bin; tubing inside it, arching above it, or hanging outside clear of the
    table and not over the tray."""
    chest, head, tube = steth_points(st)
    if not (inside_bin(chest) and all(inside_bin(p) for p in head)):
        return False
    for p in tube:
        if in_bin_xy(p, outer=True) and p[2] >= TOP:
            continue
        if over_tray(p, outer=True) or p[2] - TUBE_R < TOP + TABLE_CLEAR:
            return False
    return True


def on_cart(name, state):
    """Centre inside the tray's inner walls, nothing below the tray floor, the bottle upright."""
    if not over_tray(state.pos):
        return False
    if state.low[2] < TOP + FLOOR - EDGE_TOL:
        return False
    if name == "bottle" and state.tilt_deg() > BOTTLE_UPRIGHT_DEG:
        return False
    return True


def safe_in_bin(state):
    return in_bin_xy(state.pos) and state.low[2] <= TOP + BIN_H


def safe_off_cart(state):
    """Fallen or put off the cart: centre outside the tray's inner walls and reaching below the rim top."""
    return (not over_tray(state.pos)) and state.low[2] < TOP + FLOOR + TRAY_RIM


_POSE_EPISODE = None
_POSE_TOKEN = None
_PREVIOUS = None
_QUIET = {}

# Outline box of each rigid item in its own frame (half-width x, half-length y, height z), metres. The scorer
# follows its eight corners; the stethoscope is followed through its chestpiece, headset and tube joint points.
OUTLINE = {
    "scissors": (0.020, 0.092, SC_T),
    "penlight": (0.0105, 0.076, 2 * PEN_HEAD_R),
    "phone": (PH_W / 2, PH_L / 2, PH_T + PH_BUMP),
    "pad": (PAD_W / 2, PAD_L / 2, PAD_T),
    "wedge": (WEDGE_W / 2, WEDGE_L / 2, WEDGE_H),
    "bottle": (0.030, 0.030, 0.165),
}


def outline_points(state, name):
    hx, hy, h = OUTLINE[name]
    return np.array([state.to_world((sx * hx, sy * hy, z)) for sx in (-1, 1) for sy in (-1, 1) for z in (0.0, h)])


def item_points(objects, name):
    if name == "steth":
        chest, head, tube = steth_points(objects["steth"])
        return np.vstack([chest[None, :], head, tube])
    return outline_points(objects[name], name)


def at_rest(objects):
    """Per item: no tracked point moved STEP_M or more since the previous control step (1 cm/s at 20 Hz). Pose
    differences, not instantaneous speeds: resting contacts chatter, and a penlight lying on tubing can spin about
    its own axis at 0.5 rad/s while its outline moves a fraction of a millimetre."""
    global _POSE_EPISODE, _POSE_TOKEN, _PREVIOUS, _QUIET
    names = RIGID_HAZARDS + SAFE + ("steth",)
    if _RUN["t"] is None:
        out = {n: objects[n].resting() for n in RIGID_HAZARDS + SAFE}
        out["steth"] = objects["steth"].still(0.01, 0.5)
        return out
    if _POSE_EPISODE != _RUN["episode"]:
        _PREVIOUS, _POSE_TOKEN, _QUIET = None, None, {}
        _POSE_EPISODE = _RUN["episode"]
    token = (_RUN["step"], float(_RUN["t"]))
    if token != _POSE_TOKEN:
        poses = {n: item_points(objects, n) for n in names}
        quiet = {n: False for n in names}
        if _PREVIOUS is not None:
            step, old = _PREVIOUS
            if token[0] == step + 1:
                quiet = {n: bool(np.all(np.abs(poses[n] - old[n]) < STEP_M)) for n in names}
        _PREVIOUS, _POSE_TOKEN, _QUIET = (token[0], poses), token, quiet
    return dict(_QUIET)


def task_state(objects):
    """Per-item facts, before release, rest and hold are applied."""
    st = objects["steth"]
    stowed = {n: inside_bin(objects[n].pos) for n in RIGID_HAZARDS}
    stowed["steth"] = steth_in_bin(st)
    rest = at_rest(objects)
    kept = {n: on_cart(n, objects[n]) for n in SAFE}
    return stowed, kept, rest


def score(objects):
    stowed, kept, rest = task_state(objects)
    # Over-clearing and knocking things off are judged on the current state (the score is taken at the end): an
    # MR-safe item in the bin or off the cart zeroes the trial; one taken back out and returned to the tray doesn't.
    over_cleared = any(safe_in_bin(objects[n]) for n in SAFE)
    knocked_off = any(safe_off_cart(objects[n]) for n in SAFE)
    done = {}
    for n in RIGID_HAZARDS + ("steth",):
        ok = stowed[n] and rest[n] and released(objects[n])
        done[n] = ok and held_for("stowed " + n, ok) + 1e-9 >= STILL_S
    safe_ok = all(kept[n] and rest[n] and released(objects[n]) for n in SAFE)
    safe_done = safe_ok and held_for("safe items kept", safe_ok) + 1e-9 >= STILL_S
    if over_cleared or knocked_off:
        return 0.0
    n_done = sum(1 for v in done.values() if v)
    total = 0.2 * n_done
    if n_done == 4 and safe_done:
        total += 0.2
    return round(total, 6)


def success(objects):
    return score(objects) >= 1.0


# ------------------------------------------------------------------ scripted demo (start layout and time only)
TIP = 0.013                               # pad tips below the TCP
L_YAW, R_YAW = -math.pi / 2, math.pi / 2  # both hands close their fingers along x
HIGH = 0.165                              # pad-tip height above the table while carrying
DEMO_PACE = 1.0
L_PARK, R_PARK = (0.200, 0.330), (0.200, -0.330)
PARK_TIP = 0.120
CHEST_DROP_Y, HEAD_DROP_Y = 0.140, -0.090   # bin y of the chestpiece centre and of the ear-tube grip
HEAD_FIRST = True                           # the headset is laid in the bin before the chestpiece


def width(w):
    """Finger target for an opening of w metres."""
    return max(FINGER_OPEN, -w / 2)


def tcp(x, y, tip_z):
    return (float(x), float(y), float(TOP + tip_z + TIP))


def _steth_start(link, p):
    st = STETH.offline_state(STETH.pos, 0.0, {})
    return st.link_to_world(link, p)


def demo():
    plan = {"left": [], "right": []}
    last = {"left": (tcp(*L_PARK, PARK_TIP), FINGER_OPEN), "right": (tcp(*R_PARK, PARK_TIP), FINGER_OPEN)}
    yaw = {"left": L_YAW, "right": R_YAW}

    def step(dt, label, left=None, right=None):
        for arm, target in (("left", left), ("right", right)):
            if target is not None:
                last[arm] = target
            pos, finger = last[arm]
            plan[arm].append(Waypoint(dt * DEMO_PACE, tuple(pos), finger, label, yaw[arm]))

    def pick_to_bin(arm, name, grip_xy, grip_tip, open_w, drop_xy, drop_tip, close_s=0.8):
        def go(dt, label, target):
            step(dt, f"{name}: {label}", **{arm: target})
        gx, gy = grip_xy
        dx, dy = drop_xy
        go(1.6, "above it", (tcp(gx, gy, 0.070), width(open_w)))
        go(1.3, "pads around it", (tcp(gx, gy, grip_tip), width(open_w)))
        go(close_s, "grip", (tcp(gx, gy, grip_tip), FINGER_CLOSED))
        go(1.4, "lift", (tcp(gx, gy, HIGH), FINGER_CLOSED))
        go(2.2, "over the bin", (tcp(dx, dy, HIGH), FINGER_CLOSED))
        go(1.5, "lower into the bin", (tcp(dx, dy, drop_tip), FINGER_CLOSED))
        go(0.7, "let go", (tcp(dx, dy, drop_tip), width(open_w)))
        go(1.2, "up", (tcp(dx, dy, HIGH), width(open_w)))

    step(1.5, "ready", (tcp(*L_PARK, PARK_TIP), FINGER_OPEN), (tcp(*R_PARK, PARK_TIP), FINGER_OPEN))

    # Both hands: the stethoscope. Left takes the chestpiece by its rim, right pinches across both ear tubes where
    # the headset lies on the pad; they lift together so the tubing comes up off the wedge and pad, carry it level
    # over the bin, bring the ends together so the tubing hangs into the bin, lower the chestpiece in and lay the
    # headset down flat along the bin.
    c = _steth_start("chest", (0.0, CHEST_Y, 0.0))
    e = _steth_start("headset", (0.0, EAR_GRIP_Y, 0.0))
    chest_tip = FLOOR + 0.004
    ear_tip = PAD_Z0 + PAD_T + 0.0015
    ear_open = 0.082
    cy, hy = CHEST_DROP_Y, HEAD_DROP_Y   # where the chestpiece and the ear-tube grip come down in the bin
    step(1.8, "stethoscope: above both ends", (tcp(c[0], c[1], 0.070), FINGER_OPEN), (tcp(e[0], e[1], 0.090), width(ear_open)))
    step(1.4, "stethoscope: pads around both ends", (tcp(c[0], c[1], chest_tip), FINGER_OPEN), (tcp(e[0], e[1], ear_tip), width(ear_open)))
    step(0.9, "stethoscope: grip", (tcp(c[0], c[1], chest_tip), FINGER_CLOSED), (tcp(e[0], e[1], ear_tip), FINGER_CLOSED))
    step(2.5, "stethoscope: lift together", (tcp(c[0], c[1], HIGH), FINGER_CLOSED), (tcp(e[0], e[1], HIGH), FINGER_CLOSED))
    step(2.5, "stethoscope: back over the bin", (tcp(0.125, c[1] - 0.02, HIGH), FINGER_CLOSED), (tcp(0.125, e[1] + 0.01, HIGH), FINGER_CLOSED))
    step(2.5, "stethoscope: ends toward each other", (tcp(BIN_XC + 0.015, 0.195, HIGH), FINGER_CLOSED), (tcp(BIN_XC, -0.190, HIGH), FINGER_CLOSED))
    step(2.5, "stethoscope: tubing hangs into the bin", (tcp(BIN_XC, cy, HIGH), FINGER_CLOSED), (tcp(BIN_XC, hy, HIGH), FINGER_CLOSED))
    head_down = (tcp(BIN_XC, hy, 0.012), FINGER_CLOSED)
    head_open = (tcp(BIN_XC, hy, 0.012), width(0.084))
    head_up = (tcp(BIN_XC, hy, HIGH), width(0.084))
    chest_down = (tcp(BIN_XC, cy, 0.030), FINGER_CLOSED)
    chest_open = (tcp(BIN_XC, cy, 0.030), FINGER_OPEN)
    chest_up = (tcp(BIN_XC, cy, HIGH), FINGER_OPEN)
    if HEAD_FIRST:
        step(2.4, "stethoscope: lay the headset in the bin", None, head_down)
        step(0.8, "stethoscope: let go of the headset", None, head_open)
        step(1.4, "stethoscope: right hand up", None, head_up)
        step(2.2, "stethoscope: lower the chestpiece", chest_down, None)
        step(0.8, "stethoscope: let go of the chestpiece", chest_open, None)
        step(1.4, "stethoscope: left hand up", chest_up, None)
    else:
        step(2.2, "stethoscope: lower the chestpiece", chest_down, None)
        step(0.8, "stethoscope: let go of the chestpiece", chest_open, None)
        step(1.4, "stethoscope: left hand up", chest_up, None)
        step(2.4, "stethoscope: lay the headset in the bin", None, head_down)
        step(0.8, "stethoscope: let go of the headset", None, head_open)
        step(1.4, "stethoscope: right hand up", None, head_up)

    # Left hand: scissors (by the shanks), then the penlight (by the barrel), lowered into the bin. Meanwhile the
    # right hand lifts the pad off the phone and sets it down on the tray beside it, then takes the phone to the bin.
    mark = len(plan["left"])
    pick_to_bin("left", "scissors", (SCISSORS_POS[0], SCISSORS_POS[1] + SC_GRIP_Y), FLOOR + 0.001, 0.036,
                (0.022, 0.020), 0.045)
    pick_to_bin("left", "penlight", (PENLIGHT_POS[0], PENLIGHT_POS[1] + PEN_GRIP_Y), FLOOR + 0.002, 0.030,
                (0.092, 0.030), 0.045)
    step(1.5, "left: clear", (tcp(*L_PARK, PARK_TIP), FINGER_OPEN), None)
    px, py = PAD_POS
    pad_tip = PAD_Z0 + 0.015                # pads 15 mm up the pad's side, well clear of the phone's top
    pad_steps = [
        (1.6, "pad: above it", (tcp(px, py, 0.090), FINGER_OPEN)),
        (1.2, "pad: pads around it", (tcp(px, py, pad_tip), FINGER_OPEN)),
        (0.8, "pad: grip", (tcp(px, py, pad_tip), FINGER_CLOSED)),
        (1.0, "pad: lift", (tcp(px, py, 0.080), FINGER_CLOSED)),
        (1.6, "pad: aside", (tcp(px + 0.090, py, 0.080), FINGER_CLOSED)),
        (1.2, "pad: down on the tray", (tcp(px + 0.090, py, FLOOR + 0.019), FINGER_CLOSED)),
        (0.7, "pad: let go", (tcp(px + 0.090, py, FLOOR + 0.019), FINGER_OPEN)),
        (1.0, "pad: up", (tcp(px + 0.090, py, 0.090), FINGER_OPEN)),
    ]
    # The pad steps overwrite the right arm's waiting waypoints during the left hand's scissors trip, one for one.
    for k, (dt, label, target) in enumerate(pad_steps):
        w = plan["left"][mark + k]
        plan["right"][mark + k] = Waypoint(w.duration, tuple(target[0]), target[1], label, R_YAW)
        last["right"] = target
    for k in range(mark + len(pad_steps), len(plan["right"])):
        w = plan["right"][k]
        plan["right"][k] = Waypoint(w.duration, tuple(last["right"][0]), last["right"][1], "right: waits", R_YAW)
    pick_to_bin("right", "phone", PHONE_POS, FLOOR + 0.0015, 0.094, (BIN_XC, -0.040), 0.070)
    step(1.5, "right: clear", None, (tcp(*R_PARK, PARK_TIP), FINGER_OPEN))
    step(4.0, "stillness tail")
    return plan


def demo_duration():
    return sum(w.duration for w in demo()["right"])
