"""On the station table, on a green cutting mat between the arms: a white acrylic sheet (15 x 20 cm, 3 mm, 105 g) lies
loose, its right half covered by a brown paper protective film. The film is four strips (3.75 x 10 cm each) hinged to
each other along y, each stuck to the sheet, with a folded paper tab (3 x 2.5 cm) standing on its far edge. The sheet's
left half is bare. A blue plastic basket stands on the robot's right, beside the mat.

The task: hold the sheet down, peel the film off by its tab and drop it in the basket.

The film peels strip by strip: a strip comes unstuck when its free edge is pulled up, which twists its glue past
0.06 N m. Pulled along the sheet, the glue holds 8 N, and the tab tears off at 4 N first: a torn tab scores 0. The
sheet is light and loose: the pull that peels the first strip tips it up with the film (measured: without a hand on
it, the sheet tilts 14 degrees and slides 2.7 cm), so the other hand must hold it down. ``bonds.py`` makes the glue,
the joints between the strips and the tab's fold.

Run it:  inspect-robots run --embodiment yam_arena -E task=peel_acrylic_film.py --scorer task_score ...
"""

from __future__ import annotations

import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
from bonds import Joint, add, on_reset, on_step, tune  # noqa: E402
from looks import dress  # noqa: E402
from simready import SimReady  # noqa: E402
from yam_arena.constants import PAD_TIP_PAST_TCP  # noqa: E402
from yam_arena.demo import FINGER_CLOSED, FINGER_OPEN, Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 1.0

# -- the cutting mat and the sheet ------------------------------------------------------------------------------------
MAT_XY = (0.33, -0.05)
MAT_SIZE = (0.22, 0.24, 0.003)
MAT_TOP = TOP + MAT_SIZE[2]
MAT = Box("mat", pos=(*MAT_XY, TOP + MAT_SIZE[2] / 2), size=MAT_SIZE, color=(0.10, 0.36, 0.22))

SHEET_XY = (0.33, -0.05)
SHEET_SIZE = (0.15, 0.20, 0.003)  # 15 cm along x (the peel), 20 cm along y
SHEET_MASS = 0.105  # kg: cast acrylic, 1.18 g/cm3
SHEET_TOP = MAT_TOP + SHEET_SIZE[2]
SHEET = Box("sheet", pos=(*SHEET_XY, MAT_TOP + SHEET_SIZE[2] / 2), size=SHEET_SIZE, mass=SHEET_MASS,
            color=(0.90, 0.90, 0.88))
SHEET_NEAR = SHEET_XY[0] - SHEET_SIZE[0] / 2  # x of the edge toward the robot
SHEET_FAR = SHEET_XY[0] + SHEET_SIZE[0] / 2
SHEET_LEFT = SHEET_XY[1] + SHEET_SIZE[1] / 2

# -- the film: four strips hinged along y, the tab on the far edge ---------------------------------------------------
STRIPS = 4
STRIP_W = SHEET_SIZE[0] / STRIPS  # 3.75 cm along x
FILM_LEN = 0.10  # m along y: the film covers the sheet's right half
FILM_Y = SHEET_XY[1] - SHEET_SIZE[1] / 2 + FILM_LEN / 2  # -0.08
STRIP_T = 0.0015  # m: a paper film is 0.1 mm; PhysX needs a thicker body to stay stable
STRIP_MASS = 0.005  # kg: a real strip weighs under 1 g; lighter bodies jitter in the grip and on their hinges
STRIP_TOP = SHEET_TOP + STRIP_T
FILM_BROWN = (0.62, 0.43, 0.24)


def strip_x(i: int) -> float:
    """Centre x of strip i (1 is on the far edge, under the tab; 4 on the near edge)."""
    return SHEET_FAR - (i - 0.5) * STRIP_W


def strip_near_edge(i: int) -> float:
    """x of strip i's edge toward the robot: where its glue twists, and its hinge to strip i + 1."""
    return SHEET_FAR - i * STRIP_W


STRIP = [
    Box(f"strip_{i}", pos=(strip_x(i), FILM_Y, SHEET_TOP + STRIP_T / 2), size=(STRIP_W, FILM_LEN, STRIP_T),
        mass=STRIP_MASS, color=FILM_BROWN)
    for i in range(1, STRIPS + 1)
]

TAB_SIZE = (0.003, 0.03, 0.025)  # thick (x), wide (y), tall (z): a folded paper tab standing on the far edge
TAB_MASS = 0.004
TAB_HINGE_X = SHEET_FAR - TAB_SIZE[0] / 2  # the fold, on strip 1's top at the sheet's far edge
TAB = Box("tab", pos=(TAB_HINGE_X, FILM_Y, STRIP_TOP + TAB_SIZE[2] / 2), size=TAB_SIZE, mass=TAB_MASS,
          color=FILM_BROWN)

# The glue: a fixed joint under each strip's near edge, between the strip (the joint's first body, so PhysX measures
# the twist on the strip) and the sheet. It lets go when twisted past GLUE_TWIST, which a pull up at the strip's far
# edge (its joint to the strip before, or the tab) does. Set so that the first strip's glue holds until the loose
# sheet tips up (0.02 N m let go first, 0.06 N m tips the sheet 14 degrees), and a hand pressing the sheet's near left
# corner keeps it flat. Pulled along the sheet, it holds to GLUE_HOLD. The strips are joined at the middle of each cut by a ball joint
# that turns freely, damped, as paper bends and twists. The tab's fold is a ball joint too, held upright by a weak
# spring, and tears at TAB_HOLD. Ball joints pass no turning to the next strip, so a hand that twists the tab can't
# break the glue: only a pull up at a strip's far edge can.
GLUE_TWIST = 0.06  # N m
GLUE_HOLD = 8.0  # N
TAB_HOLD = 4.0  # N
HINGE_DAMPING = 2e-4  # N m s/rad
TAB_SPRING = 0.008  # N m/rad: holds the 4 g tab within a few degrees of upright

add(*(Joint(f"glue_{i}", body="sheet", to=f"strip_{i}", at=(strip_near_edge(i), FILM_Y, SHEET_TOP),
            hold=GLUE_HOLD, twist=GLUE_TWIST) for i in range(1, STRIPS + 1)))
add(*(Joint("hinge", body=f"strip_{i}", to=f"strip_{i + 1}", kind="ball",
            at=(strip_near_edge(i), FILM_Y, SHEET_TOP + STRIP_T / 2), damping=HINGE_DAMPING, collide=False)
      for i in range(1, STRIPS)))
add(Joint("fold", body="tab", to="strip_1", kind="ball", at=(TAB_HINGE_X, FILM_Y, STRIP_TOP),
          stiffness=TAB_SPRING, damping=HINGE_DAMPING, hold=TAB_HOLD, collide=False))
for _piece in [*STRIP, TAB]:
    tune(_piece.name, solverPositionIterationCount=32, solverVelocityIterationCount=4, maxDepenetrationVelocity=0.3,
         angularDamping=0.5)
tune("sheet", solverPositionIterationCount=32, solverVelocityIterationCount=4)

# -- the basket: a SimReady plastic basket, fixed, on the robot's right ------------------------------------------------
BASKET_XY = (0.235, -0.265)
BASKET_SCALE = 0.6  # the 20 x 27 x 11.4 cm model to 12 x 16 x 6.9 cm
BASKET_OUTER = (0.121, 0.162, 0.069)
BASKET = SimReady("basket", pos=(*BASKET_XY, TOP), size=BASKET_OUTER, package="misc_basket_plastic_a01",
                  usd="sm_misc_basket_plastic_a01_01.usd", scale=BASKET_SCALE)
BASKET_RIM = TOP + BASKET_OUTER[2]
IN_BASKET_XY = (0.06, 0.08)  # m: a piece's centre within this of the basket's centre is inside its walls

OBJECTS = [MAT, SHEET, *STRIP, TAB, BASKET]
FILM = [s.name for s in STRIP] + ["tab"]

# What the boxes look like: meshes modelled in Blender (looks/make_looks.py), the boxes' colliders unchanged.
LOOKS = {"mat": "mat.usdc", "sheet": "sheet.usdc", "tab": "tab.usdc", **{s.name: f"{s.name}.usdc" for s in STRIP}}
dress(LOOKS)

# -- scoring ---------------------------------------------------------------------------------------------------------
PEELED = 0.01  # m: a strip whose centre has moved this far from its place on the sheet has come unstuck
TORN = 0.02  # m: the tab's centre this far from its fold on strip 1 has torn off (it turns 1 cm from the fold)
SHEET_SHIFT = 0.02  # m: the sheet's centre farther than this from its start has been dragged or lifted
SHEET_TILT = math.radians(10)
OFF_TABLE_Z = -0.02

# What the episode remembers: each strip that has come unstuck, a torn tab, and a sheet that was lifted or dragged.
STATE = {"peeled": set(), "torn": False, "lifted": False}


@on_reset
def reset_memory(env=None) -> None:
    STATE["peeled"] = set()
    STATE["torn"] = False
    STATE["lifted"] = False


def _finite(objects) -> bool:
    return all(np.all(np.isfinite(s.pos)) and np.all(np.isfinite(s.quat_xyzw)) for s in objects.values())


def _rot(q) -> np.ndarray:
    x, y, z, w = np.asarray(q, float) / np.linalg.norm(q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def _in_frame(objects, frame: str, point) -> np.ndarray:
    """``point`` (world) in the frame of the object ``frame``, relative to its centre."""
    s = objects[frame]
    return _rot(s.quat_xyzw).T @ (np.asarray(point, float) - s.pos)


def strip_off(objects, i: int) -> bool:
    """Strip i's centre is 1 cm or more from where it was stuck, measured on the sheet."""
    rel = _in_frame(objects, "sheet", objects[f"strip_{i}"].pos)
    start = np.asarray(STRIP[i - 1].pos) - np.asarray(SHEET.pos)
    return bool(np.linalg.norm(rel - start) >= PEELED)


def tab_torn(objects) -> bool:
    """The tab's centre is 2 cm or more from its fold on strip 1 (on the hinge it stays 1 cm from it)."""
    rel = _in_frame(objects, "strip_1", objects["tab"].pos)
    fold = np.array([TAB_HINGE_X - STRIP[0].pos[0], 0.0, STRIP_T / 2])
    return bool(np.linalg.norm(rel - fold) >= TORN)


def sheet_in_place(objects) -> bool:
    """The sheet's centre within 2 cm of its start and the sheet within 10 degrees of flat."""
    s = objects["sheet"]
    up = _rot(s.quat_xyzw)[2, 2]
    return bool(np.linalg.norm(s.pos - np.asarray(SHEET.pos)) <= SHEET_SHIFT and up >= math.cos(SHEET_TILT))


def in_basket(objects, name: str) -> bool:
    c = objects[name].pos
    return bool(abs(c[0] - BASKET_XY[0]) < IN_BASKET_XY[0] and abs(c[1] - BASKET_XY[1]) < IN_BASKET_XY[1]
                and c[2] < BASKET_RIM)


def film_in_basket(objects) -> bool:
    return all(in_basket(objects, name) for name in FILM)


def _remember(objects) -> None:
    for i in range(1, STRIPS + 1):
        if strip_off(objects, i):
            STATE["peeled"].add(i)
    if tab_torn(objects):
        STATE["torn"] = True
    if not sheet_in_place(objects):
        STATE["lifted"] = True


def _violation(objects) -> bool:
    if not _finite(objects):
        return True
    return bool(objects["sheet"].pos[2] < TOP + OFF_TABLE_Z)


@on_step
def watch_the_film(env) -> None:
    """Record every strip that comes unstuck and a torn tab as it happens, whatever the scorer is asked."""
    from yam_arena.scene import object_states

    objects = object_states(env, ["sheet", *FILM])
    if _finite(objects):
        _remember(objects)


def success(objects) -> bool:
    """Every strip peeled, the whole film still in the basket, the tab whole and the sheet never moved."""
    return score(objects) == 1.0 and all(objects[n].resting(max_speed=0.02, max_spin=1.0) for n in FILM)


def score(objects) -> float:
    """1.0: all four strips peeled and every piece of the film in the basket. 0.5: all four peeled, the film not (all)
    in the basket. 0.25: two or three peeled. 0 otherwise. Always 0 once the tab has torn, once the sheet has been more
    than 2 cm from its start or tilted more than 10 degrees (lifted with the film, or dragged), and if it is off the
    table."""
    if _violation(objects):
        return 0.0
    _remember(objects)
    if STATE["torn"] or STATE["lifted"]:
        return 0.0
    peeled = len(STATE["peeled"])
    if peeled == STRIPS:
        return 1.0 if film_in_basket(objects) else 0.5
    return 0.25 if peeled >= 2 else 0.0


# -- scripted demo ---------------------------------------------------------------------------------------------------
# Left arm, fingers closed, straight down: press the bare left half of the sheet by its near corner and hold it until
# the film is off. Right arm, straight down, turned a quarter turn so the fingers close along x across the tab: pinch
# the tab, then peel in one stroke up and back toward the robot, so the film between the hand and the peel line leans
# up at 60 to 90 degrees, then carry the hanging film over the basket, turning the hand so the fingers open along the
# basket's long side, lower the film onto its floor and let go. Numbers are TCP positions.
PRESS_XY = (SHEET_NEAR + 0.02, SHEET_LEFT - 0.02)  # on the bare acrylic, by the near left corner
PRESS_DEPTH = 0.03  # m: the pad tips are sent this far below the sheet's top, so the arm leans on it hard
PRESS_Z = SHEET_TOP - PRESS_DEPTH + PAD_TIP_PAST_TCP
PRESS_YAW = -math.pi / 2
GRIP_YAW = math.pi / 2  # fingers close along x
GRIP_TIP_Z = STRIP_TOP + 0.011  # pad tips 11 mm above the fold: they hold the tab's top 1.4 cm, and the fingers
# stay clear of the film when the arm sags as it closes
GRIP_Z = GRIP_TIP_Z + PAD_TIP_PAST_TCP
TAB_XY = (TAB_HINGE_X, FILM_Y)
PEEL_END = (SHEET_NEAR + 0.01, FILM_Y, TOP + 0.195)  # the hand ends over the sheet's near edge, 19.5 cm up
CARRY_Z = TOP + 0.215
DROP_Z = TOP + 0.08  # the film folds onto the basket's floor, the pad tips 7 mm inside its rim
DROP_XY = (BASKET_XY[0] + 0.015, BASKET_XY[1])  # a little past the centre: the film falls back toward the robot
DROP_YAW = 0.0  # the fingers open along y, the basket's long side


def demo():
    w = Waypoint
    px, py = PRESS_XY
    tx, ty = TAB_XY
    bx, by = DROP_XY
    return {
        "left": [
            w(1.5, (px, py, TOP + 0.08), FINGER_CLOSED, "over the bare acrylic", PRESS_YAW),
            w(1.5, (px, py, PRESS_Z), FINGER_CLOSED, "press", PRESS_YAW),
            w(8.0, (px, py, PRESS_Z), FINGER_CLOSED, "hold the sheet", PRESS_YAW),
            w(1.5, (px, py, TOP + 0.10), FINGER_CLOSED, "let go", PRESS_YAW),
        ],
        "right": [
            w(1.5, (tx, ty, TOP + 0.08), FINGER_OPEN, "over the tab", GRIP_YAW),
            w(1.5, (tx, ty, GRIP_Z), FINGER_OPEN, "round the tab", GRIP_YAW),
            w(0.8, (tx, ty, GRIP_Z), FINGER_CLOSED, "pinch", GRIP_YAW),
            w(0.5, (tx, ty, GRIP_Z), FINGER_CLOSED, "hold", GRIP_YAW),
            w(6.0, PEEL_END, FINGER_CLOSED, "peel up and back", GRIP_YAW),
            w(3.0, (bx, by, CARRY_Z), FINGER_CLOSED, "carry over the basket", DROP_YAW),
            w(2.5, (bx, by, DROP_Z), FINGER_CLOSED, "lower into it", DROP_YAW),
            w(0.6, (bx, by, DROP_Z), FINGER_OPEN, "let go", DROP_YAW),
            w(1.0, (bx, by, CARRY_Z), FINGER_OPEN, "retreat", DROP_YAW),
        ],
    }
