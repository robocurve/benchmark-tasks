"""A peeled garlic clove lies on a walnut board beside an open garlic press; beyond the board a slate saucer, and on the
bench a table knife. The clove is to be pressed through the press onto the saucer and what clings under the press's
sieve scraped off onto the saucer with the knife.

How it is built. The press is two free bodies on a hinge at its tip: the lower handle with the chamber (its floor is
the sieve) and the upper handle with the plunger. It starts open, its upper handle on its stop at 120 degrees, lying
wholly on the board. The board, the saucer and the knife are free bodies. The clove is one of four free bodies of
four convex parts each (sizes by seed); the others lie on the floor under the bench, out of every camera.

What stands in for the garlic (every number from MATERIAL). The upper part passes through the cloves: the solver
never presses a rigid clove. The plunger arrives on the clove when its face comes down to the clove's highest point
inside the chamber's outline; a handle that came down past it within a control step is put back there, its fall taken
up. At that step the clove is consumed by rule: the share of it outside the chamber's outline is pinched off (waste,
drawn on the rim), the clove's body leaves the scene and the chamber is drawn filled with crushed garlic to where the
clove's top was. From then on the hinge carries the clove's resistance (press_on()): a stiff spring capped at the
yield, whose rest is the clove's top as crushed so far. It sinks with the load (CRUSHED at the yield; with the
spring's own give the plunger has then gone COMPRESSION from the touch) and, once the clove yields, follows the plunger
down SPRINGBACK behind it (what comes back when the press is let go). It pushes and never pulls: a handle lifted off
the clove is free. Past the yield, garlic goes through the sieve in proportion to the plunger's travel, THROUGH_MOST of the pressed
part with the face 3 mm over the floor. Of what goes through, DROPPED falls straight down onto whatever is under the
sieve, HANGING hangs under it as strands (one per hole), unless the sieve rests on the board: then it all goes onto
the board. Garlic moves to the knife and from it only by contact, read from the simulator's contact sensor on the
knife (scrape(), wipe()): a strand goes onto the knife when the blade presses on the sieve with SCRAPE_PRESS or more
and an edge of the blade slides past the strand's hole with the blade on the sieve there (lying flat, a long edge
passes under the hole; standing on its edge, a corner of that edge does); the knife's load leaves it when the blade
presses on the saucer with WIPE_PRESS or more and its loaded part (along the corner that took garlic, BAND either side
of where it took it) touches the saucer, well or rim, and is drawn along it: in proportion to the length drawn, all of
it after WIPE_LENGTH. Near but not touching, or touching without sliding: nothing moves.
"""

from __future__ import annotations

import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import mechkit as mk  # noqa: E402
from mechkit import body, box, capsule, cylinder, hinge, sphere  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
SUCCESS_HOLD_S = 1.0
INSTRUCTION = "Press the garlic clove through the garlic press onto the saucer, scrape what clings to the press onto the saucer with the knife, then put the press and the knife down."
RUN_LIMIT_STEPS = 3000  # 150 s at 20 Hz (designer, 2026-11-20)
TOP = TABLE_TOP[SCENE]
FLOOR_Z = -0.0125  # the station's floor (yam_arena.station.FLOOR_Z)
GRAVITY = 9.81

WALNUT, ALUMINIUM, DARK, SLATE, STEEL, GARLIC = (0.33, 0.22, 0.15), (0.76, 0.77, 0.79), (0.18, 0.18, 0.19), (0.24, 0.26, 0.29), (0.72, 0.74, 0.77), (0.93, 0.89, 0.74)

# --- the garlic, as a material: every number the simulation takes for the clove, with its source -------------------
# "assumed": our estimate; the README ("The clove as a material") says how a bench measures it.
MATERIAL = {
    "length_mm": ((26.0, 31.0), "bench text (designer, 2026-11-20): one clove of 4 to 6 g, 26 to 31 mm long, taken from a bag of ready-peeled garlic cloves as supermarkets sell them (for instance Christopher Ranch Peeled Garlic), the bag within its use-by date and kept closed in a refrigerator at 2 to 4 C (36 to 40 F); the clove taken out of the refrigerator not more than 10 minutes before the trial and patted dry with a paper towel; a clove with a cut or bruised surface is not used"),
    "width_mm": ((14.0, 19.0), "assumed from the size of such cloves"),
    "thickness_mm": ((11.0, 15.0), "assumed from the size of such cloves"),
    "mass_g": ((4.0, 6.0), "bench text (as length_mm)"),
    "friction": (0.3, "assumed: a peeled clove is slippery"),
    "yield_at_plunger_N": (250.0, "assumed; a bench measures it with the press on a kitchen scale, the peak reading while a clove goes through"),
    "compression_to_yield_mm": (3.0, "assumed; a bench measures it as the plunger's travel from touching the clove to that peak"),
    "springback_mm": (1.0, "assumed: how far the plunger comes back up when the press is let go"),
    "through_at_3mm": (0.85, "assumed: with the plunger 3 mm from the floor 85 % of the clove's weight is through the sieve, 15 % stays as skin and fibre"),
    "dropped_share": (0.30, "assumed: of what is through, 30 % drops straight down"),
    "hanging_share": (0.70, "assumed: of what is through, 70 % hangs under the sieve as strands, until a blade scrapes them off (scrape())"),
    "scrape_press_N": (0.5, "set from a reading: the blade presses on the sieve with at least this, as the simulator's contact sensor reads it, for its edge to take a strand (the scripted scrape read 1.18 N, median over its run across, trial b3g16; a blade near but not touching reads 0)"),
    "edge_touch_mm": (0.5, "assumed: the blade's face this near the sieve under a hole touches it there (the solver keeps touching bodies within a few hundredths of a millimetre)"),
    "proud_mm": (1.5, "assumed: the scraped garlic stands this far proud of the blade's face along the edge that took it; that edge this near the saucer's surface puts the garlic on it"),
    "wipe_press_N": (0.5, "set from a reading, as for the sieve: the blade presses on the saucer with at least this while it is wiped (the scripted wipe read 1.19 N on the rim, median, trial b3g16)"),
    "wipe_mm": (30.0, "assumed: the loaded edge drawn this far along the saucer leaves all its garlic there, in proportion before"),
    "band_mm": (3.0, "assumed: a strand's garlic lies along the edge that took it, this far either side of where it was taken (strands taken 5.8 mm apart join up)"),
    "strand_mm": ((6.0, 10.0), "designer: the garlic hangs as strands 6 to 10 mm long"),
}
CLOVE_FRICTION = MATERIAL["friction"][0]
YIELD_N = MATERIAL["yield_at_plunger_N"][0]
COMPRESSION = MATERIAL["compression_to_yield_mm"][0] / 1000.0
SPRINGBACK = MATERIAL["springback_mm"][0] / 1000.0
THROUGH_MOST = MATERIAL["through_at_3mm"][0]
THROUGH_FACE = 0.003  # m: the plunger's face this far over the floor: THROUGH_MOST is through
DROPPED, HANGING = MATERIAL["dropped_share"][0], MATERIAL["hanging_share"][0]
SCRAPE_PRESS, WIPE_PRESS = MATERIAL["scrape_press_N"][0], MATERIAL["wipe_press_N"][0]  # N
EDGE_TOUCH, PROUD, WIPE_LENGTH, BAND = (MATERIAL[k][0] / 1000.0 for k in ("edge_touch_mm", "proud_mm", "wipe_mm", "band_mm"))  # m
CLOVES = tuple(f"clove_{k}" for k in range(4))  # declared below; named here so that the press's upper part can pass them

# --- the board: walnut, 250 x 150 x 30 mm, about 0.73 kg (650 kg/m3, assumed), loose, its long side along y; dark, so that
# the pale clove and the aluminium press show on it (designer, 10:00) ---
BOARD_X, BOARD_Y, BOARD_T = (0.205, 0.355), (-0.285, -0.035), 0.030
BOARD_MASS = 0.73
BOARD_FRICTION = (0.4, 0.3)  # walnut on the laminate bench and under aluminium: assumed
BOARD_SPOTS = [(x, y) for y in np.linspace(-0.275, -0.050, 14) for x in (-0.015, 0.0, 0.015)]  # where garlic pressed onto the board is drawn (board frame)
BOARD_BLOB = 0.0003  # kg a drawn spot holds
_board_shapes = [box((BOARD_X[1] - BOARD_X[0], BOARD_Y[1] - BOARD_Y[0], BOARD_T))]
BOARD_SHAPES = range(len(_board_shapes), len(_board_shapes) + len(BOARD_SPOTS))
_board_mid = ((BOARD_X[0] + BOARD_X[1]) / 2, (BOARD_Y[0] + BOARD_Y[1]) / 2)
_board_shapes += [cylinder(0.0045, 0.002, pos=(x, y - _board_mid[1], BOARD_T / 2 + 0.001), color=GARLIC, solid=False) for x, y in BOARD_SPOTS]
body("board", _board_shapes, pos=(*_board_mid, TOP + BOARD_T / 2), mass=BOARD_MASS, color=WALNUT, friction=BOARD_FRICTION)

# --- the garlic press: Zyliss Susi 3, cast aluminium, 155 x 26 x 57 mm closed (the maker's page), about 150 g (our
# figure: the maker states no weight); its clip-in cleaning tool is taken out. Every other size is our reading of the
# maker's product pictures. In the press's own coordinates (m): u along it from its tip (the hinge end) to the grip
# end, v across, w up from the underside of the lower handle, which is also the underside of the sieve. The lower
# part: lower handle and chamber (inside 34 x 24 mm, 24 mm deep; its floor the sieve, thirty 2.5 mm holes); the upper
# part: upper handle and plunger (face 31 x 22 mm, its middle 26 mm from the hinge pin). The two parts' masses are
# assumed: 80 and 70 g. ---
PRESS_X = 0.280  # the press's middle line, along y
START_TIP_Y = BOARD_Y[1] - 0.010  # where its tip lies at the start: 10 mm inside the board's left edge, the press wholly on the board
PRESSING_OVERHANG = 0.051  # m of the press past the board's edge where it is pressed: the chamber's outer wall 2 mm past the edge
HINGE_UW = (0.004, 0.014)
CHAMBER_U, CHAMBER_V, FLOOR_W, RIM_W = (0.013, 0.047), 0.012, 0.002, 0.026  # the chamber's inside: u span, half width, floor top, rim
PLUNGER_U, PLUNGER_V, CLOSED_FACE_W = (0.0145, 0.0455), 0.011, 0.003  # the plunger's face; closed it stands 1 mm over the floor
PLUNGER_MID = ((PLUNGER_U[0] + PLUNGER_U[1]) / 2, CLOSED_FACE_W)
PLUNGER_ARM = PLUNGER_MID[0] - HINGE_UW[0]  # 26 mm: the clove's push on the face acts this far from the pin
YIELD_TORQUE = YIELD_N * PLUNGER_ARM  # 6.5 N m
LOWER_MASS, UPPER_MASS = 0.080, 0.070
PRESS_FRICTION = (0.4, 0.3)  # aluminium on walnut: assumed
OPEN_MOST = math.radians(120.0)  # the upper handle's stop


def uvw(u: float, v: float, w: float) -> tuple[float, float, float]:
    """A point of the press (u, v, w, m) in the frame of either part: origin on the hinge pin, x across (v), y towards
    the tip (u decreases), z up (w)."""
    return (v, -(u - HINGE_UW[0]), w - HINGE_UW[1])


def part(u0: float, u1: float, v0: float, v1: float, w0: float, w1: float, **kw):
    """A box of the press between those planes (m)."""
    return box((v1 - v0, u1 - u0, w1 - w0), pos=uvw((u0 + u1) / 2, (v0 + v1) / 2, (w0 + w1) / 2), **kw)


HOLES = [(float(u), float(v)) for u in np.linspace(0.0155, 0.0445, 6) for v in np.linspace(-0.0085, 0.0085, 5)]  # the sieve's thirty holes
STRANDS = [0.006 + 0.004 * ((7 * k) % 10) / 9 for k in range(len(HOLES))]  # each hole's strand, 6 to 10 mm long
MASH = [(FLOOR_W + 0.0015 * k, FLOOR_W + 0.0015 * (k + 1)) for k in range(13)]  # layers of crushed garlic in the chamber (w spans)
RIM_SPOTS = [(u, side * (CHAMBER_V + 0.0005)) for side in (1.0, -1.0) for u in (0.020, 0.030, 0.040)] + [(u, v) for u in (0.012, 0.048) for v in (-0.006, 0.006)]
LOWER_SHAPES = [
    part(*CHAMBER_U, -CHAMBER_V, CHAMBER_V, 0.0, FLOOR_W),  # the sieve (floor)
    part(0.011, 0.049, CHAMBER_V, CHAMBER_V + 0.001, 0.0, RIM_W), part(0.011, 0.049, -CHAMBER_V - 0.001, -CHAMBER_V, 0.0, RIM_W),  # side walls
    part(0.011, CHAMBER_U[0], -CHAMBER_V, CHAMBER_V, 0.0, RIM_W), part(CHAMBER_U[1], 0.049, -CHAMBER_V, CHAMBER_V, 0.0, RIM_W),  # end walls
    part(0.0, 0.011, 0.010, 0.013, 0.004, 0.024), part(0.0, 0.011, -0.013, -0.010, 0.004, 0.024),  # the hinge's ears
    part(0.049, 0.155, -0.013, 0.013, 0.0, 0.010),  # the lower handle
]
LOWER_SHAPES += [cylinder(0.00125, 0.0002, pos=uvw(u, v, w), color=DARK, solid=False) for u, v in HOLES for w in (-0.0001, FLOOR_W + 0.0001)]
STRAND_SHAPES = range(len(LOWER_SHAPES), len(LOWER_SHAPES) + len(HOLES))
LOWER_SHAPES += [cylinder(0.0011, length, pos=uvw(u, v, -length / 2), color=GARLIC, solid=False) for (u, v), length in zip(HOLES, STRANDS)]
MASH_SHAPES = range(len(LOWER_SHAPES), len(LOWER_SHAPES) + len(MASH))
LOWER_SHAPES += [part(CHAMBER_U[0] + 0.0003, CHAMBER_U[1] - 0.0003, -CHAMBER_V + 0.0003, CHAMBER_V - 0.0003, w0, w1, color=GARLIC, solid=False) for w0, w1 in MASH]
RIM_SHAPES = range(len(LOWER_SHAPES), len(LOWER_SHAPES) + len(RIM_SPOTS))
LOWER_SHAPES += [sphere(0.004, pos=uvw(u, v, RIM_W + 0.003), color=GARLIC, solid=False) for u, v in RIM_SPOTS]
UPPER_SHAPES = [
    part(*PLUNGER_U, -PLUNGER_V, PLUNGER_V, CLOSED_FACE_W, RIM_W, passes=CLOVES),  # the plunger
    part(0.002, 0.050, -0.013, 0.013, RIM_W, 0.038, passes=CLOVES),  # the cap over the chamber
    part(0.0, 0.0105, -0.0095, 0.0095, 0.008, RIM_W, passes=CLOVES),  # the tongue round the hinge pin, between the ears
    part(0.050, 0.058, -0.013, 0.013, 0.011, 0.038, passes=CLOVES),  # the neck
    part(0.058, 0.155, -0.013, 0.013, 0.011, 0.023, passes=CLOVES),  # the upper handle
]
START_HINGE = np.array([PRESS_X, START_TIP_Y - HINGE_UW[0], TOP + BOARD_T + HINGE_UW[1]])
body("press", LOWER_SHAPES, pos=tuple(START_HINGE), mass=LOWER_MASS, color=ALUMINIUM, friction=PRESS_FRICTION)
body("upper", UPPER_SHAPES, pos=tuple(START_HINGE), mass=UPPER_MASS, color=ALUMINIUM, friction=PRESS_FRICTION)
# Opening turns the upper part about -x: its grip end (towards -y) rises. Travel 0: closed. The hinge's drive is the
# clove's resistance (see press_on()): a stiff spring, its force capped at 0 while no clove is under the plunger.
GIVE_ANGLE = SPRINGBACK / PLUNGER_ARM  # rad
# N m/rad: twice the yield's torque SPRINGBACK behind the rest, so that the cap at the yield, not the spring, sets
# where the clove gives. (A joint's travel reads more torque than it carries when its two bodies are both held: 1.31
# times on this hinge with the lower part welded, trial b3g6; a yield set by the spring would come early.)
CLOVE_K = 2.0 * YIELD_TORQUE / GIVE_ANGLE
CRUSHED = COMPRESSION - YIELD_TORQUE / CLOVE_K * PLUNGER_ARM  # m the clove is crushed at its yield: with the spring's own give, COMPRESSION from the touch
LIFTED = math.radians(0.1)  # the handle this far over the clove's crushed top is off it
hinge("hinge", "upper", "press", anchor=tuple(START_HINGE), axis=(-1.0, 0.0, 0.0), limits=(0.0, OPEN_MOST), spring=CLOVE_K, rest=0.0, damping=0.3)
mk.weld("press_hold", "press", None, made=False)  # trials.py only: the press held where it stands (lifted for the scraping check)


def opened(u: float, w: float, phi: float) -> tuple[float, float]:
    """(u, w) of a point of the upper part when the press is opened by ``phi`` rad (from closed)."""
    du, dw = u - HINGE_UW[0], w - HINGE_UW[1]
    c, s = math.cos(phi), math.sin(phi)
    return HINGE_UW[0] + c * du - s * dw, HINGE_UW[1] + s * du + c * dw


def face_height(phi: float, u: float | None = None) -> float:
    """m from the chamber's floor up to the plunger's face at ``u`` along the press (its middle if None), the press
    opened ``phi``. The face is a straight edge in this side view."""
    (u0, w0), (u1, w1) = opened(PLUNGER_U[0], CLOSED_FACE_W, phi), opened(PLUNGER_U[1], CLOSED_FACE_W, phi)
    if u is None:
        return opened(*PLUNGER_MID, phi)[1] - FLOOR_W
    if abs(u1 - u0) < 1e-6:
        return max(w0, w1) - FLOOR_W
    s = min(1.0, max(0.0, (u - u0) / (u1 - u0)))
    return w0 + s * (w1 - w0) - FLOOR_W


def _middle(shapes) -> np.ndarray:
    """The volume-weighted middle of a part's solid shapes in its frame: where the simulator puts its centre of mass
    (it spreads a body's mass over its colliders at one density)."""
    volumes, middles = [], []
    for s in shapes:
        if not s.solid:
            continue
        if s.kind == "box":
            volume = float(np.prod(s.dims))
        elif s.kind == "cylinder":
            volume = math.pi * s.dims[0] ** 2 * s.dims[1]
        elif s.kind == "capsule":
            volume = math.pi * s.dims[0] ** 2 * s.dims[1] + 4.0 / 3.0 * math.pi * s.dims[0] ** 3
        else:
            volume = 4.0 / 3.0 * math.pi * s.dims[0] ** 3
        volumes.append(volume)
        middles.append(np.asarray(s.pos, dtype=float))
    return np.average(np.array(middles), axis=0, weights=volumes)


def centre_u(phi: float, lower: float = LOWER_MASS, upper: float = UPPER_MASS) -> float:
    """m from the press's tip to the centre of mass of its two parts, along the press, the upper part opened ``phi``
    rad. Lying with its tip past the board's edge by more than this, the press tips over the edge."""
    lo, up = _middle(LOWER_SHAPES), _middle(UPPER_SHAPES)
    lower_u = HINGE_UW[0] - lo[1]
    upper_u, _ = opened(HINGE_UW[0] - up[1], HINGE_UW[1] + up[2], phi)
    return (lower * lower_u + upper * upper_u) / (lower + upper)


# --- the saucer: slate-grey stoneware, 150 mm across, 20 mm high, 200 g (assumed), loose; against the board's left
# edge, under where the chamber comes when the press is at the board's edge ---
SAUCER_XY, SAUCER_R, SAUCER_H, SAUCER_MASS = (0.280, 0.040), 0.075, 0.020, 0.200
WELL_R, WELL_W = 0.052, 0.006  # the flat well inside the rim and its top over the bench
_rise = SAUCER_H - WELL_W
RIM_OUTER = SAUCER_R - 0.003 * math.sin(math.atan2(_rise, SAUCER_R - WELL_R))  # the rim's top edge: its 3 mm thickness reaches out to the 75 mm below it


def _rim(count: int = 24) -> list:
    shapes = []
    run = RIM_OUTER - WELL_R
    tilt, length = math.atan2(_rise, run), math.hypot(_rise, run)
    width = 2 * math.pi * RIM_OUTER / count * 1.06  # wide enough to close the gaps at the outer edge (at the mean radius they let the bench show: b3g2)
    for k in range(count):
        a = 2 * math.pi * k / count
        radial, tangent = np.array([math.cos(a), math.sin(a), 0.0]), np.array([-math.sin(a), math.cos(a), 0.0])
        along = math.cos(tilt) * radial + math.sin(tilt) * np.array([0.0, 0.0, 1.0])
        normal = np.cross(along, tangent)
        middle = (WELL_R + RIM_OUTER) / 2 * radial + np.array([0.0, 0.0, (WELL_W + SAUCER_H) / 2]) - 0.0015 * normal
        shapes.append(box((length, width, 0.003), pos=tuple(middle), quat=mk.matrix_to_quat(np.column_stack([along, tangent, normal]))))
    return shapes


def saucer_surface(r: float) -> float | None:
    """Height (m over the bench) of the saucer's top surface at ``r`` from its middle; None outside it."""
    if r <= WELL_R:
        return WELL_W
    if r <= RIM_OUTER:
        return WELL_W + (r - WELL_R) * _rise / (RIM_OUTER - WELL_R)
    return None


SAUCER_SPOTS = [(x, y) for x in np.arange(-0.045, 0.0451, 0.010) for y in np.arange(-0.045, 0.0451, 0.010) if math.hypot(x, y) <= 0.046]  # garlic on the saucer
SAUCER_BLOB = 0.00035  # kg a drawn spot holds
# The well's plate reaches 3 mm in under the rim: the staves' straight inner edges leave up to 0.5 mm open round the
# well's edge, where the bench showed through as a light ring (b3g5)
_saucer = [cylinder(0.045, 0.004, pos=(0.0, 0.0, 0.002)), cylinder(WELL_R + 0.003, 0.002, pos=(0.0, 0.0, 0.005)), *_rim()]
SAUCER_SHAPES = range(len(_saucer), len(_saucer) + len(SAUCER_SPOTS))
_saucer += [cylinder(0.0055, 0.004, pos=(float(x), float(y), WELL_W + 0.002), color=GARLIC, solid=False) for x, y in SAUCER_SPOTS]
body("saucer", _saucer, pos=(*SAUCER_XY, TOP), mass=SAUCER_MASS, color=SLATE, friction=(0.5, 0.4))

# --- the knife: a stainless table knife, 210 mm, 70 g, blunt (handle 95 x 18 x 9 mm, blade 115 x 20 x 2 mm: assumed);
# on the bench along x, the handle towards the arms. Its frame at the handle's middle ---
KNIFE_HANDLE, KNIFE_BLADE, KNIFE_MASS = (0.095, 0.018, 0.009), (0.115, 0.020, 0.002), 0.070
KNIFE_AT = (0.2175, 0.200)  # the handle's middle: the handle from x 0.170 to 0.265, the blade on to 0.380
BLADE_X = (KNIFE_HANDLE[0] / 2, KNIFE_HANDLE[0] / 2 + KNIFE_BLADE[0])  # the blade's span along the knife (to the end of its round tip)
BLADE_Z = (-KNIFE_HANDLE[2] / 2, -KNIFE_HANDLE[2] / 2 + KNIFE_BLADE[2])  # its underside and top face (knife frame)
_blade_x = KNIFE_HANDLE[0] / 2 + (KNIFE_BLADE[0] - KNIFE_BLADE[1] / 2) / 2  # the blade's straight part, then its round tip
KNIFE_SPOTS = [float(x) for x in np.arange(BLADE_X[0] + 0.004, BLADE_X[1] - 0.0119, 0.007)]  # places along the blade where garlic is drawn
KNIFE_CORNERS = ((1, 1), (1, -1), (-1, 1), (-1, -1))  # (face, edge) of the blade that can carry garlic: top or underside, along its +y or -y edge
KNIFE_SHAPES = [
    box(KNIFE_HANDLE, pos=(0.0, 0.0, 0.0)),
    box((KNIFE_BLADE[0] - KNIFE_BLADE[1] / 2, KNIFE_BLADE[1], KNIFE_BLADE[2]), pos=(_blade_x, 0.0, (BLADE_Z[0] + BLADE_Z[1]) / 2)),
    cylinder(KNIFE_BLADE[1] / 2, KNIFE_BLADE[2], pos=(BLADE_X[1] - KNIFE_BLADE[1] / 2, 0.0, (BLADE_Z[0] + BLADE_Z[1]) / 2)),
]
LOAD_SHAPES = {}  # (face, edge) -> the shapes drawing garlic there: a strip 4 mm wide along that edge on that face, standing PROUD off it
for _face, _edge in KNIFE_CORNERS:
    LOAD_SHAPES[(_face, _edge)] = range(len(KNIFE_SHAPES), len(KNIFE_SHAPES) + len(KNIFE_SPOTS))
    KNIFE_SHAPES += [box((0.007, 0.004, 0.0015), pos=(x, _edge * (KNIFE_BLADE[1] / 2 - 0.002), BLADE_Z[1] + 0.00075 if _face > 0 else BLADE_Z[0] - 0.00075), color=GARLIC, solid=False)
                     for x in KNIFE_SPOTS]  # fmt: skip
body("knife", KNIFE_SHAPES, pos=(*KNIFE_AT, TOP + KNIFE_HANDLE[2] / 2), mass=KNIFE_MASS, color=STEEL, friction=(0.4, 0.35))
mk.sense_contact("knife", "press", "saucer")  # the knife's contact with the press's lower part (the sieve) and with the saucer: scrape(), wipe()

# --- garlic dropped onto the bare bench: one heap, drawn where the last drop landed (moved by the task) ---
SPILL_PARK = np.array([0.40, 0.0, FLOOR_Z + 0.01])
body("spill", [cylinder(0.006, 0.003, pos=(0.0, 0.0, 0.0015), color=GARLIC, solid=False)], pos=tuple(SPILL_PARK), kinematic=True, color=GARLIC)

# --- the cloves: four sizes within the material table's ranges (length, width, thickness in m, mass in kg); one lies
# on the board in a trial, by seed. Each is four convex parts: a flat-bottomed blunt root end, a flat-bottomed
# narrower middle, a rounded back along the length, a rounded tip. In its frame: x from the root end to the tip, the
# flat face down at z = -thickness/2 ---
CLOVE_SIZES = ((0.0285, 0.0165, 0.013, 0.005), (0.026, 0.014, 0.011, 0.004), (0.031, 0.019, 0.015, 0.006), (0.031, 0.015, 0.012, 0.005))


def clove_shapes(length: float, width: float, thick: float) -> list:
    root_r, waist_r, tip_r = 0.48 * width, 0.36 * width, 0.22 * width
    root_x, tip_x = -length / 2 + root_r, length / 2 - tip_r
    waist_x = (root_x + tip_x) / 2 + 0.1 * length
    back_r = thick / 2
    return [
        cylinder(root_r, 0.55 * thick, pos=(root_x, 0.0, -thick / 2 + 0.275 * thick)),
        cylinder(waist_r, 0.45 * thick, pos=(waist_x, 0.0, -thick / 2 + 0.225 * thick)),
        capsule(back_r * 0.98, max(1e-4, tip_x - root_x - back_r), pos=((root_x + tip_x) / 2 - 0.05 * length, 0.0, -thick / 2 + back_r), axis="x"),
        sphere(tip_r, pos=(tip_x, 0.0, -thick / 2 + tip_r * 1.1)),
    ]


def _inside(shape, points: np.ndarray) -> np.ndarray:
    """Which of ``points`` (body frame, n x 3) lie inside a solid primitive."""
    local = (points - np.asarray(shape.pos)) @ mk.quat_to_matrix(shape.quat)  # into the shape's frame (its axis along z)
    if shape.kind == "sphere":
        return np.linalg.norm(local, axis=1) <= shape.dims[0]
    if shape.kind == "cylinder":
        return (np.hypot(local[:, 0], local[:, 1]) <= shape.dims[0]) & (np.abs(local[:, 2]) <= shape.dims[1] / 2)
    if shape.kind == "capsule":
        along = np.clip(local[:, 2], -shape.dims[1] / 2, shape.dims[1] / 2)
        return np.linalg.norm(local - np.column_stack([np.zeros_like(along), np.zeros_like(along), along]), axis=1) <= shape.dims[0]
    return np.all(np.abs(local) <= np.asarray(shape.dims) / 2, axis=1)


def fill(shapes, step: float = 0.0012) -> np.ndarray:
    """Points on a grid of ``step`` inside the shapes (body frame): the body's volume, evenly."""
    lows, highs = zip(*(s.bounds() for s in shapes))
    low, high = np.min(lows, axis=0), np.max(highs, axis=0)
    axes = [np.arange(lo + step / 2, hi, step) for lo, hi in zip(low, high)]
    grid = np.stack(np.meshgrid(*axes, indexing="ij"), axis=-1).reshape(-1, 3)
    keep = np.zeros(len(grid), dtype=bool)
    for s in shapes:
        keep |= _inside(s, grid)
    return grid[keep]


PARKED = {name: np.array([0.30 + 0.05 * k, 0.0, FLOOR_Z + CLOVE_SIZES[k][2] / 2 + 0.001]) for k, name in enumerate(CLOVES)}  # on the floor under the bench
CLOVE_POINTS = {}
for _k, _name in enumerate(CLOVES):
    _l, _w, _t, _m = CLOVE_SIZES[_k]
    _shapes = clove_shapes(_l, _w, _t)
    CLOVE_POINTS[_name] = fill(_shapes)
    body(_name, _shapes, pos=tuple(PARKED[_name]), mass=_m, color=GARLIC, friction=(CLOVE_FRICTION, CLOVE_FRICTION), friction_combine="min")

CLOVE_XY = (0.232, -0.065)  # on the board's near left part, beside the press's head, where no arm's shadow falls at the start (designer, 10:00)
OBJECTS = list(mk.MECH.bodies.values())
PARTS = ("press", "upper", "knife", "saucer", "board")  # whose rest is read for 1.0
SOLID = {name: [s.bounds() for s in mk.MECH.bodies[name].shapes if s.solid] for name in ("press", "upper", "knife", "saucer", "board")}

# --- the rules ---
SAUCER_FULL, SAUCER_PART, THROUGH_ENOUGH, WASTE_MOST = 0.65, 0.45, 0.70, 1.0 / 3.0  # shares of the clove's weight
ON_BENCH = 0.02  # m: the saucer or the board this far under the bench top has left it
LYING = 0.003  # m: a lying press or knife has its lowest point within this of the board's or the bench's top
RESTING, CLEAR, SETTLE_S = 0.02, 0.03, 0.25  # m/s over the last SETTLE_S; m both grippers keep from press and knife
SIDES = ("left", "right")
STATE: dict = {}


def lie(seed: int | None) -> tuple[int, bool, float, float, float]:
    """(which clove, on its side, its turn about the vertical in rad, x, y) for a trial's seed. Seed 0 and no seed: the
    middle-sized clove on its flat face, turned 25 degrees from the press's line, at CLOVE_XY."""
    if not seed:
        return 0, False, math.radians(25.0), *CLOVE_XY
    rng = np.random.default_rng(int(seed))
    return int(seed) % len(CLOVES), bool(rng.integers(2)), float(rng.uniform(-math.pi, math.pi)), CLOVE_XY[0] + float(rng.uniform(-0.01, 0.01)), CLOVE_XY[1] + float(rng.uniform(-0.01, 0.01))


def clove_pose(k: int, side: bool, yaw: float, x: float, y: float) -> tuple[np.ndarray, tuple]:
    """Where clove k lies on the board: on its flat face, or on its side (turned 90 degrees about its length)."""
    _, width, thick, _ = CLOVE_SIZES[k]
    rot = mk.axis_angle((0.0, 0.0, 1.0), yaw) @ (mk.axis_angle((1.0, 0.0, 0.0), math.pi / 2) if side else np.eye(3))
    return np.array([x, y, TOP + BOARD_T + (width if side else thick) / 2 + 0.0005]), mk.matrix_to_quat(rot)


def begin(seed: int | None = None) -> dict:
    """A new trial's state (no simulator needed). Amounts in kg."""
    STATE.clear()
    k, side, yaw, x, y = lie(seed)
    STATE.update(seed=seed, clove=CLOVES[k], mass=CLOVE_SIZES[k][3], lie=(k, side, yaw, x, y), touched=False, top=None, inside=0.0, waste=0.0,
                 lowest=None, rest=None, cap=0.0, touch=None, crush=0.0, contact=False, through=0.0, hanging=[0.0] * len(HOLES), knife=0.0, saucer=[0.0] * len(SAUCER_SPOTS),
                 board=[0.0] * len(BOARD_SPOTS), bench=0.0, edges=None, loaded=[], carried=0.0, wipe_at=None, wiped=0.0, track=[], events=[])  # fmt: skip
    return STATE


@mk.on_reset
def start(seed) -> None:
    begin(seed)
    for name, shapes in (("press", (*STRAND_SHAPES, *MASH_SHAPES, *RIM_SHAPES)), ("saucer", SAUCER_SHAPES), ("knife", [k for shapes in LOAD_SHAPES.values() for k in shapes]), ("board", BOARD_SHAPES), ("spill", (0,))):
        for k in shapes:
            mk.show(name, k, False)
    if mk.MECH.environment is not None:
        mk.set_spring("hinge", rest=0.0, most=0.0)  # no clove under the plunger
        mk.set_joint("hinge", OPEN_MOST)  # open, on its stop
        k, side, yaw, x, y = STATE["lie"]
        for name in CLOVES:
            if name == STATE["clove"]:
                mk.write_pose(name, *clove_pose(k, side, yaw, x, y))
            else:
                mk.write_pose(name, PARKED[name], (0.0, 0.0, 0.0, 1.0))
        mk.write_pose("spill", SPILL_PARK)


# --- where things are ---
def to_world(name: str, points) -> np.ndarray:
    position, rotation = mk.pose(name)
    return np.asarray(points, dtype=float) @ rotation.T + position


def to_body(name: str, points) -> np.ndarray:
    position, rotation = mk.pose(name)
    return (np.asarray(points, dtype=float) - position) @ rotation


def in_press(points) -> np.ndarray:
    """World points as (u, v, w) of the press's lower part (m)."""
    local = to_body("press", points)
    return np.column_stack([HINGE_UW[0] - local[:, 1], local[:, 0], HINGE_UW[1] + local[:, 2]])


def press_point(u: float, v: float, w: float) -> np.ndarray:
    """A point of the lower part (u, v, w) in the world."""
    return to_world("press", np.array([uvw(u, v, w)]))[0]


def overhang() -> float:
    """m the press's tip stands past the board's left edge (negative: inside it)."""
    return float(to_body("board", np.array([press_point(0.0, 0.0, 0.0)]))[0][1] - (BOARD_Y[1] - BOARD_Y[0]) / 2)


DEPTH = HINGE_UW[1] - CLOSED_FACE_W  # m: the plunger's face lies this far from the pin's axis


def touch_angle(lower: np.ndarray) -> float | None:
    """For points (u, v, w) in the press's frame: the largest opening (rad) at which the plunger's face passes through
    one of them that it covers there; closing from above, the face first touches them at that opening. None if the
    face covers none of them at any opening. In the side view a point at distance r from the pin is on the face's line
    (DEPTH from the pin) when it is opened by the point's angle plus asin(DEPTH / r), sqrt(r^2 - DEPTH^2) along it."""
    du, dw = lower[:, 0] - HINGE_UW[0], lower[:, 2] - HINGE_UW[1]
    r = np.hypot(du, dw)
    along = HINGE_UW[0] + np.sqrt(np.maximum(r * r - DEPTH * DEPTH, 0.0))
    covered = (r > DEPTH) & (along >= PLUNGER_U[0]) & (along <= PLUNGER_U[1]) & (np.abs(lower[:, 1]) <= PLUNGER_V)
    if not covered.any():
        return None
    return float((np.arctan2(dw, du) + np.arcsin(np.minimum(1.0, DEPTH / np.maximum(r, DEPTH))))[covered].max())


def plunger_into_clove(phi: float) -> tuple[float, float, list] | None:
    """For the clove as it lies and the press opened ``phi``: (the opening at which the plunger's face first touched
    the clove's part inside the chamber's outline, closing from above; the share of the clove inside the outline; the
    middles (u, v) of its parts outside the outline, one for each side they stick out at). None while the face is
    above that part (or no part is inside the outline). Read in the press's own frame: it holds wherever the press
    stands."""
    lower = in_press(to_world(STATE["clove"], CLOVE_POINTS[STATE["clove"]]))
    inside = (lower[:, 0] >= CHAMBER_U[0]) & (lower[:, 0] <= CHAMBER_U[1]) & (np.abs(lower[:, 1]) <= CHAMBER_V)
    if not inside.any():
        return None
    touch = touch_angle(lower[inside])
    if touch is None or phi > touch:
        return None
    sides = []
    outside = lower[~inside]
    for pick in (outside[:, 1] > CHAMBER_V, outside[:, 1] < -CHAMBER_V, outside[:, 0] < CHAMBER_U[0], outside[:, 0] > CHAMBER_U[1]):
        if pick.any():
            sides.append(outside[pick, :2].mean(axis=0))
    return touch, float(inside.mean()), sides


def resting_in_chamber() -> bool:
    """The clove lies in or on the chamber: its lowest point at most 5 mm over the rim, in the press's frame."""
    points = in_press(to_world(STATE["clove"], CLOVE_POINTS[STATE["clove"]]))
    return bool(float(points[:, 2].min()) <= RIM_W + 0.005)


def through_share(top: float, lowest: float) -> float:
    """The share of the pressed garlic that has gone through the sieve: none until the plunger has gone COMPRESSION
    down from the clove's top (the yield), THROUGH_MOST once its face is THROUGH_FACE over the floor, in proportion
    between."""
    yield_at = top - COMPRESSION
    if yield_at <= THROUGH_FACE:
        return THROUGH_MOST if lowest < yield_at else 0.0
    return THROUGH_MOST * min(1.0, max(0.0, (yield_at - lowest) / (yield_at - THROUGH_FACE)))


def crushing(load: float) -> float:
    """m the clove is crushed under ``load`` N m on the plunger: in proportion up to CRUSHED at its yield."""
    return CRUSHED * min(1.0, max(0.0, load) / YIELD_TORQUE)


def _fill(spots: list, kept: list, at, amount: float, most: float) -> None:
    """Put ``amount`` into the drawn spots nearest ``at`` (each holds ``most``), spilling outwards."""
    order = np.argsort([math.hypot(x - at[0], y - at[1]) for x, y in spots])
    for k in order:
        if amount <= 0:
            break
        room = max(0.0, most - kept[k])
        if room > 0:
            take = min(room, amount)
            kept[k] += take
            amount -= take
    if amount > 0:
        kept[int(order[0])] += amount


def land(amount: float, at) -> str:
    """Garlic that falls from ``at`` (world) straight down: onto the saucer if it is over the saucer, else the board,
    else the bench. Returns where."""
    sx, sy = to_body("saucer", np.array([at]))[0][:2]
    if math.hypot(sx, sy) <= SAUCER_R:
        _fill(SAUCER_SPOTS, STATE["saucer"], (sx, sy), amount, SAUCER_BLOB)
        return "saucer"
    bx, by, bz = to_body("board", np.array([at]))[0]
    if abs(bx) <= (BOARD_X[1] - BOARD_X[0]) / 2 and abs(by) <= (BOARD_Y[1] - BOARD_Y[0]) / 2 and bz > 0:
        _fill([(x, y - _board_mid[1]) for x, y in BOARD_SPOTS], STATE["board"], (bx, by), amount, BOARD_BLOB)
        return "board"
    STATE["bench"] += amount
    STATE["spill_at"] = (float(at[0]), float(at[1]))
    return "bench"


def sieve_on_board() -> bool:
    """The sieve rests on the board: its middle over the board and within 3 mm of its top."""
    at = press_point(PLUNGER_MID[0], 0.0, 0.0)
    bx, by, bz = to_body("board", np.array([at]))[0]
    return abs(bx) <= (BOARD_X[1] - BOARD_X[0]) / 2 and abs(by) <= (BOARD_Y[1] - BOARD_Y[0]) / 2 and bz - BOARD_T / 2 <= 0.003


def press_on() -> None:
    """The plunger on the clove, once per control step: arrival, the resistance, what goes through and where. From the
    touch on the clove is the hinge's drive: a stiff spring capped at the yield whose rest is the clove's top as
    crushed so far. The rest sinks with the load the spring carries (crushing()) and, once the clove yields, follows
    the handle down SPRINGBACK behind it. It pushes and never pulls: with the handle lifted off, the drive gives
    nothing, and a handle that comes down onto the clove again is stopped where it was crushed to."""
    phi = mk.joint("hinge")
    if not STATE["touched"]:
        if not resting_in_chamber():
            return
        found = plunger_into_clove(phi)
        if found is None:
            return
        touch, inside, sides = found
        mass = STATE["mass"]
        top = face_height(touch)  # the face's middle over the floor as the face first touches the clove
        STATE.update(touched=True, top=top, touch=touch, inside=inside * mass, waste=(1.0 - inside) * mass, lowest=top, rest=touch, crush=0.0, contact=False,
                     touch_deg=round(math.degrees(touch), 2))  # fmt: skip
        STATE["rim_spots"] = sorted({int(np.argmin([math.hypot(u - at[0], v - at[1]) for u, v in RIM_SPOTS])) for at in sides}) if inside < 1.0 else []
        STATE["events"].append((round(mk.now(), 2), f"plunger on the clove at {math.degrees(touch):.1f} deg, {1000 * top:.1f} mm over the floor, {100 * inside:.0f} % inside the chamber's outline"))
        if mk.MECH.environment is not None:
            mk.write_pose(STATE["clove"], PARKED[STATE["clove"]], (0.0, 0.0, 0.0, 1.0))
            mk.redraw(4)
    if not STATE["contact"] and phi <= STATE["rest"]:  # the handle comes down onto the clove, first or again
        if mk.MECH.environment is not None and STATE["rest"] - phi > 1e-4:  # it came down past it within the last step (the upper part passes cloves): put back on it, its fall taken up
            upper = mk.joint_pose(mk.MECH.joints["hinge"], STATE["rest"], mk.pose("press"))
            mk.write_pose("upper", upper[0], mk.matrix_to_quat(upper[1]))
            STATE["events"].append((round(mk.now(), 2), f"the handle put back {math.degrees(STATE['rest'] - phi):.1f} deg onto the clove"))
        phi = STATE["rest"]
        STATE["contact"] = True
    elif STATE["contact"] and phi > STATE["rest"] + LIFTED:
        STATE["contact"] = False
    face = face_height(phi)
    STATE["lowest"] = min(STATE["lowest"], face)
    rest, cap = STATE["rest"], 0.0
    if STATE["contact"]:
        STATE["crush"] = max(STATE["crush"], crushing(CLOVE_K * (STATE["rest"] - phi)))
        rest, cap = min(rest, STATE["touch"] - STATE["crush"] / PLUNGER_ARM, phi + GIVE_ANGLE), YIELD_TORQUE
    if mk.MECH.environment is not None and (cap != STATE["cap"] or rest != STATE["rest"]):
        mk.set_spring("hinge", rest=rest, most=cap)
    STATE.update(cap=cap, rest=rest)
    through = through_share(STATE["top"], STATE["lowest"]) * STATE["inside"]
    more = through - STATE["through"]
    if more > 1e-9:
        STATE["through"] = through
        if sieve_on_board():
            land(more, press_point(PLUNGER_MID[0], 0.0, 0.0) + np.array([0.0, 0.0, 0.001]))
        else:
            land(DROPPED * more, press_point(PLUNGER_MID[0], 0.0, -0.002))
            for k in range(len(HOLES)):
                STATE["hanging"][k] += HANGING * more / len(HOLES)


HOLES_SIEVE = np.array([uvw(u, v, 0.0) for u, v in HOLES])  # the holes' mouths on the sieve's underside, press frame


SLIDE_MIN = 0.00005  # m a hole moves past a blade's edge in one control step for it to count as sliding (1 mm/s at 20 Hz; the solver's tremble of a still blade is far less)


def _corner_passed(was, now, flat: bool) -> tuple[int, int] | None:
    """The corner of the blade's cross-section (face: +1 top, -1 underside; edge: +1, -1 the long edge at +-y) that a
    point moving from ``was`` to ``now`` ((y, z) in the knife's frame) slides past while on the blade (within
    EDGE_TOUCH of it). ``flat``: the blade lies flat on the sieve, and a point on a face that crosses a long edge
    counts (the garlic goes onto that face); else it stands on an edge, and a point on that edge that crosses either
    face's corner counts (the garlic goes onto the face the point comes from: the one that leads into the strand).
    Moving less than SLIDE_MIN past it: None."""
    (y0, z0), (y, z) = was, now
    half = KNIFE_BLADE[1] / 2
    if flat:
        for face, level in ((1, BLADE_Z[1]), (-1, BLADE_Z[0])):
            if abs(z - level) <= EDGE_TOUCH:  # on (or a hair into) that face's plane
                for edge in (1, -1):
                    if (y0 - edge * half) * (y - edge * half) <= 0.0 and abs(y - y0) >= SLIDE_MIN:
                        return face, edge
        return None
    for edge in (1, -1):
        if abs(y - edge * half) <= EDGE_TOUCH and abs(z - z0) >= SLIDE_MIN:  # on that edge, sliding
            if any((z0 - level) * (z - level) <= 0.0 for level in BLADE_Z):
                return (-1 if z > z0 else 1), edge  # from the underside's side (z rising): the underside leads
    return None


def scrape() -> None:
    """A strand goes onto the knife only by contact and scraping: while the blade presses on the press's lower part
    with SCRAPE_PRESS or more (the simulator's contact sensor, mean over the control step), an edge of the blade slides
    past the strand's hole with the blade on the sieve there (within EDGE_TOUCH): lying flat on the sieve (the sieve's
    face square to the blade's faces more than to its edges), a long edge passes under the hole; standing on its edge,
    a corner of that edge does. Near but not touching, touching without sliding, or pressed onto a hole: nothing. Where
    it rides is noted for wipe(): the corner that took it (a face and an edge), at that place along the blade."""
    holes = to_body("knife", to_world("press", HOLES_SIEVE))
    before, STATE["edges"] = STATE["edges"], holes
    if before is None or mk.contact_force("knife", "press") < SCRAPE_PRESS:
        return
    normal = mk.pose("knife")[1].T @ mk.pose("press")[1][:, 2]  # the sieve's face's normal in the knife's frame
    flat = abs(normal[2]) >= abs(normal[1])
    for k, (x, y, z) in enumerate(holes):
        if STATE["hanging"][k] <= 0.0 or not BLADE_X[0] <= x <= BLADE_X[1]:
            continue
        corner = _corner_passed(before[k][1:], (y, z), flat)
        if corner is None:
            continue
        STATE["knife"] += STATE["hanging"][k]
        STATE["hanging"][k] = 0.0
        STATE["carried"] = max(STATE["carried"], STATE["knife"])
        STATE["loaded"].append((*corner, float(x)))
        if mk.MECH.environment is not None:
            mk.redraw(2)


def saucer_gap(points: np.ndarray) -> np.ndarray:
    """m from points in the saucer's frame to its upper surface (the well, then the rim's slope up to its top edge),
    measured square to the surface; negative under it."""
    r, z = np.hypot(points[:, 0], points[:, 1]), points[:, 2]
    a, b = np.array([WELL_R, WELL_W]), np.array([RIM_OUTER, SAUCER_H])
    slope = (b - a) / np.linalg.norm(b - a)
    normal = np.array([-slope[1], slope[0]])  # up and inwards
    on_slope = np.column_stack([r, z]) - a
    t = on_slope @ slope
    gap = np.where(r <= WELL_R, z - WELL_W, np.where(t <= np.linalg.norm(b - a), on_slope @ normal, np.hypot(r - b[0], z - b[1])))
    return gap


def loaded_points() -> np.ndarray:
    """Where the garlic rides, in the knife's frame: along each corner (a face and an edge) that took some, BAND either
    side of every place it was taken there, a point every millimetre."""
    half, points = KNIFE_BLADE[1] / 2, []
    for face, edge in sorted({(f, e) for f, e, _ in STATE["loaded"]}):
        places = [x for f, e, x in STATE["loaded"] if (f, e) == (face, edge)]
        xs = np.arange(max(BLADE_X[0], min(places) - BAND), min(BLADE_X[1], max(places) + BAND) + 1e-9, 0.001)
        xs = [x for x in xs if min(abs(x - p) for p in places) <= BAND + 1e-9]
        points += [[x, edge * half, BLADE_Z[1] if face > 0 else BLADE_Z[0]] for x in xs]
    return np.array(points)


def wipe() -> None:
    """The knife's load leaves it only by wiping: while the blade presses on the saucer with WIPE_PRESS or more (the
    contact sensor) and its loaded part (loaded_points()) touches the saucer's surface, well or rim (within PROUD: the
    garlic stands that far proud of the blade), and is drawn along it, the garlic goes onto the saucer in proportion to
    the length drawn, all of it after WIPE_LENGTH. Near but not touching, touching without being drawn, or touching
    with another part of the blade: nothing."""
    if STATE["knife"] <= 0.0 or not STATE["loaded"]:
        STATE["wipe_at"] = None
        return
    points = to_body("saucer", to_world("knife", loaded_points()))
    touching = np.abs(saucer_gap(points)) <= PROUD
    before, STATE["wipe_at"] = STATE["wipe_at"], (points, touching)
    if before is None or len(before[0]) != len(points) or mk.contact_force("knife", "saucer") < WIPE_PRESS:
        return
    both = touching & before[1]
    if not both.any():
        return
    drawn = float(np.linalg.norm(points[both] - before[0][both], axis=1).mean())
    amount = min(STATE["knife"], STATE["carried"] * drawn / WIPE_LENGTH)
    if amount <= 0.0:
        return
    at = points[both].mean(axis=0)
    _fill(SAUCER_SPOTS, STATE["saucer"], at[:2], amount, SAUCER_BLOB)
    STATE["knife"] -= amount
    STATE["wiped"] += drawn
    if STATE["knife"] <= 1e-9:
        STATE["knife"] = 0.0
        STATE["events"].append((round(mk.now(), 2), f"{1000 * STATE['carried']:.2f} g wiped off the knife onto the saucer, drawn {1000 * STATE['wiped']:.1f} mm along it"))


def draw() -> None:
    """Every drawn piece of garlic as the amounts stand."""
    for k in range(len(HOLES)):
        mk.show("press", STRAND_SHAPES[k], STATE["hanging"][k] > 1e-7)
    mash = (min(STATE["top"], STATE["lowest"]) + FLOOR_W) if STATE["touched"] else None
    for k, (_, w1) in enumerate(MASH):
        mk.show("press", MASH_SHAPES[k], mash is not None and w1 <= mash + 0.00075)
    for k in range(len(RIM_SPOTS)):
        mk.show("press", RIM_SHAPES[k], k in STATE.get("rim_spots", ()) and STATE["waste"] > 1e-7)
    for k in range(len(SAUCER_SPOTS)):
        mk.show("saucer", SAUCER_SHAPES[k], STATE["saucer"][k] > 0.25 * SAUCER_BLOB)
    for k in range(len(BOARD_SPOTS)):
        mk.show("board", BOARD_SHAPES[k], STATE["board"][k] > 0.25 * BOARD_BLOB)
    left = STATE["knife"] / STATE["carried"] if STATE["carried"] > 0 else 0.0  # the share of what it took still on the knife
    for corner, shapes in LOAD_SHAPES.items():
        places = [x for f, e, x in STATE["loaded"] if (f, e) == corner]
        band = [k for k, x in enumerate(KNIFE_SPOTS) if places and min(abs(x - p) for p in places) <= BAND + 0.0035]  # the spots over its band
        keep = int(math.ceil(left * len(band) - 1e-9))
        shown = set(band[len(band) - keep :]) if keep > 0 else set()  # wiped off from the heel's end first, as a rim drawn towards the tip takes it
        for k, shape in enumerate(shapes):
            mk.show("knife", shape, k in shown)
    if STATE["bench"] > 0 and not STATE.get("spilled"):
        STATE["spilled"] = True
        mk.show("spill", 0, True)
        if mk.MECH.environment is not None:
            mk.write_pose("spill", (*STATE["spill_at"], TOP))


@mk.on_step
def step() -> None:
    if not mk.MECH.poses:
        return
    press_on()
    scrape()
    wipe()
    draw()
    track()


# --- rest, read from travel ---
def _marks() -> dict:
    return {name: mk.pose(name)[0].copy() for name in PARTS}


def track() -> None:
    """Keep where the parts are, every control step, for SETTLE_S back."""
    t, kept = mk.now(), STATE.setdefault("track", [])
    kept.append((t, _marks()))
    while len(kept) > 2 and kept[1][0] <= t - SETTLE_S:
        kept.pop(0)


def speeds() -> dict:
    kept = STATE.get("track", [])
    if len(kept) < 2 or kept[-1][0] <= kept[0][0]:
        return dict.fromkeys(PARTS)
    (t0, then), (t1, now) = kept[0], kept[-1]
    return {name: float(np.linalg.norm(now[name] - then[name])) / (t1 - t0) for name in now}


def at_rest() -> bool:
    return bool(all(v is not None and v < RESTING for v in speeds().values()))


# --- the score ---
def corners(name: str) -> np.ndarray:
    """The corners of a body's solid shapes' boxes, in the world."""
    local = np.array([[a[0] if i & 1 else b[0], a[1] if i & 2 else b[1], a[2] if i & 4 else b[2]] for a, b in SOLID[name] for i in range(8)])
    return to_world(name, local)


def lying(name: str) -> bool:
    """Set down: on the bench (its lowest point within LYING of the bench's top), or on the board (its lowest point
    over the board within LYING of the board's top, and no part of it lower than that: a press pressed past the
    board's edge lies on it, one tipped over the edge does not)."""
    world = corners(name)
    if abs(float(world[:, 2].min()) - TOP) <= LYING:
        return True
    local = to_body("board", world)
    over = (np.abs(local[:, 0]) <= (BOARD_X[1] - BOARD_X[0]) / 2) & (np.abs(local[:, 1]) <= (BOARD_Y[1] - BOARD_Y[0]) / 2)
    if not over.any():
        return False
    return bool(abs(float(local[over, 2].min()) - BOARD_T / 2) <= LYING and float(local[:, 2].min()) >= BOARD_T / 2 - LYING)


def distance_to(name: str, point) -> float:
    local = to_body(name, np.array([point]))[0]
    return min(float(np.linalg.norm(np.maximum(np.maximum(np.asarray(low) - local, local - np.asarray(high)), 0.0))) for low, high in SOLID[name])


def let_go() -> bool:
    return bool(all(distance_to(name, mk.gripper(side)[0]) >= CLEAR for side in SIDES for name in ("press", "upper", "knife")))


def on_bench() -> bool:
    return bool(all(mk.pose(name)[0][2] >= TOP - ON_BENCH for name in ("saucer", "board")))


def on_saucer() -> float:
    """The share of the clove's weight on the saucer as pressed garlic."""
    return sum(STATE["saucer"]) / STATE["mass"]


def level() -> float:
    """The score without the release: see ``score``."""
    if not on_bench() or STATE["waste"] > WASTE_MOST * STATE["mass"]:
        return 0.0
    share = on_saucer()
    if share >= SAUCER_PART:
        return 0.75
    if STATE["through"] >= THROUGH_ENOUGH * STATE["mass"]:
        return 0.5
    if STATE["touched"] and face_height(mk.joint("hinge")) <= min(STATE["top"], STATE["lowest"]) + SPRINGBACK + 0.0005:
        return 0.25
    return 0.0


def score(objects) -> float:
    """By where the garlic is at the end. 0 if the saucer or the board has left the bench, or more than a third of the
    clove was pinched off outside the chamber. 0.25: the clove in the chamber with the plunger on it; 0.5: 70 % of the
    clove's weight through the sieve; 0.75: 45 % of it on the saucer; 1.0: 65 % on the saucer, press and knife lying
    on the board or the bench, both grippers 3 cm or more from them, everything at rest. Anything else: 0."""
    found = level()
    if found == 0.75 and on_saucer() >= SAUCER_FULL and lying("press") and lying("knife") and at_rest() and let_go():
        return 1.0
    return found


def success(objects) -> bool:
    return score(objects) == 1.0


def amounts() -> dict:
    """g, rounded: where the clove's weight is."""
    g = lambda kg: round(1000 * kg, 3)  # noqa: E731
    return {"clove": g(STATE["mass"]), "pinched off": g(STATE["waste"]), "pressed": g(STATE["inside"]), "through": g(STATE["through"]),
            "hanging": g(sum(STATE["hanging"])), "on the knife": g(STATE["knife"]), "on the saucer": g(sum(STATE["saucer"])),
            "on the board": g(sum(STATE["board"])), "on the bench": g(STATE["bench"]),
            "in the chamber": g(STATE["inside"] - STATE["through"]) if STATE["touched"] else None}  # fmt: skip


def progress() -> str:
    a = amounts()
    return (f"hinge {math.degrees(mk.joint('hinge')):.1f} deg, face {1000 * face_height(mk.joint('hinge')):.1f} mm; through {a['through']} g, hanging {a['hanging']} g, "
            f"knife {a['on the knife']} g, saucer {a['on the saucer']} g; level {score(None):.2f}")  # fmt: skip


def report() -> dict:
    out = {"clove": STATE.get("clove"), "lie": STATE.get("lie"), "amounts (g)": amounts(), "events": STATE.get("events"), "touched at (deg)": STATE.get("touch_deg")}
    if mk.MECH.poses:
        out.update({"hinge (deg)": round(math.degrees(mk.joint("hinge")), 2), "face over the floor (mm)": round(1000 * face_height(mk.joint("hinge")), 2),
                    "press's tip past the board's edge (mm)": round(1000 * overhang(), 1), "on the saucer (share)": round(on_saucer(), 3),
                    "lying: press, knife": [lying("press"), lying("knife")], "on bench": on_bench()})  # fmt: skip
        out["speeds (mm/s)"] = {name: None if v is None else round(1000 * v, 2) for name, v in speeds().items()}
        if mk.MECH.grippers:
            out.update({"at rest": at_rest(), "let go": let_go(), "score": score(None)})
    return out
