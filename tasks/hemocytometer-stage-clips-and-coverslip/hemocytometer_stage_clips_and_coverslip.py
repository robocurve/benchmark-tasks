"""Seat a counting chamber in the microscope stage pocket, latch both stage clips, and lay the quartz coverslip flat.

Real product class: a precision double-grid glass cell-counting chamber (68 g optical glass block with twin silvered
rhodium counting platforms, H-moat channels, raised outer clip shoulders, and an offset frosted handling stem) paired
with a rigid 36 g fused-quartz hemocytometer coverslip on a 300 x 430 x 16 mm anodized microscope mechanical stage with
a 28 g, 95 N/m sprung front datum plunger and two 32 g spring-loaded pivoting stage clips captured by 28 g, 45 N/m
sprung brass detents.

Bimanual physics: the counting chamber starts 34 mm outside the stage pocket on the loading shelf while both spring
stage clips stand 52 deg open and the quartz coverslip rests in the front staging cradle. Seating the 68 g chamber into
the pocket compresses the 95 N/m front datum plunger (~1.8 N); releasing before both stage clips latch behind the
chamber's outer shoulders pushes the chamber back against the clip path so the clips cannot close, requiring one arm to
brace the chamber in the pocket while the other latches both clips and lays the coverslip flat across the 10 mm rails.
"""

from __future__ import annotations

import math
import pathlib
import sys

import numpy as np

_HERE = str(pathlib.Path(__file__).resolve().parent)
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

import arena_assets_fix  # noqa: E402, F401
import mechkit as mk  # noqa: E402  (must precede labkit so labkit wraps mechkit's import finder)
import labkit  # noqa: E402
import pitch_demo  # noqa: E402
from pitch_demo import Waypoint  # noqa: E402
from yam_arena import station  # noqa: E402
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402

SCENE = "station"
TOP = station.TABLE_TOP

# -- distinct PBR colours (GLASS_COLORS gives translucent optical glass in Isaac & MuJoCo) ----------------------------
STAGE_BLK = (0.14, 0.15, 0.18)
ANODIZED_SLATE = (0.25, 0.29, 0.34)
RUBBER_DARK = (0.09, 0.10, 0.11)
CHROME_CLIP = (0.78, 0.81, 0.85)
RHODIUM_GRID = (0.66, 0.72, 0.78)
BRASS_GOLD = (0.83, 0.66, 0.24)
KNURL_DARK = (0.32, 0.35, 0.39)
FROSTED_TEAL = (0.36, 0.58, 0.64)
CHAMBER_GLASS = (0.62, 0.82, 0.88)
QUARTZ_CYAN = (0.34, 0.78, 0.86)
CONDENSER_EMERALD = (0.18, 0.82, 0.56)
labkit.GLASS_COLORS = {CHAMBER_GLASS}

# -- 1:1 microscopy workcell geometry (metres, kg, N, deg) ------------------------------------------------------------
DECK_H = 0.016  # 16 mm anodized microscope stage height above tabletop (3 mm rubber feet + 13 mm deck)
DECK_Z = TOP + DECK_H
STAGE_X, STAGE_Y = 0.235, -0.045  # pocket target centre for the seated counting chamber (0 arm-arm, 0.08 mm IK!)
CHAMBER_START_DY = 0.034  # chamber starts 34 mm outside the pocket on the +Y loading shelf
STEM_X_OFF, STEM_Y_OFF = 0.060, 0.195  # offset +X/+Y handling grip -> world (0.295, +0.150) when seated (0 arm-arm!)
CS_RACK_X, CS_RACK_Y = 0.235, -0.148  # front staging cradle for the 36 g fused-quartz coverslip (in full top_cam view!)

CHAMBER_MASS = 0.068  # 68 g heavy optical glass double-grid counting chamber + frosted grip
COVERSLIP_MASS = 0.036  # 36 g thick ground fused-quartz hemocytometer cover block (>= 25 g)
DATUM_MASS = 0.028  # 28 g sprung front datum plunger (>= 25 g)
CLIP_MASS = 0.032  # 32 g chrome pivoting stage clip (>= 25 g)
DETENT_MASS = 0.028  # 28 g sprung brass clip detent catch (>= 25 g)

DATUM_K_NM = 95.0  # 95 N/m front datum spring (~1.8 N at pocket seat; ejects unbraced chamber to +19 mm!)
CLIP_OPEN_DEG = 52.0  # initial open angle of both pivoting stage clips
CLIP_K_NM = 0.045  # opening return spring (snaps unlatched clip back open to 54 deg)
DETENT_K_NM = 45.0  # 45 N/m brass clip detent spring (effortless cam-in in rigid PhysX, solid latch <= 9.5 deg!)

MOAT_RAIL_H = 0.010  # 10 mm height of the chamber's two central coverslip support rails above DECK_Z


def _build_stage():
    """300 x 430 x 16 mm anodized microscope mechanical stage with funnelled pocket, condenser lens, and coverslip rack."""
    foot_h, plate_t = 0.003, 0.013
    sy0 = 0.005
    dy_p = STAGE_Y - sy0
    dy_r = CS_RACK_Y - sy0
    boxes = [
        ((0.0, 0.0, foot_h + plate_t / 2), (0.300, 0.430, plate_t), STAGE_BLK),
        # Condenser illumination bezel & emerald optical aperture flush with DECK_H under the counting grid
        ((0.0, dy_p, DECK_H - 0.001), (0.038, 0.030, 0.002), ANODIZED_SLATE),
        ((0.0, dy_p, DECK_H - 0.0004), (0.024, 0.020, 0.001), CONDENSER_EMERALD),
        # Split front -Y reference stops (24 mm center slot clears datum_plunger; stops chamber front right at STAGE_Y!)
        ((-0.022, dy_p - 0.0225, DECK_H + 0.004), (0.020, 0.008, 0.008), BRASS_GOLD),
        ((0.022, dy_p - 0.0225, DECK_H + 0.004), (0.020, 0.008, 0.008), BRASS_GOLD),
        # Low 3.5 mm lateral X pocket walls + V-funnel mouth (guides 6 mm glass base while clearing clip hooks at z>=5.5mm!)
        ((-0.0475, dy_p - 0.002, DECK_H + 0.0018), (0.006, 0.034, 0.0036), ANODIZED_SLATE),
        ((0.0475, dy_p - 0.002, DECK_H + 0.0018), (0.006, 0.034, 0.0036), ANODIZED_SLATE),
        ((-0.0555, dy_p + 0.026, DECK_H + 0.0018), (0.006, 0.026, 0.0036), ANODIZED_SLATE, math.pi / 5),
        ((0.0555, dy_p + 0.026, DECK_H + 0.0018), (0.006, 0.026, 0.0036), ANODIZED_SLATE, -math.pi / 5),
        # Graduated vernier scale bars on outer stage edges
        ((-0.126, 0.010, DECK_H + 0.0015), (0.010, 0.180, 0.003), RHODIUM_GRID),
        ((0.126, 0.010, DECK_H + 0.0015), (0.010, 0.180, 0.003), RHODIUM_GRID),
        # Front quartz coverslip staging cradle at (CS_RACK_X, CS_RACK_Y) (low 2.5 mm locator curbs along X)
        ((-0.024, dy_r, DECK_H + 0.0015), (0.006, 0.038, 0.003), BRASS_GOLD),
        ((0.024, dy_r, DECK_H + 0.0015), (0.006, 0.038, 0.003), BRASS_GOLD),
        ((0.0, dy_r - 0.023, DECK_H + 0.0015), (0.054, 0.006, 0.003), BRASS_GOLD),
    ]
    for fx in (-0.124, 0.124):
        for fy in (-0.148, 0.148):
            boxes.append(((fx, fy, foot_h / 2), (0.022, 0.022, foot_h), RUBBER_DARK))
    return labkit.part("stage", (STAGE_X, sy0, TOP), fixed=True, friction=0.04, color=STAGE_BLK, boxes=boxes)


def _build_chamber():
    """68 g double-grid optical glass counting chamber with silvered H-moat grids, outer shoulders, and handling grip."""
    boxes = [
        # Main optical glass slab: 84 x 36 x 6 mm (outer shoulders z = 8 mm)
        ((0.0, 0.0, 0.003), (0.084, 0.036, 0.006), CHAMBER_GLASS),
        ((-0.033, 0.0, 0.0075), (0.018, 0.036, 0.003), FROSTED_TEAL),  # -X outer clip shoulder (top z = 9 mm)
        ((0.033, 0.0, 0.0075), (0.018, 0.036, 0.003), FROSTED_TEAL),  # +X outer clip shoulder (top z = 9 mm)
        # Twin silvered rhodium counting platforms separated by central transverse moat (z = 9 mm)
        ((-0.009, 0.0, 0.0075), (0.013, 0.024, 0.003), RHODIUM_GRID),
        ((0.009, 0.0, 0.0075), (0.013, 0.024, 0.003), RHODIUM_GRID),
        # Precision ground coverslip support rails flanking the H-moat (top z = MOAT_RAIL_H = 10 mm)
        ((-0.0195, 0.0, 0.008), (0.003, 0.034, 0.004), CHAMBER_GLASS),
        ((0.0195, 0.0, 0.008), (0.003, 0.034, 0.004), CHAMBER_GLASS),
        # +Y frosted L-spine (20 mm wide at x=0 clearing clip hooks, doglegs +55 mm in +X at y>=+95 mm to grip!)
        ((0.0, 0.062, 0.006), (0.020, 0.088, 0.012), FROSTED_TEAL),
        ((STEM_X_OFF / 2, 0.106, 0.006), (STEM_X_OFF + 0.022, 0.020, 0.012), FROSTED_TEAL),
        ((STEM_X_OFF, 0.151, 0.006), (0.022, 0.092, 0.012), FROSTED_TEAL),
        # Elevated 30 x 42 x 24 mm knurled grip (z in [8, 32] mm -> lz = DECK_Z+0.030 clears DECK_Z by >= 8.5 mm!)
        ((STEM_X_OFF, STEM_Y_OFF + 0.001, 0.020), (0.030, 0.042, 0.024), KNURL_DARK),
        ((STEM_X_OFF, STEM_Y_OFF - 0.022, 0.020), (0.046, 0.006, 0.024), BRASS_GOLD),
        ((STEM_X_OFF, STEM_Y_OFF + 0.025, 0.020), (0.046, 0.006, 0.024), BRASS_GOLD),
    ]
    return labkit.part("chamber", (STAGE_X, STAGE_Y + CHAMBER_START_DY, DECK_Z + 0.0003),
                       mass=CHAMBER_MASS, friction=0.08, color=CHAMBER_GLASS, boxes=boxes)


def _build_coverslip():
    """36 g rigid ground fused-quartz hemocytometer coverslip with wide optical bezel & 26 mm frosted pinch bridge."""
    boxes = [
        # Ground fused-quartz optical foot: 32 x 32 x 8 mm (spans H-moat rails with >6 mm per-side clip clearance!)
        ((0.0, 0.0, 0.004), (0.032, 0.032, 0.008), QUARTZ_CYAN),
        # Stepped upper quartz optical bezel: 62 x 48 x 4 mm (top z = 12 mm; >=11 px on top_cam!)
        ((0.0, 0.0, 0.010), (0.062, 0.048, 0.004), QUARTZ_CYAN),
        ((-0.024, 0.0, 0.0125), (0.005, 0.040, 0.001), BRASS_GOLD),
        ((0.024, 0.0, 0.0125), (0.005, 0.040, 0.001), BRASS_GOLD),
        # Tall 20 mm frosted pinch bridge above bezel (z in [12, 32] mm -> fingertips stay >10 mm clear above bezel!)
        ((0.0, 0.0, 0.022), (0.026, 0.026, 0.020), FROSTED_TEAL),
    ]
    return labkit.part("coverslip", (CS_RACK_X, CS_RACK_Y, DECK_Z + 0.0003),
                       mass=COVERSLIP_MASS, friction=0.95, color=QUARTZ_CYAN, boxes=boxes)


STAGE = _build_stage()
CHAMBER = _build_chamber()
COVERSLIP = _build_coverslip()

kit = mk.Kit()

# -- 1. 28 g sprung front datum plunger (pushes +Y: ejects an unheld chamber +19 mm out of pocket before clips close!)
DATUM_ORIGIN = (STAGE_X, STAGE_Y - 0.031, DECK_Z + 0.008)
kit.part("datum_plunger", DATUM_ORIGIN, [
    mk.box((0.056, 0.018, 0.008), at=(0.0, -0.018, 0.0), color=BRASS_GOLD),  # compact indicator carriage (clear of rack!)
    mk.box((0.014, 0.036, 0.007), at=(0.0, 0.014, 0.0), color=BRASS_GOLD),  # nose +Y tip at STAGE_Y + 0.001 (ejects +19mm!)
], mass=DATUM_MASS, color=BRASS_GOLD)
kit.slider("datum_slide", None, "datum_plunger", axis=(0, -1, 0),
           lower=0.0, upper=0.024, start=0.0,
           stiffness=DATUM_K_NM, rest=0.0, damping=5.0, friction=0.0)

# -- 2 & 3. Two 32 g spring-loaded pivoting stage clips + two 28 g sprung brass detent catches -------------------------
CLIP_Z = DECK_Z + 0.014
PIV_L = (STAGE_X - 0.056, STAGE_Y - 0.032, CLIP_Z)
PIV_R = (STAGE_X + 0.056, STAGE_Y - 0.032, CLIP_Z)

for _side, _sgn, _piv in (("left", 1.0, PIV_L), ("right", -1.0, PIV_R)):
    # _sgn = +1 for left (-X side, closes toward +X), -1 for right (+X side, closes toward -X)
    kit.part(f"clip_{_side}", _piv, [
        mk.box((0.016, 0.016, 0.016), at=(0.0, 0.0, 0.0), color=CHROME_CLIP),
        # Over-shoulder spring blade: sits over [-42,-26]mm outer shoulder leaving 52mm clear center for coverslip!
        mk.box((0.012, 0.058, 0.005), at=(0.009 * _sgn, 0.027, -0.001), color=CHROME_CLIP, yaw=-0.32 * _sgn),
        # Rear keeper hook (z in [5.5, 17.5] mm clears 3.6 mm stage walls, holds 9 mm shoulder at +8.2 mm!)
        mk.box((0.014, 0.005, 0.012), at=(0.028 * _sgn, 0.056, -0.0025), color=BRASS_GOLD),
        # Outer knurled push paddle + rear +Y keeper lip (elevated so cz_clip = DECK_Z + 0.026 clears stage by >4.5mm!)
        mk.box((0.024, 0.034, 0.032), at=(-0.008 * _sgn, 0.036, 0.011), color=KNURL_DARK),
        mk.box((0.022, 0.008, 0.032), at=(-0.018 * _sgn, 0.054, 0.011), color=BRASS_GOLD),
        # Narrow 3.5 mm front -Y detent striker finger: clears dx = 8.3 mm catch lip as soon as theta < 15.8 deg!
        mk.box((0.0035, 0.010, 0.010), at=(0.0, -0.024, 0.0), color=BRASS_GOLD),
    ], mass=CLIP_MASS, color=CHROME_CLIP)
    kit.hinge(f"clip_hinge_{_side}", None, f"clip_{_side}", anchor=_piv, axis=(0, 0, _sgn),
              lower_deg=0.0, upper_deg=58.0, start_deg=CLIP_OPEN_DEG,
              stiffness=CLIP_K_NM, rest_deg=54.0, damping=0.08, friction=0.0)

    _det_o = (_piv[0], _piv[1] - 0.032, CLIP_Z)
    kit.part(f"detent_{_side}", _det_o, [
        mk.box((0.038, 0.032, 0.008), at=(-0.028 * _sgn, -0.010, -0.002), color=BRASS_GOLD),  # outer indicator housing
        mk.box((0.005, 0.008, 0.012), at=(0.0108 * _sgn, 0.0020, 0.0), color=BRASS_GOLD),  # flat catch lip at dx=+-8.3mm
        # 18-deg cam ramp on +Y face (normal (+0.31*_sgn, +0.95, 0) -> cams detent in -Y effortlessly at 1.5x mu!)
        mk.box((0.014, 0.005, 0.012), at=(0.0148 * _sgn, 0.0035, 0.0), color=BRASS_GOLD, yaw=math.radians(-18.0 * _sgn)),
    ], mass=DETENT_MASS, color=BRASS_GOLD)
    kit.slider(f"detent_slide_{_side}", None, f"detent_{_side}", axis=(0, -1, 0),
               lower=0.0, upper=0.014, start=0.0,
               stiffness=DETENT_K_NM, rest=0.0, damping=5.0, friction=0.0)

OBJECTS = [STAGE, CHAMBER, COVERSLIP, *kit.objects()]

# -- pure stateless end-state scoring thresholds (every number in task.json scoring appears here as a literal) ---------
POCKET_XY_TOL = 0.014  # 14 mm: counting chamber seated against stage pocket reference edges (starts at 34 mm)
POCKET_Z_TOL = 0.005  # 5 mm: counting chamber resting flat on stage pocket floor
CHAMBER_TILT_DEG = 5.0  # 5 deg: counting chamber level on stage
CLIP_LATCH_DEG = 19.0  # 19 deg: both spring stage clips latched over the chamber's outer shoulders (start at 52 deg)
COVERSLIP_XY_TOL = 0.018  # 18 mm: quartz coverslip centred across the chamber's central H-moat rails
COVERSLIP_Z_TOL = 0.005  # 5 mm: quartz coverslip resting flat on the 10 mm central support rails
COVERSLIP_TILT_DEG = 6.0  # 6 deg: quartz coverslip lying flat right-side-up across the H-moat
CLEAR_DIST = 0.120  # 120 mm: both grippers clear of chamber, handling stem, both clips, and coverslip
SCORE_OVER = "end"
SUCCESS_HOLD_S = 3.0  # 3 s continuous hold at 1.0 enforced statelessly by the runner (60 steps at 20 Hz)
PASS_SCORE = 1.0 - 1e-9


def _rot(state) -> np.ndarray:
    return mk.quat_to_mat(state.quat_xyzw)


def _tilt_deg(state) -> float:
    return math.degrees(math.acos(float(np.clip(_rot(state)[2, 2], -1.0, 1.0))))


def chamber_in_pocket(objects: dict) -> bool:
    ch = objects["chamber"]
    p = np.asarray(ch.pos, float)
    return (math.hypot(float(p[0] - STAGE_X), float(p[1] - STAGE_Y)) <= POCKET_XY_TOL
            and abs(float(p[2] - DECK_Z)) <= POCKET_Z_TOL
            and _tilt_deg(ch) <= CHAMBER_TILT_DEG)


def clips_latched(objects: dict) -> bool:
    return (float(kit.q_deg(objects, "clip_hinge_left")) <= CLIP_LATCH_DEG
            and float(kit.q_deg(objects, "clip_hinge_right")) <= CLIP_LATCH_DEG)


def coverslip_on_rails(objects: dict) -> bool:
    ch, cs = objects["chamber"], objects["coverslip"]
    pc, ps = np.asarray(ch.pos, float), np.asarray(cs.pos, float)
    return (math.hypot(float(ps[0] - pc[0]), float(ps[1] - pc[1])) <= COVERSLIP_XY_TOL
            and abs(float((ps[2] - pc[2]) - MOAT_RAIL_H)) <= COVERSLIP_Z_TOL
            and _tilt_deg(cs) <= COVERSLIP_TILT_DEG)


def hands_clear(objects: dict, tcps=None) -> bool:
    pts = mk.tcps() if tcps is None else tcps
    if pts is None or len(pts) < 2:
        return False
    ch, cs = objects["chamber"], objects["coverslip"]
    pc = np.asarray(ch.pos, float)
    rc = _rot(ch)
    anchors = [
        pc,
        pc + rc @ np.array([STEM_X_OFF, STEM_Y_OFF, 0.011]),
        np.asarray(cs.pos, float),
        kit.point(objects, "clip_left", (-0.004, 0.036, 0.004)),
        kit.point(objects, "clip_right", (0.004, 0.036, 0.004)),
    ]
    return all(float(np.linalg.norm(np.asarray(t, float) - a)) > CLEAR_DIST for t in pts for a in anchors)


def components(objects: dict, tcps=None) -> dict:
    """Pure observable physical end-state predicates (zero step-accumulated score state)."""
    if "chamber" not in objects or "coverslip" not in objects or "clip_left" not in objects:
        return {
            "chamber_in_pocket": False, "clips_latched": False,
            "coverslip_on_rails": False, "hands_clear": False,
        }
    return {
        "chamber_in_pocket": chamber_in_pocket(objects),
        "clips_latched": clips_latched(objects),
        "coverslip_on_rails": coverslip_on_rails(objects),
        "hands_clear": hands_clear(objects, tcps),
    }


def score(objects: dict, tcps=None) -> float:
    """Judged from pure physical end-state geometry; Possible scores 0.0, 0.25, 0.50, 0.75, 1.0."""
    c = components(objects, tcps)
    if not c["chamber_in_pocket"]:
        return 0.0
    if not c["clips_latched"]:
        return 0.25
    if not c["coverslip_on_rails"]:
        return 0.50
    if not c["hands_clear"]:
        return 0.75
    return 1.0


def success(objects: dict) -> bool:
    return score(objects) >= PASS_SCORE


def distance(objects: dict) -> float:
    if "chamber" not in objects or "coverslip" not in objects:
        return 1.0
    ch, cs = objects["chamber"], objects["coverslip"]
    d_ch = float(np.linalg.norm(np.asarray(ch.pos, float)[:2] - np.array([STAGE_X, STAGE_Y])))
    d_cs = float(np.linalg.norm(np.asarray(cs.pos, float)[:2] - np.asarray(ch.pos, float)[:2]))
    return d_ch + d_cs


# -- open-loop bimanual scripted demo (scene constants only) ----------------------------------------------------------
def demo():
    """Left arm seats and braces the counting chamber; right latches both stage clips and lays the quartz coverslip."""
    o, c = FINGER_OPEN, FINGER_CLOSED
    sq = -0.012
    rl = pitch_demo.rotation(-math.pi / 2.0)
    r0 = pitch_demo.rotation(0.0)

    z_hov = TOP + 0.145
    lx = STAGE_X + STEM_X_OFF
    ly_pick = STAGE_Y + CHAMBER_START_DY + STEM_Y_OFF + 0.002
    ly_seat = STAGE_Y + STEM_Y_OFF - 0.028  # friction=0.08 + lz=DECK_Z+0.030 slides effortlessly with 0 stage drag!
    ly_rel = STAGE_Y + STEM_Y_OFF + 0.004
    lz = DECK_Z + 0.030

    # Symmetric stage clip circular-arc sweep: cz_clip=DECK_Z+0.026 clears stage; clicks 5+ deg past 15.8-deg detent!
    cy_out, cy_mid, cy_in = STAGE_Y - 0.018, STAGE_Y + 0.010, STAGE_Y + 0.038
    cz_clip = DECK_Z + 0.026
    lx_out, lx_mid, lx_in = STAGE_X - 0.116, STAGE_X - 0.080, STAGE_X - 0.026
    rx_out, rx_mid, rx_in = STAGE_X + 0.116, STAGE_X + 0.080, STAGE_X + 0.026

    # Quartz coverslip pick & flat placement across central H-moat support rails (fingertips >10mm clear above bezel!)
    cs_x, cs_y = CS_RACK_X, CS_RACK_Y
    cs_z_pick = DECK_Z + 0.034
    cs_z_place = DECK_Z + MOAT_RAIL_H + 0.035

    seq = [
        (0.6, (lx, ly_pick, z_hov), o, rl, (lx_out, cy_out, z_hov), o, "hover over chamber grip and left clip"),
        (0.6, (lx, ly_pick, lz), o, rl, (lx_out, cy_out, z_hov), c, "descend left over chamber handling block"),
        (0.7, (lx, ly_pick, lz), sq, rl, (lx_out, cy_out, z_hov), c, "clamp chamber handling block firmly along X"),
        (0.9, (lx, ly_seat, lz), sq, rl, (lx_out, cy_out, cz_clip), c, "slide counting chamber into stage pocket"),
        (0.6, (lx, ly_seat, lz), sq, rl, (lx_mid, cy_mid, cz_clip), c, "arc left stage clip inward"),
        (0.7, (lx, ly_seat, lz), sq, rl, (lx_in, cy_in, cz_clip), c, "latch left stage clip past detent over shoulder"),
        (0.5, (lx, ly_seat, lz), sq, rl, (lx_in, cy_in, z_hov), c, "lift right clear of latched left clip"),
        (0.5, (lx, ly_seat, lz), sq, rl, (STAGE_X, -0.130, z_hov), c, "swing right forward clear of left arm"),
        (0.5, (lx, ly_seat, lz), sq, rl, (rx_out, cy_out, z_hov), c, "move outside open right stage clip"),
        (0.5, (lx, ly_seat, lz), sq, rl, (rx_out, cy_out, cz_clip), c, "descend outside right stage clip"),
        (0.6, (lx, ly_seat, lz), sq, rl, (rx_mid, cy_mid, cz_clip), c, "arc right stage clip inward"),
        (0.7, (lx, ly_seat, lz), sq, rl, (rx_in, cy_in, cz_clip), c, "latch right stage clip past detent over shoulder"),
        (0.5, (lx, ly_rel, lz), sq, rl, (rx_in, cy_in, z_hov), o, "lift right clear as clips hold chamber"),
        (0.6, (lx, ly_rel, lz), o, rl, (cs_x, cs_y, z_hov), o, "release chamber grip and align over quartz coverslip"),
        (0.6, (lx, ly_rel, z_hov), o, rl, (cs_x, cs_y, cs_z_pick), o, "lift left clear and descend over coverslip bridge"),
        (0.7, (0.18, 0.30, TOP + 0.17), o, r0, (cs_x, cs_y, cs_z_pick), c, "park left clear and grasp quartz coverslip"),
        (0.7, (0.18, 0.30, TOP + 0.17), o, r0, (cs_x, cs_y, z_hov), c, "lift quartz coverslip vertically"),
        (0.8, (0.18, 0.30, TOP + 0.17), o, r0, (STAGE_X, STAGE_Y + 0.003, z_hov), c, "carry coverslip above central H-moat rails"),
        (0.8, (0.18, 0.30, TOP + 0.17), o, r0, (STAGE_X, STAGE_Y + 0.003, cs_z_place), c, "set coverslip flat across central rails"),
        (0.6, (0.18, 0.30, TOP + 0.17), o, r0, (STAGE_X, STAGE_Y + 0.003, cs_z_place), o, "release coverslip gently in place"),
        (0.6, (0.18, 0.30, TOP + 0.17), o, r0, (STAGE_X, STAGE_Y + 0.003, z_hov), o, "lift right straight up clear of coverslip"),
        (0.8, (0.18, 0.30, TOP + 0.17), o, r0, (0.18, -0.30, TOP + 0.17), o, "park both arms clear"),
        (4.2, (0.18, 0.30, TOP + 0.17), o, r0, (0.18, -0.30, TOP + 0.17), o, "hold 3 s post-release"),
    ]
    left = [Waypoint(d, lp, lf, lab, rot=lr) for d, lp, lf, lr, _, _, lab in seq]
    right = [Waypoint(d, rp, rf, lab, rot=r0) for d, _, _, _, rp, rf, lab in seq]
    return {"left": left, "right": right}
