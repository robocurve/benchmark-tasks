"""Release the card-file follower clutch, insert the tabbed divider upright, and slide the follower snug until it locks.

Real product class: an open-top steel 4x6 desktop index-card file tray (260 g unbolted steel box with a chrome centre
rod, front ruled card pack, spring-loaded movable rear card pack, two-position steel follower block, and side cam-clutch
release lever) paired with a rigid 42 g amber-tabbed pressboard index divider on a 300 x 430 x 14 mm smooth desk mat.

Bimanual physics: at spawn the follower block is locked 34 mm forward by the spring cam-clutch with the rear pack
backed solid against the locked steel follower plate (1 mm slack before rigid steel-on-steel stop against the clutch
tooth, leaving an 8 mm pack gap that rigidly blocks insertion of the 20 mm-shoed index divider). Pressing the side
clutch release lever inward swings its rear tooth clear so the 110 N/m card-pack spring and 65 N/m rod spring slide the
follower 34 mm back, opening a 42 mm slot between the packs; after slotting the upright tabbed divider between the
packs, one arm must brace the unbolted 260 g card tray on the low-friction desk blotter while the other pushes the
follower block forward past the 20-deg clutch ramp (~5.5 N) until the clutch snaps shut in the 12 to 26 mm divider-snug
window to lock the stack upright against back-slip.
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

# -- distinct PBR colours ---------------------------------------------------------------------------------------------
DESK_SLATE = (0.15, 0.17, 0.20)
OAK_WARM = (0.64, 0.46, 0.28)
RUBBER_DARK = (0.09, 0.10, 0.11)
TRAY_STEEL = (0.28, 0.33, 0.39)
FOLLOWER_SLATE = (0.21, 0.25, 0.30)
CHROME_ROD = (0.78, 0.81, 0.85)
BRASS_GOLD = (0.83, 0.66, 0.24)
KNURL_DARK = (0.32, 0.35, 0.39)
CARD_IVORY = (0.93, 0.91, 0.84)
RULE_BLUE = (0.24, 0.52, 0.78)
RULE_SALMON = (0.84, 0.44, 0.38)
TAB_AMBER = (0.90, 0.62, 0.16)
CLUTCH_RED = (0.78, 0.22, 0.20)

# -- 1:1 desktop card-file geometry (metres, kg, N, deg) --------------------------------------------------------------
DECK_H = 0.014  # 14 mm smooth laminate desk blotter above tabletop (3 mm rubber feet + 11 mm mat)
DECK_Z = TOP + DECK_H
TRAY_X, TRAY_Y = 0.235, -0.060  # datum centre of unbolted steel 4x6 card-file tray (0.07 mm IK, 0 arm-arm!)
STAB_X_OFF, STAB_Y_OFF = 0.060, 0.205  # rear stabilizer handle -> world (0.295, +0.145) (205 mm Y separation!)
DIV_RACK_X, DIV_RACK_Y = 0.235, -0.172  # front oak staging cradle aligned on TRAY_X (zero lateral swing!)
SLOT_Y_OFF = -0.042  # centre of the compressed divider slot relative to TRAY_Y

TRAY_MASS = 0.260  # 260 g unbolted steel 4x6 desktop card-file box + front card pack
PACK_MASS = 0.034  # 34 g movable rear index-card pack (>= 25 g)
FOLLOWER_MASS = 0.052  # 52 g steel follower compressor plate & rod carriage (>= 25 g)
CLUTCH_MASS = 0.030  # 30 g spring-loaded side cam-clutch seesaw lever (>= 25 g)
DIVIDER_MASS = 0.042  # 42 g rigid pressboard A-Z tabbed index divider (>= 25 g)

PACK_K_NM = 110.0  # 110 N/m card-pack expansion spring (opens 34 mm divider slot when clutch releases)
FOLLOWER_K_NM = 65.0  # 65 N/m follower return spring (~4.5 N combined stack push -> slides unbraced tray!)
CLUTCH_K_NM = 0.11  # 0.11 N m/rad cam-clutch locking return spring (snaps shut in <0.04 s!)
FOLLOWER_START_M = 0.034  # initial empty-divider locked stroke (leaves 8 mm pack gap -> blocks 20 mm divider shoe!)


def _build_desk_mat():
    """300 x 430 x 14 mm smooth desk blotter with lateral tray guides and front oak divider staging cradle."""
    foot_h, plate_t = 0.003, 0.011
    my0 = -0.005
    dy_t = TRAY_Y - my0
    dy_d = DIV_RACK_Y - my0
    dx_d = DIV_RACK_X - TRAY_X
    boxes = [
        ((0.0, 0.0, foot_h + plate_t / 2), (0.300, 0.430, plate_t), DESK_SLATE),
        # Lateral X alignment strips with 8 mm per-side clearance for +-5mm spawn jitter (open along +-Y!)
        ((-0.072, dy_t + 0.015, DECK_H + 0.0025), (0.008, 0.130, 0.005), OAK_WARM),
        ((0.072, dy_t + 0.015, DECK_H + 0.0025), (0.008, 0.130, 0.005), OAK_WARM),
        # Brass datum witness markers at tray reference corners
        ((-0.072, dy_t - 0.060, DECK_H + 0.0055), (0.010, 0.014, 0.001), BRASS_GOLD),
        ((0.072, dy_t - 0.060, DECK_H + 0.0055), (0.010, 0.014, 0.001), BRASS_GOLD),
        # Front oak divider staging cradle at (DIV_RACK_X, DIV_RACK_Y) (low 3 mm curbs -> snag-free vertical lift!)
        ((dx_d - 0.048, dy_d, DECK_H + 0.0015), (0.006, 0.032, 0.003), OAK_WARM),
        ((dx_d + 0.048, dy_d, DECK_H + 0.0015), (0.006, 0.032, 0.003), OAK_WARM),
        ((dx_d, dy_d - 0.017, DECK_H + 0.0015), (0.090, 0.005, 0.003), BRASS_GOLD),
        ((dx_d, dy_d + 0.017, DECK_H + 0.0015), (0.090, 0.005, 0.003), BRASS_GOLD),
    ]
    for fx in (-0.128, 0.128):
        for fy in (-0.185, 0.185):
            boxes.append(((fx, fy, foot_h / 2), (0.022, 0.022, foot_h), RUBBER_DARK))
    return labkit.part("desk_mat", (TRAY_X, my0, TOP), fixed=True, friction=0.03, color=DESK_SLATE, boxes=boxes)


def _build_divider():
    """42 g rigid pressboard index divider with 20 mm brass base shoe, amber A-Z tab, and tall top pinch handle."""
    boxes = [
        # Stable 20 mm staging base shoe in lower 4 mm -> stands 0.0 deg upright & allows 22 mm compression in 42 mm slot!
        ((0.0, 0.0, 0.002), (0.062, 0.010, 0.004), BRASS_GOLD),
        ((-0.024, 0.0, 0.002), (0.014, 0.020, 0.004), BRASS_GOLD),
        ((0.024, 0.0, 0.002), (0.014, 0.020, 0.004), BRASS_GOLD),
        # Main rigid pressboard divider card: 64 x 10 x 28 mm (top z = 32 mm; clears +-33 mm clutch tooth completely!)
        ((0.0, 0.0, 0.018), (0.064, 0.010, 0.028), TAB_AMBER),
        # Left A-Z tab + tall 24 x 20 x 24 mm centred top handle (z in [32, 56] mm -> clears 28 mm card packs above!)
        ((-0.020, 0.0, 0.035), (0.018, 0.008, 0.006), CARD_IVORY),
        ((0.0, 0.0, 0.044), (0.024, 0.020, 0.024), KNURL_DARK),
        ((0.0, 0.0, 0.0575), (0.034, 0.022, 0.003), TAB_AMBER),
    ]
    return labkit.part("divider", (DIV_RACK_X, DIV_RACK_Y, DECK_Z + 0.0003),
                       mass=DIVIDER_MASS, friction=0.95, color=TAB_AMBER, boxes=boxes)


DESK_MAT = _build_desk_mat()
DIVIDER = _build_divider()

kit = mk.Kit()

# -- 1. 260 g unbolted steel 4x6 desktop card-file box + front upright card pack + elevated rear stabilizer handle ------
kit.part("tray", (TRAY_X, TRAY_Y, DECK_Z), [
    mk.box((0.120, 0.156, 0.006), at=(0.0, -0.008, 0.003), color=TRAY_STEEL),  # steel tray floor (z_top = 6 mm)
    mk.box((0.006, 0.156, 0.018), at=(-0.057, -0.008, 0.015), color=TRAY_STEEL),  # low -X side rail
    mk.box((0.006, 0.156, 0.018), at=(0.057, -0.008, 0.015), color=TRAY_STEEL),  # low +X side rail
    mk.box((0.120, 0.006, 0.028), at=(0.0, -0.083, 0.020), color=TRAY_STEEL),  # front header wall
    mk.box((0.120, 0.006, 0.018), at=(0.0, 0.067, 0.015), color=TRAY_STEEL),  # low rear end wall
    mk.box((0.048, 0.003, 0.016), at=(0.0, -0.087, 0.020), color=BRASS_GOLD),  # brass label holder frame
    mk.box((0.040, 0.002, 0.010), at=(0.0, -0.0885, 0.020), color=CARD_IVORY),  # ivory index label
    mk.box((0.008, 0.136, 0.004), at=(0.0, -0.008, 0.008), color=CHROME_ROD),  # steel centre follower rod
    # Front upright card pack (top z = 29 mm < divider's 32 mm handle bottom; back face at y_rel = -58 mm -> 42mm slot!)
    mk.box((0.062, 0.022, 0.020), at=(0.0, -0.069, 0.016), color=CARD_IVORY),
    mk.box((0.062, 0.022, 0.003), at=(0.0, -0.069, 0.0275), color=RULE_BLUE),
    # Offset +X/+Y elevated rear stabilizer handle (z in [10, 34] mm -> lz = DECK_Z+0.030 clears DECK_Z by 8.5 mm!)
    mk.box((0.052, 0.020, 0.010), at=(STAB_X_OFF / 2, 0.080, 0.008), color=TRAY_STEEL),
    mk.box((0.022, 0.102, 0.010), at=(STAB_X_OFF, 0.136, 0.008), color=TRAY_STEEL),
    mk.box((0.030, 0.042, 0.024), at=(STAB_X_OFF, STAB_Y_OFF + 0.001, 0.022), color=KNURL_DARK),
    mk.box((0.046, 0.006, 0.024), at=(STAB_X_OFF, STAB_Y_OFF - 0.022, 0.022), color=BRASS_GOLD),
    mk.box((0.046, 0.006, 0.024), at=(STAB_X_OFF, STAB_Y_OFF + 0.025, 0.022), color=BRASS_GOLD),
], mass=TRAY_MASS, color=TRAY_STEEL)

# -- 2. 34 g spring-loaded movable rear card pack (at q=0: y_rel in [-16, +8] mm -> 42 mm slot [-58, -16] mm!) ---------
PACK_ORIGIN = (TRAY_X, TRAY_Y - 0.004, DECK_Z + 0.016)
kit.part("rear_pack", PACK_ORIGIN, [
    mk.box((0.060, 0.024, 0.020), at=(0.0, 0.0, 0.0), color=CARD_IVORY),
    mk.box((0.060, 0.024, 0.003), at=(0.0, 0.0, 0.0115), color=RULE_SALMON),
], mass=PACK_MASS, color=CARD_IVORY)
kit.slider("pack_slide", "tray", "rear_pack", axis=(0, -1, 0),
           lower=0.0, upper=0.036, start=FOLLOWER_START_M,
           stiffness=PACK_K_NM, rest=0.0, damping=6.0, friction=0.0)

# -- 3. 52 g two-position steel follower block (Lug A locks at 34 mm empty; wedge locks at >= 13.5 mm with divider!) ---
FOL_ORIGIN = (TRAY_X, TRAY_Y + 0.034, DECK_Z + 0.022)
kit.part("follower", FOL_ORIGIN, [
    # Steel compressor plate (top z = 30 mm < 32 mm handle; front face at dy=-0.025 -> y_rel=+9mm @q=0, -25mm @q=34mm!)
    mk.box((0.060, 0.008, 0.022), at=(0.0, -0.021, -0.003), color=FOLLOWER_SLATE),
    mk.box((0.028, 0.034, 0.012), at=(0.0, -0.005, -0.010), color=FOLLOWER_SLATE),  # rod carriage sleeve
    # Elevated rear knurled push paddle at dy = +0.015 (z_top = DECK_Z + 0.062 -> fol_z clears tray by >15 mm!)
    mk.box((0.038, 0.016, 0.040), at=(0.0, 0.015, 0.018), color=KNURL_DARK),
    mk.box((0.040, 0.018, 0.004), at=(0.0, 0.015, 0.040), color=BRASS_GOLD),
    # -X ratchet lugs engaging inline tooth (tooth spans y_rel in [+19.5, +24.5] mm, x_rel in [-39, -33] mm):
    mk.box((0.006, 0.004, 0.012), at=(-0.0355, 0.0170, -0.002), color=BRASS_GOLD),  # Lug A: [+15,+19]mm @q=34!
    # 20-deg cam ramp + flush yaw=0 flat keeper heel (-36.3mm <= -36.5mm ramp heel: 0 step forward, 0-deg dead-lock back!)
    mk.box((0.005, 0.003, 0.012), at=(-0.0338, 0.0000, -0.002), color=BRASS_GOLD),
    mk.box((0.006, 0.010, 0.012), at=(-0.0315, -0.0050, -0.002), color=BRASS_GOLD, yaw=math.radians(20.0)),
], mass=FOLLOWER_MASS, color=FOLLOWER_SLATE)
kit.slider("follower_slide", "tray", "follower", axis=(0, -1, 0),
           lower=0.0, upper=0.036, start=FOLLOWER_START_M,
           stiffness=FOLLOWER_K_NM, rest=0.0, damping=6.0, friction=0.0)

# -- 4. 30 g horizontal seesaw cam-clutch lever (inline pivot x_rel=-0.036 -> r_x=0 on tooth, ZERO toggle self-lock!) ---
CLUTCH_PIV = (TRAY_X - 0.036, TRAY_Y - 0.012, DECK_Z + 0.022)
kit.part("clutch_lever", CLUTCH_PIV, [
    # Front -Y red release push paddle at (-0.046, -0.030) -> world (0.153, -0.104), elevated 10 mm above tray!
    mk.box((0.024, 0.032, 0.030), at=(-0.046, -0.030, 0.008), color=CLUTCH_RED),
    mk.box((0.026, 0.034, 0.004), at=(-0.046, -0.030, 0.024), color=BRASS_GOLD),
    # Outer L-beam routed at y_rel=+0.020 & x_rel=-0.082 (57 mm clear of divider slot!) + inline rear tooth (r_x=0)!
    mk.box((0.052, 0.008, 0.008), at=(-0.023, 0.034, 0.016), color=BRASS_GOLD),
    mk.box((0.010, 0.072, 0.008), at=(-0.046, 0.002, 0.016), color=BRASS_GOLD),
    mk.box((0.006, 0.005, 0.022), at=(0.0, 0.034, 0.003), color=BRASS_GOLD),
], mass=CLUTCH_MASS, color=CLUTCH_RED)
kit.hinge("clutch_hinge", "tray", "clutch_lever", anchor=CLUTCH_PIV, axis=(0, 0, 1),
          lower_deg=0.0, upper_deg=26.0, start_deg=0.0,
          stiffness=CLUTCH_K_NM, rest_deg=0.0, damping=0.025, friction=0.0)

OBJECTS = [DESK_MAT, DIVIDER, *kit.objects()]

# -- pure stateless end-state scoring thresholds (every number in task.json scoring appears here as a literal) ---------
TRAY_XY_TOL = 0.016  # 16 mm: card-file tray stays registered on the desk blotter datum zone
TRAY_Z_TOL = 0.006  # 6 mm: card-file tray rests flat on the desk blotter
TRAY_TILT_DEG = 5.0  # 5 deg: card-file tray stays level
DIVIDER_X_TOL = 0.022  # 22 mm: tabbed divider centred laterally inside the card-file tray
DIVIDER_Y_TOL = 0.015  # 15 mm: tabbed divider positioned strictly inside the opened card-pack slot
DIVIDER_Z_TOL = 0.008  # 8 mm: tabbed divider seated down on the tray floor (not resting atop closed packs!)
DIVIDER_TILT_DEG = 10.0  # 10 deg: tabbed divider held upright between the compressed card packs
FOLLOWER_SNUG_MIN = 0.012  # 12 mm: lower bound of divider-installed follower lock window (open rest is 0 mm)
FOLLOWER_SNUG_MAX = 0.026  # 26 mm: upper bound of divider-installed follower window (excludes 34 mm empty-pack lock!)
CLUTCH_LOCK_DEG = 8.0  # 8 deg: cam-clutch seesaw lever returned into its locking notch on the follower rail
CLEAR_DIST = 0.120  # 120 mm: both grippers clear of tray, rear handle, divider, follower block, and clutch paddle
SCORE_OVER = "end"
SUCCESS_HOLD_S = 3.0  # 3 s continuous hold at 1.0 enforced statelessly by the runner (60 steps at 20 Hz)
PASS_SCORE = 1.0 - 1e-9


def _rot(state) -> np.ndarray:
    return mk.quat_to_mat(state.quat_xyzw)


def _tilt_deg(state) -> float:
    return math.degrees(math.acos(float(np.clip(_rot(state)[2, 2], -1.0, 1.0))))


def tray_registered(objects: dict) -> bool:
    p = kit.point(objects, "tray", (0.0, 0.0, 0.0))
    return (math.hypot(float(p[0] - TRAY_X), float(p[1] - TRAY_Y)) <= TRAY_XY_TOL
            and abs(float(p[2] - DECK_Z)) <= TRAY_Z_TOL
            and _tilt_deg(objects["tray"]) <= TRAY_TILT_DEG)


def divider_upright_in_slot(objects: dict) -> bool:
    pt = kit.point(objects, "tray", (0.0, 0.0, 0.0))
    rt = _rot(objects["tray"])
    rel = rt.T @ (np.asarray(objects["divider"].pos, float) - pt)
    return (abs(float(rel[0])) <= DIVIDER_X_TOL
            and abs(float(rel[1] - SLOT_Y_OFF)) <= DIVIDER_Y_TOL
            and abs(float(rel[2] - 0.006)) <= DIVIDER_Z_TOL
            and _tilt_deg(objects["divider"]) <= DIVIDER_TILT_DEG)


def follower_in_snug_window(objects: dict) -> bool:
    qf = float(kit.q(objects, "follower_slide"))
    return FOLLOWER_SNUG_MIN <= qf <= FOLLOWER_SNUG_MAX


def follower_snug_and_locked(objects: dict) -> bool:
    return (follower_in_snug_window(objects)
            and float(kit.q_deg(objects, "clutch_hinge")) <= CLUTCH_LOCK_DEG)


def hands_clear(objects: dict, tcps=None) -> bool:
    pts = mk.tcps() if tcps is None else tcps
    if pts is None or len(pts) < 2:
        return False
    anchors = [
        kit.point(objects, "tray", (0.0, 0.0, 0.0)),
        kit.point(objects, "tray", (STAB_X_OFF, STAB_Y_OFF, 0.022)),
        np.asarray(objects["divider"].pos, float),
        kit.point(objects, "follower", (0.0, 0.014, 0.018)),
        kit.point(objects, "clutch_lever", (-0.046, -0.030, 0.008)),
    ]
    return all(float(np.linalg.norm(np.asarray(t, float) - a)) > CLEAR_DIST for t in pts for a in anchors)


def components(objects: dict, tcps=None) -> dict:
    """Pure observable physical end-state predicates (zero step-accumulated score state)."""
    if "tray" not in objects or "divider" not in objects or "follower" not in objects:
        return {
            "tray_registered": False, "divider_upright_in_slot": False,
            "follower_snug_and_locked": False, "hands_clear": False,
        }
    return {
        "tray_registered": tray_registered(objects),
        "divider_upright_in_slot": divider_upright_in_slot(objects),
        "follower_snug_and_locked": follower_snug_and_locked(objects),
        "hands_clear": hands_clear(objects, tcps),
    }


def score(objects: dict, tcps=None) -> float:
    """Judged from pure physical end-state geometry; Possible scores 0.0, 0.25, 0.50, 0.75, 1.0."""
    c = components(objects, tcps)
    if not (c["tray_registered"] and c["divider_upright_in_slot"]):
        return 0.0
    if not follower_in_snug_window(objects):
        return 0.25
    if not c["follower_snug_and_locked"]:
        return 0.50
    if not c["hands_clear"]:
        return 0.75
    return 1.0


def success(objects: dict, tcps=None) -> bool:
    return score(objects, tcps=tcps) >= PASS_SCORE


def distance(objects: dict) -> float:
    if "tray" not in objects or "divider" not in objects:
        return 1.0
    pt = kit.point(objects, "tray", (0.0, 0.0, 0.0))
    pd = np.asarray(objects["divider"].pos, float)
    d_div = float(np.linalg.norm(pd[:2] - np.array([pt[0], pt[1] + SLOT_Y_OFF])))
    d_fol = max(0.0, FOLLOWER_SNUG_MIN - float(kit.q(objects, "follower_slide")))
    return d_div + d_fol


# -- open-loop bimanual scripted demo (scene constants only) ----------------------------------------------------------
def demo():
    """Left arm braces the card-file tray; right releases the clutch, slots the divider, and locks the follower snug."""
    o, c = FINGER_OPEN, FINGER_CLOSED
    sq = -0.012
    rl = pitch_demo.rotation(-math.pi / 2.0)
    r0 = pitch_demo.rotation(0.0)

    z_hov = TOP + 0.148
    lx, ly = TRAY_X + STAB_X_OFF, TRAY_Y + STAB_Y_OFF + 0.002
    lz = DECK_Z + 0.030

    # Beat 1: press front -Y red seesaw clutch paddle inward (+X) so rear tooth swings -X & follower springs back 34 mm
    cl_y = TRAY_Y - 0.044
    cl_z = DECK_Z + 0.034
    cl_x_out, cl_x_in = TRAY_X - 0.114, TRAY_X - 0.048

    # Beat 2: pick 42 g amber-tabbed index divider from front cradle & slot upright between the two card packs
    dv_x, dv_y = DIV_RACK_X, DIV_RACK_Y
    dv_z_pick = DECK_Z + 0.056
    slot_x, slot_y = TRAY_X + 0.004, TRAY_Y - 0.037
    dv_z_slot = DECK_Z + 0.062

    # Beat 3: push follower rear paddle forward (-Y) past 20-deg cam ramp until clutch locks behind wedge!
    fol_x = TRAY_X
    fol_z = DECK_Z + 0.046
    fol_y_back = TRAY_Y + 0.070
    fol_y_snug = TRAY_Y - 0.004

    seq = [
        (0.6, (lx, ly, z_hov), o, rl, (cl_x_out, cl_y, z_hov), o, "hover over tray handle and clutch paddle"),
        (0.6, (lx, ly, lz), o, rl, (cl_x_out, cl_y, cl_z), c, "descend left over handle and right outside clutch"),
        (0.6, (lx, ly, lz), sq, rl, (cl_x_out, cl_y, cl_z), c, "clamp rear tray stabilizer handle along X"),
        (0.8, (lx, ly, lz), sq, rl, (cl_x_in, cl_y, cl_z), c, "press clutch paddle inward so follower springs back"),
        (0.5, (lx, ly, lz), sq, rl, (cl_x_out, cl_y, z_hov), o, "retract right clear so clutch resets"),
        (0.6, (lx, ly, lz), sq, rl, (dv_x, dv_y, z_hov), o, "align right over tabbed index divider"),
        (0.6, (lx, ly, lz), sq, rl, (dv_x, dv_y, dv_z_pick), o, "descend over divider top handle"),
        (0.6, (lx, ly, lz), sq, rl, (dv_x, dv_y, dv_z_pick), c, "grasp tabbed index divider"),
        (0.7, (lx, ly, lz), sq, rl, (dv_x, dv_y, z_hov), c, "lift divider clear of front cradle"),
        (0.7, (lx, ly, lz), sq, rl, (slot_x, slot_y, z_hov), c, "carry divider above opened card-pack slot"),
        (0.8, (lx, ly, lz), sq, rl, (slot_x, slot_y, dv_z_slot), c, "slot divider upright between front and rear packs"),
        (0.5, (lx, ly, lz), sq, rl, (slot_x, slot_y, dv_z_slot), o, "release upright divider in slot"),
        (0.6, (lx, ly, lz), sq, rl, (slot_x, slot_y, z_hov), o, "lift right straight up clear of divider"),
        (0.6, (lx, ly, lz), sq, rl, (fol_x, fol_y_back, z_hov), c, "move behind released follower paddle"),
        (0.6, (lx, ly, lz), sq, rl, (fol_x, fol_y_back, fol_z), c, "descend behind follower push paddle"),
        (1.0, (lx, ly, lz), sq, rl, (fol_x, fol_y_snug, fol_z), c, "slide follower snug past cam ramp until clutch locks"),
        (0.4, (lx, ly, lz), sq, rl, (fol_x, fol_y_snug, fol_z), c, "dwell as clutch tooth snaps flat behind keeper"),
        (0.6, (lx, ly, lz), o, rl, (fol_x, fol_y_snug, z_hov), o, "release tray handle and lift right straight up"),
        (0.6, (lx, ly, z_hov), o, rl, (0.18, -0.30, TOP + 0.17), o, "lift left clear and park right"),
        (0.7, (0.18, 0.30, TOP + 0.17), o, r0, (0.18, -0.30, TOP + 0.17), o, "park both arms clear"),
        (4.2, (0.18, 0.30, TOP + 0.17), o, r0, (0.18, -0.30, TOP + 0.17), o, "hold 3 s post-release"),
    ]
    left = [Waypoint(d, lp, lf, lab, rot=lr) for d, lp, lf, lr, _, _, lab in seq]
    right = [Waypoint(d, rp, rf, lab, rot=r0) for d, _, _, _, rp, rf, lab in seq]
    return {"left": left, "right": right}
