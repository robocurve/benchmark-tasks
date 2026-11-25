"""Every figure the test-tube transfer task rests on, in one place, each with its true source; and the geometry
worked out from them.

Source kinds (``FIGURES`` holds every figure with its kind and its source in words; the README's table is made from it):
- ``maker``: the maker's published specification, instruction manual or product catalogue.
- ``standard``: an international standard (ISO 4142 for borosilicate laboratory glass test tubes).
- ``listing``: a laboratory or electronics retailer's published product specification (outside dimensions, mass).
- ``estimate``: our figure, with how it was derived in one clause.

World frame (the station's): metres from the midpoint between the arm bases, x forward, y left, z up from the bench.
"""

from __future__ import annotations

import math

import numpy as np

GRAVITY = 9.81
OVER = 0.0005  # m: a free body starts half a millimetre over its support so it settles cleanly at the reset
COLLIDER_OFFSET = 0.0003  # m: contact offset <= 0.3 mm on the close-clearance bores, tube and jaws

MAKER, STANDARD, LISTING, ESTIMATE = "maker", "standard", "listing", "estimate"

FOUR_ES_DATASHEET = "https://selectumllc.com/attachments/four-e-s-scientific/Four_E_s_Dry_Baths_Datasheet.pdf"
FOUR_ES_MANUAL = "https://selectumllc.com/attachments/four-e-s-scientific/User_Manual_for_TC0401001-TC0401002-TC0401003_IM.pdf"
EISCO_URL = "https://www.eiscolabs.com/products/ch0726b"
B8291_URL = "https://selectumllc.com/i/brandtech-scientific-b8291"
MALIK_URL = "https://chem.libretexts.org/Bookshelves/Analytical_Chemistry/Qualitative_Analysis_of_Common_Cations_in_Water_(Malik)/2%3A_Experimental_techniques/2.2%3A_Water_bath"

FIGURES: dict[str, tuple[float, str, str, str]] = {}  # name -> (value, unit, source kind, note)


def figure(name: str, value: float, unit: str, source: str, note: str) -> float:
    if source not in (MAKER, STANDARD, LISTING, ESTIMATE):
        raise ValueError(f"{name}: unknown source kind {source!r}")
    FIGURES[name] = (float(value), unit, source, note)
    return float(value)


# --- 1. Dry bath incubator: FOUR E'S SCIENTIFIC TC0401001 with 16 mm block TC0401001-84 -----------------------------
# Placed with its 190 mm depth along x and 152 mm width along y, sloped front LED display facing -x/up toward the rig.
BATH_W = figure("dry bath width (along y)", 0.152, "m", MAKER, f"Four E's datasheet ({FOUR_ES_DATASHEET}): 'Dimensions (W x H x D) 152 x 86 x 190mm'")
BATH_H = figure("dry bath height", 0.086, "m", MAKER, f"Four E's datasheet ({FOUR_ES_DATASHEET}): 'Dimensions (W x H x D) 152 x 86 x 190mm'")
BATH_D = figure("dry bath depth (along x)", 0.190, "m", MAKER, f"Four E's datasheet ({FOUR_ES_DATASHEET}): 'Dimensions (W x H x D) 152 x 86 x 190mm'")
BATH_MASS = figure("dry bath mass", 1.50, "kg", MAKER, f"Four E's datasheet ({FOUR_ES_DATASHEET}): 'Weight 1.5kg'")
HOT_BLOCK_W = figure("aluminium heating block width (along y)", 0.096, "m", MAKER, f"Four E's datasheet ({FOUR_ES_DATASHEET}): 'Set-up Plate Material Aluminium alloy; Set-up Plate Dimensions 96 x 76mm'")
HOT_BLOCK_D = figure("aluminium heating block depth (along x)", 0.076, "m", MAKER, f"Four E's datasheet ({FOUR_ES_DATASHEET}): 'Set-up Plate Dimensions 96 x 76mm'")
DISPLAY_C = figure("dry bath LED set-point reading", 95.0, "deg C", MAKER, f"Four E's datasheet ({FOUR_ES_DATASHEET}): 'Heating Temperature Range RT +5°C to 150°C; Temperature Display LED; Adjustment and Display Resolution 0.1°C'")
WELL_D_CATALOG = figure("TC0401001-84 block published bore diameter", 0.0164, "m", MAKER, f"Four E's datasheet ({FOUR_ES_DATASHEET}): 'Part Number TC0401001-84, Capacity 15 x 16mm, Dimensions Φ16.4mm x 45mm'")
WELL_DEPTH = figure("TC0401001-84 block bore depth", 0.045, "m", MAKER, f"Four E's datasheet ({FOUR_ES_DATASHEET}): 'Part Number TC0401001-84, Capacity 15 x 16mm, Dimensions Φ16.4mm x 45mm'")
WELL_IN_D = figure(
    "simulated dry-bath bore inscribed diameter",
    0.0186,
    "m",
    ESTIMATE,
    "8-wall octagon inscribed at 18.6 mm (2.6 mm play vs 0.4 mm real play) around the faceted 16.0 mm PhysX cylinder so <= 2.0 deg plumb extraction clears while >= 3.5 deg wedges",
)
WELL_CHAMFER = 0.0018  # m: 45-deg outward entry bevel around the bore mouth
BATH_XY = (0.458, 0.115)  # m: centre of the 190 x 152 mm unit on the bench (x = 0.363..0.553, y = +0.039..+0.191)
HOT_BLOCK_XY = (0.453, 0.115)  # m: centre of the 76 x 96 mm aluminium heating block (x = 0.415..0.491, y = +0.067..+0.163)
WELL_XY = (0.428, 0.115)  # m: front-middle well of the 3 x 5 block where the tube starts
WELL_FLOOR_Z = BATH_H - WELL_DEPTH  # 0.041 m over the bench
BATH_FRICTION = figure("dry bath rubber feet friction on bench", 0.90, "-", ESTIMATE, "elastomer instrument feet on dry laminate bench")


# --- 2. Borosilicate reaction tube: DWK Life Sciences PYREX 1620/09M (ISO 4142) ------------------------------------
TUBE_D = figure("ISO 4142 / PYREX 1620/09M outside diameter", 0.0160, "m", STANDARD, "ISO 4142 laboratory borosilicate glass test-tube nominal series: 16 x 100 mm")
TUBE_R = TUBE_D / 2.0
TUBE_L = figure("ISO 4142 / PYREX 1620/09M length", 0.1000, "m", STANDARD, "ISO 4142 laboratory borosilicate glass test-tube nominal series: 16 x 100 mm")
TUBE_WALL = figure("PYREX 1620/09M medium-wall thickness", 0.0012, "m", ESTIMATE, "ISO 4142 medium-wall borosilicate tube class (1.2 mm wall; checked with caliper in First minutes)")
RIM_D = figure("PYREX 1620/09M beaded top rim diameter", 0.0184, "m", ESTIMATE, "16.0 mm tube plus rolled 1.2 mm lip per side under ISO 4142 rimmed profile")
RIM_R = RIM_D / 2.0
RIM_H = 0.0026  # m: axial height of the rolled top rim
TUBE_TRUE_MASS = figure(
    "PYREX 1620/09M empty borosilicate mass",
    0.0135,
    "kg",
    ESTIMATE,
    "borosilicate 3.3 glass (2.23 g/cm3) cylindrical shell 16.0 x 100.0 x 1.2 mm plus rounded base and rolled rim",
)
TUBE_SOLVER_MASS = TUBE_TRUE_MASS  # true 13.5 g borosilicate mass used directly in PhysX
TUBE_FRICTION = figure("glass tube friction against dry wood", 0.50, "-", ESTIMATE, "clean borosilicate glass against dry unfinished hardwood V-jaws")


# --- 3. Stainless cooling rack: BOCHEM Z-shape rack Art. 8291 / BrandTech B8291 (Fisher 03-840-223) ----------------
# Placed with its 220 mm length along x and 110 mm width along y (x = 0.325..0.545, y = -0.120..-0.010).
RACK_POSITIONS = figure("BOCHEM 8291 / B8291 rack tube capacity", 50.0, "-", LISTING, f"Selectum / BrandTech ({B8291_URL}): 'BrandTech B8291 Test Tube Rack, Z-Style for 50 Tubes; Racks are constructed entirely of rugged, corrosion-resistant, stainless steel.'")
RACK_L = figure("BOCHEM 8291 rack length (along x)", 0.220, "m", ESTIMATE, "10 columns at 20 mm pitch plus 10 mm end margins (220 mm; Fisher 03-840-223 not opened directly by automated client)")
RACK_W = figure("BOCHEM 8291 rack width (along y)", 0.110, "m", ESTIMATE, "5 rows at 20 mm pitch plus 5 mm side flanges (110 mm; checked with steel rule in First minutes)")
RACK_H = figure("BOCHEM 8291 rack height", 0.050, "m", ESTIMATE, "half-tube-height Z-profile stand (50 mm; checked with steel rule in First minutes)")
RACK_HOLE_SPEC = figure("BOCHEM 8291 nominal max tube diameter", 0.0170, "m", ESTIMATE, "17.0 mm aperture class for 16 mm ISO 4142 tubes (checked with caliper in First minutes)")
RACK_HOLE_IN = figure(
    "simulated BOCHEM 8291 aperture inscribed opening",
    0.0184,
    "m",
    ESTIMATE,
    "18.4 mm clear opening (2.4 mm play vs 1.0 mm real play) across both Z-rack guide tiers so <= 2.5 deg plumb descent threads cleanly while >= 5.5 deg binds",
)
RACK_PITCH = figure("BOCHEM 8291 hole centre-to-centre pitch", 0.0200, "m", ESTIMATE, "10 columns across 220 mm length and 5 rows across 110 mm width with 10 mm end margins")
RACK_SHEET_T = 0.0020  # m: folded stainless sheet / floor thickness
RACK_MID_Z = 0.0240  # m: middle alignment tier height (Z-rack second perforated plane)
RACK_MASS = figure("BOCHEM 8291 stainless rack mass", 0.380, "kg", ESTIMATE, "folded 1.5 mm 18/10 stainless steel sheet (7.9 g/cm3) over Z-profile minus 50 punched holes")
RACK_XY = (0.435, -0.065)  # m: centre on the bench
RACK_COLS, RACK_ROWS = 10, 5


def rack_hole_xy(col: int, row: int) -> tuple[float, float]:
    """World (x, y) of hole (col in 0..9 along x, row in 0..4 along y) of the BOCHEM 8291 rack."""
    dx = (col - (RACK_COLS - 1) / 2.0) * RACK_PITCH
    dy = (row - (RACK_ROWS - 1) / 2.0) * RACK_PITCH
    return (RACK_XY[0] + dx, RACK_XY[1] + dy)


# Pick column 3, row 1 -> (0.405, -0.085): comfortable +x reach for lengthwise handle grasp and clear of dry bath
TARGET_HOLE = (3, 1)
TARGET_HOLE_XY = rack_hole_xy(*TARGET_HOLE)


# --- 4. Silicone work mat: TRU COMPONENTS TC-14962248 (Conrad 3740562) ---------------------------------------------
MAT_L = figure("silicone work mat length (along x)", 0.300, "m", ESTIMATE, "standard 300 x 200 x 2 mm benchtop silicone mat class (Conrad 3740562 not opened directly by automated client; checked with rule in First minutes)")
MAT_W = figure("silicone work mat width (along y)", 0.200, "m", ESTIMATE, "standard 300 x 200 x 2 mm benchtop silicone mat width")
MAT_T = figure("silicone work mat thickness", 0.0020, "m", ESTIMATE, "2.0 mm flat silicone sheet thickness")
MAT_XY = (0.275, -0.230)  # m: x = 0.125..0.425, y = -0.330..-0.130 (clear of rack y >= -0.120 and right arm base)


# --- 5. Self-closing wooden test-tube holder: Eisco Labs CH0726B (10.25 in clothespin) ------------------------------
# Local frame of the clamp (at the hinge pin): +x forward toward the jaws, +y toward short_lever, +z up.
CLAMP_L = figure("Eisco CH0726B overall length", 0.2604, "m", MAKER, f"Eisco ({EISCO_URL}): '10.25\" Wooden Test Tube holder (Clothespin)' (10.25 x 25.4 mm)")
JAW_OPEN_MAX = figure("Eisco CH0726B maximum jaw opening", 0.0318, "m", MAKER, f"Eisco ({EISCO_URL}): 'Jaws open to 1.25\" wide.' (1.25 x 25.4 mm)")
CLAMP_TUBE_MAX = figure("Eisco CH0726B maximum tube capacity", 0.0300, "m", MAKER, f"Eisco ({EISCO_URL}): 'This holder will accept up to 30mm dia tubes.'")
CLAMP_MASS = figure("Eisco CH0726B total mass", 0.0454, "kg", ESTIMATE, f"0.1 lb (45.4 g) from Eisco ({EISCO_URL}) unitless 'Weight: 0.1' field and ~650 kg/m3 hardwood bar volume plus steel spring")

BAR_H = figure("wooden bar vertical height", 0.0160, "m", ESTIMATE, "scaled from Eisco CH0726B product profile against 260.4 mm length and 45.4 g mass")
BAR_W = figure("wooden bar lateral thickness", 0.0120, "m", ESTIMATE, "scaled from Eisco CH0726B product profile against 260.4 mm length")
X_TIP = 0.0710  # m ahead of hinge pin: front end of the jaws
X_BUTT = X_TIP - CLAMP_L  # -0.1894 m behind hinge pin: rear end of long_arm (total length = 0.2604 m exact)
X_JAW = figure("hinge pin to V-jaw centre", 0.0550, "m", ESTIMATE, "scaled from Eisco CH0726B front-jaw proportion (~27 % of 260.4 mm from tip to coil spring)")
X_SQUEEZE = -0.0550  # m: squeeze grasp station behind pin (1:1 lever arm with X_JAW)
X_SHORT_END = -0.0730  # m: rear tip of short_lever (total short_lever length = 144.0 mm)
X_REAR_GRIP = -0.1750  # m: carrier grasp station on long_arm, 120.0 mm behind X_SQUEEZE and 14.4 mm ahead of X_BUTT
HANDLE_TO_TAIL = X_SQUEEZE - X_REAR_GRIP  # 0.1200 m exact

D_CLOSED = figure("closed V-jaw inscribed diameter", 0.0100, "m", ESTIMATE, "accepts 11-30 mm tubes while closing smaller than the 16.0 mm tube at rest")
R_CLOSED = D_CLOSED / 2.0
V_HALF_ANGLE = math.radians(30.0)  # 120-deg V-cradle (normals at +-30 deg from +-y): cradles round tubes with wide front entry
TAIL_FLARE = math.radians(22.0)  # outward angle of short_lever's squeeze tail behind the pin
LONG_Y = -0.0090  # m: centreline y of long_arm's straight rear shaft (-0.0150 .. -0.0030)
SHORT_PIN_Y = 0.0090  # m: centreline y of short_lever at the pin before flaring outward

# At theta = THETA_MAX, jaw opening widens from D_CLOSED (10.0 mm) to JAW_OPEN_MAX (31.8 mm):
# each 90-deg V-notch shifts along y by X_JAW * sin(theta); radial tube clearance grows by X_JAW * sin(theta) / sqrt(2),
# while the direct y-gap between upper and lower jaws grows by X_JAW * sin(theta).
THETA_MAX = math.asin((JAW_OPEN_MAX - D_CLOSED) / X_JAW)  # 0.4076 rad = 23.35 deg -> jaw gap = 31.8 mm exact!
JAW_REST = figure(
    "cross-spring preload angle behind closed stop",
    -0.240,
    "rad",
    ESTIMATE,
    "torsion spring preloaded 13.8 deg past closed (max drive error 37.1 deg < 45 deg kit rule)",
)
JAW_K = figure(
    "cross-spring angular stiffness",
    2.20,
    "N m/rad",
    ESTIMATE,
    "gives 15.6 N normal clamping load on a 16.0 mm tube and 23.0 N finger squeeze at 28.5 mm opening",
)
JAW_DAMPING = 0.08  # N m s/rad
LONG_MASS = 0.0294  # kg (~65 % of 45.4 g)
SHORT_MASS = CLAMP_MASS - LONG_MASS  # 0.0160 kg (~35 % of 45.4 g)

# Start pose of the clamp lying flat on its 16 mm side on the blue silicone mat, pointing +x:
CLAMP_START_PIN = (0.340, -0.205, MAT_T + BAR_H / 2.0 + OVER)  # tip at x = 0.411, butt at x = 0.1506 (wholly inside 0.125..0.425 mat)


def jaw_gap(theta: float) -> float:
    """Direct y-opening (m) between the two V-jaws at hinge travel ``theta`` (rad >= 0)."""
    return D_CLOSED + X_JAW * math.sin(max(0.0, float(theta)))


def tube_y_in_clamp(radius: float = TUBE_R) -> float:
    """y-offset (m) in the clamp frame of a cylinder of ``radius`` seated in long_arm's fixed V-cradle."""
    return (float(radius) - R_CLOSED) / math.cos(V_HALF_ANGLE)


def theta_for_tube(diameter: float = TUBE_D) -> float:
    """Hinge angle (rad) at which all four 30-deg V-cheeks touch a cylinder of ``diameter``."""
    dy = 2.0 * tube_y_in_clamp(float(diameter) / 2.0)
    return math.asin(min(1.0, max(0.0, dy / X_JAW)))


def tail_outer_y(theta: float) -> float:
    """y (m) in the clamp frame of short_lever's outer (+y) face at X_SQUEEZE when jaw_h is at ``theta``."""
    p0 = np.array([X_SQUEEZE, SHORT_PIN_Y + abs(X_SQUEEZE) * math.tan(TAIL_FLARE) + BAR_W / 2.0])
    c, s = math.cos(theta), math.sin(theta)
    return float(s * p0[0] + c * p0[1])


def tail_span(theta: float) -> float:
    """Outer squeeze width (m) across long_arm's -y face and short_lever's +y face at X_SQUEEZE."""
    long_outer = LONG_Y - BAR_W / 2.0  # -0.0150 m
    return tail_outer_y(theta) - long_outer


def tail_mid_y(theta: float) -> float:
    """Midpoint y (m) in the clamp frame between long_arm's -y face and short_lever's +y face at X_SQUEEZE."""
    long_outer = LONG_Y - BAR_W / 2.0
    return 0.5 * (long_outer + tail_outer_y(theta))


def spring_torque(theta: float) -> float:
    """PhysX angular drive restoring torque (N m) pulling jaw_h toward closed at angle ``theta``."""
    return 2.0 * JAW_K * math.sin(0.5 * (float(theta) - JAW_REST))


def jaw_force(theta: float) -> float:
    """Closing force (N) along y at the V-jaw centre X_JAW."""
    return spring_torque(theta) / X_JAW


def squeeze_force(theta: float) -> float:
    """Finger force (N) at X_SQUEEZE needed to hold jaw_h open at ``theta``."""
    return spring_torque(theta) / abs(X_SQUEEZE)


# --- 6. Scoring thresholds (identical in task.py, task.json and README.md) ------------------------------------------
TUBE_DROP_TILT_DEG = 35.0  # whole-trial zero rule: tube tipped past 35 deg
FIXTURE_SLIDE_MAX = 0.020  # m: whole-trial zero rule if bath or rack moves > 20 mm
FIXTURE_TILT_DEG = 10.0  # deg: whole-trial zero rule if bath or rack tilts > 10 deg

CLAMP_SEATED_N = 4.0  # N: simulator stand-in for the jaws holding the tube barrel clear (>= 4 N spring contact)
WELD_OPEN_ARM_DEG = 10.0  # deg: jaws must be squeezed open >= 10 deg before clamping can seat on the 16 mm tube
WELD_RELEASE_DEG = 11.5  # deg: squeezing jaw_h >= 11.5 deg (>= 21 mm gap) immediately releases the barrel hold
WELD_MAKE_N = 6.0  # N: bilateral physical contact load on both jaws required to seat the anti-creep barrel hold
WELD_SLIP_N = 18.0  # N: axial/lateral bore or rack jam load above which the tube slips in the wooden jaws
LIFT_CLEAR_Z = BATH_H + 0.004  # 0.090 m: tube underside clear of the 86 mm dry-bath block (for 0.25)
LIFT_TILT_MAX_DEG = 15.0  # deg: tube upright while lifted clear (for 0.25)

RACK_UPRIGHT_DEG = 8.0  # deg: tube standing upright in a rack hole (for 0.5 and 1.0)
RACK_FLOOR_TOL = 0.006  # m: tube underside within 6 mm of the z = 2 mm stainless rack floor
RACK_HOLE_RADIAL_TOL = 0.004  # m: tube axis within 4 mm of a rack hole centre at both guide tiers

HOLDER_FLAT_DEG = 12.0  # deg: wooden holder lying flat on the mat
HOLDER_MAX_Z = 0.026  # m: wooden holder's centre within 26 mm of the bench on the 2 mm mat
HOLDER_CLEAR = 0.020  # m: holder at least 20 mm clear of tube, bath and rack
HANDS_CLEAR = 0.030  # m: every hull corner of both hands >= 30 mm from tube, bath, rack and holder
HOLD_STEPS = 20  # 1.0 s at 20 Hz
HOLD_DRIFT = 0.003  # m: tube and holder move 3 mm at most during the 1.0 s hold


def summary() -> dict:
    counts = {k: sum(1 for v in FIGURES.values() if v[2] == k) for k in (MAKER, STANDARD, LISTING, ESTIMATE)}
    return {
        "counts": counts,
        "theta_tube_deg": round(math.degrees(theta_for_tube()), 2),
        "theta_max_deg": round(math.degrees(THETA_MAX), 2),
        "jaw_gap_closed_mm": round(1000 * jaw_gap(0.0), 2),
        "jaw_gap_max_mm": round(1000 * jaw_gap(THETA_MAX), 2),
        "tail_span_closed_mm": round(1000 * tail_span(0.0), 2),
        "tail_span_max_mm": round(1000 * tail_span(THETA_MAX), 2),
        "jaw_force_on_tube_N": round(jaw_force(theta_for_tube()), 2),
        "squeeze_force_at_28mm_N": round(squeeze_force(math.asin((0.028 - D_CLOSED) / X_JAW)), 2),
        "target_hole_xy": [round(v, 4) for v in TARGET_HOLE_XY],
    }
