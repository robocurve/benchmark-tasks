"""Every figure the bathroom-sink pop-up stopper task rests on, each with its source, and its derived geometry.

Source kinds in ``FIGURES`` (used directly to build the README table):
- ``maker``: the manufacturer's published product page, drawing or instruction sheet.
- ``standard``: plumbing waste fitting standard (ASME A112.18.2 / CSA B125.2) or code.
- ``listing``: a retailer's published specification for the named part.
- ``our fixture``: where we place, cut or mark something on the bench.
- ``estimate``: derived figure, with its one-clause derivation.

World frame (``station``): metres from the midpoint between the arm bases, x forward, y left, z up from the bench.
The pivot port's axis points +x (from the drain toward the wall); ``u`` is the distance along it from the drain axis.
"""

from __future__ import annotations

import math

GRAVITY = 9.81
OVER = 0.0006  # m: initial settle clearance for free bodies at reset

MAKER, STANDARD, LISTING, FIXTURE, ESTIMATE = "maker", "standard", "listing", "our fixture", "estimate"
SEEN_IN_SEARCH = "as shown in a web search result (page not opened), 2026-11-21"

DANCO_11041_MRO = "https://mrosupreme.com/products/danco-11041-pop-up-sink-stopper-pop-up-plastic-chrome"
DANCO_88532_HD = "https://www.homedepot.com/p/DANCO-Universal-Ball-Rod-for-Pop-Up-Drains-88532/203193993"
CORNAT_DRAIN_URL = "https://www.bueromarkt-ag.de/ablaufgarnitur_cornat_tec317405_aus_messing,p-tec317405.html"
CORNAT_STOP_URL = "https://www.bueromarkt-ag.de/waschbeckenstoepsel_cornat_exzenterstopfen_durchschnitt_35_mm,p-tec317414.html"
OATEY_DRAIN_PDF = "https://www.oatey.com/sites/default/files/2023-01/raw_15_038753005713_PRODINSTRUCT%20%284%29.pdf"
KOHLER_POPUP_PDF = "https://resources.kohler.com/onlinecatalog/pdf/115999_2.pdf"
DURAVIT_US = "https://www.duravit.com/en-us/p/ME-by-Starck-Washbasin-2335550000/"
DURAVIT_DE = "https://www.duravit.com/de-de/p/ME-by-Starck-Waschtisch-233555AA00/"
DEARBORN_PAGE = "https://www.oatey.com/products/P_041193094797/dearborn-plastic-114-in-tubular-ptraps"
DEARBORN_PDF = "https://media.oatey.com/asset/7a8d545f-8580-4bf1-a5df-0fc7bd83ad78/15_Plastic-Tubular-1-25-P-Trap_SUBMITTAL_001.pdf"

FIGURES: dict[str, tuple[float, str, str, str]] = {}


def figure(name: str, value: float, unit: str, source: str, note: str) -> float:
    if source not in (MAKER, STANDARD, LISTING, FIXTURE, ESTIMATE):
        raise ValueError(f"{name}: unknown source kind {source!r}")
    FIGURES[name] = (float(value), unit, source, note)
    return float(value)


# --- Pop-up stopper: Danco 11041; EU counterpart Cornat TEC317414 -----------------------------------------------------
STOP_H = figure(
    "stopper total height (Danco 11041)", 0.1321, "m", LISTING,
    f"'5.2 in in total length with 1.4 in OD head' ({DANCO_11041_MRO}); EU Cornat TEC317414: '3,5 x 12 cm' ({CORNAT_STOP_URL})",
)
STOP_HEAD_D = figure(
    "stopper head outside diameter (Danco 11041)", 0.0356, "m", LISTING,
    f"'1.4 in OD head' = 35.6 mm ({DANCO_11041_MRO}); EU Cornat TEC317414: 'Durchmesser: 35 mm' ({CORNAT_STOP_URL})",
)
STOP_LAST_HOLE = figure(
    "stopper shaft length under the head to the 6th hole (Danco 11041)", 0.1153, "m", LISTING,
    f"'the pop-up stopper shaft length from underneath the stopper head to the last hole is 4.54 in' ({DANCO_11041_MRO})",
)
STOP_CAP_H = figure(
    "stopper cap and seal height above the flange", 0.0125, "m", ESTIMATE,
    "132.1 mm total minus 115.3 mm shaft to the 6th hole minus a 4.3 mm bottom ring web",
)
STOP_BELOW_SEAL = STOP_H - STOP_CAP_H  # 0.1196 m below the flange when closed
EYE_W = figure("stopper hole inner width", 0.0085, "m", ESTIMATE, "scaled from the Danco 11041 photo against its 1.4 in head")
EYE_H = figure("stopper hole inner height", 0.0100, "m", ESTIMATE, "oblong hole scaled from the Danco 11041 shaft photo")
EYE_WALL = 0.0022  # m: wall around the hole
EYE_THICK = 0.0040  # m: shaft thickness along the hole axis
EYE_DEPTH = figure(
    "used hole's centre under the stopper's seal", 0.110, "m", ESTIMATE,
    "the sixth (last) hole: 115.3 mm to the last hole less half its 10 mm height; the five holes above it taped over",
)
STOP_MASS = figure("stopper mass (chrome-plated ABS)", 0.036, "kg", ESTIMATE, "volume of the 35.6 mm cap and 119.6 mm stem at 1,060 kg/m3")
GUIDE_SPAN = 0.0292  # m: X-fin guide span inside the 31.8 mm bore
FIN_THICK = 0.0032

# --- Pivot ball rod: Danco 88532; EU counterpart in Cornat TEC317405 --------------------------------------------------
ROD_OUTER_L = figure(
    "pivot rod steel shaft length (Danco 88532)", 0.1500, "m", MAKER,
    f"DancoSupport: '150 mm (5.9 inches) in length' ({SEEN_IN_SEARCH}: {DANCO_88532_HD})",
)
ROD_D = figure(
    "pivot rod steel shaft diameter (Danco 88532)", 0.0055, "m", MAKER,
    f"DancoSupport: '5.5 mm (0.22 inches) in diameter' ({SEEN_IN_SEARCH}: {DANCO_88532_HD})",
)
ROD_THREAD_IN = figure(
    "pivot rod thread engagement into the ball (Danco 88532)", 0.0040, "m", MAKER,
    f"DancoSupport: threads in 'approximately 4 mm (0.16 inches)' ({SEEN_IN_SEARCH}: {DANCO_88532_HD})",
)
BALL_D = figure(
    "pivot ball diameter, large ball (Danco 88532)", 0.0183, "m", MAKER,
    f"DancoSupport: 'Large Ball: Diameter = 18.3 mm (0.72 inch)' ({SEEN_IN_SEARCH}: {DANCO_88532_HD})",
)
BALL_R = BALL_D / 2.0
TIP_BEYOND_BALL = figure(
    "inner stub from the ball's edge to the tip (Danco 88532)", 0.0275, "m", MAKER,
    f"DancoSupport: 'distance from the edge of the ball to the end of the gray rod = 27.5 mm (1.08 inches)' ({SEEN_IN_SEARCH}: {DANCO_88532_HD})",
)
TIP_D = figure(
    "inner tip diameter (Danco 88532)", 0.0045, "m", MAKER,
    f"DancoSupport: 'about 4.5 mm at the tip' ({SEEN_IN_SEARCH}: {DANCO_88532_HD})",
)
TIP_R = TIP_D / 2.0
TIP_REACH = BALL_R + TIP_BEYOND_BALL  # 0.03665 m from the ball centre to the tip's end
ROD_REAR_REACH = BALL_R + (ROD_OUTER_L - ROD_THREAD_IN)  # 0.15515 m from the ball centre to the rod's rear end
CLEVIS_BLOCK = (0.022, 0.014, 0.016)  # m: clevis connector left clamped on the rod (its strap is not used)
CLEVIS_AT_X = 0.068  # m behind the ball centre
ROD_MASS = figure(
    "pivot rod + clevis connector mass", 0.044, "kg", ESTIMATE,
    "150 x 5.5 mm steel shaft (28.0 g) + 18.3 mm ball and stub (6.0 g) + 22 x 14 x 16 mm clevis connector (10.0 g)",
)

# --- Pop-up drain body, its port and its pivot nut --------------------------------------------------------------------
DRAIN_ID = figure(
    "lavatory drain bore (1-1/4 in)", 0.0318, "m", STANDARD,
    f"ASME A112.18.2 / CSA B125.2 1-1/4 in lavatory waste; Cornat TEC317405 '1 1/4 Zoll Anschluss' ({CORNAT_DRAIN_URL})",
)
DRAIN_OD = figure("drain body outside diameter", 0.0360, "m", ESTIMATE, "31.8 mm bore plus a 2.1 mm plastic wall")
FLANGE_D = figure(
    "drain flange outside diameter", 0.0540, "m", MAKER,
    f"2-1/8 in (54.0 mm) lavatory pop-up flange (Delta RP5651, {SEEN_IN_SEARCH}); Cornat TEC317405: 62 mm ({CORNAT_DRAIN_URL})",
)
SEAT_OFFSET_X = figure(
    "ball seat behind the drain axis", 0.030, "m", ESTIMATE,
    "ball centre 30 mm behind the drain axis so the 36.65 mm tip ends 6.65 mm past the stopper's axis",
)
SEAT_RING_U = SEAT_OFFSET_X - 0.0075  # m: the seat the ball's front bears on (7.5 mm in front of the ball centre)
SEAT_RING_R = (0.0062, 0.0102)  # m: its inner and outer radius
PORT_BORE_R = 0.0102  # m: port bore radius (20.4 mm round the 18.3 mm ball)
NUT_HAND_TIGHT = figure(
    "pivot nut tightened by hand", 1.0, "-", MAKER,
    f"Oatey brass lift rod drain sheet: 'Insert pivot rod (H) into pop-up body (B) ... then hand-tighten nut (I). DO NOT USE A TOOL' ({OATEY_DRAIN_PDF})",
)
OPEN_LIFT = figure(
    "stopper lift when open", 0.0095, "m", MAKER,
    f"Kohler 115999 pop-up drain: 'adjust the stopper so the stopper lifts about 3/8” when opened. Tighten the nut.' ({KOHLER_POPUP_PDF})",
)
NUT_OD = figure("pivot nut outside diameter (knurled)", 0.031, "m", ESTIMATE, "a 25 mm port thread plus a 3 mm knurled wall each side")
NUT_SKIRT_L = 0.0125  # m: threaded skirt, front face to back wall
NUT_BACK_T = 0.0035  # m: back wall
NUT_L = NUT_SKIRT_L + NUT_BACK_T  # 16.0 mm overall
NUT_MINOR_R = 0.0114  # m: the nut's thread crest radius (22.8 mm)
NUT_HOLE_R = figure(
    "pivot nut rod hole radius", 0.0086, "m", ESTIMATE,
    "17.2 mm hole: under the 18.3 mm ball, so the nut keeps it, and wide enough for the rod to tilt 30 deg",
)
NUT_MASS = figure("pivot nut mass (plastic)", 0.010, "kg", ESTIMATE, "31 x 16 mm ABS cup, 40 % solid, at 1,060 kg/m3")
PORT_THREAD_MAJOR = figure(
    "port thread major diameter", 0.0250, "m", ESTIMATE,
    "20.4 mm ball bore plus a 1.2 mm wall and a 1.1 mm thread each side (not printed by the makers)",
)
THREAD_PITCH = figure("port thread pitch", 0.0016, "m", ESTIMATE, "fine plastic waste thread, 16 per inch (not printed by the makers)")
LAND_OUTER_R = PORT_THREAD_MAJOR / 2.0  # 12.5 mm: the port's first full thread crest at the thread start
PILOT_R = NUT_MINOR_R - 0.0004  # 11.0 mm: the port end's unthreaded lead, which the nut's crests pass over (built not to touch the nut)
LAND_T = 0.0012  # m: axial thickness of the first-thread land
PILOT_L = 0.0015  # m: lead behind the land

# The washer: four rubber beads on the nut's back wall. They meet the seated ball's rear at the thread start, so a nut
# pushed onto the port stops there until its thread catches; the turn to snug squeezes them on the ball
WASHER_BEAD_R = 0.0020  # m
WASHER_RING_R = 0.0095  # m: bead centres from the nut's axis
STOP_DEG = figure(
    "simulator thread travel from its start to snug", 360.0, "deg", ESTIMATE,
    "the stand-in thread is the last turn before the washer is snug; the nut's crests slide over the port's lead before it",
)
S_START = THREAD_PITCH * STOP_DEG / 360.0  # m from the thread start to snug
BEAD_AXIAL_START = math.sqrt((BALL_R + WASHER_BEAD_R) ** 2 - WASHER_RING_R**2)  # 5.84 mm behind the ball centre: beads just touch
BEAD_AXIAL_SNUG = BEAD_AXIAL_START - S_START
WASHER_INTERFERENCE = BALL_R + WASHER_BEAD_R - math.hypot(BEAD_AXIAL_SNUG, WASHER_RING_R)  # 0.75 mm squeeze at snug
WASHER_GRIP = figure(
    "washer bead force on the ball at snug", 1.1, "N", ESTIMATE,
    "4 beads x 1.1 N: the ball turns under them at about 0.028 N m, 1.6 times the 0.018 N m of rod and stopper weight",
)
# The simulator's compliant contact gives far less than its stiffness setting: set at 6.4 N/mm, the beads squeezed 0.70
# mm pressed the ball with 1.7 N in all (sk15); set at 36 N/mm, with 4.3 N in all at snug (trials tr2: the carried nut
# gives 0.4 mm back), which is WASHER_GRIP. The setting comes from those readings.
WASHER_K = 36000.0  # N/m per bead, the compliant-contact setting
WASHER_DAMP = 120.0
WASHER_MU = figure("washer rubber friction on the ball", 1.0, "-", ESTIMATE, "rubber on plastic, dry")
U_START = SEAT_OFFSET_X + BEAD_AXIAL_START - (NUT_SKIRT_L - WASHER_BEAD_R)  # the nut's front face at the thread start
U_SNUG = U_START - S_START  # the nut's front face at snug
U_PORT_END = U_START + PILOT_L  # the port's rear end
THREAD_DRAG_NM = figure(
    "thread running torque", 0.03, "N m", ESTIMATE,
    "plastic nut on a plastic thread: half the 0.06 N m used for a dry steel 5/8-18 thread",
)
THREAD_SPRING_K = 12.0  # N m/rad on each hidden carrier hinge (0.03 N m / 12 = 0.14 deg lag)
THREAD_DAMP = 0.02
UNTHREAD_DEG = -3.0  # deg: turning back past the start releases the thread
N_TURN_HINGES = 3
HINGE_LOWER_RAD = math.radians(-10.0)
HINGE_UPPER_RAD = math.radians(STOP_DEG / N_TURN_HINGES)  # 120 deg each (< 180 deg PhysX)
SLIDE_PLAY = 0.00006  # m axial window on the carrier slide
HIDDEN_MASS = 0.050  # kg per hidden carrier link
HIDDEN_INERTIA = 2.0e-5  # kg m^2 each hidden carrier link (a 50 g body some 50 mm across): the chain holds the nut square
NUT_FACETS = 36  # collision facets round the front of the nut's knurled skirt (10 deg each): fingertips stroking it meet it nearly round
NUT_FRONT_BAND = 0.0065  # m: that front band; behind it the nut has 12 flats, where the finger pads pinch it (a rounder nut rolls in the pinch)
NUT_BACK_FACETS = 12
PORT_FACETS = 36  # the port's lead nearly round too
CATCH_XY_MAX = figure(
    "thread-start coaxial tolerance", 0.0005, "m", ESTIMATE,
    "two thirds of a 0.75 mm flank engagement on a 1.6 mm pitch",
)
CATCH_TILT_DEG = figure(
    "thread-start squareness tolerance", 1.5, "deg", ESTIMATE,
    "60 % of the 2.5 deg at which one side of a 25 mm thread rises half a pitch over the other (atan(0.8 / 25))",
)
CATCH_GAP_BAND = 0.0004  # m: the nut's front face at most 0.4 mm past the thread start (pressed on the washer)
REARM_GAP = 0.0030  # m: after an unthread the start re-arms once the nut is 3 mm back off the port

# --- Basin: Duravit ME by Starck 550 x 440 mm wall-hung washbasin --------------------------------------------------
BASIN_W = figure("basin width (Duravit ME by Starck 233555)", 0.550, "m", MAKER, f"'550 x 440 mm' ({DURAVIT_DE}); US #2335550000 '21 5/8 \" x 17 3/8 \"' ({DURAVIT_US})")
BASIN_D = figure("basin depth, wall to front edge", 0.440, "m", MAKER, f"'Tiefe / Breite 440 mm' ({DURAVIT_DE})")
BASIN_H = figure("basin height", 0.180, "m", MAKER, f"'550 x 180 x 440 mm' ({DURAVIT_DE}); US '21 5/8 \" x 7 1/8 \" x 17 3/8 \"'")
BOWL_W = figure("bowl width", 0.520, "m", MAKER, f"'Beckenbreite 520 mm' ({DURAVIT_DE}); US 'Basin width 20 1/2 \"'")
BOWL_L = figure("bowl length, front to back", 0.290, "m", MAKER, f"'Beckentiefe 290 mm' ({DURAVIT_DE}); US 'Basin Depth 11 3/8 \"'")
FRONT_RIM = figure("front rim width", 0.015, "m", ESTIMATE, "as the side rims: (550 - 520 mm) / 2")
DRAIN_FROM_FRONT = FRONT_RIM + BOWL_L / 2.0  # 160 mm: the drain in the middle of the bowl (estimate)
BOWL_FLOOR = (0.040, 0.050)  # m: half length and half width of the floor round the drain
CERAMIC_T = 0.012  # m: bowl wall
FLOOR_T = 0.015  # m: ceramic under the flange
DECK_T = 0.020  # m: rim deck
APRON_H = 0.040  # m: front and side faces under the rim
BOWL_TERRACES = 12

# --- 1-1/4 in tubular P-trap: Dearborn by Oatey P9701 ----------------------------------------------------------------
TRAP_OD = figure("P-trap tube outside diameter", 0.03175, "m", MAKER, f"Dearborn P9701BG 'Pipe Size: 1.25 in.' ({DEARBORN_PAGE})")
TRAP_J_H = figure("J-bend height, inlet nut top to bend bottom", 0.127, "m", MAKER, f"Dearborn P9701 submittal drawing: '5\"' ({DEARBORN_PDF})")
TRAP_CC = figure("J-bend leg centres", 0.0889, "m", MAKER, f"same drawing: '11\"' inlet to arm end minus '7-1/2\"' outlet to arm end = 3-1/2 in ({DEARBORN_PDF})")
TRAP_ARM_REACH = figure("trap arm reach from the J-bend's outlet", 0.1905, "m", MAKER, f"same drawing: '7-1/2\"' ({DEARBORN_PDF})")
TRAP_ARM_DROP = figure("trap arm axis under the inlet nut's top", 0.013, "m", ESTIMATE, "scaled from the same drawing against its 5 in height")
SLIP_NUT_OD = 0.044  # m
SLIP_NUT_L = 0.016  # m
TRAP_SWING_DEG = figure(
    "J-bend swung toward the left", 20.0, "deg", FIXTURE,
    "J-bend turned 20 deg on the tailpiece: the trap arm (cut to 17 cm) meets a wall outlet 3 cm left of the drain",
)

# --- Placement (our fixture) -----------------------------------------------------------------------------------------
X_DRAIN = figure("drain axis x", 0.455, "m", FIXTURE, "basin front at 29.5 cm, 1.6 cm ahead of the arms' ready pose")
Y_DRAIN = figure("drain axis y", 0.0, "m", FIXTURE, "centred between the arms")
TRAP_U_BOTTOM = figure("J-bend bottom over the bench", 0.006, "m", FIXTURE, "the trap hangs 6 mm clear of the bench")
J_INLET_TOP = TRAP_U_BOTTOM + TRAP_J_H  # 0.133 m: the J-bend's inlet nut top (tailpiece cut to suit)
PORT_Z = figure(
    "pivot port axis over the bench", 0.155, "m", FIXTURE,
    "the basin hangs low: the nut and the rod are reached at or under 16 cm, the pivot nut 6.5 mm over the J-bend's inlet nut",
)
EYE_DROP = 0.0015  # m: the closed stopper's hole centre sits 1.5 mm under the port axis
FLANGE_Z = PORT_Z + EYE_DEPTH - EYE_DROP  # closed stopper seal on the flange
X_FRONT = X_DRAIN - DRAIN_FROM_FRONT
X_WALL = X_FRONT + BASIN_D
Z_FLOOR = FLANGE_Z - 0.0025  # bowl floor under the 2.5 mm flange
Z_UNDER = Z_FLOOR - FLOOR_T  # basin underside at the drain
Z_RIM = Z_UNDER + BASIN_H
Z_DRAIN_BOT = 0.110  # m: lowest bore stave (inside the J-bend's inlet)
TRAP_U_Z = TRAP_U_BOTTOM + TRAP_CC / 2.0 + TRAP_OD / 2.0  # J-bend's bend centre
TRAP_ARM_Z = J_INLET_TOP - TRAP_ARM_DROP
_SW = math.radians(TRAP_SWING_DEG)
TRAP_OUT_XY = (X_DRAIN + TRAP_CC * math.cos(_SW), Y_DRAIN + TRAP_CC * math.sin(_SW))  # the J-bend's outlet leg
WALL_T = 0.018
RETRACT_START = figure(
    "rod's start retraction", 0.010, "m", FIXTURE,
    "ball 10 mm behind its seat: the tip ends 1.35 mm short of the stopper's hole",
)
U_BALL_START = SEAT_OFFSET_X + RETRACT_START
ROD_START_DROP = SEAT_RING_R[0] - TIP_R  # 3.95 mm: the tip rests on the seat ring's lower edge
REST_U = (0.186, 0.200)  # m: rest post under the rod's rear end at the start
NUT_START_DROOP_DEG = 13.0  # deg: the nut hangs on the rod behind the ball, its front down, as it settles
NUT_START_SHIFT = (0.0015, -0.0024)  # m (along x, z) from coaxial on the rod to where it hangs

# --- Scoring thresholds (identical across task.py, task.json, README.md) ---------------------------------------------
FALL_Z = figure("fall: ball or nut centre under this height", 0.030, "m", FIXTURE, "3 cm over the bench: fallen off the port onto the bench")
SEAT_MAX = figure("rod in: ball centre within this of its seat", 0.0030, "m", FIXTURE, "3 mm")
TIP_PAST = 0.003  # m: the tip's end at least 3 mm past the stopper's axis (1.4 mm past the hole's far bar)
SNUG_DEG = figure("snug: within this of the thread's stop", 20.0, "deg", FIXTURE, "0.09 mm of nut travel on a 1.6 mm pitch")
GAP_OPEN_MIN = figure("open: least stopper lift", 0.008, "m", FIXTURE, "8 mm, under the 9.5 mm (3/8 in) the maker sets")
GAP_OPEN_MAX = figure("open: most stopper lift", 0.022, "m", FIXTURE, "22 mm (not pulled out)")
STOP_RADIAL_MAX = 0.0040  # m: stopper upright in the bore
STOP_TILT_MAX_DEG = 8.0
REST_SPEED_MAX = 0.025  # m/s
HANDS_CLEAR = 0.030  # m
HOLD_STEPS = 20  # 1.0 s at 20 Hz
HOLD_DRIFT = 0.002  # m


def gap_for_cw_deg(cw_deg: float) -> float:
    """Axial distance (m) of the nut's front face behind snug at thread travel ``cw_deg``."""
    return THREAD_PITCH * max(0.0, STOP_DEG - float(cw_deg)) / 360.0


def cw_for_gap_deg(gap_m: float) -> float:
    """Thread travel (deg) at which the nut's front face stands ``gap_m`` behind snug (inverse of gap_for_cw_deg)."""
    return STOP_DEG - 360.0 * float(gap_m) / THREAD_PITCH


def open_gap_for_pitch(pitch_down_deg: float) -> float:
    """Stopper lift (m) with the rod's rear pressed down ``pitch_down_deg`` (the tip under the hole's top bar)."""
    lift = SEAT_OFFSET_X * math.sin(math.radians(pitch_down_deg))
    clearance = (EYE_H / 2.0 - EYE_DROP) - TIP_R
    return max(0.0, lift - clearance)


def rod_com_and_inertia() -> tuple[tuple[float, float, float], tuple[float, float, float]]:
    """Centre of mass (m) and principal inertia (kg m^2) of pivot_rod in its ball-centred frame."""
    m_steel, m_ball, m_clevis = 0.028, 0.006, 0.010
    x_steel = BALL_R + (ROD_OUTER_L - ROD_THREAD_IN) / 2.0
    x_ball, x_clevis = -0.003, CLEVIS_AT_X
    cx = (m_steel * x_steel + m_ball * x_ball + m_clevis * x_clevis) / ROD_MASS
    l_s, r_s = ROD_OUTER_L - ROD_THREAD_IN, ROD_D / 2.0
    ixx = 0.5 * m_steel * r_s**2 + 0.4 * m_ball * BALL_R**2 + m_clevis * (CLEVIS_BLOCK[1] ** 2 + CLEVIS_BLOCK[2] ** 2) / 12.0
    iyy = (
        m_steel * (l_s**2 / 12.0 + (x_steel - cx) ** 2)
        + m_ball * (0.4 * BALL_R**2 + (x_ball - cx) ** 2)
        + m_clevis * (CLEVIS_BLOCK[0] ** 2 / 12.0 + (x_clevis - cx) ** 2)
    )
    return (cx, 0.0, 0.0), (max(ixx, 1e-6), iyy, iyy)


def stopper_com_and_inertia() -> tuple[tuple[float, float, float], tuple[float, float, float]]:
    """Centre of mass (m) and inertia (kg m^2) of stopper in its frame (origin at the cap's seal, z = 0)."""
    m_cap, m_stem = 0.014, STOP_MASS - 0.014
    z_cap, z_stem = STOP_CAP_H / 2.0, -STOP_BELOW_SEAL / 2.0
    cz = (m_cap * z_cap + m_stem * z_stem) / STOP_MASS
    r_c = STOP_HEAD_D / 2.0
    izz = 0.5 * m_cap * r_c**2 + 0.5 * m_stem * (GUIDE_SPAN / 2.0) ** 2
    ixx = m_cap * (r_c**2 / 4.0 + (z_cap - cz) ** 2) + m_stem * (STOP_BELOW_SEAL**2 / 12.0 + (z_stem - cz) ** 2)
    return (0.0, 0.0, cz), (ixx, ixx, max(izz, 2e-6))


def nut_com_and_inertia() -> tuple[tuple[float, float, float], tuple[float, float, float]]:
    """Centre of mass (m) and inertia (kg m^2) of the pivot nut in its frame (origin at the front face's centre, +x back)."""
    r_o = NUT_OD / 2.0
    ixx = 0.5 * NUT_MASS * (r_o**2 + NUT_MINOR_R**2)
    iyy = NUT_MASS * (3.0 * (r_o**2 + NUT_MINOR_R**2) + NUT_L**2) / 12.0
    return (0.55 * NUT_L, 0.0, 0.0), (ixx, iyy, iyy)


def summary() -> dict:
    counts: dict[str, int] = {}
    for _, _, kind, _ in FIGURES.values():
        counts[kind] = counts.get(kind, 0) + 1
    return {
        "counts": counts,
        "seat_x_m": round(X_DRAIN + SEAT_OFFSET_X, 4),
        "tip_past_stopper_axis_mm": round(1000.0 * (TIP_REACH - SEAT_OFFSET_X), 2),
        "stopper_bottom_over_bench_mm": round(1000.0 * (FLANGE_Z - STOP_BELOW_SEAL), 2),
        "nut_front_u_snug_mm": round(1000.0 * U_SNUG, 3),
        "washer_squeeze_at_snug_mm": round(1000.0 * WASHER_INTERFERENCE, 3),
        "thread_start_u_mm": round(1000.0 * U_START, 3),
        "bead_axial_at_snug_mm": round(1000.0 * BEAD_AXIAL_SNUG, 3),
        "lift_at_25deg_mm": round(1000.0 * open_gap_for_pitch(25.0), 2),
        "rim_over_bench_mm": round(1000.0 * Z_RIM, 1),
        "underside_at_drain_mm": round(1000.0 * Z_UNDER, 1),
        "trap_arm_axis_mm": round(1000.0 * TRAP_ARM_Z, 1),
        "trap_outlet_xy_m": [round(v, 4) for v in TRAP_OUT_XY],
        "basin_front_x_m": round(X_FRONT, 4),
        "wall_x_m": round(X_WALL, 4),
        "trap_arm_end_gap_to_wall_mm": round(1000.0 * (X_WALL - (TRAP_OUT_XY[0] + TRAP_ARM_REACH)), 1),
    }
