"""Every figure the bench-grinder wheel task rests on, each with its source, and helper geometry derived from them.

Source kinds stored in ``FIGURES`` (the README table is built from these):
- ``maker``: Einhell's TC-BG 150 product page, spare-parts technical sheet, datasheet and operating manual, and
  GEDORE's No. 894 19 open-ended spanner product page.
- ``standard``: OSHA 29 CFR 1910.215 (abrasive wheel machinery), ISO 261 / DIN 13-1 (M12 coarse thread), DIN 894 / ISO 3318.
- ``our fixture``: the bench organizer rack and wrench cradle cut to the stated dimensions.
- ``estimate``: derived figure with a one-clause physical or geometric derivation.

World frame (``station``): metres from the midpoint between the arm bases, x forward, y left, z up from the bench.
"""

from __future__ import annotations

import math

import numpy as np

GRAVITY = 9.81
OVER = 0.0008  # m: loose parts start slightly above their cradle support so they settle cleanly on step 1

MAKER, STANDARD, FIXTURE, ESTIMATE = "maker", "standard", "our fixture", "estimate"

EINHELL_PAGE = "https://www.einhell.de/p/4412632-tc-bg-150/"
EINHELL_SPAREPARTS = "https://www.einhell-service.com/de_DE/show-spareparts/4412632/11019"
EINHELL_MANUAL = (
    "https://d2c5rvsfjg2eub.cloudfront.net/asset/208244749100/"
    "document_5v4h0tfogt54j7p1c0t39q5o1d?attachment;filename=4412632_11019_001_SPK2.pdf"
)
EINHELL_DATASHEET = (
    "https://d2c5rvsfjg2eub.cloudfront.net/asset/208244749100/"
    "document_tm7qql4ljp3tb2jkutmjlp5v3i?attachment;filename=4412632_11019_001_DE_TC_BG_150.pdf"
)
GEDORE_PAGE = "https://shop.gedore.com/en-de/6575060/"
OSHA_215 = "https://www.osha.gov/laws-regs/regulations/standardnumber/1910/1910.215"

FIGURES: dict[str, tuple[float, str, str, str]] = {}


def figure(name: str, value: float, unit: str, source: str, note: str) -> float:
    if source not in (MAKER, STANDARD, FIXTURE, ESTIMATE):
        raise ValueError(f"{name}: unknown source kind {source!r}")
    FIGURES[name] = (float(value), unit, source, note)
    return float(value)


# --- Einhell TC-BG 150 bench grinder (Art.-Nr. 4412632, EAN 4006825649888), unplugged, bolted to the bench ----------
WHEEL_D = figure(
    "grinding wheel outer diameter",
    0.150,
    "m",
    MAKER,
    f"Einhell TC-BG 150 (Art.-Nr. 4412632): 'zwei 16 mm dicken Schleifscheiben (Ø 150 mm)' ({EINHELL_PAGE})",
)
WHEEL_R = WHEEL_D / 2.0
WHEEL_T = figure(
    "grinding wheel thickness",
    0.016,
    "m",
    MAKER,
    f"Einhell TC-BG 150 manual sec. 4: 'Schleifscheibendicke: 16 mm' ({EINHELL_MANUAL})",
)
WHEEL_NATIVE_BORE = figure(
    "grinding wheel native bore (before factory reducing bushing)",
    0.032,
    "m",
    MAKER,
    f"Einhell TC-BG 150 manual sec. 4: 'ø Bohrung Schleifscheibe: 32 mm'; accessory 49507435 '150x32/12,7x16mmK36' ({EINHELL_DATASHEET})",
)
ARBOR_D = figure(
    "arbor wheel-seat diameter",
    0.0127,
    "m",
    MAKER,
    f"Einhell TC-BG 150 technical data: 'Schleifscheibenaufnahme-Durchmesser: 12.7 mm' ({EINHELL_SPAREPARTS}; {EINHELL_DATASHEET})",
)
ARBOR_R = ARBOR_D / 2.0
NO_LOAD_RPM = figure(
    "grinder rated no-load speed",
    2980.0,
    "min^-1",
    MAKER,
    f"Einhell TC-BG 150 manual sec. 4: 'Leerlaufdrehzahl n0: 2980 min^-1', 'Umfangsgeschwindigkeit: 23,4 m/s' ({EINHELL_MANUAL})",
)
GRINDER_MASS = figure(
    "bench grinder product mass",
    5.89,
    "kg",
    MAKER,
    f"Einhell TC-BG 150: 'Produktgewicht: 5.89 kg' ({EINHELL_PAGE})",
)
REST_GAP = figure(
    "left workpiece rest clearance to installed wheel",
    0.002,
    "m",
    MAKER,
    f"Einhell TC-BG 150 manual sec. 5.4 Fig. 7: 'max. 2 mm' between Werkstueckauflage (7) and wheel ({EINHELL_MANUAL})",
)
ARBOR_Z = figure(
    "arbor centreline height over the bench",
    0.098,
    "m",
    ESTIMATE,
    f"scaled from Einhell TC-BG 150 front view (Fig. 1 & Fig. 11: 75 mm wheel radius + 23 mm base clearance; {EINHELL_MANUAL})",
)

GRINDER_X = 0.325  # m forward of the arm-base line
GRINDER_Y = 0.042  # m left of the midpoint; left (+y) wheel installed, right (-y) arbor bare for service
MOTOR_R, MOTOR_LEN = 0.062, 0.136
BASE_SIZE = (0.168, 0.142, 0.028)
LEFT_WHEEL_Y = GRINDER_Y + 0.105  # +0.147 m: installed K60 fine wheel centre (272 mm shaft span inside 310 mm carton)
Y_INNER_FACE = GRINDER_Y - 0.095  # -0.053 m: wheel-contact face of the keyed Ø 52 mm inner driving flange on -y arbor
ARBOR_PROTRUSION = 0.052  # m of bare Ø 12.7 mm / M12 right-hand threaded shaft extending past Y_INNER_FACE
Y_ARBOR_TIP = Y_INNER_FACE - ARBOR_PROTRUSION  # -0.105 m

WHEEL_MASS = figure(
    "single 150 x 16 mm vitrified wheel mass",
    0.520,
    "kg",
    ESTIMATE,
    "vitrified corundum density ~2,000 kg/m3 times net volume of Ø 150 x 16 mm wheel with bushed Ø 12.7 mm bore",
)
BORE_D = figure(
    "simulated wheel, flange and nut bore collider diameter",
    0.0136,
    "m",
    ESTIMATE,
    "0.45 mm radial sliding clearance per side over the Ø 12.7 mm horizontal steel arbor for 16-stave rigid contact",
)
BORE_R = BORE_D / 2.0

# --- Flanges and blotters (OSHA 29 CFR 1910.215(c)-(d) + Einhell manual Fig. 11) -------------------------------------
FLANGE_MIN_D = figure(
    "minimum straight-wheel flange diameter",
    0.050,
    "m",
    STANDARD,
    f"OSHA 29 CFR 1910.215(c)(1): 'not be less than one-third the diameter of the wheel' (150 mm / 3 = 50 mm; {OSHA_215})",
)
FLANGE_MIN_RELIEF = figure(
    "minimum flange recess depth on wheel side",
    0.0016,
    "m",
    STANDARD,
    f"OSHA 29 CFR 1910.215(c)(5)(i): 'recessed at least one-sixteenth inch on the side next to the wheel' ({OSHA_215})",
)
FLANGE_D = figure(
    "steel inner and outer flange outer diameter",
    0.052,
    "m",
    ESTIMATE,
    f"stamped steel dished flange >= 50 mm minimum, scaled from Einhell manual Fig. 11 item C against Ø 150 mm wheel ({EINHELL_MANUAL})",
)
FLANGE_R = FLANGE_D / 2.0
FLANGE_DEPTH = figure(
    "stamped steel outer flange overall axial depth",
    0.008,
    "m",
    ESTIMATE,
    "3.0 mm stamped steel wall formed with a 5.0 mm concave relief dish (Einhell manual Fig. 11 item C)",
)
FLANGE_RIM_T = 0.003
FLANGE_RELIEF = FLANGE_DEPTH - FLANGE_RIM_T  # 5.0 mm dish relief (> 1.6 mm OSHA minimum)
FLANGE_MASS = figure(
    "steel outer flange mass",
    0.095,
    "kg",
    ESTIMATE,
    "steel density 7,850 kg/m3 times volume of Ø 52 mm x 3 mm stamped steel dished washer",
)
BLOTTER_D = figure(
    "compressible paper blotter diameter",
    0.060,
    "m",
    ESTIMATE,
    f"covers the entire Ø 52 mm flange contact ring per OSHA 29 CFR 1910.215(c)(6) and (d)(5) ({OSHA_215})",
)
BLOTTER_R = BLOTTER_D / 2.0

# --- M12 x 1.75 right-hand arbor nut (SW 19 mm, Einhell manual sec. 5.5) ---------------------------------------------
NUT_AF = figure(
    "arbor nut width across flats",
    0.019,
    "m",
    MAKER,
    f"Einhell TC-BG 150 manual sec. 5.5: 'Loesen Sie die Mutter (B), indem Sie einen Schraubenschluessel SW 19 mm ansetzten' ({EINHELL_MANUAL})",
)
THREAD_PITCH = figure(
    "right-hand arbor thread pitch (M12 coarse)",
    0.00175,
    "m",
    STANDARD,
    "ISO 261 / DIN 13-1 coarse pitch M12 x 1.75 mm/rev; right-side wheel uses right-hand self-tightening thread",
)
NUT_LEN = figure(
    "flanged arbor hex nut axial length",
    0.014,
    "m",
    ESTIMATE,
    "3.0 mm Ø 28 mm washer collar plus 11.0 mm SW 19 hex body (DIN 6923 M12 proportions, Einhell manual Fig. 11 item B)",
)
NUT_COLLAR_D = 0.028
NUT_COLLAR_T = 0.003
NUT_HEX_LEN = NUT_LEN - NUT_COLLAR_T
NUT_MASS = figure(
    "flanged arbor hex nut mass",
    0.038,
    "kg",
    ESTIMATE,
    "steel M12 flanged hex nut (SW 19, 14 mm tall) from steel density 7,850 kg/m3",
)

# --- GEDORE No. 894 19 single open-ended spanner (Code 6575060, DIN 894 / ISO 3318) ----------------------------------
WRENCH_SW = figure(
    "spanner nominal jaw opening",
    0.019,
    "m",
    MAKER,
    f"GEDORE No. 894 19 (Code 6575060): 'Width across flats [mm]: 19 mm', DIN 894 ISO 3318 ({GEDORE_PAGE})",
)
WRENCH_JAW_GAP = figure(
    "spanner simulated jaw collider opening",
    0.0196,
    "m",
    ESTIMATE,
    "19.0 mm nominal DIN 894 jaw plus 0.30 mm clearance per side over the 19.0 mm hex flats (locks on 21.94 mm hex corners)",
)
WRENCH_L = figure(
    "spanner total length",
    0.170,
    "m",
    MAKER,
    f"GEDORE No. 894 19 (Code 6575060): 'Total length [mm]: 170 mm' ({GEDORE_PAGE})",
)
WRENCH_HEAD_W = figure(
    "spanner head outer width",
    0.040,
    "m",
    MAKER,
    f"GEDORE No. 894 19 (Code 6575060): 'Width of head 1 [mm]: 40,0 mm' ({GEDORE_PAGE})",
)
WRENCH_T = figure(
    "spanner head and shank thickness",
    0.007,
    "m",
    MAKER,
    f"GEDORE No. 894 19 (Code 6575060): 'Head height 1 [mm]: 7 mm', 'Total height [mm]: 7 mm' ({GEDORE_PAGE})",
)
WRENCH_JAW_DEG = figure(
    "spanner jaw offset angle",
    15.0,
    "deg",
    MAKER,
    f"GEDORE No. 894 19 (Code 6575060): 'Jaw set: 15 deg' ({GEDORE_PAGE})",
)
WRENCH_MASS = figure(
    "spanner mass",
    0.081,
    "kg",
    MAKER,
    f"GEDORE No. 894 19 (Code 6575060): 'Weight [kg]: 0,081 kg' ({GEDORE_PAGE})",
)
WRENCH_SHANK_W = 0.018  # m: flat shank width tapering into the 40 mm head

# --- Clamping stack stiffness, torque-angle law and bench witness-angle thresholds -----------------------------------
STACK_K = figure(
    "blottered flange stack axial stiffness",
    1.12e7,
    "N/m",
    ESTIMATE,
    "two 0.6 mm compressed paper blotters and 3 mm dished steel flange over 867 mm2 annular bearing ring (11.2 kN/mm)",
)
NUT_FACTOR_K = figure(
    "steel M12 nut torque factor K",
    0.20,
    "-",
    ESTIMATE,
    "dry steel-on-steel coarse thread and washer face nut factor in tau = tau_0 + K * d * F_clamp",
)
TAU_RUN = figure(
    "running thread drag before snug contact",
    0.15,
    "N m",
    ESTIMATE,
    "dry M12 thread running friction prior to flange contact, overcoming rotor bearing drag only when shaft is held",
)
SPINDLE_DAMPING = 0.08  # N m s/rad: unplugged induction rotor viscous bearing drag
STEEL_MU = figure(
    "bare steel spanner jaw on steel hex flats, static friction",
    0.20,
    "-",
    ESTIMATE,
    "dry machined steel on steel, middle of the 0.15-0.25 handbook range; the jaw's 18 deg mouth chamfers only lead the hex in below tan 18 deg = 0.32",
)
COUNTER_RUBBER_MU = figure(
    "sleeved gripper pad static friction on wheel and steel shank",
    1.35,
    "-",
    ESTIMATE,
    "nitrile-coated protective pad sleeves on vitrified corundum wheel and manganese-phosphated steel",
)

# Exact slope: d(tau)/d(phi) = K * ARBOR_D * STACK_K * (THREAD_PITCH / (2*pi)) = 7.916 N m/rad = 0.13816 N m/deg
TAU_PER_RAD = NUT_FACTOR_K * ARBOR_D * STACK_K * (THREAD_PITCH / (2.0 * math.pi))
TAU_PER_DEG = TAU_PER_RAD * math.radians(1.0)

TAU_MIN = figure(
    "minimum safe arbor nut tightening torque",
    2.5,
    "N m",
    ESTIMATE,
    "gives 925 N blotter preload across both Ø 52 mm flanges (>= 185 N rim friction at 0.2, >10x the 18 N slip/tug check)",
)
TAU_MAX = figure(
    "maximum safe arbor nut tightening torque (6 in wheel, 1/2 in arbor)",
    27.1,
    "N m",
    MAKER,
    "another maker's limit for the same wheel size (Einhell gives no number: 'maessig festziehen'): JET JBG-6W manual "
    "M-577126 sect. 9.3: \'Do not overtighten nut; this may cause wheel to crack. Maximum safe torque on nut is 20 lbf ft (270 kgf cm)\' = 27.1 N m (https://assets.jpwindustries.com/asset/ade3860d-b808-4c72-8024-e4f21839d326/M-577126_JBG-W-Bench-Grinders_ED4.pdf)",
)
TAU_CRACK = TAU_MAX  # the over-tightening zero sits at the same published limit

# Corresponding witness-stripe rotation angles past hand-snug (bench-readable by protractor / 60-deg hex flats):
DEG_SNUG_MIN = (TAU_MIN - TAU_RUN) / TAU_PER_DEG  # 17.01 deg -> 17.0 deg
DEG_SNUG_MAX = (TAU_MAX - TAU_RUN) / TAU_PER_DEG  # 194.88 deg -> 195.0 deg (the simulated hinge stops at 150 deg)
DEG_CRACK = DEG_SNUG_MAX
ANGLE_MIN_DEG = figure("witness-stripe lower safe angle past snug", 17.0, "deg", ESTIMATE, "angle past hand-snug where thread torque reaches 2.5 N m (0.28 hex flat)")
ANGLE_MAX_DEG = figure("witness-stripe upper safe angle past snug", 195.0, "deg", ESTIMATE, "angle past hand-snug where the thread torque reaches the 27.1 N m limit by the clamp estimate (3.25 hex flats)")
ANGLE_CRACK_DEG = ANGLE_MAX_DEG  # past it the trial scores 0

# Nut hinge coordinates on spindle: built at 0 rad (snug reference = 0 rad), [-90, +150] deg inside [-180, +180] deg
NUT_SNUG_TRAVEL = 0.0
NUT_LIMITS = (math.radians(-90.0), math.radians(150.0))  # well inside PhysX's [-180, +180] deg clamp
NUT_DRIVE_K = 120.0  # N m/rad: max slip offset at 10.5 N m is 5.0 deg << 45 deg kit rule
NUT_DRIVE_D = 1.80
# kg m^2 (Ixx, Iyy, Izz about the thread axis y): the thread carrier, and the nut once threaded, spin like a ~0.1 kg
# ring of 45 mm instead of their own ~3e-6 (at ~2e-5 the threaded nut let the jaw roll through at 1.7 N m, run gw_d10;
# the free nut keeps its shapes' inertia: at 5x it fell off the arbor before the catch, run gw_d11)
SOLVER_SPIN_INERTIA = (1.0e-4, 2.0e-4, 1.0e-4)
# kg: once threaded (the catch weld made) the nut is carried at a quarter kilogram, as is its thread carrier; it is
# handled at its real 38 g (NUT_MASS) before. At 38 g the held spanner's cheek sank into the hex at 15-20 N and rolled
# round its corner (run gw_p17); 0.25 kg held ~90 N a corner to 3.5 N m (run gw_p19), 0.6 kg no better (2.95 N m,
# run gw_p20)
NUT_SOLVER_MASS = 0.25
CATCH_TILT_DEG = 2.0  # deg: the nut's axis off the arbor's when its thread can catch (more cross-threads)

# --- Start layout in station frame (tray rack + wrench cradle) -------------------------------------------------------
SOUND_START = (0.170, -0.132, 0.076)    # x, y, z_above_bench: upright, faces square to the arbor, so the carry is one straight +x slide
SOUND_YAW = 0.0
CRACKED_START = (0.305, 0.275, 0.083)   # upright on left, yawed -35 deg so top_cam sees radial crack + rim chip clearly
CRACKED_YAW = math.radians(-35.0)
FLANGE_START = (0.185, -0.040, 0.046)   # upright on edge in the centre-right slot, concave face pointing +x
NUT_START = (0.185, -0.192, 0.036)      # upright on its right-hand pedestal, bore along x, collar pointing +x
WRENCH_REST_PITCH = 0.0                 # lies horizontal on edge in its notched cradle
WRENCH_START = (0.275, -0.250, 0.048)   # jaw-centre start in the level edge cradle

# Seated positions on the -y right arbor (centre coordinates along y)
Y_WHEEL_SEATED = Y_INNER_FACE - WHEEL_T / 2.0                        # -0.058 m
Y_FLANGE_SEATED = Y_INNER_FACE - WHEEL_T - FLANGE_DEPTH / 2.0        # -0.070 m (when recessed face points +y)
Y_NUT_SNUG = Y_INNER_FACE - WHEEL_T - FLANGE_DEPTH - NUT_LEN / 2.0   # -0.081 m

# --- Scoring tolerances (same numbers in task.py, task.json, README.md) ----------------------------------------------
CRACKED_MOVE_ZERO = 0.010   # m: moving the cracked wheel > 10 mm from its start slot loses the trial (0.0)
DROP_Z_BELOW_TOP = -0.015   # m: any free part dropping below bench plane loses the trial (0.0)
RADIAL_TOL = 0.0030         # m: wheel / flange coaxiality with arbor (<= 3.0 mm)
AXIAL_GAP_TOL = 0.0025      # m: wheel inner face to inner flange, and outer flange bearing rim to wheel blotter (<= 2.5 mm)
TILT_TOL_DEG = 3.5          # deg: mounted wheel and outer flange perpendicular to arbor (reversed flange = 180 deg)
WRENCH_CLEAR_NUT = 0.100    # m: the spanner's jaw at least this far from the nut when it is set down
WRENCH_DOWN_Z = 0.080       # m: ... and at most this high over the bench (its rack holds the jaw at 48 mm)
HANDS_CLEAR = 0.030         # m: both hands' hulls >= 30 mm clear of grinder, wheels, flange, nut and spanner
HOLD_DRIFT = 0.003          # m: mounted parts and spanner drift <= 3.0 mm during the 1.0 s hold
REST_LIN_MAX = 0.025        # m/s
HOLD_STEPS = 20             # 1.0 s at 20 Hz


def clamp_force_for_angle_deg(phi_deg: float) -> float:
    """Axial blotter clamping force (N) at witness-stripe angle ``phi_deg`` past hand-snug."""
    if phi_deg <= 0.0:
        return 0.0
    advance_m = THREAD_PITCH * (math.radians(phi_deg) / (2.0 * math.pi))
    return STACK_K * advance_m


def thread_torque_for_angle_deg(phi_deg: float, stack_clamped: bool = True) -> float:
    """Bilateral M12 thread resistance torque (N m) at ``phi_deg`` past snug."""
    if not stack_clamped or phi_deg <= 0.0:
        return TAU_RUN
    return TAU_RUN + NUT_FACTOR_K * ARBOR_D * clamp_force_for_angle_deg(phi_deg)


def wrench_torque_y(r_vec, force_vec) -> float:
    """Signed torque (N m) along +y (tightening the right-hand M12 nut on the -y arbor): (r x F)_y = r_z F_x - r_x F_z."""
    r = np.asarray(r_vec, dtype=float)
    f = np.asarray(force_vec, dtype=float)
    return float(np.cross(r, f)[1])


def summary() -> dict:
    counts: dict[str, int] = {}
    for _, _, kind, _ in FIGURES.values():
        counts[kind] = counts.get(kind, 0) + 1
    return {"total_figures": len(FIGURES), "by_source": counts}
