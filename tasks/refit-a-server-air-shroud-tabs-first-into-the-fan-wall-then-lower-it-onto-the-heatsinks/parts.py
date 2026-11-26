"""Every figure the 2U server air-shroud task rests on, in one place, each with its source; and the shapes built from them.

Source kinds (``FIGURES`` holds every figure with its kind and its source in words; the README's table is made from it):
- ``maker``: the maker's own published datasheet or product specification page opened and quoted verbatim.
- ``standard``: a published mechanical standard (EIA-310-D / IEC 60297 19-inch rack unit, JEDEC MO-329 DDR5 RDIMM).
- ``listing``: a distributor's or retailer's product page opened and quoted verbatim.
- ``our fixture``: the 3.0 mm sheet cut-list specified here for the 2U server-bay bench fixture and 2U ABS air shroud.
- ``estimate``: our derived figure, with how it was derived in one clause.

World frame (the station's): metres from the midpoint between the arm bases, x forward, y left, z up from the bench.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from mechkit import Shape, box, cylinder

GRAVITY = 9.81
OVER = 0.0008  # m: loose shroud starts this far above the front staging deck so it settles cleanly at reset

MAKER, STANDARD, LISTING, FIXTURE, ESTIMATE = "maker", "standard", "listing", "our fixture", "estimate"

EIA310_URL = "https://en.wikipedia.org/wiki/19-inch_rack"
INTERTECH_2U_URL = "https://www.inter-tech.de/produktdetails-3/2U_2129-N.html"
DYNATRON_PDF = "https://www.inter-tech.de/files/images/cooler/88885621/S2-Specs-Dynatron.pdf"
INTERTECH_S2_URL = "https://www.inter-tech.de/4181/S-2.html"
KINGSTON_PDF = "https://www.kingston.com/datasheets/KF560R32RB-16.pdf"
ARCTIC_URL = "https://www.arctic.de/en/S8038-7K/ACFAN00288A"
ABS_URL = "https://en.wikipedia.org/wiki/Acrylonitrile_butadiene_styrene"

FIGURES: dict[str, tuple[float, str, str, str]] = {}  # name -> (value, unit, source kind, note)


def figure(name: str, value: float, unit: str, source: str, note: str) -> float:
    if source not in (MAKER, STANDARD, LISTING, FIXTURE, ESTIMATE):
        raise ValueError(f"{name}: unknown source kind {source!r}")
    FIGURES[name] = (float(value), unit, source, note)
    return float(value)


# --- 1. EIA-310-D 2U server-bay bench fixture (our 3.0 mm cut-list, unshrunk 430.0 x 88.1 x 400.0 mm) -----------------
RACK_1U = figure(
    "EIA-310-D 1U rack unit height",
    0.04445,
    "m",
    STANDARD,
    f"EIA-310-D / IEC 60297: '44.45 millimetres (1.75 in) in height' ({EIA310_URL})",
)
CHASSIS_H = figure(
    "2U server bay outer height",
    0.0881,
    "m",
    STANDARD,
    f"EIA-310-D 2U panel height formula 'h = 44.45n - 0.794' mm at n = 2: 88.106 mm ({EIA310_URL}); "
    f"compare Inter-Tech 2U-2129-N SSI-EEB 2U chassis '89 x 482 x 680mm' ({INTERTECH_2U_URL})",
)
CHASSIS_W = figure(
    "2U server bay outer width",
    0.430,
    "m",
    FIXTURE,
    f"430.0 mm body inside ETSI 'maximum width of 17+1/4 inches (438.15 mm)' and EIA-310-D 'gap of 17.75 inches (450.85 mm)' ({EIA310_URL})",
)
CHASSIS_D = figure(
    "2U server bay outer depth (unshrunk)",
    0.400,
    "m",
    FIXTURE,
    "400.0 mm bench fixture reproducing the motherboard, fan bulkhead, and front staging deck of a dual-socket SSI-EEB 2U node without a front drive cage",
)
WALL_T = figure("sheet wall thickness (bay fixture and ABS shroud)", 0.003, "m", FIXTURE, "3.0 mm laser-cut ABS sheet throughout")
INNER_W = CHASSIS_W - 2.0 * WALL_T  # 0.424 m (424.0 mm)
figure("2U server bay inner width", INNER_W, "m", FIXTURE, "430.0 mm outer width minus two 3.0 mm side walls = 424.0 mm")

X_FRONT = figure("bay fixture front face, x", 0.125, "m", FIXTURE, "12.5 cm ahead of the arm-base line; entire 400.0 mm bay lies in x = 12.5..52.5 cm")
X_REAR = X_FRONT + CHASSIS_D  # 0.525 m
DECK_H = figure("front staging deck height above bay floor", 0.008, "m", FIXTURE, "8.0 mm closed front deck where the removed shroud starts")
X_DIV_FRONT = 0.3045
X_DIV_REAR = figure("mid-bay divider rear face, x", 0.3075, "m", FIXTURE, "3.0 mm divider wall at x = 30.45..30.75 cm bounding the rear of the motherboard bay")
DIVIDER_H = figure("mid-bay divider height above bay floor", 0.052, "m", FIXTURE, "52.0 mm divider wall stopping any flat rearward shift while letting a 62 mm carry cross under 14 cm")

X_BULKHEAD = figure("slotted fan bulkhead front face, x", 0.4730, "m", FIXTURE, "fan bulkhead at x = 47.30..47.60 cm, giving a 165.5 mm bay opening (< 170.0 mm shroud + tabs)")
BULKHEAD_H = figure("fan bulkhead top lip height above bay floor", 0.083, "m", FIXTURE, "83.0 mm top lip (5.0 mm above the seated 78.0 mm shroud roof)")
HOOD_DEPTH = 0.011  # m: x = 0.4730..0.4840 m upper keeper hood above both slots
BAY_OPEN_L = X_BULKHEAD - X_DIV_REAR  # 0.1655 m (165.5 mm)

SLOT_Y = figure("fan-wall slot and CPU duct centre offset, |y|", 0.095, "m", FIXTURE, "dual-socket CPU centreline at y = +/-95.0 mm")
SLOT_W = figure("fan-wall retaining slot width", 0.022, "m", FIXTURE, "22.0 mm wide (+/-3.0 mm lateral clearance around each 16.0 mm shroud tab)")
SLOT_H = figure("fan-wall retaining slot height", 0.0075, "m", FIXTURE, "7.5 mm tall, spanning h = 70.5..78.0 mm beneath the 78.0..83.0 mm bulkhead lip")
SLOT_Z_LO = 0.0705
SLOT_Z_HI = SLOT_Z_LO + SLOT_H  # 0.0780 m

# --- 2. Commercial server internals inside the bay (Dynatron S2, JEDEC MO-329 DDR5 RDIMM, Arctic S8038-7K) -------------
SOCKET_H = figure("LGA4677 socket and PCB standoff height above bay floor", 0.008, "m", FIXTURE, "8.0 mm spacer block under both passive heatsinks")
HS_L = figure(
    "Dynatron S2 LGA4677 2U heatsink length",
    0.118,
    "m",
    MAKER,
    f"Dynatron S2 PDF: 'Model Number: S2 ... Socket LGA 4677 ... Dimension: 118 x 78 x 64 mm' ({DYNATRON_PDF}); listing '118 x 64 x 78 mm' ({INTERTECH_S2_URL})",
)
HS_W = figure(
    "Dynatron S2 LGA4677 2U heatsink width",
    0.078,
    "m",
    MAKER,
    f"Dynatron S2 PDF: 'Dimension: 118 x 78 x 64 mm' ({DYNATRON_PDF})",
)
HS_H = figure(
    "Dynatron S2 LGA4677 2U heatsink height",
    0.064,
    "m",
    MAKER,
    f"Dynatron S2 PDF: 'Dimension: 118 x 78 x 64 mm' ({DYNATRON_PDF}); top sits at h = 72.0 mm above bay floor",
)
HS_MASS = figure(
    "Dynatron S2 LGA4677 2U heatsink mass",
    0.5266,
    "kg",
    MAKER,
    f"Dynatron S2 PDF: 'Weight: 526.6 g' ({DYNATRON_PDF}); listing '0,526 kg' ({INTERTECH_S2_URL})",
)

DIMM_RAIL_H = figure("DDR5 RDIMM socket rail height above bay floor", 0.007, "m", FIXTURE, "7.0 mm slotted card rail holding four RDIMM modules per bank")
DIMM_L = figure(
    "JEDEC MO-329 DDR5 RDIMM length (Kingston KF560R32RB-16)",
    0.13335,
    "m",
    MAKER,
    f"Kingston KF560R32RB-16 datasheet Doc. 4810844B p. 2: '133.35' mm +/-0.15 mm, 'JEDEC standard electrical and mechanical specifications' ({KINGSTON_PDF})",
)
DIMM_H = figure(
    "JEDEC MO-329 DDR5 RDIMM PCB height (Kingston KF560R32RB-16)",
    0.03125,
    "m",
    MAKER,
    f"Kingston KF560R32RB-16 datasheet Doc. 4810844B p. 1-2: 'PCB: Height 1.23\" (31.25mm)' ({KINGSTON_PDF}); seated top h = 38.25 mm",
)
DIMM_T = figure(
    "DDR5 RDIMM seated module thickness",
    0.0038,
    "m",
    ESTIMATE,
    "3.80 mm overall card-and-DRAM thickness from 1.27 mm JEDEC MO-329 PCB plus dual-side FBGA packages",
)
DIMM_INNER_YS = (0.016, 0.025, 0.034, 0.043)  # m: 4 modules per inner bank (9.0 mm pitch)
DIMM_OUTER_YS = (0.158, 0.167, 0.176, 0.185)  # m: 4 modules per outer bank (9.0 mm pitch)

FAN_W = figure(
    "Arctic S8038-7K server fan width and height",
    0.080,
    "m",
    MAKER,
    f"ARCTIC S8038-7K (ACFAN00288A): 'Dimensions: 80 (L) x 80 (W) x 38 (H) mm' ({ARCTIC_URL})",
)
FAN_D = figure(
    "Arctic S8038-7K server fan depth",
    0.038,
    "m",
    MAKER,
    f"ARCTIC S8038-7K (ACFAN00288A): 'Dimensions: 80 (L) x 80 (W) x 38 (H) mm', 'Weight: 146 g' ({ARCTIC_URL})",
)
FAN_YS = (-0.164, -0.082, 0.0, 0.082, 0.164)  # 5 fans across 408.0 mm inside 424.0 mm inner bay width
X_FAN_CENTER = X_BULKHEAD + HOOD_DEPTH + FAN_D / 2.0  # 0.5020 m (x = 0.4830..0.5210 m)

# --- 3. One-piece 3.0 mm ABS 2U air shroud (our cut-list, zero roof protrusion above 78.0 mm) --------------------------
SHROUD_W = figure("air shroud outer width", 0.418, "m", FIXTURE, "418.0 mm wide, leaving 3.0 mm clearance per side inside the 424.0 mm bay")
SHROUD_L = figure("air shroud body length", 0.160, "m", FIXTURE, "160.0 mm body length along x (excluding the 10.0 mm front tabs)")
SHROUD_H = figure("air shroud roof height", 0.078, "m", FIXTURE, "78.0 mm flat roof at all four corners (10.1 mm below the 88.1 mm 2U rim; zero roof ribs)")

DUCT_CLEAR = figure(
    "lateral clearance per side between CPU duct wall and heatsink",
    0.0025,
    "m",
    FIXTURE,
    "83.0 mm inner CPU duct spacing around each 78.0 mm Dynatron S2 heatsink = 2.50 mm per side",
)
DUCT_INNER_W = HS_W + 2.0 * DUCT_CLEAR  # 0.083 m (83.0 mm)
DUCT_IN_Y = SLOT_Y - DUCT_INNER_W / 2.0  # 0.0535 m (wall spans 0.0505..0.0535 m)
DUCT_OUT_Y = SLOT_Y + DUCT_INNER_W / 2.0  # 0.1365 m (wall spans 0.1365..0.1395 m)
LOWER_REAR_X = -0.062  # m: 18.0 mm rear step relief below z = 66.0 mm so rear swings clear of the 52.0 mm divider
LOWER_FRONT_X = 0.060  # m: 20.0 mm front nose relief below z = 22.0 mm ahead of the [-59, +59] mm heatsinks so nose-down entry clears the floor
NOSE_RELIEF_Z = 0.022  # m
UPPER_SKIRT_Z = 0.066  # m

STEP_FLOOR_Z = figure(
    "stepped DIMM shroud roof and finger-well floor height",
    0.046,
    "m",
    FIXTURE,
    "46.0 mm top of 3.0 mm stepped floor (z = 43.0..46.0 mm), clearing the 38.25 mm RDIMMs by 4.75 mm and giving 32.0 mm deep finger wells",
)
CORNER_ROOF_L = 0.028  # m: solid 78.0 mm corner roof bands at x in [-80, -52] and [+52, +80] mm
WELL_L = SHROUD_L - 2.0 * CORNER_ROOF_L  # 0.104 m (104.0 mm open finger-well length)
SPINE_W = figure(
    "recessed blue touchpoint bridge pinch width",
    0.026,
    "m",
    FIXTURE,
    "26.0 mm wide touchpoint bridge flush with the 78.0 mm roof inside each 66.5 mm outer wing (20.25 mm finger slot each side)",
)
GRASP_Y = (DUCT_OUT_Y + WALL_T + (SHROUD_W / 2.0 - WALL_T)) / 2.0  # (0.1395 + 0.2060) / 2 = 0.17275 m

TAB_L = figure("shroud front retaining tab projection", 0.010, "m", FIXTURE, "10.0 mm forward (+x) from the shroud front face; total span 170.0 mm > 165.5 mm bay")
TAB_W = figure("shroud front retaining tab width", 0.016, "m", FIXTURE, "16.0 mm wide inside the 22.0 mm bulkhead slot (+/-3.0 mm margin)")
TAB_T = figure("shroud front retaining tab thickness", 0.003, "m", FIXTURE, "3.0 mm ABS tab bonded flush under the 75.0..78.0 mm duct roof (z = 72.0..75.0 mm)")
TAB_Z_MID = 0.0735

X_SEAT = figure("seated shroud centre, x", 0.3915, "m", FIXTURE, "body x = 31.15..47.15 cm (4.0 mm ahead of divider, 1.5 mm before bulkhead; tabs reach 48.15 cm = 8.5 mm in slots)")
X_START = figure("staged shroud start centre, x", 0.2100, "m", FIXTURE, "body x = 13.00..29.00 cm centered on the front staging deck (tabs pointing +x toward the bulkhead)")

ABS_DENSITY = figure(
    "ABS sheet density",
    1070.0,
    "kg/m3",
    ESTIMATE,
    f"midpoint of ABS density '1.060-1.080 g/cm3' ({ABS_URL})",
)
SURFACE_FRICTION = figure(
    "ABS shroud on chassis/heatsink dry friction",
    0.35,
    "-",
    ESTIMATE,
    "unlubricated rigid ABS on painted sheet/aluminium: midpoint of dry thermoplastic 0.30-0.40",
)
CONTACT_OFFSET = figure(
    "PhysX collider contact offset on chassis and shroud",
    0.0006,
    "m",
    ESTIMATE,
    "0.6 mm contact shell (< 1/4 of the 2.50 mm heatsink clearance so PhysX preserves true millimetre gaps)",
)

# --- 4. Scored limits (identical in task.py, task.json, README.md) -----------------------------------------------------
TAB_IN_MIN = figure("minimum tab insertion into each fan-wall slot for 0.5 and 1.0", 0.005, "m", FIXTURE, "at least 5.0 mm past the x = 47.30 cm bulkhead front face (7.5 mm nominal)")
SLOT_Y_HALF_TOL = (SLOT_W - TAB_W) / 2.0 + 0.002  # 0.005 m: tab centre within 5.0 mm of slot centre (tab inside 22 mm slot)
ROOF_TARGET = SHROUD_H  # 0.078 m above bay floor
ROOF_TOL = figure("four-corner seated roof height tolerance", 0.0015, "m", FIXTURE, "all four outer roof corners at 78.0 +/- 1.5 mm above the bay floor")
HANDS_CLEAR = figure("minimum hand clearance for 1.0", 0.030, "m", FIXTURE, "every hull corner of both hands at least 30.0 mm from the shroud and bay internals")
HOLD_S = figure("seated hold duration for 1.0", 1.0, "s", FIXTURE, "20 steps at 20 Hz after seating and letting go")
HOLD_STEPS = int(round(HOLD_S * 20))
HOLD_DRIFT = figure("maximum shroud movement during the 1.0 s hold", 0.002, "m", FIXTURE, "at most 2.0 mm centre drift during the hold")
FOUL_GAP = 0.0008  # m: gripper hull within 0.8 mm of a heatsink or RDIMM bank counts as a touch in PhysX

# --- Colours (sRGB) ----------------------------------------------------------------------------------------------------
CHASSIS_GREY = (0.58, 0.60, 0.63)
DECK_GREY = (0.64, 0.66, 0.69)
PCB_GREEN = (0.09, 0.29, 0.17)
COPPER = (0.76, 0.45, 0.24)
ALU_SILVER = (0.80, 0.82, 0.85)
FIN_DARK = (0.34, 0.36, 0.39)
SOCKET_WHITE = (0.88, 0.88, 0.85)
DIMM_GREEN = (0.14, 0.44, 0.25)
FAN_BLACK = (0.16, 0.17, 0.19)
SHROUD_ABS = (0.15, 0.16, 0.18)
WELL_ABS = (0.25, 0.27, 0.30)
TOUCH_BLUE = (0.12, 0.50, 0.88)
TAB_LIGHT = (0.88, 0.90, 0.93)
SLOT_TRIM = (0.96, 0.96, 0.94)
MARK_YELLOW = (0.96, 0.82, 0.12)


@dataclass(frozen=True)
class BoxPart:
    label: str
    size: tuple[float, float, float]
    pos: tuple[float, float, float]
    color: tuple[float, float, float]
    grippy: bool = False


def shroud_parts() -> list[BoxPart]:
    """Exact non-overlapping solid boxes of the 3.0 mm ABS shroud in its local frame (origin at bottom-centre, z in [0, 0.078])."""
    parts: list[BoxPart] = []
    duct_outer_span = DUCT_INNER_W + 2.0 * WALL_T  # 0.089 m (y: 0.0505..0.1395)
    center_w = 2.0 * (DUCT_IN_Y - WALL_T)  # 0.101 m (y: -0.0505..+0.0505)
    wing_in = DUCT_OUT_Y + WALL_T  # 0.1395 m
    wing_out = SHROUD_W / 2.0 - WALL_T  # 0.2060 m
    wing_w = wing_out - wing_in  # 0.0665 m
    lower_l = SHROUD_L / 2.0 - LOWER_REAR_X  # 0.142 m (x: -0.062..+0.080 for z in [22, 66] mm)
    lower_cx = (SHROUD_L / 2.0 + LOWER_REAR_X) / 2.0  # +0.009 m
    foot_l = LOWER_FRONT_X - LOWER_REAR_X  # 0.122 m (x: -0.062..+0.060 for z in [0, 22] mm, spanning 100% of 118 mm heatsinks)
    foot_cx = (LOWER_FRONT_X + LOWER_REAR_X) / 2.0  # -0.001 m
    mid_h = UPPER_SKIRT_Z - NOSE_RELIEF_Z  # 0.044 m
    mid_cz = (UPPER_SKIRT_Z + NOSE_RELIEF_Z) / 2.0  # 0.044 m

    # Center stepped DIMM floor (z = 0.043..0.046) + front/rear 78 mm cross-ties & vertical end webs
    parts.append(BoxPart("center_dimm_floor", (SHROUD_L, center_w, WALL_T), (0.0, 0.0, STEP_FLOOR_Z - WALL_T / 2.0), WELL_ABS))
    for sx, tag in ((1.0, "front"), (-1.0, "rear")):
        cx = sx * (SHROUD_L / 2.0 - CORNER_ROOF_L / 2.0)
        parts.append(BoxPart(f"center_{tag}_roof", (CORNER_ROOF_L, center_w, WALL_T), (cx, 0.0, SHROUD_H - WALL_T / 2.0), SHROUD_ABS))
        wx = sx * (SHROUD_L / 2.0 - WALL_T / 2.0)
        parts.append(BoxPart(f"center_{tag}_web", (WALL_T, center_w, SHROUD_H - WALL_T - STEP_FLOOR_Z), (wx, 0.0, (SHROUD_H - WALL_T + STEP_FLOOR_Z) / 2.0), SHROUD_ABS))

    for s, side in ((1.0, "left"), (-1.0, "right")):
        # Full-height CPU duct roof (z = 0.075..0.078)
        parts.append(BoxPart(f"duct_roof_{side}", (SHROUD_L, duct_outer_span, WALL_T), (0.0, s * SLOT_Y, SHROUD_H - WALL_T / 2.0), SHROUD_ABS))
        # Inner & outer vertical CPU duct walls (upper z = 66..75 full 160 mm; mid z = 22..66 142 mm; foot z = 0..22 122 mm)
        for wtag, wy in (("in", DUCT_IN_Y - WALL_T / 2.0), ("out", DUCT_OUT_Y + WALL_T / 2.0)):
            parts.append(BoxPart(f"duct_{wtag}_up_{side}", (SHROUD_L, WALL_T, SHROUD_H - WALL_T - UPPER_SKIRT_Z), (0.0, s * wy, (SHROUD_H - WALL_T + UPPER_SKIRT_Z) / 2.0), SHROUD_ABS))
            parts.append(BoxPart(f"duct_{wtag}_mid_{side}", (lower_l, WALL_T, mid_h), (lower_cx, s * wy, mid_cz), SHROUD_ABS))
            parts.append(BoxPart(f"duct_{wtag}_lo_{side}", (foot_l, WALL_T, NOSE_RELIEF_Z), (foot_cx, s * wy, NOSE_RELIEF_Z / 2.0), SHROUD_ABS))
        # Outer stepped DIMM floor (z = 0.043..0.046)
        parts.append(BoxPart(f"wing_floor_{side}", (SHROUD_L, wing_w, WALL_T), (0.0, s * GRASP_Y, STEP_FLOOR_Z - WALL_T / 2.0), WELL_ABS))
        # Front & rear outer 78 mm corner roof decks (z = 0.075..0.078)
        for sx, tag in ((1.0, "front"), (-1.0, "rear")):
            cx = sx * (SHROUD_L / 2.0 - CORNER_ROOF_L / 2.0)
            parts.append(BoxPart(f"wing_{tag}_roof_{side}", (CORNER_ROOF_L, wing_w, WALL_T), (cx, s * GRASP_Y, SHROUD_H - WALL_T / 2.0), SHROUD_ABS))
        # Recessed blue touchpoint grip U-bridge (3.0 mm ABS side pinch plates + 3.0 mm top cap, outer width 26.0 mm, z = 46..78 mm)
        sp_h = SHROUD_H - WALL_T - STEP_FLOOR_Z  # 0.029 m
        sp_cz = STEP_FLOOR_Z + sp_h / 2.0
        parts.append(BoxPart(f"touch_bridge_top_{side}", (WELL_L, SPINE_W, WALL_T), (0.0, s * GRASP_Y, SHROUD_H - WALL_T / 2.0), TOUCH_BLUE, grippy=True))
        parts.append(BoxPart(f"touch_bridge_in_{side}", (WELL_L, WALL_T, sp_h), (0.0, s * (GRASP_Y - SPINE_W / 2.0 + WALL_T / 2.0), sp_cz), TOUCH_BLUE, grippy=True))
        parts.append(BoxPart(f"touch_bridge_out_{side}", (WELL_L, WALL_T, sp_h), (0.0, s * (GRASP_Y + SPINE_W / 2.0 - WALL_T / 2.0), sp_cz), TOUCH_BLUE, grippy=True))
        # Outer side wall (|y| = 0.206..0.209: upper z = 66..78 full 160 mm; mid z = 22..66 142 mm; foot z = 0..22 122 mm)
        oy = SHROUD_W / 2.0 - WALL_T / 2.0
        parts.append(BoxPart(f"outer_wall_up_{side}", (SHROUD_L, WALL_T, SHROUD_H - UPPER_SKIRT_Z), (0.0, s * oy, (SHROUD_H + UPPER_SKIRT_Z) / 2.0), SHROUD_ABS))
        parts.append(BoxPart(f"outer_wall_mid_{side}", (lower_l, WALL_T, mid_h), (lower_cx, s * oy, mid_cz), SHROUD_ABS))
        parts.append(BoxPart(f"outer_wall_lo_{side}", (foot_l, WALL_T, NOSE_RELIEF_Z), (foot_cx, s * oy, NOSE_RELIEF_Z / 2.0), SHROUD_ABS))
        # Front +x retaining tab (x = +0.080..+0.090, width 16.0 mm at y = +/-0.095, z = 0.0720..0.0750)
        parts.append(BoxPart(f"front_tab_{side}", (TAB_L, TAB_W, TAB_T), (SHROUD_L / 2.0 + TAB_L / 2.0, s * SLOT_Y, TAB_Z_MID), TAB_LIGHT))
    return parts


def shroud_mass_props() -> tuple[float, tuple[float, float, float], tuple[float, float, float]]:
    """Exact (mass kg, centre of mass m, principal inertia kg m^2 about CoM) of the 3.0 mm ABS shroud."""
    items = []
    for p in shroud_parts():
        vol = p.size[0] * p.size[1] * p.size[2]
        items.append((vol * ABS_DENSITY, np.array(p.size, dtype=float), np.array(p.pos, dtype=float)))
    total_m = sum(m for m, _, _ in items)
    com = sum((m * c for m, _, c in items), np.zeros(3)) / total_m
    inertia = np.zeros(3)
    for m, (lx, ly, lz), c in items:
        d = c - com
        inertia += (m / 12.0) * np.array([ly * ly + lz * lz, lx * lx + lz * lz, lx * lx + ly * ly])
        inertia += m * np.array([d[1] ** 2 + d[2] ** 2, d[0] ** 2 + d[2] ** 2, d[0] ** 2 + d[1] ** 2])
    return (
        float(total_m),
        (float(com[0]), float(com[1]), float(com[2])),
        (float(inertia[0]), float(inertia[1]), float(inertia[2])),
    )


SHROUD_MASS, SHROUD_COM, SHROUD_INERTIA = shroud_mass_props()
figure("computed 3.0 mm ABS air shroud mass", round(SHROUD_MASS, 4), "kg", ESTIMATE, "summed volume of the non-overlapping 3.0 mm ABS shroud plates times 1,070 kg/m3")


def shroud_shapes() -> list[Shape]:
    """Solid and visual shapes of the shroud in its local frame."""
    fric = (SURFACE_FRICTION, SURFACE_FRICTION)
    shapes = [
        box(p.size, pos=p.pos, color=p.color, friction=None if p.grippy else fric)
        for p in shroud_parts()
    ]
    # Crisp server-blue touchpoint top-stripe markers and white front-arrow ticks (drawn only, 0.4 mm thin)
    for s in (1.0, -1.0):
        shapes.append(box((0.016, 0.010, 0.0004), pos=(SHROUD_L / 2.0 - 0.012, s * SLOT_Y, SHROUD_H + 0.0002), color=SLOT_TRIM, solid=False))
    return shapes


def foul_boxes_world(top_z: float) -> list[tuple[str, np.ndarray, np.ndarray]]:
    """Exact world (name, centre, size) of the two CPU heatsinks and four DDR5 RDIMM banks that grippers must never touch."""
    fz = top_z + WALL_T
    out: list[tuple[str, np.ndarray, np.ndarray]] = []
    for s, tag in ((1.0, "left"), (-1.0, "right")):
        out.append((f"heatsink_{tag}", np.array([X_SEAT, s * SLOT_Y, fz + SOCKET_H + HS_H / 2.0]), np.array([HS_L, HS_W, HS_H])))
        in_lo, in_hi = DIMM_INNER_YS[0] - DIMM_T / 2.0, DIMM_INNER_YS[-1] + DIMM_T / 2.0
        out.append((f"dimm_inner_{tag}", np.array([X_SEAT, s * (in_lo + in_hi) / 2.0, fz + (DIMM_RAIL_H + DIMM_H) / 2.0]), np.array([DIMM_L, in_hi - in_lo, DIMM_RAIL_H + DIMM_H])))
        out_lo, out_hi = DIMM_OUTER_YS[0] - DIMM_T / 2.0, DIMM_OUTER_YS[-1] + DIMM_T / 2.0
        out.append((f"dimm_outer_{tag}", np.array([X_SEAT, s * (out_lo + out_hi) / 2.0, fz + (DIMM_RAIL_H + DIMM_H) / 2.0]), np.array([DIMM_L, out_hi - out_lo, DIMM_RAIL_H + DIMM_H])))
    return out


def chassis_shapes(top_z: float) -> tuple[tuple[float, float, float], list[Shape]]:
    """(chassis origin in world, list of shapes in chassis frame): origin at ((X_FRONT + X_REAR)/2, 0, top_z)."""
    cx = (X_FRONT + X_REAR) / 2.0  # 0.325 m
    fz = WALL_T  # bay floor surface in chassis frame (z = 0.003 m)
    wall_h = CHASSIS_H - WALL_T  # 0.0851 m above bay floor
    shapes: list[Shape] = [
        # Base plate (400 x 430 x 3 mm)
        box((CHASSIS_D, CHASSIS_W, WALL_T), pos=(0.0, 0.0, WALL_T / 2.0), color=CHASSIS_GREY),
        # Left & right full-length 2U side walls (88.1 mm total height)
        box((CHASSIS_D, WALL_T, wall_h), pos=(0.0, INNER_W / 2.0 + WALL_T / 2.0, fz + wall_h / 2.0), color=CHASSIS_GREY),
        box((CHASSIS_D, WALL_T, wall_h), pos=(0.0, -(INNER_W / 2.0 + WALL_T / 2.0), fz + wall_h / 2.0), color=CHASSIS_GREY),
        # Rear exhaust wall (x = 0.522..0.525 m) and low front trim wall (x = 0.125..0.128 m, h = 24 mm)
        box((WALL_T, INNER_W, wall_h), pos=(CHASSIS_D / 2.0 - WALL_T / 2.0, 0.0, fz + wall_h / 2.0), color=CHASSIS_GREY),
        box((WALL_T, INNER_W, 0.024), pos=(-CHASSIS_D / 2.0 + WALL_T / 2.0, 0.0, fz + 0.012), color=CHASSIS_GREY),
        # Front closed staging deck (x = 0.1280..0.3055 m, h = 8.0 mm)
        box((X_DIV_FRONT - (X_FRONT + WALL_T), INNER_W, DECK_H), pos=(((X_DIV_FRONT + X_FRONT + WALL_T) / 2.0) - cx, 0.0, fz + DECK_H / 2.0), color=DECK_GREY),
        # Mid-bay divider wall (x = 0.3055..0.3085 m, h = 52.0 mm)
        box((WALL_T, INNER_W, DIVIDER_H), pos=(((X_DIV_FRONT + X_DIV_REAR) / 2.0) - cx, 0.0, fz + DIVIDER_H / 2.0), color=CHASSIS_GREY),
        # Motherboard PCB visual sheet on bay floor (x = 0.309..0.472 m)
        box((BAY_OPEN_L - 0.001, INNER_W - 0.002, 0.0008), pos=(((X_DIV_REAR + X_BULKHEAD) / 2.0) - cx, 0.0, fz + 0.0004), color=PCB_GREEN, solid=False),
    ]

    # Yellow L-ticks on the front staging deck marking the shroud's start rectangle
    for sx in (1.0, -1.0):
        for sy in (1.0, -1.0):
            tx = X_START + sx * (SHROUD_L / 2.0 + 0.003) - cx
            ty = sy * (SHROUD_W / 2.0 + 0.001)
            shapes.append(box((0.004, 0.018, 0.0006), pos=(tx, ty - sy * 0.008, fz + DECK_H + 0.0003), color=MARK_YELLOW, solid=False))

    # Slotted fan bulkhead at x = 0.4730..0.4760 m around the two 22.0 x 7.5 mm slots at y = +/-0.095 m, h = 70.5..78.0 mm
    bx = X_BULKHEAD + WALL_T / 2.0 - cx
    s_in, s_out = SLOT_Y - SLOT_W / 2.0, SLOT_Y + SLOT_W / 2.0  # 0.084, 0.106
    shapes.extend([
        box((WALL_T, INNER_W, SLOT_Z_LO), pos=(bx, 0.0, fz + SLOT_Z_LO / 2.0), color=CHASSIS_GREY),
        box((WALL_T, 2.0 * s_in, SLOT_H), pos=(bx, 0.0, fz + SLOT_Z_LO + SLOT_H / 2.0), color=CHASSIS_GREY),
        box((WALL_T, INNER_W / 2.0 - s_out, SLOT_H), pos=(bx, (INNER_W / 2.0 + s_out) / 2.0, fz + SLOT_Z_LO + SLOT_H / 2.0), color=CHASSIS_GREY),
        box((WALL_T, INNER_W / 2.0 - s_out, SLOT_H), pos=(bx, -(INNER_W / 2.0 + s_out) / 2.0, fz + SLOT_Z_LO + SLOT_H / 2.0), color=CHASSIS_GREY),
        # Upper bulkhead lip & 10.0 mm keeper roof (h = 78.0..83.0 mm, x = 0.4730..0.4830 m)
        box((HOOD_DEPTH, INNER_W, BULKHEAD_H - SLOT_Z_HI), pos=(X_BULKHEAD + HOOD_DEPTH / 2.0 - cx, 0.0, fz + (BULKHEAD_H + SLOT_Z_HI) / 2.0), color=CHASSIS_GREY),
    ])
    for s in (1.0, -1.0):
        # Rear slot floor shelf (x = 0.4760..0.4830 m, h = 68.0..71.0 mm)
        shapes.append(box((HOOD_DEPTH - WALL_T, SLOT_W + 0.006, WALL_T), pos=(X_BULKHEAD + (WALL_T + HOOD_DEPTH) / 2.0 - cx, s * SLOT_Y, fz + SLOT_Z_LO - WALL_T / 2.0), color=CHASSIS_GREY))
        # High-contrast white slot border trim on the -x bulkhead face and top alignment tick
        shapes.append(box((0.0006, SLOT_W + 0.006, 0.0022), pos=(X_BULKHEAD - cx - 0.0003, s * SLOT_Y, fz + SLOT_Z_HI + 0.0011), color=SLOT_TRIM, solid=False))
        shapes.append(box((0.0006, SLOT_W + 0.006, 0.0022), pos=(X_BULKHEAD - cx - 0.0003, s * SLOT_Y, fz + SLOT_Z_LO - 0.0011), color=SLOT_TRIM, solid=False))
        shapes.append(box((HOOD_DEPTH, 0.018, 0.0006), pos=(X_BULKHEAD + HOOD_DEPTH / 2.0 - cx, s * SLOT_Y, fz + BULKHEAD_H + 0.0003), color=MARK_YELLOW, solid=False))

    # Two Dynatron S2 LGA4677 heatsinks + four 4-module DDR5 RDIMM banks
    hx = X_SEAT - cx
    for s in (1.0, -1.0):
        shapes.append(box((HS_L - 0.010, HS_W - 0.008, SOCKET_H), pos=(hx, s * SLOT_Y, fz + SOCKET_H / 2.0), color=COPPER))
        shapes.append(box((HS_L, HS_W, HS_H), pos=(hx, s * SLOT_Y, fz + SOCKET_H + HS_H / 2.0), color=ALU_SILVER))
        for fy in np.linspace(-HS_W / 2.0 + 0.007, HS_W / 2.0 - 0.007, 7):
            shapes.append(box((HS_L - 0.002, 0.0022, 0.0006), pos=(hx, s * SLOT_Y + float(fy), fz + SOCKET_H + HS_H + 0.0003), color=FIN_DARK, solid=False))
        for bank_ys in (DIMM_INNER_YS, DIMM_OUTER_YS):
            b_lo, b_hi = bank_ys[0] - 0.003, bank_ys[-1] + 0.003
            shapes.append(box((DIMM_L + 0.006, b_hi - b_lo, DIMM_RAIL_H), pos=(hx, s * (b_lo + b_hi) / 2.0, fz + DIMM_RAIL_H / 2.0), color=SOCKET_WHITE))
            for dy in bank_ys:
                shapes.append(box((DIMM_L, DIMM_T, DIMM_H), pos=(hx, s * dy, fz + DIMM_RAIL_H + DIMM_H / 2.0), color=DIMM_GREEN))

    # Five Arctic S8038-7K 2U server fans (80 x 80 x 38 mm) behind the bulkhead
    fx = X_FAN_CENTER - cx
    for fy in FAN_YS:
        shapes.append(box((FAN_D, FAN_W, FAN_W), pos=(fx, fy, fz + 0.002 + FAN_W / 2.0), color=FAN_BLACK))
        shapes.append(cylinder(0.022, FAN_D + 0.001, pos=(fx, fy, fz + 0.002 + FAN_W / 2.0), axis="x", color=ALU_SILVER, solid=False))

    return (cx, 0.0, top_z), shapes
