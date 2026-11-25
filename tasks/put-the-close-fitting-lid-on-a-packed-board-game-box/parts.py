"""Product dimensions, presets, tolerances, and rigid-body shapes for the two-piece game-box lid task.

Every product size, wall thickness, per-side skirt clearance, chamfer, mass, surface friction, wedging threshold,
layout coordinate, and scoring tolerance lives in this single module so that caliper and scale readings taken later on
the bought box drop in at one place without rewriting the task, checks, trials, demo, or unit tests.

Frames:
- ``box_base`` frame B: origin at the center of its outer underside (``z = 0`` on the bench, ``z = BASE_H`` at its top
  rim); ``x`` across the short side ``BASE_W``, ``y`` along the long side ``BASE_L``.
- ``box_lid`` frame L: origin at the center of its open skirt's bottom plane (``z = 0`` at the skirt lip, ``z =
  LID_IN_H`` at the roof's inner face, ``z = LID_OUT_H`` at the roof's top outer face); ``x`` across ``LID_OUT_W``,
  ``y`` along ``LID_OUT_L``.
"""

from __future__ import annotations

from dataclasses import dataclass
import json
import math
import os

import numpy as np
from mechkit import box, cylinder, matrix_to_quat, sphere


@dataclass(frozen=True)
class BoxSpec:
    """Figures (maker, listing or estimate) of a commercial two-piece rigid game box (SI: m, kg)."""

    key: str
    label: str
    base_l: float  # outer length of box_base (along y at start)
    base_w: float  # outer width of box_base (along x at start)
    base_h: float  # outer height of box_base
    board_t: float  # wrapped greyboard wall/roof/floor thickness
    clear_side: float  # per-side skirt-to-base horizontal play: (lid_in - base_out) / 2
    lid_in_h: float  # inner skirt depth of box_lid
    chamfer_in: float  # inner bottom rim lead-in flare (wrapped paper edge)
    chamfer_h: float  # axial height of the bottom rim lead-in
    base_mass: float  # empty box_base mass
    contents_mass: float  # combined mass of tile_stack + card_deck + wood_pawn
    lid_mass: float  # box_lid mass
    bench_friction: float  # static/dynamic friction of box underside on bench
    board_friction: float  # static/dynamic friction of wrapped greyboard walls
    lid_at: tuple[float, float]  # start (x, y) of box_lid on bench
    base_at: tuple[float, float]  # start (x, y) of box_base on bench
    closed_h_override: float | None = None  # optional closed height on a flat table (a later caliper reading; none set)
    source_kind: str = "estimate"
    source_note: str = "estimate: figures of a commercial two-piece game-box class, derived from its retail size"

    @property
    def lid_in_l(self) -> float:
        return round(self.base_l + 2.0 * self.clear_side, 6)

    @property
    def lid_in_w(self) -> float:
        return round(self.base_w + 2.0 * self.clear_side, 6)

    @property
    def lid_out_l(self) -> float:
        return round(self.lid_in_l + 2.0 * self.board_t, 6)

    @property
    def lid_out_w(self) -> float:
        return round(self.lid_in_w + 2.0 * self.board_t, 6)

    @property
    def lid_out_h(self) -> float:
        return round(self.lid_in_h + self.board_t, 6)

    @property
    def closed_h(self) -> float:
        if self.closed_h_override is not None:
            return self.closed_h_override
        return round(max(self.base_h + self.board_t, self.lid_out_h), 6)

    @property
    def packed_base_mass(self) -> float:
        return round(self.base_mass + self.contents_mass, 6)


# --- commercial Ravensburger two-piece rigid game-box presets ---
# Default `ravensburger_20847` is a real, named product. Its closed-box outside size is the maker's own figure, from
# ravensburger.de/de-DE/produkte/spiele/kinderspiele/junior-labyrinth-20847 ("Junior Labyrinth", Artikelnummer 20847,
# EAN 4005556208470): "Schachtelgröße: 5,50 x 33,50 x 23,10 cm". The same page's "Abmessung: 31,00 x 31,00 x 0,30 cm"
# is, by its size, the flat board, not the box. Wall thickness, per-side skirt clearance, and masses are estimates
# derived from the box size (see FIGURES and README).
BOX_PRESETS: dict[str, BoxSpec] = {
    "ravensburger_20847": BoxSpec(
        key="ravensburger_20847",
        label="Ravensburger Junior Labyrinth, article 20847 (maker's box size 33.5 x 23.1 x 5.5 cm)",
        base_l=0.3284,
        base_w=0.2244,
        base_h=0.0520,
        board_t=0.0018,
        clear_side=0.0015,
        lid_in_h=0.0532,
        chamfer_in=0.0008,
        chamfer_h=0.0030,
        base_mass=0.175,
        contents_mass=0.320,
        lid_mass=0.185,
        bench_friction=0.38,
        board_friction=0.42,
        lid_at=(0.185, 0.000),
        base_at=(0.435, 0.000),
        source_kind="maker",
        source_note=(
            "lid outside 33.5 x 23.1 x 5.5 cm = closed box: maker's page ravensburger.de/de-DE, article 20847"
            " ('Schachtelgröße: 5,50 x 33,50 x 23,10 cm'); base, walls, clearance, masses: estimates derived from it"
        ),
    ),
    "ravensburger_275x192": BoxSpec(
        key="ravensburger_275x192",
        label="Ravensburger two-piece rigid game box (27.5 x 19.2 x 5.5 cm compact family box class)",
        base_l=0.2710,
        base_w=0.1880,
        base_h=0.0520,
        board_t=0.0018,
        clear_side=0.0015,
        lid_in_h=0.0520,
        chamfer_in=0.0008,
        chamfer_h=0.0030,
        base_mass=0.095,
        contents_mass=0.190,
        lid_mass=0.098,
        bench_friction=0.38,
        board_friction=0.42,
        lid_at=(0.195, 0.000),
        base_at=(0.420, 0.000),
        source_kind="estimate",
        source_note="27.5 x 19.2 x 5.5 cm retail two-piece game-box class (alternate preset, no single named product)",
    ),
    "ravensburger_183x115": BoxSpec(
        key="ravensburger_183x115",
        label="Ravensburger two-piece rigid game box (18.3 x 11.5 x 3.7 cm travel game-box class)",
        base_l=0.1776,
        base_w=0.1096,
        base_h=0.0355,
        board_t=0.0015,
        clear_side=0.0012,
        lid_in_h=0.0355,
        chamfer_in=0.0007,
        chamfer_h=0.0025,
        base_mass=0.055,
        contents_mass=0.110,
        lid_mass=0.058,
        bench_friction=0.38,
        board_friction=0.42,
        lid_at=(0.195, 0.000),
        base_at=(0.420, 0.000),
        source_kind="estimate",
        source_note="18.3 x 11.5 x 3.7 cm retail travel game-box class (alternate preset, no single named product)",
    ),
}


def resolve_preset(name: str | None = None) -> BoxSpec:
    raw = os.environ.get("GAME_BOX_PRESET", "ravensburger_20847") if name is None else name
    if raw not in BOX_PRESETS:
        raise ValueError(f"unknown GAME_BOX_PRESET {raw!r}; available: {sorted(BOX_PRESETS)}")
    return BOX_PRESETS[raw]


def critical_wedge_angle_deg(spec: BoxSpec | None = None) -> float:
    """Angle (deg) at which a lid tilted across its long axis uses up all 2*clear_side play over its full skirt depth:
    ``lid_in_l * cos(theta) - lid_in_h * sin(theta) = base_l``."""
    s = ACTIVE_BOX if spec is None else spec
    lo, hi = 0.0, 25.0
    for _ in range(60):
        mid = 0.5 * (lo + hi)
        th = math.radians(mid)
        span = s.lid_in_l * math.cos(th) - s.lid_in_h * math.sin(th)
        if span > s.base_l:
            lo = mid
        else:
            hi = mid
    return round(0.5 * (lo + hi), 2)


def wedge_stop_depth_m(tilt_deg: float, spec: BoxSpec | None = None, axis: str = "long") -> float:
    """Maximum axial engagement depth (m) of the higher skirt side before two-point diagonal lock at ``tilt_deg``."""
    s = ACTIVE_BOX if spec is None else spec
    lid_in = s.lid_in_l if axis == "long" else s.lid_in_w
    base_out = s.base_l if axis == "long" else s.base_w
    th = abs(math.radians(tilt_deg))
    if th < 1e-9:
        return s.lid_in_h
    avail = lid_in * math.cos(th) - base_out
    if avail <= 0.0:
        return 0.0
    return min(s.lid_in_h, avail / math.sin(th))


BOX_PRESET = resolve_preset().key
ACTIVE_BOX = resolve_preset(BOX_PRESET)

# --- active constants imported by task.py, demo.py, checks.py, trials.py, and tests ---
BASE_L = ACTIVE_BOX.base_l
BASE_W = ACTIVE_BOX.base_w
BASE_H = ACTIVE_BOX.base_h
BOARD_T = ACTIVE_BOX.board_t
CLEAR_SIDE = ACTIVE_BOX.clear_side
LID_IN_L = ACTIVE_BOX.lid_in_l
LID_IN_W = ACTIVE_BOX.lid_in_w
LID_IN_H = ACTIVE_BOX.lid_in_h
LID_OUT_L = ACTIVE_BOX.lid_out_l
LID_OUT_W = ACTIVE_BOX.lid_out_w
LID_OUT_H = ACTIVE_BOX.lid_out_h
CLOSED_H = ACTIVE_BOX.closed_h
CHAMFER_IN = ACTIVE_BOX.chamfer_in
CHAMFER_H = ACTIVE_BOX.chamfer_h

BASE_MASS = ACTIVE_BOX.base_mass
CONTENTS_MASS = ACTIVE_BOX.contents_mass
LID_MASS = ACTIVE_BOX.lid_mass
PACKED_BASE_MASS = ACTIVE_BOX.packed_base_mass
TILE_STACK_MASS = round(0.50 * CONTENTS_MASS, 6)  # contents split: tile stack 50 %
CARD_DECK_MASS = round(0.395 * CONTENTS_MASS, 6)  # token/chip deck 39.5 %
WOOD_PAWN_MASS = round(CONTENTS_MASS - TILE_STACK_MASS - CARD_DECK_MASS, 6)  # playing piece 10.5 %

BENCH_FRICTION = ACTIVE_BOX.bench_friction
BOARD_FRICTION = ACTIVE_BOX.board_friction
WEDGE_TILT_DEG = critical_wedge_angle_deg(ACTIVE_BOX)  # ~2.8 deg across 331.4 mm / 53.2 mm skirt at 1.5 mm/side play

LID_AT = ACTIVE_BOX.lid_at
BASE_AT = ACTIVE_BOX.base_at
START_GAP_X = round((BASE_AT[0] - 0.5 * BASE_W) - (LID_AT[0] + 0.5 * LID_OUT_W), 6)  # 33.7 mm clear gap on default

# Contact offsets: 0.4 mm on box_base and box_lid so the 1.5 mm/side clearance is never eaten by the kit's 2.0 mm default
BOX_CONTACT_OFFSET = 0.0004
CONTENT_CONTACT_OFFSET = 0.0005
OVER = 0.0005  # m: loose bodies start 0.5 mm above what supports them so frame 0 renders settled without interpenetration
GRAVITY = 9.81

# Corner tape L-marks framing box_base on the bench (drawn only, solid=False)
MARK_CLEAR = 0.0045  # m outside box_base's outer walls (1.2 mm outside seated box_lid so marks stay visible when closed)
MARK_LEN = 0.030  # m arm length of each yellow L-mark
MARK_W = 0.008  # m tape strip width
MARK_T = 0.0004  # m visual tape thickness

# Colours (sRGB 0..1, matte printed cardboard and wooden/card components; no logos or text)
BASE_COLOR = (0.86, 0.84, 0.79)  # warm greyboard inner box
LID_CREAM = (0.93, 0.94, 0.96)  # printed matte white-cream game-box lid
LID_NAVY = (0.09, 0.27, 0.64)  # blue border band and corner geometric triangle accent on top roof
LID_GOLD = (0.90, 0.68, 0.14)  # thin decorative accent stripe on top roof
TILE_COLOR = (0.82, 0.36, 0.16)  # terracotta maze-tile stack
CARD_COLOR = (0.16, 0.58, 0.34)  # green card deck
PAWN_COLOR = (0.88, 0.66, 0.12)  # yellow-gold turned wooden playing pawn
TAPE_YELLOW = (0.94, 0.80, 0.14)  # matte yellow bench corner registration tape

# --- scoring & timing constants (shared verbatim across task.py, task.json, README.md, and tests) ---
RUN_LIMIT_STEPS = 1200  # 60 s at 20 Hz
CONTROL_HZ = 20.0
RUN_LIMIT_S = RUN_LIMIT_STEPS / CONTROL_HZ
SUCCESS_HOLD_S = 3.5  # s: once score == 1.0 holds, keep stepping 3.5 s so both arms park cleanly at READY (Check 48)
FULL_HOLD_S = 1.0  # s: seated + clear + at-rest condition must hold continuously for 1.0 s (20 steps) to score 1.0

BASE_TILT_MAX_DEG = 15.0  # deg: whole-trial zero if box_base ever tilts more than 15.0 deg from horizontal
BASE_SHIFT_MAX = 0.025  # m: whole-trial zero if any corner of box_base shifts > 25.0 mm horizontally from its start mark

ENGAGE_HOLD_S = 1.0  # s: the on-box rest must persist for 1.0 s (20 steps) before the 0.5 level is awarded
ON_BOX_TILT_MAX_DEG = 15.0  # deg: 0.5 level (lid rests on the box, not seated) allows tilt up to 15.0 deg
ON_BOX_MIN_CONTACT_N = 0.5  # N: 0.5 level requires box_lid resting on box_base with at least this contact force
STRADDLE_SOLVER_TOL = 0.0003  # m: 0.3 mm contact-solver allowance (<< 1.8 mm wall) when checking 4-wall straddle

SEAT_CORNER_TOL = 0.001  # m: 1.0 level requires EVERY lid roof corner within 1.0 mm of CLOSED_H (no offset test)
SEAT_TILT_MAX_DEG = 2.0  # deg: 1.0 level requires lid tilt <= 2.0 deg from box_base

GRIPPER_CLEAR_MIN = 0.030  # m: both grippers' grasp points >= 30.0 mm from box_lid and box_base outer surfaces
REST_LIN_MAX = 0.010  # m/s: all free bodies < 10.0 mm/s linear speed
REST_ANG_MAX = 0.10  # rad/s: all free bodies < 0.10 rad/s angular speed

# Reference grasp parameters (scaled from active preset geometry)
PADDLE_OPENING = 0.020  # m: each gripper closed to a 20.0 mm 2-finger flat paddle
APPROACH_PITCH_DEG = 32.0  # deg below horizontal for inward outer-wall squeeze
SQUEEZE_FORCE_N = 10.0  # N inward squeeze per arm along +-y (~15x lid weight with rubber-on-board friction)
CARRY_CLEAR_Z = 0.072  # m: lid skirt bottom lifted to +72.0 mm above bench (> 52.0 mm base rim + 20 mm clearance)


# --- key-figures table metadata (Check 50 & Reader Addendum Section 1: true source per figure) ---
_MAKER_SIZE = (
    "maker's page ravensburger.de/de-DE/produkte/spiele/kinderspiele/junior-labyrinth-20847, article 20847: "
    "'Schachtelgröße: 5,50 x 33,50 x 23,10 cm'"
)
_IS_MAKER = ACTIVE_BOX.source_kind == "maker"
_SIZES = "maker" if _IS_MAKER else "estimate"
FIGURES: tuple[dict[str, str], ...] = (
    {
        "figure": "Closed-box outside size = lid outer",
        "constants": "LID_OUT_L, LID_OUT_W, LID_OUT_H",
        "value": f"{1000 * LID_OUT_L:.1f} x {1000 * LID_OUT_W:.1f} x {1000 * LID_OUT_H:.1f} mm",
        "kind": _SIZES,
        "source": _MAKER_SIZE if _IS_MAKER else "retail box-class size (no single named product)",
    },
    {
        "figure": "Base outer length, width, height, and wall thickness",
        "constants": "BASE_L, BASE_W, BASE_H, BOARD_T",
        "value": f"{1000 * BASE_L:.1f} x {1000 * BASE_W:.1f} x {1000 * BASE_H:.1f} mm, wall {1000 * BOARD_T:.1f} mm",
        "kind": "estimate",
        "source": (
            "estimate: base outer = maker's size - 2 x 1.8 mm walls - 2 x 1.5 mm/side play; 1.8 mm wall between a"
            " game-box printer's '1.5mm' standard board and the '2mm' it advises for boxes over 8.5 x 8.5 in"
            " (printninja.com/two-piece-board-game-box)"
        ),
    },
    {
        "figure": "Lid inner length, width, skirt depth, and per-side clearance",
        "constants": "LID_IN_L, LID_IN_W, LID_IN_H, CLEAR_SIDE",
        "value": f"{1000 * LID_IN_L:.1f} x {1000 * LID_IN_W:.1f} x {1000 * LID_IN_H:.1f} mm ({1000 * CLEAR_SIDE:.2f} mm/side)",
        "kind": "estimate",
        "source": (
            "estimate: lid inner = maker's size - 2 x 1.8 mm walls; skirt = 55.0 - 1.8 mm roof; 1.5 mm/side play set so"
            " a level lid falls home and one tilted past about 3 deg catches; box makers give '0.5-1mm'"
            " (brandedboxes.us/box-styles/two-piece-boxes) and for game boxes '1.5 to 2 millimetres'"
            " (vivopak.com.au/custom-boxes/?p=347403) lid inside to base outside, per side or total not stated;"
            " first-minutes step 1 calipers it"
        ),
    },
    {
        "figure": "Lid bottom inner wrapped-edge chamfer (flare over height)",
        "constants": "CHAMFER_IN, CHAMFER_H",
        "value": f"{1000 * CHAMFER_IN:.1f} mm over {1000 * CHAMFER_H:.1f} mm",
        "kind": "estimate",
        "source": "estimate: folded-paper lead-in at the skirt lip, typical of wrapped greyboard lids",
    },
    {
        "figure": "Closed total box height on a flat table",
        "constants": "CLOSED_H",
        "value": f"{1000 * CLOSED_H:.1f} mm",
        "kind": _SIZES,
        "source": _MAKER_SIZE if _IS_MAKER else "retail box-class height (no single named product)",
    },
    {
        "figure": "Empty base mass, packed contents mass, and lid mass",
        "constants": "BASE_MASS, CONTENTS_MASS, LID_MASS",
        "value": f"{1000 * BASE_MASS:.0f} g, {1000 * CONTENTS_MASS:.0f} g, {1000 * LID_MASS:.0f} g",
        "kind": "estimate",
        "source": (
            "estimate: greyboard shell volume (wall/roof area x 1.8 mm) x 0.75 g/cm3 chipboard; contents from the"
            " maker's list '1 Spielplan, 4 Spielfiguren, 17 Gänge-Karten, 12 Geheimnis-Chips', board taken as its"
            " 'Abmessung: 31,00 x 31,00 x 0,30 cm'"
        ),
    },
    {
        "figure": "Packed-base-on-bench and board-on-board friction coefficients",
        "constants": "BENCH_FRICTION, BOARD_FRICTION",
        "value": f"{BENCH_FRICTION:.2f}, {BOARD_FRICTION:.2f}",
        "kind": "estimate",
        "source": "estimate: printed-chipboard slip angles of about 21 deg on a laminate bench and 23 deg on itself",
    },
    {
        "figure": "Critical wedging tilt angle across long skirt",
        "constants": "WEDGE_TILT_DEG",
        "value": f"{WEDGE_TILT_DEG:.2f} deg",
        "kind": "derived",
        "source": (
            "derived: lid_in_l cos(t) - lid_in_h sin(t) = base_l from the sizes above; 1.02 / 1.95 / 2.80 / 4.36 /"
            " 5.07 deg at 0.5 / 1.0 / 1.5 / 2.5 / 3.0 mm a side (maker's outside held)"
        ),
    },
)


# --- shape builders -------------------------------------------------------------------------------------------


def base_shapes(spec: BoxSpec | None = None) -> list:
    """5 non-overlapping rigid wall/floor boxes of ``box_base`` in frame B (origin at bottom center)."""
    s = ACTIVE_BOX if spec is None else spec
    w, length, h, t = s.base_w, s.base_l, s.base_h, s.board_t
    wall_h = h - t
    wall_z = t + 0.5 * wall_h
    wall_fric = (s.board_friction, s.board_friction)
    return [
        # Floor: min-combined with the bench (0.5) so bench slip friction equals BENCH_FRICTION
        box((w, length, t), pos=(0.0, 0.0, 0.5 * t), color=BASE_COLOR, friction=(s.bench_friction, s.bench_friction), friction_combine="min"),
        # Front (-x) and rear (+x) walls spanning full BASE_L along y
        box((t, length, wall_h), pos=(-0.5 * (w - t), 0.0, wall_z), color=BASE_COLOR, friction=wall_fric),
        box((t, length, wall_h), pos=(+0.5 * (w - t), 0.0, wall_z), color=BASE_COLOR, friction=wall_fric),
        # Right (-y) and left (+y) short walls fitting cleanly between front and rear walls
        box((w - 2.0 * t, t, wall_h), pos=(0.0, -0.5 * (length - t), wall_z), color=BASE_COLOR, friction=wall_fric),
        box((w - 2.0 * t, t, wall_h), pos=(0.0, +0.5 * (length - t), wall_z), color=BASE_COLOR, friction=wall_fric),
    ]


def _chamfer_wall_shapes(
    axis: str,
    sign: float,
    inner_half: float,
    span: float,
    board_t: float,
    lid_in_h: float,
    chamfer_in: float,
    chamfer_h: float,
    wall_fric: tuple[float, float],
    bench_fric: tuple[float, float],
    color: tuple[float, float, float],
) -> list:
    """One skirt wall of ``box_lid``: main vertical wall [chamfer_h, lid_in_h], vertical outer lip [0, chamfer_h], and
    tilted inner wrapped-edge bevel ramping smoothly from (inner_half + chamfer_in, 0) to (inner_half, chamfer_h).
    Every corner of the tilted bevel lies strictly inside [inner_half, inner_half + board_t] x [0, lid_in_h]."""
    h_main = lid_in_h - chamfer_h
    z_main = chamfer_h + 0.5 * h_main
    lip_t = board_t - chamfer_in
    ramp_len = math.hypot(chamfer_in, chamfer_h)
    bevel_t = min(chamfer_in, 0.85 * lip_t * ramp_len / max(chamfer_h, 1e-6))
    # In (u_outward, z) coordinates: ramp runs from P0=(inner_half + chamfer_in, 0) to P1=(inner_half, chamfer_h).
    # Unit tangent along ramp: (-chamfer_in, +chamfer_h)/ramp_len; outward-into-wall normal: (+chamfer_h, +chamfer_in)/ramp_len.
    u_mid = inner_half + 0.5 * chamfer_in + 0.5 * bevel_t * (chamfer_h / ramp_len)
    z_mid = 0.5 * chamfer_h + 0.5 * bevel_t * (chamfer_in / ramp_len)
    span_bevel = max(0.004, span - 0.002)

    if axis == "x":
        main_pos = (sign * (inner_half + 0.5 * board_t), 0.0, z_main)
        main_sz = (board_t, span, h_main)
        lip_pos = (sign * (inner_half + chamfer_in + 0.5 * lip_t), 0.0, 0.5 * chamfer_h)
        lip_sz = (lip_t, span, chamfer_h)
        # Local box axes for bevel: x_b = outward normal (sign*chamfer_h, 0, chamfer_in)/ramp_len, y_b = (0, 1, 0), z_b = x_b x y_b
        nx, nz = sign * chamfer_h / ramp_len, chamfer_in / ramp_len
        xb = np.array([nx, 0.0, nz])
        yb = np.array([0.0, 1.0, 0.0])
        zb = np.cross(xb, yb)
        rot = np.column_stack([xb, yb, zb])
        bevel_pos = (sign * u_mid, 0.0, z_mid)
        bevel_sz = (bevel_t, span_bevel, ramp_len)
    else:
        main_pos = (0.0, sign * (inner_half + 0.5 * board_t), z_main)
        main_sz = (span, board_t, h_main)
        lip_pos = (0.0, sign * (inner_half + chamfer_in + 0.5 * lip_t), 0.5 * chamfer_h)
        lip_sz = (span, lip_t, chamfer_h)
        ny, nz = sign * chamfer_h / ramp_len, chamfer_in / ramp_len
        yb = np.array([0.0, ny, nz])
        xb = np.array([1.0, 0.0, 0.0])
        zb = np.cross(xb, yb)
        rot = np.column_stack([xb, yb, zb])
        bevel_pos = (0.0, sign * u_mid, z_mid)
        bevel_sz = (span_bevel, bevel_t, ramp_len)

    return [
        box(main_sz, pos=main_pos, color=color, friction=wall_fric),
        box(lip_sz, pos=lip_pos, color=color, friction=bench_fric, friction_combine="min"),
        box(bevel_sz, pos=bevel_pos, quat=matrix_to_quat(rot), color=color, friction=wall_fric),
    ]


def lid_shapes(spec: BoxSpec | None = None) -> list:
    """5-wall hollow ``box_lid`` in frame L (origin at bottom center of open skirt), with bottom wrapped-edge chamfer
    and non-solid geometric top-roof colour bands (no brand text or logos)."""
    s = ACTIVE_BOX if spec is None else spec
    in_w, in_l, in_h, t = s.lid_in_w, s.lid_in_l, s.lid_in_h, s.board_t
    out_w, out_l = s.lid_out_w, s.lid_out_l
    wall_fric = (s.board_friction, s.board_friction)
    bench_fric = (s.bench_friction, s.bench_friction)

    shapes = [
        # Solid top roof [in_h, in_h + t]
        box((out_w, out_l, t), pos=(0.0, 0.0, in_h + 0.5 * t), color=LID_CREAM, friction=wall_fric),
    ]
    # Front (-x) and rear (+x) skirt walls spanning full out_l along y
    for sgn in (-1.0, +1.0):
        shapes.extend(
            _chamfer_wall_shapes("x", sgn, 0.5 * in_w, out_l, t, in_h, s.chamfer_in, s.chamfer_h, wall_fric, bench_fric, LID_CREAM)
        )
    # Right (-y) and left (+y) short skirt walls spanning in_w along x (outer faces at +-0.5*out_l where grippers press)
    for sgn in (-1.0, +1.0):
        shapes.extend(
            _chamfer_wall_shapes("y", sgn, 0.5 * in_l, in_w, t, in_h, s.chamfer_in, s.chamfer_h, wall_fric, bench_fric, LID_CREAM)
        )

    # Non-solid printed roof graphics (0.4 mm proud on top outer face: makes top-up vs upside-down unmistakable)
    z_art = in_h + t + 0.0002
    band = 0.012
    shapes.extend(
        [
            box((band, out_l - 0.012, 0.0004), pos=(-0.5 * out_w + 0.5 * band + 0.006, 0.0, z_art), color=LID_NAVY, solid=False),
            box((band, out_l - 0.012, 0.0004), pos=(+0.5 * out_w - 0.5 * band - 0.006, 0.0, z_art), color=LID_NAVY, solid=False),
            box((out_w - 0.012, band, 0.0004), pos=(0.0, -0.5 * out_l + 0.5 * band + 0.006, z_art), color=LID_NAVY, solid=False),
            box((out_w - 0.012, band, 0.0004), pos=(0.0, +0.5 * out_l - 0.5 * band - 0.006, z_art), color=LID_NAVY, solid=False),
            # Corner geometric chevron/triangle accent on front-right roof corner + centre gold stripe
            box((0.044, 0.044, 0.0005), pos=(-0.5 * out_w + 0.036, -0.5 * out_l + 0.036, z_art + 0.0001), yaw=math.pi / 4, color=LID_NAVY, solid=False),
            box((0.036, 0.110, 0.0004), pos=(0.0, 0.0, z_art), color=LID_GOLD, solid=False),
        ]
    )
    return shapes


def content_specs(spec: BoxSpec | None = None) -> dict[str, dict]:
    """Local placement (in B) and dimensions of the 3 loose game components inside ``box_base`` (< BASE_H - 12 mm)."""
    s = ACTIVE_BOX if spec is None else spec
    max_h = s.base_h - s.board_t - 0.014  # >= 14 mm below base top rim
    scale_xy = min(1.0, (s.base_w - 2 * s.board_t) / 0.160, (s.base_l - 2 * s.board_t) / 0.240)
    tile_h = min(0.024, 0.68 * max_h)
    card_h = min(0.018, 0.52 * max_h)
    pawn_h = min(0.034, 0.94 * max_h)
    tile_w = round(0.070 * scale_xy, 4)
    card_x, card_y = round(0.058 * scale_xy, 4), round(0.086 * scale_xy, 4)
    pawn_r = round(0.012 * scale_xy, 4)
    return {
        "tile_stack": {
            "size": (tile_w, tile_w, tile_h),
            "local_pos": (0.008 * scale_xy, +0.062 * scale_xy, s.board_t + 0.5 * tile_h + OVER),
            "mass": round(0.50 * s.contents_mass, 6),
            "color": TILE_COLOR,
        },
        "card_deck": {
            "size": (card_x, card_y, card_h),
            "local_pos": (0.008 * scale_xy, -0.056 * scale_xy, s.board_t + 0.5 * card_h + OVER),
            "mass": round(0.395 * s.contents_mass, 6),
            "color": CARD_COLOR,
        },
        "wood_pawn": {
            "radius": pawn_r,
            "height": pawn_h,
            "local_pos": (-0.042 * scale_xy, 0.004 * scale_xy, s.board_t + OVER),
            "mass": round(s.contents_mass - round(0.50 * s.contents_mass, 6) - round(0.395 * s.contents_mass, 6), 6),
            "color": PAWN_COLOR,
        },
    }


def tile_stack_shapes(spec: BoxSpec | None = None) -> list:
    cs = content_specs(spec)["tile_stack"]
    sx, sy, sz = cs["size"]
    return [
        box((sx, sy, sz), pos=(0.0, 0.0, 0.0), color=cs["color"], friction=(BOARD_FRICTION, BOARD_FRICTION)),
        box((0.72 * sx, 0.72 * sy, 0.0004), pos=(0.0, 0.0, 0.5 * sz + 0.0002), color=LID_CREAM, solid=False),
    ]


def card_deck_shapes(spec: BoxSpec | None = None) -> list:
    cs = content_specs(spec)["card_deck"]
    sx, sy, sz = cs["size"]
    return [
        box((sx, sy, sz), pos=(0.0, 0.0, 0.0), color=cs["color"], friction=(BOARD_FRICTION, BOARD_FRICTION)),
        box((0.76 * sx, 0.80 * sy, 0.0004), pos=(0.0, 0.0, 0.5 * sz + 0.0002), color=LID_GOLD, solid=False),
    ]


def wood_pawn_shapes(spec: BoxSpec | None = None) -> list:
    """Turned wooden game pawn in its own frame (origin at its flat base)."""
    cs = content_specs(spec)["wood_pawn"]
    r, h = cs["radius"], cs["height"]
    foot_h = 0.28 * h
    stem_h = 0.46 * h
    head_r = min(0.62 * r, 0.5 * (h - foot_h - stem_h + 0.002))
    return [
        cylinder(r, foot_h, pos=(0.0, 0.0, 0.5 * foot_h), axis="z", color=cs["color"]),
        cylinder(0.58 * r, stem_h, pos=(0.0, 0.0, foot_h + 0.5 * stem_h), axis="z", color=cs["color"]),
        sphere(head_r, pos=(0.0, 0.0, h - head_r), color=cs["color"]),
    ]


def corner_mark_shapes(spec: BoxSpec | None = None) -> list:
    """4 yellow matte tape L-marks (2 strips each, solid=False) framing ``box_base`` on the bench."""
    s = ACTIVE_BOX if spec is None else spec
    hx = 0.5 * s.base_w + MARK_CLEAR
    hy = 0.5 * s.base_l + MARK_CLEAR
    z = 0.5 * MARK_T
    shapes = []
    for sx in (-1.0, +1.0):
        for sy in (-1.0, +1.0):
            # Arm along x from outer corner (sx*(hx + MARK_W), sy*(hy + MARK_W)) inward by MARK_LEN
            shapes.append(
                box(
                    (MARK_LEN, MARK_W, MARK_T),
                    pos=(sx * (hx + MARK_W - 0.5 * MARK_LEN), sy * (hy + 0.5 * MARK_W), z),
                    color=TAPE_YELLOW,
                    solid=False,
                )
            )
            # Arm along y (shortened by MARK_W so the corner square is not drawn twice)
            rem = MARK_LEN - MARK_W
            shapes.append(
                box(
                    (MARK_W, rem, MARK_T),
                    pos=(sx * (hx + 0.5 * MARK_W), sy * (hy + 0.5 * rem), z),
                    color=TAPE_YELLOW,
                    solid=False,
                )
            )
    return shapes


def preset_summary(spec: BoxSpec | None = None) -> dict:
    s = ACTIVE_BOX if spec is None else spec
    cs = content_specs(s)
    pawn_top = cs["wood_pawn"]["local_pos"][2] + cs["wood_pawn"]["height"]
    return {
        "key": s.key,
        "label": s.label,
        "base_outer_mm": [round(1000 * s.base_l, 2), round(1000 * s.base_w, 2), round(1000 * s.base_h, 2)],
        "lid_inner_mm": [round(1000 * s.lid_in_l, 2), round(1000 * s.lid_in_w, 2), round(1000 * s.lid_in_h, 2)],
        "lid_outer_mm": [round(1000 * s.lid_out_l, 2), round(1000 * s.lid_out_w, 2), round(1000 * s.lid_out_h, 2)],
        "board_t_mm": round(1000 * s.board_t, 2),
        "clear_side_mm": round(1000 * s.clear_side, 2),
        "closed_h_mm": round(1000 * s.closed_h, 2),
        "masses_g": {"base": round(1000 * s.base_mass, 1), "contents": round(1000 * s.contents_mass, 1), "lid": round(1000 * s.lid_mass, 1)},
        "contents_rim_clearance_mm": round(1000 * (s.base_h - pawn_top), 2),
        "start_gap_x_mm": round(1000 * ((s.base_at[0] - 0.5 * s.base_w) - (s.lid_at[0] + 0.5 * s.lid_out_w)), 2),
        "critical_wedge_deg": critical_wedge_angle_deg(s),
        "wedge_depth_at_6deg_mm": round(1000 * wedge_stop_depth_m(6.0, s, "long"), 2),
    }


def check_all_presets() -> dict[str, dict]:
    return {k: preset_summary(v) for k, v in BOX_PRESETS.items()}


if __name__ == "__main__":
    print(json.dumps(check_all_presets(), indent=2))
