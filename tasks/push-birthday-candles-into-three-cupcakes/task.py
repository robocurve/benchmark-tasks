"""Three cupcakes stand in a row on a wooden board, near its edge in front of the arms; four birthday candles stand
wick up in a lump of Play-Doh pressed into an egg cup to the right. The job: a candle in each cupcake, pushed
straight down through the icing until it stands by itself, while the light, soft cupcake is kept from sliding or
tipping without squashing it.

How it is built. A cupcake is one rigid body whose every surface gives under a press (a compliant contact, 1 mm
across for each newton of squeeze): squeezed 4 mm past touch it is squashed for good, and a squashed cupcake
(built beforehand and parked out of sight) takes its place. A candle is held by what it stands in (a cupcake, the
Play-Doh) through a hidden chain at its tip (candles.py): a resistance along its length, no wall, and a hold against
leaning that grows with the square of the depth; leaning too far it tears out.
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import candles as cd  # noqa: E402
import mechkit as mk  # noqa: E402
from mechkit import body, box, cylinder, soften, sphere  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
RUN_LIMIT_S = 120.0  # 2,400 steps at 20 Hz: the full run (91.8 s, w7c-f3) plus 30 %
INSTRUCTION = "Put one birthday candle in each of the three cupcakes so that it stands upright."
TOP = TABLE_TOP["station"]
GRAVITY = 9.81

# --- the one material table (README, "Materials"): value, unit, source. "assumed" until the owner's kitchen test ---
MATERIALS = {
    "cupcake_mass": (0.045, "kg", "Hostess CupCakes pack: serving weight"),
    "cupcake_top_d": (0.060, "m", "assumed; kitchen test step 1"),
    "cupcake_base_d": (0.050, "m", "assumed; kitchen test step 1"),
    "cupcake_h": (0.035, "m", "assumed; kitchen test step 1"),
    "cupcake_on_beech": (0.5, "", "assumed; kitchen test step 1"),
    "squeeze_give": (0.001, "m/N", "assumed (across the cupcake at its rim); kitchen test step 3"),
    "squash_at": (0.004, "m", "assumed: closing past touch at which it is squashed for good (4 N); kitchen test step 3"),
    "cupcake_contact": (420.0, "N/m", "calibrated so that 4 mm of closing past touch reads 4.0 N on the gripper (run w7c-g1: 250 N/m gave 2.5 N, 500 gave 4.7 N)"),
    "squashed_lower_wider": (0.006, "m", "assumed"),
    "candle_l": (0.063, "m", "Wilton 2 1/2 in birthday candles"),
    "candle_d": (0.005, "m", "assumed; to be measured on the bought box"),
    "candle_mass": (0.001, "kg", "assumed; to be weighed"),
    "icing_resist": (3.0, "N", "assumed: until the tip is 2 mm in; kitchen test step 2"),
    "icing_depth": (0.002, "m", "assumed"),
    "cake_resist": (1.0, "N", "assumed: along the candle in the cake; kitchen test step 2"),
    "cake_hold_at_8mm": (0.0002, "N m/rad", "assumed (HOLD_K_8MM): an 8 mm candle falls by itself; kitchen test step 2"),
    "cake_hold_at_15mm": (0.002, "N m/rad", "assumed (HOLD_K_15MM); a power law through both (exponent 3.66); kitchen test step 2"),
    "cake_tear_deg": (25.0, "deg", "assumed; kitchen test step 2"),
    "candle_snap": (0.02, "N m", "assumed: bending moment; kitchen test step 2 (bent over a table edge)"),
    "doh_resist": (0.3, "N", "assumed: pull-out from the Play-Doh; bench test"),
    "doh_hold_at_8mm": (0.0003, "N m/rad", "assumed: with 0.003 at 15 mm a brushed candle leans and stays"),
    "doh_hold_at_15mm": (0.003, "N m/rad", "assumed"),
    "doh_tear_deg": (35.0, "deg", "assumed"),
    "candle_in_doh": (0.015, "m", "the designer's set-up: 15 mm deep"),
    "board_mass": (0.63, "kg", "beech 300 x 200 x 15 mm at 700 kg/m3"),
    "eggcup_mass": (0.126, "kg", "assumed: porcelain egg cup 70 g, Play-Doh 2 oz can 56 g"),
    "pad_on_icing": (0.6, "", "assumed: nitrile finger cot on chocolate icing"),
    "pad_on_wax": (0.5, "", "assumed: nitrile finger cot on candle wax"),
}
M = {k: v[0] for k, v in MATERIALS.items()}
HOLD_K_8MM, HOLD_K_15MM = M["cake_hold_at_8mm"], M["cake_hold_at_15mm"]  # the designer's two constants (2026-11-20 11:35)

# --- the board and the row of cupcakes (bench coordinates, m; x away from the arms, y left) ---
BOARD_SIZE = (0.200, 0.300, 0.015)  # beech, loose
ROW_X, ROW_FROM_EDGE, SPACING = 0.260, 0.035, 0.090
ROW_Y = -0.030  # the row's middle: 3 cm right of the midline, so that the right hand reaches the left cupcake pointing down
BOARD_C = (ROW_X - ROW_FROM_EDGE + BOARD_SIZE[0] / 2, ROW_Y)
BOARD_TOP = TOP + BOARD_SIZE[2]
CAKE_YS = (ROW_Y + SPACING, ROW_Y, ROW_Y - SPACING)
CAKES = ("cake0", "cake1", "cake2")
CAKE_H, R_TOP, R_BASE = M["cupcake_h"], M["cupcake_top_d"] / 2, M["cupcake_base_d"] / 2
CAP_Z0 = 0.029  # the icing cap's underside over the cupcake's base: the cap is its widest part
LAYERS = 4  # the cake under the cap, as cylinders whose radius is the cone's at each one's top
SOFT = (M["cupcake_contact"], 10.0)  # N/m per contact, N s/m: the cupcake's give (the table's 4 N at 4 mm of closing)
# CUPCAKE_GAUGE="k0,k1,k2" (calibration only): each cupcake's sides with that stiffness (N/m), and none is ever squashed
GAUGE = tuple(float(v) for v in os.environ.get("CUPCAKE_GAUGE", "").split(",") if v.strip())
CHOC, ICING, SQUIGGLE = (0.30, 0.16, 0.08), (0.11, 0.06, 0.035), (0.96, 0.96, 0.94)
BEECH, PORCELAIN, DOH = (0.82, 0.68, 0.50), (0.10, 0.32, 0.22), (0.98, 0.80, 0.10)  # beech; a dark green egg cup; yellow Play-Doh
CANDLE_COLORS = ((0.12, 0.30, 0.85), (0.85, 0.10, 0.12), (0.12, 0.30, 0.85), (0.85, 0.10, 0.12))  # the box's dark ones: blue, red

# --- the egg cup with the Play-Doh and the four candles ---
CUP_XY = (0.420, -0.250)  # in the right wrist camera's picture at the ready pose (it looks 30 degrees forward of down); the
# right hand pointing down reaches 2.5 cm over the candle tops here (at x 0.44 only to the tops: run w7c-f1 swept two out)
FOOT_R, FOOT_H, STEM_R, STEM_H, CUP_R, CUP_H = 0.022, 0.006, 0.009, 0.014, 0.0255, 0.038
DOH_TOP = FOOT_H + STEM_H + CUP_H + 0.002  # the lump's top over the bench: a 2 oz can fills the cup and domes 2 mm
CANDLE_SQUARE = 0.022
CANDLES = ("candle0", "candle1", "candle2", "candle3")

# --- where spoiled things wait: a shelf out of sight under the bench ---
SHELF_Z = TOP - 0.40


def cake_layers() -> list[tuple[float, float, float]]:
    """(z0, z1, radius) of the cake under the cap: the cone's radius at each layer's top."""
    out = []
    for i in range(LAYERS):
        z0, z1 = CAP_Z0 * i / LAYERS, CAP_Z0 * (i + 1) / LAYERS
        out.append((z0, z1, R_BASE + (R_TOP - R_BASE) * z1 / CAKE_H))
    return out


def squiggle(z: float, n_loops: int = 7, length: float = 0.044, loop_r: float = 0.003, step: float = 0.6) -> list:
    """The white loop of icing along the top's middle: drawn only."""
    a = length / (2 * math.pi * n_loops)
    out = []
    t = 0.0
    while t <= 2 * math.pi * n_loops:
        x, y = a * t - loop_r * math.sin(t) - length / 2, loop_r * math.cos(t)
        out.append(sphere(0.0009, pos=(x, y, z), color=SQUIGGLE, solid=False))
        t += step
    return out


def cupcake_shapes(lower: float = 0.0, wider: float = 0.0, soft=SOFT) -> list:
    pad = (M["pad_on_icing"], M["pad_on_icing"] - 0.1)
    shapes = [soften(cylinder(r + wider / 2, (z1 - z0) * (1 - lower / CAKE_H), pos=(0.0, 0.0, (z0 + z1) / 2 * (1 - lower / CAKE_H)),
                              color=CHOC, friction=pad, friction_combine="min"), *soft) for z0, z1, r in cake_layers()]  # fmt: skip
    h = CAKE_H - lower
    cap0 = CAP_Z0 * (1 - lower / CAKE_H)
    shapes.append(soften(cylinder(R_TOP + wider / 2, h - cap0, pos=(0.0, 0.0, (cap0 + h) / 2), color=ICING, friction=pad, friction_combine="min"), *soft))
    return shapes + squiggle(h + 0.0006)


def squashed_shapes() -> list:
    """A squashed cupcake: 6 mm lower, pressed to 55.5 mm along its x (where the pads were) and wider along y."""
    h = CAKE_H - M["squashed_lower_wider"]
    narrow, long_ = 2 * R_TOP - M["squash_at"] - 0.0005, 2 * R_TOP + M["squashed_lower_wider"]
    pad = (M["pad_on_icing"], M["pad_on_icing"] - 0.1)
    out = []
    for z0, z1, colour in ((0.0, h - 0.005, CHOC), (h - 0.005, h, ICING)):
        zc, dz = (z0 + z1) / 2, z1 - z0
        out.append(soften(box((narrow, long_ - narrow, dz), pos=(0.0, 0.0, zc), color=colour, friction=pad, friction_combine="min"), *SOFT))
        for s in (-1, 1):
            out.append(soften(cylinder(narrow / 2, dz, pos=(0.0, s * (long_ - narrow) / 2, zc), color=colour, friction=pad, friction_combine="min"), *SOFT))
    return out


# --- bodies ---
body("shelf", [box((0.80, 0.80, 0.01))], pos=(0.30, 0.0, SHELF_Z - 0.005), color=(0.3, 0.3, 0.3))
body("board", [box(BOARD_SIZE)], pos=(*BOARD_C, TOP + BOARD_SIZE[2] / 2), mass=M["board_mass"], color=BEECH,
     friction=(M["cupcake_on_beech"], M["cupcake_on_beech"]))
PARKED: dict[str, tuple[float, float, float]] = {}
for _k, (_name, _y) in enumerate(zip(CAKES, CAKE_YS)):
    body(_name, cupcake_shapes(soft=(GAUGE[_k], SOFT[1]) if GAUGE else SOFT), pos=(ROW_X, _y, BOARD_TOP), mass=M["cupcake_mass"], color=CHOC)
    cd.host(_name, CAKE_H, R_TOP - 0.003, resist_top=M["icing_resist"], top_depth=M["icing_depth"], resist_body=M["cake_resist"],
            k_at=HOLD_K_8MM, k_15=HOLD_K_15MM, tear_deg=M["cake_tear_deg"])  # fmt: skip
    PARKED[f"{_name}_squashed"] = (0.10 + 0.10 * _k, -0.30, SHELF_Z + 0.001)
    body(f"{_name}_squashed", squashed_shapes(), pos=PARKED[f"{_name}_squashed"], mass=M["cupcake_mass"], color=CHOC)

body("eggcup", [cylinder(FOOT_R, FOOT_H, pos=(0.0, 0.0, FOOT_H / 2)), cylinder(STEM_R, STEM_H, pos=(0.0, 0.0, FOOT_H + STEM_H / 2)),
                cylinder(CUP_R, CUP_H, pos=(0.0, 0.0, FOOT_H + STEM_H + CUP_H / 2)),
                cylinder(CUP_R - 0.003, 0.002, pos=(0.0, 0.0, DOH_TOP - 0.001), color=DOH)],
     pos=(*CUP_XY, TOP), mass=M["eggcup_mass"], color=PORCELAIN)  # fmt: skip
cd.host("eggcup", DOH_TOP, CUP_R - 0.005, resist_top=M["doh_resist"], top_depth=0.0, resist_body=M["doh_resist"],
        k_at=M["doh_hold_at_8mm"], k_15=M["doh_hold_at_15mm"], tear_deg=M["doh_tear_deg"])  # fmt: skip
_hosts = (*CAKES, "eggcup")
# CUPCAKE_VIEW=1 (check (d) only): three candles stand 15 mm deep in the cupcakes, upright, 10 degrees sideways (+y) and
# 10 degrees away from the arms (+x); the fourth stays in the Play-Doh
VIEW = os.environ.get("CUPCAKE_VIEW") == "1"
VIEW_LEANS = ((0.0, (1.0, 0.0)), (10.0, (0.0, 1.0)), (10.0, (1.0, 0.0)))
for _i, (_name, (_sx, _sy)) in enumerate(zip(CANDLES, ((1, 1), (-1, 1), (-1, -1), (1, -1)))):
    _tip = (CUP_XY[0] + _sx * CANDLE_SQUARE / 2, CUP_XY[1] + _sy * CANDLE_SQUARE / 2, TOP + DOH_TOP - M["candle_in_doh"])
    _quat, _in = (0.0, 0.0, 0.0, 1.0), "eggcup"
    if VIEW and _i < 3:
        _deg, _to = VIEW_LEANS[_i]
        _axis = np.cross((0.0, 0.0, 1.0), (*_to, 0.0))
        _rot = mk.axis_angle(_axis, math.radians(_deg)) if _deg else np.eye(3)
        _entry = np.array([ROW_X, CAKE_YS[_i], BOARD_TOP + CAKE_H])
        _tip, _quat, _in = tuple(_entry - M["candle_in_doh"] * np.asarray(_rot)[:, 2]), mk.matrix_to_quat(np.asarray(_rot)), CAKES[_i]
    cd.declare(_name, _tip, _quat, _hosts, CANDLE_COLORS[_i], built_in=_in, built_depth=M["candle_in_doh"],
               others=tuple(f"{c}_squashed" for c in CAKES), bands=((0.015, (1.0, 1.0, 1.0)), (0.035, (1.0, 1.0, 1.0)), (0.055, (1.0, 1.0, 1.0))),
               friction=(M["pad_on_wax"], M["pad_on_wax"] - 0.05), friction_combine="min")

OBJECTS = list(mk.MECH.bodies.values())
HIDDEN = cd.hidden_shapes()
PAD_GAP_OPEN = 2 * 0.04695  # m between the pads at opening 1 (yam_arena FINGER_OPEN)

STATE: dict = {}


def placement(seed) -> dict[str, tuple[float, float]]:
    """Each cupcake's (x, y): the row moved along the board's edge and each cupcake by at most a centimetre."""
    if not seed:
        return {name: (ROW_X, y) for name, y in zip(CAKES, CAKE_YS)}
    rng = np.random.default_rng(int(seed))
    shift = float(rng.uniform(-0.010, 0.010))
    out = {}
    for name, y in zip(CAKES, CAKE_YS):
        r, a = 0.007 * math.sqrt(float(rng.uniform())), float(rng.uniform(0.0, 2 * math.pi))
        out[name] = (ROW_X + r * math.cos(a), y + shift + r * math.sin(a))
    return out


@mk.on_reset
def start(seed) -> None:
    STATE.update(seed=seed, squashed={}, squeeze={c: 0.0 for c in CAKES}, most_squeeze={c: 0.0 for c in CAKES}, snapped=set())
    if mk.MECH.environment is None:
        return
    for name, (x, y) in placement(seed).items():
        mk.write_pose(name, (x, y, BOARD_TOP), (0.0, 0.0, 0.0, 1.0))
    for name, where in PARKED.items():
        mk.write_pose(name, where, (0.0, 0.0, 0.0, 1.0))
    for name, shape in HIDDEN:
        mk.show(name, shape, False)
    cd.reset()


def squeeze_on(cake: str, side: str) -> float:
    """How far (m) the pads of gripper ``side`` have closed past touch on ``cake``: the cupcake's width between two
    planes square to the closing axis, at the widest of its parts the pads reach, minus the gap between the pads; 0
    if the cupcake is not between them. Flat pads touch a round part at its widest unless the closing line passes
    more than a pad's half width (13 mm) from its axis; then at the pad's edge."""
    point, opening = mk.gripper(side)
    turn = mk.gripper_turn(side)
    p, r = mk.pose(cake)
    r = np.asarray(r)
    q, u = r.T @ (np.asarray(point) - np.asarray(p)), r.T @ turn[:, 0]
    level = math.hypot(u[0], u[1])
    if level < 0.5:  # the closing axis is far from level in the cupcake's frame: not a squeeze across its sides
        return 0.0
    n = np.array([u[0], u[1]]) / level
    gap = opening * PAD_GAP_OPEN
    along = -(q[0] * n[0] + q[1] * n[1])  # the cupcake's axis along the closing line, from the grasp point
    off = abs(q[1] * n[0] - q[0] * n[1])  # its distance from the closing line
    if abs(along) > gap / 2 + 0.002:
        return 0.0
    reach = 0.0132 * abs((r.T @ turn[:, 1])[2]) + 0.0128 * abs((r.T @ turn[:, 2])[2]) + 0.003  # the pads' height round the point
    best = 0.0
    for z0, z1, radius in (*cake_layers(), (CAP_Z0, CAKE_H, R_TOP)):
        if q[2] + reach < z0 or q[2] - reach > z1 or off > radius + 0.0132:
            continue
        width = 2 * math.sqrt(radius**2 - max(0.0, off - 0.0132) ** 2) * level
        best = max(best, width - gap)
    return best


def squash(cake: str, side: str) -> None:
    """The cupcake is squashed for good: its squashed shape takes its place, pressed where the pads were; any candle in
    it comes loose."""
    for c in cd.CANDLES.values():
        if c.host == cake:
            cd._free(c, "cupcake squashed")
    p, r = mk.pose(cake)
    v, w = mk.velocity(cake)
    closing = mk.gripper_turn(side)[:, 0]
    yaw = math.atan2(closing[1], closing[0])
    flat = f"{cake}_squashed"
    mk.write_pose(cake, PARKED[flat], (0.0, 0.0, 0.0, 1.0))
    mk.write_pose(flat, p, mk.yaw_quat(yaw), lin_vel=v, ang_vel=w)
    STATE["squashed"][cake] = {"t": round(mk.now(), 2), "by": side, "closing_mm": round(1000 * STATE["squeeze"][cake], 2)}
    mk.redraw()


@mk.on_step
def step() -> None:
    if mk.MECH.environment is None:
        return
    cd.weightless()
    for cake in CAKES:
        if cake in STATE["squashed"]:
            continue
        most, by = 0.0, None
        for side in ("left", "right"):
            s = squeeze_on(cake, side)
            if s > most:
                most, by = s, side
        STATE["squeeze"][cake] = most
        STATE["most_squeeze"][cake] = max(STATE["most_squeeze"][cake], most)
        if most >= M["squash_at"] and not GAUGE:
            squash(cake, by)
    cd.step(host_ok=lambda name: name not in STATE["squashed"])
    STATE["snapped"] |= {name for name, c in cd.CANDLES.items() if c.snapped}


# --- rules ---
ON_BOARD_TILT_DEG = 30.0
LEAN_FULL, LEAN_MOST = 10.0, 25.0
DEPTH_FULL, OFF_FULL = 0.012, 0.012
CLEAR = 0.030  # grippers at least this far from every cupcake's top and every candle in one
AT_REST = (0.005, 0.05)  # m/s, rad/s


def cake_ok(cake: str) -> bool:
    """Not squashed, standing on the board (tilted less than 30 degrees, its base on the board's top)."""
    if cake in STATE.get("squashed", {}):
        return False
    p, r = mk.pose(cake)
    bp, br = mk.pose("board")
    local = np.asarray(br).T @ (np.asarray(p) - np.asarray(bp))
    on = abs(local[0]) <= BOARD_SIZE[0] / 2 and abs(local[1]) <= BOARD_SIZE[1] / 2 and abs(local[2] - BOARD_SIZE[2] / 2) < 0.004
    tilt = math.degrees(math.acos(float(np.clip(np.asarray(r)[2, 2], -1.0, 1.0))))
    return bool(on and tilt < ON_BOARD_TILT_DEG)


def candle_in(cake: str) -> list[dict]:
    """The candles standing in ``cake``: lean from vertical (deg), depth (m), distance of the entry from the middle
    of the top (m)."""
    out = []
    for c in cd.CANDLES.values():
        if c.host != cake or c.name in STATE.get("snapped", set()):
            continue
        top, rot = cd.host_top(cake)
        hole, _ = mk.pose(c.parts[0])
        off = float(np.hypot(*(rot.T @ (np.asarray(hole) - top))[:2]))
        out.append({"candle": c.name, "lean": cd.lean_from_vertical(c.name), "depth": cd.depth_of(c), "off": off})
    return out


def grade(cake: str) -> tuple[str, list[str]]:
    """("full", []), ("flaw", [the one flaw]) or ("none", why) for one cupcake."""
    if not cake_ok(cake):
        return "none", ["squashed" if cake in STATE.get("squashed", {}) else "off the board or on its side"]
    standing = [x for x in candle_in(cake) if x["lean"] <= LEAN_MOST]
    if len(standing) != 1:
        return "none", [f"{len(standing)} candles standing in it"]
    x = standing[0]
    flaws = [f for f, bad in (("leans", x["lean"] > LEAN_FULL), ("shallow", x["depth"] < DEPTH_FULL), ("off the middle", x["off"] > OFF_FULL)) if bad]
    return ("full", []) if not flaws else ("flaw", flaws)


def clear_and_still() -> bool:
    for side in ("left", "right"):
        point, _ = mk.gripper(side)
        for cake in CAKES:
            top, _ = cd.host_top(cake)
            if np.linalg.norm(np.asarray(point) - top) < CLEAR + 0.030:
                return False
        for c in cd.CANDLES.values():
            if c.host in CAKES and np.linalg.norm(np.asarray(point) - mk.pose(c.name)[0]) < CLEAR + 0.040:
                return False
    for name in (*CAKES, *CANDLES):
        v, w = mk.velocity(name)
        if np.linalg.norm(v) > AT_REST[0] or np.linalg.norm(w) > AT_REST[1]:
            return False
    return True


def score(objects) -> float:
    if not mk.MECH.poses:
        return 0.0
    if sum(1 for cake in CAKES if not cake_ok(cake)) >= 2:
        return 0.0
    grades = [grade(cake) for cake in CAKES]
    full = sum(1 for g, _ in grades if g == "full")
    flawed = [f for g, f in grades if g == "flaw"]
    flaws = sum(len(f) for f in flawed) + len(STATE.get("snapped", set()))
    if full == 3 and not STATE.get("snapped") and clear_and_still():
        return 1.0
    if full + len(flawed) == 3 and flaws == 1 and clear_and_still():
        return 0.75
    return {3: 0.5, 2: 0.5, 1: 0.25}.get(full, 0.0)


def success(objects) -> bool:
    return score(objects) == 1.0


def report() -> dict:
    out = {"squashed": STATE.get("squashed"), "most squeeze (mm)": {k: round(1000 * v, 2) for k, v in STATE.get("most_squeeze", {}).items()},
           "snapped": sorted(STATE.get("snapped", set())), "candles": {n: c.events for n, c in cd.CANDLES.items()}}
    if mk.MECH.poses:
        out["grades"] = {cake: grade(cake) for cake in CAKES}
        out["in each"] = {cake: [{k: (round(v, 4) if isinstance(v, float) else v) for k, v in x.items()} for x in candle_in(cake)] for cake in CAKES}
        if mk.MECH.grippers:
            out["clear and still"] = clear_and_still()
            out["score"] = score(None)
    return out


def _corners(shape) -> np.ndarray:
    """The corners of a solid primitive in its body's frame (a cylinder as a 16-sided prism round it)."""
    r = np.asarray(mk.quat_to_matrix(shape.quat))
    if shape.kind == "box":
        local = np.array([[sx, sy, sz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)]) * np.asarray(shape.dims) / 2
    else:
        radius, length = shape.dims
        k = 1.0 / math.cos(math.pi / 16)
        ring = np.array([[k * radius * math.cos(a), k * radius * math.sin(a)] for a in np.arange(16) * 2 * math.pi / 16])
        half = length / 2 + (radius if shape.kind == "capsule" else 0.0)
        local = np.vstack([np.c_[ring, np.full(16, -half)], np.c_[ring, np.full(16, half)]])
    return local @ r.T + np.asarray(shape.pos)


def loose_clearance() -> tuple[float, str]:
    """The smallest distance (m; negative: they overlap by about that much) between two loose bodies that should not
    touch, and the pair: every cupcake, squashed cupcake on the bench, candle piece, the egg cup and the board, except
    a candle's lower piece against what it can stand in, the two pieces of one candle, and a cupcake on the board."""
    import hands

    live = [n for n in (*CAKES, *[f"{c}_squashed" for c in CAKES], "eggcup", "board", *CANDLES, *[f"{c}_top" for c in CANDLES])
            if mk.pose(n)[0][2] > TOP - 0.05]  # not on the shelf
    sets = {}
    for n in live:
        p, r = mk.pose(n)
        pts = [_corners(sh) for sh in mk.MECH.bodies[n].shapes if sh.solid and sh.kind in ("box", "cylinder", "capsule")]
        sets[n] = [np.asarray(p) + x @ np.asarray(r).T for x in pts]
    hosts = {*CAKES, *[f"{c}_squashed" for c in CAKES], "eggcup"}
    best, pair = math.inf, ""
    for i, a in enumerate(live):
        for b in live[i + 1 :]:
            if {a, b} <= {*CAKES, *[f"{c}_squashed" for c in CAKES], "board"} and "board" in (a, b):
                continue  # a cupcake rests on the board
            if (a in CANDLES and b in hosts) or (b in CANDLES and a in hosts):
                continue  # a candle's lower piece goes through what it stands in
            if a in CANDLES and b == f"{a}_top" or b in CANDLES and a == f"{b}_top":
                continue
            for pa in sets[a]:
                for pb in sets[b]:
                    if np.linalg.norm(pa.mean(0) - pb.mean(0)) > 0.25:
                        continue
                    d = hands.gjk_distance(pa, pb)
                    if d < best:
                        best, pair = d, f"{a}/{b}"
    return best, pair


_FINGER_IDS: dict = {}


def pinch_forces() -> dict:
    """How hard each gripper squeezes (N): the mean of its two fingers' drive forces as the actuator model computes
    them (the arm's own reading, as a real gripper's motor current gives it)."""
    env = mk.MECH.environment
    if env is None:
        return {}
    from yam_arena.constants import FINGER_JOINTS

    out = {}
    for side in ("left", "right"):
        try:
            robot = env.unwrapped.scene[f"{side}_robot"]
        except KeyError:
            continue
        ids = _FINGER_IDS.get(side)
        if ids is None:
            ids = _FINGER_IDS[side] = robot.find_joints(FINGER_JOINTS, preserve_order=True)[0]
        out[side] = round(float(robot.data.applied_torque.torch[0][ids].abs().mean().item()), 3)
    return out


def readings() -> dict:
    """Every step, for trials and demos."""
    clear, pair = loose_clearance()
    out = {"squeeze_mm": {k: round(1000 * v, 3) for k, v in STATE.get("squeeze", {}).items()}, "pinch_N": pinch_forces(),
           "loose_clearance_mm": [round(1000 * clear, 2), pair],
           "gap_mm": {side: round(1000 * mk.gripper(side)[1] * PAD_GAP_OPEN, 2) for side in ("left", "right")}}
    for name, c in cd.CANDLES.items():
        out[name] = cd.reading(c)
    start = placement(STATE.get("seed"))
    for cake in CAKES:
        p, r = mk.pose(cake)
        r = np.asarray(r)
        out[cake] = {"at": [round(float(v), 4) for v in p], "shift_mm": round(1000 * float(np.hypot(p[0] - start[cake][0], p[1] - start[cake][1])), 2),
                     "tilt_deg": round(math.degrees(math.acos(float(np.clip(r[2, 2], -1, 1)))), 2),
                     "about_y_deg": round(math.degrees(math.atan2(r[0, 2], r[2, 2])), 2), "about_x_deg": round(math.degrees(math.atan2(-r[1, 2], r[2, 2])), 2)}
    for name in ("board", "eggcup"):
        p, r = mk.pose(name)
        b = np.asarray(mk.built_pose(name)[0])
        out[name] = {"shift_mm": round(1000 * float(np.linalg.norm(np.asarray(p) - b)), 2),
                     "tilt_deg": round(math.degrees(math.acos(float(np.clip(np.asarray(r)[2, 2], -1, 1)))), 2)}
    return out
