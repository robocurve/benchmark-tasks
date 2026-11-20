"""A front bike light rides on its bracket on a flat handlebar, switched on; a small bowl stands beyond the bar. The
light is to be taken off the bar, switched off and put in the bowl.

The bar is held at its middle by a stem bolted to a block on the bench, as a bike's stem holds it, across the bench
at x BAR_X, 80 mm over it (assumed); the light points away from the arms, as on a bike seen by its rider. The bracket
sits on the bar 70 mm left of the stem with its rail along x.

How it is built (sizes: the things' true ones; (a): assumed until bought and measured):
- The light rides the rail on a hidden shoe: the shoe slides along x on the bracket (joint ``rail``, travel 0 when
  the light sits fully on, dry friction RAIL_FRICTION), the light is welded to it (``seat``). When the shoe has slid
  RAIL_OFF forward (the light's shoe has passed the rail's front end) the weld is cut: the light is free.
- The catch: while the lever is not held down, the rail's stops stand CATCH_PLAY apart where the light is (it cannot
  be pulled on); while the lever is pressed down LEVER_OPEN or more, they stand at 0 and past RAIL_OFF. The hook reads
  the lever once per control step: a light let go of mid-slide stops within CATCH_PLAY of where that step found it.
- The lever: a tab under the rail's rear end that slides down on a preloaded spring (LEVER_PRELOAD at rest, 3 N at
  2 mm down), to a stop LEVER_TRAVEL down.
- The switch: a 10 mm button on the light's top on a preloaded spring; held down BUTTON_HELD or more for HOLD_S, it
  switches the light off; a press that starts while the light is off switches it on. On: the lens and the button's
  indicator are drawn lit; off: dark.
"""

from __future__ import annotations

import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import mechkit as mk  # noqa: E402
from mechkit import body, box, cylinder, slide, weld  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
SUCCESS_HOLD_S = 1.0
INSTRUCTION = "Take the light off the handlebar, switch it off and put it in the bowl."
RUN_LIMIT_STEPS = 900  # 45 s at 20 Hz: 1.6 times the scripted job (27.9 s to success in trial w6bt1)
TOP = TABLE_TOP[SCENE]

BLACK, BAR_BLACK, GREY, WOOD = (0.05, 0.05, 0.06), (0.10, 0.10, 0.11), (0.32, 0.32, 0.34), (0.62, 0.47, 0.30)
LAPIS, LIT, LIT_GREEN, DARK_LENS = (0.12, 0.23, 0.56), (1.0, 1.0, 0.94), (0.25, 1.0, 0.35), (0.30, 0.31, 0.33)

# --- the bar (a flat handlebar, 31.8 mm at the clamp, e.g. Ritchey Comp flat bar; 600 mm wide, straight (a)), the
# stem (70 mm (a)) on a steerer stub in a block screwed to the bench ---
BAR_X, BAR_H = 0.24, 0.080  # m: the bar's axis over the bench (a)
BAR_Z = TOP + BAR_H
BAR_R, BAR_MID, TAPER_R, TAPER_TO, GRIP_R, BAR_HALF = 0.0159, 0.090, 0.0135, 0.120, 0.0111, 0.300  # m: 31.8 mm over the middle 180 mm, 27 mm to 120 mm out, 22.2 mm grips (a)
STEM_L, STEM_W, CLAMP_R, STEERER_R, BLOCK = 0.070, 0.032, 0.021, 0.0143, (0.060, 0.060, 0.040)
STEERER_X = BAR_X - STEM_L

body("bar", [
    cylinder(BAR_R, 2 * BAR_MID, axis="y"),
    *[cylinder(TAPER_R, TAPER_TO - BAR_MID, pos=(0.0, s * (BAR_MID + TAPER_TO) / 2, 0.0), axis="y") for s in (1, -1)],
    *[cylinder(GRIP_R, BAR_HALF - TAPER_TO, pos=(0.0, s * (TAPER_TO + BAR_HALF) / 2, 0.0), axis="y") for s in (1, -1)],
], pos=(BAR_X, 0.0, BAR_Z), color=BAR_BLACK)
body("stand", [
    cylinder(CLAMP_R, 0.040, pos=(BAR_X, 0.0, BAR_Z), axis="y"),  # the stem's bar clamp
    box((STEM_L - 0.020, STEM_W, 0.028), pos=(BAR_X - STEM_L / 2, 0.0, BAR_Z)),
    cylinder(0.020, 0.040, pos=(STEERER_X, 0.0, BAR_Z)),  # the stem's steerer clamp
    cylinder(STEERER_R, BAR_Z + 0.020 - (TOP + BLOCK[2]), pos=(STEERER_X, 0.0, (BAR_Z + 0.020 + TOP + BLOCK[2]) / 2)),
    box(BLOCK, pos=(STEERER_X, 0.0, TOP + BLOCK[2] / 2), color=WOOD),
], pos=(0.0, 0.0, 0.0), color=BAR_BLACK)

# --- the bracket: CatEye H-34N FlexTight (part 5338827N), on the bar 70 mm left of the stem; its rail 30 mm long (a),
# the rail's top RAIL_RISE over the bar's top (a) ---
BRACKET_Y, RAIL_LEN, RAIL_W, RAIL_RISE = 0.070, 0.030, 0.016, 0.008
RAIL_TOP = BAR_Z + BAR_R + RAIL_RISE
RAIL_X0, RAIL_X1 = BAR_X - RAIL_LEN / 2, BAR_X + RAIL_LEN / 2  # its rear and front ends
_ped_low = BAR_Z + 0.012
body("bracket", [
    cylinder(0.0185, 0.020, pos=(BAR_X, BRACKET_Y, BAR_Z), axis="y"),  # the band round the bar
    box((0.026, 0.018, RAIL_TOP - 0.004 - _ped_low), pos=(BAR_X, BRACKET_Y, (RAIL_TOP - 0.004 + _ped_low) / 2)),
    box((RAIL_LEN, RAIL_W, 0.004), pos=(BAR_X, BRACKET_Y, RAIL_TOP - 0.002)),  # the rail
    cylinder(0.008, 0.010, pos=(BAR_X, BRACKET_Y, BAR_Z - 0.0185 - 0.005)),  # the FlexTight dial under the bar
], pos=(0.0, 0.0, 0.0), color=BLACK)

# --- the lever: a tab 10 x 8 mm (a) under the rail's rear end, sticking out behind it; pressed down 2 mm with 3 N (a)
TAB = (0.008, 0.010, 0.0025)  # m along x, y, z
TAB_X, TAB_TOP = RAIL_X0 - 0.006, RAIL_TOP - 0.002  # its middle 6 mm behind the rail's rear end, its top 2 mm under the rail's top
LEVER_K, LEVER_PRELOAD, LEVER_TRAVEL, LEVER_OPEN, LEVER_MASS = 1000.0, 1.0, 0.003, 0.002, 0.003  # N/m, N, m, m, kg
body("lever", [
    box(TAB, color=GREY),
    box((0.012, 0.006, 0.002), pos=(TAB[0] / 2 + 0.006, 0.0, -0.0005), color=GREY, solid=False),  # its arm, in under the rail
], pos=(TAB_X, BRACKET_Y, TAB_TOP - TAB[2] / 2), mass=LEVER_MASS)
slide("lever", "lever", "bracket", axis=(0.0, 0.0, 1.0), limits=(-LEVER_TRAVEL, 0.0), spring=LEVER_K, rest=LEVER_PRELOAD / LEVER_K,
      damping=2.0 * math.sqrt(LEVER_K * LEVER_MASS))

# --- the light: CatEye AMPP 400 / Volt 400 (HL-EL461RC), 108 x 31 x 31 mm, 120 g with its button (a); its frame at its
# middle, x forward. It sits with its rear face flush with the rail's rear end, GAP over the rail ---
LIGHT = (0.108, 0.031, 0.031)
GAP = 0.0005
LIGHT_AT = np.array([RAIL_X0 + LIGHT[0] / 2, BRACKET_Y, RAIL_TOP + GAP + LIGHT[2] / 2])
LIGHT_MASS, BUTTON_MASS, SHOE_MASS = 0.115, 0.005, 0.010
LIGHT_FRICTION = (0.4, 0.4)  # anodised aluminium and plastic (a)
LENS_R = 0.012
LENS_DARK, LENS_LIT = 1, 2  # shape numbers in the light's list
body("light", [
    box(LIGHT),
    cylinder(LENS_R, 0.0008, pos=(LIGHT[0] / 2 + 0.0004, 0.0, 0.0), axis="x", color=DARK_LENS, solid=False),
    cylinder(LENS_R, 0.0008, pos=(LIGHT[0] / 2 + 0.0012, 0.0, 0.0), axis="x", color=LIT, solid=False, glow=1.0),
], pos=tuple(LIGHT_AT), mass=LIGHT_MASS, color=BLACK, friction=LIGHT_FRICTION)

# the button: 10 mm across, 2 mm proud of the top, 30 mm from the rear face (a); 1 mm travel (a), 1.5 N at rest and
# 3 N pressed fully (a); its indicator lit while the light is on (as the Volt's switch shows its battery: to check)
BUTTON_R, BUTTON_PROUD, BUTTON_FROM_REAR, BUTTON_TRAVEL, BUTTON_HELD = 0.005, 0.002, 0.030, 0.001, 0.0008
BUTTON_PRELOAD, BUTTON_K = 1.5, 1500.0
HOLD_S = 1.0
BUTTON_AT = LIGHT_AT + np.array([-LIGHT[0] / 2 + BUTTON_FROM_REAR, 0.0, LIGHT[2] / 2 + BUTTON_PROUD - 0.0015])
BUTTON_LIT = 1
body("button", [
    cylinder(BUTTON_R, 0.003),
    cylinder(0.0035, 0.0004, pos=(0.0, 0.0, 0.0017), color=LIT_GREEN, solid=False, glow=1.0),
], pos=tuple(BUTTON_AT), mass=BUTTON_MASS, color=GREY, friction=(0.9, 0.9))  # rubber
slide("button", "button", "light", axis=(0.0, 0.0, 1.0), limits=(-BUTTON_TRAVEL, 0.0), spring=BUTTON_K, rest=BUTTON_PRELOAD / BUTTON_K,
      damping=2.0 * math.sqrt(BUTTON_K * BUTTON_MASS))

# the shoe under the light, on the rail (hidden; 10 g and the inertia of a 3 cm piece, so that the weld to the light
# is not between bodies of very different size)
SHOE_AT = (RAIL_X0 + 0.004, BRACKET_Y, RAIL_TOP - 0.002)
RAIL_FRICTION = 3.0  # N: the light slides off with 2 to 4 N while the lever is held (a)
RAIL_OFF = RAIL_LEN  # m the shoe slides until the light's shoe has passed the rail's front end
RAIL_OVER = 0.150  # m the open rail's front stop stands past RAIL_OFF. The light is cut free at the first control step
# (20 Hz) that finds it past RAIL_OFF; until then the slide still guides it. 20 N on the 130 g light from rest takes it
# 162 mm in one step: it never meets this stop (run w6bya). At 10 mm a constant 5 N met it (run w6bca).
CATCH_PLAY = 0.0005  # m: a caught light moves at most this much along the rail
body("shoe", [box((0.002, 0.002, 0.002), passes=("bracket", "light", "lever", "button", "bar", "stand"))], pos=SHOE_AT, mass=SHOE_MASS,
     inertia=(2e-6, 2e-6, 2e-6))  # inside the rail's plate; it goes through what is near it
slide("rail", "shoe", "bracket", axis=(1.0, 0.0, 0.0), limits=(0.0, CATCH_PLAY), friction=RAIL_FRICTION)
weld("seat", "light", "shoe")

# --- the bowl: Fiesta small bowl, item 460, Lapis: 143 mm across, 48 mm high; inside, a floor 80 mm across 6 mm over
# the bench, and a wall curving up to the rim, drawn as two flat rings: gently to 112 mm across 14 mm up, then steeply
# to the rim (133 mm across inside); walls 5 mm; 330 g (all (a)); loose on the bench beyond the bar. (With one straight
# wall from the floor to the rim the 108 mm light would bridge it with its middle 1 mm under the rim.) ---
BOWL_XY = (0.38, 0.20)
BOWL_R, BOWL_FLOOR_R, BOWL_H, BOWL_FLOOR_T, BOWL_T, BOWL_MASS, BOWL_SIDES = 0.0715, 0.040, 0.048, 0.006, 0.005, 0.330, 32
BOWL_KNEE = (0.056, 0.014)  # m: where the wall's gentle lower ring meets its steep upper one (radius, height inside)
GLAZE_FRICTION = (0.35, 0.30)  # glazed stoneware (a)


def _ring(r_in: float, r_out: float, z_in: float, z_out: float, t: float, sides: int, a: float):
    """One flat piece of a sloping ring towards angle ``a``: its top face runs from (r_in, z_in) to (r_out, z_out)."""
    slope = math.atan2(z_out - z_in, r_out - r_in)
    length = math.hypot(r_out - r_in, z_out - z_in)
    r = (r_in + r_out) / 2 + math.sin(slope) * t / 2
    z = (z_in + z_out) / 2 - math.cos(slope) * t / 2
    turn = mk.matrix_to_quat(mk.quat_to_matrix(mk.yaw_quat(a)) @ mk.axis_angle((0.0, 1.0, 0.0), -slope))
    return box((length + 0.001, 2 * max(r_in, r_out) * math.tan(math.pi / sides) + 0.001, t), pos=(r * math.cos(a), r * math.sin(a), z), quat=turn)


def bowl_shapes() -> list:
    angles = [2 * math.pi * k / BOWL_SIDES for k in range(BOWL_SIDES)]
    wall = [_ring(BOWL_FLOOR_R, BOWL_KNEE[0], BOWL_FLOOR_T, BOWL_KNEE[1], BOWL_T, BOWL_SIDES, a) for a in angles]
    wall += [_ring(BOWL_KNEE[0], BOWL_R - BOWL_T, BOWL_KNEE[1], BOWL_H, BOWL_T, BOWL_SIDES, a) for a in angles]
    return [cylinder(BOWL_FLOOR_R + 0.002, BOWL_FLOOR_T, pos=(0.0, 0.0, BOWL_FLOOR_T / 2)), *wall]


body("bowl", bowl_shapes(), pos=(*BOWL_XY, TOP), mass=BOWL_MASS, color=LAPIS, friction=GLAZE_FRICTION)

OBJECTS = list(mk.MECH.bodies.values())
STATE: dict = {}


# --- the catch and the switch ---
def _limits(lower: float, upper: float) -> None:
    """The rail's stops: in the simulator, and in STATE (what tests read)."""
    STATE["limits"] = (lower, upper)
    if "rail" in mk.MECH.joint_prims:
        mk.set_limits("rail", lower=lower, upper=upper)


def _event(what: str) -> None:
    STATE.setdefault("events", []).append((round(mk.now(), 2), what))


def _switch(on: bool) -> None:
    STATE["on"] = on
    mk.show("light", LENS_LIT, on)
    mk.show("button", BUTTON_LIT, on)
    mk.redraw()  # a small lit thing that goes dark can stay in the pictures for seconds without it (mechkit notes)
    _event("switched on" if on else "switched off")


@mk.on_reset
def start() -> None:
    STATE.clear()
    STATE.update(on=True, caught=True, pressed=False, since=None, track=[], events=[])
    _limits(0.0, CATCH_PLAY)
    mk.show("shoe", 0, False)


@mk.on_step
def step() -> None:
    t = mk.now()
    if not mk.is_cut("seat"):
        s = mk.joint("rail")
        held = mk.joint("lever") <= -LEVER_OPEN
        if s >= RAIL_OFF:
            mk.cut("seat")
            _limits(0.0, CATCH_PLAY)  # the empty shoe goes back into the rail
            _event(f"off the rail ({1000 * s:.1f} mm)")
        elif held and STATE["caught"]:
            _limits(0.0, RAIL_OFF + RAIL_OVER)
            STATE["caught"] = False
            _event(f"catch open at {1000 * s:.1f} mm")
        elif not held and not STATE["caught"]:
            low = max(0.0, s - CATCH_PLAY / 2)
            _limits(low, low + CATCH_PLAY)
            STATE["caught"] = True
            _event(f"caught at {1000 * s:.1f} mm")
    pressed = mk.joint("button") <= -BUTTON_HELD
    if pressed and not STATE["pressed"]:  # a new press
        if STATE["on"]:
            STATE["since"] = t
        else:
            _switch(True)  # the maker's on: one press (a)
    elif not pressed:
        STATE["since"] = None
    if pressed and STATE["on"] and STATE["since"] is not None and t - STATE["since"] >= HOLD_S - 1e-9:
        _switch(False)
        STATE["since"] = None
    STATE["pressed"] = pressed
    kept = STATE["track"]
    kept.append((t, mk.pose("light")[0].copy()))
    while len(kept) > 2 and kept[1][0] <= t - SETTLE_S:
        kept.pop(0)


# --- the score ---
CLEAR, RESTING, SETTLE_S, OFF_BENCH, SLID, NEAR_BOWL = 0.03, 0.02, 0.25, 0.02, 0.005, 0.15


def light_on() -> bool:
    return bool(STATE.get("on", True))


def slid() -> float:
    """How far the light has slid towards the rail's front end (m)."""
    return float(mk.joint("rail"))


def off_rail() -> bool:
    return mk.is_cut("seat")


def in_bowl() -> bool:
    """The light's middle inside the bowl's rim and under it, over its floor."""
    p, _ = mk.pose("light")
    b, rb = mk.pose("bowl")
    rel = rb.T @ (p - b)
    return bool(math.hypot(rel[0], rel[1]) < BOWL_R - BOWL_T and BOWL_FLOOR_T < rel[2] < BOWL_H)


def near_bowl() -> bool:
    """Lying on the bench (its middle no higher than when it stands on an end) within NEAR_BOWL of the bowl's rim."""
    p, _ = mk.pose("light")
    b, _ = mk.pose("bowl")
    return float(p[2]) <= TOP + LIGHT[0] / 2 + 0.005 and math.hypot(p[0] - b[0], p[1] - b[1]) <= BOWL_R + NEAR_BOWL


def on_bench() -> bool:
    return float(mk.pose("light")[0][2]) >= TOP - OFF_BENCH


def distance_to_light(point) -> float:
    p, r = mk.pose("light")
    local = r.T @ (np.asarray(point, dtype=float) - p)
    half = np.asarray(LIGHT) / 2
    return float(np.linalg.norm(np.maximum(np.abs(local) - half, 0.0)))


def hands_clear() -> bool:
    return all(distance_to_light(mk.gripper(side)[0]) >= CLEAR for side in ("left", "right"))


def speed() -> float | None:
    kept = STATE.get("track", [])
    if len(kept) < 2 or kept[-1][0] - kept[0][0] < SETTLE_S - 1e-9:
        return None
    (t0, p0), (t1, p1) = kept[0], kept[-1]
    return float(np.linalg.norm(p1 - p0)) / (t1 - t0)


def at_rest() -> bool:
    v = speed()
    return v is not None and v < RESTING


def level() -> float:
    """By where the light is, before the 1.0's conditions: 0 off the bench or still caught (slid less than 5 mm);
    0.25 slid 5 mm or more but on the rail; 0.5 off the rail; 0.75 in the bowl (on or off), or off and lying on the
    bench within 15 cm of the bowl."""
    if not on_bench():
        return 0.0
    if not off_rail():
        return 0.25 if slid() >= SLID else 0.0
    if in_bowl() or (not light_on() and near_bowl()):
        return 0.75
    return 0.5


def score(objects) -> float:
    """1.0: the light in the bowl, switched off, both grippers 3 cm clear of it, at rest (the bracket and the bar
    are fixed bodies: they stay where they are). Otherwise :func:`level`."""
    if in_bowl() and not light_on() and on_bench() and hands_clear() and at_rest():
        return 1.0
    return level()


def success(objects) -> bool:
    return score(objects) == 1.0


def report() -> dict:
    return {"events": STATE.get("events", []), "slid (mm)": round(1000 * slid(), 2), "off the rail": off_rail(), "on": light_on(),
            "in bowl": in_bowl(), "lever (mm)": round(1000 * mk.joint("lever"), 2), "button (mm)": round(1000 * mk.joint("button"), 2)}
